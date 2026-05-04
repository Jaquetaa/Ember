// ============================================
// EMBER — Gonçalo (Drone) — ESP32-S3
// ============================================
//
// Payload recebido (9 bytes — struct PayloadCtrl):
//   armed:    0 = stop/desarmado, 1 = armado
//   throttle: microsegundos (1000-1500)
//   yaw:      -100 a +100
//   pitch:    -100 a +100  (frente/tras — stick direito Y)
//   roll:     -100 a +100  (esquerda/direita — stick direito X)

#include <Arduino.h>
#include <SPI.h>

#include "EmberSensor.h"
#include "EmberDroneNRF.h"
#include "EmberCAMNRF.h"   // header leve — sem MLX/RF24/Wire
#include "EmberCalibration.h"

// LEDC directo (bypass ESP32Servo). ESP32Servo@3.1.3 no ESP32-S3 prefere
// MCPWM para Servo em freq fixa e trava dentro de mcpwm_init() com esta
// versao do IDF — allocateTimer() so afecta LEDC e nao resolve. Por isso
// geramos o PWM dos ESCs com ledcSetup/ledcAttachPin/ledcWrite.

// ── Pinos ───────────────────────────────────────────────
#define PIN_SCK  17
#define PIN_MISO 18
#define PIN_MOSI 7

#define ESC1_PIN 14
#define ESC2_PIN 4
#define ESC3_PIN 1
#define ESC4_PIN 19

// NRF Controlo (RX)
#define NRF_CTRL_CE   13
#define NRF_CTRL_CSN  12

// NRF Thermal (TX)
#define NRF_THERM_CE   2
#define NRF_THERM_CSN  40

// I2C MLX90640
#define MLX_SDA 41
#define MLX_SCL 42

// Sensores analogicos
#define FLAME_PIN 8
#define MQ7_PIN   20

// Botao de calibracao dos ESCs (active LOW, INPUT_PULLUP)
#define CAL_BTN_PIN 3

// Botao para saltar aquecimento do MQ-7 (active LOW, INPUT_PULLUP)
#define SKIP_PREHEAT_PIN 39

#define TIMEOUT_SINAL 1000UL

// ── Throttle (HobbyKing 20A ESC — range 1000-2000us) ────
const int THROTTLE_NORMAL = 1300;
const int THROTTLE_Y_LOW  = 1200;
const int THROTTLE_Y_HIGH = 1500;
const int THROTTLE_MIN    = 1000;
const int THROTTLE_ARM    = 1100;
const int YAW_MAX         = 100;

const int RAMP_STEP  = 2;
const int RAMP_DELAY = 40;

// ── ESCs (LEDC directo) ─────────────────────────────────
// Um canal LEDC por ESC, todos no mesmo timer (50Hz, 14-bit).
// 14-bit a 50Hz -> periodo 20000us, resolucao max 16383 ticks.
#define ESC1_CH        0
#define ESC2_CH        1
#define ESC3_CH        2
#define ESC4_CH        3
#define ESC_FREQ_HZ    50
#define ESC_RES_BITS   14
static const uint32_t ESC_PERIOD_US = 1000000UL / ESC_FREQ_HZ;  // 20000
static const uint32_t ESC_DUTY_MAX  = (1UL << ESC_RES_BITS) - 1;

static inline uint32_t escUsToDuty(uint32_t us) {
  return ((uint64_t)us * ESC_DUTY_MAX) / ESC_PERIOD_US;
}

bool armed           = false;
bool rampActive      = false;
bool disarming       = false;
int  currentThrottle = THROTTLE_MIN;
int  targetThrottle  = THROTTLE_MIN;

// ── Classes Ember (restantes) ───────────────────────────
EmberSensor       sensores;
EmberDroneNRF     controlRX;
EmberCAMNRF       thermalTX;
EmberCalibration  calibration;

// ── Estado radio ────────────────────────────────────────
bool radioCtrlOK    = false;
bool lastRadioArmed = false;

// Macro para debug com flush imediato
#define DBG(msg) do { Serial.println(msg); Serial.flush(); } while(0)

// ── Helpers ESC ─────────────────────────────────────────
void writeAllESCs(int us) {
  uint32_t d = escUsToDuty((uint32_t)us);
  ledcWrite(ESC1_CH, d);
  ledcWrite(ESC2_CH, d);
  ledcWrite(ESC3_CH, d);
  ledcWrite(ESC4_CH, d);
}

/*
  Layout:
    1(CW)  2(CCW)
    3(CCW) 4(CW)
*/
void writeESCsWithYaw(int base, int yaw) {
  int safeMin = armed ? THROTTLE_ARM + 50 : THROTTLE_MIN;
  int a = constrain(base + yaw, safeMin, 2000);
  int b = constrain(base - yaw, safeMin, 2000);
  ledcWrite(ESC1_CH, escUsToDuty((uint32_t)a));
  ledcWrite(ESC2_CH, escUsToDuty((uint32_t)b));
  ledcWrite(ESC3_CH, escUsToDuty((uint32_t)b));
  ledcWrite(ESC4_CH, escUsToDuty((uint32_t)a));
}

// Mixing completo: yaw mantem CW(+) / CCW(-) do writeESCsWithYaw.
// Pitch>0 inclina para tras (frente sobe), Roll>0 inclina para a direita
// (esquerda sobe). Verificar fisicamente sem helices — se um eixo
// estiver invertido, troca o sinal desse eixo nesse motor.
//
//   Frente
// ESC1 ESC2
// ESC3 ESC4
//   Tras
void writeESCsWithAll(int base, int yaw, int pitch, int roll) {
  int safeMin = armed ? THROTTLE_ARM + 50 : THROTTLE_MIN;
  int e1 = constrain(base + yaw + pitch + roll, safeMin, 2000); // FE CW
  int e2 = constrain(base - yaw + pitch - roll, safeMin, 2000); // FD CCW
  int e3 = constrain(base - yaw - pitch + roll, safeMin, 2000); // TE CCW
  int e4 = constrain(base + yaw - pitch - roll, safeMin, 2000); // TD CW
  ledcWrite(ESC1_CH, escUsToDuty((uint32_t)e1));
  ledcWrite(ESC2_CH, escUsToDuty((uint32_t)e2));
  ledcWrite(ESC3_CH, escUsToDuty((uint32_t)e3));
  ledcWrite(ESC4_CH, escUsToDuty((uint32_t)e4));
}

void rampToTarget() {
  if (abs(currentThrottle - targetThrottle) > RAMP_STEP) {
    currentThrottle += (currentThrottle < targetThrottle) ? RAMP_STEP : -RAMP_STEP;
    currentThrottle = constrain(currentThrottle, THROTTLE_MIN, 2000);
    writeAllESCs(currentThrottle);
  } else {
    currentThrottle = targetThrottle;
    writeAllESCs(currentThrottle);
    rampActive = false;
    if (disarming) {
      armed     = false;
      disarming = false;
      Serial.println("[ESC] Ramp down concluido — drone DESARMADO.");
    } else {
      Serial.println("[ESC] Ramp concluido.");
    }
  }
}

void emergencyStop() {
  armed           = false;
  rampActive      = false;
  disarming       = false;
  targetThrottle  = THROTTLE_MIN;
  currentThrottle = THROTTLE_MIN;
  writeAllESCs(THROTTLE_MIN);
}

// Callbacks para EmberCalibration
static void calWriteAll(int us) {
  writeAllESCs(us);
  currentThrottle = us;
}
static void calOnStart() {
  // Desarmar logica de voo; o ramp down e feito pela propria lib.
  // Comandos de radio sao ignorados enquanto calibration.isBusy().
  armed          = false;
  rampActive     = false;
  disarming      = false;
  targetThrottle = THROTTLE_MIN;
  lastRadioArmed = false;
  Serial.println("[GX] Calibracao iniciada — voo bloqueado, drone surdo a comandos.");
}
static void calOnDone() {
  // Pos-calibracao: garante estado seguro. So volta a armar
  // quando o utilizador mandar ARM novo no rolante.
  armed          = false;
  rampActive     = false;
  disarming      = false;
  targetThrottle = THROTTLE_MIN;
  currentThrottle= THROTTLE_MIN;
  lastRadioArmed = false;
  Serial.println("[GX] Calibracao concluida — pronto a receber novo ARM.");
}

// Empacota estado dos sensores (2 bits) + fase calibracao (3 bits)
// num unico byte ACK. Tiago descodifica:
//   estado   = ack & 0x03;
//   calPhase = (ack >> 2) & 0x07;
static inline uint8_t buildAckByte(uint8_t estado, uint8_t calPhase) {
  return (uint8_t)((estado & 0x03) | ((calPhase & 0x07) << 2));
}

// ============================================================
// DIAGNOSTICO MASSIVO DO NRF — bypassa toda a lib RF24
// ============================================================
//
// Faz 4 testes a niveis cada vez mais baixos:
//   T1 — Toggle GPIO: confirma que cada pino consegue HIGH/LOW
//   T2 — Bit-bang SPI: lê STATUS register manualmente, sem
//        usar peripheral SPI (puro digitalWrite/digitalRead)
//   T3 — Hardware SPI raw: usa SPI.transfer() directo
//   T4 — Teste de write/read: escreve em CONFIG e lê de volta
//
// Se T2 (bit-bang) lê algo != 0x00 → wiring OK, é a lib SPI.
// Se T3 (hw SPI) lê 0xff e T2 lê algo OK → SPI peripheral
// nao esta a configurar correctamente.
// Se ambos T2/T3 leem 0x00 ou 0xff → wiring/alimentacao.
// ============================================================

static void diag_pinToggle(const char *name, int pin) {
  Serial.print("  [T1] "); Serial.print(name);
  Serial.print(" (GPIO"); Serial.print(pin); Serial.print(")  ");
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);   delayMicroseconds(50);
  int rLow  = digitalRead(pin);
  digitalWrite(pin, HIGH);  delayMicroseconds(50);
  int rHigh = digitalRead(pin);
  Serial.print("LOW=");  Serial.print(rLow);
  Serial.print(" HIGH="); Serial.print(rHigh);
  if (rLow == 0 && rHigh == 1) Serial.println("  OK");
  else                          Serial.println("  *** FALHA ***");
  Serial.flush();
}

// Bit-bang um byte SPI mode 0: CPOL=0, CPHA=0, MSB first.
// Set MOSI on falling edge of SCK, sample MISO on rising edge.
static uint8_t diag_bitbangByte(int sck, int mosi, int miso, uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(mosi, (out >> i) & 1);
    delayMicroseconds(5);
    digitalWrite(sck, HIGH);
    delayMicroseconds(5);
    int b = digitalRead(miso);
    in = (in << 1) | (b & 1);
    digitalWrite(sck, LOW);
    delayMicroseconds(5);
  }
  return in;
}

static void diag_bitbangNRF(int sck, int mosi, int miso, int csn) {
  Serial.println("  [T2] Bit-bang SPI manual (NRF Controlo CSN=12)");
  Serial.flush();

  pinMode(sck,  OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  pinMode(csn,  OUTPUT);

  digitalWrite(sck,  LOW);
  digitalWrite(mosi, LOW);
  digitalWrite(csn,  HIGH);
  delayMicroseconds(50);

  // Le STATUS register (cmd 0xFF = NOP, devolve STATUS no primeiro byte)
  digitalWrite(csn, LOW);
  delayMicroseconds(5);
  uint8_t status = diag_bitbangByte(sck, mosi, miso, 0xFF);
  digitalWrite(csn, HIGH);
  delayMicroseconds(50);

  Serial.print("    STATUS (raw) = 0x");
  if (status < 0x10) Serial.print("0");
  Serial.print(status, HEX);

  // STATUS por defeito apos reset: 0x0E
  // Se for 0x00 ou 0xFF, o NRF nao responde
  if (status == 0x00) {
    Serial.println("  *** TUDO 0x00 — MISO sempre LOW (sem resposta) ***");
  } else if (status == 0xFF) {
    Serial.println("  *** TUDO 0xFF — MISO sempre HIGH (sem resposta) ***");
  } else if (status == 0x0E) {
    Serial.println("  *** 0x0E = valor por defeito! NRF VIVO! ***");
  } else {
    Serial.print("  *** valor incomum: 0b");
    for (int b = 7; b >= 0; b--) Serial.print((status >> b) & 1);
    Serial.println(" — NRF parece estar a responder ***");
  }

  // Le CONFIG register (cmd 0x00 R_REGISTER + reg 0x00 = 0x00)
  digitalWrite(csn, LOW);
  delayMicroseconds(5);
  diag_bitbangByte(sck, mosi, miso, 0x00);  // R_REGISTER + reg 0x00 (CONFIG)
  uint8_t config = diag_bitbangByte(sck, mosi, miso, 0xFF);
  digitalWrite(csn, HIGH);
  delayMicroseconds(50);

  Serial.print("    CONFIG (raw) = 0x");
  if (config < 0x10) Serial.print("0");
  Serial.print(config, HEX);
  Serial.println(config == 0x08 ? "  *** 0x08 = default OK ***" : "");
  Serial.flush();
}

static void diag_hwSPI(SPIClass &spi, int csn, uint32_t freq) {
  Serial.print("  [T3] HW SPI @ ");
  Serial.print(freq / 1000);
  Serial.println(" kHz");
  Serial.flush();

  pinMode(csn, OUTPUT);
  digitalWrite(csn, HIGH);
  delayMicroseconds(50);

  spi.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE0));
  digitalWrite(csn, LOW);
  delayMicroseconds(5);
  uint8_t status = spi.transfer(0xFF);
  digitalWrite(csn, HIGH);
  spi.endTransaction();

  Serial.print("    STATUS (hw) = 0x");
  if (status < 0x10) Serial.print("0");
  Serial.println(status, HEX);
  Serial.flush();
}

static void diag_writeReadCONFIG(SPIClass &spi, int csn) {
  Serial.println("  [T4] Write/Read CONFIG register (test de echo)");
  Serial.flush();

  // Escreve 0x0A em CONFIG (W_REGISTER 0x20 | 0x00 = 0x20)
  pinMode(csn, OUTPUT);
  digitalWrite(csn, HIGH);
  delayMicroseconds(50);

  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csn, LOW);
  delayMicroseconds(5);
  spi.transfer(0x20);    // W_REGISTER + CONFIG
  spi.transfer(0x0A);    // valor a escrever
  digitalWrite(csn, HIGH);
  delayMicroseconds(20);

  // Le CONFIG de volta
  digitalWrite(csn, LOW);
  delayMicroseconds(5);
  spi.transfer(0x00);    // R_REGISTER + CONFIG
  uint8_t back = spi.transfer(0xFF);
  digitalWrite(csn, HIGH);
  spi.endTransaction();

  Serial.print("    Escreveu 0x0A, leu 0x");
  if (back < 0x10) Serial.print("0");
  Serial.print(back, HEX);
  if (back == 0x0A) Serial.println("  *** ECHO OK — NRF VIVO ***");
  else              Serial.println("  *** sem echo — NRF NAO RESPONDE ***");
  Serial.flush();
}

// Re-aplica o mapeamento de pinos do periferico SPI atraves do GPIO matrix.
// Necessario depois de fazer pinMode/digitalWrite que desligam os pinos
// do SPI peripheral.
static void diag_restoreSPIPins() {
  SPI.end();
  delay(5);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, NRF_CTRL_CSN);
  delay(5);
}

// Diagnostico para 1 NRF (qualquer CSN/CE).
static void diag_oneNRF(const char *label, int csn, int ce) {
  Serial.print("\n----- DIAG NRF ");
  Serial.print(label);
  Serial.print(" (CSN=GPIO");
  Serial.print(csn);
  Serial.print(", CE=GPIO");
  Serial.print(ce);
  Serial.println(") -----");
  Serial.flush();

  // Garante que CE esta LOW e CSN esta HIGH antes de comecar
  pinMode(ce, OUTPUT);
  digitalWrite(ce, LOW);
  pinMode(csn, OUTPUT);
  digitalWrite(csn, HIGH);
  delay(5);

  // T3 — HW SPI com pinos limpos
  Serial.println("[T3] HW SPI test (HW SPI peripheral):");
  diag_hwSPI(SPI, csn, 100000);
  diag_hwSPI(SPI, csn, 1000000);
  diag_hwSPI(SPI, csn, 8000000);

  // T4 — Echo
  diag_writeReadCONFIG(SPI, csn);

  // T1 — toggle CSN/CE
  Serial.println("[T1] GPIO toggle test (CSN/CE):");
  diag_pinToggle("CSN ", csn);
  diag_pinToggle("CE  ", ce);
  Serial.flush();

  // T2 — Bit-bang
  diag_bitbangNRF(PIN_SCK, PIN_MOSI, PIN_MISO, csn);

  // T5 — Restaura HW SPI e tenta de novo
  Serial.println("[T5] Restaura SPI peripheral e tenta HW SPI de novo:");
  diag_restoreSPIPins();
  diag_hwSPI(SPI, csn, 1000000);
  diag_writeReadCONFIG(SPI, csn);

  Serial.print("----- FIM DIAG ");
  Serial.print(label);
  Serial.println(" -----\n");
  Serial.flush();
}

void runNRFDiagnostics() {
  Serial.println("\n========================================");
  Serial.println("    DIAGNOSTICO NRF — INICIO");
  Serial.println("========================================");
  Serial.println("Pinos SPI partilhados:");
  Serial.print("  SCK  = GPIO"); Serial.println(PIN_SCK);
  Serial.print("  MISO = GPIO"); Serial.println(PIN_MISO);
  Serial.print("  MOSI = GPIO"); Serial.println(PIN_MOSI);
  Serial.flush();

  // SEMPRE garante que AMBOS os CSN comecam HIGH antes de qualquer
  // tentativa de comunicacao. Se um NRF tem CSN flutuante ou LOW,
  // segura o MISO e impede o outro de responder.
  pinMode(NRF_CTRL_CSN, OUTPUT);
  digitalWrite(NRF_CTRL_CSN, HIGH);
  pinMode(NRF_THERM_CSN, OUTPUT);
  digitalWrite(NRF_THERM_CSN, HIGH);
  pinMode(NRF_CTRL_CE, OUTPUT);
  digitalWrite(NRF_CTRL_CE, LOW);
  pinMode(NRF_THERM_CE, OUTPUT);
  digitalWrite(NRF_THERM_CE, LOW);
  delay(10);

  // Diagnostica os DOIS NRFs
  diag_oneNRF("CONTROLO", NRF_CTRL_CSN,  NRF_CTRL_CE);
  diag_oneNRF("THERMAL ", NRF_THERM_CSN, NRF_THERM_CE);

  Serial.println("========================================");
  Serial.println("    DIAGNOSTICO NRF — FIM");
  Serial.println("========================================\n");
  Serial.flush();
}

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);   // USB CDC precisa de tempo a enumerar no ESP32-S3
  Serial.println("\n[EMBER] Goncalo — Drone");
  Serial.flush();

  DBG("[DEBUG] setup: inicio");

  // ── ESCs — LEDC directo (bypass ESP32Servo) ───────────
  // ledcSetup(channel, freqHz, resBits) -> configura o timer partilhado
  // ledcAttachPin(gpio, channel)        -> liga o GPIO ao canal
  // ledcWrite(channel, duty)            -> escreve duty (0..2^res - 1)
  DBG("[DEBUG] ESC LEDC setup 50Hz/14-bit");
  double f1 = ledcSetup(ESC1_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f2 = ledcSetup(ESC2_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f3 = ledcSetup(ESC3_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f4 = ledcSetup(ESC4_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  Serial.print("[DEBUG] ledcSetup freqs: ");
  Serial.print(f1); Serial.print(" ");
  Serial.print(f2); Serial.print(" ");
  Serial.print(f3); Serial.print(" ");
  Serial.println(f4); Serial.flush();

  if (f1 == 0 || f2 == 0 || f3 == 0 || f4 == 0) {
    DBG("[EMBER] ERRO FATAL: ledcSetup falhou (freq==0)");
    while (1) { delay(1000); }
  }

  DBG("[DEBUG] ledcAttachPin 38/39/40/41");
  ledcAttachPin(ESC1_PIN, ESC1_CH);
  ledcAttachPin(ESC2_PIN, ESC2_CH);
  ledcAttachPin(ESC3_PIN, ESC3_CH);
  ledcAttachPin(ESC4_PIN, ESC4_CH);

  writeAllESCs(THROTTLE_MIN);
  delay(100);
  DBG("[DEBUG] ESC LEDC OK");

  // ── CS pins HIGH (de-select ambos NRF antes de SPI) ───
  DBG("[DEBUG] pinMode CSN control HIGH...");
  pinMode(NRF_CTRL_CSN, OUTPUT);  digitalWrite(NRF_CTRL_CSN, HIGH);
  DBG("[DEBUG] CSN control OK");

  DBG("[DEBUG] pinMode CSN thermal HIGH...");
  pinMode(NRF_THERM_CSN, OUTPUT); digitalWrite(NRF_THERM_CSN, HIGH);
  DBG("[DEBUG] CSN thermal OK");
  delay(10);

  // ── Sensores ──────────────────────────────────────────
  DBG("[DEBUG] sensores.begin(FLAME, MQ7)...");
  sensores.begin(FLAME_PIN, MQ7_PIN, SKIP_PREHEAT_PIN);
  DBG("[DEBUG] sensores.begin OK");

  // ── SPI ───────────────────────────────────────────────
  DBG("[DEBUG] SPI.begin(SCK, MISO, MOSI, CSN_CTRL)...");
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, NRF_CTRL_CSN);
  DBG("[DEBUG] SPI.begin OK");
  delay(10);

  // ── DIAGNOSTICO MASSIVO antes de tentar lib RF24 ─────
  runNRFDiagnostics();

  // ── NRF Controlo ──────────────────────────────────────
  DBG("[DEBUG] controlRX.begin(CE_CTRL, CSN_CTRL)...");
  radioCtrlOK = controlRX.begin(NRF_CTRL_CE, NRF_CTRL_CSN, SPI);
  if (radioCtrlOK) {
    DBG("[DEBUG] controlRX.begin OK");
  } else {
    DBG("[EMBER] AVISO: NRF Controlo falhou");
  }

  // ── Thermal TX (tudo alocado dentro do begin) ─────────
  DBG("[DEBUG] thermalTX.begin(CE_TH, CSN_TH, SDA, SCL)...");
  if (!thermalTX.begin(NRF_THERM_CE, NRF_THERM_CSN, MLX_SDA, MLX_SCL, SPI)) {
    DBG("[EMBER] AVISO: Thermal falhou (continua sem camara)");
  } else {
    DBG("[DEBUG] thermalTX.begin OK");
  }

  // ── Botao de calibracao ──────────────────────────────
  calibration.begin(CAL_BTN_PIN, calWriteAll, calOnStart, calOnDone);

  DBG("[EMBER] === Pronto ===\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  static unsigned long statLoopCount = 0;
  static unsigned long statPktCount  = 0;
  static unsigned long statLastPrint = 0;
  statLoopCount++;

  // ── Calibracao (prioridade absoluta) ──────────────────
  // Se o botao foi premido, suspende todo o voo ate concluir.
  // O drone fica "surdo": ainda mantem o link radio vivo (para o
  // ACK chegar ao Tiago com a fase de calibracao), mas descarta
  // qualquer comando recebido sem agir sobre os ESCs.
  calibration.update(currentThrottle);
  if (calibration.isBusy()) {
    if (radioCtrlOK) {
      uint8_t ack = buildAckByte(sensores.getEstado(),
                                 (uint8_t)calibration.getPhase());
      controlRX.updateAckPayload(ack);
      PayloadCtrl dummy;
      (void)controlRX.receive(dummy);   // drena o pipe e descarta
    }
    sensores.update();
    thermalTX.update();
    return;
  }

  // ── Radio (PRIMEIRO — prioridade maxima) ──────────────
  // ACK payload pre-carregado ANTES de chegar pacote. Depois
  // tenta ler o pacote pendente. Faz isto na 1a linha do loop
  // para minimizar input delay.
  //
  // CRITICO: cache estatico do ultimo comando recebido. O loop
  // corre a >100Hz mas os pacotes chegam a ~50Hz. Sem cache, nas
  // iteracoes sem pacote os ESCs reverteriam aos valores default
  // (THROTTLE_NORMAL/0/0/0), causando "saltinhos". Com cache, o
  // ultimo comando mantem-se ate chegar o seguinte.
  static int16_t radioThrottle = THROTTLE_MIN;
  static int16_t radioYaw   = 0;
  static int16_t radioPitch = 0;
  static int16_t radioRoll  = 0;
  static bool    radioArmed = false;
  PayloadCtrl cmd;
  bool gotCmd = false;

  if (radioCtrlOK) {
    controlRX.updateAckPayload(
      buildAckByte(sensores.getEstado(),
                   (uint8_t)calibration.getPhase()));
    gotCmd = controlRX.receive(cmd);
    if (gotCmd) {
      statPktCount++;
      radioArmed    = (cmd.armed == 1);
      radioThrottle = cmd.throttle;
      radioYaw      = cmd.yaw;
      radioPitch    = cmd.pitch;
      radioRoll     = cmd.roll;
    }
  }

  // Sensores (analog reads, nao bloqueia)
  sensores.update();

  // ── Stats a cada 2s ────────────────────────────────────
  unsigned long now = millis();
  if (now - statLastPrint > 2000) {
    unsigned long dtRx = radioCtrlOK ? (now - controlRX.lastReceiveTime()) : 0;
    unsigned long dtMs = now - statLastPrint;
    unsigned long loopHz = dtMs ? (statLoopCount * 1000UL) / dtMs : 0;
    Serial.print("[STAT] loops="); Serial.print(statLoopCount);
    Serial.print(" Hz=");          Serial.print(loopHz);
    Serial.print(" pkts=");        Serial.print(statPktCount);
    Serial.print(" dtRx=");        Serial.print(dtRx);
    Serial.print("ms escArm=");    Serial.print(armed);
    Serial.print(" ramp=");        Serial.print(rampActive);
    Serial.print(" radArm=");      Serial.print(radioArmed);
    Serial.print(" T=");           Serial.print(radioThrottle);
    Serial.print(" Y=");           Serial.print(radioYaw);
    Serial.print(" P=");           Serial.print(radioPitch);
    Serial.print(" R=");           Serial.print(radioRoll);
    Serial.print(" flame=");       Serial.print(sensores.getFlameVal());
    Serial.print(" coRaw=");       Serial.print(sensores.getCoVal());
    Serial.print(" coPpm=");       Serial.print(sensores.getCoPpm(), 1);
    if (!sensores.isAquecido()) {
      Serial.print(" preheat=");   Serial.print(sensores.getPreheatRemaining() / 1000);
      Serial.print("s");
    }
    Serial.println();
    statLoopCount = 0;
    statPktCount = 0;
    statLastPrint = now;
  }

  // ── Radio arming logic (nao mata botoes locais) ──────
  // CRITICO: so reagir a arm/disarm quando ha pacote novo (gotCmd).
  // O loop corre a >100Hz mas os pacotes chegam mais devagar — nas
  // iteracoes sem pacote, radioArmed=false (default), o que dispararia
  // disarm em loop. So o timeout (dtRx) corre todos os loops.
  if (radioCtrlOK) {
    unsigned long dtRx = now - controlRX.lastReceiveTime();
    if (dtRx > TIMEOUT_SINAL) {
      // Radio em timeout: so disarma se radio estava a controlar.
      // Caso contrario, deixa botoes locais + ramp continuar.
      if (lastRadioArmed) {
        if (armed) {
          emergencyStop();
          Serial.println("[GX] TIMEOUT radio");
        }
        lastRadioArmed = false;
      }
      // Repor cache para defaults seguros: se voltar sinal,
      // arrancamos sem comandos antigos perigosos a actuar.
      radioThrottle = THROTTLE_MIN;
      radioYaw = 0; radioPitch = 0; radioRoll = 0;
    } else if (gotCmd) {
      // Radio STOP (falling edge): ramp down gradual ate THROTTLE_MIN.
      // currentThrottle mantem-se — rampToTarget desce passo a passo.
      // armed so passa a false quando o ramp termina (ver rampToTarget).
      if (!radioArmed && armed && lastRadioArmed) {
        if (!disarming) {
          Serial.println("[GX] radio DISARM (ramp down)");
          targetThrottle = THROTTLE_MIN;
          rampActive     = true;
          disarming      = true;
        }
        lastRadioArmed = false;
      }
      // Radio ARM (rising edge): so arma se nao estava armado.
      // Disarm e tratado pelo falling edge acima (com ramp).
      else if (radioArmed && !lastRadioArmed) {
        if (!armed && !disarming) {
          Serial.println("[GX] radio ARM (ramp up)");
          armed           = true;
          disarming       = false;
          targetThrottle  = THROTTLE_NORMAL;
          currentThrottle = THROTTLE_MIN;
          rampActive      = true;
        }
        lastRadioArmed = radioArmed;
      } else {
        lastRadioArmed = radioArmed;
      }
    }
  }

  // ── Ramp (comum a radio + botoes) ─────────────────────
  // Sem delay() — espacamento dos passos feito por millis(),
  // para o loop continuar a correr a >100Hz e nao perder
  // pacotes de controlo durante o ramp.
  static unsigned long lastRampMs = 0;
  if (rampActive) {
    if (now - lastRampMs >= (unsigned long)RAMP_DELAY) {
      lastRampMs = now;
      rampToTarget();
    }
    thermalTX.update();
    return;
  }

  // ── Voo ───────────────────────────────────────────────
  // Usa SEMPRE o ultimo comando em cache (radio*). Se nao chegou
  // pacote nesta iteracao, mantem o valor anterior — sem isto os
  // ESCs ficavam aos saltinhos (1500 -> 1300 -> 1500 -> 1300...).
  // Seguranca: o timeout (>1s sem pacotes) repoe o cache para zeros
  // e dispara emergencyStop().
  if (armed) {
    int throttle;
    if      (radioThrottle <= THROTTLE_Y_LOW)  throttle = THROTTLE_Y_LOW;
    else if (radioThrottle >= THROTTLE_Y_HIGH) throttle = THROTTLE_Y_HIGH;
    else                                       throttle = radioThrottle;

    int yaw   = constrain((int)radioYaw,   -YAW_MAX, YAW_MAX);
    int pitch = constrain((int)radioPitch, -YAW_MAX, YAW_MAX);
    int roll  = constrain((int)radioRoll,  -YAW_MAX, YAW_MAX);

    writeESCsWithAll(throttle, yaw, pitch, roll);
  }

  thermalTX.update();
}
