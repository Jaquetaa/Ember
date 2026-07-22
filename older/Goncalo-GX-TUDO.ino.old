// ============================================
// Goncalo-GX — Drone (ESP32-S3)
// ESCs: logica identica ao joyleftXY-yaw-onoff.ino
// Sensores: Flame (GPIO14) + MQ-7 (GPIO15)
// Radio: nRF24 recebe cmds do Tiago, responde com ACK
//
// PINOS:
//   ESC1=38 | ESC2=39 | ESC3=40 | ESC4=41
//   Flame SIG -> GPIO14
//   MQ-7 AO   -> GPIO15
//   nRF24 CE=4 | CSN=5 | SCK=12 | MISO=13 | MOSI=11
//
// PAYLOAD recebido (struct PayloadCtrl, 5 bytes):
//   armed:    0 = stop | 1 = armado
//   throttle: microsegundos (1000-2000)
//   yaw:      -100 a +100
//
// ACK PAYLOAD enviado (1 byte):
//   0=OK | 1=CO/Gas | 2=Fogo | 3=Fogo+CO
// ============================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <RF24.h>

// ── Pinos ESC (igual ao joyleftXY) ─────────────────────
const int ESC1_PIN = 38;
const int ESC2_PIN = 39;
const int ESC3_PIN = 40;
const int ESC4_PIN = 41;

// ── Throttle (HobbyKing 20A ESC — igual ao joyleftXY) ──
const int THROTTLE_NORMAL = 1300;
const int THROTTLE_Y_LOW  = 1200;
const int THROTTLE_Y_HIGH = 1500;
const int THROTTLE_MIN    = 1000;
const int THROTTLE_ARM    = 1100;
const int YAW_MAX         = 100;
const int RAMP_STEP       = 2;
const int RAMP_DELAY      = 40;

// ── Sensores ────────────────────────────────────────────
#define FLAME_PIN       14
#define MQ7_PIN         15
#define FLAME_THRESHOLD 1000
#define CO_PPM_LIMITE   140
#define CONFIRMACOES    3
#define PREHEAT_MS      180000UL

// ── nRF24 ───────────────────────────────────────────────
#define CE_PIN   4
#define CSN_PIN  5
#define PIN_SCK  12
#define PIN_MISO 13
#define PIN_MOSI 11

// ── Timeout sem sinal ───────────────────────────────────
#define TIMEOUT_SINAL 1000UL

// ── Payload de controlo ─────────────────────────────────
struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
};

// ── Objetos ─────────────────────────────────────────────
Servo esc1, esc2, esc3, esc4;
RF24  radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[6] = "00001";

// ── Estado ESC (igual ao joyleftXY) ────────────────────
bool armed          = false;
bool rampActive     = false;
int  currentThrottle = THROTTLE_MIN;
int  targetThrottle  = THROTTLE_MIN;

// ── Comando recebido via radio ───────────────────────────
bool     radioArmed   = false;
bool     lastRadioArmed = false;
int16_t  radioThrottle = THROTTLE_MIN;
int16_t  radioYaw      = 0;
unsigned long ultimoComando = 0;

// ── Sensores ─────────────────────────────────────────────
bool          mq7Aquecido  = false;
unsigned long tempoInicio;
int           contadorChama = 0;
uint8_t       estadoSensor  = 0;

// ── Funcoes ESC (identicas ao joyleftXY) ────────────────
void writeAllESCs(int us) {
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

void writeESCsWithYaw(int base, int yaw) {
  int safeMin = armed ? THROTTLE_ARM + 50 : THROTTLE_MIN;
  // Layout: 1(CW) 2(CCW) / 3(CCW) 4(CW)
  esc1.writeMicroseconds(constrain(base + yaw, safeMin, 2000));
  esc2.writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  esc3.writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  esc4.writeMicroseconds(constrain(base + yaw, safeMin, 2000));
}

void rampToTarget() {
  if (abs(currentThrottle - targetThrottle) > RAMP_STEP) {
    currentThrottle += (currentThrottle < targetThrottle) ? RAMP_STEP : -RAMP_STEP;
    currentThrottle  = constrain(currentThrottle, THROTTLE_MIN, 2000);
    writeAllESCs(currentThrottle);
    Serial.print("Ramp -> "); Serial.println(currentThrottle);
  } else {
    currentThrottle = targetThrottle;
    writeAllESCs(currentThrottle);
    rampActive = false;
    Serial.println("Ramp concluido.");
  }
}

// ── Sensores ─────────────────────────────────────────────
uint8_t lerSensores() {
  int  flameVal   = analogRead(FLAME_PIN);
  contadorChama   = (flameVal < FLAME_THRESHOLD) ? contadorChama + 1 : 0;
  bool chama      = (contadorChama >= CONFIRMACOES);
  uint8_t estado  = 0;

  if (mq7Aquecido) {
    int   coVal  = analogRead(MQ7_PIN);
    float tensao = coVal * (3.3f / 4095.0f);
    float rs_ro  = (tensao > 0.01f) ? (3.3f - tensao) / tensao : 99.0f;
    float ppm    = constrain(100.0f * pow(rs_ro / 5.0f, -1.5f), 0.0f, 10000.0f);
    bool  coAlto = (ppm > CO_PPM_LIMITE);
    if      (chama && coAlto) estado = 3;
    else if (chama)           estado = 2;
    else if (coAlto)          estado = 1;
  } else {
    if (chama) estado = 2;
  }
  return estado;
}

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);

  // --- ESCs PRIMEIRO (igual ao joyleftXY) ---
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  esc1.attach(ESC1_PIN, 700, 2000);
  esc2.attach(ESC2_PIN, 700, 2000);
  esc3.attach(ESC3_PIN, 700, 2000);
  esc4.attach(ESC4_PIN, 700, 2000);

  writeAllESCs(THROTTLE_MIN);
  delay(100);

  // --- nRF24 ---
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);
  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[GX] ERRO: nRF24 nao iniciou! Verifica SPI/cabos.");
    while (1) delay(1000);
  }
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();
  radio.writeAckPayload(1, &estadoSensor, 1);

  tempoInicio   = millis();
  ultimoComando = millis();

  Serial.println("\n\n");
  Serial.println("---- Debug Console -----");
  Serial.println("Goncalo-GX (ESP32-S3 | HobbyKing 20A | nRF24)");
  Serial.println("Pronto. Aguarda comando ARM do Tiago.");
  Serial.println("A aquecer MQ-7 (3 min)... envia qualquer tecla para saltar.");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  unsigned long agora = millis();

  // --- Preheat MQ-7 (nao bloqueia o loop) ---
  if (!mq7Aquecido) {
    if (Serial.available() > 0) {
      while (Serial.available()) Serial.read();
      mq7Aquecido = true;
      Serial.println(">>> Aquecimento saltado! <<<");
    } else if ((agora - tempoInicio) >= PREHEAT_MS) {
      mq7Aquecido = true;
      Serial.println("MQ-7 pronto!");
    }
  }

  // --- Sensores + atualiza ACK payload ---
  estadoSensor = lerSensores();
  radio.writeAckPayload(1, &estadoSensor, 1);

  // --- Recebe comando do Tiago ---
  if (radio.available()) {
    PayloadCtrl p;
    while (radio.available()) radio.read(&p, sizeof(p));
    ultimoComando  = agora;
    radioArmed     = (p.armed == 1);
    radioThrottle  = p.throttle;
    radioYaw       = p.yaw;
  }

  // --- Timeout: sem sinal -> STOP forcado ---
  if ((agora - ultimoComando) > TIMEOUT_SINAL) {
    if (radioArmed) {
      radioArmed    = false;
      Serial.println("[GX] TIMEOUT — sem sinal do Tiago! STOP de seguranca.");
    }
  }

  // ================================================================
  // LOGICA ESC — identica ao joyleftXY, so substitui botoes/joystick
  // por radioArmed / radioThrottle / radioYaw
  // ================================================================

  // STOP de emergencia (radioArmed=false enquanto estava armado)
  bool stopCmd = (!radioArmed && armed);
  if (stopCmd) {
    armed           = false;
    rampActive      = false;
    targetThrottle  = THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    writeAllESCs(THROTTLE_MIN);
    Serial.println("[GX] Emergency STOP");
    delay(300);
    lastRadioArmed = radioArmed;
    return;
  }

  // ARM toggle (rising edge — igual ao joyleftXY)
  if (radioArmed && !lastRadioArmed) {
    armed          = !armed;
    targetThrottle  = armed ? THROTTLE_NORMAL : THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    rampActive      = true;
    Serial.println(armed ? "[GX] ARM: ON" : "[GX] ARM: OFF");
  }
  lastRadioArmed = radioArmed;

  // Ramp (igual ao joyleftXY)
  if (rampActive) {
    rampToTarget();
    delay(RAMP_DELAY);
    return;
  }

  // Voo normal (igual ao joyleftXY mas usa radioThrottle/radioYaw)
  if (armed) {
    int throttle;
    int y = radioThrottle; // throttle ja vem calculado do Tiago
    int x = radioYaw;

    // Mantém os mesmos limites do joyleftXY
    if      (y <= THROTTLE_Y_LOW)  throttle = THROTTLE_Y_LOW;
    else if (y >= THROTTLE_Y_HIGH) throttle = THROTTLE_Y_HIGH;
    else                           throttle = y;

    int yaw = constrain((int)x, -YAW_MAX, YAW_MAX);

    targetThrottle = throttle;
    writeESCsWithYaw(throttle, yaw);

    Serial.print("[GX] T="); Serial.print(throttle);
    Serial.print(" Yaw=");   Serial.print(yaw);
    Serial.print(" sensor=");Serial.println(estadoSensor);
  }

  delay(RAMP_DELAY);
}
