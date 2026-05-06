// ============================================
// EMBER — Tiago (Controlador de Chão) — ESP32-S3
//
// Combina: Joystick L+R + ARM/STOP + Alarme + Thermal RX
//
// DOIS nRF24L01 no mesmo bus SPI:
//   Controlo:  CE=14 | CSN=13  (canal 76, ACK, envia comandos)
//   Termico:   CE=12 | CSN=11  (canal 2, sem ACK, recebe frames)
//
// TFT ST7796S (TFT_eSPI): CS=41 | DC=21 | RST=42 | BL=45
// SPI partilhado: SCK=48 | MISO=16 | MOSI=47
//
// Joystick esquerdo: Y=GPIO1  | X=GPIO2  (throttle/yaw)
// Joystick direito:  Y=GPIO4  | X=GPIO5  (pitch/roll)
// Botoes:   ARM=GPIO17 | STOP=GPIO18
// Alarme:   Buzzer (unico passivo)=GPIO10 | LED gas=GPIO3 | LED chama=GPIO8
//
// Payload enviado (9 bytes — struct PayloadCtrl):
//   armed:    0 = desarmado (ramp down gradual), 1 = armado, 2 = EMERGENCY STOP (imediato)
//   throttle: microsegundos (1000-1500)
//   yaw:      -100 a +100
//   pitch:    -100 a +100  (frente/trás — stick direito Y)
//   roll:     -100 a +100  (esquerda/direita — stick direito X)
//
// SERIAL DEBUG: usa COM12 (USB CDC nativo, ARDUINO_USB_CDC_ON_BOOT=1)
//               NAO COM11 (UART bridge)
// ============================================

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#include "EmberJoystick.h"
#include "EmberBuzzer.h"
#include "EmberNRFTX.h"
#include "EmberDisplayRX.h"

// ── Pinos ───────────────────────────────────────────────
#define SPI_SCK   48
#define SPI_MISO  16
#define SPI_MOSI  47

#define BTN_ARM_PIN  17
#define BTN_STOP_PIN 18

// ── Objectos simples (sem RF24/TFT — construtores seguros) ─────────────
EmberJoystick joystick(1, 2);   // esquerdo: Y=1, X=2
EmberJoystick joystickR(4, 5);  // direito:  Y=4, X=5
EmberBuzzer   buzzer;           // buzzer unico: buzz=GPIO10, ledGas=GPIO3, ledFire=GPIO8

// ── Objectos com RF24/TFT — criados em setup() para evitar crash ───────
EmberNRFTX* nrfTX  = nullptr;
EmberDisplayRX* displayRX = nullptr;
TFT_eSPI*   tft    = nullptr;

// ── Estado ──────────────────────────────────────────────
bool armed = false;
bool lastArmBtn = false;
bool lastStopBtn = false;
uint8_t lastEstado = 255;
uint32_t lastArmPress = 0;
uint32_t lastStopPress = 0;

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);  // Espera USB CDC enumerar

  Serial.println("\n\n========================================");
  Serial.println("[EMBER] Tiago — Controlador de Chão");
  Serial.println("[EMBER] ESP32-S3 | Arduino-ESP32 3.x");
  Serial.println("========================================");
  Serial.println("[EMBER] NOTA: Serial ativo em COM12 (USB CDC)");
  Serial.println("[EMBER] Inicio do setup...");
  Serial.flush();

  // ── Botões ──────────────────────────────────────────
  Serial.println("[SETUP] Configurando botoes ARM (GPIO17) e STOP (GPIO18)...");
  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  Serial.print("[SETUP]   ARM  (GPIO17) = ");
  Serial.println(digitalRead(BTN_ARM_PIN)  ? "HIGH (solto)" : "LOW (premido)");
  Serial.print("[SETUP]   STOP (GPIO18) = ");
  Serial.println(digitalRead(BTN_STOP_PIN) ? "HIGH (solto)" : "LOW (premido)");

  // ── CS pins HIGH antes de tudo ──────────────────────
  Serial.println("[SETUP] Forçando CS pins HIGH antes de inicializar SPI...");
  pinMode(13, OUTPUT);  digitalWrite(13, HIGH);
  Serial.println("[SETUP]   CSN NRF Controlo (GPIO13) = HIGH");
  pinMode(11, OUTPUT);  digitalWrite(11, HIGH);
  Serial.println("[SETUP]   CSN NRF Termico  (GPIO11) = HIGH");
  pinMode(41, OUTPUT);  digitalWrite(41, HIGH);
  Serial.println("[SETUP]   CS  TFT           (GPIO41) = HIGH");
  delay(10);

  // ── Joystick esquerdo ───────────────────────────────
  Serial.println("[SETUP] Iniciando joystick ESQUERDO (Y=GPIO1, X=GPIO2)...");
  joystick.begin();
  Serial.print("[SETUP]   Raw Y (throttle): "); Serial.println(analogRead(1));
  Serial.print("[SETUP]   Raw X (yaw):      "); Serial.println(analogRead(2));
  Serial.println("[SETUP]   Joystick esquerdo OK");

  // ── Joystick direito ────────────────────────────────
  Serial.println("[SETUP] Iniciando joystick DIREITO (Y=GPIO4, X=GPIO5)...");
  joystickR.begin();
  Serial.print("[SETUP]   Raw Y (pitch): "); Serial.println(analogRead(4));
  Serial.print("[SETUP]   Raw X (roll):  "); Serial.println(analogRead(5));
  Serial.println("[SETUP]   Joystick direito OK");

  // ── Buzzer ──────────────────────────────────────────
  Serial.println("[SETUP] Iniciando buzzer (buzz=GPIO10, ledGas=GPIO3, ledFire=GPIO8)...");
  buzzer.begin(10, 3, 8);
  Serial.println("[SETUP]   Buzzer OK");

  // ── Backlight ───────────────────────────────────────
  Serial.println("[SETUP] Ativando backlight TFT (GPIO45 = HIGH)...");
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);
  Serial.println("[SETUP]   Backlight ON");

  // ── SPI Bus ─────────────────────────────────────────
  Serial.println("[SETUP] Iniciando bus SPI2 (SCK=48, MISO=16, MOSI=47)...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
  delay(10);
  Serial.println("[SETUP]   SPI2 bus OK");

  // ── TFT ─────────────────────────────────────────────
  Serial.println("[SETUP] Criando objeto TFT_eSPI (ST7796S)...");
  tft = new TFT_eSPI();
  Serial.println("[SETUP]   Objeto TFT criado");

  Serial.println("[SETUP] Chamando tft->init()...");
  tft->init();
  Serial.println("[SETUP]   tft->init() concluido sem crash!");

  Serial.println("[SETUP] Configurando rotacao, cor, cursor...");
  tft->setRotation(1);
  tft->fillScreen(TFT_BLACK);
  tft->setTextSize(2);
  tft->setTextColor(TFT_GREEN);
  tft->setCursor(50, 120);
  Serial.println("[SETUP]   Ecrã preenchido a preto, cursor posicionado");

  Serial.println("[SETUP] Imprimindo texto no TFT...");
  tft->print("EMBER - Iniciando...");
  Serial.println("[SETUP]   Texto TFT impresso sem crash!");
  Serial.println("[SETUP]   >>> TFT OK <<<");

  // ── NRF Controlo ────────────────────────────────────
  Serial.println("[SETUP] Criando EmberNRFTX (CE=GPIO14, CSN=GPIO13)...");
  nrfTX = new EmberNRFTX(14, 13);
  Serial.println("[SETUP]   Objeto EmberNRFTX criado");

  Serial.println("[SETUP] Chamando nrfTX->begin(SPI)...");
  bool nrfCtrlOk = nrfTX->begin(SPI);
  if (nrfCtrlOk) {
    Serial.println("[SETUP]   >>> NRF TX OK: canal=76, 250KBPS, ACK <<<");
  } else {
    Serial.println("[SETUP]   !!! NRF TX FALHOU — verifica ligacoes CE/CSN/VCC !!!");
    Serial.println("[SETUP]       CE=GPIO14, CSN=GPIO13, VCC=3.3V");
    Serial.println("[SETUP]       MOSI=47, MISO=16, SCK=48");
    tft->setCursor(0, 160);
    tft->setTextColor(TFT_RED);
    tft->print("NRF TX: FALHOU!");
    tft->setTextColor(TFT_GREEN);
  }

  // ── NRF Termico ─────────────────────────────────────
  Serial.println("[SETUP] Criando EmberDisplayRX (CE=GPIO12, CSN=GPIO11)...");
  displayRX = new EmberDisplayRX(12, 11);
  Serial.println("[SETUP]   Objeto EmberDisplayRX criado");

  Serial.println("[SETUP] Chamando displayRX->begin(SPI, *tft)...");
  displayRX->begin(SPI, *tft);
  Serial.println("[SETUP]   displayRX->begin() concluido");

  // ── Leitura inicial dos joysticks ───────────────────
  Serial.println("[SETUP] Leituras iniciais dos joysticks:");
  Serial.print("[SETUP]   Throttle = "); Serial.println(joystick.readThrottle());
  Serial.print("[SETUP]   Yaw      = "); Serial.println(joystick.readYaw());
  Serial.print("[SETUP]   Pitch    = "); Serial.println(joystickR.readPitch());
  Serial.print("[SETUP]   Roll     = "); Serial.println(joystickR.readRoll());

  // ── Estado dos botões no arranque ───────────────────
  Serial.print("[SETUP] Botao ARM  no arranque: ");
  Serial.println(!digitalRead(BTN_ARM_PIN)  ? "PREMIDO" : "solto");
  Serial.print("[SETUP] Botao STOP no arranque: ");
  Serial.println(!digitalRead(BTN_STOP_PIN) ? "PREMIDO" : "solto");

  // ── Ecrã final ──────────────────────────────────────
  tft->fillScreen(TFT_BLACK);
  tft->setTextSize(2);
  tft->setTextColor(TFT_GREEN);
  tft->setCursor(10, 10);
  tft->print("EMBER OK");
  tft->setCursor(10, 40);
  tft->setTextColor(nrfCtrlOk ? TFT_GREEN : TFT_RED);
  tft->print(nrfCtrlOk ? "NRF TX: OK" : "NRF TX: FAIL");
  tft->setCursor(10, 70);
  tft->setTextColor(TFT_CYAN);
  tft->print("DESARMADO");
  tft->setTextSize(2);
  tft->setTextColor(TFT_WHITE);
  tft->setCursor(5, 300);
  tft->print("ESPERANDO CONNECTION...");

  Serial.println("\n========================================");
  Serial.println("[EMBER] === Setup completo! Loop a iniciar ===");
  Serial.println("========================================\n");
  Serial.flush();
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  bool armBtn  = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);

  // --- STOP de emergencia (debounce por millis, sem delay) ---
  // armed=2: sinal de corte IMEDIATO — drone nao faz ramp, corta ESCs na hora
  if (stopBtn && !lastStopBtn && (millis() - lastStopPress > 200)) {
    lastStopPress = millis();
    if (armed) Serial.println("[LOOP] !!! EMERGENCY STOP PREMIDO — corte imediato !!!");
    armed = false;
    // Envia 3x para garantir que pelo menos 1 pacote chega
    for (int i = 0; i < 3; i++) nrfTX->send(ARMED_EMERGENCY, THROTTLE_MIN, 0, 0, 0);
    Serial.println("[TX] Emergency STOP (armed=2) enviado 3x");
  }
  lastStopBtn = stopBtn;

  // --- ARM toggle (rising edge, debounce por millis, sem delay) ---
  if (armBtn && !lastArmBtn && (millis() - lastArmPress > 200)) {
    lastArmPress = millis();
    armed = !armed;
    if (armed) {
      Serial.println("[TX] ARM: ON — drone armado!");
      tft->fillRect(10, 70, 200, 25, TFT_BLACK);
      tft->setCursor(10, 70);
      tft->setTextColor(TFT_RED);
      tft->setTextSize(2);
      tft->print("ARMADO");
    } else {
      Serial.println("[TX] ARM: OFF — drone desarmado");
      tft->fillRect(10, 70, 200, 25, TFT_BLACK);
      tft->setCursor(10, 70);
      tft->setTextColor(TFT_CYAN);
      tft->setTextSize(2);
      tft->print("DESARMADO");
    }
  }
  lastArmBtn = armBtn;

  // --- Joystick esquerdo ---
  int throttle = armed ? joystick.readThrottle() : THROTTLE_MIN;
  int yaw      = armed ? joystick.readYaw()      : 0;

  // --- Joystick direito ---
  int pitch = armed ? joystickR.readPitch() : 0;
  int roll  = armed ? joystickR.readRoll()  : 0;

  // --- Enviar comando ---
  bool ok = nrfTX->send(armed ? ARMED_ON : ARMED_OFF, (int16_t)throttle, (int16_t)yaw,
                          (int16_t)pitch, (int16_t)roll);

  // --- Buzzer / alarmes de sensor ---
  // ackByte: bits 0-1 = sensor estado | bits 2-4 = CalPhase (0-7)
  uint8_t rawEstado = nrfTX->getLastAckByte();
  uint8_t calPhase  = (rawEstado >> 2) & 0x07;
  uint8_t rawSensor = rawEstado & 0x03;

  static uint8_t sensorPrev    = 0;
  static uint8_t sensorConfirm = 0;
  static uint8_t sensor        = 0;

  if (calPhase != 0) {
    // Em calibracao: reset do debounce do sensor
    sensorConfirm = 0; sensorPrev = 0; sensor = 0;
  } else if (rawSensor == 0) {
    sensorConfirm = 0; sensor = 0;
  } else {
    // Sensores 1-3: debounce de 4 leituras consecutivas
    if (rawSensor == sensorPrev) {
      if (sensorConfirm < 4) sensorConfirm++;
    } else {
      sensorPrev    = rawSensor;
      sensorConfirm = 1;
    }
    if (sensorConfirm >= 4) sensor = rawSensor;
  }

  if (sensor != lastEstado) {
    lastEstado = sensor;
    Serial.print("[SENSOR] Estado mudou para: "); Serial.println(sensor);
  }

  buzzer.update((calPhase << 2) | sensor);

  // --- Thermal display ---
  displayRX->update();

  // --- Debug (a cada ~1s) ---
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    lastDebug = millis();

    // Linha 1: estado de controlo (joystick/botoes)
    Serial.print("[CTRL] ARM=");  Serial.print(armed ? "ON " : "OFF");
    Serial.print(" T=");          Serial.print(throttle);
    Serial.print(" Yaw=");        Serial.print(yaw);
    Serial.print(" Pitch=");      Serial.print(pitch);
    Serial.print(" Roll=");       Serial.print(roll);
    Serial.print(" | Uptime=");   Serial.print(millis() / 1000);
    Serial.println("s");

    // Linha 2: estado do RF de controlo
    uint16_t fails = nrfTX->getConsecutiveFails();
    uint32_t sent  = nrfTX->getTotalSent();
    uint32_t okCnt = nrfTX->getTotalOk();
    Serial.print("[NRF-TX] ");
    Serial.print(ok ? "OK  " : "FAIL");
    Serial.print(" | Falhas=");   Serial.print(fails);
    Serial.print(" | Taxa=");     Serial.print(sent > 0 ? (okCnt * 100 / sent) : 0);
    Serial.print("% (");          Serial.print(okCnt);
    Serial.print("/");            Serial.print(sent);
    Serial.print(") | ACK=");      Serial.print(rawEstado);
    Serial.print(" Sensor=");      Serial.println(sensor);

    // Raw ADC a cada 5s para debug de joystick
    static uint8_t adcCount = 0;
    if (++adcCount >= 5) {
      adcCount = 0;
      Serial.print("[ADC]  RawL_Y="); Serial.print(analogRead(1));
      Serial.print(" RawL_X=");       Serial.print(analogRead(2));
      Serial.print(" | RawR_Y=");     Serial.print(analogRead(4));
      Serial.print(" RawR_X=");       Serial.println(analogRead(5));
    }
  }
}
