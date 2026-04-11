// ============================================
// EMBER — Tiago (Controlador de Chão) — ESP32
//
// Combina: Joystick + ARM/STOP + Alarme + Thermal RX
//
// DOIS nRF24L01 no mesmo bus SPI:
//   Controlo:  CE=4  | CSN=5   (canal 76, ACK, envia comandos)
//   Termico:   CE=33 | CSN=32  (canal 2, sem ACK, recebe frames)
//
// TFT ST7796S (TFT_eSPI): CS=27 | DC=12 | RST=14 | T_CS=13
// SPI partilhado: SCK=18 | MISO=19 | MOSI=23
//
// Joystick: Y=GPIO34 | X=GPIO35
// Botoes:   ARM=GPIO25 | STOP=GPIO26
// Alarme:   Buzzer=GPIO21 | LED=GPIO22
//
// User_Setup.h do TFT_eSPI:
//   #define ST7796_DRIVER
//   #define TFT_MOSI 23
//   #define TFT_MISO 19
//   #define TFT_SCLK 18
//   #define TFT_CS   27
//   #define TFT_DC   12
//   #define TFT_RST  14
//   #define TOUCH_CS 13
//   #define SPI_FREQUENCY       40000000
//   #define SPI_READ_FREQUENCY  20000000
// ============================================

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#include "EmberJoystick.h"
#include "EmberAlarm.h"
#include "EmberControlTX.h"
#include "EmberThermalRX.h"

// ── Pinos ───────────────────────────────────────────────
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23

#define BTN_ARM_PIN  25
#define BTN_STOP_PIN 26

// ── Objectos ────────────────────────────────────────────
EmberJoystick  joystick(34, 35);
EmberAlarm     alarme(21, 22);
EmberControlTX controlTX(4, 5);
EmberThermalRX thermalRX(33, 32);
TFT_eSPI       tft = TFT_eSPI();

// ── Estado ──────────────────────────────────────────────
bool armed = false;
bool lastArmBtn = false;
uint8_t estadoSensor = 0;
uint8_t lastEstado = 255;

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[EMBER] Tiago — Controlador");

  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);

  // CS pins HIGH antes de tudo
  pinMode(5, OUTPUT);   digitalWrite(5, HIGH);    // Control NRF CSN
  pinMode(32, OUTPUT);  digitalWrite(32, HIGH);   // Thermal NRF CSN
  pinMode(27, OUTPUT);  digitalWrite(27, HIGH);   // TFT CS
  pinMode(13, OUTPUT);  digitalWrite(13, HIGH);   // Touch CS
  delay(10);

  // SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, 5);
  delay(10);

  // Joystick + Alarme
  joystick.begin();
  alarme.begin();

  // NRF Controlo (canal 76)
  if (!controlTX.begin(SPI)) {
    Serial.println("[EMBER] ERRO: NRF Controlo falhou!");
  }

  // TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(50, 140);
  tft.print("EMBER - Iniciando...");

  // NRF Termico (canal 2)
  thermalRX.begin(SPI, tft);

  Serial.println("[EMBER] === Pronto ===\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  bool armBtn  = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);

  // --- STOP de emergencia ---
  if (stopBtn) {
    armed = false;
    controlTX.send(0, THROTTLE_MIN, 0);
    Serial.println("[TX] Emergency STOP");
    delay(300);
    alarme.update();
    thermalRX.update();
    return;
  }

  // --- ARM toggle (rising edge) ---
  if (armBtn && !lastArmBtn) {
    armed = !armed;
    Serial.println(armed ? "[TX] ARM: ON" : "[TX] ARM: OFF");
    delay(300);
  }
  lastArmBtn = armBtn;

  // --- Joystick ---
  int throttle = armed ? joystick.readThrottle() : THROTTLE_MIN;
  int yaw      = armed ? joystick.readYaw()      : 0;

  // --- Enviar comando ---
  bool ok = controlTX.send((uint8_t)armed, (int16_t)throttle, (int16_t)yaw);

  // --- Verificar estado dos sensores ---
  uint8_t sensor = controlTX.getLastSensorState();
  if (sensor != lastEstado) {
    lastEstado = sensor;
    alarme.aplicarEstado(sensor);
  }

  // --- Alarme ---
  alarme.update();

  // --- Thermal display ---
  thermalRX.update();

  // --- Debug (a cada ~1s) ---
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    lastDebug = millis();
    Serial.print("ARM="); Serial.print(armed);
    Serial.print(" T=");  Serial.print(throttle);
    Serial.print(" Yaw=");Serial.print(yaw);
    Serial.print(" RF="); Serial.print(ok ? "OK" : "FAIL");
    Serial.print(" Sensor="); Serial.println(sensor);
  }
}
