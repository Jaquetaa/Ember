// ============================================
// EMBER — Gonçalo (Drone) — ESP32-S3
// ============================================

#include <Arduino.h>
#include <SPI.h>

#include "EmberSensors.h"
#include "EmberControlRX.h"
#include "EmberThermalTX.h"   // header leve — sem MLX/RF24/Wire

// LEDC directo (bypass ESP32Servo). ESP32Servo@3.1.3 no ESP32-S3 prefere
// MCPWM para Servo em freq fixa e trava dentro de mcpwm_init() com esta
// versao do IDF — allocateTimer() so afecta LEDC e nao resolve. Por isso
// geramos o PWM dos ESCs com ledcSetup/ledcAttachPin/ledcWrite.

// ── Pinos ───────────────────────────────────────────────
#define PIN_SCK  12
#define PIN_MISO 13
#define PIN_MOSI 11

#define ESC1_PIN 38
#define ESC2_PIN 39
#define ESC3_PIN 40
#define ESC4_PIN 41

#define TIMEOUT_SINAL 1000UL

#define BTN_ARM_PIN  1
#define BTN_STOP_PIN 2

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
int  currentThrottle = THROTTLE_MIN;
int  targetThrottle  = THROTTLE_MIN;

// ── Classes Ember (restantes) ───────────────────────────
EmberSensors   sensores;
EmberControlRX controlRX;
EmberThermalTX thermalTX;

// ── Estado radio/botoes ─────────────────────────────────
bool radioCtrlOK    = false;
bool lastRadioArmed = false;
bool lastBtnArm     = false;

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

void rampToTarget() {
  if (abs(currentThrottle - targetThrottle) > RAMP_STEP) {
    currentThrottle += (currentThrottle < targetThrottle) ? RAMP_STEP : -RAMP_STEP;
    currentThrottle = constrain(currentThrottle, THROTTLE_MIN, 2000);
    writeAllESCs(currentThrottle);
  } else {
    currentThrottle = targetThrottle;
    writeAllESCs(currentThrottle);
    rampActive = false;
    Serial.println("[ESC] Ramp concluido.");
  }
}

void emergencyStop() {
  armed           = false;
  rampActive      = false;
  targetThrottle  = THROTTLE_MIN;
  currentThrottle = THROTTLE_MIN;
  writeAllESCs(THROTTLE_MIN);
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

  // ── CS pins HIGH ──────────────────────────────────────
  DBG("[DEBUG] pinMode(5, OUTPUT) HIGH...");
  pinMode(5, OUTPUT);  digitalWrite(5, HIGH);
  DBG("[DEBUG] pin 5 OK");

  DBG("[DEBUG] pinMode(7, OUTPUT) HIGH...");
  pinMode(7, OUTPUT);  digitalWrite(7, HIGH);
  DBG("[DEBUG] pin 7 OK");
  delay(10);

  // ── Sensores ──────────────────────────────────────────
  DBG("[DEBUG] sensores.begin(14, 15)...");
  sensores.begin(14, 15);
  DBG("[DEBUG] sensores.begin OK");

  // ── SPI ───────────────────────────────────────────────
  DBG("[DEBUG] SPI.begin(12, 13, 11, 5)...");
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, 5);
  DBG("[DEBUG] SPI.begin OK");
  delay(10);

  // ── NRF Controlo ──────────────────────────────────────
  DBG("[DEBUG] controlRX.begin(4, 5)...");
  radioCtrlOK = controlRX.begin(4, 5, SPI);
  if (radioCtrlOK) {
    DBG("[DEBUG] controlRX.begin OK");
  } else {
    DBG("[EMBER] AVISO: NRF Controlo falhou — modo botoes (GPIO1/2)");
  }

  // Botoes fisicos (fallback joyleftxy)
  DBG("[DEBUG] pinMode botoes GPIO1/GPIO2 INPUT_PULLUP...");
  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  DBG("[DEBUG] botoes OK");

  // ── Thermal TX (tudo alocado dentro do begin) ─────────
  DBG("[DEBUG] thermalTX.begin(6, 7, 8, 9)...");
  if (!thermalTX.begin(6, 7, 8, 9, SPI)) {
    DBG("[EMBER] AVISO: Thermal falhou (continua sem camara)");
  } else {
    DBG("[DEBUG] thermalTX.begin OK");
  }

  DBG("[EMBER] === Pronto ===\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  static unsigned long statLoopCount = 0;
  static unsigned long statPktCount  = 0;
  static unsigned long statLastPrint = 0;
  statLoopCount++;

  sensores.update();

  // ── Botoes locais SEMPRE activos (override manual) ────
  bool btnStop = !digitalRead(BTN_STOP_PIN);
  bool btnArm  = !digitalRead(BTN_ARM_PIN);

  if (btnStop) {
    if (armed || rampActive) {
      emergencyStop();
      Serial.println("[LOCAL] STOP");
    }
    lastBtnArm = btnArm;
    thermalTX.update();
    delay(RAMP_DELAY);
    return;
  }
  if (btnArm && !lastBtnArm) {
    Serial.print("[LOCAL] ARM btn pressed, armed antes=");
    Serial.println(armed);
    armed           = !armed;
    targetThrottle  = armed ? THROTTLE_NORMAL : THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    rampActive      = true;
    delay(300);
  }
  lastBtnArm = btnArm;

  // ── Radio ─────────────────────────────────────────────
  PayloadCtrl cmd;
  bool gotCmd = false;
  bool radioArmed = false;
  int16_t radioThrottle = THROTTLE_MIN;
  int16_t radioYaw = 0;

  if (radioCtrlOK) {
    controlRX.updateAckPayload(sensores.getEstado());
    gotCmd = controlRX.receive(cmd);
    if (gotCmd) {
      statPktCount++;
      radioArmed    = (cmd.armed == 1);
      radioThrottle = cmd.throttle;
      radioYaw      = cmd.yaw;
    }
  }

  // ── Stats a cada 2s ────────────────────────────────────
  unsigned long now = millis();
  if (now - statLastPrint > 2000) {
    unsigned long dtRx = radioCtrlOK ? (now - controlRX.lastReceiveTime()) : 0;
    Serial.print("[STAT] loops="); Serial.print(statLoopCount);
    Serial.print(" pkts=");        Serial.print(statPktCount);
    Serial.print(" dtRx=");        Serial.print(dtRx);
    Serial.print("ms escArm=");    Serial.print(armed);
    Serial.print(" ramp=");        Serial.print(rampActive);
    Serial.print(" radArm=");      Serial.print(radioArmed);
    Serial.print(" T=");           Serial.print(radioThrottle);
    Serial.print(" Y=");           Serial.print(radioYaw);
    Serial.print(" btnArm=");      Serial.print(btnArm);
    Serial.print(" btnStop=");     Serial.print(btnStop);
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
    } else {
      // Radio STOP (desarma se radio diz armed=0 e estava a controlar)
      if (!radioArmed && armed && lastRadioArmed) {
        emergencyStop();
        Serial.println("[GX] radio DISARM");
        lastRadioArmed = false;
      }
      // Radio ARM toggle (rising edge)
      else if (radioArmed && !lastRadioArmed) {
        Serial.println("[GX] radio ARM rising edge");
        armed           = !armed;
        targetThrottle  = armed ? THROTTLE_NORMAL : THROTTLE_MIN;
        currentThrottle = THROTTLE_MIN;
        rampActive      = true;
        lastRadioArmed  = radioArmed;
      } else {
        lastRadioArmed = radioArmed;
      }
    }
  }

  // ── Ramp (comum a radio + botoes) ─────────────────────
  if (rampActive) {
    rampToTarget();
    thermalTX.update();
    delay(RAMP_DELAY);
    return;
  }

  // ── Voo ───────────────────────────────────────────────
  if (armed) {
    int throttle = THROTTLE_NORMAL;
    int yaw = 0;

    if (radioCtrlOK && gotCmd) {
      if      (radioThrottle <= THROTTLE_Y_LOW)  throttle = THROTTLE_Y_LOW;
      else if (radioThrottle >= THROTTLE_Y_HIGH) throttle = THROTTLE_Y_HIGH;
      else                                       throttle = radioThrottle;
      yaw = constrain((int)radioYaw, -YAW_MAX, YAW_MAX);
    }
    writeESCsWithYaw(throttle, yaw);
  }

  thermalTX.update();
  delay(RAMP_DELAY);
}
