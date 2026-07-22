#include <Arduino.h>

// LEDC directo (bypass ESP32Servo) — ESP32Servo usa MCPWM e é instável
// no ESP32-S3 com esta versao do IDF (ver Goncalo-Ember.ino). Mantemos
// a calibracao consistente com o firmware principal.

const int ESC1_PIN = 19;
const int ESC2_PIN = 14;
const int ESC3_PIN = 1;
const int ESC4_PIN = 4;

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

void writeAll(int us) {
  uint32_t d = escUsToDuty((uint32_t)us);
  ledcWrite(ESC1_CH, d);
  ledcWrite(ESC2_CH, d);
  ledcWrite(ESC3_CH, d);
  ledcWrite(ESC4_CH, d);
}

void setup() {
  Serial.begin(115200);

  double f1 = ledcSetup(ESC1_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f2 = ledcSetup(ESC2_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f3 = ledcSetup(ESC3_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  double f4 = ledcSetup(ESC4_CH, ESC_FREQ_HZ, ESC_RES_BITS);
  if (f1 == 0 || f2 == 0 || f3 == 0 || f4 == 0) {
    Serial.println("[CAL] ERRO FATAL: ledcSetup falhou (freq==0)");
    while (1) { delay(1000); }
  }

  ledcAttachPin(ESC1_PIN, ESC1_CH);
  ledcAttachPin(ESC2_PIN, ESC2_CH);
  ledcAttachPin(ESC3_PIN, ESC3_CH);
  ledcAttachPin(ESC4_PIN, ESC4_CH);

  writeAll(1000); // garante MIN antes de ligar a bateria dos ESC

  Serial.println("=== CALIBRACAO ESC HobbyKing 20A ===");
  Serial.println(">> LIGA A BATERIA AGORA <<");
  delay(8000);

  Serial.println("PASSO 1: A enviar maximo (2000us)...");
  writeAll(2000);
  delay(5000);

  Serial.println("PASSO 2: A enviar minimo (1000us)...");
  writeAll(1000);
  delay(5000);

  Serial.println("Calibracao concluida! Carrega o codigo principal.");
}

void loop() {}
