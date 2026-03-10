// ============================================
// Gonçalo-GX — Transmissor
// Sensores: Flame (Grove v1.1) + MQ-7 CO
// Envia estado ao Tiago-TX via nRF24
//
// PINOS:
//   Flame SIG -> GPIO14
//   MQ-7 AO  -> GPIO15  (GPIO13 conflitua com MISO do nRF24!)
//   nRF24 CE -> GPIO4  | CSN -> GPIO5
//   SCK=12, MISO=13, MOSI=11
//
// PAYLOAD (1 byte):
//   0 = OK
//   1 = CO/Gás alto
//   2 = Fogo detetado
//   3 = Fogo + CO em simultâneo
// ============================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- Sensores ---
#define FLAME_PIN       14
#define MQ7_PIN         15    // GPIO13 conflitua com MISO -> usa 15
#define FLAME_THRESHOLD 1000
#define CO_PPM_LIMITE   500
#define CONFIRMACOES    3
#define PREHEAT_MS      180000UL

// --- nRF24 ---
#define CE_PIN   4
#define CSN_PIN  5
#define PIN_SCK  12
#define PIN_MISO 13
#define PIN_MOSI 11

RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[6] = "00001";

int           contadorChama = 0;
bool          mq7Aquecido   = false;
unsigned long tempoInicio;

uint8_t       estadoAtual   = 0;
uint8_t       ultimoEnvio   = 255;
unsigned long ultimoReenvio = 0;
const unsigned long REENVIO_MS = 2000;

// -------------------------------------------------------
void enviarEstado(uint8_t estado) {
  bool ok = radio.write(&estado, sizeof(estado));
  Serial.print("[GX] ENVIO estado="); Serial.print(estado);
  Serial.print("  write="); Serial.println(ok ? "OK(ACK)" : "FAIL(no ACK)");
  ultimoEnvio   = estado;
  ultimoReenvio = millis();
}

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);

  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[GX] ERRO: nRF24 nao iniciou (SPI/cabos/alimentacao).");
    while (1) delay(1000);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();

  tempoInicio = millis();

  Serial.println("=================================");
  Serial.println("  GX — Sensores Gas/Fogo + nRF24 ");
  Serial.println("=================================");
  Serial.println("A aquecer sensor MQ-7 (3 min)...");
  Serial.println(">>> Carrega qualquer tecla para saltar o aquecimento <<<");
}

// -------------------------------------------------------
void loop() {
  unsigned long agora = millis();

  // --- Preheat MQ-7 ---
  if (!mq7Aquecido) {
    // Verifica se chegou alguma tecla no terminal -> salta preheat
    if (Serial.available() > 0) {
      while (Serial.available()) Serial.read(); // limpa o buffer
      mq7Aquecido = true;
      Serial.println(">>> Aquecimento saltado! A assumir MQ-7 pronto. <<<");
      Serial.println("---------------------------------");
      return;
    }

    unsigned long decorrido = agora - tempoInicio;
    if (decorrido < PREHEAT_MS) {
      Serial.print("MQ-7 a aquecer... faltam ");
      Serial.print((PREHEAT_MS - decorrido) / 1000);
      Serial.println("s  (carrega qualquer tecla para saltar)");
      if (agora - ultimoReenvio >= REENVIO_MS) enviarEstado(0);
      delay(5000);
      return;
    }

    mq7Aquecido = true;
    Serial.println("MQ-7 pronto!");
    Serial.println("---------------------------------");
  }

  // --- Flame ---
  int flameVal  = analogRead(FLAME_PIN);
  contadorChama = (flameVal < FLAME_THRESHOLD) ? contadorChama + 1 : 0;
  bool chama    = (contadorChama >= CONFIRMACOES);

  // --- CO / MQ-7 ---
  int   coVal  = analogRead(MQ7_PIN);
  float tensao = coVal * (3.3f / 4095.0f);
  float rs_ro  = (3.3f - tensao) / tensao;
  float ppm    = constrain(100.0f * pow(rs_ro / 5.0f, -1.5f), 0.0f, 10000.0f);
  bool  coAlto = (ppm > CO_PPM_LIMITE);

  // --- Determina estado a enviar ---
  uint8_t novoEstado = 0;
  if      (chama && coAlto) novoEstado = 3;
  else if (chama)           novoEstado = 2;
  else if (coAlto)          novoEstado = 1;

  estadoAtual = novoEstado;

  // --- Envia se estado mudou OU reenvio periódico ---
  if (estadoAtual != ultimoEnvio || (agora - ultimoReenvio) >= REENVIO_MS) {
    enviarEstado(estadoAtual);
  }

  // --- Serial debug ---
  Serial.print("FLAME: "); Serial.print(flameVal);
  Serial.print(" | Chama: "); Serial.print(chama ? "SIM" : "NAO");
  Serial.print(" || CO: "); Serial.print(ppm, 1);
  Serial.print(" ppm | Estado: ");
  if      (estadoAtual == 3) Serial.println("3 — FOGO + CO");
  else if (estadoAtual == 2) Serial.println("2 — FOGO");
  else if (estadoAtual == 1) Serial.println("1 — CO/Gas");
  else                       Serial.println("0 — OK");
  Serial.println("---------------------------------");

  delay(100);
}
