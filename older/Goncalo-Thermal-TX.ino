// ============================================
// Goncalo-Thermal-TX v5 — Drone (ESP32-S3)
// MELHORIAS v5:
//   - PA_MAX para maximo alcance
//   - 250KBPS (mais sensibilidade = mais distancia)
//   - Canal 2 (fora da zona WiFi 2.4GHz)
//   - CRC desativado (menos overhead, mais rapido)
//   - Envio 3x por pacote para redundancia extra
//   - Delay entre pacotes ajustado para 250kbps
//
// PINOS (ESP32-S3):
//   MLX90640:  SDA=8  | SCL=9
//   nRF24L01:  CE=6   | CSN=7   | SCK=12 | MISO=13 | MOSI=11
//
// CANAL RF: 2 | ADDRESS: "THERM" | DataRate: 250KBPS
// ============================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <RF24.h>

// ── I2C para MLX90640 ───────────────────────────────────
#define SDA_PIN  8
#define SCL_PIN  9

// ── nRF24L01 termico ────────────────────────────────────
#define THERM_CE   6
#define THERM_CSN  7
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11

// ── Mapeamento temperatura -> byte ──────────────────────
#define TEMP_MIN  20.0f
#define TEMP_MAX  60.0f

// ── Objetos ─────────────────────────────────────────────
Adafruit_MLX90640 mlx;
float    mlxFrame[768];
uint8_t  thermalData[768];

RF24 radioTherm(THERM_CE, THERM_CSN);
const uint8_t THERM_ADDR[6] = "THERM";
bool nrfOK = false;

struct ThermalPacket {
  uint8_t index;
  uint8_t data[24];
};

uint32_t frameCount = 0;

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[THERM-TX] Goncalo Thermal TX v5 (MAX RANGE)");

  // CS pins HIGH
  pinMode(THERM_CSN, OUTPUT);
  digitalWrite(THERM_CSN, HIGH);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  delay(10);

  // I2C para MLX90640
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000);  // 1MHz I2C — MLX90640 suporta

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("[THERM-TX] ERRO: MLX90640 nao encontrado!");
    while (1) delay(1000);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_8_HZ);
  Serial.println("[THERM-TX] MLX90640 OK (8Hz, 1MHz I2C).");

  // SPI + NRF
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, THERM_CSN);
  delay(10);

  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-TX] NRF tentativa ");
    Serial.print(t);
    Serial.print("/3 ... ");
    if (radioTherm.begin(&SPI)) {
      if (radioTherm.isChipConnected()) {
        Serial.println("OK!");
        nrfOK = true;
        break;
      }
    }
    Serial.println("FALHOU");
    delay(100);
  }

  if (nrfOK) {
    radioTherm.setChannel(2);               // Canal 2 — fora da zona WiFi
    radioTherm.setDataRate(RF24_250KBPS);    // 250kbps = maxima sensibilidade = mais alcance
    radioTherm.setPALevel(RF24_PA_HIGH);      // Potencia maxima de transmissao
    radioTherm.setAutoAck(false);            // Sem ACK — streaming unidirecional
    radioTherm.setPayloadSize(25);           // 1 byte index + 24 bytes dados
    radioTherm.setCRCLength(RF24_CRC_DISABLED); // Sem CRC — menos overhead
    radioTherm.openWritingPipe(THERM_ADDR);
    radioTherm.stopListening();
    Serial.println("[THERM-TX] nRF24 OK: canal=2, 250KBPS, PA_MAX, CRC OFF");
  } else {
    Serial.println("[THERM-TX] *** NRF FALHOU ***");
  }

  Serial.println("[THERM-TX] === Pronto ===\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  if (!nrfOK) { delay(1000); return; }

  // Ler frame
  if (mlx.getFrame(mlxFrame) != 0) {
    return;  // sem delay — tenta logo outra vez
  }

  // Converter float -> byte
  for (int i = 0; i < 768; i++) {
    float t = constrain(mlxFrame[i], TEMP_MIN, TEMP_MAX);
    thermalData[i] = (uint8_t)((t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255.0f);
  }

  // Enviar 32 pacotes — cada um enviado 3x para redundancia
  // A 250kbps, cada pacote (25 bytes = 200 bits) demora ~0.8ms
  // 32 pacotes x 3 copias = 96 transmissoes x ~1ms = ~96ms
  ThermalPacket pkt;
  for (uint8_t p = 0; p < 32; p++) {
    pkt.index = p;
    memcpy(pkt.data, &thermalData[p * 24], 24);

    // 3 copias por pacote — aumenta probabilidade de recepcao a distancia
    radioTherm.writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);   // Mais tempo entre envios a 250kbps
    radioTherm.writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);
    radioTherm.writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);
  }
  radioTherm.txStandBy();  // garante que o ultimo pacote saiu

  frameCount++;
  if (frameCount % 10 == 0) {
    Serial.print("[THERM-TX] Frame #");
    Serial.print(frameCount);
    Serial.print(" | Centro: ");
    Serial.print(mlxFrame[400], 1);
    Serial.println(" C");
  }
}
