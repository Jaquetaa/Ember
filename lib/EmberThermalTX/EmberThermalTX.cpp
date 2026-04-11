#include "EmberThermalTX.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <RF24.h>

#define TEMP_MIN  20.0f
#define TEMP_MAX  60.0f

static const uint8_t THERM_ADDR[6] = "THERM";

struct ThermalPacket {
  uint8_t index;
  uint8_t data[24];
};

bool EmberThermalTX::begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi) {
  _mlxFrame = (float*)malloc(768 * sizeof(float));
  _thermalData = (uint8_t*)malloc(768);
  if (!_mlxFrame || !_thermalData) {
    Serial.println("[THERM-TX] ERRO: sem memoria!");
    return false;
  }
  memset(_mlxFrame, 0, 768 * sizeof(float));
  memset(_thermalData, 0, 768);

  // I2C
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(1000000);

  Adafruit_MLX90640 *mlx = new Adafruit_MLX90640();
  if (!mlx->begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("[THERM-TX] ERRO: MLX90640 nao encontrado!");
    delete mlx;
    return false;
  }
  mlx->setMode(MLX90640_CHESS);
  mlx->setResolution(MLX90640_ADC_18BIT);
  mlx->setRefreshRate(MLX90640_8_HZ);
  _mlx = (void*)mlx;
  Serial.println("[THERM-TX] MLX90640 OK (8Hz, 1MHz I2C).");

  // NRF
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  RF24 *radio = new RF24(cePin, csnPin);
  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-TX] NRF tentativa "); Serial.print(t); Serial.print("/3 ... ");
    if (radio->begin(&spi) && radio->isChipConnected()) {
      Serial.println("OK!");
      _nrfOK = true;
      break;
    }
    Serial.println("FALHOU");
    delay(100);
  }

  if (_nrfOK) {
    radio->setChannel(2);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_MAX);
    radio->setAutoAck(false);
    radio->setPayloadSize(25);
    radio->setCRCLength(RF24_CRC_DISABLED);
    radio->openWritingPipe(THERM_ADDR);
    radio->stopListening();
    _radio = (void*)radio;
    Serial.println("[THERM-TX] nRF24 OK: canal=2, 250KBPS, PA_MAX, CRC OFF");
  } else {
    Serial.println("[THERM-TX] *** NRF FALHOU ***");
    delete radio;
  }

  return _nrfOK;
}

void EmberThermalTX::update() {
  if (!_nrfOK || !_radio || !_mlx) return;

  Adafruit_MLX90640 *mlx = (Adafruit_MLX90640*)_mlx;
  RF24 *radio = (RF24*)_radio;

  if (mlx->getFrame(_mlxFrame) != 0) return;

  for (int i = 0; i < 768; i++) {
    float t = constrain(_mlxFrame[i], TEMP_MIN, TEMP_MAX);
    _thermalData[i] = (uint8_t)((t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255.0f);
  }

  ThermalPacket pkt;
  for (uint8_t p = 0; p < 32; p++) {
    pkt.index = p;
    memcpy(pkt.data, &_thermalData[p * 24], 24);
    radio->writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);
    radio->writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);
    radio->writeFast(&pkt, sizeof(pkt));
    delayMicroseconds(600);
  }
  radio->txStandBy();

  _frameCount++;
  if (_frameCount % 10 == 0) {
    Serial.print("[THERM-TX] Frame #"); Serial.print(_frameCount);
    Serial.print(" | Centro: "); Serial.print(_mlxFrame[400], 1);
    Serial.println(" C");
  }
}
