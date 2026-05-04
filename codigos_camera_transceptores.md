## EmberCAMNRF (camara termica MLX90640 + NRF24 TX)

### EmberCAMNRF.h

```cpp
#ifndef EMBER_CAM_NRF_H
#define EMBER_CAM_NRF_H

#include <Arduino.h>

// Header minimo - os includes pesados (MLX90640, RF24, Wire)
// ficam so no .cpp para nao poluir o scope do .ino

class SPIClass;

class EmberCAMNRF {
public:
  bool begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi);
  void update();

private:
  void *_radio = nullptr;      // RF24* cast internamente no .cpp
  void *_mlx = nullptr;        // Adafruit_MLX90640* cast internamente no .cpp
  bool _nrfOK = false;
  uint32_t _frameCount = 0;
  float   *_mlxFrame = nullptr;
  uint8_t *_thermalData = nullptr;

  // Maquina de estados nao-bloqueante:
  //  0 = IDLE (espera intervalo entre frames)
  //  1 = TX_BURST (a enviar pacotes em batches)
  uint8_t  _state = 0;
  uint8_t  _txIndex = 0;        // proximo pacote (0..31) a enviar no burst
  uint32_t _lastFrameMs = 0;    // millis() do inicio do ultimo frame
};

#endif
```

### EmberCAMNRF.cpp

```cpp
#include "EmberCAMNRF.h"
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

bool EmberCAMNRF::begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi) {
  Serial.print("[THERM-TX] begin() CE=");   Serial.print(cePin);
  Serial.print(" CSN=");                    Serial.print(csnPin);
  Serial.print(" SDA=");                    Serial.print(sdaPin);
  Serial.print(" SCL=");                    Serial.println(sclPin);
  Serial.flush();

  Serial.println("[THERM-TX] malloc 768*float + 768*uint8...");
  Serial.flush();
  _mlxFrame = (float*)malloc(768 * sizeof(float));
  _thermalData = (uint8_t*)malloc(768);
  if (!_mlxFrame || !_thermalData) {
    Serial.println("[THERM-TX] ERRO: sem memoria!");
    return false;
  }
  memset(_mlxFrame, 0, 768 * sizeof(float));
  memset(_thermalData, 0, 768);
  Serial.println("[THERM-TX] malloc OK.");
  Serial.flush();

  // I2C
  Serial.print("[THERM-TX] Wire.begin(SDA="); Serial.print(sdaPin);
  Serial.print(", SCL=");                     Serial.print(sclPin);
  Serial.println(") @ 1MHz...");
  Serial.flush();
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(1000000);

  // I2C scan - descobre que enderecos respondem no bus
  Serial.println("[THERM-TX] I2C scan (0x03..0x77)...");
  Serial.flush();
  uint8_t found = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  [I2C] device em 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);
      Serial.flush();
      found++;
    }
  }
  Serial.print("[THERM-TX] I2C scan: ");
  Serial.print(found);
  Serial.println(" dispositivo(s).");
  if (found == 0) {
    Serial.println("[THERM-TX] AVISO: nenhum dispositivo I2C respondeu.");
    Serial.println("           Verifica: VCC 3.3V no MLX, GND comum,");
    Serial.println("           pull-ups 4.7k em SDA e SCL, ligacoes SDA/SCL.");
  }
  Serial.flush();

  Serial.print("[THERM-TX] mlx->begin(addr=0x");
  Serial.print(MLX90640_I2CADDR_DEFAULT, HEX);
  Serial.println(")...");
  Serial.flush();
  Adafruit_MLX90640 *mlx = new Adafruit_MLX90640();
  if (!mlx->begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("[THERM-TX] ERRO: MLX90640 nao encontrado!");
    Serial.println("           (mas continua para tentar inicializar NRF)");
    delete mlx;
    mlx = nullptr;
  } else {
    mlx->setMode(MLX90640_CHESS);
    mlx->setResolution(MLX90640_ADC_18BIT);
    mlx->setRefreshRate(MLX90640_8_HZ);
    _mlx = (void*)mlx;
    Serial.println("[THERM-TX] MLX90640 OK (8Hz, 1MHz I2C).");
    Serial.flush();
  }

  // NRF
  Serial.println("[THERM-TX] pinMode CSN OUTPUT HIGH...");
  Serial.flush();
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  Serial.println("[THERM-TX] new RF24(CE,CSN, 1MHz)...");
  Serial.flush();
  RF24 *radio = new RF24(cePin, csnPin, 1000000);  // 1 MHz para fios longos
  if (!radio) {
    Serial.println("[THERM-TX] ERRO: new RF24 devolveu nullptr (OOM)");
    return false;
  }
  Serial.println("[THERM-TX] RF24 alocado.");
  Serial.flush();

  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-TX] tentativa "); Serial.print(t); Serial.print("/3 begin()... ");
    Serial.flush();
    bool beginOK = radio->begin(&spi);
    Serial.print(beginOK ? "begin=OK " : "begin=FAIL ");
    bool chipOK = radio->isChipConnected();
    Serial.print(chipOK ? "chip=OK" : "chip=FAIL");
    Serial.println();
    Serial.flush();
    if (beginOK && chipOK) {
      _nrfOK = true;
      break;
    }
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
    Serial.println("[THERM-TX] ---- printDetails (post-config) ----");
    Serial.flush();
    radio->printDetails();
    Serial.println("[THERM-TX] ------------------------------------");
    Serial.println("[THERM-TX] nRF24 OK: canal=2, 250KBPS, PA_MAX, CRC OFF");
    Serial.flush();
  } else {
    Serial.println("[THERM-TX] *** NRF FALHOU ***");
    Serial.println("[THERM-TX] ---- printDetails (raw) ----");
    Serial.flush();
    radio->printDetails();
    Serial.println("[THERM-TX] -----------------------------");
    Serial.flush();
    delete radio;
  }

  // Se o MLX falhou, devolvemos false para o caller saber
  return _nrfOK && (mlx != nullptr);
}

// Maquina de estados nao-bloqueante. Cada chamada faz uma fatia
// pequena de trabalho e devolve, para nao tapar o loop de controlo.
//
// IDLE       -> respeita intervalo de 125ms (~8Hz, alinhado ao MLX).
//               Ao expirar, faz a leitura I2C sincrona (~30-40ms - limitacao
//               da lib Adafruit) + normalizacao, e passa a TX_BURST.
// TX_BURST   -> envia PKTS_PER_TICK pacotes via writeFast() e devolve.
//               No fim do burst chama txStandBy() e volta a IDLE.
//
// Sem delayMicroseconds() entre pacotes: com autoAck=false e CRC off
// o NRF aceita escritas back-to-back ate encher o FIFO (3 slots).
void EmberCAMNRF::update() {
  if (!_nrfOK || !_radio || !_mlx) return;

  Adafruit_MLX90640 *mlx = (Adafruit_MLX90640*)_mlx;
  RF24 *radio = (RF24*)_radio;

  const uint32_t FRAME_INTERVAL_MS = 125;   // 8Hz, alinhado ao MLX
  const uint8_t  PKTS_PER_TICK     = 4;     // 4 x 25B por chamada

  if (_state == 0) {
    // IDLE: aguarda intervalo
    if (millis() - _lastFrameMs < FRAME_INTERVAL_MS) return;

    // Le frame (sincrono - limitacao I2C). Se falhar tenta de novo no
    // proximo tick sem avancar de estado.
    if (mlx->getFrame(_mlxFrame) != 0) return;

    for (int i = 0; i < 768; i++) {
      float t = constrain(_mlxFrame[i], TEMP_MIN, TEMP_MAX);
      _thermalData[i] = (uint8_t)((t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255.0f);
    }

    _txIndex = 0;
    _lastFrameMs = millis();
    _state = 1;
    return;
  }

  // _state == 1: TX_BURST
  ThermalPacket pkt;
  uint8_t end = _txIndex + PKTS_PER_TICK;
  if (end > 32) end = 32;
  for (uint8_t p = _txIndex; p < end; p++) {
    pkt.index = p;
    memcpy(pkt.data, &_thermalData[p * 24], 24);
    radio->writeFast(&pkt, sizeof(pkt));
  }
  _txIndex = end;

  if (_txIndex >= 32) {
    radio->txStandBy();
    _state = 0;
    _frameCount++;
    if (_frameCount % 10 == 0) {
      Serial.print("[THERM-TX] Frame #"); Serial.print(_frameCount);
      Serial.print(" | Centro: "); Serial.print(_mlxFrame[400], 1);
      Serial.println(" C");
    }
  }
}
```

---

## EmberDroneNRF (transceptor de controlo RX no drone)

### EmberDroneNRF.h

```cpp
#ifndef EMBER_DRONE_NRF_H
#define EMBER_DRONE_NRF_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

class EmberDroneNRF {
public:
  bool begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi);
  bool isOK() const;
  bool receive(PayloadCtrl &p);
  void updateAckPayload(uint8_t estado);
  unsigned long lastReceiveTime();

private:
  RF24 *_radio = nullptr;
  unsigned long _lastRxTime = 0;
  bool _initialized = false;
  static const uint8_t ADDRESS[6];
};

#endif
```

### EmberDroneNRF.cpp

```cpp
#include "EmberDroneNRF.h"

const uint8_t EmberDroneNRF::ADDRESS[6] = "00001";

bool EmberDroneNRF::begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi) {
  Serial.print("[CTRL-RX] begin() CE=");
  Serial.print(cePin);
  Serial.print(" CSN=");
  Serial.println(csnPin);
  Serial.flush();

  Serial.println("[CTRL-RX] pinMode CSN OUTPUT HIGH...");
  Serial.flush();
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  Serial.println("[CTRL-RX] new RF24(CE,CSN, 1MHz)...");
  Serial.flush();
  _radio = new RF24(cePin, csnPin, 1000000);  // 1 MHz para fios longos
  if (!_radio) {
    Serial.println("[CTRL-RX] ERRO: new RF24 devolveu nullptr (OOM)");
    return false;
  }
  Serial.println("[CTRL-RX] RF24 alocado.");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->begin(&spi)...");
  Serial.flush();
  bool beginOK = _radio->begin(&spi);
  Serial.print("[CTRL-RX] begin() returned: ");
  Serial.println(beginOK ? "true" : "false");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->isChipConnected()...");
  Serial.flush();
  bool chipOK = _radio->isChipConnected();
  Serial.print("[CTRL-RX] isChipConnected: ");
  Serial.println(chipOK ? "true" : "false");
  Serial.flush();

  if (!beginOK || !chipOK) {
    Serial.println("[CTRL-RX] ERRO: nRF24 nao iniciou!");
    Serial.println("[CTRL-RX] Verifica: alimentacao 3.3V (com cap 10uF),");
    Serial.println("           SPI (SCK/MISO/MOSI), CE, CSN, GND comum.");
    // Dump dos registos para diagnostico (mostra address de RX_PIPE etc.)
    Serial.println("[CTRL-RX] ---- printDetails (raw) ----");
    Serial.flush();
    _radio->printDetails();
    Serial.println("[CTRL-RX] -----------------------------");
    Serial.flush();
    return false;
  }

  Serial.println("[CTRL-RX] config: ch=76, 250KBPS, PA_LOW, autoAck, ackPayload...");
  Serial.flush();
  _radio->setChannel(76);
  _radio->setDataRate(RF24_250KBPS);
  _radio->setPALevel(RF24_PA_MAX);
  _radio->setAutoAck(true);
  _radio->setRetries(1, 3);   // 500us x 3 = 1.5ms worst case (igual ao TX)
  _radio->enableAckPayload();
  _radio->openReadingPipe(1, ADDRESS);
  _radio->startListening();

  uint8_t zero = 0;
  _radio->writeAckPayload(1, &zero, 1);

  // Diagnostico final: confirma o que ficou nos registos
  Serial.println("[CTRL-RX] ---- printDetails (post-config) ----");
  Serial.flush();
  _radio->printDetails();
  Serial.println("[CTRL-RX] ------------------------------------");
  Serial.flush();

  _lastRxTime = millis();
  _initialized = true;
  Serial.println("[CTRL-RX] nRF24 OK: canal=76, 250KBPS, ACK");
  Serial.flush();
  return true;
}

bool EmberDroneNRF::isOK() const { return _initialized; }

bool EmberDroneNRF::receive(PayloadCtrl &p) {
  if (!_initialized) return false;
  if (_radio->available()) {
    while (_radio->available()) _radio->read(&p, sizeof(p));
    _lastRxTime = millis();
    return true;
  }
  return false;
}

void EmberDroneNRF::updateAckPayload(uint8_t estado) {
  if (!_initialized) return;
  _radio->writeAckPayload(1, &estado, 1);
}

unsigned long EmberDroneNRF::lastReceiveTime() {
  if (!_initialized) return millis();  // evita timeout imediato no loop
  return _lastRxTime;
}
```
