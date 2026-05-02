#include "EmberControlTX.h"

const uint8_t EmberControlTX::ADDRESS[6] = "00001";

EmberControlTX::EmberControlTX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _csnPin(csnPin),
    _lastSensor(0), _consecutiveFails(0), _totalSent(0), _totalOk(0) {}

bool EmberControlTX::begin(SPIClass &spi) {
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  if (!_radio.begin(&spi) || !_radio.isChipConnected()) {
    Serial.println("[CTRL-RF] ERRO: nRF24 nao iniciou!");
    return false;
  }

  _radio.setChannel(76);
  _radio.setDataRate(RF24_250KBPS);
  _radio.setPALevel(RF24_PA_LOW);    // PA_LOW: nao sobrecarrega o rail 3.3V
  _radio.setAutoAck(true);
  _radio.setRetries(5, 15);          // ARD=1500us (minimo para 250KBPS+ACK payload), 15 retries
  _radio.enableAckPayload();
  _radio.openWritingPipe(ADDRESS);
  _radio.stopListening();

  Serial.println("[CTRL-RF] nRF24 OK: canal=76, 250KBPS, PA_LOW, ACK, 15 retries");
  return true;
}

bool EmberControlTX::send(uint8_t armed, int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll) {
  PayloadCtrl p = { armed, throttle, yaw, pitch, roll };
  bool ok = _radio.write(&p, sizeof(p));

  _totalSent++;
  if (ok) {
    _totalOk++;
    _consecutiveFails = 0;
    if (_radio.available()) {
      uint8_t v;
      _radio.read(&v, 1);
      _lastSensor = constrain(v, 0, 3);
    }
  } else {
    if (_consecutiveFails < 65535) _consecutiveFails++;
  }

  return ok;
}

uint8_t  EmberControlTX::getLastSensorState()  { return _lastSensor; }
uint16_t EmberControlTX::getConsecutiveFails()  { return _consecutiveFails; }
uint32_t EmberControlTX::getTotalSent()         { return _totalSent; }
uint32_t EmberControlTX::getTotalOk()           { return _totalOk; }
