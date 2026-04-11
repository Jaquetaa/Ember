#include "EmberControlTX.h"

const uint8_t EmberControlTX::ADDRESS[6] = "00001";

EmberControlTX::EmberControlTX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _csnPin(csnPin), _lastSensor(0) {}

bool EmberControlTX::begin(SPIClass &spi) {
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  if (!_radio.begin(&spi) || !_radio.isChipConnected()) {
    Serial.println("[CTRL-TX] ERRO: nRF24 nao iniciou!");
    return false;
  }

  _radio.setChannel(76);
  _radio.setDataRate(RF24_250KBPS);
  _radio.setPALevel(RF24_PA_LOW);
  _radio.setAutoAck(true);
  _radio.setRetries(5, 15);
  _radio.enableAckPayload();
  _radio.openWritingPipe(ADDRESS);
  _radio.stopListening();

  Serial.println("[CTRL-TX] nRF24 OK: canal=76, 250KBPS, ACK");
  return true;
}

bool EmberControlTX::send(uint8_t armed, int16_t throttle, int16_t yaw) {
  PayloadCtrl p = { armed, throttle, yaw };
  bool ok = _radio.write(&p, sizeof(p));

  if (ok && _radio.available()) {
    uint8_t v;
    _radio.read(&v, 1);
    _lastSensor = constrain(v, 0, 3);
  }

  return ok;
}

bool EmberControlTX::hasAck() {
  // A ultima chamada send() ja processou o ACK
  return true;
}

uint8_t EmberControlTX::getLastSensorState() {
  return _lastSensor;
}
