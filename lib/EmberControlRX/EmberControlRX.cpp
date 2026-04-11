#include "EmberControlRX.h"

const uint8_t EmberControlRX::ADDRESS[6] = "00001";

bool EmberControlRX::begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi) {
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  _radio = new RF24(cePin, csnPin);

  if (!_radio->begin(&spi) || !_radio->isChipConnected()) {
    Serial.println("[CTRL-RX] ERRO: nRF24 nao iniciou!");
    return false;
  }

  _radio->setChannel(76);
  _radio->setDataRate(RF24_250KBPS);
  _radio->setPALevel(RF24_PA_LOW);
  _radio->setAutoAck(true);
  _radio->enableAckPayload();
  _radio->openReadingPipe(1, ADDRESS);
  _radio->startListening();

  uint8_t zero = 0;
  _radio->writeAckPayload(1, &zero, 1);

  _lastRxTime = millis();
  _initialized = true;
  Serial.println("[CTRL-RX] nRF24 OK: canal=76, 250KBPS, ACK");
  return true;
}

bool EmberControlRX::isOK() const { return _initialized; }

bool EmberControlRX::receive(PayloadCtrl &p) {
  if (!_initialized) return false;
  if (_radio->available()) {
    while (_radio->available()) _radio->read(&p, sizeof(p));
    _lastRxTime = millis();
    return true;
  }
  return false;
}

void EmberControlRX::updateAckPayload(uint8_t estado) {
  if (!_initialized) return;
  _radio->writeAckPayload(1, &estado, 1);
}

unsigned long EmberControlRX::lastReceiveTime() {
  if (!_initialized) return millis();  // evita timeout imediato no loop
  return _lastRxTime;
}
