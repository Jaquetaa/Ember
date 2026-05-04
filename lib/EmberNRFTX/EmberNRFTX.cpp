#include "EmberNRFTX.h"

const uint8_t EmberNRFTX::ADDRESS[6] = "00001";

EmberNRFTX::EmberNRFTX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _csnPin(csnPin),
    _lastAckByte(0), _lastSensor(0), _consecutiveFails(0), _totalSent(0), _totalOk(0) {}

bool EmberNRFTX::begin(SPIClass &spi) {
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  if (!_radio.begin(&spi) || !_radio.isChipConnected()) {
    Serial.println("[NRF-TX] ERRO: nRF24 nao iniciou!");
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

  Serial.println("[NRF-TX] nRF24 OK: canal=76, 250KBPS, PA_LOW, ACK, 15 retries");
  return true;
}

bool EmberNRFTX::send(uint8_t armed, int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll) {
  PayloadCtrl p = { armed, throttle, yaw, pitch, roll };
  bool ok = _radio.write(&p, sizeof(p));

  _totalSent++;
  if (ok) {
    _totalOk++;
    _consecutiveFails = 0;
    if (_radio.available()) {
      uint8_t v;
      _radio.read(&v, 1);
      _lastAckByte = v;
      _lastSensor  = v & 0x03;
    }
  } else {
    if (_consecutiveFails < 65535) _consecutiveFails++;
  }

  return ok;
}

uint8_t  EmberNRFTX::getLastSensorState()  { return _lastSensor; }
uint8_t  EmberNRFTX::getLastAckByte()      { return _lastAckByte; }
uint16_t EmberNRFTX::getConsecutiveFails()  { return _consecutiveFails; }
uint32_t EmberNRFTX::getTotalSent()         { return _totalSent; }
uint32_t EmberNRFTX::getTotalOk()           { return _totalOk; }
