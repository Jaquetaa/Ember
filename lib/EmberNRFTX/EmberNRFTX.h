#ifndef EMBER_NRF_TX_H
#define EMBER_NRF_TX_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

struct __attribute__((packed)) PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

class EmberNRFTX {
public:
  EmberNRFTX(uint8_t cePin, uint8_t csnPin);
  bool begin(SPIClass &spi);
  bool send(uint8_t armed, int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll);
  uint8_t getLastSensorState();  // bits [1:0] do ACK: 0=OK, 1=CO, 2=Fogo, 3=Fogo+CO
  uint8_t getLastAckByte();      // byte ACK completo (estado + calPhase nos bits [4:2])
  uint16_t getConsecutiveFails();
  uint32_t getTotalSent();
  uint32_t getTotalOk();

private:
  RF24     _radio;
  uint8_t  _csnPin;
  uint8_t  _lastAckByte;
  uint8_t  _lastSensor;
  uint16_t _consecutiveFails;
  uint32_t _totalSent;
  uint32_t _totalOk;
  static const uint8_t ADDRESS[6];
};

#endif
