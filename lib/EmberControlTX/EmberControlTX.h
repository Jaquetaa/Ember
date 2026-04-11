#ifndef EMBER_CONTROL_TX_H
#define EMBER_CONTROL_TX_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
};

class EmberControlTX {
public:
  EmberControlTX(uint8_t cePin, uint8_t csnPin);
  bool begin(SPIClass &spi);
  bool send(uint8_t armed, int16_t throttle, int16_t yaw);
  bool hasAck();                    // true se recebeu ACK com payload
  uint8_t getLastSensorState();     // 0=OK, 1=CO, 2=Fogo, 3=Fogo+CO

private:
  RF24 _radio;
  uint8_t _csnPin;
  uint8_t _lastSensor;
  static const uint8_t ADDRESS[6];
};

#endif
