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
