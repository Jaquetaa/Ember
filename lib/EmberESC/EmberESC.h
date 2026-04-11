#ifndef EMBER_ESC_H
#define EMBER_ESC_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define THROTTLE_NORMAL 1300
#define THROTTLE_Y_LOW  1200
#define THROTTLE_Y_HIGH 1500
#define THROTTLE_MIN    1000
#define THROTTLE_ARM    1100
#define YAW_MAX         100
#define RAMP_STEP       2
#define RAMP_DELAY      40

class EmberESC {
public:
  bool begin(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);
  void writeAll(int us);
  void writeWithYaw(int base, int yaw);
  void setArmed(bool arm);
  bool isArmed();
  bool isRamping();
  void updateRamp();
  void emergencyStop();

private:
  // Ponteiros alocados no heap dentro de begin() para evitar
  // conflitos de static init order com outros globais (RF24, MLX, etc).
  Servo*  _esc[4] = {nullptr, nullptr, nullptr, nullptr};
  uint8_t _pin[4] = {0, 0, 0, 0};
  bool    _armed = false;
  bool    _rampActive = false;
  int     _currentThrottle = THROTTLE_MIN;
  int     _targetThrottle  = THROTTLE_MIN;
};

#endif
