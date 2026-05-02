#include "EmberJoystick.h"

EmberJoystick::EmberJoystick(uint8_t pinY, uint8_t pinX)
  : _pinY(pinY), _pinX(pinX) {}

void EmberJoystick::begin() {
  // ADC pins — no pinMode needed on ESP32
}

int EmberJoystick::readThrottle() {
  int y = analogRead(_pinY);
  if      (y <= 1000) return THROTTLE_Y_LOW;
  else if (y >= 3500) return THROTTLE_Y_HIGH;
  else                return THROTTLE_NORMAL;
}

int EmberJoystick::readYaw() {
  int x = analogRead(_pinX);
  if      (x <= 1000) return -YAW_MAX;
  else if (x >= 3500) return  YAW_MAX;
  else                return 0;
}

int EmberJoystick::readPitch() {
  int y = analogRead(_pinY);
  if      (y <= 1000) return -YAW_MAX;
  else if (y >= 3500) return  YAW_MAX;
  else                return 0;
}

int EmberJoystick::readRoll() {
  int x = analogRead(_pinX);
  if      (x <= 1000) return -YAW_MAX;
  else if (x >= 3500) return  YAW_MAX;
  else                return 0;
}
