#ifndef EMBER_JOYSTICK_H
#define EMBER_JOYSTICK_H

#include <Arduino.h>

class EmberJoystick {
public:
  EmberJoystick(uint8_t pinY, uint8_t pinX);
  void begin();
  int readThrottle();  // retorna microsegundos (1000-1500)
  int readYaw();       // retorna -100 a +100

private:
  uint8_t _pinY, _pinX;
};

// Constantes partilhadas (usadas também no drone)
#define THROTTLE_NORMAL 1300
#define THROTTLE_Y_LOW  1200
#define THROTTLE_Y_HIGH 1500
#define THROTTLE_MIN    1000
#define YAW_MAX         100

#endif
