#ifndef EMBER_SENSORS_H
#define EMBER_SENSORS_H

#include <Arduino.h>

#define FLAME_THRESHOLD 500
#define CO_PPM_LIMITE   160.0f
#define CONFIRMACOES    3
#define PREHEAT_MS      180000UL

class EmberSensors {
public:
  void begin(uint8_t flamePin, uint8_t mq7Pin);
  void update();
  uint8_t getEstado();
  int getFlameVal()  { return _flameVal; }
  int getCoVal()     { return _coVal; }
  float getCoPpm()   { return _coPpm; }
  bool isAquecido()  { return _mq7Aquecido; }
  unsigned long getPreheatRemaining() {
    if (_mq7Aquecido) return 0;
    unsigned long elapsed = millis() - _tempoInicio;
    return (elapsed >= PREHEAT_MS) ? 0 : (PREHEAT_MS - elapsed);
  }

private:
  uint8_t _flamePin, _mq7Pin;
  bool _mq7Aquecido = false;
  unsigned long _tempoInicio = 0;
  int _contadorChama = 0;
  uint8_t _estado = 0;
  int _flameVal = 0;
  int _coVal = 0;
  float _coPpm = 0.0f;
};

#endif
