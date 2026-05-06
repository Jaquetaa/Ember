#ifndef EMBER_BUZZER_H
#define EMBER_BUZZER_H
#include <Arduino.h>

class EmberBuzzer {
public:
  // ackByte: bits 0-1 = sensor estado (0-3) | bits 2-4 = CalPhase (0-7)
  // CalPhase != 0 tem prioridade sobre o sensor
  void begin(uint8_t buzzPin, uint8_t ledGasPin, uint8_t ledFirePin,
             uint8_t ledcChannel = 5);
  void update(uint8_t ackByte);

private:
  uint8_t  _buzz, _ledGas, _ledFire, _ch;
  uint16_t _currentFreq  = 0xFFFF;

  // Maquina de beep para calibracao (non-blocking)
  uint8_t  _lastCalPhase = 0;
  bool     _calBeepOn    = false;
  uint32_t _calLastBeep  = 0;

  void _buzzOn(uint16_t hz);
  void _buzzOff();
  void _ledsSet(bool gasOn, bool fireOn);
  void _calBeep(uint32_t durOn, uint32_t durOff, uint16_t freq, uint32_t now);
  void _driveCalibration(uint8_t calPhase, uint32_t now);
  void _driveSensor(uint8_t estado);
};
#endif
