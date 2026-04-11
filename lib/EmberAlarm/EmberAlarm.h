#ifndef EMBER_ALARM_H
#define EMBER_ALARM_H

#include <Arduino.h>

class EmberAlarm {
public:
  EmberAlarm(uint8_t buzzerPin, uint8_t ledPin);
  void begin();
  void update();                     // chamar no loop
  void aplicarEstado(uint8_t estado); // 0=OK, 1=CO, 2=Fogo, 3=Fogo+CO

private:
  uint8_t _buzzerPin, _ledPin;
  bool _ativo;
  int  _freq;
  unsigned long _duracao, _pausa;
  bool _beepEstado;
  unsigned long _ultimoBeep;

  void ativar(int freq, unsigned long dur, unsigned long pausa);
  void desativar();
};

#endif
