#ifndef EMBER_BUZZER_H
#define EMBER_BUZZER_H
#include <Arduino.h>

class EmberBuzzer {
public:
  enum CalPhase : uint8_t {
    PH_IDLE=0, PH_RAMP_DOWN=1, PH_WARN_BURST=2, PH_SILENT_500=3,
    PH_COUNTDOWN=4, PH_SILENT_PRE=5, PH_CAL_RUN=6, PH_SILENT_POST=7
  };

  void begin(uint8_t buzzPin, uint8_t ledGasPin, uint8_t ledFirePin,
             uint8_t ledcChannel = 5);
  void update(uint8_t ackByte);
  void setFrequencies(uint16_t fGas, uint16_t fFire, uint16_t fCal);

private:
  uint8_t  _buzz, _ledGas, _ledFire, _ch;
  uint16_t _fGas=700, _fFire=1400, _fCal=1600;

  // Calibracao
  CalPhase _phase       = PH_IDLE;
  uint32_t _tPhaseEnter = 0;
  uint16_t _calHz       = 0;   // cache p/ nao chamar tone() cada loop

  // Alarm state machine — port direto do Tiago-TX antigo
  uint8_t  _lastEstado  = 0xFF;
  bool     _alarmeAtivo = false;
  uint16_t _alarmFreq   = 0;
  uint32_t _beepDuracao = 0;
  uint32_t _beepPausa   = 0;
  bool     _beepEstado  = false;
  uint32_t _ultimoBeep  = 0;

  void _buzzOn(uint16_t hz);
  void _buzzOff();
  void _calTone(uint16_t hz);
  void _ledsBoth(bool on);
  void _ativarAlarme(uint16_t freq, uint32_t dur, uint32_t pausa, uint32_t agora);
  void _desativarAlarme();
  void _atualizarAlarme(uint8_t estado, uint32_t agora);
  void _drivePhase(uint32_t now);
  void _driveAlarms(uint8_t estado, uint32_t now);
};
#endif
