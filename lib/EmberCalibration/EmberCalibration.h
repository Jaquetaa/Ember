#ifndef EMBER_CALIBRATION_H
#define EMBER_CALIBRATION_H

#include <Arduino.h>

// ============================================================
// EMBER — Sequencia de calibracao dos ESCs HobbyKing 20A
// ============================================================
//
// Disparada por botao no GPIO_BTN (active LOW, INPUT_PULLUP).
//
// Fases (publicadas no ACK do NRF para o Tiago saber o que apitar):
//
//   0 PH_IDLE         — fora de calibracao
//   1 PH_RAMP_DOWN    — desce gradualmente os ESCs ate 1000us
//   2 PH_WARN_BURST   — Tiago apita 25 beeps frenicos (1500ms)
//   3 PH_SILENT_500   — 500ms totalmente silenciosos (sem leds/buzzer)
//   4 PH_COUNTDOWN    — 5 ticks de 1s, beep de 300ms cada (5000ms)
//   5 PH_SILENT_PRE   — 1s silencioso antes do codigo de calibracao
//   6 PH_CAL_RUN      — codigo de calibracao (MAX 5s -> MIN 5s = 10s)
//                       Tiago apita continuamente em freq calibracao
//                       e LEDs gas/chama piscam em sync
//   7 PH_SILENT_POST  — 1s silencioso final, depois reabilita o drone
//
// Durante PH_RAMP_DOWN..PH_SILENT_POST o drone IGNORA qualquer
// comando de controlo recebido (ver loop principal). O ACK continua
// a fluir para que o Tiago receba a fase em todos os pacotes.

class EmberCalibration {
public:
  enum Phase : uint8_t {
    PH_IDLE        = 0,
    PH_RAMP_DOWN   = 1,
    PH_WARN_BURST  = 2,
    PH_SILENT_500  = 3,
    PH_COUNTDOWN   = 4,
    PH_SILENT_PRE  = 5,
    PH_CAL_RUN     = 6,
    PH_SILENT_POST = 7,
  };

  using WriteUsFn = void (*)(int us);
  using OnStartFn = void (*)();   // chamado ao disparar (desarmar drone)
  using OnDoneFn  = void (*)();   // chamado ao reabilitar drone

  void  begin(uint8_t buttonPin,
              WriteUsFn writeUs,
              OnStartFn onStart = nullptr,
              OnDoneFn  onDone  = nullptr);

  // Chamar todos os loops. currentThrottle = ultimo us escrito nos ESCs
  // (usado como ponto de partida do ramp down).
  void  update(int currentThrottle);

  bool  isBusy() const { return _phase != PH_IDLE; }
  Phase getPhase() const { return _phase; }

  // Forca disparo (sem botao).
  void  trigger();

  // ── Duracoes (ms) — DEVEM bater certo com o lado do Tiago. ──
  // Ver CALIBRATION_PROTOCOL.md para a tabela canonica.
  static const uint32_t T_WARN_BURST_MS   = 1500;  // 25 beeps em 1500ms
  static const uint32_t T_SILENT_500_MS   = 500;
  static const uint32_t T_COUNTDOWN_MS    = 5000;  // 5 segundos
  static const uint32_t T_SILENT_PRE_MS   = 1000;
  static const uint32_t T_CAL_MAX_HOLD_MS = 5000;  // 2000us mantido 5s
  static const uint32_t T_CAL_MIN_HOLD_MS = 5000;  // 1000us mantido 5s
  static const uint32_t T_SILENT_POST_MS  = 1000;

private:
  static const int CAL_MIN_US     = 1000;
  static const int CAL_MAX_US     = 2000;
  static const int CAL_RAMP_STEP  = 5;
  static const int CAL_RAMP_MS    = 30;
  static const unsigned long DEBOUNCE_MS = 50;

  enum CalSub : uint8_t { SUB_MAX = 0, SUB_MIN = 1 };

  uint8_t   _btn       = 255;
  WriteUsFn _writeUs   = nullptr;
  OnStartFn _onStart   = nullptr;
  OnDoneFn  _onDone    = nullptr;

  Phase         _phase         = PH_IDLE;
  CalSub        _calSub        = SUB_MAX;
  int           _rampThrottle  = CAL_MIN_US;
  unsigned long _tPhaseMs      = 0;
  unsigned long _tLastStepMs   = 0;
  int           _lastBtnRead   = HIGH;
  int           _stableBtn     = HIGH;
  unsigned long _tBtnChangeMs  = 0;

  void  enterPhase(Phase p);
};

#endif
