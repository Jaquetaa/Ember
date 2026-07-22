// ============================================================
// EmberCalibration.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface da classe EmberCalibration, que executa
// a sequência de calibração de 7 fases dos ESC (Electronic Speed
// Controller, controlador eletrónico de velocidade) HobbyKing 20A.
// A sequência é disparada por um botão físico com debounce
// (técnica anti-tremor para filtrar pressões acidentais de botão)
// e comunica o progresso ao controlador do Tiago através do byte
// ACK (Acknowledgement, confirmação de receção) do NRF24L01+.
// Relaciona-se com Goncalo-Ember.ino, que chama update() em
// cada loop e bloqueia comandos de voo enquanto isBusy() for
// verdadeiro, e com CALIBRATION_PROTOCOL.md, que documenta as
// durações canónicas sincronizadas com o lado do Tiago.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Regista o pino do botão, os três callbacks e configura
//    INPUT_PULLUP. Lê o estado inicial do botão para evitar
//    falsa deteção de pressão ao arrancar.
// 2. update() em cada loop():
//    Passo 1: faz debounce (50 ms (milissegundos)) do botão.
//             Na transição HIGH->LOW em PH_IDLE, regista o
//             throttle atual como ponto de partida do ramp
//             e chama trigger().
//    Passo 2: executa a fase atual da máquina de estados via
//             switch/case, usando millis() (função que devolve
//             os ms desde que o controlador ligou) para
//             temporizar sem bloquear o loop principal.
// 3. enterPhase() é auxiliar interno:
//    Atualiza _phase e regista _tPhaseMs para as temporizações
//    relativas dentro de cada fase.
//
// FASES (byte publicado no ACK para o controlador do Tiago)
// -----------------------------------------------------------
//   0 PH_IDLE:        fora de calibração, drone operacional
//   1 PH_RAMP_DOWN:   desce os ESC de currentThrottle até 1000 us
//   2 PH_WARN_BURST:  Tiago apita 25 beeps frénicos (1500 ms)
//   3 PH_SILENT_500:  500 ms totalmente silenciosos
//   4 PH_COUNTDOWN:   5 tiques de 1 s, beep de 300 ms cada (5000 ms)
//   5 PH_SILENT_PRE:  1 s silencioso antes do código de calibração
//   6 PH_CAL_RUN:     código de calibração (MAX 5 s + MIN 5 s = 10 s)
//                     Tiago apita continuamente e LEDs piscam em sync
//   7 PH_SILENT_POST: 1 s silencioso final, depois reabilita o drone
//
// Durante PH_RAMP_DOWN a PH_SILENT_POST o drone ignora qualquer
// comando de controlo recebido. O ACK continua a fluir para que
// o Tiago receba a fase em todos os pacotes.
// ============================================================

#ifndef EMBER_CALIBRATION_H
#define EMBER_CALIBRATION_H

#include <Arduino.h>

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

  // -----------------------------------
  // Durações de cada fase em ms
  // -----------------------------------

  // DEVEM bater certo com o lado do Tiago: ver CALIBRATION_PROTOCOL.md.
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
