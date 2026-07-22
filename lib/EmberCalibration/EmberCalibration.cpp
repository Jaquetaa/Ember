// ============================================================
// EmberCalibration.cpp, Implementação
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementa a classe EmberCalibration, que executa a sequência
// de calibração de 7 fases dos ESC (Electronic Speed Controller,
// controlador eletrónico de velocidade) HobbyKing 20A.
// A sequência é disparada por um botão físico com debounce
// (técnica anti-tremor para filtrar pressões acidentais de botão)
// e comunica o progresso ao controlador do Tiago através do byte
// ACK (Acknowledgement, confirmação de receção) do NRF24L01+.
// Durante toda a sequência, Goncalo-Ember.ino ignora comandos
// de voo enquanto isBusy() devolver verdadeiro.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Regista o pino, os três callbacks (writeUs, onStart, onDone)
//    e configura INPUT_PULLUP. Lê o estado inicial do botão para
//    evitar falsa deteção de pressão no arranque.
// 2. update() em cada loop():
//    Passo 1: faz debounce (50 ms (milissegundos)) do botão.
//             Na transição HIGH->LOW em PH_IDLE, regista o
//             throttle atual como ponto de partida do ramp e
//             chama trigger().
//    Passo 2: executa a fase atual da máquina de estados via
//             switch/case, usando millis() (função que devolve
//             os ms desde que o controlador ligou) para
//             temporizar sem bloquear o loop principal.
// 3. enterPhase() é auxiliar interno:
//    Atualiza _phase e regista _tPhaseMs para as temporizações
//    relativas dentro de cada fase.
//
// SEQUÊNCIA DE CALIBRAÇÃO (ver CALIBRATION_PROTOCOL.md)
// -----------------------------------------------------------
// PH_RAMP_DOWN (1):  desce os ESC de currentThrottle até 1000 us
//                    em passos de 5 us a cada 30 ms.
// PH_WARN_BURST (2): mantém 1000 us por 1500 ms.
//                    O controlador do Tiago apita 25 vezes.
// PH_SILENT_500 (3): mantém 1000 us por 500 ms. Silêncio total.
// PH_COUNTDOWN (4):  mantém 1000 us por 5000 ms.
//                    O controlador do Tiago faz contagem de 5 s.
// PH_SILENT_PRE (5): mantém 1000 us por 1000 ms.
//                    Silêncio antes do código de calibração.
// PH_CAL_RUN (6):    envia 2000 us por 5000 ms (MAX guardado),
//                    depois 1000 us por 5000 ms (MIN guardado).
// PH_SILENT_POST (7): mantém 1000 us por 1000 ms, depois chama
//                     onDone() e volta a PH_IDLE.
// ============================================================

#include "EmberCalibration.h"

// -----------------------------------
// Inicialização
// -----------------------------------

void EmberCalibration::begin(uint8_t buttonPin,
                             WriteUsFn writeUs,
                             OnStartFn onStart,
                             OnDoneFn  onDone) {
  _btn      = buttonPin;
  _writeUs  = writeUs;
  _onStart  = onStart;
  _onDone   = onDone;
  pinMode(_btn, INPUT_PULLUP);
  _lastBtnRead  = digitalRead(_btn);
  _stableBtn    = _lastBtnRead;
  _tBtnChangeMs = millis();
  Serial.print("[CAL] Botao calibracao no GPIO");
  Serial.println(_btn);
}

// -----------------------------------
// Transição de fase
// -----------------------------------

void EmberCalibration::enterPhase(Phase p) {
  _phase    = p;
  _tPhaseMs = millis(); // regista o instante de entrada na fase para temporizações relativas
  Serial.print("[CAL] -> fase ");
  Serial.println((int)p);
}

// -----------------------------------
// Disparo manual da sequência
// -----------------------------------

void EmberCalibration::trigger() {
  if (_phase != PH_IDLE) return;
  Serial.println("[CAL] DISPARO — inicio sequencia.");
  if (_onStart) _onStart();
  _tLastStepMs = millis();
  enterPhase(PH_RAMP_DOWN);
}

// -----------------------------------
// Ciclo de atualização (chamado em cada loop)
// -----------------------------------

void EmberCalibration::update(int currentThrottle) {
  unsigned long now = millis();

  // ── Debounce do botao (so dispara em IDLE) ──────────────
  int r = digitalRead(_btn);
  if (r != _lastBtnRead) {
    _lastBtnRead  = r;
    _tBtnChangeMs = now;
  } else if (r != _stableBtn && (now - _tBtnChangeMs) >= DEBOUNCE_MS) {
    int prev   = _stableBtn;
    _stableBtn = r;
    if (prev == HIGH && _stableBtn == LOW && _phase == PH_IDLE) {
      _rampThrottle = constrain(currentThrottle, CAL_MIN_US, CAL_MAX_US); // ponto de partida do ramp down
      trigger();
    }
  }

  if (_phase == PH_IDLE || !_writeUs) return;

  switch (_phase) {

    case PH_RAMP_DOWN:
      // Desce o throttle gradualmente até 1000 us antes de iniciar a calibração,
      // para evitar que os ESC recebam um corte abrupto de alimentação de sinal.
      if (now - _tLastStepMs >= (unsigned long)CAL_RAMP_MS) {
        _tLastStepMs = now;
        if (_rampThrottle > CAL_MIN_US + CAL_RAMP_STEP) {
          _rampThrottle -= CAL_RAMP_STEP;
          _writeUs(_rampThrottle);
        } else {
          _rampThrottle = CAL_MIN_US;
          _writeUs(CAL_MIN_US);
          Serial.println("[CAL] Ramp down concluido. Drone bloqueado a comandos.");
          enterPhase(PH_WARN_BURST);
        }
      }
      break;

    case PH_WARN_BURST:
      // Drone nao apita — sinaliza fase ao Tiago via ACK.
      // Ver CALIBRATION_PROTOCOL.md: 25 beeps frenicos em ~1500ms.
      _writeUs(CAL_MIN_US);
      if (now - _tPhaseMs >= T_WARN_BURST_MS) enterPhase(PH_SILENT_500);
      break;

    case PH_SILENT_500:
      _writeUs(CAL_MIN_US);
      if (now - _tPhaseMs >= T_SILENT_500_MS) enterPhase(PH_COUNTDOWN);
      break;

    case PH_COUNTDOWN:
      _writeUs(CAL_MIN_US);
      if (now - _tPhaseMs >= T_COUNTDOWN_MS) enterPhase(PH_SILENT_PRE);
      break;

    case PH_SILENT_PRE:
      _writeUs(CAL_MIN_US);
      if (now - _tPhaseMs >= T_SILENT_PRE_MS) {
        // Inicia o codigo de calibracao: MAX primeiro
        Serial.println("[CAL] *** Desliga e liga a bateria do ESC. A enviar MAX (2000us). ***");
        _writeUs(CAL_MAX_US);
        _calSub = SUB_MAX;
        enterPhase(PH_CAL_RUN);
      }
      break;

    case PH_CAL_RUN:
      // SUB_MAX: mantém 2000 us por T_CAL_MAX_HOLD_MS para o ESC registar o máximo.
      // SUB_MIN: mantém 1000 us por T_CAL_MIN_HOLD_MS para o ESC registar o mínimo.
      if (_calSub == SUB_MAX) {
        _writeUs(CAL_MAX_US);
        if (now - _tPhaseMs >= T_CAL_MAX_HOLD_MS) {
          Serial.println("[CAL] MAX guardado — a enviar MIN (1000us)...");
          _writeUs(CAL_MIN_US);
          _calSub   = SUB_MIN;
          _tPhaseMs = now;          // reseta timer da sub-fase
        }
      } else {  // SUB_MIN
        _writeUs(CAL_MIN_US);
        if (now - _tPhaseMs >= T_CAL_MIN_HOLD_MS) {
          Serial.println("[CAL] MIN guardado — calibracao concluida.");
          enterPhase(PH_SILENT_POST);
        }
      }
      break;

    case PH_SILENT_POST:
      _writeUs(CAL_MIN_US);
      if (now - _tPhaseMs >= T_SILENT_POST_MS) {
        Serial.println("[CAL] === Drone REABILITADO. ===");
        if (_onDone) _onDone();
        _phase = PH_IDLE;
      }
      break;

    default:
      _phase = PH_IDLE;
      break;
  }
}
