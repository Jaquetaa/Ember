#include "EmberCalibration.h"

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

void EmberCalibration::enterPhase(Phase p) {
  _phase    = p;
  _tPhaseMs = millis();
  Serial.print("[CAL] -> fase ");
  Serial.println((int)p);
}

void EmberCalibration::trigger() {
  if (_phase != PH_IDLE) return;
  Serial.println("[CAL] DISPARO — inicio sequencia.");
  if (_onStart) _onStart();
  _tLastStepMs = millis();
  enterPhase(PH_RAMP_DOWN);
}

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
      _rampThrottle = constrain(currentThrottle, CAL_MIN_US, CAL_MAX_US);
      trigger();
    }
  }

  if (_phase == PH_IDLE || !_writeUs) return;

  switch (_phase) {

    case PH_RAMP_DOWN:
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
