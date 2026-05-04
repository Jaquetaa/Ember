#include "EmberBuzzer.h"

void EmberBuzzer::begin(uint8_t buzzPin, uint8_t ledGas, uint8_t ledFire,
                        uint8_t ch) {
  _buzz=buzzPin; _ledGas=ledGas; _ledFire=ledFire; _ch=ch;
  pinMode(_ledGas,  OUTPUT); digitalWrite(_ledGas,  LOW);
  pinMode(_ledFire, OUTPUT); digitalWrite(_ledFire, LOW);
  ledcAttachChannel(_buzz, 1000, 8, _ch);
  ledcWrite(_buzz, 0);
}

void EmberBuzzer::setFrequencies(uint16_t fGas, uint16_t fFire, uint16_t fCal) {
  _fGas=fGas; _fFire=fFire; _fCal=fCal;
}

void EmberBuzzer::_ledsBoth(bool on) {
  digitalWrite(_ledGas,  on ? HIGH : LOW);
  digitalWrite(_ledFire, on ? HIGH : LOW);
}

void EmberBuzzer::_buzzOn(uint16_t hz) {
  ledcWriteTone(_buzz, hz);
}

void EmberBuzzer::_buzzOff() {
  ledcWrite(_buzz, 0);
}

// Cache p/ nao reconfigurar LEDC a cada loop na calibracao
void EmberBuzzer::_calTone(uint16_t hz) {
  if (hz == _calHz) return;
  _calHz = hz;
  if (hz == 0) _buzzOff();
  else         _buzzOn(hz);
}

// ── Port direto do Tiago-TX: ativarAlarme / atualizarAlarme / desativarAlarme ──

void EmberBuzzer::_ativarAlarme(uint16_t freq, uint32_t dur, uint32_t pausa,
                                uint32_t agora) {
  if (!_alarmeAtivo || _alarmFreq != freq) {
    _alarmeAtivo = true;
    _alarmFreq   = freq;
    _beepDuracao = dur;
    _beepPausa   = pausa;
    _beepEstado  = true;
    _ultimoBeep  = agora;
    _buzzOn(freq);
  }
}

void EmberBuzzer::_desativarAlarme() {
  _alarmeAtivo = false;
  _buzzOff();
}

void EmberBuzzer::_atualizarAlarme(uint8_t estado, uint32_t agora) {
  if (!_alarmeAtivo) {
    _buzzOff();
    _ledsBoth(false);
    return;
  }
  uint32_t intervalo = _beepEstado ? _beepDuracao : _beepPausa;
  if (agora - _ultimoBeep >= intervalo) {
    _ultimoBeep = agora;
    _beepEstado = !_beepEstado;
    if (_beepEstado) {
      _buzzOn(_alarmFreq);
      digitalWrite(_ledGas,  (estado == 1 || estado == 3) ? HIGH : LOW);
      digitalWrite(_ledFire, (estado >= 2)                ? HIGH : LOW);
    } else {
      _buzzOff();
      _ledsBoth(false);
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────

void EmberBuzzer::update(uint8_t ackByte) {
  uint8_t  estado   = ackByte & 0x03;
  CalPhase calPhase = (CalPhase)((ackByte >> 2) & 0x07);
  uint32_t now      = millis();

  if (calPhase != _phase) {
    if (_phase != PH_IDLE && calPhase == PH_IDLE) {
      _desativarAlarme();
      _lastEstado = 0xFF;
      _calHz      = 0;
    }
    _phase       = calPhase;
    _tPhaseEnter = now;
  }

  if (calPhase != PH_IDLE) {
    _drivePhase(now);
  } else {
    _driveAlarms(estado, now);
  }
}

void EmberBuzzer::_drivePhase(uint32_t now) {
  uint32_t t = now - _tPhaseEnter;

  switch (_phase) {
    case PH_RAMP_DOWN:
    case PH_SILENT_500:
    case PH_SILENT_PRE:
    case PH_SILENT_POST:
      _calTone(0);
      _ledsBoth(false);
      break;

    case PH_WARN_BURST: {
      bool on = ((t % 80) < 50);
      _calTone(on ? _fCal : 0);
      _ledsBoth(on);
      break;
    }

    case PH_COUNTDOWN: {
      bool on = ((t % 1000) < 300);
      _calTone(on ? _fCal : 0);
      _ledsBoth(on);
      break;
    }

    case PH_CAL_RUN: {
      bool on = ((t / 100) % 2) == 0;
      _calTone(on ? _fCal : 0);
      _ledsBoth(on);
      break;
    }

    default:
      _calTone(0);
      _ledsBoth(false);
      break;
  }
}

void EmberBuzzer::_driveAlarms(uint8_t estado, uint32_t now) {
  if (estado != _lastEstado) {
    _lastEstado = estado;
    switch (estado) {
      case 1:
        _ativarAlarme(_fGas,  400, 400, now);
        digitalWrite(_ledGas, HIGH); digitalWrite(_ledFire, LOW);
        break;
      case 2:
        _ativarAlarme(_fFire, 150, 80, now);
        digitalWrite(_ledFire, HIGH); digitalWrite(_ledGas, LOW);
        break;
      case 3:
        _ativarAlarme(_fFire, 150, 80, now);
        digitalWrite(_ledFire, HIGH); digitalWrite(_ledGas, HIGH);
        break;
      default:
        _desativarAlarme();
        _ledsBoth(false);
        break;
    }
  }

  _atualizarAlarme(estado, now);
}
