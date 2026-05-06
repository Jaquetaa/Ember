#include "EmberBuzzer.h"

#define FREQ_CO    600
#define FREQ_FIRE  1300
#define FREQ_CAL   1500   // FREQ_FIRE + 100

void EmberBuzzer::begin(uint8_t buzzPin, uint8_t ledGas, uint8_t ledFire, uint8_t ch) {
  _buzz = buzzPin; _ledGas = ledGas; _ledFire = ledFire; _ch = ch;
  pinMode(_ledGas,  OUTPUT); digitalWrite(_ledGas,  LOW);
  pinMode(_ledFire, OUTPUT); digitalWrite(_ledFire, LOW);
  ledcAttachChannel(_buzz, 1000, 8, _ch);
  ledcWrite(_buzz, 0);
  _currentFreq = 0;
}

void EmberBuzzer::_buzzOn(uint16_t hz) {
  if (hz != _currentFreq) { _currentFreq = hz; ledcWriteTone(_buzz, hz); }
}
void EmberBuzzer::_buzzOff() {
  if (_currentFreq != 0) { _currentFreq = 0; ledcWrite(_buzz, 0); }
}
void EmberBuzzer::_ledsSet(bool gas, bool fire) {
  digitalWrite(_ledGas,  gas  ? HIGH : LOW);
  digitalWrite(_ledFire, fire ? HIGH : LOW);
}

// Maquina de beep non-blocking para calibracao.
// Reset automatico quando durOn/durOff mudam (nova fase).
void EmberBuzzer::_calBeep(uint32_t durOn, uint32_t durOff, uint16_t freq, uint32_t now) {
  uint32_t elapsed = now - _calLastBeep;
  uint32_t interval = _calBeepOn ? durOn : durOff;
  if (elapsed >= interval) {
    _calLastBeep = now;
    _calBeepOn   = !_calBeepOn;
    if (_calBeepOn) { _buzzOn(freq); _ledsSet(true, true); }
    else            { _buzzOff();    _ledsSet(false, false); }
  }
}

void EmberBuzzer::_driveCalibration(uint8_t calPhase, uint32_t now) {
  // Reset da maquina de beep quando muda de fase
  if (calPhase != _lastCalPhase) {
    _lastCalPhase = calPhase;
    _calBeepOn    = false;
    _calLastBeep  = now - 10000UL; // forca arranque imediato
  }

  switch (calPhase) {
    case 1: // PH_RAMP_DOWN — LEDs ligados, sem buzzer (descida silenciosa)
      _buzzOff();
      _ledsSet(true, true);
      break;

    case 2: // PH_WARN_BURST — 25 beeps em 1500 ms (30 ms ON / 30 ms OFF)
      _calBeep(30, 30, FREQ_CAL, now);
      break;

    case 3: // PH_SILENT_500 — silencio
      _buzzOff(); _ledsSet(false, false);
      break;

    case 4: // PH_COUNTDOWN — 5 ticks: 300 ms ON / 700 ms OFF
      _calBeep(300, 700, FREQ_CAL, now);
      break;

    case 5: // PH_SILENT_PRE — silencio (aviso "desliga/liga bateria ESC")
      _buzzOff(); _ledsSet(false, false);
      break;

    case 6: // PH_CAL_RUN — frenético durante MAX→MIN (40 ms ON / 30 ms OFF)
      _calBeep(40, 30, FREQ_CAL, now);
      break;

    case 7: // PH_SILENT_POST — silencio (re-arma e volta a IDLE)
      _buzzOff(); _ledsSet(false, false);
      break;

    default:
      _buzzOff(); _ledsSet(false, false);
      break;
  }
}

void EmberBuzzer::_driveSensor(uint8_t estado) {
  switch (estado) {
    case 0:
      _buzzOff();
      _ledsSet(false, false);
      break;
    case 1: // CO/Gas — continuo
      _buzzOn(FREQ_CO);
      _ledsSet(true, false);
      break;
    case 2: // Fogo — continuo
      _buzzOn(FREQ_FIRE);
      _ledsSet(false, true);
      break;
    case 3: // Fogo + CO — continuo
      _buzzOn(FREQ_FIRE);
      _ledsSet(true, true);
      break;
  }
}

void EmberBuzzer::update(uint8_t ackByte) {
  uint8_t calPhase = (ackByte >> 2) & 0x07;
  uint8_t estado   = ackByte & 0x03;
  uint32_t now     = millis();

  if (calPhase != 0) {
    // Calibracao tem prioridade
    if (_lastCalPhase == 0) {
      // Acabou de entrar em calibracao: reset limpo
      _calBeepOn   = false;
      _calLastBeep = now - 10000UL;
    }
    _driveCalibration(calPhase, now);
  } else {
    // Voltou ao normal: reset estado de calibracao
    if (_lastCalPhase != 0) {
      _lastCalPhase = 0;
    }
    _driveSensor(estado);
  }
}
