#include "EmberAlarm.h"

#define FREQ_CO          700
#define FREQ_FOGO       1400
#define BEEP_DUR_CO     400UL
#define BEEP_PAUSA_CO   400UL
#define BEEP_DUR_FOGO   150UL
#define BEEP_PAUSA_FOGO  80UL

EmberAlarm::EmberAlarm(uint8_t buzzerPin, uint8_t ledPin)
  : _buzzerPin(buzzerPin), _ledPin(ledPin),
    _ativo(false), _freq(0), _duracao(0), _pausa(0),
    _beepEstado(false), _ultimoBeep(0) {}

void EmberAlarm::begin() {
  pinMode(_buzzerPin, OUTPUT);
  pinMode(_ledPin, OUTPUT);
  noTone(_buzzerPin);
  digitalWrite(_ledPin, LOW);
}

void EmberAlarm::update() {
  if (!_ativo) {
    noTone(_buzzerPin);
    digitalWrite(_ledPin, LOW);
    return;
  }
  unsigned long agora = millis();
  unsigned long intervalo = _beepEstado ? _duracao : _pausa;
  if (agora - _ultimoBeep >= intervalo) {
    _ultimoBeep = agora;
    _beepEstado = !_beepEstado;
    if (_beepEstado) { tone(_buzzerPin, _freq); digitalWrite(_ledPin, HIGH); }
    else             { noTone(_buzzerPin);       digitalWrite(_ledPin, LOW);  }
  }
}

void EmberAlarm::ativar(int freq, unsigned long dur, unsigned long pausa) {
  if (!_ativo || _freq != freq) {
    _ativo     = true;
    _freq      = freq;
    _duracao   = dur;
    _pausa     = pausa;
    _beepEstado = true;
    _ultimoBeep = millis();
    tone(_buzzerPin, freq);
    digitalWrite(_ledPin, HIGH);
  }
}

void EmberAlarm::desativar() {
  _ativo = false;
  noTone(_buzzerPin);
  digitalWrite(_ledPin, LOW);
}

void EmberAlarm::aplicarEstado(uint8_t estado) {
  switch (estado) {
    case 0: desativar();
            Serial.println("[ALARM] OK");
            break;
    case 1: ativar(FREQ_CO, BEEP_DUR_CO, BEEP_PAUSA_CO);
            Serial.println("[ALARM] CO/Gas! 700Hz");
            break;
    case 2: ativar(FREQ_FOGO, BEEP_DUR_FOGO, BEEP_PAUSA_FOGO);
            Serial.println("[ALARM] FOGO! 1400Hz");
            break;
    case 3: ativar(FREQ_FOGO, BEEP_DUR_FOGO, BEEP_PAUSA_FOGO);
            Serial.println("[ALARM] FOGO + CO! 1400Hz");
            break;
    default: desativar(); break;
  }
}
