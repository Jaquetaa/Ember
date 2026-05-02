#include "EmberAlarm.h"

#define FREQ_CO          700
#define FREQ_FOGO       1400
#define BEEP_DUR_CO     400UL
#define BEEP_PAUSA_CO   400UL
#define BEEP_DUR_FOGO   150UL
#define BEEP_PAUSA_FOGO  80UL

#define LEDC_RES   10           // resolucao 10-bit: duty 0-1023
#define LEDC_DUTY  512          // 50% duty = som audivel

EmberAlarm::EmberAlarm(uint8_t buzzerPin, uint8_t ledPin, uint8_t ledcChannel)
  : _buzzerPin(buzzerPin), _ledPin(ledPin), _channel(ledcChannel),
    _ativo(false), _freq(0), _duracao(0), _pausa(0),
    _beepEstado(false), _ultimoBeep(0) {}

void EmberAlarm::begin() {
  pinMode(_ledPin, OUTPUT);
  digitalWrite(_ledPin, LOW);
  // Fixa o canal LEDC ao pino — nunca faz detach, apenas alterna duty
  ledcAttachChannel(_buzzerPin, 1000, LEDC_RES, _channel);
  ledcWrite(_buzzerPin, 0);
}

void EmberAlarm::update() {
  if (!_ativo) return;  // desativar() ja apagou as saidas
  unsigned long agora = millis();
  unsigned long intervalo = _beepEstado ? _duracao : _pausa;
  if (agora - _ultimoBeep >= intervalo) {
    _ultimoBeep = agora;
    _beepEstado = !_beepEstado;
    if (_beepEstado) {
      ledcWrite(_buzzerPin, LEDC_DUTY);
      digitalWrite(_ledPin, HIGH);
    } else {
      ledcWrite(_buzzerPin, 0);
      digitalWrite(_ledPin, LOW);
    }
  }
}

void EmberAlarm::ativar(int freq, unsigned long dur, unsigned long pausa) {
  _duracao = dur;
  _pausa   = pausa;
  if (_freq != freq) {
    _freq = freq;
    // ledcChangeFrequency muda apenas a frequencia, sem alterar o duty
    ledcChangeFrequency(_buzzerPin, freq, LEDC_RES);
  }
  if (!_ativo) {
    _ativo      = true;
    _beepEstado = true;
    _ultimoBeep = millis();
    ledcWrite(_buzzerPin, LEDC_DUTY);
    digitalWrite(_ledPin, HIGH);
  }
}

void EmberAlarm::desativar() {
  _ativo = false;
  ledcWrite(_buzzerPin, 0);
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
