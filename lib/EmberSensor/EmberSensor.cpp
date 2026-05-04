#include "EmberSensor.h"

void EmberSensor::begin(uint8_t flamePin, uint8_t mq7Pin, uint8_t skipPin) {
  _flamePin = flamePin;
  _mq7Pin = mq7Pin;
  _skipPin = skipPin;
  if (_skipPin != 255) pinMode(_skipPin, INPUT_PULLUP);
  analogReadResolution(12);
  _tempoInicio = millis();
}

void EmberSensor::update() {
  // Preheat
  if (!_mq7Aquecido) {
    bool btnSkip = (_skipPin != 255) && (digitalRead(_skipPin) == LOW);
    if (Serial.available() > 0 || btnSkip) {
      while (Serial.available()) Serial.read();
      _mq7Aquecido = true;
      Serial.println(btnSkip ? ">>> Aquecimento saltado (botao)! <<<"
                             : ">>> Aquecimento saltado! <<<");
    } else if ((millis() - _tempoInicio) >= PREHEAT_MS) {
      _mq7Aquecido = true;
      Serial.println("MQ-7 pronto!");
    }
  }

  // Flame
  _flameVal = analogRead(_flamePin);
  _contadorChama = (_flameVal < FLAME_THRESHOLD) ? _contadorChama + 1 : 0;
  bool chama = (_contadorChama >= CONFIRMACOES);
  _estado = 0;

  if (_mq7Aquecido) {
    _coVal       = analogRead(_mq7Pin);
    float tensao = _coVal * (3.3f / 4095.0f);
    float rs_ro  = (tensao > 0.01f) ? (3.3f - tensao) / tensao : 99.0f;
    float ppm    = constrain(100.0f * pow(rs_ro / 5.0f, -1.5f), 0.0f, 10000.0f);
    _coPpm       = ppm;
    bool  coAlto = (ppm > CO_PPM_LIMITE);
    if      (chama && coAlto) _estado = 3;
    else if (chama)           _estado = 2;
    else if (coAlto)          _estado = 1;
  } else {
    if (chama) _estado = 2;
  }
}

uint8_t EmberSensor::getEstado() {
  return _estado;
}
