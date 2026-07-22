// ============================================================
// EmberSensor.cpp, Implementação
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementa a classe EmberSensor, que lê o sensor de chama
// analógico e o sensor MQ-7 de monóxido de carbono (CO).
// A função update() é chamada em cada iteração do loop principal
// e atualiza o byte de estado (_estado, 0 a 3) que é depois
// incluído no payload ACK (Acknowledgement, confirmação de
// receção) enviado ao controlador do Tiago via NRF24L01+.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: guarda os pinos e configura INPUT_PULLUP no pino
//             de salto (skipPin), se diferente de 255.
//    Passo 2: configura o ADC (Analog to Digital Converter,
//             conversor analógico-digital) a 12 bits (0-4095),
//             para maior resolução na leitura do MQ-7.
//    Passo 3: regista o instante de início do pré-aquecimento
//             com millis() (função que devolve os milissegundos
//             (ms) desde que o controlador ligou).
// 2. update() no loop():
//    Passo 1: verifica pré-aquecimento: 180 s após begin(), ou
//             ao receber qualquer byte pela porta série, ou ao
//             pressionar o botão skipPin (LOW ativo).
//    Passo 2: lê o sensor de chama via ADC. Incrementa
//             _contadorChama enquanto a leitura for inferior ao
//             limiar FLAME_THRESHOLD; repõe a zero caso contrário.
//             Confirma deteção só após CONFIRMACOES (3) leituras
//             consecutivas, evitando falsos positivos por ruído.
//    Passo 3: se o MQ-7 já aqueceu, lê o pin analógico, converte
//             para tensão (0 a 3.3 V) e calcula ppm de CO com a
//             curva Rs/Ro aproximada do datasheet.
//    Passo 4: combina chama e CO no byte _estado (0-3).
// 3. getEstado() devolve _estado ao caller (EmberDroneNRF).
//
// CÁLCULO DE PPM DE CO (MQ-7)
// -----------------------------------------------------------
// Tensão lida: V = ADC * (3.3 / 4095)
// Rácio de resistências: Rs/Ro = (3.3 - V) / V
// Aproximação logarítmica do datasheet:
//   ppm = 100 * (Rs/Ro / 5.0) ^ -1.5
// Limiar de alerta: CO_PPM_LIMITE = 160 ppm.
// ============================================================

#include "EmberSensor.h"

// -----------------------------------
// Inicialização
// -----------------------------------

void EmberSensor::begin(uint8_t flamePin, uint8_t mq7Pin, uint8_t skipPin) {
  _flamePin = flamePin;
  _mq7Pin = mq7Pin;
  _skipPin = skipPin;
  if (_skipPin != 255) pinMode(_skipPin, INPUT_PULLUP);
  analogReadResolution(12); // 12 bits: resolução de 4096 níveis (0-4095)
  _tempoInicio = millis();
}

// -----------------------------------
// Leitura e cálculo de estado
// -----------------------------------

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
    float rs_ro  = (tensao > 0.01f) ? (3.3f - tensao) / tensao : 99.0f; // 0.01 V evita divisão por zero
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

// -----------------------------------
// Acesso ao byte de estado
// -----------------------------------

uint8_t EmberSensor::getEstado() {
  return _estado;
}
