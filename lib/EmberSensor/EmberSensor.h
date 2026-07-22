// ============================================================
// EmberSensor.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface da classe EmberSensor, responsável pela
// leitura dos dois sensores de deteção de incêndio: o sensor
// de chama analógico e o MQ-7 de monóxido de carbono (CO).
// Compila as leituras num byte de estado (0 a 3) enviado ao
// controlador do Tiago através do payload ACK (Acknowledgement,
// confirmação de receção) do rádio NRF24L01+.
// Relaciona-se com EmberDroneNRF, que usa getEstado() para
// construir o byte ACK, e com Goncalo-Ember.ino, que chama
// update() em cada iteração do loop principal.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: guarda os pinos e configura INPUT_PULLUP no pino
//             de salto de aquecimento (skipPin), se fornecido.
//    Passo 2: configura o ADC (Analog to Digital Converter,
//             conversor analógico-digital) a 12 bits (0-4095).
//    Passo 3: regista o instante de início do pré-aquecimento
//             com millis() (função que devolve os milissegundos
//             (ms) desde que o controlador ligou).
// 2. update() no loop():
//    Passo 1: verifica se o MQ-7 completou o pré-aquecimento
//             de 180 s (PREHEAT_MS), ou se foi saltado pelo
//             botão skipPin ou por um byte na porta série.
//    Passo 2: lê o sensor de chama e incrementa o contador de
//             confirmações consecutivas (CONFIRMACOES = 3) para
//             evitar falsos positivos por ruído.
//    Passo 3: se aquecido, lê o MQ-7, converte ADC para ppm
//             de CO com a curva Rs/Ro do datasheet.
//    Passo 4: calcula o byte _estado (0-3) combinando chama e CO.
// 3. getEstado() devolve _estado ao caller.
//
// BYTE DE ESTADO (2 bits)
// -----------------------------------------------------------
//   0: sem alerta
//   1: CO elevado (acima de 160 ppm)
//   2: chama detetada
//   3: chama e CO elevado (incêndio confirmado)
// ============================================================

#ifndef EMBER_SENSOR_H
#define EMBER_SENSOR_H

#include <Arduino.h>

// -----------------------------------
// Constantes de deteção
// -----------------------------------

// Limiar do ADC abaixo do qual o sensor de chama deteta fogo.
// O sensor é resistivo: quanto maior a intensidade de luz
// infravermelha, menor a tensão de saída.
#define FLAME_THRESHOLD 500
// Concentração de CO em ppm acima da qual se emite alerta
#define CO_PPM_LIMITE   160.0f
// Número de leituras consecutivas de chama para confirmar deteção
#define CONFIRMACOES    3
// Tempo de pré-aquecimento do MQ-7 em ms: o sensor precisa de
// 180 s para estabilizar a curva de sensibilidade ao CO
#define PREHEAT_MS      180000UL

// -----------------------------------
// Declaração da classe EmberSensor
// -----------------------------------

class EmberSensor {
public:
  void begin(uint8_t flamePin, uint8_t mq7Pin, uint8_t skipPin = 255);
  void update();
  uint8_t getEstado();
  int getFlameVal()  { return _flameVal; }
  int getCoVal()     { return _coVal; }
  float getCoPpm()   { return _coPpm; }
  bool isAquecido()  { return _mq7Aquecido; }
  unsigned long getPreheatRemaining() {
    if (_mq7Aquecido) return 0;
    unsigned long elapsed = millis() - _tempoInicio;
    return (elapsed >= PREHEAT_MS) ? 0 : (PREHEAT_MS - elapsed);
  }

private:
  uint8_t _flamePin, _mq7Pin;
  uint8_t _skipPin = 255;          // 255 indica que o pino de salto não foi configurado
  bool _mq7Aquecido = false;
  unsigned long _tempoInicio = 0;
  int _contadorChama = 0;          // leituras consecutivas abaixo do limiar FLAME_THRESHOLD
  uint8_t _estado = 0;
  int _flameVal = 0;
  int _coVal = 0;
  float _coPpm = 0.0f;
};

#endif
