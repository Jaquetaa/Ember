// ============================================================
// EmberESC.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface da classe EmberESC, que abstrai o controlo
// dos quatro ESC (Electronic Speed Controller, controlador
// eletrónico de velocidade) do drone via biblioteca ESP32Servo.
// Cada ESC recebe um pulso PWM (Pulse Width Modulation, modulação
// por largura de pulso) entre 1000 e 2000 us para regular a
// velocidade do motor correspondente.
// Nota: o firmware final Goncalo-Ember.ino não usa esta classe,
// pois substituiu ESP32Servo por controlo LEDC (LED Controller,
// controlador PWM interno do ESP32) direto, contornando
// incompatibilidades do ESP32Servo@3.1.3 com o ESP32-S3 e a
// versão do IDF instalada.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. Os ponteiros Servo* (_esc[4]) ficam a nullptr (ponteiro
//    nulo, indica que o objeto ainda não foi criado) até begin()
//    ser chamado em setup().
// 2. begin() em setup():
//    Passo 1: reserva quatro timers LEDC com allocateTimer()
//             antes de qualquer attach.
//    Passo 2: aloca quatro objetos Servo no heap para evitar
//             conflitos de static init order com globais.
//    Passo 3: configura 50 Hz (Hz, Hertz, unidade de frequência,
//             ciclos por segundo) e faz attach com range
//             1000-2000 us em cada pino.
//    Passo 4: envia THROTTLE_MIN a todos os ESC e aguarda
//             100 ms (milissegundos).
// 3. updateRamp() no loop():
//    Avança _currentThrottle em RAMP_STEP (2 us) a cada
//    RAMP_DELAY (40 ms) sem usar delay(), para não bloquear
//    o loop principal.
// 4. emergencyStop() a qualquer momento:
//    Corte imediato para THROTTLE_MIN, cancela o ramp.
//
// HARDWARE
// -----------------------------------------------------------
// ESC alvo:  HobbyKing 20A, calibrado com CALIBRAR.ino.
// Protocolo: PWM a 50 Hz, período 20 000 us, pulso 1000-2000 us.
// ============================================================

 #ifndef EMBER_ESC_H
#define EMBER_ESC_H

#include <Arduino.h>
#include <ESP32Servo.h>

// -----------------------------------
// Constantes de throttle e movimento
// -----------------------------------

// Throttle de cruzeiro ao armar (abaixo do hover real, para subida suave)
#define THROTTLE_NORMAL 1300
// Limite inferior do stick de throttle mapeado pelo controlador
#define THROTTLE_Y_LOW  1200
// Limite superior do stick de throttle mapeado pelo controlador
#define THROTTLE_Y_HIGH 1500
// Pulso mínimo absoluto: ESC desarmado ou parado
#define THROTTLE_MIN    1000
// Pulso mínimo com drone armado: mantém motores a girar sem levantar
#define THROTTLE_ARM    1100
// Amplitude máxima de yaw em unidades de stick (-100 a +100)
#define YAW_MAX         100
// Incremento de throttle por passo de ramp em microsegundos
#define RAMP_STEP       2
// Intervalo mínimo entre passos de ramp em ms
#define RAMP_DELAY      40

// -----------------------------------
// Declaração da classe EmberESC
// -----------------------------------

class EmberESC {
public:
  bool begin(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);
  void writeAll(int us);
  void writeWithYaw(int base, int yaw);
  void writeWithAll(int base, int yaw, int pitch, int roll);
  void setArmed(bool arm);
  bool isArmed();
  bool isRamping();
  void updateRamp();
  void emergencyStop();

private:
  // Ponteiros alocados no heap dentro de begin() para evitar
  // conflitos de static init order com outros globais (RF24, MLX, etc).
  Servo*  _esc[4] = {nullptr, nullptr, nullptr, nullptr};
  uint8_t _pin[4] = {0, 0, 0, 0};
  bool    _armed = false;
  bool    _rampActive = false;
  int     _currentThrottle = THROTTLE_MIN;
  int     _targetThrottle  = THROTTLE_MIN;
};

#endif
