// ============================================================
// EmberESC.cpp, Implementação
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementa os métodos da classe EmberESC, que controla os
// quatro ESC (Electronic Speed Controller, controlador eletrónico
// de velocidade) do drone via biblioteca ESP32Servo. Gera pulsos
// PWM (Pulse Width Modulation, modulação por largura de pulso)
// entre 1000 e 2000 us a 50 Hz (Hz, Hertz, unidade de frequência,
// ciclos por segundo) para regular a velocidade de cada motor.
// Nota: este ficheiro não é usado pelo firmware final
// Goncalo-Ember.ino, que substituiu ESP32Servo por controlo
// LEDC (LED Controller, controlador PWM interno do ESP32) direto,
// contornando incompatibilidades do ESP32Servo@3.1.3 com o
// ESP32-S3 e a versão do IDF instalada.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: reserva os quatro timers LEDC antes de qualquer
//             attach, para evitar que RF24 ou outros periféricos
//             os ocupem primeiro.
//    Passo 2: aloca quatro objetos Servo no heap e chama
//             setPeriodHertz(50) antes de attach.
//    Passo 3: faz attach de cada pino com range 1000-2000 us.
//    Passo 4: envia THROTTLE_MIN e aguarda 100 ms (milissegundos)
//             para os ESC reconhecerem o sinal de arranque.
// 2. writeWithAll() no loop() quando armado:
//    Aplica a mistura de throttle, yaw, pitch e roll a cada
//    motor segundo a configuração de quadricóptero em X.
// 3. updateRamp() no loop():
//    Avança _currentThrottle em RAMP_STEP (2 us) a cada
//    RAMP_DELAY (40 ms) até atingir _targetThrottle.
// 4. emergencyStop() em qualquer momento:
//    Corte imediato para THROTTLE_MIN, sem ramp gradual.
//
// HARDWARE
// -----------------------------------------------------------
// Layout dos motores (quadricóptero em X, visto de cima):
//   Motor 0: Frente-Esquerda,  CW  (sentido horário)
//   Motor 1: Frente-Direita,   CCW (contra horário)
//   Motor 2: Tras-Direita,     CCW
//   Motor 3: Tras-Esquerda,    CW
// ESC alvo: HobbyKing 20A, calibrado com CALIBRAR.ino.
// ============================================================

#include "EmberESC.h"

// -----------------------------------
// Inicialização
// -----------------------------------

bool EmberESC::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4) {
  _pin[0] = pin1;
  _pin[1] = pin2;
  _pin[2] = pin3;
  _pin[3] = pin4;

  Serial.println("[ESC] ESP32Servo: allocate timers 0-3...");
  Serial.flush();

  // CRITICO: reservar timers LEDC ANTES de qualquer attach,
  // para evitar que RF24/thermal/outros apanhem timers primeiro.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Alocar objectos Servo no heap — evita static init order
  // fiasco com outros globais que tocam em LEDC/PWM.
  for (int i = 0; i < 4; i++) {
    Serial.print("[ESC] new Servo "); Serial.print(i); Serial.print("... ");
    Serial.flush();
    _esc[i] = new Servo();
    if (!_esc[i]) {
      Serial.println("FALHOU (OOM)");
      return false;
    }
    Serial.println("OK");
    Serial.flush();
  }

  // setPeriodHertz tem de vir ANTES de attach
  for (int i = 0; i < 4; i++) {
    _esc[i]->setPeriodHertz(50);
  }

  // attach com range 1000-2000 (HobbyKing 20A calibrado em CALIBRAR.ino)
  for (int i = 0; i < 4; i++) {
    Serial.print("[ESC] attach pin "); Serial.print(_pin[i]); Serial.print("... ");
    Serial.flush();
    if (!_esc[i]->attach(_pin[i], 1000, 2000)) {
      Serial.println("FALHOU");
      return false;
    }
    Serial.println("OK");
    Serial.flush();
  }

  writeAll(THROTTLE_MIN);
  delay(100);
  Serial.println("[ESC] ESP32Servo OK (50Hz, 1000-2000us).");
  Serial.flush();
  return true;
}

// -----------------------------------
// Escrita direta em todos os ESC
// -----------------------------------

void EmberESC::writeAll(int us) {
  for (int i = 0; i < 4; i++) {
    if (_esc[i]) _esc[i]->writeMicroseconds(us);
  }
}

// -----------------------------------
// Escrita com yaw
// -----------------------------------

void EmberESC::writeWithYaw(int base, int yaw) {
  int safeMin = _armed ? THROTTLE_ARM + 50 : THROTTLE_MIN;
  if (_esc[0]) _esc[0]->writeMicroseconds(constrain(base + yaw, safeMin, 2000));
  if (_esc[1]) _esc[1]->writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  if (_esc[2]) _esc[2]->writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  if (_esc[3]) _esc[3]->writeMicroseconds(constrain(base + yaw, safeMin, 2000));
}

// -----------------------------------
// Mistura completa: throttle, yaw, pitch e roll
// -----------------------------------

void EmberESC::writeWithAll(int base, int yaw, int pitch, int roll) {
  if (!_armed) return;
  int safeMin = THROTTLE_ARM + 50;

  // Motor 0 — Frente-Esquerda
  if (_esc[0]) _esc[0]->writeMicroseconds(
    constrain(base + yaw + pitch + roll, safeMin, 2000));

  // Motor 1 — Frente-Direita
  if (_esc[1]) _esc[1]->writeMicroseconds(
    constrain(base - yaw + pitch - roll, safeMin, 2000));

  // Motor 2 — Tras-Direita
  if (_esc[2]) _esc[2]->writeMicroseconds(
    constrain(base - yaw - pitch - roll, safeMin, 2000));

  // Motor 3 — Tras-Esquerda
  if (_esc[3]) _esc[3]->writeMicroseconds(
    constrain(base + yaw - pitch + roll, safeMin, 2000));
}

// -----------------------------------
// Arming e estado
// -----------------------------------

void EmberESC::setArmed(bool arm) {
  if (arm && !_armed) {
    _armed = true;
    _targetThrottle = THROTTLE_NORMAL;
    _currentThrottle = THROTTLE_MIN;
    _rampActive = true;
    Serial.println("[ESC] ARM: ON");
  } else if (!arm && _armed) {
    _armed = false;
    _targetThrottle = THROTTLE_MIN;
    _currentThrottle = THROTTLE_MIN;
    _rampActive = false;
    Serial.println("[ESC] ARM: OFF");
  }
}

bool EmberESC::isArmed()   { return _armed; }
bool EmberESC::isRamping() { return _rampActive; }

// -----------------------------------
// Ramp não bloqueante
// -----------------------------------

void EmberESC::updateRamp() {
  if (!_rampActive) return;
  if (abs(_currentThrottle - _targetThrottle) > RAMP_STEP) {
    _currentThrottle += (_currentThrottle < _targetThrottle) ? RAMP_STEP : -RAMP_STEP;
    _currentThrottle = constrain(_currentThrottle, THROTTLE_MIN, 2000);
    writeAll(_currentThrottle);
  } else {
    _currentThrottle = _targetThrottle;
    writeAll(_currentThrottle);
    _rampActive = false;
    Serial.println("[ESC] Ramp concluido.");
  }
}

// -----------------------------------
// Paragem de emergência
// -----------------------------------

void EmberESC::emergencyStop() {
  _armed = false;
  _rampActive = false;
  _targetThrottle = THROTTLE_MIN;
  _currentThrottle = THROTTLE_MIN;
  writeAll(THROTTLE_MIN);
  Serial.println("[ESC] Emergency STOP");
}
