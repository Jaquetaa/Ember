// ============================================================
// EmberBuzzer, Header (Cabecalho)
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Classe que controla o buzzer passivo e os dois LEDs de
// alarme com base no byte ACK (Acknowledgement, confirmacao
// de rececao) recebido do drone. Um buzzer passivo nao
// produz som sozinho: precisa de receber um sinal PWM (Pulse
// Width Modulation, modulacao por largura de pulso) de uma
// frequencia especifica para vibrar e produzir um tom audivel.
// Existe no projeto para encapsular toda a logica de alerta
// sonoro e visual, mantendo o loop() principal limpo.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. begin(buzzPin, ledGasPin, ledFirePin) e chamado uma vez
//    em setup(): guarda os pinos, configura os LEDs como
//    saida (OUTPUT), associa o canal LEDC (LED Controller,
//    controlador PWM interno do ESP32) ao pino do buzzer,
//    e coloca o buzzer em silencio com duty cycle (ciclo
//    de trabalho) igual a zero.
// 2. update(ackByte) e chamado em cada iteracao de loop():
//    Passo 1: extrai calPhase (fase de calibracao) dos
//             bits[4:2] do ackByte com >> 2 e & 0x07.
//    Passo 2: extrai o estado do sensor dos bits[1:0]
//             com & 0x03.
//    Passo 3: se calPhase for diferente de zero, chama
//             _driveCalibration() (calibracao tem prioridade).
//    Passo 4: caso contrario, chama _driveSensor() para
//             alarmes normais de CO ou chama.
//
// FORMATO DO ACKBYTE (1 byte, 8 bits)
// -----------------------------------------------------------
//   bits[1:0] = estado do sensor (2 bits, valores 0 a 3)
//     0 = tudo OK
//     1 = CO (monoxido de carbono) detetado
//     2 = Chama detetada
//     3 = CO mais Chama em simultaneo
//
//   bits[4:2] = fase de calibracao (3 bits, valores 0 a 7)
//     0       = funcionamento normal
//     1 a 7   = sequencia de calibracao dos ESCs (Electronic
//               Speed Controller, controlador eletronico de
//               velocidade dos motores) do drone
//
// PRIORIDADE
// -----------------------------------------------------------
//   Calibracao tem prioridade sobre alarmes de sensor.
//   Enquanto calPhase (fase de calibracao) for diferente de
//   zero, os alarmes de sensor ficam suspensos.
//
// HARDWARE
// -----------------------------------------------------------
//   Buzzer passivo: GPIO10 (gera tom via sinal PWM)
//   LED gas:        GPIO3  (LED amarelo)
//   LED chama:      GPIO8  (LED vermelho)
//   GPIO = General Purpose Input/Output, pino de uso geral
//
// FREQUENCIAS DOS TONS
// -----------------------------------------------------------
//   Hz = Hertz, unidade de frequencia (ciclos por segundo)
//   CO, gas:     600 Hz  (tom grave, mais baixo)
//   Chama:      1300 Hz  (tom agudo, mais alto)
//   Calibracao: 1500 Hz  (tom mais agudo ainda)
//
// CANAL LEDC
// -----------------------------------------------------------
//   LEDC = LED Controller, controlador PWM interno do ESP32.
//   Canal 5 por omissao (pode ser alterado no begin()).
//   Usa resolucao de 8 bits e frequencia base de 1000 Hz.
// ============================================================

#ifndef EMBER_BUZZER_H
#define EMBER_BUZZER_H
#include <Arduino.h>


// -----------------------------------
// Classe EmberBuzzer
// -----------------------------------

class EmberBuzzer {
public:
  // Inicializa os pinos e configura o canal PWM (LEDC) do ESP32.
  void begin(uint8_t buzzPin, uint8_t ledGasPin, uint8_t ledFirePin,
             uint8_t ledcChannel = 5);

  // Atualiza o estado do buzzer e LEDs com base no ackByte.
  // Deve ser chamado a cada iteracao do loop().
  void update(uint8_t ackByte);

private:
  uint8_t  _buzz;          // pino do buzzer
  uint8_t  _ledGas;        // pino do LED de gas (amarelo)
  uint8_t  _ledFire;       // pino do LED de chama (vermelho)
  uint8_t  _ch;            // canal LEDC (PWM interno do ESP32)
  uint16_t _currentFreq = 0xFFFF;  // frequencia ativa no momento (0xFFFF = nao inicializado)

  // Estado da maquina de beep pulsado (non-blocking, sem bloqueio)
  uint8_t  _lastCalPhase = 0;    // ultima fase de calibracao (para detetar mudancas)
  bool     _calBeepOn    = false; // se o beep esta atualmente ligado ou desligado
  uint32_t _calLastBeep  = 0;    // timestamp em ms do ultimo toggle (alternancia)

  void _buzzOn(uint16_t hz);                                          // liga buzzer a uma frequencia
  void _buzzOff();                                                    // desliga buzzer
  void _ledsSet(bool gasOn, bool fireOn);                             // acende/apaga os LEDs
  void _calBeep(uint32_t durOn, uint32_t durOff, uint16_t freq, uint32_t now); // beep pulsado sem delay
  void _driveCalibration(uint8_t calPhase, uint32_t now);            // padroes sonoros de calibracao
  void _driveSensor(uint8_t estado);                                  // padroes sonoros de sensor
};

#endif
