// ============================================================
// EmberDroneNRF.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface da classe EmberDroneNRF, que gere o módulo
// NRF24L01+ usado pelo drone para receber comandos de controlo
// do operador (Tiago). Define também a estrutura PayloadCtrl,
// que descreve o formato exacto dos 9 bytes recebidos por rádio.
// Relaciona-se com Goncalo-Ember.ino, que chama receive() e
// updateAckPayload() em cada loop, e com EmberSensor, cujo byte
// de estado é incluído no ACK (Acknowledgement, confirmação de
// receção) de retorno.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: configura CSN (Chip Select Not, seleciona o
//             dispositivo no bus SPI (Serial Peripheral Interface,
//             protocolo de comunicação série)) como OUTPUT HIGH,
//             isolando o chip antes de qualquer transferência.
//    Passo 2: aloca o objeto RF24 no heap a 1 MHz, velocidade
//             reduzida para fios longos com maior imunidade a ruído.
//    Passo 3: chama _radio->begin() e isChipConnected() para
//             confirmar que o chip responde fisicamente via SPI.
//    Passo 4: configura canal 76, 250 kbps (quilobits por
//             segundo), PA (Power Amplifier, amplificador de
//             potência) MAX, ACK automático com payload de
//             retorno de 1 byte, ARD (Auto Retry Delay, atraso
//             entre retentativas automáticas) 500 us, ARC (Auto
//             Retry Count, número de retentativas automáticas) 3.
//    Passo 5: abre o pipe (canal de comunicação identificado
//             por um endereço no rádio) 1 no endereço "00001",
//             inicia escuta e pré-carrega ACK com byte zero.
// 2. updateAckPayload() antes de receive() em cada loop():
//    Substitui o ACK pendente pelo byte de estado atual dos
//    sensores e fase de calibração.
// 3. receive() em cada loop():
//    Drena todo o FIFO (3 slots) e regista _lastRxTime.
//
// RÁDIO
// -----------------------------------------------------------
// Módulo:    NRF24L01+ com antena PCB
// Canal:     76 (2476 MHz, afastado do Wi-Fi de 2.4 GHz)
// Débito:    250 kbps
// Potência:  PA_MAX
// ACK:       automático com payload de retorno de 1 byte
// Pipe RX:   1, endereço "00001"
// SPI:       1 MHz (velocidade reduzida para fios longos)
//
// FORMATO DO PAYLOAD (conjunto de dados úteis enviados numa
// mensagem) DE CONTROLO
// -----------------------------------------------------------
// struct (estrutura de dados, agrupa vários campos num único
// bloco) PayloadCtrl, 9 bytes:
//   armed:    uint8_t  0=parado, 1=armado, 2=emergência
//   throttle: int16_t  microsegundos (1000-1500)
//   yaw:      int16_t  -100 a +100
//   pitch:    int16_t  -100 a +100
//   roll:     int16_t  -100 a +100
// ============================================================

#ifndef EMBER_DRONE_NRF_H
#define EMBER_DRONE_NRF_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// -----------------------------------
// Estrutura do payload de controlo
// -----------------------------------

struct __attribute__((packed)) PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};
static_assert(sizeof(PayloadCtrl) == 9, "PayloadCtrl tem de ter exatamente 9 bytes (packed)");

// -----------------------------------
// Declaração da classe EmberDroneNRF
// -----------------------------------

class EmberDroneNRF {
public:
  bool begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi);
  bool isOK() const;
  bool receive(PayloadCtrl &p);
  void updateAckPayload(uint8_t estado);
  unsigned long lastReceiveTime();

private:
  RF24 *_radio = nullptr;           // nullptr: objeto ainda não criado, alocado em begin()
  unsigned long _lastRxTime = 0;
  bool _initialized = false;
  static const uint8_t ADDRESS[6]; // endereço de pipe partilhado com o TX do controlador
};

#endif
