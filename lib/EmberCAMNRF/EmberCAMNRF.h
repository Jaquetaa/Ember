// ============================================================
// EmberCAMNRF.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface mínima da classe EmberCAMNRF, responsável
// por capturar frames térmicos do sensor MLX90640 (32x24 píxeis)
// e transmiti-los fragmentados em 32 pacotes via NRF24L01+ para
// o controlador do Tiago.
// Os includes pesados (Adafruit_MLX90640, RF24, Wire) ficam
// confinados ao EmberCAMNRF.cpp, evitando poluir o escopo de
// Goncalo-Ember.ino. Os ponteiros internos (_radio, _mlx) são
// declarados como void* para ocultar os tipos concretos.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: aloca no heap 768 floats para o frame bruto e
//             768 bytes para os dados normalizados.
//    Passo 2: inicia o barramento I2C (Wire) nos pinos SDA/SCL
//             a 400 kHz (quilohertz, 1000 Hz (Hz, Hertz, unidade
//             de frequência, ciclos por segundo)) e faz scan
//             dos endereços 0x03 a 0x77.
//    Passo 3: inicializa o MLX90640 em modo Chess a 8 Hz com
//             ADC (Analog to Digital Converter, conversor
//             analógico-digital) de 18 bits.
//    Passo 4: aloca RF24 no heap e tenta begin() até 3 vezes.
//             Se bem sucedido: canal 2, 250 kbps (quilobits por
//             segundo), PA (Power Amplifier, amplificador de
//             potência) MAX, sem ACK (Acknowledgement,
//             confirmação de receção), sem CRC (Cyclic Redundancy
//             Check, verificação cíclica de redundância),
//             payload (conjunto de dados úteis enviados numa
//             mensagem) fixo de 25 bytes.
// 2. update() no loop():
//    Máquina de estados não bloqueante com dois estados:
//    Estado 0 (IDLE): aguarda 125 ms entre frames, lê o MLX90640
//    via I2C e normaliza as 768 temperaturas para bytes 0-255.
//    Estado 1 (TX_BURST): envia 4 pacotes ThermalPacket de 25
//    bytes por chamada via writeFast(). Ao completar 32 pacotes,
//    chama txStandBy() e volta a IDLE.
//
// RÁDIO
// -----------------------------------------------------------
// Canal:    2 (2402 MHz, separado do canal de controlo 76)
// Débito:   250 kbps
// Potência: PA_MAX
// ACK:      desativado (fluxo unidirecional: drone->controlador)
// CRC:      desativado (sem retransmissões, débito máximo)
// Endereço: "THERM"
//
// FORMATO DO PACOTE TÉRMICO
// -----------------------------------------------------------
// struct (estrutura de dados, agrupa vários campos num único
// bloco) ThermalPacket, 25 bytes:
//   index:    uint8_t   índice do pacote (0..31)
//   data[24]: uint8_t   24 bytes de temperatura normalizada
// 32 pacotes x 24 bytes = 768 bytes = frame completo 32x24.
// ============================================================

#ifndef EMBER_CAM_NRF_H
#define EMBER_CAM_NRF_H

#include <Arduino.h>

// Header minimo — os includes pesados (MLX90640, RF24, Wire)
// ficam so no .cpp para nao poluir o scope do .ino

class SPIClass;

// -----------------------------------
// Declaração da classe EmberCAMNRF
// -----------------------------------

class EmberCAMNRF {
public:
  bool begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi);
  void update();

private:
  void *_radio = nullptr;      // RF24* cast internamente no .cpp
  void *_mlx = nullptr;        // Adafruit_MLX90640* cast internamente no .cpp
  bool _nrfOK = false;
  uint32_t _frameCount = 0;
  float   *_mlxFrame = nullptr;
  uint8_t *_thermalData = nullptr;

  uint8_t _sdaPin = 0;
  uint8_t _sclPin = 0;

  // Maquina de estados nao-bloqueante:
  //  0 = IDLE (espera intervalo entre frames)
  //  1 = TX_BURST (a enviar pacotes em batches)
  uint8_t  _state = 0;
  uint8_t  _txIndex = 0;        // proximo pacote (0..31) a enviar no burst
  uint32_t _lastFrameMs = 0;    // millis() do inicio do ultimo frame
};

#endif
