// ============================================================
// EmberCAMNRF.cpp, Implementação
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementa a classe EmberCAMNRF, responsável por capturar
// frames térmicos do sensor MLX90640 (32x24 píxeis, 768 valores
// float) e transmiti-los fragmentados em 32 pacotes de 25 bytes
// via NRF24L01+ para o controlador do Tiago.
// A função update() usa uma máquina de estados não bloqueante
// para não interferir com o loop de controlo de voo: nunca
// usa delay() e limita o trabalho por chamada a 4 pacotes.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: aloca no heap dois buffers: 768 floats para o
//             frame bruto do MLX e 768 bytes normalizados para TX.
//    Passo 2: inicia o barramento I2C (Wire) nos pinos SDA/SCL
//             a 400 kHz (quilohertz, 1000 Hz (Hz, Hertz, unidade
//             de frequência, ciclos por segundo)) e faz scan
//             dos endereços 0x03 a 0x77 para diagnóstico.
//    Passo 3: inicializa o MLX90640 em modo Chess a 8 Hz, com
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
//    Estado 0 (IDLE): aguarda 125 ms (milissegundos) entre frames
//    (alinhado ao ciclo de 8 Hz do MLX). Ao expirar, faz a leitura
//    I2C síncrona (~30-40 ms, limitação da lib Adafruit) e
//    normaliza as 768 temperaturas para bytes 0-255 no intervalo
//    TEMP_MIN (20 °C) a TEMP_MAX (60 °C). Se o MLX falhar 15
//    vezes seguidas, reinicia o I2C e o sensor.
//    Estado 1 (TX_BURST): envia 4 pacotes ThermalPacket por
//    chamada com writeFast(), sem esperar ACK, aproveitando os
//    3 slots do FIFO TX. Ao completar 32 pacotes, chama
//    txStandBy() para esvaziar o FIFO e volta a IDLE.
//
// RÁDIO
// -----------------------------------------------------------
// Canal:    2 (2402 MHz, separado do canal de controlo 76)
// Débito:   250 kbps
// Potência: PA_MAX
// ACK:      desativado (fluxo unidirecional: drone->controlador)
// CRC:      desativado (débito máximo, sem retransmissões)
// SPI (Serial Peripheral Interface, protocolo de comunicação
// série): 1 MHz (lento para fios longos)
// Endereço TX: "THERM"
//
// FORMATO DO PACOTE TÉRMICO
// -----------------------------------------------------------
// struct (estrutura de dados, agrupa vários campos num único
// bloco) ThermalPacket, packed (sem bytes de preenchimento entre
// campos, tamanho exato) implicitamente por ser só tipos básicos:
//   index:    uint8_t   índice do pacote (0..31)
//   data[24]: uint8_t   24 bytes de temperatura normalizada
// 32 x 24 = 768 bytes = frame completo 32x24 píxeis.
// ============================================================

#include "EmberCAMNRF.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <RF24.h>

// -----------------------------------
// Constantes de temperatura e endereço de rádio
// -----------------------------------

#define TEMP_MIN  20.0f
#define TEMP_MAX  60.0f

static const uint8_t THERM_ADDR[6] = "THERM";

// -----------------------------------
// Estrutura de pacote térmico
// -----------------------------------

struct ThermalPacket {
  uint8_t index;
  uint8_t data[24];
};

// -----------------------------------
// Inicialização
// -----------------------------------

bool EmberCAMNRF::begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi) {
  Serial.print("[THERM-TX] begin() CE=");   Serial.print(cePin);
  Serial.print(" CSN=");                    Serial.print(csnPin);
  Serial.print(" SDA=");                    Serial.print(sdaPin);
  Serial.print(" SCL=");                    Serial.println(sclPin);
  Serial.flush();

  // Passo 1: alocar buffers para o frame bruto e para os dados normalizados
  Serial.println("[THERM-TX] malloc 768*float + 768*uint8...");
  Serial.flush();
  _mlxFrame = (float*)malloc(768 * sizeof(float));
  _thermalData = (uint8_t*)malloc(768);
  if (!_mlxFrame || !_thermalData) {
    Serial.println("[THERM-TX] ERRO: sem memoria!");
    return false;
  }
  memset(_mlxFrame, 0, 768 * sizeof(float));
  memset(_thermalData, 0, 768);
  Serial.println("[THERM-TX] malloc OK.");
  Serial.flush();

  // Passo 2: iniciar I2C e fazer scan de dispositivos
  Serial.print("[THERM-TX] Wire.begin(SDA="); Serial.print(sdaPin);
  Serial.print(", SCL=");                     Serial.print(sclPin);
  Serial.println(") @ 1MHz...");
  Serial.flush();
  _sdaPin = sdaPin;
  _sclPin = sclPin;
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(400000);   // 1MHz era instavel com fios longos

  // I2C scan — descobre que enderecos respondem no bus
  Serial.println("[THERM-TX] I2C scan (0x03..0x77)...");
  Serial.flush();
  uint8_t found = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  [I2C] device em 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);
      Serial.flush();
      found++;
    }
  }
  Serial.print("[THERM-TX] I2C scan: ");
  Serial.print(found);
  Serial.println(" dispositivo(s).");
  if (found == 0) {
    Serial.println("[THERM-TX] AVISO: nenhum dispositivo I2C respondeu.");
    Serial.println("           Verifica: VCC 3.3V no MLX, GND comum,");
    Serial.println("           pull-ups 4.7k em SDA e SCL, ligacoes SDA/SCL.");
  }
  Serial.flush();

  // Passo 3: inicializar o sensor térmico MLX90640
  Serial.print("[THERM-TX] mlx->begin(addr=0x");
  Serial.print(MLX90640_I2CADDR_DEFAULT, HEX);
  Serial.println(")...");
  Serial.flush();
  Adafruit_MLX90640 *mlx = new Adafruit_MLX90640();
  if (!mlx->begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("[THERM-TX] ERRO: MLX90640 nao encontrado!");
    Serial.println("           (mas continua para tentar inicializar NRF)");
    delete mlx;
    mlx = nullptr;
  } else {
    mlx->setMode(MLX90640_CHESS);        // modo Chess: menos artefactos que modo TV
    mlx->setResolution(MLX90640_ADC_18BIT);
    mlx->setRefreshRate(MLX90640_8_HZ);
    _mlx = (void*)mlx;
    Serial.println("[THERM-TX] MLX90640 OK (8Hz, 1MHz I2C).");
    Serial.flush();
  }

  // Passo 4: inicializar o módulo NRF24L01+ para TX unidirecional
  Serial.println("[THERM-TX] pinMode CSN OUTPUT HIGH...");
  Serial.flush();
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  Serial.println("[THERM-TX] new RF24(CE,CSN, 1MHz)...");
  Serial.flush();
  RF24 *radio = new RF24(cePin, csnPin, 1000000);  // 1 MHz para fios longos
  if (!radio) {
    Serial.println("[THERM-TX] ERRO: new RF24 devolveu nullptr (OOM)");
    return false;
  }
  Serial.println("[THERM-TX] RF24 alocado.");
  Serial.flush();

  // Tenta begin() até 3 vezes para tolerar arranque lento do chip
  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-TX] tentativa "); Serial.print(t); Serial.print("/3 begin()... ");
    Serial.flush();
    bool beginOK = radio->begin(&spi);
    Serial.print(beginOK ? "begin=OK " : "begin=FAIL ");
    bool chipOK = radio->isChipConnected();
    Serial.print(chipOK ? "chip=OK" : "chip=FAIL");
    Serial.println();
    Serial.flush();
    if (beginOK && chipOK) {
      _nrfOK = true;
      break;
    }
    delay(100);
  }

  if (_nrfOK) {
    radio->setChannel(2);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_MAX);
    radio->setAutoAck(false);            // sem ACK: fluxo de vídeo unidirecional
    radio->setPayloadSize(25);
    radio->setCRCLength(RF24_CRC_DISABLED); // sem CRC: maior débito, sem retransmissões
    radio->openWritingPipe(THERM_ADDR);
    radio->stopListening();
    _radio = (void*)radio;
    Serial.println("[THERM-TX] ---- printDetails (post-config) ----");
    Serial.flush();
    radio->printDetails();
    Serial.println("[THERM-TX] ------------------------------------");
    Serial.println("[THERM-TX] nRF24 OK: canal=2, 250KBPS, PA_MAX, CRC OFF");
    Serial.flush();
  } else {
    Serial.println("[THERM-TX] *** NRF FALHOU ***");
    Serial.println("[THERM-TX] ---- printDetails (raw) ----");
    Serial.flush();
    radio->printDetails();
    Serial.println("[THERM-TX] -----------------------------");
    Serial.flush();
    delete radio;
  }

  // Se o MLX falhou, devolvemos false para o caller saber
  return _nrfOK && (mlx != nullptr);
}

// -----------------------------------
// Máquina de estados de captura e transmissão
// -----------------------------------

// Maquina de estados nao-bloqueante. Cada chamada faz uma fatia
// pequena de trabalho e devolve, para nao tapar o loop de controlo.
//
// IDLE       -> respeita intervalo de 125ms (~8Hz, alinhado ao MLX).
//               Ao expirar, faz a leitura I2C sincrona (~30-40ms — limitacao
//               da lib Adafruit) + normalizacao, e passa a TX_BURST.
// TX_BURST   -> envia PKTS_PER_TICK pacotes via writeFast() e devolve.
//               No fim do burst chama txStandBy() e volta a IDLE.
//
// Sem delayMicroseconds() entre pacotes: com autoAck=false e CRC off
// o NRF aceita escritas back-to-back ate encher o FIFO (3 slots).
void EmberCAMNRF::update() {
  if (!_nrfOK || !_radio || !_mlx) return;

  Adafruit_MLX90640 *mlx = (Adafruit_MLX90640*)_mlx;
  RF24 *radio = (RF24*)_radio;

  const uint32_t FRAME_INTERVAL_MS = 125;   // 8Hz, alinhado ao MLX
  const uint8_t  PKTS_PER_TICK     = 4;     // 4 x 25B por chamada

  if (_state == 0) {
    // IDLE: aguarda intervalo
    if (millis() - _lastFrameMs < FRAME_INTERVAL_MS) return;

    // Le frame (sincrono — limitacao I2C). Se falhar tenta de novo no
    // proximo tick sem avancar de estado.
    static uint8_t mlxFails = 0;
    if (mlx->getFrame(_mlxFrame) != 0) {
      mlxFails++;
      if (mlxFails >= 15) {
        mlxFails = 0;
        Serial.println("[THERM-TX] MLX getFrame falhou 15x seguidas - a reiniciar I2C...");
        Wire.end();
        delay(20);
        Wire.begin(_sdaPin, _sclPin);
        Wire.setClock(400000);
        mlx->begin(MLX90640_I2CADDR_DEFAULT, &Wire);
        Serial.println("[THERM-TX] MLX re-init feito.");
      }
      return;
    }
    mlxFails = 0;

    // Normaliza as 768 temperaturas float para bytes 0-255 no intervalo
    // TEMP_MIN-TEMP_MAX: resolução de (60-20)/255 = 0.16 °C por bit.
    for (int i = 0; i < 768; i++) {
      float t = constrain(_mlxFrame[i], TEMP_MIN, TEMP_MAX);
      _thermalData[i] = (uint8_t)((t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255.0f);
    }

    _txIndex = 0;
    _lastFrameMs = millis();
    _state = 1;
    return;
  }

  // _state == 1: TX_BURST
  ThermalPacket pkt;
  uint8_t end = _txIndex + PKTS_PER_TICK;
  if (end > 32) end = 32;
  for (uint8_t p = _txIndex; p < end; p++) {
    pkt.index = p;
    memcpy(pkt.data, &_thermalData[p * 24], 24);
    radio->writeFast(&pkt, sizeof(pkt)); // enche o FIFO TX sem aguardar ACK
  }
  _txIndex = end;

  if (_txIndex >= 32) {
    bool standbyOK = radio->txStandBy(95); // aguarda até 95 ms para o FIFO esvaziar
    if (!standbyOK) {
      Serial.println("[THERM-TX] txStandBy falhou - flush TX FIFO");
      radio->flush_tx();
    }
    _state = 0;
    _frameCount++;
    if (_frameCount % 10 == 0) {
      Serial.print("[THERM-TX] Frame #"); Serial.print(_frameCount);
      Serial.print(" | Centro: "); Serial.print(_mlxFrame[400], 1);
      Serial.println(" C");
    }
  }
}
