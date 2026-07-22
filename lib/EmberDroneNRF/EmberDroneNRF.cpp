// ============================================================
// EmberDroneNRF.cpp, Implementação
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementa a classe EmberDroneNRF, que gere a receção de
// comandos de controlo via NRF24L01+ no drone. O módulo rádio
// opera em modo de escuta contínua no canal 76 a 250 kbps
// (quilobits por segundo), recebendo estruturas PayloadCtrl
// de 9 bytes enviadas pelo controlador do Tiago.
// Em troca, envia automaticamente um byte ACK (Acknowledgement,
// confirmação de receção) com o estado dos sensores e fase de
// calibração, pré-carregado em updateAckPayload() antes de
// cada receive().
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Passo 1: configura CSN (Chip Select Not, seleciona o
//             dispositivo no bus SPI (Serial Peripheral Interface,
//             protocolo de comunicação série)) como OUTPUT HIGH,
//             isolando o chip antes de qualquer transferência.
//    Passo 2: aloca RF24 no heap a 1 MHz (SPI lento para fios
//             longos, com maior imunidade a ruído).
//    Passo 3: chama _radio->begin() e isChipConnected() para
//             confirmar que o chip responde fisicamente.
//    Passo 4: configura canal 76, 250 kbps, PA (Power Amplifier,
//             amplificador de potência) MAX, ACK automático com
//             payload de retorno de 1 byte, ARD (Auto Retry Delay,
//             atraso entre retentativas automáticas) de 500 us e
//             ARC (Auto Retry Count, número de retentativas
//             automáticas) de 3. Descarrega os registos para
//             diagnóstico via printDetails().
//    Passo 5: abre o pipe (canal de comunicação identificado
//             por um endereço no rádio) 1 no endereço "00001",
//             inicia escuta e pré-carrega ACK com byte zero.
// 2. updateAckPayload() antes de receive() em cada loop():
//    Substitui o ACK pendente pelo byte de estado mais recente.
// 3. receive() em cada loop():
//    Drena todo o FIFO (3 slots) ficando com o pacote mais
//    recente. Regista _lastRxTime para o timeout de 1 s.
//
// RÁDIO
// -----------------------------------------------------------
// Módulo:    NRF24L01+ com antena PCB
// Canal:     76 (2476 MHz, afastado do Wi-Fi de 2.4 GHz)
// Débito:    250 kbps
// Potência:  PA_MAX
// ACK:       automático com payload de retorno de 1 byte
// ARD:       500 us, ARC: 3 (1.5 ms worst case)
// Pipe RX:   1, endereço "00001"
// SPI:       1 MHz (velocidade reduzida para fios longos)
// ============================================================

#include "EmberDroneNRF.h"

// -----------------------------------
// Endereço do pipe de rádio
// -----------------------------------

const uint8_t EmberDroneNRF::ADDRESS[6] = "00001";

// -----------------------------------
// Inicialização
// -----------------------------------

bool EmberDroneNRF::begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi) {
  Serial.print("[CTRL-RX] begin() CE=");
  Serial.print(cePin);
  Serial.print(" CSN=");
  Serial.println(csnPin);
  Serial.flush();

  Serial.println("[CTRL-RX] pinMode CSN OUTPUT HIGH...");
  Serial.flush();
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  Serial.println("[CTRL-RX] new RF24(CE,CSN, 1MHz)...");
  Serial.flush();
  _radio = new RF24(cePin, csnPin, 1000000);  // 1 MHz para fios longos
  if (!_radio) {
    Serial.println("[CTRL-RX] ERRO: new RF24 devolveu nullptr (OOM)");
    return false;
  }
  Serial.println("[CTRL-RX] RF24 alocado.");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->begin(&spi)...");
  Serial.flush();
  bool beginOK = _radio->begin(&spi);
  Serial.print("[CTRL-RX] begin() returned: ");
  Serial.println(beginOK ? "true" : "false");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->isChipConnected()...");
  Serial.flush();
  bool chipOK = _radio->isChipConnected();
  Serial.print("[CTRL-RX] isChipConnected: ");
  Serial.println(chipOK ? "true" : "false");
  Serial.flush();

  if (!beginOK || !chipOK) {
    Serial.println("[CTRL-RX] ERRO: nRF24 nao iniciou!");
    Serial.println("[CTRL-RX] Verifica: alimentacao 3.3V (com cap 10uF),");
    Serial.println("           SPI (SCK/MISO/MOSI), CE, CSN, GND comum.");
    // Dump dos registos para diagnostico (mostra address de RX_PIPE etc.)
    Serial.println("[CTRL-RX] ---- printDetails (raw) ----");
    Serial.flush();
    _radio->printDetails();
    Serial.println("[CTRL-RX] -----------------------------");
    Serial.flush();
    return false;
  }

  Serial.println("[CTRL-RX] config: ch=76, 250KBPS, PA_LOW, autoAck, ackPayload...");
  Serial.flush();
  _radio->setChannel(76);
  _radio->setDataRate(RF24_250KBPS);
  _radio->setPALevel(RF24_PA_MAX);
  _radio->setAutoAck(true);
  _radio->setRetries(1, 3);   // 500us x 3 = 1.5ms worst case (igual ao TX)
  _radio->enableAckPayload();
  _radio->openReadingPipe(1, ADDRESS);
  _radio->startListening();

  uint8_t zero = 0;
  _radio->writeAckPayload(1, &zero, 1);

  // Diagnostico final: confirma o que ficou nos registos
  Serial.println("[CTRL-RX] ---- printDetails (post-config) ----");
  Serial.flush();
  _radio->printDetails();
  Serial.println("[CTRL-RX] ------------------------------------");
  Serial.flush();

  _lastRxTime = millis();
  _initialized = true;
  Serial.println("[CTRL-RX] nRF24 OK: canal=76, 250KBPS, ACK");
  Serial.flush();
  return true;
}

// -----------------------------------
// Estado e receção de pacotes
// -----------------------------------

bool EmberDroneNRF::isOK() const { return _initialized; }

bool EmberDroneNRF::receive(PayloadCtrl &p) {
  if (!_initialized) return false;
  if (_radio->available()) {
    while (_radio->available()) _radio->read(&p, sizeof(p)); // drena o FIFO; fica com o pacote mais recente
    _lastRxTime = millis();
    return true;
  }
  return false;
}

// -----------------------------------
// ACK de retorno e timestamp
// -----------------------------------

void EmberDroneNRF::updateAckPayload(uint8_t estado) {
  if (!_initialized) return;
  _radio->writeAckPayload(1, &estado, 1);
}

unsigned long EmberDroneNRF::lastReceiveTime() {
  if (!_initialized) return millis();  // evita timeout imediato no loop
  return _lastRxTime;
}
