// ============================================================
// EmberNRFTX, Header (Cabecalho)
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Modulo responsavel por enviar comandos de controlo ao drone
// via radio NRF24L01 (modelo de transcetor sem fios de 2.4 GHz)
// e receber o byte de ACK (Acknowledgement, confirmacao de
// rececao) com o estado dos sensores do drone. Existe no
// projeto para encapsular toda a logica de comunicacao radio
// de controlo, expondo ao ficheiro principal apenas uma
// funcao send() simples e getters (funcoes de acesso) para
// as estatisticas de ligacao.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor EmberNRFTX(cePin, csnPin) e chamado em
//    setup() apos a inicializacao do bus SPI (Serial
//    Peripheral Interface, protocolo de comunicacao serie
//    entre componentes). Inicializa o objeto RF24 e coloca
//    todos os contadores a zero.
// 2. begin(spi) e chamado em setup(): forca o CSN (Chip
//    Select Not) a HIGH, verifica se o chip responde, e
//    configura canal, velocidade, potencia, ACK e tentativas.
// 3. send() e chamado em cada iteracao de loop():
//    Passo 1: preenche o struct PayloadCtrl com os 5 valores.
//    Passo 2: envia os 9 bytes pelo radio via radio.write().
//    Passo 3: se o drone confirmou (ACK), le o byte de retorno
//             com o estado dos sensores.
//    Passo 4: atualiza os contadores de sucesso ou falha.
// 4. Os getters sao chamados em loop() para debug serie.
//
// RADIO
// -----------------------------------------------------------
//   Pinos:
//     CE=14  (Chip Enable, ativa a transmissao)
//     CSN=13 (Chip Select Not, seleciona este radio no bus SPI)
//
//   Canal: 76  (frequencia = 2400 + 76 = 2476 MHz)
//
//   Velocidade: 250 kbps (quilobits por segundo)
//     Velocidade baixa = maior alcance e resistencia a
//     interferencias de outros dispositivos sem fios
//
//   Potencia: PA_LOW (Power Amplifier Low, amplificador de
//     potencia em modo baixo). Evita sobrecarga da fonte de
//     alimentacao de 3.3 Volts partilhada com o radio termico.
//
//   ACK: ativo, com payload de retorno
//     O drone confirma cada pacote e aproveita para enviar
//     o estado dos seus sensores (1 byte) no ACK.
//
//   Retries (tentativas): 15 tentativas, 1500 us entre cada.
//     Se o drone nao confirmar, o radio tenta novamente
//     ate 15 vezes antes de desistir e registar uma falha.
//
//   Endereco: "00001"
//     Identificador do canal de comunicacao. Tem de coincidir
//     com o endereco de rececao configurado no codigo do drone.
//
// PAYLOAD ENVIADO (9 bytes, struct PayloadCtrl)
// -----------------------------------------------------------
//   Payload = conjunto de dados uteis de uma mensagem
//
//   armed:    0=desarmado, 1=armado, 2=emergencia
//   throttle: 1000-1500 microssegundos (protocolo PWM dos ESCs)
//   yaw:      -100 a +100 (rotacao vertical)
//   pitch:    -100 a +100 (inclinacao frente/tras)
//   roll:     -100 a +100 (inclinacao lateral)
//
// ACK RECEBIDO DO DRONE (1 byte)
// -----------------------------------------------------------
//   bits[1:0] = estado do sensor
//     0 = tudo OK
//     1 = CO (monoxido de carbono detetado)
//     2 = Fogo (chama detetada)
//     3 = CO mais Fogo em simultaneo
//
//   bits[4:2] = fase de calibracao (0=normal, 1-7=a calibrar)
// ============================================================

#ifndef EMBER_NRF_TX_H
#define EMBER_NRF_TX_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>


// -----------------------------------
// Constantes do Campo Armed
// -----------------------------------

#define ARMED_OFF       ((uint8_t)0)  // desarmado, ramp down gradual dos motores
#define ARMED_ON        ((uint8_t)1)  // armado, drone aceita comandos de voo
#define ARMED_EMERGENCY ((uint8_t)2)  // emergencia, corte imediato dos ESCs sem rampa


// -----------------------------------
// Estrutura do Payload de Controlo
// -----------------------------------
// struct = estrutura de dados, agrupa varios campos num unico bloco.
// __attribute__((packed)) garante que nao ha bytes de preenchimento
// (padding) entre os campos. Sem isto, o compilador poderia adicionar
// bytes extra para alinhamento de memoria, desalinhando a comunicacao
// entre o controlador e o drone.
// Total: 1 + 2 + 2 + 2 + 2 = 9 bytes

struct __attribute__((packed)) PayloadCtrl {
  uint8_t armed;     // 1 byte: estado do drone (0, 1 ou 2)
  int16_t throttle;  // 2 bytes: forca dos motores em microssegundos (1000-1500)
  int16_t yaw;       // 2 bytes: rotacao vertical (-100 a +100)
  int16_t pitch;     // 2 bytes: inclinacao frente/tras (-100 a +100)
  int16_t roll;      // 2 bytes: inclinacao lateral (-100 a +100)
};

// Trava de seguranca em tempo de compilacao: se o struct alguma vez
// mudar (campo novo, packed removido por engano) so de um lado da
// comunicacao (aqui ou no repositorio do drone), o tamanho deixa de
// ser 9 bytes e o firmware desse lado deixa de compilar, em vez de
// corromper dados silenciosamente em voo (ver docs/DRONE-PAYLOAD-BUG.md).
static_assert(sizeof(PayloadCtrl) == 9, "PayloadCtrl tem de ter exatamente 9 bytes (packed)");


// -----------------------------------
// Classe EmberNRFTX
// -----------------------------------

class EmberNRFTX {
public:
  EmberNRFTX(uint8_t cePin, uint8_t csnPin);

  // Inicializa o radio: configura canal, velocidade, potencia, ACK e tentativas.
  bool begin(SPIClass &spi);

  // Empacota os 5 valores num PayloadCtrl e envia por radio.
  // Devolve true se o drone confirmou a rececao (ACK recebido).
  bool send(uint8_t armed, int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll);

  bool     isOK();                  // true se o radio esta inicializado e operacional
  uint8_t  getLastSensorState();   // devolve bits[1:0] do ultimo ACK: estado do sensor
  uint8_t  getLastAckByte();       // devolve o byte ACK completo (sensor mais fase calibracao)
  uint16_t getConsecutiveFails();  // numero de transmissoes consecutivas sem ACK
  uint32_t getTotalSent();         // total de transmissoes tentadas desde o arranque
  uint32_t getTotalOk();           // total de transmissoes confirmadas com sucesso

private:
  RF24     _radio;            // objeto do radio NRF24L01 (da biblioteca RF24)
  uint8_t  _csnPin;           // pino CSN guardado para forcar HIGH na inicializacao
  bool     _nrfOK;            // true se begin() inicializou o radio com sucesso
  uint8_t  _lastAckByte;      // ultimo byte ACK recebido do drone
  uint8_t  _lastSensor;       // bits[1:0] do ACK, estado dos sensores
  uint16_t _consecutiveFails; // contador de falhas consecutivas (maximo 65535)
  uint32_t _totalSent;        // contador total de envios
  uint32_t _totalOk;          // contador total de envios com ACK confirmado

  // Endereco do pipe (canal) de comunicacao.
  // Tem de ser igual ao endereco de rececao configurado no drone.
  static const uint8_t ADDRESS[6];
};

#endif
