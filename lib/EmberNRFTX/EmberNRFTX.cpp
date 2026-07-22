// ============================================================
// EmberNRFTX, Implementacao
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementacao do transmissor de controlo NRF24L01 (modelo
// de transcetor sem fios de 2.4 GHz). Gere o ciclo completo
// de cada transmissao: empacotar os valores de controlo num
// struct (estrutura de dados), enviar por radio, ler o byte
// de ACK (Acknowledgement, confirmacao de rececao) com o
// estado dos sensores do drone, e atualizar os contadores
// de estatisticas. Existe no projeto para separar a logica
// de radio do ficheiro principal, facilitando a manutencao
// e os testes de comunicacao.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor inicializa o objeto RF24 com os pinos CE
//    (Chip Enable, ativa a transmissao) e CSN (Chip Select
//    Not, seleciona este radio no bus SPI), e coloca todos
//    os contadores a zero via lista de inicializacao C++.
// 2. begin(spi) e chamado uma vez em setup():
//    Passo 1: forca o pino CSN a HIGH para isolar o radio
//             no bus SPI (Serial Peripheral Interface,
//             protocolo de comunicacao serie entre componentes)
//             partilhado com o outro radio e o ecra.
//    Passo 2: chama radio.begin() e verifica isChipConnected().
//    Passo 3: configura canal 76, velocidade 250 kbps
//             (quilobits por segundo), PA_LOW (Power Amplifier
//             Low, amplificador de potencia em modo baixo),
//             ACK automatico, 15 tentativas, enableAckPayload,
//             openWritingPipe e stopListening.
// 3. send() e chamado em cada iteracao de loop():
//    Passo 1: preenche o struct PayloadCtrl de 9 bytes.
//    Passo 2: envia pelo radio com radio.write().
//    Passo 3: se confirmado, le o byte ACK com radio.read().
//    Passo 4: atualiza _totalSent, _totalOk e _consecutiveFails.
// 4. Os getters devolvem os atributos privados ao loop().
//
// TX = Transmitter (transmissor)
// SPI = Serial Peripheral Interface (protocolo de comunicacao)
// ============================================================

#include "EmberNRFTX.h"


// -----------------------------------
// Endereco do Pipe de Comunicacao
// -----------------------------------
// Identifica o canal de comunicacao entre o controlador e o drone.
// Tem de corresponder ao endereco de rececao configurado no drone.

const uint8_t EmberNRFTX::ADDRESS[6] = "00001";


// -----------------------------------
// Construtor
// -----------------------------------
// Inicializa o objeto RF24 com os pinos CE e CSN,
// e coloca todos os contadores a zero.
// O : (lista de inicializacao) define os atributos antes do corpo.

EmberNRFTX::EmberNRFTX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _csnPin(csnPin), _nrfOK(false),
    _lastAckByte(0), _lastSensor(0), _consecutiveFails(0), _totalSent(0), _totalOk(0) {}


// -----------------------------------
// Begin (Inicializacao do Radio)
// -----------------------------------
// Passo 1: forca o pino CSN para HIGH antes de inicializar.
//          Evita conflitos no bus SPI partilhado com o outro radio.
// Passo 2: verifica se o radio responde.
// Passo 3: configura todos os parametros de comunicacao.

bool EmberNRFTX::begin(SPIClass &spi) {

  // Passo 1: CSN a HIGH para isolar este radio durante a inicializacao
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  // Passo 2: tenta inicializar o radio ate 3 vezes (absorve glitch de SPI
  // no arranque ou POR lento do chip). Reutiliza o mesmo objeto _radio
  // em cada tentativa; nao ha alocacao extra.
  bool ok = false;
  for (int t = 1; t <= 3; t++) {
    bool beginOK = _radio.begin(&spi);
    bool chipOK  = _radio.isChipConnected();
    Serial.print("[NRF-TX] begin tentativa "); Serial.print(t);
    Serial.println((beginOK && chipOK) ? " OK" : " FAIL");
    if (beginOK && chipOK) { ok = true; break; }
    delay(100);
  }
  if (!ok) {
    Serial.println("[NRF-TX] ERRO: NRF24L01 nao iniciou apos 3 tentativas!");
    _nrfOK = false;
    return false;
  }

  // Passo 3: configura os parametros de comunicacao
  _radio.setChannel(76);
  // Canal 76 = frequencia 2476 MHz (2400 + 76)
  // Separado do radio termico que usa o canal 2 (2402 MHz)

  _radio.setDataRate(RF24_250KBPS);
  // 250 kbps (quilobits por segundo): velocidade baixa para maior alcance
  // e melhor resistencia a interferencias de outros dispositivos sem fios

  _radio.setPALevel(RF24_PA_LOW);
  // PA = Power Amplifier (amplificador de potencia)
  // PA_LOW evita que a corrente elevada do radio em potencia alta
  // faca cair a tensao da fonte de 3.3V, que e partilhada com o
  // radio termico e outros componentes

  _radio.setAutoAck(true);
  // Ativa o ACK automatico: o drone confirma cada pacote recebido
  // sem que o codigo do drone precise de fazer nada explicitamente

  _radio.setRetries(5, 15);
  // Parametro 1: ARD = Auto Retry Delay = 5 x 250us = 1500us
  //   (tempo minimo necessario para 250kbps com payload no ACK)
  // Parametro 2: ARC = Auto Retry Count = 15 tentativas antes de desistir

  _radio.enableAckPayload();
  // Permite que o drone inclua dados (1 byte com estado dos sensores)
  // dentro do byte de confirmacao (ACK)

  _radio.openWritingPipe(ADDRESS);
  // Configura o endereco de destino para onde os pacotes sao enviados

  _radio.stopListening();
  // Coloca o radio em modo transmissor (o oposto seria startListening)

  Serial.println("[NRF-TX] NRF24L01 OK: canal=76, 250KBPS, PA_LOW, ACK, 15 retentativas");
  _nrfOK = true;
  return true;
}


// -----------------------------------
// Envio do Payload de Controlo
// -----------------------------------
// Passo 1: preenche o struct PayloadCtrl com os valores actuais
// Passo 2: envia os 9 bytes pelo radio
// Passo 3: se confirmado, le o byte ACK com estado dos sensores
// Passo 4: atualiza os contadores de estatisticas

bool EmberNRFTX::send(uint8_t armed, int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll) {

  if (!_nrfOK) return false;  // radio nao inicializado, nao tenta enviar

  // Passo 1: preenche a estrutura de 9 bytes
  PayloadCtrl p = { armed, throttle, yaw, pitch, roll };

  // Passo 2: envia pelo radio. A biblioteca RF24 gere automaticamente
  // as retentativas se nao houver ACK do drone.
  bool ok = _radio.write(&p, sizeof(p));

  // Passos 3 e 4: atualiza contadores e le ACK se disponivel
  _totalSent++;

  if (ok) {
    _totalOk++;
    _consecutiveFails = 0;  // reset do contador, confirmacao recebida

    // Se o drone incluiu dados no ACK, le 1 byte de retorno.
    // Bits[1:0] = estado do sensor, bits[4:2] = fase de calibracao.
    if (_radio.available()) {
      uint8_t v;
      _radio.read(&v, 1);
      _lastAckByte = v;
      _lastSensor  = v & 0x03;  // & 0x03 = mascara, extrai apenas os 2 bits mais baixos
    }
  } else {
    // Limita o contador a 65535 para nao transbordar o tipo uint16_t
    // (uint16_t = inteiro sem sinal de 16 bits, maximo = 65535)
    if (_consecutiveFails < 65535) _consecutiveFails++;
  }

  return ok;
}


// -----------------------------------
// Getters (Funcoes de Acesso)
// -----------------------------------
// Permitem ao ficheiro principal aceder aos dados internos
// do radio sem expor os atributos privados diretamente.

bool     EmberNRFTX::isOK()                { return _nrfOK; }
uint8_t  EmberNRFTX::getLastSensorState()  { return _lastSensor; }
uint8_t  EmberNRFTX::getLastAckByte()      { return _lastAckByte; }
uint16_t EmberNRFTX::getConsecutiveFails()  { return _consecutiveFails; }
uint32_t EmberNRFTX::getTotalSent()         { return _totalSent; }
uint32_t EmberNRFTX::getTotalOk()           { return _totalOk; }
