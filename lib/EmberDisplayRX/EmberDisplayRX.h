// ============================================================
// EmberDisplayRX, Header (Cabecalho)
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Modulo que combina recetor de radio e renderizador de ecra
// para visualizacao da imagem termica do drone em tempo real.
// Recebe frames (imagens) da camara termica via NRF24L01
// (modelo de transcetor sem fios de 2.4 GHz) no canal 2,
// interpola bilinealmente cada frame de 32x24 para 480x312
// pixels, e desenha-a no ecra TFT com um mapa de calor
// (heatmap). Existe no projeto para separar toda a logica
// de recepcao termica e renderizacao do loop() principal,
// que apenas precisa de chamar update() em cada iteracao.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor EmberDisplayRX(cePin, csnPin) e chamado
//    em setup() apos o SPI (Serial Peripheral Interface,
//    protocolo de comunicacao serie entre componentes) estar
//    inicializado. Inicializa o objeto RF24 e coloca todos
//    os atributos com valores seguros.
// 2. begin(spi, tft) e chamado uma vez em setup():
//    Passo 1: guarda o ponteiro para o ecra TFT.
//    Passo 2: forca o pino CSN (Chip Select Not) a HIGH.
//    Passo 3: tenta inicializar o radio ate 3 vezes.
//    Passo 4: configura canal 2, 250 kbps, PA_LOW (Power
//             Amplifier Low, potencia baixa), sem ACK
//             (Acknowledgement, confirmacao), payload de
//             25 bytes, CRC (Cyclic Redundancy Check,
//             verificacao ciclica de redundancia) desativado,
//             openReadingPipe e startListening.
//    Passo 5: chama buildHeatmapLUT(), limpa os buffers.
// 3. update() e chamado em cada iteracao de loop():
//    Passo 1: chama receivePackets() para ler o radio.
//    Passo 2: a cada 500ms chama updateConnectionState().
//    Passo 3: se connState=0, chama updateConnectionBlink().
//    Passo 4: avalia 3 criterios de decisao de desenho.
//    Passo 5: se deve desenhar, chama drawThermalSmooth()
//             e regista estatisticas no serial.
//
// FRAME TERMICA (Imagem da Camara)
// -----------------------------------------------------------
//   Resolucao da camara: 32x24 pixels = 768 bytes por frame.
//   Cada pixel = 1 byte (valor de 0 a 255, representa temperatura).
//
//   Como o NRF24L01 so envia 32 bytes por pacote, cada frame
//   e dividida em 32 pacotes de 25 bytes:
//     1 byte de indice (numero do pacote, 0 a 31)
//     24 bytes de dados termicos
//   32 x 24 = 768 bytes = 1 frame completa
//
// RADIO
// -----------------------------------------------------------
//   Pinos:
//     CE=12  (Chip Enable, ativa a rececao)
//     CSN=11 (Chip Select Not, seleciona este radio no bus SPI)
//   Canal: 2  (2402 MHz, diferente do TX de controlo que usa 76)
//   Velocidade: 250 kbps (quilobits por segundo)
//   Potencia: PA_LOW (potencia baixa, reduz consumo de corrente)
//   ACK: desativado (fire-and-forget, sem confirmacao)
//   CRC: desativado (sacrifica verificacao em favor de velocidade)
//   Endereco: "THERM"
//
// ESCALA NO ECRA
// -----------------------------------------------------------
//   Cada pixel termico ocupa 15x13 pixels no ecra:
//     32 x 15 = 480 px (largura total do ecra)
//     24 x 13 = 312 px (altura, deixa 8px em baixo para texto)
//   OFFSET_Y = 4 px de margem no topo
//
// HEATMAP (Mapa de Calor)
// -----------------------------------------------------------
//   256 cores pre-calculadas numa LUT (Look-Up Table, tabela
//   de consulta). Em vez de calcular a cor de cada pixel em
//   tempo real, consulta a tabela por indice (mais rapido).
//   RGB565 = formato de cor de 16 bits do ecra (5 bits vermelho,
//            6 bits verde, 5 bits azul)
//
//   Indice   0: azul escuro (frio, aprox. 20 graus)
//   Indice 255: vermelho    (quente, aprox. 60 graus)
//
// ESTADOS DE LIGACAO
// -----------------------------------------------------------
//   0: sem sinal (mais de 1.5 segundos sem pacotes)
//   1: ligacao fraca (menos de 24 pacotes por frame recebidos)
//   2: ligacao boa   (24 ou mais pacotes por frame recebidos)
// ============================================================

#ifndef EMBER_DISPLAY_RX_H
#define EMBER_DISPLAY_RX_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <TFT_eSPI.h>


// -----------------------------------
// Classe EmberDisplayRX
// -----------------------------------

class EmberDisplayRX {
public:
  EmberDisplayRX(uint8_t cePin, uint8_t csnPin);

  // Inicializa o radio recetor e constroi a tabela de cores heatmap.
  // Recebe o objeto tft por referencia para poder desenhar no ecra.
  bool begin(SPIClass &spi, TFT_eSPI &tft);

  // Deve ser chamado a cada iteracao do loop().
  // Recebe pacotes disponiveis e desenha quando tem dados suficientes.
  void update();

  // Getters para o loop() incluir as estatisticas na linha de debug.
  uint32_t getFramesDrawn()  { return framesDrawn; }
  uint32_t getLastPackets()  { return lastPacketCount; }
  uint8_t  getConnState()    { return connState; }
  uint32_t getLastDrawMs()   { return _lastDrawMs; }
  uint32_t getLastFPS()      { return _lastFPS; }

private:

  // -----------------------------------
  // Radio e Ecra
  // -----------------------------------

  RF24      _radio;   // objeto do radio NRF24L01 (biblioteca RF24)
  TFT_eSPI *_tft;     // ponteiro para o ecra (passado no begin())
  uint8_t   _csnPin;  // pino CSN guardado para forcar HIGH na inicializacao
  bool      _nrfOK;   // false se o radio falhou na inicializacao


  // -----------------------------------
  // Buffers da Frame Termica
  // -----------------------------------

  uint8_t  thermalData[768];     // buffer completo da frame: 32x24 bytes
  bool     packetReceived[32];   // registo de quais dos 32 pacotes ja chegaram
  uint8_t  packetsGot;           // contador de pacotes recebidos nesta frame
  uint32_t frameStartMs;         // timestamp em ms do primeiro pacote da frame
  uint32_t framesDrawn;          // total de frames desenhadas desde o arranque
  uint32_t lastFrameTime;        // timestamp da ultima frame desenhada (para FPS)
                                 // FPS = Frames Per Second, imagens por segundo


  // -----------------------------------
  // Estado de Ligacao
  // -----------------------------------

  uint8_t  lastPacketCount;   // pacotes recebidos na ultima frame (para avaliar qualidade)
  uint32_t lastPacketTime;    // timestamp do ultimo pacote recebido
  uint8_t  connState;         // estado atual: 0=sem sinal, 1=fraca, 2=boa
  uint8_t  prevConnState;     // estado anterior (para detetar mudancas)


  // -----------------------------------
  // Buffers de Renderizacao
  // -----------------------------------

  uint16_t bandBuffer[480 * 13];  // faixa horizontal de 480x13 pixels ja em RGB565
  uint16_t heatmapLUT[256];       // LUT (Look-Up Table) de 256 cores pre-calculadas


  // -----------------------------------
  // Estrutura do Pacote Termico
  // -----------------------------------

  struct ThermalPacket {
    uint8_t index;    // numero do pacote na sequencia (0 a 31)
    uint8_t data[24]; // 24 bytes de dados termicos deste pacote
  };


  // -----------------------------------
  // Estado do Pisca "CONNECTION PERDIDA"
  // -----------------------------------

  bool     _blinkVisible;  // se o texto esta visivel neste momento
  uint32_t _blinkTimer;    // timestamp do ultimo toggle (alternancia) do texto
  uint32_t _lastDrawMs;    // tempo que o ultimo drawThermalSmooth() demorou em ms
  uint32_t _lastFPS;       // FPS (imagens por segundo) da ultima frame desenhada


  // -----------------------------------
  // Metodos Privados
  // -----------------------------------

  void buildHeatmapLUT();                      // pre-calcula as 256 cores do gradiente
  void receivePackets();                       // le todos os pacotes disponiveis no radio
  void buildBandInterpolated(int row);         // interpola uma faixa horizontal (suavizacao)
  void drawThermalSmooth();                    // desenha a frame completa no ecra
  void drawConnectionIndicator(uint8_t state); // atualiza o indicador de ligacao
  void updateConnectionState();                // verifica se o estado de ligacao mudou
  void updateConnectionBlink();               // faz piscar "CONNECTION PERDIDA" sem sinal
  void resetFrame();                           // limpa os buffers para receber nova frame

  static const uint8_t THERM_ADDR[6];  // endereco do pipe de rececao ("THERM")
};

#endif
