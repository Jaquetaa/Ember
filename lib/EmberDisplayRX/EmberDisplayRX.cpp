// ============================================================
// EmberDisplayRX, Implementacao
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementacao do recetor de frames (imagens) termicas e
// da renderizacao no ecra LCD. Recebe pacotes do NRF24L01
// (modelo de transcetor sem fios de 2.4 GHz), reconstroi
// a frame de 32x24 pixels, interpola bilinealmente para
// 480x312 pixels e desenha faixa a faixa com um heatmap
// (mapa de calor) de 256 cores. Existe no projeto para
// isolar toda a logica de imagem termica, permitindo que
// o loop() principal apenas chame update() sem gerir
// buffers, temporizadores ou renderizacao diretamente.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor inicializa o objeto RF24 com CE (Chip Enable,
//    ativa a rececao) e CSN (Chip Select Not, seleciona o radio
//    no bus SPI, Serial Peripheral Interface, protocolo de
//    comunicacao serie entre componentes) e coloca todos os
//    atributos com valores seguros via lista de inicializacao.
// 2. begin(spi, tft) e chamado uma vez em setup():
//    Passo 1: guarda o ponteiro para o ecra TFT.
//    Passo 2: forca o pino CSN a HIGH para isolar o radio.
//    Passo 3: tenta inicializar o radio ate 3 vezes.
//    Passo 4: configura canal 2, 250 kbps (quilobits por segundo),
//             PA_HIGH (Power Amplifier High, potencia alta),
//             sem ACK (Acknowledgement, confirmacao), payload
//             de 25 bytes, CRC (Cyclic Redundancy Check,
//             verificacao ciclica de redundancia) desativado,
//             openReadingPipe e startListening.
//    Passo 5: chama buildHeatmapLUT() e limpa os buffers.
// 3. update() e chamado em cada iteracao de loop():
//    Passo 1: receivePackets() le todos os pacotes do radio.
//    Passo 2: a cada 500ms, updateConnectionState() avalia
//             o estado da ligacao (0=sem sinal, 1=fraca, 2=boa).
//    Passo 3: se connState=0, updateConnectionBlink() pisca
//             "CONNECTION PERDIDA" no ecra a cada 1 segundo.
//    Passo 4: avalia 3 criterios de decisao de desenho.
//    Passo 5: se deve desenhar, drawThermalSmooth() constroi
//             e envia as 24 faixas ao ecra, depois resetFrame().
//
// INTERPOLACAO BILINEAR (Suavizacao da Imagem)
// -----------------------------------------------------------
//   Em vez de esticar cada pixel termico num quadrado solido
//   (resultado pixelizado), calcula uma media ponderada dos
//   4 pixels termicos vizinhos para cada pixel do ecra.
//   Usa aritmetica inteira (sem floats, numeros com virgula)
//   porque e mais rapido no ESP32.
//
// DECISAO DE DESENHO (3 criterios por prioridade)
// -----------------------------------------------------------
//   1. Frame completa (32/32 pacotes recebidos): qualidade maxima
//   2. 10 ou mais pacotes apos 100ms (timeout): boa qualidade
//   3. Qualquer pacote apos 250ms (timeout longo): sinal fraco,
//      mas mostra o que ha para nao ficar o ecra vazio
//
// HEATMAP LUT (Look-Up Table, Tabela de Consulta de Cores)
// -----------------------------------------------------------
//   Pre-calculada uma vez no arranque em buildHeatmapLUT().
//   256 cores em formato RGB565 (16 bits por pixel) do ecra:
//     RGB565 = 5 bits vermelho + 6 bits verde + 5 bits azul
//   Byte-swap (troca de bytes) necessario porque o pushImage
//   envia em little-endian (byte menos significativo primeiro)
//   mas o display espera big-endian (byte mais significativo).
//     Indices   0-127: azul escuro para roxo  (frio)
//     Indices 128-255: roxo para vermelho     (quente)
// ============================================================

#include "EmberDisplayRX.h"


// -----------------------------------
// Constantes de Dimensao e Escala
// -----------------------------------

#define DISP_W    480   // largura do ecra em pixels
#define DISP_H    320   // altura do ecra em pixels
#define THERM_W   32    // largura da imagem termica em pixels
#define THERM_H   24    // altura da imagem termica em pixels
#define SCALE_X   15    // 32 x 15 = 480, cada pixel termico ocupa 15 pixels horizontais
#define SCALE_Y   13    // 24 x 13 = 312, cada pixel termico ocupa 13 pixels verticais
#define OFFSET_Y  4     // margem de 4 pixels no topo do ecra
#define BAND_W    480   // largura de cada faixa de renderizacao (igual a largura do ecra)
#define BAND_H    SCALE_Y  // altura de cada faixa = 13 pixels
#define FRAME_TIMEOUT_MS      100   // ms (milissegundos) de espera maxima por pacotes
#define CONN_GOOD_THRESHOLD   24    // minimo de pacotes por frame para ligacao boa
#define CONN_WEAK_THRESHOLD   10    // minimo de pacotes por frame para ligacao fraca
#define COL_WHITE   0xFFFF  // branco em formato RGB565 (todos os bits a 1)
#define COL_OUTLINE 0x8410  // cinzento em RGB565
#define COL_BLACK   0x0000  // preto em RGB565 (todos os bits a 0)


// -----------------------------------
// Endereco do Pipe de Rececao
// -----------------------------------
// Identifica o canal de rececao. Tem de corresponder ao endereco
// de envio configurado no codigo do drone (Goncalo).

const uint8_t EmberDisplayRX::THERM_ADDR[6] = "THERM";


// -----------------------------------
// Construtor
// -----------------------------------
// Inicializa todos os atributos com valores seguros antes de
// qualquer hardware ser configurado.

EmberDisplayRX::EmberDisplayRX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _tft(nullptr), _csnPin(csnPin), _nrfOK(false),
    packetsGot(0), frameStartMs(0), framesDrawn(0), lastFrameTime(0),
    lastPacketCount(0), lastPacketTime(0), connState(0), prevConnState(255),
    _blinkVisible(false), _blinkTimer(0), _lastDrawMs(0), _lastFPS(0) {}


// -----------------------------------
// Begin (Inicializacao)
// -----------------------------------
// Passo 1: guarda a referencia para o ecra TFT
// Passo 2: forca o CSN a HIGH para isolar este radio no bus SPI
// Passo 3: tenta inicializar o radio ate 3 vezes
// Passo 4: se o radio responder, configura todos os parametros
// Passo 5: pre-calcula a tabela de cores e limpa os buffers

bool EmberDisplayRX::begin(SPIClass &spi, TFT_eSPI &tft) {

  // Passo 1
  _tft = &tft;

  // Passo 2: CSN a HIGH para isolar este radio no bus SPI partilhado
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  // Passo 3: ate 3 tentativas porque o NRF24L01 pode precisar
  // de uma segunda tentativa a arrancar em alguns casos
  for (int t = 1; t <= 3; t++) {
    Serial.print("[DISPLAY-RX] NRF24L01 tentativa "); Serial.print(t); Serial.print("/3 ... ");
    if (_radio.begin(&spi) && _radio.isChipConnected()) {
      Serial.println("OK!");
      _nrfOK = true;
      break;
    }
    Serial.println("FALHOU");
    delay(200);
  }

  // Passo 4: configura os parametros de rececao
  if (_nrfOK) {
    _radio.setChannel(2);
    // Canal 2 = frequencia 2402 MHz, diferente do TX de controlo (canal 76)

    _radio.setDataRate(RF24_250KBPS);
    // 250 kbps (quilobits por segundo): velocidade baixa para maior alcance

    _radio.setPALevel(RF24_PA_LOW);
    // PA_LOW: potencia baixa para reduzir consumo de corrente

    _radio.setAutoAck(false);
    // Sem ACK: fire-and-forget
    // Perder alguns frames e aceitavel para imagem ao vivo

    _radio.setPayloadSize(25);
    // Tamanho fixo de cada pacote: 1 byte indice + 24 bytes dados

    _radio.setCRCLength(RF24_CRC_DISABLED);
    // CRC = Cyclic Redundancy Check (verificacao ciclica de redundancia)
    // Desativado para ganhar velocidade, sacrificando verificacao de erros

    _radio.openReadingPipe(1, THERM_ADDR);
    // Abre o pipe (canal) de rececao com o endereco "THERM"

    _radio.startListening();
    // Coloca o radio em modo recetor (o oposto do TX que usa stopListening)

    Serial.println("[DISPLAY-RX] NRF24L01 OK: canal=2, 250KBPS, PA_LOW, CRC desativado");
  } else {
    Serial.println("[DISPLAY-RX] *** NRF24L01 FALHOU ***");
  }

  // Passo 5: prepara os buffers
  buildHeatmapLUT();           // pre-calcula as 256 cores
  memset(thermalData, 0, 768); // limpa o buffer da imagem (preenche com zeros)
  resetFrame();
  lastPacketTime = millis();

  return _nrfOK;
}


// -----------------------------------
// Heatmap LUT (Tabela de Consulta de Cores)
// -----------------------------------
// Pre-calcula 256 cores uma vez no arranque.
// Muito mais rapido do que calcular a cor de cada pixel durante o desenho.
//
// A paleta divide-se em dois segmentos de 128 cores:
//   Indices   0-127: azul escuro a roxo  (temperaturas frias)
//   Indices 128-255: roxo a vermelho     (temperaturas quentes)
//
// RGB565 = formato de cor de 16 bits: 5 bits vermelho, 6 bits verde, 5 bits azul
// Byte-swap: inverte a ordem dos dois bytes porque o pushImage envia em
// little-endian mas o display espera big-endian

void EmberDisplayRX::buildHeatmapLUT() {
  for (int i = 0; i < 256; i++) {
    int r, g = 0, b;
    if (i < 128) {
      int t = i;
      r = (t * 255) / 127;         // vermelho sobe de 0 a 255
      g = 0;
      b = 60 + (t * 195) / 127;   // azul sobe de 60 a 255 (azul escuro para roxo)
    } else {
      int t = i - 128;
      r = 255;                      // vermelho fixo no maximo
      g = 0;
      b = 255 - (t * 255) / 127;  // azul desce de 255 a 0 (roxo para vermelho)
    }
    r = constrain(r, 0, 255);  // garante que o valor nao sai do intervalo 0-255
    b = constrain(b, 0, 255);
    // converte RGB888 (8 bits por canal) para RGB565 (16 bits total)
    uint16_t px = (((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | ((uint16_t)b >> 3);
    heatmapLUT[i] = (px >> 8) | (px << 8);  // byte-swap para o display
  }
}


// -----------------------------------
// Rececao de Pacotes
// -----------------------------------
// Le todos os pacotes disponiveis no buffer do radio.
// Para cada pacote recebido:
//   Passo 1: le o pacote de 25 bytes
//   Passo 2: valida o indice (0-31)
//   Passo 3: copia os 24 bytes de dados para a posicao correta no buffer
//   Passo 4: marca o pacote como recebido (evita duplicados)

void EmberDisplayRX::receivePackets() {
  while (_radio.available()) {
    ThermalPacket pkt;
    _radio.read(&pkt, sizeof(pkt));  // le 25 bytes do buffer do radio
    if (pkt.index < 32) {
      // Passo 3: posicao no buffer = indice x 24 (cada pacote tem 24 bytes de dados)
      memcpy(&thermalData[pkt.index * 24], pkt.data, 24);
      // Passo 4: se for a primeira vez que este pacote chega, conta-o
      if (!packetReceived[pkt.index]) {
        packetReceived[pkt.index] = true;
        packetsGot++;
      }
      if (packetsGot == 1) frameStartMs = millis();  // marca inicio da frame
      lastPacketTime = millis();
    }
  }
}


// -----------------------------------
// Interpolacao Bilinear de uma Faixa
// -----------------------------------
// Constroi uma faixa horizontal de 480x13 pixels a partir de
// duas linhas termicas consecutivas (row e row+1).
//
// Para cada pixel do ecra (sx, py) calcula:
//   Passo 1: posicao fracionaria no espaco termico (ponto fixo x256)
//   Passo 2: identifica os 4 pixels termicos vizinhos
//   Passo 3: calcula os pesos fx, fy (distancias fracionarias)
//   Passo 4: media ponderada = v00*(1-fx)*(1-fy) + v10*fx*(1-fy)
//                              + v01*(1-fx)*fy   + v11*fx*fy
//   Passo 5: converte o valor (0-255) para cor RGB565 via heatmapLUT

void EmberDisplayRX::buildBandInterpolated(int row) {
  int y0 = row;
  int y1 = (row < THERM_H - 1) ? row + 1 : row;  // nao sai dos limites na ultima linha

  for (int py = 0; py < BAND_H; py++) {
    int fy  = (py * 256) / BAND_H;  // peso vertical em ponto fixo (0-255)
    int ify = 256 - fy;              // complemento vertical (256 - fy)

    for (int sx = 0; sx < BAND_W; sx++) {
      // Passo 1: posicao fracionaria no eixo X termico, multiplicada por 256
      int txFP = (sx * 31 * 256) / (BAND_W - 1);
      int x0   = txFP >> 8;                                  // pixel termico a esquerda
      int x1   = (x0 < THERM_W - 1) ? x0 + 1 : x0;        // pixel termico a direita
      int fx   = txFP & 0xFF;                                // peso horizontal (0-255)
      int ifx  = 256 - fx;                                   // complemento horizontal

      // Passo 2: le os 4 pixels vizinhos do buffer termico
      uint16_t v00 = thermalData[y0 * THERM_W + x0];  // esquerda-cima
      uint16_t v10 = thermalData[y0 * THERM_W + x1];  // direita-cima
      uint16_t v01 = thermalData[y1 * THERM_W + x0];  // esquerda-baixo
      uint16_t v11 = thermalData[y1 * THERM_W + x1];  // direita-baixo

      // Passos 3 e 4: media ponderada com aritmetica inteira
      // >> 16 = divide por 65536 (equivale a normalizar os pesos x 256 x 256)
      // + 32768 = arredondamento correto (metade de 65536)
      uint16_t val = (uint16_t)(
        (v00 * ifx * ify + v10 * fx * ify + v01 * ifx * fy + v11 * fx * fy + 32768)
        >> 16
      );
      if (val > 255) val = 255;

      // Passo 5: consulta a tabela de cores pre-calculada
      bandBuffer[py * BAND_W + sx] = heatmapLUT[val];
    }
  }
}


// -----------------------------------
// Desenho da Frame Completa
// -----------------------------------
// Percorre as 24 linhas termicas, faixa a faixa.
// Chama receivePackets() entre faixas para nao perder pacotes
// da proxima frame enquanto o ecra esta a ser desenhado.

void EmberDisplayRX::drawThermalSmooth() {
  for (int row = 0; row < THERM_H; row++) {
    buildBandInterpolated(row);
    // pushImage: envia um bloco de pixels diretamente ao ecra via SPI,
    // muito mais rapido do que desenhar pixel a pixel
    _tft->pushImage(0, OFFSET_Y + row * BAND_H, BAND_W, BAND_H, bandBuffer);
    receivePackets();  // aproveita para receber pacotes durante o desenho
  }
  // etiquetas de escala de temperatura nos cantos inferiores
  _tft->setTextSize(1);
  _tft->setTextColor(TFT_WHITE, TFT_BLACK);
  _tft->setCursor(5, 318);
  _tft->print("20C");   // temperatura minima do mapa (azul)
  _tft->setCursor(450, 318);
  _tft->print("60C");   // temperatura maxima do mapa (vermelho)
}


// -----------------------------------
// Indicador de Ligacao
// -----------------------------------
// Quando o estado muda para 0 (sem sinal), forca o blink a
// desenhar imediatamente na proxima chamada de update().

void EmberDisplayRX::drawConnectionIndicator(uint8_t state) {
  if (state == 0) {
    _blinkTimer = millis() - 1001;  // subtrai mais de 1000ms para forcar draw imediato
  }
}

// Verifica o estado da ligacao com base nos pacotes recebidos
// e no tempo desde o ultimo pacote. Atualiza connState se mudou.
void EmberDisplayRX::updateConnectionState() {
  uint8_t newState;
  if (millis() - lastPacketTime > 1500) {
    newState = 0;  // mais de 1500ms sem pacotes = sem sinal
  } else if (lastPacketCount >= CONN_GOOD_THRESHOLD) {
    newState = 2;  // 24 ou mais pacotes por frame = ligacao boa
  } else {
    newState = 1;  // algum sinal mas abaixo do limiar = ligacao fraca
  }

  if (newState != prevConnState) {
    connState     = newState;
    prevConnState = newState;
    drawConnectionIndicator(connState);
  }
}


// -----------------------------------
// Piscar "CONNECTION PERDIDA"
// -----------------------------------
// Mostra o texto a cada 1 segundo quando nao ha sinal do drone.
// Usa millis() em vez de delay() para nao bloquear o loop.

void EmberDisplayRX::updateConnectionBlink() {
  if (millis() - _blinkTimer < 1000) return;  // ainda nao passou 1 segundo
  _blinkTimer = millis();
  _tft->setTextSize(4);
  _tft->setTextColor(COL_WHITE);
  _tft->setCursor(24, 144);
  _tft->print("CONNECTION PERDIDA");
}


// -----------------------------------
// Reset da Frame
// -----------------------------------
// Limpa os registos de pacotes para estar pronto a receber
// uma nova frame do drone.

void EmberDisplayRX::resetFrame() {
  memset(packetReceived, false, 32);  // marca todos os 32 pacotes como nao recebidos
  packetsGot   = 0;
  frameStartMs = millis();
}


// -----------------------------------
// Update (Atualizacao, chamar no loop)
// -----------------------------------
// Passo 1: recebe todos os pacotes disponiveis no radio
// Passo 2: verifica o estado de ligacao a cada 500ms
// Passo 3: mostra "CONNECTION PERDIDA" se aplicavel
// Passo 4: avalia os 3 criterios de decisao de desenho
// Passo 5: se deve desenhar, chama drawThermalSmooth() e regista estatisticas

void EmberDisplayRX::update() {
  if (!_nrfOK) return;  // se o radio falhou na inicializacao, nao faz nada

  // Passo 1
  receivePackets();

  // Passo 2: verificacao periodica a cada 500ms (nao e necessario verificar
  // em cada iteracao do loop, que corre centenas de vezes por segundo)
  static uint32_t lastConnCheck = 0;
  if (millis() - lastConnCheck > 500) {
    lastConnCheck = millis();
    updateConnectionState();
  }

  // Passo 3
  if (connState == 0 && framesDrawn > 0) {
    updateConnectionBlink();
  }


  // -----------------------------------
  // Decisao de Desenho
  // -----------------------------------
  // 3 criterios por ordem de prioridade/qualidade:

  bool shouldDraw = false;
  if (packetsGot >= 32) {
    // Criterio 1: frame completa (32/32 pacotes), qualidade maxima
    shouldDraw = true;
  } else if (packetsGot >= 10 && (millis() - frameStartMs) > FRAME_TIMEOUT_MS) {
    // Criterio 2: 10 ou mais pacotes e ja passaram 100ms, boa qualidade
    shouldDraw = true;
  } else if (packetsGot > 0 && (millis() - frameStartMs) > 250) {
    // Criterio 3: qualquer dado e ja passaram 250ms, evita ecra vazio
    shouldDraw = true;
  }

  // Passo 5: desenha e regista estatisticas
  if (shouldDraw) {
    uint32_t t0 = millis();
    drawThermalSmooth();
    uint32_t drawTime = millis() - t0;  // tempo que o desenho demorou em ms

    lastPacketCount = packetsGot;
    updateConnectionState();
    framesDrawn++;

    uint32_t agora = millis();
    uint32_t dt    = agora - lastFrameTime;  // intervalo entre frames em ms
    _lastDrawMs    = drawTime;
    _lastFPS       = dt > 0 ? 1000 / dt : 0;
    lastFrameTime  = agora;
    resetFrame();  // prepara os buffers para a proxima frame
  }
}
