// ============================================
// Tiago-Thermal-RX v10 — Comando (ESP32)
//
// MELHORIAS v10:
//   - Paleta roxa-vermelha-amarela (estilo incendio)
//   - Indicador de conexao nos cantos (branco, outline fina)
//   - RF24 otimizado para maximo alcance (250KBPS, PA_MAX, canal 2)
//   - Interpolacao bilinear fixed-point mantida
//
// PINOS (ESP32 Dev Module):
//   nRF24L01:  MOSI=23 | MISO=19 | SCK=18 | CSN=32 | CE=33
//   Ecra:      CS=27   | DC=12   | RST=14 | T_CS=13
//
// BIBLIOTECAS:
//   - RF24 (by TMRh20)
//   - TFT_eSPI (by Bodmer) — configurar User_Setup.h!
//
// User_Setup.h do TFT_eSPI:
//   #define ST7796_DRIVER
//   #define TFT_MOSI 23
//   #define TFT_MISO 19
//   #define TFT_SCLK 18
//   #define TFT_CS   27
//   #define TFT_DC   12
//   #define TFT_RST  14
//   #define TOUCH_CS 13
//   #define SPI_FREQUENCY       40000000
//   #define SPI_READ_FREQUENCY  20000000
// ============================================

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <TFT_eSPI.h>

// ── SPI ─────────────────────────────────────────────────
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23

// ── nRF24L01 ────────────────────────────────────────────
#define THERM_CE   33
#define THERM_CSN  32

// ── Frame timing ────────────────────────────────────────
#define FRAME_TIMEOUT_MS  100

// ── Display dimensions ──────────────────────────────────
#define DISP_W    480
#define DISP_H    320
#define THERM_W   32
#define THERM_H   24

// Cada pixel termico ocupa este espaco no ecra:
// scaleX = 480/32 = 15, scaleY = 312/24 = 13
// Area termica: 480 x 312, centrada com offsetY = 4
#define SCALE_X   15
#define SCALE_Y   13
#define OFFSET_Y  4
#define BAND_W    480
#define BAND_H    SCALE_Y   // 13 pixels de altura por banda

// ── Conexao — limiares ──────────────────────────────────
#define CONN_GOOD_THRESHOLD   24   // 24+ pacotes = boa conexao
#define CONN_WEAK_THRESHOLD   10   // 10-23 pacotes = fraca
// < 10 pacotes = sem conexao

// ── Objectos ────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();
RF24 radioTherm(THERM_CE, THERM_CSN);
const uint8_t THERM_ADDR[6] = "THERM";
bool nrfOK = false;

// ── Dados termicos ──────────────────────────────────────
uint8_t thermalData[768];
bool    packetReceived[32];
uint8_t packetsGot = 0;
uint32_t frameStartMs = 0;

struct ThermalPacket {
  uint8_t index;
  uint8_t data[24];
};

uint32_t framesDrawn = 0;
uint32_t lastFrameTime = 0;

// ── Estado da conexao ───────────────────────────────────
uint8_t lastPacketCount = 0;       // Pacotes do ultimo frame
uint32_t lastPacketTime = 0;       // Ultimo timestamp de pacote recebido
uint8_t connState = 0;             // 0=sem, 1=fraca, 2=boa
uint8_t prevConnState = 255;       // Para redesenhar so quando muda

// ── Buffer de banda ─────────────────────────────────────
// 480 x 13 = 6240 pixels x 2 bytes = 12480 bytes
uint16_t bandBuffer[BAND_W * BAND_H];

// ── Heatmap LUT — Roxo → Vermelho → Amarelo ────────────
uint16_t heatmapLUT[256];

void buildHeatmapLUT() {
  // Paleta: Roxo escuro → Roxo → Magenta → Vermelho → Laranja → Amarelo
  // SEM azul puro, SEM verde. Sempre R >= B para garantir tom roxo, nao azul.
  for (int i = 0; i < 256; i++) {
    int r, g, b;

    if (i < 85) {
      // Fase 1: Roxo escuro → Roxo/Magenta intenso
      // R sobe forte para manter tom roxo (nao azul)
      // R: 60→200, G: 0, B: 40→180
      r = 60 + (i * 140) / 84;    // 60 → 200
      g = 0;
      b = 40 + (i * 140) / 84;    // 40 → 180
    }
    else if (i < 150) {
      // Fase 2: Magenta → Vermelho puro
      // R fica alto, B desce a zero
      // R: 200→255, G: 0, B: 180→0
      int t = i - 85;   // 0..64
      r = 200 + (t * 55) / 64;    // 200 → 255
      g = 0;
      b = 180 - (t * 180) / 64;   // 180 → 0
    }
    else if (i < 210) {
      // Fase 3: Vermelho → Laranja
      // R: 255, G sobe, B: 0
      int t = i - 150;  // 0..59
      r = 255;
      g = (t * 180) / 59;         // 0 → 180
      b = 0;
    }
    else {
      // Fase 4: Laranja → Amarelo brilhante
      // R: 255, G sobe ao maximo, B: 0
      int t = i - 210;  // 0..45
      r = 255;
      g = 180 + (t * 75) / 45;    // 180 → 255
      b = 0;
    }

    // Clamp seguro
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;

    heatmapLUT[i] = (((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | ((uint16_t)b >> 3);
  }
}

// ── Indicador de conexao nos cantos ─────────────────────
// Desenha simbolos nos 4 cantos do ecra:
//   - Boa conexao:  4 cantos em branco solido (mini L shapes)
//   - Fraca:        4 cantos a piscar / outline
//   - Sem conexao:  X nos cantos

// Helper: cor branca RGB565
#define COL_WHITE   0xFFFF
#define COL_OUTLINE 0x8410   // Cinza escuro para outline
#define COL_BLACK   0x0000

void drawCornerBracket(int cx, int cy, int dirX, int dirY, uint16_t col) {
  // Desenha um "L" de 12x12 pixels com 2px de espessura
  // dirX/dirY: +1 ou -1 para orientar o canto
  int armLen = 12;
  int thick = 2;

  // Braco horizontal
  for (int t = 0; t < thick; t++) {
    for (int a = 0; a < armLen; a++) {
      tft.drawPixel(cx + dirX * a, cy + dirY * t, col);
    }
  }
  // Braco vertical
  for (int t = 0; t < thick; t++) {
    for (int a = 0; a < armLen; a++) {
      tft.drawPixel(cx + dirX * t, cy + dirY * a, col);
    }
  }
}

void drawConnectionIndicator(uint8_t state) {
  // Limpar cantos primeiro (area 16x16 em cada canto)
  tft.fillRect(0, 0, 18, 18, COL_BLACK);
  tft.fillRect(DISP_W - 18, 0, 18, 18, COL_BLACK);
  tft.fillRect(0, DISP_H - 18, 18, 18, COL_BLACK);
  tft.fillRect(DISP_W - 18, DISP_H - 18, 18, 18, COL_BLACK);

  int margin = 4;

  if (state == 2) {
    // === BOA CONEXAO: 4 brackets brancos solidos ===
    // Outline fina (1px offset)
    drawCornerBracket(margin - 1, margin - 1, 1, 1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin + 1, margin - 1, -1, 1, COL_OUTLINE);
    drawCornerBracket(margin - 1, DISP_H - 1 - margin + 1, 1, -1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin + 1, DISP_H - 1 - margin + 1, -1, -1, COL_OUTLINE);
    // Branco principal
    drawCornerBracket(margin, margin, 1, 1, COL_WHITE);
    drawCornerBracket(DISP_W - 1 - margin, margin, -1, 1, COL_WHITE);
    drawCornerBracket(margin, DISP_H - 1 - margin, 1, -1, COL_WHITE);
    drawCornerBracket(DISP_W - 1 - margin, DISP_H - 1 - margin, -1, -1, COL_WHITE);
  }
  else if (state == 1) {
    // === CONEXAO FRACA: apenas outlines (brackets finos) ===
    drawCornerBracket(margin, margin, 1, 1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin, margin, -1, 1, COL_OUTLINE);
    drawCornerBracket(margin, DISP_H - 1 - margin, 1, -1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin, DISP_H - 1 - margin, -1, -1, COL_OUTLINE);
  }
  else {
    // === SEM CONEXAO: X em cada canto ===
    int xSize = 8;
    // Canto superior esquerdo
    for (int d = 0; d < xSize; d++) {
      tft.drawPixel(margin + d, margin + d, COL_WHITE);
      tft.drawPixel(margin + d + 1, margin + d, COL_OUTLINE);
      tft.drawPixel(margin + xSize - 1 - d, margin + d, COL_WHITE);
      tft.drawPixel(margin + xSize - d, margin + d, COL_OUTLINE);
    }
    // Canto superior direito
    for (int d = 0; d < xSize; d++) {
      tft.drawPixel(DISP_W - 1 - margin - d, margin + d, COL_WHITE);
      tft.drawPixel(DISP_W - 1 - margin - d - 1, margin + d, COL_OUTLINE);
      tft.drawPixel(DISP_W - 1 - margin - xSize + 1 + d, margin + d, COL_WHITE);
      tft.drawPixel(DISP_W - 1 - margin - xSize + d, margin + d, COL_OUTLINE);
    }
    // Canto inferior esquerdo
    for (int d = 0; d < xSize; d++) {
      tft.drawPixel(margin + d, DISP_H - 1 - margin - d, COL_WHITE);
      tft.drawPixel(margin + d + 1, DISP_H - 1 - margin - d, COL_OUTLINE);
      tft.drawPixel(margin + xSize - 1 - d, DISP_H - 1 - margin - d, COL_WHITE);
      tft.drawPixel(margin + xSize - d, DISP_H - 1 - margin - d, COL_OUTLINE);
    }
    // Canto inferior direito
    for (int d = 0; d < xSize; d++) {
      tft.drawPixel(DISP_W - 1 - margin - d, DISP_H - 1 - margin - d, COL_WHITE);
      tft.drawPixel(DISP_W - 1 - margin - d - 1, DISP_H - 1 - margin - d, COL_OUTLINE);
      tft.drawPixel(DISP_W - 1 - margin - xSize + 1 + d, DISP_H - 1 - margin - d, COL_WHITE);
      tft.drawPixel(DISP_W - 1 - margin - xSize + d, DISP_H - 1 - margin - d, COL_OUTLINE);
    }
  }
}

// ── Receber pacotes do NRF ──────────────────────────────
void receivePackets() {
  while (radioTherm.available()) {
    ThermalPacket pkt;
    radioTherm.read(&pkt, sizeof(pkt));
    if (pkt.index < 32) {
      memcpy(&thermalData[pkt.index * 24], pkt.data, 24);
      if (!packetReceived[pkt.index]) {
        packetReceived[pkt.index] = true;
        packetsGot++;
      }
      if (packetsGot == 1) frameStartMs = millis();
      lastPacketTime = millis();
    }
  }
}

// ── Interpolacao bilinear com fixed-point ───────────────
void buildBandInterpolated(int row) {
  int y0 = row;
  int y1 = (row < THERM_H - 1) ? row + 1 : row;

  for (int py = 0; py < BAND_H; py++) {
    int fy = (py * 256) / BAND_H;
    int ify = 256 - fy;

    for (int sx = 0; sx < BAND_W; sx++) {
      int txFP = (sx * 31 * 256) / (BAND_W - 1);
      int x0 = txFP >> 8;
      int x1 = (x0 < THERM_W - 1) ? x0 + 1 : x0;
      int fx = txFP & 0xFF;
      int ifx = 256 - fx;

      uint16_t v00 = thermalData[y0 * THERM_W + x0];
      uint16_t v10 = thermalData[y0 * THERM_W + x1];
      uint16_t v01 = thermalData[y1 * THERM_W + x0];
      uint16_t v11 = thermalData[y1 * THERM_W + x1];

      uint16_t val = (uint16_t)(
        (v00 * ifx * ify + v10 * fx * ify + v01 * ifx * fy + v11 * fx * fy + 32768)
        >> 16
      );

      if (val > 255) val = 255;
      bandBuffer[py * BAND_W + sx] = heatmapLUT[val];
    }
  }
}

// ── Desenho interpolado por bandas ──────────────────────
void drawThermalSmooth() {
  for (int row = 0; row < THERM_H; row++) {
    buildBandInterpolated(row);
    tft.pushImage(0, OFFSET_Y + row * BAND_H, BAND_W, BAND_H, bandBuffer);
    receivePackets();
  }

  // Legenda em baixo
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 318);
  tft.print("20C");
  tft.setCursor(450, 318);
  tft.print("60C");
}

// ── Atualizar estado de conexao ─────────────────────────
void updateConnectionState() {
  uint8_t newState;

  // Se nao recebemos nada ha mais de 1.5 segundos = sem conexao
  if (millis() - lastPacketTime > 1500) {
    newState = 0;  // Sem conexao
  }
  else if (lastPacketCount >= CONN_GOOD_THRESHOLD) {
    newState = 2;  // Boa
  }
  else if (lastPacketCount >= CONN_WEAK_THRESHOLD) {
    newState = 1;  // Fraca
  }
  else {
    newState = 1;  // Poucos pacotes = fraca
  }

  // So redesenhar quando o estado muda
  if (newState != prevConnState) {
    connState = newState;
    prevConnState = newState;
    drawConnectionIndicator(connState);
  }
}

// ── Reset frame ─────────────────────────────────────────
void resetFrame() {
  memset(packetReceived, false, 32);
  packetsGot = 0;
  frameStartMs = millis();
}

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[THERM-RX] Tiago Thermal RX v10 (MAX RANGE + NEW PALETTE)");

  // CS pins HIGH
  pinMode(THERM_CSN, OUTPUT);
  digitalWrite(THERM_CSN, HIGH);
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(10);

  // SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, THERM_CSN);
  delay(10);

  // NRF
  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-RX] NRF tentativa ");
    Serial.print(t);
    Serial.print("/3 ... ");
    if (radioTherm.begin(&SPI) && radioTherm.isChipConnected()) {
      Serial.println("OK!");
      nrfOK = true;
      break;
    }
    Serial.println("FALHOU");
    delay(200);
  }

  if (nrfOK) {
    radioTherm.setChannel(2);               // Canal 2 — fora da zona WiFi
    radioTherm.setDataRate(RF24_250KBPS);    // 250kbps = maxima sensibilidade
    radioTherm.setPALevel(RF24_PA_HIGH);      // Potencia maxima
    radioTherm.setAutoAck(false);            // Sem ACK
    radioTherm.setPayloadSize(25);           // 1 + 24 bytes
    radioTherm.setCRCLength(RF24_CRC_DISABLED); // Sem CRC
    radioTherm.openReadingPipe(1, THERM_ADDR);
    radioTherm.startListening();
    Serial.println("[THERM-RX] nRF24 OK: canal=2, 250KBPS, PA_MAX, CRC OFF");
  } else {
    Serial.println("[THERM-RX] *** NRF FALHOU ***");
  }

  // Ecra
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  if (nrfOK) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(50, 140);
    tft.print("Thermal RX v10 - EMBER");
  } else {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(50, 100);
    tft.print("ERRO: nRF24!");
  }

  buildHeatmapLUT();
  memset(thermalData, 0, 768);
  resetFrame();
  lastPacketTime = millis();

  Serial.println("[THERM-RX] === Setup completo ===\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  if (!nrfOK) { delay(1000); return; }

  receivePackets();

  // Verificar conexao periodicamente (mesmo sem frames completos)
  static uint32_t lastConnCheck = 0;
  if (millis() - lastConnCheck > 500) {
    lastConnCheck = millis();
    updateConnectionState();
  }

  bool shouldDraw = false;
  if (packetsGot >= 32) {
    shouldDraw = true;
  } else if (packetsGot >= 10 && (millis() - frameStartMs) > FRAME_TIMEOUT_MS) {
    shouldDraw = true;
  } else if (packetsGot > 0 && (millis() - frameStartMs) > 250) {
    shouldDraw = true;
  }

  if (shouldDraw) {
    uint32_t t0 = millis();
    drawThermalSmooth();
    uint32_t drawTime = millis() - t0;

    // Guardar contagem para avaliacao de conexao
    lastPacketCount = packetsGot;

    // Atualizar indicador apos cada frame
    updateConnectionState();

    framesDrawn++;
    uint32_t agora = millis();
    uint32_t dt = agora - lastFrameTime;

    Serial.print("[RX] F#");
    Serial.print(framesDrawn);
    Serial.print(" ");
    Serial.print(packetsGot);
    Serial.print("/32 draw=");
    Serial.print(drawTime);
    Serial.print("ms dt=");
    Serial.print(dt);
    Serial.print("ms ~");
    Serial.print(dt > 0 ? 1000 / dt : 0);
    Serial.print("fps conn=");
    Serial.println(connState == 2 ? "BOA" : (connState == 1 ? "FRACA" : "SEM"));

    lastFrameTime = agora;
    resetFrame();
  }
}
