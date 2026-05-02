#include "EmberThermalRX.h"

#define DISP_W    480
#define DISP_H    320
#define THERM_W   32
#define THERM_H   24
#define SCALE_X   15
#define SCALE_Y   13
#define OFFSET_Y  4
#define BAND_W    480
#define BAND_H    SCALE_Y
#define FRAME_TIMEOUT_MS  100
#define CONN_GOOD_THRESHOLD   24
#define CONN_WEAK_THRESHOLD   10
#define COL_WHITE   0xFFFF
#define COL_OUTLINE 0x8410
#define COL_BLACK   0x0000

const uint8_t EmberThermalRX::THERM_ADDR[6] = "THERM";

EmberThermalRX::EmberThermalRX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _tft(nullptr), _csnPin(csnPin), _nrfOK(false),
    packetsGot(0), frameStartMs(0), framesDrawn(0), lastFrameTime(0),
    lastPacketCount(0), lastPacketTime(0), connState(0), prevConnState(255) {}

bool EmberThermalRX::begin(SPIClass &spi, TFT_eSPI &tft) {
  _tft = &tft;

  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  for (int t = 1; t <= 3; t++) {
    Serial.print("[THERM-RX] NRF tentativa "); Serial.print(t); Serial.print("/3 ... ");
    if (_radio.begin(&spi) && _radio.isChipConnected()) {
      Serial.println("OK!");
      _nrfOK = true;
      break;
    }
    Serial.println("FALHOU");
    delay(200);
  }

  if (_nrfOK) {
    _radio.setChannel(2);
    _radio.setDataRate(RF24_250KBPS);
    _radio.setPALevel(RF24_PA_HIGH);
    _radio.setAutoAck(false);
    _radio.setPayloadSize(25);
    _radio.setCRCLength(RF24_CRC_DISABLED);
    _radio.openReadingPipe(1, THERM_ADDR);
    _radio.startListening();
    Serial.println("[THERM-RX] nRF24 OK: canal=2, 250KBPS, PA_HIGH, CRC OFF");
  } else {
    Serial.println("[THERM-RX] *** NRF FALHOU ***");
  }

  buildHeatmapLUT();
  memset(thermalData, 0, 768);
  resetFrame();
  lastPacketTime = millis();

  return _nrfOK;
}

void EmberThermalRX::buildHeatmapLUT() {
  for (int i = 0; i < 256; i++) {
    int r, g, b;
    if (i < 85) {
      r = 60 + (i * 140) / 84;
      g = 0;
      b = 40 + (i * 140) / 84;
    } else if (i < 150) {
      int t = i - 85;
      r = 200 + (t * 55) / 64;
      g = 0;
      b = 180 - (t * 180) / 64;
    } else if (i < 210) {
      int t = i - 150;
      r = 255;
      g = (t * 180) / 59;
      b = 0;
    } else {
      int t = i - 210;
      r = 255;
      g = 180 + (t * 75) / 45;
      b = 0;
    }
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
    heatmapLUT[i] = (((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | ((uint16_t)b >> 3);
  }
}

void EmberThermalRX::receivePackets() {
  while (_radio.available()) {
    ThermalPacket pkt;
    _radio.read(&pkt, sizeof(pkt));
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

void EmberThermalRX::buildBandInterpolated(int row) {
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

void EmberThermalRX::drawThermalSmooth() {
  for (int row = 0; row < THERM_H; row++) {
    buildBandInterpolated(row);
    _tft->pushImage(0, OFFSET_Y + row * BAND_H, BAND_W, BAND_H, bandBuffer);
    receivePackets();
  }
  _tft->setTextSize(1);
  _tft->setTextColor(TFT_WHITE, TFT_BLACK);
  _tft->setCursor(5, 318);
  _tft->print("20C");
  _tft->setCursor(450, 318);
  _tft->print("60C");
}

void EmberThermalRX::drawCornerBracket(int cx, int cy, int dirX, int dirY, uint16_t col) {
  int armLen = 12;
  int thick = 2;
  for (int t = 0; t < thick; t++) {
    for (int a = 0; a < armLen; a++) {
      _tft->drawPixel(cx + dirX * a, cy + dirY * t, col);
    }
  }
  for (int t = 0; t < thick; t++) {
    for (int a = 0; a < armLen; a++) {
      _tft->drawPixel(cx + dirX * t, cy + dirY * a, col);
    }
  }
}

void EmberThermalRX::drawConnectionIndicator(uint8_t state) {
  _tft->fillRect(0, 0, 18, 18, COL_BLACK);
  _tft->fillRect(DISP_W - 18, 0, 18, 18, COL_BLACK);
  _tft->fillRect(0, DISP_H - 18, 18, 18, COL_BLACK);
  _tft->fillRect(DISP_W - 18, DISP_H - 18, 18, 18, COL_BLACK);

  int margin = 4;

  if (state == 2) {
    drawCornerBracket(margin - 1, margin - 1, 1, 1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin + 1, margin - 1, -1, 1, COL_OUTLINE);
    drawCornerBracket(margin - 1, DISP_H - 1 - margin + 1, 1, -1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin + 1, DISP_H - 1 - margin + 1, -1, -1, COL_OUTLINE);
    drawCornerBracket(margin, margin, 1, 1, COL_WHITE);
    drawCornerBracket(DISP_W - 1 - margin, margin, -1, 1, COL_WHITE);
    drawCornerBracket(margin, DISP_H - 1 - margin, 1, -1, COL_WHITE);
    drawCornerBracket(DISP_W - 1 - margin, DISP_H - 1 - margin, -1, -1, COL_WHITE);
  } else if (state == 1) {
    drawCornerBracket(margin, margin, 1, 1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin, margin, -1, 1, COL_OUTLINE);
    drawCornerBracket(margin, DISP_H - 1 - margin, 1, -1, COL_OUTLINE);
    drawCornerBracket(DISP_W - 1 - margin, DISP_H - 1 - margin, -1, -1, COL_OUTLINE);
  } else {
    int xSize = 8;
    for (int d = 0; d < xSize; d++) {
      _tft->drawPixel(margin + d, margin + d, COL_WHITE);
      _tft->drawPixel(margin + d + 1, margin + d, COL_OUTLINE);
      _tft->drawPixel(margin + xSize - 1 - d, margin + d, COL_WHITE);
      _tft->drawPixel(margin + xSize - d, margin + d, COL_OUTLINE);
    }
    for (int d = 0; d < xSize; d++) {
      _tft->drawPixel(DISP_W - 1 - margin - d, margin + d, COL_WHITE);
      _tft->drawPixel(DISP_W - 1 - margin - d - 1, margin + d, COL_OUTLINE);
      _tft->drawPixel(DISP_W - 1 - margin - xSize + 1 + d, margin + d, COL_WHITE);
      _tft->drawPixel(DISP_W - 1 - margin - xSize + d, margin + d, COL_OUTLINE);
    }
    for (int d = 0; d < xSize; d++) {
      _tft->drawPixel(margin + d, DISP_H - 1 - margin - d, COL_WHITE);
      _tft->drawPixel(margin + d + 1, DISP_H - 1 - margin - d, COL_OUTLINE);
      _tft->drawPixel(margin + xSize - 1 - d, DISP_H - 1 - margin - d, COL_WHITE);
      _tft->drawPixel(margin + xSize - d, DISP_H - 1 - margin - d, COL_OUTLINE);
    }
    for (int d = 0; d < xSize; d++) {
      _tft->drawPixel(DISP_W - 1 - margin - d, DISP_H - 1 - margin - d, COL_WHITE);
      _tft->drawPixel(DISP_W - 1 - margin - d - 1, DISP_H - 1 - margin - d, COL_OUTLINE);
      _tft->drawPixel(DISP_W - 1 - margin - xSize + 1 + d, DISP_H - 1 - margin - d, COL_WHITE);
      _tft->drawPixel(DISP_W - 1 - margin - xSize + d, DISP_H - 1 - margin - d, COL_OUTLINE);
    }
  }
}

void EmberThermalRX::updateConnectionState() {
  uint8_t newState;
  if (millis() - lastPacketTime > 1500) {
    newState = 0;
  } else if (lastPacketCount >= CONN_GOOD_THRESHOLD) {
    newState = 2;
  } else if (lastPacketCount >= CONN_WEAK_THRESHOLD) {
    newState = 1;
  } else {
    newState = 1;
  }

  if (newState != prevConnState) {
    connState = newState;
    prevConnState = newState;
    drawConnectionIndicator(connState);
  }
}

void EmberThermalRX::resetFrame() {
  memset(packetReceived, false, 32);
  packetsGot = 0;
  frameStartMs = millis();
}

void EmberThermalRX::update() {
  if (!_nrfOK) return;

  receivePackets();

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

    lastPacketCount = packetsGot;
    updateConnectionState();
    framesDrawn++;

    uint32_t agora = millis();
    uint32_t dt = agora - lastFrameTime;

    Serial.print("[THERM] frame="); Serial.print(framesDrawn);
    Serial.print(" pkts="); Serial.print(packetsGot);
    Serial.print("/32 draw="); Serial.print(drawTime);
    Serial.print("ms fps="); Serial.print(dt > 0 ? 1000 / dt : 0);
    Serial.print(" conn=");
    Serial.println(connState == 2 ? "BOA" : (connState == 1 ? "FRACA" : "SEM"));

    lastFrameTime = agora;
    resetFrame();
  }
}
