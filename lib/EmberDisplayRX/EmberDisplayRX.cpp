#include "EmberDisplayRX.h"

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

const uint8_t EmberDisplayRX::THERM_ADDR[6] = "THERM";

EmberDisplayRX::EmberDisplayRX(uint8_t cePin, uint8_t csnPin)
  : _radio(cePin, csnPin), _tft(nullptr), _csnPin(csnPin), _nrfOK(false),
    packetsGot(0), frameStartMs(0), framesDrawn(0), lastFrameTime(0),
    lastPacketCount(0), lastPacketTime(0), connState(0), prevConnState(255),
    _blinkVisible(false), _blinkTimer(0) {}

bool EmberDisplayRX::begin(SPIClass &spi, TFT_eSPI &tft) {
  _tft = &tft;

  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  delay(10);

  for (int t = 1; t <= 3; t++) {
    Serial.print("[DISPLAY-RX] NRF tentativa "); Serial.print(t); Serial.print("/3 ... ");
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
    Serial.println("[DISPLAY-RX] nRF24 OK: canal=2, 250KBPS, PA_HIGH, CRC OFF");
  } else {
    Serial.println("[DISPLAY-RX] *** NRF FALHOU ***");
  }

  buildHeatmapLUT();
  memset(thermalData, 0, 768);
  resetFrame();
  lastPacketTime = millis();

  return _nrfOK;
}

void EmberDisplayRX::buildHeatmapLUT() {
  for (int i = 0; i < 256; i++) {
    int r, g = 0, b;
    if (i < 128) {
      // Ciano → Roxo: vermelho sobe, verde desce, azul sobe ao pico
      int t = i;
      r = (t * 255) / 127;         // 0   → 255
      g = 180 - (t * 180) / 127;  // 180 → 0
      b = 220 + (t * 35)  / 127;  // 220 → 255  (pico roxo = 255,0,255)
    } else {
      // Roxo → Vermelho: azul desce, vermelho e verde fixos
      int t = i - 128;
      r = 255;
      g = 0;
      b = 255 - (t * 255) / 127;  // 255 → 0
    }
    r = constrain(r, 0, 255);
    b = constrain(b, 0, 255);
    // pushImage envia bytes em ordem little-endian (low byte primeiro via SPI),
    // mas o display espera high byte primeiro — byte-swap corrige o desvio.
    uint16_t px = (((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | ((uint16_t)b >> 3);
    heatmapLUT[i] = (px >> 8) | (px << 8);
  }
}

void EmberDisplayRX::receivePackets() {
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

void EmberDisplayRX::buildBandInterpolated(int row) {
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

void EmberDisplayRX::drawThermalSmooth() {
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

void EmberDisplayRX::drawConnectionIndicator(uint8_t state) {
  if (state == 0) {
    _blinkTimer = millis() - 1001; // forca draw imediato na proxima atualizacao
  }
}

void EmberDisplayRX::updateConnectionState() {
  uint8_t newState;
  if (millis() - lastPacketTime > 1500) {
    newState = 0;
  } else if (lastPacketCount >= CONN_GOOD_THRESHOLD) {
    newState = 2;
  } else {
    newState = 1;
  }

  if (newState != prevConnState) {
    connState = newState;
    prevConnState = newState;
    drawConnectionIndicator(connState);
  }
}

void EmberDisplayRX::updateConnectionBlink() {
  if (millis() - _blinkTimer < 1000) return;
  _blinkTimer = millis();
  _tft->setTextSize(4);
  _tft->setTextColor(COL_WHITE);
  _tft->setCursor(24, 144);
  _tft->print("CONNECTION PERDIDA");
}

void EmberDisplayRX::resetFrame() {
  memset(packetReceived, false, 32);
  packetsGot = 0;
  frameStartMs = millis();
}

void EmberDisplayRX::update() {
  if (!_nrfOK) return;

  receivePackets();

  static uint32_t lastConnCheck = 0;
  if (millis() - lastConnCheck > 500) {
    lastConnCheck = millis();
    updateConnectionState();
  }

  if (connState == 0 && framesDrawn > 0) {
    updateConnectionBlink();
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

    Serial.print("[DISPLAY-RX] frame="); Serial.print(framesDrawn);
    Serial.print(" pkts="); Serial.print(packetsGot);
    Serial.print("/32 draw="); Serial.print(drawTime);
    Serial.print("ms fps="); Serial.print(dt > 0 ? 1000 / dt : 0);
    Serial.print(" conn=");
    Serial.println(connState == 2 ? "BOA" : (connState == 1 ? "FRACA" : "SEM"));

    lastFrameTime = agora;
    resetFrame();
  }
}
