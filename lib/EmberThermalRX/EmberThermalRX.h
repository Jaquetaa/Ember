#ifndef EMBER_THERMAL_RX_H
#define EMBER_THERMAL_RX_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <TFT_eSPI.h>

class EmberThermalRX {
public:
  EmberThermalRX(uint8_t cePin, uint8_t csnPin);
  bool begin(SPIClass &spi, TFT_eSPI &tft);
  void update();   // chamar no loop — recebe pacotes, desenha quando pronto

private:
  RF24 _radio;
  TFT_eSPI *_tft;
  uint8_t _csnPin;
  bool _nrfOK;

  // Dados termicos
  uint8_t thermalData[768];
  bool    packetReceived[32];
  uint8_t packetsGot;
  uint32_t frameStartMs;
  uint32_t framesDrawn;
  uint32_t lastFrameTime;

  // Conexao
  uint8_t lastPacketCount;
  uint32_t lastPacketTime;
  uint8_t connState;
  uint8_t prevConnState;

  // Buffer de banda
  uint16_t bandBuffer[480 * 13];

  // Heatmap LUT
  uint16_t heatmapLUT[256];

  struct ThermalPacket {
    uint8_t index;
    uint8_t data[24];
  };

  void buildHeatmapLUT();
  void receivePackets();
  void buildBandInterpolated(int row);
  void drawThermalSmooth();
  void drawConnectionIndicator(uint8_t state);
  void drawCornerBracket(int cx, int cy, int dirX, int dirY, uint16_t col);
  void updateConnectionState();
  void resetFrame();

  static const uint8_t THERM_ADDR[6];
};

#endif
