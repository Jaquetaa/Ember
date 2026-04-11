#ifndef EMBER_THERMAL_TX_H
#define EMBER_THERMAL_TX_H

#include <Arduino.h>

// Header minimo — os includes pesados (MLX90640, RF24, Wire)
// ficam so no .cpp para nao poluir o scope do .ino

class SPIClass;

class EmberThermalTX {
public:
  bool begin(uint8_t cePin, uint8_t csnPin, uint8_t sdaPin, uint8_t sclPin, SPIClass &spi);
  void update();

private:
  void *_radio = nullptr;      // RF24* cast internamente no .cpp
  void *_mlx = nullptr;        // Adafruit_MLX90640* cast internamente no .cpp
  bool _nrfOK = false;
  uint32_t _frameCount = 0;
  float   *_mlxFrame = nullptr;
  uint8_t *_thermalData = nullptr;
};

#endif
