#ifndef EMBER_CAM_NRF_H
#define EMBER_CAM_NRF_H

#include <Arduino.h>

// Header minimo — os includes pesados (MLX90640, RF24, Wire)
// ficam so no .cpp para nao poluir o scope do .ino

class SPIClass;

class EmberCAMNRF {
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

  uint8_t _sdaPin = 0;
  uint8_t _sclPin = 0;

  // Maquina de estados nao-bloqueante:
  //  0 = IDLE (espera intervalo entre frames)
  //  1 = TX_BURST (a enviar pacotes em batches)
  uint8_t  _state = 0;
  uint8_t  _txIndex = 0;        // proximo pacote (0..31) a enviar no burst
  uint32_t _lastFrameMs = 0;    // millis() do inicio do ultimo frame
};

#endif
