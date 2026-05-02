#include "EmberControlRX.h"

const uint8_t EmberControlRX::ADDRESS[6] = "00001";

bool EmberControlRX::begin(uint8_t cePin, uint8_t csnPin, SPIClass &spi) {
  Serial.print("[CTRL-RX] begin() CE=");
  Serial.print(cePin);
  Serial.print(" CSN=");
  Serial.println(csnPin);
  Serial.flush();

  Serial.println("[CTRL-RX] pinMode CSN OUTPUT HIGH...");
  Serial.flush();
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  delay(10);

  Serial.println("[CTRL-RX] new RF24(CE,CSN, 1MHz)...");
  Serial.flush();
  _radio = new RF24(cePin, csnPin, 1000000);  // 1 MHz para fios longos
  if (!_radio) {
    Serial.println("[CTRL-RX] ERRO: new RF24 devolveu nullptr (OOM)");
    return false;
  }
  Serial.println("[CTRL-RX] RF24 alocado.");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->begin(&spi)...");
  Serial.flush();
  bool beginOK = _radio->begin(&spi);
  Serial.print("[CTRL-RX] begin() returned: ");
  Serial.println(beginOK ? "true" : "false");
  Serial.flush();

  Serial.println("[CTRL-RX] _radio->isChipConnected()...");
  Serial.flush();
  bool chipOK = _radio->isChipConnected();
  Serial.print("[CTRL-RX] isChipConnected: ");
  Serial.println(chipOK ? "true" : "false");
  Serial.flush();

  if (!beginOK || !chipOK) {
    Serial.println("[CTRL-RX] ERRO: nRF24 nao iniciou!");
    Serial.println("[CTRL-RX] Verifica: alimentacao 3.3V (com cap 10uF),");
    Serial.println("           SPI (SCK/MISO/MOSI), CE, CSN, GND comum.");
    // Dump dos registos para diagnostico (mostra address de RX_PIPE etc.)
    Serial.println("[CTRL-RX] ---- printDetails (raw) ----");
    Serial.flush();
    _radio->printDetails();
    Serial.println("[CTRL-RX] -----------------------------");
    Serial.flush();
    return false;
  }

  Serial.println("[CTRL-RX] config: ch=76, 250KBPS, PA_LOW, autoAck, ackPayload...");
  Serial.flush();
  _radio->setChannel(76);
  _radio->setDataRate(RF24_250KBPS);
  _radio->setPALevel(RF24_PA_MAX);
  _radio->setAutoAck(true);
  _radio->setRetries(1, 3);   // 500us x 3 = 1.5ms worst case (igual ao TX)
  _radio->enableAckPayload();
  _radio->openReadingPipe(1, ADDRESS);
  _radio->startListening();

  uint8_t zero = 0;
  _radio->writeAckPayload(1, &zero, 1);

  // Diagnostico final: confirma o que ficou nos registos
  Serial.println("[CTRL-RX] ---- printDetails (post-config) ----");
  Serial.flush();
  _radio->printDetails();
  Serial.println("[CTRL-RX] ------------------------------------");
  Serial.flush();

  _lastRxTime = millis();
  _initialized = true;
  Serial.println("[CTRL-RX] nRF24 OK: canal=76, 250KBPS, ACK");
  Serial.flush();
  return true;
}

bool EmberControlRX::isOK() const { return _initialized; }

bool EmberControlRX::receive(PayloadCtrl &p) {
  if (!_initialized) return false;
  if (_radio->available()) {
    while (_radio->available()) _radio->read(&p, sizeof(p));
    _lastRxTime = millis();
    return true;
  }
  return false;
}

void EmberControlRX::updateAckPayload(uint8_t estado) {
  if (!_initialized) return;
  _radio->writeAckPayload(1, &estado, 1);
}

unsigned long EmberControlRX::lastReceiveTime() {
  if (!_initialized) return millis();  // evita timeout imediato no loop
  return _lastRxTime;
}
