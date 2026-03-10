#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define LED_REMOTE_PIN  21

#define CE_PIN          4
#define CSN_PIN         5

// Ajusta aos GPIOs reais do SPI no ESP32-S3 do Gonçalo
#define PIN_SCK         12
#define PIN_MISO        13
#define PIN_MOSI        11

// Se o LED estiver ligado em active-low (3.3V->R->LED->GPIO), mete 1
#define LED_ACTIVE_LOW  0

RF24 radio(CE_PIN, CSN_PIN);m
const uint8_t ADDRESS[6] = "00001";
  
uint8_t ledState = 0;     // estado atual aplicado
uint8_t lastPrinted = 255;

void applyLed(uint8_t v) {
  uint8_t out = v ? 1 : 0;
  if (LED_ACTIVE_LOW) out = out ? 0 : 1;
  digitalWrite(LED_REMOTE_PIN, out ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_REMOTE_PIN, OUTPUT);
  applyLed(0);            // começa OFF
  ledState = 0;

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);

  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[RX] ERRO: nRF24 nao iniciou (SPI/cabos/alimentacao).");
    while (1) delay(1000);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.setAutoAck(true);

  radio.openReadingPipe(1, ADDRESS);
  radio.startListening(); // tem de estar a ouvir para receber [web:40]

  Serial.println("[RX] Pronto: ledState=1 liga LED, ledState=0 desliga LED (sem spam)");
}

void loop() {
  if (radio.available()) {
    uint8_t v;

    // drena fila e fica com o mais recente
    while (radio.available()) {
      radio.read(&v, sizeof(v));
    }

    v = v ? 1 : 0;

    if (v != ledState) {
      ledState = v;
      applyLed(ledState);

      Serial.print("[RX] MUDOU ledState=");
      Serial.println(ledState);
    }
  }

  delay(5);
}
