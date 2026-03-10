#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define BUTTON_PIN      21
#define LED_DEBUG_PIN   22

#define CE_PIN          4
#define CSN_PIN         5
#define PIN_SCK         18
#define PIN_MISO        19
#define PIN_MOSI        23

// ======= AJUSTE AQUI (1 linha) =======
// 1 = botão ligado ao GND (premido lê LOW) -> INPUT_PULLUP
// 0 = botão ligado ao 3.3V (premido lê HIGH) -> INPUT_PULLDOWN
#define BUTTON_TO_GND   0

RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[6] = "00001";

uint8_t ledState = 0;          // o que enviamos (0/1)
uint8_t lastSent = 255;        // para não reenviar igual

// debounce
uint8_t lastRaw = 0;
uint8_t debouncedRaw = 0;
unsigned long tChange = 0;
const unsigned long debounceMs = 120;

static inline const char* HL(uint8_t v) { return v ? "HIGH" : "LOW"; }

uint8_t isPressedRaw(uint8_t rawLevel) {
#if BUTTON_TO_GND
  // INPUT_PULLUP: solto=HIGH, premido=LOW
  return (rawLevel == LOW) ? 1 : 0;  // premido => 1
#else
  // INPUT_PULLDOWN: solto=LOW, premido=HIGH
  return (rawLevel == HIGH) ? 1 : 0; // premido => 1
#endif
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_DEBUG_PIN, OUTPUT);
  digitalWrite(LED_DEBUG_PIN, LOW); // começa OFF

#if BUTTON_TO_GND
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[TX] Botão: INPUT_PULLUP (premido=LOW)");
#else
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  Serial.println("[TX] Botão: INPUT_PULLDOWN (premido=HIGH)");
#endif

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);

  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[TX] ERRO: nRF24 nao iniciou (SPI/cabos/alimentacao).");
    while (1) delay(1000);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  // Para não precisares de bursts/spam: usa ACK.
  radio.setAutoAck(true);
  radio.setRetries(5, 15);

  radio.openWritingPipe(ADDRESS);
  radio.stopListening();

  // Estado inicial fixo
  ledState = 0;
  digitalWrite(LED_DEBUG_PIN, LOW);

  // inicializa debounce
  lastRaw = digitalRead(BUTTON_PIN);
  debouncedRaw = lastRaw;
  tChange = millis();

  // envia 0 uma vez no boot (mantém “começa desligado” sincronizado)
  bool ok = radio.write(&ledState, sizeof(ledState));
  Serial.print("[TX] BOOT send ledState=0 write=");
  Serial.println(ok ? "OK(ACK)" : "FAIL(no ACK)");
  lastSent = ledState;

  Serial.println("[TX] Pronto: premir => 1 (LED ON), soltar => 0 (LED OFF)");
}

void loop() {
  const unsigned long now = millis();
  const uint8_t raw = digitalRead(BUTTON_PIN);

  if (raw != lastRaw) {
    lastRaw = raw;
    tChange = now;
    Serial.print("[TX] raw mudou -> "); Serial.println(HL(raw));
  }

  if ((now - tChange) > debounceMs && raw != debouncedRaw) {
    debouncedRaw = raw;

    uint8_t pressed = isPressedRaw(debouncedRaw);
    uint8_t newState = pressed ? 1 : 0;

    Serial.print("[TX] debounced="); Serial.print(HL(debouncedRaw));
    Serial.print(" pressed="); Serial.print(pressed);
    Serial.print(" => ledState="); Serial.println(newState);

    if (newState != ledState) {
      ledState = newState;
      digitalWrite(LED_DEBUG_PIN, ledState ? HIGH : LOW);

      if (ledState != lastSent) {
        bool ok = radio.write(&ledState, sizeof(ledState));
        Serial.print("[TX] SEND ledState="); Serial.print(ledState);
        Serial.print(" write="); Serial.println(ok ? "OK(ACK)" : "FAIL(no ACK)");
        lastSent = ledState;
      }
    }
  }

  delay(5);
}
