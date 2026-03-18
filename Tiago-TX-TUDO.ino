// ============================================
// Tiago-TX — Controlador + Alarme
// Envia comandos (ARM/throttle/yaw) ao Goncalo-GX
// Recebe estado dos sensores via ACK payload
//
// PINOS (ESP32):
//   Joystick Y  -> GPIO34 (ADC)
//   Joystick X  -> GPIO35 (ADC)
//   BTN ARM     -> GPIO25
//   BTN STOP    -> GPIO26
//   Buzzer      -> GPIO21
//   LED         -> GPIO22
//   nRF24 CE    -> GPIO4  | CSN -> GPIO5
//   SCK=18, MISO=19, MOSI=23
//
// PAYLOAD enviado (5 bytes — struct PayloadCtrl):
//   armed:    0 = stop/desarmado, 1 = armado
//   throttle: microsegundos (1000-2000)
//   yaw:      -100 a +100
//
// ACK PAYLOAD recebido (1 byte — estado sensores):
//   0 = OK | 1 = CO/Gas | 2 = Fogo | 3 = Fogo+CO
// ============================================

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// --- Joystick ---
#define JOY_Y_PIN   34
#define JOY_X_PIN   35

// --- Botoes ---
#define BTN_ARM_PIN  25
#define BTN_STOP_PIN 26

// --- Buzzer + LED ---
#define BUZZER_PIN 21
#define LED_PIN    22

// --- nRF24 ---
#define CE_PIN   4
#define CSN_PIN  5
#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23

// --- Throttle ---
#define THROTTLE_NORMAL 1300
#define THROTTLE_Y_LOW  1200
#define THROTTLE_Y_HIGH 1500
#define THROTTLE_MIN    1000
#define YAW_MAX         100

// --- Alarme ---
#define FREQ_CO          700
#define FREQ_FOGO       1400
#define BEEP_DUR_CO     400UL
#define BEEP_PAUSA_CO   400UL
#define BEEP_DUR_FOGO   150UL
#define BEEP_PAUSA_FOGO  80UL

// --- Payload de controlo ---
struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
};

RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[6] = "00001";

bool armed = false;
uint8_t estadoSensor = 0;

// --- Alarme non-blocking ---
bool alarmeAtivo    = false;
int  alarmFreq      = 0;
unsigned long beepDuracao = 0, beepPausa = 0;
bool beepEstado     = false;
unsigned long ultimoBeep = 0;

void atualizarAlarme() {
  if (!alarmeAtivo) {
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    return;
  }
  unsigned long agora = millis();
  unsigned long intervalo = beepEstado ? beepDuracao : beepPausa;
  if (agora - ultimoBeep >= intervalo) {
    ultimoBeep = agora;
    beepEstado = !beepEstado;
    if (beepEstado) { tone(BUZZER_PIN, alarmFreq); digitalWrite(LED_PIN, HIGH); }
    else            { noTone(BUZZER_PIN);           digitalWrite(LED_PIN, LOW);  }
  }
}

void ativarAlarme(int freq, unsigned long dur, unsigned long pausa) {
  if (!alarmeAtivo || alarmFreq != freq) {
    alarmeAtivo  = true;
    alarmFreq    = freq;
    beepDuracao  = dur;
    beepPausa    = pausa;
    beepEstado   = true;
    ultimoBeep   = millis();
    tone(BUZZER_PIN, freq);
    digitalWrite(LED_PIN, HIGH);
  }
}

void desativarAlarme() {
  alarmeAtivo = false;
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
}

void aplicarEstado(uint8_t estado) {
  switch (estado) {
    case 0: desativarAlarme();
            Serial.println("[TX] OK — silencio");
            break;
    case 1: ativarAlarme(FREQ_CO, BEEP_DUR_CO, BEEP_PAUSA_CO);
            Serial.println("[TX] CO/Gas! — 700Hz lento");
            break;
    case 2: ativarAlarme(FREQ_FOGO, BEEP_DUR_FOGO, BEEP_PAUSA_FOGO);
            Serial.println("[TX] FOGO! — 1400Hz rapido");
            break;
    case 3: ativarAlarme(FREQ_FOGO, BEEP_DUR_FOGO, BEEP_PAUSA_FOGO);
            Serial.println("[TX] FOGO + CO! — 1400Hz rapido");
            break;
    default: desativarAlarme(); break;
  }
}

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(LED_PIN,      OUTPUT);
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);

  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[TX] ERRO: nRF24 nao iniciou! Verifica SPI/cabos.");
    while (1) delay(1000);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.enableAckPayload();        // recebe estado dos sensores no ACK
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();           // Tiago = mestre, sempre envia

  Serial.println("=================================");
  Serial.println("  TX Tiago — Controlo + Alarme  ");
  Serial.println("=================================");
  Serial.println("[TX] Pronto. Aguarda botao ARM.");
}

// -------------------------------------------------------
void loop() {
  bool armBtn  = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);
  static bool lastArm = false;

  // --- STOP de emergencia ---
  if (stopBtn) {
    armed = false;
    PayloadCtrl p = {0, THROTTLE_MIN, 0};
    radio.write(&p, sizeof(p));
    Serial.println("[TX] Emergency STOP");
    delay(300);
    atualizarAlarme();
    return;
  }

  // --- ARM toggle (rising edge) ---
  if (armBtn && !lastArm) {
    armed = !armed;
    Serial.println(armed ? "[TX] ARM: ON" : "[TX] ARM: OFF");
    delay(300);
  }
  lastArm = armBtn;

  // --- Leitura joystick ---
  int y = analogRead(JOY_Y_PIN);
  int x = analogRead(JOY_X_PIN);

  int throttle;
  if      (y <= 1000) throttle = THROTTLE_Y_LOW;
  else if (y >= 3500) throttle = THROTTLE_Y_HIGH;
  else                throttle = THROTTLE_NORMAL;

  int yaw = 0;
  if      (x <= 1000) yaw = -YAW_MAX;
  else if (x >= 3500) yaw =  YAW_MAX;

  // Se desarmado, envia throttle minimo
  if (!armed) { throttle = THROTTLE_MIN; yaw = 0; }

  // --- Envio ao Goncalo ---
  PayloadCtrl p = { (uint8_t)armed, (int16_t)throttle, (int16_t)yaw };
  bool ok = radio.write(&p, sizeof(p));

  // --- Recebe ACK payload (estado dos sensores do Goncalo) ---
  if (ok && radio.available()) {
    uint8_t v;
    radio.read(&v, 1);
    v = constrain(v, 0, 3);
    if (v != estadoSensor) {
      estadoSensor = v;
      Serial.print("[TX] Sensor mudou para estado=");
      Serial.println(estadoSensor);
      aplicarEstado(estadoSensor);
    }
  }

  Serial.print("ARM="); Serial.print(armed);
  Serial.print(" Y=");  Serial.print(y);
  Serial.print(" X=");  Serial.print(x);
  Serial.print(" T=");  Serial.print(throttle);
  Serial.print(" Yaw=");Serial.print(yaw);
  Serial.print(" RF="); Serial.println(ok ? "OK" : "FAIL");

  atualizarAlarme();
  delay(40);
}
