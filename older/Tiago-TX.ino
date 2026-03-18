// ============================================
// Tiago-TX — Receptor
// Recebe estado do Gonçalo-GX e ativa Buzzer + LED
//
// PINOS:
//   Buzzer Piezo -> GPIO21  (onde estava o botão)
//   LED          -> GPIO22
//   nRF24 CE -> GPIO4  | CSN -> GPIO5
//   SCK=18, MISO=19, MOSI=23
//
// PAYLOAD recebido (1 byte):
//   0 = OK        → silêncio + LED OFF
//   1 = CO/Gás    → 700Hz lento  + LED pisca lento
//   2 = Fogo      → 1400Hz rápido + LED pisca rápido
//   3 = Fogo + CO → igual ao fogo (prioridade)
// ============================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- Outputs ---
#define BUZZER_PIN  21
#define LED_PIN     22

// --- nRF24 ---
#define CE_PIN   4
#define CSN_PIN  5
#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23

// --- Parâmetros de alarme ---
#define FREQ_CO            700
#define FREQ_FOGO         1400
#define BEEP_DURACAO_CO    400UL
#define BEEP_PAUSA_CO      400UL
#define BEEP_DURACAO_FOGO  150UL
#define BEEP_PAUSA_FOGO     80UL

RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[6] = "00001";

uint8_t estadoAtual = 0;

// --- Buzzer + LED non-blocking ---
bool          alarmeAtivo = false;
int           alarmFreq   = 0;
unsigned long beepDuracao = 0;
unsigned long beepPausa   = 0;
bool          beepEstado  = false;
unsigned long ultimoBeep  = 0;

// LED acompanha o buzzer: HIGH quando toca, LOW quando pausa
void atualizarAlarme() {
  if (!alarmeAtivo) {
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    return;
  }
  unsigned long agora     = millis();
  unsigned long intervalo = beepEstado ? beepDuracao : beepPausa;
  if (agora - ultimoBeep >= intervalo) {
    ultimoBeep = agora;
    beepEstado = !beepEstado;
    if (beepEstado) {
      tone(BUZZER_PIN, alarmFreq);
      digitalWrite(LED_PIN, HIGH);
    } else {
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void ativarAlarme(int freq, unsigned long duracao, unsigned long pausa) {
  if (!alarmeAtivo || alarmFreq != freq) {
    alarmeAtivo = true;
    alarmFreq   = freq;
    beepDuracao = duracao;
    beepPausa   = pausa;
    beepEstado  = true;
    ultimoBeep  = millis();
    tone(BUZZER_PIN, freq);
    digitalWrite(LED_PIN, HIGH);
  }
}

void desativarAlarme() {
  alarmeAtivo = false;
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
}

// -------------------------------------------------------
void aplicarEstado(uint8_t estado) {
  switch (estado) {
    case 0:
      desativarAlarme();
      Serial.println("[TX] OK — silencio, LED OFF");
      break;
    case 1:
      ativarAlarme(FREQ_CO, BEEP_DURACAO_CO, BEEP_PAUSA_CO);
      Serial.println("[TX] CO/Gas! — 700Hz lento, LED pisca lento");
      break;
    case 2:
      ativarAlarme(FREQ_FOGO, BEEP_DURACAO_FOGO, BEEP_PAUSA_FOGO);
      Serial.println("[TX] FOGO! — 1400Hz rapido, LED pisca rapido");
      break;
    case 3:
      ativarAlarme(FREQ_FOGO, BEEP_DURACAO_FOGO, BEEP_PAUSA_FOGO);
      Serial.println("[TX] FOGO + CO! — 1400Hz rapido, LED pisca rapido");
      break;
    default:
      desativarAlarme();
      break;
  }
}

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CSN_PIN);

  if (!radio.begin() || !radio.isChipConnected()) {
    Serial.println("[TX] ERRO: nRF24 nao iniciou (SPI/cabos/alimentacao).");
    while (1) delay(1000);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);

  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();

  estadoAtual = 0;

  Serial.println("=================================");
  Serial.println("  TX Tiago — Buzzer + LED Alarme ");
  Serial.println("=================================");
  Serial.println("[TX] Pronto: a aguardar dados do Goncalo-GX...");
}

// -------------------------------------------------------
void loop() {
  if (radio.available()) {
    uint8_t v;
    while (radio.available()) radio.read(&v, sizeof(v));
    v = constrain(v, 0, 3);

    if (v != estadoAtual) {
      estadoAtual = v;
      Serial.print("[TX] RECEBEU estado="); Serial.println(estadoAtual);
      aplicarEstado(estadoAtual);
    }
  }

  atualizarAlarme();
  delay(5);
}