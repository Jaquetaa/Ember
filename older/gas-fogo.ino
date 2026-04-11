// ============================================
// Flame Sensor (Grove v1.1) + MQ-7 CO Sensor
//             + Piezo Buzzer Alarme
// ESP32-S3
// Flame SIG -> GPIO14 | MQ-7 AO -> GPIO13
// Piezo     -> GPIO12
// ============================================

#define FLAME_PIN       14
#define MQ7_PIN         13
#define BUZZER_PIN      12

#define FLAME_THRESHOLD 1000
#define CO_PPM_LIMITE   500
#define CONFIRMACOES    3
#define PREHEAT_MS      180000

#define FREQ_CO             700
#define FREQ_FOGO           1400
#define BEEP_DURACAO_CO     400   // ms a tocar — CO
#define BEEP_PAUSA_CO       400   // ms em silêncio — CO
#define BEEP_DURACAO_FOGO   150   // ms a tocar — FOGO (mais rápido)
#define BEEP_PAUSA_FOGO     80    // ms em silêncio — FOGO (quase sem pausa)

int contadorChama    = 0;
bool mq7Aquecido     = false;
unsigned long tempoInicio;

bool alarmeAtivo     = false;
int  alarmFreq       = 0;
int  beepDuracao     = 0;
int  beepPausa       = 0;
bool beepEstado      = false;
unsigned long ultimoBeep = 0;

void atualizarBuzzer() {
  if (!alarmeAtivo) { noTone(BUZZER_PIN); return; }
  unsigned long agora = millis();
  unsigned long intervalo = beepEstado ? beepDuracao : beepPausa;
  if (agora - ultimoBeep >= intervalo) {
    ultimoBeep = agora;
    beepEstado = !beepEstado;
    beepEstado ? tone(BUZZER_PIN, alarmFreq) : noTone(BUZZER_PIN);
  }
}

void ativarAlarme(int freq, int duracao, int pausa) {
  if (!alarmeAtivo || alarmFreq != freq) {
    alarmeAtivo  = true;
    alarmFreq    = freq;
    beepDuracao  = duracao;
    beepPausa    = pausa;
    beepEstado   = true;
    ultimoBeep   = millis();
    tone(BUZZER_PIN, freq);
  }
}

void desativarAlarme() {
  alarmeAtivo = false;
  noTone(BUZZER_PIN);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(BUZZER_PIN, OUTPUT);
  tempoInicio = millis();
  Serial.println("=================================");
  Serial.println(" Flame + CO + Buzzer — ESP32-S3");
  Serial.println("=================================");
  Serial.println("A aquecer sensor MQ-7 (3 min)...");
}

void loop() {
  if (!mq7Aquecido) {
    unsigned long decorrido = millis() - tempoInicio;
    if (decorrido < PREHEAT_MS) {
      Serial.print("MQ-7 a aquecer... faltam ");
      Serial.print((PREHEAT_MS - decorrido) / 1000);
      Serial.println("s");
      atualizarBuzzer();
      delay(5000);
      return;
    }
    mq7Aquecido = true;
    Serial.println("MQ-7 pronto!");
    Serial.println("---------------------------------");
  }

  // ---- Flame ----
  int flameVal = analogRead(FLAME_PIN);
  contadorChama = (flameVal < FLAME_THRESHOLD) ? contadorChama + 1 : 0;
  bool chama = (contadorChama >= CONFIRMACOES);

  // ---- CO / MQ-7 ----
  int   coVal  = analogRead(MQ7_PIN);
  float tensao = coVal * (3.3f / 4095.0f);
  float rs_ro  = (3.3f - tensao) / tensao;
  float ppm    = constrain(100.0f * pow(rs_ro / 5.0f, -1.5f), 0, 10000);
  bool coAlto  = (ppm > CO_PPM_LIMITE);

  // ---- Alarme (fogo tem prioridade) ----
  if (chama)
    ativarAlarme(FREQ_FOGO, BEEP_DURACAO_FOGO, BEEP_PAUSA_FOGO); // rápido e agressivo
  else if (coAlto)
    ativarAlarme(FREQ_CO, BEEP_DURACAO_CO, BEEP_PAUSA_CO);       // lento e grave
  else
    desativarAlarme();

  atualizarBuzzer();

  // ---- Serial ----
  Serial.print("FLAME: "); Serial.print(flameVal);
  Serial.print(" | Chama: "); Serial.print(chama ? "SIM 🔥" : "NAO");
  Serial.print(" || CO: "); Serial.print(ppm, 1);
  Serial.print(" ppm | ");
  if      (chama && coAlto) Serial.println("🚨 CHAMA + CO > 500ppm!");
  else if (chama)           Serial.println("🔥 CHAMA — 1400Hz rápido");
  else if (coAlto)          Serial.println("⚠️  CO > 500ppm — 700Hz lento");
  else                      Serial.println("OK");
  Serial.println("---------------------------------");

  delay(100);
}
