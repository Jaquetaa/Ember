#include <Arduino.h>
#include <ESP32Servo.h>

// ── Pinos (ESP32-S3 DevModule) ─────────────────────────
const int JOY_Y_PIN    = 7;
const int JOY_X_PIN    = 6;
const int ESC1_PIN     = 38;
const int ESC2_PIN     = 39;
const int ESC3_PIN     = 40;
const int ESC4_PIN     = 41;
const int BTN_ARM_PIN  = 1;
const int BTN_STOP_PIN = 2;

// ── Throttle (HobbyKing 20A ESC — range 700-2000us) ────
const int THROTTLE_NORMAL = 1300;
const int THROTTLE_Y_LOW  = 1200;
const int THROTTLE_Y_HIGH = 1500;
const int THROTTLE_MIN    = 1000;
const int THROTTLE_ARM    = 1100;
const int YAW_MAX         = 100;

const int RAMP_STEP  = 2;
const int RAMP_DELAY = 40;

Servo esc1, esc2, esc3, esc4;

bool armed          = false;
bool rampActive     = false;
int currentThrottle = THROTTLE_MIN;
int targetThrottle  = THROTTLE_MIN;

// ── Funções de escrita ─────────────────────────────────
void writeAllESCs(int us) {
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

/*
  Layout:
    1(CW)  2(CCW)
    3(CCW) 4(CW)
*/
void writeESCsWithYaw(int base, int yaw) {
  int safeMin = armed ? THROTTLE_ARM + 50 : THROTTLE_MIN;
 
/*
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  a = constrain(base + yaw, safeMin, 2000);
  b = constrain(base - yaw, safeMin, 2000);
  c = constrain(base - yaw, safeMin, 2000);
  d = constrain(base + yaw, safeMin, 2000);
  Serial.print("\nESC1: "); Serial.print(a); Serial.print("    ");
  Serial.print("  ESC2: "); Serial.print(b); Serial.print("    ");
  Serial.print("  ESC3: "); Serial.print(c); Serial.print("    ");
  Serial.print("  ESC4: "); Serial.print(d); Serial.print("    ");
*/
  esc1.writeMicroseconds(constrain(base + yaw, safeMin, 2000));
  esc2.writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  esc3.writeMicroseconds(constrain(base - yaw, safeMin, 2000));
  esc4.writeMicroseconds(constrain(base + yaw, safeMin, 2000));
}

// ── Setup ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  esc1.attach(ESC1_PIN, 700, 2000);
  esc2.attach(ESC2_PIN, 700, 2000);
  esc3.attach(ESC3_PIN, 700, 2000);
  esc4.attach(ESC4_PIN, 700, 2000);

  writeAllESCs(THROTTLE_MIN);
  delay(100);

  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);

  Serial.println("\n\n\n\n");
  Serial.println("---- Debug Console -----");
  Serial.println("Ember (HobbyKing 20A | ESP32-S3 | ESP32Servo)");
  Serial.println("Pronto. Aguarda botao ARM.");
}

// ── Ramp ───────────────────────────────────────────────
void rampToTarget() {
  if (abs(currentThrottle - targetThrottle) > RAMP_STEP) {
    currentThrottle += (currentThrottle < targetThrottle) ? RAMP_STEP : -RAMP_STEP;
    currentThrottle = constrain(currentThrottle, THROTTLE_MIN, 2000);
    writeAllESCs(currentThrottle);
    Serial.print("Ramp -> "); Serial.println(currentThrottle);
  } else {
    currentThrottle = targetThrottle;
    writeAllESCs(currentThrottle);
    rampActive = false;
    Serial.println("Ramp concluido.");
  }
}

// ── Loop ───────────────────────────────────────────────
void loop() {
  bool armBtn  = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);
  static bool lastArm = false;

  if (stopBtn) {
    armed           = false;
    rampActive      = false;
    targetThrottle  = THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    writeAllESCs(THROTTLE_MIN);
    Serial.println("Emergency STOP");
    delay(300);
    return;
  }

  if (armBtn && !lastArm) {
    armed = !armed;
    targetThrottle  = armed ? THROTTLE_NORMAL : THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    rampActive      = true;
    Serial.println(armed ? "ARM: ON" : "ARM: OFF");
    delay(300);
  }
  lastArm = armBtn;

  if (rampActive) {
    rampToTarget();
    delay(RAMP_DELAY);
    return;
  }

  if (armed) {
    int y = analogRead(JOY_Y_PIN);
    int x = analogRead(JOY_X_PIN);

    int throttle;
    if      (y <= 1000) throttle = THROTTLE_Y_LOW;
    else if (y >= 3500) throttle = THROTTLE_Y_HIGH;
    else                throttle = THROTTLE_NORMAL;

    int yaw = 0;
    if      (x <= 1000) yaw = -YAW_MAX;
    else if (x >= 3500) yaw =  YAW_MAX;

    targetThrottle = throttle;
    writeESCsWithYaw(throttle, yaw);

    Serial.print("Y="); Serial.print(y);
    Serial.print(" X="); Serial.print(x);
    Serial.print(" T="); Serial.print(throttle);
    Serial.print(" Yaw="); Serial.println(yaw);
  }

  delay(RAMP_DELAY);
}
