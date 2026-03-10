#include <ESP32Servo.h>
#include <Arduino.h>

Servo esc;

const int JOY_Y_PIN    = 6;
const int ESC_PIN      = 18;
const int BTN_ARM_PIN  = 4;
const int BTN_STOP_PIN = 5;

// Valores
const int THROTTLE_NORMAL = 1200;
const int THROTTLE_Y_LOW  = 1100;
const int THROTTLE_Y_HIGH = 1400;
const int THROTTLE_MIN    = 1000;

const int RAMP_STEP = 2;     // MEGA SUAVE: +2us por passo
const int RAMP_DELAY = 80;   // 80ms por passo (1000→1200 = 100 passos × 80ms = ~8s ULTRA LENTO!)

bool armed = false;
bool rampActive = false;
int currentThrottle = THROTTLE_MIN;
int targetThrottle = THROTTLE_MIN;

void setup() {
  Serial.begin(115200);
  
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  
  esc.writeMicroseconds(THROTTLE_MIN);
  delay(3000);
  
  pinMode(BTN_ARM_PIN, INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  
  Serial.println("EMBER - RAMP MEGA LENTO 8s");
}

void rampToTarget() {
  if (abs(currentThrottle - targetThrottle) > RAMP_STEP) {
    if (currentThrottle < targetThrottle) {
      currentThrottle += RAMP_STEP;
    } else {
      currentThrottle -= RAMP_STEP;
    }
    currentThrottle = constrain(currentThrottle, THROTTLE_MIN, 2000);
    esc.writeMicroseconds(currentThrottle);
  } else {
    currentThrottle = targetThrottle;
    esc.writeMicroseconds(currentThrottle);
    rampActive = false;
  }
}

void loop() {
  bool armBtn = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);
  
  static bool lastArm = false;
  
  // STOP imediato (único sem ramp)
  if (stopBtn) {
    armed = false;
    targetThrottle = THROTTLE_MIN;
    currentThrottle = THROTTLE_MIN;
    esc.writeMicroseconds(THROTTLE_MIN);
    Serial.println("STOP imediato");
    delay(300);
    return;
  }
  
  // ARM/DISARM MEGA GRADUAL (ON e OFF)
  if (armBtn && !lastArm) {
    armed = !armed;
    if (armed) {
      targetThrottle = THROTTLE_NORMAL;  // 1200
      Serial.println("ON - MEGA RAMP 1000->1200 (~8s)");
    } else {
      targetThrottle = THROTTLE_MIN;     // 1000
      Serial.println("OFF - MEGA RAMP 1200->1000 (~3s)");
    }
    rampActive = true;
    delay(300);
  }
  lastArm = armBtn;
  
  // Ramp MEGA LENTO
  if (rampActive) {
    rampToTarget();
  }
  
  // Joystick (após ramp)
  if (armed && !rampActive) {
    int y = analogRead(JOY_Y_PIN);
    
    int throttle;
    if (y <= 1000) {
      throttle = THROTTLE_Y_LOW;   // 1050
    } else if (y >= 3500) {
      throttle = THROTTLE_Y_HIGH;  // 1400
    } else {
      throttle = THROTTLE_NORMAL;  // 1200
    }
    
    targetThrottle = throttle;
    esc.writeMicroseconds(throttle);
    Serial.print("Y="); Serial.print(y); Serial.print(" us="); Serial.println(throttle);
  }
  
  delay(RAMP_DELAY);
}
