#include <Arduino.h>
#include <ESP32Servo.h>

const int ESC1_PIN = 38;
const int ESC2_PIN = 39;
const int ESC3_PIN = 40;
const int ESC4_PIN = 41;

Servo esc1, esc2, esc3, esc4;

void writeAll(int us) {
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);

  // ESP32Servo: setPeriodHertz antes do attach
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  // attach(pin, minUs, maxUs)
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);

  Serial.println("=== CALIBRACAO ESC HobbyKing 20A ===");
  Serial.println(">> LIGA A BATERIA AGORA <<");
  delay(5000);

  Serial.println("PASSO 1: A enviar maximo (2000us)...");
  writeAll(2000);
  delay(5000);

  Serial.println("PASSO 2: A enviar minimo (1000us)...");
  writeAll(1000);
  delay(5000);

  Serial.println("Calibracao concluida! Carrega o codigo principal.");
}

void loop() {}