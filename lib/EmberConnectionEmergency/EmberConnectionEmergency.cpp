// ============================================================
// EmberConnectionEmergency.cpp, Implementação
// ============================================================

#include "EmberConnectionEmergency.h"

void EmberConnectionEmergency::begin(bool enabled, OnEmergencyFn onEmergency) {
  _enabled     = enabled;
  _onEmergency = onEmergency;
  _triggered   = false;
  _wasOK       = true;
  Serial.print("[CONN-EMERGENCY] begin() enabled=");
  Serial.println(_enabled ? "true" : "false");
}

void EmberConnectionEmergency::setEnabled(bool enabled) {
  _enabled = enabled;
  if (!_enabled) {
    // Desligado: limpa qualquer disparo pendente, para nao ficar
    // "preso" em triggered=true quando o toggle voltar a ligar.
    _triggered = false;
    _wasOK     = true;
  }
  Serial.print("[CONN-EMERGENCY] setEnabled(");
  Serial.print(enabled ? "true" : "false");
  Serial.println(")");
}

void EmberConnectionEmergency::update(bool connectionOK) {
  if (!_enabled) return;

  if (!connectionOK && _wasOK) {
    // Falling edge: a ligacao acabou de cair -> dispara emergencia
    // uma unica vez (nao repete a cada loop enquanto continuar em falta).
    _triggered = true;
    Serial.println("[CONN-EMERGENCY] Ligacao perdida — EMERGENCY STOP disparado.");
    if (_onEmergency) _onEmergency();
  } else if (connectionOK && !_wasOK) {
    // Ligacao restabelecida: permite novo disparo se cair outra vez.
    _triggered = false;
    Serial.println("[CONN-EMERGENCY] Ligacao restabelecida.");
  }

  _wasOK = connectionOK;
}
