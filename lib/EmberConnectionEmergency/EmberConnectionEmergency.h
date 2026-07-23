// ============================================================
// EmberConnectionEmergency.h, Header
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Define a interface da classe EmberConnectionEmergency,
// responsável por vigiar o estado de uma ligação qualquer
// (câmara térmica, rádio de controlo, etc.) e disparar
// automaticamente uma paragem de emergência quando essa ligação
// cai. É genérica de propósito: quem a instancia decide, a cada
// update(), se a ligação vigiada está OK ou não (por exemplo,
// EmberCAMNRF::isConnected()) e qual a função a chamar em caso
// de perda (normalmente emergencyStop() do Goncalo-Ember.ino).
// A funcionalidade é ligável/desligável em runtime através de
// setEnabled(), para permitir o toggle true/false definido no
// .ino logo a seguir aos trims dos ESC.
//
// ORDEM DE EXECUÇÃO
// -----------------------------------------------------------
// 1. begin() em setup():
//    Regista se a monitorização começa ligada (enabled) e o
//    callback onEmergency a disparar em caso de perda de ligação.
// 2. update(connectionOK) em cada loop():
//    Se desligada (enabled=false), não faz nada.
//    Deteta a transição OK->perdida (falling edge) e dispara
//    onEmergency() uma única vez por queda. Deteta a transição
//    perdida->OK para permitir novo disparo se a ligação cair
//    outra vez mais tarde.
// ============================================================

#ifndef EMBER_CONNECTION_EMERGENCY_H
#define EMBER_CONNECTION_EMERGENCY_H

#include <Arduino.h>

class EmberConnectionEmergency {
public:
  using OnEmergencyFn = void (*)();

  // enabled: estado inicial do toggle (ver bool no .ino, logo abaixo dos trims).
  // onEmergency: chamado uma vez, na transição OK->perdida (deve fazer emergencyStop()).
  void begin(bool enabled, OnEmergencyFn onEmergency);

  // Liga/desliga a monitorização em runtime (toggle).
  void setEnabled(bool enabled);
  bool isEnabled() const { return _enabled; }

  // Chamar em todos os loops com o estado atual da ligação vigiada.
  void update(bool connectionOK);

  // true a partir do momento em que a ligação caiu, até restabelecer.
  bool isTriggered() const { return _triggered; }

private:
  bool _enabled    = true;
  bool _triggered  = false;
  bool _wasOK      = true;
  OnEmergencyFn _onEmergency = nullptr;
};

#endif
