// ============================================================
// EmberBuzzer, Implementacao
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementacao do controlo do buzzer passivo e LEDs de
// alarme. Interpreta o byte ACK (Acknowledgement, confirmacao
// de rececao) recebido do drone e produz o padrao sonoro e
// visual correto para cada situacao: calibracao dos ESCs
// (Electronic Speed Controller, controlador eletronico de
// velocidade dos motores) ou alarme de sensor. Existe no
// projeto para encapsular toda a logica de alertas, impedindo
// que o loop() principal fique sobrecarregado com codigo de
// temporizacao e controlo de hardware sonoro.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. begin() e chamado uma vez em setup():
//    Passo 1: guarda os pinos nos atributos privados.
//    Passo 2: configura pinos dos LEDs como OUTPUT e apaga-os.
//    Passo 3: configura o canal LEDC (LED Controller, controlador
//             PWM interno do ESP32) com ledcSetup() a 1000 Hz e
//             8 bits de resolucao (duty cycle, ciclo de trabalho),
//             e associa-o ao pino do buzzer com ledcAttachPin().
//    Passo 4: escreve duty cycle 0 para silencio inicial.
// 2. update(ackByte) e chamado em cada iteracao de loop():
//    Passo 1: extrai calPhase (fase de calibracao) com >> 2
//             e & 0x07 (mascara de 3 bits).
//    Passo 2: extrai estado do sensor com & 0x03 (2 bits).
//    Passo 3: se calPhase != 0, chama _driveCalibration().
//    Passo 4: caso contrario, chama _driveSensor().
// 3. _driveCalibration() verifica se a fase mudou, reinicia
//    a maquina de beep se necessario, e executa o padrao
//    sonoro da fase atual (switch/case 1 a 7).
// 4. _calBeep() e o motor do beep pulsado: compara millis()
//    com o ultimo toggle (alternancia) e inverte o estado
//    ON/OFF sem usar delay().
//
// SEQUENCIA DE CALIBRACAO DOS ESCs
// -----------------------------------------------------------
//   Fase 1: ramp down (descida gradual), LEDs ligados, sem som
//   Fase 2: 25 beeps rapidos (30ms ligado / 30ms desligado)
//   Fase 3: silencio
//   Fase 4: 5 ticks lentos (300ms ligado / 700ms desligado)
//   Fase 5: silencio (piloto deve desligar e ligar a bateria)
//   Fase 6: beeps freneticos durante varrimento MAX para MIN
//   Fase 7: silencio pos-calibracao, volta ao estado normal
//
// ALARMES DE SENSOR (funcionamento normal)
// -----------------------------------------------------------
//   Estado 0: sem som, LEDs apagados (tudo OK)
//   Estado 1: 600 Hz continuo, LED amarelo (CO detetado)
//   Estado 2: 1300 Hz continuo, LED vermelho (chama detetada)
//   Estado 3: 1300 Hz continuo, ambos os LEDs (CO mais chama)
//
// Hz = Hertz, unidade de frequencia (ciclos por segundo)
// ms = milissegundos
// ============================================================

#include "EmberBuzzer.h"


// -----------------------------------
// Frequencias dos Tons
// -----------------------------------

#define FREQ_CO    600   // Hz, tom grave para gas CO (monoxido de carbono)
#define FREQ_FIRE  1300  // Hz, tom agudo para chama
#define FREQ_CAL   1500  // Hz, tom mais agudo para calibracao


// -----------------------------------
// Begin (Inicializacao)
// -----------------------------------
// Passo 1: guarda os pinos nos atributos privados
// Passo 2: configura os pinos dos LEDs como saida e apaga-os
// Passo 3: associa o canal LEDC (PWM interno) ao pino do buzzer
// Passo 4: coloca o buzzer em silencio (duty cycle = 0)

void EmberBuzzer::begin(uint8_t buzzPin, uint8_t ledGas, uint8_t ledFire, uint8_t ch) {
  _buzz = buzzPin; _ledGas = ledGas; _ledFire = ledFire; _ch = ch;

  pinMode(_ledGas,  OUTPUT); digitalWrite(_ledGas,  LOW);  // LED gas apagado
  pinMode(_ledFire, OUTPUT); digitalWrite(_ledFire, LOW);  // LED chama apagado

  // ledcSetup + ledcAttachPin: configura o canal PWM (frequencia base 1000 Hz,
  // resolucao 8 bits) e associa-o ao pino do buzzer. Nesta versao do core
  // Arduino-ESP32 (2.x) o canal e um numero separado do pino: ledcWrite()
  // e ledcWriteTone() recebem sempre o canal (_ch), nunca o pino (_buzz).
  ledcSetup(_ch, 1000, 8);
  ledcAttachPin(_buzz, _ch);
  ledcWrite(_ch, 0);  // duty cycle 0 = sem sinal = buzzer silencioso
  _currentFreq = 0;
}


// -----------------------------------
// Ligar o Buzzer a uma Frequencia
// -----------------------------------
// So chama ledcWriteTone se a frequencia for diferente da atual.
// Evita chamadas redundantes porque o loop() corre centenas de
// vezes por segundo.

void EmberBuzzer::_buzzOn(uint16_t hz) {
  if (hz != _currentFreq) {
    _currentFreq = hz;
    ledcWriteTone(_ch, hz);  // gera sinal PWM na frequencia especificada (por canal, nao por pino)
  }
}


// -----------------------------------
// Desligar o Buzzer
// -----------------------------------
// Coloca o duty cycle a 0 (silencio) sem desligar o canal PWM.

void EmberBuzzer::_buzzOff() {
  if (_currentFreq != 0) {
    _currentFreq = 0;
    ledcWrite(_ch, 0);  // por canal, nao por pino
  }
}


// -----------------------------------
// Controlar os LEDs
// -----------------------------------

void EmberBuzzer::_ledsSet(bool gas, bool fire) {
  digitalWrite(_ledGas,  gas  ? HIGH : LOW);  // LED amarelo (gas)
  digitalWrite(_ledFire, fire ? HIGH : LOW);  // LED vermelho (chama)
}


// -----------------------------------
// Beep Pulsado sem Bloqueio (Non-Blocking)
// -----------------------------------
// Gera um padrao ON/OFF sem usar delay().
// Funciona como um "pisca-pisca" temporizado:
//   Passo 1: calcula quanto tempo passou desde o ultimo toggle
//   Passo 2: verifica se o intervalo atual (ON ou OFF) ja terminou
//   Passo 3: se sim, alterna o estado e atualiza o buzzer e LEDs

void EmberBuzzer::_calBeep(uint32_t durOn, uint32_t durOff, uint16_t freq, uint32_t now) {
  uint32_t elapsed  = now - _calLastBeep;
  uint32_t interval = _calBeepOn ? durOn : durOff;  // qual o intervalo atual?
  if (elapsed >= interval) {
    _calLastBeep = now;
    _calBeepOn   = !_calBeepOn;  // alterna entre ligado e desligado
    if (_calBeepOn) { _buzzOn(freq); _ledsSet(true, true); }
    else            { _buzzOff();    _ledsSet(false, false); }
  }
}


// -----------------------------------
// Padroes Sonoros de Calibracao
// -----------------------------------
// Cada fase tem um ritmo diferente para o piloto acompanhar
// o progresso da calibracao sem olhar para o ecra.
// Passo 1: verifica se a fase mudou e reinicia a maquina de beep
// Passo 2: executa o padrao sonoro da fase atual

void EmberBuzzer::_driveCalibration(uint8_t calPhase, uint32_t now) {

  // Passo 1: quando a fase muda, reinicia a maquina de beep para
  // comecar imediatamente o novo padrao sonoro
  if (calPhase != _lastCalPhase) {
    _lastCalPhase = calPhase;
    _calBeepOn    = false;
    _calLastBeep  = now - 10000UL;  // subtrai 10s para forcar arranque imediato
  }

  // Passo 2: executa o padrao da fase atual
  switch (calPhase) {
    case 1: // ramp down (descida gradual dos motores), LEDs ligados sem som
      _buzzOff();
      _ledsSet(true, true);
      break;

    case 2: // 25 beeps rapidos para alertar que a calibracao comeca
      _calBeep(30, 30, FREQ_CAL, now);  // 30ms ligado / 30ms desligado
      break;

    case 3: // pausa silenciosa entre o aviso e a contagem decrescente
      _buzzOff(); _ledsSet(false, false);
      break;

    case 4: // 5 ticks lentos (como uma contagem decrescente: 5, 4, 3, 2, 1)
      _calBeep(300, 700, FREQ_CAL, now);  // 300ms ligado / 700ms desligado
      break;

    case 5: // silencio enquanto o piloto desliga e liga a bateria dos ESCs
      _buzzOff(); _ledsSet(false, false);
      break;

    case 6: // beeps freneticos durante o varrimento de sinal MAX para MIN
      _calBeep(40, 30, FREQ_CAL, now);  // 40ms ligado / 30ms desligado
      break;

    case 7: // silencio final, o drone vai regressar ao estado normal
      _buzzOff(); _ledsSet(false, false);
      break;

    default:
      _buzzOff(); _ledsSet(false, false);
      break;
  }
}


// -----------------------------------
// Alarmes de Sensor (Funcionamento Normal)
// -----------------------------------
// Sons continuos (nao pulsados) para cada tipo de perigo.
// Tom diferente para gas e para chama, para o piloto distinguir
// o tipo de alerta sem olhar para o ecra.

void EmberBuzzer::_driveSensor(uint8_t estado) {
  switch (estado) {
    case 0: _buzzOff();          _ledsSet(false, false); break;  // tudo OK, sem alarme
    case 1: _buzzOn(FREQ_CO);    _ledsSet(true,  false); break;  // CO, tom grave, LED amarelo
    case 2: _buzzOn(FREQ_FIRE);  _ledsSet(false, true);  break;  // fogo, tom agudo, LED vermelho
    case 3: _buzzOn(FREQ_FIRE);  _ledsSet(true,  true);  break;  // CO mais fogo, tom agudo, ambos LEDs
  }
}


// -----------------------------------
// Update (Atualizacao, chamar no loop)
// -----------------------------------
// Passo 1: extrai a fase de calibracao dos bits[4:2] do ackByte
// Passo 2: extrai o estado do sensor dos bits[1:0] do ackByte
// Passo 3: se estiver em calibracao, chama _driveCalibration()
//          (calibracao tem prioridade sobre alarmes de sensor)
// Passo 4: senao, chama _driveSensor() para alarmes normais

void EmberBuzzer::update(uint8_t ackByte) {
  uint8_t calPhase = (ackByte >> 2) & 0x07;  // >> 2 = desloca 2 bits, & 0x07 = mantem 3 bits
  uint8_t estado   = ackByte & 0x03;           // & 0x03 = mantem apenas os 2 bits mais baixos
  uint32_t now     = millis();                 // millis() = ms desde que o controlador ligou

  if (calPhase != 0) {
    // Passo 3: em calibracao, reinicia o estado se acabou de entrar
    if (_lastCalPhase == 0) {
      _calBeepOn   = false;
      _calLastBeep = now - 10000UL;
    }
    _driveCalibration(calPhase, now);
  } else {
    // Passo 4: voltou ao estado normal, limpa o estado de calibracao
    if (_lastCalPhase != 0) _lastCalPhase = 0;
    _driveSensor(estado);
  }
}
