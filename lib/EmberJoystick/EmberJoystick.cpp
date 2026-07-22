// ============================================================
// EmberJoystick, Implementacao
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Implementacao das leituras dos dois eixos de cada joystick
// analogico. Converte o valor bruto do ADC (Analog to Digital
// Converter, conversor analogico-digital) em comandos de
// controlo prontos a ser enviados ao drone. Existe no projeto
// para isolar a logica de leitura e conversao do joystick,
// mantendo o ficheiro principal limpo e focado na logica
// de controlo de alto nivel.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor e chamado em Tiago-Ember.ino, guardando
//    os pinos Y e X nos atributos privados _pinY e _pinX
//    atraves da lista de inicializacao do C++.
// 2. begin() e chamado em setup(): faz a media de varias
//    leituras ADC em repouso e guarda o resultado como o
//    centro real de cada eixo (_centerY, _centerX). Isto
//    corrige a tolerancia de fabrico do joystick, que raramente
//    repousa exatamente a meio da escala do ADC (2048 em 0-4095).
// 3. Em cada iteracao de loop(), uma das quatro funcoes de
//    leitura e invocada:
//    Passo 1: analogRead() le o valor ADC do pino (0 a 4095).
//    Passo 2: mapAxis() usa o centro calibrado e uma zona morta
//    pequena a volta dele para converter o valor bruto num
//    valor de saida proporcional ao deslocamento real do stick.
//
// Antes desta versao, a leitura comparava o valor bruto com
// limiares fixos (1000 e 3500) e so devolvia 3 posicoes possiveis
// (minimo, centro, maximo). Isso criava uma zona morta enorme
// (tudo entre 1001 e 3499 virava "centro") e nao tinha em conta
// que o repouso real do joystick pode estar longe de 2048,
// como por exemplo em 1900. Agora a leitura e proporcional ao
// deslocamento do stick a partir do seu proprio centro medido.
// ============================================================

#include "EmberJoystick.h"


// -----------------------------------
// Construtor
// -----------------------------------
// Guarda os numeros dos pinos nos atributos privados da classe.
// O : na linha seguinte e a lista de inicializacao, uma forma
// de definir atributos privados antes do corpo da funcao.
// _centerY/_centerX comecam no meio teorico da escala do ADC
// (2048) e sao substituidos pelo valor real assim que begin()
// fizer a calibracao.

EmberJoystick::EmberJoystick(uint8_t pinY, uint8_t pinX)
  : _pinY(pinY), _pinX(pinX), _centerY(2048), _centerX(2048),
    _filtY(2048), _filtX(2048) {}


// -----------------------------------
// Begin (Calibracao do Centro de Repouso)
// -----------------------------------
// Le o ADC varias vezes seguidas (CALIBRATION_SAMPLES) com um
// pequeno atraso entre leituras e guarda a media como o centro
// real de cada eixo. O joystick deve estar largado/centrado
// durante o arranque para a calibracao ser valida.
// O filtro (_filtY/_filtX) arranca a partir do mesmo centro
// calibrado, em vez de 0, para nao ter de "subir" lentamente
// nas primeiras leituras depois do boot.

void EmberJoystick::begin() {
  long sumY = 0;
  long sumX = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sumY += analogRead(_pinY);
    sumX += analogRead(_pinX);
    delay(2);
  }
  _centerY = sumY / CALIBRATION_SAMPLES;
  _centerX = sumX / CALIBRATION_SAMPLES;
  _filtY = _centerY;
  _filtX = _centerX;
}


// -----------------------------------
// Leitura Filtrada (Media Exponencial)
// -----------------------------------
// Um pino com mau contacto eletrico (fio solto, solda fria) ou
// exposto a ruido eletrico pode saltar centenas de contagens ADC
// sem o stick se mexer. Este filtro (EMA, Exponential Moving
// Average) suaviza esses saltos: cada leitura nova so desloca o
// valor filtrado uma fracao (ALPHA) da diferenca, em vez de
// aceitar o valor bruto de uma so vez. Reduz o impacto de ruido
// mantendo resposta rapida o suficiente para controlo em tempo real.

#define EMA_ALPHA 0.3f

int EmberJoystick::filteredRead(uint8_t pin, float &filt) {
  int raw = analogRead(pin);
  filt += EMA_ALPHA * (raw - filt);
  return (int)(filt + 0.5f);
}


// -----------------------------------
// Mapeamento Proporcional de um Eixo
// -----------------------------------
// Converte uma leitura ADC bruta (0-4095) num valor de saida
// entre outMin e outMax, proporcional a distancia do centro
// calibrado. Dentro da zona morta (ADC_DEADZONE contagens para
// cada lado do centro) devolve sempre outCenter, para ignorar
// o ruido normal do ADC e a folga mecanica do stick em repouso.

int EmberJoystick::mapAxis(int raw, int center, int outMin, int outCenter, int outMax) {
  raw = constrain(raw, 0, 4095);
  int diff = raw - center;

  if (abs(diff) < ADC_DEADZONE) return outCenter;

  if (diff < 0) {
    int lowEdge = center - ADC_DEADZONE;
    if (lowEdge <= 0) return outCenter;  // calibracao invalida, evita divisao por zero
    return map(raw, 0, lowEdge, outMin, outCenter);
  } else {
    int highEdge = center + ADC_DEADZONE;
    if (highEdge >= 4095) return outCenter;  // calibracao invalida, evita divisao por zero
    return map(raw, highEdge, 4095, outCenter, outMax);
  }
}


// -----------------------------------
// Leitura do Throttle (Forca dos Motores)
// -----------------------------------
// Passo 1: le o eixo Y do joystick esquerdo (pino _pinY).
// Passo 2: mapeia proporcionalmente entre THROTTLE_MIN (stick
// todo para baixo) e THROTTLE_Y_HIGH (stick todo para cima),
// passando por THROTTLE_NORMAL no centro calibrado.

int EmberJoystick::readThrottle() {
  int y = filteredRead(_pinY, _filtY);
  return mapAxis(y, _centerY, THROTTLE_MIN, THROTTLE_NORMAL, THROTTLE_Y_HIGH);
}


// -----------------------------------
// Leitura do Yaw (Rotacao Vertical)
// -----------------------------------
// Passo 1: le o eixo X do joystick esquerdo (pino _pinX).
// Passo 2: mapeia proporcionalmente entre -YAW_MAX (esquerda)
// e +YAW_MAX (direita), passando por 0 no centro calibrado.

int EmberJoystick::readYaw() {
  int x = filteredRead(_pinX, _filtX);
  return mapAxis(x, _centerX, -YAW_MAX, 0, YAW_MAX);
}


// -----------------------------------
// Leitura do Pitch (Inclinacao Frente/Tras)
// -----------------------------------
// Passo 1: le o eixo Y do joystick direito (pino _pinY).
// Passo 2: mapeia proporcionalmente entre -YAW_MAX (para tras)
// e +YAW_MAX (para a frente), passando por 0 no centro calibrado.

int EmberJoystick::readPitch() {
  int y = filteredRead(_pinY, _filtY);
  return mapAxis(y, _centerY, -YAW_MAX, 0, YAW_MAX);
}


// -----------------------------------
// Leitura do Roll (Inclinacao Lateral)
// -----------------------------------
// Passo 1: le o eixo X do joystick direito (pino _pinX).
// Passo 2: mapeia proporcionalmente entre -YAW_MAX (esquerda)
// e +YAW_MAX (direita), passando por 0 no centro calibrado.

int EmberJoystick::readRoll() {
  int x = filteredRead(_pinX, _filtX);
  return mapAxis(x, _centerX, -YAW_MAX, 0, YAW_MAX);
}


// -----------------------------------
// Leituras Normalizadas (so para debug/serial)
// -----------------------------------
// Bidirecional: usa NORM_THRESHOLD (2000) como ponto de referencia "0",
// com uma zona morta (ADC_DEADZONE) a volta dele para absorver ruido.
// Abaixo da zona morta desce proporcionalmente ate -4095, acima sobe
// proporcionalmente ate +4095. Reaproveita mapAxis(), a mesma logica
// ja usada por readThrottle/readYaw/readPitch/readRoll, so que aqui a
// saida fica em contagens ADC (-4095 a +4095) em vez de microssegundos
// ou percentagem, para ser facil de ler no monitor serie.
//
// AVISO: pinos com mau contacto (ex: GPIO5/LX, que deriva sozinho entre
// ~400-1400 sem o stick se mexer) vao aparecer aqui como valores
// negativos falsos, porque esse ruido cai do lado negativo da zona
// morta. Nao ha forma de distinguir isso de um movimento real do stick
// so por filtragem de software: a leitura desse pino so fica fiavel
// depois de verificar a ligacao fisica (fio solto/solda fria).

int EmberJoystick::readNormY() {
  int y = filteredRead(_pinY, _filtY);
  return mapAxis(y, NORM_THRESHOLD, -4095, 0, 4095);
}

int EmberJoystick::readNormX() {
  int x = filteredRead(_pinX, _filtX);
  return mapAxis(x, NORM_THRESHOLD, -4095, 0, 4095);
}
