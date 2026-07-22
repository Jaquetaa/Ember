// ============================================================
// EmberJoystick, Header (Cabecalho)
//
// FICHA TECNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Classe (molde) que representa um joystick analogico de dois
// eixos ligado ao ESP32-S3. Sao criados dois objetos a partir
// deste molde: o joystick esquerdo, que le throttle (forca
// dos motores) e yaw (rotacao vertical), e o joystick direito,
// que le pitch (inclinacao frente/tras) e roll (inclinacao
// lateral). Existe no projeto para encapsular a logica de
// leitura ADC e a conversao de valores brutos em comandos
// prontos a incluir no payload (conjunto de dados uteis)
// enviado ao drone.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. O construtor EmberJoystick(pinY, pinX) e chamado em
//    Tiago-Ember.ino durante a inicializacao dos objetos
//    globais, guardando os numeros dos pinos.
// 2. begin() e chamado em setup(): faz a calibracao do ponto
//    de repouso real de cada eixo (media de varias leituras),
//    porque joysticks analogicos baratos raramente repousam
//    exatamente a meio da escala do ADC (2048 em 0-4095).
//    Os sticks devem estar largados/centrados durante o boot.
// 3. Em cada iteracao de loop(), uma das quatro funcoes de
//    leitura e chamada: readThrottle(), readYaw(),
//    readPitch() ou readRoll().
// 4. Cada funcao de leitura: le o valor ADC do pino (0-4095) e
//    mapeia-o proporcionalmente para o intervalo de saida,
//    usando o centro calibrado em begin() e uma zona morta
//    pequena a volta desse centro para absorver o ruido do ADC.
//
// PINOS
// -----------------------------------------------------------
//   GPIO = General Purpose Input/Output, pino de uso geral
//   Joystick esquerdo: Y=GPIO4 (throttle), X=GPIO5 (yaw)
//   Joystick direito:  Y=GPIO2 (pitch),    X=GPIO1 (roll)
//
// VALORES DE RETORNO
// -----------------------------------------------------------
//   readThrottle: microssegundos (1000-1500)
//     Protocolo PWM (Pulse Width Modulation, modulacao por
//     largura de pulso) usado pelos ESCs (Electronic Speed
//     Controller, controlador eletronico de velocidade dos
//     motores do drone)
//   readYaw:   -100 a +100 (percentagem de rotacao)
//   readPitch: -100 a +100 (percentagem de inclinacao)
//   readRoll:  -100 a +100 (percentagem de inclinacao)
// ============================================================

#ifndef EMBER_JOYSTICK_H
#define EMBER_JOYSTICK_H

#include <Arduino.h>


// -----------------------------------
// Classe EmberJoystick
// -----------------------------------

class EmberJoystick {
public:
  // Construtor: recebe os pinos dos dois eixos como argumentos.
  EmberJoystick(uint8_t pinY, uint8_t pinX);

  void begin();          // calibra o centro de repouso real de cada eixo
  int readThrottle();    // le eixo Y e devolve microssegundos (1000-1500), proporcional
  int readYaw();         // le eixo X e devolve -100 a +100, proporcional
  int readPitch();       // le eixo Y e devolve -100 a +100 (joystick direito), proporcional
  int readRoll();        // le eixo X e devolve -100 a +100 (joystick direito), proporcional

  // Leituras "normalizadas" so para debug/serial: 0 no centro calibrado,
  // sobem ate 4095 conforme o stick se afasta do centro. Sao unidirecionais
  // (o lado oposto ao centro fica agarrado a 0) porque servem apenas para
  // ser faceis de ler no monitor serie, NAO substituem readThrottle/readYaw/
  // readPitch/readRoll, que continuam bidirecionais (-100 a +100 etc.).
  int readNormY();
  int readNormX();

private:
  uint8_t _pinY;   // pino do eixo vertical guardado internamente
  uint8_t _pinX;   // pino do eixo horizontal guardado internamente
  int     _centerY; // valor ADC medido em repouso no eixo Y (calibrado em begin())
  int     _centerX; // valor ADC medido em repouso no eixo X (calibrado em begin())
  float   _filtY;   // valor filtrado (media exponencial) do eixo Y, reduz ruido/glitches
  float   _filtX;   // valor filtrado (media exponencial) do eixo X, reduz ruido/glitches

  // Le o ADC de um pino e aplica um filtro de media exponencial (EMA) para
  // suavizar picos de ruido ou mau contacto eletrico (ex: fio solto).
  int filteredRead(uint8_t pin, float &filt);

  // Mapeia uma leitura ADC bruta para o intervalo de saida pretendido,
  // usando o centro calibrado e uma zona morta para ignorar ruido.
  int mapAxis(int raw, int center, int outMin, int outCenter, int outMax);
};


// -----------------------------------
// Constantes Partilhadas
// -----------------------------------
// Usadas tambem no ficheiro principal Tiago-Ember.ino.
// Os valores em microssegundos correspondem ao protocolo PWM
// (Pulse Width Modulation) dos ESCs dos motores do drone.

#define THROTTLE_NORMAL 1300   // throttle central, velocidade de cruzeiro
#define THROTTLE_Y_LOW  1200   // joystick empurrado para baixo
#define THROTTLE_Y_HIGH 1500   // joystick empurrado para cima (maximo)
#define THROTTLE_MIN    1000   // throttle minimo absoluto, motores parados
#define YAW_MAX         100    // valor maximo de yaw, pitch e roll em percentagem

// -----------------------------------
// Constantes de Leitura ADC
// -----------------------------------
// ADC_DEADZONE: largura da zona morta a volta do centro calibrado,
// em contagens ADC (0-4095). Absorve o ruido normal do ADC do ESP32
// e a folga mecanica do joystick perto do repouso, sem criar uma
// zona morta gigante como acontecia com os limiares fixos antigos.
// CALIBRATION_SAMPLES: numero de leituras usadas em begin() para
// calcular a media do ponto de repouso de cada eixo.

#define ADC_DEADZONE         120
#define CALIBRATION_SAMPLES  20

// NORM_THRESHOLD: ponto de referencia "0" fixo usado so pelas leituras
// normalizadas de debug (readNormY/readNormX), com zona morta
// ADC_DEADZONE a volta dele. Abaixo desce ate -4095, acima sobe ate
// +4095. Fixo (em vez do centro calibrado por eixo) para dar um ponto
// de comparacao igual entre os 4 canais no monitor serie.
#define NORM_THRESHOLD       2000

#endif
