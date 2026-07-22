// ============================================================
// EMBER, Tiago (Controlador de Chão), ESP32-S3
//
// FICHA TÉCNICA
// -----------------------------------------------------------
// OBJETIVO
// -----------------------------------------------------------
// Ficheiro principal do controlador de chão do drone Ember.
// Coordena todos os módulos: le os joysticks, envia comandos
// ao drone por radio, recebe o estado dos sensores na
// confirmacao de rececao, atualiza o buzzer e os LEDs de
// alarme, e mostra a imagem termica no ecra em tempo real.
// Existe no projeto como ponto de entrada unico que une
// todos os modulos da biblioteca Ember num ciclo continuo.
//
// ORDEM DE EXECUCAO
// -----------------------------------------------------------
// 1. setup() e chamado uma vez ao ligar: inicializa a porta
//    serie, configura os botoes com pull-up, forca os pinos
//    CS (Chip Select) a HIGH, inicializa os dois joysticks,
//    o buzzer, a retroiluminacao do ecra, o bus SPI (Serial
//    Peripheral Interface, protocolo de comunicacao serie
//    entre componentes), o ecra TFT, o radio de controlo
//    EmberNRFTX e o radio termico EmberDisplayRX.
// 2. loop() corre continuamente: le os botoes STOP e ARM,
//    aplica debounce (anti-tremor) a cada um, le os eixos
//    dos joysticks (ou forca neutro se desarmado), envia o
//    payload (conjunto de dados uteis) ao drone via nrfTX->send(),
//    extrai do byte ACK (Acknowledgement, confirmacao de
//    rececao) a fase de calibracao e o estado do sensor,
//    aplica debounce ao sensor, atualiza buzzer e LEDs com
//    buzzer.update(), atualiza o ecra termico com
//    displayRX->update(), e imprime debug serie a cada 1 s.
//
// HARDWARE LIGADO
// -----------------------------------------------------------
// Dois radios NRF24L01 (modelo de transcetor sem fios de
// 2.4 GHz) no mesmo bus SPI:
//
//   Controlo:  CE=14, CSN=13
//     Canal 76 (2.476 GHz), com confirmacao de rececao (ACK),
//     envia comandos ao drone
//
//   Termico:   CE=12, CSN=11
//     Canal 2 (2.402 GHz), sem confirmacao, recebe imagens
//     da camara termica
//
// Ecra TFT ST7796S (ecra LCD colorido de 480x320 pixels):
//   CS=41, DC=21, RST=42, BL=45 (retroiluminacao, backlight)
//
// Bus SPI partilhado pelos dois radios e pelo ecra:
//   SCK=48  (Serial Clock, relogio que cadencia as transmissoes)
//   MISO=16 (Master In Slave Out, dados que chegam ao controlador)
//   MOSI=47 (Master Out Slave In, dados que saem do controlador)
//
// Joystick esquerdo (throttle e yaw):  Y=GPIO4, X=GPIO5
// Joystick direito  (pitch e roll):    Y=GPIO2, X=GPIO1
//   GPIO = General Purpose Input/Output, pino de uso geral
//
// Botoes:  ARM=GPIO17 (armar/desarmar), STOP=GPIO18 (emergencia)
// Alarme:  Buzzer=GPIO10, LED gas=GPIO3, LED chama=GPIO20
//
// PAYLOAD ENVIADO AO DRONE (9 bytes, struct PayloadCtrl)
// -----------------------------------------------------------
//   armed:    0=desarmado, 1=armado, 2=paragem de emergencia
//   throttle: forca dos motores em microssegundos (1000-1500)
//             protocolo PWM (Pulse Width Modulation, modulacao
//             por largura de pulso) usado pelos ESCs
//             ESC = Electronic Speed Controller, controlador
//             eletronico de velocidade dos motores
//   yaw:      rotacao em torno do eixo vertical (-100 a +100)
//   pitch:    inclinacao frente/tras, stick direito Y (-100 a +100)
//   roll:     inclinacao esquerda/direita, stick direito X (-100 a +100)
//
// ACK RECEBIDO DO DRONE (1 byte)
// -----------------------------------------------------------
//   bits[1:0] = estado sensor  (0=OK, 1=CO, 2=Fogo, 3=Ambos)
//   bits[4:2] = fase de calibracao (0=normal, 1-7=a calibrar)
//   bits = digitos binarios (0 ou 1) que compoem o byte
//
// SERIAL DEBUG (Monitor de Diagnostico)
// -----------------------------------------------------------
//   Porta: COM12 (USB CDC, Communications Device Class,
//          classe de dispositivo de comunicacao USB nativo)
//   Baud:  115200 (velocidade de transmissao em bits por segundo)
// ============================================================


// -----------------------------------
// Imports (Bibliotecas Externas)
// -----------------------------------

#include <Arduino.h>   // base do framework Arduino
#include <SPI.h>       // protocolo SPI para comunicar com radios e ecra
#include <TFT_eSPI.h>  // biblioteca do ecra LCD colorido ST7796S

#include "EmberJoystick.h"    // leitura dos dois joysticks analogicos
#include "EmberBuzzer.h"      // buzzer e LEDs de alarme
#include "EmberNRFTX.h"       // radio de controlo: envia comandos e recebe ACK
#include "EmberDisplayRX.h"   // radio termico: recebe e desenha imagem da camara


// -----------------------------------
// Pinos do Bus SPI Partilhado
// -----------------------------------
// SCK, MISO e MOSI sao partilhados pelos dois radios e pelo ecra.
// Cada dispositivo tem o seu proprio pino CS/CSN para indicar
// quando e a sua vez de comunicar.

#define SPI_SCK   48   // Serial Clock, relogio do bus
#define SPI_MISO  16   // Master In Slave Out, dados recebidos
#define SPI_MOSI  47   // Master Out Slave In, dados enviados

#define BTN_ARM_PIN  17   // botao de armar/desarmar o drone
#define BTN_STOP_PIN 18   // botao de paragem de emergencia


// -----------------------------------
// Instancias dos Modulos
// -----------------------------------
// EmberJoystick recebe os pinos Y e X do joystick como argumentos.
// Y = eixo vertical, X = eixo horizontal.

// NOTA: os argumentos estavam trocados (Y e X invertidos) numa versao
// anterior, o que fazia o eixo vertical do stick controlar yaw/roll e
// o eixo horizontal controlar throttle/pitch. Corrigido para corresponder
// a fiacao real (ver comentario "HARDWARE LIGADO" no topo do ficheiro).
EmberJoystick joystick(4, 5);   // joystick esquerdo: Y=pino4 (throttle), X=pino5 (yaw)
EmberJoystick joystickR(2, 1);  // joystick direito:  Y=pino2 (pitch),    X=pino1 (roll)
EmberBuzzer   buzzer;           // buzzer e LEDs, pinos definidos em setup()

// Estes tres sao criados em setup() e nao aqui, porque o bus SPI
// ainda nao esta inicializado neste momento. Criar antes causaria crash.
EmberNRFTX*     nrfTX     = nullptr;   // TX = Transmitter, radio de controlo
EmberDisplayRX* displayRX = nullptr;   // RX = Receiver, radio termico mais ecra
TFT_eSPI*       tft       = nullptr;   // ecra LCD colorido


// -----------------------------------
// Fundo Degradê (Gradiente de Cor)
// -----------------------------------
// Percorre cada uma das 320 linhas horizontais do ecra e pinta-a
// com uma cor ligeiramente diferente, criando um gradiente visual
// de azul escuro no topo ate roxo escuro na base.

static void drawGradientBG(TFT_eSPI *t) {
  for (int y = 0; y < 320; y++) {
    int v = (y * 255) / 319;   // normaliza y para o intervalo 0-255
    // color565 = formato de cor RGB565 usado pelo ecra (5 bits vermelho,
    // 6 bits verde, 5 bits azul, total 16 bits por pixel)
    t->drawFastHLine(0, y, 480, t->color565(v / 4, 0, 70 + v / 10));
  }
}

// Cor de fundo aproximada na zona y=70, usada para apagar o texto
// ARM/DESARMADO sem ter de redesenhar o fundo inteiro.
#define COL_ARMBG_565 0x0248


// -----------------------------------
// Variaveis de Estado Global
// -----------------------------------
// Guardam o estado atual entre iteracoes do loop().

bool     nrfCtrlOK     = false;  // true se o radio de controlo esta operacional
bool     armed         = false;  // true = drone armado, false = desarmado
bool     lastArmBtn    = false;  // estado do botao ARM na iteracao anterior
bool     lastStopBtn   = false;  // estado do botao STOP na iteracao anterior
uint8_t  lastEstado    = 255;    // ultimo estado de sensor confirmado;
                                 // 255 garante reacao na primeira leitura real (0-3)
uint32_t lastArmPress  = 0;      // timestamp em ms da ultima pressao do ARM
uint32_t lastStopPress = 0;      // timestamp em ms da ultima pressao do STOP


// -----------------------------------
// Setup (Inicializacao)
// -----------------------------------
// Executado UMA VEZ quando o controlador liga.
// Configura todos os componentes de hardware antes de entrar no loop.

void setup() {

  // Passo 0: SEGURANCA DE BUS - antes de tudo o resto.
  // Forcamos CE a LOW e CSN a HIGH nos dois radios imediatamente apos reset,
  // antes do delay do USB. Se o CSN ficasse a flutuar LOW, o radio conduziria
  // o MISO partilhado e corromperia a leitura SPI do outro dispositivo.
  pinMode(14, OUTPUT); digitalWrite(14, LOW);   // CE NRF Controlo
  pinMode(13, OUTPUT); digitalWrite(13, HIGH);  // CSN NRF Controlo
  pinMode(12, OUTPUT); digitalWrite(12, LOW);   // CE NRF Termico
  pinMode(11, OUTPUT); digitalWrite(11, HIGH);  // CSN NRF Termico

  // Passo 1: abre a comunicacao serie com o computador a 115200 bps
  // (bps = bits por segundo, velocidade de transmissao de dados).
  // delay(1500) espera que a porta USB CDC estabilize antes de imprimir,
  // caso contrario as primeiras mensagens perdem-se.
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n\n========================================");
  Serial.println("[EMBER] Tiago, Controlador de Chao");
  Serial.println("[EMBER] ESP32-S3, Arduino-ESP32 3.x");
  Serial.println("========================================");
  Serial.println("[EMBER] NOTA: Serial ativo em COM12 (USB CDC)");
  Serial.println("[EMBER] Inicio do setup...");
  Serial.flush(); // pausa o programa ate que todos os dados enviados pela porta serie terminem de ser transmitidos

  // Passo 2: configura os botoes com resistencia pull-up interna.
  // INPUT_PULLUP mantem o pino em HIGH (1) quando o botao esta solto.
  // Quando o botao e premido, o pino vai a LOW (0).
  // Sem pull-up, o pino flutuaria aleatoriamente entre 0 e 1.
  Serial.println("[SETUP] Configurando botoes ARM (GPIO17) e STOP (GPIO18)...");
  pinMode(BTN_ARM_PIN,  INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  Serial.print("[SETUP]   ARM  (GPIO17) = ");
  Serial.println(digitalRead(BTN_ARM_PIN)  ? "HIGH (solto)" : "LOW (premido)");
  Serial.print("[SETUP]   STOP (GPIO18) = ");
  Serial.println(digitalRead(BTN_STOP_PIN) ? "HIGH (solto)" : "LOW (premido)");

  // Passo 3: forca todos os pinos CS (Chip Select) e CSN (Chip Select Not)
  // para HIGH antes de inicializar o bus SPI.
  // HIGH = dispositivo ignorado. Se ficassem a LOW, todos os dispositivos
  // tentariam comunicar ao mesmo tempo e causariam conflitos no bus.
  Serial.println("[SETUP] A forcar CS pins para HIGH antes de inicializar SPI...");
  pinMode(13, OUTPUT);  digitalWrite(13, HIGH);  // CSN do radio de controlo (NRF TX)
  Serial.println("[SETUP]   CSN NRF Controlo (GPIO13) = HIGH");
  pinMode(11, OUTPUT);  digitalWrite(11, HIGH);  // CSN do radio termico (NRF RX)
  Serial.println("[SETUP]   CSN NRF Termico  (GPIO11) = HIGH");
  pinMode(41, OUTPUT);  digitalWrite(41, HIGH);  // CS do ecra TFT
  Serial.println("[SETUP]   CS  TFT           (GPIO41) = HIGH");
  delay(10);

  // Passo 4: inicializa o joystick esquerdo (throttle e yaw).
  // begin() calibra o centro de repouso real de cada eixo fazendo a
  // media de varias leituras ADC. Os sticks devem estar LARGADOS/
  // CENTRADOS durante o arranque, caso contrario a calibracao fica errada.
  Serial.println("[SETUP] A inicializar joystick ESQUERDO (Y=GPIO4, X=GPIO5)...");
  Serial.println("[SETUP]   >>> Mantem os sticks LARGADOS durante a calibracao <<<");
  joystick.begin();
  Serial.print("[SETUP]   Raw Y (throttle): "); Serial.println(analogRead(4));
  Serial.print("[SETUP]   Raw X (yaw):      "); Serial.println(analogRead(5));
  Serial.println("[SETUP]   Joystick esquerdo OK, centro calibrado");

  // Passo 5: inicializa o joystick direito (pitch e roll).
  Serial.println("[SETUP] A inicializar joystick DIREITO (Y=GPIO2, X=GPIO1)...");
  joystickR.begin();
  Serial.print("[SETUP]   Raw Y (pitch): "); Serial.println(analogRead(2));
  Serial.print("[SETUP]   Raw X (roll):  "); Serial.println(analogRead(1));
  Serial.println("[SETUP]   Joystick direito OK, centro calibrado");

  // Passo 6: inicializa o buzzer e os LEDs de alarme.
  // Argumentos: pino do buzzer (10), LED gas (3), LED chama (20).
  Serial.println("[SETUP] A inicializar buzzer (buzz=GPIO10, ledGas=GPIO3, ledFogo=GPIO20)...");
  buzzer.begin(10, 3, 20);
  Serial.println("[SETUP]   Buzzer OK");

  // Passo 7: ativa a retroiluminacao (backlight) do ecra.
  // Sem HIGH neste pino, o ecra fica escuro mesmo com tudo a funcionar.
  Serial.println("[SETUP] A ativar retroiluminacao TFT (GPIO45 = HIGH)...");
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);
  Serial.println("[SETUP]   Retroiluminacao ON");

  // Passo 8: inicializa o bus SPI com os tres pinos partilhados.
  // O argumento -1 no ultimo parametro significa "sem CS global",
  // cada dispositivo controla o seu proprio pino CS/CSN.
  Serial.println("[SETUP] A inicializar bus SPI2 (SCK=48, MISO=16, MOSI=47)...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
  delay(10);
  Serial.println("[SETUP]   SPI2 bus OK");

  // Passo 9: cria e inicializa o ecra TFT.
  // Criado com 'new' (alocacao dinamica de memoria) para garantir
  // que o SPI ja esta pronto. setRotation(1) = modo landscape (horizontal).
  Serial.println("[SETUP] A criar objeto TFT_eSPI (ST7796S)...");
  tft = new TFT_eSPI();
  Serial.println("[SETUP]   Objeto TFT criado");

  Serial.println("[SETUP] A chamar tft->init()...");
  tft->init();
  Serial.println("[SETUP]   tft->init() concluido sem crash!");

  Serial.println("[SETUP] A configurar rotacao, cor e cursor...");
  tft->setRotation(1);  // modo landscape (horizontal, 480 de largura)

  // IMPORTANTE: o construtor de TFT_eSPI nunca inicializa o membro gfxFont
  // (usado só para fontes GFX personalizadas, que este projeto não usa).
  // Como tft foi criado com 'new' (heap, não estático), esse ponteiro fica
  // com lixo em vez de NULL. write() verifica "if (!gfxFont)" para decidir
  // se desenha com as fontes embutidas (LOAD_FONT2/4/6/7/8) ou com uma
  // fonte GFX customizada; com lixo no ponteiro ele tenta ler uma fonte
  // GFX inexistente e crasha (LoadProhibited) no primeiro tft->print().
  // setTextFont(1) força gfxFont = NULL e evita o crash.
  tft->setTextFont(1);

  drawGradientBG(tft);  // desenha o fundo gradiente
  tft->setTextSize(2);
  tft->setTextColor(TFT_GREEN);
  tft->setCursor(50, 120);
  Serial.println("[SETUP]   Ecra preenchido, cursor posicionado");

  tft->print("EMBER - A inicializar...");
  Serial.println("[SETUP]   >>> TFT OK <<<");

  // Passo 10: cria e inicializa o radio de controlo (NRF TX).
  // CE = Chip Enable, ativa a transmissao.
  // CSN = Chip Select Not, seleciona este radio no bus SPI.
  Serial.println("[SETUP] A criar EmberNRFTX (CE=GPIO14, CSN=GPIO13)...");
  nrfTX = new EmberNRFTX(14, 13);
  Serial.println("[SETUP]   Objeto EmberNRFTX criado");

  Serial.println("[SETUP] A chamar nrfTX->begin(SPI)...");
  nrfCtrlOK = nrfTX->begin(SPI);
  if (nrfCtrlOK) {
    Serial.println("[SETUP]   >>> NRF TX OK: canal=76, 250KBPS, ACK <<<");
  } else {
    // Se falhar, o programa continua mas avisa o piloto no ecra a vermelho.
    Serial.println("[SETUP]   !!! NRF TX FALHOU, verifica ligacoes CE/CSN/VCC !!!");
    Serial.println("[SETUP]       CE=GPIO14, CSN=GPIO13, VCC=3.3V");
    Serial.println("[SETUP]       MOSI=47, MISO=16, SCK=48");
    tft->setCursor(0, 160);
    tft->setTextColor(TFT_RED);
    tft->print("NRF TX: FALHOU!");
    tft->setTextColor(TFT_GREEN);
  }


  // Passo 11: cria e inicializa o radio termico (NRF RX).
  // Recebe o objeto *tft por referencia porque e ele que vai
  // chamar as funcoes de desenho no ecra.
  Serial.println("[SETUP] A criar EmberDisplayRX (CE=GPIO12, CSN=GPIO11)...");
  displayRX = new EmberDisplayRX(12, 11);
  Serial.println("[SETUP]   Objeto EmberDisplayRX criado");

  Serial.println("[SETUP] A chamar displayRX->begin(SPI, *tft)...");
  displayRX->begin(SPI, *tft);
  Serial.println("[SETUP]   displayRX->begin() concluido");

  // Passo 12: leituras iniciais dos joysticks para confirmar que respondem.
  Serial.println("[SETUP] Leituras iniciais dos joysticks:");
  Serial.print("[SETUP]   Throttle (Y/GPIO4) = "); Serial.println(joystick.readThrottle());
  Serial.print("[SETUP]   Yaw      (X/GPIO5) = "); Serial.println(joystick.readYaw());
  Serial.print("[SETUP]   Pitch    (Y/GPIO2) = "); Serial.println(joystickR.readPitch());
  Serial.print("[SETUP]   Roll     (X/GPIO1) = "); Serial.println(joystickR.readRoll());

  Serial.print("[SETUP] Botao ARM  no arranque: ");
  Serial.println(!digitalRead(BTN_ARM_PIN)  ? "PREMIDO" : "solto");
  Serial.print("[SETUP] Botao STOP no arranque: ");
  Serial.println(!digitalRead(BTN_STOP_PIN) ? "PREMIDO" : "solto");

  // Passo 13: redesenha o ecra com o estado final de arranque.
  // Verde = componente OK, Vermelho = componente falhou.
  drawGradientBG(tft);
  tft->setTextSize(2);
  tft->setTextColor(TFT_GREEN);
  tft->setCursor(10, 10);
  tft->print("EMBER OK");
  tft->setCursor(10, 40);
  tft->setTextColor(nrfCtrlOK ? TFT_GREEN : TFT_RED);
  tft->print(nrfCtrlOK ? "NRF TX: OK" : "NRF TX: FAIL");
  tft->setCursor(10, 70);
  tft->setTextColor(TFT_CYAN);
  tft->print("DESARMADO");
  tft->setTextSize(2);
  tft->setTextColor(TFT_WHITE);
  tft->setCursor(5, 300);
  tft->print("A ESPERA DE LIGACAO...");

  Serial.println("\n========================================");
  Serial.println("[EMBER] === Setup completo! Loop a iniciar ===");
  Serial.println("========================================\n");
  Serial.flush();
}


// -----------------------------------
// Loop Principal
// -----------------------------------
// Executado CONTINUAMENTE enquanto o controlador estiver ligado.
// Nao usa delay() para nao bloquear a comunicacao com o drone.

void loop() {

  // -----------------------------------
  // Recuperacao do Radio de Controlo
  // -----------------------------------
  // Se o radio falhou no arranque (ou caiu), tenta reinicializar a cada 2 s.
  // Sem delay() bloqueante: usa millis() para nao atrasar o resto do loop.
  // Enquanto o radio nao recuperar, send() devolve false e nao ha envio.
  static unsigned long lastRadioRetry = 0;
  if (!nrfTX->isOK() && millis() - lastRadioRetry > 2000) {
    lastRadioRetry = millis();
    Serial.println("[LOOP] Radio de controlo inativo, a tentar reinicializar...");
    nrfCtrlOK = nrfTX->begin(SPI);
    if (nrfCtrlOK) {
      Serial.println("[LOOP] Radio de controlo recuperado!");
      tft->fillRect(0, 40, 200, 20, COL_ARMBG_565);
      tft->setCursor(10, 40);
      tft->setTextColor(TFT_GREEN);
      tft->setTextSize(2);
      tft->print("NRF TX: OK");
    }
  }

  // -----------------------------------
  // Leitura dos Botoes
  // -----------------------------------
  // digitalRead devolve HIGH (1) quando o botao esta solto,
  // e LOW (0) quando esta premido (por causa do INPUT_PULLUP).
  // O operador ! inverte esse valor para que "premido" seja true.

  bool armBtn  = !digitalRead(BTN_ARM_PIN);
  bool stopBtn = !digitalRead(BTN_STOP_PIN);


  // -----------------------------------
  // Paragem de Emergencia (STOP)
  // -----------------------------------
  // Tres condicoes em simultaneo para evitar ativacoes acidentais
  // (tecnica chamada debounce, anti-tremor):
  //   1. stopBtn         = botao esta premido AGORA
  //   2. !lastStopBtn    = na iteracao anterior NAO estava premido
  //                        (apanha so o momento exato de pressao)
  //   3. millis() > 200  = passaram pelo menos 200ms desde a ultima
  //                        vez, para filtrar o tremor fisico do botao

  if (stopBtn && !lastStopBtn && (millis() - lastStopPress > 200)) {
    lastStopPress = millis();
    if (armed) Serial.println("[LOOP] !!! PARAGEM DE EMERGENCIA PREMIDA, corte imediato !!!");
    armed = false;
    // Envia o sinal de emergencia 3 vezes para garantir que pelo menos
    // um pacote chega mesmo com possivel interferencia de radio.
    // ARMED_EMERGENCY (valor 2) diz ao drone para cortar os ESCs
    // imediatamente, sem rampa de descida gradual.
    for (int i = 0; i < 3; i++) nrfTX->send(ARMED_EMERGENCY, THROTTLE_MIN, 0, 0, 0);
    Serial.println("[TX] Emergencia STOP (armed=2) enviado 3x");
  }
  lastStopBtn = stopBtn;  // guarda estado atual para comparar na proxima iteracao


  // -----------------------------------
  // Armar / Desarmar (ARM Toggle)
  // -----------------------------------
  // Mesmo padrao de debounce (anti-tremor) do botao STOP.
  // O operador !armed inverte o estado: armado passa a desarmado
  // e vice-versa. Atualiza o texto no ecra sem redesenhar o fundo.

  if (armBtn && !lastArmBtn && (millis() - lastArmPress > 200)) {
    lastArmPress = millis();
    armed = !armed;
    if (armed) {
      Serial.println("[TX] ARM: ON, drone armado!");
      tft->fillRect(10, 70, 200, 25, COL_ARMBG_565);  // apaga texto anterior
      tft->setCursor(10, 70);
      tft->setTextColor(TFT_RED);
      tft->setTextSize(2);
      tft->print("ARMADO");
    } else {
      Serial.println("[TX] ARM: OFF, drone desarmado");
      tft->fillRect(10, 70, 200, 25, COL_ARMBG_565);
      tft->setCursor(10, 70);
      tft->setTextColor(TFT_CYAN);
      tft->setTextSize(2);
      tft->print("DESARMADO");
    }
  }
  lastArmBtn = armBtn;


  // -----------------------------------
  // Leitura dos Joysticks
  // -----------------------------------
  // Leituras reais dos joysticks (usadas no debug, independentes do armed).
  // Os valores transmitidos ao drone sao forcados a neutro quando desarmado
  // para evitar movimentos involuntarios ao armar.

  int rawThrottle = joystick.readThrottle();
  int rawYaw      = joystick.readYaw();
  int rawPitch    = joystickR.readPitch();
  int rawRoll     = joystickR.readRoll();

  int throttle = armed ? rawThrottle : THROTTLE_MIN;
  int yaw      = armed ? rawYaw      : 0;
  int pitch    = armed ? rawPitch    : 0;
  int roll     = armed ? rawRoll     : 0;


  // -----------------------------------
  // Envio do Comando ao Drone
  // -----------------------------------
  // Empacota os 5 valores num struct PayloadCtrl de 9 bytes
  // e envia por radio. A variavel 'ok' e true se o drone
  // confirmou a rececao (ACK automatico do NRF24L01).

  bool ok = nrfTX->send(armed ? ARMED_ON : ARMED_OFF, (int16_t)throttle, (int16_t)yaw,
                          (int16_t)pitch, (int16_t)roll);


  // -----------------------------------
  // Processamento do ACK (Estado dos Sensores)
  // -----------------------------------
  // O drone aproveita o byte de confirmacao (ACK) para devolver
  // informacao sobre os seus sensores. O byte tem dois campos
  // comprimidos em 8 bits:
  //   bits[1:0] = estado do sensor (2 bits, valores 0 a 3)
  //   bits[4:2] = fase de calibracao (3 bits, valores 0 a 7)
  //
  // >> 2 = desloca 2 bits para a direita (extrai os bits 2 a 4)
  // & 0x07 = mascara que mantem apenas os 3 bits menos significativos
  // & 0x03 = mascara que mantem apenas os 2 bits menos significativos

  uint8_t rawEstado = nrfTX->getLastAckByte();
  uint8_t calPhase  = (rawEstado >> 2) & 0x07;  // extrai fase de calibracao
  uint8_t rawSensor = rawEstado & 0x03;           // extrai estado do sensor

  // 'static' mantem o valor entre chamadas do loop(),
  // como variaveis globais mas visiveis apenas dentro desta funcao.
  static uint8_t sensorPrev    = 0;  // ultimo valor bruto do sensor
  static uint8_t sensorConfirm = 0;  // contador de leituras consecutivas iguais
  static uint8_t sensor        = 0;  // estado confirmado apos 4 leituras iguais

  if (calPhase != 0) {
    // O drone esta em calibracao: os sensores ainda nao sao fiaveis,
    // por isso ignora-se o estado e limpa-se o debounce.
    sensorConfirm = 0; sensorPrev = 0; sensor = 0;
  } else if (rawSensor == 0) {
    // Sem perigo detetado: limpa imediatamente.
    sensorConfirm = 0; sensor = 0;
  } else {
    // Debounce do sensor: so aceita uma leitura depois de 4 confirmacoes
    // consecutivas iguais. Protege contra picos eletricos (ruido) que
    // poderiam causar alarmes falsos.
    if (rawSensor == sensorPrev) {
      if (sensorConfirm < 4) sensorConfirm++;
    } else {
      sensorPrev    = rawSensor;
      sensorConfirm = 1;
    }
    if (sensorConfirm >= 4) sensor = rawSensor;
  }

  // Imprime no monitor serie apenas quando o estado muda,
  // para nao encher o terminal com mensagens repetidas.
  if (sensor != lastEstado) {
    lastEstado = sensor;
    Serial.print("[SENSOR] Estado mudou para: "); Serial.println(sensor);
  }


  // -----------------------------------
  // Atualizacao do Buzzer e LEDs
  // -----------------------------------
  // Combina calPhase (bits 2-4) e sensor (bits 0-1) num unico byte
  // no mesmo formato do ACK original e passa ao buzzer.
  // << 2 = desloca calPhase 2 bits para a esquerda
  // | = operador OR binario, junta os dois campos num so byte

  buzzer.update((calPhase << 2) | sensor);


  // -----------------------------------
  // Atualizacao do Ecra Termico
  // -----------------------------------
  // Uma unica chamada que internamente recebe pacotes disponiveis
  // no radio termico, verifica a qualidade da ligacao e desenha
  // uma nova imagem quando tem dados suficientes.

  displayRX->update();


  // -----------------------------------
  // Debug Serie (a cada 100ms)
  // -----------------------------------
  // Tudo numa unica linha separada por |||.
  // Dentro de cada bloco os valores sao separados por /.

  static uint32_t lastDebug = 0;
  if (millis() - lastDebug < 100) return;
  lastDebug = millis();

  uint16_t fails = nrfTX->getConsecutiveFails();
  uint32_t sent  = nrfTX->getTotalSent();
  uint32_t okCnt = nrfTX->getTotalOk();
  const char* conn = displayRX->getConnState() == 2 ? "BOA" :
                     displayRX->getConnState() == 1 ? "FRACA" : "SEM_SINAL";

  Serial.print("[CTRL] ARM="); Serial.print(armed ? "ON" : "OFF");
  Serial.print("/T=");   Serial.print(rawThrottle);
  Serial.print("/Y=");   Serial.print(rawYaw);
  Serial.print("/P=");   Serial.print(rawPitch);
  Serial.print("/R=");   Serial.print(rawRoll);
  Serial.print("/Up=");  Serial.print(millis() / 1000); Serial.print("s");

  Serial.print(" ||| NRF="); Serial.print(ok ? "OK" : "FAIL");
  Serial.print("/Fails="); Serial.print(fails);
  Serial.print("/Rate=");  Serial.print(sent > 0 ? okCnt * 100 / sent : 0);
  Serial.print("%(");      Serial.print(okCnt);
  Serial.print("/");       Serial.print(sent);
  Serial.print(")/ACK=");  Serial.print(rawEstado);
  Serial.print("/Snsr=");  Serial.print(sensor);

  // Leituras normalizadas: 0 no centro calibrado, sobem ate 4095 conforme
  // o stick se afasta do centro (ver EmberJoystick::readNormY/readNormX).
  Serial.print(" ||| ADC=LY"); Serial.print(joystick.readNormY());
  Serial.print("/LX");          Serial.print(joystick.readNormX());
  Serial.print("/RY");          Serial.print(joystickR.readNormY());
  Serial.print("/RX");          Serial.print(joystickR.readNormX());

  Serial.print(" ||| DISP=f"); Serial.print(displayRX->getFramesDrawn());
  Serial.print("/pkts");        Serial.print(displayRX->getLastPackets());
  Serial.print("/drw");         Serial.print(displayRX->getLastDrawMs()); Serial.print("ms");
  Serial.print("/fps");         Serial.print(displayRX->getLastFPS());
  Serial.print("/");            Serial.println(conn);
}
