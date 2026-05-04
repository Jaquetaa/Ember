# EMBER — Protocolo de Calibração + Sincronização Drone ↔ Tiago

Este documento descreve a calibração dos ESCs disparada por botão no drone
e como o controlador (Tiago) deve reagir através da nova biblioteca
**EmberBuzzer**. Inclui o protocolo de comunicação NRF, o diagrama temporal,
a especificação da biblioteca a criar do lado do Tiago, e a lógica de
prioridade do buzzer único.

---

## 1. Visão geral

Quando o utilizador prime o **botão GPIO3** no drone, é executada uma
sequência sincronizada entre os dois micro-controladores:

```
[Botão GPIO3 premido]
        │
        ▼
┌─────────────────────┐         ┌──────────────────────────────┐
│  DRONE (este repo)  │ ──ACK── │  TIAGO (controlador)         │
│  EmberCalibration   │  byte   │  EmberBuzzer + LEDs          │
│  · ramp down ESCs   │  com    │  · 25 beeps frenicos         │
│  · bloqueia voo     │  fase   │  · 500 ms silêncio           │
│  · roda max/min ESCs│         │  · countdown 5 s             │
│  · re-arma estado   │         │  · 1 s silêncio              │
│                     │         │  · beeps + LEDs em sync      │
│                     │         │  · 1 s silêncio              │
└─────────────────────┘         └──────────────────────────────┘
```

O drone é o **mestre** da máquina de estados. O Tiago apenas observa a
fase publicada no ACK byte e reage localmente — não há temporizadores
acoplados, o que torna o sistema robusto a perdas de pacotes.

---

## 2. Protocolo NRF — encoding do ACK byte

A ligação NRF entre Tiago (TX, envia comandos) e drone (RX, recebe
comandos) usa **ACK payloads** de 1 byte. Anteriormente o byte era apenas
o `estado` dos sensores (0..3). Foi estendido para empacotar também a
fase de calibração:

```
  byte ACK (8 bits) :  [ x  x  x  | calPhase (3 bits) | estado (2 bits) ]
                          7  6  5      4    3    2          1    0

  estado    = ack & 0x03;          // 0..3 — sensores
  calPhase  = (ack >> 2) & 0x07;   // 0..7 — fase de calibração
```

### 2.1 Valores de `estado` (sensores)

| Valor | Significado          |
|------:|----------------------|
| 0     | nada                 |
| 1     | CO/gás acima limite  |
| 2     | chama detectada      |
| 3     | chama + gás          |

### 2.2 Valores de `calPhase`

| Valor | Símbolo          | Duração   | O que o Tiago faz                         |
|------:|------------------|-----------|-------------------------------------------|
| 0     | `PH_IDLE`        | —         | Buzzer normal (gás/chama). LEDs normais.  |
| 1     | `PH_RAMP_DOWN`   | ~variável | **Silêncio** + LEDs OFF (drone a abrandar)|
| 2     | `PH_WARN_BURST`  | 1500 ms   | **25 beeps frenéticos** na freq calibração|
| 3     | `PH_SILENT_500`  |  500 ms   | **Silêncio total** (buzzer + LEDs OFF)    |
| 4     | `PH_COUNTDOWN`   | 5000 ms   | 5 ticks: beep 300 ms ON / 700 ms OFF      |
| 5     | `PH_SILENT_PRE`  | 1000 ms   | Silêncio total                            |
| 6     | `PH_CAL_RUN`     | 10 000 ms | Beep contínuo freq calibração + LEDs sync |
| 7     | `PH_SILENT_POST` | 1000 ms   | Silêncio total                            |

> **Importante:** durante TODAS as fases ≥ 1, o drone **ignora** comandos
> de controlo recebidos (descarta sem agir nos ESCs). Mantém só o link
> vivo para o ACK byte continuar a chegar ao Tiago.

---

## 3. Sequência temporal completa

```
t=0       botão premido no drone
          │
RAMP_DOWN │ desce 5us a cada 30ms até 1000us  (~variável)
          │
WARN_BURST│ ████ ████ ████ ████ ████ ...  25 beeps em 1500ms
          │   (60ms ciclo: 30ms ON / 30ms OFF, freq calibração)
          │
SILENT_500│ (silêncio total)                500ms
          │
COUNTDOWN │  ███         ███         ███         ███         ███
          │  300ms       300ms       300ms       300ms       300ms
          │  ←     1s    →←    1s    →←    1s    →←    1s    →←  1s →
          │                                                   = 5000ms
SILENT_PRE│ (silêncio total)               1000ms
          │
CAL_RUN   │ ████████████████████████████  drone:  MAX 5s -> MIN 5s
          │      buzzer contínuo freq calibração + LEDs blink em sync
          │                                            ~10 000ms
SILENT_POS│ (silêncio total)               1000ms
          │
t=fim     drone re-armável, calPhase=0
```

---

## 4. Lado do drone (este repositório) — o que está feito

### 4.1 Biblioteca `lib/EmberCalibration/`

- `EmberCalibration.h` — declara o enum `Phase` e os timers canónicos.
- `EmberCalibration.cpp` — máquina de estados não-bloqueante.

API:

```cpp
calibration.begin(CAL_BTN_PIN,
                  calWriteAll,   // void(int us)  — escreve nos 4 ESCs
                  calOnStart,    // void()        — desarma drone
                  calOnDone);    // void()        — re-arma drone
calibration.update(currentThrottle);   // chamar todos os loops
calibration.isBusy();                  // true se != PH_IDLE
calibration.getPhase();                // Phase actual (0..7)
```

### 4.2 Integração no `.ino`

- `#define CAL_BTN_PIN 3` — botão entre **GPIO3 e GND**, `INPUT_PULLUP`,
  active LOW, debounce 50 ms.
- No início do `loop()`:

  ```cpp
  calibration.update(currentThrottle);
  if (calibration.isBusy()) {
      // mantém ACK a fluir (com fase) mas descarta comandos
      controlRX.updateAckPayload(buildAckByte(estado, calPhase));
      controlRX.receive(dummy);
      sensores.update();
      thermalTX.update();
      return;     // bloqueia toda a lógica de voo
  }
  ```

- Na chamada normal do ACK:

  ```cpp
  controlRX.updateAckPayload(
      buildAckByte(sensores.getEstado(),
                   (uint8_t)calibration.getPhase()));
  ```

### 4.3 Constantes canónicas (drone)

Definidas em `EmberCalibration.h` — **TÊM de bater certo no Tiago**:

```cpp
T_WARN_BURST_MS    = 1500
T_SILENT_500_MS    =  500
T_COUNTDOWN_MS     = 5000
T_SILENT_PRE_MS    = 1000
T_CAL_MAX_HOLD_MS  = 5000   // sub-fase MAX dentro de CAL_RUN
T_CAL_MIN_HOLD_MS  = 5000   // sub-fase MIN dentro de CAL_RUN
T_SILENT_POST_MS   = 1000
```

---

## 5. Lado do Tiago — biblioteca `EmberBuzzer` a criar

### 5.1 Hardware

- **Um único buzzer (passivo) no GPIO 10**. Os outros pinos de buzzer
  que existiam antes desaparecem.
- LED de gás e LED de chama mantêm-se nos seus pinos atuais. Durante a
  fase `PH_CAL_RUN` ambos piscam **em sincronia com o buzzer**.

### 5.2 Prioridade do buzzer

Só toca uma fonte de cada vez, exceto gás+chama que se misturam:

```
PH_CAL_RUN > PH_COUNTDOWN > PH_WARN_BURST > chama+gás (mix) > chama > gás > silêncio
```

Tabela completa (do mais prioritário para o menos):

| Prioridade | Fonte                  | Frequência (Hz) | Padrão                          |
|-----------:|------------------------|-----------------|---------------------------------|
| 1 (max)    | `PH_CAL_RUN`           | `F_CAL`         | Contínuo + LEDs gás/chama sync  |
| 2          | `PH_COUNTDOWN`         | `F_CAL`         | 300 ms ON / 700 ms OFF × 5      |
| 3          | `PH_WARN_BURST`        | `F_CAL`         | 30 ms ON / 30 ms OFF × 25       |
| 4          | `PH_SILENT_*` / `PH_RAMP_DOWN` | —     | Silêncio total                  |
| 5          | chama **e** gás        | mix `F_FIRE`+`F_GAS` | Padrão chama (mais urgente) |
| 6          | só chama               | `F_FIRE`        | Padrão chama (ex: 100 ms beep)  |
| 7          | só gás                 | `F_GAS`         | Padrão gás (ex: 500 ms beep)    |
| 8          | nada                   | —               | Silêncio                        |

### 5.3 Frequências sugeridas

```cpp
F_GAS  = 1500   // grave/médio
F_FIRE = 2500   // mais agudo
F_CAL  = 4000   // FINA — mais aguda do que gás (e do que chama)
```

> A regra obrigatória é só `F_CAL > F_GAS`. As restantes são sugestão e
> podem ser afinadas ao gosto.

### 5.4 Mistura de gás + chama

Um buzzer num único pino digital não consegue tocar duas frequências
verdadeiramente em simultâneo. Soluções aceitáveis (por ordem de
qualidade):

1. **Time-multiplex rápido (recomendado, simples)** — alternar entre
   `F_FIRE` e `F_GAS` em janelas de 20 ms. Soa como dois tons
   simultâneos para o ouvido humano. Implementação: `ledcWriteTone` no
   ESP32 a saltar entre as duas freqs cada 20 ms.
2. **XOR de ondas quadradas** — software loop a 100 µs gerando
   `digitalWrite(BUZZ, sqA(t) ^ sqB(t))`. Produz batimento audível.
3. **Soma com PWM analógico** — só viável com DAC/PWM rápido suficiente.
   Não recomendado em buzzer passivo barato.

### 5.5 LEDs em sincronia (só durante `PH_CAL_RUN`)

Padrão sugerido: **on/off a 5 Hz** (100 ms ON / 100 ms OFF) em ambos os
LEDs (gás e chama) ao mesmo tempo, com o buzzer também em on/off no
mesmo ritmo a `F_CAL`. Resultado: pulsar audível + visual sincronizado.

Pseudo-código:

```cpp
// Dentro de PH_CAL_RUN, a 5 Hz:
bool tick = ((millis() / 100) % 2) == 0;
digitalWrite(LED_GAS,   tick);
digitalWrite(LED_FIRE,  tick);
if (tick) ledcWriteTone(BUZZ_CH, F_CAL);
else      ledcWriteTone(BUZZ_CH, 0);
```

### 5.6 Esqueleto da biblioteca

`lib/EmberBuzzer/EmberBuzzer.h`:

```cpp
#ifndef EMBER_BUZZER_H
#define EMBER_BUZZER_H
#include <Arduino.h>

class EmberBuzzer {
public:
  enum CalPhase : uint8_t {
    PH_IDLE        = 0,
    PH_RAMP_DOWN   = 1,
    PH_WARN_BURST  = 2,
    PH_SILENT_500  = 3,
    PH_COUNTDOWN   = 4,
    PH_SILENT_PRE  = 5,
    PH_CAL_RUN     = 6,
    PH_SILENT_POST = 7,
  };

  // buzzPin   — GPIO do buzzer passivo (= 10 no Tiago)
  // ledGasPin / ledFirePin — LEDs que piscam em PH_CAL_RUN
  // ledcChannel — canal LEDC livre para tone()
  void begin(uint8_t buzzPin,
             uint8_t ledGasPin,
             uint8_t ledFirePin,
             uint8_t ledcChannel = 5);

  // Chamar a cada loop, com o byte ACK recebido do drone.
  // Internamente descodifica estado + calPhase e gere o buzzer/LEDs.
  void update(uint8_t ackByte);

  // Frequencias configuraveis (defaults: 1500/2500/4000 Hz)
  void setFrequencies(uint16_t fGas, uint16_t fFire, uint16_t fCal);

private:
  // ── State driving ──
  uint8_t  _buzz, _ledGas, _ledFire, _ch;
  uint16_t _fGas = 1500, _fFire = 2500, _fCal = 4000;

  CalPhase _phase = PH_IDLE, _prevPhase = PH_IDLE;
  uint32_t _tPhaseEnter = 0;

  // helpers
  void  _tone(uint16_t hz);   // 0 = silencio
  void  _silence();
  void  _ledsBoth(bool on);
  void  _drivePhase(uint8_t estado, uint32_t now);
  void  _driveAlarms(uint8_t estado, uint32_t now);  // gas/chama mix
};
#endif
```

`lib/EmberBuzzer/EmberBuzzer.cpp` — esboço dos blocos importantes:

```cpp
#include "EmberBuzzer.h"

void EmberBuzzer::begin(uint8_t buzzPin, uint8_t ledGas, uint8_t ledFire,
                        uint8_t ch) {
  _buzz = buzzPin; _ledGas = ledGas; _ledFire = ledFire; _ch = ch;
  pinMode(_ledGas,  OUTPUT); digitalWrite(_ledGas,  LOW);
  pinMode(_ledFire, OUTPUT); digitalWrite(_ledFire, LOW);
  ledcSetup(_ch, 4000, 10);          // resolucao p/ tone()
  ledcAttachPin(_buzz, _ch);
  _silence();
}

void EmberBuzzer::_tone(uint16_t hz) {
  if (hz == 0) { ledcWriteTone(_ch, 0); return; }
  ledcWriteTone(_ch, hz);
  ledcWrite(_ch, 512);               // ~50% duty
}
void EmberBuzzer::_silence()       { ledcWriteTone(_ch, 0); }
void EmberBuzzer::_ledsBoth(bool on){
  digitalWrite(_ledGas,  on ? HIGH : LOW);
  digitalWrite(_ledFire, on ? HIGH : LOW);
}

void EmberBuzzer::update(uint8_t ackByte) {
  uint8_t  estado   = ackByte & 0x03;
  CalPhase calPhase = (CalPhase)((ackByte >> 2) & 0x07);
  uint32_t now      = millis();

  if (calPhase != _phase) {
    _phase        = calPhase;
    _tPhaseEnter  = now;
    _prevPhase    = calPhase;
  }

  if (calPhase != PH_IDLE) {
    _drivePhase(estado, now);
  } else {
    _driveAlarms(estado, now);
  }
}

void EmberBuzzer::_drivePhase(uint8_t estado, uint32_t now) {
  uint32_t t = now - _tPhaseEnter;   // ms desde entrada na fase
  switch (_phase) {

    case PH_RAMP_DOWN:
    case PH_SILENT_500:
    case PH_SILENT_PRE:
    case PH_SILENT_POST:
      _silence();
      _ledsBoth(false);
      break;

    case PH_WARN_BURST: {
      // 25 beeps em 1500ms -> ciclo 60ms (30 ON / 30 OFF)
      bool on = ((t / 30) % 2) == 0;
      _tone(on ? _fCal : 0);
      _ledsBoth(false);
      break;
    }

    case PH_COUNTDOWN: {
      // 5 ticks de 1s: 300ms ON / 700ms OFF
      uint32_t intra = t % 1000;
      bool on = (intra < 300);
      _tone(on ? _fCal : 0);
      _ledsBoth(false);
      break;
    }

    case PH_CAL_RUN: {
      // 5 Hz: 100ms ON / 100ms OFF, buzzer + LEDs em sync
      bool on = ((t / 100) % 2) == 0;
      _tone(on ? _fCal : 0);
      _ledsBoth(on);
      break;
    }

    default:
      _silence();
      _ledsBoth(false);
      break;
  }
}

void EmberBuzzer::_driveAlarms(uint8_t estado, uint32_t now) {
  // PH_IDLE — gere alarmes normais. LEDs aqui sao geridos
  // pelo codigo principal do Tiago (não tocamos neles). Limpo:
  _ledsBoth(false);

  if (estado == 3) {
    // mix chama + gás via time-multiplex 20ms
    bool useFire = ((now / 20) % 2) == 0;
    _tone(useFire ? _fFire : _fGas);
  } else if (estado == 2) {
    // chama: padrao 100ms ON / 100ms OFF
    bool on = ((now / 100) % 2) == 0;
    _tone(on ? _fFire : 0);
  } else if (estado == 1) {
    // gas: padrao 250ms ON / 250ms OFF
    bool on = ((now / 250) % 2) == 0;
    _tone(on ? _fGas : 0);
  } else {
    _silence();
  }
}
```

### 5.7 Integração no `.ino` do Tiago

```cpp
#include "EmberBuzzer.h"
EmberBuzzer buzzer;

// no setup():
buzzer.begin(/*buzz=*/10, /*ledGas=*/<pino>, /*ledFire=*/<pino>);

// quando recebe ACK do drone (depois de cada radio.write()):
uint8_t ack = 0;
if (radio.isAckPayloadAvailable()) radio.read(&ack, 1);
buzzer.update(ack);
```

> **Importante:** o Tiago já lê 1 byte de ACK. Mantém o tamanho. A
> mudança é apenas a interpretação do byte (estado vs. estado+calPhase).

---

## 6. Checklist de validação

### No drone

- [ ] Botão entre **GPIO3 e GND** (sem resistor — `INPUT_PULLUP` interno).
- [ ] `pio run` compila sem erros.
- [ ] No monitor série após premir o botão: aparecem mensagens
      `[CAL] -> fase 1` ... `[CAL] -> fase 7`.
- [ ] Comandos do rolante são ignorados durante a sequência.
- [ ] Após a sequência, drone está desarmado mas pronto a re-armar
      via comando ARM normal.

### No Tiago

- [ ] Buzzer físico passou para **GPIO10** (todos os outros pinos
      de buzzer removidos).
- [ ] `EmberBuzzer` adicionado em `lib/EmberBuzzer/`.
- [ ] No `.ino` chama-se `buzzer.update(ack)` a cada pacote recebido.
- [ ] Frequências: gás=1500 Hz, chama=2500 Hz, calibração=4000 Hz
      (cumpre `F_CAL > F_GAS`).
- [ ] Em gás+chama os dois tons alternam rapidamente (mix).
- [ ] Em `PH_CAL_RUN`, LEDs de gás e chama piscam em sync com o buzzer.
- [ ] Em fases silenciosas, **nada** soa nem pisca.

### Fim a fim

- [ ] Premir o botão no drone → 25 beeps muito rápidos no Tiago.
- [ ] 500 ms de silêncio total.
- [ ] 5 beeps espaçados (1 por segundo, 300 ms cada) — countdown.
- [ ] 1 s silêncio.
- [ ] Buzzer + LEDs a piscar 10 s — calibração a correr.
- [ ] 1 s silêncio.
- [ ] Tudo volta ao normal — alarmes de gás/chama voltam a funcionar.

---

## 7. Notas e gotchas

- **GPIO3 no ESP32-S3** é strapping pin (JTAG_SEL). Usar `INPUT_PULLUP`
  é seguro porque a leitura ocorre depois do boot. Evitar puxá-lo a
  LOW durante reset.
- O drone é o relógio: se houver perdas de pacotes, o Tiago fica preso
  na última fase conhecida apenas durante o tempo do timeout (já existe
  no Tiago para detectar perda de drone). Reentrar em `PH_IDLE` depois
  de timeout devolve estado normal.
- **Não bloquear** com `delay()` em nenhum lado da máquina de estados —
  perderia pacotes e dessincronizaria.
- Na fase `PH_CAL_RUN` o utilizador deve fisicamente desligar e
  reconectar a bateria do ESC (mensagem do drone via serial avisa-o
  antes da fase começar).
