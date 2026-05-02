# EMBER — Protocolo RF e Diagnóstico de Delay

Este ficheiro descreve o protocolo de comunicação entre o controlador de chão (Tiago)
e o drone (Gonçalo), as causas de input delay, e o que cada lado pode fazer para
o minimizar.

---

## Visão geral do sistema

```
[Controlador — Tiago]                    [Drone — Gonçalo]
  Joystick L + R                            Motores / ESCs
  Botão ARM / STOP                          Sensores (gás, chama)
  Ecrã TFT 480x320                         Câmara térmica
  NRF Controlo  ──── canal 76 ────►  NRF Controlo
  NRF Térmico   ◄─── canal 2  ────   NRF Térmico
```

Existem dois NRF24L01 em cada lado, num bus SPI partilhado:
- **Canal 76** — controlo bidirecional (Tiago envia comandos, Gonçalo responde com ACK + estado dos sensores)
- **Canal 2** — câmara térmica unidirecional (Gonçalo envia frames, Tiago recebe e exibe)

---

## Payload de controlo (canal 76)

### Struct enviada por Tiago a cada iteração do loop

```cpp
struct PayloadCtrl {
  uint8_t armed;     // 0 = desarmado, 1 = armado
  int16_t throttle;  // 1000 a 1500 (microsegundos para ESC)
  int16_t yaw;       // -100 a +100
  int16_t pitch;     // -100 a +100  (stick direito Y — frente/trás)
  int16_t roll;      // -100 a +100  (stick direito X — esquerda/direita)
};
// sizeof(PayloadCtrl) == 9 bytes
```

Quando `armed == 0`, Tiago envia sempre `throttle=1000, yaw=0, pitch=0, roll=0`.

### ACK payload enviado por Gonçalo (1 byte)

```cpp
uint8_t sensorState;
// 0 = tudo OK
// 1 = gás / CO detectado
// 2 = chama detectada
// 3 = gás + chama
```

Gonçalo deve chamar `radio.writeAckPayload(pipe, &sensorState, 1)` antes de cada
pacote de controlo chegar, para que o ACK automático do NRF inclua este byte.

---

## Configuração RF24 — canal de controlo (canal 76)

### Lado Tiago (TX)

```cpp
radio.setChannel(76);
radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.setAutoAck(true);
radio.setRetries(1, 3);      // 3 retries x 0.5ms = 1.5ms worst case
radio.enableAckPayload();
radio.openWritingPipe("00001");
radio.stopListening();
```

### Lado Gonçalo (RX) — deve estar assim

```cpp
radio.setChannel(76);
radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);   // ou HIGH se precisar de alcance
radio.setAutoAck(true);
radio.enableAckPayload();
radio.setRetries(1, 3);          // deve ser igual ao TX
radio.openReadingPipe(1, "00001");
radio.startListening();
```

---

## Configuração RF24 — câmara térmica (canal 2)

### Lado Gonçalo (TX térmico)

```cpp
radio.setChannel(2);
radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_HIGH);
radio.setAutoAck(false);
radio.setPayloadSize(25);
radio.setCRCLength(RF24_CRC_DISABLED);
radio.openWritingPipe("THERM");
radio.stopListening();
```

### Formato do pacote térmico (25 bytes)

```cpp
struct ThermalPacket {
  uint8_t index;    // 0 a 31 (qual das 32 fatias da frame)
  uint8_t data[24]; // 24 valores de temperatura normalizados (0-255)
};
```

O sensor MLX90640 produz uma frame de 32x24 pixels = 768 bytes.
Gonçalo divide em 32 pacotes de 24 bytes cada, com o índice na primeira posição.
Tiago recebe e reconstrói a frame quando tiver pacotes suficientes.

---

## Causas de delay e como corrigir

### No lado de Tiago (controlador)

| Causa | Impacto | Estado |
|---|---|---|
| `delay(300)` nos botões ARM/STOP | 300ms de bloqueio por press | **Corrigido** — millis debounce |
| RF24 `setRetries(5, 15)` | 22.5ms worst case por pacote | **Corrigido** — agora 1.5ms |
| `drawThermalSmooth()` bloqueia o loop | 100-150ms por frame térmico | **Pendente** |

O `drawThermalSmooth()` desenha os 24 blocos da imagem térmica de uma só vez,
bloqueando o loop durante ~100-150ms. O controlo não é enviado nesse período.
Se o delay ainda for notório, a solução definitiva é mover o render térmico para
o Core 0 do ESP32-S3 (dual-core).

### No lado de Gonçalo (drone)

#### 1. Usar `writeFast()` + `txStandBy()` em vez de `write()`

`radio.write()` bloqueia até receber ACK ou esgotar retries. Com `writeFast()`, o
envio é não-bloqueante — o código continua imediatamente:

```cpp
// Em vez disto:
radio.write(&payload, sizeof(payload));

// Usar isto:
radio.writeFast(&payload, sizeof(payload));
radio.txStandBy();  // só bloqueia no mínimo necessário
```

#### 2. Nunca usar `delay()` no loop de controlo

Qualquer `delay()` no loop principal bloqueia a leitura de novos pacotes.
Se o drone precisa de temporização, usar `millis()`:

```cpp
// Errado:
loop() {
  if (radio.available()) { ... }
  delay(10);  // bloqueia 10ms, perde pacotes
}

// Correto:
loop() {
  if (radio.available()) { ... }
  // sem delay — o loop corre o mais rápido possível
}
```

#### 3. Ler o NRF em CADA iteração do loop

```cpp
void loop() {
  // Ler NRF primeiro, antes de qualquer outra coisa
  if (radio.available()) {
    PayloadCtrl cmd;
    radio.read(&cmd, sizeof(cmd));
    aplicarComando(cmd);  // aplicar imediatamente
  }

  // Resto do loop (sensores, telemetria, etc.)
  // ...
}
```

#### 4. Preparar o ACK payload ANTES de o pacote chegar

O NRF envia o ACK automaticamente quando recebe um pacote. O payload do ACK tem
de estar carregado com antecedência, não depois de receber:

```cpp
void loop() {
  // Preparar ACK com estado atual dos sensores
  uint8_t estado = lerSensores();  // 0/1/2/3
  radio.writeAckPayload(1, &estado, 1);  // pipe 1

  // Agora ler o pacote
  if (radio.available()) {
    PayloadCtrl cmd;
    radio.read(&cmd, sizeof(cmd));
    aplicarComando(cmd);
  }
}
```

#### 5. Separar o loop de controlo do envio térmico

O NRF de controlo e o NRF térmico estão em objetos separados mas no mesmo bus SPI.
Chamar operações no NRF térmico enquanto o NRF de controlo precisa de responder
causa contenção no bus. Sugestão de estrutura:

```cpp
void loop() {
  // 1. Controlo — prioridade máxima, corre sempre
  lerEAplicarComando();

  // 2. Térmico — só envia quando houver dados novos do sensor
  static uint32_t ultimoTerm = 0;
  if (millis() - ultimoTerm >= 40) {  // ~25fps máximo
    ultimoTerm = millis();
    enviarFrameTermica();
  }

  // 3. Sensores e telemetria — pode ser mais lento
  static uint32_t ultimoSensor = 0;
  if (millis() - ultimoSensor >= 100) {
    ultimoSensor = millis();
    atualizarSensores();
  }
}
```

#### 6. Verificar o rate do loop

Adicionar este código temporariamente para medir o rate real do loop:

```cpp
void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t loopCount = 0;
  loopCount++;

  if (millis() - lastPrint >= 1000) {
    Serial.print("[LOOP] rate=");
    Serial.print(loopCount);
    Serial.println("Hz");
    loopCount = 0;
    lastPrint = millis();
  }

  // ... resto do loop
}
```

Para um drone de controlo responsivo, o loop deve correr a **>= 100Hz** (idealmente
200-500Hz). Se for < 50Hz, há algo a bloquear o loop.

---

## Resumo — checklist para Gonçalo

- [ ] Loop sem `delay()` em lado nenhum
- [ ] NRF lido na primeira linha do loop
- [ ] `writeAckPayload()` chamado ANTES de `radio.available()`
- [ ] Loop rate >= 100Hz (verificar com o contador acima)
- [ ] Envio térmico rate-limitado (não enviar em cada iteração do loop)
- [ ] `setRetries(1, 3)` no NRF de controlo (igual ao controlador)
- [ ] Canal 76 para controlo, canal 2 para térmico
- [ ] Address `"00001"` no pipe de leitura de controlo
- [ ] Address `"THERM"` no pipe de escrita térmico
