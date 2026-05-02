# Ember — Alterações necessárias no lado do Gonçalo

## Contexto

O Tiago expandiu o `PayloadCtrl` de **5 bytes para 9 bytes** ao adicionar `pitch` e `roll`
para suportar o segundo joystick (stick direito do controlador).

**O que isto significa para ti:**
O teu ESP32 recebe os dados do Tiago via RF24. A função `receive()` usa `_radio->read(&p, sizeof(p))`
— ou seja, lê exatamente `sizeof(PayloadCtrl)` bytes do rádio. Se o teu `PayloadCtrl` ainda tiver
5 bytes mas o Tiago estiver a enviar 9, os bytes extra ficam no buffer e na próxima leitura o
`armed`, `throttle` e `yaw` chegam com valores completamente errados. **O drone vai ficar incontrolável.**

É por isso que esta atualização é obrigatória antes de qualquer teste conjunto.

---

## Resumo dos ficheiros a modificar

| Ficheiro | Tipo de alteração |
|---|---|
| `lib/EmberControlRX/EmberControlRX.h` | Obrigatório — struct desalinhado quebra tudo |
| `lib/EmberESC/EmberESC.h` | Novo método para controlo completo dos 4 motores |
| `lib/EmberESC/EmberESC.cpp` | Implementação do mixing pitch/roll nos motores |
| `src/Goncalo-Ember.ino` | Usar os novos campos do payload e o novo método ESC |

---

## Ficheiro 1 — `lib/EmberControlRX/EmberControlRX.h`

### Porquê
Este ficheiro define o `PayloadCtrl` — a estrutura de dados que chega pelo rádio.
O Tiago e o Gonçalo têm de ter **exatamente o mesmo struct, com os campos na mesma ordem**,
caso contrário o `read()` interpreta os bytes na posição errada.

Atualmente tens:
```cpp
struct PayloadCtrl {
  uint8_t armed;    // 1 byte — offset 0
  int16_t throttle; // 2 bytes — offset 1
  int16_t yaw;      // 2 bytes — offset 3
  // total: 5 bytes
};
```

O Tiago está agora a enviar:
```cpp
struct PayloadCtrl {
  uint8_t armed;    // 1 byte — offset 0
  int16_t throttle; // 2 bytes — offset 1
  int16_t yaw;      // 2 bytes — offset 3
  int16_t pitch;    // 2 bytes — offset 5  ← NOVO
  int16_t roll;     // 2 bytes — offset 7  ← NOVO
  // total: 9 bytes
};
```

### O que fazer

**Encontra** (no início do ficheiro, antes da declaração da classe):
```cpp
struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
};
```

**Substitui por:**
```cpp
struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};
```

### Nota sobre o `.cpp`
Não precisas alterar o `EmberControlRX.cpp`. A função `receive()` usa:
```cpp
_radio->read(&p, sizeof(p));
```
O `sizeof(p)` é calculado automaticamente pelo compilador a partir do struct.
Ao alterares o header, o `.cpp` vai automaticamente ler 9 bytes em vez de 5.

---

## Ficheiro 2 — `lib/EmberESC/EmberESC.h`

### Porquê
Atualmente tens `writeWithYaw(int base, int yaw)` que só controla rotação.
Para um drone real com os 4 eixos, precisas de um método que também misture `pitch` (frente/trás)
e `roll` (esquerda/direita) nos motores.

### O que fazer

**Encontra** a declaração do método existente:
```cpp
void writeWithYaw(int base, int yaw);
```

**Adiciona a linha imediatamente abaixo:**
```cpp
void writeWithAll(int base, int yaw, int pitch, int roll);
```

O header vai ficar assim na secção `public`:
```cpp
void writeAll(int us);
void writeWithYaw(int base, int yaw);
void writeWithAll(int base, int yaw, int pitch, int roll);  // ← novo
void setArmed(bool arm);
bool isArmed();
bool isRamping();
void updateRamp();
void emergencyStop();
```

---

## Ficheiro 3 — `lib/EmberESC/EmberESC.cpp`

### Porquê — Como funciona o motor mixing

O teu drone tem 4 motores em configuração X. Com base no teu código existente em `writeWithYaw()`:

```
Motores 0 e 3 recebem base + yaw  →  par diagonal A (rodam no mesmo sentido)
Motores 1 e 2 recebem base - yaw  →  par diagonal B (rodam no sentido contrário)
```

Isto significa que o layout físico dos teus motores é:

```
        FRENTE
   M0 ─────── M1
    │           │
   M3 ─────── M2
        TRÁS
```

Onde M0/M3 são diagonalmente opostos (um par) e M1/M2 são o outro par diagonal.
Para pitch e roll, a lógica é:

- **Pitch para frente (pitch < 0):** motores da frente (M0, M1) aumentam empuxo,
  motores de trás (M2, M3) diminuem → drone inclina para frente
- **Pitch para trás (pitch > 0):** o inverso
- **Roll para a direita (roll > 0):** motores da esquerda (M0, M3) aumentam,
  motores da direita (M1, M2) diminuem → drone inclina para a direita
- **Roll para a esquerda (roll < 0):** o inverso

O mixing completo para cada motor:

| Motor | Posição | Yaw | Pitch | Roll |
|-------|---------|-----|-------|------|
| M0    | Frente-Esq | `+yaw` | `+pitch` | `+roll` |
| M1    | Frente-Dir | `-yaw` | `+pitch` | `-roll` |
| M2    | Trás-Dir   | `-yaw` | `-pitch` | `-roll` |
| M3    | Trás-Esq   | `+yaw` | `-pitch` | `+roll` |

### O que fazer

**Vai ao fim do ficheiro `EmberESC.cpp`** (depois do último `}`) e adiciona:

```cpp
void EmberESC::writeWithAll(int base, int yaw, int pitch, int roll) {
  if (!_armed) return;
  int safeMin = THROTTLE_ARM + 50;

  // Motor 0 — Frente-Esquerda
  if (_esc[0]) _esc[0]->writeMicroseconds(
    constrain(base + yaw + pitch + roll, safeMin, 2000));

  // Motor 1 — Frente-Direita
  if (_esc[1]) _esc[1]->writeMicroseconds(
    constrain(base - yaw + pitch - roll, safeMin, 2000));

  // Motor 2 — Trás-Direita
  if (_esc[2]) _esc[2]->writeMicroseconds(
    constrain(base - yaw - pitch - roll, safeMin, 2000));

  // Motor 3 — Trás-Esquerda
  if (_esc[3]) _esc[3]->writeMicroseconds(
    constrain(base + yaw - pitch + roll, safeMin, 2000));
}
```

### ⚠️ AVISO IMPORTANTE — verificação física obrigatória

Os sinais `+/-` para pitch e roll **dependem de como os teus motores estão fisicamente montados**
e em que direção cada um roda. O mixing acima é baseado no layout mais comum para drones
em configuração X, mas pode ter de ser ajustado.

**Como verificar (SEM HÉLICES):**

1. Compila e carrega o código
2. Abre o Serial Monitor (115200 baud)
3. Pede ao Tiago para armar e mover apenas o stick direito para cima/frente
4. Observa no serial `P=` e `R=` — confirma que chegam valores esperados (-100/0/+100)
5. Com os motores a girar devagar (throttle mínimo armado), inclina levemente o drone
   e observa se os motores que deviam compensar é que aumentam rotação
6. Se um eixo estiver invertido, troca o sinal desse motor nesse eixo

---

## Ficheiro 4 — `src/Goncalo-Ember.ino`

### Alteração 1 — Substituir a chamada ao ESC no loop de voo

**Encontra** (dentro do loop de voo, quando o drone está armado):
```cpp
esc.writeWithYaw(throttle, yaw);
```

**Substitui por:**
```cpp
esc.writeWithAll(throttle, yaw, p.pitch, p.roll);
```

Onde `p` é a variável `PayloadCtrl` que já tens declarada para receber os dados do rádio.
Se a tua variável tiver outro nome (ex: `payload`, `ctrl`), usa o nome correto.

### Alteração 2 — Adicionar Pitch e Roll ao debug serial

**Encontra** o bloco de debug que corre a cada 2 segundos
(procura por `Serial.print` perto de onde está `throttle` e `yaw`).

**Adiciona as linhas de pitch e roll a seguir ao yaw:**
```cpp
Serial.print(" P=");    Serial.print(p.pitch);
Serial.print(" R=");    Serial.print(p.roll);
```

O resultado no terminal vai ficar tipo:
```
ARM=1 T=1300 Yaw=0 P=0 R=0 RF=OK Sensor=0
ARM=1 T=1300 Yaw=0 P=-100 R=50 RF=OK Sensor=0
```

Isto é essencial para confirmar que os valores chegam corretamente antes de testar com hélices.

### Alteração 3 — Atualizar o cabeçalho do ficheiro (comentários de pinos)

No topo do `.ino` há normalmente um comentário com a lista de pinos.
Atualiza para refletir que o Tiago agora tem dois joysticks:

```cpp
// Payload recebido (9 bytes — struct PayloadCtrl):
//   armed:    0 = stop/desarmado, 1 = armado
//   throttle: microsegundos (1000-1500)
//   yaw:      -100 a +100
//   pitch:    -100 a +100  (frente/trás — stick direito Y)
//   roll:     -100 a +100  (esquerda/direita — stick direito X)
```

---

## Checklist final — por ordem de execução

- [ ] **1.** Abrir `lib/EmberControlRX/EmberControlRX.h` e adicionar `pitch` e `roll` ao `PayloadCtrl`
- [ ] **2.** Abrir `lib/EmberESC/EmberESC.h` e declarar `writeWithAll(...)`
- [ ] **3.** Abrir `lib/EmberESC/EmberESC.cpp` e implementar `writeWithAll(...)` no fim
- [ ] **4.** Abrir `src/Goncalo-Ember.ino` e substituir `writeWithYaw` por `writeWithAll`
- [ ] **5.** Adicionar `p.pitch` e `p.roll` ao debug serial no `.ino`
- [ ] **6.** Compilar — deve compilar sem erros
- [ ] **7.** Carregar para o ESP32 do drone
- [ ] **8.** Testar **sem hélices** com o Tiago a controlar
- [ ] **9.** Confirmar no Serial Monitor que P= e R= chegam com os valores certos
- [ ] **10.** Verificar fisicamente se os motores reagem corretamente ao pitch/roll
- [ ] **11.** Ajustar sinais no `writeWithAll` se necessário
- [ ] **12.** Só depois de tudo verificado → colocar hélices

---

## Referência rápida — o que mudou no Tiago

Para referência, aqui está o estado atual do lado do Tiago:

**`lib/EmberControlTX/EmberControlTX.h`**
```cpp
struct PayloadCtrl {
  uint8_t armed;
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;   // novo
  int16_t roll;    // novo
};

bool send(uint8_t armed, int16_t throttle, int16_t yaw,
          int16_t pitch, int16_t roll);
```

**`src/Tiago-Ember.ino`** — dois joysticks:
```cpp
EmberJoystick joystick(34, 35);   // esquerdo: throttle/yaw
EmberJoystick joystickR(15, 13);  // direito:  pitch/roll
```

Os valores de pitch e roll variam entre `-100` e `+100`, igual ao yaw.
O valor `0` significa centro (sem inclinação).
