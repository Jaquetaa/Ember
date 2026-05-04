# Fix: Câmara perde conexão após 1 frame

## Causa

1. **I2C a 1MHz** — instável com fios longos; `getFrame()` começa a falhar silenciosamente
2. **`getFrame()` sem recovery** — se falhar, o estado fica em IDLE para sempre (sem pacotes TX)
3. **`txStandBy()` sem verificação** — se falhar, o NRF fica preso em FIFO cheio no burst seguinte

---

## Ficheiro: `EmberCAMNRF.h`

Adicionar dois campos privados para guardar os pinos I2C (precisamos deles no recovery):

```cpp
private:
  void *_radio = nullptr;
  void *_mlx   = nullptr;
  bool _nrfOK  = false;
  uint32_t _frameCount = 0;
  float   *_mlxFrame    = nullptr;
  uint8_t *_thermalData = nullptr;

  uint8_t _sdaPin = 0;   // <-- NOVO
  uint8_t _sclPin = 0;   // <-- NOVO

  uint8_t  _state       = 0;
  uint8_t  _txIndex     = 0;
  uint32_t _lastFrameMs = 0;
```

---

## Ficheiro: `EmberCAMNRF.cpp`

### 1. Guardar pinos I2C e baixar clock para 400kHz

No `begin()`, onde está `Wire.begin(sdaPin, sclPin)`:

```cpp
  // ANTES:
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(1000000);

  // DEPOIS:
  _sdaPin = sdaPin;
  _sclPin = sclPin;
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(400000);   // 1MHz era instável com fios longos
```

---

### 2. Recovery do MLX no `update()` — estado IDLE

Substituir este bloco no estado 0:

```cpp
  // ANTES:
  if (mlx->getFrame(_mlxFrame) != 0) return;
```

```cpp
  // DEPOIS:
  static uint8_t mlxFails = 0;
  if (mlx->getFrame(_mlxFrame) != 0) {
    mlxFails++;
    if (mlxFails >= 15) {
      mlxFails = 0;
      Serial.println("[THERM-TX] MLX getFrame falhou 15x seguidas — a reiniciar I2C...");
      Wire.end();
      delay(20);
      Wire.begin(_sdaPin, _sclPin);
      Wire.setClock(400000);
      mlx->begin(MLX90640_I2CADDR_DEFAULT, &Wire);
      Serial.println("[THERM-TX] MLX re-init feito.");
    }
    return;
  }
  mlxFails = 0;
```

---

### 3. Verificar `txStandBy()` no burst — estado TX_BURST

Substituir este bloco no final do estado 1:

```cpp
  // ANTES:
  if (_txIndex >= 32) {
    radio->txStandBy();
    _state = 0;
    _frameCount++;
    ...
  }
```

```cpp
  // DEPOIS:
  if (_txIndex >= 32) {
    bool standbyOK = radio->txStandBy(95);
    if (!standbyOK) {
      Serial.println("[THERM-TX] txStandBy falhou — flush TX FIFO");
      radio->flush_tx();
    }
    _state = 0;
    _frameCount++;
    ...
  }
```

---

## Resumo das mudanças

| Ficheiro | Linha / zona | O que muda |
|---|---|---|
| `EmberCAMNRF.h` | campos privados | Adicionar `_sdaPin` e `_sclPin` |
| `EmberCAMNRF.cpp` `begin()` | `Wire.setClock(...)` | 1MHz → 400kHz, guardar pinos |
| `EmberCAMNRF.cpp` `update()` estado 0 | `getFrame()` | Adicionar contador de falhas + re-init I2C após 15 falhas |
| `EmberCAMNRF.cpp` `update()` estado 1 | `txStandBy()` | Verificar retorno + flush se falhar |

---

## Como confirmar que era este o problema

No serial da câmara, antes do fix, provavelmente vias:

```
[THERM-TX] Frame #10 | Centro: XX.X C
```
...e depois silêncio total (sem mais frames). Após o fix deves ver:
```
[THERM-TX] MLX getFrame falhou 15x seguidas — a reiniciar I2C...
[THERM-TX] MLX re-init feito.
[THERM-TX] Frame #11 | Centro: XX.X C
```
e a câmara recupera sozinha.
