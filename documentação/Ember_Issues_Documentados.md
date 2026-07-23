# Ember — Issues Cruciais
Caso fores recriar este projeto, tem aqui problemas (mais importantes) e soluções que tivemos ao longo do projeto:

---

## Hardware Bugs

### #4 — Problemas Prévios
**Problema:** Alimentação incorreta dos motores brushless.
**Solução:** Não alimentar o 3V do ESC.

### #6 — Reviver a bateria
**Problema:** Bateria LiPo queimada quimicamente por nunca ter sido posta a carregar corretamente.
**Solução:** Não deixes a bateria descarregar! Usa um lipo buzzer (recomendado) ou programa tu próprio um sistema de detecção de bateria fraca.

### #11 — Delay na conexão
**Problema:** Ligação com atraso elevado entre comando e drone.
**Solução:** Causa apontada para fios mal ligados na breadboard e clips desgastados/largos; resolvido indiretamente com a troca de breadboards (#16).

### #16 — Trocar as breadboards
**Problema:** Má conexão dos pinos na breadboard.
**Solução:** Substituição das breadboards para umas mais fixas e melhores, correção que resolveu praticamente todos os problemas de comunicação.

### #38 — Comunicação só funciona apenas numa porta USB (sem bateria ligada)
**Problema:** Alimentação insuficiente nalgumas portas USB fazia os transceptores falhar.
**Solução:** Mudança da porta USB, literalmente só isso..

### #38 — Verificar que todos os motores estão a girar na direção correta
**Problema:** Fazer os motores girarem para dentro
**Solução:** Trocar os cabos do motor com o ESC.

### #47 — Hélice direita e esquerda
**Problema:** As hélices têm direções específicas!
**Solução:** Colocar as hélices certas no lugar correto, existem estilo "esquerdas e direitas", não estou a falar de girarem na direção correta, isso é outra coisa.


---

## Software Bugs

### #1 — Sincronismo das hélices
**Problema:** Ao ligar o sistema, as hélices não arrancavam em sincronismo.
**Solução:** Tens que calibrar o drone! (Retira hélice do motor antes para evitar danos)

---

## Montagem

### #32 — Encontrar o ponto de hover
**Problema:** Necessário o drone montado por completo (peso final) para calibrar o ponto de hover.
**Solução:** Varia com a hélice, bateria, motor... Ajusta o valor em Gonçalo_Ember.ino (Drone)
