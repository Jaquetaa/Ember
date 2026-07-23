# Ember — Registo de Testes

## Alcance do rádio
Testado visualmente a ~30m, com ligação estável.
Segundo o datasheet do NRF24L01: 20-50m com obstáculos, até 100m em ar livre.

## Autonomia da bateria
10 minutos com throttle constante. Considerado bom para este tipo de drone.

## Precisão da câmara térmica
Funciona bem, destaca claramente fontes de calor. Alcance eficaz de 5-10 metros.
Sendo uma prova de conceito, não foi otimizada para alcance máximo.

## Calibração
Por motivos de segurança, correr o código de calibração (`(drone)/older/CALIBRAR.ino`) com as hélices retiradas dos motores (recomendo), ou usar a built-in calibração (não recomendo)

## Peso
Atualmente 1.1kg, levanta voo (tens que levantar o drone com CUIDADO por segurança)
