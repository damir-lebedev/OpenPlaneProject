# Placa da controladora de voo: blocos de conectores

> 🌐 Esta página é uma tradução do [original em russo](../../FC_BOARD.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Uma placa portadora para a ESP32-S3 DevKitC-1 (N16R8): a DevKit se encaixa em duas barras de soquetes fêmea, e em volta ficam blocos de conectores JST-XH. Este documento responde a três perguntas: quais conectores montar em blocos, onde colocar os capacitores e o que ligar em cada lugar. Os pinos coincidem com o `include/config/Config.h` (o bloco `BOARD_ESP32_S3`).

A placa foi pensada para ser **de um lado só**: os GPIOs foram escolhidos de modo que os pinos de cada bloco sigam em sequência ao longo da barra da DevKit, e as trilhas de sinal se abram em leque sem se cruzar. Eu não verifiquei o roteamento em um CAD. Se em algum ponto não fechar, coloque um jumper de fio pelo lado dos componentes: de um a três em uma placa assim é normal.

**Todos os conectores são JST-XH, de 1 a 5 contatos.** Os fios não são soldados na placa: nos fios dos servos, do ESC, do receptor e dos módulos crimpam-se as carcaças correspondentes. O XH tem chave, então não dá para ligar ao contrário. Cada contato aguenta ~3 A.

---

## 1. Esquema de disposição

Vista de cima, pelo lado dos componentes. Os conectores USB da DevKit ficam na borda inferior.

```
                    topo: antena do ESP, sem cobre embaixo
+----------------------------------------------------------------------+
| anel de GND em toda a borda                                          |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (atrás do DevKit)   |    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  trilho I2C    |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | 5V lógica |
|    [diodo 1N5822] --> 5V lógica --- embaixo, sob o USB --+           |
+----------------------------------------------------------------------+
```

Quatro zonas:

| Zona | Onde | O que há | Alimentação |
|---|---|---|---|
| **A** Servos | a borda esquerda, em frente a J1-4…11 | 8 × XH-3: superfícies, ESC, AUX, receptor | 5V dos servos (sujo) |
| **B** Bateria, telemetria | embaixo à esquerda, em frente a J1-12…16 | sensores de bateria e de corrente, o rádio modem | 5V lógica |
| **C** 3V3 | em cima à direita, em frente a J3-4…7 | OLED, sensores no trilho I2C, conectores I2C | 3V3 |
| **D** 5V lógica | embaixo à direita, em frente a J3-8…18 | GPS, buzzer, AUX3, luzes | 5V lógica |

À esquerda fica a parte de potência (servos, ESC, bateria), e à direita, os sensores e as comunicações. A corrente dos servos fica à esquerda e não passa ao lado dos sensores.

---

## 2. As barras da DevKit: qual perna vai para onde

A ordem das pernas é a da ESP32-S3-DevKitC-1 (2 × 22). Confira com a serigrafia da sua placa e meça a distância entre as fileiras antes de desenhar. A numeração vai da antena para o USB.

| J1 (fileira esquerda) | GPIO | Para | | J3 (fileira direita) | GPIO | Para |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 por cima → zona C | | 1 | GND | — |
| 2 | 3V3 | (o mesmo) | | 2 | 43 | — (console "COM") |
| 3 | RST | — | | 3 | 44 | — (console "COM") |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | trilho I2C: SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | trilho I2C: SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS: TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS: RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 buzzer (via Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | — PSRAM |
| 12 | 8 | B1 VBAT (ADC) | | 12 | 36 | — PSRAM |
| 13 | 3 | B1 CURR (ADC) | | 13 | 35 | — PSRAM |
| 14 | 46 | — strapping | | 14 | 0 | — botão BOOT |
| 15 | 9 | B2 TELEM: TX | | 15 | 45 | — strapping |
| 16 | 10 | B2 TELEM: RX | | 16 | 48 | — LED RGB |
| 17 | 11 | livre | | 17 | 47 | D3 AUX3 |
| 18 | 12 | livre | | 18 | 21 | D4 LIGHT (via Q2) |
| 19 | 13 | livre | | 19 | 20 | — USB |
| 20 | 14 | livre | | 20 | 19 | — USB |
| 21 | 5V | entrada de 5V lógica (depois do diodo) | | 21 | GND | terra da zona D |
| 22 | GND | terra da zona B | | 22 | GND | terra da zona D |

Na bancada, o firmware usa os GPIO11–14 livres para o SPI (ICM-42688); nesta placa o SPI não é roteado.

---

## 3. Regras para caber em uma única camada

1. **Componentes com terminais de furo passante em cima, SMD embaixo.** Os soquetes da DevKit, os XH, os eletrolíticos e o diodo ficam pelo lado dos componentes. Os 0805, 0603 e SOT-23 são soldados direto no cobre. No método de transferência de toner (o do ferro de passar), o desenho do cobre é impresso espelhado.
2. **Em cada conector, o sinal fica mais perto da DevKit, a alimentação mais longe e o GND na borda.** Por isso, em todos os conectores o **contato 1 é o mais próximo da DevKit**. Assim, as trilhas de sinal não cruzam a alimentação. A exceção é o trilho I2C (item 5).
3. **O GND é um preenchimento de cobre em todo o perímetro** (um anel). Os contatos das extremidades dos conectores saem direto nele.
4. **De um lado da DevKit para o outro passam apenas duas linhas.** O 3V3 sobe a partir de J1-1/2, passa pela extremidade superior da DevKit e segue para a direita, além da borda da placa da DevKit, e não sob a antena. A 5V lógica vai de J1-21 por baixo da DevKit para baixo e pela borda inferior para a direita, sob os conectores USB (ali há só trilhas; o plugue fica mais acima).
5. **O trilho I2C.** Quatro trilhas paralelas com passo de 2,54 mm, saindo da DevKit para fora: **3V3 · GND · SCL · SDA**. Essa é a ordem dos pinos dos módulos GY (VCC GND SCL SDA). Os soquetes e conectores ficam atravessados no trilho, como vagões: cada trilha passa pelo seu próprio contato. O trilho começa em J3-6/7, mergulha sob o OLED e sobe ao longo da fileira direita. Se não couber na altura, dobre-o para a esquerda por cima da extremidade superior da DevKit; a ordem das linhas se mantém na curva.
6. **Não passe trilhas entre as pernas da DevKit**: o passo de 2,54 é estreito demais para a transferência de toner. Sob a própria DevKit pode: ali há 11 mm até a placa dela.
7. **Componentes de furo passante são jumpers de graça.** Uma trilha passa folgadamente sob o corpo do diodo (passo dos terminais de 12,5–15 mm) e entre as pernas de um eletrolítico (5 mm).
8. **0805 entre os contatos do conector.** Com o passo de 2,5 mm do XH, um capacitor 0805 é soldado direto entre os contatos vizinhos +5V e GND, pelo lado do cobre.
9. **Largura das trilhas:** 5V dos servos e GND dos servos, a partir de 2 mm; 5V lógica, a partir de 1 mm; sinais, 0,4–0,5 mm.
10. **Rótulos na serigrafia:** o número do conector (A1, B2…), a inscrição e uma seta junto ao contato 1.

---

## 4. Blocos de conectores

### Bloco A — servos: 8 × XH-3, a borda esquerda, em coluna em frente a J1-4…11

Os contatos de cada conector:
**1 — sinal** (mais perto da DevKit) · **2 — +5V dos servos** · **3 — GND** (para a borda).
Essa é a ordem do fio de um servo: laranja, vermelho, marrom.

| Conector | Rótulo | O que ligar | GPIO (perna) |
|---|---|---|---|
| A1 | AIL-L | o aileron esquerdo | 4 (J1-4) |
| A2 | AIL-R | o aileron direito | 5 (J1-5) |
| A3 | ELE | o profundor | 6 (J1-6) |
| A4 | ESC | o controlador: sinal do acelerador; **no fio vermelho, a entrada do BEC de 5V** | 7 (J1-7) |
| A5 | AUX1 | o servo de lançamento de carga | 15 (J1-8) |
| A6 | AUX2 | flaps (dois servos por um cabo em Y) ou qualquer servo | 16 (J1-9) |
| A7 | RC | o receptor FS-iA6B, porta iBUS SERVO (o receptor é alimentado daqui) | 17 (J1-10) |
| A8 | RUD | o leme + a roda | 18 (J1-11) |

O que mais é soldado no bloco:

- **O barramento de +5V dos servos**: a coluna do meio dos contatos, com trilha de 2 mm ou mais. **GND**: a coluna externa, que também faz parte do anel de GND.
- **330 Ω (0603)** em série com cada linha de sinal, junto ao conector. Se 5 V chegarem a um contato de sinal (servo defeituoso, crimpagem torta), o pino da ESP sobrevive.
- **10 kΩ (0603)** do sinal do A4 ESC para o GND: enquanto a ESP reinicia, nenhum lixo chega ao controlador.
- **100 nF (0805)** entre os contatos 2 e 3 de cada conector.
- O **A4 ESC** leva ainda 10 µF + 100 pF (0805): a entrada de alimentação da placa toda (filtragem de baixa, alta e altíssima frequência).
- O **A7 RC** leva ainda 10 µF (0805): o receptor é sensível a quedas de tensão.
- **2 × 470 µF 16 V** no barramento dos servos, um em cada ponta da coluna (acima do A1 e abaixo do A8): o positivo no barramento, o negativo no anel. Dentro da coluna o negativo não alcança o anel, e 3 cm de trilha larga não fazem diferença para um eletrolítico.

Um soquete de ESC aguenta ~3 A. Para 4–6 servos MG90S, isso basta. Se houver mais servos e eles forem mais potentes, adicione ao lado do A4 um soquete de alimentação separado vindo do BEC.

### Bloco B — bateria e telemetria, embaixo à esquerda, sob os servos

**B1 BAT — XH-5** (o único XH-5 da placa: um cabo com 12–17 V não entra em nenhum outro conector)

| Contato | O quê | Para onde vai na placa |
|---|---|---|
| 1 | **VBAT** — o positivo da bateria por um fio fino (o "+" da ponta do conector de balanceamento, ou do conector da bateria, não pelo ESC) | um divisor de 56 kΩ / 10 kΩ → GPIO8 (J1-12) |
| 2 | vazio — um vão entre a tensão da bateria e todo o resto | — |
| 3 | **CURR** — a saída do sensor de corrente | um divisor de 10 kΩ / 15 kΩ → GPIO3 (J1-13) |
| 4 | +5V lógica — alimentação do sensor de corrente | o barramento de 5V lógica |
| 5 | GND | o anel |

- **O divisor de VBAT:** 56 kΩ em cima, 10 kΩ embaixo, 100 nF em paralelo com o de baixo. 3S (12,6 V) → 1,91 V, 4S (16,8 V) → 2,55 V, com folga até o limite do ADC (~3,1 V). Não é preciso um fio de terra separado: o terra é comum pelo ESC, então o contato 5 pode ficar sem crimpagem.
- **O divisor de CURR:** 10 kΩ em cima, 15 kΩ embaixo, 100 nF em paralelo com o de baixo. Um sensor Hall de 5 V (ACS758 e semelhantes) dá no máximo 5 V → 3,0 V no pino. Se a saída do sensor for de 3,3 V, o resistor de cima é de 0 Ω e o de baixo não é soldado.
- Coloque os divisores bem junto ao conector e pegue o terra do contato 5 dele: assim só uma trilha vai até a ESP.
- Ainda não tem sensor de corrente? Simplesmente não crimpe os contatos 3–4.

**B2 TELEM — XH-4:** um rádio modem de telemetria ou iBUS-SENS.

| Contato | O quê | GPIO (perna) |
|---|---|---|
| 1 | TX → para o RX do modem | 9 (J1-15) |
| 2 | RX ← do TX do modem | 10 (J1-16) |
| 3 | +5V lógica | — |
| 4 | GND | — |

- 10 µF + 100 nF entre os contatos 3 e 4. Para um modem de 1 W, acrescente um eletrolítico de 470 µF.
- O firmware ainda não suporta o TELEM: as três UARTs estão ocupadas (console, iBUS, GPS). Para ativá-lo, é preciso passar o console para o USB integrado. Isso é uma alteração de firmware; o conector já é roteado desde agora.

### Bloco C — 3V3: a tela e os sensores, em cima à direita

Tudo nesta zona fica no **trilho I2C** (seção 3, item 5): quatro trilhas **3V3 · GND · SCL · SDA** saindo da DevKit para fora. O SCL vem do GPIO42 (J3-6) e o SDA, do GPIO41 (J3-7). O 3V3 chega por cima, de J1-1/2. O OLED fica mais embaixo no trilho, e acima dele, em ordem, C2–C5.

**C1 OLED — XH-4.** Ele pega a alimentação do trilho e os dados do seu próprio barramento (GPIO1/2), que chegam pelo lado de dentro.

| Contato | O quê | De onde |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | o trilho |
| 4 | GND | o trilho |

Essa é a ordem dos pinos do módulo OLED (GND VCC SCL SDA) de trás para a frente, então o cabo vai sem torções.

**C2 I2C-A e C3 I2C-B — XH-4**, idênticos:

| Contato | O quê |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** — a bússola: um GY-273 em um mastro (um cabo direto, com a mesma ordem do módulo) ou a bússola de um módulo GPS. No GPS só se crimpam GND, SCL e SDA: a bússola recebe a alimentação pelo cabo do GPS.
- **C3** — o reserva: um sensor de velocidade do ar (MS4525DO), um telêmetro e assim por diante.

**C4 BARO — um soquete 1×4 para um módulo BMP388:** 1 — VCC, 2 — GND, 3 — SCL, 4 — SDA.

- No próprio módulo, solde jumpers de fio **CSB→VCC** e **SDO→GND**, o que hoje, na bancada, é feito com fios. Sem CSB→VCC o chip vai para o modo SPI. Os demais pinos do módulo ficam soltos no ar.
- A ordem dos pinos varia entre os módulos BMP388: confira o seu. Se não coincidir, o módulo vai em um cabo para o C3 e o soquete não é montado.
- Por cima, um pedaço de espuma de poros abertos (contra o fluxo de ar e a luz).

**C5 IMU — um soquete 1×8 para um GY-521:**

| Contato | Pino do módulo | Para onde |
|---|---|---|
| 1 | VCC | o trilho de 3V3 |
| 2 | GND | o trilho de GND |
| 3 | SCL | o trilho de SCL |
| 4 | SDA | o trilho de SDA |
| 5, 6 | XDA, XCL | para nada |
| 7 | AD0 | para o preenchimento de GND fora do trilho (endereço 0x68) |
| 8 | INT | para nada |

Como o IMU está girado na placa não importa: a instalação é definida pela calibração `o` (PILOT_GUIDE, "Instalação do IMU").

Os capacitores do bloco: **10 µF + 100 nF** no trilho junto ao C1 (onde começa o 3V3), **100 nF** entre os contatos 1 e 2 em C2–C5. Os pull-ups do I2C já estão nos módulos; não os coloque na placa.

### Bloco D — 5V lógica: GPS, buzzer, luzes, embaixo à direita

O barramento de 5V lógica chega por baixo (de sob a DevKit, pela borda inferior) e sobe pela borda direita através dos contatos "+5V" de todos os conectores do bloco.

**D1 GPS — XH-4.** Fica logo abaixo do trilho, para que o soquete do GPS e o da bússola dele (C2) fiquem juntos.

| Contato | O quê | GPIO (perna) |
|---|---|---|
| 1 | TX → para o RX do GPS | 40 (J3-8) |
| 2 | RX ← do TX do GPS | 39 (J3-9) |
| 3 | +5V lógica | — |
| 4 | GND | — |

10 µF + 100 nF entre os contatos 3 e 4.

**D2 BUZ — XH-2:** um buzzer ativo de 5 V, para achar o avião na grama e avisar sobre a bateria e o ARM.

| Contato | O quê |
|---|---|
| 1 | o "−" do buzzer → chave Q1 |
| 2 | +5V lógica → o "+" do buzzer |

**D3 AUX3 — XH-3:** 1 — o sinal do GPIO47 (J3-17) por um resistor de 330 Ω, 2 — +5V lógica, 3 — GND, mais 100 nF entre 2 e 3. Serve para um botão, os dados de uma fita de LED, o gatilho de uma câmera. **Não pendure um servo aqui:** é 5V lógica, e a corrente dele passaria pelo diodo e faria a alimentação da ESP oscilar.

**D4 LIGHT — XH-2:** uma chave para uma carga de até ~0,5–1 A: luzes de navegação, um farol, o eletroímã de lançamento.

| Contato | O quê |
|---|---|
| 1 | o "−" da carga → chave Q2 |
| 2 | +5V lógica → o "+" da carga |

**As chaves Q1 e Q2** — a mesma pegada SOT-23. No BC817 e no Si2302 as pernas coincidem no papel que desempenham:

| Perna do SOT-23 | BC817 | Si2302 | Para onde |
|---|---|---|---|
| 1 | base | gate | ← 1 kΩ ← GPIO (38 para o Q1, 21 para o Q2); 10 kΩ da perna 1 para o GND |
| 2 | emissor | source | GND (o terra da zona — J3-21/22) |
| 3 | coletor | dreno | contato 1 do conector (D2 / D4) |

- **Q1 (buzzer):** serve qualquer um dos dois.
- **Q2 (luzes):** **Si2302**; o BC817 esquenta com centenas de miliampères.
- O resistor de 10 kΩ mantém a chave aberta enquanto a ESP inicializa: o buzzer não grita e as luzes não piscam.
- Se a carga for uma bobina (um eletroímã, um buzzer magnético), coloque um diodo SS14 ou 1N4148 em paralelo com o conector, com o cátodo no +5V. Deixe espaço para ele entre os contatos 1 e 2.

---

## 5. Alimentação e todos os capacitores

```
 ESC (BEC 5V/5A) ──► A4 ──► barramento 5V dos servos ──┬──► A1…A8 (servos, receptor)
                                                       │    2×470 µF nas pontas da coluna
                                                       │
                                                       └──► diodo 1N5822 ──► 5V lógica ──┬──► J1-21 (5V DevKit)
                                                                                         ├──► B1, B2 (sensor de corrente, modem)
                                                                                         └──► D1…D4 (GPS, buzzer, AUX3, luzes)
 DevKit: regulador 3V3 próprio ──► J1-1/2 ──► por cima ──► trilho C (OLED, sensores, conectores I2C)
```

- **A entrada é uma só: A4 ESC.** Não é preciso um conector de alimentação separado.
- **O diodo Schottky 1N5822** (3 A, de furo passante; o substituto SMD é o SS34): é preciso comprá-lo. Ele faz três coisas:
  - USB e BEC não brigam: dá para deixar o USB plugado com a bateria conectada;
  - quando os servos derrubam o barramento, o eletrolítico da lógica não se descarrega de volta nos servos, e a ESP não reinicia;
  - o corpo de furo passante funciona como jumper sobre a trilha de GND no canto junto a J1-22.

  O ânodo vai na ponta inferior do barramento dos servos e o cátodo, em J1-21. Não use um 1N5819 (1 A): pelo diodo passam a ESP, o modem, o GPS e as luzes.
- Só com o USB os servos não são alimentados; o diodo os desconecta. É assim mesmo que foi pensado. O GPS, o modem e o buzzer funcionam pelo USB sobre a mesa apenas se o pino de 5V da DevKit fornecer energia a partir do USB. Alguns clones têm ali um diodo próprio, e então não funcionam: isso é normal.
- Os 3,3 V vêm só do regulador da DevKit e só para a zona C.

**Todos os capacitores em uma tabela** (cerâmicos: 0805; eletrolíticos: 16 V):

| Onde | O quê | Para quê |
|---|---|---|
| Barramento dos servos, acima do A1 e abaixo do A8 | 470 µF + 470 µF | as quedas quando todos os servos se mexem de uma vez |
| A4 ESC, entre + e GND | 10 µF + 100 nF + 100 pF | a entrada de alimentação: baixa, alta e altíssima frequência |
| A1–A3, A5–A8 | 100 nF em cada um | ruído dos motores dos servos, na origem |
| A7 RC | + 10 µF | o receptor |
| 5V lógica, em J1-21 | 470 µF + 10 µF + 100 nF | sustenta a ESP quando o BEC cai |
| Trilho de 3V3, em C1 | 10 µF + 100 nF | alimentação dos sensores |
| C2–C5 | 100 nF em cada um | |
| B1: a entrada do ADC de VBAT e CURR | 100 nF em cada uma, em paralelo com o resistor de baixo | o filtro do ADC |
| B1: o +5V do sensor de corrente | 100 nF entre os contatos 4 e 5 | |
| B2 TELEM | 10 µF + 100 nF (um modem de 1 W: + 470 µF) | os picos de corrente do transmissor |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

Um cerâmico 0805 de 10 µF perde até metade da capacitância a 5 V: isso já está considerado, e há eletrolíticos por perto de qualquer forma.

Pontos de teste: **5VS** (o barramento dos servos), **5VL** (5V lógica), **3V3**, **GND**. São práticos para medir com um multímetro.

---

## 6. O que liga onde

| Dispositivo | Conector | Observações |
|---|---|---|
| Os servos dos ailerons, do profundor e do leme | A1, A2, A3, A8 | o sinal vai no contato 1 |
| ESC | A4 | o fio vermelho é a entrada do BEC |
| O receptor FS-iA6B | A7 | a porta iBUS SERVO, um cabo comum de 3 fios |
| O servo de lançamento de carga, os flaps | A5, A6 | |
| GY-521 (MPU6500) | C5, soquete | |
| BMP388 | C4, soquete | jumpers no módulo CSB→VCC, SDO→GND |
| GY-273 (bússola) ou a bússola do GPS | C2 | longe dos fios de potência, de preferência em um mastro |
| OLED 128×64 | C1 | |
| Sensor de velocidade do ar e semelhantes | C3 | |
| GPS u-blox M10 | D1 + C2 | os 6 fios se dividem em duas carcaças: D1 (alimentação, UART) e C2 (bússola) |
| Tensão da bateria, sensor de corrente | B1 | |
| Rádio modem / iBUS-SENS | B2 | |
| Buzzer | D2 | |
| Luzes, farol | D4 | |

Posicionamento no avião:

- Monte a placa sobre uma fixação macia (espuma, almofadas de gel), mais perto do centro de gravidade: a vibração do motor estraga os ângulos.
- Mantenha os fios de potência (bateria → ESC → motor) longe da zona C e da bússola.
- O barômetro vai sob espuma; o IMU, como você quiser (calibração `o`).

---

## 7. Lista de componentes

| Componente | Qtd. | Onde |
|---|---|---|
| Soquete PBS 1×22 (para a DevKit) | 2 | |
| XH-3 em ângulo ou reto | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| Soquetes PBS 1×8 e 1×4 | 1 de cada | C5, C4 |
| Diodo Schottky 1N5822 (ou SS34) | 1 | **comprar** |
| Eletrolítico de 470 µF 16 V | 3 (+1 para um modem potente) | barramento dos servos ×2, 5V lógica |
| 0805 de 10 µF | 6 | A4, A7, 5V lógica, 3V3, B2, D1 |
| 0805 de 100 nF | 20 | veja a tabela de capacitores |
| 0805 de 100 pF | 1 | A4 |
| 0603 de 330 Ω | 9 | sinais A1–A8, D3 |
| 0603 de 10 kΩ | 5 | ESC para o GND, parte de baixo de VBAT, parte de cima de CURR, perna 1 de Q1 e Q2 |
| 0603 de 56 kΩ | 1 | parte de cima de VBAT |
| 0603 de 15 kΩ | 1 | parte de baixo de CURR |
| 0603 de 1 kΩ | 2 | para a perna 1 de Q1 e Q2 |
| BC817 ou Si2302 | 1 | Q1 (buzzer) |
| Si2302 | 1 | Q2 (luzes) |
| SS14 / 1N4148 | 0–2 | só para bobinas em D2 e D4 |

A placa sai com cerca de 80×95 mm: a coluna dos servos e o trilho dos sensores ultrapassam a extremidade superior da DevKit. Se não couber na fuselagem, a forma mais simples de reduzi-la é levar o BARO e o IMU para cabos até o C3 e encurtar o trilho.

---

## 8. Quais GPIOs não mexer

0, 45, 46: deles depende o modo de boot; 19/20: USB; 26–37: a flash e a PSRAM do módulo N16R8; 43/44: o conector "COM" (console); 48: o LED RGB. Depois deste plano, só os GPIO11–14 ficam livres (na bancada, SPI).

---

## 9. Antes de ligar pela primeira vez

1. Sem a DevKit e sem a bateria, confira a continuidade: +5V dos servos ↔ GND, 5V lógica ↔ GND, 3V3 ↔ GND; não pode haver curto em lugar nenhum.
2. Aplique o BEC (pelo A4), com a DevKit ainda não encaixada. Em 5VS deve haver 5,0–5,2 V e em 5VL, 0,3–0,5 V a menos (a queda no diodo).
3. Encaixe a DevKit e conecte só o USB. Em 5VS há 0 V: o diodo não deixa o USB entrar nos servos.
4. Tudo junto. O `s` do console mostrará se os sensores respondem e quantos erros há no I2C.

---

## 10. O que mudou em relação ao plano anterior

- **Os pinos foram remanejados para uma única camada** (já no `Config.h`):
  - I2C dos sensores 8/9 → **41/42**;
  - GPS 15/16 → **39/40**;
  - AUX1/AUX2 41/42 → **15/16**;
  - VBAT 3 → **8**, sensor de corrente 10 → **3**;
  - telemetria 39/40 → **9/10**;
  - uma nova saída **LIGHT** no GPIO21.

  Na bancada, troque dois fios: SDA 8 → 41, SCL 9 → 42. Os demais pinos são reserva e não estão conectados na bancada.
- **Um só BEC pelo ESC**: o conector PWR separado foi retirado.
- **Os sensores na placa ficam no I2C** (como na bancada). Os soquetes SPI foram retirados e os GPIO11–14 estão livres. O ICM-42688 também fala I2C, mas o firmware vai precisar de uma variante I2C dele no `SensorSelection.h`.
- **O GPS-MAG foi retirado**: a bússola do GPS é ligada no C2.
- **VBAT e o sensor de corrente foram combinados** em um único XH-5 (B1).
