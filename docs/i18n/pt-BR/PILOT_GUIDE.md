# Guia do piloto do OpenPlaneProject

> 🌐 Esta página é uma tradução do [original em russo](../../PILOT_GUIDE.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Este é um guia prático de "o que ligar onde e como voar" para quem tem nas mãos um ferro de solda e um rádio, e não lê código-fonte. Se você quer entender a arquitetura do código, veja os outros documentos do repositório. Aqui só há hardware, canais, firmware e voos.

Repositório: https://github.com/damir-lebedev/OpenPlaneProject, branch `main`.

Já de início, com franqueza: o projeto está em desenvolvimento ativo e **não é um produto pronto**. O primeiro protótipo já voou, mas com ressalvas, descritas abaixo em uma seção própria. Leia-a antes de voar, e não depois.

---

## Sumário

1. [De que hardware você precisa](#de-que-hardware-você-precisa)
2. [Escolha da placa e pinagem](#escolha-da-placa-e-pinagem)
3. [Ligação do receptor](#ligação-do-receptor)
4. [Mapa de canais RC](#mapa-de-canais-rc)
5. [Canais do piloto automático](#canais-do-piloto-automático)
6. [ARM e failsafe](#arm-e-failsafe)
7. [Gravação do firmware na placa](#gravação-do-firmware-na-placa)
8. [Dashboard web no campo](#painel-web-em-campo)
9. [Caixa-preta](#caixa-preta)
10. [Checklist pré-voo e segurança](#checklist-pré-voo-e-segurança)
11. [Solução de problemas](#solução-de-problemas)
12. [Situação atual do aeromodelo-protótipo](#estado-atual-da-estrutura-do-protótipo)

---

## De que hardware você precisa

O kit da montagem atual (o firmware foi verificado nele):

- **Uma placa ESP32-S3 N16R8** (clone da DevKitC-1 com duas USB-C: "USB" e "COM").
- **Um rádio FS-i6 + um receptor FS-iA6B** (protocolo iBUS, 10 canais). É preciso um único fio de dados: a porta iBUS SERVO. O rádio precisa permitir configurar o failsafe; essa configuração é obrigatória, veja a seção sobre failsafe.
- **2 servos MG90S** para os ailerons, um para cada semiasa (dois servos independentes, e não um para as duas asas).
- **1 servo MG90S** para o profundor.
- **1 servo MG90S** para o leme; no mesmo eixo dele está montada a roda direcional do trem de pouso (para manobrar no solo).
- **Um controlador de velocidade (ESC)** de 60–80 A com BEC de 5 V (o BEC alimenta os servos e o receptor).
- **Um motor D3548 1100KV** + **uma hélice 10x5**.
- **Uma bateria LiPo 3S**.

Sensores do piloto automático (todos por I2C; sem eles a aeronave voa no modo manual):

- **GY-521**: giroscópio + acelerômetro (na placa pode haver um MPU6050 ou, como no nosso caso, um MPU6500; os dois são aceitos).
- **BMP581**: barômetro (o BMP388 anterior também é suportado).
- **GY-273**: bússola (a nossa tem um QMC5883P; o QMC5883L também é aceito).
- Opcionalmente, um **OLED 128×64 SSD1306** (I2C): tela de status a bordo.

Estrutura: envergadura de 1200 mm, corda de 250 mm, perfil NACA 4412, construção em PETG (impressão 3D). O primeiro protótipo voou com um ESP32-C3, um motor D2212 1000KV e um ESC de 40 A.

---

## Escolha da placa e pinagem

O firmware suporta três placas; para trocar de uma para outra basta um parâmetro de build (`pio run -e <nome do ambiente>`). Cada placa tem a sua própria pinagem, fixada no firmware para cada ambiente específico: não troque os fios por conta própria, consulte a tabela da sua placa.

> **Importante:** a placa principal agora é a **esp32-s3 (N16R8)**; a pinagem dela foi verificada na bancada com todos os sensores. A **esp32-c3** é o antigo protótipo que já voou. A pinagem da **esp32-dev** foi escolhida com base na documentação do chip e **não foi verificada em hardware real**.

### esp32-s3 (N16R8): a placa principal, verificada na bancada

`pio run -e esp32-s3`, placa `esp32-s3-devkitc-1` com as configurações do módulo N16R8 (16 MB de flash, 8 MB de PSRAM octal). É a placa padrão (`default_envs = esp32-s3`).

| Função | GPIO |
|---|---|
| Aileron, semiasa esquerda | GPIO4 |
| Aileron, semiasa direita | GPIO5 |
| Profundor | GPIO6 |
| ESC (acelerador) | GPIO7 |
| Leme + roda direcional | GPIO18 |
| iBUS do receptor (RX) | GPIO17 |
| I2C dos sensores SDA / SCL (MPU, BMP581, bússola) | GPIO41 / GPIO42 |
| I2C do OLED SDA / SCL (barramento separado) | GPIO1 / GPIO2 |
| Reserva: GPS RX / TX | GPIO39 / GPIO40 |
| Reserva: AUX1 / AUX2 (servos), AUX3, buzzer, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Bateria / sensor de corrente (ADC, registrado pela caixa-preta); reserva: telemetria TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Só bancada: SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ CS do BMP388: GPIO21) |

> O barramento dos sensores ficava antes nos GPIO8/9 e foi movido para o 41/42 para acompanhar o layout
> da placa do controlador de voo. Na bancada: SDA 8→41, SCL 9→42.

Não use: GPIO0/45/46 (o modo de boot depende deles), 19/20 (USB), 26–32 (flash), 33–37 (PSRAM na N16R8), 43/44 (conector "COM"), 48 (LED RGB). Os pinos livres já estão distribuídos como reserva; uma placa-base com conectores para o futuro: [`FC_BOARD.md`](FC_BOARD.md).

Ligação dos sensores na bancada (todos os módulos funcionam com **3,3 V**, e não com 5 V):

| Módulo | Pinos |
|---|---|
| MPU-6050 / GY-521 (na placa pode haver um MPU6500, e isso é normal) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL: não ligar. Módulo MPU-6500 avulso (10 pinos): o mesmo, mais **NCS–3.3V** (senão o chip passa para SPI) e FSYNC–GND; EDA/ECL: não ligar. Com o chip para cima e a seta X apontando para o nariz; a rotação dos eixos do chip é definida por `IMU_ROTATION_CW_DEG` em `Config.h` (90 no nosso clone) |
| BMP581 | VCC–3.3V (**só 3.3V**: muitos módulos não têm regulador próprio), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (endereço 0x46; não deixar solto), **CSB–3.3V** (senão o chip passa para SPI), INT: não ligar |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY: não ligar. Longe dos fios dos servos, do ESC e do motor |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

Os servos são alimentados **não pela placa**, e sim pelo BEC do controlador de velocidade (ou por uma fonte separada de 5 V com pelo menos 2 A); o terra de todas as fontes é comum. Não ligue o fio vermelho do ESC aos 5 V da placa enquanto o USB estiver conectado.

### esp32-c3: o antigo protótipo, já voou

`pio run -e esp32-c3`, placa `esp32-c3-devkitm-1`.

| Função | GPIO |
|---|---|
| Aileron, semiasa esquerda | GPIO5 |
| Aileron, semiasa direita | GPIO4 |
| Profundor | GPIO6 |
| ESC (acelerador) | GPIO7 |
| Leme | — (não há pinos livres) |
| iBUS do receptor (RX) | GPIO8 |
| I2C SDA (sensores) | GPIO1 |
| I2C SCL (sensores) | GPIO3 |

### esp32-dev (a ESP32 clássica comum, de 38 pinos): para a bancada e a depuração, NÃO VOOU

`pio run -e esp32-dev`, placa `esp32dev`.

| Função | GPIO |
|---|---|
| Aileron, semiasa esquerda | GPIO13 |
| Aileron, semiasa direita | GPIO14 |
| Profundor | GPIO27 |
| ESC (acelerador) | GPIO26 |
| Leme | GPIO25 |
| iBUS do receptor (RX) | GPIO16 |
| I2C SDA (sensores) | GPIO21 |
| I2C SCL (sensores) | GPIO22 |

A vantagem dela é ser a placa mais comum e mais barata da linha: serve para depurar o firmware na bancada, mas a pinagem não foi verificada em hardware.

---

## Ligação do receptor

Tudo o que se precisa do receptor é **um fio de dados iBUS**, que na maioria dos receptores compatíveis com FlySky sai por uma porta própria (muitas vezes marcada como "iBUS", ou é a única saída que não é PPM). A ligação:

- **TX do receptor (saída iBUS)** → **pino RX da placa** da tabela acima (GPIO8 na esp32-c3, GPIO17 na esp32-s3, GPIO16 na esp32-dev).
- **Terra (GND) do receptor** → **GND da placa**. É obrigatório; sem terra comum o protocolo não funciona.
- **Alimentação do receptor**: por um BEC/regulador separado ou pelos 5 V da placa, conforme você costuma alimentar o receptor nas suas montagens; isso não depende de nenhum pino específico do firmware.

O firmware não envia nada de volta ao receptor, apenas escuta; por isso a linha TX da placa não precisa ser ligada em lugar nenhum.

A velocidade da porta iBUS no firmware é 115200 baud, que é o padrão do protocolo; não é preciso mudá-la, porque o próprio receptor mantém essa velocidade.

---

## Mapa de canais RC

O mapa foi verificado na bancada com um rádio FS-i6 (10 canais, modo 2) e um receptor FS-iA6B.

| Canal | Controle do rádio | Nome | O que faz |
|---|---|---|---|
| CH1 | stick direito ←→ | AILERON | Rolagem: ailerons (2000 = para a direita) |
| CH2 | stick direito ↑↓ | ELEVATOR | Arfagem: profundor (2000 = stick para a frente, nariz para baixo) |
| CH3 | stick esquerdo ↑↓ | THROTTLE | Acelerador. 1000 µs = desligado, 2000 µs = máximo, sem limitação |
| CH4 | stick esquerdo ←→ | RUDDER | Leme e roda direcional do trem de pouso (um único servo) |
| CH5 | SwA | ARM | Chave ARM: veja a seção sobre ARM abaixo |
| CH6 | SwB | SWB | Por padrão, flaps: para baixo, em sua direção, estendidos; para cima, recolhidos (veja abaixo) |
| CH7 | SwC (3 posições) | SWC | Por padrão, modo: em cima MANUAL, no meio STABILIZE, embaixo AUTO_TAKEOFF |
| CH8 | SwD | SWD | Por padrão, RTH (para casa) enquanto estiver ligada |
| CH9 | VrA | VRA | Por padrão, intensidade da estabilização |
| CH10 | VrB | VRB | Por padrão, velocidade de cruzeiro |

Os canais CH6–CH10 podem ser **qualquer coisa, com uma única linha** em `include/config/Controls.h`: qualquer um dos 12 modos, das 10 funções (flaps, freio, lançamento de carga, geofence, buzzer…) e dos 7 potenciômetros. Quando a placa liga, a distribuição real é impressa no monitor serial. Tudo sobre os modos e as atribuições está em [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

Todos os valores dos canais usam a faixa padrão de pulso do receptor, de 1000 a 2000 µs, com 1500 µs como centro/neutro.

### Flaps (flaperons)

Não há flaps separados: os ailerons fazem esse papel. **SwB para baixo** (em sua direção): os dois ailerons descem suavemente, em cerca de um segundo, no mesmo ângulo (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° de giro do braço do servo; muda-se em `include/config/Config.h`). Isso aumenta a sustentação para decolar e pousar em velocidade menor. A rolagem pelo stick e pelo piloto automático funciona como de costume: os ailerons se movem em sentidos opostos, mas agora em torno da posição abaixada. **SwB para cima**: eles recolhem com a mesma suavidade. No OLED, com os flaps estendidos, `FL` acende na primeira linha.

Com rolagem total e flaps estendidos, o aileron que desce chega ao fim do curso antes do que sobe; isso é normal e funciona como diferencial de ailerons.

Estender os flaps costuma levantar o nariz: esteja pronto para empurrar um pouco o stick para a frente; se o efeito for forte, ele se ajusta reduzindo `FLAPS_DEPLOYED_US`.

---

## Canais do piloto automático

Em resumo, a distribuição padrão (`include/config/Controls.h`):

| Chave | O que faz |
|---|---|
| **SwC** (CH7) | em cima **MANUAL** · no meio **STABILIZE** · embaixo **AUTO_TAKEOFF** |
| **SwD** (CH8) | **RTH**: para casa enquanto estiver ligada |
| **SwB** (CH6) | flaps |
| **VrA / VrB** (CH9/10) | intensidade da estabilização / velocidade de cruzeiro |

- **STABILIZE: "o stick define o ângulo".** Com o stick no fim do curso, 45° de rolagem e 25° de arfagem; ao soltar, o avião se nivela sozinho. O acelerador é seu.
- **AUTO_TAKEOFF.** Depois do ARM nada acontece até você levantar o acelerador acima da metade. Em seguida: 0–1 s, acelerador subindo suavemente até 100%, asas niveladas; 1–3 s, arfagem de +15°; depois +10° até você mexer na SwC. Acelerador = o maior valor entre o stick e o programa.
- **RTH.** Rumo ao ponto do ARM, altitude de 40 m, círculos sobre o ponto de origem. Precisa de um GPS com fix 3D **antes do ARM**.

Os 12 modos (ALT_HOLD, ACRO, CRUISE, LOITER, LAUNCH manual, AUTO_LAND, SOARING, RESCUE…), o que cada um exige em sensores e como colocá-lo em uma chave estão em [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). Um modo escolhido pelo dashboard ou pela estação de solo permanece até você mexer na chave de modo.

A estabilização move as superfícies de comando mesmo quando a aeronave **não está armada**: assim, na bancada, dá para ver para que lado ela responde a uma inclinação. O integrador, nesse caso, não acumula.

Sinais dos ângulos (como no OLED e no log): **rolagem R > 0: asa direita para baixo; arfagem P > 0: nariz para cima.**

---

## ARM e failsafe

### Procedimento de ARM

- **ARM:** acelerador (CH3) embaixo → chave **SwA (CH5) para baixo, em sua direção** (na FS-i6 isso é CH5 = 2000; para cima = 1000). No Serial aparece `ArmingManager: ARM`, e no OLED, `ARMED`.
- **DISARM:** SwA para cima, para longe de você: instantaneamente, a qualquer momento; o motor para na hora.
- Se a SwA for levada para ARM com o acelerador fora da posição mais baixa, ou se as verificações pré-voo não passarem, o ARM **não** acontece (o motivo é impresso no Serial). É preciso devolver a SwA para cima, abaixar o acelerador e levá-la para baixo de novo.
- Se a placa for ligada com a SwA já na posição de ARM (para baixo), ela **não** arma: o firmware precisa ver primeiro a SwA para cima (OFF).

**O ARM bloqueia de fato o acelerador:** enquanto a aeronave não está armada, o acelerador do ESC é mantido à força no mínimo, seja qual for a posição do stick. Antes de armar, verifica-se se os sensores exigidos pelo modo escolhido respondem; por exemplo, o STABILIZE sem um IMU funcionando não arma até você passar para o MANUAL. Mesmo assim, comporte-se como se a hélice pudesse começar a girar a qualquer momento depois do ARM.

### Failsafe

A perda de link tem **prioridade absoluta** sobre todo o resto:

- **aeronave armada, com GPS e ponto de origem**: **retorno para casa com motor** (como no ArduPilot/INAV): rumo ao ponto de origem, altitude de 40 m, círculos sobre ele até o link voltar. No OLED aparece `FSRTH`. Desativa-se com `FAILSAFE_RTH = false` em `Config.h`;
- **aeronave armada, sem GPS**: **planeio**: motor desligado; o piloto automático, em qualquer modo, até no MANUAL, mantém as asas niveladas e o nariz um pouco abaixo do horizonte (−3°), e os flaps recolhem. No OLED aparece `RX LOST ... GLIDE`. Se você definir `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, o avião fará uma espiral suave sobre você;
- **desarmada** (no solo) ou com o IMU sem responder: motor desligado; ailerons, profundor e leme em neutro (1500 µs); depois de 10 s sem link toca o buzzer "estou aqui" (se estiver soldado).

O firmware reconhece a perda de link de duas maneiras:

1. **Sem quadros iBUS por mais de 500 ms**: fio rompido ou receptor sem alimentação.
2. **Acelerador abaixo de 950 µs**: é assim que o receptor avisa que perdeu o rádio. **Isso exige configurar o failsafe no rádio** (veja abaixo): quando o link se perde, o FS-iA6B NÃO deixa de enviar quadros, e sim repete os últimos valores dos sticks; sem a configuração, o firmware não perceberá a perda de link, e o avião continuará voando com o último acelerador.

O ARM **não** é desfeito pelo failsafe: quando o link volta, o avião volta a obedecer aos sticks e ao modo escolhido sem precisar armar de novo (mexer na chave no ar com o acelerador no zero é mais perigoso).

Teste na bancada (sem a hélice): ARM → desligar o rádio → no OLED aparece `RX LOST ... GLIDE` e o motor parou; incline o avião: as superfícies de comando devem devolvê-lo ao nivelamento. Ligue o rádio: `RX ok`, e o controle volta aos sticks.

### Configuração do failsafe no rádio FS-i6 (obrigatória, uma única vez)

A ideia: ao perder o link, o receptor deve entregar um acelerador de cerca de 900 µs, abaixo do mínimo normal de 1000.

1. `Menu → Functions setup → End points` → canal 3: coloque o ponto inferior (o valor da esquerda) em **120%**. Salve (pressione Cancel por um tempo).
2. Acelerador **todo para baixo**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, com o acelerador ainda embaixo → salve com um toque longo em Cancel. O receptor vai memorizar cerca de 900 µs.
4. Volte para `End points` → canal 3 → devolva o ponto inferior a **100%**. Salve.
5. Teste: não é preciso ARM. Desligue o rádio; em cerca de 1 s aparece no Serial `RX=LOST(failsafe пульта)` e, no OLED, uma linha `RX LOST` em vídeo invertido. Ligue o rádio: `RX=OK`.

Mantenha o trim do acelerador no centro: com o trim muito baixo, o acelerador pode cair abaixo de 950 e o firmware vai interpretar isso como perda de link.

Antes havia aqui um **boost no CH8** e um limite de acelerador de 40%; foram removidos: o limite protegia uma montagem 3S1P fraca, e as baterias novas não temem o acelerador total. Além disso, o boost se ativava sozinho se a SwD estivesse para cima quando a placa era ligada.

---

## Gravação do firmware na placa

O firmware é compilado com o **PlatformIO** (framework Arduino, C++).

### Instalação do PlatformIO

O mais fácil é instalar a extensão **PlatformIO IDE** no VS Code (Extensions → procurar "PlatformIO IDE" → Install); assim você terá a CLI e também botões práticos de build na interface. Também é possível instalar com `pip install platformio` e trabalhar pelo terminal; as duas opções usam os mesmos comandos `pio`.

### Build e gravação

Abra o projeto (a pasta do repositório) no VS Code com o PlatformIO instalado, conecte a placa por USB e execute no terminal o comando correspondente à sua placa:

```bash
# esp32-s3 N16R8 (placa principal)
pio run -e esp32-s3 -t upload

# esp32-c3 (antigo protótipo)
pio run -e esp32-c3 -t upload

# esp32-dev (ESP32 clássica de 38 pinos, para a bancada)
pio run -e esp32-dev -t upload
```

Se você não indicar `-e`, será compilada a placa padrão: `esp32-s3`.

**esp32-s3:** a placa tem dois conectores USB-C. A gravação e o Serial passam pelo conector **"COM"** (ponte CH343; no Windows aparece como "USB-Enhanced-SERIAL CH343"). O conector "USB" (o USB nativo do chip) não é necessário para o trabalho, mas pode ficar conectado: não atrapalha em nada.

### Monitor serial

Para ver a saída de depuração (estado dos canais, ARM, sensores) diretamente no console pelo USB:

```bash
pio device monitor -b 115200
```

A velocidade tem de ser 115200; senão você verá uma confusão ilegível em vez de texto. A cada 10 segundos é impressa uma linha `SYS`: a frequência do laço (deve ficar em torno de 500 Hz), o tempo médio e o pior do laço em 10 s, os contadores do iBUS e a memória livre. O restante sai pelos canais ativados no menu do log (veja abaixo).

### Console: menu e log (monitor serial)

A tecla age na hora; não é preciso Enter (em um monitor que envia por linha, digite a letra + Enter). As calibrações e a verificação das saídas só funcionam com a aeronave desarmada.

| Tecla | O que faz |
|---|---|
| `h` | **menu principal** (de texto, com itens numerados) |
| `l` | menu "o que mostrar no log" |
| espaço | pausar o log / retomar |
| `s` | status detalhado de todos os sensores (inclusive os contadores de erros do I2C) |
| `i` | recalibrar o giroscópio e fazer a verificação pré-voo do IMU: 2 s, sem mexer no avião |
| `o` | **calibração da instalação do IMU**: uma vez, depois de montar a placa no avião (veja abaixo) |
| `m` | calibração da bússola: por 15 s, gire a placa ou o avião em todos os eixos. O resultado é salvo na flash e sobrevive a uma reinicialização |
| `p` | verificação das saídas: o pulso real em cada pino |

**Log por canais.** Cada tipo de dado é uma linha com o seu próprio prefixo, e cada um tem o seu modo: **desligado**, **ao mudar** (a linha só aparece quando os valores realmente mudaram; a vibração dos sticks e o ruído dos sensores não contam) ou **contínuo** (a cada 0,2 / 0,5 / 1 / 2 s; o período é definido no mesmo menu).

| Canal | O que mostra | Padrão |
|---|---|---|
| `STAT` | link, ARM, modo, flaps, se o IMU e o barômetro estão bem | ao mudar |
| `RC` | canais do rádio, µs | desligado |
| `OUT` | saídas para as superfícies de comando e o ESC, µs | desligado |
| `ATT` | rolagem, arfagem, rumo | desligado |
| `AP` | piloto automático: alvos e correções | desligado |
| `ALT` | altitude, velocidade vertical, alvo do ALT_HOLD | desligado |
| `MAG` | rumo pela bússola | desligado |
| `GPS` | fix, satélites, coordenadas, velocidade | desligado |
| `IMU` | giroscópio e acelerômetro | desligado |
| `SYS` | frequência e duração do laço, memória (a cada 10 s) | ligado |

Enquanto o menu está aberto, o log fica em silêncio para que o menu não suma da tela; depois de sair, todos os canais ativados são impressos de novo. A seleção é salva na flash ao sair do menu e sobrevive a uma reinicialização. Com a aeronave armada, ela vale na hora, mas só é gravada depois do DISARM: uma gravação na flash para o laço de voo por cerca de 0,4 s.

### Instalação do IMU: como quiser, com uma única calibração

A placa com o IMU pode ser instalada no avião **do jeito que for conveniente**: em qualquer ângulo, de lado, de cabeça para baixo; o firmware descobre sozinho onde ficam o nariz e o topo dela. Isso se faz **uma única vez** depois da instalação (e de novo, se a placa for mudada de lugar):

1. O avião sobre a mesa, sem precisar do rádio, com o motor desarmado. No monitor serial, pressione `o`.
2. **Passo 1:** o avião fica nivelado, como em voo horizontal (se tiver roda de cauda, calce algo sob a cauda). Não encoste nele por cerca de 3 s.
3. **Passo 2:** levante o **nariz** de 30 a 60°, com as asas niveladas, e segure parado por cerca de 1 s.
4. **Passo 3:** abaixe o nariz de volta, baixe a **asa direita** de 30 a 60° e segure por cerca de 1 s.

Cada passo é contabilizado automaticamente (no log aparece "засчитано", ou seja, "contado"). No fim, é mostrado o resultado ("нос = +Y чипа, верх = −Z чипа", ou seja, "nariz = +Y do chip, topo = −Z do chip") e "установка сохранена" ("instalação salva"). Se você confundiu algo (abaixou o nariz em vez de levantar, a asa esquerda em vez da direita), a calibração é recusada com uma explicação; basta repetir `o`. O resultado fica na flash e sobrevive a uma reinicialização. Verificação: nariz para cima → o P no OLED fica positivo; asa direita para baixo → o R fica positivo.

Enquanto não houver calibração da instalação, vale o método antigo: a placa deve ficar com o chip para cima, e a rotação é `IMU_ROTATION_CW_DEG` em `Config.h`.

### Verificação pré-voo ao ligar

A cada vez que é ligado, o IMU gasta cerca de 2 s calibrando o giroscópio e, ao mesmo tempo, verifica a si mesmo:

- **o avião está parado**: se naquele momento ele estava sendo segurado nas mãos ou movimentado, o desvio do giroscópio ficará errado;
- o acelerômetro em repouso marca 1g;
- **o "topo" coincide com a calibração da instalação**: se a placa foi mudada de lugar ou virada, isso aparece na hora (um avião sobre a roda de cauda ou numa ladeira não é problema; a tolerância é de 45°).

**Ligue o avião com ele parado.** Não precisa estar nivelado, se a instalação estiver calibrada (senão, a posição ao ligar vira o horizonte). O resultado aparece no log: `предполётная проверка пройдена` ("verificação pré-voo aprovada") ou `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` ("VERIFICAÇÃO PRÉ-VOO REPROVADA", seguida do motivo). Se não passou, **o ARM em STABILIZE e AUTO_TAKEOFF é proibido** (o motivo é impresso quando você tenta), e o piloto automático não dá correções em nenhum modo, nem no planeio por perda de link: com ângulos errados ele comandaria no sentido contrário. O ARM em MANUAL continua possível. Para corrigir: deixe o avião parado e reconecte a bateria (ou pressione `i`); se a placa foi mudada de lugar, pressione `o`.

### Tela OLED

Se o OLED estiver conectado (GPIO1/GPIO2), ele mostra o seguinte 5 vezes por segundo:

```
RX ok disarm STAB        <- link / ARM / modo (se o link se perde, a linha fica invertida)
R  +1.2 P  -0.4          <- rolagem / arfagem, °
Alt +0.3 Vz +0.1         <- altitude em relação ao ponto de ligação, m / velocidade vertical, m/s
Hdg 123  Thr 1000        <- rumo pela bússola / acelerador para o ESC, µs
L1500 R1500 E1500        <- PWM dos ailerons e do profundor, µs
Loop 500Hz max 1100us    <- frequência e pior duração do laço
```

---

## Painel web em campo

A placa cria o seu próprio ponto de acesso Wi-Fi: não é preciso roteador de casa nem internet, e tudo funciona direto no campo, a partir do celular.

**Como conectar:**

1. No celular ou no notebook, abra a lista de redes Wi-Fi.
2. Conecte-se à rede **`OpenPlane-Debug`**, senha **`12345678`**.
3. Abra no navegador o endereço **`http://192.168.4.1`**.

Não é preciso instalar nenhum aplicativo: é uma página web comum.

**O que dá para fazer pelo celular, no campo, sem programar nada:**

- Ver as **barras ao vivo dos 10 canais RC**: é prático para conferir se o rádio e o receptor realmente enviam o que você mexe no stick, antes mesmo de ligar os servos.
- Ver o status de cada saída (aileron esquerdo/direito, profundor, ESC): se o canal está conectado por software.
- Ver se os sensores (IMU, barômetro) respondem, caso estejam soldados: o painel mostra com honestidade ou dados reais (rolagem/arfagem/altitude), ou uma indicação explícita de que o sensor fisicamente não existe ou não responde.
- **Trocar o modo do piloto automático** com botões (manual / estabilização / decolagem automática / manutenção de altitude) direto da página, sem o rádio.
- **Ajustar os coeficientes do controlador PID** (de rolagem e arfagem) por um formulário na página: útil para afinar a estabilização aos poucos, sem regravar o firmware.

O alcance desse ponto de acesso é, na prática, de algumas dezenas de metros; é o Wi-Fi comum do ESP32, não uma telemetria de longo alcance. É uma ferramenta para ajustes na mesa, na bancada e ao lado do campo, não para controlar a aeronave em voo à distância.

---

## Caixa-preta

O firmware do ESP32-S3 grava cada voo na flash por conta própria: tudo o que os sensores viram, o que os sticks fizeram, para onde foram os servos e o que o piloto automático decidiu. Não é preciso fazer nada:

- **grava** a partir do momento em que a aeronave está armada e o acelerador é levantado (mais os 10 s anteriores);
- **para** 10 s depois do DISARM, ou se uma aeronave armada ficar parada, com o motor desligado, por 30 s (pousou ou caiu, e o DISARM foi esquecido);
- a perda de link, o motor em zero e o planeio **não** interrompem a gravação.

Ao ligar, o monitor serial mostra quanto espaço há para um voo (o console imprime em russo; a linha abaixo diz "espera ARM e acelerador | apagado à frente 12,9 MB (≈11 min) de 13,9 MB | voos 1"):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**Depois do voo**, conecte o USB (o conector COM), feche o monitor serial e baixe o voo:

```bash
python tools/blackbox.py download
```

O voo vai parar na pasta `blackbox/`, junto com a análise: `summary.txt` (resumo e eventos), `events.txt` e tabelas CSV por sensor. A caixa-preta apaga sozinha os voos antigos quando precisa de espaço, então **baixe depois de cada saída**. O menu no console é a tecla `k`. Tudo em detalhes está em [BLACKBOX.md](BLACKBOX.md).

**Na placa STM32H743**, a caixa-preta grava em um cartão SD: o cartão é formatado em FAT32 e preparado uma única vez em um PC (`python tools/blackbox.py sd-prepare E:`). Depois do voo, ele pode ser baixado por USB com o mesmo comando `download`, ou você pode tirar o cartão e decodificar o arquivo direto dele: `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Checklist pré-voo e segurança

Leia esta seção inteira **antes** de ligar pela primeira vez, e não depois de um incidente.

### Obrigatório antes de qualquer teste na bancada

- [ ] **A hélice está fisicamente RETIRADA** se você estiver verificando canais, ARM ou o painel, ajustando o PID, ou simplesmente ligando a placa pela primeira vez com uma nova pinagem. O ESC pode dar um tranco no motor em qualquer etapa do teste: não é um risco hipotético, é o comportamento normal na primeira ligação.
- [ ] A bateria LiPo foi verificada quanto a estufamento e danos, foi carregada com um carregador de LiPo adequado e é guardada e carregada sobre uma superfície não inflamável.
- [ ] O rádio está ligado e seus canais foram verificados no painel (`http://192.168.4.1`) **antes** de a bateria ser conectada ao ESC.

### Antes do voo

- [ ] Uma área aberta, sem pessoas nem construções em um raio suficiente para um avião de 1200 mm de envergadura, sob controle manual, com comportamento anormal dos servos ou da asa (veja abaixo a seção sobre as limitações do protótipo: a fixação do motor e a asa ainda não foram reforçadas com carbono).
- [ ] As três superfícies se movem no sentido certo; confira na mesa antes de cada saída, em vez de confiar na memória da vez anterior:
  - stick direito para a direita → **aileron direito para cima, esquerdo para baixo**;
  - stick direito para você → **profundor para cima**;
  - stick esquerdo para a direita → **leme e roda direcional para a direita**;
  - SwB (flaps) para baixo → **os dois ailerons descem suavemente**, e a rolagem pelo stick continua separando-os em sentidos opostos;
  - em STABILIZE, incline o avião com a asa direita para baixo → **aileron direito para baixo, esquerdo para cima** (as superfícies o devolvem ao horizonte); nariz para baixo → **profundor para cima**.
  Se algo estiver errado, mude o `*_REVERSED` correspondente em `include/config/Config.h` (seção "Direção dos servos"), e não a reversão no rádio: senão o stick e o piloto automático vão divergir.
- [ ] Ao ligar, o avião estava parado e não há `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА` no log nem no painel; ao inclinar o avião nas mãos, o P e o R no OLED mudam no sentido certo.
- [ ] O failsafe está configurado no rádio e foi testado: desligue o rádio → `RX LOST` no OLED ou no Serial (veja a seção sobre failsafe).
- [ ] O alcance do rádio foi testado e a bateria do rádio está carregada.
- [ ] Ao ligar, o log mostra `BlackBox: ... стёрто впереди N МБ (≈M мин)`: há espaço para o voo. O voo anterior foi baixado (`python tools/blackbox.py download`), se você precisar dele.
- [ ] Mantenha as mãos e o rosto longe da hélice sempre que uma LiPo estiver conectada ao ESC: depois do ARM, o acelerador responde na hora ao stick, sem nenhum aviso adicional. DISARM: SwA para cima.
- [ ] Verifique se você consegue, fisicamente, cortar a alimentação rápido (acesso ao conector da LiPo), em vez de confiar apenas no failsafe por perda de sinal.

### Segurança geral com LiPo

- Nunca deixe uma LiPo em carga sem vigilância.
- Não conecte nem desconecte a LiPo do ESC estando ao lado do plano de rotação da hélice.
- Transporte e guarde as LiPos em uma bolsa ou caixa de proteção.

---

## Solução de problemas

**"Sem sinal do receptor" / o RX parece normal, mas os canais no painel não se mexem**
Verifique se o fio de dados do receptor está ligado exatamente no pino RX do iBUS da tabela da sua placa (GPIO8/GPIO17/GPIO16), sem ter sido trocado com o terra ou a alimentação, e se o terra do receptor e o da placa estão unidos. Se a pinagem confere e os fios estão íntegros, mas ainda assim não há sinal, verifique se o receptor está de fato pareado (bind) com o rádio e se a saída do receptor está configurada para iBUS, e não para PPM/SBUS.

**Um sensor (IMU, barômetro, bússola) mostra "não responde" / NO_RESPONSE**
O firmware informa com honestidade que o sensor não responde, em vez de entregar zeros. Verifique: (1) a alimentação do módulo (3.3V e o terra da placa); (2) SDA/SCL, nos pinos I2C da sua placa em particular; (3) o endereço na linha: MPU 0x68 (AD0 no GND), BMP581 0x46 (SDO no GND, CSB no 3.3V; 0x47 se o SDO estiver no 3.3V), BMP388 0x76 (SDO no GND, **CSB no 3.3V**; senão o chip fica em modo SPI), QMC5883P 0x2C, QMC5883L 0x0D. Se o sensor às vezes responde e às vezes não (ou atende em um endereço alheio), é mau contato na protoboard: aperte VCC/GND/SDA/SCL e, de preferência, alimente cada módulo direto do 3.3V/GND da placa. O comando `s` no console mostra os contadores de erros de I2C de cada sensor.

**Os ângulos no OLED estão trocados (nariz para cima muda o R, não o P) ou com o sinal invertido**
Faça a calibração da instalação do IMU (`o`, veja "Instalação do IMU"): ela não depende de como o chip está soldado no módulo nem de como o módulo está posicionado no avião. Sem ela: nos clones do GY-521, o chip às vezes vem soldado girado em relação às setas impressas; gire os eixos em `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Verificação: nariz para cima → o P fica positivo; asa direita para baixo → o R fica positivo.

**ARM recusado: "IMU: ..."**
A verificação pré-voo do IMU não passou (veja "Verificação pré-voo ao ligar"): o avião foi mexido ao ligar; deixe-o parado e reconecte a bateria; o "topo" não coincide com a calibração: a placa foi mudada de lugar, faça `o`; "a placa não está com o chip para cima": a instalação não está calibrada, faça `o`.

**Um servo ou o ESC reage ao stick errado / não reage**
O comando `p` no console mede o pulso real em cada saída (GPIO4–7) e o compara com o esperado. Se estiver tudo "OK" e o servo não se mexer, o problema está além da placa: (1) o servo não está alimentado (BEC/5V, terra comum); (2) o fio de sinal está no pino errado; (3) a mecânica travou. "НЕ СОВПАДАЕТ" ("NÃO CONFERE") indica que o problema está no firmware ou nos periféricos: avise o desenvolvedor.

**O motor não gira de jeito nenhum, embora o acelerador no OLED ou no painel responda ao stick**
Verifique se a LiPo está conectada ao ESC e se a aeronave está realmente armada: até o ARM, o acelerador para o ESC é mantido à força no mínimo, e isso não é defeito. Um ESC que, ao ligar, viu um acelerador acima do mínimo pode apitar sem parar e não armar: reconecte a bateria com o acelerador embaixo. Se `ArmingManager: ARM` não aparecer depois de abaixar o SwA, olhe o Serial: o firmware imprime o motivo (acelerador não está embaixo, um sensor não responde, um sensor que o modo escolhido no momento em CH7 exige); volte o SwA para cima, elimine a causa e abaixe-o de novo.

**O motor engasga ou dá tranco ao acelerar**
Se o ESC é alimentado por uma fonte de bancada, você está esbarrando no limite de corrente da fonte: mesmo sem hélice, o motor puxa por um instante vários ampères, a tensão cai e o ESC reinicia. Aumente o limite de corrente ou use uma LiPo. Os picos de uma reinicialização dessas podem travar a ponte USB da placa (a porta "COM" para de abrir); reconecte o cabo.

**Depois de gravar o firmware, a placa não responde / o ponto de acesso `OpenPlane-Debug` não aparece**
Confirme que a gravação (`pio run -e <sua placa> -t upload`) terminou sem erros e que você gravou exatamente o ambiente da placa que está em suas mãos (a esp32-c3 difere da esp32-s3 e da esp32-dev não só nos pinos, mas também no chip: o firmware de outro chip não instala na placa, ou instala de forma incorreta). Veja a saída do monitor serial (`pio device monitor -b 115200`) logo depois de a placa reiniciar: ali é impresso em que etapa do setup() a placa está.

---

## Estado atual da estrutura do protótipo

Para que as expectativas sejam honestas:

- O primeiro protótipo **já voou**. Foram identificados problemas: **resistência insuficiente da fixação do motor** e **resistência insuficiente da asa**; a asa precisa de reforço com carbono. Também é preciso afinar mais os servos. Leve isso em conta ao planejar seus próprios voos: não é uma ressalva abstrata, e sim uma falha real que já aconteceu com este protótipo.
- **A bancada com a esp32-s3 está montada com todos os sensores** (GY-521 com um MPU6500, BMP388 por I2C, GY-273 com um QMC5883P, OLED): todos respondem, e o laço roda a 500 Hz. Os modos do piloto automático foram verificados na mesa, mas **ainda não foram testados em voo**.
- O barômetro padrão agora é o BMP581 (o BMP388 da bancada foi verificado ao vivo; o BMP581 ainda não foi testado no hardware). A altitude é relativa ao ponto de ligação. A altitude absoluta acima do mar vem da atmosfera padrão, sem correção para o tempo.
- A bússola QMC5883P precisa de calibração (`m` no console) já no avião montado: perto do motor e dos fios, os desvios são diferentes dos da protoboard. O rumo ainda não tem compensação de inclinação e não é usado por nenhum modo.
- A licença é a OpenPlane License: MIT com atribuição obrigatória ao autor (Damir Lebedev), proibição de uso militar e proibição de causar dano intencional a pessoas e bens sem o consentimento delas, veja [LICENSE](LICENSE.md).

Se você estiver montando a sua própria aeronave com este guia, voe primeiro com controle manual (MANUAL) e só depois passe ao piloto automático.
