# OpenPlaneProject — roteiro e apresentação para investidores e parceiros

> 🌐 Esta página é uma tradução do [original em russo](../../ROADMAP.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

> Repositório: [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), ramo `main`.
> Este documento é uma versão mais aprofundada do teaser do README, dirigida a quem está avaliando investir dinheiro,
> tempo ou uma parceria no projeto. Ele descreve onde o projeto está agora, para onde e por que ele caminha,
> e o que exatamente no código já escrito torna essa trajetória realista, e não apenas declarada.

## 1. Situação atual — com honestidade

O OpenPlaneProject é hoje um protótipo de planador que já voou, mas ainda imaturo, com firmware próprio no ESP32 — não é um produto pronto nem um drone autônomo. Hardware: envergadura de 1200 mm, corda de 250 mm, perfil NACA 4412, estrutura de PETG (impressa em 3D), servos MG90S (um separado para cada aileron), alimentação por LiPo 3S. O primeiro protótipo (ESP32-C3, motor D2212 1000KV, ESC de 40A) já voou com controle manual e, depois do voo, revelou problemas concretos: resistência insuficiente da fixação do motor e da asa (é preciso reforçar com carbono) e a necessidade de ajustar os servos.

A montagem atual migrou para o ESP32-S3 (N16R8) com motor D3548 1100KV e ESC de 60–80A, e na bancada estão ligados a ela todos os sensores do piloto automático: IMU (MPU6500), barômetro BMP388, bússola QMC5883P e uma tela OLED de status. Verificado ao vivo: controle manual por rádio via iBUS com mixer de ailerons, profundor, leme (com a roda direcional) e flaps (flaperons); ARM por uma chave separada; failsafe com o rádio desligado; ciclo de controle de 500 Hz; dashboard web ao vivo por Wi-Fi. Sobre a mesa, a estabilização responde às inclinações no sentido certo.

De lá para cá, o firmware ganhou 12 modos de piloto automático (estabilização, manutenção de altitude, cruzeiro, círculos e retorno ao ponto de origem por GPS, lançamento manual, pouso automático, voo planado em térmicas, “resgate”), geofence, retorno ao ponto de origem na perda de sinal, lançamento de carga, um tubo de Pitot feito de dois barômetros, suporte a novos sensores (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) e um firmware completo para o STM32H743 com telemetria MAVLink para o QGroundControl. Tudo isso foi verificado por 387 testes automatizados e simulações de voo em malha fechada (o firmware inteiro pilota um modelo de avião), mas **ainda não foi testado no ar**. Em outras palavras: até agora só o controle manual voou; o piloto automático está escrito, verificado por tudo que dá para usar sem voar, e aguarda os testes em voo.

## 2. Por que isso importa

A aviação pequena autônoma com baixa barreira de entrada cobre tarefas em que o que decide não é a carga útil nem o alcance máximos, mas a rapidez de reação e o baixo custo de operação:

- **Entrega de medicamentos e sangue a regiões de difícil acesso e a áreas pós-desastre** — estradas destruídas, falta de infraestrutura, destruição após desastres naturais ou conflitos. O que decide aqui não é a capacidade de carga (o pacote é pequeno), mas o tempo de reação: minutos e horas em vez de um dia de carro ou a pé.
- **Operações de busca e salvamento** — lançamento pontual de equipamentos, kits de primeiros socorros, meios de comunicação e dispositivos de flutuação às vítimas antes da chegada da equipe em terra, em locais onde o helicóptero é caro demais ou inviável por causa do clima ou do terreno.
- **Agricultura de precisão** — monitoramento de lavouras e pulverização/aplicação pontual de produtos onde as plataformas de drones fechadas para essas tarefas custam a partir de milhares de dólares, o que não compensa para propriedades pequenas e médias.

Nas três categorias a economia é a mesma: a diferença entre “existe solução, mas é cara e fechada” e “na prática não existe solução, porque é cara” — e é exatamente esse nicho que uma plataforma aberta e barata tem como alvo. Esta é uma descrição da área de aplicação e do problema de mercado, e não uma afirmação de que o OpenPlaneProject já sabe entregar cargas — na fase atual, é uma afirmação sobre o objetivo e sobre por que ele vale a pena.

## 3. Tese de investimento: por que uma arquitetura aberta em ESP32 é uma assimetria

As plataformas comerciais de drones autônomos para entrega/monitoramento costumam ser construídas sobre controladoras de voo fechadas e software fechado, custam de centenas a milhares de dólares por aeronave e exigem taxas de licença ou contratos de serviço para operar a frota. O OpenPlaneProject parte de outra premissa:

- **Uma base barata.** Um módulo ESP32 custa da ordem de US$ 5–15, e o restante (servos MG90S, ESC, receptor iBUS) são componentes padrão da categoria hobby. É uma ordem de grandeza mais barato que o ingresso nas plataformas comerciais fechadas, o que é crítico para implantações-piloto em condições de orçamento limitado (ONGs, agricultura de pequeno porte, serviços regionais de resgate).
- **O código aberto muda a economia da confiança.** Uma organização que implanta uma frota para entregas médicas pode auditar a segurança (failsafe, lógica do ARM) e adaptar o firmware aos seus sensores e regulamentos, em vez de depender de um único fornecedor e do roteiro dele.
- **A arquitetura já hoje é projetada para expansão, e não só para o planador atual.** Isso não é uma declaração, mas consequência direta de como o código é organizado:
  - Um novo sensor é adicionado como uma classe que implementa a interface existente `Sensor` → `ImuSensor`/`BarometerSensor` (`include/sensors/SensorInterface.h`), sem alterar o núcleo. Foi assim que já foram feitos os drivers de IMU (MPU6050/MPU6500, ICM-42688), de barômetros (BMP388, BME280), de bússolas (QMC5883P/L) e de GPS (u-blox M10) — todos escritos diretamente pelos registradores do barramento (interfaces `II2CBus`/`ISpiBus`/`IUartPort`), sem bibliotecas de terceiros, isto é, sem dependências ocultas de um SDK específico de fabricante.
  - Uma nova placa é adicionada com um único bloco `#elif` em `include/config/Config.h` mais um único bloco `[env:...]` em `platformio.ini` — a troca de placa já funciona hoje para quatro alvos (veja a tabela abaixo); não é uma possibilidade hipotética.
  - O `Autopilot.h` já recebe `ImuSensor*`/`BarometerSensor*` como parâmetros (podem ser `nullptr`) — ou seja, o contrato entre o piloto automático e o hardware prevê que a composição dos sensores vai mudar (o próximo passo é o GPS como mais uma classe com o mesmo padrão, veja a Fase 3).
  - O `WebDebugServer.h` já entrega um único JSON agregado (`GET /api/status`) e aceita comandos (`POST /api/setmode`, `/api/setpid`) — ou seja, o protocolo “a aeronave entrega telemetria e aceita comandos” já existe, e a estação de solo deve crescer a partir dele, e não ser escrita do zero.

A assimetria está em que a barreira de entrada (dinheiro, tempo de adaptação a uma nova tarefa) desta plataforma é uma ordem de grandeza menor que a dos análogos fechados, e o caminho até a autonomia não exige reescrever o núcleo — apenas adicionar novas classes sobre as interfaces existentes. É uma plataforma de engenharia aberta, e não um produto comercial pronto — assim, a tese para o investidor/parceiro não é “compre uma solução pronta”, mas “entre numa etapa em que a base já foi verificada e os próximos passos são tecnicamente claros”.

## 4. A arquitetura hoje — a base das fases seguintes

### 4.1 Placas compatíveis

A escolha da placa é uma única opção de build do PlatformIO; trocar de placa não exige mexer na lógica (`include/config/Config.h` + `platformio.ini`):

| Ambiente (`pio run -e ...`) | Placa | Status | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (default) | ESP32-S3 N16R8 (DevKitC-1) | **Principal, verificada na bancada com todos os sensores** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | Primeiro protótipo, voou com controle manual | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | ESP32 clássica de 38 pinos | Para a bancada, **não verificada no hardware** (o firmware inteiro está nos testes) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Firmware completo + MAVLink, **ainda sem placa** (o firmware inteiro roda nos testes no PC) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Gravar o firmware: `pio run -t upload`. Monitor: `pio device monitor` (115200).

### 4.2 Mapa dos canais de rádio (FS-i6 + FS-iA6B, iBUS, 10 canais, 1000–2000 µs)

| Canal | Nome | Função padrão |
|---|---|---|
| CH1–CH4 | sticks | rolagem, arfagem, acelerador, leme |
| CH5 | ARM | chave SwA: ARM com o acelerador embaixo, DISARM instantâneo |
| CH6 | SWB | flaps |
| CH7 | SWC | modo: MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH — ponto de origem |
| CH9 | VRA | intensidade da estabilização |
| CH10 | VRB | velocidade de cruzeiro |

CH6–CH10 são atribuídos com uma única linha em `include/config/Controls.h`: qualquer um dos 12 modos, 10 funções (flaps, freio, lançamento de carga, geofence…) e 7 potenciômetros — [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Fluxo de controle (`FlightController::update()`)

Um único orquestrador, com ordem fixa e ciclo de 500 Hz de período fixo: recepção do iBUS → acelerador do piloto → modo vindo do CH7 → **leitura dos sensores e cálculo do piloto automático (sempre, mesmo sem enlace)** → **verificação da perda de sinal com prioridade absoluta** (no ar — de volta ao ponto de origem por GPS com o motor, ou planeio com as asas niveladas sem GPS; no solo — superfícies no neutro) → ARM → comando dos sticks + correções do piloto automático em sinais aeronáuticos unificados → mixer com reversão de servo → acelerador do modo → bloqueio do acelerador sem ARM → escrita nos servos/ESC. É justamente essa disciplina de ordem (primeiro a segurança, depois o controle manual, depois o piloto automático como camada superior) que permite embutir com segurança um comportamento cada vez mais autônomo sem reescrever o ciclo básico. O Wi-Fi, o dashboard e a tela rodam no segundo núcleo e não atrasam o controle.

### 4.4 O dashboard web como embrião da estação de solo

O `WebDebugServer.h` já hoje levanta um ponto de acesso (SSID `OpenPlane-Debug`, IP `192.168.4.1`) e entrega/aceita JSON:

| Método e caminho | O que faz |
|---|---|
| `GET /api/status` | Um único JSON agregado: RC (10 canais), armed/failsafe, 7 saídas (`us`, `attached`), IMU, barômetro, bússola, GPS, tubo de Pitot (cada um com `attached`/`available` + dados), piloto automático (modo, correções, PID, navegação, funções ativadas) |
| `POST /api/setmode` | `{mode: 0-11}` — trocar o modo do piloto automático |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}` — qualquer campo é opcional |
| `GET /` | O dashboard HTML: barras ao vivo dos 10 canais, o status de cada saída/sensor, botões de modo, um formulário de PID |

Os campos `attached`/`available` estão sempre presentes no JSON — o dashboard mostra com honestidade “não está na configuração” separado de “está na configuração, mas não responde”, em vez de silenciar sobre um sensor ausente. É o mesmo princípio de honestidade que sustenta todo o documento: não fazer passar o desejado pelo existente.

## 5. Roteiro técnico fase a fase

A seguir estão oito fases, cada uma descrita como o próximo passo lógico sobre as classes que já existem, sem datas nem valores inventados.

### Fase 1 — Controle manual e uma base segura [pronto]

**Objetivo:** um planador radiocontrolado confiável e previsível, com depuração transparente. **O que já existe tecnicamente:** a análise do iBUS com detecção de perda de sinal em `IBusReceiver.h`, o mixer `ControlMixer.h` (sticks → comando de rolagem/arfagem/flaps → PWM com reversão de servo, sem conhecer UART/PWM), `ThrottleManager.h`, `ArmingManager.h` (ARM por uma chave separada com o acelerador embaixo, DISARM instantâneo), o failsafe com prioridade absoluta em `FlightController.h`, a saída para o Serial (`DebugLogger.h`), o dashboard web (`WebDebugServer.h`) e a tela OLED (`OledDisplay.h`). **Por que isto é a base de todo o resto:** é a única camada que precisa funcionar sempre, mesmo que todas as outras fases ainda não estejam implementadas ou seus sensores estejam desligados — é justamente por isso que o failsafe e o ARM foram escritos primeiro e verificados no hardware (inclusive o comportamento real do receptor FS-iA6B com o rádio desligado).

### Fase 2 — IMU + barômetro → piloto automático [escrito e verificado por testes e simulação, aguardando os testes em voo]

**Objetivo:** o primeiro modo de voo autônomo — estabilização do horizonte, decolagem automática, manutenção de altitude. **O que já existe tecnicamente:** `imu/MPU6050_Sensor.h` (MPU6050 e MPU6500, registradores diretamente, rotação dos eixos conforme a instalação da placa, sinais aeronáuticos, filtro complementar em `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C ou SPI, compensação completa da Bosch, leitura pelo sinalizador de dado pronto, velocidade vertical filtrada), `Autopilot.h` com `PidController` (o termo D vindo do giroscópio, o integrador só acumula depois do ARM) e doze modos (de MANUAL a SOARING e RESCUE), trocados por chaves conforme a tabela de `Controls.h`, pelo dashboard web e pelo QGroundControl, além do failsafe (ponto de origem ou planeio). Cada modo voa em uma simulação em malha fechada de todo o firmware com um modelo de avião (`test/native/test_sim`). Na bancada com o ESP32-S3 todos os sensores respondem e os sinais foram verificados ao vivo: inclinação → correção das superfícies no sentido do nivelamento. **O que é preciso para fechar a fase:** levar a eletrônica para o planador, verificar os sentidos das superfícies na aeronave montada e fazer os primeiros testes em voo — começando pelo STABILIZE em altitude segura.

### Fase 2.5 — Realimentação a partir do avião real [base preparada, verificada em simulação]

**Objetivo:** que o piloto automático dependa não de coeficientes ajustados para uma única velocidade, mas de como o avião real responde ao comando neste exato momento. O PID da fase 2 desvia a superfície “por fórmula” e não verifica o resultado; em baixa velocidade ele corrige de menos, em alta velocidade exagera. **O que já existe tecnicamente** (`include/autopilot/feedback/`, **não conectado** ao firmware): a estimativa da eficácia das superfícies em voo (mínimos quadrados recursivos, reescalonada pela velocidade ∝ V²), o controlador “ângulo → taxa de giro → superfície” com correção complementar (“a superfície não chegou até o fim — gire mais”), a proteção contra perda de velocidade e estol (acelerador, nariz para baixo, asas niveladas), a decolagem de pista ou manual e o pouso por etapas conforme os sensores. Tudo foi verificado por uma simulação em malha fechada do avião na própria placa (`pio test -e esp32-s3 -f test_feedback`, 10 cenários) — inclusive um aileron com o sentido trocado, turbulência, nariz alto com pouco acelerador, decolagem e pouso. **O que é preciso para fechar a fase:** depois dos primeiros voos da fase 2 — o “modo sombra” (a realimentação só registra no log o que teria feito), depois a conexão eixo por eixo, um sensor de velocidade do ar (tubo de Pitot) e um telêmetro para o nivelamento antes do toque. Detalhes — a seção “Realimentação” do [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Fase 3 — GPS e bússola [no firmware, verificada por simulação]

**Status (atualizado): a navegação funciona** — rumo por GPS/bússola/giroscópio com histerese, ponto de origem no ARM, CRUISE, LOITER (um campo vetorial sobre uma circunferência), RTH, geofence; verificado por simulações em malha fechada. A seguir, o registro original da fase. O GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, protocolo UBX-NAV-PVT) e o magnetômetro (duas variantes da placa “GY-273”: QMC5883P — `sensors/mag/QMC5883P_Sensor.h`, que está na bancada atual e foi verificado ao vivo; QMC5883L — `sensors/mag/QMC5883L_Sensor.h`) foram adicionados como classes novas que implementam as interfaces `GpsSensor`/`MagnetometerSensor` (`include/sensors/SensorInterface.h`) pelo mesmo princípio da IMU e do barômetro — `Autopilot.h` e `FlightController.h` não foram reescritos, ganharam mais duas fontes de dados do mesmo jeito (um ponteiro anulável no construtor). De quebra, surgiu a camada HAL (`include/hal/`), por onde os sensores acessam os barramentos — I2C/SPI/UART deixaram de estar ligados diretamente aos `Wire`/`SPI`/`HardwareSerial` específicos do ESP32.

Por enquanto os dados do GPS/bússola estão disponíveis por `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` e em `GET /api/status`, além de um ajuste único do yaw inicial pela bússola na partida — **mas não participam do controle**. Questões em aberto para concluir a fase: conectar o módulo GPS ao ESP32-S3 (os pinos da UART2 já estão reservados), calibrar a bússola na aeronave montada e adicionar ao rumo a compensação de inclinação. No ESP32-C3 um GPS completo é impossível — faltam GPIOs para o TX (veja a pinagem em `DEVELOPER_GUIDE.md`).

### Fase 4 — Voo por pontos de rota (waypoint navigation) [próximo passo]

**Status:** as primitivas estão prontas — `Guidance::rollForCourse`, um círculo em torno de um ponto, o retorno a um ponto (RTH), um canal MAVLink para carregar a missão (por ora, a um pedido de missão a aeronave responde com honestidade “0 pontos”). Falta: o armazenamento da rota, a transição entre os pontos, o protocolo MISSION_* do MAVLink.

**Objetivo:** a aeronave voa por um conjunto dado de coordenadas sem a participação do operador em cada trecho da rota. **Como isso se encaixa na arquitetura:** é um novo `AutopilotMode` em `Autopilot.h`, ao lado dos já existentes MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD — ou seja, o mecanismo de troca de modos (pelos slots do rádio e por `POST /api/setmode`) não muda, e é acrescentado um quinto modo, que toma o rumo e a distância do GPS (Fase 3) em vez da entrada manual pelo rádio. **O que é preciso tecnicamente:** um algoritmo para calcular o rumo até um ponto e a lógica da transição entre os pontos da rota, mais uma forma de carregar a própria rota na aeronave (o candidato natural é a extensão da mesma API HTTP com que já se controlam os modos e o PID).

### Fase 5 — Telemetria de longo alcance [feita no firmware da STM32]

**Status:** na STM32H743 — MAVLink 2 por rádio modem (SiK, ELRS em modo MAVLink): atitude, posição, velocidade, modo, parâmetros do PID, troca de modo a partir do solo. Os quadros foram conferidos com a referência pymavlink. No ESP32 não há UART livre — lá usa-se o dashboard por Wi-Fi. A seguir, o registro original da fase.

**Objetivo:** um enlace aeronave↔solo nas distâncias relevantes para uma entrega real, e não para a bancada. **Uma avaliação honesta do estado atual:** o ponto de acesso Wi-Fi do `WebDebugServer` já hoje transmite o status agregado completo e os comandos de controle, mas o alcance de um AP Wi-Fi comum é de dezenas de metros, o que basta para depurar sobre a mesa ou em um aeródromo, mas não para uma rota autônoma além da linha de visada. **O que é preciso tecnicamente:** um canal de rádio separado, de maior alcance (por exemplo, um módulo LoRa ou um rádio modem de telemetria especializado), como transporte do mesmo formato de dados que já está definido em `GET /api/status` — ou seja, substituir ou complementar a camada de transporte, e não reescrever o formato da telemetria.

### Fase 6 — Uma GUI completa de controle em solo [em parte: QGroundControl / Mission Planner]

**Status:** graças ao MAVLink, as estações de solo padrão já enxergam a aeronave (mapa, ponto de origem, instrumentos, modos com os nomes do ArduPlane). Uma estação própria para uma frota continua sendo um objetivo. A seguir, o registro original da fase.

**Objetivo:** uma estação de planejamento de missões com mapa, telemetria ao vivo e gestão de frota, e não uma página de depuração de uma única aeronave. **Como isso se encaixa na arquitetura:** o `WebDebugServer.h` já hoje não é um esboço, mas um servidor web funcionando, com um status JSON agregado e uma API de comandos (veja a tabela na seção 4.4); ele é o ponto de partida, e não algo que será preciso jogar fora. Os próximos passos são um mapa com a posição atual (depois da Fase 3), exibir e carregar uma rota (depois da Fase 4), operar sobre um canal de rádio de longo alcance (depois da Fase 5) e escalar a interface de uma aeronave para várias. Mais detalhes na seção 6.

### Fase 7 — Mecanismo de lançamento de carga e medidas de proteção para a entrega [feito no firmware, ainda não voado]

**Status:** lançamento de carga (servo AUX1, a função `PAYLOAD_DROP` em qualquer chave), geofence (raio e teto → RTH), retorno ao ponto de origem na perda de sinal, o buzzer de “modelo perdido”. A seguir, o registro original da fase.

**Objetivo:** transformar a plataforma de “um avião que voa de forma autônoma” em “um avião que entrega de forma autônoma”. **O que é preciso tecnicamente:** um servo adicional para o mecanismo de lançamento ou dispensa da carga, controlado pelo mesmo princípio das demais saídas em `FlightOutputs.h`; e medidas de proteção específicas da entrega, e não do voo neutro — geofences (limite da área de voo) e retorno automático ao ponto de partida na perda de sinal (hoje, ao perder o sinal no ar, o `FlightController` corta o motor e passa a planar com as asas niveladas, o que é correto para um planador pilotado manualmente, mas para a entrega autônoma o próximo passo lógico é voltar à base por GPS em vez de simplesmente planar).

### Fase 8 — Escala para uma frota

**Objetivo:** gerenciar várias aeronaves ao mesmo tempo — despacho de tarefas, um painel de operações, histórico de voos. É o nível em que o projeto deixa de ser apenas um protótipo de engenharia por hobby e se torna uma ferramenta operacional, interessante como problema de negócio: planejamento de rotas para várias aeronaves, uma fila de tarefas, o status de cada aeronave em tempo real. Tecnicamente, é uma superestrutura sobre as Fases 3–6 (GPS, telemetria, GUI) — na prática, a mesma API do `WebDebugServer`, mas estendida a muitas fontes de telemetria em vez de uma.

## 6. GUI: de uma página de depuração a uma estação de solo

A tese central desta seção: a GUI não precisa ser construída do zero — ela já existe em parte e funciona. Hoje o `WebDebugServer.h`:

- entrega um único instantâneo JSON agregado do estado da aeronave (`GET /api/status`) — os canais de rádio, os sinalizadores armed/failsafe, o estado de cada saída, o estado de cada sensor (com honestidade, por sinalizadores separados `attached` e `available`) e o estado do piloto automático;
- aceita comandos de controle em tempo real (troca de modo, edição do PID) sem regravar o firmware;
- entrega um dashboard HTML pronto, com barras de canais ao vivo e botões de controle.

O caminho até uma estação de solo completa é uma extensão sucessiva do protocolo que já funciona, e não uma mudança de arquitetura:

1. Adicionar ao dashboard um mapa e a posição atual — isso exige o GPS (Fase 3) como mais um campo do mesmo status JSON.
2. Adicionar a construção e o carregamento de rotas — isso exige um modo de pontos de rota (Fase 4) e a extensão da API POST nos moldes de `/api/setmode`/`/api/setpid`.
3. Transferir o transporte do AP Wi-Fi para um enlace de longo alcance (Fase 5), mantendo o mesmo formato de mensagens para não reescrever o frontend existente.
4. Escalar a interface de uma aeronave para várias fontes de telemetria (Fase 8).

Em outras palavras, o elemento da futura estação de solo mais arriscado quanto a “será que vai ser preciso escrever do zero” — a serialização do estado da aeronave e a API de comandos — já está implementado e foi verificado ao vivo na placa.

## 7. Limitações com honestidade — o que ainda não funciona

Para que o roteiro não pareça marketing, registramos à parte o que ainda não foi feito ou foi feito só em parte:

- O piloto automático foi verificado na bancada, por 387 testes automatizados e por simulações em malha fechada, mas nunca foi testado em voo — até agora só o controle manual voou (no primeiro protótipo). O modelo do avião nas simulações é simplificado e os coeficientes são valores iniciais.
- Os novos sensores (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) foram verificados por emuladores de registradores feitos a partir dos datasheets; no hardware, ainda não.
- STM32H743: o firmware inteiro roda no PC sobre fakes do STM32duino; ainda não há uma placa física.
- O horizonte para a estabilização é a posição da aeronave ao ligar (a IMU é calibrada a cada partida), ou a calibração da instalação a partir de três posições.
- A conexão física de um servo não é visível para o software; só se vê que o pulso realmente sai no pino (uma autoverificação pelo console).
- A pinagem da ESP32 comum de 38 pinos foi escolhida pela documentação do chip e não foi verificada no hardware.
- A licença é a [OpenPlane License](LICENSE.md): MIT com atribuição obrigatória do autor e com proibições do uso militar e do dano intencional a pessoas e bens sem o consentimento deles. Por causa dessas proibições, ela não é considerada “de código aberto” no sentido da OSI.

## 8. Questões em aberto — um convite à discussão

A seguir estão os pontos sobre os quais o projeto ainda não tem resposta, formulados de propósito como perguntas para um possível parceiro ou investidor, e não como fatos resolvidos:

- O modelo de financiamento e o seu volume — negociável; por ora não há valores nem prazos concretos, e este documento não vai inventá-los.
- A forma jurídica do projeto (uma empresa, uma fundação, uma comunidade puramente de código aberto) — aberta à discussão com os interessados em uma parceria.
- A composição da equipe — por enquanto o projeto é conduzido publicamente e está aberto à participação; papéis e compromissos concretos não são postulados de antemão.

Se algo disso for importante para você como possível parceiro, o lugar certo para conversar a respeito são as Discussions do GitHub do projeto (veja a seção 9), e não as suposições deste documento.

## 9. Como entrar em contato e participar

O único canal oficial do projeto hoje é o repositório no GitHub:
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(ramo `main`). Não existem outros contatos (e-mail, redes sociais, pessoa jurídica) no momento, e eles não são indicados aqui de propósito, para não induzir a erro.

- **Issues** — relatar um bug, propor uma mudança técnica concreta, informar os resultados de um teste em voo na sua própria cópia do protótipo.
- **Discussions** — discutir o roteiro, a parceria, o uso em uma tarefa concreta (entrega, busca e salvamento, agricultura), as questões de licenciamento e de financiamento da seção 8.
- **Pull requests** — adicionar um novo sensor pela interface `Sensor`, uma nova placa por um bloco em `Config.h`, um novo `AutopilotMode`, melhorias do dashboard web — a arquitetura foi projetada para que isso possa ser feito sem mexer no núcleo.

Se você lê este documento como possível investidor ou parceiro: o próximo passo com substância não é assinar nada, mas abrir uma Discussion no repositório com uma pergunta ou proposta concreta. O roteiro acima é um convite para discuti-lo fase a fase, com acesso total ao código em que ele se baseia.
