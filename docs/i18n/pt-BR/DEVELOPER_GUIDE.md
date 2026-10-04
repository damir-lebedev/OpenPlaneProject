# DEVELOPER_GUIDE.md — guia do desenvolvedor do OpenPlaneProject

> 🌐 Esta página é uma tradução do [original em russo](../../DEVELOPER_GUIDE.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

Um mapa técnico do firmware: qual arquivo é responsável por quê, como os dados fluem do receptor e dos sensores até os servos, quais convenções de sinais mantêm toda a cadeia unida, como a API web é organizada e como estender o projeto. Destina-se a um desenvolvedor que escreve C++ e quer se orientar rápido neste repositório (o ramo `main`), e não a aprender os fundamentos da linguagem ou do PlatformIO.

Para uma visão geral do projeto e o status do protótipo, veja [`../README.md`](README.md); para o que ligar onde e como voar, [`PILOT_GUIDE.md`](PILOT_GUIDE.md); para os planos, [`ROADMAP.md`](ROADMAP.md). Aqui há só código. A arquitetura completa (camadas, tarefas, máquinas de estados) está em [`ARCHITECTURE.md`](ARCHITECTURE.md), a referência de cada classe em [`reference/`](reference/README.md) e os testes em [`TESTING.md`](TESTING.md).

> O projeto está em desenvolvimento ativo. A bancada com o ESP32-S3 foi montada e verificada com todos os sensores, mas **o piloto automático ainda não foi testado em voo** — isso é indicado onde diz respeito a um módulo específico. Se você duvidar do que o código faz, releia o código-fonte, não o documento.

---

## Conteúdo

1. [Arquitetura das camadas](#arquitetura-das-camadas)
2. [Tarefas do FreeRTOS e o laço de controle](#tarefas-do-freertos-e-o-laço-de-controle)
3. [Referência de arquivos](#referência-de-arquivos)
4. [Convenção de sinais: da IMU ao servo](#convenção-de-sinais-da-imu-ao-servo)
5. [Mapa dos canais de rádio, ARM e failsafe](#mapa-dos-canais-de-rádio-arm-e-failsafe)
6. [Dados dos sensores](#dados-dos-sensores)
7. [Detalhamento de FlightController::update()](#detalhamento-de-flightcontrollerupdate)
8. [API HTTP do dashboard web](#api-http-do-dashboard-web)
9. [Console e diagnóstico](#console-e-diagnóstico)
10. [Escolha da placa e pinagem](#escolha-da-placa-e-pinagem)
11. [Como adicionar um novo sensor](#como-adicionar-um-novo-sensor)
12. [Como adicionar um novo modo do piloto automático](#como-adicionar-um-novo-modo-do-piloto-automático)
13. [Realimentação (base preparada, não conectada)](#realimentação-base-preparada-não-conectada)
14. [Como adicionar uma nova placa](#como-adicionar-uma-nova-placa)
15. [Comandos de build, gravação e monitor](#comandos-de-build-gravação-e-monitor)
16. [Limitações conhecidas](#limitações-conhecidas)
17. [Como fazer alterações](#como-fazer-alterações)

---

## Arquitetura das camadas

Quase todas as classes vivem em cabeçalhos organizados nas pastas `include/<camada>/`. Cada cabeçalho inclui por conta própria o que usa (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... — caminhos a partir de `include/`). O `src/main.cpp` é o único ponto de montagem (composition root): cria todos os objetos, liga-os entre si e executa `setup()`/`loop()`. As dependências são unidirecionais — uma camada inferior não sabe nada sobre a superior.

```
include/
├── config/      Config.h (pinos, todas as configurações), Channels.h (nomes dos canais),
│                Controls.h (o que cada chave faz — uma linha por canal)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (dispositivo de registradores sobre I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + invólucros sobre Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (configurações na flash em vez do NVS)
├── storage/     KeyValueStore, KvPreferences — armazenamento de configurações sem NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  base da realimentação — NÃO conectada (veja a seção abaixo)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (um tubo feito de dois barômetros)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — firmware do ESP32 (S3, C3, 38 pinos)
src/stm32/main.cpp  — firmware do STM32H743 (tarefas do FreeRTOS, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — montagem dos objetos │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — ordem das operações por ciclo │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 modos   │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ navegação, PID     │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, flash    │
└───────────────────────────────────────────────────────────────────────┘
```

As regras que mantêm a arquitetura limpa:

- A **HAL** é a única camada autorizada a conhecer uma MCU específica (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Tudo acima dela trabalha só com interfaces. Migrar para outra MCU é criar um novo `hal/<mcu>/<Mcu>Board.h`; o resto do código não muda (um exemplo é `hal/stm32/` para a STM32H743).
- **Os drivers de sensores não conhecem o barramento.** Eles recebem um `IRegisterDevice&` — o dispositivo I2C com endereço ou SPI com CS é criado em `SensorSelection.h`. O mesmo `BMP388_Sensor` funciona tanto por I2C quanto por SPI.
- **O que é comum fica nas classes base.** Calibração, rotação dos eixos, sinais, filtro de orientação, altitude e velocidade vertical, armazenamento da calibração da bússola, contagem de erros do barramento — em `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. O driver de um chip contém apenas os registradores e as fórmulas do datasheet.
- **RC e Outputs** não sabem nada sobre o avião: bytes do iBUS → canais, valores de PWM → saídas.
- **Control e Autopilot** são lógica sobre dados, sem UART, PWM ou Wi-Fi. O tempo, onde é preciso (flaps), é passado como parâmetro.
- **Coordination** (`FlightController`) é a única classe que enxerga várias camadas inferiores ao mesmo tempo e decide a ordem das operações.
- **Application** (`main.cpp`) é o único lugar onde `Esp32Board`, os dispositivos e os sensores são criados e onde tudo é ligado à mão, sem um framework de injeção de dependências.

---

## Tarefas do FreeRTOS e o laço de controle

| Onde | O quê | Período |
|---|---|---|
| Núcleo 1, `loop()` (o loopTask do Arduino) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Núcleo 0, a tarefa `web` | `WebServer::handleClient()` | a cada 2 ms |
| Núcleo 0, a tarefa `oled` | desenho do SSD1306 pelo segundo barramento I2C | 200 ms |
| Núcleo 0 | a pilha Wi-Fi do ESP-IDF | — |

- O período do laço é mantido por `vTaskDelayUntil`, e não por um `delay(2)` depois do trabalho — a frequência não depende de quanto durou o ciclo. Depois de um bloqueio longo (uma calibração pelo console), a contagem recomeça e os ciclos perdidos não são recuperados em rajada.
- Na bancada (ESP32-S3, todos os sensores): 500 Hz, em média ~0,7 ms de trabalho por ciclo e ~1,4 ms no pior ciclo. Isso é impresso a cada 10 s numa linha `SYS:`.
- O timeout de uma transação I2C é de 5 ms (o padrão do Wire é de 50 ms): uma transação travada por interferência não para o laço por muito tempo.
- **Separação de dados entre tarefas.** A web e o OLED apenas *leem* o estado (`FlightController`/`Autopilot`/`LoopStats`) — são campos individuais de 16/32 bits, então no pior caso aparecem os valores de ciclos vizinhos. Os *comandos* do dashboard (`setmode`/`setpid`) não são aplicados diretamente pela tarefa web: eles são colocados em uma “caixa de correio” sob `portMUX` e recolhidos pelo laço de voo em `WebDebugServer::applyPendingCommands()`.
- `Serial` (UART0 → a ponte CH343 → o conector “COM”) com buffer de transmissão de 4 KB: um quadro de depuração (~600 caracteres) não trava o laço enquanto é enviado.

---

## Referência de arquivos

### `config/`

| Arquivo | Responsável por |
|---|---|
| `Config.h` | Todos os pinos (um bloco por placa: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) e as configurações: iBUS e perda de sinal; curso das superfícies; flaps; reversão de servos; instalação da IMU e da bússola; ARM; failsafe (RTH ou planeio); o tubo de Pitot (`PITOT_*`); todos os números dos modos e das funções do piloto automático; o ciclo; Wi-Fi; MAVLink; depuração |
| `Channels.h` | Nomes dos canais: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | A tabela `BINDINGS`: o que cada chave e cada potenciômetro faz, uma linha por canal, verificações com `static_assert` |

### `hal/`

| Arquivo | Responsável por |
|---|---|
| `IBoard.h` | O ponto de entrada para o hardware: `i2c()`, `displayI2c()` (um segundo barramento para a tela, pode ser `nullptr`), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, pode ser `nullptr`), `servo(ServoChannel::*)` (7 saídas com AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | As tarefas do FreeRTOS do mesmo jeito no ESP32 (núcleo 0) e na STM32 (prioridades), o heap livre |
| `II2CBus.h` | O barramento I2C: primitivas no formato do `Wire` + os auxiliares `writeRegister()`, `readRegisters()` (verifica se chegaram exatamente `count` bytes), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, uma saída PWM (`measurePulseUs()` — diagnóstico do pulso real) |
| `RegisterDevice.h` | `IRegisterDevice` — “um conjunto de registradores de 8 bits”; `I2cRegisterDevice` (endereço), `SpiRegisterDevice` (CS, frequência, bytes fictícios antes dos dados) |
| `esp32/Esp32Board.h` | A implementação de `IBoard`: `Wire` (sensores), `Wire1` (a tela, se o chip tem dois controladores I2C), `SPI`, dois `HardwareSerial`, 5 canais LEDC |
| `esp32/Esp32I2CBus.h` | `II2CBus` sobre qualquer `TwoWire`, timeout de 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM via LEDC: 50 Hz, 14 bits; pino −1 — a saída não está ligada. A biblioteca ESP32Servo não é usada — veja as [limitações](#limitações-conhecidas) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Invólucros finos sobre `SPI` e `HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ a UART4 do rádio modem), barramentos, timers PWM, `Stm32FlashStorage` (configurações em um setor da flash, gravadas por uma tarefa em segundo plano), `compat/Preferences.h` |

### `storage/`

| Arquivo | Responsável por |
|---|---|
| `KeyValueStore.h` | Uma imagem “espaço/chave → bytes” com CRC32 na RAM sobre qualquer meio (`IFlashStorage`); um valor idêntico não é regravado |
| `KvPreferences.h` | A API `Preferences` do ESP32 sobre o `KeyValueStore` |

### `rc/`

| Arquivo | Responsável por |
|---|---|
| `RcChannelState.h` | Um instantâneo dos 10 canais |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → canais: quadro de 32 bytes, CRC, o valor do canal são os 12 bits menos significativos (`& 0x0FFF`); `isSignalLost()` = sem quadros (ou ainda nenhum) ∥ o valor de failsafe do acelerador; contadores de quadros |

### `control/`

| Arquivo | Responsável por |
|---|---|
| `ControlCommand.h` | O comando das superfícies em sinais físicos — a linguagem comum dos sticks, do piloto automático e do mixer |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(alvo, now)`; `mix(command)` → PWM com reversão de servos; flaperons: os ailerons `flaps ± roll` (o menos, freio aerodinâmico) |
| `FlapsController.h` | Extensão e recolhimento suaves dos flaps, com o tempo passado como parâmetro |
| `ThrottleManager.h` | Acelerador vindo do stick; na perda de sinal — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM pela chave SwA (uma transição OFF→ON com o acelerador embaixo + as verificações dos sensores do modo), DISARM instantâneo |
| `FlightOutputState.h` | Os PWM desejados: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (carga), `aux2` (câmera) |
| `Beeper.h` | O buzzer: pela função `BEEPER` ou “modelo perdido” no solo |
| `FlightOutputs.h` | A tabela de saídas (`outputInfo()`: chave, nome, pino, se é obrigatória, campo de estado) e tudo o que fica por cima, em um laço: `begin()`, `write()`, `setFailsafe()`, status, `printPulseSelfTest()` |
| `FlightController.h` | A ordem das operações por ciclo, a perda de sinal (`applyLinkLoss()`), getters para a telemetria |

### `autopilot/`

| Arquivo | Responsável por |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 modos), `Feature`, `Knob`, `PilotInputs`, nomes |
| `ControlBinding.h` | `Binding`, as fábricas `Bind::modes/mode/feature/knob`, as verificações `BindingCheck` |
| `PilotSwitches.h` | A tabela de atribuições → o modo, as funções e os potenciômetros de cada ciclo; a disposição ao ligar |
| `Autopilot.h` | 12 modos, failsafe RTH/planeio, geofence, ponto de origem, coordenação da curva, auto-trim; `update(armed, linkLost, acelerador, sticks)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (distância, rumo, deslocamento), `Guidance` (rolagem para um rumo, o campo vetorial do círculo) |
| `AltitudeSpeedController.h` | Arfagem para a altitude, acelerador para a velocidade do ar (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | As máquinas de estados do lançamento manual e do voo planado |
| `AutoTrim.h` | Auto-trim, guardado no NVS/flash |
| `PidController.h` | PID: D sobre a taxa do sensor (giroscópio, variômetro), anti-windup, integrador congelado sem ARM |
| `feedback/*` | **Base preparada, não conectada:** realimentação adaptativa, decolagem e pouso — veja [Realimentação](#realimentação-base-preparada-não-conectada) |

### `sensors/`

| Arquivo | Responsável por |
|---|---|
| `SensorInterface.h` | As interfaces `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` e as estruturas de dados |
| `SensorSelection.h` | Qual chip é compilado (`#define SENSOR_*`, pode ser substituído por uma flag de compilação) e em qual barramento ele está (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Rotação dos eixos do chip para os eixos da aeronave (0/90/180/270° no sentido horário) — para a bússola e para uma IMU sem calibração de instalação |
| `imu/ImuOrientation.h` | A instalação da IMU como matriz “eixos do chip → eixos da aeronave”: a partir de `IMU_ROTATION_CW_DEG` ou de três posições (nivelada, nariz para cima, asa direita para baixo) com verificação; guardada no NVS |
| `imu/ImuSensorBase.h` | O que é comum às IMUs: calibração do giroscópio + verificação pré-voo (imobilidade, 1g, o “para cima” coincide com a instalação), calibração da instalação (`calibrateOrientation()`), escala, rotação, sinais aeronáuticos, erros do barramento |
| `imu/AttitudeEstimator.h` | Filtro complementar de rolagem/arfagem, integral da guinada |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (o chip é identificado pelo WHO_AM_I): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **Na bancada** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, filtro UI de 50 Hz. Não testado no hardware |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B ou SPI. Não testado no hardware |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1,6 kHz, filtro passa-baixa pelos registradores indiretos IPREG; I2C 0x68/0x69 ou SPI. Não testado no hardware |
| `baro/BarometerBase.h` | O que é comum aos barômetros: leitura apenas de amostras novas, altitude, velocidade vertical por um filtro passa-baixa, calibração da base, erros |
| `baro/BMP388_Sensor.h` | BMP388 por I2C ou SPI (com o byte fictício do SPI), compensação da Bosch, leitura pelo flag de dado pronto. **Na bancada (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, compensação da Bosch §8.1. Não testado no hardware |
| `baro/SPL06_Sensor.h` | SPL06-001: coeficientes e fórmulas do datasheet, 32 Hz ×16; I2C 0x76/0x77 ou SPI. Não testado no hardware |
| `baro/BMP581_Sensor.h` | BMP581: a sequência do BMP5_SensorAPI, 16×/2×, IIR; I2C 0x46/0x47 ou SPI; serve tanto de barômetro principal quanto de tubo de Pitot. Não testado no hardware |
| `mag/MagnetometerBase.h` | O que é comum às bússolas: leitura a 50 Hz, calibração hard-iron no NVS, rotação dos eixos, rumo, erros |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **Na bancada** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. Não testado no hardware |
| `gps/UbloxM10_Gps.h` | u-blox M10: configuração por CFG-VALSET (115200 baud, 10 Hz, NAV-PVT, sem NMEA), análise do NAV-PVT. Não conectado na bancada |
| `airspeed/AirspeedSensor.h` | A interface do sensor de velocidade do ar: pressão diferencial, IAS, TAS, densidade |
| `airspeed/PitotDualBaroAirspeed.h` | O tubo de Pitot caseiro: um BMP581 no tubo + um barômetro na fuselagem; zero no solo, filtro passa-baixa, densidade a partir da pressão estática, detecção de falha |

### `telemetry/` e a aplicação

| Arquivo | Responsável por |
|---|---|
| `DebugLogger.h` | Log por canais (`LogSettings.h`): cada um tem a sua linha, a sua tolerância a oscilação e o seu modo; fica em silêncio enquanto o menu está aberto |
| `DebugConsole.h` | Um menu de texto no monitor da porta (`h`) e atalhos de teclado (`l`/espaço/`s`/`i`/`o`/`m`/`p`/`b`); grava as configurações do log no NVS ao sair do menu e somente sem ARM |
| `LogSettings.h` | Os canais do log (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) e seus modos: desligado / ao mudar / contínuo; guardados no NVS |
| `WebDebugServer.h` | O ponto de acesso, as rotas, o JSON `/api/status`, a caixa de correio de comandos; tem uma tarefa própria no núcleo 0 |
| `WebDashboardPage.h` | O HTML/JS do painel em um único literal; as linhas de canais, saídas e sensores são montadas pelo navegador a partir do JSON |
| `OledDisplay.h` | SSD1306 via U8g2 sobre o `II2CBus`, com tarefa própria (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 para o QGroundControl / Mission Planner: quadros, fluxos, parâmetros do PID, troca de modo a partir do solo |
| `LoopStats.h` | Frequência, tempo médio e pior tempo de ciclo por segundo (OLED) e o pior desde a última leitura (`takePeakUs()`, linha SYS) |
| `src/main.cpp` | ESP32: criação dos objetos, `setup()`, `loop()` com `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: os mesmos objetos, MAVLink, a caixa-preta no cartão SD, as tarefas `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: pinos e clocks do SDMMC1 para o `HAL_SD_Init`; a tecla `D` do console — reinício no bootloader USB DFU |

---

## Convenção de sinais: da IMU ao servo

Um único sistema de sinais para toda a cadeia — assim o stick e o piloto
automático movem as superfícies sempre no mesmo sentido, e o sentido de cada
servo é definido em um único lugar.

**1. Eixos do sensor → eixos da aeronave.** O `ImuSensorBase` gira os eixos
do chip com a matriz `ImuOrientation` (body = R · chip) até os eixos da
aeronave: X para o nariz, Y para a esquerda, Z para cima. A matriz vem:

- da **calibração da instalação** (o comando `o`, guardada no NVS) — a placa
  pode estar posicionada de qualquer jeito. Três posições: “nivelada” dá o
  eixo Z (e o horizonte — o desvio de zero do acelerômetro entra nele),
  “nariz para cima” dá o eixo X (a parte do “para cima” perpendicular a Z),
  “asa direita para baixo” dá o eixo Y. O nariz do passo 2 e o nariz do passo
  3 (Y × Z) devem coincidir com precisão de ~25°; do contrário, o piloto
  inclinou para o lado errado e a calibração é rejeitada; o resultado é a
  média das duas estimativas. Verificado em 300 instalações aleatórias
  (`test/test_imu_orientation`, erro < 0,1°);
- caso contrário, de `Config::IMU_ROTATION_CW_DEG` (placa com o chip voltado
  para cima; o valor indica para onde aponta o eixo X do *chip* se o nariz
  estiver nas “12 horas”), e o horizonte é a posição na hora de ligar.

A cada calibração do giroscópio (ao ligar, `i`) há uma **verificação
pré-voo**: ruído do giroscópio < 0,5 °/s (imobilidade; em repouso ~0,08),
|a| ≈ 1g, o “para cima” a até 45° do salvo (a placa não foi movida). Se não
passar, `ImuSensor::getPreflightProblem()` ≠ nullptr: o `ArmingManager` não
arma os modos com estabilização e `Autopilot::imuReady()` = false (correções
nulas em todos os modos, inclusive o planeio por perda de link).

> No GY-521 atual (um clone com MPU6500) o chip foi soldado girado 90° em
> relação às setas impressas: a seta X na serigrafia = o eixo Y do chip. Por
> isso, sem calibração da instalação, `IMU_ROTATION_CW_DEG = 90`. A
> verificação depois de qualquer reposicionamento: nariz para cima → P cresce
> para o lado positivo, asa direita para baixo → R para o lado positivo.

**2. Ângulos e velocidades (`ImuData`) — sinais aeronáuticos:**

| Grandeza | “+” significa |
|---|---|
| `roll`, `gyroX` | asa direita para baixo |
| `pitch`, `gyroY` | nariz para cima |
| `yaw`, `gyroZ` | nariz para a direita (sentido horário visto de cima) |

**3. O comando (`ControlCommand`, µs de deflexão, ±500 = curso total):**

| Campo | “+” significa | Vindo do stick |
|---|---|---|
| `roll` | rolagem para a direita (aileron direito para cima, esquerdo para baixo) | CH1: 2000 = para a direita |
| `pitch` | nariz para cima (profundor para cima) | CH2 com o sinal oposto: 2000 = para longe de você = nariz para baixo |
| `yaw` | nariz para a direita (leme e roda do nariz para a direita) | CH4: 2000 = para a direita |
| `flaps` | flaps para baixo (os dois ailerons para baixo) | SwB (CH6): 0 ou `FLAPS_DEPLOYED_US`, suavemente em `FLAPS_TRANSITION_MS` |

O PID calcula `erro = alvo − real`: rolagem para a direita (roll > 0) →
comando de rolagem negativo → a aeronave se nivela. As correções do piloto
automático são somadas ao comando dos sticks **antes** do mixer, nos mesmos
sinais.

**4. Comando → PWM.** O `ControlMixer::mix()` calcula a deflexão do bordo de
fuga de cada superfície (ailerons: para baixo = “+”, esquerdo = `flaps + roll`,
direito = `flaps − roll`; profundor: para cima = “+”; leme: para a direita =
“+”) e a converte em PWM `1500 ± deflexão`, invertendo o sinal nos servos com
`Config::*_REVERSED = true`. Os valores padrão reproduzem o comportamento
anterior do firmware para os sticks. A verificação na aeronave montada está
na lista de verificação pré-voo do [`PILOT_GUIDE.md`](PILOT_GUIDE.md). A
reversão deve ser alterada no `Config.h`, **e não no rádio** — senão o stick
e o piloto automático vão divergir.

---

## Mapa dos canais de rádio, ARM e failsafe

A fonte é `include/config/Channels.h`. Rádio FS-i6 (10 canais, modo 2) +
receptor FS-iA6B, iBUS 115200.

| Canal | Controle do rádio | Nome | Função |
|---|---|---|---|
| CH1 | stick direito ←→ | `AILERON` | Rolagem |
| CH2 | stick direito ↑↓ | `ELEVATOR` | Arfagem |
| CH3 | stick esquerdo ↑↓ | `THROTTLE` | Acelerador, curso total; < 950 = failsafe do receptor |
| CH4 | stick esquerdo ←→ | `RUDDER` | Leme + roda de direção (um único servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (no FS-i6 é a chave para baixo, em sua direção) |
| CH6 | SwB | `SWB` | por padrão, os flaps (≥ 1750 — estendidos) |
| CH7 | SwC (3 posições) | `SWC` | por padrão, o modo: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | por padrão, RTH |
| CH9 | VrA | `VRA` | por padrão, a intensidade da estabilização |
| CH10 | VrB | `VRB` | por padrão, a velocidade de cruzeiro |

CH6–CH10 são atribuídos com uma única linha em `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#atribuir-uma-função-com-uma-única-linha)).

**ARM** (`ArmingManager`): a chave passa de OFF para ON, o acelerador está
abaixo de `THROTTLE_LOW_US` e as verificações dos sensores do modo atual
foram aprovadas. Caso contrário, recusa e informa o motivo no Serial; é
preciso um novo ciclo OFF→ON. Ligar a placa com a chave já em ON não arma.
OFF — DISARM imediato. Enquanto não estiver armado, o acelerador enviado ao
ESC é forçado a `PWM_MIN`.

**Perda de link** (`IBusReceiver::isSignalLost()`):

1. Sem quadros por mais de `RX_TIMEOUT_US` (500 ms) — fio rompido ou falta
   de alimentação no receptor. Até o primeiro quadro após ligar, o link
   também é considerado perdido: os valores padrão dos canais (todos em 1500)
   não são tomados como comandos do rádio.
2. Acelerador < `RX_FAILSAFE_THROTTLE_US` (950) — o failsafe configurado no
   rádio. **Quando o rádio é perdido, o FS-iA6B não para de enviar quadros**:
   repete os últimos valores (verificado na bancada); por isso, sem o failsafe
   configurado no rádio, a perda de link não é detectada. A configuração está
   no `PILOT_GUIDE.md`.

O que acontece na perda de link (`FlightController::applyLinkLoss()`):

- **a aeronave está armada, há GPS e ponto de origem** (`FAILSAFE_RTH`) —
  **retorno ao ponto de origem** com o motor, e círculos sobre ele; no OLED —
  `FSRTH`, no log — `FAILSAFE_RTH`;
- **a aeronave está armada, sem GPS** — **planeio**, motor em
  `FAILSAFE_THROTTLE`: em qualquer modo, até no MANUAL, o `Autopilot` mantém a
  rolagem `FAILSAFE_GLIDE_ROLL_DEG` (0 — reto, 10–20° — um círculo sobre o
  piloto) e a arfagem `FAILSAFE_GLIDE_PITCH_DEG` (−3°, para não perder
  velocidade sem o motor), com os flaps recolhidos; no OLED — `GLIDE`, no log
  — o modo `FAILSAFE_GLIDE`;
- **não armada** (no solo) ou a IMU não responde — superfícies no neutro;
- o modo e as funções não são trocados pelas chaves, e os sensores continuam
  sendo lidos. O ARM não é cancelado — quando o link volta, a aeronave
  obedece de novo aos sticks e ao modo escolhido (a decolagem automática e o
  lançamento manual só recomeçam do zero).

---

## Dados dos sensores

As estruturas estão em `include/sensors/SensorInterface.h`.

### `ImuData`

| Campo | Unidade | Significado |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Velocidades angulares nos eixos da aeronave, sinais aeronáuticos (veja acima) |
| `accelX`, `accelY`, `accelZ` | g | Aceleração nos eixos da aeronave: X para o nariz, Y para a esquerda, Z para cima |
| `roll`, `pitch` | ° | Filtro complementar (α = 0,98, τ ≈ 0,1 s); partem direto do ângulo do acelerômetro |
| `yaw` | ° | Integral do giroscópio, deriva lentamente; o valor inicial é o rumo da bússola |
| `temperature` | °C | Temperatura do chip (fórmula para o MPU6050 ou o MPU6500) |
| `timestamp` | µs | `micros()` no instante da leitura |

Calibração da IMU (a cada partida e com o comando `i`): 2 s parada, giroscópio
→ desvio de zero, acelerômetro → **a posição atual passa a ser o horizonte**.

### `BarometerData`

| Campo | Unidade | Significado |
|---|---|---|
| `pressure` | Pa | Pressão |
| `temperature` | °C | Temperatura do sensor |
| `altitude` | m | Altitude **em relação ao ponto de calibração** (na partida); fórmula `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Derivada da altitude sobre as amostras reais (50 Hz) por um filtro passa-baixa com τ = 0,5 s |
| `timestamp` | µs | Instante da última amostra nova |

### `MagData`

| Campo | Unidade | Significado |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | O campo após a calibração hard-iron, nos eixos da aeronave (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, sem compensação de inclinação; o sentido da contagem ainda não foi verificado na aeronave montada |
| `timestamp` | µs | Instante da leitura (50 Hz) |

### `GpsData`

| Campo | Unidade | Significado |
|---|---|---|
| `latitude`, `longitude` | ° | Do UBX-NAV-PVT |
| `altitude` | m | Acima do nível do mar (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Velocidade sobre o solo e rumo sobre o solo |
| `numSatellites`, `fixType` | — | 0 = sem fix, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | Estimativas de precisão do módulo |

**O que significa `isAvailable()`.** Para os sensores I2C: o sensor respondeu
no `begin()` **e** as últimas leituras não estão falhando em sequência (MPU —
~0,1 s; barômetro e bússola — ~0,5 s sem resposta). Quando uma leitura falha,
os dados não são sobrescritos com lixo: os anteriores permanecem e o contador
de erros cresce (visível com o comando `s`). Para o GPS: pelo menos um
NAV-PVT válido, e o último não pode ser mais antigo que `GPS_TIMEOUT_US`.

**Se o sensor não existe** (`nullptr` ou `isAvailable() == false`), o
`Autopilot` não dá correções e a aeronave é pilotada como em MANUAL. O
`main.cpp` calibra apenas os sensores que responderam.

---

## Detalhamento de FlightController::update()

Chamado pelo `loop()` a cada 2 ms. A ordem é a prioridade:

1. **`receiver.update()`** — análise dos bytes do iBUS acumulados.
2. **Chaves** — `switches->update(rc)`, somente com o link vivo (em um quadro
   de failsafe os canais não refletem as chaves): o modo (apenas na mudança),
   as funções, os potenciômetros.
3. **Acelerador do piloto** — `throttle.update(rc, receiverFailsafe)`.
4. **Sticks** — `mixer.fromSticks(rc)` × `Knob::RATES`; flaps —
   `mixer.updateFlaps(target)` (freio, chave, potenciômetro; sem link — 0).
5. **Sensores e piloto automático** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **sempre**, mesmo sem link: os filtros de ângulo não devem congelar.
   Enquanto não estiver armado, o PID roda (as superfícies respondem à
   inclinação — prático sobre a mesa), mas o integrador é mantido em zero. Sem
   link e armado — failsafe RTH ou planeio.
6. **Buzzer** — `Beeper`.
7. **Perda de link** — `applyLinkLoss()`: armado — as superfícies e o
   acelerador seguem o comando de failsafe do piloto automático; caso
   contrário, neutro e motor desligado; `return`. Prioridade absoluta sobre
   tudo o que vem abaixo.
8. **ARM** — `arming.update(rc, false)`.
9. **Comando** — `autopilot->getCommand()`: nos modos com estabilização o
   stick é o ângulo desejado, e o piloto automático emite os comandos finais
   das superfícies.
10. **Mixer** — `mixer.mix(command)` → PWM dos ailerons (flaps + rolagem), do
    profundor e do leme, levando em conta a reversão.
11. **Acelerador** — `autopilot->applyThrottle(pilotThrottle)`: o acelerador do
    piloto, o do piloto automático ou o maior dos dois (decolagem
    automática). Depois, se não estiver armado ou com `MOTOR_KILL`, —
    forçado a `PWM_MIN`. Essa verificação vem por último para que nenhum modo
    consiga passar o acelerador por fora do ARM.
12. **AUX** — carga (`PAYLOAD_DROP`) e câmera (`CAMERA_TILT`, `CAMERA_STAB`).
13. **`outputs.write(output)`** — PWM nas 7 saídas.

---

## API HTTP do dashboard web

A implementação está em `include/telemetry/WebDebugServer.h`. O ponto de
acesso: SSID `OpenPlane-Debug`, senha `12345678`, endereço `http://192.168.4.1`.

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` — o objeto existe na compilação; `available` — o sensor
  realmente responde. Os campos de dados são adicionados **somente** com
  `available: true`.
- `outputs.*.attached` — o MCU reservou um canal LEDC e um pino; se há um
  servo físico conectado não dá para saber pelo software (para verificar o
  pulso, use o console, comando `p`).
- `rollCorr`/`pitchCorr` — o comando final do piloto automático menos os
  sticks, em µs. `throttleCorr` — o acelerador do piloto automático, em %
  (0 enquanto o acelerador está com o piloto).
- `nav` — navegação: o ponto de origem, a distância e o rumo até ele, o curso
  e o curso-alvo, a velocidade usada na navegação (tubo de Pitot / GPS), a
  geofence, o estol; `features` — as funções das chaves que estão ativas.

### `POST /api/setmode`

`{ "mode": 1 }` — o número do `AutopilotMode`: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. O modo se mantém até o
piloto acionar a chave de modo.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — qualquer um dos campos
`kpRoll`, `kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch`; os omitidos
mantêm os valores anteriores.

Os dois comandos são aplicados pelo laço de voo no ciclo seguinte (veja as
[tarefas do FreeRTOS](#tarefas-do-freertos-e-o-laço-de-controle)).

### `GET /`

O dashboard HTML: barras dos 10 canais, ARM/link, as saídas, os sensores,
botões de modo, o formulário do PID. Consulta `/api/status` a cada 200 ms.

---

## Console e diagnóstico

O monitor da porta — 115200, conector “COM”. A implementação é o
`DebugConsole` e o `DebugLogger` ([referência](reference/telemetry.md)). As
teclas funcionam na hora, Enter não é obrigatório; as calibrações e o `p`
bloqueiam o ciclo e por isso só ficam disponíveis sem ARM.

| Tecla | O que faz |
|---|---|
| `h` / `?` | Menu principal |
| `l` | O menu “o que mostrar no log” (canais, modos, período) |
| espaço | Pausar o log / continuar |
| `s` | `printStatus()` de todos os sensores: dados, contadores de erros do barramento, calibrações, a verificação pré-voo |
| `i` | Calibração do giroscópio + verificação pré-voo (2 s parada) |
| `o` | Calibração da instalação da IMU por três posições, salva no NVS |
| `m` | Calibração da bússola (15 s girando), salva no NVS |
| `p` | Autoteste das saídas: o pulso real em cada pino contra o esperado |

O log é dividido em canais (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`, `MAG`,
`GPS`, `IMU`, `SYS`), cada um com o modo “desligado / ao mudar / contínuo”; as
configurações ficam no NVS e são gravadas ao fechar o menu, somente sem ARM.
Por padrão ficam ativados `STAT` (ao mudar) e `SYS` (uma vez a cada 10 s):

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (o pior em 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

Os formatos de todos os canais estão na [referência](reference/telemetry.md#debuglogger).

---

## Escolha da placa e pinagem

| Comando | `board` | Macro | Status |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **Principal, a padrão.** Testada na bancada com todos os sensores |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | O protótipo antigo, voou sob controle manual |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | Para a bancada, a pinagem não foi testada no hardware |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: o firmware completo + MAVLink + caixa-preta no SD; testada em uma placa nua ([abaixo](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | O mesmo na DevEBox H743: o console é USB CDC e o firmware é gravado por DFU |

| Função | ESP32-S3 (bancada) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Aileron esquerdo / direito | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Profundor / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Leme | GPIO18 | — (sem pino) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| I2C dos sensores SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| I2C do OLED SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / nenhum (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → conector “COM” | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** os GPIO33–37 são ocupados pela PSRAM octal, os 26–32
  pela flash, os 19/20 pelo USB, os 43/44 pelo Serial, o 48 é o LED RGB; os
  0/3/45/46 são pinos de strapping.
- **ESP32-C3:** os ailerons nos GPIO4/5 estão trocados em relação à S3. Não há
  pinos suficientes para o conjunto completo: o CS do BMP388 e o RX do GPS
  estão em pinos de strapping, o GPS não tem TX (somente recepção, sem
  UBX-CFG). Os detalhes estão no `Config.h`.

### STM32H743

A STM32H743VIT6 (Cortex-M7 a 480 MHz, 2 MB de flash, 1 MB de RAM) executa o
**firmware completo**: os mesmos sensores, piloto automático, chaves, console
e tela da ESP32-S3, mais a telemetria MAVLink e uma caixa-preta no cartão SD.
Compila, passa no cppcheck e em todos os testes nativos do código comum. No
hardware foi testada a **placa DevEBox H743 sem sensores**: inicialização,
console por USB, cartão SD, caixa-preta —
[TESTING.md](TESTING.md#testes-na-placa-stm32) — e também iBUS, ARM e PWM para
os servos e o motor: controle pelo rádio em modo manual (em vídeo). Os sensores
na STM32 ainda esperam uma bancada. A placa principal de voo é a ESP32-S3.

- **HAL** — `include/hal/stm32/`: `Stm32Board` (a mesma API do `Esp32Board`,
  mais `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (PWM por hardware do `HardwareTimer`, um timer para vários
  canais de saída). Em detalhe — [reference/hal.md](reference/hal.md#implementação-para-o-stm32h743).
- **Configurações e calibrações** — não no NVS, mas em um `KeyValueStore` no
  último setor da flash (`include/storage/`,
  `hal/stm32/Stm32FlashStorage.h`). O código do projeto continua escrevendo
  `#include <Preferences.h>`: no env `stm32h743` o diretório
  `include/hal/stm32/compat/` está no `-I`, e lá existe um `Preferences` com a
  mesma API. A imagem leva CRC32: uma imagem corrompida (a energia caiu durante
  o apagamento) é lida como vazia. A gravação na flash acontece em uma tarefa
  em segundo plano: apagar um setor de 128 KB leva segundos, mas o setor está
  no banco 2 e o código executa a partir do banco 1, e a tarefa de voo preempta
  a de segundo plano sem parar.
- **Tarefas** — FreeRTOS da biblioteca STM32duino FreeRTOS, um núcleo,
  preempção por prioridade (`hal/Rtos.h`): `flight` (5) — o laço de voo,
  MAVLink, log, console; `oled` (1) e `storage` (1) — em segundo plano;
  `bbox` (2) — a gravação da caixa-preta no cartão SD.
- **A caixa-preta no cartão SD** — SDMMC1, 4 bits, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, os pinos em `src/stm32/sd_msp.cpp`). O cartão
  continua sendo um FAT32 comum: nele fica um arquivo `BLACKBOX.BIN` criado
  previamente, o firmware grava blocos brutos dentro dele e não mexe no
  sistema de arquivos em si (`storage/Fat32File.h` é somente leitura).
  Preparação do cartão e download — [BLACKBOX.md](BLACKBOX.md#cartão-sd-stm32h743).
- **Telemetria** — MAVLink 2 na UART4 (`telemetry/MavlinkTelemetry.h`) no
  lugar do dashboard Wi-Fi: QGroundControl / Mission Planner, troca de modo e
  de PID a partir do solo. Em detalhe —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#estação-de-solo-painel-wi-fi-e-mavlink).
- **Pinagem** — o bloco `BOARD_STM32H743` no `Config.h`, com os pinos
  escolhidos entre os livres na WeAct MiniSTM32H743VITx e conferidos com as
  tabelas do STM32duino:

| Função | STM32H743 | Periférico |
|---|---|---|
| Aileron esquerdo / direito | PA0 / PA1 | TIM2_CH1 / CH2 |
| Profundor / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Leme | PD14 | TIM4_CH3 |
| AUX1 (carga) / AUX2 (câmera) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX — reserva) | PE7 (PE8) | UART7 |
| I2C dos sensores SDA / SCL | PB11 / PB10 | I2C2 |
| I2C do OLED SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / barômetro | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Rádio modem MAVLink RX / TX | PD0 / PD1 | UART4 |
| Buzzer | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** — o env `stm32h743-devebox`: o mesmo código, a sua
  própria variante do núcleo, o console por USB-C como porta COM virtual (CDC)
  — não é preciso um USB-UART. A primeira gravação do firmware é por USB pelo
  bootloader embutido (DFU):
  1. Windows: instalar uma vez o driver WinUSB para o “STM32 BOOTLOADER”
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. Ligar com um fio o pino **BT0** (BOOT0) ao **3V3**, pressionar e soltar o
     **RST**: a placa entra em modo DFU (a DevEBox não tem botão BOOT0).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. O fio do BT0 pode ser retirado — o firmware inicia sozinho.

  Depois disso o fio não é mais necessário: a tecla **`D`** no console (a
  partir de qualquer menu, não com ARM) reinicia a placa no bootloader: uma
  marca na RAM → reset → salto para a memória do sistema antes de configurar os
  clocks (`src/stm32/bootloader.cpp`). Saltar direto do firmware em execução
  trava no H7 — verificado na placa, por isso são dois passos. É preciso o
  console aberto (USB CDC); se a placa não responder — RST com o fio do BT0
  ligado.
- **Ponto de entrada** — `src/stm32/main.cpp` (excluído das compilações do
  ESP32 por `build_src_filter`). Os objetos são os mesmos de `src/main.cpp`; no
  lugar do `loop()` há tarefas, e o `vTaskStartScheduler()` fica no fim do
  `setup()`.
- **Primeira energização da placa:** `pio run -e stm32h743 -t upload` (ST-Link),
  o monitor no LPUART1 por um USB-UART; `b` — se os sensores aparecem nos
  barramentos, `s` — status dos sensores, `p` — pulsos nas saídas (tire a
  hélice), depois o rádio e o QGroundControl pelo rádio modem.

---

## Como adicionar um novo sensor

### A) Outro chip de uma categoria existente (IMU, barômetro, bússola)

O que é comum já está escrito nas classes base — o driver do chip sai pequeno:

1. Crie `include/sensors/<category>/<Name>_Sensor.h` e herde de
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. O construtor
   recebe um `IRegisterDevice&` — o driver não sabe se é I2C ou SPI.
2. Implemente:
   - `begin()` — `device.begin()`, verificar o ID do chip, gravar os
     registradores, chamar `setAvailable(true/false)`;
   - IMU: `readSample()` (accel/gyro/temp brutos nos eixos do chip),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - barômetro: `isNewSampleReady()` (um flag de dado pronto ou simplesmente
     `true`) e `readSample()` (pressão em Pa, temperatura em °C); o período de
     leitura fica no construtor da base;
   - bússola: `readRaw()` (X/Y/Z nos eixos do chip) e `lsbPerMicroTesla()`; o
     nome do espaço do NVS para a calibração fica no construtor da base.
3. Se o chip precisa, por SPI, de um byte fictício antes dos dados ou de uma
   frequência especial — adicione uma fábrica estática `spiDevice(bus, cs)`,
   como a do `BMP388_Sensor`.
4. Um ramo em `SensorSelection.h`: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` e `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` ou a fábrica de SPI). O
   `main.cpp` não é tocado quando o sensor muda.
5. Verifique a compilação com o novo sensor sem editar o arquivo — com uma
   flag: `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`,
   depois os três ambientes e, por fim, no hardware.

### B) Uma nova categoria

1. A estrutura de dados e a interface vão em `SensorInterface.h`, nos moldes de
   `GpsSensor`/`GpsData`.
2. Se a categoria tem lógica comum (filtros, calibração) — uma classe base nos
   moldes de `BarometerBase`.
3. Um ponteiro anulável no construtor do `Autopilot` (sem sensor — sem
   efeitos, em vez de uma falha) e campos em `GET /api/status` com o par
   `attached`/`available`.

### Um novo barramento ou periférico

Uma nova interface em `include/hal/`, a implementação em
`include/hal/esp32/` e em `include/hal/stm32/`, acesso por meio do `IBoard`.

---

## Como adicionar um novo modo do piloto automático

1. Um valor em `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, antes de
   `MODE_COUNT`), o nome e um nome curto (até 5 caracteres, para o OLED) em
   `AutopilotNames::mode()` / `modeShort()`.
2. Um tratador `run<Mode>()` e um ramo em `Autopilot::runMode()`; os alvos
   iniciais (rumo, altitude, centro dos círculos) ficam em
   `initializeMode()`. O modo define `desiredRoll`/`desiredPitch` e chama
   `stabilizeOrManual()` (sem IMU as superfícies ficam com o piloto) ou
   `stabilizeOrNeutral()` (sem IMU — neutro). Sem o sensor necessário — um
   comportamento seguro, e não uma falha. O integrador só acumula com
   `armed`.
3. Acelerador: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) e
   `autoThrottlePct`, ou `autoThrottle()` — o acelerador de cruzeiro vindo do
   potenciômetro / do tubo de Pitot. O `FlightController` não muda.
4. No rádio — uma única linha em `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). O dashboard e o MAVLink pegam o
   modo pelo número; para o MAVLink — o modo mais próximo do ArduPlane em
   `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. Se o modo precisa de sensores para o ARM — `ArmingManager`.
6. Testes: a reação a cada sensor — `test/native/test_autopilot_modes`; o voo
   em malha fechada — um cenário em `test/native/test_sim` (o modelo da
   aeronave `helpers/PlaneSim.h`, a bancada `helpers/SimHarness.h`). Depois — a
   mesa sem hélice: as superfícies devem responder à inclinação no sentido de
   nivelar.
7. Uma seção no [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Realimentação (base preparada, não conectada)

`include/autopilot/feedback/` é o próximo passo do piloto automático. **Nem o
`FlightController`, nem o `Autopilot`, nem o `main.cpp` incluem esses
arquivos:** ainda não há protótipo para testes de voo, e o firmware funciona
sem eles. Eles são verificados por uma simulação em malha fechada
(`test/test_feedback/`) diretamente na placa.

### Por quê

Hoje o `Autopilot` é um PID sobre o ângulo: erro × ganho = superfície. Ele
não sabe o que resultou disso na aeronave, e os ganhos só estão corretos para
uma velocidade: em baixa velocidade a superfície é mais fraca e o PID corrige
de menos; em alta, de mais. A realimentação fecha a malha sobre **a resposta
da aeronave**:

- a superfície foi defletida, mas a aeronave gira mais devagar do que o
  necessário — somar mais, até chegar lá;
- quanto de superfície é preciso é medido em voo e recalculado conforme a
  velocidade;
- a aeronave gira para o lado errado — o sinal está trocado, inverter e
  verificar;
- o ângulo foi nivelado, mas a velocidade está caindo — acelerador e nariz
  para baixo, até a aeronave não estolar;
- decolagem e pouso — em fases, conforme o que os sensores mostram.

### Módulos

| Arquivo | O que faz |
|---|---|
| `FlightSnapshot.h` | Tudo o que a realimentação sabe sobre a aeronave em um ciclo. É a única entrada: os módulos não leem os sensores nem o RC diretamente, por isso podem rodar em simulação e em logs |
| `FeedbackOutput.h` | A saída de um ciclo: deflexões das superfícies por eixo, se o eixo está ativado, o sinal do eixo, o acelerador (definir / não abaixo de), o motivo |
| `FeedbackConfig.h` | Todas as constantes (ao conectar, irão para o `Config.h`) |
| `SpeedEstimator.h` | A velocidade (tubo de Pitot > GPS) e a aceleração longitudinal pela IMU: `dV/dt = g·(ax − sin θ)` — dá para ver que “a velocidade está caindo” mesmo sem sensor de velocidade |
| `AirborneDetector.h` | No ar / no solo: aprender, acumular a integral e procurar o estol só faz sentido em voo |
| `ControlEffectivenessEstimator.h` | Para cada eixo aprende o modelo `ε = b·u(t−delay) + a·ω + c` por mínimos quadrados recursivos |
| `AdaptiveRateController.h` | Uma cascata ângulo → velocidade angular → aceleração angular → superfície, pelo modelo aprendido |
| `StallGuard.h` | Proteção contra perda de velocidade e estol |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Decolagem (de uma pista ou à mão) e pouso em fases, pelos sensores |
| `FeedbackSupervisor.h` | Tudo junto: a ordem dentro de um ciclo, as prioridades, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, o plano de conexão |
| `FeedbackModules.h` | Um único include para tudo |

### Como funciona

**Efetividade das superfícies.** O modelo do eixo: aceleração angular
`ε = b·u + a·ω + c`. `b` é quantos °/s² 1 µs de superfície produz (o sinal é o
sentido da resposta), `a` é o amortecimento (o ar freia a rotação; sem esse
termo a estimativa de `b` iria a zero numa rotação estacionária), `c` é um
momento constante (centragem, compensador, hélice). A força de uma superfície
∝ ρV², por isso `b` é aprendido numa velocidade de referência e multiplicado
por `(V/Vref)²`: a aeronave acelerou — a superfície instantaneamente “ficou
mais forte”, sem reaprender. A velocidade indicada do tubo de Pitot já
contém a densidade do ar, então a altitude é considerada por si mesma; sem
sensor de velocidade a escala é 1, e `b` é aprendido diretamente.

Os dados são tomados em intervalos de 20 ms: a aceleração média no intervalo
é a diferença do giroscópio nas pontas / a duração, e a ela correspondem a
superfície média e a velocidade angular média do mesmo intervalo (a
superfície — com o atraso `RESPONSE_DELAY_MS`). Depois os dois lados da
equação passam pelo mesmo filtro passa-baixa de 2 Hz: a relação não muda,
enquanto as altas frequências, em que o modelo de “atraso puro” erra por causa
da inércia do servo, são removidas. Só é possível aprender no ar e somente
quando a superfície está sendo “agitada” (uma amplitude ≥ `MIN_EXCITATION_US`
em ~0,3 s); os sticks do piloto também agitam, por isso a estimativa aprende
também em MANUAL.

**O regulador.** Três estágios, eixo por eixo:

```
ω* = ANGLE_GAIN · (target − angle)              "nariz 10° abaixo — levantar a 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "gira mais devagar que o necessário — corrigir"
surface = (ε* − a·ω − c) / b                    pelo modelo aprendido
```

A integral `I` é guardada em °/s, e não em µs de superfície — por isso ela
continua correta quando a estimativa de `b` muda. No solo a integral fica
congelada (exceto o rumo na corrida de decolagem / de pouso) e, com a
superfície no limite, não se acumula na direção do limite. A curva coordenada
é considerada (se a velocidade é conhecida): na inclinação são necessários
arfagem `g·sin φ·tg φ / V` e guinada `g·sin φ / V`.

**Sinais dos eixos — somente no solo.** Em voo os eixos não são desligados
nem invertidos: a instalação da IMU é determinada pela calibração `o` e pela
verificação ao ligar, e os sentidos das superfícies, pela verificação pré-voo
do piloto. Indícios indiretos no ar (um estol, um parafuso, manobras,
rajadas) podem enganar, e um eixo desligado ou invertido num momento desses
custa a aeronave. Se a estimativa de `b` de um eixo é confiantemente negativa,
isso é apenas um aviso em `reason` (“responde à superfície ao contrário?
verificar no solo”); uma estimativa negativa não vai para o regulador — o
eixo trabalha com o modelo a priori.

**Proteção contra estol.** Dois níveis. *LowEnergy* — a velocidade cai rápido
com o nariz levantado, ou está perto do estol (< 1.25·Vs), ou o profundor
perdeu efetividade: acelerador ≥ 80 %, arfagem ≤ 5°. *Stall* — a velocidade
está abaixo da de estol, o nariz cai contra o profundor, a asa cai contra os
ailerons com pouca energia: acelerador total, nariz para baixo, inclinação
≤ 10°, ailerons limitados (um aileron grande estola a ponta da asa). As
medidas são retiradas com histerese (velocidade ≥ 1.5·Vs). Na perda de link o
acelerador não é tocado, e bem perto do solo (arredondamento, corrida de
pouso) a proteção fica desligada — o pouso é, em si, um estol controlado.

**Decolagem.** `WaitThrottle` (o motor está parado) → o piloto deu
acelerador ≥ 50 % → `GroundRoll` (acelerador total, asas niveladas, o rumo
mantido pelo leme e pela roda, o profundor livre) → velocidade de decolagem,
ou um tempo limite sem sensor de velocidade → `Climb` (12°, acelerador
total) → altitude de 30 m → `Complete`. À mão (`TAKEOFF_HAND_LAUNCH`), no
lugar da corrida — `WaitLaunch`: o motor só parte depois do arremesso
(aceleração longitudinal ≥ 1g). Acelerador reduzido antes da decolagem —
cancelamento.

**Pouso.** `Approach` (acelerador 25 %, descida de 1 m/s — a arfagem a partir
do erro da velocidade vertical, a inclinação vem do piloto ≤ 20°) → altitude
2 m → `Flare` (acelerador 0, a descida é amortecida até 0,3 m/s pela mesma
regra) → impacto no acelerômetro ou “baixo e sem girar” → `Rollout` (rumo com
a roda) → `Complete`. Acelerador do piloto ≥ 80 % — arremetida. O
arredondamento precisa de um telêmetro: o barômetro erra em um metro.

**Prioridades** (`FeedbackSupervisor`): não armado > proteção contra estol >
decolagem/pouso > alvos do modo. Na perda de link as fases são canceladas, e
a estabilização cumpre os alvos de planeio do failsafe.

### Simulação

`test/test_feedback/test_main.cpp` (no PC: `pio test -e native -f test_feedback`) — um modelo de aeronave (eixos independentes,
atraso e inércia do servo, efetividade das superfícies ∝ V², amortecimento ∝ V, momentos
constantes, sustentação pelo ângulo de ataque a partir da velocidade, estol, trem de pouso com
roda de direção) e 10 cenários:

| Cenário | O que é verificado |
|---|---|
| Saída de uma inclinação de 30° / arfagem de −15° com momento constante | Nivelamento e “continuar corrigindo”: a integral encontra o compensador sozinha |
| Agitação de ±15° a 14 e 20 m/s, sem sensor de velocidade | A estimativa de `b` converge para a verdade e é reescalada com a velocidade |
| Aileron trocado, o piloto balança as asas em MANUAL | A estimativa de `b` é negativa → só um aviso, o eixo não é desligado |
| 30 s de turbulência | As rajadas são contidas, a inclinação não passa de 10° |
| Nariz a 15° com 20 % de acelerador (com sensor de velocidade e sem) | A velocidade não cai até o estol |
| Decolagem de uma pista com o torque de reação da hélice | Fases, altitude, rumo na corrida |
| Pouso a partir de 15 m | Fases, sem acelerador perto do solo, toque suave |
| Perda de link na corrida de decolagem; não armado; MANUAL | Cancelamento, o acelerador não é tocado, as superfícies ficam com o piloto |

O modelo é rudimentar — ele verifica a lógica e os sinais, e não o ajuste para
uma célula específica.

```bash
pio test -e native -f test_feedback      # no PC, em segundos
pio test -e esp32-s3 -f test_feedback    # grava o firmware de teste e o executa
pio run -t upload                        # restaurar o firmware normal
```

### Plano de conexão

1. O `FlightController::update()`, depois de ler os sensores e calcular os
   comandos, preenche um `FlightSnapshot` e chama
   `FeedbackSupervisor::update()`. No começo — **modo sombra**: a saída vai
   apenas para o log (`printStatus()`) e para o dashboard, não para as
   superfícies. Em voo sob controle manual, a estimativa de `b` de cada eixo
   deve ser positiva e crescer com a velocidade.
2. No solo, com a aeronave nas mãos, STABILIZE: incline-a — as superfícies
   contrapõem.
3. Um eixo de cada vez: `deflectionUs` no lugar de
   `Autopilot::getRollCorrection()` (primeiro só a rolagem), depois a arfagem.
4. Acelerador: `throttleOverridePercent`/`throttleFloorPercent` — depois de
   `Autopilot::applyThrottle()`, antes do failsafe (o failsafe manda em
   tudo).
5. Decolagem/pouso — em uma chave livre; remover o modo `AUTO_TAKEOFF` do
   `Autopilot`.
6. As constantes de `FeedbackConfig` — para o `Config.h`; o sensor de
   velocidade do ar — uma implementação de `AirspeedSensor` e uma categoria
   em `SensorSelection.h`.

---

## Como adicionar uma nova placa

1. `[env:<name>]` no `platformio.ini` com um `-D BOARD_ESP32_<NAME>` único.
2. Um bloco `#elif defined(BOARD_ESP32_<NAME>)` no `Config.h` com todos os
   pinos, incluindo `PIN_I2C2_SDA/SCL` (−1 se não houver OLED). Calcule o
   orçamento de GPIO com antecedência: flash/PSRAM/USB/strapping.
3. As saídas dos servos precisam de 5 canais LEDC — todas as ESP32 os têm. Se
   não há pino para o leme — `PIN_RUDDER = -1`, e a saída simplesmente é
   desativada.
4. Não altere `default_envs` até a placa ser testada no hardware; indique
   explicitamente no commit se a pinagem não foi testada.

---

## Comandos de build, gravação e monitor

```bash
pio run                        # compilar a placa padrão (esp32-s3)
pio run -t upload              # gravar
pio device monitor             # monitor, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # verificar se todas as placas compilam
```

- **ESP32-S3:** a gravação e o Serial passam pelo conector “COM” (CH343). Se a
  ponte travar (o Windows responde “o dispositivo não está funcionando” — pode
  acontecer por interferência do ESC), reconectar o cabo ajuda; também é
  possível gravar pelo conector “USB” (o USB-JTAG embutido):
  `pio run -t upload --upload-port <USB COM port>`.
- Enquanto o monitor da porta estiver aberto, a gravação na mesma porta não
  funcionará.
- `lib_deps`: `olikraus/U8g2` (OLED) é a única biblioteca externa.
- `test/` — em detalhe no [`TESTING.md`](TESTING.md):
  - `pio test -e native -e native-stm32` — 387 testes no PC (simulacros do
    hardware em `test/native/support/`), cobertura — `gcovr`;
  - `pio test -e esp32-s3` — `test_feedback/` (a simulação em malha fechada da
    realimentação) e `test_imu_orientation/` na própria placa; cada um grava
    um firmware de teste, depois grave o normal com `pio run -t upload`.
- Análise estática: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck sobre
  `hal/stm32/` e `src/stm32/`) e `tools/clang-tidy.sh`
  (o perfil `.clang-tidy`).

---

## Limitações conhecidas

- **O piloto automático não foi testado em voo.** Na mesa os sinais foram
  verificados ao vivo (inclinação → correção no sentido de nivelar); os
  coeficientes do PID são valores iniciais.
- **STABILIZE é um nivelamento sobreposto aos sticks**, e não um “modo
  angular” (FBWA) em que o stick define o ângulo de rolagem/arfagem. O piloto
  e o piloto automático se somam.
- **O planeio na perda de link não foi testado em voo.** Os ângulos
  `FAILSAFE_GLIDE_*` são valores iniciais; a arfagem de −3° é ajustada a uma
  célula específica (o nariz não deve nem levantar até o estol nem picar).
- **O horizonte.** Com a calibração da instalação (`o`) — vem dela (NVS); o
  desvio de zero do acelerômetro deriva com a temperatura (~1–2° a cada
  20 °C), se o horizonte “derivou” — repita `o`. Sem ela — a posição ao
  ligar (ligue com a aeronave nivelada).
- **A instalação da bússola** continua sendo definida por
  `MAG_ROTATION_CW_DEG` (a calibração por posições não a afeta).
- **Bússola:** o rumo é sem compensação de inclinação, o sentido da contagem
  não foi verificado na aeronave montada, e a calibração precisa ser feita
  já dentro da aeronave. Nenhum modo usa o rumo ainda.
- **O GPS** não é usado na navegação; na ESP32-C3 é somente recepção.
- **A realimentação (`autopilot/feedback/`) não está conectada** e foi
  verificada apenas em simulação com um modelo rudimentar de aeronave. Todos
  os números em `FeedbackConfig.h` marcados como “прикидка” (“estimativa grosseira”) precisam ser refinados
  em uma célula real; ainda não há sensor de velocidade do ar (sem ele a
  efetividade das superfícies é aprendida mais devagar, e o estol só é visível
  pela desaceleração).
- **Não testados no hardware:** `ICM42688_Sensor` (levado à convenção comum
  por meio do `ImuSensorBase`), `BME280_Sensor` (a compensação da Bosch foi
  reimplementada), BMP388 por SPI, `QMC5883L_Sensor`, a configuração do GPS
  por CFG-VALSET. Ao conectar — o log de inicialização, `s` no console, os
  sinais pela inclinação.
- **O I2C em protoboard capta interferência** do ESC/motor (erros isolados
  aparecem com `s`). Os drivers aguentam, mas na aeronave os fios do I2C devem
  ser curtos e ficar longe dos de potência.
- **O ESP32Servo não é usado.** A versão 3.2.1 na ESP32-S3 distribui os
  servos pelos MCPWM e, no `attachPin()`, confunde o número da unidade MCPWM
  com o do timer: os GPIO6/7 emitiam o sinal dos GPIO4/5 (o ESC era controlado
  pelo stick direito). As saídas foram reescritas em LEDC; só devolva a
  biblioteca depois de verificar com `p`.
- **O ESC é PWM de 50 Hz**; o firmware ainda não tem um modo de calibração da
  faixa do acelerador.
- **Dashboard web:** a senha do ponto de acesso é fraca, e os comandos são
  aceitos também em voo. É uma ferramenta para a bancada e o campo, não para o
  voo.
- **Mecânica do protótipo:** o primeiro protótipo voou, e foram constatadas
  uma fixação fraca do motor e rigidez insuficiente da asa.
- **A licença é a OpenPlane License** ([LICENSE](LICENSE.md)): MIT com
  atribuição obrigatória do autor, proibição do uso militar e proibição de dano
  intencional a pessoas e bens sem o seu consentimento por escrito. Não
  adicione outros cabeçalhos de licença aos arquivos e não remova o nome do
  autor.

---

## Como fazer alterações

- **Commits pequenos:** um passo lógico — um commit.
- **Testes e análise antes de um commit:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` —
  tudo verde ([`TESTING.md`](TESTING.md)).
- **Compile todas as placas** depois de mudanças no código comum — a S3 é a
  principal, mas a C3, a de 38 pinos e a `stm32h743` não podem quebrar; antes
  de um lançamento — `tools/build_matrix.sh` (todas as placas × todos os
  sensores).
- **Verifique no hardware o que puder ser verificado:** os sinais — pela
  inclinação, as saídas — com o comando `p`, o link — desligando o rádio.
- **Não invente APIs.** Consulte os fontes do framework em
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x)
  — a internet costuma descrever a versão 3.x, com outra API (por exemplo, o
  LEDC).
- **Não embeleze o status.** Não testado no hardware — escreva exatamente
  isso.
- **Uma camada não deve saber mais do que lhe cabe.** Se uma classe inferior
  de repente precisa de uma superior, a lógica deve subir para o
  `FlightController`.
- **Ao mudar um contrato de dados** (`FlightOutputState`, `ControlCommand`,
  `ImuData`, o JSON de `/api/status`) — atualize todos os consumidores no mesmo
  commit.
