# APPLICATION — `src/main.cpp` e `src/stm32/main.cpp`

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/application.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

Os dois pontos de entrada são a **composition root** das suas placas: a única
unidade de tradução do firmware e o único lugar onde os objetos são criados e
ligados por referências. Não há lógica de voo neles e o conjunto de objetos é o
mesmo; o que muda é a placa, a telemetria (Wi-Fi ou MAVLink) e a forma como o
laço de voo é acionado.

## Objetos globais (comuns)

A ordem de declaração = a ordem de construção.

| Objeto | Tipo | Ligações |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | o barramento de `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | com o tubo de Pitot — é a pressão estática |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | somente se `SENSOR_MAG != NONE`, senão `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | somente se `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | somente se `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | todos os sensores (anuláveis) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, a tabela `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | tudo o que está acima |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | o controlador, o piloto automático, as estatísticas |
| `debugConsole` | `DebugConsole` | o controlador, as saídas, o piloto automático, o log, `&board` (varredura dos barramentos `b`) |
| `oledDisplay` | `OledDisplay` | o controlador, o piloto automático, as estatísticas |
| ESP32: `webDebugServer` | `WebDebugServer` | o controlador, o piloto automático |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, o controlador, o piloto automático, as estatísticas |

## `src/main.cpp` — ESP32 (S3, C3, 38 pinos)

| Função | Descrição |
|---|---|
| `static void printBanner()` | a tela de abertura no `Serial` |
| `static void setupSensors()` | `begin()` de cada sensor; calibração dos que responderam: IMU `calibrate()` (2 s parada + a verificação pré-voo), barômetro `calibrateAltitude()`, bússola — a primeira amostra após 25 ms define o rumo da IMU (`setYaw`); GPS `begin()`; Pitot `begin()` (o zero — no primeiro segundo do laço); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **antes** de `begin(115200)`; a tela de abertura; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; o servidor web; a disposição das chaves; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; o período é `vTaskDelayUntil(LOOP_PERIOD_MS)`; um atraso > 100 ms — a contagem recomeça (sem “correr atrás”) |

## `src/stm32/main.cpp` — STM32H743

O ponto de entrada do env `stm32h743` (nas compilações para ESP32 o diretório
`src/stm32/` é excluído por meio de `build_src_filter`). No hardware foi
testada a placa DevEBox H743 sem sensores (inicialização, console por USB,
cartão SD, caixa-preta, iBUS e controle manual dos servos e do motor); o
conjunto todo roda no PC pelos testes `test/native_stm32` (o env
`native-stm32`). Ao lado: `sd_msp.cpp` — os pinos e clocks do SDMMC1,
`bootloader.cpp` — a tecla `D` do console (reinício em DFU).

| Função | Descrição |
|---|---|
| `setup()` | `Serial.begin(115200)`; a tela de abertura; `board.begin()`; as saídas para a posição segura; `Stm32FlashStorage::store().mount()` — a imagem das configurações (vazia / N bytes / corrompida — valores padrão); `setupSensors()` (como na ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; a disposição das chaves; `debugLogger.begin()`; as tarefas `flight` e `storage`; `vTaskStartScheduler()` (não retorna) |
| `static void flightTask(void*)` | prioridade `Rtos::PRIORITY_FLIGHT`, pilha de 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, um atraso > 100 ms — a contagem recomeça |
| `static void storageTask(void*)` | em segundo plano: `Stm32FlashStorage::instance().service()` uma vez a cada 100 ms — apagamento e gravação do setor de configurações, preemptada pela tarefa de voo |
| `loop()` | vazio: depois de `vTaskStartScheduler()` só as tarefas funcionam |

O console (`Serial`, LPUART1 PA9/PA10, 115200) é o mesmo `DebugConsole` da
ESP32: `h` menu, `s` sensores, `b` varredura dos barramentos, `p` saídas,
calibrações.

## Invariantes

- As saídas vão para a posição segura **antes** da inicialização dos sensores
  (a calibração da IMU segura o laço por ~2 s).
- ESP32: o buffer TX do `Serial` é definido antes de `begin()`. STM32: os
  buffers das UART são `SERIAL_RX/TX_BUFFER_SIZE` no `platformio.ini`.
- Nenhum objeto é dono de outro: todas as referências são não proprietárias, e
  o tempo de vida é o do programa inteiro.
- Para mudar o que uma chave faz — `config/Controls.h`, e não o `main.cpp`.
