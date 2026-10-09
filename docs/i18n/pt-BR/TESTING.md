# TESTING.md — testes, cobertura e análise estática

> 🌐 Esta página é uma tradução do [original em russo](../../TESTING.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

O firmware é verificado em dois níveis:

| Onde | Comando | O quê |
|---|---|---|
| **PC (native)** | `pio test -e native` | Os cabeçalhos do firmware são compilados no PC sem alterações, com o hardware substituído por fakes controláveis: módulos, drivers, simulações de voo em malha fechada, o firmware inteiro do ESP32 (S3 e de 38 pinos) com cada kit de sensores. A cobertura é contabilizada |
| **PC (native-stm32)** | `pio test -e native-stm32` | O firmware inteiro da STM32H743 (`src/stm32/main.cpp`) sobre uma camada de fakes do STM32duino: tarefas do FreeRTOS, flash, MAVLink, sensores em I2C e SPI |
| **Matriz de builds** | `tools/build_matrix.sh` | 4 placas × 6 kits de sensores com `-Wall -Wextra (-Wshadow)`; qualquer aviso no código do projeto é um erro |
| **Placa** | `pio test -e esp32-s3` | `test_feedback` e `test_imu_orientation` em um ESP32-S3 real (grava um firmware de teste; depois coloque de volta o normal: `pio run -t upload`) |
| **Placa STM32** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | A caixa-preta em um **cartão SD real** da DevEBox H743, mais `test_feedback` e `test_imu_orientation` em um Cortex-M7 — [abaixo](#testes-na-placa-stm32) |

O contexto de arquitetura está em [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-testabilidade).

---

## Início rápido

```bash
pip install platformio gcovr        # uma única vez
# Windows: é preciso ter g++ no PATH, por exemplo o WinLibs (winlibs.com, zip UCRT):
# descompacte e adicione mingw64\bin ao PATH; não precisa instalar
pio test -e native -e native-stm32  # todos os testes nativos (~1,5 min)
gcovr                               # cobertura por arquivo (configurações: gcovr.cfg)
tools/build_matrix.sh               # todas as placas × todos os sensores (~25 min)
gcovr --html-details -o coverage/index.html   # relatório HTML (coverage/ está no .gitignore)

pio test -e native -f native/test_rc          # um único conjunto
pio test -e native -f test_feedback           # simulação da realimentação no PC

# Trajetórias das simulações em malha fechada em CSV (para gráficos):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# O fluxo MAVLink, para conferir com um decodificador de referência (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Antes de calcular a cobertura após mudanças nos testes, convém começar de um build limpo: `rm -rf .pio/build/native`; senão, os contadores de execuções anteriores entram no relatório.

---

## Como funciona o build nativo

`[env:native]` no `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (a pinagem da ESP32-S3), `-I test/native/support`, `-Wall -Wextra -Wshadow`, cobertura com `--coverage` e `-fkeep-inline-functions -fkeep-static-functions`; sem eles, o gcov não enxerga as funções dos cabeçalhos que nunca foram chamadas e superestima a cobertura.

### Fakes do hardware — `test/native/support/`

Cabeçalhos com os mesmos nomes e assinaturas do núcleo Arduino para ESP32 2.0.x, do ESP-IDF e das bibliotecas, mas sobre um mundo simulado em `namespace fake`:

| Arquivo | Substitui | O que a simulação sabe fazer |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | o núcleo Arduino | Macros (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, formatação do `print()` como no original. O `ARDUINO` **não** é definido de propósito |
| `esp32-hal-fake.h` | tempo, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | O relógio só avança por `fake::advance*()`/`delay()`; `millis()/micros()` são `uint32_t`, como no ESP32 (o estouro se comporta como na placa). Canais LEDC, `pulseIn` pelo ciclo de trabalho real (visível só se o buffer de entrada do pino estiver ativado), `analogReadMilliVolts`: a tensão vem de `fake::gpio().analogMv`. As tarefas são registradas (o handle é não nulo); `fake::runTask(task, n)` executa n passadas do seu laço infinito, `ulTaskNotifyTake` conta como uma passada e `xTaskNotifyGive`, como um contador. Os mutexes do FreeRTOS são uma flag de "ocupado". `psramFound()`/`ps_malloc()`. As seções críticas são contadas |
| `HardwareSerial.h` | UART | As portas são registradas por número (`fake::uart(1)`); `pushRx()`, `txBytes()`, troca de velocidade em andamento (`updateBaudRate`, o histórico é `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | partições de flash do ESP-IDF | Uma partição é um vetor de bytes com comportamento NOR: apagamento só por setores de 4 KB, apagado = 0xFF, uma gravação só abaixa bits (a tentativa de subir um bit é contada: `bitRaises`); `beforeWrite`: "faltou energia"; contadores de leituras, gravações e apagamentos |
| `esp_system.h` | a causa da reinicialização | `esp_reset_reason()` a partir de `fake::chip().resetReason` |
| `Wire.h` | I2C | Dispositivos por endereço; `fake::RegisterMapDevice`: registradores com autoincremento, log de gravações, falhas (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), ganchos `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Dispositivos por pino CS; `fake::SpiRegisterMapDevice`: o protocolo Bosch/InvenSense, `dummyBytes` antes dos dados |
| `Preferences.h` | NVS | Armazenamento em memória, comportamento de `begin(readOnly)`/`get*`/`getBytes` como no original; `failBegin` |
| `WiFi.h`, `WebServer.h` | ponto de acesso Wi-Fi, HTTP | O resultado de `softAP()` é definido pelo teste; `WebServer::request(método, uri, corpo)` chama o handler registrado; `fake::webServers()`: todas as instâncias |
| `U8g2lib.h` | U8g2 | No lugar de pixels, uma lista das strings e dos retângulos desenhados; `begin()/sendBuffer()` passam bytes por um callback de bytes do usuário; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### A camada do STM32duino — `test/native/support_stm32/` (ambiente `native-stm32`)

Fica em `-I` antes de `support/` e complementa esses mesmos fakes com o que só o STM32duino tem. O `<Preferences.h>` neste ambiente é o `include/hal/stm32/compat/Preferences.h` **real**, sobre o `KeyValueStore`.

| Arquivo | Substitui | O que sabe fazer |
|---|---|---|
| `Arduino.h` | o núcleo STM32duino | pinos `PA0..PE15` (porta·16 + número), `pin_size_t`, `PinMap_TIM` para os pinos de saída dos servos, `HardwareTimer` (o pulso é visto por `fake::timerPulseUs(pin)` e `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | FreeRTOS do STM32duino | `xTaskCreate` no registro comum de tarefas (pilha em palavras), `vTaskStartScheduler()` retorna: as tarefas são movidas pelo próprio teste (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | emulação de EEPROM | uma "flash" de 8 KB (apagada = 0xFF) e um buffer, `fake::eeprom()`: contadores e corrupção da imagem |
| `SPI.h` | | `SPIMode` |

Nos fakes comuns foi adicionado, para a STM32: `TwoWire(sda, scl)`, `setSDA/SCL` e `fake::wireWithSda(pin)` (para achar o segundo barramento da placa), `HardwareSerial(rx, tx)` e `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Emuladores de chips e modelo do avião — `test/native/helpers/`

| Arquivo | O que é |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (com registradores indiretos IPREG), QMC6309, SPL06-001, BMP581, quadros NAV-PVT da u-blox: mapas de registradores em I2C ou SPI, com dados do "mundo" `World` (ângulos e velocidades, altitude, velocidade do ar, rumo, coordenadas), nos eixos do chip, levando em conta o `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | um modelo de avião de ~1,2 kg: uma massa pontual + rotação em rolagem/arfagem, CL(α) com estol, arrasto, empuxo, vento, térmicas, solo |
| `SimHarness.h` | um laço fechado: rádio → quadro iBUS → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → deflexões das superfícies → `PlaneSim` → sensores (inclusive um tubo de Pitot sobre dois barômetros ruidosos). Uma trajetória em CSV com `OPENPLANE_SIM_DIR` |

O `test/native/helpers/TestSupport.h` é o que os conjuntos têm em comum: `resetWorld()` (chamado de `setUp()`), os substitutos `FakeUart`/`FakeServo`/`FakeBoard` e dos sensores (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), o montador de quadros `ibusFrame()`, as bancadas `I2cRig`/`SpiRig` (um driver sobre os `Esp32I2CBus`/`Esp32SpiBus` reais e um `*RegisterDevice` com um chip simulado).

Os testes da placa (`test_feedback`, `test_imu_orientation`) são portáteis: com `ARDUINO`, `setup()/loop()`; senão, `main()`. Os conjuntos `test/native/*` não são compilados para a placa (`test_ignore` em `[esp32_common]` e `[env:stm32h743]`: os padrões vão um por linha; separados por espaço, o PlatformIO os lê como um só). Na STM32: `pio test -e stm32h743`.

---

## Conjuntos de testes

| Conjunto | Testes | O que verifica |
|---|---|---|
| `native/test_hal` | 17 | Os auxiliares de `II2CBus` (NACK, leitura curta — o buffer não é tocado), `I2cRegisterDevice`, `SpiRegisterDevice` (bit de leitura, byte fictício do BMP388), `Esp32I2CBus` (timeout de 5 ms), `Esp32SpiBus` (modos 0–3), `Esp32UartPort` (8N1, pinos), `Esp32ServoOutput` (50 Hz/14 bits, limitação do pulso, falha do LEDC, medição pelo buffer de entrada), `Esp32Board` (barramentos, UART, ordem dos canais, AUX, buzzer) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, análise do iBUS: quadros que chegam em pedaços, CRC, valores de 12 bits, failsafe do rádio, timeout de 500 ms (inclusive com o estouro de `micros()`), lixo, ressincronização |
| `native/test_control` | 21 | Flaps (velocidade, primeira chamada, pausas), o mixer (sinais, reversão, flaperons), acelerador, a máquina de estados do ARM e as verificações dos sensores dos modos, a tabela de saídas e a autoverificação dos pulsos |
| `native/test_autopilot` | 22 | PID (termo D a partir da taxa do sensor, integral, anti-windup, `dt`), STABILIZE como modo de ângulos, decolagem automática por tempo, ALT_HOLD com o profundor, planeio na perda de sinal |
| `native/test_autopilot_modes` | 31 | Os 12 modos e a reação de cada um à falta de um sensor, a tabela de atribuições e `static_assert`, funções e potenciômetros, navegação (rumo, círculo, ponto de origem, geofence), failsafe RTH/planeio, lançamento manual, voo planado (soaring), auto-trim (gravação só no solo) |
| `native/test_flight_controller` | 12 | Um ciclo completo do `FlightController` com as classes reais: prioridades perda de sinal > ARM > sticks/piloto automático > acelerador; AUX, `MOTOR_KILL`, buzzer |
| `native/test_imu` | 21 | MPU6050/6500/9250 e ICM-42688: identificação, registradores, escalas, rotação dos eixos e sinais aeronáuticos, erros de barramento, calibração do giroscópio e verificação pré-voo, calibração da instalação a partir de três posições, NVS, filtro de orientação |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 por I2C e SPI, BME280/BMP280 conforme a referência da Bosch, bússolas (rumo, calibração hard-iron no NVS), u-blox M10 (CFG-VALSET, NAV-PVT, quadros corrompidos, timeout), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, endereço alternativo, SPI), ICM-45686 (registradores indiretos), QMC6309, SPL06-001 (fórmulas do datasheet), BMP581 (DRDY e o caminho reserva), o tubo de Pitot (zero, filtro, densidade, mangueiras trocadas, dados desatualizados, um “voo” com o ruído de dois barômetros) |
| `native/test_storage` | 16 | `KeyValueStore` (recarga, desgaste — um valor idêntico não é regravado, estouro sem perda de dados, CRC, falta de energia durante o apagamento, lixo, versão do formato), `KvPreferences` (comportamento como o NVS do ESP32) |
| `native/test_mavlink` | 20 | O codec contra quadros de referência do pymavlink (v1, v2, assinado), CRC, ressincronização; telemetria: frequências dos fluxos, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, parâmetros do PID (lista, leitura, gravação, recusa de valores ruins), troca de modo a partir do solo, ARM a partir do solo — recusado, missões — 0, um buffer de UART cheio não trava o laço |
| `native/test_blackbox` | 19 | A caixa-preta: formato e CRC, o anel de setores sobre um fake de NOR (partição nova sem apagamento, lixo é sempre apagado, voos antigos são apagados inteiros e só para liberar espaço, o último nunca é tocado, volta ao fim do anel, a cabeça após a reinicialização, falta de energia, um registro escrito pela metade detectado pelo CRC), gravação do voo com os `FlightController`/`Autopilot` reais: início por ARM e acelerador com pré-gravação, parada após o DISARM e “parado no solo”, a perda de sinal não a interrompe, gravação após uma reinicialização com falha, início manual, eventos, bateria, a flash acabou no ar, um voo mais longo que a partição, download em quadros com CRC e troca de velocidade, o menu do console `k`, sem partição — desativada |
| `native/test_blackbox_scan` | 3 | A verificação por amostragem do anel na inicialização contra a completa: 300 históricos aleatórios do anel × 5 passos de sondagem (a cabeça, os números e a lista de voos coincidem, e quando o quadro não fecha, cede à verificação completa) e o custo em uma área de SD de 64 MB (≈530 leituras em vez de 32 mil) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, versão), `DebugLogger` (todos os canais, NAV), `DebugConsole` (menu, teclas de atalho, sondagem dos barramentos `b`, proibida com ARM, salvamento só sem ARM), `WebDebugServer` (rotas, JSON, caixa de correio), `OledDisplay` (bytes por I2C, quadro, inversão na perda de sinal) |
| `native/test_sim` | 15 | Voos em malha fechada de todo o firmware com o modelo do avião: recuperação de uma inclinação lateral, CRUISE com vento de través, LOITER, RTH, failsafe RTH/planeio, geofence, decolagem automática de uma pista, lançamento manual, pouso automático, uma térmica, RESCUE a partir de uma espiral, manutenção da velocidade e proteção contra estol, um tubo de Pitot real na malha, auto-trim de um avião “torto”, falhas de sensores em voo (IMU, barômetro, tubo de Pitot, GPS) |
| `native/test_feedback_units` | 14 | Os módulos de realimentação um a um: fontes de velocidade, no ar/no solo, a estimativa RLS, o controlador, sinais de estol, cancelamentos da decolagem e do pouso |
| `native/test_app` | 10 | `src/main.cpp` no ESP32-S3 com o kit de bancada MPU6500/BMP581/QMC5883P/OLED: período do `loop()`, rádio → servos, ARM, modos, perda de sinal, console, dashboard, tela, caixa-preta (uma tarefa no núcleo 0, gravação com o acelerador, voo após o DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` no ESP32-S3 com o kit de voo: LSM6DSV + QMC6309 + SPL06 + BMP581 no tubo + GPS — identificação de todos os chips, zero do tubo e velocidade, altitude, ponto de origem pelo GPS, STABILIZE pelos ângulos do chip, RTH até o ponto de origem, sondagem dos barramentos, dashboard |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` no **ESP32 de 38 pinos** (`BOARD_ESP32_CLASSIC`) com o kit ICM-45686 + QMC6309 + SPL06 + BMP581: pinagem da placa, filtros IPREG, lançamento manual, estabilização e velocidade, sondagem de um único barramento |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` no **STM32H743** com o kit de voo: tarefas e prioridades, período de 2 ms, o tubo, timers PWM e `pulseIn`, MAVLink em voo, troca de modo a partir da GCS, configurações gravadas por uma tarefa em segundo plano na “flash”, a tela no I2C1, o console |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 com o ICM-45686 e o BMP581 **por SPI** + QMC6309: flash corrompida na inicialização, ALT_HOLD a partir da GCS mantém a altitude, perda de sinal → RTH, visível no MAVLink; regravação de uma imagem danificada |
| `native_stm32/test_blackbox_sd` | 29 | A caixa-preta no cartão SD: FAT32 (com e sem MBR, um diretório em dois clusters, entradas de ruído, um volume alheio/fragmentado/vazio), `SdFileRegion` (blocos incompletos, cache, apagamento, limites, falhas), o driver real `Stm32SdCard` sobre um `HAL_SD` falso (4 bits, velocidades reserva, nova tentativa, cartão ocupado, buffers desalinhados), o anel no cartão (reinicialização, falta de energia, custo da verificação), a marca “anel vazio”, gravação do voo em um `FlightController`, uma reinicialização com falha detectada por `RCC->RSR`, o ADC da bateria, erros do cartão em voo, um cartão lento, download pelo console, a tecla `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` com cartão: a inicialização encontra o cartão e o arquivo, a tarefa `bbox` grava o voo, o período do ciclo não se alonga, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` sem cartão: a caixa-preta fica desativada e explica o motivo, o avião voa, o menu `k` não quebra |
| `test_feedback` | 10 | Simulação do avião em malha fechada com o laço de realimentação (no PC e na placa) |
| `test_imu_orientation` | 5 | Calibração da instalação da IMU em 300 instalações aleatórias (no PC e na placa) |
| **Total** | **387** | 340 em `native` + 47 em `native-stm32` (mais 9 só na placa — `test_blackbox_sd`) |

### Testes na placa STM32

O `test/test_blackbox_sd` não é nativo: o driver do SDMMC, o cartão e o tempo são reais. Os testes rodam em uma tarefa do FreeRTOS e, ao lado dela, roda uma tarefa que imita o ciclo de voo com a prioridade mais alta (período de 2 ms): ela preempta os testes no meio dos acessos ao cartão, como no firmware. Sem ela, não dá para pegar o erro que foi encontrado na placa: na preempção, o FIFO do SDMMC estourava (`HAL_SD_ERROR_RX_OVERRUN`), algo que não acontece em um laço simples.

| Teste | O que verifica |
|---|---|
| `reset_cause_is_a_normal_one` | a causa da reinicialização (`RCC->RSR`) não é o watchdog nem uma queda de tensão |
| `card_is_detected_on_four_bit_bus` | o cartão é reconhecido em um barramento de 4 bits a 24 MHz |
| `file_is_found_and_contiguous` | o `BLACKBOX.BIN` é encontrado no FAT32 e é contíguo |
| `multi_block_writes_work_at_every_length` | gravação de 1, 2, 4 e 8 blocos em um único acesso |
| `pages_write_with_bounded_latency_and_read_back_intact` | páginas de 256 B: a pior gravação < 250 ms (o limite do SD), de forma estável > 40 KB/s, leitura e apagamento |
| `header_scan_cost_on_the_whole_area` | o custo de ler o cabeçalho de um setor e da verificação completa |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | apagar tudo, dois voos de 20 000 registros, “reinicialização”: a verificação por amostragem leva < 2 s, os registros são lidos em ordem com o CRC correto; um anel vazio é reconhecido pela marca em < 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | o `BlackBox` real com IMU a 500 Hz em tempo real: nenhum registro perdido, o voo é lido após a “reinicialização” |
| `the_flight_task_was_not_disturbed` | a gravação no cartão não alterou o período da tarefa que imita o voo (desvio < 3 ms) |

Execução (um cartão com o arquivo — `python tools/blackbox.py sd-prepare E:`; **o teste apaga todos os voos do arquivo**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

A placa precisa estar em modo DFU (na DevEBox — o fio BT0→3V3 e o RST, driver WinUSB via Zadig; detalhes em [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). O console da STM32 é USB CDC: depois de gravar o firmware, a porta não aparece de imediato, e o `pio test` às vezes não consegue abri-la a tempo (“could not open port”) — nesse caso, execute `pio test ... --without-testing` e leia a saída em qualquer programa de terminal com o DTR ativado (os testes esperam até 60 s a porta ser aberta). Depois dos testes, a placa espera a tecla **`D`** — ela reinicia a placa em DFU sem o fio.

Resultados na DevEBox H743 + cartão de 16 GB (2026-10-02): `test_blackbox_sd` — 9/9, `test_feedback` — 10/10, `test_imu_orientation` — 5/5; os números de velocidade do cartão estão em [BLACKBOX.md](BLACKBOX.md#o-que-foi-medido-na-placa).

### Firmwares de bancada — `test/bench/`

Não são conjuntos de testes, mas miniprojetos independentes do PlatformIO, que são gravados na placa no lugar do firmware de voo (o `pio test` não os enxerga: os nomes das pastas não começam com `test_`). Os pinos e os limites vêm do `Config.h` comum.

| Projeto | O que faz |
|---|---|
| `bench/elevator_sweep` | Move por programa o stick do profundor (CH2) através do `ControlMixer` e do `FlightOutputs`, como um stick de verdade: para cima 100% do curso, para baixo 60%, com suavidade e com pausas; 20 s de trabalho — 20 s no neutro. Nas posições extremas mede o pulso nas saídas. Acelerador no mínimo |

Para gravar: `pio run -d test/bench/elevator_sweep -t upload`. Para voltar ao firmware de voo: `pio run -e esp32-s3 -t upload`.

---

## Cobertura

É calculada pelo `gcovr` sobre `include/` e `src/` (tudo que entra no firmware), nos dois ambientes nativos juntos: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Camada | Linhas | Ramificações |
|---|---|---|
| `autopilot` | 920/943 (97,6%) | 645/731 (88,2%) |
| `autopilot/feedback` | 683/702 (97,3%) | 501/570 (87,9%) |
| `control` | 252/256 (98,4%) | 171/189 (90,5%) |
| `hal` | 98/102 (96,1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99,0%) | 21/22 (95,5%) |
| `hal/stm32` | 149/158 (94,3%) | 35/52 (67,3%) |
| `rc` | 92/92 (100%) | 41/42 (97,6%) |
| `sensors` (todos) | 1444/1446 (99,9%) | 716/835 (85,7%) |
| `storage` | 220/220 (100%) | 158/178 (88,8%) |
| `telemetry` | 1413/1440 (98,1%) | 1123/1269 (88,5%) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95,9%) | 20/29 (69,0%) |
| **Total** | **5490/5584 (98,3%)** | **3457/3943 (87,7%)**; funções 877/902 (97,2%) |

O que continua sem cobertura e por quê:

- **Lançamento manual** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) — inalcançável com `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`; vai aparecer nos testes quando a constante se tornar configurável (a mudança para o `Config.h`).
- **Dependente da placa:** uma saída sem pino (`PIN_RUDDER = -1` só ocorre na C3), GPS sem pino TX (C3) — os testes nativos exercitam a pinagem da S3, da de 38 pinos e da STM32, mas não a da C3 (a C3 é verificada pela matriz de builds).
- **STM32:** os ramos de erro do núcleo (não há timer no pino, o conjunto de timers se esgotou), a mensagem `FreeRTOS не запустился` (“o FreeRTOS não iniciou”) — no PC o `vTaskStartScheduler()` sempre retorna.
- **Ramos defensivos** que não dá para alcançar pela API pública: `default`/`Count` em um `switch` sobre enumerações, `return "?"`.
- Arquivos sem linhas executáveis (`Config.h`, `Channels.h`, `FeedbackConfig.h`, as estruturas `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, as macros do `SensorSelection.h`, o HTML do dashboard) não aparecem no relatório — eles são compilados nos testes, mas o gcov não tem o que contar neles.

---

## Análise estática

| Ferramenta | Comando | Perfil |
|---|---|---|
| GCC | `tools/build_matrix.sh` (ou `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Todas as placas × todos os kits de sensores. A compilação nativa dos testes usa sempre `-Wall -Wextra -Wshadow`; o `stm32h743` usa `-Wall -Wextra` (`build_src_flags`; o `-Wshadow` faz barulho nos próprios cabeçalhos do STM32duino) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` em `[esp32_common]`: `include/` e `src/` (exceto `stm32/`), warning/style/performance/portability, supressões em linha `// cppcheck-suppress` só para falsos positivos (o callback do U8g2, `setup/loop`). Para o `stm32h743`, as mesmas flags sobre `include/hal/stm32/` e `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` e outras; as verificações desativadas são explicadas no próprio arquivo |

O clang-tidy é executado com os fakes de `test/native/support`: o clang não consegue interpretar os cabeçalhos do ESP-IDF para a arquitetura do host (se você tentar o `pio check` com `clangtidy`, a análise para nos erros de interpretação e, na prática, não verifica nada). O `misc-include-cleaner` garante que cada cabeçalho inclua o que usa: os cabeçalhos “guarda-chuva” (`FeedbackModules.h`, a API de `IBoard.h`/`RegisterDevice.h`, as macros do `SensorSelection.h`) são marcados com `// IWYU pragma: export`. O script pula o código da STM32 (`include/hal/stm32/`, `src/stm32/`) — ele é verificado pelo build, pelo cppcheck do ambiente `stm32h743` e pelos testes do ambiente `native-stm32`.

A matriz de builds na última rodada — **24/24 sem avisos**:

| Placa | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) — 0 observações no código do projeto.

---

## Como escrever novos testes

1. Um módulo com lógica e sem hardware — um teste unitário direto: o tempo é passado como parâmetro ou avançado com `fake::advanceMs()`.
2. Um driver de chip — por meio de `I2cRig`/`SpiRig`: os registradores de um chip simulado, verificação dos valores gravados (`chip.lastWrite(reg)`) e da interpretação dos dados. Para fórmulas, use uma referência do datasheet ou um cálculo independente, e não uma cópia do código.
3. Classes com tarefas infinitas do FreeRTOS — `fake::findTask("nome")` + `fake::runTask(task, n)`; é assim também que se move a tarefa de voo da STM32.
4. O firmware inteiro com outro kit de sensores ou outra placa — um conjunto separado que, antes de `#include "../../../src/main.cpp"`, define `SENSOR_KIT` (ou `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); os chips vêm de `helpers/ChipEmulators.h`. Para a STM32 — `test/native_stm32/`.
5. Um novo modo do piloto automático — um cenário de voo em malha fechada no `test_sim`.
6. Um novo conjunto — uma pasta `test/native/test_<nome>/test_main.cpp` com `main()`; o `setUp()` chama `resetWorld()` se o conjunto não precisar de estado entre os testes.
7. Achou um bug — primeiro um teste que o pegue, depois a correção.
