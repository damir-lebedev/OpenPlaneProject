# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/config.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

[← Referência](README.md)

A camada de configuração são apenas constantes `constexpr`, sem código. A
lógica das classes não deve conter pinos, tempos limite e limiares “mágicos”:
tudo o que pode ser preciso mudar para uma aeronave ou placa específica mora
aqui.

---

## namespace `Config`

**Arquivo:** `include/config/Config.h` · **Depende de:** `<stdint.h>` ·
**Usada por:** quase todas as camadas.

### Pinos (dependem da placa)

O bloco de pinos é escolhido pela macro que o `[env:*]` define no
`platformio.ini` (`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Sem macro — `#error`. O bloco da STM32 está descrito
[abaixo](#stm32h743vit6-board_stm32h743).

| Constante | Tipo | Função | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Ailerons | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Profundor | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Controlador do motor | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Leme + roda; `-1` — saída desativada | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX do receptor iBUS (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | O barramento dos sensores (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | O barramento do OLED (`Wire1`); `-1` — não há | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | O barramento SPI comum | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS da IMU por SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS do barômetro por SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | A UART do GPS; TX `-1` — somente recepção | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | O número da UART de hardware para o GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | saídas de servo: soltar a carga, flaps; `-1` — não há | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | o buzzer por um transistor; `-1` — não há | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **Somente S3:** reservados para a placa do controlador de voo ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

O barramento SPI dos sensores se chama `PIN_SENSOR_SPI_*`, e não `PIN_SPI_*`: no
núcleo STM32duino (e em outros núcleos do Arduino) `PIN_SPI_SCK/MISO/MOSI` são
macros da variante e substituiriam as constantes do `Config`.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Ainda não há placa: a pinagem **não foi testada no hardware** (o firmware roda no PC, env `native-stm32`). Os pinos foram escolhidos entre os livres
da WeAct MiniSTM32H743VITx (a placa do env `stm32h743` do PlatformIO) e
conferidos com as tabelas `PeripheralPins` da variante do STM32duino. Os
valores são macros da variante (`PA0`…), por isso no início do `Config.h`, sob
`#if defined(BOARD_STM32H743)`, é incluído o `<Arduino.h>`. O tipo de todos os
pinos é `int16_t` (os pinos analógicos têm o número `0xC0 + N`). Não há números
de UART — o núcleo escolhe o periférico pelos pinos.

| Constante | Pino | Periférico |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — reservado para o iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — sensores |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — a tela (na WeAct — o conector da câmera) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — carga / câmera |
| `PIN_BUZZER` | PE15 | GPIO — o buzzer |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — reservados |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — o rádio modem MAVLink (os mesmos pinos são o FDCAN1) |

O console `Serial` é a LPUART1 (PA9 TX / PA10 RX), o padrão da variante.

### iBUS e perda de link

| Constante | Valor | Significado |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Quantos canais do quadro são usados |
| `IBUS_FRAME_LENGTH` | 32 | Comprimento do quadro, bytes |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | O cabeçalho do quadro |
| `IBUS_BAUDRATE` | 115200 | Velocidade da UART |
| `RX_TIMEOUT_US` | 500 000 | Sem um quadro correto por mais tempo que isso — o link foi perdido |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Acelerador abaixo disso — o receptor informa o failsafe do rádio |

### GPS

| Constante | Valor | Significado |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | Um NAV-PVT mais antigo que isso — `UbloxM10_Gps::isAvailable() == false` |

### Faixa do PWM e cursos das superfícies

| Constante | Valor | Significado |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | O pulso RC padrão, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Deflexão a partir do centro com o curso total do stick, µs. O leme é menor: no mesmo servo fica a roda do trem de pouso |
| `THROTTLE_LIMIT_PCT` | 100 | O teto do acelerador para o ESC, %, igual para o stick e para o piloto automático (`FlightController::capThrottle`). Para os testes de bancada com uma bateria 3S1P fraca foi usado 50; os testes calculam a saída esperada a partir desse valor |

### Flaps (flaperons)

| Constante | Valor | Significado |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 acima disso — flaps estendidos (não 1500: até o primeiro quadro os canais = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Deflexão para baixo de cada aileron, µs (~20° do braço do MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | O tempo da extensão/recolhimento completo |

### Sentido dos servos

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` — os servos dos ailerons estão em espelho), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — o único lugar onde a reversão é definida. O `ControlMixer`
calcula em sinais físicos e inverte o sinal somente aqui, por isso os sticks e
o piloto automático não podem divergir. A reversão no rádio **não deve** ser
usada.

### Instalação dos sensores

| Constante | Valor | Significado |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | A rotação dos eixos do chip da IMU em torno da vertical (0/90/180/270), para onde aponta o eixo X do chip. Usada apenas enquanto não há a calibração da instalação `o` no NVS |
| `MAG_ROTATION_CW_DEG` | 0 | O mesmo para a bússola (a bússola não tem calibração da instalação) |

### ARM

| Constante | Valor | Significado |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 acima disso — a chave ARM está ligada |
| `THROTTLE_LOW_US` | 1050 | Acelerador abaixo disso — “acelerador embaixo”, é possível armar |

### Failsafe

| Constante | Valor | Significado |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Neutro das superfícies |
| `FAILSAFE_THROTTLE` | 1000 | Motor desligado |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | A inclinação do planeio na perda de link no ar |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | A arfagem do planeio (um pouco abaixo do horizonte) |
| `FAILSAFE_RTH` | `true` | Com GPS e ponto de origem, a perda de link no ar — retorno ao ponto de origem com o motor, e não planeio |

### Chaves, tubo de Pitot, piloto automático

Os números de todos os modos e funções estão no `Config.h`, junto de
comentários detalhados; o que eles significam para o piloto está no
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Grupo | Constantes |
|---|---|
| Chaves | `SWITCH_ON_US` = 1750 (canal acima disso — chave ligada; não 1500, para que nada seja ligado antes do primeiro quadro) |
| Tubo de Pitot | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Estabilização | `MAX_BANK_DEG` 45 (potenciômetro 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navegação | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Altitude | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Acelerador e velocidade | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Estol | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Círculos e ponto de origem | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Geofence | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Lançamento manual | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Pouso | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Voo planado | `SOAR_*`: planeio −3°, uma térmica > 0.5 m/s por 1.5 s, um círculo de 25°, saída < −0.2 m/s por 8 s, motor abaixo de 30 m até 100 m, retorno ao ponto de origem além de 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Auto-trim | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, gravação no solo: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Coordenação | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Funções | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Ciclo, Wi-Fi, depuração

| Constante | Valor | Significado |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | O período do laço de voo (500 Hz); também o `dt` nominal do `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | O ponto de acesso do dashboard (a senha é fraca — uma ferramenta de bancada) |
| `WEB_SERVER_PORT` | 80 | Porta HTTP |
| `TELEM_BAUDRATE` | 57600 | A velocidade do rádio modem MAVLink (o padrão do SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | O endereço da aeronave no MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | Com que frequência o `DebugLogger` verifica os canais do log |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Tolerância à oscilação do RC/PWM no modo “ao mudar” |

### Caixa-preta

Em detalhe — [BLACKBOX.md](../BLACKBOX.md).

| Constante | Valor | Significado |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | A fila de registros na PSRAM (sem PSRAM — na memória interna) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: a fila na RAM — 10 s de pré-gravação e folga para os atrasos do cartão |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: o arquivo no cartão SD e o teto da sua parte usada (o tempo de verificação na inicialização cresce com a área) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Gravação antes do início (ARM + acelerador) e depois do DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Armado, o motor parado e a aeronave imóvel por esse tempo — parar |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | O que se considera “imóvel” |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Depois de um reinício anormal — gravar por no mínimo esse tempo |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Espaço apagado mantido pronto; os voos antigos são apagados inteiros no solo |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | A pausa entre apagamentos |
| `BLACKBOX_IMU_DIVIDER` | 1 | A IMU a cada N-ésimo ciclo (1 — 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | Os divisores da bateria (56k/10k) e do sensor de corrente (10k/15k) na placa do controlador de voo |

---

## namespace `Channels`

**Arquivo:** `include/config/Channels.h` · **Depende de:** `<stdint.h>`

O único lugar onde o número físico do canal é ligado à sua função. Os valores
são **índices** (a partir de 0) em `RcChannelState`.

| Constante | Índice | Canal | Controle do FS-i6 | Função |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | stick direito ←→ | Rolagem |
| `ELEVATOR` | 1 | CH2 | stick direito ↑↓ | Arfagem (2000 = para longe de você = nariz para baixo) |
| `THROTTLE` | 2 | CH3 | stick esquerdo ↑↓ | Acelerador |
| `RUDDER` | 3 | CH4 | stick esquerdo ←→ | Leme + roda |
| `ARM` | 4 | CH5 | SwA | A chave ARM (não pode ser reatribuída) |
| `SWB` | 5 | CH6 | SwB | conforme a tabela do `Controls.h` (por padrão, os flaps) |
| `SWC` | 6 | CH7 | SwC (3 pos.) | por padrão, o modo MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | por padrão, RTH |
| `VRA` | 8 | CH9 | VrA | por padrão, `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | por padrão, `CRUISE_SPEED` |
| `COUNT` | 10 | | | o número de canais |

---

## namespace `Controls`

**Arquivo:** `include/config/Controls.h` · **Depende de:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — o que cada chave e cada potenciômetro faz,
**uma linha por canal** (`Bind::modes/mode/feature/knob`, veja
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). Ao lado há ideias
prontas, comentadas. Três `static_assert` pegam erros da tabela na hora da
compilação: um stick ou o ARM na tabela, um canal fora da faixa, um canal
repetido, mais de uma chave de seleção de modo.
