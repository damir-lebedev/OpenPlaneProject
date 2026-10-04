# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/config.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

[← Referencia](README.md)

La capa de configuración son solo constantes `constexpr`, sin código. La lógica
de las clases no debe contener pines, tiempos de espera ni umbrales «mágicos»:
todo lo que pueda hacer falta cambiar para un avión o una placa concretos vive
aquí.

---

## namespace `Config`

**Archivo:** `include/config/Config.h` · **Depende de:** `<stdint.h>` ·
**La usan:** casi todas las capas.

### Pines (dependen de la placa)

El bloque de pines se elige con la macro que fija `[env:*]` en `platformio.ini`
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Sin macro: `#error`. El bloque de la STM32 se describe
[más abajo](#stm32h743vit6-board_stm32h743).

| Constante | Tipo | Función | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Alerones | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Timón de profundidad | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Regulador del motor | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Timón de dirección + rueda; `-1`: la salida está desactivada | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX del receptor iBUS (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | El bus de sensores (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | El bus del OLED (`Wire1`); `-1`: no hay | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | El bus SPI común | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS de la IMU por SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS del barómetro por SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | La UART del GPS; TX `-1`: solo recepción | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | El número de la UART por hardware para el GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | salidas de servo: lanzamiento de carga, flaps; `-1`: no hay | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | el zumbador mediante un transistor; `-1`: no hay | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **Solo S3:** reservados para la placa del controlador de vuelo ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

El bus SPI de los sensores se llama `PIN_SENSOR_SPI_*` y no `PIN_SPI_*`: en el
núcleo STM32duino (y en otros núcleos de Arduino) `PIN_SPI_SCK/MISO/MOSI` son
macros de la variante y sustituirían a las constantes de `Config`.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Todavía no hay placa: la distribución de pines **no está probada en hardware** (el firmware se ejecuta en el PC, env `native-stm32`). Los pines se han elegido entre los libres
de la WeAct MiniSTM32H743VITx (la placa del env `stm32h743` de PlatformIO) y se
han contrastado con las tablas `PeripheralPins` de la variante de STM32duino.
Los valores son macros de la variante (`PA0`…), por eso al principio de
`Config.h`, bajo `#if defined(BOARD_STM32H743)`, se incluye `<Arduino.h>`. El
tipo de todos los pines es `int16_t` (los pines analógicos se numeran
`0xC0 + N`). No hay números de UART: el núcleo elige el periférico por los
pines.

| Constante | Pin | Periférico |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX: reservado para iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2: sensores |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1: la pantalla (en la WeAct, el conector de la cámara) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1: carga / cámara |
| `PIN_BUZZER` | PE15 | GPIO: el zumbador |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11: reservados |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4: el radiomódem MAVLink (esos mismos pines son FDCAN1) |

La consola `Serial` es la LPUART1 (PA9 TX / PA10 RX), el valor predeterminado de
la variante.

### iBUS y pérdida de enlace

| Constante | Valor | Significado |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Cuántos canales de la trama se usan |
| `IBUS_FRAME_LENGTH` | 32 | Longitud de la trama, bytes |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | La cabecera de la trama |
| `IBUS_BAUDRATE` | 115200 | Velocidad de la UART |
| `RX_TIMEOUT_US` | 500 000 | Sin una trama correcta durante más tiempo que este: enlace perdido |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Acelerador por debajo de este valor: el receptor informa del failsafe de la emisora |

### GPS

| Constante | Valor | Significado |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | Un NAV-PVT más antiguo que esto: `UbloxM10_Gps::isAvailable() == false` |

### Rango del PWM y recorrido de las superficies

| Constante | Valor | Significado |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | El pulso RC estándar, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Deflexión desde el centro con el recorrido completo del stick, µs. El timón de dirección es menor: en el mismo servo va la rueda del tren de aterrizaje |
| `THROTTLE_LIMIT_PCT` | 100 | El techo del acelerador hacia el ESC, %, igual para el stick y para el piloto automático (`FlightController::capThrottle`). Para las pruebas de banco con una batería 3S1P débil se puso 50; las pruebas calculan la salida esperada a partir de este valor |

### Flaps (flaperones)

| Constante | Valor | Significado |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 por encima de este valor: flaps extendidos (no 1500: hasta la primera trama los canales = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Deflexión hacia abajo de cada alerón, µs (~20° del brazo del MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | El tiempo de la extensión/recogida completa |

### Sentido de los servos

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true`: los servos de los alerones están en espejo), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED`: el único lugar donde se fija la inversión. `ControlMixer`
calcula con signos físicos y cambia el signo solo aquí, por eso los sticks y el
piloto automático no pueden desincronizarse. La inversión en la emisora **no
debe** usarse.

### Montaje de los sensores

| Constante | Valor | Significado |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | El giro de los ejes del chip de la IMU alrededor de la vertical (0/90/180/270), hacia dónde mira el eje X del chip. Se usa solo mientras no haya una calibración del montaje `o` en NVS |
| `MAG_ROTATION_CW_DEG` | 0 | Lo mismo para la brújula (la brújula no tiene calibración del montaje) |

### ARM

| Constante | Valor | Significado |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 por encima de este valor: el interruptor ARM está activado |
| `THROTTLE_LOW_US` | 1050 | Acelerador por debajo de este valor: «acelerador abajo», se puede armar |

### Failsafe

| Constante | Valor | Significado |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Neutro de las superficies |
| `FAILSAFE_THROTTLE` | 1000 | Motor apagado |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | El alabeo del planeo al perder el enlace en el aire |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | El cabeceo del planeo (algo por debajo del horizonte) |
| `FAILSAFE_RTH` | `true` | Con GPS y punto de origen, la pérdida de enlace en el aire: regreso a casa con motor, y no planeo |

### Interruptores, tubo de Pitot, piloto automático

Los números de todos los modos y funciones están en `Config.h`, junto a
comentarios detallados; lo que significan para el piloto está en
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Grupo | Constantes |
|---|---|
| Interruptores | `SWITCH_ON_US` = 1750 (canal por encima de este valor: interruptor activado; no 1500, para que antes de la primera trama no se active nada) |
| Tubo de Pitot | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Estabilización | `MAX_BANK_DEG` 45 (potenciómetro 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navegación | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Altitud | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Acelerador y velocidad | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Pérdida de sustentación | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Círculos y punto de origen | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Geovalla | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Lanzamiento a mano | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Aterrizaje | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Vuelo a vela | `SOAR_*`: planeo −3°, una térmica > 0.5 m/s durante 1.5 s, un círculo de 25°, salida < −0.2 m/s durante 8 s, motor por debajo de 30 m hasta 100 m, regreso a casa a más de 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Autotrim | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, guardado en tierra: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Coordinación | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Funciones | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Ciclo, Wi-Fi, depuración

| Constante | Valor | Significado |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | El periodo del ciclo de vuelo (500 Hz); también el `dt` nominal de `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | El punto de acceso del dashboard (la contraseña es débil: una herramienta de banco) |
| `WEB_SERVER_PORT` | 80 | Puerto HTTP |
| `TELEM_BAUDRATE` | 57600 | La velocidad del radiomódem MAVLink (el valor predeterminado de SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | La dirección del aparato en MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | Cada cuánto comprueba `DebugLogger` los canales del registro |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Tolerancia a las oscilaciones del RC/PWM en el modo «al cambiar» |

### Caja negra

Con detalle en [BLACKBOX.md](../BLACKBOX.md).

| Constante | Valor | Significado |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | La cola de registros en PSRAM (sin PSRAM, en la memoria interna) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: la cola en RAM: 10 s de pregrabación y margen para los retrasos de la tarjeta |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: el archivo en la tarjeta SD y el techo de su parte utilizada (el tiempo de verificación al encender crece con el área) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Grabación antes del inicio (ARM + acelerador) y después del DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Armado, el motor parado y el avión inmóvil durante este tiempo: parar |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | Qué se considera «inmóvil» |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Tras un reinicio anómalo, grabar al menos este tiempo |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Espacio borrado que se mantiene preparado; los vuelos antiguos se borran enteros en tierra |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | La pausa entre borrados |
| `BLACKBOX_IMU_DIVIDER` | 1 | La IMU cada N ciclos (1: 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | Los divisores de la batería (56k/10k) y del sensor de corriente (10k/15k) en la placa del controlador de vuelo |

---

## namespace `Channels`

**Archivo:** `include/config/Channels.h` · **Depende de:** `<stdint.h>`

El único lugar donde el número físico de un canal se vincula con su función. Los
valores son **índices** (desde 0) en `RcChannelState`.

| Constante | Índice | Canal | Mando de la FS-i6 | Función |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | stick derecho ←→ | Alabeo |
| `ELEVATOR` | 1 | CH2 | stick derecho ↑↓ | Cabeceo (2000 = hacia delante = morro abajo) |
| `THROTTLE` | 2 | CH3 | stick izquierdo ↑↓ | Acelerador |
| `RUDDER` | 3 | CH4 | stick izquierdo ←→ | Timón de dirección + rueda |
| `ARM` | 4 | CH5 | SwA | El interruptor ARM (no se puede reasignar) |
| `SWB` | 5 | CH6 | SwB | según la tabla de `Controls.h` (por defecto, los flaps) |
| `SWC` | 6 | CH7 | SwC (3 pos.) | por defecto, el modo MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | por defecto, RTH |
| `VRA` | 8 | CH9 | VrA | por defecto, `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | por defecto, `CRUISE_SPEED` |
| `COUNT` | 10 | | | el número de canales |

---

## namespace `Controls`

**Archivo:** `include/config/Controls.h` · **Depende de:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]`: lo que hace cada interruptor y cada
potenciómetro, **una línea por canal** (`Bind::modes/mode/feature/knob`;
véase [autopilot.md](autopilot.md#binding-bind-bindingcheck)). Al lado hay
ideas ya preparadas, comentadas. Tres `static_assert` detectan errores de la
tabla al compilar: un stick o el ARM en la tabla, un canal fuera de rango, un
canal repetido, más de un interruptor de selección de modo.
