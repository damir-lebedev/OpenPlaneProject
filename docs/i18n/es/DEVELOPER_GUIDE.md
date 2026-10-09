# DEVELOPER_GUIDE.md: guía del desarrollador de OpenPlaneProject

> 🌐 Esta página es una traducción del [original en ruso](../../DEVELOPER_GUIDE.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Un mapa técnico del firmware: qué archivo se encarga de qué, cómo fluyen los datos desde el receptor y los sensores hasta los servos, qué convenios de signos mantienen unida toda la cadena, cómo está organizada la API web y cómo ampliar el proyecto. Está pensada para un desarrollador que escribe C++ y quiere orientarse rápido en este repositorio (la rama `main`), no para aprender los fundamentos del lenguaje ni de PlatformIO.

Para una visión general del proyecto y el estado del prototipo, véase [`../README.md`](README.md); para saber qué conectar a dónde y cómo volar, [`PILOT_GUIDE.md`](PILOT_GUIDE.md); para los planes, [`ROADMAP.md`](ROADMAP.md). Aquí solo hay código. La arquitectura completa (capas, tareas, máquinas de estados) está en [`ARCHITECTURE.md`](ARCHITECTURE.md), la referencia de cada clase en [`reference/`](reference/README.md) y las pruebas en [`TESTING.md`](TESTING.md).

> El proyecto está en desarrollo activo. El banco con el ESP32-S3 está montado y comprobado con todos los sensores, pero **el piloto automático todavía no se ha probado en vuelo**; esto se indica donde afecta a un módulo concreto. Si dudas de lo que hace el código, vuelve a leer el código fuente, no el documento.

---

## Contenido

1. [Arquitectura de capas](#arquitectura-de-capas)
2. [Tareas de FreeRTOS y el lazo de control](#tareas-de-freertos-y-el-lazo-de-control)
3. [Referencia de archivos](#referencia-de-archivos)
4. [Convenio de signos: de la IMU al servo](#convenio-de-signos-de-la-imu-al-servo)
5. [Mapa de canales RC, ARM y failsafe](#mapa-de-canales-rc-arm-y-failsafe)
6. [Datos de los sensores](#datos-de-los-sensores)
7. [Desglose de FlightController::update()](#desglose-de-flightcontrollerupdate)
8. [API HTTP del dashboard web](#api-http-del-dashboard-web)
9. [Consola y diagnóstico](#consola-y-diagnóstico)
10. [Elección de la placa y distribución de pines](#elección-de-la-placa-y-distribución-de-pines)
11. [Cómo añadir un sensor nuevo](#cómo-añadir-un-sensor-nuevo)
12. [Cómo añadir un modo nuevo del piloto automático](#cómo-añadir-un-modo-nuevo-del-piloto-automático)
13. [Realimentación (base preparada, no conectada)](#realimentación-base-preparada-no-conectada)
14. [Cómo añadir una placa nueva](#cómo-añadir-una-placa-nueva)
15. [Comandos de compilación, carga y monitor](#comandos-de-compilación-carga-y-monitor)
16. [Limitaciones conocidas](#limitaciones-conocidas)
17. [Cómo hacer cambios](#cómo-hacer-cambios)

---

## Arquitectura de capas

Casi todas las clases viven en cabeceras repartidas en las carpetas `include/<capa>/`. Cada cabecera incluye por sí misma lo que usa (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ...: rutas desde `include/`). `src/main.cpp` es el único punto de ensamblaje (composition root): crea todos los objetos, los enlaza y ejecuta `setup()`/`loop()`. Las dependencias son unidireccionales: una capa inferior no sabe nada de la superior.

```
include/
├── config/      Config.h (pines, todos los ajustes), Channels.h (nombres de los canales),
│                Controls.h (qué hace cada interruptor: una línea por canal)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (dispositivo de registros sobre I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + envoltorios sobre Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (ajustes en la flash en lugar de NVS)
├── storage/     KeyValueStore, KvPreferences — almacenamiento de ajustes sin NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  base de la realimentación: NO conectada (véase la sección de más abajo)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (un tubo hecho con dos barómetros)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — firmware del ESP32 (S3, C3, 38 pines)
src/stm32/main.cpp  — firmware del STM32H743 (tareas de FreeRTOS, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — montaje de objetos   │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — orden de las operaciones      │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 modos   │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ navegación, PID    │  │ RcChannelState        │
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

Las reglas que mantienen limpia la arquitectura:

- **HAL** es la única capa a la que se permite conocer una MCU concreta (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Todo lo de encima trabaja solo con interfaces. Pasar a otra MCU significa un nuevo `hal/<mcu>/<Mcu>Board.h`; el resto del código no cambia (un ejemplo es `hal/stm32/` para el STM32H743).
- **Los drivers de sensores no conocen el bus.** Reciben un `IRegisterDevice&`: el dispositivo I2C con su dirección o el SPI con su CS se crea en `SensorSelection.h`. Un mismo `BMP388_Sensor` funciona tanto por I2C como por SPI.
- **Lo común vive en las clases base.** Calibración, giro de ejes, signos, filtro de orientación, altitud y velocidad vertical, almacenamiento de la calibración de la brújula, recuento de errores del bus: en `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. El driver de un chip solo contiene los registros y las fórmulas de la hoja de datos.
- **RC y Outputs** no saben nada del avión: bytes de iBUS → canales, valores de PWM → salidas.
- **Control y Autopilot** son lógica sobre datos, sin UART, PWM ni Wi-Fi. El tiempo, donde hace falta (flaps), se pasa como parámetro.
- **Coordination** (`FlightController`) es la única clase que ve a la vez varias capas inferiores y decide el orden de las operaciones.
- **Application** (`main.cpp`) es el único lugar donde se crean `Esp32Board`, los dispositivos y los sensores y donde todo se conecta a mano, sin un framework de inyección de dependencias.

---

## Tareas de FreeRTOS y el lazo de control

| Dónde | Qué | Periodo |
|---|---|---|
| Núcleo 1, `loop()` (el loopTask de Arduino) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Núcleo 0, la tarea `web` | `WebServer::handleClient()` | cada 2 ms |
| Núcleo 0, la tarea `oled` | dibujo del SSD1306 por el segundo bus I2C | 200 ms |
| Núcleo 0 | la pila Wi-Fi de ESP-IDF | — |

- El periodo del lazo lo mantiene `vTaskDelayUntil`, y no un `delay(2)` tras el trabajo: la frecuencia no depende de cuánto dure el ciclo. Tras un bloqueo largo (una calibración desde la consola) la cuenta empieza de nuevo y los ciclos perdidos no se recuperan de golpe.
- En el banco (ESP32-S3, todos los sensores): 500 Hz, unos ~0.7 ms de trabajo por ciclo de media y ~1.4 ms en el peor ciclo. Esto se imprime cada 10 s en una línea `SYS:`.
- El tiempo de espera de una transacción I2C es de 5 ms (el de Wire por defecto es de 50 ms): una transacción colgada por una interferencia no detiene el lazo durante mucho tiempo.
- **Separación de datos entre tareas.** La web y la OLED solo *leen* el estado (`FlightController`/`Autopilot`/`LoopStats`): son campos independientes de 16/32 bits, así que en el peor de los casos se ven los valores de ciclos contiguos. Las *órdenes* del dashboard (`setmode`/`setpid`) no se aplican directamente desde la tarea web: se colocan en un «buzón» bajo `portMUX` y el lazo de vuelo las recoge en `WebDebugServer::applyPendingCommands()`.
- `Serial` (UART0 → el puente CH343 → el conector «COM») con un búfer de transmisión de 4 KB: una trama de depuración (~600 caracteres) no bloquea el lazo mientras se envía.

---

## Referencia de archivos

### `config/`

| Archivo | De qué se encarga |
|---|---|
| `Config.h` | Todos los pines (un bloque por placa: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) y los ajustes: iBUS y pérdida de señal; recorrido de las superficies; flaps; inversión de servos; montaje de la IMU y de la brújula; ARM; failsafe (RTH o planeo); el tubo de Pitot (`PITOT_*`); todos los números de los modos y las funciones del piloto automático; el ciclo; Wi-Fi; MAVLink; depuración |
| `Channels.h` | Nombres de los canales: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | La tabla `BINDINGS`: qué hace cada interruptor y cada potenciómetro, una línea por canal, comprobaciones con `static_assert` |

### `hal/`

| Archivo | De qué se encarga |
|---|---|
| `IBoard.h` | El punto de entrada al hardware: `i2c()`, `displayI2c()` (un segundo bus para la pantalla, puede ser `nullptr`), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, puede ser `nullptr`), `servo(ServoChannel::*)` (7 salidas con AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | Las tareas de FreeRTOS de la misma forma en el ESP32 (núcleo 0) y en la STM32 (prioridades), la memoria dinámica libre |
| `II2CBus.h` | El bus I2C: primitivas con la forma de `Wire` + los auxiliares `writeRegister()`, `readRegisters()` (comprueba que llegaron exactamente `count` bytes), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, una salida PWM (`measurePulseUs()` — diagnóstico del pulso real) |
| `RegisterDevice.h` | `IRegisterDevice` — «un conjunto de registros de 8 bits»; `I2cRegisterDevice` (dirección), `SpiRegisterDevice` (CS, frecuencia, bytes ficticios antes de los datos) |
| `esp32/Esp32Board.h` | La implementación de `IBoard`: `Wire` (sensores), `Wire1` (la pantalla, si el chip tiene dos controladores I2C), `SPI`, dos `HardwareSerial`, 5 canales LEDC |
| `esp32/Esp32I2CBus.h` | `II2CBus` sobre cualquier `TwoWire`, tiempo de espera de 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM mediante LEDC: 50 Hz, 14 bits; pin −1: la salida no está cableada. No se usa la biblioteca ESP32Servo; véanse las [limitaciones](#limitaciones-conocidas) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Envoltorios finos sobre `SPI` y `HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ la UART4 del radiomódem), buses, temporizadores PWM, `Stm32FlashStorage` (ajustes en un sector de la flash, escritos por una tarea en segundo plano), `compat/Preferences.h` |

### `storage/`

| Archivo | De qué se encarga |
|---|---|
| `KeyValueStore.h` | Una imagen «espacio/clave → bytes» con CRC32 en RAM sobre cualquier soporte (`IFlashStorage`); un valor idéntico no se reescribe |
| `KvPreferences.h` | La API `Preferences` del ESP32 sobre `KeyValueStore` |

### `rc/`

| Archivo | De qué se encarga |
|---|---|
| `RcChannelState.h` | Una instantánea de los 10 canales |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → canales: trama de 32 bytes, CRC, el valor del canal son los 12 bits bajos (`& 0x0FFF`); `isSignalLost()` = no hay tramas (o aún no ha habido ninguna) ∥ el valor de failsafe del acelerador; contadores de tramas |

### `control/`

| Archivo | De qué se encarga |
|---|---|
| `ControlCommand.h` | La orden a las superficies en signos físicos: el lenguaje común de los sticks, el piloto automático y el mezclador |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(objetivo, now)`; `mix(command)` → PWM con inversión de servos; flaperones: los alerones `flaps ± roll` (el menos, aerofreno) |
| `FlapsController.h` | Salida y recogida suaves de los flaps, con el tiempo pasado como parámetro |
| `ThrottleManager.h` | Acelerador desde el stick; al perder la señal, `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM con el interruptor SwA (una transición OFF→ON con el acelerador abajo + las comprobaciones de sensores del modo), DISARM instantáneo |
| `FlightOutputState.h` | Los PWM deseados: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (carga), `aux2` (cámara) |
| `Beeper.h` | El zumbador: por la función `BEEPER` o «modelo perdido» en tierra |
| `FlightOutputs.h` | La tabla de salidas (`outputInfo()`: clave, nombre, pin, si es obligatoria, campo de estado) y todo lo que va encima en un bucle: `begin()`, `write()`, `setFailsafe()`, estado, `printPulseSelfTest()` |
| `FlightController.h` | El orden de las operaciones por ciclo, la pérdida de señal (`applyLinkLoss()`), getters para la telemetría |

### `autopilot/`

| Archivo | De qué se encarga |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 modos), `Feature`, `Knob`, `PilotInputs`, nombres |
| `ControlBinding.h` | `Binding`, las fábricas `Bind::modes/mode/feature/knob`, las comprobaciones `BindingCheck` |
| `PilotSwitches.h` | La tabla de asignaciones → el modo, las funciones y los potenciómetros de cada ciclo; la disposición al encender |
| `Autopilot.h` | 12 modos, failsafe RTH/planeo, geovalla, punto de origen, coordinación del viraje, autotrim; `update(armed, linkLost, acelerador, sticks)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (distancia, rumbo, desplazamiento), `Guidance` (alabeo hacia un rumbo, el campo vectorial del círculo) |
| `AltitudeSpeedController.h` | Cabeceo para la altitud, acelerador para la velocidad aerodinámica (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | Las máquinas de estados del lanzamiento a mano y del vuelo a vela |
| `AutoTrim.h` | Autotrim, guardado en NVS/flash |
| `PidController.h` | PID: D sobre la velocidad del sensor (giroscopio, variómetro), anti-windup, integrador congelado sin ARM |
| `feedback/*` | **Base preparada, no conectada:** realimentación adaptativa, despegue y aterrizaje; véase [Realimentación](#realimentación-base-preparada-no-conectada) |

### `sensors/`

| Archivo | De qué se encarga |
|---|---|
| `SensorInterface.h` | Las interfaces `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` y las estructuras de datos |
| `SensorSelection.h` | Qué chip se compila (`#define SENSOR_*`, se puede cambiar con una bandera de compilación) y en qué bus está (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Giro de los ejes del chip a los ejes del avión (0/90/180/270° en sentido horario): para la brújula y para una IMU sin calibración de montaje |
| `imu/ImuOrientation.h` | Montaje de la IMU como matriz «ejes del chip → ejes del avión»: a partir de `IMU_ROTATION_CW_DEG` o de tres posturas (nivelado, morro arriba, ala derecha abajo) con verificación; se guarda en NVS |
| `imu/ImuSensorBase.h` | Lo común de las IMU: calibración del giroscopio + comprobación prevuelo (inmovilidad, 1g, el «arriba» coincide con el montaje), calibración del montaje (`calibrateOrientation()`), escala, giro, signos aeronáuticos, errores del bus |
| `imu/AttitudeEstimator.h` | Filtro complementario de alabeo/cabeceo, integral de guiñada |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (el chip se distingue por WHO_AM_I): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **En el banco** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, filtro UI de 50 Hz. No probado en hardware |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B o SPI. No probado en hardware |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1.6 kHz, filtro paso bajo mediante los registros indirectos IPREG; I2C 0x68/0x69 o SPI. No probado en hardware |
| `baro/BarometerBase.h` | Lo común de los barómetros: consulta solo de muestras nuevas, altitud, velocidad vertical mediante un filtro paso bajo, calibración de la base, errores |
| `baro/BMP388_Sensor.h` | BMP388 por I2C o SPI (con el byte ficticio de SPI), compensación de Bosch, lectura según el indicador de dato listo. **En el banco (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, compensación de Bosch §8.1. No probado en hardware |
| `baro/SPL06_Sensor.h` | SPL06-001: coeficientes y fórmulas de la hoja de datos, 32 Hz ×16; I2C 0x76/0x77 o SPI. No probado en hardware |
| `baro/BMP581_Sensor.h` | BMP581: la secuencia de BMP5_SensorAPI, 16×/2×, IIR; I2C 0x46/0x47 o SPI; sirve tanto de barómetro principal (conjunto de banco por defecto) como de tubo de Pitot. No probado en hardware |
| `mag/MagnetometerBase.h` | Lo común de las brújulas: consulta a 50 Hz, calibración hard-iron en NVS, giro de ejes, rumbo, errores |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **En el banco** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. No probado en hardware |
| `gps/UbloxM10_Gps.h` | u-blox M10: configuración con CFG-VALSET (115200 baudios, 10 Hz, NAV-PVT, sin NMEA), análisis de NAV-PVT. No conectado en el banco |
| `airspeed/AirspeedSensor.h` | La interfaz del sensor de velocidad aerodinámica: diferencial de presión, IAS, TAS, densidad |
| `airspeed/PitotDualBaroAirspeed.h` | El tubo de Pitot casero: un BMP581 en el tubo + un barómetro en el fuselaje; cero en tierra, filtro paso bajo, densidad a partir de la presión estática, detección de averías |

### `telemetry/` y la aplicación

| Archivo | De qué se encarga |
|---|---|
| `DebugLogger.h` | Registro por canales (`LogSettings.h`): cada uno tiene su propia línea, su propia tolerancia al rebote y su modo; calla mientras el menú está abierto |
| `DebugConsole.h` | Un menú de texto en el monitor del puerto (`h`) y atajos de teclado (`l`/espacio/`s`/`i`/`o`/`m`/`p`/`b`); guarda los ajustes del registro en NVS al salir del menú y solo sin ARM |
| `LogSettings.h` | Los canales del registro (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) y sus modos: desactivado / al cambiar / continuo; se guarda en NVS |
| `WebDebugServer.h` | El punto de acceso, las rutas, el JSON `/api/status`, el buzón de comandos; tiene su propia tarea en el núcleo 0 |
| `WebDashboardPage.h` | El HTML/JS del panel en un solo literal; las filas de canales, salidas y sensores las construye el navegador a partir del JSON |
| `OledDisplay.h` | SSD1306 mediante U8g2 sobre `II2CBus`, con su propia tarea (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 para QGroundControl / Mission Planner: tramas, flujos, parámetros PID, cambio de modo desde tierra |
| `LoopStats.h` | Frecuencia, tiempo medio y peor tiempo de ciclo por segundo (OLED) y el peor desde la última lectura (`takePeakUs()`, línea SYS) |
| `src/main.cpp` | ESP32: creación de los objetos, `setup()`, `loop()` con `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: los mismos objetos, MAVLink, la caja negra en la tarjeta SD, las tareas `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: pines y relojes de SDMMC1 para `HAL_SD_Init`; la tecla `D` de la consola: reinicio en el cargador de arranque USB DFU |

---

## Convenio de signos: de la IMU al servo

Un único sistema de signos para toda la cadena: así el stick y el piloto
automático mueven las superficies siempre en el mismo sentido, y el sentido
de cada servo se fija en un solo lugar.

**1. Ejes del sensor → ejes del avión.** `ImuSensorBase` gira los ejes del
chip con la matriz `ImuOrientation` (body = R · chip) hasta los ejes del
avión: X hacia el morro, Y a la izquierda, Z hacia arriba. La matriz se
obtiene:

- de la **calibración del montaje** (el comando `o`, se guarda en NVS): la
  placa puede estar colocada de cualquier manera. Tres posturas: «nivelado»
  da el eje Z (y el horizonte: el desvío de cero del acelerómetro queda
  incluido en él), «morro arriba» da el eje X (la parte de «arriba»
  perpendicular a Z), «ala derecha abajo» da el eje Y. El morro del paso 2 y
  el morro del paso 3 (Y × Z) deben coincidir con una precisión de ~25°; si
  no, el piloto inclinó hacia otro lado y la calibración se rechaza; el
  resultado es la media de las dos estimaciones. Verificado con 300 montajes
  aleatorios (`test/test_imu_orientation`, error < 0.1°);
- en caso contrario, de `Config::IMU_ROTATION_CW_DEG` (la placa con el chip
  hacia arriba; el valor indica hacia dónde mira el eje X del *chip* si el
  morro está a las «12 en punto»), y el horizonte es la postura al encender.

En cada calibración del giroscopio (al encender, `i`) hay una **comprobación
prevuelo**: ruido del giroscopio < 0.5 °/s (inmovilidad; en reposo ~0.08),
|a| ≈ 1g, el «arriba» a menos de 45° del guardado (la placa no se ha
movido). Si no la supera, `ImuSensor::getPreflightProblem()` ≠ nullptr:
`ArmingManager` no arma los modos con estabilización y
`Autopilot::imuReady()` = false (correcciones nulas en todos los modos,
incluido el planeo por pérdida de enlace).

> En el GY-521 actual (un clon con MPU6500) el chip está soldado girado 90°
> respecto a las flechas impresas: la flecha X de la serigrafía = el eje Y del
> chip. Por eso, sin calibración del montaje, `IMU_ROTATION_CW_DEG = 90`.
> Comprobación tras cualquier cambio: morro arriba → P crece hacia el
> positivo, ala derecha abajo → R hacia el positivo.

**2. Ángulos y velocidades (`ImuData`): signos aeronáuticos:**

| Magnitud | «+» significa |
|---|---|
| `roll`, `gyroX` | ala derecha abajo |
| `pitch`, `gyroY` | morro arriba |
| `yaw`, `gyroZ` | morro a la derecha (en sentido horario visto desde arriba) |

**3. La orden (`ControlCommand`, µs de deflexión, ±500 = recorrido completo):**

| Campo | «+» significa | Desde el stick |
|---|---|---|
| `roll` | alabeo a la derecha (alerón derecho arriba, izquierdo abajo) | CH1: 2000 = a la derecha |
| `pitch` | morro arriba (timón de profundidad arriba) | CH2 con el signo contrario: 2000 = hacia delante = morro abajo |
| `yaw` | morro a la derecha (timón de dirección y rueda a la derecha) | CH4: 2000 = a la derecha |
| `flaps` | flaps abajo (ambos alerones abajo) | SwB (CH6): 0 o `FLAPS_DEPLOYED_US`, de forma suave en `FLAPS_TRANSITION_MS` |

El PID calcula `error = objetivo − real`: alabeo a la derecha (roll > 0) →
orden de alabeo negativa → el avión se nivela. Las correcciones del piloto
automático se suman a la orden de los sticks **antes** del mezclador, con los
mismos signos.

**4. Orden → PWM.** `ControlMixer::mix()` calcula la deflexión del borde de
salida de cada superficie (alerones: abajo = «+», izquierdo = `flaps + roll`,
derecho = `flaps − roll`; timón de profundidad: arriba = «+»; timón de
dirección: a la derecha = «+») y la convierte en PWM `1500 ± deflexión`,
invirtiendo el signo en los servos con `Config::*_REVERSED = true`. Los
valores por defecto reproducen el comportamiento anterior del firmware para
los sticks. La comprobación en el avión montado está en la lista de
comprobación prevuelo de [`PILOT_GUIDE.md`](PILOT_GUIDE.md). La inversión hay
que cambiarla en `Config.h`, **no en el mando**; de lo contrario el stick y
el piloto automático se desincronizarán.

---

## Mapa de canales RC, ARM y failsafe

La fuente es `include/config/Channels.h`. Emisora FS-i6 (10 canales, modo 2) +
receptor FS-iA6B, iBUS 115200.

| Canal | Mando de la emisora | Nombre | Función |
|---|---|---|---|
| CH1 | stick derecho ←→ | `AILERON` | Alabeo |
| CH2 | stick derecho ↑↓ | `ELEVATOR` | Cabeceo |
| CH3 | stick izquierdo ↑↓ | `THROTTLE` | Acelerador, recorrido completo; < 950 = failsafe del receptor |
| CH4 | stick izquierdo ←→ | `RUDDER` | Timón de dirección + rueda de dirección (un solo servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (en la FS-i6 es el interruptor abajo, hacia uno) |
| CH6 | SwB | `SWB` | por defecto, los flaps (≥ 1750: extendidos) |
| CH7 | SwC (3 posiciones) | `SWC` | por defecto, el modo: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | por defecto, RTH |
| CH9 | VrA | `VRA` | por defecto, la intensidad de la estabilización |
| CH10 | VrB | `VRB` | por defecto, la velocidad de crucero |

CH6–CH10 se asignan con una sola línea en `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#asignar-una-función-con-una-sola-línea)).

**ARM** (`ArmingManager`): el interruptor pasa de OFF a ON, el acelerador está
por debajo de `THROTTLE_LOW_US` y se han superado las comprobaciones de
sensores del modo actual. De lo contrario, lo rechaza e indica el motivo por
Serial; hace falta un nuevo ciclo OFF→ON. Encender la placa con el
interruptor ya en ON no arma. OFF: DISARM inmediato. Mientras no esté armado,
el acelerador hacia el ESC se fuerza a `PWM_MIN`.

**Pérdida de enlace** (`IBusReceiver::isSignalLost()`):

1. No hay tramas durante más de `RX_TIMEOUT_US` (500 ms): se ha roto un cable
   o se ha perdido la alimentación del receptor. Hasta la primera trama tras
   el encendido el enlace también se considera perdido: los valores de canal
   por defecto (todos a 1500) no se toman por órdenes de la emisora.
2. Acelerador < `RX_FAILSAFE_THROTTLE_US` (950): el failsafe configurado en
   la emisora. **El FS-iA6B no deja de enviar tramas cuando se pierde la
   emisora**, sino que repite los últimos valores (comprobado en el banco),
   así que sin configurar el failsafe en la emisora la pérdida de enlace no
   se detecta. La configuración está en `PILOT_GUIDE.md`.

Qué ocurre al perder el enlace (`FlightController::applyLinkLoss()`):

- **el aparato está armado y hay GPS y punto de origen** (`FAILSAFE_RTH`):
  **regreso a casa** con motor y círculos sobre el punto de origen; en el
  OLED, `FSRTH`; en el registro, `FAILSAFE_RTH`;
- **el aparato está armado y no hay GPS**: **planeo**, con el motor a
  `FAILSAFE_THROTTLE`: `Autopilot`, en cualquier modo, incluso en MANUAL,
  mantiene el alabeo `FAILSAFE_GLIDE_ROLL_DEG` (0, recto; 10–20°, un círculo
  sobre el piloto) y el cabeceo `FAILSAFE_GLIDE_PITCH_DEG` (−3°, para no
  perder velocidad sin motor), con los flaps recogidos; en el OLED, `GLIDE`;
  en el registro, el modo `FAILSAFE_GLIDE`;
- **no armado** (en tierra) o la IMU no responde: las superficies a neutro;
- el modo y las funciones no se cambian con los interruptores, los sensores
  siguen leyéndose. El ARM no se anula: al restablecerse el enlace el avión
  vuelve a obedecer los sticks y el modo elegido (el despegue automático y el
  lanzamiento a mano solo se reinician desde cero).

---

## Datos de los sensores

Las estructuras están en `include/sensors/SensorInterface.h`.

### `ImuData`

| Campo | Unidad | Significado |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Velocidades angulares en los ejes del avión, signos aeronáuticos (véase más arriba) |
| `accelX`, `accelY`, `accelZ` | g | Aceleración en los ejes del avión: X hacia el morro, Y a la izquierda, Z hacia arriba |
| `roll`, `pitch` | ° | Filtro complementario (α = 0.98, τ ≈ 0.1 s); arrancan directamente con el ángulo del acelerómetro |
| `yaw` | ° | Integral del giroscopio, se desvía lentamente; el valor inicial es el rumbo de la brújula |
| `temperature` | °C | Temperatura del chip (fórmula para el MPU6050 o el MPU6500) |
| `timestamp` | µs | `micros()` en el momento de la lectura |

Calibración de la IMU (en cada arranque y con el comando `i`): 2 s inmóvil,
giroscopio → desvío de cero, acelerómetro → **la posición actual pasa a ser
el horizonte**.

### `BarometerData`

| Campo | Unidad | Significado |
|---|---|---|
| `pressure` | Pa | Presión |
| `temperature` | °C | Temperatura del sensor |
| `altitude` | m | Altitud **respecto al punto de calibración** (al arrancar); fórmula `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Derivada de la altitud sobre las muestras reales (50 Hz) mediante un filtro paso bajo con τ = 0.5 s |
| `timestamp` | µs | Momento de la última muestra nueva |

### `MagData`

| Campo | Unidad | Significado |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | El campo tras la calibración hard-iron, en los ejes del avión (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, sin compensación de inclinación; el sentido de la lectura aún no se ha comprobado en el avión montado |
| `timestamp` | µs | Momento de la lectura (50 Hz) |

### `GpsData`

| Campo | Unidad | Significado |
|---|---|---|
| `latitude`, `longitude` | ° | De UBX-NAV-PVT |
| `altitude` | m | Sobre el nivel del mar (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Velocidad sobre el suelo y rumbo sobre el suelo |
| `numSatellites`, `fixType` | — | 0 = sin posición, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | Estimaciones de precisión del módulo |

**Qué significa `isAvailable()`.** Para los sensores I2C: el sensor respondió
en `begin()` **y** las últimas lecturas no fallan de forma consecutiva (MPU:
~0.1 s; barómetro y brújula: ~0.5 s sin respuesta). Si una lectura falla, los
datos no se sobrescriben con basura: se conservan los anteriores y crece el
contador de errores (se ve con el comando `s`). Para el GPS: al menos un
NAV-PVT válido, y que el último no sea más antiguo que `GPS_TIMEOUT_US`.

**Si no hay sensor** (`nullptr` o `isAvailable() == false`), `Autopilot` no
da correcciones y el avión se pilota como en MANUAL. `main.cpp` solo calibra
los sensores que respondieron.

---

## Desglose de FlightController::update()

Se llama desde `loop()` cada 2 ms. El orden es la prioridad:

1. **`receiver.update()`**: análisis de los bytes de iBUS acumulados.
2. **Interruptores**: `switches->update(rc)`, solo con el enlace vivo (en una
   trama de failsafe los canales no reflejan los interruptores): el modo
   (solo al cambiar), las funciones, los potenciómetros.
3. **Acelerador del piloto**: `throttle.update(rc, receiverFailsafe)`.
4. **Sticks**: `mixer.fromSticks(rc)` × `Knob::RATES`; flaps:
   `mixer.updateFlaps(target)` (freno, interruptor, potenciómetro; sin
   enlace, 0).
5. **Sensores y piloto automático**: `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **siempre**, incluso sin enlace: los filtros de ángulos no deben
   congelarse. Mientras no esté armado, el PID funciona (las superficies
   responden a la inclinación, cómodo sobre la mesa), pero el integrador se
   mantiene en cero. Sin enlace y armado: failsafe RTH o planeo.
6. **Zumbador**: `Beeper`.
7. **Pérdida de enlace**: `applyLinkLoss()`: si está armado, las superficies
   y el acelerador siguen la orden de failsafe del piloto automático; si no,
   neutro y motor apagado; `return`. Prioridad absoluta sobre todo lo que
   sigue.
8. **ARM**: `arming.update(rc, false)`.
9. **Orden**: `autopilot->getCommand()`: en los modos con estabilización el
   stick es el ángulo deseado y el piloto automático emite las órdenes finales
   a las superficies.
10. **Mezclador**: `mixer.mix(command)` → PWM de los alerones (flaps +
    alabeo), del timón de profundidad y del timón de dirección, teniendo en
    cuenta la inversión.
11. **Acelerador**: `autopilot->applyThrottle(pilotThrottle)`: el acelerador
    del piloto, el del piloto automático o el máximo de los dos (despegue
    automático). Después, si no está armado o hay `MOTOR_KILL`, se fuerza a
    `PWM_MIN`. Esta comprobación va la última para que ningún modo pueda
    colar el acelerador sin pasar por el ARM.
12. **AUX**: carga (`PAYLOAD_DROP`) y cámara (`CAMERA_TILT`, `CAMERA_STAB`).
13. **`outputs.write(output)`**: PWM en las 7 salidas.

---

## API HTTP del dashboard web

La implementación está en `include/telemetry/WebDebugServer.h`. El punto de
acceso: SSID `OpenPlane-Debug`, contraseña `12345678`, dirección
`http://192.168.4.1`.

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

- `attached`: el objeto existe en la compilación; `available`: el sensor
  realmente responde. Los campos de datos se añaden **solo** con
  `available: true`.
- `outputs.*.attached`: el MCU ha reservado un canal LEDC y un pin; si hay un
  servo físico conectado no se puede saber por software (para comprobar el
  pulso, use la consola, comando `p`).
- `rollCorr`/`pitchCorr`: la orden final del piloto automático menos los
  sticks, en µs. `throttleCorr`: el acelerador del piloto automático, en %
  (0 mientras el acelerador lo lleva el piloto).
- `nav`: navegación: el punto de origen, la distancia y la marcación hacia él,
  el rumbo y el rumbo objetivo, la velocidad para la navegación (tubo de
  Pitot / GPS), la geovalla, la pérdida de sustentación; `features`: las
  funciones de los interruptores que están activas.

### `POST /api/setmode`

`{ "mode": 1 }`: el número de `AutopilotMode`: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. El modo se mantiene hasta
que el piloto accione el interruptor de modo.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }`: cualquiera de los campos
`kpRoll`, `kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch`; los omitidos
conservan su valor anterior.

Ambas órdenes las aplica el ciclo de vuelo en el siguiente ciclo (véanse las
[tareas de FreeRTOS](#tareas-de-freertos-y-el-lazo-de-control)).

### `GET /`

El dashboard HTML: barras de los 10 canales, ARM/enlace, las salidas, los
sensores, botones de modo, el formulario del PID. Consulta `/api/status`
cada 200 ms.

---

## Consola y diagnóstico

El monitor del puerto: 115200, conector «COM». La implementación es
`DebugConsole` y `DebugLogger` ([referencia](reference/telemetry.md)). Las
teclas actúan al instante, no hace falta Enter; las calibraciones y `p`
bloquean el ciclo y por eso solo están disponibles sin ARM.

| Tecla | Qué hace |
|---|---|
| `h` / `?` | Menú principal |
| `l` | El menú «qué mostrar en el registro» (canales, modos, periodo) |
| espacio | Pausar el registro / reanudar |
| `s` | `printStatus()` de todos los sensores: datos, contadores de errores del bus, calibraciones, la comprobación prevuelo |
| `i` | Calibración del giroscopio + comprobación prevuelo (2 s inmóvil) |
| `o` | Calibración del montaje de la IMU con tres posturas, se guarda en NVS |
| `m` | Calibración de la brújula (15 s girando), se guarda en NVS |
| `p` | Autocomprobación de las salidas: el pulso real en cada pin frente al esperado |

El registro se divide en canales (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`,
`MAG`, `GPS`, `IMU`, `SYS`), cada uno con el modo «desactivado / al cambiar /
continuo»; los ajustes se guardan en NVS y se escriben al cerrar el menú, solo
sin ARM. Por defecto están activados `STAT` (al cambiar) y `SYS` (una vez
cada 10 s):

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (el peor en 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

Los formatos de todos los canales están en la [referencia](reference/telemetry.md#debuglogger).

---

## Elección de la placa y distribución de pines

| Comando | `board` | Macro | Estado |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **Principal, la predeterminada.** Probada en el banco con todos los sensores |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | El prototipo antiguo, voló con control manual |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | Para el banco, la distribución de pines no está probada en hardware |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: el firmware completo + MAVLink + caja negra en SD; probada en una placa desnuda ([más abajo](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | Lo mismo en la DevEBox H743: la consola es USB CDC y el firmware se carga por DFU |

| Función | ESP32-S3 (banco) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Alerón izquierdo / derecho | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Timón de profundidad / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Timón de dirección | GPIO18 | — (sin pin) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| I2C de sensores SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| I2C del OLED SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / ninguno (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → conector «COM» | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** los GPIO33–37 los ocupa la PSRAM octal, los 26–32 la
  flash, los 19/20 el USB, los 43/44 el Serial, el 48 es el LED RGB; los
  0/3/45/46 son pines de strapping.
- **ESP32-C3:** los alerones en GPIO4/5 están intercambiados respecto a la
  S3. No hay pines suficientes para el conjunto completo: el CS del BMP388 y
  el RX del GPS están en pines de strapping, el GPS no tiene TX (solo
  recepción, sin UBX-CFG). Los detalles, en `Config.h`.

### STM32H743

La STM32H743VIT6 (Cortex-M7 a 480 MHz, 2 MB de flash, 1 MB de RAM) ejecuta el
**firmware completo**: los mismos sensores, piloto automático, interruptores,
consola y pantalla que en la ESP32-S3, más la telemetría MAVLink y una caja
negra en la tarjeta SD. Compila, pasa cppcheck y todas las pruebas nativas del
código común. En hardware se ha probado la **placa DevEBox H743 sin sensores**:
arranque, consola por USB, tarjeta SD, caja negra
([TESTING.md](TESTING.md#pruebas-en-la-placa-stm32)), y además iBUS, ARM y PWM a
los servos y al motor: control desde la emisora en modo manual (en vídeo). Los
sensores en la STM32 aún esperan un banco. La placa principal de vuelo es la
ESP32-S3.

- **HAL**: `include/hal/stm32/`: `Stm32Board` (la misma API que `Esp32Board`,
  más `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (PWM por hardware con `HardwareTimer`, un temporizador
  para varias salidas). Con detalle: [reference/hal.md](reference/hal.md#implementación-para-el-stm32h743).
- **Ajustes y calibraciones**: no en NVS, sino en un `KeyValueStore` en el
  último sector de la flash (`include/storage/`,
  `hal/stm32/Stm32FlashStorage.h`). El código del proyecto sigue escribiendo
  `#include <Preferences.h>`: en el env `stm32h743` el directorio
  `include/hal/stm32/compat/` está en `-I`, y allí hay un `Preferences` con la
  misma API. La imagen lleva CRC32: una imagen dañada (se cortó la corriente
  durante el borrado) se lee como vacía. La escritura en flash se hace en una
  tarea en segundo plano: borrar un sector de 128 KB lleva segundos, pero el
  sector está en el banco 2 y el código se ejecuta desde el banco 1, y la tarea
  de vuelo expulsa a la de segundo plano sin detenerse.
- **Tareas**: FreeRTOS de la biblioteca STM32duino FreeRTOS, un solo núcleo,
  expropiación por prioridad (`hal/Rtos.h`): `flight` (5), el ciclo de vuelo,
  MAVLink, registro, consola; `oled` (1) y `storage` (1), en segundo plano;
  `bbox` (2), la escritura de la caja negra en la tarjeta SD.
- **La caja negra en la tarjeta SD**: SDMMC1, 4 bits, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, los pines en `src/stm32/sd_msp.cpp`). La tarjeta
  sigue siendo una FAT32 normal: en ella hay un archivo `BLACKBOX.BIN` creado
  de antemano, el firmware escribe bloques en bruto dentro de él y no toca el
  sistema de archivos en sí (`storage/Fat32File.h` es de solo lectura).
  Preparación de la tarjeta y descarga: [BLACKBOX.md](BLACKBOX.md#tarjeta-sd-stm32h743).
- **Telemetría**: MAVLink 2 en la UART4 (`telemetry/MavlinkTelemetry.h`) en
  lugar del dashboard Wi-Fi: QGroundControl / Mission Planner, cambio de modo
  y de PID desde tierra. Con detalle:
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#estación-de-tierra-panel-wi-fi-y-mavlink).
- **Distribución de pines**: el bloque `BOARD_STM32H743` de `Config.h`, con
  pines elegidos entre los libres de la WeAct MiniSTM32H743VITx y contrastados
  con las tablas de STM32duino:

| Función | STM32H743 | Periférico |
|---|---|---|
| Alerón izquierdo / derecho | PA0 / PA1 | TIM2_CH1 / CH2 |
| Timón de profundidad / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Timón de dirección | PD14 | TIM4_CH3 |
| AUX1 (carga) / AUX2 (cámara) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX de reserva) | PE7 (PE8) | UART7 |
| I2C de sensores SDA / SCL | PB11 / PB10 | I2C2 |
| I2C del OLED SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / barómetro | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Radiomódem MAVLink RX / TX | PD0 / PD1 | UART4 |
| Zumbador | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)**: el env `stm32h743-devebox`: el mismo código, su
  propia variante del núcleo, la consola por USB-C como puerto COM virtual
  (CDC); no hace falta un USB-UART. La primera carga del firmware se hace por
  USB con el cargador de arranque integrado (DFU):
  1. Windows: instalar una sola vez el controlador WinUSB para «STM32
     BOOTLOADER» ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB →
     Install Driver).
  2. Conectar con un cable el pin **BT0** (BOOT0) a **3V3**, pulsar y soltar
     **RST**: la placa queda en modo DFU (la DevEBox no tiene botón BOOT0).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. El cable de BT0 se puede quitar: el firmware arranca solo.

  Después ya no hace falta el cable: la tecla **`D`** de la consola (desde
  cualquier menú, no con ARM) reinicia la placa en el cargador de arranque:
  una marca en la RAM → reinicio → salto a la memoria del sistema antes de
  configurar los relojes (`src/stm32/bootloader.cpp`). Saltar directamente
  desde el firmware en ejecución se cuelga en la H7 (comprobado en la placa),
  por eso son dos pasos. Hace falta la consola abierta (USB CDC); si la placa
  no responde, RST con el cable de BT0 puesto.
- **Punto de entrada**: `src/stm32/main.cpp` (excluido de las compilaciones
  para ESP32 mediante `build_src_filter`). Los objetos son los mismos que en
  `src/main.cpp`; en lugar de `loop()` hay tareas, y `vTaskStartScheduler()`
  está al final de `setup()`.
- **Primer encendido de la placa:** `pio run -e stm32h743 -t upload`
  (ST-Link), el monitor en LPUART1 mediante un USB-UART; `b`: si los sensores
  se ven en los buses, `s`: estado de los sensores, `p`: pulsos en las salidas
  (quite la hélice), y después la emisora y QGroundControl a través del
  radiomódem.

---

## Cómo añadir un sensor nuevo

### A) Otro chip de una categoría existente (IMU, barómetro, brújula)

Lo común ya está escrito en las clases base, así que el controlador del chip
resulta pequeño:

1. Cree `include/sensors/<category>/<Name>_Sensor.h` y herede de
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. El constructor
   recibe un `IRegisterDevice&`: el controlador no sabe si es I2C o SPI.
2. Implemente:
   - `begin()`: `device.begin()`, comprobar el ID del chip, escribir los
     registros, llamar a `setAvailable(true/false)`;
   - IMU: `readSample()` (acelerómetro/giroscopio/temperatura en bruto en los
     ejes del chip), `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - barómetro: `isNewSampleReady()` (un indicador de dato listo o
     simplemente `true`) y `readSample()` (presión en Pa, temperatura en °C);
     el periodo de consulta está en el constructor de la base;
   - brújula: `readRaw()` (X/Y/Z en los ejes del chip) y
     `lsbPerMicroTesla()`; el nombre del espacio de NVS para la calibración
     está en el constructor de la base.
3. Si por SPI el chip necesita un byte ficticio antes de los datos o una
   frecuencia especial, añada una fábrica estática `spiDevice(bus, cs)`, como
   la de `BMP388_Sensor`.
4. Una rama en `SensorSelection.h`: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` y `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` o la fábrica de SPI). Al cambiar
   de sensor no se toca `main.cpp`.
5. Compruebe la compilación con el sensor nuevo sin editar el archivo, con una
   bandera: `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`,
   luego los tres entornos y después en el hardware.

### B) Una categoría nueva

1. La estructura de datos y la interfaz van en `SensorInterface.h`, siguiendo
   el modelo de `GpsSensor`/`GpsData`.
2. Si la categoría tiene lógica común (filtros, calibración), una clase base
   según el modelo de `BarometerBase`.
3. Un puntero que admita nulo en el constructor de `Autopilot` (sin sensor, sin
   efectos, en lugar de un fallo) y campos en `GET /api/status` con el par
   `attached`/`available`.

### Un bus o periférico nuevo

Una interfaz nueva en `include/hal/`, la implementación en
`include/hal/esp32/` y en `include/hal/stm32/`, y acceso a través de `IBoard`.

---

## Cómo añadir un modo nuevo del piloto automático

1. Un valor en `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, antes de
   `MODE_COUNT`), el nombre y un nombre corto (hasta 5 caracteres, para el
   OLED) en `AutopilotNames::mode()` / `modeShort()`.
2. Un manejador `run<Mode>()` y una rama en `Autopilot::runMode()`; los
   objetivos iniciales (rumbo, altitud, centro de los círculos) van en
   `initializeMode()`. El modo establece `desiredRoll`/`desiredPitch` y llama a
   `stabilizeOrManual()` (sin IMU las superficies quedan en manos del piloto) o
   a `stabilizeOrNeutral()` (sin IMU, neutro). Sin el sensor necesario, un
   comportamiento seguro y no un fallo. El integrador se acumula solo con
   `armed`.
3. Acelerador: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) y
   `autoThrottlePct`, o `autoThrottle()`: el acelerador de crucero desde el
   potenciómetro o desde el tubo de Pitot. `FlightController` no cambia.
4. En la emisora, una sola línea en `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). El dashboard y MAVLink recogen el
   modo por su número; para MAVLink, el modo más cercano de ArduPlane en
   `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. Si el modo necesita sensores para el ARM, `ArmingManager`.
6. Pruebas: la reacción a cada sensor, en `test/native/test_autopilot_modes`;
   el vuelo en lazo cerrado, un escenario en `test/native/test_sim` (el modelo
   del avión `helpers/PlaneSim.h`, el banco `helpers/SimHarness.h`). Después, la
   mesa sin hélice: las superficies deben responder a la inclinación en el
   sentido de nivelar.
7. Una sección en [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Realimentación (base preparada, no conectada)

`include/autopilot/feedback/` es el siguiente paso del piloto automático.
**Ni `FlightController`, ni `Autopilot`, ni `main.cpp` incluyen estos
archivos:** todavía no hay un prototipo para pruebas de vuelo, y el firmware
funciona sin ellos. Se comprueban con una simulación en lazo cerrado
(`test/test_feedback/`) directamente en la placa.

### Para qué

Hoy `Autopilot` es un PID sobre el ángulo: error × ganancia = superficie. No
sabe qué resultó de eso en el avión, y las ganancias son correctas solo para
una velocidad: a baja velocidad la superficie es más débil y el PID corrige
de menos; a alta velocidad, de más. La realimentación cierra el lazo sobre
**la respuesta del avión**:

- se desvió la superficie, pero el avión gira más despacio de lo necesario:
  añadir más, hasta que llegue;
- cuánta superficie hace falta se mide en vuelo y se recalcula según la
  velocidad;
- el avión gira en el sentido contrario: el signo está cambiado, invertirlo y
  comprobar;
- se niveló el ángulo, pero la velocidad cae: gas y morro abajo, hasta que el
  avión deje de pararse;
- despegue y aterrizaje: por fases, según lo que muestren los sensores.

### Módulos

| Archivo | Qué hace |
|---|---|
| `FlightSnapshot.h` | Todo lo que la realimentación sabe del avión en un ciclo. Es la única entrada: los módulos no leen los sensores ni el RC directamente, así que pueden ejecutarse sobre una simulación y sobre registros |
| `FeedbackOutput.h` | La salida de un ciclo: deflexiones de las superficies por eje, si el eje está activado, el signo del eje, el acelerador (fijar / no menos de), el motivo |
| `FeedbackConfig.h` | Todas las constantes (al conectarlo pasarán a `Config.h`) |
| `SpeedEstimator.h` | La velocidad (tubo de Pitot > GPS) y la aceleración longitudinal a partir de la IMU: `dV/dt = g·(ax − sin θ)`: se ve que «la velocidad cae» incluso sin sensor de velocidad |
| `AirborneDetector.h` | En el aire / en tierra: aprender, acumular la integral y buscar la pérdida de sustentación solo tiene sentido en vuelo |
| `ControlEffectivenessEstimator.h` | Para cada eje aprende el modelo `ε = b·u(t−delay) + a·ω + c` por mínimos cuadrados recursivos |
| `AdaptiveRateController.h` | Una cascada ángulo → velocidad angular → aceleración angular → superficie a través del modelo aprendido |
| `StallGuard.h` | Protección contra la pérdida de velocidad y la pérdida de sustentación |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Despegue (desde una pista o a mano) y aterrizaje por fases, según los sensores |
| `FeedbackSupervisor.h` | Todo junto: el orden dentro de un ciclo, las prioridades, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, el plan de conexión |
| `FeedbackModules.h` | Un único include para todo |

### Cómo funciona

**Efectividad de las superficies.** El modelo del eje: aceleración angular
`ε = b·u + a·ω + c`. `b` es cuántos °/s² da 1 µs de superficie (el signo es el
sentido de la respuesta), `a` es el amortiguamiento (el aire frena el giro; sin
este término la estimación de `b` se iría a cero con un giro estacionario), `c`
es un momento constante (centrado, compensador, hélice). La fuerza de una
superficie ∝ ρV², por eso `b` se aprende a una velocidad de referencia y se
multiplica por `(V/Vref)²`: el avión se acelera y la superficie «se vuelve más
fuerte» al instante, sin reaprender. La velocidad indicada del tubo de Pitot
ya contiene la densidad del aire, así que la altitud queda tenida en cuenta por
sí sola; sin sensor de velocidad la escala es 1 y `b` se aprende directamente.

Los datos se toman en intervalos de 20 ms: la aceleración media en un
intervalo es la diferencia del giroscopio en los extremos / la duración, y le
corresponden la superficie media y la velocidad angular media del mismo
intervalo (la superficie, con el retardo `RESPONSE_DELAY_MS`). Después los dos
lados de la ecuación pasan por el mismo filtro paso bajo de 2 Hz: la relación
no cambia, mientras que se eliminan las altas frecuencias, donde el modelo de
«retardo puro» miente por la inercia del servo. Solo se puede aprender en el
aire y solo cuando se «agita» la superficie (una amplitud ≥
`MIN_EXCITATION_US` en ~0.3 s); los sticks del piloto también agitan, así que
la estimación aprende también en MANUAL.

**El regulador.** Tres etapas, eje por eje:

```
ω* = ANGLE_GAIN · (target − angle)              "el morro está 10° abajo — subirlo a 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "gira más despacio de lo necesario — corregir"
surface = (ε* − a·ω − c) / b                    a través del modelo aprendido
```

La integral `I` se guarda en °/s y no en µs de superficie, por eso sigue siendo
correcta cuando cambia la estimación de `b`. En tierra la integral está
congelada (salvo el rumbo en la carrera de despegue / de aterrizaje) y, con la
superficie en el tope, no se acumula hacia el tope. Se tiene en cuenta el
viraje coordinado (si se conoce la velocidad): en alabeo hacen falta un
cabeceo `g·sin φ·tg φ / V` y una guiñada `g·sin φ / V`.

**Signos de los ejes: solo en tierra.** En vuelo los ejes no se desactivan ni
se invierten: el montaje de la IMU lo determinan la calibración `o` y la
comprobación al encender, y los sentidos de las superficies, la comprobación
prevuelo del piloto. Los indicios indirectos en el aire (una pérdida, una
barrena, las figuras, las ráfagas) pueden engañar, y un eje desactivado o
invertido en un momento así cuesta el avión. Si la estimación de `b` de un eje
es claramente negativa, es solo una advertencia en `reason` («¿responde a la
superficie al revés? comprobar en tierra»); una estimación negativa no llega al
regulador: el eje trabaja con el modelo a priori.

**Protección contra la pérdida de sustentación.** Dos niveles. *LowEnergy*: la
velocidad cae rápido con el morro levantado, o está cerca de la pérdida
(< 1.25·Vs), o el timón de profundidad ha perdido efectividad: gas ≥ 80 %,
cabeceo ≤ 5°. *Stall*: la velocidad está por debajo de la de pérdida, el morro
cae contra el timón de profundidad, el ala se cae contra los alerones con poca
energía: gas a fondo, morro abajo, alabeo ≤ 10°, alerones limitados (un alerón
grande desprende la punta del ala). Las medidas se retiran con histéresis
(velocidad ≥ 1.5·Vs). Al perder el enlace no se toca el gas, y muy cerca del
suelo (enderezamiento, carrera de aterrizaje) la protección está desactivada:
el aterrizaje es en sí una pérdida controlada.

**Despegue.** `WaitThrottle` (el motor está parado) → el piloto dio gas ≥ 50 %
→ `GroundRoll` (gas a fondo, alas niveladas, el rumbo lo mantienen el timón de
dirección y la rueda, el timón de profundidad libre) → velocidad de
despegue, o un tiempo límite sin sensor de velocidad → `Climb` (12°, gas a
fondo) → altitud 30 m → `Complete`. A mano (`TAKEOFF_HAND_LAUNCH`), en lugar de
la carrera, `WaitLaunch`: el motor arranca solo después del lanzamiento
(aceleración longitudinal ≥ 1g). Gas retirado antes del despegue: cancelación.

**Aterrizaje.** `Approach` (gas 25 %, descenso de 1 m/s: el cabeceo a partir
del error de velocidad vertical, el alabeo del piloto ≤ 20°) → altitud 2 m →
`Flare` (gas 0, el descenso se reduce hasta 0.3 m/s con la misma regla) →
impacto detectado por el acelerómetro o «bajo y sin girar» → `Rollout` (rumbo
con la rueda) → `Complete`. Gas del piloto ≥ 80 %: motor y al aire. Para el
enderezamiento hace falta un telémetro: el barómetro se equivoca en un metro.

**Prioridades** (`FeedbackSupervisor`): no armado > protección contra la
pérdida de sustentación > despegue/aterrizaje > objetivos del modo. Al perder
el enlace se cancelan las fases y la estabilización cumple los objetivos de
planeo del failsafe.

### Simulación

`test/test_feedback/test_main.cpp` (en un PC: `pio test -e native -f test_feedback`): un modelo de avión (ejes independientes,
retardo e inercia del servo, efectividad de las superficies ∝ V², amortiguamiento ∝ V, momentos
constantes, sustentación a través del ángulo de ataque según la velocidad, pérdida de sustentación, tren de aterrizaje con
rueda de dirección) y 10 escenarios:

| Escenario | Qué se comprueba |
|---|---|
| Salida de un alabeo de 30° / cabeceo de −15° con un momento constante | Nivelación y «seguir corrigiendo»: la integral encuentra sola la compensación |
| Oscilación de ±15° a 14 y 20 m/s, sin sensor de velocidad | La estimación de `b` converge a la verdad y se reescala con la velocidad |
| Alerón cambiado, el piloto balancea las alas en MANUAL | La estimación de `b` es negativa → solo una advertencia, el eje no se desactiva |
| 30 s de turbulencia | Se contrarrestan las ráfagas, el alabeo no pasa de 10° |
| Morro a 15° con el 20 % de gas (con sensor de velocidad y sin él) | La velocidad no cae hasta la pérdida de sustentación |
| Despegue desde una pista con el par de reacción de la hélice | Fases, altitud, rumbo en la carrera |
| Aterrizaje desde 15 m | Fases, sin gas cerca del suelo, toma suave |
| Pérdida de enlace en la carrera de despegue; no armado; MANUAL | Cancelación, no se toca el gas, las superficies quedan en manos del piloto |

El modelo es tosco: comprueba la lógica y los signos, no el ajuste para un
fuselaje concreto.

```bash
pio test -e native -f test_feedback      # en el PC, en segundos
pio test -e esp32-s3 -f test_feedback    # carga el firmware de prueba y lo ejecuta
pio run -t upload                        # devolver el firmware normal
```

### Plan de conexión

1. `FlightController::update()`, tras leer los sensores y calcular las
   órdenes, rellena un `FlightSnapshot` y llama a
   `FeedbackSupervisor::update()`. Al principio, en **modo sombra**: la salida
   va solo al registro (`printStatus()`) y al dashboard, no a las superficies.
   En vuelo con control manual, la estimación de `b` de cada eje debe ser
   positiva y crecer con la velocidad.
2. En tierra, con el avión en las manos, STABILIZE: inclinarlo; las
   superficies lo contrarrestan.
3. De un eje en un eje: `deflectionUs` en lugar de
   `Autopilot::getRollCorrection()` (primero solo el alabeo), luego el cabeceo.
4. Acelerador: `throttleOverridePercent`/`throttleFloorPercent`, después de
   `Autopilot::applyThrottle()` y antes del failsafe (el failsafe manda sobre
   todo).
5. Despegue/aterrizaje: en un interruptor libre; quitar de `Autopilot` el modo
   `AUTO_TAKEOFF`.
6. Las constantes de `FeedbackConfig`, a `Config.h`; el sensor de velocidad
   aerodinámica: una implementación de `AirspeedSensor` y una categoría en
   `SensorSelection.h`.

---

## Cómo añadir una placa nueva

1. `[env:<name>]` en `platformio.ini` con un `-D BOARD_ESP32_<NAME>` único.
2. Un bloque `#elif defined(BOARD_ESP32_<NAME>)` en `Config.h` con todos los
   pines, incluidos `PIN_I2C2_SDA/SCL` (−1 si no hay OLED). Calcule el
   presupuesto de GPIO de antemano: flash/PSRAM/USB/strapping.
3. Las salidas de los servos necesitan 5 canales LEDC, que tienen todas las
   ESP32. Si no hay pin para el timón de dirección, `PIN_RUDDER = -1` y la
   salida simplemente se desactiva.
4. No cambie `default_envs` hasta que la placa se haya probado en el
   hardware; indique explícitamente en el commit si la distribución de pines
   no está probada.

---

## Comandos de compilación, carga y monitor

```bash
pio run                        # compilar la placa predeterminada (esp32-s3)
pio run -t upload              # cargar
pio device monitor             # monitor, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # comprobar que se compilan todas las placas
```

- **ESP32-S3:** la carga y el Serial van por el conector «COM» (CH343). Si el
  puente se cuelga (Windows responde «el dispositivo no funciona», algo que
  puede deberse a las interferencias del ESC), ayuda volver a conectar el
  cable; también se puede cargar por el conector «USB» (el USB-JTAG
  integrado): `pio run -t upload --upload-port <USB COM port>`.
- Mientras esté abierto el monitor del puerto, la carga al mismo puerto no
  funcionará.
- `lib_deps`: `olikraus/U8g2` (OLED) es la única biblioteca externa.
- `test/`: con detalle en [`TESTING.md`](TESTING.md):
  - `pio test -e native -e native-stm32`: 387 pruebas en el PC (simulacros
    del hardware en `test/native/support/`), cobertura con `gcovr`;
  - `pio test -e esp32-s3`: `test_feedback/` (la simulación en lazo cerrado de
    la realimentación) y `test_imu_orientation/` en la propia placa; cada una
    carga un firmware de prueba y después hay que cargar el normal con
    `pio run -t upload`.
- Análisis estático: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck sobre
  `hal/stm32/` y `src/stm32/`) y `tools/clang-tidy.sh`
  (el perfil `.clang-tidy`).

---

## Limitaciones conocidas

- **El piloto automático no se ha probado en vuelo.** Sobre la mesa los signos
  se han comprobado en vivo (inclinación → corrección hacia la nivelación); los
  coeficientes del PID son valores iniciales.
- **STABILIZE es una nivelación superpuesta a los sticks**, y no un «modo
  angular» (FBWA) en el que el stick fija el ángulo de alabeo/cabeceo. El
  piloto y el piloto automático se suman.
- **El planeo por pérdida de enlace no se ha probado en vuelo.** Los ángulos
  `FAILSAFE_GLIDE_*` son valores iniciales; el cabeceo de −3° se ajusta a un
  fuselaje concreto (el morro no debe ni levantarse hasta la pérdida de
  sustentación ni picar).
- **El horizonte.** Con la calibración del montaje (`o`), sale de ella (NVS); el
  desvío de cero del acelerómetro deriva con la temperatura (~1–2° cada
  20 °C); si el horizonte se ha «ido», repetir `o`. Sin ella, la postura al
  encender (encender con el avión nivelado).
- **El montaje de la brújula** sigue fijándose con `MAG_ROTATION_CW_DEG`
  (la calibración por posturas no lo afecta).
- **Brújula:** el rumbo no tiene compensación de inclinación, el sentido de la
  lectura no se ha comprobado en el avión montado y la calibración hay que
  hacerla ya dentro del avión. Ningún modo usa todavía el rumbo.
- **El GPS** no se usa para la navegación; en la ESP32-C3 es solo de recepción.
- **La realimentación (`autopilot/feedback/`) no está conectada** y solo se
  ha comprobado en simulación con un modelo tosco del avión. Todos los
  números de `FeedbackConfig.h` marcados «прикидка» («estimación aproximada») hay que
  afinarlos en un fuselaje real; todavía no hay sensor de velocidad
  aerodinámica (sin él, la efectividad de las superficies se aprende más
  despacio y la pérdida de sustentación solo se ve por la deceleración).
- **No probados en hardware:** `ICM42688_Sensor` (adaptado al convenio común
  mediante `ImuSensorBase`), `BME280_Sensor` (la compensación de Bosch se
  implementó de nuevo), BMP388 por SPI, `QMC5883L_Sensor`, la configuración
  del GPS mediante CFG-VALSET. Al conectarlos: el registro de arranque, `s` en
  la consola, los signos con la inclinación.
- **El I2C en una protoboard capta interferencias** del ESC/motor (los errores
  aislados se ven con `s`). Los controladores las soportan, pero en el avión
  los cables del I2C deben ser cortos y estar alejados de los de potencia.
- **No se usa ESP32Servo.** La versión 3.2.1 en la ESP32-S3 reparte los
  servos entre los MCPWM y, en `attachPin()`, confunde el número del bloque
  MCPWM con el del temporizador: los GPIO6/7 sacaban la señal de los GPIO4/5
  (el ESC se controlaba con el stick derecho). Las salidas se reescribieron
  sobre LEDC; solo se debe devolver la biblioteca tras comprobarlo con `p`.
- **El ESC es de PWM a 50 Hz**; el firmware todavía no tiene un modo de
  calibración del rango del acelerador.
- **Dashboard web:** la contraseña del punto de acceso es débil y las órdenes
  se aceptan también en vuelo. Es una herramienta para el banco y el campo,
  no para el vuelo.
- **Mecánica del prototipo:** el primer prototipo voló y se detectaron una
  sujeción débil del motor y una rigidez insuficiente del ala.
- **La licencia es la OpenPlane License** ([LICENSE](LICENSE.md)): MIT con
  mención obligatoria del autor, prohibición del uso militar y prohibición del
  daño intencionado a personas y bienes sin su consentimiento por escrito. No
  añada otras cabeceras de licencia a los archivos ni quite el nombre del
  autor.

---

## Cómo hacer cambios

- **Commits pequeños:** un paso lógico, un commit.
- **Pruebas y análisis antes de un commit:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh`,
  todo en verde ([`TESTING.md`](TESTING.md)).
- **Compile todas las placas** tras cambios en el código común: la S3 es la
  principal, pero la C3, la de 38 pines y `stm32h743` no deben romperse;
  antes de una versión, `tools/build_matrix.sh` (todas las placas × todos los
  sensores).
- **Compruebe en el hardware lo que se pueda comprobar:** los signos, con la
  inclinación; las salidas, con el comando `p`; el enlace, apagando la
  emisora.
- **No invente APIs.** Consulte los fuentes del framework en
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x);
  internet suele describir la versión 3.x con otra API (por ejemplo, LEDC).
- **No adorne el estado.** Si no está probado en el hardware, dígalo así.
- **Una capa no debe saber más de lo que le corresponde.** Si una clase
  inferior necesita de pronto una superior, la lógica debe subir a
  `FlightController`.
- **Al cambiar un contrato de datos** (`FlightOutputState`, `ControlCommand`,
  `ImuData`, el JSON de `/api/status`), actualice a todos los consumidores en
  el mismo commit.
