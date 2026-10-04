# TESTING.md: pruebas, cobertura y análisis estático

> 🌐 Esta página es una traducción del [original en ruso](../../TESTING.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

El firmware se verifica en dos niveles:

| Dónde | Comando | Qué |
|---|---|---|
| **PC (native)** | `pio test -e native` | Los encabezados del firmware se compilan en el PC sin cambios, con el hardware sustituido por simulaciones (fakes) controlables: módulos, controladores, simulaciones de vuelo en lazo cerrado, todo el firmware del ESP32 (S3 y de 38 pines) con cada kit de sensores. Se calcula la cobertura |
| **PC (native-stm32)** | `pio test -e native-stm32` | Todo el firmware de la STM32H743 (`src/stm32/main.cpp`) sobre una capa de fakes de STM32duino: tareas de FreeRTOS, flash, MAVLink, sensores en I2C y SPI |
| **Matriz de compilaciones** | `tools/build_matrix.sh` | 4 placas × 6 kits de sensores con `-Wall -Wextra (-Wshadow)`; cualquier aviso en el código del proyecto es un error |
| **Placa** | `pio test -e esp32-s3` | `test_feedback` y `test_imu_orientation` en un ESP32-S3 real (carga un firmware de prueba; después vuelve a poner el normal: `pio run -t upload`) |
| **Placa STM32** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | La caja negra en una **tarjeta SD real** de la DevEBox H743, además de `test_feedback` y `test_imu_orientation` en un Cortex-M7 — [más abajo](#pruebas-en-la-placa-stm32) |

El contexto de arquitectura está en [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-facilidad-de-prueba).

---

## Inicio rápido

```bash
pip install platformio gcovr        # una sola vez
# Windows: hace falta g++ en el PATH, por ejemplo WinLibs (winlibs.com, zip UCRT):
# descomprímelo y añade mingw64\bin al PATH; no hace falta instalar nada
pio test -e native -e native-stm32  # todas las pruebas nativas (~1.5 min)
gcovr                               # cobertura por archivos (ajustes: gcovr.cfg)
tools/build_matrix.sh               # todas las placas × todos los sensores (~25 min)
gcovr --html-details -o coverage/index.html   # informe HTML (coverage/ está en .gitignore)

pio test -e native -f native/test_rc          # un solo conjunto
pio test -e native -f test_feedback           # simulación de la realimentación en el PC

# Trayectorias de las simulaciones en lazo cerrado en CSV (para gráficas):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# El flujo MAVLink, para comprobarlo con un descodificador de referencia (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Antes de calcular la cobertura tras cambiar las pruebas conviene empezar con una compilación limpia: `rm -rf .pio/build/native`; si no, en el informe entran los contadores de ejecuciones anteriores.

---

## Cómo funciona la compilación nativa

`[env:native]` en `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (la distribución de pines de la placa principal), `-I test/native/support`, `-Wall -Wextra -Wshadow`, cobertura con `--coverage` y `-fkeep-inline-functions -fkeep-static-functions`; sin ellos, gcov no ve las funciones de los encabezados que nunca se llaman y exagera la cobertura.

### Fakes del hardware — `test/native/support/`

Encabezados con los mismos nombres y firmas que el núcleo Arduino para ESP32 2.0.x, ESP-IDF y las bibliotecas, pero sobre un mundo simulado en `namespace fake`:

| Archivo | Sustituye a | Qué sabe hacer la simulación |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | el núcleo Arduino | Macros (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, formato de `print()` como el original. `ARDUINO` **no** se define a propósito |
| `esp32-hal-fake.h` | tiempo, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | El reloj solo avanza con `fake::advance*()`/`delay()`; `millis()/micros()` son `uint32_t`, como en el ESP32 (el desbordamiento se comporta como en la placa). Canales LEDC, `pulseIn` según el ciclo de trabajo real (visible solo si está activado el búfer de entrada del pin), `analogReadMilliVolts`: la tensión sale de `fake::gpio().analogMv`. Las tareas se registran (el manejador no es nulo); `fake::runTask(task, n)` ejecuta n pasadas de su bucle infinito, `ulTaskNotifyTake` cuenta como una pasada y `xTaskNotifyGive`, como un contador. Los mutex de FreeRTOS son un indicador de «ocupado». `psramFound()`/`ps_malloc()`. Se cuentan las secciones críticas |
| `HardwareSerial.h` | UART | Los puertos se registran por número (`fake::uart(1)`); `pushRx()`, `txBytes()`, cambio de velocidad sobre la marcha (`updateBaudRate`, el historial es `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | particiones de flash de ESP-IDF | Una partición es un vector de bytes con comportamiento NOR: borrado solo por sectores de 4 KB, borrado = 0xFF, una escritura solo baja bits (el intento de subir un bit se cuenta: `bitRaises`); `beforeWrite`: «se fue la alimentación»; contadores de lecturas, escrituras y borrados |
| `esp_system.h` | la causa del reinicio | `esp_reset_reason()` a partir de `fake::chip().resetReason` |
| `Wire.h` | I2C | Dispositivos por dirección; `fake::RegisterMapDevice`: registros con autoincremento, registro de escrituras, fallos (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), ganchos `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Dispositivos por pin CS; `fake::SpiRegisterMapDevice`: el protocolo Bosch/InvenSense, `dummyBytes` antes de los datos |
| `Preferences.h` | NVS | Almacenamiento en memoria, comportamiento de `begin(readOnly)`/`get*`/`getBytes` como el original; `failBegin` |
| `WiFi.h`, `WebServer.h` | punto de acceso Wi-Fi, HTTP | El resultado de `softAP()` lo fija la prueba; `WebServer::request(método, uri, cuerpo)` llama al manejador registrado; `fake::webServers()`: todas las instancias |
| `U8g2lib.h` | U8g2 | En lugar de píxeles, una lista de las cadenas y los rectángulos dibujados; `begin()/sendBuffer()` pasan bytes por un callback de bytes del usuario; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### La capa de STM32duino — `test/native/support_stm32/` (entorno `native-stm32`)

Está en `-I` antes que `support/` y completa esos mismos fakes con lo que solo tiene STM32duino. `<Preferences.h>` en este entorno es el `include/hal/stm32/compat/Preferences.h` **real**, sobre `KeyValueStore`.

| Archivo | Sustituye a | Qué sabe hacer |
|---|---|---|
| `Arduino.h` | el núcleo STM32duino | pines `PA0..PE15` (puerto·16 + número), `pin_size_t`, `PinMap_TIM` para los pines de salida de servos, `HardwareTimer` (el pulso se ve con `fake::timerPulseUs(pin)` y `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | FreeRTOS de STM32duino | `xTaskCreate` al registro común de tareas (pila en palabras), `vTaskStartScheduler()` retorna: las tareas las mueve la propia prueba (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | emulación de EEPROM | una «flash» de 8 KB (borrada = 0xFF) y un búfer, `fake::eeprom()`: contadores y corrupción de la imagen |
| `SPI.h` | | `SPIMode` |

En los fakes comunes se ha añadido para STM32: `TwoWire(sda, scl)`, `setSDA/SCL` y `fake::wireWithSda(pin)` (para encontrar el segundo bus de la placa), `HardwareSerial(rx, tx)` y `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Emuladores de chips y modelo del avión — `test/native/helpers/`

| Archivo | Qué es |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (con registros indirectos IPREG), QMC6309, SPL06-001, BMP581, tramas NAV-PVT de u-blox: mapas de registros en I2C o SPI, con datos del «mundo» `World` (ángulos y velocidades, altitud, velocidad aerodinámica, rumbo, coordenadas), en los ejes del chip teniendo en cuenta `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | un modelo de avión de ~1.2 kg: una masa puntual + giro en alabeo/cabeceo, CL(α) con pérdida de sustentación, resistencia, empuje, viento, térmicas, suelo |
| `SimHarness.h` | un lazo cerrado: emisora → trama iBUS → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → deflexiones de las superficies → `PlaneSim` → sensores (incluido un tubo de Pitot sobre dos barómetros ruidosos). Una trayectoria en CSV con `OPENPLANE_SIM_DIR` |

`test/native/helpers/TestSupport.h` es lo que comparten los conjuntos: `resetWorld()` (se llama desde `setUp()`), los sustitutos `FakeUart`/`FakeServo`/`FakeBoard` y de los sensores (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), el constructor de tramas `ibusFrame()`, los bancos `I2cRig`/`SpiRig` (un controlador sobre los `Esp32I2CBus`/`Esp32SpiBus` reales y un `*RegisterDevice` con un chip simulado).

Las pruebas de la placa (`test_feedback`, `test_imu_orientation`) son portables: con `ARDUINO`, `setup()/loop()`; si no, `main()`. Los conjuntos `test/native/*` no se compilan para la placa (`test_ignore` en `[esp32_common]` y `[env:stm32h743]`: los patrones van uno por línea; separados por un espacio, PlatformIO los lee como uno solo). En STM32: `pio test -e stm32h743`.

---

## Conjuntos de pruebas

| Conjunto | Pruebas | Qué comprueba |
|---|---|---|
| `native/test_hal` | 17 | Los auxiliares de `II2CBus` (NACK, lectura corta: el búfer no se toca), `I2cRegisterDevice`, `SpiRegisterDevice` (bit de lectura, byte ficticio del BMP388), `Esp32I2CBus` (tiempo de espera de 5 ms), `Esp32SpiBus` (modos 0–3), `Esp32UartPort` (8N1, pines), `Esp32ServoOutput` (50 Hz/14 bits, límite del pulso, fallo del LEDC, medición a través del búfer de entrada), `Esp32Board` (buses, UART, orden de los canales, AUX, zumbador) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, análisis de iBUS: tramas que llegan por partes, CRC, valores de 12 bits, failsafe de la emisora, tiempo de espera de 500 ms (también con desbordamiento de `micros()`), basura, resincronización |
| `native/test_control` | 21 | Flaps (velocidad, primera llamada, pausas), el mezclador (signos, inversión, flaperones), acelerador, la máquina de estados de ARM y las comprobaciones de sensores por modo, la tabla de salidas y la autocomprobación de pulsos |
| `native/test_autopilot` | 22 | PID (término D a partir de la velocidad angular del sensor, integral, anti-windup, `dt`), STABILIZE como modo de ángulos, despegue automático por tiempo, ALT_HOLD con el timón de profundidad, planeo al perder la señal |
| `native/test_autopilot_modes` | 31 | Los 12 modos y la reacción de cada uno a la falta de un sensor, la tabla de asignaciones y `static_assert`, funciones y potenciómetros, navegación (rumbo, círculo, punto de origen, geovalla), failsafe RTH/planeo, lanzamiento a mano, vuelo a vela, autotrim (solo se guarda en tierra) |
| `native/test_flight_controller` | 12 | Un ciclo completo de `FlightController` con las clases reales: prioridades pérdida de señal > ARM > sticks/piloto automático > acelerador; AUX, `MOTOR_KILL`, zumbador |
| `native/test_imu` | 21 | MPU6050/6500/9250 e ICM-42688: identificación, registros, escalas, giro de ejes y signos aeronáuticos, errores del bus, calibración del giroscopio y comprobación previa al vuelo, calibración del montaje a partir de tres posiciones, NVS, filtro de orientación |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 por I2C y SPI, BME280/BMP280 según la referencia de Bosch, brújulas (rumbo, calibración hard-iron en NVS), u-blox M10 (CFG-VALSET, NAV-PVT, tramas dañadas, tiempo de espera), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, dirección alternativa, SPI), ICM-45686 (registros indirectos), QMC6309, SPL06-001 (fórmulas de la hoja de datos), BMP581 (DRDY y la vía de respaldo), el tubo de Pitot (cero, filtro, densidad, mangueras cruzadas, datos obsoletos, un «vuelo» con el ruido de dos barómetros) |
| `native/test_storage` | 16 | `KeyValueStore` (recarga, desgaste: un valor idéntico no se reescribe, desbordamiento sin pérdida de datos, CRC, corte de energía durante el borrado, basura, versión del formato), `KvPreferences` (se comporta como el NVS del ESP32) |
| `native/test_mavlink` | 20 | El códec frente a tramas de referencia de pymavlink (v1, v2, firmado), CRC, resincronización; telemetría: frecuencias de los flujos, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, parámetros del PID (lista, lectura, escritura, rechazo de valores incorrectos), cambio de modo desde tierra, ARM desde tierra: rechazado, misiones: 0, un búfer UART desbordado no bloquea el ciclo |
| `native/test_blackbox` | 19 | La caja negra: formato y CRC, el anillo de sectores sobre un fake de NOR (una partición nueva sin borrado, la basura siempre se borra, los vuelos antiguos se borran enteros y solo para liberar espacio, el último no se toca, paso por el final del anillo, la cabeza tras un reinicio, corte de energía, un registro a medio escribir detectado por CRC), grabación del vuelo con los `FlightController`/`Autopilot` reales: inicio por ARM y acelerador con pregrabación, parada tras DISARM y «parado en tierra», la pérdida de señal no la detiene, grabación tras un reinicio por fallo, inicio manual, eventos, batería, la flash se agotó en el aire, un vuelo más largo que la partición, descarga en tramas con CRC y cambio de velocidad, el menú de la consola `k`, sin partición: desactivada |
| `native/test_blackbox_scan` | 3 | La verificación por muestreo del anillo al encender frente a la completa: 300 historiales aleatorios del anillo × 5 pasos de sondeo (la cabeza, los números y la lista de vuelos coinciden, y cuando el cuadro no cuadra, cede ante la verificación completa) y el coste en un área de SD de 64 MB (≈530 lecturas en lugar de 32 000) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, versión), `DebugLogger` (todos los canales, NAV), `DebugConsole` (menú, teclas rápidas, sondeo de los buses `b`, prohibido con ARM, guardado solo sin ARM), `WebDebugServer` (rutas, JSON, buzón), `OledDisplay` (bytes por I2C, fotograma, inversión al perder la señal) |
| `native/test_sim` | 15 | Vuelos en lazo cerrado de todo el firmware con el modelo del avión: salida de un alabeo, CRUISE con viento cruzado, LOITER, RTH, failsafe RTH/planeo, geovalla, despegue automático desde una pista, lanzamiento a mano, aterrizaje automático, una térmica, RESCUE desde una espiral, mantenimiento de la velocidad y protección contra la pérdida de sustentación, un tubo de Pitot real en el lazo, autotrim de un avión «torcido», fallos de sensores en vuelo (IMU, barómetro, tubo de Pitot, GPS) |
| `native/test_feedback_units` | 14 | Los módulos de realimentación por separado: fuentes de velocidad, en el aire/en tierra, la estimación RLS, el regulador, indicios de pérdida de sustentación, cancelaciones del despegue y del aterrizaje |
| `native/test_app` | 10 | `src/main.cpp` en el ESP32-S3 con el kit de banco MPU6500/BMP388/QMC5883P/OLED: periodo de `loop()`, emisora → servos, ARM, modos, pérdida de señal, consola, dashboard, pantalla, caja negra (una tarea en el núcleo 0, grabación con el acelerador, vuelo tras DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` en el ESP32-S3 con el kit de vuelo: LSM6DSV + QMC6309 + SPL06 + BMP581 en el tubo + GPS: identificación de todos los chips, cero del tubo y velocidad, altitud, punto de origen por GPS, STABILIZE por los ángulos del chip, RTH al punto de origen, sondeo de los buses, dashboard |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` en el **ESP32 de 38 pines** (`BOARD_ESP32_CLASSIC`) con el kit ICM-45686 + QMC6309 + SPL06 + BMP581: distribución de pines de la placa, filtros IPREG, lanzamiento a mano, estabilización y velocidad, sondeo de un solo bus |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` en el **STM32H743** con el kit de vuelo: tareas y prioridades, periodo de 2 ms, el tubo, temporizadores PWM y `pulseIn`, MAVLink en vuelo, cambio de modo desde la GCS, ajustes escritos por una tarea en segundo plano en la «flash», pantalla en I2C1, consola |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 con el ICM-45686 y el BMP581 **por SPI** + QMC6309: flash dañada al encender, ALT_HOLD desde la GCS mantiene la altitud, pérdida de señal → RTH, visible en MAVLink; reescritura de una imagen dañada |
| `native_stm32/test_blackbox_sd` | 29 | La caja negra en la tarjeta SD: FAT32 (con MBR y sin él, un directorio en dos clústeres, entradas de ruido, un volumen ajeno/disperso/vacío), `SdFileRegion` (bloques incompletos, caché, borrado, límites, fallos), el driver real `Stm32SdCard` sobre un `HAL_SD` falso (4 bits, velocidades de respaldo, reintento, tarjeta ocupada, búferes desalineados), el anillo en la tarjeta (reinicio, corte de energía, coste de la verificación), la marca «anillo vacío», grabación del vuelo con un `FlightController`, un reinicio por fallo detectado mediante `RCC->RSR`, el ADC de la batería, errores de la tarjeta en vuelo, una tarjeta lenta, descarga por la consola, la tecla `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` con tarjeta: el arranque encuentra la tarjeta y el archivo, la tarea `bbox` graba el vuelo, el periodo del ciclo no se alarga, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` sin tarjeta: la caja negra está desactivada y explica por qué, el avión vuela, el menú `k` no se rompe |
| `test_feedback` | 10 | Simulación del avión en lazo cerrado con el lazo de realimentación (en el PC y en la placa) |
| `test_imu_orientation` | 5 | Calibración del montaje de la IMU en 300 montajes aleatorios (en el PC y en la placa) |
| **Total** | **387** | 340 en `native` + 47 en `native-stm32` (más 9 solo en la placa: `test_blackbox_sd`) |

### Pruebas en la placa STM32

`test/test_blackbox_sd` no es nativo: el driver SDMMC, la tarjeta y el tiempo son reales. Las pruebas se ejecutan en una tarea de FreeRTOS y, a su lado, funciona una tarea que imita el ciclo de vuelo con la máxima prioridad (periodo de 2 ms): desaloja a las pruebas en mitad de los accesos a la tarjeta, igual que en el firmware. Sin ella no se puede atrapar el error que se encontró en la placa: con el desalojo, el FIFO del SDMMC se desbordaba (`HAL_SD_ERROR_RX_OVERRUN`), algo que no ocurre en un bucle simple.

| Prueba | Qué comprueba |
|---|---|
| `reset_cause_is_a_normal_one` | la causa del reinicio (`RCC->RSR`) no es el watchdog ni una caída de tensión |
| `card_is_detected_on_four_bit_bus` | la tarjeta se reconoce en un bus de 4 bits a 24 MHz |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` se encuentra en FAT32 y es contiguo |
| `multi_block_writes_work_at_every_length` | escritura de 1, 2, 4 y 8 bloques en un solo acceso |
| `pages_write_with_bounded_latency_and_read_back_intact` | páginas de 256 B: la peor escritura < 250 ms (el límite de SD), de forma estable > 40 KB/s, lectura y borrado |
| `header_scan_cost_on_the_whole_area` | el coste de leer la cabecera de un sector y de la verificación completa |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | borrado de todo, dos vuelos de 20 000 registros, «reinicio»: la verificación por muestreo tarda < 2 s, los registros se leen en orden con el CRC correcto; un anillo vacío se reconoce por su marca en < 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | el `BlackBox` real con una IMU a 500 Hz en tiempo real: ni un solo registro perdido, el vuelo se lee tras un «reinicio» |
| `the_flight_task_was_not_disturbed` | la escritura en la tarjeta no alteró el periodo de la tarea que imita el vuelo (desviación < 3 ms) |

Ejecución (una tarjeta con el archivo: `python tools/blackbox.py sd-prepare E:`; **la prueba borra todos los vuelos del archivo**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

La placa debe estar en modo DFU (en la DevEBox: el cable BT0→3V3 y RST; driver WinUSB mediante Zadig; los detalles, en [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). La consola de la STM32 es USB CDC: tras cargar el firmware el puerto no aparece de inmediato, y `pio test` a veces no llega a abrirlo a tiempo («could not open port»); en ese caso ejecuta `pio test ... --without-testing` y lee la salida con cualquier programa de terminal con DTR activado (las pruebas esperan hasta 60 s a que se abra el puerto). Al terminar las pruebas, la placa espera la tecla **`D`**, que la reinicia en DFU sin el cable.

Resultados en la DevEBox H743 + una tarjeta de 16 GB (2026-10-02): `test_blackbox_sd`: 9/9, `test_feedback`: 10/10, `test_imu_orientation`: 5/5; las cifras de velocidad de la tarjeta están en [BLACKBOX.md](BLACKBOX.md#qué-se-ha-medido-en-la-placa).

### Firmware de banco — `test/bench/`

No son conjuntos de pruebas, sino miniproyectos de PlatformIO independientes que se cargan en la placa en lugar del firmware de vuelo (`pio test` no los ve: los nombres de las carpetas no empiezan por `test_`). Los pines y los límites se toman del `Config.h` común.

| Proyecto | Qué hace |
|---|---|
| `bench/elevator_sweep` | Mueve por programa el stick del timón de profundidad (CH2) a través de `ControlMixer` y `FlightOutputs`, como un stick real: hacia arriba el 100 % del recorrido, hacia abajo el 60 %, con suavidad y con pausas; 20 s de trabajo y 20 s en neutro. En las posiciones extremas mide el pulso en las salidas. Acelerador al mínimo |

Para cargarlo: `pio run -d test/bench/elevator_sweep -t upload`. Para volver al firmware de vuelo: `pio run -e esp32-s3 -t upload`.

---

## Cobertura

La calcula `gcovr` sobre `include/` y `src/` (todo lo que entra en el firmware), con los dos entornos nativos juntos: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Capa | Líneas | Ramas |
|---|---|---|
| `autopilot` | 920/943 (97.6 %) | 645/731 (88.2 %) |
| `autopilot/feedback` | 683/702 (97.3 %) | 501/570 (87.9 %) |
| `control` | 252/256 (98.4 %) | 171/189 (90.5 %) |
| `hal` | 98/102 (96.1 %) | 26/26 (100 %) |
| `hal/esp32` | 101/102 (99.0 %) | 21/22 (95.5 %) |
| `hal/stm32` | 149/158 (94.3 %) | 35/52 (67.3 %) |
| `rc` | 92/92 (100 %) | 41/42 (97.6 %) |
| `sensors` (todos) | 1444/1446 (99.9 %) | 716/835 (85.7 %) |
| `storage` | 220/220 (100 %) | 158/178 (88.8 %) |
| `telemetry` | 1413/1440 (98.1 %) | 1123/1269 (88.5 %) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95.9 %) | 20/29 (69.0 %) |
| **Total** | **5490/5584 (98.3 %)** | **3457/3943 (87.7 %)**; funciones 877/902 (97.2 %) |

Lo que queda sin cubrir y por qué:

- **Lanzamiento a mano** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`): es inalcanzable con `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`; aparecerá en las pruebas cuando la constante pase a ser configurable (se mudará a `Config.h`).
- **Lo que depende de la placa:** una salida sin pin (`PIN_RUDDER = -1` solo se da en la C3), un GPS sin pin TX (C3): las pruebas nativas ejercitan la distribución de pines de la S3, la de 38 pines y la de la STM32, pero no la de la C3 (la C3 se comprueba con la matriz de compilaciones).
- **STM32:** las ramas de error del núcleo (no hay temporizador en el pin, el conjunto de temporizadores está agotado), el mensaje `FreeRTOS не запустился` («FreeRTOS no arrancó»): en el PC `vTaskStartScheduler()` siempre retorna.
- **Ramas defensivas** a las que no se puede llegar por la API pública: `default`/`Count` en un `switch` sobre enumeraciones, `return "?"`.
- Los archivos sin líneas ejecutables (`Config.h`, `Channels.h`, `FeedbackConfig.h`, las estructuras `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, las macros de `SensorSelection.h`, el HTML del dashboard) no aparecen en el informe: se compilan en las pruebas, pero gcov no tiene nada que contar en ellos.

---

## Análisis estático

| Herramienta | Comando | Perfil |
|---|---|---|
| GCC | `tools/build_matrix.sh` (o `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Todas las placas × todos los kits de sensores. La compilación nativa de las pruebas siempre usa `-Wall -Wextra -Wshadow`; `stm32h743` usa `-Wall -Wextra` (`build_src_flags`; `-Wshadow` hace ruido en las propias cabeceras de STM32duino) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` en `[esp32_common]`: `include/` y `src/` (salvo `stm32/`), warning/style/performance/portability, supresiones en línea `// cppcheck-suppress` solo para falsos positivos (el callback de U8g2, `setup/loop`). Para `stm32h743`: los mismos indicadores sobre `include/hal/stm32/` y `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` y otros; las comprobaciones desactivadas se explican en el propio archivo |

clang-tidy se ejecuta con los fakes de `test/native/support`: clang no puede analizar las cabeceras de ESP-IDF para la arquitectura del anfitrión (si se intenta `pio check` con `clangtidy`, el análisis se interrumpe por errores de análisis sintáctico y, siendo honestos, no comprueba nada). `misc-include-cleaner` vela por que cada cabecera incluya lo que usa: las cabeceras «paraguas» (`FeedbackModules.h`, la API de `IBoard.h`/`RegisterDevice.h`, las macros de `SensorSelection.h`) están marcadas con `// IWYU pragma: export`. El script se salta el código de STM32 (`include/hal/stm32/`, `src/stm32/`): lo comprueban la compilación, cppcheck con el entorno `stm32h743` y las pruebas del entorno `native-stm32`.

La matriz de compilaciones en la última pasada: **24/24 sin advertencias**:

| Placa | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`): 0 observaciones sobre el código del proyecto.

---

## Cómo escribir pruebas nuevas

1. Un módulo con lógica y sin hardware: una prueba unitaria directa; el tiempo se pasa como parámetro o se avanza con `fake::advanceMs()`.
2. Un driver de chip: mediante `I2cRig`/`SpiRig`: los registros de un chip simulado, comprobación de los valores escritos (`chip.lastWrite(reg)`) y del análisis de los datos. Para las fórmulas, una referencia de la hoja de datos o un cálculo independiente, no una copia del código.
3. Clases con tareas infinitas de FreeRTOS: `fake::findTask("nombre")` + `fake::runTask(task, n)`; así se mueve también la tarea de vuelo de la STM32.
4. El firmware completo con otro kit de sensores u otra placa: un conjunto aparte que, antes de `#include "../../../src/main.cpp"`, define `SENSOR_KIT` (o `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); los chips, de `helpers/ChipEmulators.h`. Para STM32: `test/native_stm32/`.
5. Un modo nuevo del piloto automático: un escenario de vuelo en lazo cerrado en `test_sim`.
6. Un conjunto nuevo: una carpeta `test/native/test_<nombre>/test_main.cpp` con `main()`; `setUp()` llama a `resetWorld()` si el conjunto no necesita estado entre pruebas.
7. Si encuentras un fallo, primero una prueba que lo detecte y después la corrección.
