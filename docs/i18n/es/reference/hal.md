# HAL — abstracción del hardware

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/hal.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

El HAL es la única capa a la que se permite conocer un MCU concreto. Las
interfaces están en `include/hal/`; las implementaciones son:

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x);
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x), **la principal**: el firmware completo se compila (`pio run -e stm32h743-devebox`) y se ejecuta en el PC (`pio test -e native-stm32`); en la placa DevEBox se han verificado la tarjeta SD, la caja negra, el iBUS y los servos, pero todavía no los sensores;
- `hal/Rtos.h` — tareas de FreeRTOS, iguales en ambas plataformas.

Todo lo que hay por encima trabaja solo con las interfaces, así que pasar a otro
MCU significa una nueva implementación de `IBoard`, no reescribir los sensores.

---

## namespace `ServoChannel`

**Archivo:** `hal/IBoard.h`

Índices de las salidas para `IBoard::servo(channel)`. Es una lista plana, y no
métodos con nombre: añadir una salida no cambia la interfaz `IBoard`. El orden
coincide con las filas de la tabla `FlightOutputs::outputInfo()`.

| Constante | Valor |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — lanzamiento de carga (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — cámara (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**Archivo:** `hal/IBoard.h` · **Clase:** interfaz · **Implementaciones:** `Esp32Board`, `Stm32Board`

El único punto de entrada al hardware. Nada de lo que hay por encima incluye
`<Wire.h>`, `<SPI.h>` ni `HardwareSerial`, ni llama directamente a LEDC.

| Método | Descripción |
|---|---|
| `virtual void begin()` | Inicialización única de los buses I2C/SPI. Los UART los abren sus propietarios (`IBusReceiver`, GPS) con su propia velocidad, el PWM lo abre `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | El bus de sensores |
| `virtual ISpiBus& spi()` | El bus SPI |
| `virtual II2CBus* displayI2c()` | Un segundo bus I2C solo para la pantalla; `nullptr` si no existe |
| `virtual IUartPort& rcUart()` | El UART del receptor iBUS |
| `virtual IUartPort& gpsUart()` | El UART del GPS |
| `virtual IUartPort* telemetryUart()` | El UART del módem de radio MAVLink; por defecto `nullptr` (el ESP32 no tiene un UART libre) |
| `virtual IServoOutput& servo(uint8_t channel)` | Una salida PWM por índice `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | El zumbador `PIN_BUZZER`; por defecto no hace nada |

---

## `II2CBus`

**Archivo:** `hal/II2CBus.h` · **Clase:** interfaz con funciones auxiliares no virtuales ·
**Implementaciones:** `Esp32I2CBus`, `Stm32I2CBus`

Una abstracción del bus I2C con la forma de `Wire`. Los pines y la frecuencia los
fija la implementación en el constructor, por eso `begin()`/`setClock()` no llevan
pines: el bus se inicializa exactamente una vez, aunque tenga varios dispositivos.

| Método | Descripción |
|---|---|
| `begin()`, `setClock(hz)` | Inicialización, frecuencia |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Escritura; `endTransmission` devuelve 0 si todo va bien (como `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Lectura |
| `bool writeRegister(addr, reg, value)` | Auxiliar: escribe un solo registro; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Auxiliar: inicio repetido + lectura de `count` bytes. `false` si hay NACK **o llegan menos de `count` bytes**; el búfer no se toca |
| `int readRegister(addr, reg)` | El valor del registro o `-1` |
| `bool probe(addr)` | El dispositivo responde con ACK a la dirección |

Invariante: si algo falla, los auxiliares no escriben en el búfer: el driver
conserva los datos anteriores, y no basura (el `0xFF` que `read()` devuelve con
el búfer vacío).

---

## `ISpiBus`

**Archivo:** `hal/ISpiBus.h` · **Clase:** interfaz · **Implementaciones:** `Esp32SpiBus`, `Stm32SpiBus`

Un bus SPI **sin gestión de CS**: varios dispositivos comparten un bus, y es
`SpiRegisterDevice` quien conmuta CS.

| Método | Descripción |
|---|---|
| `begin()` | Configura SCK/MISO/MOSI (los pines están en el constructor de la implementación) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Intercambio de un byte en dúplex completo |
| `endTransaction()` | Fin de la transacción |

---

## `IUartPort`

**Archivo:** `hal/IUartPort.h` · **Clase:** interfaz · **Implementaciones:** `Esp32UartPort`, `Stm32UartPort`

Un UART con la forma de `HardwareSerial`, pero `begin()` solo recibe la velocidad:
los pines y el formato (8N1) los fija la implementación.

| Método | Descripción |
|---|---|
| `begin(baud)` | Abrir el puerto |
| `int available()`, `int read()` | Recepción |
| `size_t write(byte)`, `size_t write(buffer, size)` | Transmisión |
| `virtual int availableForWrite()` | Espacio libre en el búfer de transmisión; `-1` — desconocido (por defecto). La telemetría lo usa para aplazar una trama en lugar de esperar |

---

## `IServoOutput`

**Archivo:** `hal/IServoOutput.h` · **Clase:** interfaz · **Implementaciones:** `Esp32ServoOutput`, `Stm32ServoOutput`

Una única salida PWM. El pin lo fija la implementación.

| Método | Descripción |
|---|---|
| `bool attach(minUs, maxUs)` | Reserva el canal/temporizador y configura el pin; el rango de limitación del pulso. `true` solo indica que el MCU ha reservado los recursos, **no** que haya un servo conectado |
| `writeMicroseconds(us)` | Anchura del pulso, µs (se limita al rango de `attach`) |
| `bool isAttached() const` | El resultado de `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnóstico: la anchura real del pulso en el pin o `-1`. La implementación por defecto devuelve `-1` |

---

## `IFlashRegion`

**Archivo:** `hal/IFlashRegion.h` · **Clase:** interfaz · **Implementaciones:** `Esp32FlashPartition`, `SdFileRegion`

Una región de flash NOR para el registro (la caja negra): el borrado solo se hace
por sectores de 4 KB (lo borrado se lee como `0xFF`), la escritura solo pone bits
a cero: se puede escribir en bytes borrados, también por partes dentro de una misma
página. En el ESP32, tanto la escritura como el borrado detienen los dos núcleos;
decide quien llama cuándo es admisible.

| Método | Descripción |
|---|---|
| `uint32_t size() const` | Tamaño de la región, bytes; 0 — no hay región |
| `bool read(offset, data, length)` | Leer |
| `bool write(offset, data, length)` | Escribir (en bytes borrados) |
| `bool erase(offset, length)` | Borrar; la dirección y la longitud son múltiplos de 4096 |

`Esp32FlashPartition(const char* name)` — una partición de datos por su nombre en
la tabla de particiones (`esp_partition_*`); `begin()` busca la partición (después
de arrancar el núcleo); si no existe, devuelve `false` y `size() == 0`.

---

## `IBlockDevice`

**Archivo:** `hal/IBlockDevice.h` · **Clase:** interfaz · **Implementaciones:** `Stm32SdCard` (en las pruebas — `fake::SdCardModel`)

Una tarjeta SD como un array de bloques de 512 bytes. No hay borrado: un bloque se puede sobrescribir.

| Método | Descripción |
|---|---|
| `uint32_t blockCount() const` | Tamaño en bloques; 0 — no hay tarjeta |
| `bool read(block, data, count)` / `write(...)` | `count` bloques seguidos, `data` — cualquier dirección |

## `SdFileRegion`

**Archivo:** `hal/SdFileRegion.h` · **Hereda de:** `IFlashRegion` · **Depende de:** `IBlockDevice`, `Fat32::locate`

La región de la caja negra en la tarjeta SD: un archivo en la raíz de FAT32 (por
defecto `BLACKBOX.BIN`), creado de antemano en el PC de una sola pieza
(`tools/blackbox.py
sd-prepare`) y relleno con `0xFF`. El archivo solo se **localiza** (las tablas FAT
y el directorio no se tocan); después se escriben bloques en bruto dentro de él.
Para `BlackBoxStorage` es el mismo `IFlashRegion` que la partición de flash del ESP32.

| Método | Descripción |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` es el techo de la región: el tiempo de verificación de sectores al encender crece con él |
| `Fat32::Result begin()` | Busca el archivo. `Ok` — `size() > 0`; si no, la causa (`Fat32::describe()`): no hay tarjeta, no es FAT32, no hay archivo, está fragmentado, está vacío |
| `size()` | El archivo (no más de `maxBytes`), redondeado hacia abajo a un sector de 4 KB; 0 — no hay región |
| `read` / `write` | Cualquier desplazamiento y longitud. Un bloque incompleto se lee, se completa y se escribe entero; el bloque que se acaba de escribir se recuerda (caché de escritura directa): las páginas de 256 bytes seguidas no vuelven a leer la tarjeta. Un corte de energía no pierde nada de lo que ya ha vuelto de `write()` |
| `erase(offset, length)` | Múltiplo de 4096; escribe `0xFF` (la tarjeta tiene su propio borrado interno, por fuera no hace falta) |

## `IRegisterDevice`

**Archivo:** `hal/RegisterDevice.h` · **Clase:** interfaz ·
**Implementaciones:** `I2cRegisterDevice`, `SpiRegisterDevice`

«Un conjunto de registros de 8 bits». El driver de un sensor se escribe una sola vez,
y el bus se elige al crear el objeto en `SensorSelection.h`.

| Método | Descripción |
|---|---|
| `virtual void begin()` | Prepara las líneas del dispositivo (en SPI — CS). Por defecto no hace nada |
| `virtual bool probe()` | El dispositivo ha respondido (en SPI siempre `true`: no hay ACK, se comprueba el registro de ID) |
| `virtual bool writeRegister(reg, value)` | Escribe un registro |
| `virtual bool writeRegisters(reg, data, count)` | Escribe de forma consecutiva (autoincremento de la dirección) |
| `virtual bool readRegisters(reg, buffer, count)` | Lee `count` bytes consecutivos; si devuelve `false`, el búfer no se toca |
| `int readRegister(reg)` | El valor o `-1` (una función auxiliar no virtual) |

---

## `I2cRegisterDevice`

**Archivo:** `hal/RegisterDevice.h` · **Hereda de:** `IRegisterDevice`

Un dispositivo en un `II2CBus` con dirección de 7 bits. Todas las operaciones se
delegan en las funciones auxiliares de `II2CBus`.

| Método | Descripción |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` es la segunda dirección del chip (el pin SDO/SA0): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | la principal no responde pero la alternativa sí: se sigue trabajando con la alternativa |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → las funciones auxiliares de `II2CBus(address, …)` |
| `uint8_t getAddress() const` | la dirección actual del dispositivo |

---

## `SpiRegisterDevice`

**Archivo:** `hal/RegisterDevice.h` · **Hereda de:** `IRegisterDevice`

Un dispositivo en un `ISpiBus` con su propio pin CS. Protocolo Bosch/InvenSense:
para leer, la dirección con el bit `0x80`; para escribir, con el bit 7 a cero.

| Método | Descripción |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` es cuántos bytes «basura» entrega el chip tras la dirección antes de los datos (BMP388 — 1, ICM42688 — 0); `mode` es el modo SPI 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Siempre `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; siempre `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, se salta `dummyReadBytes`, `n` bytes, CS↑; siempre `true` |

Cada operación es una transacción independiente `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Archivo:** `hal/esp32/Esp32Board.h` · **Hereda de:** `IBoard`

El único lugar que crea los objetos concretos de periféricos del ESP32 y conoce
los pines de `Config.h`.

| Campo | Tipo | Qué es |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` en `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` en `PIN_I2C2_SDA/SCL` — solo si `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | El `SPI` global |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS en `PIN_IBUS`, solo RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS en `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | Canales LEDC 0..6 en el orden de `ServoChannel` (AUX1/AUX2 — `PIN_AUX1/2`, si están cableados) |

| Método | Descripción |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, luego `displayBus.begin()` si existe el segundo bus; el pin del zumbador |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` si el pin está cableado |
| `displayI2c()` | `&displayBus` si `hasDisplayBus()`, y `nullptr` en caso contrario |
| `static constexpr bool hasDisplayBus()` | Los dos pines del segundo bus son ≥ 0. Existe (igual que el campo `displayBus`) solo con `SOC_I2C_NUM > 1`: el C3 tiene un único controlador I2C |
| el resto | Devuelven los campos correspondientes |

---

## `Esp32I2CBus`

**Archivo:** `hal/esp32/Esp32I2CBus.h` · **Hereda de:** `II2CBus`

Una envoltura fina sobre `TwoWire` (`Wire` o `Wire1`).

| Método | Descripción |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Guarda los parámetros |
| `begin()` | `wire.begin(sda, scl, hz)` y `wire.setTimeOut(TIMEOUT_MS)`: la única llamada a `wire.begin()` |
| el resto | Delegación directa en `TwoWire` |

`TIMEOUT_MS = 5`: leer 14 bytes de la IMU a 400 kHz lleva ~0.4 ms; una transacción
colgada por una interferencia detendría de otro modo el bucle durante los 50 ms estándar.

---

## `Esp32SpiBus`

**Archivo:** `hal/esp32/Esp32SpiBus.h` · **Hereda de:** `ISpiBus`

Una envoltura sobre el `SPI` global. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS
lo gestionan los dispositivos). `beginTransaction()` construye `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` convierte 0..3 en constantes de Arduino, y un valor
desconocido → `SPI_MODE0`.

---

## `Esp32UartPort`

**Archivo:** `hal/esp32/Esp32UartPort.h` · **Hereda de:** `IUartPort`

Una envoltura sobre `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — solo recepción. El resto es delegación.

---

## `Esp32ServoOutput`

**Archivo:** `hal/esp32/Esp32ServoOutput.h` · **Hereda de:** `IServoOutput`

PWM directamente a través de LEDC (`ledcSetup/ledcAttachPin/ledcWrite` del Arduino core 2.x).
La biblioteca ESP32Servo **no se usa**: la versión 3.2.1 en el S3 confundía los bloques
MCPWM (GPIO6/7 repetían GPIO4/5).

| Constante | Valor |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1.2 µs por paso) |

| Método | Descripción |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Pin `< 0` — la salida no está cableada |
| `attach(minUs, maxUs)` | Guarda el rango; pin < 0 → `false`; si no, `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Si no está attached, nada; si lo está, `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Activa el búfer de entrada del mismo GPIO (`PIN_INPUT_ENABLE`, la salida no se toca) y mide `pulseIn(pin, HIGH, 30 ms)`; sin pulso → `-1` |

Los canales 2n y 2n+1 comparten un temporizador LEDC: todas las salidas van a 50 Hz, así que no hay conflicto.

---

# Implementación para el STM32H743

La placa de la siguiente generación es la STM32H743VIT6 (Cortex-M7 a 480 MHz, 2 MB de flash,
1 MB de RAM). El firmware completo se compila (env `stm32h743` — la placa de PlatformIO
`weact_mini_h743vitx`, y `stm32h743-devebox` — la DevEBox H743, consola por
USB CDC), pasa cppcheck y las pruebas en el PC (env `native-stm32` con la capa de
simulación de STM32duino). En la placa DevEBox **sin sensores** se ha verificado: el arranque, la tarjeta SD, la caja negra —
[pruebas en la placa](../TESTING.md#pruebas-en-la-placa-stm32) — y la recepción de iBUS, ARM, el PWM hacia
los servos y el motor: el avión se maneja desde el mando en modo manual (el lanzamiento está grabado en vídeo).
Los sensores todavía no se han conectado a la placa.
La distribución de pines es el bloque `BOARD_STM32H743` de [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

Las diferencias generales respecto al ESP32 que oculta esta capa:

- **El núcleo elige los periféricos.** STM32duino encuentra por sí mismo el controlador
  (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) a partir de los números de pin, usando las
  tablas `PeripheralPins` de la variante; por eso en `Config.h` no hay números de UART ni de canales.
- **Los números de pin** son los «pines de Arduino» de la variante (`PA0`, `PD14`...), no GPIO; en
  los pines analógicos son `0xC0 + N`, por lo que los pines del bloque STM32 son `int16_t`.
- **Los pines del UART** se fijan al crear el objeto `Uart(rx, tx)`, y no en `begin()`.

## `Stm32Board`

**Archivo:** `hal/stm32/Stm32Board.h` · **Hereda de:** `IBoard`

Lo mismo que `Esp32Board`, sobre STM32duino.

| Campo | Tipo | Qué es |
|---|---|---|
| `displayWire` | `TwoWire` | El segundo controlador I2C (el `Wire` global lo ocupan los sensores). Se declara antes que `displayBus`, que guarda una referencia a él |
| `i2cBus` | `Stm32I2CBus` | `Wire` en `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` en `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) — el segundo bus siempre está presente |
| `spiBus` | `Stm32SpiBus` | El `SPI` global en `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, reservado para iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | el módem de radio MAVLink: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | En el orden de `ServoChannel` (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| Método | Descripción |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, el pin del zumbador |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Siempre `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Convierte un pin de `Config.h` al tipo de la API del núcleo |
| el resto | Devuelven los campos correspondientes |

## `Stm32I2CBus`

**Archivo:** `hal/stm32/Stm32I2CBus.h` · **Hereda de:** `II2CBus`

| Método | Descripción |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Guarda los parámetros |
| `begin()` | `setSDA()`/`setSCL()` (solo surten efecto antes de `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; el resultado `size_t` se convierte a `uint8_t` |
| el resto | Delegación directa en `TwoWire` |

En STM32duino el tiempo de espera de la transacción no es un método, sino la macro
`I2C_TIMEOUT_TICK` (ms, por defecto 100). En el env `stm32h743` se fija con la bandera
`-D I2C_TIMEOUT_TICK=5`, por la misma razón que `TIMEOUT_MS` en `Esp32I2CBus`.

## `Stm32SpiBus`

**Archivo:** `hal/stm32/Stm32SpiBus.h` · **Hereda de:** `ISpiBus`

Una envoltura sobre `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
el NSS por hardware no se usa: CS lo conmuta `SpiRegisterDevice`, como en el ESP32.
`beginTransaction()` construye `SPISettings(hz, MSBFIRST, SPIMode)`; `spiModeOf()`
convierte 0..3 en `SPI_MODEn`, y un valor desconocido → `SPI_MODE0`.

## `Stm32UartPort`

**Archivo:** `hal/stm32/Stm32UartPort.h` · **Hereda de:** `IUartPort`

Una envoltura sobre `HardwareSerial&` (en STM32duino 3.x es la base abstracta
`arduino::HardwareSerial`; el objeto concreto `Uart` lo crea `Stm32Board`).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` viene
de `HardwareSerial`. Los búferes (`SERIAL_RX/TX_BUFFER_SIZE` en el env): recepción 256
bytes (una trama NAV-PVT son 100, los 64 estándar se quedan cortos), transmisión 1024 (líneas del registro y
tramas MAVLink sin esperar).

## `Stm32ServoOutput`

**Archivo:** `hal/stm32/Stm32ServoOutput.h` · **Hereda de:** `IServoOutput`

PWM por hardware de un temporizador mediante `HardwareTimer`, 50 Hz. El pulso lo
genera el temporizador sin interrupciones y sin CPU, a diferencia de la biblioteca
`Servo` para STM32, que conmuta los pines desde la interrupción de un solo temporizador
y provoca jitter.

| Constante | Valor |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — cuántos temporizadores distintos pueden ocupar las salidas (ahora están ocupados TIM2 y TIM4) |

| Método | Descripción |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Pin `< 0` — la salida no está cableada |
| `attach(minUs, maxUs)` | El temporizador y el canal salen de `PinMap_TIM` según el pin (`pinmap_peripheral`, `STM_PIN_CHANNEL`), igual que en `analogWrite()`. Sin temporizador en el pin o con el conjunto agotado → `false`. Si no, `setMode(PWM1)`, comparación 0 (no hay pulso hasta la primera escritura), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. El registro de comparación tiene precarga: el valor entra en vigor en el siguiente periodo |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` sin reconfigurar el pin: en el STM32 el registro IDR ve el nivel incluso en modo de función alternativa |
| `static acquireTimer(TIM_TypeDef*)` | Un conjunto compartido: **un `HardwareTimer` por TIMx**. Un segundo objeto para el mismo temporizador sobrescribiría el manejador del núcleo (`HardwareTimer_Handle[index]`). El periodo se fija con la primera salida del temporizador; `setOverflow(MICROSEC_FORMAT)` elige el divisor — un paso de ~0.3 µs con un reloj del temporizador de 240 MHz |

## `Stm32FlashStorage`

**Archivo:** `hal/stm32/Stm32FlashStorage.h` · **Hereda de:** `IFlashStorage` ([storage.md](storage.md))

El soporte de `KeyValueStore` en el STM32: el último sector de la flash (banco 2) mediante
la emulación de EEPROM de STM32duino (`eeprom_buffer_fill/flush`, un búfer de 8 KB del que
se usan los primeros `KeyValueStore::CAPACITY` bytes).

| Método | Descripción |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + lectura del búfer byte a byte |
| `write(src, n)` | **rápido**: copia la imagen a su propio búfer bajo `noInterrupts()` y levanta la bandera «hay escritura pendiente». Se llama desde `KvPreferences::end()` en la tarea de vuelo |
| `bool service()` | **lento**: una instantánea al búfer de emulación (bajo `noInterrupts()`) y `eeprom_buffer_flush()` — borrado de un sector de 128 KB (segundos) y escritura. Solo desde la tarea en segundo plano `storage` |
| `hasPending()`, `flushCount()` | diagnóstico |
| `static instance()`, `static store()` | el soporte y el `KeyValueStore` común del firmware |

Por qué el vuelo no se congela: el sector de ajustes está en el banco 2 y el código en el
banco 1; la flash del H7 puede leer un banco mientras se escribe el otro;
la tarea de vuelo desaloja a la de segundo plano.

## `compat/Preferences.h`

**Archivo:** `hal/stm32/compat/Preferences.h` — en el env `stm32h743` (y en
`native-stm32`) el directorio `compat/` va en `-I` antes que las bibliotecas, y
el `#include <Preferences.h>` de los drivers de sensores, del autotrimmer y de los ajustes
del registro lo encuentra. `class Preferences : public KvPreferences` sobre
`Stm32FlashStorage::store()` — la misma API que NVS del ESP32 ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**Archivo:** `hal/stm32/Stm32SdCard.h` · **Hereda de:** `IBlockDevice` · **Pines:** `src/stm32/sd_msp.cpp`

Una tarjeta SD en SDMMC1: bus de 4 bits, `HAL_SD` en modo de sondeo (sin DMA ni
interrupciones) **con control de flujo por hardware**: la tarea de vuelo desaloja a la tarea
de escritura en mitad de un bloque, y sin él la FIFO se desbordaba
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20): en la placa eso se veía como la consola y el
registro congelados durante segundos. Los pines PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) son la ranura µSD de la
DevEBox y de WeAct. El núcleo SDMMC recibe el reloj de PLL1Q = 48 MHz, `ClockDiv = 1` →
**24 MHz**; si la primera lectura a 24 MHz falla, se prueban 12 y 6.

| Miembro | Descripción |
|---|---|
| `bool begin()` | Levanta el bus, identifica la tarjeta, lectura de prueba. `false` — no hay tarjeta; `initError()` — el código |
| `read` / `write` | En trozos de no más de 4 KB (con pausas cortas); una dirección que no es múltiplo de 4 se copia a través de un búfer alineado (el HAL lee la FIFO por palabras). Si falla, un reintento |
| espera | Antes de un acceso tras una escritura espera a que la tarjeta vuelva al estado de transferencia (`Rtos::sleepMs(1)`: las tareas en segundo plano no se quedan sin CPU), hasta 1 s. Tras una lectura no se envía una consulta de estado adicional: la verificación al encender lee decenas de miles de sectores |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | Para la línea de estado |
| `readOps`, `writeOps`, `errors`, `retries` | Contadores |

## `ResetCause`

**Archivo:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

La causa del reinicio, igual en ambas placas. ESP32 — `esp_reset_reason()`;
STM32 — las banderas de `RCC->RSR` (se leen una vez y se borran; en el H7 `PINRSTF`
se activa con cualquier reinicio, así que primero se comprueban las causas más
concretas: perro guardián → encendido → caída de tensión → reinicio por software).
Un pánico, los perros guardianes y una caída de la alimentación cuentan como «fallo»:
la caja negra empieza a grabar al instante con ellos.

## `Rtos`

**Archivo:** `hal/Rtos.h` · namespace

| Miembro | Descripción |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | prioridades de las tareas |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., núcleo 0)`, la pila en bytes; STM32 — `xTaskCreate`, la pila se convierte a palabras; `handle` es para `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`; antes de arrancar el planificador (el `setup()` del STM32) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 — el spinlock `portMUX`, STM32 — `taskENTER_CRITICAL()`. Dentro, solo copia de bytes (la cola de la caja negra) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`; STM32 — `xPortGetFreeHeapSize()` |

## El punto de entrada `src/stm32/main.cpp`

El firmware completo: los mismos objetos que `src/main.cpp`, telemetría MAVLink en lugar de
Wi-Fi, tareas de FreeRTOS en lugar de `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).
