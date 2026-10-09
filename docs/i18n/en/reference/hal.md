# HAL — hardware abstraction

> 🌐 This page is a translation of the [Russian original](../../../reference/hal.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Reference](README.md)

The HAL is the only layer allowed to know a specific MCU. The interfaces live
in `include/hal/`, the implementations are:

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x);
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x), **the main one**: the full firmware builds (`pio run -e stm32h743-devebox`) and runs on the PC (`pio test -e native-stm32`); on the DevEBox board the SD card, the black box, iBUS and the servos have been verified, the sensors not yet;
- `hal/Rtos.h` — FreeRTOS tasks, identical on both platforms.

Everything above works only with the interfaces, so moving to another MCU
means a new `IBoard` implementation, not a rewrite of the sensors.

---

## namespace `ServoChannel`

**File:** `hal/IBoard.h`

Output indices for `IBoard::servo(channel)`. A flat list rather than named
methods — adding an output does not change the `IBoard` interface. The order
matches the rows of the `FlightOutputs::outputInfo()` table.

| Constant | Value |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — payload drop (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — camera (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**File:** `hal/IBoard.h` · **Kind:** interface · **Implementations:** `Esp32Board`, `Stm32Board`

The single entry point to the hardware. Nothing above includes `<Wire.h>`,
`<SPI.h>` or `HardwareSerial`, and nothing calls LEDC directly.

| Method | Description |
|---|---|
| `virtual void begin()` | One-time initialization of the I2C/SPI buses. UARTs are opened by their owners (`IBusReceiver`, GPS) at their own baud rate, PWM by `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | The sensor bus |
| `virtual ISpiBus& spi()` | The SPI bus |
| `virtual II2CBus* displayI2c()` | A second I2C bus for the display only; `nullptr` if there is none |
| `virtual IUartPort& rcUart()` | The UART of the iBUS receiver |
| `virtual IUartPort& gpsUart()` | The GPS UART |
| `virtual IUartPort* telemetryUart()` | The UART of the MAVLink radio modem; `nullptr` by default (the ESP32 has no free UART) |
| `virtual IServoOutput& servo(uint8_t channel)` | A PWM output by `ServoChannel::*` index |
| `virtual void setBuzzer(bool on)` | The `PIN_BUZZER` buzzer; does nothing by default |

---

## `II2CBus`

**File:** `hal/II2CBus.h` · **Kind:** interface with non-virtual helpers ·
**Implementations:** `Esp32I2CBus`, `Stm32I2CBus`

An I2C bus abstraction shaped like `Wire`. The implementation fixes the pins
and the frequency in its constructor, so `begin()`/`setClock()` take no pins —
the bus is initialized exactly once, even if several devices share it.

| Method | Description |
|---|---|
| `begin()`, `setClock(hz)` | Initialization, frequency |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Writing; `endTransmission` returns 0 on success (like `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Reading |
| `bool writeRegister(addr, reg, value)` | Helper: write a single register; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Helper: repeated start + read of `count` bytes. `false` if NACK **or fewer than `count` bytes arrived**; the buffer is left untouched |
| `int readRegister(addr, reg)` | The register value or `-1` |
| `bool probe(addr)` | The device answers the address with ACK |

Invariant: on failure the helpers do not write to the buffer — the driver keeps
the previous data rather than garbage (the `0xFF` that `read()` returns on an
empty buffer).

---

## `ISpiBus`

**File:** `hal/ISpiBus.h` · **Kind:** interface · **Implementations:** `Esp32SpiBus`, `Stm32SpiBus`

An SPI bus **without CS management**: several devices share one bus, and
`SpiRegisterDevice` toggles CS.

| Method | Description |
|---|---|
| `begin()` | Set up SCK/MISO/MOSI (the pins are in the implementation's constructor) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Full-duplex exchange of a byte |
| `endTransaction()` | End of the transaction |

---

## `IUartPort`

**File:** `hal/IUartPort.h` · **Kind:** interface · **Implementations:** `Esp32UartPort`, `Stm32UartPort`

A UART shaped like `HardwareSerial`, but `begin()` takes only the baud rate:
the implementation fixes the pins and the format (8N1).

| Method | Description |
|---|---|
| `begin(baud)` | Open the port |
| `int available()`, `int read()` | Receiving |
| `size_t write(byte)`, `size_t write(buffer, size)` | Transmitting |
| `virtual int availableForWrite()` | Free space in the transmit buffer; `-1` — unknown (the default). Telemetry uses it to postpone a frame rather than wait |

---

## `IServoOutput`

**File:** `hal/IServoOutput.h` · **Kind:** interface · **Implementations:** `Esp32ServoOutput`, `Stm32ServoOutput`

A single PWM output. The implementation fixes the pin.

| Method | Description |
|---|---|
| `bool attach(minUs, maxUs)` | Allocate the channel/timer and configure the pin; the pulse limiting range. `true` only means that the MCU allocated the resources, **not** that a servo is connected |
| `writeMicroseconds(us)` | Pulse width, µs (limited to the `attach` range) |
| `bool isAttached() const` | The result of `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnostics: the real pulse width on the pin or `-1`. The default implementation returns `-1` |

---

## `IFlashRegion`

**File:** `hal/IFlashRegion.h` · **Kind:** interface · **Implementations:** `Esp32FlashPartition`, `SdFileRegion`

A NOR-flash region for the log (the black box): erasing only in 4 KB sectors
(erased bytes read as `0xFF`), writing only clears bits — you can write into
erased bytes, including piecewise into a single page. On the ESP32 both writing
and erasing halt both cores — the caller decides when that is acceptable.

| Method | Description |
|---|---|
| `uint32_t size() const` | Size of the region, bytes; 0 — there is no region |
| `bool read(offset, data, length)` | Read |
| `bool write(offset, data, length)` | Write (into erased bytes) |
| `bool erase(offset, length)` | Erase; the address and length are multiples of 4096 |

`Esp32FlashPartition(const char* name)` — a data partition by name from the
partition table (`esp_partition_*`); `begin()` finds the partition (after the
core has started), and if there is none — `false` and `size() == 0`.

---

## `IBlockDevice`

**File:** `hal/IBlockDevice.h` · **Kind:** interface · **Implementations:** `Stm32SdCard` (in tests — `fake::SdCardModel`)

An SD card as an array of 512-byte blocks. There is no erase: a block can be overwritten.

| Method | Description |
|---|---|
| `uint32_t blockCount() const` | Size in blocks; 0 — there is no card |
| `bool read(block, data, count)` / `write(...)` | `count` consecutive blocks, `data` — any address |

## `SdFileRegion`

**File:** `hal/SdFileRegion.h` · **Inherits:** `IFlashRegion` · **Depends on:** `IBlockDevice`, `Fat32::locate`

A black-box region on the SD card: a file in the root of the FAT32 volume (by
default `BLACKBOX.BIN`), created in advance on the PC as a single contiguous
piece (`tools/blackbox.py
sd-prepare`) and filled with `0xFF`. The file is only **located** (the FAT
tables and the directory are not touched); after that, raw blocks are written
inside it. For `BlackBoxStorage` it is the same `IFlashRegion` as the ESP32
flash partition.

| Method | Description |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` is the ceiling of the region: the sector check at power-on takes longer the larger it is |
| `Fat32::Result begin()` | Find the file. `Ok` — `size() > 0`; otherwise the reason (`Fat32::describe()`): no card, not FAT32, no file, fragmented, empty |
| `size()` | The file (no more than `maxBytes`), rounded down to a 4 KB sector; 0 — there is no region |
| `read` / `write` | Any offset and length. A partial block is read, completed and written whole; the block that was just written is remembered (write-through cache): consecutive 256-byte pages do not read the card again. A power loss does not lose anything that has already returned from `write()` |
| `erase(offset, length)` | A multiple of 4096; writes `0xFF` (the card has its own erase inside, the outside does not need it) |

## `IRegisterDevice`

**File:** `hal/RegisterDevice.h` · **Kind:** interface ·
**Implementations:** `I2cRegisterDevice`, `SpiRegisterDevice`

"A set of 8-bit registers". A sensor driver is written once, and the bus is
chosen when the object is created in `SensorSelection.h`.

| Method | Description |
|---|---|
| `virtual void begin()` | Prepare the device's lines (for SPI — CS). Does nothing by default |
| `virtual bool probe()` | The device responded (for SPI always `true` — there is no ACK, the ID register is checked instead) |
| `virtual bool writeRegister(reg, value)` | Write a register |
| `virtual bool writeRegisters(reg, data, count)` | Write consecutively (address auto-increment) |
| `virtual bool readRegisters(reg, buffer, count)` | Read `count` consecutive bytes; on `false` the buffer is left untouched |
| `int readRegister(reg)` | The value or `-1` (a non-virtual helper) |

---

## `I2cRegisterDevice`

**File:** `hal/RegisterDevice.h` · **Inherits:** `IRegisterDevice`

A device on an `II2CBus` at a 7-bit address. All operations are delegated to
the `II2CBus` helpers.

| Method | Description |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` is the chip's second address (the SDO/SA0 pin): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | the primary does not answer but the alternate does — keep working with the alternate from then on |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → the `II2CBus(address, …)` helpers |
| `uint8_t getAddress() const` | the device's current address |

---

## `SpiRegisterDevice`

**File:** `hal/RegisterDevice.h` · **Inherits:** `IRegisterDevice`

A device on an `ISpiBus` with its own CS pin. The Bosch/InvenSense protocol:
a read is the address with bit `0x80`, a write is with bit 7 cleared.

| Method | Description |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` is how many "junk" bytes the chip outputs after the address before the data (BMP388 — 1, ICM42688 — 0); `mode` is the SPI mode 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Always `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; always `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, skip `dummyReadBytes`, `n` bytes, CS↑; always `true` |

Each operation is a separate `beginTransaction(clockHz, spiMode)` …
`endTransaction()` transaction.

---

## `Esp32Board`

**File:** `hal/esp32/Esp32Board.h` · **Inherits:** `IBoard`

The only place that creates the concrete ESP32 peripheral objects and knows the
pins from `Config.h`.

| Field | Type | What it is |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` on `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` on `PIN_I2C2_SDA/SCL` — only if `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | The global `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS on `PIN_IBUS`, RX only |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS on `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | LEDC channels 0..6 in `ServoChannel` order (AUX1/AUX2 — `PIN_AUX1/2`, if routed) |

| Method | Description |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, then `displayBus.begin()` if the second bus exists; the buzzer pin |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` if the pin is routed |
| `displayI2c()` | `&displayBus` if `hasDisplayBus()`, otherwise `nullptr` |
| `static constexpr bool hasDisplayBus()` | Both pins of the second bus are ≥ 0. It exists (like the `displayBus` field) only when `SOC_I2C_NUM > 1` — the C3 has a single I2C controller |
| the rest | Return the corresponding fields |

---

## `Esp32I2CBus`

**File:** `hal/esp32/Esp32I2CBus.h` · **Inherits:** `II2CBus`

A thin wrapper over `TwoWire` (`Wire` or `Wire1`).

| Method | Description |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Stores the parameters |
| `begin()` | `wire.begin(sda, scl, hz)` and `wire.setTimeOut(TIMEOUT_MS)` — the only call of `wire.begin()` |
| the rest | Direct delegation to `TwoWire` |

`TIMEOUT_MS = 5`: reading 14 IMU bytes at 400 kHz takes ~0.4 ms; a transaction
hung by interference would otherwise stall the loop for the standard 50 ms.

---

## `Esp32SpiBus`

**File:** `hal/esp32/Esp32SpiBus.h` · **Inherits:** `ISpiBus`

A wrapper over the global `SPI`. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS
is held by the devices). `beginTransaction()` builds `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` converts 0..3 to Arduino constants, and an unknown
value → `SPI_MODE0`.

---

## `Esp32UartPort`

**File:** `hal/esp32/Esp32UartPort.h` · **Inherits:** `IUartPort`

A wrapper over `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — receive only. The rest is delegation.

---

## `Esp32ServoOutput`

**File:** `hal/esp32/Esp32ServoOutput.h` · **Inherits:** `IServoOutput`

PWM directly through LEDC (`ledcSetup/ledcAttachPin/ledcWrite` of Arduino core 2.x).
The ESP32Servo library is **not used**: version 3.2.1 on the S3 mixed up the MCPWM
blocks (GPIO6/7 repeated GPIO4/5).

| Constant | Value |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1.2 µs per step) |

| Method | Description |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Pin `< 0` — the output is not routed |
| `attach(minUs, maxUs)` | Stores the range; pin < 0 → `false`; otherwise `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Not attached — nothing; otherwise `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Enables the input buffer of the same GPIO (`PIN_INPUT_ENABLE`, the output is not touched) and measures `pulseIn(pin, HIGH, 30 ms)`; no pulse → `-1` |

Channels 2n and 2n+1 share an LEDC timer — all outputs run at 50 Hz, so there is no conflict.

---

# Implementation for the STM32H743

The next-generation board is the STM32H743VIT6 (Cortex-M7 480 MHz, 2 MB of flash,
1 MB of RAM). The full firmware builds (env `stm32h743` — the PlatformIO board
`weact_mini_h743vitx`, and `stm32h743-devebox` — the DevEBox H743, console over
USB CDC), passes cppcheck and the tests on the PC (env `native-stm32` with the
STM32duino fake layer). On the DevEBox board **without sensors** the following has been verified: boot, the SD card, the black box —
[tests on the board](../TESTING.md#tests-on-the-stm32-board) — and iBUS reception, ARM, PWM to
the servos and the motor: the plane is controlled from the transmitter in manual mode (the launch was filmed).
The sensors have not yet been connected to the board.
The pinout is the `BOARD_STM32H743` block in [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

The general differences from the ESP32 that this layer hides:

- **The core picks the peripherals.** STM32duino finds the controller (I2C1/I2C2,
  SPI2, USART3, UART4, UART7, TIMx) by itself from the pin numbers using the
  variant's `PeripheralPins` tables, so there are no UART/channel numbers in `Config.h`.
- **Pin numbers** are the variant's "Arduino pins" (`PA0`, `PD14`...), not GPIOs; for
  analog pins they are `0xC0 + N`, which is why the pins in the STM32 block are `int16_t`.
- **UART pins** are set when the `Uart(rx, tx)` object is created, not in `begin()`.

## `Stm32Board`

**File:** `hal/stm32/Stm32Board.h` · **Inherits:** `IBoard`

The same as `Esp32Board`, on top of STM32duino.

| Field | Type | What it is |
|---|---|---|
| `displayWire` | `TwoWire` | The second I2C controller (the global `Wire` is taken by the sensors). Declared before `displayBus`, which keeps a reference to it |
| `i2cBus` | `Stm32I2CBus` | `Wire` on `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` on `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) — the second bus is always present |
| `spiBus` | `Stm32SpiBus` | The global `SPI` on `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, reserved for iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | the MAVLink radio modem: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | In `ServoChannel` order (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| Method | Description |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, the buzzer pin |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Always `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Converts a pin from `Config.h` to the core API's type |
| the rest | Return the corresponding fields |

## `Stm32I2CBus`

**File:** `hal/stm32/Stm32I2CBus.h` · **Inherits:** `II2CBus`

| Method | Description |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Stores the parameters |
| `begin()` | `setSDA()`/`setSCL()` (effective only before `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; the `size_t` result is cast to `uint8_t` |
| the rest | Direct delegation to `TwoWire` |

In STM32duino the transaction timeout is not a method but the macro
`I2C_TIMEOUT_TICK` (ms, 100 by default). In the `stm32h743` env it is set with
the flag `-D I2C_TIMEOUT_TICK=5` — for the same reason as `TIMEOUT_MS` in `Esp32I2CBus`.

## `Stm32SpiBus`

**File:** `hal/stm32/Stm32SpiBus.h` · **Inherits:** `ISpiBus`

A wrapper over `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
the hardware NSS is not used — CS is toggled by `SpiRegisterDevice`, as on the ESP32.
`beginTransaction()` builds `SPISettings(hz, MSBFIRST, SPIMode)`; `spiModeOf()`
converts 0..3 to `SPI_MODEn`, and an unknown value → `SPI_MODE0`.

## `Stm32UartPort`

**File:** `hal/stm32/Stm32UartPort.h` · **Inherits:** `IUartPort`

A wrapper over `HardwareSerial&` (in STM32duino 3.x it is the abstract base
`arduino::HardwareSerial`; `Stm32Board` creates the concrete `Uart` object).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` comes
from `HardwareSerial`. The buffers (`SERIAL_RX/TX_BUFFER_SIZE` in the env): receive 256
bytes (a NAV-PVT frame is 100, the standard 64 is too few), transmit 1024 (log lines and
MAVLink frames without waiting).

## `Stm32ServoOutput`

**File:** `hal/stm32/Stm32ServoOutput.h` · **Inherits:** `IServoOutput`

Hardware timer PWM through `HardwareTimer`, 50 Hz. The pulse is generated by the
timer without interrupts and without the CPU — unlike the `Servo` library for STM32,
which toggles the pins from the interrupt of a single timer and produces jitter.

| Constant | Value |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — how many different timers the outputs can occupy (TIM2 and TIM4 are used now) |

| Method | Description |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Pin `< 0` — the output is not routed |
| `attach(minUs, maxUs)` | The timer and channel come from `PinMap_TIM` by pin (`pinmap_peripheral`, `STM_PIN_CHANNEL`), as in `analogWrite()`. No timer on the pin or the pool is exhausted → `false`. Otherwise `setMode(PWM1)`, compare 0 (no pulse until the first write), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. The compare register is preloaded — the value takes effect from the next period |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` without reconfiguring the pin: on the STM32 the IDR register sees the level even in alternate-function mode |
| `static acquireTimer(TIM_TypeDef*)` | A shared pool: **one `HardwareTimer` per TIMx**. A second object for the same timer would overwrite the core's handler (`HardwareTimer_Handle[index]`). The period is set at the first output on the timer; `setOverflow(MICROSEC_FORMAT)` picks the prescaler — a step of ~0.3 µs at a timer clock of 240 MHz |

## `Stm32FlashStorage`

**File:** `hal/stm32/Stm32FlashStorage.h` · **Inherits:** `IFlashStorage` ([storage.md](storage.md))

The `KeyValueStore` medium on the STM32: the last flash sector (bank 2) through the
STM32duino EEPROM emulation (`eeprom_buffer_fill/flush`, an 8 KB buffer of which
the first `KeyValueStore::CAPACITY` bytes are used).

| Method | Description |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + a byte-by-byte read of the buffer |
| `write(src, n)` | **fast**: copies the image into its own buffer under `noInterrupts()`, sets the "pending write" flag. Called from `KvPreferences::end()` in the flight task |
| `bool service()` | **slow**: a snapshot into the emulation buffer (under `noInterrupts()`) and `eeprom_buffer_flush()` — erasing a 128 KB sector (seconds) and writing. Only from the `storage` background task |
| `hasPending()`, `flushCount()` | diagnostics |
| `static instance()`, `static store()` | the medium and the firmware's shared `KeyValueStore` |

Why the flight does not freeze: the settings sector is in bank 2 and the code in
bank 1, the H7 flash can be read from one bank while the other is being written;
the flight task preempts the background one.

## `compat/Preferences.h`

**File:** `hal/stm32/compat/Preferences.h` — in the `stm32h743` env (and
`native-stm32`) the `compat/` directory comes in `-I` before the libraries, and
the `#include <Preferences.h>` of the sensor drivers, the auto-trimmer and the log
settings finds it. `class Preferences : public KvPreferences` on top of
`Stm32FlashStorage::store()` — the same API as the ESP32 NVS ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**File:** `hal/stm32/Stm32SdCard.h` · **Inherits:** `IBlockDevice` · **Pins:** `src/stm32/sd_msp.cpp`

An SD card on SDMMC1: a 4-bit bus, `HAL_SD` in polling mode (no DMA and no
interrupts) **with hardware flow control**: the flight task preempts the write
task in the middle of a block, and without it the FIFO overflowed
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20) — on the board that showed up as the console and
the logging freezing for seconds. The pins PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) are the µSD slot of the
DevEBox and WeAct. The SDMMC core is clocked from PLL1Q = 48 MHz, `ClockDiv = 1` →
**24 MHz**; if the first read fails at 24 MHz, 12 and 6 are tried.

| Member | Description |
|---|---|
| `bool begin()` | Bring up the bus, identify the card, a trial read. `false` — there is no card; `initError()` — the code |
| `read` / `write` | In chunks of at most 4 KB (short pauses); an address that is not a multiple of 4 is copied through an aligned buffer (the HAL reads the FIFO by words). On failure — one retry |
| waiting | Before an access after a write it waits for the card to return to the transfer state (`Rtos::sleepMs(1)`: the background tasks do not starve), up to 1 s. After a read no extra status request is sent — the power-on check reads tens of thousands of sectors |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | For the status line |
| `readOps`, `writeOps`, `errors`, `retries` | Counters |

## `ResetCause`

**File:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

The reset cause, identical on both boards. ESP32 — `esp_reset_reason()`;
STM32 — the `RCC->RSR` flags (read once and cleared; on the H7 `PINRSTF`
is set on any reset, so the more specific causes are checked first:
watchdog → power-on → brown-out → software reset).
A panic, the watchdogs and a power dip count as a "crash": the black box starts
recording at once on them.

## `Rtos`

**File:** `hal/Rtos.h` · namespace

| Member | Description |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | task priorities |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., core 0)`, the stack in bytes; STM32 — `xTaskCreate`, the stack is converted to words; `handle` is for `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`; before the scheduler starts (STM32 `setup()`) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 — the `portMUX` spinlock, STM32 — `taskENTER_CRITICAL()`. Inside only byte copying (the black-box queue) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`; STM32 — `xPortGetFreeHeapSize()` |

## The entry point `src/stm32/main.cpp`

The full firmware: the same objects as `src/main.cpp`, MAVLink telemetry instead of
Wi-Fi, FreeRTOS tasks instead of `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).
