# HAL — абстракция железа

[← Справочник](README.md)

HAL — единственный слой, которому разрешено знать конкретный MCU. Интерфейсы
лежат в `include/hal/`, единственная реализация — ESP32 (Arduino core 2.0.x) в
`include/hal/esp32/`. Всё выше работает только с интерфейсами, поэтому перенос
на другой MCU — это новая реализация `IBoard`, а не переписывание датчиков.

---

## namespace `ServoChannel`

**Файл:** `hal/IBoard.h`

Индексы выходов для `IBoard::servo(channel)`. Плоский список, а не именованные
методы — добавление выхода не меняет интерфейс `IBoard`. Порядок совпадает со
строками таблицы `FlightOutputs::outputInfo()`.

| Константа | Значение |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `COUNT` | 5 |

---

## `IBoard`

**Файл:** `hal/IBoard.h` · **Вид:** интерфейс · **Реализации:** `Esp32Board`

Единственная точка входа в железо. Ничего выше не включает `<Wire.h>`,
`<SPI.h>`, `HardwareSerial` и не вызывает LEDC напрямую.

| Метод | Описание |
|---|---|
| `virtual void begin()` | Разовая инициализация шин I2C/SPI. UART открывают их владельцы (`IBusReceiver`, GPS) со своей скоростью, PWM — `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | Шина датчиков |
| `virtual ISpiBus& spi()` | Шина SPI |
| `virtual II2CBus* displayI2c()` | Вторая шина I2C только для экрана; `nullptr`, если её нет |
| `virtual IUartPort& rcUart()` | UART приёмника iBUS |
| `virtual IUartPort& gpsUart()` | UART GPS |
| `virtual IServoOutput& servo(uint8_t channel)` | PWM-выход по индексу `ServoChannel::*` |

---

## `II2CBus`

**Файл:** `hal/II2CBus.h` · **Вид:** интерфейс с невиртуальными помощниками ·
**Реализации:** `Esp32I2CBus`

Абстракция шины I2C в форме `Wire`. Пины и частота фиксируются реализацией в
конструкторе, поэтому `begin()`/`setClock()` без пинов — шина инициализируется
ровно один раз, даже если на ней несколько устройств.

| Метод | Описание |
|---|---|
| `begin()`, `setClock(hz)` | Инициализация, частота |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Запись; `endTransmission` возвращает 0 при успехе (как `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Чтение |
| `bool writeRegister(addr, reg, value)` | Помощник: запись одного регистра; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Помощник: повторный старт + чтение `count` байт. `false`, если NACK **или пришло меньше `count` байт**; буфер при этом не трогается |
| `int readRegister(addr, reg)` | Значение регистра или `-1` |
| `bool probe(addr)` | Устройство отвечает ACK на адрес |

Инвариант: при неудаче помощники не пишут в буфер — драйвер оставляет прошлые
данные, а не мусор (`0xFF` от `read()` на пустом буфере).

---

## `ISpiBus`

**Файл:** `hal/ISpiBus.h` · **Вид:** интерфейс · **Реализации:** `Esp32SpiBus`

Шина SPI **без управления CS**: на одной шине несколько устройств, CS
переключает `SpiRegisterDevice`.

| Метод | Описание |
|---|---|
| `begin()` | Настроить SCK/MISO/MOSI (пины — в конструкторе реализации) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Полнодуплексный обмен байтом |
| `endTransaction()` | Конец транзакции |

---

## `IUartPort`

**Файл:** `hal/IUartPort.h` · **Вид:** интерфейс · **Реализации:** `Esp32UartPort`

UART в форме `HardwareSerial`, но `begin()` берёт только скорость: пины и
формат (8N1) фиксирует реализация.

| Метод | Описание |
|---|---|
| `begin(baud)` | Открыть порт |
| `int available()`, `int read()` | Приём |
| `size_t write(byte)`, `size_t write(buffer, size)` | Передача |

---

## `IServoOutput`

**Файл:** `hal/IServoOutput.h` · **Вид:** интерфейс · **Реализации:** `Esp32ServoOutput`

Один PWM-выход. Пин фиксируется реализацией.

| Метод | Описание |
|---|---|
| `bool attach(minUs, maxUs)` | Выделить канал/таймер и настроить пин; диапазон ограничения импульса. `true` отражает только то, что MCU выделил ресурсы, а **не** что серво подключено |
| `writeMicroseconds(us)` | Ширина импульса, мкс (ограничивается диапазоном `attach`) |
| `bool isAttached() const` | Результат `attach()` |
| `virtual int32_t measurePulseUs()` | Диагностика: реальная ширина импульса на пине или `-1`. Реализация по умолчанию возвращает `-1` |

---

## `IRegisterDevice`

**Файл:** `hal/RegisterDevice.h` · **Вид:** интерфейс ·
**Реализации:** `I2cRegisterDevice`, `SpiRegisterDevice`

«Набор 8-битных регистров». Драйвер датчика пишется один раз, шина выбирается
при создании объекта в `SensorSelection.h`.

| Метод | Описание |
|---|---|
| `virtual void begin()` | Подготовить линии устройства (для SPI — CS). По умолчанию ничего |
| `virtual bool probe()` | Устройство отозвалось (для SPI всегда `true` — ACK нет, проверяется ID-регистр) |
| `virtual bool writeRegister(reg, value)` | Запись регистра |
| `virtual bool readRegisters(reg, buffer, count)` | Чтение `count` байт подряд; при `false` буфер не трогается |
| `int readRegister(reg)` | Значение или `-1` (невиртуальный помощник) |

---

## `I2cRegisterDevice`

**Файл:** `hal/RegisterDevice.h` · **Наследует:** `IRegisterDevice`

Устройство на `II2CBus` по 7-битному адресу. Все операции делегируются
помощникам `II2CBus`.

| Метод | Описание |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress)` | Конструктор |
| `probe()`, `writeRegister()`, `readRegisters()` | → `bus.probe/writeRegister/readRegisters(address, …)` |
| `uint8_t getAddress() const` | Адрес устройства |

---

## `SpiRegisterDevice`

**Файл:** `hal/RegisterDevice.h` · **Наследует:** `IRegisterDevice`

Устройство на `ISpiBus` со своим пином CS. Протокол Bosch/InvenSense: чтение —
адрес с битом `0x80`, запись — со сброшенным битом 7.

| Метод | Описание |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 МГц, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` — сколько «мусорных» байт чип отдаёт после адреса перед данными (BMP388 — 1, ICM42688 — 0); `mode` — режим SPI 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Всегда `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; всегда `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg | 0x80`, пропуск `dummyReadBytes`, `n` байт, CS↑; всегда `true` |

Каждая операция — отдельная транзакция `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Файл:** `hal/esp32/Esp32Board.h` · **Наследует:** `IBoard`

Единственное место, которое создаёт конкретные периферийные объекты ESP32 и
знает пины из `Config.h`.

| Поле | Тип | Что это |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` на `PIN_I2C_SDA/SCL`, 400 кГц |
| `displayBus` | `Esp32I2CBus` | `Wire1` на `PIN_I2C2_SDA/SCL` — только если `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | Глобальный `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS на `PIN_IBUS`, только RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS на `PIN_GPS_RX/TX` |
| `servos[5]` | `Esp32ServoOutput` | Каналы LEDC 0..4 в порядке `ServoChannel` |

| Метод | Описание |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, затем `displayBus.begin()`, если вторая шина есть |
| `displayI2c()` | `&displayBus`, если `hasDisplayBus()`, иначе `nullptr` |
| `static constexpr bool hasDisplayBus()` | Оба пина второй шины ≥ 0. Существует (как и поле `displayBus`) только при `SOC_I2C_NUM > 1` — у C3 один контроллер I2C |
| остальные | Возвращают соответствующие поля |

---

## `Esp32I2CBus`

**Файл:** `hal/esp32/Esp32I2CBus.h` · **Наследует:** `II2CBus`

Тонкая обёртка над `TwoWire` (`Wire` или `Wire1`).

| Метод | Описание |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Запоминает параметры |
| `begin()` | `wire.begin(sda, scl, hz)` и `wire.setTimeOut(TIMEOUT_MS)` — единственный вызов `wire.begin()` |
| остальные | Прямое делегирование `TwoWire` |

`TIMEOUT_MS = 5`: чтение 14 байт IMU на 400 кГц занимает ~0.4 мс; зависшая из-за
помехи транзакция иначе стопорила бы цикл на штатные 50 мс.

---

## `Esp32SpiBus`

**Файл:** `hal/esp32/Esp32SpiBus.h` · **Наследует:** `ISpiBus`

Обёртка над глобальным `SPI`. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS
держат устройства). `beginTransaction()` строит `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` переводит 0..3 в константы Arduino, неизвестное
значение → `SPI_MODE0`.

---

## `Esp32UartPort`

**Файл:** `hal/esp32/Esp32UartPort.h` · **Наследует:** `IUartPort`

Обёртка над `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — только приём. Остальное — делегирование.

---

## `Esp32ServoOutput`

**Файл:** `hal/esp32/Esp32ServoOutput.h` · **Наследует:** `IServoOutput`

PWM напрямую через LEDC (`ledcSetup/ledcAttachPin/ledcWrite` Arduino core 2.x).
Библиотека ESP32Servo **не используется**: версия 3.2.1 на S3 путала блоки MCPWM
(GPIO6/7 повторяли GPIO4/5).

| Константа | Значение |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1.2 мкс на шаг) |

| Метод | Описание |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Пин `< 0` — выход не разведён |
| `attach(minUs, maxUs)` | Запоминает диапазон; пин < 0 → `false`; иначе `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Не attached — ничего; иначе `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Включает входной буфер того же GPIO (`PIN_INPUT_ENABLE`, выход не трогается) и меряет `pulseIn(pin, HIGH, 30 мс)`; нет импульса → `-1` |

Каналы 2n и 2n+1 делят таймер LEDC — у всех выходов 50 Гц, конфликта нет.
