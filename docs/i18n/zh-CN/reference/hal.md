# HAL — 硬件抽象

> 🌐 本页是[俄语原文](../../../reference/hal.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

HAL 是唯一允许了解具体 MCU 的层。接口位于 `include/hal/`，实现有：

- `include/hal/esp32/` — ESP32（Arduino core 2.0.x），**主平台**；
- `include/hal/stm32/` — STM32H743（STM32duino 3.x）：完整固件可以构建（`pio run -e stm32h743`），也可以在 PC 上运行（`pio test -e
  native-stm32`）；在硬件上已验证裸的 DevEBox 板（SD 卡、黑匣子），传感器和舵机尚未验证；
- `hal/Rtos.h` — FreeRTOS 任务，在两个平台上完全一致。

上层只与接口打交道，因此迁移到另一款 MCU，只需要新写一个 `IBoard` 实现，而不必重写传感器。

---

## namespace `ServoChannel`

**文件：** `hal/IBoard.h`

`IBoard::servo(channel)` 的输出索引。这是一个扁平的列表，而不是具名方法——
新增输出不会改变 `IBoard` 接口。顺序与 `FlightOutputs::outputInfo()` 表的行一致。

| 常量 | 值 |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — 投放货物（`Feature::PAYLOAD_DROP`） |
| `AUX2` | 6 — 相机（`Knob::CAMERA_TILT`、`Feature::CAMERA_STAB`） |
| `COUNT` | 7 |

---

## `IBoard`

**文件：** `hal/IBoard.h` · **类别：** 接口 · **实现：** `Esp32Board`、`Stm32Board`

通往硬件的唯一入口。上层没有任何代码包含 `<Wire.h>`、`<SPI.h>`、
`HardwareSerial`，也不直接调用 LEDC。

| 方法 | 说明 |
|---|---|
| `virtual void begin()` | 一次性初始化 I2C/SPI 总线。UART 由各自的所有者（`IBusReceiver`、GPS）以自己的波特率打开，PWM 由 `FlightOutputs::begin()` 打开 |
| `virtual II2CBus& i2c()` | 传感器总线 |
| `virtual ISpiBus& spi()` | SPI 总线 |
| `virtual II2CBus* displayI2c()` | 仅供屏幕使用的第二条 I2C 总线；没有则为 `nullptr` |
| `virtual IUartPort& rcUart()` | iBUS 接收机的 UART |
| `virtual IUartPort& gpsUart()` | GPS 的 UART |
| `virtual IUartPort* telemetryUart()` | MAVLink 无线电模块的 UART；默认为 `nullptr`（ESP32 没有空闲的 UART） |
| `virtual IServoOutput& servo(uint8_t channel)` | 按 `ServoChannel::*` 索引取得的 PWM 输出 |
| `virtual void setBuzzer(bool on)` | 蜂鸣器 `PIN_BUZZER`；默认什么也不做 |

---

## `II2CBus`

**文件：** `hal/II2CBus.h` · **类别：** 带非虚辅助函数的接口 ·
**实现：** `Esp32I2CBus`、`Stm32I2CBus`

仿照 `Wire` 形式的 I2C 总线抽象。引脚和频率由实现在构造函数中固定，因此 `begin()`/`setClock()` 不带引脚——即使总线上有多个设备，总线也只会初始化一次。

| 方法 | 说明 |
|---|---|
| `begin()`、`setClock(hz)` | 初始化、频率 |
| `beginTransmission(addr)`、`write(byte)`、`write(data, len)`、`endTransmission(sendStop = true)` | 写入；`endTransmission` 成功时返回 0（与 `Wire` 相同） |
| `requestFrom(addr, n)`、`available()`、`read()` | 读取 |
| `bool writeRegister(addr, reg, value)` | 辅助函数：写入单个寄存器；`false` 表示 NACK |
| `bool readRegisters(addr, reg, buf, count)` | 辅助函数：重复起始条件 + 读取 `count` 字节。出现 NACK **或收到的字节少于 `count`** 时返回 `false`；此时缓冲区不会被改动 |
| `int readRegister(addr, reg)` | 寄存器的值，或 `-1` |
| `bool probe(addr)` | 设备对该地址回应 ACK |

不变式：失败时辅助函数不会写缓冲区——驱动保留的是之前的数据，而不是垃圾数据（空缓冲区上 `read()` 返回的 `0xFF`）。

---

## `ISpiBus`

**文件：** `hal/ISpiBus.h` · **类别：** 接口 · **实现：** `Esp32SpiBus`、`Stm32SpiBus`

**不管理 CS** 的 SPI 总线：同一条总线上有多个设备，由 `SpiRegisterDevice` 切换 CS。

| 方法 | 说明 |
|---|---|
| `begin()` | 配置 SCK/MISO/MOSI（引脚在实现的构造函数中） |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 为 0..3（CPOL/CPHA） |
| `uint8_t transfer(data)` | 全双工交换一个字节 |
| `endTransaction()` | 结束事务 |

---

## `IUartPort`

**文件：** `hal/IUartPort.h` · **类别：** 接口 · **实现：** `Esp32UartPort`、`Stm32UartPort`

仿照 `HardwareSerial` 的 UART，但 `begin()` 只接收波特率：引脚和格式（8N1）由实现固定。

| 方法 | 说明 |
|---|---|
| `begin(baud)` | 打开端口 |
| `int available()`、`int read()` | 接收 |
| `size_t write(byte)`、`size_t write(buffer, size)` | 发送 |
| `virtual int availableForWrite()` | 发送缓冲区的空闲空间；`-1` 表示未知（默认）。遥测据此推迟发送一帧，而不是等待 |

---

## `IServoOutput`

**文件：** `hal/IServoOutput.h` · **类别：** 接口 · **实现：** `Esp32ServoOutput`、`Stm32ServoOutput`

单个 PWM 输出。引脚由实现固定。

| 方法 | 说明 |
|---|---|
| `bool attach(minUs, maxUs)` | 分配通道/定时器并配置引脚；脉冲限制范围。`true` 只表示 MCU 分配了资源，**并不**表示舵机已连接 |
| `writeMicroseconds(us)` | 脉冲宽度，µs（限制在 `attach` 的范围内） |
| `bool isAttached() const` | `attach()` 的结果 |
| `virtual int32_t measurePulseUs()` | 诊断：引脚上的实际脉冲宽度，或 `-1`。默认实现返回 `-1` |

---

## `IFlashRegion`

**文件：** `hal/IFlashRegion.h` · **类别：** 接口 · **实现：** `Esp32FlashPartition`、`SdFileRegion`

用于日志（黑匣子）的 NOR 闪存区域：擦除只能按 4 KB 扇区进行（擦除后读出
`0xFF`），写入只会把位清零——可以写入已擦除的字节，包括分多次写入同一页。在 ESP32 上，写入和擦除都会让两个内核停下——何时允许这样做，由调用方决定。

| 方法 | 说明 |
|---|---|
| `uint32_t size() const` | 区域大小，字节；0 表示没有该区域 |
| `bool read(offset, data, length)` | 读取 |
| `bool write(offset, data, length)` | 写入（写到已擦除的字节中） |
| `bool erase(offset, length)` | 擦除；地址和长度必须是 4096 的倍数 |

`Esp32FlashPartition(const char* name)` — 按名称从分区表中取得的数据分区（`esp_partition_*`）；`begin()` 负责查找该分区（在内核启动之后），没有该分区则返回 `false`，且 `size() == 0`。

---

## `IBlockDevice`

**文件：** `hal/IBlockDevice.h` · **类别：** 接口 · **实现：** `Stm32SdCard`（测试中为 `fake::SdCardModel`）

把 SD 卡视为由 512 字节块组成的数组。没有擦除操作：块可以直接覆盖重写。

| 方法 | 说明 |
|---|---|
| `uint32_t blockCount() const` | 以块为单位的大小；0 表示没有卡 |
| `bool read(block, data, count)` / `write(...)` | 连续 `count` 个块，`data` 可以是任意地址 |

## `SdFileRegion`

**文件：** `hal/SdFileRegion.h` · **继承：** `IFlashRegion` · **依赖：** `IBlockDevice`、`Fat32::locate`

SD 卡上的黑匣子区域：FAT32 根目录中的一个文件（默认为 `BLACKBOX.BIN`），事先在 PC 上作为一整块连续空间创建（`tools/blackbox.py
sd-prepare`），并用 `0xFF` 填满。程序只是**定位**该文件（不会改动 FAT 表和目录），之后在文件内部写入原始块。对 `BlackBoxStorage` 来说，它与 ESP32 闪存分区是同一个 `IFlashRegion`。

| 方法 | 说明 |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` 是区域的上限：上电时核对扇区的时间随它增长 |
| `Fat32::Result begin()` | 查找文件。`Ok` 表示 `size() > 0`；否则给出原因（`Fat32::describe()`）：没有卡、不是 FAT32、没有该文件、文件不连续、文件为空 |
| `size()` | 文件大小（不超过 `maxBytes`），向下取整到 4 KB 扇区；0 表示没有该区域 |
| `read` / `write` | 任意偏移和长度。不完整的块先读出、补全后整块写入；刚写过的块会被记住（写直达缓存）：连续的 256 字节页不会再次读卡。断电不会丢失任何已从 `write()` 返回的数据 |
| `erase(offset, length)` | 4096 的倍数；写入 `0xFF`（卡内部有自己的擦除机制，外部不需要） |

## `IRegisterDevice`

**文件：** `hal/RegisterDevice.h` · **类别：** 接口 ·
**实现：** `I2cRegisterDevice`、`SpiRegisterDevice`

“一组 8 位寄存器”。传感器驱动只写一次，总线在 `SensorSelection.h` 中创建对象时选择。

| 方法 | 说明 |
|---|---|
| `virtual void begin()` | 准备设备的线路（对 SPI 而言是 CS）。默认什么也不做 |
| `virtual bool probe()` | 设备有应答（对 SPI 始终为 `true`——没有 ACK，改为检查 ID 寄存器） |
| `virtual bool writeRegister(reg, value)` | 写寄存器 |
| `virtual bool writeRegisters(reg, data, count)` | 连续写入（地址自动递增） |
| `virtual bool readRegisters(reg, buffer, count)` | 连续读取 `count` 字节；返回 `false` 时缓冲区不会被改动 |
| `int readRegister(reg)` | 值，或 `-1`（非虚辅助函数） |

---

## `I2cRegisterDevice`

**文件：** `hal/RegisterDevice.h` · **继承：** `IRegisterDevice`

挂在 `II2CBus` 上、使用 7 位地址的设备。所有操作都委托给 `II2CBus` 的辅助函数。

| 方法 | 说明 |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` 是芯片的第二个地址（SDO/SA0 引脚）：LSM6DSV 0x6A/0x6B、ICM-45686 0x68/0x69、SPL06 0x76/0x77、BMP581 0x46/0x47 |
| `begin()` | 主地址无应答而备用地址有应答——此后改用备用地址工作 |
| `probe()`、`writeRegister()`、`writeRegisters()`、`readRegisters()` | → `II2CBus(address, …)` 的辅助函数 |
| `uint8_t getAddress() const` | 设备当前使用的地址 |

---

## `SpiRegisterDevice`

**文件：** `hal/RegisterDevice.h` · **继承：** `IRegisterDevice`

挂在 `ISpiBus` 上、有独立 CS 引脚的设备。Bosch/InvenSense 协议：读取时地址带
`0x80` 位，写入时第 7 位清零。

| 方法 | 说明 |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` 表示芯片在地址之后、数据之前会输出多少个“无用”字节（BMP388 为 1，ICM42688 为 0）；`mode` 为 SPI 模式 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`，CS = HIGH |
| `probe()` | 始终为 `true` |
| `writeRegister(reg, value)` | CS↓、`reg & 0x7F`、`value`、CS↑；始终为 `true` |
| `readRegisters(reg, buf, n)` | CS↓、`reg \| 0x80`、跳过 `dummyReadBytes`、`n` 字节、CS↑；始终为 `true` |

每个操作都是一次独立的 `beginTransaction(clockHz, spiMode)` …
`endTransaction()` 事务。

---

## `Esp32Board`

**文件：** `hal/esp32/Esp32Board.h` · **继承：** `IBoard`

唯一负责创建 ESP32 具体外设对象、并了解 `Config.h` 中各引脚的地方。

| 字段 | 类型 | 含义 |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | 位于 `PIN_I2C_SDA/SCL` 的 `Wire`，400 kHz |
| `displayBus` | `Esp32I2CBus` | 位于 `PIN_I2C2_SDA/SCL` 的 `Wire1`——仅当 `SOC_I2C_NUM > 1` 时存在 |
| `spiBus` | `Esp32SpiBus` | 全局的 `SPI` |
| `rcSerial`、`rcPort` | `HardwareSerial(1)`、`Esp32UartPort` | 位于 `PIN_IBUS` 的 iBUS，仅 RX |
| `gpsSerial`、`gpsPort` | `HardwareSerial(UART_NUM_GPS)`、`Esp32UartPort` | 位于 `PIN_GPS_RX/TX` 的 GPS |
| `servos[7]` | `Esp32ServoOutput` | 按 `ServoChannel` 顺序排列的 LEDC 通道 0..6（AUX1/AUX2 为 `PIN_AUX1/2`，如果已引出） |

| 方法 | 说明 |
|---|---|
| `begin()` | `i2cBus.begin()`、`spiBus.begin()`，如果存在第二条总线则再 `displayBus.begin()`；蜂鸣器引脚 |
| `setBuzzer(on)` | 如果该引脚已引出，则 `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | 若 `hasDisplayBus()` 则为 `&displayBus`，否则为 `nullptr` |
| `static constexpr bool hasDisplayBus()` | 第二条总线的两个引脚都 ≥ 0。它（连同 `displayBus` 字段）仅在 `SOC_I2C_NUM > 1` 时存在——C3 只有一个 I2C 控制器 |
| 其余方法 | 返回对应的字段 |

---

## `Esp32I2CBus`

**文件：** `hal/esp32/Esp32I2CBus.h` · **继承：** `II2CBus`

对 `TwoWire`（`Wire` 或 `Wire1`）的薄封装。

| 方法 | 说明 |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | 保存参数 |
| `begin()` | `wire.begin(sda, scl, hz)` 和 `wire.setTimeOut(TIMEOUT_MS)`——这是 `wire.begin()` 唯一的调用处 |
| 其余方法 | 直接委托给 `TwoWire` |

`TIMEOUT_MS = 5`：在 400 kHz 下读取 IMU 的 14 字节约需 0.4 ms；否则，一次因干扰而挂起的事务会让循环卡住标准的 50 ms。

---

## `Esp32SpiBus`

**文件：** `hal/esp32/Esp32SpiBus.h` · **继承：** `ISpiBus`

对全局 `SPI` 的封装。`begin()` → `SPI.begin(sck, miso, mosi, -1)`（CS 由各设备自己控制）。
`beginTransaction()` 构造 `SPISettings(hz, MSBFIRST,
SPI_MODEn)`；`spiModeOf()` 把 0..3 转换为 Arduino 常量，未知值 → `SPI_MODE0`。

---

## `Esp32UartPort`

**文件：** `hal/esp32/Esp32UartPort.h` · **继承：** `IUartPort`

对 `HardwareSerial` 的封装：`begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`；`tx = -1` 表示只接收。其余都是委托。

---

## `Esp32ServoOutput`

**文件：** `hal/esp32/Esp32ServoOutput.h` · **继承：** `IServoOutput`

直接通过 LEDC 输出 PWM（Arduino core 2.x 的 `ledcSetup/ledcAttachPin/ledcWrite`）。
**不使用** ESP32Servo 库：3.2.1 版本在 S3 上会混淆 MCPWM 模块（GPIO6/7 重复了 GPIO4/5 的输出）。

| 常量 | 值 |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384（每步约 1.2 µs） |

| 方法 | 说明 |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | 引脚 `< 0` 表示该输出未引出 |
| `attach(minUs, maxUs)` | 保存范围；引脚 < 0 → `false`；否则 `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | 未 attach 则什么也不做；否则 `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | 打开同一 GPIO 的输入缓冲（`PIN_INPUT_ENABLE`，不改动输出），并测量 `pulseIn(pin, HIGH, 30 ms)`；没有脉冲 → `-1` |

通道 2n 和 2n+1 共用一个 LEDC 定时器——所有输出都是 50 Hz，因此不会冲突。

---

# STM32H743 的实现

下一代板卡是 STM32H743VIT6（Cortex-M7，480 MHz，2 MB 闪存，
1 MB RAM）。完整固件可以构建（env `stm32h743` 对应 PlatformIO 板卡
`weact_mini_h743vitx`，`stm32h743-devebox` 对应 DevEBox H743，控制台走
USB CDC），并通过 cppcheck 和 PC 上的测试（env `native-stm32`，使用 STM32duino
的仿真层）。在**没有传感器**的 DevEBox 板上已验证：启动、SD 卡、黑匣子——
[板上测试](../TESTING.md#stm32-开发板上的测试)——以及 iBUS 接收、ARM、输出到舵机和电机的 PWM：飞机可以用遥控器以手动模式操控（首飞已录像）。传感器尚未接到板上。引脚分配见 [`Config.h`](config.md#stm32h743vit6board_stm32h743) 中的 `BOARD_STM32H743` 块。

这一层所隐藏的、与 ESP32 的共同差异：

- **外设由内核选择。** STM32duino 会根据变体的 `PeripheralPins` 表中的引脚编号，自行找到对应的控制器（I2C1/I2C2、SPI2、USART3、UART4、UART7、TIMx），所以 `Config.h` 中没有 UART/通道编号。
- **引脚编号**是变体的“Arduino 引脚”（`PA0`、`PD14`……），而不是 GPIO；模拟引脚是 `0xC0 + N`，所以 STM32 块中的引脚类型是 `int16_t`。
- **UART 引脚**在创建 `Uart(rx, tx)` 对象时指定，而不是在 `begin()` 中。

## `Stm32Board`

**文件：** `hal/stm32/Stm32Board.h` · **继承：** `IBoard`

与 `Esp32Board` 相同，只是建立在 STM32duino 之上。

| 字段 | 类型 | 含义 |
|---|---|---|
| `displayWire` | `TwoWire` | 第二个 I2C 控制器（全局 `Wire` 被传感器占用）。声明在 `displayBus` 之前，因为后者保存对它的引用 |
| `i2cBus` | `Stm32I2CBus` | 位于 `PIN_I2C_SDA/SCL`（I2C2：PB11/PB10）的 `Wire`，400 kHz |
| `displayBus` | `Stm32I2CBus` | 位于 `PIN_I2C2_SDA/SCL`（I2C1：PB9/PB8）的 `displayWire`——第二条总线始终存在 |
| `spiBus` | `Stm32SpiBus` | 位于 `PIN_SENSOR_SPI_*`（SPI2）的全局 `SPI` |
| `rcSerial`、`rcPort` | `Uart`、`Stm32UartPort` | iBUS：UART7，RX 为 `PIN_IBUS`（PE7），TX 为 `PIN_IBUS_TX`（PE8，为 iBUS-SENS 预留） |
| `gpsSerial`、`gpsPort` | `Uart`、`Stm32UartPort` | GPS：USART3，`PIN_GPS_RX/TX`（PD9/PD8） |
| `telemetrySerial`、`telemetryPort` | `Uart`、`Stm32UartPort` | MAVLink 无线电模块：UART4，`PIN_TELEM_RX/TX`（PD0/PD1） |
| `servos[7]` | `Stm32ServoOutput` | 按 `ServoChannel` 顺序（AUX1 — PD15/TIM4，AUX2 — PE9/TIM1） |

| 方法 | 说明 |
|---|---|
| `begin()` | `i2cBus.begin()`、`spiBus.begin()`、`displayBus.begin()`、蜂鸣器引脚 |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | 始终为 `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | 把 `Config.h` 中的引脚转换为内核 API 的类型 |
| 其余方法 | 返回对应的字段 |

## `Stm32I2CBus`

**文件：** `hal/stm32/Stm32I2CBus.h` · **继承：** `II2CBus`

| 方法 | 说明 |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | 保存参数 |
| `begin()` | `setSDA()`/`setSCL()`（仅在 `begin()` 之前有效）、`wire.begin()`、`wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`；`size_t` 类型的结果被转换为 `uint8_t` |
| 其余方法 | 直接委托给 `TwoWire` |

在 STM32duino 中，事务超时不是方法，而是宏 `I2C_TIMEOUT_TICK`（单位 ms，默认 100）。在 env `stm32h743` 中，它通过标志 `-D I2C_TIMEOUT_TICK=5` 设定——
原因与 `Esp32I2CBus` 的 `TIMEOUT_MS` 相同。

## `Stm32SpiBus`

**文件：** `hal/stm32/Stm32SpiBus.h` · **继承：** `ISpiBus`

对 `SPIClass&` 的封装。`begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`；不使用硬件 NSS——CS 由 `SpiRegisterDevice` 切换，与 ESP32 上一样。
`beginTransaction()` 构造 `SPISettings(hz, MSBFIRST, SPIMode)`；`spiModeOf()`
把 0..3 转换为 `SPI_MODEn`，未知值 → `SPI_MODE0`。

## `Stm32UartPort`

**文件：** `hal/stm32/Stm32UartPort.h` · **继承：** `IUartPort`

对 `HardwareSerial&` 的封装（在 STM32duino 3.x 中它是抽象基类
`arduino::HardwareSerial`，具体的 `Uart` 对象由 `Stm32Board` 创建）。
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`；`availableForWrite()` 来自
`HardwareSerial`。缓冲区（env 中的 `SERIAL_RX/TX_BUFFER_SIZE`）：接收 256
字节（一个 NAV-PVT 帧为 100 字节，标准的 64 不够），发送 1024（日志行和
MAVLink 帧无需等待）。

## `Stm32ServoOutput`

**文件：** `hal/stm32/Stm32ServoOutput.h` · **继承：** `IServoOutput`

通过 `HardwareTimer` 使用定时器硬件 PWM，50 Hz。脉冲由定时器生成，不需要中断，也不占用 CPU——这与 STM32 的 `Servo` 库不同，后者在单个定时器的中断中翻转各个引脚，会产生抖动。

| 常量 | 值 |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4——各输出最多可以占用多少个不同的定时器（目前占用 TIM2 和 TIM4） |

| 方法 | 说明 |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | 引脚 `< 0` 表示该输出未引出 |
| `attach(minUs, maxUs)` | 定时器和通道按引脚从 `PinMap_TIM` 中取得（`pinmap_peripheral`、`STM_PIN_CHANNEL`），与 `analogWrite()` 相同。引脚上没有定时器或资源池耗尽 → `false`。否则 `setMode(PWM1)`，比较值为 0（第一次写入之前没有脉冲），`resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`。比较寄存器带预装载——新值从下一个周期开始生效 |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)`，不重新配置引脚：在 STM32 上，即使处于复用功能模式，IDR 寄存器也能看到电平 |
| `static acquireTimer(TIM_TypeDef*)` | 共享资源池：**每个 TIMx 只有一个 `HardwareTimer`**。为同一个定时器再建一个对象，会覆盖内核的处理函数（`HardwareTimer_Handle[index]`）。周期在该定时器上的第一个输出处设定；`setOverflow(MICROSEC_FORMAT)` 选取分频系数——定时器时钟为 240 MHz 时，步长约 0.3 µs |

## `Stm32FlashStorage`

**文件：** `hal/stm32/Stm32FlashStorage.h` · **继承：** `IFlashStorage`（[storage.md](storage.md)）

STM32 上 `KeyValueStore` 的存储介质：闪存的最后一个扇区（bank 2），通过
STM32duino 的 EEPROM 仿真（`eeprom_buffer_fill/flush`，缓冲区 8 KB，其中使用前 `KeyValueStore::CAPACITY` 字节）。

| 方法 | 说明 |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + 逐字节读取缓冲区 |
| `write(src, n)` | **快速**：在 `noInterrupts()` 下把映像复制到自己的缓冲区，并置位“有待写入”标志。由飞行任务中的 `KvPreferences::end()` 调用 |
| `bool service()` | **缓慢**：在 `noInterrupts()` 下做快照到仿真缓冲区，并执行 `eeprom_buffer_flush()`——擦除 128 KB 扇区（耗时数秒）并写入。只在后台任务 `storage` 中调用 |
| `hasPending()`、`flushCount()` | 诊断 |
| `static instance()`、`static store()` | 存储介质，以及固件共用的 `KeyValueStore` |

飞行为什么不会卡住：设置扇区在 bank 2，代码在 bank 1，H7 的闪存在写一个 bank 的同时可以读另一个 bank；飞行任务会抢占后台任务。

## `compat/Preferences.h`

**文件：** `hal/stm32/compat/Preferences.h`——在 env `stm32h743`（以及
`native-stm32`）中，`compat/` 目录在 `-I` 中排在各库之前，因此传感器驱动、自动配平器和日志设置中的 `#include <Preferences.h>` 会找到它。`class Preferences : public KvPreferences`
建立在 `Stm32FlashStorage::store()` 之上——API 与 ESP32 的 NVS 相同（[storage.md](storage.md#kvpreferences)）。

## `Stm32SdCard`

**文件：** `hal/stm32/Stm32SdCard.h` · **继承：** `IBlockDevice` · **引脚：** `src/stm32/sd_msp.cpp`

位于 SDMMC1 上的 SD 卡：4 位总线，`HAL_SD` 工作在轮询模式（不使用 DMA 和中断），**并启用硬件流控**：飞行任务会在写入任务写到一个块中途时抢占它，没有流控时 FIFO 会溢出（`HAL_SD_ERROR_RX_OVERRUN`，0x20）——在板子上表现为控制台和记录冻结数秒。引脚 PC8..PC11（D0..D3）、PC12（CK）、PD2（CMD）是 DevEBox 和 WeAct 的
µSD 插槽。SDMMC 内核的时钟来自 PLL1Q = 48 MHz，`ClockDiv = 1` →
**24 MHz**；如果在 24 MHz 下第一次读取失败，则依次尝试 12 和 6。

| 成员 | 说明 |
|---|---|
| `bool begin()` | 启动总线、识别卡、试读。`false` 表示没有卡；`initError()` 给出错误码 |
| `read` / `write` | 每次最多 4 KB（带短暂停顿）；地址不是 4 的倍数时，通过对齐的缓冲区复制（HAL 按字读取 FIFO）。失败时重试一次 |
| 等待 | 写入之后再次访问之前，会等待卡回到传输状态（`Rtos::sleepMs(1)`：后台任务不会被饿死），最长 1 s。读取之后不会多发状态请求——上电时的核对要读取数万个扇区 |
| `blockCount()`、`cardType()`、`clockDivider()`、`lastErrorCode()` | 用于状态行 |
| `readOps`、`writeOps`、`errors`、`retries` | 计数器 |

## `ResetCause`

**文件：** `hal/ResetCause.h` · `readResetCause()`、`isCrashReset()`、`resetCauseName()`

重启原因，两块板上一致。ESP32 用 `esp_reset_reason()`；
STM32 用 `RCC->RSR` 标志（读取一次后即清除；H7 上 `PINRSTF`
在任何复位时都会置位，所以先检查更具体的原因：看门狗 → 上电 → 欠压 → 软件复位）。崩溃（panic）、看门狗和电源跌落都算作“故障”：黑匣子遇到这些情况会立即开始记录。

## `Rtos`

**文件：** `hal/Rtos.h` · namespace

| 成员 | 说明 |
|---|---|
| `PRIORITY_BACKGROUND`（1）、`PRIORITY_TELEMETRY`（2）、`PRIORITY_FLIGHT`（5） | 任务优先级 |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., 内核 0)`，栈以字节计；STM32 — `xTaskCreate`，栈换算为字；`handle` 用于 `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`；调度器启动之前（STM32 的 `setup()`）为 `delay()` |
| `class CriticalSection` | `enter()`/`exit()`：ESP32 — `portMUX` 自旋锁，STM32 — `taskENTER_CRITICAL()`。内部只做字节复制（黑匣子队列） |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`；STM32 — `xPortGetFreeHeapSize()` |

## 入口点 `src/stm32/main.cpp`

完整固件：与 `src/main.cpp` 相同的对象，用 MAVLink 遥测取代 Wi-Fi，用 FreeRTOS 任务取代 `loop()`——见 [application.md](application.md#srcstm32maincpp--stm32h743)。
