# TESTING.md——测试、覆盖率与静态分析

> 🌐 本页是[俄语原文](../../TESTING.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

固件在两个层面上接受验证：

| 位置 | 命令 | 内容 |
|---|---|---|
| **电脑（native）** | `pio test -e native` | 固件头文件原封不动地在电脑上编译，硬件被可控的模拟替身（fake）取代：模块、驱动、闭环飞行仿真，以及搭配每一种传感器套件的整套 ESP32 固件（S3 和 38 针版）。统计覆盖率 |
| **电脑（native-stm32）** | `pio test -e native-stm32` | 整套 STM32H743 固件（`src/stm32/main.cpp`）运行在 STM32duino 模拟替身层之上：FreeRTOS 任务、闪存、MAVLink、I2C 和 SPI 上的传感器 |
| **构建矩阵** | `tools/build_matrix.sh` | 4 种开发板 × 6 种传感器套件，使用 `-Wall -Wextra (-Wshadow)`；项目代码中的任何警告都算错误 |
| **开发板** | `pio test -e esp32-s3` | 在真实的 ESP32-S3 上运行 `test_feedback` 和 `test_imu_orientation`（会烧录测试固件；之后请换回正常固件：`pio run -t upload`） |
| **STM32 开发板** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | 在 DevEBox H743 上用**真实的 SD 卡**测试黑匣子，另外在 Cortex-M7 上运行 `test_feedback` 和 `test_imu_orientation`——见[下文](#stm32-开发板上的测试) |

架构方面的背景见 [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-可测试性)。

---

## 快速开始

```bash
pip install platformio gcovr        # 只需一次
# Windows：需要 PATH 中有 g++，例如 WinLibs（winlibs.com，zip UCRT）：
# 解压后把 mingw64\bin 加入 PATH——无需安装
pio test -e native -e native-stm32  # 全部原生测试（约 1.5 分钟）
gcovr                               # 按文件统计覆盖率（设置见 gcovr.cfg）
tools/build_matrix.sh               # 全部开发板 × 全部传感器（约 25 分钟）
gcovr --html-details -o coverage/index.html   # HTML 报告（coverage/ 已在 .gitignore 中）

pio test -e native -f native/test_rc          # 单个测试集
pio test -e native -f test_feedback           # 在电脑上仿真反馈

# 闭环仿真的轨迹，导出为 CSV（用来画图）：
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# MAVLink 数据流——交给参考解码器检查（pip install pymavlink）：
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

修改测试之后要统计覆盖率时，最好从干净的构建开始：`rm -rf .pio/build/native`，否则以前运行留下的计数会混进报告。

---

## 原生构建的结构

`platformio.ini` 中的 `[env:native]`：`platform = native`，Unity，`-std=gnu++17`，`-D BOARD_ESP32_S3`（主力板的引脚分配），`-I test/native/support`，`-Wall -Wextra -Wshadow`，用 `--coverage` 统计覆盖率，另有 `-fkeep-inline-functions -fkeep-static-functions`——没有它们，gcov 看不到从未被调用过的头文件函数，会高估覆盖率。

### 硬件模拟替身——`test/native/support/`

这些头文件的名称和签名与 ESP32 的 Arduino core 2.0.x、ESP-IDF 及各个库相同，但底层是 `namespace fake` 中的模拟世界：

| 文件 | 替代对象 | 模拟能做什么 |
|---|---|---|
| `Arduino.h`、`Print.h`、`WString.h`、`Stream.h` | Arduino core | 各种宏（`constrain`、`sq`、`DEG_TO_RAD`……）、`map()`、`String`、与原版一致的 `print()` 格式化。`ARDUINO` 有意**不**定义 |
| `esp32-hal-fake.h` | 时间、GPIO、ADC、LEDC、FreeRTOS、PSRAM、`ESP` | 时钟只会随 `fake::advance*()`/`delay()` 前进；`millis()/micros()` 是 `uint32_t`，与 ESP32 一致（溢出行为与开发板相同）。LEDC 通道、按真实占空比工作的 `pulseIn`（只有在引脚的输入缓冲打开时才可见）、`analogReadMilliVolts`——电压取自 `fake::gpio().analogMv`。任务会被登记（句柄非空）；`fake::runTask(task, n)` 让它的无限循环执行 n 轮，`ulTaskNotifyTake` 算一轮，`xTaskNotifyGive` 是一个计数器。FreeRTOS 互斥量是一个“占用”标志。`psramFound()`/`ps_malloc()`。临界区会被计数 |
| `HardwareSerial.h` | UART | 端口按编号登记（`fake::uart(1)`）；`pushRx()`、`txBytes()`、运行中改变速率（`updateBaudRate`，历史记录是 `baudChanges()`）。`Serial` = UART0 |
| `esp_partition.h` | ESP-IDF 的闪存分区 | 分区是一个字节向量，行为类似 NOR：只能按 4 KB 扇区擦除，擦除后为 0xFF，写入只能把位拉低（试图把位拉高会被计数——`bitRaises`）；`beforeWrite`——“断电了”；读取、写入、擦除的计数器 |
| `esp_system.h` | 重启原因 | 来自 `fake::chip().resetReason` 的 `esp_reset_reason()` |
| `Wire.h` | I2C | 按地址区分的设备；`fake::RegisterMapDevice`——带自动递增的寄存器、写入日志、故障（`present`、`failWrites`、`failReads`、`failReadIf`、`shortRead`）、钩子 `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | 按 CS 引脚区分的设备；`fake::SpiRegisterMapDevice`——Bosch/InvenSense 协议，数据之前有 `dummyBytes` |
| `Preferences.h` | NVS | 内存中的存储，`begin(readOnly)`/`get*`/`getBytes` 的行为与原版一致；`failBegin` |
| `WiFi.h`、`WebServer.h` | Wi-Fi 热点、HTTP | `softAP()` 的结果由测试指定；`WebServer::request(方法, uri, 请求体)` 调用已注册的处理函数；`fake::webServers()`——所有实例 |
| `U8g2lib.h` | U8g2 | 记录的不是像素，而是画出的字符串和矩形的列表；`begin()/sendBuffer()` 通过用户字节回调传送字节；`fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`、`GPIO_PIN_MUX_REG`、`PIN_INPUT_ENABLE` |

### STM32duino 层——`test/native/support_stm32/`（环境 `native-stm32`）

它在 `-I` 中排在 `support/` 之前，为同样的模拟替身补充只有 STM32duino 才有的东西。在这个环境中，`<Preferences.h>` 是**真实的** `include/hal/stm32/compat/Preferences.h`，建立在 `KeyValueStore` 之上。

| 文件 | 替代对象 | 能做什么 |
|---|---|---|
| `Arduino.h` | STM32duino core | 引脚 `PA0..PE15`（端口·16 + 编号）、`pin_size_t`、舵机输出引脚用的 `PinMap_TIM`、`HardwareTimer`（脉冲可通过 `fake::timerPulseUs(pin)` 和 `pulseIn()` 看到）、`Uart`、`noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino 的 FreeRTOS | `xTaskCreate` 登记到公共任务注册表（栈以字为单位），`vTaskStartScheduler()` 会返回——由测试自己驱动任务（`fake::runTask`），`xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM 仿真 | 8 KB 的“闪存”（擦除后为 0xFF）和一个缓冲区，`fake::eeprom()`——计数器和镜像损坏 |
| `SPI.h` | | `SPIMode` |

为 STM32 在公共模拟替身中新增了：`TwoWire(sda, scl)`、`setSDA/SCL` 和 `fake::wireWithSda(pin)`（用来找到开发板的第二条总线）、`HardwareSerial(rx, tx)` 和 `fake::uartByRx(pin)`、`SPIClass::setSCLK/MISO/MOSI`。

### 芯片模拟器与飞机模型——`test/native/helpers/`

| 文件 | 说明 |
|---|---|
| `ChipEmulators.h` | LSM6DSV、ICM-45686（带 IPREG 间接寄存器）、QMC6309、SPL06-001、BMP581，以及 u-blox 的 NAV-PVT 帧——I2C 或 SPI 上的寄存器映射，数据取自 `World`“世界”（角度和角速度、高度、空速、航向、坐标），按芯片自身的坐标轴给出，并考虑了 `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | 约 1.2 kg 的飞机模型：质点 + 横滚/俯仰转动，带失速的 CL(α)，阻力、推力、风、热气流、地面 |
| `SimHarness.h` | 闭环：遥控器 → iBUS 帧 → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → 舵面偏转 → `PlaneSim` → 传感器（包括基于两个带噪声气压计的皮托管）。设置 `OPENPLANE_SIM_DIR` 时输出 CSV 轨迹 |

`test/native/helpers/TestSupport.h` 是各测试集共用的部分：`resetWorld()`（在 `setUp()` 中调用）、`FakeUart`/`FakeServo`/`FakeBoard` 以及各传感器（`FakeImu`、`FakeBaro`、`FakeMag`、`FakeGps`）的替身、帧构造器 `ibusFrame()`、台架 `I2cRig`/`SpiRig`（真实的 `Esp32I2CBus`/`Esp32SpiBus` 和带模拟芯片的 `*RegisterDevice` 之上的驱动）。

开发板上的测试（`test_feedback`、`test_imu_orientation`）是可移植的：定义了 `ARDUINO` 时使用 `setup()/loop()`，否则使用 `main()`。`test/native/*` 的测试集不会为开发板构建（`[esp32_common]` 和 `[env:stm32h743]` 中的 `test_ignore`：每个模式单独占一行——用空格隔开的话，PlatformIO 会把它们当成一个）。在 STM32 上：`pio test -e stm32h743`。

---

## 测试集

| 测试集 | 测试数 | 检查内容 |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus` 的辅助函数（NACK、短读取——缓冲区保持不变）、`I2cRegisterDevice`、`SpiRegisterDevice`（读位、BMP388 的哑字节）、`Esp32I2CBus`（5 ms 超时）、`Esp32SpiBus`（模式 0–3）、`Esp32UartPort`（8N1、引脚）、`Esp32ServoOutput`（50 Hz/14 位、脉宽限制、LEDC 故障、通过输入缓冲区测量）、`Esp32Board`（总线、UART、通道顺序、AUX、蜂鸣器） |
| `native/test_rc` | 16 | `RcChannelState`、`RcInput`，iBUS 解析：分段到达的帧、CRC、12 位数值、遥控器的失控保护、500 ms 超时（包括 `micros()` 溢出时）、乱码数据、重新同步 |
| `native/test_control` | 21 | 襟翼（速度、首次调用、暂停）、混控器（符号、反向、襟副翼）、油门、ARM 状态机与模式传感器检查、输出表与脉冲自检 |
| `native/test_autopilot` | 22 | PID（取自传感器角速度的 D 项、积分、抗积分饱和、`dt`）、作为角度模式的 STABILIZE、按时间的自动起飞、用升降舵实现的 ALT_HOLD、信号丢失时的滑翔 |
| `native/test_autopilot_modes` | 31 | 全部 12 种模式及各自对传感器缺失的反应、绑定表与 `static_assert`、功能与旋钮、导航（航向、绕圈、返航点、地理围栏）、failsafe RTH/滑翔、手抛起飞、翱翔、自动配平（仅在地面写入） |
| `native/test_flight_controller` | 12 | 在真实的类上完整运行一个 `FlightController` 控制周期：优先级为 信号丢失 > ARM > 摇杆/自动驾驶仪 > 油门；AUX、`MOTOR_KILL`、蜂鸣器 |
| `native/test_imu` | 21 | MPU6050/6500/9250 与 ICM-42688：识别、寄存器、比例系数、轴旋转与航空坐标符号约定、总线错误、陀螺仪校准与飞行前检查、基于三个姿态的安装校准、NVS、姿态滤波器 |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`、通过 I2C 和 SPI 的 BMP388、对照 Bosch 参考算法的 BME280/BMP280、电子罗盘（航向、NVS 中的 hard-iron 校准）、u-blox M10（CFG-VALSET、NAV-PVT、损坏的帧、超时）、`SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV（16X/32X、备用地址、SPI）、ICM-45686（间接寄存器）、QMC6309、SPL06-001（数据手册公式）、BMP581（DRDY 与备用路径）、皮托管（零点、滤波、空气密度、软管接反、过期数据、带两个气压计噪声的“飞行”） |
| `native/test_storage` | 16 | `KeyValueStore`（重新加载、磨损——相同的值不重复写入、溢出不丢数据、CRC、擦除时断电、乱码数据、格式版本）、`KvPreferences`（行为与 ESP32 的 NVS 相同） |
| `native/test_mavlink` | 20 | 编解码器对照 pymavlink 的参考帧（v1、v2、带签名）、CRC、重新同步；遥测：各数据流的频率、HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS、PID 参数（列表、读取、写入、拒绝非法值）、从地面切换模式、从地面 ARM——被拒绝、任务——0、UART 缓冲区写满也不会阻塞循环 |
| `native/test_blackbox` | 19 | 黑匣子：格式与 CRC，NOR 模拟件上的扇区环（全新分区无需擦除、垃圾数据总会被擦除、旧飞行记录整段擦除且仅为腾出空间、最新一次飞行从不触碰、环尾回绕、重启后的头指针、断电、写了一半的记录由 CRC 识别），在真实的 `FlightController`/`Autopilot` 上记录飞行：按 ARM 和油门启动并带预录、DISARM 且“停在地面”后停止、信号丢失不会停止记录、异常重启后的记录、手动启动、事件、电池、空中闪存写满、飞行时长超过分区、按带 CRC 的帧导出并切换速率、控制台菜单 `k`、没有分区——黑匣子关闭 |
| `native/test_blackbox_scan` | 3 | 开机时环的抽样核对与完整核对的对比：300 段随机的环历史 × 5 个探测步（头指针、编号和飞行列表一致；当情况对不上时则退回完整核对），以及在 64 MB SD 区域上的开销（约 530 次读取，而不是 32,000 次） |
| `native/test_telemetry` | 28 | `LoopStats`、`LogSettings`（NVS、版本）、`DebugLogger`（所有通道、NAV）、`DebugConsole`（菜单、热键、总线探测 `b`、ARM 时禁用、仅在未 ARM 时保存）、`WebDebugServer`（路由、JSON、邮箱）、`OledDisplay`（通过 I2C 发送的字节、帧、信号丢失时反色） |
| `native/test_sim` | 15 | 整套固件与飞机模型的闭环飞行：从倾斜姿态改平、侧风中的 CRUISE、LOITER、RTH、failsafe RTH/滑翔、地理围栏、从跑道自动起飞、手抛起飞、自动降落、热气流、从螺旋中 RESCUE、速度保持与失速保护、回路中的真实皮托管、“歪”飞机的自动配平、飞行中的传感器故障（IMU、气压计、皮托管、GPS） |
| `native/test_feedback_units` | 14 | 各反馈模块的单独测试：速度来源、空中/地面判断、RLS 估计、控制器、失速征兆、起飞/降落中止 |
| `native/test_app` | 10 | 在 ESP32-S3 上运行 `src/main.cpp`，配台架套件 MPU6500/BMP388/QMC5883P/OLED：`loop()` 周期、遥控器 → 舵机、ARM、模式、信号丢失、控制台、仪表盘、屏幕、黑匣子（运行在核心 0 上的任务、按油门记录、DISARM 之后的飞行、`bb list`） |
| `native/test_app_lsm6dsv_pitot` | 9 | 在 ESP32-S3 上运行 `src/main.cpp`，配飞行套件：LSM6DSV + QMC6309 + SPL06 + 皮托管内的 BMP581 + GPS——识别所有芯片、皮托管零点与速度、高度、按 GPS 确定的返航点、按芯片给出的角度做 STABILIZE、RTH 返回返航点、总线探测、仪表盘 |
| `native/test_app_icm45686_esp32dev` | 5 | 在 **ESP32 38 针版**（`BOARD_ESP32_CLASSIC`）上运行 `src/main.cpp`，配 ICM-45686 + QMC6309 + SPL06 + BMP581 套件：开发板引脚分配、IPREG 滤波器、手抛起飞、增稳与速度、仅探测一条总线 |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | 在 **STM32H743** 上运行 `src/stm32/main.cpp`，配飞行套件：任务与优先级、2 ms 周期、皮托管、PWM 定时器与 `pulseIn`、飞行中的 MAVLink、从 GCS 切换模式、由后台任务把设置写入“闪存”、I2C1 上的屏幕、控制台 |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 配 **SPI 接口**的 ICM-45686 和 BMP581 + QMC6309：开机时闪存已损坏、从 GCS 发起的 ALT_HOLD 保持高度、信号丢失 → RTH、在 MAVLink 中可见；改写损坏的映像 |
| `native_stm32/test_blackbox_sd` | 29 | SD 卡上的黑匣子：FAT32（有 MBR 与无 MBR、占两个簇的目录、噪声条目、他人的/碎片化的/空的卷）、`SdFileRegion`（不完整的块、缓存、擦除、边界、故障）、基于模拟 `HAL_SD` 的真实 `Stm32SdCard` 驱动（4 位、回退速率、重试、卡忙、未对齐的缓冲区）、卡上的环（重启、断电、核对开销）、“环为空”标记、在 `FlightController` 上记录飞行、通过 `RCC->RSR` 识别的异常重启、电池 ADC、飞行中的卡错误、慢速卡、通过控制台导出、`D` 键 |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | 带卡的 `src/stm32/main.cpp`：启动时能找到卡和文件、`bbox` 任务记录飞行、循环周期不被拉长、`bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | 不带卡的 `src/stm32/main.cpp`：黑匣子关闭并说明原因、飞机照常飞行、菜单 `k` 不出错 |
| `test_feedback` | 10 | 带反馈回路的飞机闭环仿真（在电脑和开发板上） |
| `test_imu_orientation` | 5 | 在 300 种随机安装方式下校准 IMU 的安装姿态（在电脑和开发板上） |
| **合计** | **387** | `native` 中 340 个 + `native-stm32` 中 47 个（另有 9 个仅在开发板上运行——`test_blackbox_sd`） |

### STM32 开发板上的测试

`test/test_blackbox_sd` 不是原生测试：SDMMC 驱动、SD 卡和时间都是真实的。测试在一个 FreeRTOS 任务中运行，旁边还运行着一个最高优先级的飞行循环模拟任务（周期 2 ms）：它会在访问 SD 卡的过程中抢占测试，与固件中的情形一致。没有它，就抓不到在开发板上真正发现的那个错误：抢占时 SDMMC 的 FIFO 会溢出（`HAL_SD_ERROR_RX_OVERRUN`），而在空转的循环里不会出现。

| 测试 | 检查内容 |
|---|---|
| `reset_cause_is_a_normal_one` | 复位原因（`RCC->RSR`）既不是看门狗，也不是电压跌落 |
| `card_is_detected_on_four_bit_bus` | 卡在 4 位总线、24 MHz 下被识别 |
| `file_is_found_and_contiguous` | 在 FAT32 上找到 `BLACKBOX.BIN`，且文件是连续的 |
| `multi_block_writes_work_at_every_length` | 一次访问写入 1、2、4、8 个块 |
| `pages_write_with_bounded_latency_and_read_back_intact` | 256 B 页：最差写入 < 250 ms（SD 的上限），持续速率 > 40 KB/s，读取与擦除 |
| `header_scan_cost_on_the_whole_area` | 读取扇区头和完整核对的开销 |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | 全部擦除、写入两次各 20,000 条记录的飞行、“重启”：抽样核对 < 2 s，记录按顺序读出且 CRC 正确；空环凭标记在 < 100 ms 内识别 |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | 真实的 `BlackBox` 在 500 Hz IMU 下实时运行：一条记录都不丢，“重启”后飞行记录可读 |
| `the_flight_task_was_not_disturbed` | 写卡没有打乱模拟任务的周期（偏差 < 3 ms） |

运行方法（卡上要有文件——`python tools/blackbox.py sd-prepare E:`；**测试会擦除文件中的全部飞行记录**）：

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

开发板需处于 DFU 模式（DevEBox 上是 BT0→3V3 的跳线和 RST，通过 Zadig 安装 WinUSB 驱动，详见 [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)）。STM32 的控制台是 USB CDC：烧录之后端口不会马上出现，`pio test` 有时来不及打开它（“could not open port”）——这时请运行 `pio test ... --without-testing`，再用任何开启了 DTR 的终端程序查看输出（测试最多会等待 60 s，直到端口被打开）。测试结束后，开发板会等待按下 **`D`** 键——这会让它不接跳线直接重启进入 DFU。

在 DevEBox H743 + 16 GB 卡上的结果（2026-10-02）：`test_blackbox_sd` — 9/9，`test_feedback` — 10/10，`test_imu_orientation` — 5/5；卡速度的数据见 [BLACKBOX.md](BLACKBOX.md#在开发板上的实测结果)。

### 台架固件——`test/bench/`

它们不是测试集，而是独立的 PlatformIO 小项目，会替代飞行固件烧录到开发板上（`pio test` 看不到它们：文件夹名不以 `test_` 开头）。引脚和限值取自公用的 `Config.h`。

| 项目 | 作用 |
|---|---|
| `bench/elevator_sweep` | 通过 `ControlMixer` 和 `FlightOutputs` 以程序方式扫动升降舵摇杆（CH2），就像真实摇杆那样：向上 100% 行程，向下 60%，平滑运动并带停顿；工作 20 s——中位 20 s。在两端位置测量输出端的脉冲。油门保持最小 |

烧录：`pio run -d test/bench/elevator_sweep -t upload`。恢复飞行固件：`pio run -e esp32-s3 -t upload`。

---

## 覆盖率

由 `gcovr` 对 `include/` 和 `src/`（所有进入固件的代码）统计，两个原生环境合并计算：`gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`。

| 层 | 行 | 分支 |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors`（全部） | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src`（`main.cpp`、`stm32/main.cpp`） | 118/123 (95.9%) | 20/29 (69.0%) |
| **总计** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**；函数 877/902 (97.2%) |

仍未覆盖的部分及原因：

- **手抛起飞**（`TakeoffSequencer`：`WaitLaunch`、`launchDetected()`）——当 `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` 时无法到达；等该常量可配置（迁入 `Config.h`）后，会加入测试。
- **与开发板相关的部分**：没有引脚的输出（`PIN_RUDDER = -1` 只会出现在 C3 上）、没有 TX 引脚的 GPS（C3）——原生测试覆盖 S3、38 针版和 STM32 的引脚分配，但不覆盖 C3（C3 由构建矩阵检查）。
- **STM32**：内核的错误分支（引脚上没有定时器、定时器池耗尽）、消息 `FreeRTOS не запустился`（“FreeRTOS 未能启动”）——在电脑上 `vTaskStartScheduler()` 总是会返回。
- **防御性分支**，无法通过公共 API 到达：对枚举做 `switch` 时的 `default`/`Count`、`return "?"`。
- 没有可执行行的文件（`Config.h`、`Channels.h`、`FeedbackConfig.h`、结构体 `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`、`SensorSelection.h` 中的宏、仪表盘的 HTML）不会出现在报告中——它们会被编译进测试，但 gcov 没有可统计的内容。

---

## 静态分析

| 工具 | 命令 | 配置 |
|---|---|---|
| GCC | `tools/build_matrix.sh`（或 `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`） | 所有开发板 × 所有传感器套件。原生测试构建始终使用 `-Wall -Wextra -Wshadow`；`stm32h743` 使用 `-Wall -Wextra`（`build_src_flags`；`-Wshadow` 在 STM32duino 自身的头文件上会产生大量噪声） |
| cppcheck | `pio check -e esp32-s3`；`pio check -e stm32h743` | `[esp32_common]` 中的 `check_*`：`include/` 和 `src/`（`stm32/` 除外），warning/style/performance/portability，只在误报时使用行内的 `// cppcheck-suppress`（U8g2 的回调、`setup/loop`）。`stm32h743` 对 `include/hal/stm32/` 和 `src/stm32/` 使用同样的参数 |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`：bugprone、clang-analyzer、performance、`misc-include-cleaner` 等；被关闭的检查及理由写在文件里 |

clang-tidy 借助 `test/native/support` 里的模拟替身运行：clang 无法按主机架构解析 ESP-IDF 的头文件（如果用 `clangtidy` 去跑 `pio check`，分析会在解析错误处中断，实际上什么也没检查）。`misc-include-cleaner` 负责保证每个头文件都包含自己用到的东西：“伞形”头文件（`FeedbackModules.h`、`IBoard.h`/`RegisterDevice.h` 的 API、`SensorSelection.h` 的宏）标有 `// IWYU pragma: export`。脚本会跳过 STM32 代码（`include/hal/stm32/`、`src/stm32/`）——这部分由构建、`stm32h743` 环境的 cppcheck 和 `native-stm32` 环境的测试来检查。

最近一次运行时的构建矩阵——**24/24 无警告**：

| 开发板 | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck（`esp32-s3`、`stm32h743`）——项目代码中 0 条问题。

---

## 如何编写新的测试

1. 不依赖硬件、只含逻辑的模块——直接写单元测试：时间通过参数传入，或用 `fake::advanceMs()` 推进。
2. 芯片驱动——通过 `I2cRig`/`SpiRig`：模拟芯片的寄存器，检查写入的值（`chip.lastWrite(reg)`）和数据的解析。公式要用数据手册里的参考值或独立计算的结果，而不是代码的拷贝。
3. 带有 FreeRTOS 无限循环任务的类——`fake::findTask("名称")` + `fake::runTask(task, n)`；STM32 的飞行任务也是这样驱动的。
4. 使用另一套传感器或另一块开发板的整套固件——单独建一个测试集，在 `#include "../../../src/main.cpp"` 之前设置 `SENSOR_KIT`（或 `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`）；芯片取自 `helpers/ChipEmulators.h`。STM32 请用 `test/native_stm32/`。
5. 新的自动驾驶仪模式——在 `test_sim` 里添加闭环飞行场景。
6. 新的测试集——文件夹 `test/native/test_<名称>/test_main.cpp`，带 `main()`；如果测试集不需要在各测试之间保留状态，`setUp()` 调用 `resetWorld()`。
7. 发现了 bug——先写出能抓到它的测试，再修复。
