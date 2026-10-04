# APPLICATION — `src/main.cpp` 与 `src/stm32/main.cpp`

> 🌐 本页是[俄语原文](../../../reference/application.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

[← 参考](README.md)

这两个入口点是各自开发板的 **composition root**：固件中唯一的编译单元，也是唯一创建对象并用引用把它们连接起来的地方。其中没有飞行逻辑，对象集合也相同；不同之处在于开发板、遥测方式（Wi-Fi 或 MAVLink）以及飞行循环的驱动方式。

## 全局对象（通用）

声明顺序 = 构造顺序。

| 对象 | 类型 | 关联 |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`、`imuSensor` | `SELECTED_IMU_DEVICE(board)`、`SelectedImu` | `SensorSelection.h` 中指定的总线 |
| `baroDevice`、`baroSensor` | `SELECTED_BARO_DEVICE(board)`、`SelectedBaro` | 有皮托管时——这是静压 |
| `magDevice`、`magSensor`、`magnetometer` | … `SelectedMag`、`MagnetometerSensor* const` | 仅当 `SENSOR_MAG != NONE`，否则为 `nullptr` |
| `gpsSensor`、`gpsReceiver` | `SelectedGps`、`GpsSensor* const` | 仅当 `SENSOR_GPS != NONE` |
| `pitotDevice`、`pitotBaro`、`pitotSensor`、`airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`、`SelectedPitotBaro`（`"PITOT-BMP581"`）、`PitotDualBaroAirspeed(pitotBaro, baroSensor)` | 仅当 `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`、`throttleManager` | `ControlMixer`、`ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | 全部传感器（可为空） |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`、`Controls::BINDINGS` 表 |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | 以上全部 |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | 控制器、自动驾驶仪、统计 |
| `debugConsole` | `DebugConsole` | 控制器、各路输出、自动驾驶仪、日志、`&board`（总线扫描 `b`） |
| `oledDisplay` | `OledDisplay` | 控制器、自动驾驶仪、统计 |
| ESP32：`webDebugServer` | `WebDebugServer` | 控制器、自动驾驶仪 |
| STM32：`mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`、控制器、自动驾驶仪、统计 |

## `src/main.cpp` — ESP32（S3、C3、38 针）

| 函数 | 说明 |
|---|---|
| `static void printBanner()` | 向 `Serial` 输出启动横幅 |
| `static void setupSensors()` | 对每个传感器调用 `begin()`；校准有应答的传感器：IMU `calibrate()`（静止 2 s + 飞行前检查）、气压计 `calibrateAltitude()`、电子罗盘——25 ms 后的第一个采样用来设定 IMU 的航向（`setYaw`）；GPS `begin()`；皮托管 `begin()`（归零在循环的第一秒内进行）；`autopilot.begin()` |
| `void setup()` | 在 `begin(115200)` **之前**调用 `Serial.setTxBufferSize(4096)`；横幅；`board.begin()`；`flightOutputs.begin()` + `setFailsafe()`；`setupSensors()`；`flightController.begin()`；OLED；Web 服务器；拨杆布局；`debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`；周期为 `vTaskDelayUntil(LOOP_PERIOD_MS)`；延迟 > 100 ms 时——重新计时（不“追赶”） |

## `src/stm32/main.cpp` — STM32H743

env `stm32h743` 的入口点（在 ESP32 的构建中，`src/stm32/` 目录通过 `build_src_filter` 被排除）。在硬件上验证过的是没有接传感器的 DevEBox H743
板（启动、通过 USB 的控制台、SD 卡、黑匣子、iBUS，以及对舵机和电机的手动控制）；整个程序在 PC 上由 `test/native_stm32` 测试（env `native-stm32`）运行。旁边还有：`sd_msp.cpp`——SDMMC1 的引脚和时钟，`bootloader.cpp`——控制台的
`D` 键（重启进入 DFU）。

| 函数 | 说明 |
|---|---|
| `setup()` | `Serial.begin(115200)`；横幅；`board.begin()`；各路输出置于安全位置；`Stm32FlashStorage::store().mount()`——设置映像（空 / N 字节 / 已损坏——使用默认值）；`setupSensors()`（与 ESP32 相同）；`flightController.begin()`；`mavlink.begin()`；OLED；拨杆布局；`debugLogger.begin()`；任务 `flight` 和 `storage`；`vTaskStartScheduler()`（不会返回） |
| `static void flightTask(void*)` | 优先级 `Rtos::PRIORITY_FLIGHT`，栈 16 KB：`flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`；`vTaskDelayUntil(LOOP_PERIOD_MS)`，延迟 > 100 ms 时——重新计时 |
| `static void storageTask(void*)` | 后台任务：每 100 ms 调用一次 `Stm32FlashStorage::instance().service()`——擦除并写入设置扇区，会被飞行任务抢占 |
| `loop()` | 空：`vTaskStartScheduler()` 之后只有各任务在运行 |

控制台（`Serial`，LPUART1 PA9/PA10，115200）与 ESP32 上的是同一个 `DebugConsole`：
`h` 菜单，`s` 传感器，`b` 总线扫描，`p` 输出，校准。

## 不变量

- 各路输出在传感器初始化**之前**就进入安全位置（IMU 校准会使循环停顿约 2 s）。
- ESP32：`Serial` 的 TX 缓冲区在 `begin()` 之前设置。STM32：UART 缓冲区由
  `platformio.ini` 中的 `SERIAL_RX/TX_BUFFER_SIZE` 决定。
- 没有任何对象拥有另一个对象：所有引用都不持有所有权，生命周期是整个程序。
- 要改变某个拨杆的作用——修改 `config/Controls.h`，而不是 `main.cpp`。
