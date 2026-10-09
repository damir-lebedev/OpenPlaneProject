# 黑匣子

> 🌐 本页是[俄语原文](../../BLACKBOX.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。 本译文由 AI 完成，未经母语者审校。如发现错误，请联系 [Damir Lebedev](https://github.com/damir-lebedev)，或在[问题追踪页](https://github.com/damir-lebedev/OpenPlaneProject/issues)中提出。

固件会自己把每一次飞行写进开发板内置的闪存：传感器、摇杆、舵机输出、自动驾驶仪的决策、各类事件。飞行结束后，通过 USB 把记录下载下来，并解析成 CSV 表。

它在两种开发板上工作：

| 开发板 | 写到哪里 | 能存多少（按约 20 KB/s 计） |
|---|---|---|
| **ESP32-S3 N16R8** | 内置闪存中 13.9 MB 的分区 | 约 **11 分钟** |
| **STM32H743**（DevEBox——主力板；WeAct） | SD 卡上的一个文件，见[下文](#sd-卡stm32h743) | 64 MB——约 **55 分钟**，大小由文件决定 |

其他开发板（ESP32-C3、普通 ESP32）没有存储介质：黑匣子处于关闭状态，不会影响飞行。

---

## 何时记录

| | 条件 |
|---|---|
| **开始** | 已解锁，**并且**油门已推起（摇杆或 ESC 高于 `THROTTLE_LOW_US`）。**此前的 10 s** 也会被记录——包括 ARM 的那一刻和起飞前的停放阶段 |
| | 因故障而重启（panic、看门狗、电压跌落）——从第一个控制周期开始记录，且至少记录 60 s：如果发生在空中，就能看到后来发生了什么 |
| | 在控制台手动开始（`k` → `r`）——用于台架测试 |
| **停止** | **DISARM 后 10 s** |
| | 已解锁，但电机不转，且飞机**静止不动达 30 s**——已经降落或坠毁，而忘了 DISARM |
| | 手动停止（`k` → `r`） |
| **不会停止** | 信号丢失、failsafe、空中电机归零、滑翔、没有 DISARM 就降落并且飞机还在滑跑 |

“静止不动”是下面这些条件同时成立：各轴旋转速度都小于 5 °/s，加速度计读数为 1g ± 0.1，气压计显示几乎没有垂直速度，GPS 和皮托管（如果有）显示速度低于 2 m/s。在飞行中，不可能连续整整 30 秒都这么平稳。

## 记录什么

| 记录 | 频率 | 内容 |
|---|---|---|
| `IMU` | 每个控制周期，500 Hz | 陀螺仪（°/s）、加速度计（g）、该控制周期的运算耗时（µs） |
| `CTRL` | 100 Hz | 横滚/俯仰/航向、自动驾驶仪的目标、飞手的摇杆、最终指令、**全部 7 路输出**（µs）、横滚和俯仰 PID 的各分量（P、I、D）、飞手和自动驾驶仪的油门、襟翼、模式、标志位（ARM、链路、failsafe、传感器是否正常……）、已启用的功能 |
| `RC` | 50 Hz | 遥控器全部 10 个通道、iBUS 帧计数（完整帧和损坏帧） |
| `BARO` | 每次采样（约 50 Hz） | 气压、温度、高度、垂直速度、目标高度 |
| `MAG` | 最高 50 Hz | 三轴磁场、航向 |
| `GPS` | 每次定位解算 | 坐标、高度、速度、航向、卫星数、定位状态、精度 |
| `AIR` | 最高 50 Hz | 皮托管：压差、指示空速和真空速、空气密度 |
| `NAV` | 10 Hz | 返航点（距离、方位）、航向和目标航向、导航速度、航向来源、手抛起飞和翱翔的阶段、自动配平 |
| `POWER` | 10 Hz | 电池电压和电流传感器输出（飞控板上的分压器，[FC_BOARD.md](FC_BOARD.md)，B 部分） |
| `SYS` | 1 Hz | 控制回路的频率和最差的控制周期、剩余内存、iBUS 计数、IMU 温度、黑匣子队列、丢失的记录、最长的一次闪存写入、剩余空间 |
| `EVENT` | 事件触发 | ARM/DISARM、拒绝 ARM 及原因、模式切换、信号丢失/恢复、传感器故障/恢复、GPS 定位、返航点已记录、开关功能、地理围栏、失速保护、手抛起飞和翱翔的阶段 |

每次飞行的开头都有一组参数：固件（编译日期）、本次启动和上一次重启的原因、装了哪些传感器以及它们是否通过了飞行前检查、PID 系数（包括在仪表盘上做过的修改）、配平值、`Config` 中的重要数值，以及开关绑定。

---

## 如何使用

### 飞行前

什么都不用做。开机时，串口监视器会显示状态（控制台用俄语输出；下面这一行的意思是“等待 ARM 和油门 | 前方已擦除 12.9 MB（约 11 分钟），共 13.9 MB | 飞行 1 次”）：

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

“前方已擦除”指的是下一次飞行还能存下多少。开机后，黑匣子会用几秒钟（长时间飞行之后最多一分钟）腾出空间：擦除旧的记录。在这段时间里，地面上的飞行循环有时会停顿约 0.15 s——舵面可能会晚一点才抽动一下，这是正常现象。**在空中永远不会擦除闪存。**

### 飞行后——下载

1. 把 USB 接到 **COM** 接口。关闭串口监视器（它会占用端口）。
2. 在项目文件夹中运行：

   ```bash
   python tools/blackbox.py download          # 最近一次飞行
   python tools/blackbox.py download --all    # 全部
   python tools/blackbox.py list              # 开发板上有什么
   ```

   需要 `pyserial`：`pip install pyserial`。或者使用 PlatformIO 自带的 Python，它已经装好了：`%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`。

3. 飞行记录会下载到 `blackbox/` 文件夹（约 200 KB/s：10 分钟的飞行大约要一分钟），并立刻在旁边的同名文件夹中解析出来。

下载期间飞行循环是停止的，所以只能在没有 ARM 的情况下进行。

### 飞行文件夹里有什么

| 文件 | 说明 |
|---|---|
| `summary.txt` | 摘要：时长、频率、角度/高度/速度/电压的范围、最差的控制周期、丢失的记录、全部事件 |
| `events.txt` | 本次飞行的参数和按时间排列的全部事件 |
| `IMU.csv`、`CTRL.csv`、`RC.csv`…… | 每种记录一张表 |

所有表中的时间都是 `time_s`，即从记录开始（ARM 并推起油门）算起的秒数；预先记录的部分为负数。数值已经换算成实际单位：度、g、米、m/s、脉冲的微秒数。在 `CTRL.csv` 中，模式以名称补充在后面（`mode_name`），功能以列表形式给出（`features_on`），标志位被拆成 0/1 的列（`armed`、`rx_lost`、`fs_glide`、`imu_ok`……）。

CSV 可以用 Excel/LibreOffice 打开，但要按时间画曲线，用 [PlotJuggler](https://github.com/facontidavide/PlotJuggler) 更方便：File → Load Data → CSV，时间列选 `time_s`。

`.bbl` 文件是闪存的原始镜像，可以重新解析：`python tools/blackbox.py decode blackbox/flight_001_....bbl`。

### 控制台：`k`

在串口监视器中，按 `k` 键会打开黑匣子菜单：状态、飞行列表，`r`——手动开始/停止记录（用于在台架上检查），`e`——擦除所有飞行记录（需要按 `y` 确认，约 40 s）。

---

## 闪存空间

- 飞行记录是循环写入的。空间不足时，黑匣子会在地面上**整段擦除最旧的飞行记录**，直到前方空出 10 MB（`BLACKBOX_MIN_FREE_BYTES`，约 9 分钟）。
- **最后一次记录的飞行永远不会被擦除**——只会被下一次记录覆盖，前提是下一次记录空间不够。
- 如果已擦除的空间在空中用完，记录会继续写入 PSRAM 中的队列（4 MB，约 3 分钟的最新数据）；降落并 DISARM 之后，黑匣子会腾出空间并把队列写进去。比整个分区（约 11 分钟）还长的飞行无法完整保存：开头保留，结尾丢失。
- 所以**每次飞行之后都要下载记录**——尤其是第一次飞行之后。

## 可靠性

- 任何时刻断电（坠机、电池脱落）：除最后约 15 ms 外，其余全部保留。没有写完的记录会按 CRC 被丢弃——在 `summary.txt` 中是这一行：“Недописанных записей (CRC)”（摘要是用俄语写的，意思是“未写完的记录数（CRC）”）。
- 飞行编号、环的头部位置和飞行列表都根据扇区本身恢复：没有任何可能被破坏的独立“地图”。
- 下载时会校验每个扇区和整次飞行的 CRC-32。

## 对飞行的影响

- 飞行循环只是把一份快照放进 PSRAM 中的队列——只需几微秒。真正写闪存的是核心 0 上的一个独立任务，一次写一页（256 字节），**紧接在一个控制周期之后**：写闪存会让 ESP32 的两个核心停顿 0.6–0.9 ms，而这段停顿正好落在两个控制周期之间的空隙里。
- 台架实测（没有传感器的 DevKit，两次各 30–40 s 的记录）：控制周期间隔为 2.00 ms，99.2–99.7% 的间隔在 1.9–2.1 ms 之内，最长的是 2.5 ms，没有丢过一个控制周期；控制周期的工作量与不记录时相同。只有在地面未 ARM 时，黑匣子核对并擦除空间期间，控制周期才会有明显的抖动（读取一个 64 KB 的块——停顿约 3 ms，擦除——约 0.15 s）。
- 装上传感器后，一个控制周期约占 0.7 ms，写一页仍然能放进剩下的 1.3 ms。第一次飞行之后可以检查：在 `summary.txt` 中查看“Такт IMU”和“Цикл: худший такт”这两行（俄语，意思是“IMU 周期”和“循环：最差周期”）。

---

## SD 卡（STM32H743）

在 STM32H743 上，黑匣子写入 SD 卡（SDMMC1 上的 µSD 卡槽，4 位，24 MHz）。卡仍然是普通的 **FAT32**：卡的根目录里放着一个事先创建好的文件 `BLACKBOX.BIN`，固件把原始数据块写进这个文件里，而绝不碰 FAT 表和目录。所以飞行中断电时没有什么东西可以被损坏，而且可以直接把这个文件复制到电脑上。

### 准备卡（只做一次）

1. 把卡格式化为 **FAT32**（不是 exFAT；Windows 对 32 GB 以内的卡会提供 FAT32）。
2. 把卡插进读卡器，在电脑上运行：

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB，E: 是卡所在的盘符
   python tools/blackbox.py sd-prepare E: --size 256   # 或者更大
   ```

   文件是在空卡上一次性连续创建的，并用 `0xFF` 填满；第一个扇区是一个“环为空”的标记。如果文件不是连续存放的（卡不是空的而且碎片很严重）或者根本没有这个文件，开机时控制台会给出原因，而黑匣子处于关闭状态。
3. 把卡插进开发板。开机时（控制台用俄语输出：“SD 卡：15204 MB，SDMMC 24 MHz，4 位；文件 BLACKBOX.BIN：正常”，然后是状态行，再然后是“300 ms 后就绪”）：

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   “前方已擦除”的量会在后台增长：开发板以约 2.5 MB/s 的速度核对空间。

### 取回飞行记录

- **通过开发板的 USB**——和 ESP32 一样：`python tools/blackbox.py download`（STM32 的控制台是 USB CDC，下载速度约 400 KB/s，1 MB 不到 3 s）。`list`、`--all`、`--flight N` 的用法相同。
- **取出卡**：卡上的 `BLACKBOX.BIN` 文件可以直接解析成 CSV——

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # 全部飞行 -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # 只列出来
  ```

  这个文件是一个由扇区组成的环：工具会根据扇区编号自己把飞行记录拼起来，包括绕过文件末尾的那些。

### 在开发板上的实测结果

DevEBox H743 + 16 GB 的卡（测试 `test_blackbox_sd`，[TESTING.md](TESTING.md#stm32-开发板上的测试)）：

| | |
|---|---|
| 识别卡 | 12–18 ms，4 位，24 MHz |
| 写一页 256 B | 平均 2.3–3.7 ms，**最差 60–190 ms**，持续约 75–110 KB/s（需要约 20 KB/s） |
| 读取 | 4 KB 扇区——4.2 MB/s；随机块——0.6 ms |
| 擦除 | 64 KB——13 ms；整个 64 MB 区域——20–28 s |
| 开机 | 环为空时——0 ms（靠标记）；有飞行记录时——0.3 s（抽样核对，约 530 次读取）；对 64 MB 做完整核对大约需要 20 s |
| 20 s 实时记录（500 Hz 的 IMU） | 没有丢失任何一条记录，错误为 0 |
| 记录时的飞行任务 | 周期偏差 2 ms——**1 µs**（在记录旁边放一个最高优先级的模拟任务） |

最差的那次写页是卡内部的“整理”；RAM 中的队列（384 KB ≈ 19 s 的数据流）能挺过这样的停顿。便宜的卡在这方面差别最大：飞行之前，最好用 `test_blackbox_sd` 测试检查一下卡（最差的一次写入必须小于 250 ms——SD 规范的上限）。

### 与 ESP32 闪存的区别

- **写入任务**（`bbox`，优先级 2）会在访问卡的过程中途被飞行任务（5）抢占——而不是像 ESP32 那样“在控制周期的间隙里”，因为 ESP32 会让两个核心停顿。数据传输使用 SDMMC 的硬件流控：没有它的话，被抢占时 FIFO 会溢出（在开发板上表现为 `HAL_SD_ERROR_RX_OVERRUN`，控制台和记录会卡住好几秒）。
- **队列在 RAM 中**，384 KB（`BLACKBOX_RING_STM32_BYTES`），而不是 4 MB 的 PSRAM。
- **开机时的核对是抽样的**：环中真实的扇区构成一段连续的弧，只读取约 500 个扇区头，弧的边界和各次飞行的边界用二分法确定。结果与完整核对相同；如果对不上——就做完整核对。
- **第一个扇区里的“环为空”标记**：这样每次开机就不必花几秒钟去核对空白区域。擦除全部内容时以及完整核对什么都没找到时会设置这个标记；第一次写入之前会清除它。
- **卡的错误**（被拔出、总线故障）每秒一次以事件形式写入日志：“носитель: ошибок записи …”（俄语，意思是“存储介质：写入错误数 …”）；丢失的页会留下一个 `0xFF` 的空洞，该扇区的解析会在这里停止（与 `tools/blackbox.py` 中一样），其余扇区完好无损。

## 设置（`include/config/Config.h`，“黑匣子”部分）

| 常量 | 默认值 | 含义 |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | PSRAM 中的队列（没有 PSRAM 时用 `BLACKBOX_RING_NO_PSRAM_BYTES`，32 KB） |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32：RAM 中的队列 |
| `BLACKBOX_SD_FILE`、`BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`、256 MB | STM32：卡上的文件，以及实际使用部分的上限 |
| `BLACKBOX_PREROLL_MS` | 10 000 | 开始之前要记录多久 |
| `BLACKBOX_POSTROLL_MS` | 10 000 | DISARM 之后要记录多久 |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | 已 ARM 但静止不动——停止 |
| `BLACKBOX_LANDED_GYRO_DPS`、`_ACCEL_G`、`_CLIMB_MS`、`_SPEED_MS` | 5、0.1、0.5、2 | 什么算“静止不动” |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | 故障重启之后的记录——不得短于此值 |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | 为下一次飞行要保持擦除多少空间 |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | 在地面上两次擦除之间的停顿 |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU 每隔 N 个控制周期记录一次：2——250 Hz，记录时间延长约 25% |
| `BLACKBOX_VBAT_DIVIDER`、`_CURRENT_DIVIDER` | 6.6、1.667 | 开发板上电池和电流传感器的分压比 |

分区表是 `partitions_blackbox.csv`：应用程序 2 MB（固件目前约 0.9 MB），黑匣子 13.9 MB，coredump 64 KB。NVS 分区仍在原来的位置——换用这张分区表之后，IMU 和罗盘的校准数据以及配平值都会保留。没有用于空中升级（OTA）的第二个槽位。

---

## 给开发者

代码在 `include/telemetry/BlackBox*.h`：

| 文件 | 内容 |
|---|---|
| `BlackBoxFormat.h` | 格式：扇区头、记录类型和结构体、字段模式、CRC-8/CRC-32 |
| `BlackBoxStorage.h` | 建立在 `IFlashRegion` 上的扇区环：开机时查找头部、飞行列表、按页写入、分步擦除旧的飞行记录 |
| `BlackBoxRing.h` | 核心之间的记录队列（自旋锁），满了就丢弃最旧的 |
| `BlackBox.h` | 控制周期中的快照、开始/停止、事件、写入任务、通过 UART 下载 |
| `hal/esp32/Esp32FlashPartition.h` | 建立在 `esp_partition` 之上的 `IFlashRegion` |
| `hal/SdFileRegion.h`、`storage/Fat32File.h` | 建立在 FAT32 卡上的文件之上的 `IFlashRegion`：查找文件（只读 FAT）、不完整的块、用 `0xFF` 擦除、“环为空”标记 |
| `hal/stm32/Stm32SdCard.h`、`src/stm32/sd_msp.cpp` | `IBlockDevice`：基于 `HAL_SD` 的 SDMMC1（轮询、4 位、硬件流控）以及引脚 |
| `hal/ResetCause.h` | ESP32 和 STM32 上的重启原因（`RCC->RSR`） |

### 闪存上的格式

4 KB 的扇区 = 16 字节的头部（`magic "OPBB"`、连续递增的 `seq`、打开时的 `millis()`、飞行编号、格式版本、校验字节）+ 依次排列的记录。记录不会跨越扇区边界；扇区的尾部是 `0xFF`。

记录：`[类型 u8][长度 u8][数据][CRC-8]`，数据以 `t_us`（`micros()`）开头。每次飞行最开头的记录是 `SCHEMA`：文本 `"16 IMU t_us:I gx:h/10 ..."`——字段名、Python `struct` 的格式字符、除数。解码器从日志中获取字段，所以在记录里新增一个字段，就是修改 `BlackBoxFormat.h` 中的结构体和模式字符串（它们的大小由 `static_assert` 核对），不需要修改解码器。

### 下载协议

命令是 STX 字节（`0x02`）之后的一行文本，控制台会把它转交给黑匣子：

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   （在 115200 下），随后切换到 2 Mbaud
PC:  切换到 2 Mbaud，发送 'G'
FC:  234 帧：A5 5A，u16 编号，4096 字节的扇区数据，u32 CRC-32
     回到 115200，BB:DONE n=3 crc=<所有扇区的 CRC-32>
```

### 测试

`pio test -e native -f native/test_blackbox_scan`——在随机的历史记录（300 个环 × 5 个探测步长）上，把环的抽样核对与完整核对做比较，并测算在 SD 区域上的开销。`pio test -e native-stm32 -f native_stm32/test_blackbox_sd` 和 `test_app_stm32_*`——FAT32、区域、卡驱动、卡上的黑匣子、整套 STM32 固件。在开发板上——`pio test -e stm32h743-devebox -f test_blackbox_sd`（真实的卡）。

`pio test -e native -f native/test_blackbox`——格式、建立在分区的 NOR 模拟之上的环（按扇区擦除，写入只能把位拉低——把某一位重新拉高就算错误）、重启、断电、在真实的 `FlightController`/`Autopilot` 上记录一次飞行、开始/停止、事件、空中闪存溢出、下载。用来检查解码器的飞行镜像：`OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`。
