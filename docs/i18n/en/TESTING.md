# TESTING.md — tests, coverage and static analysis

> 🌐 This page is a translation of the [Russian original](../../TESTING.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

The firmware is verified on two levels:

| Where | Command | What |
|---|---|---|
| **PC (native)** | `pio test -e native` | The firmware headers are built on the PC unchanged, with the hardware replaced by controllable fakes: modules, drivers, closed-loop flight simulations, the whole ESP32 firmware (S3 and 38-pin) with every sensor kit. Coverage is counted |
| **PC (native-stm32)** | `pio test -e native-stm32` | The whole STM32H743 firmware (`src/stm32/main.cpp`) on top of a layer of STM32duino fakes: FreeRTOS tasks, flash, MAVLink, sensors on I2C and SPI |
| **Build matrix** | `tools/build_matrix.sh` | 4 boards × 6 sensor kits with `-Wall -Wextra (-Wshadow)`; any warning in the project's code is an error |
| **Board** | `pio test -e esp32-s3` | `test_feedback` and `test_imu_orientation` on a real ESP32-S3 (it flashes a test firmware; afterward put the normal one back: `pio run -t upload`) |
| **STM32 board** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | The black box on a **real SD card** in the DevEBox H743, plus `test_feedback` and `test_imu_orientation` on a Cortex-M7 — [below](#tests-on-the-stm32-board) |

The architectural context is in [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-testability).

---

## Quick start

```bash
pip install platformio gcovr        # once
# Windows: you need g++ in PATH, for example WinLibs (winlibs.com, zip UCRT):
# unpack it and add mingw64\bin to PATH — no installation needed
pio test -e native -e native-stm32  # all the native tests (~1.5 min)
gcovr                               # coverage by file (settings — gcovr.cfg)
tools/build_matrix.sh               # all boards × all sensors (~25 min)
gcovr --html-details -o coverage/index.html   # HTML report (coverage/ is in .gitignore)

pio test -e native -f native/test_rc          # one suite
pio test -e native -f test_feedback           # feedback simulation on the PC

# Trajectories of the closed-loop simulations as CSV (for plots):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# The MAVLink stream — for checking with a reference decoder (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Before counting coverage after changes to the tests, it helps to start from a clean build: `rm -rf .pio/build/native`, otherwise the counters from earlier runs end up in the report.

---

## How the native build works

`[env:native]` in `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (the main board's pinout), `-I test/native/support`, `-Wall -Wextra -Wshadow`, coverage with `--coverage` and `-fkeep-inline-functions -fkeep-static-functions` — without them gcov does not see header functions that were never called and overstates the coverage.

### Hardware fakes — `test/native/support/`

Headers with the same names and signatures as the Arduino core for ESP32 2.0.x, ESP-IDF and the libraries, but on top of a simulated world in `namespace fake`:

| File | Replaces | What the simulation can do |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | the Arduino core | Macros (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, `print()` formatting like the original. `ARDUINO` is deliberately **not** defined |
| `esp32-hal-fake.h` | time, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | The clock advances only through `fake::advance*()`/`delay()`; `millis()/micros()` are `uint32_t`, as on the ESP32 (overflow behaves as on the board). LEDC channels, `pulseIn` by the real duty cycle (visible only if the pin's input buffer is enabled), `analogReadMilliVolts` — the voltage from `fake::gpio().analogMv`. Tasks are registered (the handle is non-null); `fake::runTask(task, n)` runs n passes of its endless loop, `ulTaskNotifyTake` counts as a pass, `xTaskNotifyGive` as a counter. FreeRTOS mutexes are a "busy" flag. `psramFound()`/`ps_malloc()`. Critical sections are counted |
| `HardwareSerial.h` | UART | Ports are registered by number (`fake::uart(1)`); `pushRx()`, `txBytes()`, changing the speed on the fly (`updateBaudRate`, the history is `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | ESP-IDF flash partitions | A partition is a vector of bytes with NOR behavior: erasing only by 4 KB sectors, erased = 0xFF, a write only lowers bits (an attempt to raise a bit is counted — `bitRaises`); `beforeWrite` — "the power went out"; counters of reads, writes and erases |
| `esp_system.h` | the reset cause | `esp_reset_reason()` from `fake::chip().resetReason` |
| `Wire.h` | I2C | Devices by address; `fake::RegisterMapDevice` — registers with auto-increment, a write log, faults (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), hooks `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Devices by CS pin; `fake::SpiRegisterMapDevice` — the Bosch/InvenSense protocol, `dummyBytes` before the data |
| `Preferences.h` | NVS | In-memory storage, the behavior of `begin(readOnly)`/`get*`/`getBytes` as in the original; `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | The result of `softAP()` is set by the test; `WebServer::request(method, uri, body)` calls the registered handler; `fake::webServers()` — all the instances |
| `U8g2lib.h` | U8g2 | Instead of pixels — a list of the strings and rectangles drawn; `begin()/sendBuffer()` push bytes through a user byte callback; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### The STM32duino layer — `test/native/support_stm32/` (env `native-stm32`)

It sits in `-I` ahead of `support/` and supplements the same fakes with what only STM32duino has. `<Preferences.h>` in this environment is the **real** `include/hal/stm32/compat/Preferences.h` on top of `KeyValueStore`.

| File | Replaces | What it can do |
|---|---|---|
| `Arduino.h` | the STM32duino core | pins `PA0..PE15` (port·16 + number), `pin_size_t`, `PinMap_TIM` for the servo-output pins, `HardwareTimer` (the pulse is visible through `fake::timerPulseUs(pin)` and `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino FreeRTOS | `xTaskCreate` into the shared task registry (stack in words), `vTaskStartScheduler()` returns — the test runs the tasks itself (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM emulation | an 8 KB "flash" (erased = 0xFF) and a buffer, `fake::eeprom()` — counters and image corruption |
| `SPI.h` | | `SPIMode` |

Added to the shared fakes for STM32: `TwoWire(sda, scl)`, `setSDA/SCL` and `fake::wireWithSda(pin)` (to find the board's second bus), `HardwareSerial(rx, tx)` and `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Chip emulators and the airplane model — `test/native/helpers/`

| File | What it is |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (with IPREG indirect registers), QMC6309, SPL06-001, BMP581, u-blox NAV-PVT frames — register maps on I2C or SPI, with data from the `World` "world" (angles and rates, altitude, airspeed, heading, coordinates), in the chip's axes with `IMU_ROTATION_CW_DEG` taken into account |
| `PlaneSim.h` | a ~1.2 kg airplane model: a point mass + rotation in roll/pitch, CL(α) with stall, drag, thrust, wind, thermals, ground |
| `SimHarness.h` | a closed loop: transmitter → iBUS frame → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → surface deflections → `PlaneSim` → sensors (including a pitot tube on two noisy barometers). A CSV trajectory with `OPENPLANE_SIM_DIR` |

`test/native/helpers/TestSupport.h` is what the suites share: `resetWorld()` (called from `setUp()`), the stand-ins `FakeUart`/`FakeServo`/`FakeBoard` and for the sensors (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), the frame builder `ibusFrame()`, the rigs `I2cRig`/`SpiRig` (a driver on top of the real `Esp32I2CBus`/`Esp32SpiBus` and `*RegisterDevice` with a simulated chip).

The board tests (`test_feedback`, `test_imu_orientation`) are portable: with `ARDUINO` — `setup()/loop()`, otherwise `main()`. The `test/native/*` suites are not built for the board (`test_ignore` in `[esp32_common]` and `[env:stm32h743]`: the patterns go one per line — separated by a space, PlatformIO reads them as one). On STM32: `pio test -e stm32h743`.

---

## Test suites

| Suite | Tests | What it checks |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus` helpers (NACK, short read — the buffer is left untouched), `I2cRegisterDevice`, `SpiRegisterDevice` (read bit, BMP388 dummy byte), `Esp32I2CBus` (5 ms timeout), `Esp32SpiBus` (modes 0–3), `Esp32UartPort` (8N1, pins), `Esp32ServoOutput` (50 Hz/14-bit, pulse limiting, LEDC failure, measurement through the input buffer), `Esp32Board` (buses, UART, channel order, AUX, buzzer) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, iBUS parsing: frames arriving in pieces, CRC, 12-bit values, transmitter failsafe, 500 ms timeout (also across a `micros()` overflow), garbage, resynchronization |
| `native/test_control` | 21 | Flaps (speed, first call, pauses), the mixer (signs, reverse, flaperons), throttle, the ARM state machine and the mode sensor checks, the output table and the pulse self-test |
| `native/test_autopilot` | 22 | PID (D term from the sensor rate, integral, anti-windup, `dt`), STABILIZE as an angle mode, time-based auto-takeoff, ALT_HOLD with the elevator, gliding on link loss |
| `native/test_autopilot_modes` | 31 | All 12 modes and how each reacts to a missing sensor, the binding table and `static_assert`, functions and knobs, navigation (heading, circle, home, geofence), failsafe RTH/gliding, hand launch, soaring, auto-trim (written only on the ground) |
| `native/test_flight_controller` | 12 | A full `FlightController` tick on the real classes: priorities link loss > ARM > sticks/autopilot > throttle; AUX, `MOTOR_KILL`, buzzer |
| `native/test_imu` | 21 | MPU6050/6500/9250 and ICM-42688: identification, registers, scales, axis rotation and aviation sign conventions, bus errors, gyroscope calibration and the pre-flight check, mounting calibration from three poses, NVS, the orientation filter |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 over I2C and SPI, BME280/BMP280 against the Bosch reference, compasses (heading, hard-iron calibration in NVS), u-blox M10 (CFG-VALSET, NAV-PVT, corrupted frames, timeout), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, alternate address, SPI), ICM-45686 (indirect registers), QMC6309, SPL06-001 (datasheet formulas), BMP581 (DRDY and the fallback path), the pitot tube (zero, filter, density, swapped hoses, stale data, a “flight” with the noise of two barometers) |
| `native/test_storage` | 16 | `KeyValueStore` (reload, wear — an identical value is not rewritten, overflow without data loss, CRC, power loss during erase, garbage, format version), `KvPreferences` (behaves like the ESP32 NVS) |
| `native/test_mavlink` | 20 | The codec against reference pymavlink frames (v1, v2, signed), CRC, resynchronization; telemetry: stream rates, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, PID parameters (list, read, write, rejection of bad values), mode change from the ground, ARM from the ground — refused, missions — 0, an overfull UART buffer does not block the loop |
| `native/test_blackbox` | 19 | The black box: format and CRC, the sector ring on a NOR fake (a fresh partition without erasing, garbage is always erased, old flights are erased whole and only to free space, the latest is never touched, wrap-around at the end of the ring, the head after a reboot, power loss, a half-written record caught by CRC), flight recording on the real `FlightController`/`Autopilot`: start on ARM and throttle with pre-recording, stop after DISARM and “sitting on the ground”, link loss does not stop it, recording after a faulty reboot, manual start, events, battery, flash ran out in the air, a flight longer than the partition, download in CRC frames with a speed change, the console menu `k`, no partition — disabled |
| `native/test_blackbox_scan` | 3 | A sampled ring check at power-up against the full one: 300 random ring histories × 5 probe steps (the head, the numbers and the flight list match, and when the picture does not add up — it falls back to the full scan) and the cost on a 64 MB SD area (≈530 reads instead of 32,000) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, version), `DebugLogger` (all channels, NAV), `DebugConsole` (menu, hotkeys, bus probing `b`, forbidden during ARM, saving only without ARM), `WebDebugServer` (routes, JSON, mailbox), `OledDisplay` (bytes over I2C, frame, inversion on link loss) |
| `native/test_sim` | 15 | Closed-loop flights of the whole firmware with the airplane model: recovery from a bank, CRUISE in a crosswind, LOITER, RTH, failsafe RTH/gliding, geofence, auto-takeoff from a runway, hand launch, auto-landing, a thermal, RESCUE from a spiral, speed hold and stall protection, a real pitot tube in the loop, auto-trim of a “crooked” airplane, sensor failures in flight (IMU, barometer, pitot tube, GPS) |
| `native/test_feedback_units` | 14 | The feedback modules one by one: speed sources, in the air/on the ground, the RLS estimate, the controller, stall indicators, takeoff/landing aborts |
| `native/test_app` | 10 | `src/main.cpp` on the ESP32-S3 with the bench kit MPU6500/BMP388/QMC5883P/OLED: `loop()` period, transmitter → servos, ARM, modes, link loss, console, dashboard, display, black box (a task on core 0, recording on throttle, a flight after DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` on the ESP32-S3 with the flight kit: LSM6DSV + QMC6309 + SPL06 + BMP581 in the pitot tube + GPS — identification of all chips, pitot zero and speed, altitude, home from GPS, STABILIZE by the angles from the chip, RTH to home, bus probing, dashboard |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` on the **ESP32 38-pin** (`BOARD_ESP32_CLASSIC`) with the ICM-45686 + QMC6309 + SPL06 + BMP581 kit: board pinout, IPREG filters, hand launch, stabilization and speed, probing a single bus |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` on the **STM32H743** with the flight kit: tasks and priorities, a 2 ms period, the pitot tube, PWM timers and `pulseIn`, MAVLink in flight, a mode change from the GCS, settings written by a background task into the “flash”, the display on I2C1, the console |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 with the ICM-45686 and BMP581 **over SPI** + QMC6309: corrupted flash at power-up, ALT_HOLD from the GCS holds altitude, link loss → RTH, visible in MAVLink; rewriting a damaged image |
| `native_stm32/test_blackbox_sd` | 29 | The black box on an SD card: FAT32 (with and without MBR, a directory spanning two clusters, noise entries, a foreign/fragmented/empty volume), `SdFileRegion` (partial blocks, cache, erase, boundaries, failures), the real `Stm32SdCard` driver on top of a fake `HAL_SD` (4 bits, fallback speeds, retry, busy card, unaligned buffers), the ring on the card (reboot, power loss, scan cost), the “ring is empty” marker, flight recording on a `FlightController`, a faulty reboot detected via `RCC->RSR`, the battery ADC, card errors in flight, a slow card, download through the console, the `D` key |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` with a card: boot finds the card and the file, the `bbox` task writes the flight, the loop period does not stretch, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` without a card: the black box is disabled and explains why, the airplane flies, the `k` menu does not break |
| `test_feedback` | 10 | A closed-loop airplane simulation with a feedback loop (on the PC and on the board) |
| `test_imu_orientation` | 5 | IMU mounting calibration on 300 random mountings (on the PC and on the board) |
| **Total** | **387** | 340 in `native` + 47 in `native-stm32` (plus 9 only on the board — `test_blackbox_sd`) |

### Tests on the STM32 board

`test/test_blackbox_sd` is not native: the SDMMC driver, the card and the timing are real. The tests run in a FreeRTOS task, and next to it runs a flight-loop imitator task with the highest priority (2 ms period): it preempts the tests in the middle of card accesses, just as in the firmware. Without it you cannot catch the bug that was actually found on the board: on preemption the SDMMC FIFO overflowed (`HAL_SD_ERROR_RX_OVERRUN`), which never happens in a bare loop.

| Test | What it checks |
|---|---|
| `reset_cause_is_a_normal_one` | the reset cause (`RCC->RSR`) is neither the watchdog nor a brown-out |
| `card_is_detected_on_four_bit_bus` | the card is detected on a 4-bit bus at 24 MHz |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` is found on FAT32 and is contiguous |
| `multi_block_writes_work_at_every_length` | writing 1, 2, 4 and 8 blocks in a single access |
| `pages_write_with_bounded_latency_and_read_back_intact` | 256 B pages: the worst write < 250 ms (the SD limit), steadily > 40 KB/s, reading and erasing |
| `header_scan_cost_on_the_whole_area` | the cost of reading a sector header and of a full scan |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | erase everything, two flights of 20,000 records each, “reboot”: the sampled scan takes < 2 s, the records are read in order with a correct CRC; an empty ring is recognized by its marker in < 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | the real `BlackBox` at a 500 Hz IMU in real time: not a single lost record, the flight can be read after a “reboot” |
| `the_flight_task_was_not_disturbed` | writing to the card did not upset the period of the imitator task (deviation < 3 ms) |

Running (a card with the file — `python tools/blackbox.py sd-prepare E:`; **the test erases all flights in the file**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

Put the board into DFU mode (on the DevEBox — a wire from BT0 to 3V3 plus RST, the WinUSB driver via Zadig; details in [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). The STM32 console is USB CDC: after flashing, the port does not appear right away, and `pio test` sometimes fails to open it in time (“could not open port”) — in that case run `pio test ... --without-testing` and read the output with any terminal program with DTR enabled (the tests wait up to 60 s for the port to be opened). After the tests the board waits for the **`D`** key — it reboots into DFU without the wire.

Results on the DevEBox H743 + a 16 GB card (2026-10-02): `test_blackbox_sd` — 9/9, `test_feedback` — 10/10, `test_imu_orientation` — 5/5; the card speed figures are in [BLACKBOX.md](BLACKBOX.md#what-was-measured-on-the-board).

### Bench firmware — `test/bench/`

These are not test suites but separate mini PlatformIO projects that are flashed to the board instead of the flight firmware (`pio test` does not see them: the folder names do not start with `test_`). Pins and limits are taken from the shared `Config.h`.

| Project | What it does |
|---|---|
| `bench/elevator_sweep` | Sweeps the elevator stick (CH2) programmatically through `ControlMixer` and `FlightOutputs`, like a live stick: up 100% of travel, down 60%, smoothly, with pauses; 20 s of work — 20 s at neutral. At the extreme positions it measures the pulse on the outputs. Throttle at minimum |

To flash: `pio run -d test/bench/elevator_sweep -t upload`. To restore the flight firmware: `pio run -e esp32-s3 -t upload`.

---

## Coverage

It is computed by `gcovr` over `include/` and `src/` (everything that goes into the firmware), over both native environments together: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Layer | Lines | Branches |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors` (all) | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95.9%) | 20/29 (69.0%) |
| **Total** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**; functions 877/902 (97.2%) |

What remains uncovered, and why:

- **Hand launch** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) — unreachable while `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`; it will appear in the tests once the constant becomes configurable (moving to `Config.h`).
- **Board-dependent code:** an output without a pin (`PIN_RUDDER = -1` occurs only on the C3), a GPS without a TX pin (C3) — the native tests exercise the pinouts of the S3, the 38-pin and the STM32, but not the C3 (the C3 is checked by the build matrix).
- **STM32:** the error branches of the core (no timer on the pin, the timer pool exhausted), the message `FreeRTOS не запустился` (“FreeRTOS did not start”) — on the PC `vTaskStartScheduler()` always returns.
- **Defensive branches** that cannot be reached through the public API: `default`/`Count` in a `switch` over enumerations, `return "?"`.
- Files with no executable lines (`Config.h`, `Channels.h`, `FeedbackConfig.h`, the structs `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, the `SensorSelection.h` macros, the dashboard HTML) do not appear in the report — they are compiled into the tests, but gcov has nothing to count in them.

---

## Static analysis

| Tool | Command | Profile |
|---|---|---|
| GCC | `tools/build_matrix.sh` (or `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | All boards × all sensor kits. The native test build always uses `-Wall -Wextra -Wshadow`; `stm32h743` uses `-Wall -Wextra` (`build_src_flags`; `-Wshadow` is noisy on the STM32duino headers themselves) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` in `[esp32_common]`: `include/` and `src/` (except `stm32/`), warning/style/performance/portability, inline `// cppcheck-suppress` comments only for false positives (the U8g2 callback, `setup/loop`). For `stm32h743` — the same flags over `include/hal/stm32/` and `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` and others; disabled checks are explained in the file itself |

clang-tidy is run with the fakes from `test/native/support`: clang cannot parse the ESP-IDF headers for the host architecture (if you try `pio check` with `clangtidy`, the analysis aborts on parse errors and honestly checks nothing). `misc-include-cleaner` makes sure every header includes what it uses: the “umbrella” headers (`FeedbackModules.h`, the `IBoard.h`/`RegisterDevice.h` API, the `SensorSelection.h` macros) are marked `// IWYU pragma: export`. The script skips the STM32 code (`include/hal/stm32/`, `src/stm32/`) — it is covered by the build, by cppcheck for the `stm32h743` env, and by the tests of the `native-stm32` env.

The build matrix as of the last run — **24/24 without warnings**:

| Board | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) — 0 findings in the project code.

---

## How to write new tests

1. A module with logic and no hardware — a direct unit test: time is passed as a parameter or advanced with `fake::advanceMs()`.
2. A chip driver — through `I2cRig`/`SpiRig`: the registers of a simulated chip, checking the written values (`chip.lastWrite(reg)`) and the parsing of the data. For formulas, use a reference from the datasheet or an independent calculation, not a copy of the code.
3. Classes with FreeRTOS infinite tasks — `fake::findTask("name")` + `fake::runTask(task, n)`; the STM32 flight task is driven the same way.
4. The whole firmware with a different sensor kit or a different board — a separate suite that, before `#include "../../../src/main.cpp"`, sets `SENSOR_KIT` (or `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); the chips come from `helpers/ChipEmulators.h`. For STM32 — `test/native_stm32/`.
5. A new autopilot mode — a closed-loop flight scenario in `test_sim`.
6. A new suite — a folder `test/native/test_<name>/test_main.cpp` with `main()`; `setUp()` calls `resetWorld()` if the suite does not need state between tests.
7. Found a bug — first write a test that catches it, then fix it.
