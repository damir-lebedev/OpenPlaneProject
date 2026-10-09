# Black box

> 🌐 This page is a translation of the [Russian original](../../BLACKBOX.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

The firmware records every flight by itself into the board's built-in flash: sensors, sticks, servo outputs, autopilot decisions, events. After the flight the recording is downloaded over USB and decoded into CSV tables.

It works on two boards:

| Board | Where it records | How much fits (at ~20 KB/s) |
|---|---|---|
| **ESP32-S3 N16R8** | a 13.9 MB partition of the built-in flash | about **11 minutes** |
| **STM32H743** (DevEBox — main; WeAct) | a file on an SD card, [below](#sd-card-stm32h743) | 64 MB — about **55 minutes**, the size is set by the file |

On the other boards (ESP32-C3, the ordinary ESP32) there is no storage: the black box is off and does not interfere with the flight.

---

## When it records

| | Condition |
|---|---|
| **Start** | Armed **and** the throttle is raised (the stick or the ESC above `THROTTLE_LOW_US`). The **10 s before that** are recorded too — the moment of ARM and standing before takeoff |
| | A reboot caused by a fault (panic, watchdog, power sag) — recording from the first cycle and for at least 60 s: if it happened in the air, you can see what came next |
| | Manually from the console (`k` → `r`) — for the bench |
| **Stop** | **10 s after DISARM** |
| | Armed, but the motor is stopped and the airplane has been **motionless for 30 s** — it landed or crashed, and DISARM was forgotten |
| | Manually (`k` → `r`) |
| **Not a stop** | Link loss, failsafe, the motor to zero in the air, gliding, landing without DISARM while the airplane is still rolling |

"Motionless" means all of this at once: rotation below 5 °/s on every axis, the accelerometer shows 1g ± 0.1, there is almost no vertical speed by the barometer, and by GPS and the pitot tube (if present) it is slower than 2 m/s. In flight, it is never this steady for 30 seconds in a row.

## What is recorded

| Record | Rate | What is in it |
|---|---|---|
| `IMU` | every cycle, 500 Hz | gyroscope (°/s), accelerometer (g), how long the control cycle's work took (µs) |
| `CTRL` | 100 Hz | roll/pitch/heading, the autopilot's targets, the pilot's sticks, the final commands, **all 7 outputs** (µs), the components of the roll and pitch PID (P, I, D), pilot and autopilot throttle, flaps, mode, flags (ARM, link, failsafe, sensors alive...), the features switched on |
| `RC` | 50 Hz | all 10 transmitter channels, iBUS frame counters (good and bad) |
| `BARO` | every sample (~50 Hz) | pressure, temperature, altitude, vertical speed, altitude target |
| `MAG` | up to 50 Hz | the field on three axes, heading |
| `GPS` | every solution | coordinates, altitude, speed, course, satellites, fix, accuracy |
| `AIR` | up to 50 Hz | pitot tube: pressure difference, indicated and true airspeed, density |
| `NAV` | 10 Hz | home (distance, bearing), course and course target, navigation speed, course source, hand-launch and soaring stages, auto-trim |
| `POWER` | 10 Hz | battery voltage and the current sensor output (the flight-controller board's dividers, [FC_BOARD.md](FC_BOARD.md), block B) |
| `SYS` | 1 Hz | frequency and worst cycle of the control loop, free memory, iBUS counters, IMU temperature, the black box queue, lost records, the longest flash write, free space |
| `EVENT` | on an event | ARM/DISARM, an ARM refusal with the reason, mode change, link lost/restored, sensor failed/recovered, GPS fix, home recorded, switch features, geofence, stall protection, hand-launch and soaring stages |

At the start of each flight come the parameters: the firmware (build date), the reason for the start and for the last reboot, which sensors are fitted and whether they passed the pre-flight check, the PID coefficients (including edits from the dashboard), the trims, important `Config` values and the switch bindings.

---

## How to use it

### Before the flight

There is nothing to do. At power-up the serial monitor shows the status (the console prints in Russian; the line below reads "waits for ARM and throttle | erased ahead 12.9 MB (≈11 min) of 13.9 MB | flights 1"):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

"Erased ahead" is how much will fit into the next flight. After power-up the black box spends a few seconds (after a long flight — up to a minute) preparing space: it erases old records. During that time the flight loop on the ground sometimes freezes for ~0.15 s — the surfaces may twitch late, which is normal. **In the air the flash is never erased.**

### After the flight — downloading

1. Connect USB to the **COM** connector. Close the serial monitor (it holds the port).
2. Run in the project folder:

   ```bash
   python tools/blackbox.py download          # the last flight
   python tools/blackbox.py download --all    # all of them
   python tools/blackbox.py list              # what is on the board
   ```

   `pyserial` is required: `pip install pyserial`. Or use PlatformIO's Python, which already has it: `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. The flight is downloaded into the `blackbox/` folder (~200 KB/s: 10 minutes of flight take about a minute) and is decoded right next to it, into a folder with the same name.

While the download is running, the flight loop is stopped, so it works only without ARM.

### What is inside a flight's folder

| File | What it is |
|---|---|
| `summary.txt` | The summary: duration, rates, ranges of angles, altitudes, speeds, voltages, the worst control cycle, lost records, all the events |
| `events.txt` | The flight's parameters and all the events by time |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | One table per record type |

The time in all the tables is `time_s`, seconds from the start of recording (ARM and throttle); the pre-recording is negative. The values are already in units: degrees, g, meters, m/s, microseconds of pulse. In `CTRL.csv` the mode is added by name (`mode_name`), the features as a list (`features_on`), and the flags are split into 0/1 columns (`armed`, `rx_lost`, `fs_glide`, `imu_ok`...).

The CSVs open in Excel/LibreOffice, but for plots against time [PlotJuggler](https://github.com/facontidavide/PlotJuggler) is more convenient: File → Load Data → CSV, the time column `time_s`.

A `.bbl` file is a raw image of the flash, and it can be decoded again: `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Console: `k`

In the serial monitor the `k` key opens the black box menu: status, the list of flights, `r` — start/stop recording by hand (to check on the bench), `e` — erase all flights (with `y` to confirm, ~40 s).

---

## Space on the flash

- Flights are written in a ring. When space runs low, on the ground the black box erases **the oldest flights whole** until 10 MB is free ahead (`BLACKBOX_MIN_FREE_BYTES`, ~9 minutes).
- **The last recorded flight is never erased** — only by the next recording, if it ran out of space.
- If the erased space runs out in the air, recording continues into a queue in PSRAM (4 MB, ~3 minutes of the latest data); after landing and DISARM the black box frees up space and writes it out. A flight longer than the whole partition (~11 min) does not fit entirely: the beginning is kept and the end is lost.
- That is why **you should download the flight after every outing** — especially the first one.

## Reliability

- A power cut at any moment (a crash, the battery came loose): everything is preserved except the last ~15 ms. Partially written records are discarded by CRC — in `summary.txt` this is the line "Недописанных записей (CRC)" (the summary is in Russian; it means "Incomplete records (CRC)").
- The flight number, the ring's head and the list of flights are restored from the sectors themselves: there is no separate "map" that could be corrupted.
- The download checks the CRC-32 of every sector and of the whole flight.

## Effect on the flight

- The flight loop only puts a snapshot into a queue in PSRAM — a few microseconds. A separate task on core 0 writes to flash, one page (256 bytes) at a time, **right after a control cycle**: a flash write halts both ESP32 cores for 0.6–0.9 ms, and it falls into the pause between cycles.
- Measured on the bench (a DevKit without sensors, two runs of 30–40 s of recording): the interval between cycles is 2.00 ms, 99.2–99.7% of the intervals are within 1.9–2.1 ms, the longest is 2.5 ms, and not a single cycle was skipped; the cycle's workload is the same as without recording. The cycle only jitters noticeably on the ground without ARM, while the black box verifies and erases space (reading a 64 KB block — a pause of ~3 ms, an erase — ~0.15 s).
- With sensors a cycle takes ~0.7 ms, and a page write still fits into the remaining 1.3 ms. Check after the first flight: in `summary.txt` the lines "Такт IMU" and "Цикл: худший такт" (in Russian: "IMU cycle" and "Loop: worst cycle").

---

## SD card (STM32H743)

On the STM32H743 the black box records to an SD card (a µSD slot on SDMMC1, 4 bits, 24 MHz). The card stays an ordinary **FAT32** card: in its root lies a pre-created file `BLACKBOX.BIN`, inside which the firmware writes raw blocks, and it never touches the FAT table or the directory. So there is nothing to corrupt when power is lost in flight, and the file can simply be copied to a PC.

### Preparing the card (once)

1. Format the card as **FAT32** (not exFAT; Windows offers FAT32 for cards up to 32 GB).
2. With the card in a card reader, on a PC:

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB, E: is the card's drive
   python tools/blackbox.py sd-prepare E: --size 256   # or larger
   ```

   The file is created as one contiguous piece on an empty card and filled with `0xFF`; the first sector is a service label "the ring is empty". If the file is not contiguous (the card is not empty and badly fragmented) or is missing, the console will show the reason at power-up and the black box is off.
3. Insert the card into the board. At power-up (the console prints in Russian: "SD card: 15204 MB, SDMMC 24 MHz, 4 bits; file BLACKBOX.BIN: ok", then the status line, then "ready in 300 ms"):

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   "Erased ahead" grows in the background: the board verifies the space at ~2.5 MB/s.

### Getting the flight

- **Through the board over USB** — as on the ESP32: `python tools/blackbox.py download` (the STM32 console is USB CDC, the download speed is ~400 KB/s, 1 MB takes less than 3 s). `list`, `--all`, `--flight N` work the same way.
- **By removing the card**: the `BLACKBOX.BIN` file from the card is decoded straight into CSV —

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # all flights -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # just list them
  ```

  The file is a ring of sectors: the utility assembles the flights from the sector numbers itself, including ones that wrapped past the end of the file.

### What was measured on the board

DevEBox H743 + a 16 GB card (the `test_blackbox_sd` test, [TESTING.md](TESTING.md#tests-on-the-stm32-board)):

| | |
|---|---|
| Card identification | 12–18 ms, 4 bits, 24 MHz |
| Writing a 256 B page | 2.3–3.7 ms on average, **worst 60–190 ms**, ~75–110 KB/s sustained (~20 KB/s needed) |
| Reading | a 4 KB sector — 4.2 MB/s; a random block — 0.6 ms |
| Erasing | 64 KB — 13 ms; the whole 64 MB area — 20–28 s |
| Power-up | with an empty ring — 0 ms (by the label); with flights — 0.3 s (a sampled check of ~530 reads); a full check of 64 MB would take ~20 s |
| 20 s of real-time recording (500 Hz IMU) | not a single lost record, 0 errors |
| The flight task while recording | period deviation of 2 ms — **1 µs** (a highest-priority simulator task next to the recording) |

The worst page write is the card's internal "housekeeping"; the queue in RAM (384 KB ≈ 19 s of the stream) survives such pauses. Cheap cards differ most in this: before flying, the card is worth checking with the `test_blackbox_sd` test (the worst write must be under 250 ms — the limit of the SD specification).

### How it differs from the ESP32 flash

- **The writer task** (`bbox`, priority 2) is preempted by the flight task (5) in the middle of a card access — rather than "in the cycle's pause", as on the ESP32 with its core halt. The transfer runs with SDMMC hardware flow control: without it the FIFO overflowed on preemption (on the board that was `HAL_SD_ERROR_RX_OVERRUN` and the console and recording freezing for seconds).
- **The queue is in RAM**, 384 KB (`BLACKBOX_RING_STM32_BYTES`), not 4 MB of PSRAM.
- **The power-up check is sampled**: the real ring sectors form one continuous arc, ~500 headers are read, and the boundaries of the arc and of the flights are refined by bisection. The result is the same as a full check; if the picture does not add up — a full one.
- **The "ring is empty" label** in the file's first sector: so that an empty area is not verified for seconds at every power-up. It is set when everything is erased and when a full check found nothing; it is cleared before the first write.
- **Card errors** (pulled out, a bus fault) go into the log once per second as the event "носитель: ошибок записи …" (in Russian: "storage: write errors …"); a lost page leaves a `0xFF` hole, and decoding of the sector stops at it (just as in `tools/blackbox.py`), while the other sectors are intact.

## Settings (`include/config/Config.h`, the "Black box" section)

| Constant | Default | Meaning |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | The queue in PSRAM (without PSRAM — `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 KB) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32: the queue in RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 MB | STM32: the file on the card and the ceiling of the part that is used |
| `BLACKBOX_PREROLL_MS` | 10 000 | How much to record before the start |
| `BLACKBOX_POSTROLL_MS` | 10 000 | How much to record after DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Standing motionless while armed — stop |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0.1, 0.5, 2 | What counts as "motionless" |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Recording after a fault reboot — no shorter than this |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | How much to keep erased for the next flight |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | The pause between erases on the ground |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU every Nth cycle: 2–250 Hz and ~25% longer recording |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6.6, 1.667 | The battery and current-sensor dividers on the board |

The partition table is `partitions_blackbox.csv`: the application 2 MB (the firmware is now ~0.9 MB), the black box 13.9 MB, coredump 64 KB. The NVS partition stayed where it was — the IMU and compass calibrations and the trims are preserved after moving to this table. There is no second slot for over-the-air (OTA) updates.

---

## For developers

The code is `include/telemetry/BlackBox*.h`:

| File | What |
|---|---|
| `BlackBoxFormat.h` | The format: the sector header, record types and structures, field schemas, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | A ring of sectors on `IFlashRegion`: finding the head at power-up, the list of flights, page-by-page writing, erasing old flights in steps |
| `BlackBoxRing.h` | A queue of records between the cores (spinlock), drops the oldest |
| `BlackBox.h` | Snapshots in the loop, start/stop, events, the writer task, download over UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` on top of `esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` on top of a file on a FAT32 card: finding the file (FAT is read-only), partial blocks, erasing with `0xFF`, the "ring is empty" label |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice`: SDMMC1 on `HAL_SD` (polling, 4 bits, hardware flow control) and the pins |
| `hal/ResetCause.h` | The reboot cause on ESP32 and STM32 (`RCC->RSR`) |

### Format on the flash

A 4 KB sector = a 16-byte header (`magic "OPBB"`, a running `seq`, `millis()` at the time of opening, the flight number, the format version, a check byte) + records one after another. A record does not cross a sector boundary; the tail of the sector is `0xFF`.

A record: `[type u8][length u8][data][CRC-8]`, the data begins with `t_us` (`micros()`). The first records of a flight are `SCHEMA`: the text `"16 IMU t_us:I gx:h/10 ..."` — the field name, the Python `struct` character, the divisor. The decoder takes the fields from the log, so a new field in a record means editing the structure and the schema string in `BlackBoxFormat.h` (their sizes are cross-checked by `static_assert`); the decoder does not need to change.

### Download protocol

Commands are a line after the STX byte (`0x02`), and the console hands it to the black box:

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (at 115200), then switches to 2 Mbaud
PC:  switches to 2 Mbaud, sends 'G'
FC:  234 frames: A5 5A, u16 number, 4096 bytes of the sector, u32 CRC-32
     returns to 115200, BB:DONE n=3 crc=<CRC-32 of all the sectors>
```

### Tests

`pio test -e native -f native/test_blackbox_scan` — a sampled ring check against a full one on random histories (300 rings × 5 probe steps) and the cost on an SD area. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` and `test_app_stm32_*` — FAT32, the area, the card driver, the black box on the card, the whole STM32 firmware. On the board — `pio test -e stm32h743-devebox -f test_blackbox_sd` (a real card).

`pio test -e native -f native/test_blackbox` — the format, the ring on a NOR fake of the partition (erasing by sectors, a write only lowers bits — raising a bit counts as an error), reboots, power cuts, recording a flight on the real `FlightController`/`Autopilot`, start/stop, events, flash overflow in the air, download. A flight image for checking the decoder: `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
