# OpenPlaneProject pilot's guide

> 🌐 This page is a translation of the [Russian original](../../PILOT_GUIDE.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

This is a practical "what to connect where and how to fly" guide for people who hold a soldering iron and a transmitter rather than read source code. If you want to understand the architecture of the code, see the other documents in the repository. Here there is only hardware, channels, firmware and flying.

Repository: https://github.com/damir-lebedev/OpenPlaneProject, branch `main`.

Let's be upfront: the project is under active development and is **not a finished product**. The first prototype has already flown, but with caveats, which are described in a separate section below. Read it before flying, not after.

---

## Contents

1. [What hardware you need](#what-hardware-you-need)
2. [Choosing a board and pinout](#choosing-a-board-and-pinout)
3. [Connecting the receiver](#connecting-the-receiver)
4. [RC channel map](#rc-channel-map)
5. [Autopilot channels](#autopilot-channels)
6. [ARM and failsafe](#arm-and-failsafe)
7. [Flashing the board](#flashing-the-board)
8. [Web dashboard in the field](#web-dashboard-in-the-field)
9. [Black box](#black-box)
10. [Pre-flight checklist and safety](#pre-flight-checklist-and-safety)
11. [Troubleshooting](#troubleshooting)
12. [Current state of the prototype airframe](#current-state-of-the-prototype-airframe)

---

## What hardware you need

The kit of the current build (the firmware has been verified on it):

- **An STM32H743 DevEBox H743 board** (MCUDEV) — the main one. The former main board, the ESP32-S3 below, is supported too.
- **An ESP32-S3 N16R8 board** (a DevKitC-1 clone with two USB-C ports: "USB" and "COM").
- **An FS-i6 transmitter + FS-iA6B receiver** (iBUS protocol, 10 channels). You need one data wire — the iBUS SERVO port. The transmitter must be able to set failsafe — this setting is mandatory, see the failsafe section.
- **2 MG90S servos** for the ailerons — one for each wing panel (two independent servos, not one for both wings).
- **1 MG90S servo** for the elevator.
- **1 MG90S servo** for the rudder — the landing gear's steering wheel is mounted on the same shaft (steering on the ground).
- **An electronic speed controller (ESC)** 60–80 A with a 5 V BEC (the BEC powers the servos and the receiver).
- **A D3548 1100KV motor** + **a 10x5 propeller**.
- **A 3S LiPo battery**.

Autopilot sensors (all on I2C; without them the aircraft flies in manual mode):

- **GY-521** — gyroscope + accelerometer (the board may carry an MPU6050 or, as in our case, an MPU6500 — both are supported).
- **BMP581** — barometer (the former BMP388 is supported too).
- **GY-273** — compass (ours has a QMC5883P on it; the QMC5883L is supported too).
- Optionally an **OLED 128×64 SSD1306** (I2C) — an on-board status screen.

Airframe: wingspan 1200 mm, chord 250 mm, NACA 4412 airfoil, PETG construction (3D-printed). The first prototype flew on an ESP32-C3, a D2212 1000KV motor and a 40 A ESC.

---

## Choosing a board and pinout

The firmware supports four boards; switching takes one build parameter (`pio run -e <environment name>`). Each board has its own pinout, hard-coded in the firmware for the specific environment — do not rearrange wires on your own; look at the table for your board.

> **Important:** the main board is now the **STM32H743 (DevEBox H743)** — boot, the USB console, the SD card, iBUS, servos and the motor have been verified on it; the sensors are being connected for the first time. **esp32-s3 (N16R8)** is the former main board; its pinout has been verified on the bench with all sensors. **esp32-c3** is the old flown prototype. The **esp32-dev** pinout was chosen from the chip documentation and **has not been checked on real hardware**.

### STM32H743 (DevEBox H743) — the main board

`pio run -e stm32h743-devebox`, an MCUDEV DevEBox H743 board (STM32H743VIT6). This is the default board (`default_envs = stm32h743-devebox`). Boot, the USB console, the SD card and the black box, iBUS reception, ARM, servos and the motor from the transmitter have already been verified on it; the sensors are being connected for the first time.

| Purpose | Pin |
|---|---|
| Aileron, left wing panel | PA0 |
| Aileron, right wing panel | PA1 |
| Elevator | PA2 |
| ESC (throttle) | PA3 |
| Rudder + steering wheel | PD14 |
| iBUS from the receiver (RX) | PE7 |
| Sensor I2C SDA / SCL (MPU, BMP581, compass) | PB11 / PB10 |
| OLED I2C SDA / SCL (separate bus) | PB9 / PB8 |
| GPS: RX (← GPS TX) / TX (→ GPS RX) | PD9 / PD8 |
| MAVLink telemetry (radio modem): RX / TX | PD0 / PD1 |
| AUX1 / AUX2 (servos), buzzer | PD15 / PE9, PE15 |
| Sensor SPI SCK / MISO / MOSI, IMU CS / barometer CS | PB13 / PB14 / PB15, PB12 / PD10 |
| Battery / current sensor (ADC, recorded by the black box) | PC0 / PC1 |

The console, the log and the black box download go over the board's USB-C (a virtual COM port). Keep free: PA11/PA12 (USB), PA13/PA14 (SWD), PC8–PC12 and PD2 (the µSD slot), PE3 and PC5 (the K1/K2 buttons).

Connecting the sensors on the bench (all modules run from **3.3 V**, not 5 V):

| Module | Pins |
|---|---|
| MPU-6050 / GY-521 (the board may carry an MPU6500 — that is fine) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, AD0–GND, INT/XDA/XCL — leave unconnected. A standalone MPU-6500 module (10 pins): the same, plus **NCS–3.3V** (otherwise the chip switches to SPI) and FSYNC–GND; EDA/ECL — leave unconnected. Chip facing up, the X arrow pointing to the nose; the rotation of the chip's axes is `IMU_ROTATION_CW_DEG` in `Config.h` (90 on our clone) |
| BMP581 | VCC–3.3V (**3.3V only**: many modules have no regulator of their own), GND–GND, SCL–PB10, SDA–PB11, **SDO–GND** (address 0x46; do not leave it floating), **CSB–3.3V** (otherwise the chip switches to SPI), INT — leave unconnected |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, DRDY — leave unconnected. Keep it away from the servo, ESC and motor wires |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–PB8, SDA–PB9 |

The servos are powered **not from the board** but from the ESC's BEC (or from a separate 5 V supply rated at 2 A or more); the grounds of all sources are common. Do not connect the ESC's red wire to the board's 5 V while USB is plugged in.

### esp32-s3 (N16R8) — the former main board, verified on the bench

`pio run -e esp32-s3`, board `esp32-s3-devkitc-1` with settings for the N16R8 module (16 MB of flash, 8 MB of octal PSRAM).

| Purpose | GPIO |
|---|---|
| Aileron, left wing panel | GPIO4 |
| Aileron, right wing panel | GPIO5 |
| Elevator | GPIO6 |
| ESC (throttle) | GPIO7 |
| Rudder + steering wheel | GPIO18 |
| iBUS from the receiver (RX) | GPIO17 |
| Sensor I2C SDA / SCL (MPU, BMP581, compass) | GPIO41 / GPIO42 |
| OLED I2C SDA / SCL (separate bus) | GPIO1 / GPIO2 |
| Reserved: GPS RX / TX | GPIO39 / GPIO40 |
| Reserved: AUX1 / AUX2 (servos), AUX3, buzzer, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Battery / current sensor (ADC, recorded by the black box); reserved: telemetry TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Bench only: SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ BMP388 CS — GPIO21) |

> The sensor bus used to be on GPIO8/9 — it was moved to 41/42 to match the flight controller board layout.
> On the bench: SDA 8→41, SCL 9→42.

Do not use: GPIO0/45/46 (the boot mode depends on them), 19/20 (USB), 26–32 (flash), 33–37 (PSRAM on the N16R8), 43/44 (the "COM" connector), 48 (the RGB LED). The free pins are already allocated as reserves — a carrier board with connectors for the future: [`FC_BOARD.md`](FC_BOARD.md).

Connecting the sensors on the bench (all modules run from **3.3 V**, not 5 V):

| Module | Pins |
|---|---|
| MPU-6050 / GY-521 (the board may carry an MPU6500 — that is fine) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL — leave unconnected. A standalone MPU-6500 module (10 pins): the same, plus **NCS–3.3V** (otherwise the chip switches to SPI) and FSYNC–GND; EDA/ECL — leave unconnected. Chip facing up, the X arrow pointing to the nose; the rotation of the chip's axes is `IMU_ROTATION_CW_DEG` in `Config.h` (90 on our clone) |
| BMP581 | VCC–3.3V (**3.3V only**: many modules have no regulator of their own), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (address 0x46; do not leave it floating), **CSB–3.3V** (otherwise the chip switches to SPI), INT — leave unconnected |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY — leave unconnected. Keep it away from the servo, ESC and motor wires |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

The servos are powered **not from the board** but from the ESC's BEC (or from a separate 5 V supply rated at 2 A or more); the grounds of all sources are common. Do not connect the ESC's red wire to the board's 5 V while USB is plugged in.

### esp32-c3 — the old prototype, flown

`pio run -e esp32-c3`, board `esp32-c3-devkitm-1`.

| Purpose | GPIO |
|---|---|
| Aileron, left wing panel | GPIO5 |
| Aileron, right wing panel | GPIO4 |
| Elevator | GPIO6 |
| ESC (throttle) | GPIO7 |
| Rudder | — (no free pins) |
| iBUS from the receiver (RX) | GPIO8 |
| I2C SDA (sensors) | GPIO1 |
| I2C SCL (sensors) | GPIO3 |

### esp32-dev (the ordinary classic ESP32, 38 pins) — for the bench and debugging, NOT FLOWN

`pio run -e esp32-dev`, board `esp32dev`.

| Purpose | GPIO |
|---|---|
| Aileron, left wing panel | GPIO13 |
| Aileron, right wing panel | GPIO14 |
| Elevator | GPIO27 |
| ESC (throttle) | GPIO26 |
| Rudder | GPIO25 |
| iBUS from the receiver (RX) | GPIO16 |
| I2C SDA (sensors) | GPIO21 |
| I2C SCL (sensors) | GPIO22 |

Its advantage is that it is the most common and cheapest board in the family — suitable for bench debugging of the firmware, but its pinout has not been checked on hardware.

---

## Connecting the receiver

All you need from the receiver is **one iBUS data wire**, which on most FlySky-compatible receivers is brought out to a separate port (often labeled "iBUS", or it is the only non-PPM output). Connection:

- **Receiver TX (iBUS output)** → the board's **RX pin** from the table above (GPIO8 on the esp32-c3, GPIO17 on the esp32-s3, GPIO16 on the esp32-dev).
- **Receiver ground (GND)** → the board's **GND**. This is mandatory; without a common ground the protocol will not work.
- **Receiver power** — from a separate BEC/regulator or from the board's 5 V, depending on how you usually power the receiver in your builds; it is not tied to any particular firmware pin.

The firmware sends nothing back to the receiver — it only listens, so the board's TX line does not need to be connected anywhere.

The iBUS port speed in the firmware is 115200 baud, which is standard for the protocol; there is no need to change it, because the receiver holds that speed itself.

---

## RC channel map

The map was verified on the bench with an FS-i6 transmitter (10 channels, mode 2) and an FS-iA6B receiver.

| Channel | Transmitter control | Name | What it does |
|---|---|---|---|
| CH1 | right stick ←→ | AILERON | Roll — ailerons (2000 = right) |
| CH2 | right stick ↑↓ | ELEVATOR | Pitch — elevator (2000 = stick forward, nose down) |
| CH3 | left stick ↑↓ | THROTTLE | Throttle. 1000 µs = off, 2000 µs = maximum, no limiting |
| CH4 | left stick ←→ | RUDDER | Rudder and the landing gear's steering wheel (one servo) |
| CH5 | SwA | ARM | The ARM switch — see the ARM section below |
| CH6 | SwB | SWB | Flaps by default: down, toward you — deployed, up — retracted (see below) |
| CH7 | SwC (3 positions) | SWC | Mode by default: up MANUAL, middle STABILIZE, down AUTO_TAKEOFF |
| CH8 | SwD | SWD | RTH (home) by default, while switched on |
| CH9 | VrA | VRA | Stabilization strength by default |
| CH10 | VrB | VRB | Cruise speed by default |

CH6–CH10 can be **anything, in one line** of `include/config/Controls.h`: any of the 12 modes, 10 functions (flaps, brake, payload drop, geofence, buzzer…) and 7 knobs. When the board powers up, the actual layout is printed to the serial monitor. Everything about modes and bindings is in [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

All channel values use the standard receiver pulse range of 1000–2000 µs, with 1500 µs as center/neutral.

### Flaps (flaperons)

There are no separate flaps — the ailerons play their role. **SwB down** (toward you): both ailerons smoothly, in about a second, lower by the same angle (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° of servo horn rotation — changed in `include/config/Config.h`). This increases lift for takeoff and landing at a lower speed. Roll from the stick and from the autopilot works as usual — the ailerons move in opposite directions, but now around the lowered position. **SwB up** — they retract just as smoothly. On the OLED, `FL` lights up in the first line while the flaps are deployed.

At full roll with the flaps deployed, the aileron that moves down reaches its travel limit before the one that moves up — this is normal and works as aileron differential.

Deploying flaps usually raises the nose — be ready to ease the stick forward a little; if the effect is strong, tune it by reducing `FLAPS_DEPLOYED_US`.

---

## Autopilot channels

In short — the default layout (`include/config/Controls.h`):

| Switch | What it does |
|---|---|
| **SwC** (CH7) | up **MANUAL** · middle **STABILIZE** · down **AUTO_TAKEOFF** |
| **SwD** (CH8) | **RTH** — home, while switched on |
| **SwB** (CH6) | flaps |
| **VrA / VrB** (CH9/10) | stabilization strength / cruise speed |

- **STABILIZE — "the stick sets the angle".** Full stick is a 45° roll and 25° pitch; let go and the airplane levels itself. The throttle is yours.
- **AUTO_TAKEOFF.** After ARM nothing happens until you raise the throttle above half yourself. Then: 0–1 s — throttle smoothly up to 100%, wings level; 1–3 s — pitch +15°; after that +10° until you flip SwC. Throttle = the maximum of the stick and the program.
- **RTH.** Heading to the ARM point, altitude 40 m, circles above home. Needs a GPS with a 3D fix **before ARM**.

All 12 modes (ALT_HOLD, ACRO, CRUISE, LOITER, hand LAUNCH, AUTO_LAND, SOARING, RESCUE…), what each one needs in terms of sensors and how to put it on a switch are in [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). A mode selected from the dashboard or the ground station stays in effect until you flick the mode switch.

Stabilization moves the control surfaces even when the aircraft is **not armed** — this way, on the bench, you can see which way it responds to a tilt. The integrator does not accumulate in this case.

Angle signs (as on the OLED and in the log): **roll R > 0 — right wing down, pitch P > 0 — nose up.**

---

## ARM and failsafe

### ARM procedure

- **ARM:** throttle (CH3) at the bottom → switch **SwA (CH5) down, toward you** (on the FS-i6 this is CH5 = 2000; up = 1000). `ArmingManager: ARM` appears in Serial, and `ARMED` on the OLED.
- **DISARM:** SwA up, away from you — instantly, at any moment; the motor stops immediately.
- If SwA is moved to ARM with the throttle not at the bottom, or the pre-flight checks have not passed, ARM does **not** happen (the reason is printed in Serial). You need to return SwA up, lower the throttle and move it down again.
- If the board was powered on with SwA already in the ARM position (down), it does **not** arm: the firmware must first see SwA up (OFF).

**ARM really blocks the throttle:** while the aircraft is not armed, the ESC throttle is forcibly held at the minimum regardless of the stick. Before arming, the firmware checks that the sensors required by the selected mode respond — for example, STABILIZE without a live IMU will not arm until you switch to MANUAL. Still, behave as if the propeller could start spinning at any moment after ARM.

### Failsafe

Loss of link has **absolute priority** over everything else:

- **the aircraft is armed, GPS and home are available** — **return home with the motor** (as in ArduPilot/INAV): heading to home, altitude 40 m, circles above home until the link returns. The OLED shows `FSRTH`. Disabled with `FAILSAFE_RTH = false` in `Config.h`;
- **the aircraft is armed, no GPS** — **gliding**: the motor is off, the autopilot in any mode, even MANUAL, keeps the wings level and the nose slightly below the horizon (−3°), and the flaps retract. The OLED shows `RX LOST ... GLIDE`. If you set `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, the airplane will circle above you in a shallow spiral;
- **not armed** (on the ground) or the IMU does not respond — the motor is off, the ailerons, elevator and rudder go to neutral (1500 µs); after 10 s without a link the "I'm here" buzzer beeps (if it is soldered on).

The firmware recognizes loss of link in two ways:

1. **No iBUS frames for longer than 500 ms** — a broken wire or an unpowered receiver.
2. **Throttle below 950 µs** — this is how the receiver reports that it has lost the transmitter. **This requires setting up failsafe in the transmitter** (see below): when the link is lost, the FS-iA6B does NOT stop sending frames but repeats the last stick values — without the setup the firmware will not see the loss of link, and the airplane will keep flying with the last throttle.

ARM is **not** reset on failsafe: when the link returns, the airplane obeys the sticks and the selected mode again without re-arming (flipping the switch in the air with the throttle at zero is more dangerous).

Bench check (propeller removed): ARM → switch the transmitter off → the OLED shows `RX LOST ... GLIDE`, the motor has stopped; tilt the airplane — the control surfaces should return it to level. Switch the transmitter on — `RX ok`, and control is back with the sticks.

### Setting up failsafe in the FS-i6 transmitter (mandatory, once)

The idea: when the link is lost, the receiver should output a throttle of ~900 µs — below the normal minimum of 1000.

1. `Menu → Functions setup → End points` → channel 3: set the lower point (the left value) to **120%**. Save (long press of Cancel).
2. Throttle — **all the way down**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, throttle still down → save with a long press of Cancel. The receiver will remember ~900 µs.
4. Go back to `End points` → channel 3 → set the lower point back to **100%**. Save.
5. Check: ARM is not needed. Switch the transmitter off — about 1 s later `RX=LOST(failsafe пульта)` appears in Serial, and an inverted `RX LOST` line on the OLED. Switch the transmitter on — `RX=OK`.

Keep the throttle trim in the center: with the trim lowered far down, the throttle can drop below 950 and the firmware will take it for a loss of link.

There used to be a **boost on CH8** and a 40% throttle limit here — they have been removed: the limit protected a weak 3S1P build, and new batteries are not afraid of full throttle. The boost also used to switch itself on if SwD was up when the board was powered on.

---

## Flashing the board

The firmware is built with **PlatformIO** (Arduino framework, C++).

### Installing PlatformIO

The easiest way is to install the **PlatformIO IDE** extension in VS Code (Extensions → find "PlatformIO IDE" → Install); you get both the CLI and convenient build buttons in the interface. You can also install it with `pip install platformio` and work from the terminal — both options use the same `pio` commands.

### Building and uploading

Open the project (the repository folder) in VS Code with PlatformIO installed, connect the board over USB and run, in the terminal, the command for your board:

```bash
# STM32H743 DevEBox (main board)
pio run -e stm32h743-devebox -t upload

# esp32-s3 N16R8 (former main board)
pio run -e esp32-s3 -t upload

# esp32-c3 (old prototype)
pio run -e esp32-c3 -t upload

# esp32-dev (classic ESP32 38-pin, for the bench)
pio run -e esp32-dev -t upload
```

If you do not specify `-e` at all, the default board is built — `stm32h743-devebox`.

**STM32 DevEBox:** the first flash goes over USB DFU: a jumper BT0→3V3, press RST, then the command above (Windows needs the WinUSB driver for "STM32 BOOTLOADER", installed with Zadig). After that the `D` key in the console reboots the board into the bootloader by itself, and the jumper is no longer needed. The console runs over the same USB-C.

**esp32-s3:** the board has two USB-C connectors. Flashing and Serial go through the **"COM"** connector (a CH343 bridge; on Windows it shows up as "USB-Enhanced-SERIAL CH343"). The "USB" connector (the chip's native USB) is not needed for operation, but it may be plugged in — it interferes with nothing.

### Serial monitor

To see the debug output (channel states, ARM, sensors) right in the console over USB:

```bash
pio device monitor -b 115200
```

The speed must be 115200 — otherwise you will see unreadable garbage instead of text. Once every 10 seconds a `SYS` line is printed — the loop frequency (it should be ~500 Hz), the average and worst loop time over 10 s, the iBUS counters, free memory. Everything else comes through the channels enabled in the log menu (see below).

### Console: menu and log (serial monitor)

A keypress takes effect immediately; Enter is not needed (in a monitor that sends by line, type the letter + Enter). Calibrations and the output check work only when the aircraft is not armed.

| Key | What it does |
|---|---|
| `h` | **main menu** (text-based, items are numbered) |
| `l` | the "what to print to the log" menu |
| space | pause the log / resume |
| `s` | detailed status of all sensors (including the I2C error counters) |
| `i` | re-calibrate the gyroscope and run the IMU pre-flight check — 2 s, do not move the airplane |
| `o` | **IMU mounting calibration** — once, after installing the board in the airplane (see below) |
| `m` | compass calibration — rotate the board/airplane around all axes for 15 s. The result is saved to flash and survives a reboot |
| `p` | output check: the actual pulse on each pin |

**Logging by channel.** Each kind of data is its own line with its own prefix, and each has its own mode: **off**, **on change** (a line appears only when the values have really changed — stick jitter and sensor noise do not count) or **continuous** (every 0.2 / 0.5 / 1 / 2 s — the period is set in the same menu).

| Channel | What it shows | Default |
|---|---|---|
| `STAT` | link, ARM, mode, flaps, whether the IMU/barometer are OK | on change |
| `RC` | transmitter channels, µs | off |
| `OUT` | outputs to the control surfaces and the ESC, µs | off |
| `ATT` | roll, pitch, heading | off |
| `AP` | autopilot: targets and corrections | off |
| `ALT` | altitude, vertical speed, ALT_HOLD target | off |
| `MAG` | compass heading | off |
| `GPS` | fix, satellites, coordinates, speed | off |
| `IMU` | gyroscope and accelerometer | off |
| `SYS` | loop frequency and time, memory (every 10 s) | on |

While the menu is open, the log stays silent so that the menu does not scroll away; after you leave, all enabled channels are printed again. The selection is saved to flash when you leave the menu and survives a reboot. While armed, it takes effect immediately but is written only after DISARM: a flash write stops the flight loop for ~0.4 s.

### IMU mounting: any way you like, one calibration

The board with the IMU can be installed in the airplane **however is convenient** — at any angle, on its side, upside down: the firmware works out by itself where its nose and top are. This is done **once** after installation (and again if the board has been moved):

1. The airplane on the table, no transmitter needed, the motor not armed. In the serial monitor press `o`.
2. **Step 1:** the airplane stands level, as in level flight (for an airplane with a tail wheel, put something under the tail). Do not touch it for ~3 s.
3. **Step 2:** raise the **nose** by 30–60°, wings level, and hold still for ~1 s.
4. **Step 3:** nose back down, lower the **right wing** by 30–60°, and hold for ~1 s.

Each step is counted by itself (the log says "засчитано", that is, "counted"). At the end it shows what came out ("нос = +Y чипа, верх = −Z чипа", that is, "nose = +Y of the chip, up = −Z of the chip") and "установка сохранена" ("mounting saved"). If you mixed something up (lowered the nose instead of raising it, the left wing instead of the right), the calibration is rejected with an explanation; just repeat `o`. The result is stored in flash and survives a reboot. Check: nose up → P on the OLED goes positive; right wing down → R goes positive.

Until there is a mounting calibration, the old method applies: the board must lie chip-up, and the rotation is `IMU_ROTATION_CW_DEG` in `Config.h`.

### Pre-flight check at power-up

On every power-up the IMU spends ~2 s calibrating the gyroscope and checks itself at the same time:

- **the airplane is still** — if it was being held in hands or moved at that moment, the gyroscope offset will be wrong;
- the accelerometer at rest reads 1g;
- **"up" matches the mounting calibration** — if the board has been moved or turned over, this is visible immediately (an airplane on its tail wheel or on a slope is not a problem; the tolerance is 45°).

**Power the airplane on while it stands still.** It does not have to be level if the mounting has been calibrated (otherwise the position at power-up becomes the horizon). The result is in the log: `предполётная проверка пройдена` ("pre-flight check passed") or `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` ("PRE-FLIGHT CHECK FAILED" followed by the reason). If it failed, **ARM in STABILIZE and AUTO_TAKEOFF is forbidden** (the reason is printed when you try), and the autopilot gives no corrections in any mode, including gliding on link loss: with wrong angles it would steer the wrong way. ARM in MANUAL is still possible. To fix it: put the airplane down motionless and reconnect the battery (or press `i`); if the board has been moved, press `o`.

### OLED screen

If the OLED is connected (GPIO1/GPIO2), it shows the following 5 times a second:

```
RX ok disarm STAB        <- link / ARM / mode (when the link is lost the line is inverted)
R  +1.2 P  -0.4          <- roll / pitch, °
Alt +0.3 Vz +0.1         <- altitude from the power-up point, m / vertical speed, m/s
Hdg 123  Thr 1000        <- compass heading / throttle to the ESC, µs
L1500 R1500 E1500        <- PWM of the ailerons and elevator, µs
Loop 500Hz max 1100us    <- frequency and worst loop time
```

---

## Web dashboard in the field

The board brings up its own Wi-Fi access point — no home router or internet is needed, and everything works right in the field from a phone.

**How to connect:**

1. On a phone or laptop, open the list of Wi-Fi networks.
2. Connect to the network **`OpenPlane-Debug`**, password **`12345678`**.
3. Open the address **`http://192.168.4.1`** in a browser.

No app needs to be installed — it is an ordinary web page.

**What you can do from a phone in the field, with no programming at all:**

- Watch the **live bars of all 10 RC channels** — handy for checking that the transmitter and receiver really send what you move on the stick, even before connecting the servos.
- See the status of each output (left/right aileron, elevator, ESC) — whether the channel is connected in software.
- See whether the sensors (IMU, barometer) respond, if you have them soldered in — it honestly shows either real data (roll/pitch/altitude) or an explicit note that the sensor is physically absent or not responding.
- **Change the autopilot mode** with buttons (manual / stabilize / auto takeoff / altitude hold) right from the page — without the transmitter.
- **Tune the PID controller coefficients** (for roll and pitch) through a form on the page — useful for gradually tuning stabilization without re-flashing.

The range of this access point is, in practice, tens of meters; it is an ordinary ESP32 Wi-Fi, not long-range telemetry. It is a tool for tuning on the table, on the bench and next to the field — not for controlling the aircraft in flight at a distance.

---

## Black box

The ESP32-S3 firmware records every flight to flash by itself: everything the sensors saw, what the sticks did, where the servos went and what the autopilot decided. There is nothing to do:

- it **records** from the moment the aircraft is armed and the throttle is raised (plus the 10 s before that);
- it **stops** 10 s after DISARM — or if an armed aircraft sits motionless with the motor off for 30 s (it landed or crashed, and DISARM was forgotten);
- link loss, motor to zero and gliding do **not** stop the recording.

At power-up the serial monitor shows how much room there is for a flight (the console prints in Russian; the line below reads "waits for ARM and throttle | erased ahead 12.9 MB (≈11 min) of 13.9 MB | flights 1"):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**After the flight**, connect USB (the COM connector), close the serial monitor and download the flight:

```bash
python tools/blackbox.py download
```

The flight ends up in the `blackbox/` folder together with the decoded output: `summary.txt` (the summary and events), `events.txt`, and CSV tables per sensor. The black box erases old flights by itself when it needs space — **download after every flight**. The console menu is the `k` key. All the details are in [BLACKBOX.md](BLACKBOX.md).

**On the STM32H743 board** the black box records to an SD card: the card is formatted as FAT32 and prepared once on a PC (`python tools/blackbox.py sd-prepare E:`). After a flight it can be downloaded over USB with the same `download` command, or you can take out the card and decode the file straight from it: `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Pre-flight checklist and safety

Read this section in full **before** the first power-up, not after an incident.

### Mandatory before any bench test

- [ ] **The propeller is physically REMOVED** if you are checking channels, ARM, the dashboard, tuning the PID, or simply powering the board up for the first time on a new pinout. The ESC can jerk the motor at any stage of a test — this is not a hypothetical risk, it is normal behavior on first power-up.
- [ ] The LiPo battery has been checked for swelling and damage, charged with a proper LiPo charger, and is stored and charged on a non-flammable surface.
- [ ] The transmitter is on and its channels have been checked on the dashboard (`http://192.168.4.1`) **before** the battery is connected to the ESC.

### Before flight

- [ ] An open area, with no people or buildings within a radius sufficient for an airplane with a 1200 mm wingspan under manual control with abnormal behavior of the servos or the wing (see the section on the prototype's limitations below — the motor mount and the wing have not yet been reinforced with carbon).
- [ ] All three surfaces move in the right direction — check on the table before every flight, and do not rely on your memory from last time:
  - right stick to the right → **right aileron up, left aileron down**;
  - right stick toward you → **elevator up**;
  - left stick to the right → **rudder and wheel to the right**;
  - SwB (flaps) down → **both ailerons smoothly down**, and roll from the stick still moves them in opposite directions;
  - in STABILIZE tilt the airplane right wing down → **right aileron down, left aileron up** (the surfaces return it to level); nose down → **elevator up**.
  If something is wrong, change the corresponding `*_REVERSED` in `include/config/Config.h` (the "Servo direction" section), not the reverse on the transmitter: otherwise the stick and the autopilot will disagree.
- [ ] At power-up the airplane stood still, and the log or dashboard does not show `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА`; as you tilt the airplane in your hands, P and R on the OLED change in the right direction.
- [ ] Failsafe is set up in the transmitter and tested: switch the transmitter off → `RX LOST` on the OLED/in Serial (see the failsafe section).
- [ ] The transmitter's range has been checked, and the transmitter battery is charged.
- [ ] At power-up the log shows `BlackBox: ... стёрто впереди N МБ (≈M мин)` — there is enough room for the flight. The previous flight has been downloaded (`python tools/blackbox.py download`), if you need it.
- [ ] Keep your hands and face away from the propeller whenever a LiPo is connected to the ESC — after ARM the throttle responds to the stick immediately, with no further warnings. DISARM — SwA up.
- [ ] Make sure you can physically disconnect the power quickly (access to the LiPo connector) instead of relying only on failsafe on signal loss.

### General LiPo safety

- Never leave a charging LiPo unattended.
- Do not connect or disconnect the LiPo to the ESC while standing next to the plane of the propeller's rotation.
- Transport and store LiPos in a protective bag or case.

---

## Troubleshooting

**"No receiver signal" / RX looks fine, but the channels on the dashboard do not move**
Check that the receiver's data wire is connected exactly to the iBUS RX pin from your board's table (GPIO8/GPIO17/GPIO16), not mixed up with ground or power, and that the receiver and board grounds are joined. If the pinout matches and the wires are intact but there is still no signal, check that the receiver is bound to the transmitter at all and that the receiver's output is configured for iBUS, not PPM/SBUS.

**A sensor (IMU, barometer, compass) shows "not responding" / NO_RESPONSE**
The firmware honestly reports that the sensor does not respond instead of outputting zeros. Check: (1) the module's power — 3.3V and the board's ground; (2) SDA/SCL — on the I2C pins of your particular board; (3) the address on the line: MPU 0x68 (AD0 to GND), BMP581 0x46 (SDO to GND, CSB to 3.3V; 0x47 if SDO is on 3.3V), BMP388 0x76 (SDO to GND, **CSB to 3.3V** — otherwise the chip is in SPI mode), QMC5883P 0x2C, QMC5883L 0x0D. If a sensor responds sometimes and sometimes not (or answers at someone else's address), it is a bad contact on the breadboard: press down on VCC/GND/SDA/SCL, and it is best to power each module straight from the board's 3.3V/GND. The `s` command in the console shows the I2C error counters for each sensor.

**The angles on the OLED are mixed up (nose up changes R, not P) or have the wrong sign**
Do the IMU mounting calibration (`o`, see "IMU mounting") — it does not depend on how the chip is soldered on the module or how the module sits in the airplane. Without it: on GY-521 clones the chip is sometimes soldered rotated relative to the printed arrows — rotate the axes in `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Check: nose up → P goes positive, right wing down → R goes positive.

**ARM refused: "IMU: ..."**
The IMU pre-flight check did not pass (see "Pre-flight check at power-up"): the airplane was moved at power-up — put it down motionless and reconnect the battery; "up" does not match the calibration — the board was moved, do `o`; "the board is not lying chip-up" — the mounting is not calibrated, do `o`.

**A servo or the ESC responds to the wrong stick / does not respond**
The `p` command in the console measures the real pulse on each output (GPIO4–7) and compares it with the expected one. If everything is "OK" but the servo does not move, the problem is beyond the board: (1) the servo is not powered (BEC/5V, common ground); (2) the signal wire is on the wrong pin; (3) the mechanism is jammed. "НЕ СОВПАДАЕТ" ("MISMATCH") means the problem is in the firmware or peripherals — report it to the developer.

**The motor does not spin at all, although the throttle on the OLED/dashboard follows the stick**
Check that the LiPo is connected to the ESC and that the aircraft is really armed — until ARM the throttle to the ESC is forcibly held at the minimum, and that is not a fault. An ESC that saw a non-minimum throttle at power-up may beep constantly and refuse to arm — reconnect the battery with the throttle down. If `ArmingManager: ARM` does not appear after SwA is moved down, look at Serial — the firmware prints the reason (the throttle is not down, a sensor is not responding, one needed by the mode currently selected on CH7); return SwA up, fix the cause and move it down again.

**The motor stalls or jerks while spooling up**
If the ESC is powered from a bench power supply, you are hitting the supply's current limit: even without a propeller the motor briefly draws several amps, the voltage sags and the ESC restarts. Raise the current limit or use a LiPo. Spikes from such a restart can hang the board's USB bridge (the "COM" port stops opening) — reconnect the cable.

**After flashing, the board does not respond / the `OpenPlane-Debug` access point does not appear**
Make sure the upload (`pio run -e <your board> -t upload`) finished without errors and that you flashed exactly the environment you are physically holding (the esp32-c3 differs from the esp32-s3 and the esp32-dev not only in pins but also in the chip — firmware for a different chip will not install on the board, or will install incorrectly). Check the serial monitor output (`pio device monitor -b 115200`) right after the board reboots — it prints which stage of setup() the board is at.

---

## Current state of the prototype airframe

To keep expectations honest:

- The first prototype **has already flown**. Problems were found: **insufficient strength of the motor mount** and **insufficient strength of the wing** — the wing needs carbon reinforcement. The servos also need further tuning. Take this into account when planning your own flights — this is not an abstract disclaimer but a real failure that has already happened on this prototype.
- **The esp32-s3 bench is built with all the sensors** (GY-521 with an MPU6500, a BMP388 on I2C, a GY-273 with a QMC5883P, an OLED) — all of them respond, and the loop runs at 500 Hz. The autopilot modes have been verified on the table but **have not yet been tested in flight**.
- The default barometer is now the BMP581 (the bench BMP388 was verified live; the BMP581 has not been tested on hardware yet). Altitude is relative to the power-up point. Absolute altitude above sea level is computed from the standard atmosphere, without a weather correction.
- The QMC5883P compass needs calibration (`m` in the console) on the assembled airplane — next to the motor and wires the offsets differ from those on the breadboard. The heading has no tilt compensation yet and is not used by any mode.
- The license is the OpenPlane License: MIT with mandatory credit to the author (Damir Lebedev), a ban on military use, and a ban on intentionally harming people or property without their consent, see [LICENSE](../../../LICENSE).

If you are building your own aircraft from this guide, fly it first under manual control (MANUAL), and only then move on to the autopilot.
