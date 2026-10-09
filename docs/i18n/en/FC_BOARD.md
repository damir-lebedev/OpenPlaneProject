# Flight controller board: connector blocks

> 🌐 This page is a translation of the [Russian original](../../FC_BOARD.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is. The translation was made by AI and has not been checked by native speakers. Please report mistakes to [Damir Lebedev](https://github.com/damir-lebedev) or in the [issue tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues).

A carrier board for the ESP32-S3 DevKitC-1 (N16R8): the DevKit plugs into two female pin-header strips, with blocks of JST-XH connectors around it. This document answers three questions: which connectors to assemble into blocks, where to put the capacitors, and what plugs in where. The pins match `include/config/Config.h` (the `BOARD_ESP32_S3` block).

The board is designed to be **single-sided**: the GPIOs are chosen so that the pins of each block run consecutively along the DevKit's header, and the signal traces fan out without crossing. I have not checked the routing in a CAD tool. If somewhere it does not work out, put in a wire jumper on the component side: one to three of them on a board like this is fine.

**All connectors are JST-XH, 1–5 pins.** Wires are not soldered to the board: mating housings are crimped onto the wires of the servos, ESC, receiver and modules. XH is keyed, so it cannot be plugged in backwards. One pin carries ~3 A.

---

## 1. Layout diagram

A top view, from the component side. The DevKit's USB connectors are at the bottom edge.

```
                    top: the ESP antenna, no copper under it
+----------------------------------------------------------------------+
| GND ring along the whole edge                                        |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (behind DevKit end) |    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  I2C rail      |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | logic 5V  |
|    [diode 1N5822] --> logic 5V --- bottom, under USB ----+           |
+----------------------------------------------------------------------+
```

Four zones:

| Zone | Where | What is there | Power |
|---|---|---|---|
| **A** Servos | the left edge, opposite J1-4…11 | 8 × XH-3: surfaces, ESC, AUX, receiver | servo 5V (dirty) |
| **B** Battery, telemetry | bottom left, opposite J1-12…16 | battery and current sensors, the radio modem | logic 5V |
| **C** 3V3 | top right, opposite J3-4…7 | OLED, sensors on the I2C rail, I2C connectors | 3V3 |
| **D** Logic 5V | bottom right, opposite J3-8…18 | GPS, buzzer, AUX3, lights | logic 5V |

The power section is on the left (servos, ESC, battery), the sensors and communications on the right. The servo current stays on the left and does not pass by the sensors.

---

## 2. The DevKit headers: which pin goes where

The pin order is that of the ESP32-S3-DevKitC-1 (2 × 22). Check it against the silkscreen of your board and measure the distance between the rows before you draw anything. The numbering runs from the antenna to the USB.

| J1 (left row) | GPIO | To | | J3 (right row) | GPIO | To |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 over the top → zone C | | 1 | GND | — |
| 2 | 3V3 | (same) | | 2 | 43 | — ("COM" console) |
| 3 | RST | — | | 3 | 44 | — ("COM" console) |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | I2C rail: SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | I2C rail: SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS: TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS: RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 buzzer (via Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | — PSRAM |
| 12 | 8 | B1 VBAT (ADC) | | 12 | 36 | — PSRAM |
| 13 | 3 | B1 CURR (ADC) | | 13 | 35 | — PSRAM |
| 14 | 46 | — strapping | | 14 | 0 | — BOOT button |
| 15 | 9 | B2 TELEM: TX | | 15 | 45 | — strapping |
| 16 | 10 | B2 TELEM: RX | | 16 | 48 | — RGB LED |
| 17 | 11 | free | | 17 | 47 | D3 AUX3 |
| 18 | 12 | free | | 18 | 21 | D4 LIGHT (via Q2) |
| 19 | 13 | free | | 19 | 20 | — USB |
| 20 | 14 | free | | 20 | 19 | — USB |
| 21 | 5V | logic 5V input (after the diode) | | 21 | GND | zone D ground |
| 22 | GND | zone B ground | | 22 | GND | zone D ground |

On the bench the firmware uses the free GPIO11–14 for SPI (ICM-42688); on this board SPI is not routed.

---

## 3. Rules for fitting into a single layer

1. **Through-hole parts on top, SMD on the bottom.** The DevKit sockets, XH connectors, electrolytics and the diode go on the component side. 0805, 0603 and SOT-23 parts are soldered right onto the copper. With toner transfer the copper pattern is printed mirrored.
2. **In every connector the signal is nearer the DevKit, power farther, GND at the edge.** That is why in all connectors **pin 1 is the one nearest the DevKit**. The signal traces then do not cross the power. The exception is the I2C rail (item 5).
3. **GND is a pour around the whole perimeter** (a ring). The outermost pins of the connectors go straight onto it.
4. **Only two lines pass from one side of the DevKit to the other.** 3V3 goes up from J1-1/2, past the top end of the DevKit, and to the right — beyond the edge of the DevKit's board, not under the antenna. Logic 5V goes from J1-21 down under the DevKit and along the bottom edge to the right, under the USB connectors (only traces are there, the plug hangs higher up).
5. **The I2C rail.** Four parallel traces at a 2.54 mm pitch, running outward from the DevKit: **3V3 · GND · SCL · SDA**. That is the pin order of the GY modules (VCC GND SCL SDA). The sockets and connectors sit across the rail like railway cars: each trace passes through its own pin. The rail starts at J3-6/7, dives under the OLED and runs up along the right row. If it does not fit in height, bend it left over the top end of the DevKit; the order of the lines is preserved through the turn.
6. **Do not run traces between the DevKit's pins**: the 2.54 pitch is too tight for toner transfer. Under the DevKit itself traces are fine — there are 11 mm to its board there.
7. **Through-hole parts are free jumpers.** A trace passes comfortably under the body of the diode (lead pitch 12.5–15 mm) and between the legs of an electrolytic (5 mm).
8. **0805 between the connector's pins.** The XH pitch of 2.5 mm — an 0805 capacitor is soldered right between the neighboring +5V and GND pins on the copper side.
9. **Trace widths:** servo 5V and servo GND — from 2 mm, logic 5V — from 1 mm, signals — 0.4–0.5 mm.
10. **Silkscreen labels:** the connector number (A1, B2…), the name, and an arrow at pin 1.

---

## 4. Connector blocks

### Block A — servos: 8 × XH-3, the left edge, in a column opposite J1-4…11

The pins in each connector:
**1 — signal** (nearer the DevKit) · **2 — servo +5V** · **3 — GND** (toward the edge).
This is the order of a servo's wire: orange, red, brown.

| Connector | Label | What to plug in | GPIO (pin) |
|---|---|---|---|
| A1 | AIL-L | the left aileron | 4 (J1-4) |
| A2 | AIL-R | the right aileron | 5 (J1-5) |
| A3 | ELE | the elevator | 6 (J1-6) |
| A4 | ESC | the controller: the throttle signal; **on the red wire — the BEC 5V input** | 7 (J1-7) |
| A5 | AUX1 | the payload-drop servo | 15 (J1-8) |
| A6 | AUX2 | flaps (two servos through a Y-cable) or any servo | 16 (J1-9) |
| A7 | RC | the FS-iA6B receiver, the iBUS SERVO port (the receiver is powered from here) | 17 (J1-10) |
| A8 | RUD | the rudder + the wheel | 18 (J1-11) |

What else is soldered in the block:

- **The servo +5V bus** — the middle column of pins, a trace from 2 mm. **GND** — the outer column, which is also part of the GND ring.
- **330 Ω (0603)** into a break of each signal line, at the connector. If 5 V ever reaches a signal pin (a bad servo, a poor crimp), the ESP pin will survive.
- **10 kΩ (0603)** from the A4 ESC signal to GND: while the ESP reboots, no junk gets through to the controller.
- **100 nF (0805)** between pins 2 and 3 at every connector.
- **A4 ESC** additionally has 10 µF + 100 pF (0805): the power input of the whole board — low-frequency, high-frequency and microwave filtering.
- **A7 RC** additionally has 10 µF (0805): the receiver is sensitive to sags.
- **2 × 470 µF 16 V** on the servo bus, one at each end of the column (above A1 and below A8): plus to the bus, minus to the ring. Inside the column the minus cannot reach the ring, and 3 cm of wide trace makes no difference to an electrolytic.

One ESC socket carries ~3 A. For 4–6 MG90S servos that is enough. If there will be more servos and they are more powerful, add a separate BEC power socket next to A4.

### Block B — battery and telemetry, bottom left under the servos

**B1 BAT — XH-5** (the only XH-5 on the board: a cable carrying 12–17 V will not fit into any other connector)

| Pin | What | Where on the board |
|---|---|---|
| 1 | **VBAT** — the battery positive by a thin wire (the outermost "+" of the balance lead, or from the battery connector, not through the ESC) | a 56 kΩ / 10 kΩ divider → GPIO8 (J1-12) |
| 2 | empty — a gap between the battery voltage and everything else | — |
| 3 | **CURR** — the current sensor output | a 10 kΩ / 15 kΩ divider → GPIO3 (J1-13) |
| 4 | logic +5V — power for the current sensor | the logic 5V bus |
| 5 | GND | the ring |

- **The VBAT divider:** 56 kΩ on top, 10 kΩ on the bottom, 100 nF in parallel with the bottom one. 3S (12.6 V) → 1.91 V, 4S (16.8 V) → 2.55 V — with margin up to the ADC limit (~3.1 V). A separate ground wire is not needed: the ground is common through the ESC, so pin 5 can be left uncrimped.
- **The CURR divider:** 10 kΩ on top, 15 kΩ on the bottom, 100 nF in parallel with the bottom one. A 5 V Hall sensor (ACS758 and similar) gives a maximum of 5 V → 3.0 V at the pin. If the sensor's output is 3.3 V, the top resistor is 0 Ω and the bottom one is not soldered.
- Put the dividers right at the connector and take the ground from its pin 5 — then only one trace goes to the ESP.
- No current sensor yet? Simply do not crimp pins 3–4.

**B2 TELEM — XH-4:** a telemetry radio modem or iBUS-SENS.

| Pin | What | GPIO (pin) |
|---|---|---|
| 1 | TX → to the modem's RX | 9 (J1-15) |
| 2 | RX ← from the modem's TX | 10 (J1-16) |
| 3 | logic +5V | — |
| 4 | GND | — |

- 10 µF + 100 nF between pins 3 and 4. For a 1 W modem add a 470 µF electrolytic.
- The firmware does not support TELEM yet: all three UARTs are taken (console, iBUS, GPS). To enable it, the console has to be moved to the built-in USB. That is a firmware change; the connector is already routed.

### Block C — 3V3: the display and sensors, top right

Everything in this zone sits on the **I2C rail** (section 3, item 5): four traces **3V3 · GND · SCL · SDA** running outward from the DevKit. SCL comes from GPIO42 (J3-6), SDA from GPIO41 (J3-7). 3V3 arrives over the top from J1-1/2. The OLED sits lowest on the rail, with C2–C5 above it in order.

**C1 OLED — XH-4.** It takes power from the rail and data from its own bus (GPIO1/2), which approach from the inner side.

| Pin | What | From |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | the rail |
| 4 | GND | the rail |

This is the pin order of the OLED module (GND VCC SCL SDA) back to front, so the cable goes without twists.

**C2 I2C-A and C3 I2C-B — XH-4**, identical:

| Pin | What |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** — the compass: a GY-273 on a mast (a straight cable, the order is the same as on the module) or the compass from a GPS module. For the GPS only GND, SCL and SDA are crimped: the compass gets power over the GPS cable.
- **C3** — the spare: an airspeed sensor (MS4525DO), a rangefinder, and so on.

**C4 BARO — a 1×4 socket for a BMP581 module:** 1 — VCC, 2 — GND, 3 — SCL, 4 — SDA.

- On the module itself, solder wire jumpers **CSB→VCC** and **SDO→GND** (address 0x46; 0x47 stays free for the pitot tube). Without CSB→VCC the chip goes into SPI mode; with SDO floating the address wanders. The module's other pins hang in the air.
- 3V3 from the rail only: many BMP581 modules have no regulator of their own.
- The pin order differs between modules — check yours. If it does not match, the module goes on a cable to C3 and the socket is not fitted.
- On top — a piece of open-cell foam (against airflow and light).

**C5 IMU — a 1×8 socket for a GY-521:**

| Pin | Module pin | To |
|---|---|---|
| 1 | VCC | the 3V3 rail |
| 2 | GND | the GND rail |
| 3 | SCL | the SCL rail |
| 4 | SDA | the SDA rail |
| 5, 6 | XDA, XCL | nothing |
| 7 | AD0 | to the GND pour outside the rail (address 0x68) |
| 8 | INT | nothing |

How the IMU is rotated on the board does not matter: the mounting is set by the `o` calibration (PILOT_GUIDE, "IMU mounting").

A standalone MPU-6500 module (10 pins: VCC GND SCL SDA EDA ECL AD0 INT NCS FSYNC) does not fit this socket — the pin order is different. It goes on a cable to C3 (VCC, GND, SCL, SDA), with jumpers on the module: **NCS→VCC** (otherwise the chip goes into SPI mode), **AD0→GND** (address 0x68) and **FSYNC→GND**.

The block's capacitors: **10 µF + 100 nF** on the rail at C1 (that is where 3V3 starts), **100 nF** between pins 1 and 2 at C2–C5. The I2C pull-ups are already on the modules; do not fit them on the board.

### Block D — logic 5V: GPS, buzzer, lights, bottom right

The logic 5V bus arrives from below (from under the DevKit along the bottom edge) and climbs the right edge through the "+5V" pins of all the connectors in the block.

**D1 GPS — XH-4.** It sits right under the rail, so that the GPS socket and the socket of its compass (C2) are close together.

| Pin | What | GPIO (pin) |
|---|---|---|
| 1 | TX → to the GPS RX | 40 (J3-8) |
| 2 | RX ← from the GPS TX | 39 (J3-9) |
| 3 | logic +5V | — |
| 4 | GND | — |

10 µF + 100 nF between pins 3 and 4.

**D2 BUZ — XH-2:** an active 5 V buzzer, to find the airplane in the grass and to warn about the battery and ARM.

| Pin | What |
|---|---|
| 1 | the buzzer's "−" → switch Q1 |
| 2 | logic +5V → the buzzer's "+" |

**D3 AUX3 — XH-3:** 1 — the GPIO47 signal (J3-17) through 330 Ω, 2 — logic +5V, 3 — GND, plus 100 nF between 2 and 3. Suitable for a button, the data of an LED strip, a camera trigger. **Do not hang a servo on it:** this is logic 5V, and its current would go through the diode and pump the ESP's power.

**D4 LIGHT — XH-2:** a switch for a load of up to ~0.5–1 A — navigation lights, a headlight, the payload-release electromagnet.

| Pin | What |
|---|---|
| 1 | the load's "−" → switch Q2 |
| 2 | logic +5V → the load's "+" |

**Switches Q1 and Q2** — the same SOT-23 footprint. On the BC817 and the Si2302 the pins match in role:

| SOT-23 pin | BC817 | Si2302 | To |
|---|---|---|---|
| 1 | base | gate | ← 1 kΩ ← GPIO (38 for Q1, 21 for Q2); 10 kΩ from pin 1 to GND |
| 2 | emitter | source | GND (the zone ground — J3-21/22) |
| 3 | collector | drain | pin 1 of the connector (D2 / D4) |

- **Q1 (buzzer):** either of the two will do.
- **Q2 (lights):** **Si2302** — the BC817 gets hot at hundreds of milliamps.
- The 10 kΩ holds the switch closed while the ESP boots: the buzzer does not scream, the lights do not blink.
- If the load is a coil (an electromagnet, a magnetic buzzer), fit an SS14 or 1N4148 diode in parallel with the connector, cathode to +5V. Leave room for it between pins 1 and 2.

---

## 5. Power and all the capacitors

```
 ESC (BEC 5V/5A) ──► A4 ──► servo 5V bus ──┬──► A1…A8 (servos, receiver)
                                           │    2×470 µF at the ends of the column
                                           │
                                           └──► diode 1N5822 ──► logic 5V ──┬──► J1-21 (5V DevKit)
                                                                            ├──► B1, B2 (current sensor, modem)
                                                                            └──► D1…D4 (GPS, buzzer, AUX3, lights)
 DevKit: its own 3V3 regulator ──► J1-1/2 ──► over the top ──► rail C (OLED, sensors, I2C connectors)
```

- **There is only one input — A4 ESC.** A separate power connector is not needed.
- **The 1N5822 Schottky diode** (3 A, through-hole; the SMD substitute is the SS34) — buy it. It does three things:
  - USB and the BEC do not fight: you can keep USB plugged in with the battery connected;
  - when the servos sag the bus, the logic electrolytic does not discharge back into the servos, and the ESP does not reboot;
  - the through-hole body works as a jumper over the GND trace in the corner by J1-22.

  The anode goes to the lower end of the servo bus, the cathode to J1-21. Do not take a 1N5819 (1 A): the ESP, the modem, the GPS and the lights all draw through the diode.
- The servos are not powered from USB alone; the diode cuts them off. That is by design. The GPS, the modem and the buzzer work from USB on the desk only if the DevKit's 5V pin supplies power from USB. Some clones have a diode of their own there, and then they do not work — that is normal.
- 3.3 V comes only from the DevKit's regulator and only for zone C.

**All the capacitors in one table** (ceramics — 0805, electrolytics — 16 V):

| Where | What | Why |
|---|---|---|
| Servo bus, above A1 and below A8 | 470 µF + 470 µF | sags when all the servos twitch at once |
| A4 ESC, between + and GND | 10 µF + 100 nF + 100 pF | the power input: low-, high- and ultra-high-frequency |
| A1–A3, A5–A8 | 100 nF at each | servo motor noise — at the source |
| A7 RC | + 10 µF | the receiver |
| Logic 5V, at J1-21 | 470 µF + 10 µF + 100 nF | holds up the ESP when the BEC sags |
| 3V3 rail, at C1 | 10 µF + 100 nF | power for the sensors |
| C2–C5 | 100 nF at each | |
| B1: the ADC input of VBAT and CURR | 100 nF on each, in parallel with the bottom resistor | the ADC filter |
| B1: the current sensor's +5V | 100 nF between pins 4 and 5 | |
| B2 TELEM | 10 µF + 100 nF (a 1 W modem — + 470 µF) | the transmitter's current spikes |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

A 10 µF 0805 ceramic loses up to half its capacitance at 5 V — that is accounted for, and there are electrolytics nearby anyway.

Test points: **5VS** (the servo bus), **5VL** (logic 5V), **3V3**, **GND**. They are convenient for measuring with a multimeter.

---

## 6. What plugs in where

| Device | Connector | Notes |
|---|---|---|
| The aileron, elevator and rudder servos | A1, A2, A3, A8 | the signal goes to pin 1 |
| ESC | A4 | the red wire is the BEC input |
| The FS-iA6B receiver | A7 | the iBUS SERVO port, an ordinary 3-wire cable |
| The payload-drop servo, flaps | A5, A6 | |
| GY-521 (MPU6500) | C5, socket | |
| BMP581 | C4, socket | jumpers on the module CSB→VCC, SDO→GND (0x46) |
| GY-273 (compass) or the GPS compass | C2 | away from the power wires, preferably on a mast |
| OLED 128×64 | C1 | |
| Airspeed sensor and the like | C3 | |
| u-blox M10 GPS | D1 + C2 | the 6 wires are split between two housings: D1 (power, UART) and C2 (compass) |
| Battery voltage, current sensor | B1 | |
| Radio modem / iBUS-SENS | B2 | |
| Buzzer | D2 | |
| Lights, headlight | D4 | |

Placement in the airplane:

- Put the board on a soft mount (foam, gel pads), nearer the center of gravity: motor vibration spoils the angles.
- Keep the power wires (battery → ESC → motor) away from zone C and from the compass.
- The barometer goes under foam, the IMU any way you like (the `o` calibration).

---

## 7. Parts list

| Part | Qty | Where |
|---|---|---|
| PBS 1×22 socket (for the DevKit) | 2 | |
| XH-3, right-angle or straight | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| PBS 1×8 and 1×4 sockets | 1 each | C5, C4 |
| Schottky diode 1N5822 (or SS34) | 1 | **buy it** |
| Electrolytic 470 µF 16 V | 3 (+1 for a powerful modem) | servo bus ×2, logic 5V |
| 0805 10 µF | 6 | A4, A7, logic 5V, 3V3, B2, D1 |
| 0805 100 nF | 20 | see the capacitor table |
| 0805 100 pF | 1 | A4 |
| 0603 330 Ω | 9 | signals A1–A8, D3 |
| 0603 10 kΩ | 5 | ESC to GND, bottom of VBAT, top of CURR, pin 1 at Q1 and Q2 |
| 0603 56 kΩ | 1 | top of VBAT |
| 0603 15 kΩ | 1 | bottom of CURR |
| 0603 1 kΩ | 2 | to pin 1 at Q1 and Q2 |
| BC817 or Si2302 | 1 | Q1 (buzzer) |
| Si2302 | 1 | Q2 (lights) |
| SS14 / 1N4148 | 0–2 | only for coils on D2 and D4 |

The board comes out at about 80×95 mm: the servo column and the sensor rail stick out above the top end of the DevKit. If it does not fit in the fuselage, the simplest way to shrink it is to move BARO and the IMU onto cables to C3 and shorten the rail.

---

## 8. Which GPIOs not to touch

0, 45, 46 — the boot mode depends on them; 19/20 — USB; 26–37 — the flash and PSRAM of the N16R8 module; 43/44 — the "COM" connector (console); 48 — the RGB LED. After this plan only GPIO11–14 remain free (on the bench — SPI).

---

## 9. Before the first power-up

1. Without the DevKit and without the battery, buzz it out: servo +5V ↔ GND, logic 5V ↔ GND, 3V3 ↔ GND — there must be a short nowhere.
2. Apply the BEC (through A4), with the DevKit not yet inserted. 5VS should read 5.0–5.2 V, and 5VL 0.3–0.5 V lower (the drop across the diode).
3. Insert the DevKit and connect only USB. 5VS reads 0 V: the diode does not let USB into the servos.
4. Everything together. The console's `s` will show whether the sensors respond and how many I2C errors there are.

---

## 10. What changed compared with the previous plan

- **The pins were reshuffled for a single layer** (already in `Config.h`):
  - sensor I2C 8/9 → **41/42**;
  - GPS 15/16 → **39/40**;
  - AUX1/AUX2 41/42 → **15/16**;
  - VBAT 3 → **8**, current sensor 10 → **3**;
  - telemetry 39/40 → **9/10**;
  - a new **LIGHT** output on GPIO21.

  On the bench, move two wires: SDA 8 → 41, SCL 9 → 42. The other pins are in reserve and not connected on the bench.
- **One BEC through the ESC**: the separate PWR connector is gone.
- **The sensors on the board are on I2C** (as on the bench). The SPI sockets are gone, GPIO11–14 are free. The ICM-42688 also speaks I2C, but the firmware will need an I2C variant of it in `SensorSelection.h`.
- **GPS-MAG is gone**: the GPS compass plugs into C2.
- **VBAT and the current sensor are combined** into a single XH-5 (B1).
