# Flugsteuerungsplatine: Steckverbinderblöcke

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../FC_BOARD.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Eine Trägerplatine für den ESP32-S3 DevKitC-1 (N16R8): Das DevKit wird in zwei Buchsenleisten gesteckt, und ringsherum sitzen Blöcke von JST-XH-Steckverbindern. Dieses Dokument beantwortet drei Fragen: welche Steckverbinder zu Blöcken zusammengefasst werden, wohin die Kondensatoren kommen und was wohin gesteckt wird. Die Pins stimmen mit `include/config/Config.h` überein (dem Block `BOARD_ESP32_S3`).

> Diese Platine ist für den ESP32-S3 entworfen, das frühere Hauptboard. Hauptboard ist jetzt der STM32H743 (DevEBox H743): Eine Trägerplatine dafür ist noch nicht gezeichnet, die Sensoren werden vorerst an die Stiftleisten des DevEBox angeschlossen – die Pinbelegung steht in [PILOT_GUIDE](PILOT_GUIDE.md).

Die Platine ist als **einseitige** Platine gedacht: Die GPIOs sind so gewählt, dass die Pins jedes Blocks entlang der Leiste des DevKit aufeinanderfolgen und die Signalbahnen sich fächerförmig ohne Kreuzungen ausbreiten. Das Layout habe ich in keinem CAD-Programm geprüft. Geht es irgendwo nicht auf, setzen Sie eine Drahtbrücke auf der Bestückungsseite: Eine bis drei davon sind auf einer solchen Platine normal.

**Alle Steckverbinder sind JST-XH mit 1–5 Kontakten.** Die Leitungen werden nicht an die Platine gelötet: An die Leitungen von Servos, ESC, Empfänger und Modulen werden passende Steckergehäuse gecrimpt. Der XH ist kodiert, deshalb lässt er sich nicht verkehrt herum stecken. Ein Kontakt hält ~3 A aus.

---

## 1. Anordnungsplan

Ansicht von oben, von der Bestückungsseite. Die USB-Buchsen des DevKit liegen am unteren Rand.

```
                    oben: ESP-Antenne, darunter kein Kupfer
+----------------------------------------------------------------------+
| GND-Ring am gesamten Rand                                            |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (hinter dem DevKit) |    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  I2C-Schiene   |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | 5V Logik  |
|    [Diode 1N5822] --> 5V Logik --- unten, unter USB -----+           |
+----------------------------------------------------------------------+
```

Vier Zonen:

| Zone | Wo | Was dort sitzt | Versorgung |
|---|---|---|---|
| **A** Servos | linker Rand, gegenüber J1-4…11 | 8 × XH-3: Ruder, ESC, AUX, Empfänger | Servo-5V (verrauscht) |
| **B** Akku, Telemetrie | links unten, gegenüber J1-12…16 | Akku- und Stromsensoren, das Funkmodem | Logik-5V |
| **C** 3V3 | rechts oben, gegenüber J3-4…7 | OLED, Sensoren auf der I2C-Schiene, I2C-Steckverbinder | 3V3 |
| **D** Logik-5V | rechts unten, gegenüber J3-8…18 | GPS, Summer, AUX3, Lichter | Logik-5V |

Links liegt der Leistungsteil (Servos, ESC, Akku), rechts liegen Sensoren und Kommunikation. Der Servostrom bleibt links und fließt nicht an den Sensoren vorbei.

---

## 2. Die Leisten des DevKit: welcher Pin wohin

Die Pin-Reihenfolge ist die des ESP32-S3-DevKitC-1 (2 × 22). Gleichen Sie sie mit dem Bestückungsdruck Ihres Boards ab und messen Sie den Abstand zwischen den Reihen, bevor Sie zeichnen. Die Nummerierung läuft von der Antenne zum USB.

| J1 (linke Reihe) | GPIO | Wohin | | J3 (rechte Reihe) | GPIO | Wohin |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 über oben → Zone C | | 1 | GND | — |
| 2 | 3V3 | (dasselbe) | | 2 | 43 | — („COM“-Konsole) |
| 3 | RST | — | | 3 | 44 | — („COM“-Konsole) |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | I2C-Schiene: SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | I2C-Schiene: SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS: TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS: RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 Summer (über Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | — PSRAM |
| 12 | 8 | B1 VBAT (ADC) | | 12 | 36 | — PSRAM |
| 13 | 3 | B1 CURR (ADC) | | 13 | 35 | — PSRAM |
| 14 | 46 | — Strapping | | 14 | 0 | — BOOT-Taster |
| 15 | 9 | B2 TELEM: TX | | 15 | 45 | — Strapping |
| 16 | 10 | B2 TELEM: RX | | 16 | 48 | — RGB-LED |
| 17 | 11 | frei | | 17 | 47 | D3 AUX3 |
| 18 | 12 | frei | | 18 | 21 | D4 LIGHT (über Q2) |
| 19 | 13 | frei | | 19 | 20 | — USB |
| 20 | 14 | frei | | 20 | 19 | — USB |
| 21 | 5V | Eingang der Logik-5V (nach der Diode) | | 21 | GND | Masse der Zone D |
| 22 | GND | Masse der Zone B | | 22 | GND | Masse der Zone D |

Auf dem Prüfstand verwendet die Firmware die freien GPIO11–14 für SPI (ICM-42688); auf dieser Platine ist SPI nicht herausgeführt.

---

## 3. Regeln, um mit einer Lage auszukommen

1. **Bedrahtete Bauteile oben, SMD unten.** Die Buchsen des DevKit, die XH-Steckverbinder, die Elkos und die Diode sitzen auf der Bestückungsseite. 0805, 0603 und SOT-23 werden direkt auf das Kupfer gelötet. Beim Tonertransfer (Bügelmethode) wird das Kupfermuster spiegelverkehrt gedruckt.
2. **In jedem Steckverbinder liegt das Signal näher am DevKit, die Versorgung weiter weg und GND zum Rand hin.** Deshalb ist bei allen Steckverbindern **Kontakt 1 der dem DevKit nächste**. Die Signalbahnen kreuzen dann die Versorgung nicht. Die Ausnahme ist die I2C-Schiene (Punkt 5).
3. **GND ist eine Kupferfläche rund um den gesamten Rand** (ein Ring). Die äußersten Kontakte der Steckverbinder führen direkt darauf.
4. **Von einer Seite des DevKit zur anderen laufen nur zwei Leitungen.** 3V3 geht von J1-1/2 nach oben, über das obere Ende des DevKit hinweg und nach rechts – außerhalb des Randes der DevKit-Platine, nicht unter der Antenne. Logik-5V geht von J1-21 unter dem DevKit nach unten und am unteren Rand nach rechts, unter den USB-Buchsen (dort liegen nur Bahnen, der Stecker hängt weiter oben).
5. **Die I2C-Schiene.** Vier parallele Bahnen im Raster 2,54 mm, die vom DevKit nach außen laufen: **3V3 · GND · SCL · SDA**. Das ist die Pin-Reihenfolge der GY-Module (VCC GND SCL SDA). Die Buchsen und Steckverbinder stehen quer zur Schiene wie Eisenbahnwagen: Jede Bahn läuft durch ihren eigenen Kontakt. Die Schiene beginnt bei J3-6/7, taucht unter dem OLED ab und verläuft nach oben entlang der rechten Reihe. Passt sie in der Höhe nicht, knicken Sie sie über dem oberen Ende des DevKit nach links ab; die Reihenfolge der Leitungen bleibt in der Kurve erhalten.
6. **Führen Sie keine Bahnen zwischen den Pins des DevKit hindurch**: Das Raster 2,54 ist für den Tonertransfer zu eng. Unter dem DevKit selbst geht es, dort sind es 11 mm bis zu seiner Platine.
7. **Bedrahtete Bauteile sind kostenlose Brücken.** Unter dem Gehäuse der Diode (Anschlussabstand 12,5–15 mm) und zwischen den Beinen eines Elkos (5 mm) läuft problemlos eine Bahn hindurch.
8. **0805 zwischen den Kontakten des Steckverbinders.** Beim XH-Raster von 2,5 mm wird ein 0805-Kondensator direkt zwischen die benachbarten Kontakte +5V und GND gelötet, auf der Kupferseite.
9. **Bahnbreiten:** Servo-5V und Servo-GND – ab 2 mm, Logik-5V – ab 1 mm, Signale – 0,4–0,5 mm.
10. **Beschriftung im Bestückungsdruck:** die Nummer des Steckverbinders (A1, B2 …), die Aufschrift und ein Pfeil an Kontakt 1.

---

## 4. Steckverbinderblöcke

### Block A – Servos: 8 × XH-3, linker Rand, in einer Spalte gegenüber J1-4…11

Die Kontakte in jedem Steckverbinder:
**1 – Signal** (näher am DevKit) · **2 – Servo-+5V** · **3 – GND** (zum Rand).
Das ist die Reihenfolge der Servoleitung: orange, rot, braun.

| Steckverbinder | Aufschrift | Was angeschlossen wird | GPIO (Pin) |
|---|---|---|---|
| A1 | AIL-L | das linke Querruder | 4 (J1-4) |
| A2 | AIL-R | das rechte Querruder | 5 (J1-5) |
| A3 | ELE | das Höhenruder | 6 (J1-6) |
| A4 | ESC | der Fahrtregler: das Gassignal; **auf der roten Leitung – der BEC-5V-Eingang** | 7 (J1-7) |
| A5 | AUX1 | das Servo für den Lastabwurf | 15 (J1-8) |
| A6 | AUX2 | Klappen (zwei Servos über ein Y-Kabel) oder irgendein Servo | 16 (J1-9) |
| A7 | RC | der Empfänger FS-iA6B, Port iBUS SERVO (der Empfänger wird von hier versorgt) | 17 (J1-10) |
| A8 | RUD | das Seitenruder + das Rad | 18 (J1-11) |

Was in dem Block sonst noch gelötet wird:

- **Der Servo-+5V-Bus** – die mittlere Kontaktspalte, eine Bahn ab 2 mm. **GND** – die äußere Spalte, die zugleich Teil des GND-Rings ist.
- **330 Ω (0603)** in die Unterbrechung jeder Signalleitung, am Steckverbinder. Gelangen einmal 5 V an einen Signalkontakt (defektes Servo, schiefes Crimpen), übersteht der ESP-Pin das.
- **10 kΩ (0603)** vom Signal des A4 ESC nach GND: Solange der ESP neu startet, gelangt kein Müll an den Fahrtregler.
- **100 nF (0805)** zwischen den Kontakten 2 und 3 an jedem Steckverbinder.
- Am **A4 ESC** zusätzlich 10 µF + 100 pF (0805): der Spannungseingang der ganzen Platine – Filterung für niedrige, hohe und sehr hohe Frequenzen.
- Am **A7 RC** zusätzlich 10 µF (0805): Der Empfänger reagiert empfindlich auf Spannungseinbrüche.
- **2 × 470 µF 16 V** am Servobus, je eines an jedem Ende der Spalte (über A1 und unter A8): Plus an den Bus, Minus an den Ring. Innerhalb der Spalte kommt das Minus nicht an den Ring heran, und 3 cm breite Bahn spielen für einen Elko keine Rolle.

Eine ESC-Buchse hält ~3 A aus. Für 4–6 Servos MG90S reicht das. Kommen mehr und stärkere Servos hinzu, setzen Sie neben A4 eine eigene Versorgungsbuchse vom BEC.

### Block B – Akku und Telemetrie, links unten unter den Servos

**B1 BAT – XH-5** (der einzige XH-5 auf der Platine: Ein Kabel mit 12–17 V passt in keinen anderen Steckverbinder)

| Kontakt | Was | Wohin auf der Platine |
|---|---|---|
| 1 | **VBAT** – Akku-Plus über eine dünne Leitung (das äußerste „+“ des Balancersteckers oder vom Akkustecker, nicht über den ESC) | ein Teiler 56 kΩ / 10 kΩ → GPIO8 (J1-12) |
| 2 | leer – ein Abstand zwischen der Akkuspannung und allem anderen | — |
| 3 | **CURR** – Ausgang des Stromsensors | ein Teiler 10 kΩ / 15 kΩ → GPIO3 (J1-13) |
| 4 | Logik-+5V – Versorgung des Stromsensors | der Logik-5V-Bus |
| 5 | GND | der Ring |

- **Der VBAT-Teiler:** 56 kΩ oben, 10 kΩ unten, 100 nF parallel zum unteren. 3S (12,6 V) → 1,91 V, 4S (16,8 V) → 2,55 V – mit Reserve bis zur Grenze des ADC (~3,1 V). Eine eigene Masseleitung ist nicht nötig: Die Masse ist über den ESC gemeinsam, Kontakt 5 kann ungecrimpt bleiben.
- **Der CURR-Teiler:** 10 kΩ oben, 15 kΩ unten, 100 nF parallel zum unteren. Ein 5-V-Hall-Sensor (ACS758 und ähnliche) liefert maximal 5 V → 3,0 V am Pin. Hat der Sensor einen 3,3-V-Ausgang, ist der obere Widerstand 0 Ω und der untere wird nicht gelötet.
- Setzen Sie die Teiler direkt an den Steckverbinder und nehmen Sie die Masse von dessen Kontakt 5 – dann geht nur eine Bahn zum ESP.
- Noch kein Stromsensor? Die Kontakte 3–4 einfach nicht crimpen.

**B2 TELEM – XH-4:** ein Telemetrie-Funkmodem oder iBUS-SENS.

| Kontakt | Was | GPIO (Pin) |
|---|---|---|
| 1 | TX → zum RX des Modems | 9 (J1-15) |
| 2 | RX ← vom TX des Modems | 10 (J1-16) |
| 3 | Logik-+5V | — |
| 4 | GND | — |

- 10 µF + 100 nF zwischen den Kontakten 3 und 4. Für ein 1-W-Modem kommt ein Elko mit 470 µF hinzu.
- Die Firmware unterstützt TELEM noch nicht: Alle drei UARTs sind belegt (Konsole, iBUS, GPS). Um es zu aktivieren, muss die Konsole auf das eingebaute USB verlegt werden. Das ist eine Änderung an der Firmware; der Steckverbinder wird schon jetzt verdrahtet.

### Block C – 3V3: Display und Sensoren, rechts oben

Alles in dieser Zone sitzt auf der **I2C-Schiene** (Abschnitt 3, Punkt 5): vier Bahnen **3V3 · GND · SCL · SDA** vom DevKit nach außen. SCL kommt von GPIO42 (J3-6), SDA von GPIO41 (J3-7). 3V3 kommt über oben von J1-1/2. Das OLED sitzt als unterstes auf der Schiene, darüber der Reihe nach C2–C5.

**C1 OLED – XH-4.** Es bezieht die Versorgung von der Schiene und die Daten von seinem eigenen Bus (GPIO1/2), die von der Innenseite herankommen.

| Kontakt | Was | Von wo |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | die Schiene |
| 4 | GND | die Schiene |

Das ist die Pin-Reihenfolge des OLED-Moduls (GND VCC SCL SDA) von hinten nach vorn, deshalb läuft das Kabel ohne Verdrehungen.

**C2 I2C-A und C3 I2C-B – XH-4**, identisch:

| Kontakt | Was |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** – der Kompass: ein GY-273 auf einem Mast (ein gerades Kabel, die Reihenfolge wie am Modul) oder der Kompass eines GPS-Moduls. Beim GPS werden nur GND, SCL und SDA gecrimpt: Der Kompass erhält seine Versorgung über das GPS-Kabel.
- **C3** – der Reserveanschluss: ein Fluggeschwindigkeitssensor (MS4525DO), ein Entfernungsmesser und Ähnliches.

**C4 BARO – eine 1×4-Buchse für ein BMP581-Modul:** 1 – VCC, 2 – GND, 3 – SCL, 4 – SDA.

- Löten Sie auf dem Modul selbst Drahtbrücken **CSB→VCC** und **SDO→GND** (Adresse 0x46; 0x47 bleibt für das Pitotrohr frei). Ohne CSB→VCC geht der Chip in den SPI-Modus, bei offenem SDO wandert die Adresse. Die übrigen Pins des Moduls hängen in der Luft.
- Nur 3V3 von der Schiene: Viele BMP581-Module haben keinen eigenen Spannungsregler.
- Die Pin-Reihenfolge ist bei den Modulen verschieden – prüfen Sie Ihres. Passt sie nicht, kommt das Modul an einem Kabel an C3, und die Buchse wird nicht bestückt.
- Obenauf ein Stück offenporiger Schaumstoff (gegen Luftzug und Licht).

**C5 IMU – eine 1×8-Buchse für ein GY-521:**

| Kontakt | Pin des Moduls | Wohin |
|---|---|---|
| 1 | VCC | Schiene 3V3 |
| 2 | GND | Schiene GND |
| 3 | SCL | Schiene SCL |
| 4 | SDA | Schiene SDA |
| 5, 6 | XDA, XCL | nirgendwohin |
| 7 | AD0 | auf die GND-Fläche außerhalb der Schiene (Adresse 0x68) |
| 8 | INT | nirgendwohin |

Wie die IMU auf der Platine gedreht ist, spielt keine Rolle: Den Einbau bestimmt die Kalibrierung `o` (PILOT_GUIDE, „IMU-Einbau“).

Ein separates MPU-6500-Modul (10 Pins: VCC GND SCL SDA EDA ECL AD0 INT NCS FSYNC) passt nicht in diese Buchse – die Pin-Reihenfolge ist anders. Es kommt an einem Kabel an C3 (VCC, GND, SCL, SDA), auf dem Modul mit den Brücken **NCS→VCC** (sonst geht der Chip in den SPI-Modus), **AD0→GND** (Adresse 0x68) und **FSYNC→GND**.

Die Kondensatoren des Blocks: **10 µF + 100 nF** auf der Schiene bei C1 (dort beginnt 3V3), **100 nF** zwischen den Kontakten 1 und 2 bei C2–C5. Die I2C-Pull-ups sitzen schon auf den Modulen; setzen Sie sie nicht auf die Platine.

### Block D – Logik-5V: GPS, Summer, Lichter, rechts unten

Der Logik-5V-Bus kommt von unten (unter dem DevKit entlang des unteren Rands) und steigt am rechten Rand durch die „+5V“-Kontakte aller Steckverbinder des Blocks auf.

**D1 GPS – XH-4.** Er sitzt direkt unter der Schiene, damit die Buchse des GPS und die seines Kompasses (C2) nahe beieinander liegen.

| Kontakt | Was | GPIO (Pin) |
|---|---|---|
| 1 | TX → zum RX des GPS | 40 (J3-8) |
| 2 | RX ← vom TX des GPS | 39 (J3-9) |
| 3 | Logik-+5V | — |
| 4 | GND | — |

10 µF + 100 nF zwischen den Kontakten 3 und 4.

**D2 BUZ – XH-2:** ein aktiver 5-V-Summer, um das Flugzeug im Gras zu finden und vor dem Akkustand und dem ARM zu warnen.

| Kontakt | Was |
|---|---|
| 1 | „−“ des Summers → Schalter Q1 |
| 2 | Logik-+5V → „+“ des Summers |

**D3 AUX3 – XH-3:** 1 – das Signal von GPIO47 (J3-17) über 330 Ω, 2 – Logik-+5V, 3 – GND, dazu 100 nF zwischen 2 und 3. Geeignet für einen Taster, die Daten eines LED-Streifens, den Auslöser einer Kamera. **Hängen Sie hier kein Servo an:** Das sind Logik-5V, sein Strom würde über die Diode fließen und die Versorgung des ESP aufschaukeln.

**D4 LIGHT – XH-2:** ein Schalter für eine Last von bis zu ~0,5–1 A – Positionslichter, ein Scheinwerfer, der Abwurf-Elektromagnet.

| Kontakt | Was |
|---|---|
| 1 | „−“ der Last → Schalter Q2 |
| 2 | Logik-+5V → „+“ der Last |

**Die Schalter Q1 und Q2** – dasselbe SOT-23-Footprint. Beim BC817 und beim Si2302 entsprechen sich die Beine in ihrer Rolle:

| SOT-23-Bein | BC817 | Si2302 | Wohin |
|---|---|---|---|
| 1 | Basis | Gate | ← 1 kΩ ← GPIO (38 für Q1, 21 für Q2); 10 kΩ von Bein 1 nach GND |
| 2 | Emitter | Source | GND (Masse der Zone – J3-21/22) |
| 3 | Kollektor | Drain | Kontakt 1 des Steckverbinders (D2 / D4) |

- **Q1 (Summer):** Beide passen.
- **Q2 (Lichter):** **Si2302** – der BC817 wird bei einigen Hundert Milliampere heiß.
- Der 10-kΩ-Widerstand hält den Schalter gesperrt, solange der ESP bootet: Der Summer schreit nicht, die Lichter blinken nicht.
- Ist die Last eine Spule (ein Elektromagnet, ein magnetischer Summer), setzen Sie eine Diode SS14 oder 1N4148 parallel zum Steckverbinder, Kathode an +5V. Sehen Sie dafür Platz zwischen den Kontakten 1 und 2 vor.

---

## 5. Stromversorgung und alle Kondensatoren

```
 ESC (BEC 5V/5A) ──► A4 ──► Servo-5V-Bus ──┬──► A1…A8 (Servos, Empfänger)
                                           │    2×470 µF an den Enden der Spalte
                                           │
                                           └──► Diode 1N5822 ──► Logik-5V ──┬──► J1-21 (5V DevKit)
                                                                            ├──► B1, B2 (Stromsensor, Modem)
                                                                            └──► D1…D4 (GPS, Summer, AUX3, Lichter)
 DevKit: eigener 3V3-Regler ──► J1-1/2 ──► über oben ──► Schiene C (OLED, Sensoren, I2C-Steckverbinder)
```

- **Es gibt nur einen Eingang – A4 ESC.** Ein eigener Versorgungsanschluss ist nicht nötig.
- **Die Schottky-Diode 1N5822** (3 A, bedrahtet; der SMD-Ersatz ist die SS34) – muss gekauft werden. Sie erfüllt drei Aufgaben:
  - USB und BEC kommen sich nicht in die Quere: Man kann USB bei angeschlossenem Akku stecken lassen;
  - wenn die Servos den Bus einbrechen lassen, entlädt sich der Logik-Elko nicht zurück in die Servos, und der ESP startet nicht neu;
  - das bedrahtete Gehäuse dient als Brücke über der GND-Bahn in der Ecke bei J1-22.

  Die Anode kommt an das untere Ende des Servobusses, die Kathode an J1-21. Nehmen Sie keine 1N5819 (1 A): Durch die Diode fließen der ESP, das Modem, das GPS und die Lichter.
- Allein über USB werden die Servos nicht versorgt, die Diode trennt sie ab. Das ist so gewollt. GPS, Modem und Summer arbeiten auf dem Tisch über USB nur, wenn der 5V-Pin des DevKit Spannung vom USB liefert. Manche Klone haben dort eine eigene Diode, dann funktionieren sie nicht – das ist normal.
- Die 3,3 V kommen nur vom Regler des DevKit und nur für die Zone C.

**Alle Kondensatoren in einer Tabelle** (Keramik – 0805, Elkos – 16 V):

| Wo | Was | Wozu |
|---|---|---|
| Servobus, über A1 und unter A8 | 470 µF + 470 µF | Einbrüche, wenn alle Servos auf einmal zucken |
| A4 ESC, zwischen + und GND | 10 µF + 100 nF + 100 pF | der Spannungseingang: niedrige, hohe und sehr hohe Frequenzen |
| A1–A3, A5–A8 | 100 nF an jedem | Störungen der Servomotoren – an der Quelle |
| A7 RC | + 10 µF | der Empfänger |
| Logik-5V, bei J1-21 | 470 µF + 10 µF + 100 nF | stützt den ESP bei einem Einbruch des BEC |
| 3V3-Schiene, bei C1 | 10 µF + 100 nF | Versorgung der Sensoren |
| C2–C5 | 100 nF an jedem | |
| B1: der ADC-Eingang von VBAT und CURR | 100 nF an jedem, parallel zum unteren Widerstand | der ADC-Filter |
| B1: das +5V des Stromsensors | 100 nF zwischen den Kontakten 4 und 5 | |
| B2 TELEM | 10 µF + 100 nF (ein 1-W-Modem – + 470 µF) | Stromspitzen des Senders |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

Ein 10-µF-Keramikkondensator in 0805 verliert bei 5 V bis zur Hälfte seiner Kapazität – das ist berücksichtigt, und Elkos sind ohnehin in der Nähe.

Testpunkte: **5VS** (der Servobus), **5VL** (Logik-5V), **3V3**, **GND**. An ihnen lässt sich bequem mit dem Multimeter messen.

---

## 6. Was wohin gesteckt wird

| Gerät | Steckverbinder | Hinweise |
|---|---|---|
| Die Servos von Querrudern, Höhenruder und Seitenruder | A1, A2, A3, A8 | das Signal kommt an Kontakt 1 |
| ESC | A4 | die rote Leitung ist der BEC-Eingang |
| Der Empfänger FS-iA6B | A7 | der Port iBUS SERVO, ein gewöhnliches 3-adriges Kabel |
| Das Servo für den Lastabwurf, Klappen | A5, A6 | |
| GY-521 (MPU6500) | C5, Buchse | |
| BMP581 | C4, Buchse | Brücken auf dem Modul CSB→VCC, SDO→GND (0x46) |
| GY-273 (Kompass) oder der Kompass des GPS | C2 | weg von den Leistungsleitungen, am besten auf einem Mast |
| OLED 128×64 | C1 | |
| Fluggeschwindigkeitssensor und Ähnliches | C3 | |
| GPS u-blox M10 | D1 + C2 | die 6 Leitungen verteilen sich auf zwei Gehäuse: D1 (Versorgung, UART) und C2 (Kompass) |
| Akkuspannung, Stromsensor | B1 | |
| Funkmodem / iBUS-SENS | B2 | |
| Summer | D2 | |
| Lichter, Scheinwerfer | D4 | |

Platzierung im Flugzeug:

- Setzen Sie die Platine auf eine weiche Halterung (Schaumstoff, Gelkissen), näher am Schwerpunkt: Motorvibrationen verfälschen die Winkel.
- Halten Sie die Leistungsleitungen (Akku → ESC → Motor) von Zone C und vom Kompass fern.
- Das Barometer kommt unter Schaumstoff, die IMU beliebig (Kalibrierung `o`).

---

## 7. Stückliste

| Bauteil | Anz. | Wo |
|---|---|---|
| Buchsenleiste PBS 1×22 (für das DevKit) | 2 | |
| XH-3, gewinkelt oder gerade | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| Buchsenleisten PBS 1×8 und 1×4 | je 1 | C5, C4 |
| Schottky-Diode 1N5822 (oder SS34) | 1 | **kaufen** |
| Elko 470 µF 16 V | 3 (+1 für ein leistungsstarkes Modem) | Servobus ×2, Logik-5V |
| 0805 10 µF | 6 | A4, A7, Logik-5V, 3V3, B2, D1 |
| 0805 100 nF | 20 | siehe Kondensatortabelle |
| 0805 100 pF | 1 | A4 |
| 0603 330 Ω | 9 | Signale A1–A8, D3 |
| 0603 10 kΩ | 5 | ESC nach GND, unten bei VBAT, oben bei CURR, Bein 1 bei Q1 und Q2 |
| 0603 56 kΩ | 1 | oben bei VBAT |
| 0603 15 kΩ | 1 | unten bei CURR |
| 0603 1 kΩ | 2 | zu Bein 1 bei Q1 und Q2 |
| BC817 oder Si2302 | 1 | Q1 (Summer) |
| Si2302 | 1 | Q2 (Lichter) |
| SS14 / 1N4148 | 0–2 | nur für Spulen an D2 und D4 |

Die Platine wird etwa 80×95 mm groß: Die Servospalte und die Sensorschiene ragen über das obere Ende des DevKit hinaus. Passt sie nicht in den Rumpf, ist der einfachste Weg, sie zu verkleinern, BARO und IMU auf Kabeln nach C3 auszulagern und die Schiene zu kürzen.

---

## 8. Welche GPIOs nicht angefasst werden

0, 45, 46 – von ihnen hängt der Boot-Modus ab; 19/20 – USB; 26–37 – Flash und PSRAM des Moduls N16R8; 43/44 – der „COM“-Anschluss (Konsole); 48 – die RGB-LED. Nach diesem Plan bleiben nur GPIO11–14 frei (auf dem Prüfstand – SPI).

---

## 9. Vor dem ersten Einschalten

1. Ohne DevKit und ohne Akku durchklingeln: Servo-+5V ↔ GND, Logik-5V ↔ GND, 3V3 ↔ GND – nirgends darf ein Kurzschluss sein.
2. Legen Sie den BEC an (über A4), das DevKit ist noch nicht gesteckt. An 5VS müssen 5,0–5,2 V anliegen, an 5VL 0,3–0,5 V weniger (der Abfall an der Diode).
3. Stecken Sie das DevKit und schließen Sie nur USB an. An 5VS liegen 0 V an: Die Diode lässt USB nicht in die Servos.
4. Alles zusammen. Das `s` der Konsole zeigt, ob die Sensoren antworten und wie viele Fehler es auf dem I2C gibt.

---

## 10. Was sich gegenüber dem früheren Plan geändert hat

- **Die Pins wurden für eine einzige Lage neu verteilt** (schon in `Config.h`):
  - I2C der Sensoren 8/9 → **41/42**;
  - GPS 15/16 → **39/40**;
  - AUX1/AUX2 41/42 → **15/16**;
  - VBAT 3 → **8**, Stromsensor 10 → **3**;
  - Telemetrie 39/40 → **9/10**;
  - ein neuer Ausgang **LIGHT** an GPIO21.

  Stecken Sie auf dem Prüfstand zwei Leitungen um: SDA 8 → 41, SCL 9 → 42. Die übrigen Pins sind Reserve und auf dem Prüfstand nicht angeschlossen.
- **Ein einziger BEC über den ESC**: Der separate PWR-Anschluss ist entfallen.
- **Die Sensoren auf der Platine hängen am I2C** (wie auf dem Prüfstand). Die SPI-Buchsen sind entfallen, GPIO11–14 sind frei. Der ICM-42688 kann auch I2C, aber die Firmware braucht dafür in `SensorSelection.h` eine I2C-Variante.
- **GPS-MAG ist entfallen**: Der Kompass des GPS wird in C2 gesteckt.
- **VBAT und der Stromsensor sind zusammengelegt** zu einem einzigen XH-5 (B1).
