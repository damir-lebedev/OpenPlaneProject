# SENSORS — Sensoren

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/sensors.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

[← Referenz](README.md)

Die Sensoren sind in drei Ebenen aufgebaut:

1. **Kategorie-Schnittstellen** (`SensorInterface.h`) — was `Autopilot`,
   `ArmingManager` und die Telemetrie sehen.
2. **Basisklassen der Kategorien** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — alles Gemeinsame: Kalibrierungen, Filter, Achsendrehung, Vorzeichen,
   Fehlerzählung, Speicherung im NVS. Das Muster Template Method.
3. **Chip-Treiber** — nur die Register und Formeln aus dem Datenblatt. Sie erhalten
   `IRegisterDevice&` (sie unterscheiden den Bus nicht) oder `IUartPort&`.

Welcher Chip einkompiliert wird, entscheidet `SensorSelection.h`.

---

## Schnittstellen und Datenstrukturen

**Datei:** `sensors/SensorInterface.h`

### Strukturen

| Struktur | Felder |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (Rollen + rechter Flügel nach unten, Nicken + Nase nach oben, Gieren + Nase nach rechts); `accelX/Y/Z` g (Achsen des Flugzeugs: X zur Nase, Y nach links, Z nach oben); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, das Integral des Gyroskops) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m **relativ zum Kalibrierpunkt**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT nach der Hard-Iron-Kalibrierung, in den Achsen des Flugzeugs; `headingDegrees` 0..360 (ohne Neigungskompensation); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 keiner, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (Schnittstelle)

| Methode | Beschreibung |
|---|---|
| `bool begin()` | Den Chip erkennen und konfigurieren; `true` — der Sensor arbeitet |
| `bool isAvailable() const` | Angeschlossen und antwortet **gerade jetzt** |
| `void update()` | In jedem Takt aufrufen; er entscheidet selbst, ob es Zeit zum Lesen ist |
| `const char* getSensorType() const` | Ein Name für das Protokoll |
| `void printStatus() const` | Eine Diagnosezeile (Konsolenbefehl `s`) |

### `ImuSensor : Sensor`

| Methode | Beschreibung |
|---|---|
| `const ImuData& getImuData() const` | Die letzten Daten |
| `void calibrate()` | Kalibrierung des Gyroskops (in Ruhe) + die Prüfung vor dem Flug |
| `void setYaw(float)` | Den Kurs setzen (zum Beispiel beim Start nach dem Kompass) |
| `virtual void calibrateOrientation()` | Kalibrierung des Einbaus der Platine (standardmäßig nicht unterstützt) |
| `virtual const char* getPreflightProblem() const` | Das Problem der Prüfung vor dem Flug oder `nullptr` (standardmäßig `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (die aktuelle Höhe = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (Hard-Iron: 15 s lang drehen).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (es gibt mindestens einen 2D-Fix).

---

## `AirspeedSensor`

**Datei:** `sensors/airspeed/AirspeedSensor.h` · **Art:** Schnittstelle · **Implementierung:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
der Differenzdruck nach Nullung und Filter, die angezeigte Geschwindigkeit (ρ0 = 1,225), die wahre
Geschwindigkeit (ρ aus dem statischen Druck und der Temperatur), die Dichte. Methoden: `getAirspeedData()`,
`calibrateZero()` (die Nullung neu beginnen), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Datei:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Erbt von:** `AirspeedSensor` · **Status:** in einer Closed-Loop-Simulation mit Rauschen geprüft, nicht beflogen

Ein selbstgebautes Pitotrohr mit **zwei Absolutbarometern**: `total` — das
BMP581 im Rohr (Gesamtdruck), `stat` — das Hauptbarometer des Rumpfes
(statischer Druck). Die Bauanleitung steht in [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#ein-pitotrohr-zum-selberbauen).

| Methode | Beschreibung |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` des Rohrbarometers (das statische läuft bereits), Nullung starten |
| `update()` | eine neue Probe des Rohrs → Differenz − Null, Tiefpass `PITOT_FILTER_TAU_S`; die ersten `PITOT_ZERO_SAMPLES` Proben mitteln die Null; die Geschwindigkeit ist `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | beide Barometer leben, die Null ist gesammelt, es gibt keinen Fehler, die Probe ist frischer als `PITOT_STALE_US` |
| `hasFault()` | die Differenz liegt länger als `PITOT_NEGATIVE_FAULT_MS` unter −`PITOT_NEGATIVE_FAULT_PA` (Schläuche, Wasser) |
| `getZeroOffset()`, `printStatus()` | Diagnose |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | Formeln |

---

## namespace `SensorMounting`

**Datei:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
dreht die Chipachsen um die Vertikale (Chip nach oben) in die Achsen des Flugzeugs (X zur Nase,
Y nach links). `rotationCwDeg` ist die Richtung, in die die X-Achse des Chips zeigt, im Uhrzeigersinn von oben gesehen:

| Wert | bodyX | bodyY |
|---|---|---|
| 0 (und jeder unbekannte Wert) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Wird vom Kompass (`MAG_ROTATION_CW_DEG`) und von einer IMU ohne Einbaukalibrierung verwendet.

---

## `SensorSelection.h`

**Datei:** `sensors/SensorSelection.h` · **Art:** Präprozessor-Konfiguration

Die einzige Stelle, an der ein physischer Sensor gewechselt wird: ein fertiges Set in einer Zeile
(`SENSOR_KIT`) oder jeder Sensor einzeln. Jede Wahl lässt sich
mit einem Build-Flag überschreiben (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Set `SENSOR_KIT` | IMU | Barometer | Kompass | Fluggeschwindigkeit | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, Standard) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (Rumpf) | QMC6309 | BMP581 im Rohr | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (Rumpf) | QMC6309 | BMP581 im Rohr | M10 |
| `SENSOR_KIT_CUSTOM` (0) | alle fünf folgenden Makros setzen | | | | |

| Auswahlmakro | Varianten |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — ein BMP581 I2C 0x47 im Rohr |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Alle Kombinationen lassen sich auf allen Boards bauen — `tools/build_matrix.sh`.

Das Ergebnis sind Typ-Aliase und Gerätefabriken:

| Name | MPU6050 / ICM42688 usw. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (für NONE nicht definiert) |
| die neuen IMUs | `LSM6DSV_Sensor` + I2C 0x6A (Reserve 0x6B) oder SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) oder SPI |
| die neuen Barometer | `SPL06_Sensor` + I2C 0x76 (0x77) oder SPI; `BMP581_Sensor` + I2C 0x46 (0x47) oder SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (für NONE nicht definiert) |
| `SelectedGps` | `UbloxM10_Gps` (für NONE nicht definiert) |

`main.cpp` umschließt das Anlegen von Kompass, GPS und Rohr mit `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Datei:** `sensors/imu/ImuOrientation.h` · **Abhängig von:** `SensorMounting`, `Preferences` (NVS)

Die Drehmatrix `R` von den Chipachsen in die Achsen des Flugzeugs: `body = R · chip`; die Zeilen von
`R` sind die Achsen des Flugzeugs in den Chipachsen.

| Methode | Beschreibung |
|---|---|
| `ImuOrientation()` | Die Einheitsmatrix (entspricht `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | Eine Drehung um die Vertikale in Schritten von 90°, die Platine mit dem Chip nach oben |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Kalibrierung aus drei Lagen (der Messwert „oben“ des Beschleunigungssensors in den Chipachsen). `nullptr` — Erfolg, sonst der Grund der Ablehnung |
| `void apply(const float chip[3], float body[3]) const` | Die Drehung anwenden |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | Der Winkel zwischen dem gemessenen „oben“ und der Z-Achse des Flugzeugs (180°, wenn der Vektor null ist) |
| `bool load(const char* ns)` | Aus dem NVS laden; weist ein nicht orthonormales oder linkshändiges Tripel zurück |
| `void save(const char* ns) const` | Im NVS speichern |
| `void describe(Print&) const` | „Nase = +Y des Chips, oben = +Z des Chips“ (mit einem Winkel, wenn eine Achse nicht auf ±14° mit einer Chipachse zusammenfällt) |

Der Algorithmus von `fromPoses`: Z = norm(level); X₁ = der Anteil von noseUp ⟂ Z; Y = der Anteil von
rightWingDown ⟂ Z, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X. Die Ablehnungen:

| Bedingung | Meldung |
|---|---|
| Nullvektor | „keine Messwerte des Beschleunigungssensors“ |
| die Neigung in Schritt 2 oder 3 liegt außerhalb von 20..80° | „in Schritt N ist eine Neigung von 30-60° nötig“ |
| cos(X₁, X₂) < −0,5 | „die Schritte 2 und 3 widersprechen einander…“ (die Nase wurde gesenkt oder es wurde der falsche Flügel genommen) |
| cos(X₁, X₂) < 0,9 (≈25°) | „…es wurden die falschen Achsen geneigt…“ |

---

## `AttitudeEstimator`

**Datei:** `sensors/imu/AttitudeEstimator.h`

Ein Komplementärfilter für Rollen/Nicken und ein Integral für das Gieren, unabhängig vom Chip.

| Methode | Beschreibung |
|---|---|
| `void reset()` | Das nächste `update()` beginnt sofort mit dem Winkel des Beschleunigungssensors |
| `void setYaw(float deg)` | Den Kurs setzen (auf ±180 reduziert) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | Die Beschleunigung in g in den Achsen des Flugzeugs, die Raten in °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0,1 s bei 2 ms).
Der erste Aufruf nach `reset()` liefert sofort die Winkel des Beschleunigungssensors; bei `dt ≤ 0` oder über 0,1 s
wird der Schritt übersprungen (eine Pause, ein hängender Bus).

---

## `ImuSensorBase`

**Datei:** `sensors/imu/ImuSensorBase.h` · **Erbt von:** `ImuSensor` · **Art:** abstrakt

`RawImuSample` — eine Probe in ADC-Einheiten in den Chipachsen:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

Die Pipeline `update()` → `process()`:

```
Rohproben − Offsets (ADC) → Skalierung (g, °/s) → ImuOrientation (Achsen des Flugzeugs)
→ Luftfahrt-Vorzeichen (gyroY, gyroZ mit geändertem Vorzeichen) → AttitudeEstimator
```

| Methode | Beschreibung |
|---|---|
| `bool isAvailable() const` | `begin()` war erfolgreich und es gibt weniger als 50 Lesefehler hintereinander |
| `void update()` | Ein Lesevorgang; bei einem Fehler bleiben die Daten unverändert und die Zähler wachsen |
| `void calibrate()` | 200 Proben × 10 ms: der Offset des Gyroskops, das Rauschen, „oben“; ohne Einbaukalibrierung — der Horizont = die aktuelle Lage. Danach die Prüfung vor dem Flug |
| `void calibrateOrientation()` | Drei Lagen (`capturePose`: ~1 s ruhig, die Lage weicht von den vorigen um ≥ 20° ab, Timeout von 30 s), `ImuOrientation::fromPoses`, Speichern im NVS. Beseitigt die Einbauprobleme der Prüfung vor dem Flug |
| `const char* getPreflightProblem() const` | Der Text des Problems oder `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

Die geschützte API für Treiber:

| Methode | Beschreibung |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Jeder Treiber hat seinen eigenen NVS-Namensraum |
| `virtual bool readSample(RawImuSample&) = 0` | Eine Probe; `false` — der Chip hat nicht geantwortet |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | Die Skalen |
| `virtual float temperatureC(int16_t raw) const = 0` | Die Temperaturformel |
| `void setAvailable(bool)` | Das Ergebnis von `begin()`; bei `true` lädt es den Einbau aus dem NVS oder aus `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Den Namen nach der Erkennung präzisieren |

Die Prüfung vor dem Flug (`runPreflightCheck`), der Reihe nach:

| Problem | Bedingung |
|---|---|
| `NotResponding` | weniger als die Hälfte der Kalibrierproben wurde gelesen |
| `Moved` | das Rauschen des Gyroskops > 0,5 °/s |
| `NotOneG` | \|a\| weicht um mehr als 0,2g von 1g ab |
| `MountingMismatch` | (der Einbau ist kalibriert) „oben“ liegt mehr als 45° vom gespeicherten entfernt |
| `NotChipUp` | (nicht kalibriert) die Platine liegt nicht mit dem Chip nach oben (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Datei:** `sensors/imu/MPU6050_Sensor.h` · **Erbt von:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Status:** auf dem Prüfstand (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 und Klone (GY-521-Platinen), I2C 0x68/0x69 oder
SPI. Der Chip wird über `WHO_AM_I` erkannt (0x68 — MPU6050, sonst die 6500-Familie).

- `begin()`: WHO_AM_I (keine Antwort → nicht verfügbar), der Name nach der ID, Reset, Verlassen
  des Ruhezustands (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; der 6500 hat einen separaten
  Tiefpass für den Beschleunigungssensor, `ACCEL_CONFIG2`.
- `readSample()`: 14 Bytes ab `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Skalen: 2048 LSB/g, 16,4 LSB/(°/s).
- Temperatur: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Datei:** `sensors/imu/ICM42688_Sensor.h` · **Erbt von:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Status:** nicht an der Hardware geprüft

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, ohne
  Dummy-Byte.
- `begin()`: Bank 0, Software-Reset, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, UI-Filter 50 Hz.
- `readSample()`: 14 Bytes ab `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Temperatur: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Datei:** `sensors/imu/LSM6DSV_Sensor.h` · **Erbt von:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Status:** nicht an der Hardware geprüft

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Die Register wurden mit dem `lsm6dsv-pid` von ST und mit ArduPilot abgeglichen.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (Bit 0 von CTRL3) und ein Warten;
  der 32X wird am Variantenbit in CTRL8 erkannt (er hat einen eigenen Code für ±16 g); BDU + Auto-Inkrement,
  ±2000 °/s mit LPF1, ±16 g mit LPF2, 960 Hz High-Performance.
- `readSample()`: 14 Bytes ab 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Skalen: 1000/0,488 LSB/g, 1000/70 LSB/(°/s); Temperatur `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI-Modus 0, ohne Dummy-Byte.

## `ICM45686_Sensor`

**Datei:** `sensors/imu/ICM45686_Sensor.h` · **Erbt von:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Status:** nicht an der Hardware geprüft

ICM-45686 (TDK). Die Register wurden mit dem Treiber von TDK, mit Zephyr und mit ArduPilot abgeglichen.

- `begin()`: Reset über `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s und ±16 g
  bei 1,6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); der Tiefpass
  ODR/32 — Lesen-Ändern-Schreiben der **indirekten** Register IPREG
  (0xA4AC, 0xA583) über das Fenster 0x7C..0x7E; 45 ms für den Start des Gyroskops.
- `readSample()`: 14 Bytes ab 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Skalen: 2048 LSB/g, 16,4 LSB/(°/s); Temperatur `raw/132.48 + 25`.

---

## `BarometerBase`

**Datei:** `sensors/baro/BarometerBase.h` · **Erbt von:** `BarometerSensor` · **Art:** abstrakt

| Methode | Beschreibung |
|---|---|
| `bool isAvailable() const` | `begin()` war erfolgreich und es gibt weniger als 100 Fehler hintereinander |
| `void update()` | Nicht öfter als `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → Höhe und Vertikalgeschwindigkeit |
| `void calibrateAltitude()` | 20 Proben × 50 ms: die mittlere absolute Höhe = die Basis; setzt Höhe und Geschwindigkeit zurück |
| `void setSeaLevelPressure(Pa)` | P₀ (standardmäßig 101325) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

Die geschützte API: der Konstruktor `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Formeln: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; die Vertikalgeschwindigkeit ist die
Ableitung der Höhe über die **tatsächlich neuen** Proben durch einen Tiefpass mit τ = 0,5 s
(liegt `dt` außerhalb von `(0, 0.5 s)`, wird die Geschwindigkeit nicht aktualisiert). Das Lesen nur neuer Proben
beseitigt das „treppenförmige“ Rauschen (0 m/s im Wechsel mit Sprüngen von Δh/2 ms).

---

## `BMP388_Sensor`

**Datei:** `sensors/baro/BMP388_Sensor.h` · **Erbt von:** `BarometerBase` · **Status:** auf dem Prüfstand (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **1 Dummy-Byte** (Datenblatt §5.3.2).
- `begin()`: Chip-ID `0x50`, Software-Reset, 21 Bytes NVM-Koeffizienten ab
  `0x31` (Skalen §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, Normalmodus.
- `isNewSampleReady()`: das Flag `drdy_press` (Bit 0x20) des Registers STATUS; abgefragt
  alle 5 ms.
- `readSample()`: 6 Bytes ab `0x04`; Kompensation von Bosch §9.3 (double):
  zuerst die Temperatur (`tLin`), dann der Druck.

---

## `BME280_Sensor`

**Datei:** `sensors/baro/BME280_Sensor.h` · **Erbt von:** `BarometerBase` · **Status:** nicht an der Hardware geprüft

BME280 (ID 0x60) und BMP280 (ID 0x58), I2C 0x76/0x77 oder SPI ohne Dummy-
Byte. Die Feuchte wird nicht gelesen.

- `begin()`: Chip-ID, Reset, 24 Bytes Kalibrierdaten ab `0x88`, `CTRL_HUM` (nur
  BME280, wird **vor** `CTRL_MEAS` geschrieben), `CONFIG` = IIR 4 + 0,5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- Ein Bereitschafts-Flag gibt es nicht — `ready = true`, Abfrage alle 25 ms.
- Kompensation — die Formeln von Bosch §8.1 (double); Schutz vor der Division durch null.

---

## `SPL06_Sensor`

**Datei:** `sensors/baro/SPL06_Sensor.h` · **Erbt von:** `BarometerBase` · **Status:** nicht an der Hardware geprüft

SPL06-001 (Goertek). Die Formeln stammen aus dem Datenblatt §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 ist der SPA06 mit einem anderen Koeffizientensatz —
  er wird abgelehnt); Reset, Warten auf `COEF_RDY | SENSOR_RDY`; 18 Bytes Koeffizienten
  (vorzeichenbehaftete Felder zu 12/20/16 Bit); die Temperaturquelle — nach `COEF_SRCE`;
  Druck 16× (32 Hz), Temperatur 1×, kontinuierlicher Modus.
- `isNewSampleReady()` — das Bit `PRS_RDY`; `readSample()` — 24-Bit-Proben
  big-endian, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Datei:** `sensors/baro/BMP581_Sensor.h` · **Erbt von:** `BarometerBase` · **Status:** nicht an der Hardware geprüft

BMP581 (Bosch). Die Abfolge folgt der offiziellen BMP5_SensorAPI.

- `begin()`: ein Dummy-Lesen (für SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  Software-Reset, `INT_STATUS` POR und `STATUS` mit bereitem NVM ohne Fehler;
  standby → OSR (Druck 16×, Temperatur 2×), IIR, DRDY; eine Prüfung, ob
  der ODR machbar ist (`OSR_EFF`); kontinuierlicher Modus.
- Eine Probe — bei DRDY oder alle 40 ms, wenn das Flag verloren geht; Temperatur
  `int24/65536`, Druck `uint24/64`.
- Zwei Instanzen im Build mit dem Rohr: das Hauptbarometer und `PITOT-BMP581`.

---

## `MagnetometerBase`

**Datei:** `sensors/mag/MagnetometerBase.h` · **Erbt von:** `MagnetometerSensor` · **Art:** abstrakt

| Methode | Beschreibung |
|---|---|
| `bool isAvailable() const` | `begin()` war erfolgreich und es gibt weniger als 25 Fehler hintereinander |
| `void update()` | 50 Hz: `readRaw()` → Offsets abziehen → Skalierung → Drehung um `MAG_ROTATION_CW_DEG` → Kurs `atan2(Y, X)` in 0..360 |
| `void calibrate()` | 15 s Drehen: Offset = (min + max)/2 je Achse (Hard-Iron), Speichern im NVS. Gab es nicht ein einziges erfolgreiches Lesen, wird die Kalibrierung abgelehnt und die alte im NVS bleibt unberührt |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

Die geschützte API: der Konstruktor `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (bei `true` lädt es die Kalibrierung aus dem NVS).

Der Kurs ohne Neigungskompensation: richtig, solange das Flugzeug fast waagerecht liegt. Nase nach
Norden → das Feld entlang +X → 0°; Nase nach Osten → 90°.

---

## `QMC5883P_Sensor`

**Datei:** `sensors/mag/QMC5883P_Sensor.h` · **Erbt von:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Status:** auf dem Prüfstand

`DEFAULT_ADDRESS = 0x2C`. `begin()`: Chip-ID `0x80` (Register 0x00), Software-
Reset, Achsenvorzeichen `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). Die Daten — 6 Bytes ab `0x01`,
little-endian. 37,5 LSB/µT.

---

## `QMC5883L_Sensor`

**Datei:** `sensors/mag/QMC5883L_Sensor.h` · **Erbt von:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Status:** nicht an der Hardware geprüft

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (der Chip hat keine verlässliche ID),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
Die Daten — 6 Bytes ab `0x00`, little-endian. 30 LSB/µT. Die Register sind **nicht kompatibel**
mit dem QMC5883P.

---

## `QMC6309_Sensor`

**Datei:** `sensors/mag/QMC6309_Sensor.h` · **Erbt von:** `MagnetometerBase` · **NVS:** `qmc6309` · **Status:** nicht an der Hardware geprüft

QMC6309 (QST), I2C 0x7C — eine Adresse außerhalb des üblichen Bereichs 0x08..0x77
(der Bus-Scan des Konsolenbefehls `b` geht bis 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; Reset (CTRL2 0x80 → 0x00), Warten auf
  `NVM_RDY | NVM_LOAD_DONE`; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian ab 0x01; 40,96 LSB/µT.

---

## `UbloxM10_Gps`

**Datei:** `sensors/gps/UbloxM10_Gps.h` · **Erbt von:** `GpsSensor` · **Abhängig von:** `IUartPort`, `Config` · **Status:** auf dem Prüfstand nicht angeschlossen

Ein u-blox M10 über UART, das UBX-Protokoll. Nur **NAV-PVT** wird ausgewertet (Klasse 0x01,
ID 0x07, 92 Bytes).

| Methode | Beschreibung |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 Baud → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 Baud → CFG-VALSET: 10 Hz, NAV-PVT auf UART1, UBX an, NMEA aus. Ohne TX-Pin (`PIN_GPS_TX < 0`) hört es nur mit 9600 zu. Immer `true` (es gibt kein ACK) |
| `bool isAvailable() const` | Es gab ein gültiges NAV-PVT und das letzte ist nicht älter als `GPS_TIMEOUT_US` |
| `void update()` | Dem Parser alles aus dem UART zuführen |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = verfügbar und `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` zeigt `available` nach `isAvailable()` (unter Berücksichtigung des Timeouts) |

Der Parser ist ein Byte-für-Byte-Automat `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, die Fletcher-8-Prüfsumme über class+id+len+payload.
Eine Länge > 512 ist ein Synchronisationsverlust, und die Suche beginnt von vorn. Die Felder von NAV-PVT werden so ausgewertet: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, auf 0..360 reduziert).

Der verschachtelte `ValsetBuilder` ist die Nutzlast von UBX-CFG-VALSET (version 0, layer RAM, Paare
aus einem Schlüssel U4 LE — einem Wert von 1/2/4 Bytes LE, ein Puffer von 64 Bytes).
