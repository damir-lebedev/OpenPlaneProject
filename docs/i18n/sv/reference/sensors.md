# SENSORER – sensorer

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/sensors.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

Sensorerna är uppbyggda i tre nivåer:

1. **Kategorigränssnitt** (`SensorInterface.h`) – det som `Autopilot`,
   `ArmingManager` och telemetrin ser.
2. **Kategoriernas basklasser** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) – allt gemensamt: kalibreringar, filter, axelrotation, tecken,
   felräkning, lagring i NVS. Mönstret Template Method.
3. **Kretsdrivrutiner** – bara registren och formlerna från databladet. De får
   `IRegisterDevice&` (de skiljer inte på bussarna) eller `IUartPort&`.

Vilken krets som kompileras in avgörs av `SensorSelection.h`.

---

## Gränssnitt och datastrukturer

**Fil:** `sensors/SensorInterface.h`

### Strukturer

| Struktur | Fält |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (roll + höger vinge ned, tippning + nosen upp, gir + nosen åt höger); `accelX/Y/Z` g (flygplanets axlar: X mot nosen, Y åt vänster, Z uppåt); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, gyrointegralen) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m **relativt kalibreringspunkten**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT efter hard-iron-kalibreringen, i flygplanets axlar; `headingDegrees` 0..360 (utan lutningskompensation); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 ingen, 2 – 2D, 3 – 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (gränssnitt)

| Metod | Beskrivning |
|---|---|
| `bool begin()` | Identifiera och konfigurera kretsen; `true` – sensorn fungerar |
| `bool isAvailable() const` | Ansluten och svarar **just nu** |
| `void update()` | Anropa varje cykel; den avgör själv om det är dags att läsa |
| `const char* getSensorType() const` | Ett namn för loggen |
| `void printStatus() const` | En diagnostikrad (konsol `s`) |

### `ImuSensor : Sensor`

| Metod | Beskrivning |
|---|---|
| `const ImuData& getImuData() const` | De senaste data |
| `void calibrate()` | Gyroskopkalibrering (håll stilla) + kontrollen före flygning |
| `void setYaw(float)` | Sätt kursen (till exempel från kompassen vid start) |
| `virtual void calibrateOrientation()` | Kalibrering av kortets montering (stöds inte som standard) |
| `virtual const char* getPreflightProblem() const` | Problemet i kontrollen före flygning eller `nullptr` (`nullptr` som standard) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (den aktuella höjden = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: rotera i 15 s).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (det finns minst en 2D-fix).

---

## `AirspeedSensor`

**Fil:** `sensors/airspeed/AirspeedSensor.h` · **Typ:** gränssnitt · **Implementering:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
differenstrycket efter nollställning och filtrering, den indikerade lufthastigheten (ρ0 = 1.225), den sanna
lufthastigheten (ρ från statiskt tryck och temperatur), densiteten. Metoder: `getAirspeedData()`,
`calibrateZero()` (starta om nollställningen), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Fil:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Ärver:** `AirspeedSensor` · **Status:** verifierad i en simulering i sluten slinga med brus, inte provad under flygning

Ett hemmabyggt pitotrör med **två absoluta barometrar**: `total` –
BMP581 inne i röret (totaltryck), `stat` – flygkroppens huvudbarometer
(statiskt tryck). Bygginstruktionen finns i [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#ett-hemmabyggt-pitotrör).

| Metod | Beskrivning |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` för rörets barometer (den statiska körs redan), starta nollställningen |
| `update()` | en ny mätning från röret → differens − noll, lågpassfilter `PITOT_FILTER_TAU_S`; de första `PITOT_ZERO_SAMPLES` mätningarna medelvärdesbildar nollpunkten; farten är `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | båda barometrarna lever, nollpunkten är insamlad, det finns inget fel, mätningen är färskare än `PITOT_STALE_US` |
| `hasFault()` | differensen ligger under −`PITOT_NEGATIVE_FAULT_PA` i mer än `PITOT_NEGATIVE_FAULT_MS` (slangar, vatten) |
| `getZeroOffset()`, `printStatus()` | diagnostik |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | formler |

---

## namnrymd `SensorMounting`

**Fil:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` –
roterar kretsens axlar kring vertikalen (kretsen uppåt) till flygplanets axlar (X mot nosen,
Y åt vänster). `rotationCwDeg` är vart kretsens X-axel pekar, medurs sett ovanifrån:

| Värde | bodyX | bodyY |
|---|---|---|
| 0 (och alla okända värden) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Används av kompassen (`MAG_ROTATION_CW_DEG`) och av en IMU utan monteringskalibrering.

---

## `SensorSelection.h`

**Fil:** `sensors/SensorSelection.h` · **Typ:** preprocessorkonfiguration

Det enda stället för att byta en fysisk sensor: en färdig sats på en rad
(`SENSOR_KIT`) eller varje sensor för sig. Alla val kan åsidosättas
med en byggflagga (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Sats `SENSOR_KIT` | IMU | Barometer | Kompass | Lufthastighet | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, standard) | MPU6500 | BMP581 I2C (0x46/0x47) | QMC5883P | – | – |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (flygkropp) | QMC6309 | BMP581 i röret | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (flygkropp) | QMC6309 | BMP581 i röret | M10 |
| `SENSOR_KIT_CUSTOM` (0) | sätt alla fem makrona nedan | | | | |

| Valmakro | Alternativ |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) – en BMP581 I2C 0x47 i röret |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Alla kombinationer byggs på alla kort – `tools/build_matrix.sh`.

Resultatet är typalias och enhetsfabriker:

| Namn | MPU6050 / ICM42688 osv. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (inte definierat för NONE) |
| de nya IMU:erna | `LSM6DSV_Sensor` + I2C 0x6A (reserv 0x6B) eller SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) eller SPI |
| de nya barometrarna | `SPL06_Sensor` + I2C 0x76 (0x77) eller SPI; `BMP581_Sensor` + I2C 0x46 (0x47) eller SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (inte definierat för NONE) |
| `SelectedGps` | `UbloxM10_Gps` (inte definierat för NONE) |

`main.cpp` omsluter skapandet av kompassen, GPS:en och röret med `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Fil:** `sensors/imu/ImuOrientation.h` · **Beror på:** `SensorMounting`, `Preferences` (NVS)

Rotationsmatrisen `R` från kretsens axlar till flygplanets axlar: `body = R · chip`; raderna i
`R` är flygplanets axlar i kretsens axlar.

| Metod | Beskrivning |
|---|---|
| `ImuOrientation()` | Identiteten (motsvarar `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | En rotation kring vertikalen i steg om 90°, kortet med kretsen uppåt |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Kalibrering från tre positioner (accelerometerns avläsning av ”upp” i kretsens axlar). `nullptr` – lyckat, annars orsaken till vägran |
| `void apply(const float chip[3], float body[3]) const` | Tillämpa rotationen |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | Vinkeln mellan det uppmätta ”upp” och flygplanets Z-axel (180° om vektorn är noll) |
| `bool load(const char* ns)` | Ladda från NVS; avvisar en icke-ortonormal eller vänsterhänt trippel |
| `void save(const char* ns) const` | Spara i NVS |
| `void describe(Print&) const` | ”nos = kretsens +Y, upp = kretsens +Z” (med en vinkel om en axel inte sammanfaller med en kretsaxel inom ±14°) |

Algoritmen i `fromPoses`: Z = norm(level); X₁ = den del av noseUp som är ⟂ Z; Y = den del av
rightWingDown som är ⟂ Z, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X. Vägranden:

| Villkor | Meddelande |
|---|---|
| nollvektor | ”inga accelerometeravläsningar” |
| lutningen i steg 2 eller 3 ligger utanför 20..80° | ”steg N behöver en lutning på 30-60°” |
| cos(X₁, X₂) < −0,5 | ”steg 2 och 3 motsäger varandra…” (nosen sänktes eller fel vinge användes) |
| cos(X₁, X₂) < 0,9 (≈25°) | ”…fel axlar lutades…” |

---

## `AttitudeEstimator`

**Fil:** `sensors/imu/AttitudeEstimator.h`

Ett komplementärfilter för roll/tippning och en girintegral, oberoende av kretsen.

| Metod | Beskrivning |
|---|---|
| `void reset()` | Nästa `update()` startar direkt från den vinkel som accelerometern ger |
| `void setYaw(float deg)` | Sätt kursen (omslagen till ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | Accelerationen i g i flygplanets axlar, hastigheterna i °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0,1 s vid 2 ms).
Det första anropet efter `reset()` ger accelerometerns vinklar direkt; `dt ≤ 0` eller > 0,1 s –
steget hoppas över (en paus, en buss som hängt sig).

---

## `ImuSensorBase`

**Fil:** `sensors/imu/ImuSensorBase.h` · **Ärver:** `ImuSensor` · **Typ:** abstrakt

`RawImuSample` – en mätning i ADC-enheter i kretsens axlar:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

Kedjan `update()` → `process()`:

```
råa mätningar − förskjutningar (ADC) → skala (g, °/s) → ImuOrientation (flygplanets axlar)
→ flygtekniska tecken (gyroY, gyroZ med bytt tecken) → AttitudeEstimator
```

| Metod | Beskrivning |
|---|---|
| `bool isAvailable() const` | `begin()` lyckades och färre än 50 läsfel i följd |
| `void update()` | En avläsning; vid fel ändras inte data och räknarna ökar |
| `void calibrate()` | 200 mätningar × 10 ms: gyroskopets förskjutning, bruset, ”upp”; utan monteringskalibrering – horisonten = det aktuella läget. Därefter kontrollen före flygning |
| `void calibrateOrientation()` | Tre positioner (`capturePose`: stilla i ~1 s, positionen skiljer sig från de föregående med ≥ 20°, en tidsgräns på 30 s), `ImuOrientation::fromPoses`, sparande i NVS. Rensar kontrollens problem med monteringen |
| `const char* getPreflightProblem() const` | Problemets text eller `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

Det skyddade API:t för drivrutiner:

| Metod | Beskrivning |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Varje drivrutin har sin egen NVS-namnrymd |
| `virtual bool readSample(RawImuSample&) = 0` | En mätning; `false` – kretsen svarade inte |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | Skalorna |
| `virtual float temperatureC(int16_t raw) const = 0` | Temperaturformeln |
| `void setAvailable(bool)` | Resultatet av `begin()`; vid `true` laddar den monteringen från NVS eller `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Förfina namnet efter identifieringen |

Kontrollen före flygning (`runPreflightCheck`), i ordning:

| Problem | Villkor |
|---|---|
| `NotResponding` | färre än hälften av kalibreringsmätningarna lästes |
| `Moved` | gyroskopets brus > 0,5 °/s |
| `NotOneG` | \|a\| skiljer sig från 1g med mer än 0,2g |
| `MountingMismatch` | (monteringen är kalibrerad) ”upp” ligger mer än 45° från det sparade |
| `NotChipUp` | (inte kalibrerad) kortet ligger inte med kretsen uppåt (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Fil:** `sensors/imu/MPU6050_Sensor.h` · **Ärver:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Status:** på bänken (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 och kloner (GY-521-kort), I2C 0x68/0x69 eller
SPI. Kretsen identifieras med `WHO_AM_I` (0x68 – MPU6050, annars 6500-familjen).

- `begin()`: WHO_AM_I (inget svar → otillgänglig), namnet efter ID, återställning, lämna
  viloläge (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; 6500 har ett separat
  lågpassfilter för accelerometern `ACCEL_CONFIG2`.
- `readSample()`: 14 byte från `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Skalor: 2048 LSB/g, 16,4 LSB/(°/s).
- Temperatur: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Fil:** `sensors/imu/ICM42688_Sensor.h` · **Ärver:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Status:** inte verifierad på hårdvara

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` – SPI 8 MHz, utan
  dummybyte.
- `begin()`: bank 0, mjukvaruåterställning, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, UI-filter 50 Hz.
- `readSample()`: 14 byte från `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Temperatur: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Fil:** `sensors/imu/LSM6DSV_Sensor.h` · **Ärver:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Status:** inte verifierad på hårdvara

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Registren kontrollerades mot ST:s `lsm6dsv-pid` och ArduPilot.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (CTRL3 bit 0) och en väntan;
  32X känns igen på variantbiten i CTRL8 (den har en egen kod för ±16 g); BDU + autoinkrement,
  ±2000 °/s med LPF1, ±16 g med LPF2, 960 Hz high-performance.
- `readSample()`: 14 byte från 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Skalor: 1000/0,488 LSB/g, 1000/70 LSB/(°/s); temperatur `raw/256 + 25`.
- `static spiDevice(bus, cs)` – SPI-läge 0, utan dummybyte.

## `ICM45686_Sensor`

**Fil:** `sensors/imu/ICM45686_Sensor.h` · **Ärver:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Status:** inte verifierad på hårdvara

ICM-45686 (TDK). Registren kontrollerades mot TDK:s drivrutin, Zephyr och ArduPilot.

- `begin()`: återställning via `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s och ±16 g
  vid 1,6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); lågpassfiltret
  ODR/32 – en läs-modifiera-skriv av de **indirekta** IPREG-registren
  (0xA4AC, 0xA583) genom fönstret 0x7C..0x7E; 45 ms för att gyroskopet ska starta.
- `readSample()`: 14 byte från 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Skalor: 2048 LSB/g, 16,4 LSB/(°/s); temperatur `raw/132.48 + 25`.

---

## `BarometerBase`

**Fil:** `sensors/baro/BarometerBase.h` · **Ärver:** `BarometerSensor` · **Typ:** abstrakt

| Metod | Beskrivning |
|---|---|
| `bool isAvailable() const` | `begin()` lyckades och färre än 100 fel i följd |
| `void update()` | Inte oftare än `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → höjd och vertikal hastighet |
| `void calibrateAltitude()` | 20 mätningar × 50 ms: den genomsnittliga absoluta höjden = basen; nollställer höjden och hastigheten |
| `void setSeaLevelPressure(Pa)` | P₀ (101325 som standard) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

Det skyddade API:t: konstruktorn `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Formler: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; den vertikala hastigheten är
höjdens derivata över de **verkligt nya** mätningarna via ett lågpassfilter med τ = 0,5 s
(`dt` utanför `(0, 0.5 s)` – hastigheten uppdateras inte). Att bara läsa nya mätningar
tar bort ”trappstegs”-bruset (0 m/s varvat med hopp på Δh/2 ms).

---

## `BMP388_Sensor`

**Fil:** `sensors/baro/BMP388_Sensor.h` · **Ärver:** `BarometerBase` · **Status:** på bänken (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` – SPI 8 MHz,
  **1 dummybyte** (datablad §5.3.2).
- `begin()`: krets-ID `0x50`, mjukvaruåterställning, 21 byte NVM-koefficienter från
  `0x31` (skalor §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, normalläge.
- `isNewSampleReady()`: flaggan `drdy_press` (bit 0x20) i registret STATUS; avläses
  var 5:e ms.
- `readSample()`: 6 byte från `0x04`; Bosch-kompensering §9.3 (double):
  temperaturen först (`tLin`), sedan trycket.

---

## `BME280_Sensor`

**Fil:** `sensors/baro/BME280_Sensor.h` · **Ärver:** `BarometerBase` · **Status:** inte verifierad på hårdvara

BME280 (ID 0x60) och BMP280 (ID 0x58), I2C 0x76/0x77 eller SPI utan dummy-
byte. Luftfuktigheten läses inte.

- `begin()`: krets-ID, återställning, 24 byte kalibrering från `0x88`, `CTRL_HUM` (bara
  BME280, skrivs **före** `CTRL_MEAS`), `CONFIG` = IIR 4 + 0,5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- Det finns ingen klar-flagga – `ready = true`, avläses var 25:e ms.
- Kompensering – Bosch-formlerna i §8.1 (double); skydd mot division med noll.

---

## `SPL06_Sensor`

**Fil:** `sensors/baro/SPL06_Sensor.h` · **Ärver:** `BarometerBase` · **Status:** inte verifierad på hårdvara

SPL06-001 (Goertek). Formlerna kommer från datablad §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 är SPA06, som har en annan uppsättning koefficienter –
  den avvisas); återställning, vänta på `COEF_RDY | SENSOR_RDY`; 18 byte koefficienter
  (12/20/16-bitars fält med tecken); temperaturkällan – enligt `COEF_SRCE`;
  tryck 16× (32 Hz), temperatur 1×, kontinuerligt läge.
- `isNewSampleReady()` – biten `PRS_RDY`; `readSample()` – 24-bitars
  big-endian-mätningar, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Fil:** `sensors/baro/BMP581_Sensor.h` · **Ärver:** `BarometerBase` · **Status:** huvudbarometern i standardsatsen, inte verifierad på hårdvara

BMP581 (Bosch). Sekvensen följer den officiella BMP5_SensorAPI.

- `begin()`: en dummyläsning (för SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  mjukvaruåterställning, `INT_STATUS` POR och `STATUS` NVM ready utan fel;
  standby → OSR (tryck 16×, temperatur 2×), IIR, DRDY; en kontroll av att
  ODR är genomförbar (`OSR_EFF`); kontinuerligt läge.
- En mätning – vid DRDY, eller var 40:e ms om flaggan går förlorad; temperatur
  `int24/65536`, tryck `uint24/64`.
- Två instanser i bygget med röret: huvudbarometern och `PITOT-BMP581`.

---

## `MagnetometerBase`

**Fil:** `sensors/mag/MagnetometerBase.h` · **Ärver:** `MagnetometerSensor` · **Typ:** abstrakt

| Metod | Beskrivning |
|---|---|
| `bool isAvailable() const` | `begin()` lyckades och färre än 25 fel i följd |
| `void update()` | 50 Hz: `readRaw()` → dra ifrån förskjutningarna → skala → rotera med `MAG_ROTATION_CW_DEG` → kurs `atan2(Y, X)` i 0..360 |
| `void calibrate()` | 15 s rotation: förskjutning = (min + max)/2 per axel (hard-iron), sparas i NVS. Om det inte fanns en enda lyckad avläsning avvisas kalibreringen och den föregående i NVS lämnas orörd |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

Det skyddade API:t: konstruktorn `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (vid `true` laddar den kalibreringen från NVS).

Kursen är utan lutningskompensation: korrekt medan farkosten är nästan plan. Nosen mot
norr → fältet längs +X → 0°; nosen mot öster → 90°.

---

## `QMC5883P_Sensor`

**Fil:** `sensors/mag/QMC5883P_Sensor.h` · **Ärver:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Status:** på bänken

`DEFAULT_ADDRESS = 0x2C`. `begin()`: krets-ID `0x80` (register 0x00), mjukvaru-
återställning, axeltecken `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). Data – 6 byte från `0x01`,
little-endian. 37,5 LSB/µT.

---

## `QMC5883L_Sensor`

**Fil:** `sensors/mag/QMC5883L_Sensor.h` · **Ärver:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Status:** inte verifierad på hårdvara

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (kretsen har inget pålitligt ID),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (kontinuerligt, 200 Hz, ±8 G, OSR 512).
Data – 6 byte från `0x00`, little-endian. 30 LSB/µT. Registren är **inte kompatibla**
med QMC5883P.

---

## `QMC6309_Sensor`

**Fil:** `sensors/mag/QMC6309_Sensor.h` · **Ärver:** `MagnetometerBase` · **NVS:** `qmc6309` · **Status:** inte verifierad på hårdvara

QMC6309 (QST), I2C 0x7C – en adress utanför det vanliga intervallet 0x08..0x77
(konsolens bussavsökning `b` går upp till 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; återställning (CTRL2 0x80 → 0x00), vänta på
  `NVM_RDY | NVM_LOAD_DONE`; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian från 0x01; 40,96 LSB/µT.

---

## `UbloxM10_Gps`

**Fil:** `sensors/gps/UbloxM10_Gps.h` · **Ärver:** `GpsSensor` · **Beror på:** `IUartPort`, `Config` · **Status:** inte ansluten på bänken

En u-blox M10 över UART, UBX-protokollet. Bara **NAV-PVT** tolkas (klass 0x01,
id 0x07, 92 byte).

| Metod | Beskrivning |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 baud → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 baud → CFG-VALSET: 10 Hz, NAV-PVT på UART1, UBX på, NMEA av. Utan TX-stift (`PIN_GPS_TX < 0`) lyssnar den bara på 9600. Alltid `true` (det finns inget ACK) |
| `bool isAvailable() const` | Det fanns en giltig NAV-PVT och den senaste är inte äldre än `GPS_TIMEOUT_US` |
| `void update()` | Mata allt från UART:en till tolken |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = tillgänglig och `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` visar `available` enligt `isAvailable()` (med hänsyn till tidsgränsen) |

Tolken är en tillståndsmaskin som arbetar byte för byte `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, kontrollsumman Fletcher-8 över class+id+len+payload.
En längd > 512 är en förlorad synkronisering, och sökningen börjar om. Fälten i NAV-PVT tolkas som: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, omslagen till 0..360).

Den inbäddade `ValsetBuilder` är nyttolasten för UBX-CFG-VALSET (version 0, lager RAM, par
av en U4 LE-nyckel – ett LE-värde på 1/2/4 byte, en buffert på 64 byte).
