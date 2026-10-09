# SENSORS — sensores

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/sensors.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

Los sensores se organizan en tres niveles:

1. **Interfaces de categoría** (`SensorInterface.h`) — lo que ven `Autopilot`,
   `ArmingManager` y la telemetría.
2. **Clases base de categoría** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — todo lo común: calibraciones, filtros, rotación de ejes, signos,
   conteo de errores, almacenamiento en NVS. El patrón Template Method.
3. **Drivers de chips** — solo los registros y las fórmulas de la hoja de datos. Reciben
   `IRegisterDevice&` (no distinguen el bus) o `IUartPort&`.

Qué chip se compila lo decide `SensorSelection.h`.

---

## Interfaces y estructuras de datos

**Archivo:** `sensors/SensorInterface.h`

### Estructuras

| Estructura | Campos |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (alabeo + ala derecha hacia abajo, cabeceo + morro hacia arriba, guiñada + morro hacia la derecha); `accelX/Y/Z` g (ejes del avión: X hacia el morro, Y a la izquierda, Z hacia arriba); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, la integral del giroscopio) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m **relativa al punto de calibración**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT tras la calibración hard-iron, en los ejes del avión; `headingDegrees` 0..360 (sin compensación de inclinación); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 ninguno, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (interfaz)

| Método | Descripción |
|---|---|
| `bool begin()` | Identifica y configura el chip; `true` — el sensor funciona |
| `bool isAvailable() const` | Conectado y respondiendo **ahora mismo** |
| `void update()` | Se llama en cada ciclo; decide por sí mismo si toca leer |
| `const char* getSensorType() const` | Un nombre para el registro |
| `void printStatus() const` | Una línea de diagnóstico (consola `s`) |

### `ImuSensor : Sensor`

| Método | Descripción |
|---|---|
| `const ImuData& getImuData() const` | Los últimos datos |
| `void calibrate()` | Calibración del giroscopio (en reposo) + la comprobación previa al vuelo |
| `void setYaw(float)` | Fija el rumbo (por ejemplo, con la brújula al arrancar) |
| `virtual void calibrateOrientation()` | Calibración del montaje de la placa (por defecto no se admite) |
| `virtual const char* getPreflightProblem() const` | El problema de la comprobación previa al vuelo o `nullptr` (por defecto `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (la altitud actual = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: girar durante 15 s).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (hay al menos una posición 2D).

---

## `AirspeedSensor`

**Archivo:** `sensors/airspeed/AirspeedSensor.h` · **Clase:** interfaz · **Implementación:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
la presión diferencial tras la puesta a cero y el filtro, la velocidad indicada (ρ0 = 1.225), la
velocidad verdadera (ρ a partir de la presión estática y la temperatura), la densidad. Métodos: `getAirspeedData()`,
`calibrateZero()` (reiniciar la puesta a cero), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Archivo:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Hereda de:** `AirspeedSensor` · **Estado:** verificada en una simulación en lazo cerrado con ruido, sin probar en vuelo

Un tubo de Pitot casero con **dos barómetros absolutos**: `total` — el
BMP581 dentro del tubo (presión total), `stat` — el barómetro principal del fuselaje
(presión estática). La guía de montaje está en [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#un-tubo-de-pitot-casero).

| Método | Descripción |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` del barómetro del tubo (el estático ya está en marcha), inicia la puesta a cero |
| `update()` | una muestra nueva del tubo → diferencial − cero, filtro paso bajo `PITOT_FILTER_TAU_S`; las primeras `PITOT_ZERO_SAMPLES` muestras promedian el cero; la velocidad es `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | ambos barómetros están vivos, el cero está reunido, no hay fallo, la muestra es más reciente que `PITOT_STALE_US` |
| `hasFault()` | el diferencial permanece por debajo de −`PITOT_NEGATIVE_FAULT_PA` durante más de `PITOT_NEGATIVE_FAULT_MS` (mangueras, agua) |
| `getZeroOffset()`, `printStatus()` | diagnóstico |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | fórmulas |

---

## namespace `SensorMounting`

**Archivo:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
gira los ejes del chip alrededor de la vertical (chip hacia arriba) hasta los ejes del avión (X hacia el morro,
Y a la izquierda). `rotationCwDeg` es hacia dónde apunta el eje X del chip, en sentido horario visto desde arriba:

| Valor | bodyX | bodyY |
|---|---|---|
| 0 (y cualquier valor desconocido) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Lo usan la brújula (`MAG_ROTATION_CW_DEG`) y una IMU sin calibración del montaje.

---

## `SensorSelection.h`

**Archivo:** `sensors/SensorSelection.h` · **Clase:** configuración del preprocesador

El único lugar donde se cambia un sensor físico: un kit preparado en una sola línea
(`SENSOR_KIT`) o cada sensor por separado. Cualquier elección puede sobrescribirse
con una bandera de compilación (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Kit `SENSOR_KIT` | IMU | Barómetro | Brújula | Velocidad del aire | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, por defecto) | MPU6500 | BMP581 I2C (0x46/0x47) | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (fuselaje) | QMC6309 | BMP581 en el tubo | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (fuselaje) | QMC6309 | BMP581 en el tubo | M10 |
| `SENSOR_KIT_CUSTOM` (0) | definir las cinco macros siguientes | | | | |

| Macro de selección | Opciones |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — un BMP581 I2C 0x47 en el tubo |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Todas las combinaciones se compilan en todas las placas — `tools/build_matrix.sh`.

El resultado son alias de tipos y fábricas de dispositivos:

| Nombre | MPU6050 / ICM42688, etc. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (no definidas para NONE) |
| las nuevas IMU | `LSM6DSV_Sensor` + I2C 0x6A (de reserva 0x6B) o SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) o SPI |
| los nuevos barómetros | `SPL06_Sensor` + I2C 0x76 (0x77) o SPI; `BMP581_Sensor` + I2C 0x46 (0x47) o SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (no definidas para NONE) |
| `SelectedGps` | `UbloxM10_Gps` (no definido para NONE) |

`main.cpp` envuelve la creación de la brújula, del GPS y del tubo en `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Archivo:** `sensors/imu/ImuOrientation.h` · **Depende de:** `SensorMounting`, `Preferences` (NVS)

La matriz de rotación `R` de los ejes del chip a los ejes del avión: `body = R · chip`; las filas de
`R` son los ejes del avión expresados en los ejes del chip.

| Método | Descripción |
|---|---|
| `ImuOrientation()` | La identidad (equivale a `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | Una rotación alrededor de la vertical en pasos de 90°, con la placa y el chip hacia arriba |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Calibración a partir de tres posturas (la lectura «arriba» del acelerómetro en los ejes del chip). `nullptr` — éxito; si no, el motivo del rechazo |
| `void apply(const float chip[3], float body[3]) const` | Aplica la rotación |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | El ángulo entre el «arriba» medido y el eje Z del avión (180° si el vector es nulo) |
| `bool load(const char* ns)` | Carga desde NVS; rechaza una terna no ortonormal o levógira |
| `void save(const char* ns) const` | Guarda en NVS |
| `void describe(Print&) const` | «morro = +Y del chip, arriba = +Z del chip» (con un ángulo si un eje no coincide con un eje del chip con una tolerancia de ±14°) |

El algoritmo de `fromPoses`: Z = norm(level); X₁ = la parte de noseUp ⟂ Z; Y = la parte de
rightWingDown ⟂ Z, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X. Los rechazos:

| Condición | Mensaje |
|---|---|
| vector nulo | «sin lecturas del acelerómetro» |
| la inclinación del paso 2 o 3 queda fuera de 20..80° | «el paso N necesita una inclinación de 30-60°» |
| cos(X₁, X₂) < −0.5 | «los pasos 2 y 3 se contradicen entre sí…» (se bajó el morro o se usó el ala equivocada) |
| cos(X₁, X₂) < 0.9 (≈25°) | «…se inclinaron los ejes equivocados…» |

---

## `AttitudeEstimator`

**Archivo:** `sensors/imu/AttitudeEstimator.h`

Un filtro complementario de alabeo/cabeceo y una integral de guiñada, independiente del chip.

| Método | Descripción |
|---|---|
| `void reset()` | El siguiente `update()` empieza de inmediato con el ángulo del acelerómetro |
| `void setYaw(float deg)` | Fija el rumbo (se reduce a ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | La aceleración en g en los ejes del avión, las velocidades en °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0.1 s con 2 ms).
La primera llamada tras `reset()` da de inmediato los ángulos del acelerómetro; con `dt ≤ 0` o mayor de 0.1 s
se omite el paso (una pausa, un bus colgado).

---

## `ImuSensorBase`

**Archivo:** `sensors/imu/ImuSensorBase.h` · **Hereda de:** `ImuSensor` · **Clase:** abstracta

`RawImuSample` — una muestra en unidades del ADC en los ejes del chip:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

El flujo `update()` → `process()`:

```
muestras en bruto − desfases (ADC) → escala (g, °/s) → ImuOrientation (ejes del avión)
→ signos aeronáuticos (gyroY, gyroZ con el signo cambiado) → AttitudeEstimator
```

| Método | Descripción |
|---|---|
| `bool isAvailable() const` | `begin()` tuvo éxito y hay menos de 50 errores de lectura seguidos |
| `void update()` | Una lectura; si hay un error los datos no cambian y los contadores aumentan |
| `void calibrate()` | 200 muestras × 10 ms: el desfase del giroscopio, el ruido, el «arriba»; sin calibración del montaje — el horizonte = la posición actual. Después, la comprobación previa al vuelo |
| `void calibrateOrientation()` | Tres posturas (`capturePose`: quieto ~1 s, la postura difiere de las anteriores en ≥ 20°, tiempo límite de 30 s), `ImuOrientation::fromPoses`, guardado en NVS. Elimina los problemas de montaje de la comprobación previa al vuelo |
| `const char* getPreflightProblem() const` | El texto del problema o `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

La API protegida para los drivers:

| Método | Descripción |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Cada driver tiene su propio espacio de nombres de NVS |
| `virtual bool readSample(RawImuSample&) = 0` | Una muestra; `false` — el chip no respondió |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | Las escalas |
| `virtual float temperatureC(int16_t raw) const = 0` | La fórmula de la temperatura |
| `void setAvailable(bool)` | El resultado de `begin()`; con `true` carga el montaje desde NVS o desde `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Precisar el nombre tras la identificación |

La comprobación previa al vuelo (`runPreflightCheck`), por orden:

| Problema | Condición |
|---|---|
| `NotResponding` | se leyeron menos de la mitad de las muestras de calibración |
| `Moved` | el ruido del giroscopio > 0.5 °/s |
| `NotOneG` | \|a\| difiere de 1g en más de 0.2g |
| `MountingMismatch` | (el montaje está calibrado) el «arriba» está a más de 45° del guardado |
| `NotChipUp` | (sin calibrar) la placa no está colocada con el chip hacia arriba (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Archivo:** `sensors/imu/MPU6050_Sensor.h` · **Hereda de:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Estado:** en el banco de pruebas (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 y clones (placas GY-521), I2C 0x68/0x69 o
SPI. El chip se identifica por `WHO_AM_I` (0x68 — MPU6050, si no, la familia 6500).

- `begin()`: WHO_AM_I (sin respuesta → no disponible), el nombre según el ID, reinicio, salida
  del modo de reposo (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; el 6500 tiene un filtro
  paso bajo del acelerómetro aparte, `ACCEL_CONFIG2`.
- `readSample()`: 14 bytes desde `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Escalas: 2048 LSB/g, 16.4 LSB/(°/s).
- Temperatura: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Archivo:** `sensors/imu/ICM42688_Sensor.h` · **Hereda de:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Estado:** no verificado en el hardware

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, sin
  byte ficticio.
- `begin()`: banco 0, reinicio por software, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, filtro UI 50 Hz.
- `readSample()`: 14 bytes desde `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Temperatura: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Archivo:** `sensors/imu/LSM6DSV_Sensor.h` · **Hereda de:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Estado:** no verificado en el hardware

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Los registros se contrastaron con `lsm6dsv-pid` de ST y con ArduPilot.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (bit 0 de CTRL3) y una espera;
  el 32X se distingue por el bit de variante de CTRL8 (tiene su propio código de ±16 g); BDU + autoincremento,
  ±2000 °/s con LPF1, ±16 g con LPF2, 960 Hz de alto rendimiento.
- `readSample()`: 14 bytes desde 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Escalas: 1000/0.488 LSB/g, 1000/70 LSB/(°/s); temperatura `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI modo 0, sin byte ficticio.

## `ICM45686_Sensor`

**Archivo:** `sensors/imu/ICM45686_Sensor.h` · **Hereda de:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Estado:** no verificado en el hardware

ICM-45686 (TDK). Los registros se contrastaron con el driver de TDK, con Zephyr y con ArduPilot.

- `begin()`: reinicio mediante `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s y ±16 g
  a 1.6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); el filtro
  paso bajo ODR/32 — lectura-modificación-escritura de los registros **indirectos** IPREG
  (0xA4AC, 0xA583) a través de la ventana 0x7C..0x7E; 45 ms para que arranque el giroscopio.
- `readSample()`: 14 bytes desde 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Escalas: 2048 LSB/g, 16.4 LSB/(°/s); temperatura `raw/132.48 + 25`.

---

## `BarometerBase`

**Archivo:** `sensors/baro/BarometerBase.h` · **Hereda de:** `BarometerSensor` · **Clase:** abstracta

| Método | Descripción |
|---|---|
| `bool isAvailable() const` | `begin()` tuvo éxito y hay menos de 100 errores seguidos |
| `void update()` | No más a menudo que `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → altitud y velocidad vertical |
| `void calibrateAltitude()` | 20 muestras × 50 ms: la altitud absoluta media = la base; pone a cero la altitud y la velocidad |
| `void setSeaLevelPressure(Pa)` | P₀ (101325 por defecto) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

La API protegida: el constructor `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Fórmulas: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; la velocidad vertical es la
derivada de la altitud sobre las muestras **realmente nuevas**, a través de un filtro paso bajo con τ = 0.5 s
(si `dt` queda fuera de `(0, 0.5 s)`, la velocidad no se actualiza). Leer solo muestras nuevas
elimina el ruido «escalonado» (0 m/s intercalado con saltos de Δh/2 ms).

---

## `BMP388_Sensor`

**Archivo:** `sensors/baro/BMP388_Sensor.h` · **Hereda de:** `BarometerBase` · **Estado:** en el banco de pruebas (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **1 byte ficticio** (hoja de datos §5.3.2).
- `begin()`: ID del chip `0x50`, reinicio por software, 21 bytes de coeficientes de NVM desde
  `0x31` (escalas §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, modo normal.
- `isNewSampleReady()`: la bandera `drdy_press` (bit 0x20) del registro STATUS; se consulta
  cada 5 ms.
- `readSample()`: 6 bytes desde `0x04`; compensación de Bosch §9.3 (double):
  primero la temperatura (`tLin`) y después la presión.

---

## `BME280_Sensor`

**Archivo:** `sensors/baro/BME280_Sensor.h` · **Hereda de:** `BarometerBase` · **Estado:** no verificado en el hardware

BME280 (ID 0x60) y BMP280 (ID 0x58), I2C 0x76/0x77 o SPI sin byte
ficticio. La humedad no se lee.

- `begin()`: ID del chip, reinicio, 24 bytes de calibración desde `0x88`, `CTRL_HUM` (solo
  BME280, se escribe **antes** de `CTRL_MEAS`), `CONFIG` = IIR 4 + 0.5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- No hay bandera de disponibilidad — `ready = true`, se consulta cada 25 ms.
- Compensación — las fórmulas de Bosch §8.1 (double); protección contra la división por cero.

---

## `SPL06_Sensor`

**Archivo:** `sensors/baro/SPL06_Sensor.h` · **Hereda de:** `BarometerBase` · **Estado:** no verificado en el hardware

SPL06-001 (Goertek). Las fórmulas son de la hoja de datos §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 es el SPA06, con otro conjunto de coeficientes —
  se rechaza); reinicio, espera de `COEF_RDY | SENSOR_RDY`; 18 bytes de coeficientes
  (campos con signo de 12/20/16 bits); la fuente de temperatura — según `COEF_SRCE`;
  presión 16× (32 Hz), temperatura 1×, modo continuo.
- `isNewSampleReady()` — el bit `PRS_RDY`; `readSample()` — muestras de 24 bits
  big-endian, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Archivo:** `sensors/baro/BMP581_Sensor.h` · **Hereda de:** `BarometerBase` · **Estado:** barómetro principal del conjunto por defecto, no verificado en el hardware

BMP581 (Bosch). La secuencia sigue la BMP5_SensorAPI oficial.

- `begin()`: una lectura ficticia (para SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  reinicio por software, `INT_STATUS` POR y `STATUS` con la NVM lista y sin errores;
  standby → OSR (presión 16×, temperatura 2×), IIR, DRDY; una comprobación de que
  el ODR es viable (`OSR_EFF`); modo continuo.
- Una muestra — con DRDY, o cada 40 ms si se pierde la bandera; temperatura
  `int24/65536`, presión `uint24/64`.
- Dos instancias en la compilación con el tubo: el barómetro principal y `PITOT-BMP581`.

---

## `MagnetometerBase`

**Archivo:** `sensors/mag/MagnetometerBase.h` · **Hereda de:** `MagnetometerSensor` · **Clase:** abstracta

| Método | Descripción |
|---|---|
| `bool isAvailable() const` | `begin()` tuvo éxito y hay menos de 25 errores seguidos |
| `void update()` | 50 Hz: `readRaw()` → restar los desfases → escalar → girar según `MAG_ROTATION_CW_DEG` → rumbo `atan2(Y, X)` en 0..360 |
| `void calibrate()` | 15 s de giro: desfase = (min + max)/2 por eje (hard-iron), guardado en NVS. Si no hubo ni una sola lectura correcta, la calibración se rechaza y la anterior en NVS no se toca |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

La API protegida: el constructor `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (con `true` carga la calibración desde NVS).

El rumbo sin compensación de inclinación: es correcto mientras el avión esté casi nivelado. Morro al
norte → el campo a lo largo de +X → 0°; morro al este → 90°.

---

## `QMC5883P_Sensor`

**Archivo:** `sensors/mag/QMC5883P_Sensor.h` · **Hereda de:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Estado:** en el banco de pruebas

`DEFAULT_ADDRESS = 0x2C`. `begin()`: ID del chip `0x80` (registro 0x00), reinicio
por software, signos de los ejes `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). Los datos — 6 bytes desde `0x01`,
little-endian. 37.5 LSB/µT.

---

## `QMC5883L_Sensor`

**Archivo:** `sensors/mag/QMC5883L_Sensor.h` · **Hereda de:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Estado:** no verificado en el hardware

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (el chip no tiene un ID fiable),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
Los datos — 6 bytes desde `0x00`, little-endian. 30 LSB/µT. Los registros **no son compatibles**
con el QMC5883P.

---

## `QMC6309_Sensor`

**Archivo:** `sensors/mag/QMC6309_Sensor.h` · **Hereda de:** `MagnetometerBase` · **NVS:** `qmc6309` · **Estado:** no verificado en el hardware

QMC6309 (QST), I2C 0x7C — una dirección fuera del rango habitual 0x08..0x77
(el sondeo de buses de la consola `b` llega hasta 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; reinicio (CTRL2 0x80 → 0x00), espera de
  `NVM_RDY | NVM_LOAD_DONE`; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian desde 0x01; 40.96 LSB/µT.

---

## `UbloxM10_Gps`

**Archivo:** `sensors/gps/UbloxM10_Gps.h` · **Hereda de:** `GpsSensor` · **Depende de:** `IUartPort`, `Config` · **Estado:** sin conectar en el banco de pruebas

Un u-blox M10 por UART, protocolo UBX. Solo se analiza **NAV-PVT** (clase 0x01,
id 0x07, 92 bytes).

| Método | Descripción |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 baudios → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 baudios → CFG-VALSET: 10 Hz, NAV-PVT en UART1, UBX activado, NMEA desactivado. Sin pin TX (`PIN_GPS_TX < 0`) solo escucha a 9600. Siempre `true` (no hay ACK) |
| `bool isAvailable() const` | Hubo un NAV-PVT válido y el último no es más antiguo que `GPS_TIMEOUT_US` |
| `void update()` | Pasar al analizador todo lo que haya en el UART |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = disponible y `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` muestra `available` según `isAvailable()` (teniendo en cuenta el tiempo de espera) |

El analizador es un autómata byte a byte `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, con la suma de comprobación Fletcher-8 sobre class+id+len+payload.
Una longitud > 512 es una pérdida de sincronización, y la búsqueda empieza de nuevo. Los campos de NAV-PVT se analizan así: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, reducido a 0..360).

El `ValsetBuilder` anidado es la carga útil de UBX-CFG-VALSET (version 0, layer RAM, pares
de clave U4 LE — valor de 1/2/4 bytes LE, un búfer de 64 bytes).
