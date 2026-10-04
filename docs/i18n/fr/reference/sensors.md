# SENSORS — capteurs

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/sensors.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

[← Référence](README.md)

Les capteurs sont construits sur trois niveaux :

1. **Interfaces de catégorie** (`SensorInterface.h`) — ce que voient `Autopilot`,
   `ArmingManager` et la télémétrie.
2. **Classes de base de catégorie** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — tout ce qui est commun : étalonnages, filtres, rotation des axes, signes,
   comptage des erreurs, stockage dans la NVS. Le patron Template Method.
3. **Pilotes de puces** — seulement les registres et les formules de la fiche technique. Ils reçoivent
   `IRegisterDevice&` (ils ne distinguent pas le bus) ou `IUartPort&`.

La puce qui est compilée est décidée par `SensorSelection.h`.

---

## Interfaces et structures de données

**Fichier :** `sensors/SensorInterface.h`

### Structures

| Structure | Champs |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (roulis + aile droite vers le bas, tangage + nez vers le haut, lacet + nez vers la droite) ; `accelX/Y/Z` g (axes de l’avion : X vers le nez, Y vers la gauche, Z vers le haut) ; `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, l’intégrale du gyroscope) ° ; `temperature` °C ; `timestamp` µs |
| `BarometerData` | `pressure` Pa ; `temperature` °C ; `altitude` m **par rapport au point d’étalonnage** ; `verticalSpeed` m/s ; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT après l’étalonnage hard-iron, dans les axes de l’avion ; `headingDegrees` 0..360 (sans compensation de l’inclinaison) ; `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °) ; `altitude` m MSL ; `groundSpeed` m/s ; `heading` 0..360 ; `numSatellites` ; `fixType` (0 aucun, 2 — 2D, 3 — 3D) ; `horizontalAccuracy`, `verticalAccuracy` m ; `timestamp` |

### `Sensor` (interface)

| Méthode | Description |
|---|---|
| `bool begin()` | Identifier et configurer la puce ; `true` — le capteur fonctionne |
| `bool isAvailable() const` | Branché et répondant **en ce moment** |
| `void update()` | À appeler à chaque cycle ; il décide lui-même s’il est temps de lire |
| `const char* getSensorType() const` | Un nom pour le journal |
| `void printStatus() const` | Une ligne de diagnostic (commande `s` de la console) |

### `ImuSensor : Sensor`

| Méthode | Description |
|---|---|
| `const ImuData& getImuData() const` | Les dernières données |
| `void calibrate()` | Étalonnage du gyroscope (immobile) + la vérification avant le vol |
| `void setYaw(float)` | Fixer le cap (par exemple d’après le compas au démarrage) |
| `virtual void calibrateOrientation()` | Étalonnage du montage de la carte (non pris en charge par défaut) |
| `virtual const char* getPreflightProblem() const` | Le problème de la vérification avant le vol ou `nullptr` (par défaut `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (l’altitude actuelle = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron : tourner pendant 15 s).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (il y a au moins une position 2D).

---

## `AirspeedSensor`

**Fichier :** `sensors/airspeed/AirspeedSensor.h` · **Genre :** interface · **Implémentation :** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }` :
la pression différentielle après la mise à zéro et le filtre, la vitesse indiquée (ρ0 = 1,225), la
vitesse vraie (ρ d’après la pression statique et la température), la densité. Méthodes : `getAirspeedData()`,
`calibrateZero()` (relancer la mise à zéro), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Fichier :** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Hérite de :** `AirspeedSensor` · **Statut :** vérifiée en simulation en boucle fermée avec du bruit, non éprouvée en vol

Un tube de Pitot fait maison avec **deux baromètres absolus** : `total` — le
BMP581 à l’intérieur du tube (pression totale), `stat` — le baromètre principal du fuselage
(pression statique). Le guide de montage se trouve dans [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#un-tube-de-pitot-fait-maison).

| Méthode | Description |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` du baromètre du tube (le statique est déjà lancé), démarre la mise à zéro |
| `update()` | un nouvel échantillon du tube → différentiel − zéro, filtre passe-bas `PITOT_FILTER_TAU_S` ; les `PITOT_ZERO_SAMPLES` premiers échantillons moyennent le zéro ; la vitesse est `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | les deux baromètres sont vivants, le zéro est réuni, il n’y a pas de défaut, l’échantillon est plus récent que `PITOT_STALE_US` |
| `hasFault()` | le différentiel reste sous −`PITOT_NEGATIVE_FAULT_PA` plus longtemps que `PITOT_NEGATIVE_FAULT_MS` (tuyaux, eau) |
| `getZeroOffset()`, `printStatus()` | diagnostic |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | formules |

---

## namespace `SensorMounting`

**Fichier :** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
fait tourner les axes de la puce autour de la verticale (puce vers le haut) vers les axes de l’avion (X vers le nez,
Y vers la gauche). `rotationCwDeg` est la direction que montre l’axe X de la puce, dans le sens horaire vu du dessus :

| Valeur | bodyX | bodyY |
|---|---|---|
| 0 (et toute valeur inconnue) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Utilisé par le compas (`MAG_ROTATION_CW_DEG`) et par une IMU sans étalonnage du montage.

---

## `SensorSelection.h`

**Fichier :** `sensors/SensorSelection.h` · **Genre :** configuration du préprocesseur

Le seul endroit pour changer un capteur physique : un kit prêt à l’emploi en une ligne
(`SENSOR_KIT`) ou chaque capteur séparément. Tout choix peut être remplacé
par une option de compilation (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Kit `SENSOR_KIT` | IMU | Baromètre | Compas | Vitesse de l’air | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, par défaut) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (fuselage) | QMC6309 | BMP581 dans le tube | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (fuselage) | QMC6309 | BMP581 dans le tube | M10 |
| `SENSOR_KIT_CUSTOM` (0) | définir les cinq macros ci-dessous | | | | |

| Macro de sélection | Variantes |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — un BMP581 I2C 0x47 dans le tube |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Toutes les combinaisons se compilent sur toutes les cartes — `tools/build_matrix.sh`.

Le résultat, ce sont des alias de types et des fabriques de périphériques :

| Nom | MPU6050 / ICM42688, etc. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (non définies pour NONE) |
| les nouvelles IMU | `LSM6DSV_Sensor` + I2C 0x6A (de secours 0x6B) ou SPI ; `ICM45686_Sensor` + I2C 0x68 (0x69) ou SPI |
| les nouveaux baromètres | `SPL06_Sensor` + I2C 0x76 (0x77) ou SPI ; `BMP581_Sensor` + I2C 0x46 (0x47) ou SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (non définies pour NONE) |
| `SelectedGps` | `UbloxM10_Gps` (non défini pour NONE) |

`main.cpp` enveloppe la création du compas, du GPS et du tube dans `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Fichier :** `sensors/imu/ImuOrientation.h` · **Dépend de :** `SensorMounting`, `Preferences` (NVS)

La matrice de rotation `R` des axes de la puce vers les axes de l’avion : `body = R · chip` ; les lignes de
`R` sont les axes de l’avion exprimés dans les axes de la puce.

| Méthode | Description |
|---|---|
| `ImuOrientation()` | L’identité (équivalente à `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | Une rotation autour de la verticale par pas de 90°, la carte avec la puce vers le haut |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Étalonnage à partir de trois poses (la lecture « haut » de l’accéléromètre dans les axes de la puce). `nullptr` — succès, sinon la raison du refus |
| `void apply(const float chip[3], float body[3]) const` | Appliquer la rotation |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | L’angle entre le « haut » mesuré et l’axe Z de l’avion (180° si le vecteur est nul) |
| `bool load(const char* ns)` | Charger depuis la NVS ; rejette un trièdre non orthonormé ou indirect |
| `void save(const char* ns) const` | Enregistrer dans la NVS |
| `void describe(Print&) const` | « nez = +Y de la puce, haut = +Z de la puce » (avec un angle si un axe ne coïncide pas avec un axe de la puce à ±14° près) |

L’algorithme de `fromPoses` : Z = norm(level) ; X₁ = la partie de noseUp ⟂ Z ; Y = la partie de
rightWingDown ⟂ Z, X₂ = Y × Z ; X = norm(X₁ + X₂), Y = Z × X. Les refus :

| Condition | Message |
|---|---|
| vecteur nul | « pas de lectures de l’accéléromètre » |
| l’inclinaison de l’étape 2 ou 3 est hors de 20..80° | « l’étape N demande une inclinaison de 30-60° » |
| cos(X₁, X₂) < −0,5 | « les étapes 2 et 3 se contredisent… » (le nez a été baissé ou la mauvaise aile a été utilisée) |
| cos(X₁, X₂) < 0,9 (≈25°) | « …on a incliné les mauvais axes… » |

---

## `AttitudeEstimator`

**Fichier :** `sensors/imu/AttitudeEstimator.h`

Un filtre complémentaire de roulis/tangage et une intégrale de lacet, indépendant de la puce.

| Méthode | Description |
|---|---|
| `void reset()` | Le prochain `update()` démarre aussitôt avec l’angle de l’accéléromètre |
| `void setYaw(float deg)` | Fixer le cap (ramené à ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | L’accélération en g dans les axes de l’avion, les vitesses en °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))` ;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0,1 s à 2 ms).
Le premier appel après `reset()` donne aussitôt les angles de l’accéléromètre ; avec `dt ≤ 0` ou supérieur à 0,1 s,
le pas est ignoré (une pause, un bus bloqué).

---

## `ImuSensorBase`

**Fichier :** `sensors/imu/ImuSensorBase.h` · **Hérite de :** `ImuSensor` · **Genre :** abstraite

`RawImuSample` — un échantillon en unités du CAN dans les axes de la puce :
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

La chaîne `update()` → `process()` :

```
échantillons bruts − décalages (CAN) → échelle (g, °/s) → ImuOrientation (axes de l’avion)
→ signes aéronautiques (gyroY, gyroZ avec le signe changé) → AttitudeEstimator
```

| Méthode | Description |
|---|---|
| `bool isAvailable() const` | `begin()` a réussi et il y a moins de 50 erreurs de lecture d’affilée |
| `void update()` | Une lecture ; en cas d’erreur, les données ne changent pas et les compteurs augmentent |
| `void calibrate()` | 200 échantillons × 10 ms : le décalage du gyroscope, le bruit, le « haut » ; sans étalonnage du montage — l’horizon = la position actuelle. Puis la vérification avant le vol |
| `void calibrateOrientation()` | Trois poses (`capturePose` : immobile ~1 s, la pose diffère des précédentes d’au moins 20°, délai de 30 s), `ImuOrientation::fromPoses`, enregistrement dans la NVS. Supprime les problèmes de montage de la vérification avant le vol |
| `const char* getPreflightProblem() const` | Le texte du problème ou `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

L’API protégée pour les pilotes :

| Méthode | Description |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Chaque pilote a son propre espace de noms NVS |
| `virtual bool readSample(RawImuSample&) = 0` | Un échantillon ; `false` — la puce n’a pas répondu |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | Les échelles |
| `virtual float temperatureC(int16_t raw) const = 0` | La formule de la température |
| `void setAvailable(bool)` | Le résultat de `begin()` ; avec `true`, charge le montage depuis la NVS ou depuis `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Préciser le nom après l’identification |

La vérification avant le vol (`runPreflightCheck`), dans l’ordre :

| Problème | Condition |
|---|---|
| `NotResponding` | moins de la moitié des échantillons d’étalonnage ont été lus |
| `Moved` | le bruit du gyroscope > 0,5 °/s |
| `NotOneG` | \|a\| diffère de 1g de plus de 0,2g |
| `MountingMismatch` | (le montage est étalonné) le « haut » est à plus de 45° de celui qui est enregistré |
| `NotChipUp` | (non étalonné) la carte n’est pas posée puce vers le haut (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Fichier :** `sensors/imu/MPU6050_Sensor.h` · **Hérite de :** `ImuSensorBase` · **NVS :** `imu_mpu6050` · **Statut :** sur le banc d’essai (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 et clones (cartes GY-521), I2C 0x68/0x69 ou
SPI. La puce est identifiée par `WHO_AM_I` (0x68 — MPU6050, sinon la famille 6500).

- `begin()` : WHO_AM_I (pas de réponse → indisponible), le nom d’après l’ID, réinitialisation, sortie
  de veille (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz ; le 6500 a un filtre
  passe-bas distinct pour l’accéléromètre, `ACCEL_CONFIG2`.
- `readSample()` : 14 octets depuis `0x3B`, big-endian : accel XYZ, temp, gyro XYZ.
- Échelles : 2048 LSB/g, 16,4 LSB/(°/s).
- Température : MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Fichier :** `sensors/imu/ICM42688_Sensor.h` · **Hérite de :** `ImuSensorBase` · **NVS :** `imu_icm42688` · **Statut :** non vérifié sur le matériel

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz, sans
  octet factice.
- `begin()` : banque 0, réinitialisation logicielle, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, filtre UI de 50 Hz.
- `readSample()` : 14 octets depuis `0x1D`, big-endian : temp, accel XYZ, gyro XYZ.
- Température : `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Fichier :** `sensors/imu/LSM6DSV_Sensor.h` · **Hérite de :** `ImuSensorBase` · **NVS :** `imu_lsm6dsv` · **Statut :** non vérifié sur le matériel

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Les registres ont été confrontés au `lsm6dsv-pid` de ST et à ArduPilot.

- `begin()` : `WHO_AM_I` (0x0F) = 0x70 ; `SW_RESET` (bit 0 de CTRL3) et une attente ;
  le 32X se reconnaît au bit de variante dans CTRL8 (il a son propre code pour ±16 g) ; BDU + autoincrémentation,
  ±2000 °/s avec LPF1, ±16 g avec LPF2, 960 Hz en haute performance.
- `readSample()` : 14 octets depuis 0x20, little-endian : temp, gyro XYZ, accel XYZ.
- Échelles : 1000/0,488 LSB/g, 1000/70 LSB/(°/s) ; température `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI mode 0, sans octet factice.

## `ICM45686_Sensor`

**Fichier :** `sensors/imu/ICM45686_Sensor.h` · **Hérite de :** `ImuSensorBase` · **NVS :** `imu_icm45686` · **Statut :** non vérifié sur le matériel

ICM-45686 (TDK). Les registres ont été confrontés au pilote de TDK, à Zephyr et à ArduPilot.

- `begin()` : réinitialisation par `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9 ; ±2000 °/s et ±16 g
  à 1,6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F) ; le filtre
  passe-bas ODR/32 — lecture-modification-écriture des registres **indirects** IPREG
  (0xA4AC, 0xA583) par la fenêtre 0x7C..0x7E ; 45 ms pour le démarrage du gyroscope.
- `readSample()` : 14 octets depuis 0x00, little-endian : accel XYZ, gyro XYZ, temp.
- Échelles : 2048 LSB/g, 16,4 LSB/(°/s) ; température `raw/132.48 + 25`.

---

## `BarometerBase`

**Fichier :** `sensors/baro/BarometerBase.h` · **Hérite de :** `BarometerSensor` · **Genre :** abstraite

| Méthode | Description |
|---|---|
| `bool isAvailable() const` | `begin()` a réussi et il y a moins de 100 erreurs d’affilée |
| `void update()` | Pas plus souvent que `pollPeriodUs` : `isNewSampleReady()` → `readSample()` → altitude et vitesse verticale |
| `void calibrateAltitude()` | 20 échantillons × 50 ms : l’altitude absolue moyenne = la base ; remet à zéro l’altitude et la vitesse |
| `void setSeaLevelPressure(Pa)` | P₀ (101325 par défaut) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

L’API protégée : le constructeur `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Formules : `h = 44330 · (1 − (P/P₀)^0.1903) − base` ; la vitesse verticale est la
dérivée de l’altitude sur les échantillons **réellement nouveaux**, à travers un filtre passe-bas avec τ = 0,5 s
(si `dt` est hors de `(0, 0.5 s)`, la vitesse n’est pas mise à jour). Ne lire que les nouveaux échantillons
supprime le bruit « en escalier » (0 m/s entrecoupé de sauts de Δh/2 ms).

---

## `BMP388_Sensor`

**Fichier :** `sensors/baro/BMP388_Sensor.h` · **Hérite de :** `BarometerBase` · **Statut :** sur le banc d’essai (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz,
  **1 octet factice** (fiche technique §5.3.2).
- `begin()` : ID de la puce `0x50`, réinitialisation logicielle, 21 octets de coefficients de la NVM depuis
  `0x31` (échelles §9.1), OSR ×8/×1, ODR 50 Hz, IIR 3, mode normal.
- `isNewSampleReady()` : le drapeau `drdy_press` (bit 0x20) du registre STATUS ; interrogé
  toutes les 5 ms.
- `readSample()` : 6 octets depuis `0x04` ; compensation de Bosch §9.3 (double) :
  d’abord la température (`tLin`), puis la pression.

---

## `BME280_Sensor`

**Fichier :** `sensors/baro/BME280_Sensor.h` · **Hérite de :** `BarometerBase` · **Statut :** non vérifié sur le matériel

BME280 (ID 0x60) et BMP280 (ID 0x58), I2C 0x76/0x77 ou SPI sans octet
factice. L’humidité n’est pas lue.

- `begin()` : ID de la puce, réinitialisation, 24 octets d’étalonnage depuis `0x88`, `CTRL_HUM` (seulement
  BME280, écrit **avant** `CTRL_MEAS`), `CONFIG` = IIR 4 + 0,5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- Il n’y a pas de drapeau de disponibilité — `ready = true`, interrogation toutes les 25 ms.
- Compensation — les formules de Bosch §8.1 (double) ; protection contre la division par zéro.

---

## `SPL06_Sensor`

**Fichier :** `sensors/baro/SPL06_Sensor.h` · **Hérite de :** `BarometerBase` · **Statut :** non vérifié sur le matériel

SPL06-001 (Goertek). Les formules viennent de la fiche technique §4.9.

- `begin()` : `ID` (0x0D) = 0x10 (0x11 est le SPA06, avec un autre jeu de coefficients —
  il est rejeté) ; réinitialisation, attente de `COEF_RDY | SENSOR_RDY` ; 18 octets de coefficients
  (champs signés de 12/20/16 bits) ; la source de température — d’après `COEF_SRCE` ;
  pression 16× (32 Hz), température 1×, mode continu.
- `isNewSampleReady()` — le bit `PRS_RDY` ; `readSample()` — échantillons de 24 bits
  big-endian, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Fichier :** `sensors/baro/BMP581_Sensor.h` · **Hérite de :** `BarometerBase` · **Statut :** non vérifié sur le matériel

BMP581 (Bosch). La séquence suit la BMP5_SensorAPI officielle.

- `begin()` : une lecture factice (pour le SPI), `CHIP_ID` (0x01) = 0x50/0x51 ;
  réinitialisation logicielle, `INT_STATUS` POR et `STATUS` avec la NVM prête et sans erreurs ;
  standby → OSR (pression 16×, température 2×), IIR, DRDY ; une vérification que
  l’ODR est réalisable (`OSR_EFF`) ; mode continu.
- Un échantillon — sur DRDY, ou toutes les 40 ms si le drapeau est perdu ; température
  `int24/65536`, pression `uint24/64`.
- Deux instances dans la version avec le tube : le baromètre principal et `PITOT-BMP581`.

---

## `MagnetometerBase`

**Fichier :** `sensors/mag/MagnetometerBase.h` · **Hérite de :** `MagnetometerSensor` · **Genre :** abstraite

| Méthode | Description |
|---|---|
| `bool isAvailable() const` | `begin()` a réussi et il y a moins de 25 erreurs d’affilée |
| `void update()` | 50 Hz : `readRaw()` → soustraire les décalages → échelle → rotation par `MAG_ROTATION_CW_DEG` → cap `atan2(Y, X)` dans 0..360 |
| `void calibrate()` | 15 s de rotation : décalage = (min + max)/2 par axe (hard-iron), enregistré dans la NVS. S’il n’y a pas eu une seule lecture réussie, l’étalonnage est rejeté et l’ancien, dans la NVS, n’est pas touché |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

L’API protégée : le constructeur `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (avec `true`, charge l’étalonnage depuis la NVS).

Le cap sans compensation de l’inclinaison : il est juste tant que l’avion est presque à l’horizontale. Nez vers le
nord → le champ le long de +X → 0° ; nez vers l’est → 90°.

---

## `QMC5883P_Sensor`

**Fichier :** `sensors/mag/QMC5883P_Sensor.h` · **Hérite de :** `MagnetometerBase` · **NVS :** `qmc5883p` · **Statut :** sur le banc d’essai

`DEFAULT_ADDRESS = 0x2C`. `begin()` : ID de la puce `0x80` (registre 0x00), réinitialisation
logicielle, signes des axes `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). Les données — 6 octets depuis `0x01`,
little-endian. 37,5 LSB/µT.

---

## `QMC5883L_Sensor`

**Fichier :** `sensors/mag/QMC5883L_Sensor.h` · **Hérite de :** `MagnetometerBase` · **NVS :** `qmc5883l` · **Statut :** non vérifié sur le matériel

`DEFAULT_ADDRESS = 0x0D`. `begin()` : `probe()` (la puce n’a pas d’ID fiable),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
Les données — 6 octets depuis `0x00`, little-endian. 30 LSB/µT. Les registres **ne sont pas compatibles**
avec le QMC5883P.

---

## `QMC6309_Sensor`

**Fichier :** `sensors/mag/QMC6309_Sensor.h` · **Hérite de :** `MagnetometerBase` · **NVS :** `qmc6309` · **Statut :** non vérifié sur le matériel

QMC6309 (QST), I2C 0x7C — une adresse hors de la plage habituelle 0x08..0x77
(l’interrogation des bus par la commande `b` de la console va jusqu’à 0x7F).

- `begin()` : `CHIP_ID` (0x00) = 0x90 ; réinitialisation (CTRL2 0x80 → 0x00), attente de
  `NVM_RDY | NVM_LOAD_DONE` ; ±8 G, 200 Hz, set/reset ; LPF 16, OSR 8, normal.
- `readRaw()` : X/Y/Z little-endian depuis 0x01 ; 40,96 LSB/µT.

---

## `UbloxM10_Gps`

**Fichier :** `sensors/gps/UbloxM10_Gps.h` · **Hérite de :** `GpsSensor` · **Dépend de :** `IUartPort`, `Config` · **Statut :** non connecté sur le banc d’essai

Un u-blox M10 par UART, protocole UBX. Seul **NAV-PVT** est analysé (classe 0x01,
id 0x07, 92 octets).

| Méthode | Description |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 bauds → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 bauds → CFG-VALSET : 10 Hz, NAV-PVT sur UART1, UBX activé, NMEA désactivé. Sans broche TX (`PIN_GPS_TX < 0`), il se contente d’écouter à 9600. Toujours `true` (il n’y a pas d’ACK) |
| `bool isAvailable() const` | Il y a eu un NAV-PVT valide et le dernier n’est pas plus ancien que `GPS_TIMEOUT_US` |
| `void update()` | Donner à l’analyseur tout ce qui se trouve dans l’UART |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = disponible et `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` affiche `available` d’après `isAvailable()` (en tenant compte du délai) |

L’analyseur est un automate octet par octet `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, avec la somme de contrôle de Fletcher-8 sur class+id+len+payload.
Une longueur > 512 est une perte de synchronisation, et la recherche reprend. Les champs de NAV-PVT sont analysés ainsi : `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, ramené à 0..360).

Le `ValsetBuilder` imbriqué est la charge utile d’UBX-CFG-VALSET (version 0, layer RAM, des paires
clé U4 LE — valeur de 1/2/4 octets LE, un tampon de 64 octets).
