# SENSORS — sensores

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/sensors.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

Os sensores são construídos em três níveis:

1. **Interfaces de categoria** (`SensorInterface.h`) — o que `Autopilot`,
   `ArmingManager` e a telemetria enxergam.
2. **Classes base de categoria** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — tudo o que é comum: calibrações, filtros, rotação dos eixos, sinais,
   contagem de erros, armazenamento na NVS. O padrão Template Method.
3. **Drivers dos chips** — apenas os registradores e as fórmulas do datasheet. Recebem
   `IRegisterDevice&` (não distinguem o barramento) ou `IUartPort&`.

Qual chip é compilado é decidido por `SensorSelection.h`.

---

## Interfaces e estruturas de dados

**Arquivo:** `sensors/SensorInterface.h`

### Estruturas

| Estrutura | Campos |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s (rolagem + asa direita para baixo, arfagem + nariz para cima, guinada + nariz para a direita); `accelX/Y/Z` g (eixos do avião: X para o nariz, Y para a esquerda, Z para cima); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, a integral do giroscópio) °; `temperature` °C; `timestamp` µs |
| `BarometerData` | `pressure` Pa; `temperature` °C; `altitude` m **em relação ao ponto de calibração**; `verticalSpeed` m/s; `timestamp` µs |
| `MagData` | `magX/Y/Z` µT após a calibração hard-iron, nos eixos do avião; `headingDegrees` 0..360 (sem compensação de inclinação); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` m MSL; `groundSpeed` m/s; `heading` 0..360; `numSatellites`; `fixType` (0 nenhum, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` m; `timestamp` |

### `Sensor` (interface)

| Método | Descrição |
|---|---|
| `bool begin()` | Identifica e configura o chip; `true` — o sensor funciona |
| `bool isAvailable() const` | Conectado e respondendo **agora** |
| `void update()` | Chamar a cada ciclo; ele mesmo decide se é hora de ler |
| `const char* getSensorType() const` | Um nome para o registro |
| `void printStatus() const` | Uma linha de diagnóstico (comando `s` do console) |

### `ImuSensor : Sensor`

| Método | Descrição |
|---|---|
| `const ImuData& getImuData() const` | Os dados mais recentes |
| `void calibrate()` | Calibração do giroscópio (parado) + a verificação pré-voo |
| `void setYaw(float)` | Define o rumo (por exemplo, pela bússola na partida) |
| `virtual void calibrateOrientation()` | Calibração da montagem da placa (por padrão não é suportada) |
| `virtual const char* getPreflightProblem() const` | O problema da verificação pré-voo ou `nullptr` (por padrão `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (a altitude atual = 0),
`setSeaLevelPressure(Pa)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: girar por 15 s).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (há pelo menos uma posição 2D).

---

## `AirspeedSensor`

**Arquivo:** `sensors/airspeed/AirspeedSensor.h` · **Tipo:** interface · **Implementação:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
a pressão diferencial após o zeramento e o filtro, a velocidade indicada (ρ0 = 1,225), a
velocidade verdadeira (ρ a partir da pressão estática e da temperatura), a densidade. Métodos: `getAirspeedData()`,
`calibrateZero()` (reiniciar o zeramento), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Arquivo:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Herda de:** `AirspeedSensor` · **Status:** verificada em uma simulação em malha fechada com ruído, não testada em voo

Um tubo de Pitot feito em casa com **dois barômetros absolutos**: `total` — o
BMP581 dentro do tubo (pressão total), `stat` — o barômetro principal da fuselagem
(pressão estática). O guia de montagem está em [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#tubo-de-pitot-caseiro).

| Método | Descrição |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` do barômetro do tubo (o estático já está em funcionamento), inicia o zeramento |
| `update()` | uma nova amostra do tubo → diferencial − zero, filtro passa-baixa `PITOT_FILTER_TAU_S`; as primeiras `PITOT_ZERO_SAMPLES` amostras fazem a média do zero; a velocidade é `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | os dois barômetros estão vivos, o zero foi reunido, não há falha, a amostra é mais recente que `PITOT_STALE_US` |
| `hasFault()` | o diferencial fica abaixo de −`PITOT_NEGATIVE_FAULT_PA` por mais de `PITOT_NEGATIVE_FAULT_MS` (mangueiras, água) |
| `getZeroOffset()`, `printStatus()` | diagnóstico |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | fórmulas |

---

## namespace `SensorMounting`

**Arquivo:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
gira os eixos do chip em torno da vertical (chip para cima) até os eixos do avião (X para o nariz,
Y para a esquerda). `rotationCwDeg` é para onde aponta o eixo X do chip, no sentido horário visto de cima:

| Valor | bodyX | bodyY |
|---|---|---|
| 0 (e qualquer valor desconhecido) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Usado pela bússola (`MAG_ROTATION_CW_DEG`) e por uma IMU sem calibração da montagem.

---

## `SensorSelection.h`

**Arquivo:** `sensors/SensorSelection.h` · **Tipo:** configuração do pré-processador

O único lugar para trocar um sensor físico: um kit pronto em uma linha
(`SENSOR_KIT`) ou cada sensor separadamente. Qualquer escolha pode ser sobrescrita
por uma flag de compilação (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Kit `SENSOR_KIT` | IMU | Barômetro | Bússola | Velocidade do ar | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, o padrão) | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (fuselagem) | QMC6309 | BMP581 no tubo | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (fuselagem) | QMC6309 | BMP581 no tubo | M10 |
| `SENSOR_KIT_CUSTOM` (0) | definir todas as cinco macros abaixo | | | | |

| Macro de seleção | Opções |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — um BMP581 I2C 0x47 no tubo |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Todas as combinações compilam em todas as placas — `tools/build_matrix.sh`.

O resultado são aliases de tipos e fábricas de dispositivos:

| Nome | MPU6050 / ICM42688 etc. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (não definidas para NONE) |
| as novas IMUs | `LSM6DSV_Sensor` + I2C 0x6A (reserva 0x6B) ou SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) ou SPI |
| os novos barômetros | `SPL06_Sensor` + I2C 0x76 (0x77) ou SPI; `BMP581_Sensor` + I2C 0x46 (0x47) ou SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (não definidas para NONE) |
| `SelectedGps` | `UbloxM10_Gps` (não definido para NONE) |

O `main.cpp` envolve a criação da bússola, do GPS e do tubo em `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Arquivo:** `sensors/imu/ImuOrientation.h` · **Depende de:** `SensorMounting`, `Preferences` (NVS)

A matriz de rotação `R` dos eixos do chip para os eixos do avião: `body = R · chip`; as linhas de
`R` são os eixos do avião expressos nos eixos do chip.

| Método | Descrição |
|---|---|
| `ImuOrientation()` | A identidade (equivale a `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | Uma rotação em torno da vertical em passos de 90°, com a placa e o chip para cima |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Calibração a partir de três poses (a leitura de “cima” do acelerômetro nos eixos do chip). `nullptr` — sucesso; caso contrário, o motivo da recusa |
| `void apply(const float chip[3], float body[3]) const` | Aplica a rotação |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | O ângulo entre o “cima” medido e o eixo Z do avião (180° se o vetor for nulo) |
| `bool load(const char* ns)` | Carrega da NVS; rejeita uma tripla não ortonormal ou levógira |
| `void save(const char* ns) const` | Salva na NVS |
| `void describe(Print&) const` | “nariz = +Y do chip, cima = +Z do chip” (com um ângulo se um eixo não coincidir com um eixo do chip dentro de ±14°) |

O algoritmo de `fromPoses`: Z = norm(level); X₁ = a parte de noseUp ⟂ Z; Y = a parte de
rightWingDown ⟂ Z, X₂ = Y × Z; X = norm(X₁ + X₂), Y = Z × X. As recusas:

| Condição | Mensagem |
|---|---|
| vetor nulo | “sem leituras do acelerômetro” |
| a inclinação do passo 2 ou 3 fora de 20..80° | “o passo N exige uma inclinação de 30-60°” |
| cos(X₁, X₂) < −0,5 | “os passos 2 e 3 se contradizem…” (o nariz foi abaixado ou foi usada a asa errada) |
| cos(X₁, X₂) < 0,9 (≈25°) | “…foram inclinados os eixos errados…” |

---

## `AttitudeEstimator`

**Arquivo:** `sensors/imu/AttitudeEstimator.h`

Um filtro complementar de rolagem/arfagem e uma integral de guinada, independente do chip.

| Método | Descrição |
|---|---|
| `void reset()` | O próximo `update()` começa de imediato pelo ângulo do acelerômetro |
| `void setYaw(float deg)` | Define o rumo (reduzido a ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | A aceleração em g nos eixos do avião, as velocidades em °/s |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0,1 s com 2 ms).
A primeira chamada após `reset()` dá de imediato os ângulos do acelerômetro; com `dt ≤ 0` ou maior que 0,1 s
o passo é ignorado (uma pausa, um barramento travado).

---

## `ImuSensorBase`

**Arquivo:** `sensors/imu/ImuSensorBase.h` · **Herda de:** `ImuSensor` · **Tipo:** abstrata

`RawImuSample` — uma amostra em unidades do ADC nos eixos do chip:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

O fluxo `update()` → `process()`:

```
amostras brutas − deslocamentos (ADC) → escala (g, °/s) → ImuOrientation (eixos do avião)
→ sinais aeronáuticos (gyroY, gyroZ com o sinal trocado) → AttitudeEstimator
```

| Método | Descrição |
|---|---|
| `bool isAvailable() const` | `begin()` teve sucesso e há menos de 50 erros de leitura seguidos |
| `void update()` | Uma leitura; em caso de erro os dados não mudam e os contadores crescem |
| `void calibrate()` | 200 amostras × 10 ms: o deslocamento do giroscópio, o ruído, o “cima”; sem calibração da montagem — o horizonte = a posição atual. Depois, a verificação pré-voo |
| `void calibrateOrientation()` | Três poses (`capturePose`: parado por ~1 s, a pose difere das anteriores em ≥ 20°, tempo limite de 30 s), `ImuOrientation::fromPoses`, gravação na NVS. Elimina os problemas de montagem da verificação pré-voo |
| `const char* getPreflightProblem() const` | O texto do problema ou `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

A API protegida para os drivers:

| Método | Descrição |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Cada driver tem o seu próprio namespace da NVS |
| `virtual bool readSample(RawImuSample&) = 0` | Uma amostra; `false` — o chip não respondeu |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | As escalas |
| `virtual float temperatureC(int16_t raw) const = 0` | A fórmula da temperatura |
| `void setAvailable(bool)` | O resultado de `begin()`; com `true` carrega a montagem da NVS ou de `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Refinar o nome após a identificação |

A verificação pré-voo (`runPreflightCheck`), em ordem:

| Problema | Condição |
|---|---|
| `NotResponding` | menos da metade das amostras de calibração foi lida |
| `Moved` | o ruído do giroscópio > 0,5 °/s |
| `NotOneG` | \|a\| difere de 1g em mais de 0,2g |
| `MountingMismatch` | (a montagem está calibrada) o “cima” está a mais de 45° do salvo |
| `NotChipUp` | (não calibrada) a placa não está deitada com o chip para cima (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Arquivo:** `sensors/imu/MPU6050_Sensor.h` · **Herda de:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Status:** na bancada (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 e clones (placas GY-521), I2C 0x68/0x69 ou
SPI. O chip é identificado por `WHO_AM_I` (0x68 — MPU6050; caso contrário, a família 6500).

- `begin()`: WHO_AM_I (sem resposta → indisponível), o nome conforme o ID, reset, saída
  do repouso (PLL), ±2000 °/s, ±16 g, DLPF ~41 Hz, 1 kHz; o 6500 tem um filtro
  passa-baixa do acelerômetro separado, `ACCEL_CONFIG2`.
- `readSample()`: 14 bytes a partir de `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Escalas: 2048 LSB/g, 16,4 LSB/(°/s).
- Temperatura: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Arquivo:** `sensors/imu/ICM42688_Sensor.h` · **Herda de:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Status:** não verificado no hardware

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI de 8 MHz, sem
  byte fictício.
- `begin()`: banco 0, reset por software, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/s / ±16 g, 1 kHz, filtro UI de 50 Hz.
- `readSample()`: 14 bytes a partir de `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Temperatura: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Arquivo:** `sensors/imu/LSM6DSV_Sensor.h` · **Herda de:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Status:** não verificado no hardware

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Os registradores foram conferidos com o `lsm6dsv-pid` da ST e com o ArduPilot.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (bit 0 de CTRL3) e uma espera;
  o 32X é distinguido pelo bit de variante em CTRL8 (ele tem o seu próprio código para ±16 g); BDU + autoincremento,
  ±2000 °/s com LPF1, ±16 g com LPF2, 960 Hz de alto desempenho.
- `readSample()`: 14 bytes a partir de 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Escalas: 1000/0,488 LSB/g, 1000/70 LSB/(°/s); temperatura `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI modo 0, sem byte fictício.

## `ICM45686_Sensor`

**Arquivo:** `sensors/imu/ICM45686_Sensor.h` · **Herda de:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Status:** não verificado no hardware

ICM-45686 (TDK). Os registradores foram conferidos com o driver da TDK, o Zephyr e o ArduPilot.

- `begin()`: reset por `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/s e ±16 g
  a 1,6 kHz (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); o filtro
  passa-baixa ODR/32 — leitura-modificação-escrita dos registradores **indiretos** IPREG
  (0xA4AC, 0xA583) pela janela 0x7C..0x7E; 45 ms para o giroscópio iniciar.
- `readSample()`: 14 bytes a partir de 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Escalas: 2048 LSB/g, 16,4 LSB/(°/s); temperatura `raw/132.48 + 25`.

---

## `BarometerBase`

**Arquivo:** `sensors/baro/BarometerBase.h` · **Herda de:** `BarometerSensor` · **Tipo:** abstrata

| Método | Descrição |
|---|---|
| `bool isAvailable() const` | `begin()` teve sucesso e há menos de 100 erros seguidos |
| `void update()` | No máximo a cada `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → altitude e velocidade vertical |
| `void calibrateAltitude()` | 20 amostras × 50 ms: a altitude absoluta média = a base; zera a altitude e a velocidade |
| `void setSeaLevelPressure(Pa)` | P₀ (101325 por padrão) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

A API protegida: o construtor `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Fórmulas: `h = 44330 · (1 − (P/P₀)^0.1903) − base`; a velocidade vertical é a
derivada da altitude sobre as amostras **realmente novas**, por um filtro passa-baixa com τ = 0,5 s
(com `dt` fora de `(0, 0.5 s)`, a velocidade não é atualizada). Ler apenas amostras novas
elimina o ruído “em degraus” (0 m/s intercalado com saltos de Δh/2 ms).

---

## `BMP388_Sensor`

**Arquivo:** `sensors/baro/BMP388_Sensor.h` · **Herda de:** `BarometerBase` · **Status:** na bancada (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI de 8 MHz,
  **1 byte fictício** (datasheet §5.3.2).
- `begin()`: ID do chip `0x50`, reset por software, 21 bytes de coeficientes da NVM a partir de
  `0x31` (escalas §9.1), OSR ×8/×1, ODR de 50 Hz, IIR 3, modo normal.
- `isNewSampleReady()`: o sinalizador `drdy_press` (bit 0x20) do registrador STATUS; consultado
  a cada 5 ms.
- `readSample()`: 6 bytes a partir de `0x04`; compensação da Bosch §9.3 (double):
  primeiro a temperatura (`tLin`), depois a pressão.

---

## `BME280_Sensor`

**Arquivo:** `sensors/baro/BME280_Sensor.h` · **Herda de:** `BarometerBase` · **Status:** não verificado no hardware

BME280 (ID 0x60) e BMP280 (ID 0x58), I2C 0x76/0x77 ou SPI sem byte
fictício. A umidade não é lida.

- `begin()`: ID do chip, reset, 24 bytes de calibração a partir de `0x88`, `CTRL_HUM` (somente
  BME280, escrito **antes** de `CTRL_MEAS`), `CONFIG` = IIR 4 + 0,5 ms, `CTRL_MEAS`
  = T×2, P×8, normal.
- Não há sinalizador de prontidão — `ready = true`, consulta a cada 25 ms.
- Compensação — as fórmulas da Bosch §8.1 (double); proteção contra divisão por zero.

---

## `SPL06_Sensor`

**Arquivo:** `sensors/baro/SPL06_Sensor.h` · **Herda de:** `BarometerBase` · **Status:** não verificado no hardware

SPL06-001 (Goertek). As fórmulas são do datasheet §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 é o SPA06, com outro conjunto de coeficientes —
  é rejeitado); reset, espera por `COEF_RDY | SENSOR_RDY`; 18 bytes de coeficientes
  (campos com sinal de 12/20/16 bits); a fonte da temperatura — por `COEF_SRCE`;
  pressão 16× (32 Hz), temperatura 1×, modo contínuo.
- `isNewSampleReady()` — o bit `PRS_RDY`; `readSample()` — amostras de 24 bits
  big-endian, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Arquivo:** `sensors/baro/BMP581_Sensor.h` · **Herda de:** `BarometerBase` · **Status:** não verificado no hardware

BMP581 (Bosch). A sequência segue a BMP5_SensorAPI oficial.

- `begin()`: uma leitura fictícia (para o SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  reset por software, `INT_STATUS` POR e `STATUS` com a NVM pronta e sem erros;
  standby → OSR (pressão 16×, temperatura 2×), IIR, DRDY; uma verificação de que
  o ODR é viável (`OSR_EFF`); modo contínuo.
- Uma amostra — por DRDY, ou a cada 40 ms se o sinalizador se perder; temperatura
  `int24/65536`, pressão `uint24/64`.
- Duas instâncias na compilação com o tubo: o barômetro principal e o `PITOT-BMP581`.

---

## `MagnetometerBase`

**Arquivo:** `sensors/mag/MagnetometerBase.h` · **Herda de:** `MagnetometerSensor` · **Tipo:** abstrata

| Método | Descrição |
|---|---|
| `bool isAvailable() const` | `begin()` teve sucesso e há menos de 25 erros seguidos |
| `void update()` | 50 Hz: `readRaw()` → subtrair os deslocamentos → escala → rotação por `MAG_ROTATION_CW_DEG` → rumo `atan2(Y, X)` em 0..360 |
| `void calibrate()` | 15 s de rotação: deslocamento = (min + max)/2 por eixo (hard-iron), salvo na NVS. Se não houve nem uma leitura bem-sucedida, a calibração é rejeitada e a anterior na NVS não é tocada |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

A API protegida: o construtor `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (com `true` carrega a calibração da NVS).

O rumo sem compensação de inclinação: está correto enquanto o avião estiver quase nivelado. Nariz para o
norte → o campo ao longo de +X → 0°; nariz para o leste → 90°.

---

## `QMC5883P_Sensor`

**Arquivo:** `sensors/mag/QMC5883P_Sensor.h` · **Herda de:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Status:** na bancada

`DEFAULT_ADDRESS = 0x2C`. `begin()`: ID do chip `0x80` (registrador 0x00), reset por
software, sinais dos eixos `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 G),
`CONTROL1 = 0xCD` (normal, 200 Hz, OSR 8/8). Os dados — 6 bytes a partir de `0x01`,
little-endian. 37,5 LSB/µT.

---

## `QMC5883L_Sensor`

**Arquivo:** `sensors/mag/QMC5883L_Sensor.h` · **Herda de:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Status:** não verificado no hardware

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (o chip não tem um ID confiável),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Hz, ±8 G, OSR 512).
Os dados — 6 bytes a partir de `0x00`, little-endian. 30 LSB/µT. Os registradores **não são compatíveis**
com o QMC5883P.

---

## `QMC6309_Sensor`

**Arquivo:** `sensors/mag/QMC6309_Sensor.h` · **Herda de:** `MagnetometerBase` · **NVS:** `qmc6309` · **Status:** não verificado no hardware

QMC6309 (QST), I2C 0x7C — um endereço fora da faixa usual 0x08..0x77
(a varredura de barramentos do console `b` vai até 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; reset (CTRL2 0x80 → 0x00), espera por
  `NVM_RDY | NVM_LOAD_DONE`; ±8 G, 200 Hz, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian a partir de 0x01; 40,96 LSB/µT.

---

## `UbloxM10_Gps`

**Arquivo:** `sensors/gps/UbloxM10_Gps.h` · **Herda de:** `GpsSensor` · **Depende de:** `IUartPort`, `Config` · **Status:** não conectado na bancada

Um u-blox M10 por UART, protocolo UBX. Apenas o **NAV-PVT** é analisado (classe 0x01,
id 0x07, 92 bytes).

| Método | Descrição |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 baud → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 baud → CFG-VALSET: 10 Hz, NAV-PVT na UART1, UBX ligado, NMEA desligado. Sem o pino TX (`PIN_GPS_TX < 0`) apenas escuta a 9600. Sempre `true` (não há ACK) |
| `bool isAvailable() const` | Houve um NAV-PVT válido e o último não é mais antigo que `GPS_TIMEOUT_US` |
| `void update()` | Alimentar o analisador com tudo o que há na UART |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = disponível e `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` mostra `available` conforme `isAvailable()` (levando em conta o tempo limite) |

O analisador é uma máquina de estados byte a byte `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, com o checksum Fletcher-8 sobre class+id+len+payload.
Um comprimento > 512 é uma perda de sincronismo, e a busca recomeça. Os campos do NAV-PVT são analisados assim: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, mm), `hAcc`/`vAcc`
(40/44, mm), `gSpeed` (60, mm/s), `headMot` (64, ×1e−5 °, reduzido a 0..360).

O `ValsetBuilder` aninhado é o payload do UBX-CFG-VALSET (version 0, layer RAM, pares
de chave U4 LE — valor de 1/2/4 bytes LE, um buffer de 64 bytes).
