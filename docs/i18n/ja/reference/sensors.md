# SENSORS — センサー

> 🌐 このページは[ロシア語の原文](../../../reference/sensors.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。

[← リファレンス](README.md)

センサーは3つの階層で構成されています。

1. **カテゴリのインターフェース**（`SensorInterface.h`）— `Autopilot`、
   `ArmingManager`、テレメトリから見える部分です。
2. **カテゴリの基底クラス**（`ImuSensorBase`、`BarometerBase`、
   `MagnetometerBase`）— 共通部分のすべて、つまり較正、フィルタ、軸の回転、符号、エラーの計数、NVS への保存を担います。Template Method パターンです。
3. **チップのドライバ** — データシートのレジスタと数式だけです。
   `IRegisterDevice&`（バスは区別しません）または `IUartPort&` を受け取ります。

どのチップをコンパイルするかは `SensorSelection.h` が決めます。

---

## インターフェースとデータ構造

**ファイル:** `sensors/SensorInterface.h`

### 構造体

| 構造体 | フィールド |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/s（ロール + 右翼が下、ピッチ + 機首が上、ヨー + 機首が右）、`accelX/Y/Z` g（機体の軸: X は機首方向、Y は左、Z は上）、`roll`（−180..180）、`pitch`（−90..90）、`yaw`（−180..180。ジャイロの積分値）°、`temperature` °C、`timestamp` µs |
| `BarometerData` | `pressure` Pa、`temperature` °C、`altitude` m（**較正点を基準とした値**）、`verticalSpeed` m/s、`timestamp` µs |
| `MagData` | hard-iron 較正後の、機体の軸での `magX/Y/Z` µT、`headingDegrees` 0..360（傾きの補正なし）、`timestamp` |
| `GpsData` | `latitude`、`longitude`（double、°）、`altitude` m MSL、`groundSpeed` m/s、`heading` 0..360、`numSatellites`、`fixType`（0 なし、2 は 2D、3 は 3D）、`horizontalAccuracy`、`verticalAccuracy` m、`timestamp` |

### `Sensor`（インターフェース）

| メソッド | 説明 |
|---|---|
| `bool begin()` | チップを識別して設定します。`true` ならセンサーは動作しています |
| `bool isAvailable() const` | 接続されていて、**いま**応答している |
| `void update()` | 毎ティック呼びます。読み出すタイミングかどうかは自分で判断します |
| `const char* getSensorType() const` | ログ用の名前 |
| `void printStatus() const` | 診断の1行（コンソールの `s`） |

### `ImuSensor : Sensor`

| メソッド | 説明 |
|---|---|
| `const ImuData& getImuData() const` | 最新のデータ |
| `void calibrate()` | ジャイロの較正（静止状態で）+ 飛行前チェック |
| `void setYaw(float)` | 方位を設定します（たとえば起動時にコンパスから） |
| `virtual void calibrateOrientation()` | 基板の取り付けの較正（既定では未対応） |
| `virtual const char* getPreflightProblem() const` | 飛行前チェックで見つかった問題、または `nullptr`（既定は `nullptr`） |

### `BarometerSensor : Sensor`

`getBarometerData()`、`calibrateAltitude()`（現在の高度を 0 にする）、
`setSeaLevelPressure(Pa)`。

### `MagnetometerSensor : Sensor`

`getMagData()`、`calibrate()`（hard-iron: 15秒間回転させる）。

### `GpsSensor : Sensor`

`getGpsData()`、`hasFix()`（少なくとも 2D のフィックスがある）。

---

## `AirspeedSensor`

**ファイル:** `sensors/airspeed/AirspeedSensor.h` · **種別:** インターフェース · **実装:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
ゼロ合わせとフィルタ後の差圧、指示対気速度（ρ0 = 1.225）、真対気速度（ρ は静圧と温度から求めます）、空気密度です。メソッド: `getAirspeedData()`、
`calibrateZero()`（ゼロ合わせをやり直す）、`isZeroing()`。

## `PitotDualBaroAirspeed`

**ファイル:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **継承元:** `AirspeedSensor` · **状態:** ノイズ入りの閉ループシミュレーションで検証済み、飛行試験は未実施

**2つの絶対圧バロメーター**で作る自作のピトー管です。`total` は管の中の
BMP581（全圧）、`stat` は胴体のメインのバロメーター（静圧）です。製作ガイドは [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#自作ピトー管) にあります。

| メソッド | 説明 |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | 管のバロメーターの `begin()`（静圧側はすでに起動済み）、ゼロ合わせを開始します |
| `update()` | 管の新しいサンプル → 差圧 − ゼロ、ローパスフィルタ `PITOT_FILTER_TAU_S`。最初の `PITOT_ZERO_SAMPLES` 個のサンプルでゼロを平均します。速度は `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | 両方のバロメーターが生きていて、ゼロが取得済みで、故障がなく、サンプルが `PITOT_STALE_US` より新しい |
| `hasFault()` | 差圧が −`PITOT_NEGATIVE_FAULT_PA` を下回った状態が `PITOT_NEGATIVE_FAULT_MS` より長く続く（ホース、水） |
| `getZeroOffset()`、`printStatus()` | 診断 |
| `static speedFrom(Δp, ρ)`、`static densityOf(p, T)` | 数式 |

---

## namespace `SensorMounting`

**ファイル:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
チップの軸を鉛直軸（チップを上向き）まわりに回して機体の軸（X は機首方向、
Y は左）に変換します。`rotationCwDeg` は、チップの X 軸が向く方向を、真上から見て時計回りに表した角度です。

| 値 | bodyX | bodyY |
|---|---|---|
| 0（および未知の値すべて） | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

コンパス（`MAG_ROTATION_CW_DEG`）と、取り付けの較正をしていない IMU が使います。

---

## `SensorSelection.h`

**ファイル:** `sensors/SensorSelection.h` · **種別:** プリプロセッサによる構成

物理的なセンサーを切り替える唯一の場所です。1行で決まる既製のセット（`SENSOR_KIT`）か、センサーを個別に指定できます。どの選択もビルドフラグで上書きできます（`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`、
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`）。

| `SENSOR_KIT` のセット | IMU | バロメーター | コンパス | 対気速度 | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521`（1、既定） | MPU6500 | BMP388 I2C | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT`（2） | LSM6DSV | SPL06（胴体） | QMC6309 | 管内の BMP581 | M10 |
| `SENSOR_KIT_ICM45686_PITOT`（3） | ICM-45686 | SPL06（胴体） | QMC6309 | 管内の BMP581 | M10 |
| `SENSOR_KIT_CUSTOM`（0） | 下の5つのマクロをすべて指定 | | | | |

| 選択用マクロ | 選択肢 |
|---|---|
| `SENSOR_IMU` | `MPU6050`（1）、`ICM42688`（2、SPI）、`LSM6DSV`（3）、`LSM6DSV_SPI`（4）、`ICM45686`（5）、`ICM45686_SPI`（6） |
| `SENSOR_BARO` | `BME280`（1）、`BMP388`（2、SPI）、`BMP388_I2C`（3）、`SPL06`（4）、`SPL06_SPI`（5）、`BMP581`（6）、`BMP581_SPI`（7） |
| `SENSOR_MAG` | `NONE`（0）、`QMC5883P`（1）、`QMC5883L`（2）、`QMC6309`（3） |
| `SENSOR_AIRSPEED` | `NONE`（0）、`PITOT_BMP581`（1）— 管内の BMP581、I2C 0x47 |
| `SENSOR_GPS` | `NONE`（0）、`UBLOX_M10`（1） |

すべての組み合わせが、すべてのボードでビルドできます — `tools/build_matrix.sh`。

結果として得られるのは、型のエイリアスとデバイスのファクトリです。

| 名前 | MPU6050 / ICM42688 など |
|---|---|
| `SelectedImu`、`SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`、`SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI（CS は `PIN_SPI_CS_BARO`）/ `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`、`SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C（NONE では未定義） |
| 新しい IMU | `LSM6DSV_Sensor` + I2C 0x6A（予備は 0x6B）または SPI、`ICM45686_Sensor` + I2C 0x68（0x69）または SPI |
| 新しいバロメーター | `SPL06_Sensor` + I2C 0x76（0x77）または SPI、`BMP581_Sensor` + I2C 0x46（0x47）または SPI |
| `SelectedPitotBaro`、`SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47（NONE では未定義） |
| `SelectedGps` | `UbloxM10_Gps`（NONE では未定義） |

`main.cpp` は、コンパス、GPS、ピトー管の生成を `#if SENSOR_* != SENSOR_*_NONE` で囲みます。

---

## `ImuOrientation`

**ファイル:** `sensors/imu/ImuOrientation.h` · **依存先:** `SensorMounting`、`Preferences`（NVS）

チップの軸から機体の軸への回転行列 `R` です: `body = R · chip`。`R` の行は、チップの軸で表した機体の軸です。

| メソッド | 説明 |
|---|---|
| `ImuOrientation()` | 単位行列（`fromYawSteps(0)` と同等） |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | 鉛直軸まわりに 90° 刻みで回転します。基板はチップが上向き |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | 3つの姿勢からの較正（チップの軸での、加速度計の「上」の読み値）。`nullptr` なら成功、そうでなければ拒否の理由 |
| `void apply(const float chip[3], float body[3]) const` | 回転を適用します |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | 測定した「上」と機体の Z 軸のなす角（ベクトルがゼロなら 180°） |
| `bool load(const char* ns)` | NVS から読み込みます。正規直交でない組や左手系の組は拒否します |
| `void save(const char* ns) const` | NVS に保存します |
| `void describe(Print&) const` | 「機首 = チップの +Y、上 = チップの +Z」（軸がチップの軸と ±14° 以内で一致しない場合は角度付き） |

`fromPoses` のアルゴリズム: Z = norm(level)、X₁ = noseUp の ⟂ Z の成分、Y =
rightWingDown の ⟂ Z の成分、X₂ = Y × Z、X = norm(X₁ + X₂)、Y = Z × X。拒否されるケース:

| 条件 | メッセージ |
|---|---|
| ゼロベクトル | 「加速度計の読み値がありません」 |
| ステップ 2 または 3 の傾きが 20..80° の範囲外 | 「ステップ N では 30-60° の傾きが必要です」 |
| cos(X₁, X₂) < −0.5 | 「ステップ 2 と 3 が互いに矛盾しています…」（機首を下げた、または違う翼を使った） |
| cos(X₁, X₂) < 0.9（≈25°） | 「…傾けた軸が違います…」 |

---

## `AttitudeEstimator`

**ファイル:** `sensors/imu/AttitudeEstimator.h`

ロール/ピッチの相補フィルタとヨーの積分で、チップには依存しません。

| メソッド | 説明 |
|---|---|
| `void reset()` | 次の `update()` は、ただちに加速度計の角度から始まります |
| `void setYaw(float deg)` | 方位を設定します（±180 に正規化されます） |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | 機体の軸での加速度（g）、角速度（°/s） |
| `getRoll()`、`getPitch()`、`getYaw()` | ° |

`roll_acc = atan2(ay, az)`、`pitch_acc = atan2(ax, √(ay² + az²))`、
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc`（2 ms のとき τ ≈ 0.1秒）。
`reset()` の後の最初の呼び出しは、すぐに加速度計の角度を返します。`dt ≤ 0` または 0.1秒より大きいときは、そのステップを飛ばします（一時停止やバスのハング）。

---

## `ImuSensorBase`

**ファイル:** `sensors/imu/ImuSensorBase.h` · **継承元:** `ImuSensor` · **種別:** 抽象クラス

`RawImuSample` — チップの軸での、ADC 単位の1サンプルです:
`accelX/Y/Z`、`gyroX/Y/Z`、`temperature`（`int16_t`）。

`update()` → `process()` のパイプライン:

```
生のサンプル − オフセット（ADC）→ スケール（g、°/s）→ ImuOrientation（機体の軸）
→ 航空機の符号（gyroY、gyroZ は符号を反転）→ AttitudeEstimator
```

| メソッド | 説明 |
|---|---|
| `bool isAvailable() const` | `begin()` が成功し、読み出しエラーの連続が 50 回未満 |
| `void update()` | 1回の読み出し。エラーならデータは変わらず、カウンタが増えます |
| `void calibrate()` | 200 サンプル × 10 ms: ジャイロのオフセット、ノイズ、「上」。取り付けの較正がない場合は、水平 = 現在の姿勢です。その後、飛行前チェックを行います |
| `void calibrateOrientation()` | 3つの姿勢（`capturePose`: 約1秒の静止、前の姿勢と 20° 以上異なること、タイムアウト30秒）、`ImuOrientation::fromPoses`、NVS への保存。飛行前チェックの取り付けに関する問題を解消します |
| `const char* getPreflightProblem() const` | 問題のテキスト、または `nullptr` |
| `void setYaw(float)`、`getSensorType()`、`printStatus()` | |

ドライバ向けの protected API:

| メソッド | 説明 |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | ドライバごとに専用の NVS 名前空間を持ちます |
| `virtual bool readSample(RawImuSample&) = 0` | 1サンプル。`false` はチップが応答しなかったことを表します |
| `virtual float accelLsbPerG() const = 0`、`gyroLsbPerDps() const = 0` | スケール |
| `virtual float temperatureC(int16_t raw) const = 0` | 温度の計算式 |
| `void setAvailable(bool)` | `begin()` の結果。`true` なら、取り付けを NVS または `Config::IMU_ROTATION_CW_DEG` から読み込みます |
| `void setName(const char*)` | 識別後に名前を詳しくします |

飛行前チェック（`runPreflightCheck`）を、順に示します。

| 問題 | 条件 |
|---|---|
| `NotResponding` | 較正用サンプルの半分未満しか読めなかった |
| `Moved` | ジャイロのノイズが 0.5 °/s を超える |
| `NotOneG` | \|a\| が 1g から 0.2g を超えてずれている |
| `MountingMismatch` | （取り付けが較正済み）「上」が保存した値から 45° を超えて離れている |
| `NotChipUp` | （較正なし）基板がチップを上にして置かれていない（`z < 0.5g`） |

---

## `MPU6050_Sensor`

**ファイル:** `sensors/imu/MPU6050_Sensor.h` · **継承元:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **状態:** ベンチで動作確認（MPU6500）

MPU6050 / MPU6500 / MPU9250 / MPU9255 とそのクローン（GY-521 基板）で、I2C 0x68/0x69 または
SPI です。チップは `WHO_AM_I` で識別します（0x68 は MPU6050、それ以外は 6500 ファミリー）。

- `begin()`: WHO_AM_I（応答なし → 利用不可）、ID による名前の決定、リセット、スリープからの復帰（PLL）、±2000 °/s、±16 g、DLPF 約 41 Hz、1 kHz。6500 には、加速度計用に独立したローパスフィルタ `ACCEL_CONFIG2` があります。
- `readSample()`: `0x3B` から 14 バイト、ビッグエンディアン: accel XYZ、temp、gyro XYZ。
- スケール: 2048 LSB/g、16.4 LSB/(°/s)。
- 温度: MPU6050 は `raw/340 + 36.53`、MPU6500 は `raw/333.87 + 21`。

---

## `ICM42688_Sensor`

**ファイル:** `sensors/imu/ICM42688_Sensor.h` · **継承元:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **状態:** 実機では未確認

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz、ダミーバイトなし。
- `begin()`: バンク 0、ソフトウェアリセット、`WHO_AM_I == 0x47`、Low Noise、
  ±2000 °/s / ±16 g、1 kHz、UI フィルタ 50 Hz。
- `readSample()`: `0x1D` から 14 バイト、ビッグエンディアン: temp、accel XYZ、gyro XYZ。
- 温度: `raw/132.48 + 25`。

---

## `LSM6DSV_Sensor`

**ファイル:** `sensors/imu/LSM6DSV_Sensor.h` · **継承元:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **状態:** 実機では未確認

LSM6DSV / LSM6DSV16X / LSM6DSV32X（ST）。レジスタは ST の `lsm6dsv-pid` と ArduPilot に照らして確認してあります。

- `begin()`: `WHO_AM_I`（0x0F）= 0x70、`SW_RESET`（CTRL3 のビット 0）と待機。
  32X は CTRL8 のバリアントビットで見分けます（±16 g に専用のコードがあります）。BDU + 自動インクリメント、
  LPF1 付きの ±2000 °/s、LPF2 付きの ±16 g、960 Hz の高性能モード。
- `readSample()`: 0x20 から 14 バイト、リトルエンディアン: temp、gyro XYZ、accel XYZ。
- スケール: 1000/0.488 LSB/g、1000/70 LSB/(°/s)。温度は `raw/256 + 25`。
- `static spiDevice(bus, cs)` — SPI モード 0、ダミーバイトなし。

## `ICM45686_Sensor`

**ファイル:** `sensors/imu/ICM45686_Sensor.h` · **継承元:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **状態:** 実機では未確認

ICM-45686（TDK）。レジスタは TDK のドライバ、Zephyr、ArduPilot に照らして確認してあります。

- `begin()`: `REG_MISC2`（0x7F）によるリセット、`WHO_AM_I`（0x72）= 0xE9、±2000 °/s と ±16 g を
  1.6 kHz で（`GYRO/ACCEL_CONFIG0` = 0x15）、Low Noise（`PWR_MGMT0` = 0x0F）。ODR/32 のローパスフィルタは、**間接**レジスタ IPREG（0xA4AC、0xA583）の読み出し-変更-書き込みを、0x7C..0x7E のウィンドウ経由で行います。ジャイロの起動に 45 ms。
- `readSample()`: 0x00 から 14 バイト、リトルエンディアン: accel XYZ、gyro XYZ、temp。
- スケール: 2048 LSB/g、16.4 LSB/(°/s)。温度は `raw/132.48 + 25`。

---

## `BarometerBase`

**ファイル:** `sensors/baro/BarometerBase.h` · **継承元:** `BarometerSensor` · **種別:** 抽象クラス

| メソッド | 説明 |
|---|---|
| `bool isAvailable() const` | `begin()` が成功し、エラーの連続が 100 回未満 |
| `void update()` | `pollPeriodUs` より頻繁には実行しません: `isNewSampleReady()` → `readSample()` → 高度と垂直速度 |
| `void calibrateAltitude()` | 20 サンプル × 50 ms: 絶対高度の平均 = 基準。高度と速度をリセットします |
| `void setSeaLevelPressure(Pa)` | P₀（既定は 101325） |
| `getBarometerData()`、`getSensorType()`、`printStatus()` | |

protected API: コンストラクタ `(name, pollPeriodUs)`、
`virtual bool isNewSampleReady(bool& ready) = 0`、
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`、
`setAvailable(bool)`。

数式: `h = 44330 · (1 − (P/P₀)^0.1903) − base`。垂直速度は、高度を**実際に新しい**
サンプルについて微分し、τ = 0.5秒のローパスフィルタに通したものです（`dt` が `(0, 0.5 s)` の範囲外なら速度は更新しません）。新しいサンプルだけを読むことで、「階段状」のノイズ（0 m/s の合間に Δh/2 ms の跳びが混じる現象）がなくなります。

---

## `BMP388_Sensor`

**ファイル:** `sensors/baro/BMP388_Sensor.h` · **継承元:** `BarometerBase` · **状態:** ベンチで動作確認（I2C）

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 MHz、
  **ダミーバイト 1 個**（データシート §5.3.2）。
- `begin()`: チップ ID `0x50`、ソフトウェアリセット、`0x31` から NVM の係数 21 バイト（スケールは §9.1）、OSR ×8/×1、ODR 50 Hz、IIR 3、ノーマルモード。
- `isNewSampleReady()`: STATUS レジスタのフラグ `drdy_press`（ビット 0x20）。
  5 ms ごとにポーリングします。
- `readSample()`: `0x04` から 6 バイト。Bosch の補正 §9.3（double）:
  先に温度（`tLin`）、次に気圧。

---

## `BME280_Sensor`

**ファイル:** `sensors/baro/BME280_Sensor.h` · **継承元:** `BarometerBase` · **状態:** 実機では未確認

BME280（ID 0x60）と BMP280（ID 0x58）で、I2C 0x76/0x77 または SPI（ダミーバイトなし）です。湿度は読みません。

- `begin()`: チップ ID、リセット、`0x88` から較正データ 24 バイト、`CTRL_HUM`（BME280
  のみ。`CTRL_MEAS` の**前**に書き込む）、`CONFIG` = IIR 4 + 0.5 ms、`CTRL_MEAS`
  = T×2、P×8、ノーマル。
- 準備完了フラグはありません。`ready = true` とし、25 ms ごとにポーリングします。
- 補正は Bosch §8.1 の数式（double）。ゼロ除算への対策あり。

---

## `SPL06_Sensor`

**ファイル:** `sensors/baro/SPL06_Sensor.h` · **継承元:** `BarometerBase` · **状態:** 実機では未確認

SPL06-001（Goertek）。数式はデータシート §4.9 によります。

- `begin()`: `ID`（0x0D）= 0x10（0x11 は SPA06 で、係数の組が異なるため拒否します）、リセット、`COEF_RDY | SENSOR_RDY` の待機、係数 18 バイト（符号付きの 12/20/16 ビットのフィールド）、温度の取得元は `COEF_SRCE` で決定、気圧 16×（32 Hz）、温度 1×、連続モード。
- `isNewSampleReady()` は `PRS_RDY` ビット、`readSample()` は 24 ビットのビッグエンディアンのサンプルで、`kP = 253952`、`kT = 524288`。

## `BMP581_Sensor`

**ファイル:** `sensors/baro/BMP581_Sensor.h` · **継承元:** `BarometerBase` · **状態:** 実機では未確認

BMP581（Bosch）。手順は公式の BMP5_SensorAPI に従っています。

- `begin()`: ダミーの読み出し（SPI 向け）、`CHIP_ID`（0x01）= 0x50/0x51、ソフトウェアリセット、`INT_STATUS` の POR と `STATUS` の NVM 準備完了（エラーなし）、
  standby → OSR（気圧 16×、温度 2×）、IIR、DRDY、ODR が実現可能かの確認（`OSR_EFF`）、連続モード。
- サンプルは DRDY で、フラグが失われた場合は 40 ms ごとに読みます。温度は
  `int24/65536`、気圧は `uint24/64`。
- ピトー管付きのビルドでは、2つのインスタンスがあります: メインのバロメーターと `PITOT-BMP581`。

---

## `MagnetometerBase`

**ファイル:** `sensors/mag/MagnetometerBase.h` · **継承元:** `MagnetometerSensor` · **種別:** 抽象クラス

| メソッド | 説明 |
|---|---|
| `bool isAvailable() const` | `begin()` が成功し、エラーの連続が 25 回未満 |
| `void update()` | 50 Hz: `readRaw()` → オフセットを引く → スケール → `MAG_ROTATION_CW_DEG` で回転 → 方位 `atan2(Y, X)` を 0..360 に |
| `void calibrate()` | 15秒間の回転: オフセット = 各軸の (min + max)/2（hard-iron）、NVS に保存します。1回も読み出しに成功しなかった場合は較正を拒否し、NVS の以前の値には触れません |
| `getMagData()`、`getSensorType()`、`printStatus()` | |

protected API: コンストラクタ `(name, nvsNamespace)`、
`virtual bool readRaw(int16_t raw[3]) = 0`、`virtual float lsbPerMicroTesla() const = 0`、
`setAvailable(bool)`（`true` なら較正を NVS から読み込みます）。

傾きの補正がない方位: 機体がほぼ水平のうちは正しい値です。機首が北のとき、磁場は +X 方向 → 0°。機首が東のとき → 90°。

---

## `QMC5883P_Sensor`

**ファイル:** `sensors/mag/QMC5883P_Sensor.h` · **継承元:** `MagnetometerBase` · **NVS:** `qmc5883p` · **状態:** ベンチで動作確認

`DEFAULT_ADDRESS = 0x2C`。`begin()`: チップ ID `0x80`（レジスタ 0x00）、ソフトウェアリセット、軸の符号 `0x29 = 0x06`、`CONTROL2 = 0x08`（SET/RESET、±8 G）、
`CONTROL1 = 0xCD`（ノーマル、200 Hz、OSR 8/8）。データは `0x01` から 6 バイト、リトルエンディアン。37.5 LSB/µT。

---

## `QMC5883L_Sensor`

**ファイル:** `sensors/mag/QMC5883L_Sensor.h` · **継承元:** `MagnetometerBase` · **NVS:** `qmc5883l` · **状態:** 実機では未確認

`DEFAULT_ADDRESS = 0x0D`。`begin()`: `probe()`（このチップには信頼できる ID がありません）、
`SET/RESET = 0x01`、`CONTROL1 = 0x1D`（continuous、200 Hz、±8 G、OSR 512）。データは `0x00` から 6 バイト、リトルエンディアン。30 LSB/µT。レジスタは QMC5883P と
**互換性がありません**。

---

## `QMC6309_Sensor`

**ファイル:** `sensors/mag/QMC6309_Sensor.h` · **継承元:** `MagnetometerBase` · **NVS:** `qmc6309` · **状態:** 実機では未確認

QMC6309（QST）、I2C 0x7C — 通常の範囲 0x08..0x77 を外れたアドレスです（コンソールの `b` によるバスのスキャンは 0x7F まで行います）。

- `begin()`: `CHIP_ID`（0x00）= 0x90、リセット（CTRL2 を 0x80 → 0x00）、
  `NVM_RDY | NVM_LOAD_DONE` の待機、±8 G、200 Hz、set/reset、LPF 16、OSR 8、ノーマル。
- `readRaw()`: 0x01 から X/Y/Z をリトルエンディアンで。40.96 LSB/µT。

---

## `UbloxM10_Gps`

**ファイル:** `sensors/gps/UbloxM10_Gps.h` · **継承元:** `GpsSensor` · **依存先:** `IUartPort`、`Config` · **状態:** ベンチでは未接続

UART 接続の u-blox M10 で、プロトコルは UBX です。解析するのは **NAV-PVT**（クラス 0x01、
id 0x07、92 バイト）だけです。

| メソッド | 説明 |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 ボー → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 ボー → CFG-VALSET: 10 Hz、UART1 に NAV-PVT、UBX 有効、NMEA 無効。TX ピンがない場合（`PIN_GPS_TX < 0`）は 9600 で聞くだけです。常に `true`（ACK がないため） |
| `bool isAvailable() const` | 有効な NAV-PVT があり、最後のものが `GPS_TIMEOUT_US` より古くない |
| `void update()` | UART にあるものをすべてパーサーに渡します |
| `const GpsData& getGpsData() const`、`bool hasFix() const` | `hasFix` = 利用可能で `fixType ≥ 2` |
| `getSensorType()`、`printStatus()` | `printStatus()` は `isAvailable()` に基づいて `available` を表示します（タイムアウトを考慮） |

パーサーはバイト単位のステートマシン `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B` で、class+id+len+payload に対して Fletcher-8 チェックサムを計算します。長さが 512 を超えたら同期外れとみなし、探索をやり直します。NAV-PVT のフィールドは次のように解析します: `fixType`
（20）、`numSV`（23）、`lon`/`lat`（24/28、×1e−7）、`hMSL`（36、mm）、`hAcc`/`vAcc`
（40/44、mm）、`gSpeed`（60、mm/s）、`headMot`（64、×1e−5 °、0..360 に正規化）。

入れ子の `ValsetBuilder` は UBX-CFG-VALSET のペイロードです（version 0、layer RAM、
U4 LE のキーと 1/2/4 バイト LE の値のペア、64 バイトのバッファ）。
