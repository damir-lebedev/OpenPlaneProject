# AUTOPILOT — モード、ナビゲーション、スイッチ

> 🌐 このページは[ロシア語の原文](../../../reference/autopilot.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

オートパイロットは、パイロットのスティック、スイッチ／ノブ（`PilotInputs`）、センサーを受け取り、**最終的な舵面コマンド**（`getCommand()`）とモードのスロットル（`applyThrottle()`）を出力します。必要なセンサーがない場合、モードは落ちることなく安全に動作します（舵面はパイロットのものになるか、ニュートラルになる）。各モードがパイロットにとって何をするかは [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md) にあります。これまでの飛行で使ったのは手動モードだけで、オートパイロットはテストベンチ、テスト、閉ループのシミュレーション（`test/native/test_sim`）で確認しています。

---

## `AutopilotMode`、`Feature`、`Knob`

**ファイル：** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t`（スコープなし。数値コードは `/api/setmode` と `/api/status` の
JSON、および `MavlinkModes` で使われる）：

| 値 | コード | 略称（OLED） | 概要 |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | 舵面 = スティック |
| `MODE_STABILIZE` | 1 | STAB | スティックがロール／ピッチの角度 |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | パイロットのスロットルを契機とする離陸プログラム |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE ＋ エレベーターによる高度保持 |
| `MODE_ACRO` | 4 | ACRO | スティックが角速度 |
| `MODE_CRUISE` | 5 | CRZ | 針路 ＋ 高度 ＋ 自動スロットル |
| `MODE_LOITER` | 6 | LOIT | 有効にした地点の上空で旋回 |
| `MODE_RTH` | 7 | RTH | ホームへ帰還し、ホーム上空で旋回 |
| `MODE_LAUNCH` | 8 | LNCH | 手投げ発進 |
| `MODE_AUTO_LAND` | 9 | LAND | 滑空 ＋ フレア |
| `MODE_SOARING` | 10 | SOAR | モーターなしでサーマルを利用 |
| `MODE_RESCUE` | 11 | RESQ | 翼を水平、機首上げ、スロットル |
| `MODE_COUNT` | 12 | | 境界（`setMode()` は ≥ を無視する） |

`enum class Feature : uint8_t`：スイッチの機能。`FLAPS`、`AIRBRAKE`、
`AUTO_TRIM`、`TURN_COORDINATION`、`MOTOR_KILL`、`BEEPER`、`PAYLOAD_DROP`、
`GEOFENCE`、`HOME_RESET`、`CAMERA_STAB`、`COUNT`。

`enum class Knob : uint8_t`：ノブ。`STAB_GAIN`、`MAX_BANK`、
`CRUISE_SPEED`、`FLAPS`、`CAMERA_TILT`、`RATES`、`LOITER_RADIUS`、`COUNT`。

`namespace AutopilotNames`：`mode()`、`modeShort()`（5 文字以内）、
`feature()`、`knob()`。ログ、OLED、ダッシュボード、MAVLink 向けの名前です。

### `PilotInputs`

1 周期分のスイッチとノブの状態です。

| メンバー | 説明 |
|---|---|
| `bool has(Feature) const` | その機能がオン |
| `float knob(Knob) const` | ノブの位置 −1…+1 |
| `bool isBound(Knob) const` | ノブが割り当てテーブルにある |
| `float knobValue(Knob, min, default, max) const` | 単位付きの値：中央が `default`、両端が `min`/`max`。割り当てがなければ `default` |

---

## `Binding`、`Bind`、`BindingCheck`

**ファイル：** `autopilot/ControlBinding.h` · テーブル：`config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`、
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`。テーブルの各行は `namespace Bind` のファクトリです（すべて `constexpr`）：

| ファクトリ | 意味 |
|---|---|
| `modes(ch, up, middle, down)`、`modes(ch, up, down)` | モード選択スイッチ（ゾーンは `PilotSwitches::zoneOf` による） |
| `mode(ch, m)` | チャンネルが `SWITCH_ON_US` 以上の間、上に重ねるモード |
| `feature(ch, f)` | チャンネルが `SWITCH_ON_US` 以上の間、有効になる機能 |
| `knob(ch, k)` | ノブ。`(us − 1500) / 500` を ±1 に制限 |

`namespace BindingCheck`：再帰的な `constexpr` 関数群（ESP32 のコアは C++11 でビルドされる）。
`channelIsFree`、`channelsFree`、`channelsUnique`、`modeSwitchCount`、
`atMostOneModeSwitch`。`Controls.h` の `static_assert` で使われます。

---

## `PilotSwitches`

**ファイル：** `autopilot/PilotSwitches.h` · **依存先：** `Autopilot*`、`RcChannelState`、割り当てテーブル

| メソッド | 説明 |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | 独自のテーブル（テスト、シミュレーション用） |
| `explicit PilotSwitches(Autopilot* = nullptr)` | `Controls::BINDINGS` テーブル |
| `void update(const RcChannelState&)` | `PilotInputs` を集めて `autopilot->setInputs()` に渡す。`setMode()` は**スイッチの結果が変わったときだけ**呼ぶ（ダッシュボードや地上局から設定したモードを毎周期上書きしない）。`FlightController` はリンクが生きているときだけ呼び出す |
| `void printBindings() const` | 電源投入時に Serial へ割り当てを出力：`SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`、`"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 ゾーン：< 1500 / ≥ 1500。3 ゾーン：< 1250 / < 1750 / ≥ 1750 |
| `getInputs()`、`binding(i)` | テレメトリとテスト用 |

`Bind::mode` は `Bind::modes` より優先されます。オンになっている `Bind::mode` が複数ある場合は、上の行が勝ちます。

---

## `Autopilot`

**ファイル：** `autopilot/Autopilot.h` · **依存先：** `PidController`、`Navigation`、`AltitudeSpeedController`、`LaunchController`、`SoaringController`、`AutoTrim`、各センサー（すべて null 可）

### ライフサイクル

| メソッド | 説明 |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | ロール／ピッチの PID：Kp 5、Ki 0.5、Kd 0.5、出力 ±500 µs |
| `bool begin()` | トリムを読み込む。IMU か気圧計がなければ `false` とメッセージ |
| `void setInputs(const PilotInputs&)` | この周期のスイッチとノブ（`update` の前に呼ぶ） |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | 周期ごとに 1 回：センサー（常に）→ ナビゲーションとホーム → 地上でのトリム保存 → failsafe → ジオフェンス → モード → 旋回の協調 → オートトリム |
| `ControlCommand getCommand() const` | 最終的な舵面コマンド（roll/pitch/yaw、µs） |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | モードのスロットル：`PILOT` はパイロットのもの、`AUTO` は自前、`AT_LEAST` は自前の値以上（自動離陸）。ARM と `MOTOR_KILL` は `FlightController` が扱う |

### モード

| メソッド | 説明 |
|---|---|
| `void setMode(AutopilotMode)` | 同じモード、または ≥ `MODE_COUNT` なら何もしない。そうでなければ PID とステートマシンをリセットし、目標 = 現在の針路と高度、旋回円の中心 = 現在地（GPS があれば）、RTH は帰還高度 |
| `getMode()`、`getModeName()` | 名前：リンク喪失時は `FAILSAFE_GLIDE` / `FAILSAFE_RTH`、それ以外はモード名 |
| `isFailsafeActive()`、`isFailsafeGliding()`、`isFailsafeReturning()` | モードの上に重なる failsafe |
| `isAutoThrottle()`、`getThrottleCorrection()` | モードのスロットル（%、ログとダッシュボード用） |
| `getLaunchState()`、`getSoaringState()` | LAUNCH と SOARING のステートマシン |

### 出力と診断

| メソッド | 説明 |
|---|---|
| `getRollCorrection()`、`getPitchCorrection()`、`getYawCorrection()` | コマンド − スティック（µs） |
| `getDesiredRoll()`、`getDesiredPitch()`、`getTargetAltitude()` | 目標値 |
| `const NavStatus& getNavStatus()` | GPS、ホーム、位置、ホームまでの距離と方位、針路と目標針路、ナビゲーション用の速度、ジオフェンス、失速 |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | 気圧計による高度（電源投入地点からの m） |
| `getInputs()`、`getAutoTrim()` | テレメトリ用 |
| `getImuSensor()` … `getAirspeedSensor()` | 各センサー（`nullptr` の場合あり） |
| `getRollPid()`、`getPitchPid()`、`setPIDGains(...)` | PID（ダッシュボード、MAVLink パラメータ） |

### 内部の仕組み

- `stabilize()`：角度に対する PID で、D 項はジャイロから取り、`Knob::STAB_GAIN` を掛ける。積分器は ARM していて誤差が `STAB_INTEGRATOR_ZONE_DEG` 未満のときだけ蓄積する。
  `stabilizeOrManual()` は IMU がなければ舵面をパイロットに任せ、
  `stabilizeOrNeutral()` は IMU がなければニュートラルにする（自動モード用）。
- `imuReady()` = IMU があり、使用可能で、飛行前チェックに問題がない。
- ナビゲーション用の速度：ピトー管 → GPS → `NAV_ASSUMED_SPEED_MS`。
- `looksLanded()`：気圧計で地面近くにあり、垂直速度がほぼなく、ピトー管／GPS のしきい値より遅い場合。このときだけトリムをフラッシュに書き込む。
- Failsafe：GPS とホームがあればモーターを使った RTH、なければ滑空。開始済みの RTH は
  GPS の短い喪失では中断しない。

---

## `Geo`、`Guidance`、`GeoPoint`

**ファイル：** `autopilot/Navigation.h`

メートル単位のローカルな「北／東」平面です（正距円筒図法。数 km なら誤差は 1 % の何分の一かにとどまります）。

| 関数 | 説明 |
|---|---|
| `Geo::wrap180`、`Geo::wrap360` | 角度の正規化 |
| `Geo::offsetNE(a, b, north, east)`、`distance(a, b)`、`bearing(a, b)` | オフセット、距離、方位 0..360 |
| `Geo::moved(a, north, east)` | オフセットを加えた点 |
| `Geo::fromGps(GpsData)` | GPS から `GeoPoint` を作る |
| `Guidance::rollForCourse(target, course, bankLimit)` | 針路誤差に対するバンク（`NAV_COURSE_GAIN`）、制限付き |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | 円周へ導くベクトル場の針路（`LOITER_CONVERGENCE`） |
| `Guidance::orbitBankDeg(speed, radius)` | 旋回円の先行バンク：atan(V²/(g·R)) |

## `AltitudeSpeedController`

**ファイル：** `autopilot/AltitudeSpeedController.h`：TECS-lite。

| メソッド | 説明 |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | 目標の垂直速度 = `NAV_ALT_GAIN`·誤差（≤ `NAV_MAX_CLIMB/SINK`）。ピッチ = 先行項 asin(Vz/V) ＋ Vz 誤差に対する PI で、`NAV_MAX_CLIMB/DIVE_PITCH_DEG` の範囲内 |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | ピトー管があれば `cruisePct` を中心に対気速度で PI 制御、なければ `cruisePct`。さらに必要な上昇分として `THROTTLE_PER_CLIMB_PCT` を加える |
| `reset()`、`getWantedClimb()` | |

## `LaunchController`

**ファイル：** `autopilot/LaunchController.h`

`State`：`IDLE → READY`（スロットルを上げた）`→ THROWN`（過負荷 > `LAUNCH_ACCEL_G` が
`LAUNCH_ACCEL_TIME_MS` より長く続く）`→ CLIMB`（`LAUNCH_MOTOR_DELAY_MS` 後：モーター始動、ピッチ `LAUNCH_CLIMB_PITCH_DEG`）`→ DONE`（`LAUNCH_CLIMB_MS` または
`LAUNCH_ALTITUDE_M`）。投げる前にスティックを動かすとキャンセル。メソッド：
`update(...)`、`reset()`、`getState()`、`motorOn()`、`pitchTargetDeg()`、`stateName()`。

## `SoaringController`

**ファイル：** `autopilot/SoaringController.h`

`State`：`GLIDE ⇄ THERMAL`（バリオメーター > `SOAR_THERMAL_CLIMB_MS` が
`SOAR_THERMAL_CONFIRM_MS` より長く続く／`SOAR_EXIT_WINDOW_MS` の平均が
`SOAR_EXIT_CLIMB_MS` 未満）、`→ MOTOR_CLIMB`（`SOAR_MIN_ALTITUDE_M` を下回ったら
`SOAR_MAX_ALTITUDE_M` まで）、`→ RETURN`（`SOAR_MAX_DISTANCE_M` を超えたらその 70 % まで）。メソッド：`update(climb, alt, distHome, dt, now)`、`reset(now)`、
`getState()`、`motorOn()`、`getAverageClimb()`、`stateName()`。

## `AutoTrim`

**ファイル：** `autopilot/AutoTrim.h` · 保存先：`Preferences`（NVS／STM32 のフラッシュ）、名前空間 `"autotrim"`

| メソッド | 説明 |
|---|---|
| `void load()` | NVS からトリムを読み込む（なければ 0） |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += コマンド · `AUTOTRIM_RATE` · dt、上限は ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | 変更があれば書き込む（地上での DISARM 後に `Autopilot` が呼ぶ） |
| `reset()`、`getRoll()`、`getPitch()` | |

---

## `PidController`

**ファイル：** `autopilot/PidController.h` · **依存先：** `Config`（公称の `dt`）

| メソッド | 説明 |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`、`getKp/Ki/Kd()` | ゲイン |
| `setLimits(minOut, maxOut)` | 出力の制限（既定は ±500） |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | 出力を `[min, max]` に制限して返す |
| `void reset()` | 積分器をゼロにし、`dt` は「今」から数える |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (センサーから得た速度！)
out = constrain(P + I + D, min, max)
```

D 項は誤差の微分ではなく、測定量の変化率（ジャイロ °/s）から求めます。微分ノイズがなく、目標値を変えたときにも跳ねません。`dt` は `micros()` から求め、`reset()` 後の最初の呼び出しや 0.1 秒を超える中断のあとは、公称の `LOOP_PERIOD_MS` を使います。
