# AUTOPILOT / feedback — フィードバックループ（土台）

> 🌐 このページは[ロシア語の原文](../../../reference/feedback.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

> ⚠️ **土台の段階で、ファームウェアには接続されていません**。`FlightController`、
> `Autopilot`、`main.cpp` のいずれもこれらのヘッダをインクルードしていません。検証は閉ループシミュレーション（`test/test_feedback`。PC上でもボード上でも実行）と、ネイティブのユニットテストで行っています。接続計画は
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#接続計画) にあります。

考え方はこうです。単一の速度に合わせて調整したゲインの角度PIDではなく、
**機体の応答**に閉じたレギュレータを使います。軸ごとのモデルは飛行中に学習し、失速からの保護と、センサー駆動の離陸・着陸フェーズを備えます。入力は
`FlightSnapshot` だけ、出力は `FeedbackOutput` だけです。

すべてのモジュールはヘッダのみで構成され、`FeedbackModules.h` が1行でまとめてインクルードします。

---

## namespace `FeedbackConfig`

**ファイル:** `autopilot/feedback/FeedbackConfig.h`

ループの定数をすべてまとめたものです（接続時に `Config.h` へ移します）。「прикидка」（「概算」）と記した値は、重さ約1 kg、翼幅1.2 mの機体を想定しています。
`[AXIS_COUNT]` の配列は軸で添字を付けます。

| グループ | 定数 |
|---|---|
| 全般 | `GRAVITY = 9.80665`、軸 `AXIS_ROLL = 0`、`AXIS_PITCH = 1`、`AXIS_YAW = 2`、`AXIS_COUNT = 3` |
| 速度 | `STALL_SPEED_MS = 8`、`REFERENCE_SPEED_MS = 14`、`ACCEL_FILTER_TAU_S = 0.3` |
| 空中/地上 | `AIRBORNE_HEIGHT_M = 3`、`AIRBORNE_CONFIRM_MS = 500`、`GROUND_STILL_MS = 2000`、`GROUND_ACCEL_TOLERANCE_G = 0.1` |
| レギュレータ | `ANGLE_GAIN = {4, 4, 2}` 1/s、`MAX_RATE_DPS = {120, 60, 30}`、`RATE_TAU_S = {0.15, 0.20, 0.30}`、`RATE_INTEGRAL_GAIN = {2, 2, 1}`、`MAX_DEFLECTION_US = {400, 400, 400}`、`DAMPING_COMPENSATION = 0.5` |
| 舵の効き | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs、`EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`、`EFFECTIVENESS_MAX = {30, 15, 6}`、`RESPONSE_DELAY_MS = 40`、`RLS_FORGETTING = 0.995`、`ESTIMATOR_PERIOD_MS = 20`、`ESTIMATOR_PREFILTER_HZ = 2`、`MIN_EXCITATION_US = 30` |
| 失速 | `DECEL_WARN_MS2 = 2`、`DECEL_CONFIRM_MS = 300`、`LOW_ENERGY_PITCH_DEG = 5`、`NOSE_DROP_RATE_DPS = 60`、`WING_DROP_RATE_DPS = 120`、`STALL_NOSE_UP_COMMAND_US = 50`、`LOW_EFFECTIVENESS_RATIO = 0.35`、`LOW_SPEED_MARGIN = 1.25`、`LOW_SPEED_EXIT_MARGIN = 1.5`、`LOW_ENERGY_THROTTLE_PERCENT = 80`、`LOW_ENERGY_MAX_PITCH_DEG = 5`、`STALL_THROTTLE_PERCENT = 100`、`STALL_MAX_PITCH_DEG = −5`、`STALL_MAX_BANK_DEG = 10`、`STALL_AILERON_LIMIT_US = 150`、`RECOVERY_HOLD_MS = 1000` |
| 離陸 | `TAKEOFF_HAND_LAUNCH = false`、`TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`、`TAKEOFF_THROTTLE_PERCENT = 100`、`LAUNCH_ACCEL_G = 1`、`LAUNCH_DETECT_MS = 50`、`ROTATE_SPEED_MS = 10`、`ROTATE_FALLBACK_MS = 1500`、`CLIMB_PITCH_DEG = 12`、`TAKEOFF_TARGET_ALTITUDE_M = 30`、`TAKEOFF_CLIMB_FALLBACK_MS = 10000`、`LAUNCH_TIMEOUT_MS = 8000`、`HEADING_HOLD_GAIN = 2` |
| 着陸 | `APPROACH_SINK_RATE_MS = 1`、`APPROACH_THROTTLE_PERCENT = 25`、`APPROACH_BASE_PITCH_DEG = −3`、`APPROACH_MIN_PITCH_DEG = −10`、`APPROACH_MAX_BANK_DEG = 20`、`GO_AROUND_THROTTLE_PERCENT = 80`、`SINK_TO_PITCH_GAIN = 4`、`FLARE_HEIGHT_M = 2`、`FLARE_SINK_RATE_MS = 0.3`、`FLARE_MAX_PITCH_DEG = 8`、`TOUCHDOWN_ACCEL_G = 0.5`、`TOUCHDOWN_HEIGHT_M = 0.3`、`TOUCHDOWN_STILL_MS = 500`、`TOUCHDOWN_STILL_RATE_DPS = 5`、`ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**ファイル:** `autopilot/feedback/FeedbackMath.h` · **依存:** `<math.h>`

| 関数 | 説明 |
|---|---|
| `float wrap180(float deg)` | 角度を `(−180, 180]` に収めます。方位350°と10°の差は−20°になります |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | `[−limit, limit]` に制限します |

---

## `FlightSnapshot`

**ファイル:** `autopilot/feedback/FlightSnapshot.h` · **種別:** struct

1サイクルの間にループが把握している機体の情報すべてです。符号は航空機の慣例に従います。

| グループ | フィールド |
|---|---|
| 時刻/状態 | `timeUs`、`armed`、`linkLost` |
| 姿勢 | `imuValid`、`rollDeg`、`pitchDeg`、`yawDeg`、`rollRateDps`、`pitchRateDps`、`yawRateDps`、`accelXg/Yg/Zg` |
| 高度 | `baroValid`、`altitudeM`（電源投入地点からの高度）、`climbRateMs`、`heightAglValid`、`heightAglM`（将来の距離計） |
| 速度 | `airspeedValid`、`airspeedMs`（将来のピトー管）、`gpsValid`、`groundSpeedMs` |
| モードの目標 | `stabilizationActive`（false = MANUAL。学習のみ）、`targetRollDeg`、`targetPitchDeg` |
| コマンド、µs | `stick*Us` — パイロットの入力、`command*Us` — 実際に舵へ出ているもの |
| スロットル、% | `pilotThrottlePercent`、`throttlePercent`（実際にESCへ出ているもの） |
| フラップ | `flapsUs`、`flapsMoving` |

---

## `FeedbackOutput`

**ファイル:** `autopilot/feedback/FeedbackOutput.h` · **種別:** struct

| フィールド | 説明 |
|---|---|
| `float deflectionUs[3]` | 軸ごとの舵の振れ角、µs（`ControlCommand` の符号） |
| `bool axisEnabled[3]` | `false` — その軸は制御せず、舵はパイロットに任せます |
| `float throttleOverridePercent` | 飛行フェーズが指定する絶対スロットル。`< 0` — 未指定 |
| `float throttleFloorPercent` | スロットルの下限（失速からの保護）。`< 0` — なし |
| `targetRollDeg`、`targetPitchDeg` | 制限をかけた後の最終的な目標（デバッグ用） |
| `const char* reason` | ログ/OLED向けの短い説明 |

---

## `PhaseTargets`

**ファイル:** `autopilot/feedback/PhaseTargets.h` · **種別:** struct

`TakeoffSequencer` と `LandingSequencer` に共通の出力です。「何を」であって、「どうやって」ではありません。

| フィールド | 既定値 | 説明 |
|---|---|---|
| `active` | `false` | そのフェーズがいま機体を操縦している |
| `targetRollDeg`、`targetPitchDeg` | 0 | 目標 |
| `controlRoll`、`controlPitch` | `true` | `false` — その軸には触れません（車輪の上ではピッチを降着装置が決めます） |
| `holdHeading`、`headingDeg` | `false`、0 | ラダーと操舵輪で方位を保ちます |
| `throttlePercent` | −1 | −1 — パイロットのスロットル |
| `reason` | `""` | 説明 |

---

## `SpeedEstimator`

**ファイル:** `autopilot/feedback/SpeedEstimator.h`

速度（対気 > GPSの対地速度 > 不明）と、IMUから求めた前後方向の加速度を扱います。
`dV/dt = g · (ax − sin θ)` を `ACCEL_FILTER_TAU_S` のローパスフィルタに通すので、速度センサーがなくても「速度が落ちている」ことが分かります。

| メソッド | 説明 |
|---|---|
| `void update(const FlightSnapshot&)` | 1ステップ進めます。前回の呼び出しから `dt ≤ 0` または `> 0.5 s`（最初の呼び出しでは `timeUs = 0` から）の場合は無視します |
| `bool hasSpeed() const`、`float getSpeed() const`、`Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`、`float getAcceleration() const` | m/s²、「+」は加速。IMUがなければ `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`、`0.05..4` に制限。速度がなければ 1 |

---

## `AirborneDetector`

**ファイル:** `autopilot/feedback/AirborneDetector.h`

機体が空中にいるかどうかを判定します。学習、積分の蓄積、失速の監視は、飛行中にしか意味がありません。

| メソッド | 説明 |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | アームされていなければ「地上」に戻します。状態が変わる候補は、`AIRBORNE_CONFIRM_MS`（離陸）または `GROUND_STILL_MS`（着陸）の間続く必要があります |
| `void force(bool)` | 明示的に設定します（離陸と着陸は状態を把握しています） |
| `void reset()` | 地上に戻します |
| `bool isAirborne() const` | |

「飛行に見える」条件は、距離計または気圧計の高度が `AIRBORNE_HEIGHT_M` を超えること、または速度が `ROTATE_SPEED_MS` を超えることです。「地上に見える」条件は、低い高度で、全軸の角速度が `TOUCHDOWN_STILL_RATE_DPS` 未満、かつ |a| ≈ 1g（± `GROUND_ACCEL_TOLERANCE_G`）であることです。

---

## `ControlEffectivenessEstimator`

**ファイル:** `autopilot/feedback/ControlEffectivenessEstimator.h`

1軸分の推定器です。モデルは **角加速度 = b·舵(t − 遅れ) + a·ω + c** です。
`b` は舵の効き（µsあたりの °/s²。符号は応答の向き）、`a` は減衰（1/s。通常は < 0）、
`c` は一定のモーメント（オートトリム）です。`b` は基準速度で学習し、
`b = b_ref · (V/V_ref)²`、`a = a_ref · V/V_ref` として換算します。推定には忘却付きの再帰最小二乗法を使います（`λ = 0.995`、記憶は約4秒）。

| メソッド | 説明 |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | `reset()` を呼びます |
| `void reset()` | θ = (prior, 0, 0)。共分散は b ± prior、a ± 5、c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | 毎サイクル呼びます。`ESTIMATOR_PERIOD_MS` の区間で平均を蓄積し、区間の終わりにジャイロの差分から加速度を求め、コマンドの遅れを反映し、両辺に共通のローパスをかけ、RLSを1ステップ進めます（学習が許可され、かつ励起がある場合） |
| `getEffectiveness()` | 現在の速度での `b` |
| `getReferenceEffectiveness()` | 基準速度での `b` |
| `getDamping()` | 現在の速度での `a` |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b`（`\|b\|` が小さいときは 0） |
| `getEffectivenessSigma()` | 現在の速度での `b` の推定のσ |
| `bool isConfident() const` | RLSが50ステップ以上、`\|b\|` が最小値以上、かつσ < 0.3·`\|b\|` |
| `getAngularAccel()` | 直近の区間の加速度（デバッグ用） |

特記事項:

- 区間が `MAX_GAP_MS = 200` より長い場合（ループが止まっていた場合）は、データをゼロから取り直します（遅れの履歴とフィルタをリセットします）。
- 学習するのは**励起**があるときだけです。16区間（約0.3秒）で平均したコマンドの振幅が `MIN_EXCITATION_US` 以上であること。それ以外は推定を固定します。
- 1ステップごとのfloatの後始末として、`P` の対称化、分散の上限（初期値の10倍）、
  `b`（`±EFFECTIVENESS_MAX`）と `a`（`−40..5`）の制限を行います。

---

## `AxisModel`

**ファイル:** `autopilot/feedback/AdaptiveRateController.h` · **種別:** struct

レギュレータが使う、1軸の応答についての既知の情報です。`effectiveness`
（b、既定は1）、`damping`（a、0 — 補償しない）、`bias`（c、0）。

---

## `AdaptiveRateController`

**ファイル:** `autopilot/feedback/AdaptiveRateController.h`

1軸のレギュレータで、3段構成です。

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| メソッド | 説明 |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | 積分、飽和、出力を 0 にします |
| `float angleToRate(targetDeg, angleDeg) const` | 第1段（方位では最短経路を取ります） |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | 第2〜3段。振れ角（µs）を返します |
| `getDesiredRate()`、`getIntegral()`、`getOutput()`、`isSaturated()` | 状態 |

不変条件: `|b|` は `EFFECTIVENESS_MIN` を下回りません（bの符号は保ちます）。積分は °/s の単位で保持するため、`b` が変わっても正しいままです。また、
**ストッパーに向かって積み上がりません**（前のステップの飽和の向きによるアンチワインドアップ）。
`dt ≤ 0` のときは積分は変化しません。

---

## `StallGuard`

**ファイル:** `autopilot/feedback/StallGuard.h`

速度低下と失速からの保護です。レベルは `Level::{Normal, LowEnergy, Stall}` です。

| メソッド | 説明 |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | 地上、またはIMUがない場合は Normal に戻します |
| `Level getLevel() const`、`const char* getLevelName() const` | `"OK"`、`"LOW_ENERGY"`、`"STALL"` |
| `const char* getReason() const` | 最後に作動した兆候 |
| `float maxPitchDeg() const` | Stall: −5°、LowEnergy: 5°、それ以外は 90° |
| `float maxBankDeg() const` | Stall: 10°、それ以外は 180° |
| `float maxAileronUs() const` | Stall: 150 µs、それ以外は `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall は 100 %、LowEnergy は 80 %、それ以外/リンクなしは −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` は `pitchEffectivenessKnown` と `pitchEffectiveness`
（ピッチのbの推定値の絶対値）です。

**LowEnergy** の兆候は次のとおりです。ピッチが5°を超える状態で、`DECEL_WARN_MS2`
を上回る減速が（`DECEL_CONFIRM_MS` の間）確認されたとき。速度が `1.25·Vs` 未満のとき。信頼できる昇降舵の効きが、事前の効きの35 %未満のとき。**Stall** の兆候は次のとおりです。速度が Vs 未満のとき。昇降舵が50 µsを超えて「上げ」の状態で、機首が60 °/s より速く下がるとき。エネルギーが低い状態で、翼がエルロンと逆向きに120 °/s より速く落ちるとき。対策は `RECOVERY_HOLD_MS` 経過後、かつエネルギーが回復したときにだけ解除します（速度が `1.5·Vs` 以上。速度センサーがなければ加速度が 0 以上）。

---

## `TakeoffSequencer`

**ファイル:** `autopilot/feedback/TakeoffSequencer.h`

フェーズごとの離陸です。`State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`。遷移図は [ARCHITECTURE.md §7](../ARCHITECTURE.md#離陸と着陸フィードバックループ未接続) にあります。

| メソッド | 説明 |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | 有効なフェーズ → `Aborted`。目標はリセットされます |
| `void update(snapshot, speed, nowMs)` | 1サイクルにつき遷移は高々1回。その後、新しいフェーズの目標を設定します |
| `void reset()` | → `Idle` |
| `getTargets()`、`getState()`、`getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

各フェーズの目標は次のとおりです。待機ではスロットル0で、舵はパイロットに任せます。滑走ではスロットル100 %、翼を水平に保ち、ピッチには触れず、走り出した時点で固定した方位を保ちます。上昇ではスロットル100 %、翼を水平に保ち、ピッチは
`CLIMB_PITCH_DEG` にします。手投げ発進は、前後方向の加速度
`ax − sin θ ≥ LAUNCH_ACCEL_G` が `LAUNCH_DETECT_MS` を超えて続いたときに検出します。

---

## `LandingSequencer`

**ファイル:** `autopilot/feedback/LandingSequencer.h`

フェーズごとの着陸です。`State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`。

| メソッド | 説明 |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`、`void reset()`、`update(snapshot, nowMs)` | 離陸と同じです |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — ここでは失速保護を無効にします |
| `bool isOnGround() const` | Rollout / Complete |

降下中とフレアでのピッチは、垂直速度の誤差から求めます。
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)` を `[min, FLARE_MAX_PITCH_DEG]`
に制限します。気圧計がなければ基準角を使います。高度は距離計から取り、なければ気圧計から取ります。接地は、|a − 1g| ≥ 0.5g のピークか、`TOUCHDOWN_STILL_MS`
の間「低く、回転していない」状態で判定します。着陸後の滑走では、接地した時点の方位を固定します。

---

## `FeedbackSupervisor`

**ファイル:** `autopilot/feedback/FeedbackSupervisor.h`

ループ全体を束ねます。`SpeedEstimator`、`AirborneDetector`、3つの
`ControlEffectivenessEstimator`、3つの `AdaptiveRateController`、`StallGuard`、
`TakeoffSequencer`、`LandingSequencer` を所有します。

| メソッド | 説明 |
|---|---|
| `bool requestTakeoff()` | アーム済みでリンクがあり、地上にいる場合のみ。着陸を中止します |
| `bool requestLanding()` | アーム済みでリンクがあり、空中にいる場合のみ。離陸を中止します |
| `void cancelPhase()` | フェーズを中止します |
| `const FeedbackOutput& update(const FlightSnapshot&)` | 1サイクル（順序は [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-フィードバックループ未接続) にあります） |
| `getOutput()`、`isAirborne()`、`getSpeedEstimator()`、`getEstimator(axis)`、`getController(axis)`、`getStallGuard()`、`getTakeoff()`、`getLanding()` | ログとテスト向けの状態 |
| `void printStatus(Print& out) const` | 状態行1行と、軸ごとに1行（`b ± σ`、`*` — 信頼できる、`a`、`c`、`I`、出力） |

重要なルール:

- **アームされていない**場合は全軸を無効にし、`reason = "未アーム"` とします。
  ARM/DISARM（新しいフライト）で、学習した内容はすべて消えます。
- **リンクが失われる**とフェーズを中止します。スロットルには触れません（ファームウェアのフェイルセーフが動きます）。
- **地上からの離陸**で、推定値とレギュレータをリセットします（車輪の上で「見えた」ものは当てはまりません）。
- `b` の信頼できる推定値が負になった場合、それは**決して**レギュレータに渡しません。その軸は事前モデルで動き、`reason` に「… 舵に対して逆に反応していませんか？地上で確認してください」という警告が入ります。
- 積分は地上では凍結します。ただし、離陸滑走と着陸滑走中の方位は例外です。
- 協調旋回（空中で速度が分かっている場合）: ピッチの目標角速度に
  `+ g/V · sin φ · tg φ` を、ヨーの目標角速度に `g/V · sin φ` を加えます（バンク角は±60°に制限します）。
- `reason` の優先順位は、失速 > エネルギー不足 > フェーズ > 符号の警告 >
  「安定化」/「マニュアル（学習）」です。
