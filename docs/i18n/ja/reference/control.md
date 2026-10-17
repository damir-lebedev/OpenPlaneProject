# CONTROL と COORDINATION — ミキサー、スロットル、ARM、出力、オーケストレーター

> 🌐 このページは[ロシア語の原文](../../../reference/control.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

CONTROL レイヤーは、UART、PWM、Wi-Fi を含まない、データに対するロジックです。
`FlightController`（COORDINATION）は、すべての下位レイヤーを 1 つの周期にまとめる唯一のクラスです。

---

## `ControlCommand`

**ファイル：** `control/ControlCommand.h` · **種別：** struct

**物理的な符号**で表した舵面へのコマンドで、舵角は µs です（±500 = フルストローク）。スティック、オートパイロット、ミキサーに共通の言語です。

| フィールド | 「+」の意味 |
|---|---|
| `int16_t roll` | 右ロール（右エルロン上げ、左下げ） |
| `int16_t pitch` | 機首上げ（エレベーター上げ） |
| `int16_t yaw` | 機首右（ラダーとホイールが右） |
| `int16_t flaps` | フラップ下げ（左右のエルロンとも下げ）。「−」はエアブレーキ（左右とも上げ） |

すべてのフィールドの既定値は 0 です。

---

## `FlightOutputState`

**ファイル：** `control/FlightOutputState.h` · **種別：** struct

出力の目標パルスで、単位は PWM の µs です。既定では舵面がニュートラル、スロットルは
`PWM_MIN` です。

`throttle` はどちらの方向でもスロットルの大きさ（1000..2000 = 0..100%）です。これを `motorReverse` とあわせて ESC パルスに変換するのは `FlightOutputs` です（`EscSignal` 経由）。

| フィールド | 既定値 |
|---|---|
| `aileronLeft`、`aileronRight`、`elevator`、`rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false`：モーターは前進 |
| `aux1` | `PAYLOAD_CLOSED_US`（ペイロード投下は閉じた状態） |
| `aux2` | `PWM_CENTER`（カメラ） |

---

## `FlapsController`

**ファイル：** `control/FlapsController.h` · **依存先：** `Config`

フラップを滑らかに展開／格納します。位置は目標（スイッチのフラップ、ノブのフラップ、上向きのエアブレーキなど任意の値）に向かって、`FLAPS_TRANSITION_MS` で全ストローク
`FLAPS_DEPLOYED_US` を動く速度を上限に近づきます。時刻は引数で渡します。

| メソッド | 説明 |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | 目標に向かって 1 ステップ進め、現在位置を µs で返す（+ は下げ、− は上げ） |
| `int16_t getPosition() const` | 現在位置 |

不変条件：

- **最初の呼び出し**では位置を直ちに目標に合わせます。電源投入時に机の上でフラップが「出てくる」ことはありません。
- 時間刻みは `MAX_STEP_MS = 20` に制限されます。長い中断（failsafe、校正）のあとでも、フラップが 1 周期で目標へ飛ぶことはありません。

---

## `ControlMixer`

**ファイル：** `control/ControlMixer.h` · **依存先：** `RcInput`、`RcChannelState`、`FlapsController`、`ControlCommand`、`FlightOutputState`、`Config`、`Channels`

空力的なロジックを 2 段階で行います。`FlapsController` を所有します。

| メソッド | 説明 |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll`（2000 = 右）、CH2 → `pitch`（**符号が逆**：2000 = 自分から遠ざける = 機首下げ）、CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | フラップの目標は `FlightController` が選び（エアブレーキ → フラップスイッチ → ノブ）、ここでは滑らかな動きを担当する |
| `FlightOutputState mix(const ControlCommand& c) const` | コマンド → PWM。ロール／ピッチ／ヨーはストローク（`*_MAX_US`）で制限される。エルロンは左 = `flaps + roll`、右 = `flaps − roll`（下げ = 「+」）。PWM = `1500 ± 舵角` で、符号は `Config::*_REVERSED`、フラップの符号は `FLAPS_REVERSED` に従い、1000..2000 に制限される。`throttle` は設定しない |
| `int16_t getFlaps() const` | 現在のフラップ位置（µs） |

フラッペロン：展開すると左右のエルロンが `FLAPS_DEPLOYED_US` だけ下がり（新しい「ニュートラル」）、ロールはその上に重ねて働きます。フルロールでは、下がるほうのエルロンが上がるほうより先にストロークの端に達します。これがエルロンのディファレンシャルとして働きます。

---

## `ThrottleManager`

**ファイル：** `control/ThrottleManager.h` · **依存先：** `RcInput`、`RcChannelState`、`Config`、`Channels`

| メソッド | 説明 |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | CH3 のスロットルを 1000..2000 に制限して返す。リンク喪失時は `FAILSAFE_THROTTLE` |

ARM とオートパイロットのことは知りません。それらの補正は `FlightController` が適用します。

---

## `EscSignal`

**ファイル：** `control/EscSignal.h` · **種別：** namespace · **依存先：** `Config`

スロットル（1000..2000 = 0..100%）と方向 → ESC ピンのパルス。

| 関数 | 説明 |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL`（3D モードの BLHeli 80A）：0% → `ESC_NEUTRAL_US`（1500）、前進 1500..2000、後退 1500..1000。一方向 ESC：1000..2000 をそのまま、`reverse` は無視。1000..2000 の範囲外のスロットルは切り詰める |
| `uint16_t stopUs()` | 「モーター停止」のパルス（双方向 ESC では 1500、普通の ESC では 1000） |

変換だけで、保護はしません。方向を変えてよいかは `MotorReverse` が、後退時のスロットル上限は `FlightController` が決めます。

---

## `MotorReverse`

**ファイル：** `control/MotorReverse.h` · **依存先：** `Config`

`Feature::MOTOR_REVERSE` スイッチ（既定では SwD）によるモーターの方向。**ARM していないときのみ**。

| メソッド | 説明 |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | ARM していない：方向はすぐにスイッチに従う（Serial に `MotorReverse: мотор НАЗАД` / `вперёд`「モーター後退」/「前進」と表示）。ARM 中：スイッチは効かず、「реверс НЕ переключён」（「逆転は切り替わっていません」）を 1 回表示し、新しい方向は DISARM を待つ。`ESC_BIDIRECTIONAL = false` では逆転は決して入らない |
| `bool isReversed() const` | モーターが後退 |
| `bool isPending() const` | スイッチはすでに反対の位置にあり、方向は DISARM を待っている |
| `const char* armBlockReason(uint32_t nowMs) const` | ARM を拒否する理由：最後に方向を変えてから `REVERSE_SETTLE_MS` 未満（モーターがまだ回っているかもしれない）。それ以外は `nullptr` |

---

## `ArmingManager`

**ファイル：** `control/ArmingManager.h` · **依存先：** `Autopilot`（null 可）、`RcChannelState`、`Config`、`Channels`

ARM は専用のスイッチ SwA（CH5）で行います。ステートマシンは
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager) にあります。

| メソッド | 説明 |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | オートパイロットがなければスロットルだけを検査する |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | failsafe 中は何もしない（failsafe フレームのスイッチはパイロットの操作を反映していない）。スイッチ OFF → DISARM、`switchSeenOff = true`。OFF→ON への遷移 → 検査 → ARM または拒否。`blockReason` は外部からの拒否理由（`FlightController` から：`MotorReverse::armBlockReason()`） |
| `bool isArmed() const` | ARM 済みか |
| `const char* getLastRefusalReason() const` | 最後に拒否した理由、または `nullptr`。スイッチを切るとクリアされる |

`checkFailureReason(rc)`：ARM の検査項目

| 条件 | 拒否の理由 |
|---|---|
| スロットル ≥ `THROTTLE_LOW_US` | 「スロットルが最低位置にない」 |
| MANUAL 以外のモードで、IMU はあるが応答しない | 「IMU が応答しない…」 |
| MANUAL 以外のモードで、IMU の飛行前チェックに問題がある | `ImuSensor::getPreflightProblem()` の文言 |
| 高度を使うモード（`needsAltitude`：ALT_HOLD、CRUISE、LOITER、RTH、AUTO_LAND、SOARING）で、気圧計はあるが応答しない | 「気圧計が応答しない…」 |
| ほかはすべて合格したが、`blockReason` が渡された | `blockReason`（例：「モーターの方向が変わったばかり…」） |

ビルドに存在しないセンサー（`nullptr`）は ARM を妨げません。MANUAL では、センサーがまったくなくても機体は ARM できます。GPS の測位は検査項目に意図的に含めていません。
GPS がなくてもナビゲーション系のモードは安全に動作し（その場で旋回）、ホームは GPS が衛星を捕捉した時点で記録されるためです。

不変条件：スイッチを ON にしたまま基板の電源を入れても ARM しない。OFF→ON の遷移ごとに試行は 1 回。リンク喪失で ARM は解除されない。

---

## `FlightOutputs`

**ファイル：** `control/FlightOutputs.h` · **依存先：** `IBoard`、`FlightOutputState`、`EscSignal`、`Config`

PWM 出力の集合とその順序を知る唯一のクラスです。すべての出力が 1 つのテーブルに記述されており、`begin()`、`write()`、状態表示、セルフテストはそれをループでたどります。

### `FlightOutputs::OutputInfo`

| フィールド | 説明 |
|---|---|
| `const char* key` | JSON／ログでの名前（`aileronLeft`、…、`esc`、`rudder`、`aux1`、`aux2`） |
| `const char* label` | 人間向けの名前 |
| `int16_t pin` | ピン番号。`-1` は未配線。STM32 ではアナログピンの番号が `0xC0 + N` なので `int16_t` |
| `bool required` | これがないと機体は飛べない（ラダーは任意） |
| `uint16_t FlightOutputState::* field` | 状態のフィールドへのポインタ |

| メソッド | 説明 |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | テーブルの行。順序は `ServoChannel` と同じ |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | 各出力の `attach(PWM_MIN, PWM_MAX)`、状態と ESC の種類を表示。**必須**の出力がすべてチャンネルを得たら `true` |
| `bool isAttached(uint8_t ch) const` | 出力が接続されている（範囲外のインデックスは `false`） |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | 出力ピンのパルス：表に従ったフィールド。ESC は `EscSignal::pulseUs(throttle, motorReverse)` を通す |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | 配線済みの各ピンで、測定したパルスを期待値と比較する。差が 15 µs 以内なら「OK」 |
| `void write(const FlightOutputState&)` | すべての出力を書き込み、状態を記憶する |
| `void setFailsafe()` | 舵面はニュートラル（`FAILSAFE_*`）、スロットルは `FAILSAFE_THROTTLE`（双方向 ESC のピンでは 1500）。AUX はそのまま（通信途絶でペイロードは投下しない） |
| `void setBuzzer(bool on)` | 基板のブザー（`IBoard::setBuzzer`） |
| `const FlightOutputState& getLastState() const` | 最後に書き込んだ状態 |

出力を追加するには、テーブルに行を足し、`FlightOutputState` にフィールドを足し、
`ServoChannel` にインデックスを足します（さらに `Esp32Board` にピンと LEDC
チャンネルも）。

---

## `FlightController`

**ファイル：** `control/FlightController.h` · **レイヤー：** COORDINATION ·
**依存先：** `IBusReceiver`、`ControlMixer`、`ThrottleManager`、`ArmingManager`、`FlightOutputs`、`Autopilot*`、`PilotSwitches*`、`Beeper`、`MotorReverse`

制御ループの唯一のコーディネーターです。自分では UART を解析せず、PWM にも触れず、ミキサーの計算もしません。ほかのクラスを正しい順序で呼ぶだけです。詳しい図は
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-制御周期flightcontrollerupdate) にあります。

| メソッド | 説明 |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | オートパイロットなしなら純粋な手動操縦、スイッチなしならスティックのみ |
| `void begin()` | `outputs.setFailsafe()`、`receiver.begin()` |
| `void update()` | 1 周期分の処理（下記参照） |
| `bool isReceiverFailsafe() const` | リンクを失っている |
| `const IBusReceiver& getReceiver() const` | ログ用（フレームカウンタ、喪失の原因） |
| `bool isArmed() const`、`const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | 出力に最後に書き込んだ状態 |
| `const RcChannelState& getRcState() const` | チャンネル |
| `const FlightOutputs& getOutputs() const` | 出力テーブルと `attached` |
| `int16_t getFlapsUs() const` | フラップの位置 |
| `const PilotSwitches* getSwitches() const`、`const PilotInputs& getInputs() const` | この周期のスイッチとノブ |
| `bool isLostModelBeeping() const` | 「ここにいるよ」ブザーが鳴っている |
| `const MotorReverse& getMotorReverse() const` | モーターの方向と保留中の逆転 |

`update()` の順序：

1. `receiver.update()`。`failsafe = receiver.isSignalLost()`。
2. リンクが生きていれば `switches->update(rc)`（モード、機能、ノブ）。
3. `pilotThrottle = throttle.update(rc, failsafe)`。
4. スティック `mixer.fromSticks(rc)`（リンクが生きていれば）× `Knob::RATES`。フラップは
   `mixer.updateFlaps(target)`：`AIRBRAKE` → −`AIRBRAKE_US`、`FLAPS` →
   `FLAPS_DEPLOYED_US`、`Knob::FLAPS` → 滑らかに、リンク喪失時は 0。
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` を**常に**実行。
   逆転中は `pilotThrottle` の代わりに `PWM_MIN` を渡す（オートパイロットにとってモーターは停止）。
6. ブザー：`Beeper::update(BEEPER, armed, failsafe, now)`。
7. リンク喪失 → `applyLinkLoss()` を実行して周期を抜ける。
8. `arming.update(rc, false, reverse.armBlockReason(now))`、続いて `reverse.update(MOTOR_REVERSE, armed, now)`。DISARM 時は同じ周期で方向が変わる。
9. `command = autopilot->getCommand()`（オートパイロットがなければスティック）。フラップは専用の値。
10. `output = mixer.mix(command)`。`output.motorReverse = reverse.isReversed()`。前進なら `output.throttle = autopilot->applyThrottle(pilotThrottle)` に上限 `THROTTLE_LIMIT_PCT` × ダイヤル `Knob::THROTTLE_LIMIT`、後退ならパイロットのスロットルに上限 `min(REVERSE_THROTTLE_LIMIT_PCT, 前進の上限)`。許された範囲はスティックの全ストロークに引き伸ばす。
11. ARM していない、または `MOTOR_KILL` → `throttle = PWM_MIN`（最後に実行）。
12. AUX1 はペイロード（`PAYLOAD_DROP`）、AUX2 はカメラ（`Knob::CAMERA_TILT`、`CAMERA_STAB` はピッチを差し引く）。`AUX1_REVERSED`/`AUX2_REVERSED` はパルスを反転。
13. `outputs.write(output)`。

`applyLinkLoss()`：オートパイロットが failsafe 中（ARM 済みなら RTH または滑空）なら、舵面とスロットルはオートパイロットのコマンドに従います（フラップは滑らかに格納され、
`MOTOR_KILL` は引き続きモーターを止め、AUX はそのまま）。そうでなければ
`outputs.setFailsafe()` を実行します。逆転中、モーターは常に停止しています。

---

## `Beeper`

**ファイル：** `control/Beeper.h` · **依存先：** `Config`

| メソッド | 説明 |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | ブザーの状態：`Feature::BEEPER` がある場合、または「機体ロスト」（ARM しておらず、`LOST_MODEL_BEEP_DELAY_MS` を超えてリンクがない）の場合に 2 Hz で鳴らす |
| `bool isLostModel() const` | 「草むらで探して」モード |
