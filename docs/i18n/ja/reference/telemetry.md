# TELEMETRY — ログ、コンソール、Web ダッシュボード、OLED、ブラックボックス

> 🌐 このページは[ロシア語の原文](../../../reference/telemetry.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。

[← リファレンス](README.md)

テレメトリは飛行ロジックから完全に分離されています。読むのは、
`FlightController`、`Autopilot`、センサー、`LoopStats` の const なゲッターだけです。「逆向き」の経路はダッシュボードのコマンドだけで、これは `WebDebugServer` のメールボックスを通り、飛行ループが適用します。

---

## `LoopStats`

**ファイル:** `telemetry/LoopStats.h` · **種別:** struct

飛行ループの周波数と所要時間です。

| メンバー | 説明 |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | 1秒に1回公開され、他のタスクから読まれます（32 ビットなので「ちぎれた」読み出しは起きません） |
| `void record(uint32_t durationUs)` | 毎ティック `loop()` から呼びます |
| `uint32_t takePeakUs()` | 前回の呼び出し以降で最悪のティック（10秒ごとの SYS 行用）。`record()` と同じタスクから呼びます |

`maxUs` は直近の1秒間だけの最悪値です。まれな引っかかりは
`takePeakUs()` で分かります。

---

## `LogSettings`

**ファイル:** `telemetry/LogSettings.h` · **依存先:** `Preferences`（NVS、名前空間 `debuglog`）

### `LogChannel`（enum class）

| チャンネル | 接頭辞 | 出力する内容 | 既定 |
|---|---|---|---|
| `Status` | `STAT` | リンク、ARM、モード、フラップ、センサー | 変化時 |
| `Rc` | `RC` | 送信機のチャンネル | オフ |
| `Outputs` | `OUT` | 舵と ESC への出力 | オフ |
| `Attitude` | `ATT` | ロール、ピッチ、方位 | オフ |
| `Autopilot` | `AP` | 目標と補正量 | オフ |
| `Altitude` | `ALT` | 高度、垂直速度 | オフ |
| `Heading` | `MAG` | コンパスの方位 | オフ |
| `Gps` | `GPS` | 衛星、座標 | オフ |
| `Imu` | `IMU` | ジャイロと加速度計 | オフ |
| `Nav` | `NAV` | ホーム、方位、速度、ピトー管、有効な機能 | オフ |
| `System` | `SYS` | ループの周波数、メモリ（10秒ごと）、オフ/オンのみ | オン |
| `Count` | — | チャンネルの数 | — |

`LogMode`（enum class）: `Off`、`OnChange`、`Periodic`。

`LogChannelInfo`: `tag`、`title`、`periodicOnly`、`defaultMode`。

| メソッド | 説明 |
|---|---|
| `static constexpr uint8_t COUNT`、`PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | チャンネル表の1行 |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms（順に巡回） |
| `LogSettings()`、`void setDefaults()` | 既定のモード、周期は1秒 |
| `LogMode mode(uint8_t)`、`mode(LogChannel)` | チャンネルのモード |
| `void setMode(uint8_t, LogMode)` | `periodicOnly` のチャンネルでは `OnChange` が `Periodic` になります |
| `void cycleMode(uint8_t)` | オフ → 変化時 → 常時 → オフ（SYS: オフ ↔ オン） |
| `void setAll(LogMode)` | 全チャンネルに適用します。「すべて変化時」のコマンドは SYS には触れません |
| `uint16_t periodMs() const`、`void cyclePeriod()` | 「常時」モードの周期 |
| `static const char* modeName(LogMode, bool periodicOnly)` | 「オフ」/「変化時」/「常時」（または「オン」） |
| `void load()` | NVS から読み込みます。`VERSION` または長さが一致しなければ既定値のままとなり、未知のモードコードはそのチャンネルの既定値になります |
| `void save() const` | NVS に保存します（モード、周期、バージョン） |

`VERSION` はチャンネルの一覧と一緒に変わります。古い設定はリセットされます（`VERSION = 2`: NAV チャンネルを追加）。メニューでのチャンネルのキーは `1`..`9`、NAV は
`n`、SYS は `s` です。

---

## `DebugLogger`

**ファイル:** `telemetry/DebugLogger.h` · **依存先:** `FlightController`、`Autopilot*`、`LoopStats*`、`LogSettings`、`Config`

状態をチャンネルごとにシリアルモニタへ出力します。チャンネルごとに専用の行、専用のモード、専用の許容幅があります。

| メソッド | 説明 |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | `DEBUG_INTERVAL_MS` ごとに1回、チャンネルを巡回します（一時停止中とメニューを開いている間は黙ります） |
| `LogSettings& getSettings()`、`void saveSettings() const` | コンソールのメニュー用 |
| `void suspend(bool)` | メニューが開いている間は黙ります。解除時は `refresh()` |
| `void setPaused(bool)`、`bool isPaused() const` | スペースキーで一時停止。解除時は `refresh()` |
| `void refresh()` | 次のティックで、有効なチャンネルをすべて出力します |

チャンネルのロジック（`updateChannel`）:

- `Off` — 出力しません。
- `Periodic` — `periodMs()` ごとに1回（SYS は10秒ごとに1回）、値は「そのまま」。
- `OnChange` — 行は**許容幅**付きで組み立てます（入れ子の `Shown` は、新しい値が許容幅を超えて離れるまで古い値を保持します。RC/PWM は 3 µs、角度は
  0.5°、方位は 1°、補正量は 2、高度は 0.3 m、加速度は 0.03 g、座標は
  1e−5°）。直前に出力した行と異なる場合にだけ出力します。

入れ子の型: `LineBuffer : Print`（出力前の比較用の、最大 200 バイトの行）、
`Shown`（ヒステリシス付きの値）。

行のフォーマット:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  目標 0.0 m
MAG  方位 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (10秒間の最悪値) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` は `LOST(フレームなし)` と `LOST(送信機のフェイルセーフ)` を区別します。`IMU=` は
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK` のいずれかです。

---

## `DebugConsole`

**ファイル:** `telemetry/DebugConsole.h` · **依存先:** `FlightController`、`FlightOutputs`、`Autopilot`、`DebugLogger`、`LogSettings`、`IBoard*`（バスのスキャン）

シリアルモニタ上のテキストメニューです。画面のステートマシン `Screen::{None, Main, Log}` を持ちます。

| メソッド | 説明 |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | ボードがある場合 — コマンド `b` とメニューの項目 7 |
| `static const char* guessI2cDevice(uint8_t address)` | アドレスからチップを推定します: 0x6A LSM6DSV、0x68 MPU/ICM、0x76 BME280/BMP388/SPL06、0x46/0x47 BMP581、0x7C QMC6309、0x2C QMC5883P、0x0D QMC5883L、0x3C OLED |
| `void printHint() const` | 1行のヒント |
| `void update()` | `Serial` のバイトをすべて処理します。ログ設定が変更されていて、メニューが閉じており、機体が **armed ではない**場合は、NVS に保存します |

ホットキー（メニューの外）: `h`/`?` — メインメニュー、`l` — ログのメニュー、スペース —
ログの一時停止、`s` — センサーの状態、`i` — ジャイロの較正、`o` —
IMU の取り付けの較正、`m` — コンパスの較正、`p` — 出力のセルフテスト、
`b` — I2C バスのスキャン（0x08..0x7F — 0x7F まで。QMC6309 が 0x7C にあるため）でチップ名を表示します。それ以外はヒントを表示します。`\r`/`\n` は無視します。

ログのメニュー: `1`..`9` — チャンネル 0..8 のモードを巡回、`n` — NAV、`s` — SYS、`p` — 周期、`a` —
すべて「変化時」、`x` — すべてオフ、`d` — 既定値、`0`/`q` — 戻る、`l`/`h` —
閉じる。

ブロックする操作（`i`、`o`、`m`、`p`）は **ARM 中は禁止**です。メニューが開いている間は、ログを停止します（`DebugLogger::suspend`）。メニュー項目の幅はバイト数ではなく UTF-8 の文字数で数えます（キリル文字は 2 バイト）。

---

## `WebDashboardPage`

**ファイル:** `telemetry/WebDashboardPage.h` · **種別:** namespace

`static const char HTML[] PROGMEM` — ページ全体（HTML + CSS + JS）を1つのリテラルにしたものです。動的な部分はすべて、ブラウザが `/api/status` の JSON から作ります（
200 ms ごとにポーリング）。チャンネル、出力、センサーの行は JSON のキーから生成されるので、新しい出力はページを編集しなくても現れます。ユーザーが編集し始めた PID のフィールドは、ポーリングで上書きされなくなります。

---

## `WebDebugServer`

**ファイル:** `telemetry/WebDebugServer.h` · **依存先:** `WebServer`、`WiFi`、`FlightController`、`Autopilot*`、`WebDashboardPage`、`Config`

| メソッド | 説明 |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Wi-Fi の AP（`persistent(false)` — フラッシュには書き込みません）、ルート、コア 0 上のタスク `web`。アクセスポイントが立ち上がらなければ `false` |
| `void applyPendingCommands()` | 飛行ループから呼びます。スピンロックの下でコマンドを取り出し、オートパイロットに適用します |

ルート:

| ルート | 応答 |
|---|---|
| `GET /` | ダッシュボードのページ |
| `GET /api/status` | 状態の JSON（`buildStatusJson()`）。形式は [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) にあります |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`、ボディなし → 400 `no data`、オートパイロットなし → 503、不正なモード → 400 `invalid mode` |
| `POST /api/setpid` | `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch` のうちの任意のもの。省略したものは現在の値のままです |
| それ以外 | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — `portMUX` の下にあるメールボックスです。`extractJsonNumber(body, key, fallback)` — ArduinoJson を使わない、フラットな JSON の最小限の解析です: `"key"`、空白、`:`、空白、JSON のあらゆる表記の数値（符号、小数、指数 `1e-7`）。キーまたは数値がなければ
`fallback`。

JSON のフィールド `attached`/`available` は**常に**あります。センサーのデータは
`available: true` のときだけです。

---

## `OledDisplay`

**ファイル:** `telemetry/OledDisplay.h` · **依存先:** U8g2、`II2CBus`、`FlightController`、`Autopilot*`、`LoopStats`

2本目の I2C バスにつないだ SSD1306 128×64（I2C 0x3C）です。専用のタスク `oled`
（`Rtos::startTask`: ESP32 ではコア 0、STM32 では低い優先度）が 200 ms ごとに動きます。

| メソッド | 説明 |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr`、または画面が 0x3C で応答しなければ `false`。そうでなければ U8g2 を設定してタスクを起動します |

U8g2 は `II2CBus` の上に載せた `byteCallback` でバイトを送ります（画面は
`Wire1` のことを知りません）。C のコールバックにはコンテキストが渡されないため、バスは静的な変数 `busSlot()` に保持します。機体上の画面は1つだけです。

画面:

```
RX ok ARM STAB FL       リンク（喪失時は反転行）/ ARM / モード / フラップ
R  +1.2 P  -0.4         ロール / ピッチ、°           (IMU --)
Alt +0.3 Vz +0.1 A14    高度 / 垂直速度 / 対気速度（ピトー管がある場合）(BARO --)
H123 T1000 Y1500        方位 / スロットル / ラダー (H---)
L1500 R1500 E1500       エルロン / エレベーター
Loop 500Hz max1100us    周波数と、1秒間で最悪のティック
```

モードの短い名前は `AutopilotNames::modeShort()` です（`MAN`、`STAB`、
`TKOFF`、`ALT`、`ACRO`、`CRZ`、`LOIT`、`RTH`、`LNCH`、`LAND`、`SOAR`、`RESQ`）。空中でリンクを失ったときは `GLIDE` または `FSRTH` になります。

---

## `BlackBox`

**ファイル:** `telemetry/BlackBox.h` · **依存先:** `FlightController`、`Autopilot`、`LoopStats`、`BlackBoxStorage`、`PilotSwitches*`

飛行の記録を、フラッシュ（ESP32-S3）または SD カード（STM32H743）に残します。何をいつどうやって取り出すかは [BLACKBOX.md](../BLACKBOX.md) を参照してください。

| メソッド | 説明 |
|---|---|
| `bool begin(bool startTask = true)` | 媒体を読み（`BlackBoxStorage::begin()`）、キューを確保し（ESP32 は PSRAM、STM32 は `malloc`）、消去済みの領域を照合し（最大 0.3秒）、タスク `bbox` を起動します（`Rtos::startTask`）。記録する場所（パーティション、カード、ファイル）がなければ `false` で、ブラックボックスはオフになります |
| `void update(uint32_t workUs)` | 毎ティックの後に `loop()` から呼びます: イベント、開始/停止、キューへのスナップショット、書き込みタスクの起床 |
| `void writerStep()` | 書き込みタスクの1ステップ: フラッシュへ1〜2ページ、または地上で1回の消去 |
| `requestManualStart()` / `requestManualStop()` | 手動の記録（コンソールの `k` → `r`） |
| `State getState()` / `bool isRecording()` | `Off`、`Idle`、`Recording`、`Stopping`（END を記録する前に、キューを最後まで書き出します） |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | コンソール用 |
| `void handleHostCommand(const char*)` | `bb list`、`bb get <n> [ボー]` — `tools/blackbox.py` 用（USB CDC では速度は何にも影響しません） |

プラットフォームごとに異なる部分: 再起動の原因は `readResetCause()`、バッテリーの電圧と電流は
ADC（S3 は `analogReadMilliVolts`、STM32 は 12 ビットの `analogRead`）、媒体のエラー（`BlackBoxStorage::writeErrors`/`eraseErrors`）は1秒に1回、「媒体: 書き込みエラー …」というイベントとしてログに入り、飛行の妨げにはなりません。

## `BlackBoxStorage`

**ファイル:** `telemetry/BlackBoxStorage.h` · **依存先:** `IFlashRegion`

4 KB のセクタのリングです。先頭と飛行の一覧は `begin()` のときにセクタのヘッダから得ます（1回目のパスで各セクタのヘッダを読んで本物を記憶し、2回目はそれだけを対象にします。空の領域は1回だけ読みます）。`openFlight()`/`append()`/`flush()`/`closeFlight()` — ページ単位の書き込み（各レコードに CRC-8）。`eraseStep(target, protect, allowErase)` — 先頭の前方で行う照合/消去の1ステップ: ごみは常に、飛行は丸ごと、かつ空きが `target` を下回っている間だけ。`protect` には決して触れません。

## `BlackBoxRing`、`BlackBoxFormat`

`BlackBoxRing` は、タスク/コア間のレコードのバイトキューで、`Rtos::CriticalSection` の下で動きます。あふれたときは最も古いものを捨てます。`BlackBoxFormat` — セクタのヘッダ、レコードの種類と構造体、スキーマの文字列（サイズは `static_assert` で照合）、CRC-8 と CRC-32。

---

## `Mavlink`（コーデック）

**ファイル:** `telemetry/MavlinkCodec.h` · **種別:** namespace · **依存先:** なし（移植可能）

生成ライブラリを使わない MAVLink 2 です。MAVLink の順序でフィールドを詰め込みます（pymavlink と照合済み）。CRC-16/MCRF4XX + `CRC_EXTRA`、末尾のゼロの切り詰め。

| エンティティ | 説明 |
|---|---|
| `Msg::*` | 識別子: HEARTBEAT、SYS_STATUS、SET_MODE、PARAM_*、GPS_RAW_INT、ATTITUDE、GLOBAL_POSITION_INT、SERVO_OUTPUT_RAW、MISSION_REQUEST_LIST/COUNT、NAV_CONTROLLER_OUTPUT、RC_CHANNELS、REQUEST_DATA_STREAM、VFR_HUD、COMMAND_LONG/ACK、HOME_POSITION、STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | メッセージの `CRC_EXTRA`。−1 は未知 |
| `crcAccumulate`、`crcCalculate` | X.25（mavlink の `crc_accumulate()` と同じ） |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — フィールドを順に |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — 連番の `seq` を持つ v2 フレーム |
| `Message` | 受信したメッセージ: `msgid`、`sysid`、`compid`、ペイロード（ゼロで埋めたもの）、オフセットによるフィールドの読み出し |
| `Parser` | `bool feed(byte)` → `message()`。v1 と v2 に対応し、v2 の署名は読み飛ばします。`goodCount()`、`badCrcCount()`。`CRC_EXTRA` が未知のメッセージは黙って読み飛ばします |

## `MavlinkModes`

**ファイル:** `telemetry/MavlinkTelemetry.h` · **種別:** namespace

| 関数 | 説明 |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | ArduPlane のモード番号: MANUAL 0、STABILIZE→FBWA 5、ALT_HOLD→FBWB 6、ACRO 4、CRUISE 7、LOITER 12、RTH→RTL 11、AUTO_TAKEOFF/LAUNCH→TAKEOFF 13、AUTO_LAND→AUTO 10、SOARING→THERMAL 24、RESCUE→STABILIZE 2。フェイルセーフは RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | 逆変換で、地上からのコマンド用です。AUTO、CIRCLE、GUIDED は `false` |
| `isAutonomous(mode)` | HEARTBEAT の `AUTO_ENABLED` フラグ |

## `MavlinkTelemetry`

**ファイル:** `telemetry/MavlinkTelemetry.h` · **依存先:** `IUartPort`、`FlightController`、`Autopilot*`、`LoopStats*`

QGroundControl / Mission Planner 向けの、無線モデム経由のテレメトリです（機体は
`MAV_TYPE_FIXED_WING`、`MAV_AUTOPILOT_ARDUPILOTMEGA`）。Wi-Fi のない STM32
（UART4）で使います。

| メソッド | 説明 |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | ポートを開き、メッセージ「OpenPlane online」を送ります |
| `void update()` | 飛行ループから呼びます: 受信データの解析（1ティックあたり 128 バイトまで）、イベントのメッセージ、1ティックあたり 2 フレームまで |
| `void statusText(severity, text)` | GCS のフィードへ（4 行のキュー、1 行あたり 50 文字まで） |
| `isGcsConnected()` | 直近 3秒以内に GCS からの HEARTBEAT がある |
| `getSentFrames()`、`getDeferredFrames()`、`getParser()` | 診断 |
| `static const char* paramName(uint8_t)` | `RLL_KP`、`RLL_KI`、`RLL_KD`、`PTCH_KP`、`PTCH_KI`、`PTCH_KD` |

ストリーム（Hz）: ATTITUDE 10、GLOBAL_POSITION_INT と VFR_HUD 5、GPS_RAW_INT、
RC_CHANNELS、SERVO_OUTPUT_RAW、NAV_CONTROLLER_OUTPUT 2、HEARTBEAT と SYS_STATUS 1、
HOME_POSITION 0.2。フレームは `availableForWrite()` に収まる場合にだけ送ります。収まらなければ次のティックまで待ちます（ループは決してブロックされません）。

受信するもの: GCS の HEARTBEAT、PARAM_REQUEST_LIST / READ / SET（PID はそのままオートパイロットへ。値は 0..100 で、保存はされません）、SET_MODE と COMMAND_LONG の
`DO_SET_MODE`（176）— 次にスイッチを切り替えるまでのモード、`COMPONENT_ARM_DISARM`
（400）— **DENIED**、`REQUEST_MESSAGE`（512）— ストリームの順番外の送信、
MISSION_REQUEST_LIST — 同じ `mission_type` の MISSION_COUNT 0。外部のデコーダでストリームを確認するには `tools/check_mavlink.py`（pymavlink）を使います。
