# DEVELOPER_GUIDE.md — OpenPlaneProject 開発者ガイド

> 🌐 このページは[ロシア語の原文](../../DEVELOPER_GUIDE.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

ファームウェアの技術マップです。どのファイルが何を担当するか、受信機とセンサーからサーボまでデータがどう流れるか、鎖全体をまとめている符号の規約、Web API の構成、そしてプロジェクトの拡張方法を説明します。C++ を書き、このリポジトリ（`main` ブランチ）で素早く全体像をつかみたい開発者を対象としており、言語や PlatformIO の基礎を学ぶための文書ではありません。

プロジェクトの概要と試作機の状況は [`../README.md`](README.md)、何をどこに接続してどう飛ばすかは [`PILOT_GUIDE.md`](PILOT_GUIDE.md)、今後の計画は [`ROADMAP.md`](ROADMAP.md) にあります。ここで扱うのはコードだけです。アーキテクチャ全体（層、タスク、ステートマシン）は [`ARCHITECTURE.md`](ARCHITECTURE.md)、各クラスのリファレンスは [`reference/`](reference/README.md)、テストは [`TESTING.md`](TESTING.md) にあります。

> プロジェクトは活発に開発中です。ESP32-S3 のベンチは組み上がり、すべてのセンサーで確認済みですが、**オートパイロットはまだ飛行で試験していません**。特定のモジュールに関わる箇所には、その旨を記してあります。コードが何をするのか疑わしいときは、文書ではなくソースを読み直してください。

---

## 目次

1. [層のアーキテクチャ](#層のアーキテクチャ)
2. [FreeRTOS のタスクと制御ループ](#freertos-のタスクと制御ループ)
3. [ファイルリファレンス](#ファイルリファレンス)
4. [符号の規約：IMU からサーボまで](#符号の規約imu-からサーボまで)
5. [RC チャンネルの割り当て、ARM、failsafe](#rc-チャンネルの割り当てarmfailsafe)
6. [センサーデータ](#センサーデータ)
7. [FlightController::update() の詳細](#flightcontrollerupdate-の詳細)
8. [Web ダッシュボードの HTTP API](#web-ダッシュボードの-http-api)
9. [コンソールと診断](#コンソールと診断)
10. [ボードの選択とピン配置](#ボードの選択とピン配置)
11. [新しいセンサーの追加方法](#新しいセンサーの追加方法)
12. [オートパイロットの新しいモードの追加方法](#オートパイロットの新しいモードの追加方法)
13. [フィードバック（土台、未接続）](#フィードバック土台未接続)
14. [新しいボードの追加方法](#新しいボードの追加方法)
15. [ビルド、書き込み、モニターのコマンド](#ビルド書き込みモニターのコマンド)
16. [既知の制限事項](#既知の制限事項)
17. [変更の加え方](#変更の加え方)

---

## 層のアーキテクチャ

ほぼすべてのクラスは、フォルダー `include/<層>/` に分けて置いたヘッダーの中にあります。各ヘッダーは、使うものを自分でインクルードします（`#include "config/Config.h"`、`"hal/II2CBus.h"` など。パスは `include/` からの相対）。`src/main.cpp` は唯一の組み立て地点（composition root）で、すべてのオブジェクトを生成して結び付け、`setup()`/`loop()` を回します。依存は一方向で、下位の層は上位の層について何も知りません。

```
include/
├── config/      Config.h (ピン、すべての設定), Channels.h (チャンネル名),
│                Controls.h (各スイッチの役割（1 チャンネルにつき 1 行）)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (I2C/SPI の上のレジスタデバイス), Rtos
│   ├── esp32/   Esp32Board + Wire/SPI/HardwareSerial/LEDC のラッパー
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (NVS の代わりにフラッシュへ保存する設定)
├── storage/     KeyValueStore, KvPreferences — NVS を使わない設定ストレージ
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  フィードバックの土台。未接続（下の節を参照）
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (気圧センサー 2 つで作るピトー管)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32 のファームウェア（S3、C3、38 ピン）
src/stm32/main.cpp  — STM32H743 のファームウェア（FreeRTOS のタスク、MAVLink）
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — オブジェクト組立     │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — 1 周期の処理順序              │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 モード  │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ 航法、PID          │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, Flash    │
└───────────────────────────────────────────────────────────────────────┘
```

アーキテクチャをきれいに保つためのルール：

- **HAL** は、特定の MCU を知ってよい唯一の層です（`Wire`、`SPI`、`HardwareSerial`、`ledc*`）。それより上はインターフェースだけを相手にします。別の MCU に移るには新しい `hal/<mcu>/<Mcu>Board.h` を作るだけで、残りのコードは変わりません（例は STM32H743 向けの `hal/stm32/`）。
- **センサードライバーはバスを知りません。** 受け取るのは `IRegisterDevice&` で、アドレス付きの I2C デバイスや CS 付きの SPI デバイスは `SensorSelection.h` で作られます。同じ `BMP388_Sensor` が I2C でも SPI でも動きます。
- **共通部分は基底クラスにあります。** 校正、軸の回転、符号、姿勢フィルター、高度と垂直速度、コンパス校正の保存、バスエラーの計数は、`ImuSensorBase`/`BarometerBase`/`MagnetometerBase` にあります。チップのドライバーにあるのは、データシートのレジスタと式だけです。
- **RC と Outputs** は飛行機のことを知りません。iBUS のバイト → チャンネル、PWM の値 → 出力です。
- **Control と Autopilot** はデータに対するロジックで、UART も PWM も Wi-Fi もありません。時間が必要な箇所（フラップ）では、パラメーターで渡します。
- **Coordination**（`FlightController`）は、複数の下位層を同時に見て処理の順序を決める唯一のクラスです。
- **Application**（`main.cpp`）は、`Esp32Board`、デバイス、センサーを生成し、すべてを DI フレームワークなしで手作業で結び付ける唯一の場所です。

---

## FreeRTOS のタスクと制御ループ

| 場所 | 内容 | 周期 |
|---|---|---|
| コア 1、`loop()`（Arduino の loopTask） | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms（500 Hz）、`vTaskDelayUntil` |
| コア 0、`web` タスク | `WebServer::handleClient()` | 2 ms ごと |
| コア 0、`oled` タスク | 2 本目の I2C バスでの SSD1306 の描画 | 200 ms |
| コア 0 | ESP-IDF の Wi-Fi スタック | — |

- ループの周期は、作業のあとの `delay(2)` ではなく `vTaskDelayUntil` で保ちます。そのため周波数は、1 周期にかかった時間に左右されません。長いブロック（コンソールからの校正）のあとは、カウントをやり直し、逃した周期をまとめて取り戻すことはしません。
- ベンチ（ESP32-S3、すべてのセンサー）では、500 Hz で、1 周期あたりの作業は平均 約 0.7 ms、最悪の周期で 約 1.4 ms です。これは 10 秒ごとに `SYS:` の行として出力されます。
- I2C トランザクションのタイムアウトは 5 ms です（Wire の標準は 50 ms）。ノイズで固まったトランザクションがループを長く止めることはありません。
- **タスク間のデータ分離。** Web と OLED は状態（`FlightController`/`Autopilot`/`LoopStats`）を*読むだけ*です。これらは個別の 16/32 ビットのフィールドなので、最悪でも隣り合う周期の値が見えるだけです。ダッシュボードの*コマンド*（`setmode`/`setpid`）は、Web タスクから直接は適用されません。`portMUX` の下で「メールボックス」に置かれ、飛行ループが `WebDebugServer::applyPendingCommands()` で取り出します。
- `Serial`（UART0 → CH343 ブリッジ → 「COM」コネクター）は 4 KB の送信バッファを持ちます。デバッグのフレーム（約 600 文字）は、送信中もループを止めません。

---

## ファイルリファレンス

### `config/`

| ファイル | 担当 |
|---|---|
| `Config.h` | すべてのピン（ボードごとに 1 ブロック：`BOARD_ESP32_S3/C3/CLASSIC`、`BOARD_STM32H743`）と設定：iBUS と信号喪失、舵面の可動量、フラップ、サーボのリバース、IMU とコンパスの取り付け、ARM、failsafe（RTH または滑空）、ピトー管（`PITOT_*`）、オートパイロットの各モードと機能のすべての数値、制御ループ、Wi-Fi、MAVLink、デバッグ |
| `Channels.h` | チャンネル名：`AILERON`、`ELEVATOR`、`THROTTLE`、`RUDDER`、`ARM`、`SWB`、`SWC`、`SWD`、`VRA`、`VRB` |
| `Controls.h` | `BINDINGS` テーブル：各スイッチとノブの役割をチャンネルごとに 1 行で記述し、`static_assert` で検査する |

### `hal/`

| ファイル | 担当 |
|---|---|
| `IBoard.h` | ハードウェアへの入口：`i2c()`、`displayI2c()`（ディスプレイ用の第 2 バス、`nullptr` の場合あり）、`spi()`、`rcUart()`、`gpsUart()`、`telemetryUart()`（MAVLink、`nullptr` の場合あり）、`servo(ServoChannel::*)`（AUX1/AUX2 を含む 7 出力）、`setBuzzer()` |
| `Rtos.h` | FreeRTOS のタスク。ESP32（コア 0）と STM32（優先度）で同じ形で扱える。空きヒープ |
| `II2CBus.h` | I2C バス：`Wire` の形をした基本操作と、補助関数 `writeRegister()`、`readRegisters()`（ちょうど `count` バイト届いたかを確認）、`readRegister()`、`probe()` |
| `ISpiBus.h`、`IUartPort.h`、`IServoOutput.h` | SPI、UART、PWM 出力 1 本（`measurePulseUs()`：実際のパルスの診断） |
| `RegisterDevice.h` | `IRegisterDevice`：「8 ビットレジスタの集合」。`I2cRegisterDevice`（アドレス）、`SpiRegisterDevice`（CS、周波数、データ前のダミーバイト） |
| `esp32/Esp32Board.h` | `IBoard` の実装：`Wire`（センサー）、`Wire1`（ディスプレイ。チップに I2C コントローラが 2 つある場合）、`SPI`、`HardwareSerial` 2 つ、LEDC チャンネル 5 本 |
| `esp32/Esp32I2CBus.h` | 任意の `TwoWire` の上に載る `II2CBus`、タイムアウト 5 ms |
| `esp32/Esp32ServoOutput.h` | LEDC による PWM：50 Hz、14 ビット。ピンが −1 の場合、その出力は配線されていない。ESP32Servo ライブラリは使わない（[制限事項](#既知の制限事項)を参照） |
| `esp32/Esp32SpiBus.h`、`esp32/Esp32UartPort.h` | `SPI` と `HardwareSerial` の薄いラッパー |
| `stm32/*` | STM32H743：`Stm32Board`（＋無線モデム用の UART4）、各バス、PWM タイマー、`Stm32FlashStorage`（設定をフラッシュの 1 セクタに保存し、バックグラウンドタスクが書き込む）、`compat/Preferences.h` |

### `storage/`

| ファイル | 担当 |
|---|---|
| `KeyValueStore.h` | 任意の記憶媒体（`IFlashStorage`）の上に載る、RAM 上の CRC32 付き「名前空間/キー → バイト列」イメージ。同じ値は書き直さない |
| `KvPreferences.h` | `KeyValueStore` の上に載る ESP32 の `Preferences` API |

### `rc/`

| ファイル | 担当 |
|---|---|
| `RcChannelState.h` | 10 チャンネルのスナップショット |
| `RcInput.h` | `clamp()`、`centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → チャンネル：32 バイトのフレーム、CRC、チャンネル値は下位 12 ビット（`& 0x0FFF`）。`isSignalLost()` ＝ フレームがない（またはまだ 1 つも来ていない）∥ スロットルの failsafe 値。フレームカウンタ |

### `control/`

| ファイル | 担当 |
|---|---|
| `ControlCommand.h` | 物理的な符号で表した舵面へのコマンド。スティック、オートパイロット、ミキサーに共通の言語 |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`、`updateFlaps(目標, now)`、`mix(command)` → サーボのリバースを反映した PWM。フラッペロン：エルロンは `flaps ± roll`（マイナス側はエアブレーキ） |
| `FlapsController.h` | フラップの滑らかな展開と格納。時刻は引数で渡す |
| `ThrottleManager.h` | スティックからのスロットル。信号喪失時は `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | SwA スイッチによる ARM（スロットルが最低位置のときの OFF→ON 切り替え＋そのモードに必要なセンサーの検査）、DISARM は即時 |
| `FlightOutputState.h` | 目標の PWM：`aileronLeft`、`aileronRight`、`elevator`、`rudder`、`throttle`、`aux1`（ペイロード）、`aux2`（カメラ） |
| `Beeper.h` | ブザー：`BEEPER` 機能による鳴動、または地上での「機体ロスト」 |
| `FlightOutputs.h` | 出力のテーブル（`outputInfo()`：キー、名前、ピン、必須かどうか、状態フィールド）と、その上でループ処理されるすべて：`begin()`、`write()`、`setFailsafe()`、状態、`printPulseSelfTest()` |
| `FlightController.h` | 各周期での処理の順序、信号喪失（`applyLinkLoss()`）、テレメトリ用のゲッター |

### `autopilot/`

| ファイル | 担当 |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode`（12 モード）、`Feature`、`Knob`、`PilotInputs`、名称 |
| `ControlBinding.h` | `Binding`、ファクトリ `Bind::modes/mode/feature/knob`、`BindingCheck` による検査 |
| `PilotSwitches.h` | 割り当てテーブル → 各周期のモード、機能、ノブ。起動時の配置 |
| `Autopilot.h` | 12 モード、failsafe RTH／滑空、ジオフェンス、ホームポイント、旋回の協調、オートトリム。`update(armed, linkLost, スロットル, スティック)` → `getCommand()`、`applyThrottle()` |
| `Navigation.h` | `Geo`（距離、方位、オフセット）、`Guidance`（針路に向けたロール、円周のベクトル場） |
| `AltitudeSpeedController.h` | 高度はピッチで、対気速度はスロットルで制御（TECS-lite） |
| `LaunchController.h`、`SoaringController.h` | 手投げ発進とソアリングのステートマシン |
| `AutoTrim.h` | オートトリム。NVS／フラッシュに保存 |
| `PidController.h` | PID：D 項はセンサーの変化率（ジャイロ、バリオメーター）から取り、アンチワインドアップ付き。ARM していない間は積分器を固定 |
| `feedback/*` | **下準備のみで未接続**：適応フィードバック、離陸と着陸（[フィードバック](#フィードバック土台未接続)を参照） |

### `sensors/`

| ファイル | 担当 |
|---|---|
| `SensorInterface.h` | `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` の各インターフェースとデータ構造 |
| `SensorSelection.h` | どのチップをコンパイルするか（`#define SENSOR_*`、ビルドフラグで上書き可能）と、どのバスに接続されているか（`SELECTED_*_DEVICE(board)`） |
| `SensorMounting.h` | チップの軸を機体の軸へ回転する（時計回りに 0/90/180/270°）。コンパス用、および取り付け校正をしていない IMU 用 |
| `imu/ImuOrientation.h` | 「チップ軸 → 機体軸」の行列で表した IMU の取り付け：`IMU_ROTATION_CW_DEG` から、または 3 つの姿勢（水平、機首上げ、右翼下げ）から検証付きで求める。NVS に保存 |
| `imu/ImuSensorBase.h` | IMU に共通の処理：ジャイロの校正＋飛行前チェック（静止、1g、「上」が取り付けと一致）、取り付け校正（`calibrateOrientation()`）、スケール、回転、航空機の符号、バスエラー |
| `imu/AttitudeEstimator.h` | ロール／ピッチの相補フィルタ、ヨーの積分 |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500（チップは WHO_AM_I で判別）：±2000°/s、±16g、DLPF 約 41 Hz、1 kHz。**テストベンチで確認済み** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P：±2000°/s、±16g、1 kHz、UI フィルタ 50 Hz。実機では未確認 |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X：±2000°/s、±16g、960 Hz、LPF1/LPF2。I2C 0x6A/0x6B または SPI。実機では未確認 |
| `imu/ICM45686_Sensor.h` | ICM-45686：±2000°/s、±16g、1.6 kHz、間接レジスタ IPREG 経由のローパスフィルタ。I2C 0x68/0x69 または SPI。実機では未確認 |
| `baro/BarometerBase.h` | 気圧計に共通の処理：新しいサンプルだけを取得、高度、ローパスフィルタ経由の垂直速度、基準の校正、エラー |
| `baro/BMP388_Sensor.h` | I2C または SPI（SPI のダミーバイト付き）の BMP388、Bosch の補正、データ準備完了フラグによる読み出し。**テストベンチで確認済み（I2C）** |
| `baro/BME280_Sensor.h` | BME280/BMP280、Bosch の補正（§8.1）。実機では未確認 |
| `baro/SPL06_Sensor.h` | SPL06-001：データシートの係数と式、32 Hz ×16。I2C 0x76/0x77 または SPI。実機では未確認 |
| `baro/BMP581_Sensor.h` | BMP581：BMP5_SensorAPI の手順、16×/2×、IIR。I2C 0x46/0x47 または SPI。メインの気圧計（既定のベンチセット）としてもピトー管としても使える。実機では未確認 |
| `mag/MagnetometerBase.h` | コンパスに共通の処理：50 Hz のポーリング、NVS への hard-iron 校正、軸の回転、方位、エラー |
| `mag/QMC5883P_Sensor.h` | QMC5883P、0x2C。**テストベンチで確認済み** |
| `mag/QMC5883L_Sensor.h` | QMC5883L、0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309、0x7C：±8 G、200 Hz。実機では未確認 |
| `gps/UbloxM10_Gps.h` | u-blox M10：CFG-VALSET による設定（115200 ボー、10 Hz、NAV-PVT、NMEA なし）、NAV-PVT の解析。テストベンチでは未接続 |
| `airspeed/AirspeedSensor.h` | 対気速度センサーのインターフェース：差圧、IAS、TAS、密度 |
| `airspeed/PitotDualBaroAirspeed.h` | 自作のピトー管：管内の BMP581＋胴体の気圧計。地上でゼロ点を取り、ローパスフィルタ、静圧から密度を求め、故障を検出 |

### `telemetry/` とアプリケーション

| ファイル | 担当 |
|---|---|
| `DebugLogger.h` | チャンネルごとのログ（`LogSettings.h`）：チャンネルごとに専用の行、チャタリング許容値、モードを持つ。メニューを開いている間は出力しない |
| `DebugConsole.h` | ポートモニターのテキストメニュー（`h`）とホットキー（`l`/スペース/`s`/`i`/`o`/`m`/`p`/`b`）。ログ設定はメニューを抜けるときに NVS へ書き込み、ARM していないときだけ書く |
| `LogSettings.h` | ログのチャンネル（STAT、RC、OUT、ATT、AP、ALT、MAG、GPS、IMU、NAV、SYS）とそのモード：オフ／変化時／常時。NVS に保存 |
| `WebDebugServer.h` | アクセスポイント、ルート、JSON `/api/status`、コマンド用のメールボックス。コア 0 上の専用タスク |
| `WebDashboardPage.h` | ダッシュボードの HTML/JS を 1 つのリテラルにしたもの。チャンネル／出力／センサーの行はブラウザーが JSON から組み立てる |
| `OledDisplay.h` | `II2CBus` の上で U8g2 を介して動く SSD1306。専用タスク（`Rtos`） |
| `MavlinkCodec.h`、`MavlinkTelemetry.h` | QGroundControl / Mission Planner 向けの MAVLink 2：フレーム、ストリーム、PID パラメータ、地上からのモード切り替え |
| `LoopStats.h` | 1 秒あたりの周波数、平均周期時間と最悪周期時間（OLED）、および前回の読み出し以降の最悪値（`takePeakUs()`、SYS 行） |
| `src/main.cpp` | ESP32：オブジェクトの生成、`setup()`、`vTaskDelayUntil` を使う `loop()` |
| `src/stm32/main.cpp` | STM32H743：同じオブジェクト、MAVLink、SD カードのブラックボックス、`flight`/`storage`/`oled`/`bbox` の各タスク |
| `src/stm32/sd_msp.cpp`、`src/stm32/bootloader.cpp` | STM32H743：`HAL_SD_Init` 用の SDMMC1 のピンとクロック。コンソールの `D` キーで USB DFU ブートローダーへ再起動 |

---

## 符号の規約：IMU からサーボまで

経路全体で符号の体系を 1 つにそろえています。そのため、スティックとオートパイロットは必ず同じ向きに舵面を動かし、各サーボの向きは 1 か所だけで決まります。

**1. センサー軸 → 機体軸。** `ImuSensorBase` は、`ImuOrientation` 行列（body = R · chip）でチップの軸を機体の軸へ回転します。X は機首方向、Y は左、
Z は上です。行列は次のいずれかから得ます。

- **取り付け校正**（コマンド `o`、NVS に保存）：基板の向きは自由です。姿勢は
  3 つあり、「水平」で Z 軸が決まり（水平線も同時に決まる。加速度計のゼロ点オフセットもここに含まれる）、「機首上げ」で X 軸（「上」のうち Z に垂直な成分）が決まり、「右翼下げ」で Y 軸が決まります。手順 2 の機首と手順 3 の機首（Y × Z）は約 25° 以内で一致する必要があります。一致しない場合はパイロットが違う方向へ傾けたことになり、校正は却下されます。最終結果は
  2 つの推定値の平均です。300 通りのランダムな取り付けで検証済みです（`test/test_imu_orientation`、誤差 < 0.1°）。
- それ以外の場合は `Config::IMU_ROTATION_CW_DEG` から求めます（チップを上にした基板。値は、機首を「12 時」としたときに*チップ*の X 軸がどちらを向くかを表す）。水平線は電源投入時の姿勢です。

ジャイロを校正するたび（電源投入時、`i`）に**飛行前チェック**を行います。ジャイロのノイズ < 0.5 °/s（静止。静止時は約 0.08）、|a| ≈ 1g、「上」が保存済みの値から
45° 以内（基板が動かされていない）。通らなかった場合、
`ImuSensor::getPreflightProblem()` ≠ nullptr となり、`ArmingManager` はスタビライズ付きのモードを ARM せず、`Autopilot::imuReady()` = false になります（リンク喪失時の滑空を含め、すべてのモードで補正量はゼロ）。

> 現在の GY-521（MPU6500 のクローン）では、チップが印刷された矢印に対して 90°
> 回転した向きではんだ付けされています。シルクの X 矢印 = チップの Y 軸です。そのため、取り付け校正をしない場合は `IMU_ROTATION_CW_DEG = 90` とします。配置を変えたあとの確認：機首上げ → P が正に増える、右翼下げ → R が正に増える。

**2. 角度と角速度（`ImuData`）：航空機の符号**

| 量 | 「+」の意味 |
|---|---|
| `roll`、`gyroX` | 右翼下げ |
| `pitch`、`gyroY` | 機首上げ |
| `yaw`、`gyroZ` | 機首右（上から見て時計回り） |

**3. コマンド（`ControlCommand`、舵角を µs で表す、±500 = 全ストローク）**

| フィールド | 「+」の意味 | スティックから |
|---|---|---|
| `roll` | 右ロール（右エルロン上げ、左下げ） | CH1：2000 = 右 |
| `pitch` | 機首上げ（エレベーター上げ） | CH2 は符号が逆：2000 = 自分から遠ざける = 機首下げ |
| `yaw` | 機首右（ラダーと前輪が右） | CH4：2000 = 右 |
| `flaps` | フラップ下げ（左右のエルロンとも下げ） | SwB（CH6）：0 または `FLAPS_DEPLOYED_US`。`FLAPS_TRANSITION_MS` かけて滑らかに移行 |

PID は `誤差 = 目標 − 実測` を計算します。右ロール（roll > 0）→ ロールのコマンドが負 → 機体が水平に戻ります。オートパイロットの補正は、ミキサーの**前**でスティックのコマンドに同じ符号で加算されます。

**4. コマンド → PWM。** `ControlMixer::mix()` は各舵面の後縁の舵角を計算し（エルロン：下げ = 「+」、左 = `flaps + roll`、右 = `flaps − roll`、エレベーター：上げ = 「+」、ラダー：右 = 「+」）、PWM `1500 ± 舵角` に変換します。
`Config::*_REVERSED = true` のサーボでは符号を反転します。デフォルト値は、スティックに対するファームウェアのこれまでの挙動を再現します。組み立て済みの機体での確認は、
[`PILOT_GUIDE.md`](PILOT_GUIDE.md) の飛行前チェックリストにあります。リバースは
`Config.h` で変更してください。**送信機側では変更しないでください**。そうしないとスティックとオートパイロットの向きが食い違います。

---

## RC チャンネルの割り当て、ARM、failsafe

出典は `include/config/Channels.h` です。送信機 FS-i6（10 チャンネル、モード 2）＋受信機 FS-iA6B、iBUS 115200。

| チャンネル | 送信機の操作部 | 名前 | 用途 |
|---|---|---|---|
| CH1 | 右スティック ←→ | `AILERON` | ロール |
| CH2 | 右スティック ↑↓ | `ELEVATOR` | ピッチ |
| CH3 | 左スティック ↑↓ | `THROTTLE` | スロットル、フルストローク。< 950 は受信機の failsafe |
| CH4 | 左スティック ←→ | `RUDDER` | ラダー＋ステアリングホイール（サーボ 1 つ） |
| CH5 | SwA | `ARM` | ≥ 1750 で ARM（FS-i6 ではスイッチを下、手前に倒した状態） |
| CH6 | SwB | `SWB` | 既定ではフラップ（≥ 1750 で展開） |
| CH7 | SwC（3 ポジション） | `SWC` | 既定ではモード：< 1250 は MANUAL、1250–1749 は STABILIZE、≥ 1750 は AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | 既定では RTH |
| CH9 | VrA | `VRA` | 既定ではスタビライズの強さ |
| CH10 | VrB | `VRB` | 既定では巡航速度 |

CH6〜CH10 は、`include/config/Controls.h` に 1 行書くだけで割り当てられます（[AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#1-行で機能を割り当てる)）。

**ARM**（`ArmingManager`）：スイッチが OFF→ON になり、スロットルが
`THROTTLE_LOW_US` 未満で、現在のモードに必要なセンサー検査を通過していること。そうでない場合は理由を Serial に出して拒否し、新たに OFF→ON の操作が必要です。スイッチを ON にしたまま基板の電源を入れても ARM されません。OFF にするとすぐ DISARM します。ARM されていない間、ESC へのスロットルは強制的に
`PWM_MIN` になります。

**リンク喪失**（`IBusReceiver::isSignalLost()`）：

1. `RX_TIMEOUT_US`（500 ms）を超えてフレームが来ない：配線の断線、または受信機の電源喪失です。電源投入後に最初のフレームが来るまでもリンク喪失とみなします。チャンネルの既定値（すべて 1500）を送信機のコマンドと取り違えないためです。
2. スロットル < `RX_FAILSAFE_THROTTLE_US`（950）：送信機で設定した failsafe です。
   **FS-iA6B は送信機を失ってもフレームの送出を止めず**、最後の値を繰り返します（テストベンチで確認済み）。そのため、送信機側で failsafe を設定していないとリンク喪失を検出できません。設定方法は `PILOT_GUIDE.md` にあります。

リンク喪失時の動作（`FlightController::applyLinkLoss()`）：

- **機体が ARM 済みで、GPS とホームポイントがある**（`FAILSAFE_RTH`）：モーターを使って**ホームへ帰還**し、ホームの上空で旋回します。OLED には
  `FSRTH`、ログには `FAILSAFE_RTH` と表示されます。
- **機体が ARM 済みで、GPS がない**：**滑空**し、モーターは
  `FAILSAFE_THROTTLE` になります。`Autopilot` はどのモードでも（MANUAL でも）ロール `FAILSAFE_GLIDE_ROLL_DEG`（0 は直進、10〜20° はパイロットの上空での旋回）とピッチ `FAILSAFE_GLIDE_PITCH_DEG`（−3°。モーターなしで速度を失わないため）を保ち、フラップは格納されます。OLED には `GLIDE`、ログにはモード
  `FAILSAFE_GLIDE` と表示されます。
- **ARM していない**（地上）、または IMU が応答しない：舵面はニュートラルになります。
- モードと機能はスイッチでは切り替わらず、センサーは読み取りを続けます。
  ARM は解除されません。リンクが回復すると、機体は再びスティックと選択中のモードに従います（自動離陸と手投げ発進は最初からやり直しになります）。

---

## センサーデータ

構造体は `include/sensors/SensorInterface.h` にあります。

### `ImuData`

| フィールド | 単位 | 意味 |
|---|---|---|
| `gyroX`、`gyroY`、`gyroZ` | °/s | 機体軸での角速度。航空機の符号（上記参照） |
| `accelX`、`accelY`、`accelZ` | g | 機体軸での加速度：X は機首方向、Y は左、Z は上 |
| `roll`、`pitch` | ° | 相補フィルタ（α = 0.98、τ ≈ 0.1 秒）。加速度計から求めた角度でそのまま開始する |
| `yaw` | ° | ジャイロの積分値。ゆっくりドリフトする。初期値はコンパスの方位 |
| `temperature` | °C | ダイ温度（MPU6050 または MPU6500 用の式） |
| `timestamp` | µs | 読み取り時点の `micros()` |

IMU の校正（起動のたび、および `i` コマンド）：2 秒間静止し、ジャイロ →
ゼロ点オフセット、加速度計 → **現在の姿勢が水平になる**。

### `BarometerData`

| フィールド | 単位 | 意味 |
|---|---|---|
| `pressure` | Pa | 気圧 |
| `temperature` | °C | センサーの温度 |
| `altitude` | m | **校正点（起動時）を基準とした**高度。式は `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | 実際のサンプル（50 Hz）に対する高度の微分。τ = 0.5 秒のローパスフィルタ経由 |
| `timestamp` | µs | 最後の新しいサンプルの時刻 |

### `MagData`

| フィールド | 単位 | 意味 |
|---|---|---|
| `magX`、`magY`、`magZ` | µT | hard-iron 校正後の磁場。機体軸で表す（`MAG_ROTATION_CW_DEG`） |
| `headingDegrees` | °（0..360） | `atan2(magY, magX)`。傾き補正なし。角度を数える向きは、組み立て済みの機体ではまだ確認していない |
| `timestamp` | µs | 読み取り時点（50 Hz） |

### `GpsData`

| フィールド | 単位 | 意味 |
|---|---|---|
| `latitude`、`longitude` | ° | UBX-NAV-PVT から取得 |
| `altitude` | m | 海抜（hMSL） |
| `groundSpeed`、`heading` | m/s、° | 対地速度と対地針路 |
| `numSatellites`、`fixType` | — | 0 = 測位なし、2 = 2D、3 = 3D |
| `horizontalAccuracy`、`verticalAccuracy` | m | モジュールによる精度の推定値 |

**`isAvailable()` の意味。** I2C センサーの場合：`begin()` で応答があり、**かつ**
直近の読み取りが連続して失敗していないこと（MPU は約 0.1 秒、気圧計とコンパスは約 0.5 秒応答がない場合）。読み取りに失敗してもデータはゴミで上書きされません。以前の値が残り、エラーカウンタが増えます（`s` コマンドで確認できます）。GPS の場合：有効な NAV-PVT が少なくとも 1 つあり、最新のものが `GPS_TIMEOUT_US` より古くないこと。

**センサーがない場合**（`nullptr` または `isAvailable() == false`）、`Autopilot` は補正を一切行わず、機体は MANUAL と同様に操縦されます。`main.cpp` は応答したセンサーだけを校正します。

---

## FlightController::update() の詳細

`loop()` から 2 ms ごとに呼ばれます。順序がそのまま優先度です。

1. **`receiver.update()`**：溜まった iBUS のバイト列を解析します。
2. **スイッチ**：`switches->update(rc)`。リンクが生きているときだけ実行します（failsafe フレームではチャンネルがスイッチの状態を反映しません）。対象はモード（変化したときだけ）、機能、ノブです。
3. **パイロットのスロットル**：`throttle.update(rc, receiverFailsafe)`。
4. **スティック**：`mixer.fromSticks(rc)` × `Knob::RATES`。フラップは
   `mixer.updateFlaps(target)`（ブレーキ、スイッチ、ノブ。リンクがなければ 0）。
5. **センサーとオートパイロット**：`autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   を**常に**実行します。リンクがなくても同じです。角度フィルタを止めてはいけないためです。ARM していない間も PID は動き（舵面が傾きに反応するので、机の上での確認に便利）、積分器だけはゼロに保たれます。リンクがなく ARM
   している場合は failsafe RTH または滑空になります。
6. **ブザー**：`Beeper`。
7. **リンク喪失**：`applyLinkLoss()`。ARM 中は舵面とスロットルがオートパイロットの
   failsafe コマンドに従い、そうでなければニュートラルでモーター停止となり、
   `return` します。これより下のすべてに対して絶対的に優先されます。
8. **ARM**：`arming.update(rc, false)`。
9. **コマンド**：`autopilot->getCommand()`。スタビライズ付きのモードではスティックは目標角度を表し、最終的な舵面コマンドはオートパイロットが出します。
10. **ミキサー**：`mixer.mix(command)` → エルロン（フラップ＋ロール）、エレベーター、ラダーの PWM。リバースも考慮されます。
11. **スロットル**：`autopilot->applyThrottle(pilotThrottle)`。パイロットのスロットル、オートパイロットのスロットル、または両者の大きいほう（自動離陸）。その後、
    ARM していない場合や `MOTOR_KILL` の場合は強制的に `PWM_MIN` にします。どのモードも ARM をすり抜けてスロットルを通せないよう、この検査は最後に置いています。
12. **AUX**：ペイロード（`PAYLOAD_DROP`）とカメラ（`CAMERA_TILT`、`CAMERA_STAB`）。
13. **`outputs.write(output)`**：7 つの出力へ PWM を出します。

---

## Web ダッシュボードの HTTP API

実装は `include/telemetry/WebDebugServer.h` です。アクセスポイント：SSID
`OpenPlane-Debug`、パスワード `12345678`、アドレス `http://192.168.4.1`。

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached`：オブジェクトがビルドに含まれている。`available`：センサーが実際に応答している。データのフィールドは `available: true` のときに**限り**追加されます。
- `outputs.*.attached`：MCU が LEDC チャンネルとピンを確保した。物理的なサーボが接続されているかどうかはソフトウェアからは分かりません（パルスの確認にはコンソールの `p` コマンドを使う）。
- `rollCorr`/`pitchCorr`：オートパイロットの最終コマンドからスティック分を引いた値（µs）。`throttleCorr`：オートパイロットのスロットル（%）。スロットルがパイロットにある間は 0。
- `nav`：ナビゲーション情報。ホーム、ホームまでの距離と方位、針路と目標針路、ナビゲーションに使う速度（ピトー管／GPS）、ジオフェンス、失速。`features`：有効になっているスイッチの機能。

### `POST /api/setmode`

`{ "mode": 1 }`：`AutopilotMode` の番号。`0` MANUAL、`1` STABILIZE、`2`
AUTO_TAKEOFF、`3` ALT_HOLD、`4` ACRO、`5` CRUISE、`6` LOITER、`7` RTH、`8`
LAUNCH、`9` AUTO_LAND、`10` SOARING、`11` RESCUE。このモードは、パイロットがモードスイッチを切り替えるまで保持されます。

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }`：`kpRoll`、`kiRoll`、`kdRoll`、
`kpPitch`、`kiPitch`、`kdPitch` のうち任意のフィールドを指定できます。省略したものは以前の値のままです。

どちらのコマンドも、飛行ループが次の周期で適用します（
[FreeRTOS のタスク](#freertos-のタスクと制御ループ)を参照）。

### `GET /`

HTML ダッシュボード：10 チャンネルのバー、ARM／リンク、出力、センサー、モードボタン、PID のフォーム。200 ms ごとに `/api/status` を問い合わせます。

---

## コンソールと診断

ポートモニターは 115200、コネクタは「COM」です。実装は `DebugConsole` と
`DebugLogger` です（[リファレンス](reference/telemetry.md)）。キーは押すとすぐに動作し、Enter は不要です。校正と `p` はループを止めるため、ARM していないときだけ使えます。

| キー | 動作 |
|---|---|
| `h` / `?` | メインメニュー |
| `l` | 「ログに何を出力するか」のメニュー（チャンネル、モード、周期） |
| スペース | ログの一時停止／再開 |
| `s` | 全センサーの `printStatus()`：データ、バスのエラーカウンタ、校正、飛行前チェック |
| `i` | ジャイロの校正＋飛行前チェック（2 秒間静止） |
| `o` | 3 つの姿勢による IMU 取り付けの校正。NVS に保存 |
| `m` | コンパスの校正（15 秒間回転）。NVS に保存 |
| `p` | 出力のセルフテスト：各ピンの実際のパルスと期待値を比較 |

ログはチャンネルに分かれており（`STAT`、`RC`、`OUT`、`ATT`、`AP`、`ALT`、`MAG`、
`GPS`、`IMU`、`SYS`）、それぞれに「オフ／変化時／常時」のモードがあります。設定は
NVS に保存され、メニューを閉じるときに書き込まれます（ARM していないときのみ）。既定では `STAT`（変化時）と `SYS`（10 秒に 1 回）が有効です。

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (10 秒間での最悪値) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

すべてのチャンネルのフォーマットは[リファレンス](reference/telemetry.md#debuglogger)にあります。

---

## ボードの選択とピン配置

| コマンド | `board` | マクロ | 状況 |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8（`qio_opi`、16 MB） | `BOARD_ESP32_S3` | **メイン、既定。** すべてのセンサーを接続してテストベンチで確認済み |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | 古いプロトタイプ。手動操縦で飛行した |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | テストベンチ用。ピン配置は実機では未確認 |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6：完全なファームウェア＋MAVLink＋SD ブラックボックス。素の基板で確認済み（[下記](#stm32h743)） |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | DevEBox H743 でも同じ：コンソールは USB CDC、ファームウェアは DFU で書き込む |

| 用途 | ESP32-S3（テストベンチ） | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| エルロン左／右 | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| エレベーター／ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| ラダー | GPIO18 | —（ピンなし） | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| センサーの I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED の I2C SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40（UART2） | GPIO9 ⚠️ / なし（UART0） | GPIO4 / GPIO17（UART2） |
| Serial | UART0 → 「COM」コネクタ | USB-CDC | UART0 |

- **ESP32-S3 N16R8：** GPIO33–37 はオクタル PSRAM、26–32 はフラッシュ、19/20 は
  USB、43/44 は Serial が使用し、48 は RGB LED です。0/3/45/46 は strapping
  ピンです。
- **ESP32-C3：** GPIO4/5 のエルロンは S3 と入れ替わっています。フルセットには、ピンが足りません。BMP388 の CS と GPS の RX は strapping ピン上にあり、
  GPS には TX がありません（受信専用で UBX-CFG は送れない）。詳細は `Config.h`
  にあります。

### STM32H743

STM32H743VIT6（Cortex-M7 480 MHz、フラッシュ 2 MB、RAM 1 MB）は**完全なファームウェア**を実行します。ESP32-S3 と同じセンサー、オートパイロット、スイッチ、コンソール、ディスプレイに加えて、MAVLink テレメトリと SD カードのブラックボックスがあります。ビルドでき、cppcheck と共通コードのすべてのネイティブテストに合格します。実機で確認したのは**センサーなしの DevEBox H743
基板**です：起動、USB 経由のコンソール、SD カード、ブラックボックス（[TESTING.md](TESTING.md#stm32-ボード上のテスト)）、さらに iBUS、ARM、サーボとモーターへの PWM（送信機からの手動モードでの操縦。動画あり）。STM32 のセンサーはまだテストベンチを待っています。主力の飛行用ボードは ESP32-S3 です。

- **HAL**：`include/hal/stm32/`。`Stm32Board`（`Esp32Board` と同じ API に
  `telemetryUart()` を追加）、`Stm32I2CBus`、`Stm32SpiBus`、`Stm32UartPort`、
  `Stm32ServoOutput`（`HardwareTimer` によるハードウェア PWM。1 つのタイマーで複数の出力を担当）。詳しくは [reference/hal.md](reference/hal.md#stm32h743-向けの実装)。
- **設定と校正**：NVS ではなく、フラッシュの最後のセクタにある
  `KeyValueStore`（`include/storage/`、`hal/stm32/Stm32FlashStorage.h`）。プロジェクトのコードは従来どおり `#include <Preferences.h>` と書きます。env
  `stm32h743` では `include/hal/stm32/compat/` が `-I` に入っており、そこに同じ
  API の `Preferences` があります。イメージには CRC32 が付き、壊れたイメージ（消去中に電源が落ちた場合）は空として読まれます。フラッシュへの書き込みはバックグラウンドのタスクで行います。128 KB のセクタの消去には数秒かかりますが、そのセクタはバンク 2 にあり、コードはバンク 1 から実行されるため、飛行タスクは止まることなくバックグラウンドタスクを押しのけます。
- **タスク**：STM32duino FreeRTOS ライブラリの FreeRTOS、シングルコア、優先度によるプリエンプション（`hal/Rtos.h`）。`flight`（5）は飛行ループ、
  MAVLink、ログ、コンソール。`oled`（1）と `storage`（1）はバックグラウンド。
  `bbox`（2）は SD カードへのブラックボックス書き込みです。
- **SD カードのブラックボックス**：SDMMC1、4 ビット、24 MHz
  （`hal/stm32/Stm32SdCard.h`、ピンは `src/stm32/sd_msp.cpp`）。カードは通常の
  FAT32 のままです。あらかじめ作成した `BLACKBOX.BIN` ファイルを置いておき、ファームウェアはその内部に生のブロックを書き込み、ファイルシステム自体には触れません（`storage/Fat32File.h` は読み取り専用）。カードの準備とデータの取り出しは [BLACKBOX.md](BLACKBOX.md#sd-カードstm32h743) を参照してください。
- **テレメトリ**：Wi-Fi ダッシュボードの代わりに、UART4 の MAVLink 2
  （`telemetry/MavlinkTelemetry.h`）を使います。QGroundControl / Mission
  Planner から、地上でモードや PID を変更できます。詳しくは
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#地上局wi-fi-ダッシュボードと-mavlink)。
- **ピン配置**：`Config.h` の `BOARD_STM32H743` ブロック。ピンは WeAct
  MiniSTM32H743VITx の空きピンから選び、STM32duino の表と照合しています。

| 用途 | STM32H743 | ペリフェラル |
|---|---|---|
| エルロン左／右 | PA0 / PA1 | TIM2_CH1 / CH2 |
| エレベーター／ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| ラダー | PD14 | TIM4_CH3 |
| AUX1（ペイロード）／AUX2（カメラ） | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX（TX は予備） | PE7（PE8） | UART7 |
| センサーの I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED の I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU／気圧計 | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| 無線モデム MAVLink RX / TX | PD0 / PD1 | UART4 |
| ブザー | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743（MCUDEV）**：env `stm32h743-devebox`。コードは同じで、コアは専用のバリアントを使い、コンソールは USB-C 経由の仮想 COM ポート（CDC）なので USB-UART は不要です。最初の書き込みは、内蔵ブートローダー（DFU）を使って
  USB 経由で行います。
  1. Windows：「STM32 BOOTLOADER」用の WinUSB ドライバーを一度だけインストールします（[Zadig](https://zadig.akeo.ie)：DFU in FS Mode → WinUSB → Install
     Driver）。
  2. ピン **BT0**（BOOT0）を **3V3** に配線で接続し、**RST** を押して離します。基板が DFU モードになります（DevEBox には BOOT0 ボタンがありません）。
  3. `pio run -e stm32h743-devebox -t upload`（`upload_protocol = dfu`）。
  4. BT0 の配線は外して構いません。ファームウェアは自動的に起動します。

  以降は配線が不要です。コンソールの **`D`** キー（どのメニューからでも可、ARM
  中は不可）で、基板をブートローダーへ再起動します。RAM にマーカーを残す → リセット
  → クロック設定の前にシステムメモリへジャンプ（`src/stm32/bootloader.cpp`）。
  H7 では、動作中のファームウェアから直接ジャンプするとハングします。基板で確認済みで、そのため 2 段階にしています。コンソール（USB CDC）を開いておく必要があります。基板が応答しない場合は、BT0 を配線したまま RST を押してください。
- **エントリポイント**：`src/stm32/main.cpp`（ESP32 のビルドでは
  `build_src_filter` で除外）。オブジェクトは `src/main.cpp` と同じで、`loop()`
  の代わりにタスクがあり、`vTaskStartScheduler()` は `setup()` の最後にあります。
- **基板の初回電源投入：** `pio run -e stm32h743 -t upload`（ST-Link）、モニターは
  USB-UART 経由で LPUART1 に接続します。`b` でバス上にセンサーが見えるか、`s` でセンサーの状態、`p` で出力のパルス（プロペラは外す）を確認し、そのあと送信機と、無線モデム経由の QGroundControl を試します。

---

## 新しいセンサーの追加方法

### A) 既存カテゴリの別のチップ（IMU、気圧計、コンパス）

共通部分は基底クラスにすでに書かれているので、チップのドライバーは小さく済みます。

1. `include/sensors/<category>/<Name>_Sensor.h` を作成し、`ImuSensorBase` /
   `BarometerBase` / `MagnetometerBase` を継承します。コンストラクタは
   `IRegisterDevice&` を受け取ります。ドライバーは I2C か SPI かを知りません。
2. 次を実装します。
   - `begin()`：`device.begin()`、チップ ID の確認、レジスタの書き込み、
     `setAvailable(true/false)` の呼び出し。
   - IMU：`readSample()`（チップ軸での生の accel/gyro/temp）、
     `accelLsbPerG()`、`gyroLsbPerDps()`、`temperatureC()`。
   - 気圧計：`isNewSampleReady()`（データ準備完了フラグ、または単に `true`）と
     `readSample()`（気圧は Pa、温度は °C）。ポーリング周期は基底クラスのコンストラクタで指定します。
   - コンパス：`readRaw()`（チップ軸での X/Y/Z）と `lsbPerMicroTesla()`。校正用の NVS 名前空間の名前は基底クラスのコンストラクタで指定します。
3. SPI でデータの前にダミーバイトが必要なチップや、特別な周波数が必要なチップでは、`BMP388_Sensor` のように静的ファクトリ `spiDevice(bus, cs)` を追加します。
4. `SensorSelection.h` に分岐を追加します：`#define SENSOR_<CATEGORY>_<NAME>`、
   `using Selected... = ...;`、`#define SELECTED_..._DEVICE(board) ...`
   （`I2cRegisterDevice(board.i2c(), address)` または SPI のファクトリ）。センサーを変えても `main.cpp` には触れません。
5. ファイルを編集せずに、フラグで新しいセンサーのビルドを確認します：
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`。続いて
   3 つの環境すべて、その後に実機で確認します。

### B) 新しいカテゴリ

1. データ構造とインターフェースは `SensorInterface.h` に、`GpsSensor`/`GpsData`
   にならって書きます。
2. カテゴリに共通のロジック（フィルタ、校正）がある場合は、`BarometerBase` にならった基底クラスを用意します。
3. `Autopilot` のコンストラクタには null を許容するポインタを渡し（センサーがなければ何の影響もなく、クラッシュもしない）、`GET /api/status` には
   `attached`/`available` の組を持つフィールドを追加します。

### 新しいバスや周辺機器

`include/hal/` に新しいインターフェースを置き、`include/hal/esp32/` と
`include/hal/stm32/` に実装を置いて、`IBoard` 経由でアクセスします。

---

## オートパイロットの新しいモードの追加方法

1. `enum AutopilotMode`（`autopilot/AutopilotTypes.h`、`MODE_COUNT` の前）に値を追加し、`AutopilotNames::mode()` / `modeShort()` に名前と短い名前（OLED 用、最大 5 文字）を追加します。
2. ハンドラー `run<Mode>()` を作り、`Autopilot::runMode()` に分岐を追加します。初期目標（針路、高度、旋回円の中心）は `initializeMode()` に書きます。モードは
   `desiredRoll`/`desiredPitch` を設定し、`stabilizeOrManual()`（IMU がなければ舵面はパイロットが操作）または `stabilizeOrNeutral()`（IMU がなければニュートラル）を呼びます。必要なセンサーがない場合は、クラッシュではなく安全な動作にします。積分器は `armed` のときだけ蓄積します。
3. スロットル：`throttleMode`（`PILOT` / `AUTO` / `AT_LEAST`）と
   `autoThrottlePct`、または `autoThrottle()`（ノブ／ピトー管から得る巡航スロットル）。このとき `FlightController` は変更しません。
4. 送信機側は `config/Controls.h` に 1 行加えるだけです（`Bind::mode(Channels::SWD, MODE_NEW)`）。ダッシュボードと MAVLink はモードを番号で拾います。MAVLink の場合は、`MavlinkModes::toCustomMode()` /
   `fromCustomMode()` で最も近い ArduPlane のモードに対応付けます。
5. モードの ARM にセンサーが必要な場合は、`ArmingManager` を変更します。
6. テスト：各センサーに対する反応は `test/native/test_autopilot_modes`、閉ループの飛行は `test/native/test_sim` のシナリオ（機体モデル `helpers/PlaneSim.h`、テスト用の環境 `helpers/SimHarness.h`）で確認します。その後、プロペラなしで机の上で確認します。舵面は、傾けると水平に戻す方向へ反応する必要があります。
7. [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md) にセクションを追加します。

---

## フィードバック（土台、未接続）

`include/autopilot/feedback/` はオートパイロットの次のステップです。
**`FlightController` も `Autopilot` も `main.cpp` も、これらのファイルをインクルードしていません**。飛行試験用のプロトタイプはまだなく、ファームウェアはこれらなしで動作します。検証は、閉ループのシミュレーション（`test/test_feedback/`）を基板上でそのまま実行して行います。

### 目的

現在の `Autopilot` は角度に対する PID です：誤差 × ゲイン = 舵量。その結果が機体でどうなったかは知らず、ゲインも 1 つの速度でしか正しくありません。低速では舵の効きが弱く PID は補正不足になり、高速では補正過多になります。フィードバックは、
**機体の応答**でループを閉じます。

- 舵を振ったのに機体が必要より遅く回っている → 届くまで足す。
- 必要な舵量は飛行中に測り、速度に応じて計算し直す。
- 機体が逆方向に回っている → 符号が入れ違っているので、反転して確認する。
- 角度は水平になったのに速度が落ちている → 機体が失速しないよう、スロットルを上げて機首を下げる。
- 離陸と着陸 → センサーの示す状態に応じて、段階ごとに進める。

### モジュール

| ファイル | 内容 |
|---|---|
| `FlightSnapshot.h` | 1 周期でフィードバックが機体について把握しているすべて。唯一の入力であり、各モジュールはセンサーや RC を直接読まないので、シミュレーションやログでも動かせる |
| `FeedbackOutput.h` | 1 周期の出力：軸ごとの舵角、その軸が有効かどうか、軸の符号、スロットル（指定／下限）、理由 |
| `FeedbackConfig.h` | すべての定数（接続時に `Config.h` へ移す） |
| `SpeedEstimator.h` | 速度（ピトー管 > GPS）と、IMU から求めた前後方向の加速度：`dV/dt = g·(ax − sin θ)`。速度センサーがなくても「速度が落ちている」ことが分かる |
| `AirborneDetector.h` | 空中／地上の判定：学習、積分の蓄積、失速の検出は、飛行中にだけ意味がある |
| `ControlEffectivenessEstimator.h` | 軸ごとに、再帰最小二乗法でモデル `ε = b·u(t−delay) + a·ω + c` を学習する |
| `AdaptiveRateController.h` | 角度 → 角速度 → 角加速度 → 学習したモデルを通した舵、というカスケード |
| `StallGuard.h` | 速度低下と失速に対する保護 |
| `TakeoffSequencer.h`、`LandingSequencer.h`、`PhaseTargets.h` | 離陸（滑走路または手投げ）と着陸を、センサーに基づく段階ごとに実行する |
| `FeedbackSupervisor.h` | すべてをまとめる：1 周期内の順序、優先度、`requestTakeoff()`/`requestLanding()`/`cancelPhase()`、`printStatus()`、接続計画 |
| `FeedbackModules.h` | すべてをまとめて取り込む include 1 つ |

### 仕組み

**舵の有効性**。軸のモデルは角加速度 `ε = b·u + a·ω + c` です。`b` は舵 1 µs あたり何 °/s² 得られるか（符号は応答の向き）、`a` は減衰（空気が回転を抑える。この項がないと、定常的な回転中は `b` の推定がゼロに向かってしまう）、`c` は一定のモーメント（重心位置、トリム、プロペラ）です。舵の力は ∝ ρV² なので、`b` は基準速度で学習し、
`(V/Vref)²` を掛けます。機体が加速すれば、再学習なしで舵がすぐ「強くなる」わけです。ピトー管の指示対気速度にはすでに空気密度が含まれているため、高度は自動的に考慮されます。速度センサーがなければスケールは 1 で、`b` を直接学習します。

データは 20 ms の区間ごとに取ります。区間の平均加速度は、両端でのジャイロの差 ÷
区間の長さであり、これに対応するのは同じ区間の平均の舵量と平均の角速度です（舵量は遅れ `RESPONSE_DELAY_MS` を考慮）。そのあと方程式の両辺を同じ 2 Hz のローパスフィルタに通します。比率は変わらず、サーボの慣性のせいで「純粋な遅れ」モデルが成り立たなくなる高周波成分は取り除かれます。学習できるのは空中で、かつ舵が「揺すられている」ときだけです（約 0.3 秒間に振幅が `MIN_EXCITATION_US` 以上）。パイロットのスティック操作も揺すりになるので、MANUAL でも推定は学習します。

**制御器**。3 段構成で、軸ごとに処理します。

```
ω* = ANGLE_GAIN · (target − angle)              "機首が 10° 下 — 40°/s で引き上げる"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "必要より遅く回っている — 補正する"
surface = (ε* − a·ω − c) / b                    学習したモデルを通す
```

積分項 `I` は舵量の µs ではなく °/s で保持します。そのため `b` の推定が変わっても正しいままです。地上では積分を固定し（離陸滑走・着陸滑走中の針路を除く）、舵が限界に達したときは限界の方向には蓄積しません。協調旋回も考慮します（速度が分かっている場合）。バンク中には、ピッチ `g·sin φ·tg φ / V` とヨー `g·sin φ / V` が必要です。

**軸の符号は地上でのみ決める**。飛行中に軸を無効にしたり反転したりしません。IMU の取り付けは校正 `o` と電源投入時のチェックで決まり、舵の向きはパイロットの飛行前チェックで決まります。空中での間接的な兆候（失速からの逸脱、スピン、アクロバット、突風）は誤認を招くおそれがあり、そのような瞬間に軸を無効化したり反転したりすれば機体を失いかねません。ある軸の `b` の推定値が確実に負であっても、`reason` に警告（「舵に対して逆に反応している？ 地上で確認」）を出すだけです。負の推定値は制御器には渡さず、その軸は事前モデルで動かします。

**失速保護**。2 段階あります。*LowEnergy*：機首を上げた状態で速度が急に落ちる、失速に近い（< 1.25·Vs）、またはエレベーターの効きが失われた場合：スロットル ≥ 80 %、ピッチ ≤ 5°。*Stall*：速度が失速速度を下回り、エレベーターに逆らって機首が下がる、または低エネルギー時にエルロンに逆らって翼が落ちる場合：フルスロットル、機首下げ、バンク ≤ 10°、エルロンを制限（大きなエルロン操作は翼端を失速させる）。解除にはヒステリシスを持たせます（速度 ≥ 1.5·Vs）。リンク喪失時はスロットルに触れず、地面すれすれ（フレア、着陸滑走）では保護を無効にします。着陸そのものが制御された失速だからです。

**離陸**。`WaitThrottle`（モーター停止）→ パイロットがスロットル ≥ 50 % を入れる →
`GroundRoll`（フルスロットル、翼は水平、針路はラダーと車輪で保持、エレベーターはフリー）→ 離陸速度に達する（速度センサーがなければタイムアウト）→ `Climb`（12°、フルスロットル）→ 高度 30 m → `Complete`。手投げ（`TAKEOFF_HAND_LAUNCH`）では滑走の代わりに `WaitLaunch` となり、投げた後にだけモーターが始動します（前後方向の加速度
≥ 1g）。離陸前にスロットルを戻した場合は中止です。

**着陸**。`Approach`（スロットル 25 %、降下率 1 m/s。ピッチは垂直速度の誤差から、バンクはパイロットから ≤ 20°）→ 高度 2 m → `Flare`（スロットル 0、降下率は同じ規則で
0.3 m/s まで減らす）→ 加速度計で接地の衝撃を検出、または「低くて回転していない」→
`Rollout`（車輪で針路を保持）→ `Complete`。パイロットのスロットル ≥ 80 % はゴーアラウンドです。フレアには距離計が必要です。気圧計は 1 m ほどずれます。

**優先度**（`FeedbackSupervisor`）：ARM していない > 失速保護 > 離陸／着陸 >
モードの目標。リンクを失うと各段階は中止され、スタビライズ制御は failsafe の滑空の目標を実行します。

### シミュレーション

`test/test_feedback/test_main.cpp`（PC 上では `pio test -e native -f test_feedback`）には、機体モデル（独立した各軸、サーボの遅れと慣性、舵の有効性 ∝ V²、減衰 ∝ V、一定のモーメント、速度に応じた迎角を介した揚力、失速、ステアリングホイール付きの降着装置）と、10 のシナリオがあります。

| シナリオ | 確認する内容 |
|---|---|
| 一定のモーメントがある状態で、バンク 30° ／ ピッチ −15° からの回復 | 水平への復帰と「さらに補正する」動作：積分項がトリムを自力で見つける |
| 速度センサーなしで、14 と 20 m/s で ±15° の揺さぶり | `b` の推定が真値に収束し、速度に応じて再計算される |
| エルロンの取り違え、パイロットが MANUAL で翼を揺らす | `b` の推定が負 → 警告のみで、軸は無効にならない |
| 30 秒間の乱気流 | 突風を打ち消し、バンクが 10° を超えない |
| スロットル 20 % で機首 15°（速度センサーありとなし） | 速度が失速まで落ちない |
| プロペラの反トルクがある状態での滑走路からの離陸 | 各段階、高度、滑走中の針路 |
| 15 m からの着陸 | 各段階、地面付近でスロットルなし、ソフトな接地 |
| 離陸滑走中のリンク喪失、ARM していない場合、MANUAL | 中止、スロットルには触れない、舵面はパイロットのもの |

モデルは大まかなもので、確認するのはロジックと符号であり、特定の機体向けの調整ではありません。

```bash
pio test -e native -f test_feedback      # PC 上で数秒
pio test -e esp32-s3 -f test_feedback    # テスト用ファームウェアを書き込んで実行
pio run -t upload                        # 通常のファームウェアに戻す
```

### 接続計画

1. `FlightController::update()` が、センサーの読み取りとコマンドの計算のあとで
   `FlightSnapshot` を埋め、`FeedbackSupervisor::update()` を呼びます。最初は
   **シャドウモード**です。出力はログ（`printStatus()`）とダッシュボードにだけ出し、舵には送りません。手動操縦での飛行中は、各軸の `b` の推定が正で、速度とともに増えるはずです。
2. 地上で、機体を手に持って STABILIZE：傾けると、舵が打ち消す方向に動きます。
3. 1 軸ずつ：`Autopilot::getRollCorrection()` の代わりに `deflectionUs` を使います（最初はロールだけ）。次にピッチです。
4. スロットル：`throttleOverridePercent`/`throttleFloorPercent`。
   `Autopilot::applyThrottle()` の後、failsafe の前に置きます（failsafe がすべてに優先します）。
5. 離陸／着陸は空いているスイッチに割り当て、`Autopilot` から `AUTO_TAKEOFF`
   モードを削除します。
6. `FeedbackConfig` の定数は `Config.h` へ。対気速度センサーは `AirspeedSensor` の実装と、`SensorSelection.h` のカテゴリとして用意します。

---

## 新しいボードの追加方法

1. `platformio.ini` に `[env:<name>]` を追加し、他と重複しない
   `-D BOARD_ESP32_<NAME>` を付けます。
2. `Config.h` に `#elif defined(BOARD_ESP32_<NAME>)` ブロックを追加し、すべてのピンを書きます。`PIN_I2C2_SDA/SCL` も含めます（OLED がなければ −1）。GPIO の割り当て可能数をあらかじめ見積もってください：flash/PSRAM/USB/strapping。
3. サーボ出力には LEDC チャンネルが 5 つ必要です。どの ESP32 にもあります。ラダー用のピンがない場合は `PIN_RUDDER = -1` とし、その出力は単に無効になります。
4. ボードを実機で確認するまでは `default_envs` を変更しないでください。ピン配置を確認していない場合は、コミットに明記してください。

---

## ビルド、書き込み、モニターのコマンド

```bash
pio run                        # 既定のボード（esp32-s3）をビルド
pio run -t upload              # 書き込み
pio device monitor             # モニター、115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # すべてのボードがビルドできることを確認
```

- **ESP32-S3：** 書き込みと Serial は「COM」コネクタ（CH343）経由です。ブリッジが固まった場合（Windows が「デバイスが機能していません」と応答する。ESC のノイズで起こることがある）は、ケーブルを挿し直すと直ります。「USB」コネクタ（内蔵の USB-JTAG）経由でも書き込めます：
  `pio run -t upload --upload-port <USB COM port>`。
- ポートモニターを開いている間は、同じポートへの書き込みは通りません。
- `lib_deps`：`olikraus/U8g2`（OLED）が唯一の外部ライブラリです。
- `test/` の詳細は [`TESTING.md`](TESTING.md) にあります。
  - `pio test -e native -e native-stm32`：PC 上で 387 件のテスト（ハードウェアの代役は `test/native/support/`）。カバレッジは `gcovr`。
  - `pio test -e esp32-s3`：基板上で `test_feedback/`（フィードバックの閉ループシミュレーション）と `test_imu_orientation/` を実行します。それぞれテスト用ファームウェアを書き込むので、そのあと `pio run -t upload` で通常のファームウェアを書き戻してください。
- 静的解析：`pio check -e esp32-s3`（cppcheck）、`pio check -e stm32h743`（`hal/stm32/` と `src/stm32/` に対する
  cppcheck）、`tools/clang-tidy.sh`（`.clang-tidy` のプロファイル）。

---

## 既知の制限事項

- **オートパイロットは飛行で試していません。** 机の上で符号を実際に確認しました（傾ける → 水平に戻す方向への補正）。PID の係数は初期値です。
- **STABILIZE はスティックの上に重ねる水平復帰**であり、スティックでロール／ピッチの角度を指定する「角度モード」（FBWA）ではありません。パイロットとオートパイロットの操作は合算されます。
- **リンク喪失時の滑空は飛行で試していません。** `FAILSAFE_GLIDE_*` の角度は初期値です。ピッチ −3° は個々の機体に合わせて決めます（機首が失速するほど上がっても、急降下してもいけません）。
- **水平線。** 取り付け校正（`o`）がある場合はその結果（NVS）から求めます。加速度計のゼロ点オフセットは温度でドリフトし（20 °C あたり約 1〜2°）、水平線が「ずれた」場合は `o` をやり直してください。校正がない場合は電源投入時の姿勢です（水平にして電源を入れます）。
- **コンパスの取り付け**は、これまでどおり `MAG_ROTATION_CW_DEG` で指定します（姿勢による校正は影響しません）。
- **コンパス：** 方位は傾き補正なしで、角度を数える向きは組み立て済みの機体では確認しておらず、校正は機体に載せた状態で行う必要があります。方位を使うモードはまだありません。
- **GPS** はナビゲーションには使っていません。ESP32-C3 では受信専用です。
- **フィードバック（`autopilot/feedback/`）は接続しておらず**、大まかな機体モデルのシミュレーションでしか検証していません。`FeedbackConfig.h` の数値のうち「прикидка」（「大まかな見積もり」）と注記したものは、実際の機体で見直す必要があります。対気速度センサーはまだありません（ないと、舵の有効性の学習が遅くなり、失速は減速でしか分かりません）。
- **実機で未確認：** `ICM42688_Sensor`（`ImuSensorBase` を通じて共通の規約に合わせた）、`BME280_Sensor`（Bosch の補正を新たに実装した）、SPI 接続の
  BMP388、`QMC5883L_Sensor`、CFG-VALSET による GPS の設定。接続の際は、起動ログ、コンソールの `s`、傾けて符号を確認してください。
- **ブレッドボード上の I2C はノイズを拾います。** ESC／モーターによるもので、単発のエラーは `s` で確認できます。ドライバーは耐えますが、機体では I2C の配線を短くし、動力線から離してください。
- **ESP32Servo は使っていません。** バージョン 3.2.1 は ESP32-S3 でサーボを MCPWM
  に割り振り、`attachPin()` で MCPWM のユニット番号とタイマー番号を取り違えます。
  GPIO6/7 から GPIO4/5 の信号が出ていました（ESC が右スティックで動いていた）。出力は LEDC に書き直しました。ライブラリを戻すなら、`p` で確認してからにしてください。
- **ESC は 50 Hz の PWM** で、ファームウェアにはスロットル範囲の校正モードがまだありません。
- **Web ダッシュボード：** アクセスポイントのパスワードは弱く、コマンドは飛行中も受け付けます。テストベンチと野外で使う道具であり、飛行中に使うものではありません。
- **プロトタイプの機構：** 最初のプロトタイプは飛行しましたが、モーターの固定が弱いことと、翼の剛性が不足していることが分かりました。
- **ライセンスは OpenPlane License** です（[LICENSE](LICENSE.md)）。作者名の明記を必須とする MIT で、軍事利用の禁止と、人や財産に対する、本人の書面による同意のない意図的な危害の禁止を定めています。ファイルに他のライセンスヘッダーを追加したり、作者名を削除したりしないでください。

---

## 変更の加え方

- **小さなコミット：** 論理的な 1 ステップを 1 コミットにします。
- **コミット前にテストと解析：** `pio test -e native -e native-stm32`、
  `pio check -e esp32-s3`、`pio check -e stm32h743`、`tools/clang-tidy.sh` をすべて通します（[`TESTING.md`](TESTING.md)）。
- **共通コードを変更したら、すべてのボードをビルドする。** S3 が主力ですが、C3、
  38 ピン版、`stm32h743` を壊してはいけません。リリース前には
  `tools/build_matrix.sh`（全ボード × 全センサー）を実行します。
- **実機で確認できることは実機で確認する：** 符号は傾けて、出力は `p` コマンドで、リンクは送信機の電源を切って確認します。
- **API を作り話で書かない。** `~/.platformio/packages/framework-arduinoespressif32/`
  にあるフレームワークのソース（Arduino core 2.0.x）と照らし合わせてください。ネット上の情報は API の異なる 3.x 系を説明していることが多くあります（たとえば
  LEDC）。
- **状況を飾らない。** 実機で確認していないなら、そう書きます。
- **レイヤーは分をわきまえる。** 下位のクラスが急に上位のクラスを必要とするなら、ロジックを `FlightController` まで引き上げるべきです。
- **データの契約を変更するとき**（`FlightOutputState`、`ControlCommand`、
  `ImuData`、`/api/status` の JSON）は、利用側をすべて同じコミットで更新します。
