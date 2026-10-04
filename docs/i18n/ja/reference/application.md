# APPLICATION — `src/main.cpp` と `src/stm32/main.cpp`

> 🌐 このページは[ロシア語の原文](../../../reference/application.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

この 2 つのエントリポイントは、それぞれのボード向けの **composition root** です。ファームウェアで唯一の翻訳単位であり、オブジェクトが生成されて参照でつなぎ合わされる唯一の場所でもあります。飛行ロジックはここにはなく、オブジェクトの構成も同じです。違うのは、ボード、テレメトリ（Wi-Fi か MAVLink か）、そして飛行ループの回し方です。

## グローバルオブジェクト（共通）

宣言の順序 = 構築の順序です。

| オブジェクト | 型 | 関連 |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`、`imuSensor` | `SELECTED_IMU_DEVICE(board)`、`SelectedImu` | `SensorSelection.h` で指定したバス |
| `baroDevice`、`baroSensor` | `SELECTED_BARO_DEVICE(board)`、`SelectedBaro` | ピトー管がある場合はこれが静圧 |
| `magDevice`、`magSensor`、`magnetometer` | … `SelectedMag`、`MagnetometerSensor* const` | `SENSOR_MAG != NONE` の場合のみ。そうでなければ `nullptr` |
| `gpsSensor`、`gpsReceiver` | `SelectedGps`、`GpsSensor* const` | `SENSOR_GPS != NONE` の場合のみ |
| `pitotDevice`、`pitotBaro`、`pitotSensor`、`airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`、`SelectedPitotBaro`（`"PITOT-BMP581"`）、`PitotDualBaroAirspeed(pitotBaro, baroSensor)` | `SENSOR_AIRSPEED != NONE` の場合のみ |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`、`throttleManager` | `ControlMixer`、`ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | すべてのセンサー（null 可） |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`、`Controls::BINDINGS` テーブル |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | 上記すべて |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | コントローラー、オートパイロット、統計 |
| `debugConsole` | `DebugConsole` | コントローラー、出力、オートパイロット、ログ、`&board`（バスの走査 `b`） |
| `oledDisplay` | `OledDisplay` | コントローラー、オートパイロット、統計 |
| ESP32：`webDebugServer` | `WebDebugServer` | コントローラー、オートパイロット |
| STM32：`mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`、コントローラー、オートパイロット、統計 |

## `src/main.cpp` — ESP32（S3、C3、38 ピン）

| 関数 | 説明 |
|---|---|
| `static void printBanner()` | `Serial` へのスプラッシュ表示 |
| `static void setupSensors()` | 各センサーの `begin()`。応答したものの校正：IMU `calibrate()`（2 秒間静止＋飛行前チェック）、気圧計 `calibrateAltitude()`、コンパスは 25 ms 後の最初のサンプルで IMU の針路を決める（`setYaw`）。GPS `begin()`。ピトー管 `begin()`（ゼロ点はループの最初の 1 秒間）。`autopilot.begin()` |
| `void setup()` | `begin(115200)` の**前**に `Serial.setTxBufferSize(4096)`、スプラッシュ表示、`board.begin()`、`flightOutputs.begin()` ＋ `setFailsafe()`、`setupSensors()`、`flightController.begin()`、OLED、Web サーバー、スイッチの配置、`debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`。周期は `vTaskDelayUntil(LOOP_PERIOD_MS)`。100 ms を超えて遅れたら、計時をやり直す（「追いつこう」とはしない） |

## `src/stm32/main.cpp` — STM32H743

env `stm32h743` のエントリポイントです（ESP32 のビルドでは `src/stm32/` ディレクトリを
`build_src_filter` で除外しています）。実機で確認したのは、センサーなしの DevEBox H743
基板です（起動、USB 経由のコンソール、SD カード、ブラックボックス、iBUS、サーボとモーターの手動操縦）。全体は PC 上で `test/native_stm32` のテスト（env `native-stm32`）により実行されます。隣にあるのは、SDMMC1 のピンとクロックを扱う `sd_msp.cpp`と、コンソールの `D` キー（DFU への再起動）を扱う `bootloader.cpp` です。

| 関数 | 説明 |
|---|---|
| `setup()` | `Serial.begin(115200)`、スプラッシュ表示、`board.begin()`、出力を安全な位置へ、`Stm32FlashStorage::store().mount()`（設定イメージ：空／N バイト／破損の場合は既定値）、`setupSensors()`（ESP32 と同じ）、`flightController.begin()`、`mavlink.begin()`、OLED、スイッチの配置、`debugLogger.begin()`、タスク `flight` と `storage`、`vTaskStartScheduler()`（戻らない） |
| `static void flightTask(void*)` | 優先度 `Rtos::PRIORITY_FLIGHT`、スタック 16 KB：`flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`。`vTaskDelayUntil(LOOP_PERIOD_MS)`、100 ms を超えて遅れたら計時をやり直す |
| `static void storageTask(void*)` | バックグラウンド：100 ms に 1 回 `Stm32FlashStorage::instance().service()` を呼び、設定セクタの消去と書き込みを行う。飛行タスクにプリエンプトされる |
| `loop()` | 空：`vTaskStartScheduler()` のあとはタスクだけが動く |

コンソール（`Serial`、LPUART1 PA9/PA10、115200）は ESP32 と同じ `DebugConsole` です。
`h` はメニュー、`s` はセンサー、`b` はバスの走査、`p` は出力、そして校正です。

## 不変条件

- 出力は、センサーを初期化する**前**に安全な位置へ移します（IMU の校正でループが約 2 秒止まるため）。
- ESP32：`Serial` の TX バッファは `begin()` の前に設定します。STM32：UART のバッファは `platformio.ini` の `SERIAL_RX/TX_BUFFER_SIZE` です。
- どのオブジェクトも他のオブジェクトを所有しません。参照はすべて所有権を持たず、寿命はプログラム全体です。
- スイッチの動作を変えるには `config/Controls.h` を変更します。`main.cpp` ではありません。
