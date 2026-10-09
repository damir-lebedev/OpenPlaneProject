# TESTING.md：テスト、カバレッジ、静的解析

> 🌐 このページは[ロシア語の原文](../../TESTING.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

ファームウェアは 2 つのレベルで検証されます。

| 場所 | コマンド | 内容 |
|---|---|---|
| **PC（native）** | `pio test -e native` | ファームウェアのヘッダーを PC でそのままビルドし、ハードウェアは制御可能なフェイクに置き換えます。モジュール、ドライバー、閉ループのフライトシミュレーション、各センサーセットを載せた ESP32 のファームウェア全体（S3 と 38 ピン）です。カバレッジを計測します |
| **PC（native-stm32）** | `pio test -e native-stm32` | STM32duino のフェイク層の上で、STM32H743 のファームウェア全体（`src/stm32/main.cpp`）を動かします。FreeRTOS のタスク、フラッシュ、MAVLink、I2C と SPI のセンサーです |
| **ビルドマトリクス** | `tools/build_matrix.sh` | 4 種類のボード × 6 種類のセンサーセットを `-Wall -Wextra (-Wshadow)` でビルドします。プロジェクトのコードに警告が 1 つでもあればエラーです |
| **ボード** | `pio test -e esp32-s3` | 実機の ESP32-S3 での `test_feedback` と `test_imu_orientation`（テスト用ファームウェアを書き込みます。終わったら通常のものに戻してください：`pio run -t upload`） |
| **STM32 ボード** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | DevEBox H743 の**本物の SD カード**でのブラックボックスのテストと、Cortex-M7 での `test_feedback` と `test_imu_orientation` です。[後述](#stm32-ボード上のテスト) |

アーキテクチャ上の背景は [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-テスト容易性) にあります。

---

## クイックスタート

```bash
pip install platformio gcovr        # 1 回だけ
# Windows：PATH に g++ が必要です。たとえば WinLibs（winlibs.com、zip UCRT）：
# 展開して mingw64\bin を PATH に追加します。インストールは不要です
pio test -e native -e native-stm32  # すべてのネイティブテスト（約 1.5 分）
gcovr                               # ファイルごとのカバレッジ（設定は gcovr.cfg）
tools/build_matrix.sh               # すべてのボード × すべてのセンサー（約 25 分）
gcovr --html-details -o coverage/index.html   # HTML レポート（coverage/ は .gitignore に入っています）

pio test -e native -f native/test_rc          # 1 つのセット
pio test -e native -f test_feedback           # PC でのフィードバックのシミュレーション

# 閉ループシミュレーションの軌跡を CSV に（グラフ用）：
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# MAVLink のストリームを基準のデコーダーで検証する（pip install pymavlink）：
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

テストを変更したあとにカバレッジを集計するときは、クリーンなビルドから始めると安全です：`rm -rf .pio/build/native`。そうしないと、以前の実行のカウンターがレポートに混ざります。

---

## ネイティブビルドの仕組み

`platformio.ini` の `[env:native]`：`platform = native`、Unity、`-std=gnu++17`、`-D BOARD_ESP32_S3`（ESP32-S3 のピン配置）、`-I test/native/support`、`-Wall -Wextra -Wshadow`、`--coverage` によるカバレッジ計測、そして `-fkeep-inline-functions -fkeep-static-functions` です。この 2 つのオプションがないと、一度も呼ばれなかったヘッダー内の関数が gcov に見えず、カバレッジが過大に出ます。

### ハードウェアのフェイク：`test/native/support/`

ESP32 2.0.x の Arduino コア、ESP-IDF、各ライブラリと同じ名前・シグネチャのヘッダーですが、中身は `namespace fake` の中のシミュレートされた世界の上で動きます。

| ファイル | 置き換えるもの | シミュレーションでできること |
|---|---|---|
| `Arduino.h`、`Print.h`、`WString.h`、`Stream.h` | Arduino コア | マクロ（`constrain`、`sq`、`DEG_TO_RAD` など）、`map()`、`String`、元と同じ `print()` の書式化。`ARDUINO` は意図的に定義**しません** |
| `esp32-hal-fake.h` | 時間、GPIO、ADC、LEDC、FreeRTOS、PSRAM、`ESP` | 時計は `fake::advance*()`/`delay()` でのみ進みます。`millis()/micros()` は ESP32 と同じ `uint32_t` で、オーバーフローも実機と同じ挙動です。LEDC チャンネル、実際のデューティ比に基づく `pulseIn`（ピンの入力バッファが有効なときだけ見える）、`analogReadMilliVolts`（電圧は `fake::gpio().analogMv` から取る）。タスクは登録され（ハンドルは非ゼロ）、`fake::runTask(task, n)` はその無限ループを n 回実行し、`ulTaskNotifyTake` は 1 回分として数え、`xTaskNotifyGive` はカウンターとして数えます。FreeRTOS のミューテックスは「使用中」のフラグです。`psramFound()`/`ps_malloc()`。クリティカルセクションは回数を数えます |
| `HardwareSerial.h` | UART | ポートは番号で登録されます（`fake::uart(1)`）。`pushRx()`、`txBytes()`、動作中の速度変更（`updateBaudRate`、履歴は `baudChanges()`）。`Serial` = UART0 |
| `esp_partition.h` | ESP-IDF のフラッシュパーティション | パーティションは NOR の挙動をするバイト列です。消去は 4 KB のセクター単位のみ、消去後は 0xFF、書き込みはビットを下げるだけで（ビットを上げようとした回数を数えます：`bitRaises`）、`beforeWrite` は「電源が落ちた」状況を作り、読み出し・書き込み・消去の回数カウンターがあります |
| `esp_system.h` | 再起動の原因 | `fake::chip().resetReason` から返す `esp_reset_reason()` |
| `Wire.h` | I2C | アドレスごとのデバイス。`fake::RegisterMapDevice` は自動インクリメントするレジスタ、書き込みログ、故障（`present`、`failWrites`、`failReads`、`failReadIf`、`shortRead`）、フック `beforeRead`/`onRegisterWrite` を持ちます |
| `SPI.h` | SPI | CS ピンごとのデバイス。`fake::SpiRegisterMapDevice` は Bosch / InvenSense のプロトコルで、データの前に `dummyBytes` を置きます |
| `Preferences.h` | NVS | メモリ上のストレージ。`begin(readOnly)`/`get*`/`getBytes` の挙動は元と同じです。`failBegin` |
| `WiFi.h`、`WebServer.h` | Wi-Fi AP、HTTP | `softAP()` の結果はテストが決めます。`WebServer::request(メソッド, uri, 本文)` が登録済みのハンドラーを呼び出します。`fake::webServers()` はすべてのインスタンスです |
| `U8g2lib.h` | U8g2 | ピクセルの代わりに、描画された文字列と矩形のリストを持ちます。`begin()/sendBuffer()` はユーザーのバイトコールバックにバイトを渡します。`fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`、`GPIO_PIN_MUX_REG`、`PIN_INPUT_ENABLE` |

### STM32duino の層：`test/native/support_stm32/`（環境 `native-stm32`）

`-I` の中で `support/` より前に置かれ、同じフェイクに STM32duino にしかないものを補います。この環境の `<Preferences.h>` は、`KeyValueStore` の上に作られた**本物の** `include/hal/stm32/compat/Preferences.h` です。

| ファイル | 置き換えるもの | できること |
|---|---|---|
| `Arduino.h` | STM32duino コア | ピン `PA0..PE15`（ポート·16 + 番号）、`pin_size_t`、サーボ出力ピン用の `PinMap_TIM`、`HardwareTimer`（パルスは `fake::timerPulseUs(pin)` と `pulseIn()` で見える）、`Uart`、`noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino の FreeRTOS | 共通のタスク登録簿に入る `xTaskCreate`（スタックはワード単位）、戻ってくる `vTaskStartScheduler()`（タスクはテスト自身が回します：`fake::runTask`）、`xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM のエミュレーション | 8 KB の「フラッシュ」（消去後は 0xFF）とバッファ、`fake::eeprom()`（カウンターとイメージの破損） |
| `SPI.h` | | `SPIMode` |

STM32 向けに共通のフェイクへ追加されたもの：`TwoWire(sda, scl)`、`setSDA/SCL` と `fake::wireWithSda(pin)`（ボードの 2 本目のバスを探す）、`HardwareSerial(rx, tx)` と `fake::uartByRx(pin)`、`SPIClass::setSCLK/MISO/MOSI`。

### チップのエミュレーターと機体モデル：`test/native/helpers/`

| ファイル | 内容 |
|---|---|
| `ChipEmulators.h` | LSM6DSV、ICM-45686（IPREG の間接レジスタ付き）、QMC6309、SPL06-001、BMP581、u-blox の NAV-PVT フレーム。I2C または SPI 上のレジスタマップで、データは `World`「世界」（角度と角速度、高度、対気速度、針路、座標）から得て、`IMU_ROTATION_CW_DEG` を考慮したチップの軸で返します |
| `PlaneSim.h` | 約 1.2 kg の飛行機のモデル：質点 + ロール / ピッチの回転、失速を含む CL(α)、抗力、推力、風、サーマル、地面 |
| `SimHarness.h` | 閉ループ：送信機 → iBUS フレーム → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → 舵面の振れ → `PlaneSim` → センサー（ノイズのある 2 つの気圧センサーによるピトー管を含む）。`OPENPLANE_SIM_DIR` を指定すると CSV の軌跡を出力します |

`test/native/helpers/TestSupport.h` は、各セットが共有する部分です。`resetWorld()`（`setUp()` から呼ばれます）、`FakeUart`/`FakeServo`/`FakeBoard` とセンサー（`FakeImu`、`FakeBaro`、`FakeMag`、`FakeGps`）の代役、フレームの組み立て用 `ibusFrame()`、実物の `Esp32I2CBus`/`Esp32SpiBus` とシミュレートされたチップを持つ `*RegisterDevice` の上にドライバーを載せるテスト台 `I2cRig`/`SpiRig` です。

ボード上のテスト（`test_feedback`、`test_imu_orientation`）は移植可能です。`ARDUINO` があれば `setup()/loop()`、なければ `main()` になります。`test/native/*` のセットはボード向けにはビルドされません（`[esp32_common]` と `[env:stm32h743]` の `test_ignore`：パターンは 1 行に 1 つずつ書きます。スペースで区切ると PlatformIO は 1 つのパターンとして読みます）。STM32 では `pio test -e stm32h743` です。

---

## テストセット

| セット | テスト数 | 検証内容 |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus` の補助機能（NACK、短い読み出し：バッファには触れない）、`I2cRegisterDevice`、`SpiRegisterDevice`（読み出しビット、BMP388 のダミーバイト）、`Esp32I2CBus`（5 ms のタイムアウト）、`Esp32SpiBus`（モード 0〜3）、`Esp32UartPort`（8N1、ピン）、`Esp32ServoOutput`（50 Hz/14 ビット、パルスの制限、LEDC の故障、入力バッファ経由の測定）、`Esp32Board`（バス、UART、チャンネル順、AUX、ブザー） |
| `native/test_rc` | 16 | `RcChannelState`、`RcInput`、iBUS の解析：分割されて届くフレーム、CRC、12 ビットの値、送信機のフェイルセーフ、500 ms のタイムアウト（`micros()` のオーバーフローをまたぐ場合も）、ゴミデータ、再同期 |
| `native/test_control` | 21 | フラップ（速度、初回の呼び出し、一時停止）、ミキサー（符号、リバース、フラッペロン）、スロットル、ARM のステートマシンとモードのセンサー確認、出力テーブルとパルスの自己診断 |
| `native/test_autopilot` | 22 | PID（センサーの角速度に基づく D 項、積分、アンチウィンドアップ、`dt`）、角度モードとしての STABILIZE、時間による自動離陸、昇降舵による ALT_HOLD、信号喪失時の滑空 |
| `native/test_autopilot_modes` | 31 | 全 12 モードと、センサーがないときの各モードの反応、割り当て表と `static_assert`、機能とダイヤル、ナビゲーション（針路、円、ホーム地点、ジオフェンス）、failsafe RTH/滑空、手投げ発進、ソアリング、オートトリム（書き込みは地上のみ） |
| `native/test_flight_controller` | 12 | 実物のクラスでの `FlightController` の 1 周期：優先順位は 信号喪失 > ARM > スティック/オートパイロット > スロットル、AUX、`MOTOR_KILL`、ブザー |
| `native/test_imu` | 21 | MPU6050/6500/9250 と ICM-42688：識別、レジスタ、スケール、軸の回転と航空機の符号、バスのエラー、ジャイロの校正と飛行前チェック、3 つの姿勢による取り付け校正、NVS、姿勢フィルター |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`、I2C と SPI の BMP388、Bosch の基準実装との照合による BME280/BMP280、コンパス（針路、NVS に保存するハードアイアン校正）、u-blox M10（CFG-VALSET、NAV-PVT、壊れたフレーム、タイムアウト）、`SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV（16X/32X、代替アドレス、SPI）、ICM-45686（間接レジスタ）、QMC6309、SPL06-001（データシートの式）、BMP581（DRDY と予備経路）、ピトー管（ゼロ点、フィルター、密度、ホースの取り違え、古いデータ、2 つの気圧センサーのノイズを含む「飛行」） |
| `native/test_storage` | 16 | `KeyValueStore`（再読み込み、摩耗：同じ値は書かない、データを失わないオーバーフロー、CRC、消去中の電源断、ゴミデータ、フォーマットのバージョン）、`KvPreferences`（ESP32 の NVS と同じ挙動） |
| `native/test_mavlink` | 20 | pymavlink の基準フレームと突き合わせるコーデック（v1、v2、署名付き）、CRC、再同期、テレメトリー：ストリームの周波数、HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS、PID パラメーター（一覧、読み出し、書き込み、不正な値の拒否）、地上からのモード変更、地上からの ARM は拒否、ミッションは 0 件、あふれた UART バッファがループを止めないこと |
| `native/test_blackbox` | 19 | ブラックボックス：フォーマットと CRC、NOR のフェイク上のセクターリング（消去なしの新しいパーティション、ゴミは必ず消去、古いフライトは丸ごと、かつ空き領域のためだけに消去、最新のフライトには触れない、リング末尾をまたぐ書き込み、再起動後の先頭位置、電源断、書きかけのレコードを CRC で検出）、本物の `FlightController`/`Autopilot` でのフライト記録：ARM とスロットルでの開始（プリ録画付き）、DISARM と「地上に静止」のあとの停止、信号喪失では止まらない、異常再起動のあとの記録、手動開始、イベント、バッテリー、飛行中にフラッシュが尽きる、パーティションより長いフライト、CRC 付きフレームでの吐き出しと速度の切り替え、コンソールのメニュー `k`、パーティションがない場合は無効 |
| `native/test_blackbox_scan` | 3 | 起動時のリングの抽出照合と全照合の比較：リングの 300 通りのランダムな履歴 × 5 段階のプローブ（先頭位置、番号、フライト一覧が一致し、状況が合わないときは全照合に譲る）と、64 MB の SD 領域でのコスト（32,000 回ではなく約 530 回の読み出し） |
| `native/test_telemetry` | 28 | `LoopStats`、`LogSettings`（NVS、バージョン）、`DebugLogger`（すべてのチャンネル、NAV）、`DebugConsole`（メニュー、ホットキー、バスの探索 `b`、ARM 中は禁止、保存は ARM していないときのみ）、`WebDebugServer`（ルート、JSON、メールボックス）、`OledDisplay`（I2C 上のバイト列、フレーム、信号喪失時の反転） |
| `native/test_sim` | 15 | 機体モデルを使った、ファームウェア全体の閉ループ飛行：バンクからの回復、横風の中の CRUISE、LOITER、RTH、failsafe RTH/滑空、ジオフェンス、滑走路からの自動離陸、手投げ発進、自動着陸、サーマル、スパイラルからの RESCUE、速度保持と失速防止、ループ内の本物のピトー管、「ゆがんだ」機体のオートトリム、飛行中のセンサー故障（IMU、気圧センサー、ピトー管、GPS） |
| `native/test_feedback_units` | 14 | フィードバックの各モジュール単体：速度の取得元、空中/地上の判定、RLS 推定、コントローラー、失速の兆候、離陸/着陸の中止 |
| `native/test_app` | 10 | ESP32-S3 上の `src/main.cpp`、ベンチ用セット MPU6500/BMP581/QMC5883P/OLED：`loop()` の周期、送信機 → サーボ、ARM、モード、信号喪失、コンソール、ダッシュボード、画面、ブラックボックス（コア 0 のタスク、スロットルでの記録、DISARM 後のフライト、`bb list`） |
| `native/test_app_lsm6dsv_pitot` | 9 | ESP32-S3 上の `src/main.cpp`、飛行用セット：LSM6DSV + QMC6309 + SPL06 + ピトー管内の BMP581 + GPS：すべてのチップの識別、ピトー管のゼロ点と速度、高度、GPS によるホーム地点、チップの角度による STABILIZE、ホーム地点への RTH、バスの探索、ダッシュボード |
| `native/test_app_icm45686_esp32dev` | 5 | **ESP32 38 ピン**（`BOARD_ESP32_CLASSIC`）上の `src/main.cpp`、ICM-45686 + QMC6309 + SPL06 + BMP581 のセット：ボードのピン配置、IPREG フィルター、手投げ発進、安定化と速度、1 本のバスの探索 |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | **STM32H743** 上の `src/stm32/main.cpp`、飛行用セット：タスクと優先度、2 ms の周期、ピトー管、PWM タイマーと `pulseIn`、飛行中の MAVLink、GCS からのモード変更、バックグラウンドタスクによる設定の「フラッシュ」への書き込み、I2C1 の画面、コンソール |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 と **SPI 接続**の ICM-45686 および BMP581 + QMC6309：起動時のフラッシュ破損、GCS からの ALT_HOLD が高度を保持、信号喪失 → RTH、MAVLink で確認できること、壊れたイメージの書き直し |
| `native_stm32/test_blackbox_sd` | 29 | SD カード上のブラックボックス：FAT32（MBR あり・なし、2 クラスターにまたがるディレクトリ、ノイズのエントリ、他のデータ/断片化した/空のボリューム）、`SdFileRegion`（不完全なブロック、キャッシュ、消去、境界、故障）、偽の `HAL_SD` の上で動く本物の `Stm32SdCard` ドライバー（4 ビット、予備の速度、リトライ、ビジー状態のカード、アラインされていないバッファ）、カード上のリング（再起動、電源断、照合のコスト）、「リングは空」の目印、`FlightController` でのフライト記録、`RCC->RSR` で判別する異常再起動、バッテリーの ADC、飛行中のカードのエラー、遅いカード、コンソールからの吐き出し、`D` キー |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | カードありの `src/stm32/main.cpp`：起動時にカードとファイルが見つかる、`bbox` タスクがフライトを書き込む、ループの周期が伸びない、`bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | カードなしの `src/stm32/main.cpp`：ブラックボックスは無効になり理由を説明する、飛行機は飛ぶ、メニュー `k` は壊れない |
| `test_feedback` | 10 | フィードバックループを備えた飛行機の閉ループシミュレーション（PC とボードの両方） |
| `test_imu_orientation` | 5 | 300 通りのランダムな取り付けでの IMU の取り付け校正（PC とボードの両方） |
| **合計** | **387** | `native` に 340 + `native-stm32` に 47（さらにボード専用が 9：`test_blackbox_sd`） |

### STM32 ボード上のテスト

`test/test_blackbox_sd` はネイティブではありません。SDMMC ドライバー、カード、時間はすべて本物です。テストは FreeRTOS のタスクで動き、その隣で、最高優先度（周期 2 ms）の飛行ループを模したタスクが動きます。このタスクは、ファームウェアの中と同じように、カードへのアクセスの途中でテストを横取りします。これがなければ、ボードで実際に見つかったエラーは捕まえられません。横取りされると SDMMC の FIFO があふれていたのです（`HAL_SD_ERROR_RX_OVERRUN`）。何もない素のループでは起こりません。

| テスト | 検証内容 |
|---|---|
| `reset_cause_is_a_normal_one` | リセットの原因（`RCC->RSR`）がウォッチドッグでも電圧低下でもないこと |
| `card_is_detected_on_four_bit_bus` | カードが 4 ビットのバス、24 MHz で認識されること |
| `file_is_found_and_contiguous` | FAT32 上で `BLACKBOX.BIN` が見つかり、連続して配置されていること |
| `multi_block_writes_work_at_every_length` | 1、2、4、8 ブロックを 1 回のアクセスで書き込むこと |
| `pages_write_with_bounded_latency_and_read_back_intact` | 256 B のページ：最悪の書き込みが 250 ms 未満（SD の上限）、安定して 40 KB/s 超、読み出しと消去 |
| `header_scan_cost_on_the_whole_area` | セクターのヘッダーの読み出しと全照合のコスト |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | すべて消去、20,000 レコードずつの 2 回のフライト、「再起動」：抽出照合が 2 s 未満、レコードが順番どおりに正しい CRC で読めること。空のリングは目印により 100 ms 未満で判別 |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | 本物の `BlackBox` を 500 Hz の IMU でリアルタイムに動かす：失われたレコードはゼロ、「再起動」のあともフライトが読めること |
| `the_flight_task_was_not_disturbed` | カードへの書き込みが、模擬タスクの周期を乱さなかったこと（ずれは 3 ms 未満） |

実行方法（ファイル入りのカード：`python tools/blackbox.py sd-prepare E:`。**このテストはファイル内のすべてのフライトを消去します**）：

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

ボードは DFU モードにしておきます（DevEBox では BT0→3V3 の配線と RST。WinUSB ドライバーは Zadig 経由。詳しくは [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743) を参照）。STM32 のコンソールは USB CDC です。書き込み直後はポートがすぐには現れず、`pio test` がタイミングよく開けないことがあります（「could not open port」）。その場合は `pio test ... --without-testing` を実行し、DTR を有効にした任意のターミナルソフトで出力を読んでください（テストはポートが開かれるまで最大 60 s 待ちます）。テストのあと、ボードは **`D`** キーを待ちます。このキーで、配線なしで DFU に再起動します。

DevEBox H743 + 16 GB カードでの結果（2026-10-02）：`test_blackbox_sd` は 9/9、`test_feedback` は 10/10、`test_imu_orientation` は 5/5。カードの速度の数値は [BLACKBOX.md](BLACKBOX.md#ボードで測定した結果) にあります。

### ベンチ用ファームウェア：`test/bench/`

これらはテストセットではなく、飛行用ファームウェアの代わりにボードへ書き込む、独立した小さな PlatformIO プロジェクトです（`pio test` には見えません。フォルダー名が `test_` で始まらないためです）。ピンと上限は共通の `Config.h` から取ります。

| プロジェクト | 内容 |
|---|---|
| `bench/elevator_sweep` | `ControlMixer` と `FlightOutputs` を通して、昇降舵のスティック（CH2）をプログラムで動かします。本物のスティックと同じように、上へ行程の 100%、下へ 60%、なめらかに、途中で一時停止を挟みます。20 s 動かしたら 20 s ニュートラルにします。両端の位置では出力のパルスを測ります。スロットルは最小のままです |

書き込み：`pio run -d test/bench/elevator_sweep -t upload`。飛行用ファームウェアに戻す：`pio run -e esp32-s3 -t upload`。

---

## カバレッジ

`gcovr` が `include/` と `src/`（ファームウェアに含まれるものすべて）を対象に、2 つのネイティブ環境をまとめて集計します：`gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`。

| レイヤー | 行 | 分岐 |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors`（すべて） | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src`（`main.cpp`、`stm32/main.cpp`） | 118/123 (95.9%) | 20/29 (69.0%) |
| **合計** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**、関数 877/902 (97.2%) |

まだカバーされていない部分とその理由：

- **手投げ発進**（`TakeoffSequencer`：`WaitLaunch`、`launchDetected()`）：`FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` の間は到達できません。この定数を設定可能にしたとき（`Config.h` へ移すとき）に、テストへ加わります。
- **ボードに依存する部分**：ピンのない出力（`PIN_RUDDER = -1` になるのは C3 だけ）、TX ピンのない GPS（C3）。ネイティブテストは S3、38 ピン、STM32 のピン配置を動かしますが、C3 は動かしません（C3 はビルドマトリクスで確認します）。
- **STM32**：コアのエラー分岐（ピンにタイマーがない、タイマーのプールを使い切った）、メッセージ `FreeRTOS не запустился`（「FreeRTOS が起動しなかった」）：PC では `vTaskStartScheduler()` は必ず戻ってきます。
- **防御用の分岐**で、公開 API からは到達できないもの：列挙型に対する `switch` の `default`/`Count`、`return "?"`。
- 実行可能な行を持たないファイル（`Config.h`、`Channels.h`、`FeedbackConfig.h`、構造体 `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`、`SensorSelection.h` のマクロ、ダッシュボードの HTML）はレポートに現れません。テストにはコンパイルされますが、gcov が数えるものがないためです。

---

## 静的解析

| ツール | コマンド | プロファイル |
|---|---|---|
| GCC | `tools/build_matrix.sh`（または `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`） | すべてのボード × すべてのセンサーセット。ネイティブのテストビルドは常に `-Wall -Wextra -Wshadow` で行います。`stm32h743` は `-Wall -Wextra` です（`build_src_flags`。`-Wshadow` は STM32duino 自身のヘッダーでノイズを出すため） |
| cppcheck | `pio check -e esp32-s3`；`pio check -e stm32h743` | `[esp32_common]` の `check_*`：`include/` と `src/`（`stm32/` を除く）、warning/style/performance/portability、行内の `// cppcheck-suppress` は誤検出に対してのみ（U8g2 のコールバック、`setup/loop`）。`stm32h743` は `include/hal/stm32/` と `src/stm32/` に同じフラグ |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`：bugprone、clang-analyzer、performance、`misc-include-cleaner` など。無効にした検査の理由はファイル自体に書いてあります |

clang-tidy は `test/native/support` のフェイクを使って実行します。clang はホストのアーキテクチャ向けに ESP-IDF のヘッダーを解析できないためです（`pio check` を `clangtidy` で試すと、解析は構文エラーで中断し、実際には何も検査しません）。`misc-include-cleaner` は、各ヘッダーが使うものをきちんとインクルードしているかを見ます。「傘」のヘッダー（`FeedbackModules.h`、`IBoard.h`/`RegisterDevice.h` の API、`SensorSelection.h` のマクロ）には `// IWYU pragma: export` を付けてあります。スクリプトは STM32 のコード（`include/hal/stm32/`、`src/stm32/`）を飛ばします。これはビルド、環境 `stm32h743` の cppcheck、環境 `native-stm32` のテストで確認します。

ビルドマトリクス（直近の実行時点）：**24/24 で警告なし**

| ボード | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck（`esp32-s3`、`stm32h743`）：プロジェクトのコードへの指摘は 0 件です。

---

## 新しいテストの書き方

1. ハードウェアなしでロジックだけを持つモジュール：直接のユニットテスト。時間はパラメーターで渡すか、`fake::advanceMs()` で進めます。
2. チップのドライバー：`I2cRig`/`SpiRig` を使います。シミュレートしたチップのレジスタ、書き込まれた値（`chip.lastWrite(reg)`）とデータの解析を検証します。式の検証には、コードのコピーではなく、データシートの基準値か独立した計算を使います。
3. FreeRTOS の無限ループのタスクを持つクラス：`fake::findTask("名前")` + `fake::runTask(task, n)`。STM32 の飛行タスクも同じように回します。
4. 別のセンサーセットや別のボードでファームウェア全体を動かす場合：別のセットを作り、`#include "../../../src/main.cpp"` の前に `SENSOR_KIT` を定義します（または `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`）。チップは `helpers/ChipEmulators.h` のものを使います。STM32 の場合は `test/native_stm32/` です。
5. オートパイロットの新しいモード：`test_sim` に閉ループ飛行のシナリオを追加します。
6. 新しいセット：`main()` を持つフォルダー `test/native/test_<名前>/test_main.cpp` を作ります。テスト間で状態が不要なセットでは、`setUp()` で `resetWorld()` を呼びます。
7. バグを見つけたら：まずそれを捕まえるテストを書き、そのあとで修正します。
