# CONFIG — `Config`、`Channels`、`Controls`

> 🌐 このページは[ロシア語の原文](../../../reference/config.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

設定レイヤーは `constexpr` の定数だけで、コードは含みません。各クラスのロジックに「マジックナンバー」のようなピン、タイムアウト、しきい値を持たせてはいけません。特定の機体やボードに合わせて変える可能性のあるものは、すべてここに置きます。

---

## namespace `Config`

**ファイル：** `include/config/Config.h` · **依存先：** `<stdint.h>` ·
**利用元：** ほぼすべてのレイヤー。

### ピン（ボードごとに異なる）

ピンのブロックは、`platformio.ini` の `[env:*]` が定義するマクロで選ばれます（`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`）。マクロがなければ `#error` になります。STM32 のブロックは
[後述](#stm32h743vit6board_stm32h743)します。

| 定数 | 型 | 用途 | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | エルロン | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | エレベーター | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | モーターのコントローラー | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | ラダー＋ホイール。`-1` は出力無効 | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS 受信機の RX（UART1） | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | センサーのバス（`Wire`） | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | OLED のバス（`Wire1`）。`-1` はなし | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | 共用の SPI バス | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | SPI 接続の IMU の CS | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | SPI 接続の気圧計の CS | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | GPS の UART。TX が `-1` なら受信専用 | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | GPS 用のハードウェア UART の番号 | 2 | 0 | 2 |
| `PIN_AUX1`、`PIN_AUX2` | `int8_t` | サーボ出力：ペイロード投下、フラップ。`-1` はなし | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | トランジスタ経由のブザー。`-1` はなし | 38 | −1 | 2 |
| `PIN_AUX3`、`PIN_LIGHT`、`PIN_VBAT_ADC`、`PIN_CURRENT_ADC`、`PIN_TELEM_TX/RX` | `int8_t` | **S3 のみ：** フライトコントローラー基板用の予約（[FC_BOARD.md](../FC_BOARD.md)） | 47, 21, 8, 3, 9/10 | — | — |

センサーの SPI バスは `PIN_SPI_*` ではなく `PIN_SENSOR_SPI_*` という名前です。
STM32duino のコア（および他の Arduino コア）では `PIN_SPI_SCK/MISO/MOSI` がバリアントのマクロであり、`Config` の定数を置き換えてしまうためです。

<a id="stm32h743"></a>

#### STM32H743VIT6（`BOARD_STM32H743`）

基板はまだありません。ピン配置は**実機では未確認**です（ファームウェアは PC 上で実行、env `native-stm32`）。ピンは WeAct MiniSTM32H743VITx
（PlatformIO の env `stm32h743` の基板）の空きピンから選び、STM32duino バリアントの
`PeripheralPins` の表と照合しました。値はバリアントのマクロ（`PA0`…）なので、
`Config.h` の冒頭で `#if defined(BOARD_STM32H743)` の下に `<Arduino.h>` を取り込みます。すべてのピンの型は `int16_t` です（アナログピンの番号は `0xC0 + N`）。UART の番号はなく、ペリフェラルはコアがピンから選びます。

| 定数 | ピン | ペリフェラル |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7（TX は iBUS-SENS 用の予約） |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2：センサー |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1：ディスプレイ（WeAct ではカメラ用コネクタ） |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1：ペイロード／カメラ |
| `PIN_BUZZER` | PE15 | GPIO：ブザー |
| `PIN_VBAT_ADC`、`PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11：予約 |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4：MAVLink の無線モデム（同じピンは FDCAN1 でもある） |

コンソール `Serial` は LPUART1（PA9 TX / PA10 RX）で、バリアントの既定です。

### iBUS とリンク喪失

| 定数 | 値 | 意味 |
|---|---|---|
| `IBUS_CHANNELS` | 10 | フレームのうち使用するチャンネル数 |
| `IBUS_FRAME_LENGTH` | 32 | フレームの長さ（バイト） |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | フレームのヘッダー |
| `IBUS_BAUDRATE` | 115200 | UART の速度 |
| `RX_TIMEOUT_US` | 500 000 | これより長く正しいフレームが来ない場合、リンク喪失 |
| `RX_FAILSAFE_THROTTLE_US` | 950 | スロットルがこれより低い場合、受信機が送信機の failsafe を通知している |

### GPS

| 定数 | 値 | 意味 |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | NAV-PVT がこれより古ければ `UbloxM10_Gps::isAvailable() == false` |

### PWM の範囲と舵面のストローク

| 定数 | 値 | 意味 |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | 標準の RC パルス（µs） |
| `AILERON_MAX_US`、`ELEVATOR_MAX_US`、`RUDDER_MAX_US` | 500 / 500 / 300 | スティックをフルストロークにしたときの中心からの舵角（µs）。ラダーが小さいのは、同じサーボに降着装置のホイールが付いているため |
| `THROTTLE_LIMIT_PCT` | 100 | ESC へのスロットルの上限（%）。スティックにもオートパイロットにも同じ値が適用される（`FlightController::capThrottle`）。出力の弱い 3S1P のバッテリーでのテストベンチ試験では 50 にしていた。テストはこの値から期待出力を計算する |

### フラップ（フラッペロン）

| 定数 | 値 | 意味 |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 がこれより上ならフラップ展開（1500 ではない。最初のフレームが来るまでチャンネルは 1500 のため） |
| `FLAPS_DEPLOYED_US` | 220 | 各エルロンの下げ舵角（µs）（MG90S のホーンで約 20°） |
| `FLAPS_TRANSITION_MS` | 1000 | 完全に展開／格納するまでの時間 |

### サーボの向き

`AILERON_LEFT_REVERSED`、`AILERON_RIGHT_REVERSED`（`true` はエルロンのサーボが鏡像配置）、`ELEVATOR_REVERSED`（`true`）、
`RUDDER_REVERSED` は、リバースを設定する唯一の場所です。`ControlMixer` は物理的な符号で計算し、符号の反転はここだけで行うため、スティックとオートパイロットの向きが食い違うことはありません。送信機側でのリバースは**してはいけません**。

### センサーの取り付け

| 定数 | 値 | 意味 |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | IMU チップの軸を鉛直軸まわりに回す角度（0/90/180/270）、つまりチップの X 軸が向く方向。NVS に取り付け校正 `o` がない間だけ使われる |
| `MAG_ROTATION_CW_DEG` | 0 | コンパスについて同じ設定（コンパスには取り付け校正がない） |

### ARM

| 定数 | 値 | 意味 |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 がこれより上なら ARM スイッチがオン |
| `THROTTLE_LOW_US` | 1050 | スロットルがこれより低ければ「スロットル最低」とみなし、ARM できる |

### Failsafe

| 定数 | 値 | 意味 |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | 舵面のニュートラル |
| `FAILSAFE_THROTTLE` | 1000 | モーター停止 |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | 空中でリンクを失ったときの滑空のバンク角 |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | 滑空のピッチ（水平線よりやや下） |
| `FAILSAFE_RTH` | `true` | GPS とホームポイントがある場合、空中でのリンク喪失では滑空ではなくモーターを使ってホームへ帰還する |

### スイッチ、ピトー管、オートパイロット

すべてのモードと機能の数値は、詳しいコメントとともに `Config.h` にあります。パイロットにとっての意味は [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md) を参照してください。

| グループ | 定数 |
|---|---|
| スイッチ | `SWITCH_ON_US` = 1750（チャンネルがこれより上ならスイッチがオン。1500 ではないのは、最初のフレームが来るまで何も有効にならないようにするため） |
| ピトー管 | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| スタビライズ | `MAX_BANK_DEG` 45（ノブ 15…60）, `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| ナビゲーション | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| 高度 | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| スロットルと速度 | `CRUISE_THROTTLE_PCT` 55（30…85）, `CRUISE_AIRSPEED_MS` 14（10…22）, `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| 失速 | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| 旋回円とホーム | `LOITER_RADIUS_M` 50（25…150）, `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| ジオフェンス | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| 手投げ発進 | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| 着陸 | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| ソアリング | `SOAR_*`：滑空 −3°、サーマル > 0.5 m/s が 1.5 秒、旋回 25°、離脱 < −0.2 m/s が 8 秒、30 m 未満では 100 m までモーター使用、400 m を超えたらホームへ |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| オートトリム | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, 地上での保存：`AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| 協調 | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| 機能 | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### ループ、Wi-Fi、デバッグ

| 定数 | 値 | 意味 |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | 飛行ループの周期（500 Hz）。`PidController` の公称の `dt` でもある |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | ダッシュボードのアクセスポイント（パスワードは弱い。テストベンチ用のツール） |
| `WEB_SERVER_PORT` | 80 | HTTP ポート |
| `TELEM_BAUDRATE` | 57600 | MAVLink 無線モデムの速度（SiK の既定値） |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | MAVLink での機体のアドレス |
| `DEBUG_INTERVAL_MS` | 100 | `DebugLogger` がログのチャンネルを確認する頻度 |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | 「変化時」モードでの RC/PWM のチャタリングの許容値 |

### ブラックボックス

詳しくは [BLACKBOX.md](../BLACKBOX.md) を参照してください。

| 定数 | 値 | 意味 |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | PSRAM 上の記録キュー（PSRAM がなければ内蔵メモリ上） |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743：RAM 上のキュー。10 秒分のプリ録画と、カードの遅延に備えた余裕 |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743：SD カード上のファイルと、使用する部分の上限（起動時の照合時間は領域とともに長くなる） |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | 開始（ARM ＋ スロットル）前と DISARM 後の記録時間 |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | ARM 済みでモーターが止まり、機体がこの時間静止していたら停止 |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | 「静止」とみなす条件 |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | 異常な再起動のあと、最低でもこの時間は記録する |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | 常に確保しておく消去済みの領域。古いフライトは地上でまとめて消去する |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | 消去の間の休止 |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU を N 周期に 1 回記録（1 なら 500 Hz） |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | フライトコントローラー基板上のバッテリー（56k/10k）と電流センサー（10k/15k）の分圧比 |

---

## namespace `Channels`

**ファイル：** `include/config/Channels.h` · **依存先：** `<stdint.h>`

物理的なチャンネル番号と用途を結びつける唯一の場所です。値は `RcChannelState` での
**インデックス**（0 始まり）です。

| 定数 | インデックス | チャンネル | FS-i6 の操作部 | 用途 |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | 右スティック ←→ | ロール |
| `ELEVATOR` | 1 | CH2 | 右スティック ↑↓ | ピッチ（2000 = 自分から遠ざける = 機首下げ） |
| `THROTTLE` | 2 | CH3 | 左スティック ↑↓ | スロットル |
| `RUDDER` | 3 | CH4 | 左スティック ←→ | ラダー＋ホイール |
| `ARM` | 4 | CH5 | SwA | ARM スイッチ（再割り当て不可） |
| `SWB` | 5 | CH6 | SwB | `Controls.h` のテーブルによる（既定ではフラップ） |
| `SWC` | 6 | CH7 | SwC（3 ポジション） | 既定ではモード MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | 既定では RTH |
| `VRA` | 8 | CH9 | VrA | 既定では `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | 既定では `CRUISE_SPEED` |
| `COUNT` | 10 | | | チャンネル数 |

---

## namespace `Controls`

**ファイル：** `include/config/Controls.h` · **依存先：** `ControlBinding.h`、`Channels`

`constexpr Binding BINDINGS[]` は、各スイッチとノブの役割を**チャンネルごとに 1 行**で記述したものです（`Bind::modes/mode/feature/knob`、
[autopilot.md](autopilot.md#bindingbindbindingcheck) を参照）。隣にはコメントアウトされたそのまま使えるアイデアがあります。3 つの `static_assert` が、ビルド時にテーブルの誤りを検出します：テーブル内のスティックや ARM、範囲外のチャンネル、チャンネルの重複、モード選択スイッチが 2 つ以上あること。
