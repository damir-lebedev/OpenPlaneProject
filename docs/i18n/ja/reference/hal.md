# HAL — ハードウェア抽象化

> 🌐 このページは[ロシア語の原文](../../../reference/hal.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

HAL は、特定の MCU を知ってよい唯一の層です。インターフェースは
`include/hal/` にあり、実装は次のとおりです。

- `include/hal/esp32/` — ESP32（Arduino core 2.0.x）です。
- `include/hal/stm32/` — STM32H743（STM32duino 3.x）。**メインの実装**です。ファームウェア全体がビルドでき（`pio run -e stm32h743-devebox`）、PC 上でも動作します（`pio test -e native-stm32`）。DevEBox ボードでは SD カード、ブラックボックス、iBUS、サーボを確認済みで、センサーはまだ確認していません。
- `hal/Rtos.h` — FreeRTOS のタスク。両方のプラットフォームで同じように動きます。

この上位のコードはすべてインターフェースだけを相手にするので、別の MCU へ移植するときは、センサーを書き直すのではなく、`IBoard` の実装を新しく書くだけで済みます。

---

## namespace `ServoChannel`

**ファイル:** `hal/IBoard.h`

`IBoard::servo(channel)` に渡す出力のインデックスです。名前付きのメソッドではなくフラットなリストにしてあるため、出力を追加しても `IBoard` インターフェースは変わりません。順序は `FlightOutputs::outputInfo()` の表の行と一致します。

| 定数 | 値 |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — 貨物の投下（`Feature::PAYLOAD_DROP`） |
| `AUX2` | 6 — カメラ（`Knob::CAMERA_TILT`、`Feature::CAMERA_STAB`） |
| `COUNT` | 7 |

---

## `IBoard`

**ファイル:** `hal/IBoard.h` · **種別:** インターフェース · **実装:** `Esp32Board`、`Stm32Board`

ハードウェアへの唯一の入口です。これより上のコードは `<Wire.h>`、
`<SPI.h>`、`HardwareSerial` をインクルードせず、LEDC を直接呼ぶこともありません。

| メソッド | 説明 |
|---|---|
| `virtual void begin()` | I2C/SPI バスの一回限りの初期化。UART はそれぞれの所有者（`IBusReceiver`、GPS）が独自の速度で開き、PWM は `FlightOutputs::begin()` が開きます |
| `virtual II2CBus& i2c()` | センサー用のバス |
| `virtual ISpiBus& spi()` | SPI バス |
| `virtual II2CBus* displayI2c()` | 画面専用の2本目の I2C バス。なければ `nullptr` |
| `virtual IUartPort& rcUart()` | iBUS 受信機の UART |
| `virtual IUartPort& gpsUart()` | GPS の UART |
| `virtual IUartPort* telemetryUart()` | MAVLink 無線モデムの UART。既定は `nullptr`（ESP32 には空いている UART がありません） |
| `virtual IServoOutput& servo(uint8_t channel)` | `ServoChannel::*` のインデックスで指定する PWM 出力 |
| `virtual void setBuzzer(bool on)` | ブザー `PIN_BUZZER`。既定では何もしません |

---

## `II2CBus`

**ファイル:** `hal/II2CBus.h` · **種別:** 非仮想のヘルパーを持つインターフェース ·
**実装:** `Esp32I2CBus`、`Stm32I2CBus`

`Wire` の形をした I2C バスの抽象化です。ピンと周波数は実装がコンストラクタで固定するため、
`begin()`/`setClock()` はピンを取りません。バスに複数のデバイスがあっても、初期化はちょうど一度だけです。

| メソッド | 説明 |
|---|---|
| `begin()`、`setClock(hz)` | 初期化、周波数 |
| `beginTransmission(addr)`、`write(byte)`、`write(data, len)`、`endTransmission(sendStop = true)` | 書き込み。`endTransmission` は成功時に 0 を返します（`Wire` と同じ） |
| `requestFrom(addr, n)`、`available()`、`read()` | 読み出し |
| `bool writeRegister(addr, reg, value)` | ヘルパー: 1つのレジスタを書き込みます。`false` は NACK |
| `bool readRegisters(addr, reg, buf, count)` | ヘルパー: リピートスタート + `count` バイトの読み出し。NACK の場合、**または届いたバイト数が `count` に満たない場合**は `false`。このときバッファには触れません |
| `int readRegister(addr, reg)` | レジスタの値、または `-1` |
| `bool probe(addr)` | デバイスがそのアドレスに ACK で応答する |

不変条件: 失敗したとき、ヘルパーはバッファに書き込みません。ドライバは、ゴミ（空のバッファに対して `read()` が返す `0xFF`）ではなく、以前のデータを保持します。

---

## `ISpiBus`

**ファイル:** `hal/ISpiBus.h` · **種別:** インターフェース · **実装:** `Esp32SpiBus`、`Stm32SpiBus`

**CS の管理を行わない** SPI バスです。1本のバスに複数のデバイスがぶら下がり、
CS の切り替えは `SpiRegisterDevice` が行います。

| メソッド | 説明 |
|---|---|
| `begin()` | SCK/MISO/MOSI を設定します（ピンは実装のコンストラクタにあります） |
| `beginTransaction(clockHz, spiMode)` | `spiMode` は 0〜3（CPOL/CPHA） |
| `uint8_t transfer(data)` | 1バイトの全二重交換 |
| `endTransaction()` | トランザクションの終了 |

---

## `IUartPort`

**ファイル:** `hal/IUartPort.h` · **種別:** インターフェース · **実装:** `Esp32UartPort`、`Stm32UartPort`

`HardwareSerial` の形をした UART ですが、`begin()` が取るのは速度だけです。ピンとフォーマット（8N1）は実装が固定します。

| メソッド | 説明 |
|---|---|
| `begin(baud)` | ポートを開く |
| `int available()`、`int read()` | 受信 |
| `size_t write(byte)`、`size_t write(buffer, size)` | 送信 |
| `virtual int availableForWrite()` | 送信バッファの空き。`-1` は不明（既定）。テレメトリはこれを見て、待たずにフレームを先送りします |

---

## `IServoOutput`

**ファイル:** `hal/IServoOutput.h` · **種別:** インターフェース · **実装:** `Esp32ServoOutput`、`Stm32ServoOutput`

1つの PWM 出力です。ピンは実装が固定します。

| メソッド | 説明 |
|---|---|
| `bool attach(minUs, maxUs)` | チャンネル/タイマーを確保してピンを設定します。パルスを制限する範囲を指定します。`true` が示すのは MCU がリソースを確保したことだけで、サーボが接続されているという意味では**ありません** |
| `writeMicroseconds(us)` | パルス幅、µs（`attach` の範囲に制限されます） |
| `bool isAttached() const` | `attach()` の結果 |
| `virtual int32_t measurePulseUs()` | 診断用: ピン上の実際のパルス幅、または `-1`。既定の実装は `-1` を返します |

---

## `IFlashRegion`

**ファイル:** `hal/IFlashRegion.h` · **種別:** インターフェース · **実装:** `Esp32FlashPartition`、`SdFileRegion`

ログ（ブラックボックス）用の NOR フラッシュの領域です。消去は 4 KB のセクタ単位でしかできず（消去済みは `0xFF` として読めます）、書き込みはビットを落とすだけです。消去済みのバイトには書き込めて、同じページに分割して書くこともできます。ESP32 では、書き込みも消去も両方のコアを止めます。それが許される場面かどうかは、呼び出し側が判断します。

| メソッド | 説明 |
|---|---|
| `uint32_t size() const` | 領域のサイズ、バイト。0 は領域がないことを表します |
| `bool read(offset, data, length)` | 読み出し |
| `bool write(offset, data, length)` | 書き込み（消去済みのバイトへ） |
| `bool erase(offset, length)` | 消去。アドレスと長さは 4096 の倍数 |

`Esp32FlashPartition(const char* name)` は、パーティションテーブルにある名前で指定するデータパーティションです（`esp_partition_*`）。`begin()` がパーティションを探し（コアの起動後）、なければ `false` を返して `size() == 0` になります。

---

## `IBlockDevice`

**ファイル:** `hal/IBlockDevice.h` · **種別:** インターフェース · **実装:** `Stm32SdCard`（テストでは `fake::SdCardModel`）

512 バイトのブロックの配列として扱う SD カードです。消去はなく、ブロックは上書きできます。

| メソッド | 説明 |
|---|---|
| `uint32_t blockCount() const` | ブロック単位のサイズ。0 はカードがないことを表します |
| `bool read(block, data, count)` / `write(...)` | 連続した `count` ブロック。`data` は任意のアドレス |

## `SdFileRegion`

**ファイル:** `hal/SdFileRegion.h` · **継承元:** `IFlashRegion` · **依存先:** `IBlockDevice`、`Fat32::locate`

SD カード上のブラックボックス領域です。FAT32 のルートにあるファイル（既定は
`BLACKBOX.BIN`）で、あらかじめ PC 上で連続した1つの塊として作成し（`tools/blackbox.py
sd-prepare`）、`0xFF` で埋めておきます。ファイルは**見つける**だけで（FAT テーブルとディレクトリには触れません）、その中に生のブロックを書き込みます。
`BlackBoxStorage` から見ると、ESP32 のフラッシュパーティションと同じ `IFlashRegion` です。

| メソッド | 説明 |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` は領域の上限です。電源投入時のセクタ照合にかかる時間は、これに比例して長くなります |
| `Fat32::Result begin()` | ファイルを探します。`Ok` なら `size() > 0`。そうでなければ理由（`Fat32::describe()`）: カードがない、FAT32 ではない、ファイルがない、断片化している、空 |
| `size()` | ファイル（`maxBytes` 以下）を 4 KB のセクタ単位に切り捨てた値。0 は領域がないことを表します |
| `read` / `write` | 任意のオフセットと長さ。不完全なブロックは読み出して補い、丸ごと書き込みます。直前に書いたブロックは覚えておきます（ライトスルーキャッシュ）。そのため、256 バイトのページを連続して書いてもカードを読み直しません。電源が落ちても、すでに `write()` から戻った分は失われません |
| `erase(offset, length)` | 4096 の倍数。`0xFF` を書き込みます（カードの内部に独自の消去があり、外側からは不要です） |

## `IRegisterDevice`

**ファイル:** `hal/RegisterDevice.h` · **種別:** インターフェース ·
**実装:** `I2cRegisterDevice`、`SpiRegisterDevice`

「8ビットレジスタの集まり」です。センサーのドライバは一度書くだけで済み、バスは `SensorSelection.h` でオブジェクトを作るときに選びます。

| メソッド | 説明 |
|---|---|
| `virtual void begin()` | デバイスの信号線を準備します（SPI では CS）。既定では何もしません |
| `virtual bool probe()` | デバイスが応答した（SPI では常に `true`。ACK がないので ID レジスタで確認します） |
| `virtual bool writeRegister(reg, value)` | レジスタの書き込み |
| `virtual bool writeRegisters(reg, data, count)` | 連続書き込み（アドレスの自動インクリメント） |
| `virtual bool readRegisters(reg, buffer, count)` | `count` バイトの連続読み出し。`false` のときバッファには触れません |
| `int readRegister(reg)` | 値、または `-1`（非仮想のヘルパー） |

---

## `I2cRegisterDevice`

**ファイル:** `hal/RegisterDevice.h` · **継承元:** `IRegisterDevice`

`II2CBus` 上の、7ビットアドレスのデバイスです。すべての操作は `II2CBus` のヘルパーに委譲します。

| メソッド | 説明 |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` はチップの2つ目のアドレスです（SDO/SA0 ピン）: LSM6DSV 0x6A/0x6B、ICM-45686 0x68/0x69、SPL06 0x76/0x77、BMP581 0x46/0x47 |
| `begin()` | 主アドレスが応答せず、予備のアドレスが応答する場合は、以後は予備のアドレスで動作します |
| `probe()`、`writeRegister()`、`writeRegisters()`、`readRegisters()` | → `II2CBus(address, …)` のヘルパー |
| `uint8_t getAddress() const` | デバイスの現在のアドレス |

---

## `SpiRegisterDevice`

**ファイル:** `hal/RegisterDevice.h` · **継承元:** `IRegisterDevice`

`ISpiBus` 上の、専用の CS ピンを持つデバイスです。Bosch/InvenSense のプロトコルで、読み出しはアドレスにビット `0x80` を立て、書き込みはビット 7 を下げて行います。

| メソッド | 説明 |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` は、アドレスの後、データの前にチップが出力する「ゴミ」バイトの数です（BMP388 は 1、ICM42688 は 0）。`mode` は SPI モード 0〜3 |
| `begin()` | `pinMode(cs, OUTPUT)`、CS = HIGH |
| `probe()` | 常に `true` |
| `writeRegister(reg, value)` | CS↓、`reg & 0x7F`、`value`、CS↑。常に `true` |
| `readRegisters(reg, buf, n)` | CS↓、`reg \| 0x80`、`dummyReadBytes` を読み飛ばし、`n` バイト、CS↑。常に `true` |

各操作は、独立した `beginTransaction(clockHz, spiMode)` …
`endTransaction()` のトランザクションです。

---

## `Esp32Board`

**ファイル:** `hal/esp32/Esp32Board.h` · **継承元:** `IBoard`

ESP32 の具体的なペリフェラルのオブジェクトを作り、`Config.h` のピンを把握している唯一の場所です。

| フィールド | 型 | 内容 |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `PIN_I2C_SDA/SCL` 上の `Wire`、400 kHz |
| `displayBus` | `Esp32I2CBus` | `PIN_I2C2_SDA/SCL` 上の `Wire1`。`SOC_I2C_NUM > 1` の場合のみ |
| `spiBus` | `Esp32SpiBus` | グローバルの `SPI` |
| `rcSerial`、`rcPort` | `HardwareSerial(1)`、`Esp32UartPort` | `PIN_IBUS` 上の iBUS、RX のみ |
| `gpsSerial`、`gpsPort` | `HardwareSerial(UART_NUM_GPS)`、`Esp32UartPort` | `PIN_GPS_RX/TX` 上の GPS |
| `servos[7]` | `Esp32ServoOutput` | `ServoChannel` の順に並ぶ LEDC チャンネル 0〜6（AUX1/AUX2 は `PIN_AUX1/2`。配線されている場合） |

| メソッド | 説明 |
|---|---|
| `begin()` | `i2cBus.begin()`、`spiBus.begin()`、2本目のバスがあれば続けて `displayBus.begin()`。ブザーのピン |
| `setBuzzer(on)` | ピンが配線されていれば `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | `hasDisplayBus()` なら `&displayBus`、そうでなければ `nullptr` |
| `static constexpr bool hasDisplayBus()` | 2本目のバスのピンが両方とも 0 以上であること。`SOC_I2C_NUM > 1` の場合にのみ存在します（フィールド `displayBus` も同様）。C3 には I2C コントローラが1つしかありません |
| そのほか | 対応するフィールドを返します |

---

## `Esp32I2CBus`

**ファイル:** `hal/esp32/Esp32I2CBus.h` · **継承元:** `II2CBus`

`TwoWire`（`Wire` または `Wire1`）の薄いラッパーです。

| メソッド | 説明 |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | パラメータを保持します |
| `begin()` | `wire.begin(sda, scl, hz)` と `wire.setTimeOut(TIMEOUT_MS)`。`wire.begin()` を呼ぶのはここだけです |
| そのほか | `TwoWire` へそのまま委譲します |

`TIMEOUT_MS = 5`: IMU の 14 バイトを 400 kHz で読むのにかかるのは約 0.4 ms です。ノイズでハングしたトランザクションがあると、そのままではループが標準の 50 ms も止まってしまいます。

---

## `Esp32SpiBus`

**ファイル:** `hal/esp32/Esp32SpiBus.h` · **継承元:** `ISpiBus`

グローバルの `SPI` のラッパーです。`begin()` → `SPI.begin(sck, miso, mosi, -1)`（CS は各デバイスが持ちます）。`beginTransaction()` は `SPISettings(hz, MSBFIRST,
SPI_MODEn)` を作ります。`spiModeOf()` は 0〜3 を Arduino の定数に変換し、未知の値は `SPI_MODE0` になります。

---

## `Esp32UartPort`

**ファイル:** `hal/esp32/Esp32UartPort.h` · **継承元:** `IUartPort`

`HardwareSerial` のラッパーです。`begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`、`tx = -1` なら受信専用です。残りは委譲です。

---

## `Esp32ServoOutput`

**ファイル:** `hal/esp32/Esp32ServoOutput.h` · **継承元:** `IServoOutput`

LEDC を使って直接 PWM を出力します（Arduino core 2.x の `ledcSetup/ledcAttachPin/ledcWrite`）。ライブラリ ESP32Servo は**使用しません**。S3 ではバージョン 3.2.1 が MCPWM
ブロックを取り違えていました（GPIO6/7 が GPIO4/5 を繰り返していました）。

| 定数 | 値 |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384（1ステップあたり約 1.2 µs） |

| メソッド | 説明 |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | ピン `< 0` は出力が配線されていないことを表します |
| `attach(minUs, maxUs)` | 範囲を保持します。ピン < 0 なら `false`。そうでなければ `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | attached でなければ何もしません。そうでなければ `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | 同じ GPIO の入力バッファを有効にし（`PIN_INPUT_ENABLE`。出力には触れません）、`pulseIn(pin, HIGH, 30 ms)` を測定します。パルスがなければ `-1` |

チャンネル 2n と 2n+1 は LEDC のタイマーを共有しますが、すべての出力が 50 Hz なので競合はありません。

---

# STM32H743 向けの実装

次世代のボードは STM32H743VIT6（Cortex-M7 480 MHz、フラッシュ 2 MB、
RAM 1 MB）です。ファームウェア全体がビルドでき（env `stm32h743` は PlatformIO のボード
`weact_mini_h743vitx`、`stm32h743-devebox` は DevEBox H743 で、コンソールは
USB CDC）、cppcheck と PC 上のテストにも通ります（env `native-stm32`。STM32duino のフェイク層を使用）。**センサーなし**の DevEBox ボードでは、起動、SD カード、ブラックボックス（[ボード上のテスト](../TESTING.md#stm32-ボード上のテスト)）に加え、iBUS の受信、ARM、サーボとモーターへの
PWM を確認済みです。機体は送信機からマニュアルモードで操縦でき、その様子は動画に撮ってあります。センサーはまだボードに接続していません。ピン配置は [`Config.h`](config.md#stm32h743vit6board_stm32h743) の `BOARD_STM32H743` ブロックにあります。

この層が隠している、ESP32 との共通の違いは次のとおりです。

- **ペリフェラルはコアが選びます。** STM32duino は、バリアントの `PeripheralPins` テーブルにあるピン番号から、コントローラ（I2C1/I2C2、SPI2、USART3、UART4、UART7、TIMx）を自分で探し出します。そのため `Config.h` には UART やチャンネルの番号がありません。
- **ピン番号**は、GPIO ではなく、バリアントの「Arduino ピン」（`PA0`、`PD14`…）です。アナログピンでは `0xC0 + N` になるため、STM32 ブロックのピンは `int16_t` です。
- **UART のピン**は、`begin()` ではなく、`Uart(rx, tx)` オブジェクトを作るときに指定します。

## `Stm32Board`

**ファイル:** `hal/stm32/Stm32Board.h` · **継承元:** `IBoard`

`Esp32Board` と同じ役割を、STM32duino の上で担います。

| フィールド | 型 | 内容 |
|---|---|---|
| `displayWire` | `TwoWire` | 2つ目の I2C コントローラ（グローバルの `Wire` はセンサーが使用中）。これへの参照を持つ `displayBus` より前に宣言されています |
| `i2cBus` | `Stm32I2CBus` | `PIN_I2C_SDA/SCL`（I2C2: PB11/PB10）上の `Wire`、400 kHz |
| `displayBus` | `Stm32I2CBus` | `PIN_I2C2_SDA/SCL`（I2C1: PB9/PB8）上の `displayWire`。2本目のバスは常にあります |
| `spiBus` | `Stm32SpiBus` | `PIN_SENSOR_SPI_*`（SPI2）上のグローバルの `SPI` |
| `rcSerial`、`rcPort` | `Uart`、`Stm32UartPort` | iBUS: UART7、RX は `PIN_IBUS`（PE7）、TX は `PIN_IBUS_TX`（PE8。iBUS-SENS 用に予約） |
| `gpsSerial`、`gpsPort` | `Uart`、`Stm32UartPort` | GPS: USART3、`PIN_GPS_RX/TX`（PD9/PD8） |
| `telemetrySerial`、`telemetryPort` | `Uart`、`Stm32UartPort` | MAVLink 無線モデム: UART4、`PIN_TELEM_RX/TX`（PD0/PD1） |
| `servos[7]` | `Stm32ServoOutput` | `ServoChannel` の順（AUX1 は PD15/TIM4、AUX2 は PE9/TIM1） |

| メソッド | 説明 |
|---|---|
| `begin()` | `i2cBus.begin()`、`spiBus.begin()`、`displayBus.begin()`、ブザーのピン |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | 常に `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | `Config.h` のピンをコア API の型に変換します |
| そのほか | 対応するフィールドを返します |

## `Stm32I2CBus`

**ファイル:** `hal/stm32/Stm32I2CBus.h` · **継承元:** `II2CBus`

| メソッド | 説明 |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | パラメータを保持します |
| `begin()` | `setSDA()`/`setSCL()`（`begin()` の前でのみ有効）、`wire.begin()`、`wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`。`size_t` の結果は `uint8_t` に変換します |
| そのほか | `TwoWire` へそのまま委譲します |

STM32duino では、トランザクションのタイムアウトはメソッドではなく、マクロ
`I2C_TIMEOUT_TICK`（ms、既定は 100）で指定します。env `stm32h743` では、フラグ
`-D I2C_TIMEOUT_TICK=5` で設定しています。`Esp32I2CBus` の `TIMEOUT_MS` と同じ理由です。

## `Stm32SpiBus`

**ファイル:** `hal/stm32/Stm32SpiBus.h` · **継承元:** `ISpiBus`

`SPIClass&` のラッパーです。`begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`。ハードウェアの NSS は使わず、ESP32 と同様に CS は `SpiRegisterDevice` が切り替えます。
`beginTransaction()` は `SPISettings(hz, MSBFIRST, SPIMode)` を作ります。`spiModeOf()` は
0〜3 を `SPI_MODEn` に変換し、未知の値は `SPI_MODE0` になります。

## `Stm32UartPort`

**ファイル:** `hal/stm32/Stm32UartPort.h` · **継承元:** `IUartPort`

`HardwareSerial&` のラッパーです（STM32duino 3.x では抽象基底クラスの
`arduino::HardwareSerial` で、具体的な `Uart` オブジェクトは `Stm32Board` が作ります）。
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`。`availableForWrite()` は
`HardwareSerial` のものです。バッファ（env の `SERIAL_RX/TX_BUFFER_SIZE`）は、受信が 256
バイト（NAV-PVT のフレームは 100 バイトあり、標準の 64 では足りません）、送信が 1024 です（ログの行と
MAVLink のフレームを待たずに出すため）。

## `Stm32ServoOutput`

**ファイル:** `hal/stm32/Stm32ServoOutput.h` · **継承元:** `IServoOutput`

`HardwareTimer` を通じたタイマーのハードウェア PWM で、50 Hz です。パルスはタイマーが割り込みも CPU も使わずに生成します。STM32 の `Servo` ライブラリは、1つのタイマーの割り込みからピンを操作するためジッタが出ますが、それとは異なります。

| 定数 | 値 |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — 出力が占有できる異なるタイマーの数（現在は TIM2 と TIM4 が使用中） |

| メソッド | 説明 |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | ピン `< 0` は出力が配線されていないことを表します |
| `attach(minUs, maxUs)` | タイマーとチャンネルは、ピンから `PinMap_TIM`（`pinmap_peripheral`、`STM_PIN_CHANNEL`）で求めます。`analogWrite()` と同じです。ピンにタイマーがない、またはプールが尽きた場合は `false`。そうでなければ `setMode(PWM1)`、比較値 0（最初の書き込みまではパルスなし）、`resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`。比較レジスタはプリロード付きなので、値は次の周期から有効になります |
| `measurePulseUs()` | ピンを再設定せずに `pulseIn(pin, HIGH, 30 ms)`。STM32 では、IDR レジスタが代替機能モードでもレベルを読み取れます |
| `static acquireTimer(TIM_TypeDef*)` | 共有プール: **TIMx ごとに `HardwareTimer` を1つ**だけ持ちます。同じタイマーに2つ目のオブジェクトを作ると、コアのハンドラ（`HardwareTimer_Handle[index]`）を上書きしてしまいます。周期はタイマー上の最初の出力で決まります。`setOverflow(MICROSEC_FORMAT)` が分周比を選び、タイマーのクロックが 240 MHz のときのステップは約 0.3 µs です |

## `Stm32FlashStorage`

**ファイル:** `hal/stm32/Stm32FlashStorage.h` · **継承元:** `IFlashStorage`（[storage.md](storage.md)）

STM32 での `KeyValueStore` の保存先です。フラッシュ（バンク 2）の最後のセクタを、
STM32duino の EEPROM エミュレーション（`eeprom_buffer_fill/flush`、8 KB のバッファのうち先頭の `KeyValueStore::CAPACITY` バイトを使用）を通じて使います。

| メソッド | 説明 |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + バッファを1バイトずつ読み出し |
| `write(src, n)` | **高速**: `noInterrupts()` の下でイメージを自前のバッファにコピーし、「書き込み待ち」フラグを立てます。飛行タスクの `KvPreferences::end()` から呼ばれます |
| `bool service()` | **低速**: （`noInterrupts()` の下で）エミュレーション用のバッファへスナップショットを取り、`eeprom_buffer_flush()` を実行します。128 KB のセクタの消去（数秒）と書き込みです。バックグラウンドのタスク `storage` からのみ呼びます |
| `hasPending()`、`flushCount()` | 診断用 |
| `static instance()`、`static store()` | 保存先と、ファームウェア共通の `KeyValueStore` |

飛行が止まらない理由: 設定のセクタはバンク 2、コードはバンク 1 にあり、H7 のフラッシュは一方のバンクに書き込んでいる間にもう一方を読み出せます。また、飛行タスクはバックグラウンドのタスクに優先して割り込みます。

## `compat/Preferences.h`

**ファイル:** `hal/stm32/compat/Preferences.h` — env `stm32h743`（および
`native-stm32`）では、ディレクトリ `compat/` がライブラリより前に `-I` へ入るため、センサードライバ、オートトリマー、ログ設定の `#include <Preferences.h>` はこれを見つけます。`class Preferences : public KvPreferences` を
`Stm32FlashStorage::store()` の上に載せたもので、API は ESP32 の NVS と同じです（[storage.md](storage.md#kvpreferences)）。

## `Stm32SdCard`

**ファイル:** `hal/stm32/Stm32SdCard.h` · **継承元:** `IBlockDevice` · **ピン:** `src/stm32/sd_msp.cpp`

SDMMC1 上の SD カードです。4 ビットバスで、`HAL_SD` をポーリングモード（DMA も割り込みも不使用）で動かし、**ハードウェアのフロー制御付き**です。飛行タスクがブロックの途中で書き込みタスクに割り込むため、フロー制御がないと FIFO があふれました（`HAL_SD_ERROR_RX_OVERRUN`、0x20）。ボード上では、コンソールと記録が数秒間固まる症状として現れました。ピン PC8〜PC11（D0〜D3）、PC12（CK）、PD2（CMD）は、DevEBox と WeAct の µSD スロットです。
SDMMC コアは PLL1Q = 48 MHz でクロックされ、`ClockDiv = 1` → **24 MHz** になります。
24 MHz で最初の読み出しに失敗した場合は、12 と 6 を試します。

| メンバー | 説明 |
|---|---|
| `bool begin()` | バスを立ち上げ、カードを識別し、試し読みをします。`false` はカードがないことを表し、`initError()` がコードを返します |
| `read` / `write` | 1回あたり 4 KB 以下の塊で行います（短い休止あり）。4 の倍数でないアドレスは、アラインしたバッファ経由でコピーします（HAL は FIFO をワード単位で読みます）。失敗したら1回だけ再試行します |
| 待機 | 書き込みの後に次のアクセスをする前に、カードが転送状態へ戻るのを待ちます（`Rtos::sleepMs(1)`。バックグラウンドのタスクが飢えないようにするため）。最大 1 秒です。読み出しの後は余分な状態要求を送りません。電源投入時の照合では数万セクタを読むためです |
| `blockCount()`、`cardType()`、`clockDivider()`、`lastErrorCode()` | 状態行用 |
| `readOps`、`writeOps`、`errors`、`retries` | カウンタ |

## `ResetCause`

**ファイル:** `hal/ResetCause.h` · `readResetCause()`、`isCrashReset()`、`resetCauseName()`

再起動の原因を、両方のボードで同じ形で返します。ESP32 は `esp_reset_reason()`、
STM32 は `RCC->RSR` のフラグです（一度だけ読み出してクリアします。H7 では `PINRSTF` がどのリセットでも立つため、より具体的な原因を先に調べます:
ウォッチドッグ → 電源投入 → 電圧低下 → ソフトウェアリセット）。パニック、ウォッチドッグ、電源電圧の低下は「異常」として扱われ、これらではブラックボックスがただちに記録を始めます。

## `Rtos`

**ファイル:** `hal/Rtos.h` · namespace

| メンバー | 説明 |
|---|---|
| `PRIORITY_BACKGROUND`（1）、`PRIORITY_TELEMETRY`（2）、`PRIORITY_FLIGHT`（5） | タスクの優先度 |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 は `xTaskCreatePinnedToCore(..., コア 0)` でスタックはバイト単位、STM32 は `xTaskCreate` でスタックはワード単位に変換します。`handle` は `xTaskNotifyGive` 用です |
| `void sleepMs(ms)` | `vTaskDelay`。スケジューラの開始前（STM32 の `setup()`）は `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 は `portMUX` のスピンロック、STM32 は `taskENTER_CRITICAL()`。内部ではバイトのコピー（ブラックボックスのキュー）だけを行います |
| `uint32_t freeHeapBytes()` | ESP32 は `ESP.getFreeHeap()`、STM32 は `xPortGetFreeHeapSize()` |

## エントリポイント `src/stm32/main.cpp`

ファームウェア全体です。`src/main.cpp` と同じオブジェクトを使い、Wi-Fi の代わりに MAVLink テレメトリ、
`loop()` の代わりに FreeRTOS のタスクを使います（[application.md](application.md#srcstm32maincpp--stm32h743)）。
