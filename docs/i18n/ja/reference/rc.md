# RC — 送信機のコマンドの受信

> 🌐 このページは[ロシア語の原文](../../../reference/rc.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。 この翻訳は AI によるもので、ネイティブスピーカーによる確認は行っていません。誤りを見つけたら、[Damir Lebedev](https://github.com/damir-lebedev) までご連絡いただくか、[Issue](https://github.com/damir-lebedev/OpenPlaneProject/issues) でお知らせください。

[← リファレンス](README.md)

RC レイヤーは、UART のバイト列をチャンネル値と「リンクなし」のフラグに変換します。機体、ARM、failsafe の動作、サーボのことは何も知りません。受信機のプロトコル（iBUS、SBUS、CRSF）は `Config::RX_PROTOCOL` で決まります。新しいプロトコルは、既存の 3 つのフレームパーサーの隣にもう 1 つ加えるだけです。

---

## `RcChannelState`

**ファイル：** `rc/RcChannelState.h` · **依存先：** `Config`、`Channels`

受信機の 10 チャンネル（µs）のスナップショットで、制御ロジックは持ちません。

| メソッド | 説明 |
|---|---|
| `RcChannelState()` | `reset()` を呼ぶ |
| `void reset()` | 安全な値：全チャンネルを `PWM_CENTER`、スロットルを `PWM_MIN` にする |
| `uint16_t get(uint8_t index) const` | チャンネルの値。範囲外のインデックスは `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | チャンネルを書き込む。範囲外のインデックスは無視する |
| `const uint16_t* data() const` | 配列全体（デバッグ用） |

---

## `RcInput`

**ファイル：** `rc/RcInput.h` · **種別：** 静的関数の集まり · **依存先：** `Config`

RC 信号の共通の変換処理です。

| メソッド | 説明 |
|---|---|
| `static uint16_t clamp(uint16_t value)` | `PWM_MIN..PWM_MAX` に制限する |
| `static uint16_t reversed(uint16_t value)` | チャンネルを逆に：`3000 − clamp(value)`（1000 ↔ 2000）。`Config::CHANNEL_REVERSED` 用 |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 線形変換：1000 → `−max`、1500 → 0、2000 → `+max`（入力は先に制限される）。`reverse` は符号を反転する。結果は ±`max` に制限される |

例：`centered(1750, 500) == 250`、`centered(1750, 500, true) == -250`。

---

## `RcReceiver`

**ファイル：**`rc/RcReceiver.h` · **依存先：**`IUartPort`、`IBusParser`、`SbusParser`、`CrsfParser`、`RcChannelState`、`RcInput`、`Config`、`Channels`

RC 受信機：プロトコルの形式で UART を開き、バイトをパーサーに渡し、全プロトコル共通の処理を行います——先頭の `RC_CHANNELS` = 10 チャンネル、チャンネル反転、フレームタイムアウト、failsafe、カウンタ。

| プロトコル（`Config::RX_PROTOCOL`） | 受信機 | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B、FS-iA10B | 115200、8N1 |
| `SBUS` | FrSky、Futaba、RadioMaster、Jumper などほとんどの受信機 | 100000、8E2、反転信号——UART 自身が反転 |
| `CRSF` | ExpressLRS、TBS Crossfire、Tracer | 420000、8N1 |

| メソッド | 説明 |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | コンストラクタではポートを開かない。テストでは任意のプロトコルを渡せる |
| `void begin()` | プロトコルの形式で UART を開き（`IUartPort::begin(baud, format)`）、タイムアウトを「今」から数える |
| `void update()` | UART にたまったものをすべて読む。毎周期呼ぶ |
| `const RcChannelState& getState() const` | 最後に受信したチャンネル |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | まだフレームが 1 つも来ていない、**または**最後のフレームが `RX_TIMEOUT_US` より古い |
| `bool isFailsafeReported() const` | 受信機がリンク喪失を通知：SBUS フレームの failsafe フラグ、CRSF のリンク品質 0 %、またはスロットル < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | 最後の正しいフレームの `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 受理したフレームと破棄したフレーム（CRC、フレーム末尾）のカウンタ |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | プロトコル。名前は `"iBUS"`、`"SBUS"`、`"CRSF"` |
| `int16_t getLinkQuality() const` | 無線リンク品質、%——CRSF のみ、それ以外は −1 |
| `bool isPortReady() const` | UART が必要な形式で開けた。`false`——ポートが対応していない（反転なしの SBUS）ため、リンクは得られない |

自由関数 `rxProtocolName(Config::RxProtocol)` はプロトコル名を返します。ファームウェアのパスポート（`@id` の `rx` フィールド）が出力するのも同じ名前です。

不変条件：

- 最初の正しいフレームまでは `isSignalLost() == true`——既定値（すべて 1500）を送信機の指令とはみなしません。
- failsafe のフラグはチャンネルフレームの**たびに**計算し直されます——正常なスロットルの最初のフレームでリンクが戻ります（CRSF ではリンク品質が 0 より大きい統計フレームでも）。
- `Config::CHANNEL_REVERSED` のチャンネルは反転した値で保存されます（`RcInput::reversed`）。failsafe はその前に**生の**スロットルで判定し、CH3 の反転は `static_assert` で禁止されています。

---

## `IBusParser`

**ファイル：**`rc/IBusParser.h` · **依存先：**`Config`

FlySky iBUS のバイト単位パーサー：`bool feed(uint8_t)` は CRC の正しいフレームがそろうと `true` を返し、チャンネルは `values()` にあります。**フレーム**（32 バイト）：`0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`、`CRC = 0xFFFF − Σ(先頭 30 バイト)`。チャンネル値は**下位 12 ビット**です（上位ビットで FS-iA6B はサービスデータを送ります。例：failsafe で `0x2384` → 900 µs）。`0x20` を待ち、次のバイトは `0x40` でなければならず、そうでなければ探索をやり直します（2 つ目の `0x20` 自体がフレームの始まりになりうる）。iBUS には failsafe フラグがなく、FS-iA6B は正常範囲より低いスロットルでリンク喪失を知らせます。

---

## `SbusParser`

**ファイル：**`rc/SbusParser.h` · **依存先：**`Packed11`

**フレーム**（25 バイト）：`0x0F | 11 ビット × 16 チャンネル（22 バイト） | フラグ | 終端`。フラグ：ビット 2——無線で 1 フレーム欠落（まだリンク喪失ではない）、ビット 3——failsafe。終端——SBUS は `0x00`、SBUS2 は `0x04`/`0x14`/`0x24`/`0x34`。バイト `0x0F` はデータ中にも現れるため、先頭と終端の両方が正しい位置にあるときだけフレームを受理し、そうでなければ受信済みデータ内の次の `0x0F` から解析を続けます——ストリームは 1〜2 フレームで同期します。

---

## `CrsfParser`

**ファイル：**`rc/CrsfParser.h` · **依存先：**`Packed11`

**フレーム：**`0xC8 | 長さ | 種類 | データ | CRC8`。長さ = 種類 + データ + CRC、フレーム全体は 64 バイト以内。CRC8 は多項式 `0xD5`（DVB-S2）で種類とデータにかけます（`static uint8_t crc8(...)`）。必要な種類は 2 つ：`0x16`——チャンネル（SBUS と同じく 11 ビット × 16）と `0x14`——リンク統計（3 バイト目がリンク品質、%）。ExpressLRS は既定ではリンク喪失時にチャンネルの送信をやめるだけで、これがフレームタイムアウトになります。「最後の位置」モードではチャンネルは届き続けますがリンク品質が 0 になり、これが `failsafe()` です。

---

## `Packed11`

**ファイル：**`rc/Packed11.h` · **種別：**名前空間

22 バイトに 11 ビット × 16 チャンネル、下位ビットが先——SBUS も CRSF もこの形で詰めます。`toMicros(raw)`：172…1811 → 988…2012 µs、992 → 1500（`5/8 · 値 + 880`、Betaflight や INAV と同じ）。`unpack(data, out)` は 22 バイトを µs 単位の 16 個の値に変換します。
