# RC — 送信機のコマンドの受信

> 🌐 このページは[ロシア語の原文](../../../reference/rc.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。

[← リファレンス](README.md)

RC レイヤーは、UART のバイト列をチャンネル値と「リンクなし」のフラグに変換します。機体、ARM、failsafe の動作、サーボのことは何も知りません。プロトコルを（S-Bus や PPM に）置き換えても、影響を受けるのはこのレイヤーだけです。

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 線形変換：1000 → `−max`、1500 → 0、2000 → `+max`（入力は先に制限される）。`reverse` は符号を反転する。結果は ±`max` に制限される |

例：`centered(1750, 500) == 250`、`centered(1750, 500, true) == -250`。

---

## `IBusReceiver`

**ファイル：** `rc/IBusReceiver.h` · **依存先：** `IUartPort`、`RcChannelState`、`Config`、`Channels`

FlySky の iBUS プロトコルを 1 バイトずつ解析するパーサーです。

**フレームの形式**（32 バイト）：`0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`、
`CRC = 0xFFFF − Σ(first 30 bytes)`。先頭の `IBUS_CHANNELS` = 10 チャンネルを使います。チャンネル値は**下位 12 ビット**です（上位ビットで FS-iA6B はサービス用のデータを送ってきます。たとえば failsafe 時の `0x2384` → 900 µs）。

| メソッド | 説明 |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | コンストラクタではポートを開かない |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`。タイムアウトの計時は「今」から始まる |
| `void update()` | UART に溜まったものをすべて読み出す。周期ごとに呼ぶ |
| `const RcChannelState& getState() const` | 最後に受信したチャンネル |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | まだ 1 つもフレームが来ていない、**または**最後のフレームが `RX_TIMEOUT_US` より古い |
| `bool isFailsafeReported() const` | 最後のフレームでスロットルが `RX_FAILSAFE_THROTTLE_US` 未満 |
| `uint32_t getLastFrameTime() const` | 最後の正しいフレームの `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 正しいフレームと CRC エラーのカウンタ |

解析のステートマシン（`processByte`）：`0x20` を待ちます。次のバイトは `0x40` でなければならず、そうでなければ探索をやり直します。そのあと 32 バイトを集めて `processFrame()` を呼びます。CRC が正しくないフレームは丸ごと捨てられます（チャンネルは変わらず、
`badFrames++`）。

不変条件：

- 最初の正しいフレームが来るまで `isSignalLost() == true` です。既定値（すべて 1500）は送信機のコマンドとは見なされません。
- failsafe のフラグは**正しいフレームごとに**再計算されます。スロットルが正常な最初のフレームで、リンクは回復します。
