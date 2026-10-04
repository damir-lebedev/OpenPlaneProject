# STORAGE — NVS を使わない設定の保存

> 🌐 このページは[ロシア語の原文](../../../reference/storage.md)の翻訳です。翻訳と原文に違いがある場合は、原文が優先されます。ファームウェアはコンソールメッセージをロシア語で出力するため、そのまま引用しています。

[← リファレンス](README.md)

センサーの校正、IMU の取り付け、オートトリム、ログの設定は、`Preferences` API で保存されます。ESP32 ではこれが NVS で、NVS のないボード（STM32）ではフラッシュのセクタにある「キー → バイト列」のイメージです。このレイヤーのコードは移植可能（Arduino に依存しない）で、PC 上で検証され（`test/native/test_storage`）、STM32 ではファームウェア全体の一部としても検証されます（`test/native_stm32`）。

---

## `IFlashStorage`

**ファイル：** `storage/KeyValueStore.h` · **種別：** インターフェース · **実装：** `Stm32FlashStorage`（[hal.md](hal.md#stm32flashstorage)）、テストでは RAM 上の記憶媒体

| メソッド | 説明 |
|---|---|
| `size_t capacity() const` | 記憶媒体のバイト数 |
| `void read(uint8_t* dst, size_t n)` | オフセット 0 から読み出す |
| `bool write(const uint8_t* src, size_t n)` | イメージ全体を書き込む |

## `KeyValueStore`

**ファイル：** `storage/KeyValueStore.h`

イメージ全体（`CAPACITY` = 2048 バイト）は RAM 上にあります。フラッシュは 1 回だけ読み出し（`mount()`、遅延実行）、`commit()` で丸ごと書き直します。

```
ヘッダー 12 バイト："OPKV" | バージョン u16 | 使用量 u16 | レコードの CRC32 u32
レコードが連続：   [len ns u8][len key u8][len value u16][ns][key][value]
```

| メソッド | 説明 |
|---|---|
| `void mount()` | イメージを読み出して検証する：まっさらなフラッシュ（0xFF）は空。マジックがない、バージョンが違う、長さが媒体より大きい、CRC やレコードの構造が合わない場合は `wasCorrupt()` で空とする |
| `const uint8_t* get(ns, key, size_t* len)` | 値、または `nullptr` |
| `contains(ns, key)`、`hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | 同じ値ならフラッシュには触れない（摩耗が進まない）。容量が足りなければ `false` で、古い値は無事 |
| `remove(ns, key)`、`clear(ns)` | |
| `bool commit()` | 変更があれば書き込む。媒体のエラー時は `isDirty()` のまま残る |
| `bytesUsed()`、`isDirty()`、`wasCorrupt()`、`commitCount()` | 診断用 |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

名前空間とキーの名前は 1〜15 文字で、NVS と同じです。

## `KvPreferences`

**ファイル：** `storage/KvPreferences.h`

`KeyValueStore` の上に載る ESP32 の `Preferences` クラスの API で、プロジェクトが使うサブセットです：`begin(name, readOnly)`、`end()`（変更があれば書き込む）、
`clear`、`remove`、`isKey`、`Bool`、`UChar`、`Int`、`UInt`、`Float`、`Bytes` に対する
`put/get`、`getBytesLength`。

動作は元の API と同じです：存在しない名前空間に `begin(name, true)` すると
`false`。キーがない、またはサイズが違う場合は既定値（`getFloat` は `NAN`）。小さいバッファへの `getBytes` は 0。`begin()` なしでは何も読み書きしません。

STM32 では `class Preferences : public KvPreferences` が
`hal/stm32/compat/Preferences.h` にあります。

---

## `Fat32::locate`

**ファイル：** `storage/Fat32File.h` · 名前空間 `Fat32`

FAT32 の**読み取り専用**処理です：カードのルートでファイルを探し、それがどこにあるかを返します。ファームウェアはファイルシステムに何も作成せず、何も変更しません。

| 関数 | 説明 |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | FAT32 パーティション（タイプ 0x0B/0x0C）を持つ MBR、またはパーティションテーブルのない FAT32。セクタは 512 バイト、名前は 8.3 形式。ルートのクラスタチェーンをたどり、ボリュームラベル、ディレクトリ、LFN、削除済みエントリを飛ばして、ファイルのクラスタが**連続している**ことを確認する |
| `Extent` | `firstBlock`：データが始まるカード上のブロック。`bytes`：ファイルサイズ |
| `Result` | `Ok`、`NoCard`、`ReadError`、`NotFat32`、`NotFound`、`Fragmented`、`Empty` |
| `const char* describe(Result)` | ロシア語による原因と、どうすればよいかのヒント |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
