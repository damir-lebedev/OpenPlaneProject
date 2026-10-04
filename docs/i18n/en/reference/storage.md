# STORAGE — settings without NVS

> 🌐 This page is a translation of the [Russian original](../../../reference/storage.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

Sensor calibrations, the IMU mounting, the auto-trim and the log settings are
stored through the `Preferences` API. On the ESP32 that is NVS; on boards
without NVS (STM32) — a "key → bytes" image in a flash sector. The layer's code
is portable (no Arduino) and is verified on a PC (`test/native/test_storage`),
and on the STM32 — also as part of the whole firmware (`test/native_stm32`).

---

## `IFlashStorage`

**File:** `storage/KeyValueStore.h` · **Kind:** interface · **Implementations:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), RAM media in the tests

| Method | Description |
|---|---|
| `size_t capacity() const` | bytes on the medium |
| `void read(uint8_t* dst, size_t n)` | read from offset zero |
| `bool write(const uint8_t* src, size_t n)` | write the whole image |

## `KeyValueStore`

**File:** `storage/KeyValueStore.h`

The whole image (`CAPACITY` = 2048 bytes) lives in RAM; the flash is read once
(`mount()`, lazily) and rewritten entirely in `commit()`.

```
header 12 bytes:   "OPKV" | version u16 | used u16 | CRC32 of the records u32
records in a row:  [len ns u8][len key u8][len value u16][ns][key][value]
```

| Method | Description |
|---|---|
| `void mount()` | read and verify the image: clean flash (0xFF) — empty; no magic, a foreign version, a length larger than the medium, a CRC or record structure that does not match — `wasCorrupt()`, empty |
| `const uint8_t* get(ns, key, size_t* len)` | the value or `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | the same value — the flash is not touched (wear does not grow); not enough room — `false`, the old value is intact |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | write, if it changed; a medium error — `isDirty()` remains |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | diagnostics |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Namespace and key names are 1..15 characters, as in NVS.

## `KvPreferences`

**File:** `storage/KvPreferences.h`

The ESP32 `Preferences` class API on top of `KeyValueStore` — the subset the
project uses: `begin(name, readOnly)`, `end()` (writes, if it changed),
`clear`, `remove`, `isKey`, `put/get` for `Bool`, `UChar`, `Int`, `UInt`,
`Float`, `Bytes`, `getBytesLength`.

The behavior is as in the original: `begin(name, true)` for a non-existent
namespace — `false`; no key or a different size — the default value
(`getFloat` — `NAN`); `getBytes` into a smaller buffer — 0; without `begin()`
nothing is read or written.

On the STM32 `class Preferences : public KvPreferences` —
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**File:** `storage/Fat32File.h` · namespace `Fat32`

FAT32 **read-only**: find a file in the card's root and say where it lies.
The firmware creates and changes nothing in the file system.

| Function | Description |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | An MBR with a FAT32 partition (type 0x0B/0x0C) or FAT32 without a partition table; 512-byte sector; an 8.3 name. Walks the root's cluster chain, skipping the volume label, directories, LFN and deleted entries, and checks that the file's clusters run **contiguously** |
| `Extent` | `firstBlock` — the card block where the data starts; `bytes` — the file size |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | The reason in Russian, with a hint on what to do |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
