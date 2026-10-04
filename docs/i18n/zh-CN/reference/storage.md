# STORAGE — 不使用 NVS 的设置存储

> 🌐 本页是[俄语原文](../../../reference/storage.md)的译文。译文与原文如有出入，以原文为准。固件的控制台信息以俄语输出，因此文中照原样引用。

[← 参考](README.md)

传感器校准、IMU 的安装、自动配平和日志设置都通过 `Preferences` API 保存。在 ESP32 上这是 NVS；在没有 NVS 的开发板（STM32）上，则是闪存扇区里的
“键 → 字节”映像。该层的代码是可移植的（不依赖 Arduino），在 PC 上验证（`test/native/test_storage`），在 STM32 上还作为整个固件的一部分进行验证（`test/native_stm32`）。

---

## `IFlashStorage`

**文件：** `storage/KeyValueStore.h` · **类别：** 接口 · **实现：** `Stm32FlashStorage`（[hal.md](hal.md#stm32flashstorage)），测试中的 RAM 存储介质

| 方法 | 说明 |
|---|---|
| `size_t capacity() const` | 存储介质的字节数 |
| `void read(uint8_t* dst, size_t n)` | 从零偏移处读取 |
| `bool write(const uint8_t* src, size_t n)` | 整体写入映像 |

## `KeyValueStore`

**文件：** `storage/KeyValueStore.h`

整个映像（`CAPACITY` = 2048 字节）都放在 RAM 中；闪存只读取一次（`mount()`，延迟执行），并在 `commit()` 中整体重写。

```
头部 12 字节：     "OPKV" | 版本 u16 | 已用 u16 | 记录的 CRC32 u32
记录依次排列：     [len ns u8][len key u8][len value u16][ns][key][value]
```

| 方法 | 说明 |
|---|---|
| `void mount()` | 读取并校验映像：干净的闪存（0xFF）——视为空；没有魔数、版本不符、长度超过存储介质、CRC 或记录结构对不上——`wasCorrupt()`，视为空 |
| `const uint8_t* get(ns, key, size_t* len)` | 值，或 `nullptr` |
| `contains(ns, key)`、`hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | 值相同——不触碰闪存（不增加磨损）；空间不足——`false`，旧值保持完好 |
| `remove(ns, key)`、`clear(ns)` | |
| `bool commit()` | 有改动时写入；存储介质出错——保持 `isDirty()` |
| `bytesUsed()`、`isDirty()`、`wasCorrupt()`、`commitCount()` | 诊断信息 |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

命名空间和键的名称为 1..15 个字符，与 NVS 相同。

## `KvPreferences`

**文件：** `storage/KvPreferences.h`

基于 `KeyValueStore` 实现的 ESP32 `Preferences` 类 API——只包含本项目用到的子集：`begin(name, readOnly)`、`end()`（有改动时写入）、
`clear`、`remove`、`isKey`，以及针对 `Bool`、`UChar`、`Int`、`UInt`、
`Float`、`Bytes` 的 `put/get` 和 `getBytesLength`。

行为与原版一致：对不存在的命名空间调用 `begin(name, true)`
——`false`；键不存在或大小不同——返回默认值（`getFloat` 返回 `NAN`）；`getBytes` 读入更小的缓冲区——0；没有调用 `begin()`
时不读也不写。

在 STM32 上，`class Preferences : public KvPreferences` 位于
`hal/stm32/compat/Preferences.h`。

---

## `Fat32::locate`

**文件：** `storage/Fat32File.h` · 命名空间 `Fat32`

FAT32 **只读**：在卡的根目录中找到文件，并说明它在哪里。固件不会在文件系统中创建或修改任何东西。

| 函数 | 说明 |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | 带有 FAT32 分区（类型 0x0B/0x0C）的 MBR，或没有分区表的 FAT32；扇区 512 字节；8.3 文件名。沿根目录的簇链前进，跳过卷标、目录、LFN 和已删除的条目，并检查文件的簇是否**连续** |
| `Extent` | `firstBlock`——数据所在的卡上起始块；`bytes`——文件大小 |
| `Result` | `Ok`、`NoCard`、`ReadError`、`NotFat32`、`NotFound`、`Fragmented`、`Empty` |
| `const char* describe(Result)` | 用俄语给出原因，并附带该怎么做的提示 |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
