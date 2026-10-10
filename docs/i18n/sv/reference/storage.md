# LAGRING – inställningar utan NVS

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/storage.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

Sensorkalibreringar, IMU-monteringen, autotrimningen och logginställningarna
lagras via `Preferences`-API:t. På ESP32 är det NVS; på kort
utan NVS (STM32) – en avbild ”nyckel → byte” i en flashsektor. Lagrets kod
är portabel (ingen Arduino) och verifieras på en dator (`test/native/test_storage`),
och på STM32 – också som en del av hela firmwaren (`test/native_stm32`).

---

## `IFlashStorage`

**Fil:** `storage/KeyValueStore.h` · **Typ:** gränssnitt · **Implementeringar:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), RAM-medier i testerna

| Metod | Beskrivning |
|---|---|
| `size_t capacity() const` | byte på mediet |
| `void read(uint8_t* dst, size_t n)` | läs från förskjutning noll |
| `bool write(const uint8_t* src, size_t n)` | skriv hela avbilden |

## `KeyValueStore`

**Fil:** `storage/KeyValueStore.h`

Hela avbilden (`CAPACITY` = 2048 byte) finns i RAM; flashen läses en gång
(`mount()`, lat) och skrivs om helt i `commit()`.

```
huvud 12 byte:     "OPKV" | version u16 | used u16 | CRC32 för posterna u32
poster i följd:    [len ns u8][len key u8][len value u16][ns][key][value]
```

| Metod | Beskrivning |
|---|---|
| `void mount()` | läs och verifiera avbilden: ren flash (0xFF) – tom; ingen magic, en främmande version, en längd större än mediet, en CRC eller poststruktur som inte stämmer – `wasCorrupt()`, tom |
| `const uint8_t* get(ns, key, size_t* len)` | värdet eller `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | samma värde – flashen rörs inte (slitaget ökar inte); inte tillräckligt med plats – `false`, det gamla värdet är intakt |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | skriv, om det har ändrats; ett mediefel – `isDirty()` kvarstår |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | diagnostik |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Namn på namnrymder och nycklar är 1..15 tecken, som i NVS.

## `KvPreferences`

**Fil:** `storage/KvPreferences.h`

API:t för ESP32-klassen `Preferences` ovanpå `KeyValueStore` – den delmängd som
projektet använder: `begin(name, readOnly)`, `end()` (skriver, om det har ändrats),
`clear`, `remove`, `isKey`, `put/get` för `Bool`, `UChar`, `Int`, `UInt`,
`Float`, `Bytes`, `getBytesLength`.

Beteendet är som i originalet: `begin(name, true)` för en namnrymd som inte finns
– `false`; ingen nyckel eller en annan storlek – standardvärdet
(`getFloat` – `NAN`); `getBytes` till en mindre buffert – 0; utan `begin()`
läses eller skrivs ingenting.

På STM32 är `class Preferences : public KvPreferences` –
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**Fil:** `storage/Fat32File.h` · namnrymd `Fat32`

FAT32 **skrivskyddat**: hitta en fil i kortets rot och tala om var den ligger.
Firmwaren skapar och ändrar ingenting i filsystemet.

| Funktion | Beskrivning |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | En MBR med en FAT32-partition (typ 0x0B/0x0C) eller FAT32 utan partitionstabell; sektor på 512 byte; ett 8.3-namn. Går igenom rotens klusterkedja, hoppar över volymetiketten, kataloger, LFN och raderade poster, och kontrollerar att filens kluster ligger **i följd** |
| `Extent` | `firstBlock` – kortets block där data börjar; `bytes` – filens storlek |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | Orsaken på ryska, med en ledtråd om vad man ska göra |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
