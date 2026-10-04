# STORAGE — Einstellungen ohne NVS

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/storage.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Die Kalibrierungen der Sensoren, der Einbau der IMU, die Autotrimmung und die
Log-Einstellungen werden über die `Preferences`-API gespeichert. Am ESP32 ist
das NVS; auf Platinen ohne NVS (STM32) — ein Abbild „Schlüssel → Bytes“ in
einem Flash-Sektor. Der Code der Schicht ist portabel (ohne Arduino) und wird
am PC geprüft (`test/native/test_storage`), und am STM32 zusätzlich als Teil
der gesamten Firmware (`test/native_stm32`).

---

## `IFlashStorage`

**Datei:** `storage/KeyValueStore.h` · **Art:** Schnittstelle · **Implementierungen:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), RAM-Medien in den Tests

| Methode | Beschreibung |
|---|---|
| `size_t capacity() const` | Bytes auf dem Medium |
| `void read(uint8_t* dst, size_t n)` | ab dem Offset null lesen |
| `bool write(const uint8_t* src, size_t n)` | das ganze Abbild schreiben |

## `KeyValueStore`

**Datei:** `storage/KeyValueStore.h`

Das ganze Abbild (`CAPACITY` = 2048 Bytes) liegt im RAM; der Flash wird einmal
gelesen (`mount()`, träge) und in `commit()` vollständig neu geschrieben.

```
Kopf 12 Bytes:       "OPKV" | Version u16 | belegt u16 | CRC32 der Einträge u32
Einträge hintereinander: [len ns u8][len key u8][len value u16][ns][key][value]
```

| Methode | Beschreibung |
|---|---|
| `void mount()` | das Abbild lesen und prüfen: sauberer Flash (0xFF) — leer; keine Kennung, fremde Version, Länge größer als das Medium, CRC oder Struktur der Einträge passen nicht — `wasCorrupt()`, leer |
| `const uint8_t* get(ns, key, size_t* len)` | der Wert oder `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | derselbe Wert — der Flash wird nicht angefasst (der Verschleiß wächst nicht); nicht genug Platz — `false`, der alte Wert bleibt heil |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | schreiben, wenn es sich geändert hat; Fehler des Mediums — `isDirty()` bleibt bestehen |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | Diagnose |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Die Namen von Namensräumen und Schlüsseln haben 1..15 Zeichen, wie im NVS.

## `KvPreferences`

**Datei:** `storage/KvPreferences.h`

Die API der ESP32-Klasse `Preferences` über `KeyValueStore` — die Teilmenge,
die das Projekt benutzt: `begin(name, readOnly)`, `end()` (schreibt, wenn es
sich geändert hat), `clear`, `remove`, `isKey`, `put/get` für `Bool`, `UChar`,
`Int`, `UInt`, `Float`, `Bytes`, `getBytesLength`.

Das Verhalten ist wie beim Original: `begin(name, true)` für einen
nicht vorhandenen Namensraum — `false`; kein Schlüssel oder eine andere
Größe — der Standardwert (`getFloat` — `NAN`); `getBytes` in einen kleineren
Puffer — 0; ohne `begin()` wird nichts gelesen und nichts geschrieben.

Am STM32 liegt `class Preferences : public KvPreferences` in
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**Datei:** `storage/Fat32File.h` · Namensraum `Fat32`

FAT32 **nur lesend**: eine Datei im Wurzelverzeichnis der Karte finden und
sagen, wo sie liegt. Die Firmware legt im Dateisystem nichts an und ändert
nichts.

| Funktion | Beschreibung |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | Ein MBR mit einer FAT32-Partition (Typ 0x0B/0x0C) oder FAT32 ohne Partitionstabelle; Sektor von 512 Bytes; Name im 8.3-Format. Läuft die Clusterkette der Wurzel entlang und überspringt Datenträgerbezeichnung, Verzeichnisse, LFN und gelöschte Einträge, und prüft, dass die Cluster der Datei **lückenlos aufeinanderfolgen** |
| `Extent` | `firstBlock` — der Block der Karte, ab dem die Daten beginnen; `bytes` — die Dateigröße |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | Die Ursache auf Russisch, mit einem Hinweis, was zu tun ist |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
