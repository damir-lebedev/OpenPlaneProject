# STORAGE — настройки без NVS

[← Справочник](README.md)

Калибровки датчиков, установка IMU, автотриммер и настройки лога хранятся
через API `Preferences`. На ESP32 это NVS; на платах без NVS (STM32) — образ
«ключ → байты» в секторе флеша. Код слоя переносимый (без Arduino) и
проверяется на ПК (`test/native/test_storage`), а на STM32 — ещё и в составе
всей прошивки (`test/native_stm32`).

---

## `IFlashStorage`

**Файл:** `storage/KeyValueStore.h` · **Вид:** интерфейс · **Реализации:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), RAM-носители в тестах

| Метод | Описание |
|---|---|
| `size_t capacity() const` | байт на носителе |
| `void read(uint8_t* dst, size_t n)` | прочитать с нулевого смещения |
| `bool write(const uint8_t* src, size_t n)` | записать образ целиком |

## `KeyValueStore`

**Файл:** `storage/KeyValueStore.h`

Весь образ (`CAPACITY` = 2048 байт) — в ОЗУ; флеш читается один раз
(`mount()`, лениво) и перезаписывается целиком в `commit()`.

```
заголовок 12 байт: "OPKV" | версия u16 | занято u16 | CRC32 записей u32
записи подряд:     [len ns u8][len key u8][len value u16][ns][key][value]
```

| Метод | Описание |
|---|---|
| `void mount()` | прочитать и проверить образ: чистый флеш (0xFF) — пусто; нет магии, чужая версия, длина больше носителя, CRC или структура записей не сходятся — `wasCorrupt()`, пусто |
| `const uint8_t* get(ns, key, size_t* len)` | значение или `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | такое же значение — флеш не трогается (не растёт износ); места не хватает — `false`, старое значение цело |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | записать, если менялось; ошибка носителя — остаётся `isDirty()` |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | диагностика |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Имена пространств и ключей — 1..15 символов, как в NVS.

## `KvPreferences`

**Файл:** `storage/KvPreferences.h`

API класса `Preferences` ESP32 поверх `KeyValueStore` — подмножество, которым
пользуется проект: `begin(name, readOnly)`, `end()` (запись, если менялось),
`clear`, `remove`, `isKey`, `put/get` для `Bool`, `UChar`, `Int`, `UInt`,
`Float`, `Bytes`, `getBytesLength`.

Поведение как у оригинала: `begin(name, true)` для несуществующего
пространства — `false`; ключа нет или другой размер — значение по умолчанию
(`getFloat` — `NAN`); `getBytes` в меньший буфер — 0; без `begin()` ничего не
читается и не пишется.

На STM32 `class Preferences : public KvPreferences` —
`hal/stm32/compat/Preferences.h`.
