# STORAGE — réglages sans NVS

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/storage.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

Les calibrations des capteurs, le montage de l’IMU, l’auto-trim et les
réglages du journal sont conservés via l’API `Preferences`. Sur l’ESP32, c’est
la NVS ; sur les cartes sans NVS (STM32), une image « clé → octets » dans un
secteur de la flash. Le code de la couche est portable (sans Arduino) et est
vérifié sur PC (`test/native/test_storage`), et sur la STM32 aussi au sein de
tout le micrologiciel (`test/native_stm32`).

---

## `IFlashStorage`

**Fichier :** `storage/KeyValueStore.h` · **Genre :** interface · **Implémentations :** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), supports en RAM dans les tests

| Méthode | Description |
|---|---|
| `size_t capacity() const` | octets sur le support |
| `void read(uint8_t* dst, size_t n)` | lire à partir du décalage zéro |
| `bool write(const uint8_t* src, size_t n)` | écrire l’image entière |

## `KeyValueStore`

**Fichier :** `storage/KeyValueStore.h`

Toute l’image (`CAPACITY` = 2048 octets) est en RAM ; la flash est lue une
seule fois (`mount()`, paresseusement) et réécrite en entier dans `commit()`.

```
en-tête de 12 octets : "OPKV" | version u16 | occupé u16 | CRC32 des enregistrements u32
enregistrements à la suite : [len ns u8][len key u8][len value u16][ns][key][value]
```

| Méthode | Description |
|---|---|
| `void mount()` | lire et vérifier l’image : flash vierge (0xFF) — vide ; pas de signature, version étrangère, longueur supérieure au support, CRC ou structure des enregistrements qui ne concordent pas — `wasCorrupt()`, vide |
| `const uint8_t* get(ns, key, size_t* len)` | la valeur ou `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | la même valeur — la flash n’est pas touchée (l’usure ne croît pas) ; pas assez de place — `false`, l’ancienne valeur reste intacte |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | écrire, si cela a changé ; erreur du support — `isDirty()` reste vrai |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | diagnostic |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Les noms d’espaces et de clés font de 1 à 15 caractères, comme dans la NVS.

## `KvPreferences`

**Fichier :** `storage/KvPreferences.h`

L’API de la classe `Preferences` de l’ESP32 au-dessus de `KeyValueStore` — le
sous-ensemble qu’utilise le projet : `begin(name, readOnly)`, `end()` (écrit,
si cela a changé), `clear`, `remove`, `isKey`, `put/get` pour `Bool`, `UChar`,
`Int`, `UInt`, `Float`, `Bytes`, `getBytesLength`.

Le comportement est celui de l’original : `begin(name, true)` pour un espace
inexistant — `false` ; pas de clé ou taille différente — la valeur par défaut
(`getFloat` — `NAN`) ; `getBytes` dans un tampon plus petit — 0 ; sans
`begin()`, rien n’est lu ni écrit.

Sur la STM32, `class Preferences : public KvPreferences` se trouve dans
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**Fichier :** `storage/Fat32File.h` · espace de noms `Fat32`

FAT32 **en lecture seule** : trouver un fichier à la racine de la carte et dire
où il se trouve. Le micrologiciel ne crée ni ne modifie rien dans le système de
fichiers.

| Fonction | Description |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | Un MBR avec une partition FAT32 (type 0x0B/0x0C) ou une FAT32 sans table de partitions ; secteur de 512 octets ; nom 8.3. Parcourt la chaîne de clusters de la racine en sautant l’étiquette du volume, les répertoires, les LFN et les entrées supprimées, et vérifie que les clusters du fichier se suivent **sans interruption** |
| `Extent` | `firstBlock` — le bloc de la carte où commencent les données ; `bytes` — la taille du fichier |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | La cause en russe, avec une indication de ce qu’il faut faire |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
