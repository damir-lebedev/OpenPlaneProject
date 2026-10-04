# STORAGE — ajustes sin NVS

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/storage.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

Las calibraciones de los sensores, el montaje de la IMU, el autotrim y los
ajustes del registro se guardan mediante la API `Preferences`. En la ESP32 es
NVS; en las placas sin NVS (STM32), una imagen «clave → bytes» en un sector de
la flash. El código de la capa es portátil (sin Arduino) y se comprueba en el
PC (`test/native/test_storage`), y en la STM32, además, como parte de todo el
firmware (`test/native_stm32`).

---

## `IFlashStorage`

**Archivo:** `storage/KeyValueStore.h` · **Clase:** interfaz · **Implementaciones:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), soportes en RAM en las pruebas

| Método | Descripción |
|---|---|
| `size_t capacity() const` | bytes en el soporte |
| `void read(uint8_t* dst, size_t n)` | leer desde el desplazamiento cero |
| `bool write(const uint8_t* src, size_t n)` | escribir la imagen completa |

## `KeyValueStore`

**Archivo:** `storage/KeyValueStore.h`

Toda la imagen (`CAPACITY` = 2048 bytes) está en RAM; la flash se lee una sola
vez (`mount()`, de forma perezosa) y se reescribe por completo en `commit()`.

```
cabecera de 12 bytes: "OPKV" | versión u16 | usado u16 | CRC32 de los registros u32
registros seguidos:   [len ns u8][len key u8][len value u16][ns][key][value]
```

| Método | Descripción |
|---|---|
| `void mount()` | leer y comprobar la imagen: flash limpia (0xFF): vacía; sin firma, versión ajena, longitud mayor que el soporte, CRC o estructura de los registros que no coinciden: `wasCorrupt()`, vacía |
| `const uint8_t* get(ns, key, size_t* len)` | el valor o `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | un valor idéntico: no se toca la flash (el desgaste no crece); si no hay espacio: `false`, el valor anterior queda intacto |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | escribir, si hubo cambios; un error del soporte: sigue `isDirty()` |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | diagnóstico |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Los nombres de los espacios y de las claves tienen de 1 a 15 caracteres, como en NVS.

## `KvPreferences`

**Archivo:** `storage/KvPreferences.h`

La API de la clase `Preferences` del ESP32 sobre `KeyValueStore`: el
subconjunto que usa el proyecto: `begin(name, readOnly)`, `end()` (escribe, si
hubo cambios), `clear`, `remove`, `isKey`, `put/get` para `Bool`, `UChar`,
`Int`, `UInt`, `Float`, `Bytes` y `getBytesLength`.

El comportamiento es el del original: `begin(name, true)` para un espacio que
no existe: `false`; si no hay clave o el tamaño es distinto: el valor por
defecto (`getFloat`: `NAN`); `getBytes` a un búfer menor: 0; sin `begin()` no
se lee ni se escribe nada.

En la STM32, `class Preferences : public KvPreferences` está en
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**Archivo:** `storage/Fat32File.h` · espacio de nombres `Fat32`

FAT32 **de solo lectura**: encontrar un archivo en la raíz de la tarjeta y
decir dónde está. El firmware no crea ni modifica nada en el sistema de
archivos.

| Función | Descripción |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | Un MBR con una partición FAT32 (tipo 0x0B/0x0C) o FAT32 sin tabla de particiones; sector de 512 bytes; nombre 8.3. Recorre la cadena de clústeres de la raíz, saltándose la etiqueta del volumen, los directorios, los LFN y las entradas borradas, y comprueba que los clústeres del archivo van **seguidos** |
| `Extent` | `firstBlock`: el bloque de la tarjeta donde empiezan los datos; `bytes`: el tamaño del archivo |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | El motivo en ruso, con una indicación de qué hacer |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
