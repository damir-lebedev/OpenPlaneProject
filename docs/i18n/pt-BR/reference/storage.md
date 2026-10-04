# STORAGE — configurações sem NVS

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/storage.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

As calibrações dos sensores, a instalação da IMU, o auto-trim e as
configurações do log são guardados pela API `Preferences`. Na ESP32 isso é o
NVS; nas placas sem NVS (STM32), uma imagem “chave → bytes” em um setor da
flash. O código da camada é portátil (sem Arduino) e é verificado no PC
(`test/native/test_storage`), e na STM32 também como parte de todo o firmware
(`test/native_stm32`).

---

## `IFlashStorage`

**Arquivo:** `storage/KeyValueStore.h` · **Tipo:** interface · **Implementações:** `Stm32FlashStorage` ([hal.md](hal.md#stm32flashstorage)), meios em RAM nos testes

| Método | Descrição |
|---|---|
| `size_t capacity() const` | bytes no meio |
| `void read(uint8_t* dst, size_t n)` | ler a partir do deslocamento zero |
| `bool write(const uint8_t* src, size_t n)` | gravar a imagem inteira |

## `KeyValueStore`

**Arquivo:** `storage/KeyValueStore.h`

A imagem inteira (`CAPACITY` = 2048 bytes) fica na RAM; a flash é lida uma vez
(`mount()`, de forma preguiçosa) e regravada por completo em `commit()`.

```
cabeçalho de 12 bytes: "OPKV" | versão u16 | usado u16 | CRC32 dos registros u32
registros em sequência: [len ns u8][len key u8][len value u16][ns][key][value]
```

| Método | Descrição |
|---|---|
| `void mount()` | ler e verificar a imagem: flash limpa (0xFF) — vazia; sem assinatura, versão alheia, comprimento maior que o meio, CRC ou estrutura dos registros que não batem — `wasCorrupt()`, vazia |
| `const uint8_t* get(ns, key, size_t* len)` | o valor ou `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | o mesmo valor — a flash não é tocada (o desgaste não cresce); sem espaço — `false`, o valor antigo fica intacto |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | gravar, se mudou; um erro do meio — `isDirty()` permanece |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | diagnóstico |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

Os nomes de espaços e de chaves têm de 1 a 15 caracteres, como no NVS.

## `KvPreferences`

**Arquivo:** `storage/KvPreferences.h`

A API da classe `Preferences` da ESP32 sobre o `KeyValueStore` — o subconjunto
que o projeto usa: `begin(name, readOnly)`, `end()` (grava, se mudou), `clear`,
`remove`, `isKey`, `put/get` para `Bool`, `UChar`, `Int`, `UInt`, `Float`,
`Bytes`, `getBytesLength`.

O comportamento é o do original: `begin(name, true)` para um espaço
inexistente — `false`; sem a chave ou com tamanho diferente — o valor padrão
(`getFloat` — `NAN`); `getBytes` para um buffer menor — 0; sem `begin()`
nada é lido nem gravado.

Na STM32, `class Preferences : public KvPreferences` fica em
`hal/stm32/compat/Preferences.h`.

---

## `Fat32::locate`

**Arquivo:** `storage/Fat32File.h` · namespace `Fat32`

FAT32 **somente leitura**: achar um arquivo na raiz do cartão e dizer onde ele
está. O firmware não cria nem altera nada no sistema de arquivos.

| Função | Descrição |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | Um MBR com partição FAT32 (tipo 0x0B/0x0C) ou FAT32 sem tabela de partições; setor de 512 bytes; nome 8.3. Percorre a cadeia de clusters da raiz, pulando o rótulo do volume, os diretórios, os LFN e as entradas apagadas, e verifica se os clusters do arquivo são **contíguos** |
| `Extent` | `firstBlock` — o bloco do cartão em que os dados começam; `bytes` — o tamanho do arquivo |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | O motivo em russo, com uma dica do que fazer |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
