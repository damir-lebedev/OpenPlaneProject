# STORAGE — NVS 없는 설정 저장

> 🌐 이 문서는 [러시아어 원문](../../../reference/storage.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다.

[← 참조](README.md)

센서 보정, IMU 장착, 자동 트림, 로그 설정은 `Preferences` API로 저장합니다.
ESP32에서는 이것이 NVS이고, NVS가 없는 보드(STM32)에서는 플래시 섹터에 둔 “키 → 바이트”
이미지입니다. 이 계층의 코드는 이식 가능하며(Arduino에 의존하지 않음) PC에서
검증하고(`test/native/test_storage`), STM32에서는 펌웨어 전체의 일부로도
검증합니다(`test/native_stm32`).

---

## `IFlashStorage`

**파일:** `storage/KeyValueStore.h` · **종류:** 인터페이스 · **구현:** `Stm32FlashStorage`([hal.md](hal.md#stm32flashstorage)), 테스트에서는 RAM 매체

| 메서드 | 설명 |
|---|---|
| `size_t capacity() const` | 매체의 바이트 수 |
| `void read(uint8_t* dst, size_t n)` | 오프셋 0부터 읽기 |
| `bool write(const uint8_t* src, size_t n)` | 이미지 전체 쓰기 |

## `KeyValueStore`

**파일:** `storage/KeyValueStore.h`

이미지 전체(`CAPACITY` = 2048바이트)가 RAM에 있습니다. 플래시는 한 번만
읽고(`mount()`, 지연 실행) `commit()`에서 통째로 다시 씁니다.

```
헤더 12바이트:     "OPKV" | 버전 u16 | 사용량 u16 | 레코드의 CRC32 u32
레코드가 연속:     [len ns u8][len key u8][len value u16][ns][key][value]
```

| 메서드 | 설명 |
|---|---|
| `void mount()` | 이미지를 읽고 검증: 깨끗한 플래시(0xFF)는 비어 있음. 매직이 없거나, 버전이 다르거나, 길이가 매체보다 크거나, CRC나 레코드 구조가 맞지 않으면 `wasCorrupt()`이며 비어 있음으로 처리 |
| `const uint8_t* get(ns, key, size_t* len)` | 값 또는 `nullptr` |
| `contains(ns, key)`, `hasNamespace(ns)` | |
| `bool put(ns, key, data, len)` | 같은 값이면 플래시를 건드리지 않음(마모가 늘지 않음). 공간이 부족하면 `false`이며 기존 값은 그대로 |
| `remove(ns, key)`, `clear(ns)` | |
| `bool commit()` | 변경이 있으면 기록. 매체 오류가 나면 `isDirty()`가 유지됨 |
| `bytesUsed()`, `isDirty()`, `wasCorrupt()`, `commitCount()` | 진단용 |
| `static uint32_t crc32(data, len)` | CRC-32/ISO-HDLC |

네임스페이스와 키의 이름은 1~15자이며, NVS와 같습니다.

## `KvPreferences`

**파일:** `storage/KvPreferences.h`

`KeyValueStore` 위에서 동작하는 ESP32 `Preferences` 클래스의 API로, 프로젝트가
쓰는 부분 집합입니다: `begin(name, readOnly)`, `end()`(변경이 있으면 기록),
`clear`, `remove`, `isKey`, `Bool`, `UChar`, `Int`, `UInt`, `Float`, `Bytes`에 대한
`put/get`, `getBytesLength`.

동작은 원본과 같습니다. 존재하지 않는 네임스페이스에 `begin(name, true)`를 하면
`false`입니다. 키가 없거나 크기가 다르면 기본값을 돌려줍니다(`getFloat`은 `NAN`).
더 작은 버퍼로의 `getBytes`는 0입니다. `begin()` 없이는 아무것도 읽거나 쓰지 않습니다.

STM32에서는 `class Preferences : public KvPreferences`가
`hal/stm32/compat/Preferences.h`에 있습니다.

---

## `Fat32::locate`

**파일:** `storage/Fat32File.h` · 네임스페이스 `Fat32`

FAT32 **읽기 전용** 처리입니다: 카드의 루트에서 파일을 찾아 그것이 어디에 있는지
알려 줍니다. 펌웨어는 파일 시스템에 아무것도 만들거나 바꾸지 않습니다.

| 함수 | 설명 |
|---|---|
| `Result locate(IBlockDevice&, const char* name, Extent& out)` | FAT32 파티션(타입 0x0B/0x0C)이 있는 MBR, 또는 파티션 테이블이 없는 FAT32. 섹터는 512바이트, 이름은 8.3 형식. 루트의 클러스터 체인을 따라가며 볼륨 레이블, 디렉터리, LFN, 삭제된 항목을 건너뛰고, 파일의 클러스터가 **연속**인지 확인 |
| `Extent` | `firstBlock`: 데이터가 시작되는 카드의 블록. `bytes`: 파일 크기 |
| `Result` | `Ok`, `NoCard`, `ReadError`, `NotFat32`, `NotFound`, `Fragmented`, `Empty` |
| `const char* describe(Result)` | 러시아어로 된 원인과 어떻게 해야 하는지에 대한 안내 |
| `bool shortName(name, out[11])` | `blackbox.bin` → `BLACKBOXBIN ` |
