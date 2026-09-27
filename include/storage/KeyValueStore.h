#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// ХРАНИЛИЩЕ "ПРОСТРАНСТВО ИМЁН / КЛЮЧ -> БАЙТЫ" ПОВЕРХ СЫРОЙ ПАМЯТИ
//
// Замена NVS (Preferences ESP32) там, где NVS нет — на STM32 это
// кусок флеша (EEPROM-эмуляция STM32duino). Весь образ живёт в ОЗУ
// (CAPACITY байт), флеш читается один раз при первом обращении и
// перезаписывается целиком в commit(). Ключей немного (калибровки,
// триммер, настройки лога) — сотни байт, поэтому простота важнее
// износа: запись только если что-то действительно поменялось.
//
// Формат образа (little-endian):
//   заголовок, 12 байт: "OPKV" | версия u16 | занято байт u16 | CRC32 записей u32
//   записи подряд: [длина ns u8][длина ключа u8][длина значения u16][ns][ключ][значение]
// Битый образ (нет магии, не сошлась CRC — например, питание пропало
// во время стирания сектора) читается как пустой: настройки вернутся
// к умолчаниям, а не превратятся в мусор.
//
// Код переносимый (без Arduino) — тестируется на ПК (test_storage).
// ============================================================

// Носитель образа: читает/пишет байты с нулевого смещения.
class IFlashStorage
{
public:
    virtual ~IFlashStorage() = default;

    virtual size_t capacity() const = 0;
    virtual void read(uint8_t* destination, size_t size) = 0;
    virtual bool write(const uint8_t* source, size_t size) = 0;
};


class KeyValueStore
{
public:

    static constexpr size_t CAPACITY = 2048;
    static constexpr size_t HEADER_SIZE = 12;
    static constexpr size_t MAX_NAME_LENGTH = 15;   // как в NVS
    static constexpr uint16_t FORMAT_VERSION = 1;

    explicit KeyValueStore(IFlashStorage& storageRef);

    KeyValueStore(const KeyValueStore&) = delete;
    KeyValueStore& operator=(const KeyValueStore&) = delete;

    // Прочитать образ с носителя (один раз; повторный вызов — ничего).
    void mount();

    // Значение или nullptr; длина — в *length.
    const uint8_t* get(const char* space, const char* key, size_t* length);

    bool contains(const char* space, const char* key);

    bool hasNamespace(const char* space);

    // Записать значение. Такое же значение уже лежит — флеш не трогаем.
    bool put(const char* space, const char* key, const void* data, size_t length);

    bool remove(const char* space, const char* key);

    // Удалить всё пространство имён.
    bool clear(const char* space);

    // Записать образ на носитель, если он менялся.
    bool commit();

    size_t bytesUsed() const { return HEADER_SIZE + used; }
    bool isDirty() const { return dirty; }
    bool wasCorrupt() const { return corrupt; }
    uint32_t commitCount() const { return commits; }

    static uint32_t crc32(const uint8_t* data, size_t length);


private:

    IFlashStorage& storage;
    uint8_t image[CAPACITY] = {};
    size_t used = 0;
    bool mounted = false;
    bool dirty = false;
    bool corrupt = false;
    uint32_t commits = 0;

    uint8_t* records() { return image + HEADER_SIZE; }

    static const uint8_t* magic();

    static size_t recordSize(const uint8_t* record);

    // Длина имени, 0 — пустое, нет или длиннее MAX_NAME_LENGTH.
    static size_t nameLength(const char* name);

    long find(const char* space, const char* key);

    void erase(size_t at);

    // Записи ровно заполняют length байт, имена непустые.
    bool recordsValid(size_t length) const;

    static bool isErased(const uint8_t* data, size_t length);

    static uint16_t read16(const uint8_t* p);

    static uint32_t read32(const uint8_t* p);

    static void write16(uint8_t* p, uint16_t value);

    static void write32(uint8_t* p, uint32_t value);
};
