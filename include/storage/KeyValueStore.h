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

    explicit KeyValueStore(IFlashStorage& storageRef)
        : storage(storageRef)
    {
    }

    KeyValueStore(const KeyValueStore&) = delete;
    KeyValueStore& operator=(const KeyValueStore&) = delete;

    // Прочитать образ с носителя (один раз; повторный вызов — ничего).
    void mount()
    {
        if (mounted) return;
        mounted = true;
        used = 0;
        corrupt = false;

        const size_t size = storage.capacity() < CAPACITY ? storage.capacity() : CAPACITY;
        // Носитель меньше заголовка — хранилище просто пустое. На STM32
        // ёмкость постоянная, и cppcheck видит условие всегда ложным.
        // cppcheck-suppress knownConditionTrueFalse
        if (size < HEADER_SIZE) return;

        memset(image, 0xFF, sizeof(image));
        storage.read(image, size);

        if (memcmp(image, magic(), 4) != 0)
        {
            // Чистый флеш (0xFF) — не ошибка, просто пусто.
            corrupt = !isErased(image, HEADER_SIZE);
            return;
        }

        const uint16_t version = read16(image + 4);
        const uint16_t length = read16(image + 6);
        const uint32_t crc = read32(image + 8);
        if (version != FORMAT_VERSION || length > size - HEADER_SIZE ||
            crc32(image + HEADER_SIZE, length) != crc || !recordsValid(length))
        {
            corrupt = true;
            return;
        }
        used = length;
    }

    // Значение или nullptr; длина — в *length.
    const uint8_t* get(const char* space, const char* key, size_t* length)
    {
        mount();
        const long at = find(space, key);
        if (at < 0) return nullptr;

        const uint8_t* record = records() + at;
        if (length) *length = read16(record + 2);
        return record + 4 + record[0] + record[1];
    }

    bool contains(const char* space, const char* key)
    {
        return get(space, key, nullptr) != nullptr;
    }

    bool hasNamespace(const char* space)
    {
        mount();
        const size_t spaceLength = nameLength(space);
        if (spaceLength == 0) return false;

        for (size_t at = 0; at < used; at += recordSize(records() + at))
        {
            const uint8_t* record = records() + at;
            if (record[0] == spaceLength && memcmp(record + 4, space, spaceLength) == 0) return true;
        }
        return false;
    }

    // Записать значение. Такое же значение уже лежит — флеш не трогаем.
    bool put(const char* space, const char* key, const void* data, size_t length)
    {
        mount();
        const size_t spaceLength = nameLength(space);
        const size_t keyLength = nameLength(key);
        if (spaceLength == 0 || keyLength == 0 || (length > 0 && !data)) return false;

        const long existing = find(space, key);
        if (existing >= 0)
        {
            const uint8_t* record = records() + existing;
            if (read16(record + 2) == length && memcmp(record + 4 + spaceLength + keyLength, data, length) == 0)
            {
                return true;
            }
        }

        const size_t needed = 4 + spaceLength + keyLength + length;
        const size_t freed = existing >= 0 ? recordSize(records() + existing) : 0;
        if (used - freed + needed > CAPACITY - HEADER_SIZE) return false;

        if (existing >= 0) erase(static_cast<size_t>(existing));

        uint8_t* record = records() + used;
        record[0] = static_cast<uint8_t>(spaceLength);
        record[1] = static_cast<uint8_t>(keyLength);
        write16(record + 2, static_cast<uint16_t>(length));
        memcpy(record + 4, space, spaceLength);
        memcpy(record + 4 + spaceLength, key, keyLength);
        if (length > 0) memcpy(record + 4 + spaceLength + keyLength, data, length);
        used += needed;
        dirty = true;
        return true;
    }

    bool remove(const char* space, const char* key)
    {
        mount();
        const long at = find(space, key);
        if (at < 0) return false;
        erase(static_cast<size_t>(at));
        dirty = true;
        return true;
    }

    // Удалить всё пространство имён.
    bool clear(const char* space)
    {
        mount();
        const size_t spaceLength = nameLength(space);
        if (spaceLength == 0) return false;

        size_t at = 0;
        while (at < used)
        {
            const uint8_t* record = records() + at;
            if (record[0] == spaceLength && memcmp(record + 4, space, spaceLength) == 0)
            {
                erase(at);
                dirty = true;
            }
            else
            {
                at += recordSize(record);
            }
        }
        return true;
    }

    // Записать образ на носитель, если он менялся.
    bool commit()
    {
        if (!dirty) return true;

        memcpy(image, magic(), 4);
        write16(image + 4, FORMAT_VERSION);
        write16(image + 6, static_cast<uint16_t>(used));
        write32(image + 8, crc32(image + HEADER_SIZE, used));

        if (!storage.write(image, HEADER_SIZE + used)) return false;
        dirty = false;
        ++commits;
        return true;
    }

    size_t bytesUsed() const { return HEADER_SIZE + used; }
    bool isDirty() const { return dirty; }
    bool wasCorrupt() const { return corrupt; }
    uint32_t commitCount() const { return commits; }

    static uint32_t crc32(const uint8_t* data, size_t length)
    {
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
            }
        }
        return ~crc;
    }


private:

    IFlashStorage& storage;
    uint8_t image[CAPACITY] = {};
    size_t used = 0;
    bool mounted = false;
    bool dirty = false;
    bool corrupt = false;
    uint32_t commits = 0;

    uint8_t* records() { return image + HEADER_SIZE; }

    static const uint8_t* magic()
    {
        static const uint8_t value[4] = { 'O', 'P', 'K', 'V' };
        return value;
    }

    static size_t recordSize(const uint8_t* record)
    {
        return 4 + record[0] + record[1] + read16(record + 2);
    }

    // Длина имени, 0 — пустое, нет или длиннее MAX_NAME_LENGTH.
    static size_t nameLength(const char* name)
    {
        if (!name) return 0;
        size_t length = 0;
        while (name[length] != '\0')
        {
            if (++length > MAX_NAME_LENGTH) return 0;
        }
        return length;
    }

    long find(const char* space, const char* key)
    {
        const size_t spaceLength = nameLength(space);
        const size_t keyLength = nameLength(key);
        if (spaceLength == 0 || keyLength == 0) return -1;

        for (size_t at = 0; at < used; at += recordSize(records() + at))
        {
            const uint8_t* record = records() + at;
            if (record[0] == spaceLength && record[1] == keyLength &&
                memcmp(record + 4, space, spaceLength) == 0 &&
                memcmp(record + 4 + spaceLength, key, keyLength) == 0)
            {
                return static_cast<long>(at);
            }
        }
        return -1;
    }

    void erase(size_t at)
    {
        const size_t size = recordSize(records() + at);
        memmove(records() + at, records() + at + size, used - at - size);
        used -= size;
    }

    // Записи ровно заполняют length байт, имена непустые.
    bool recordsValid(size_t length) const
    {
        const uint8_t* data = image + HEADER_SIZE;
        size_t at = 0;
        while (at < length)
        {
            if (length - at < 4) return false;
            const uint8_t* record = data + at;
            if (record[0] == 0 || record[0] > MAX_NAME_LENGTH || record[1] == 0 || record[1] > MAX_NAME_LENGTH)
            {
                return false;
            }
            const size_t size = recordSize(record);
            if (size > length - at) return false;
            at += size;
        }
        return true;
    }

    static bool isErased(const uint8_t* data, size_t length)
    {
        for (size_t i = 0; i < length; ++i)
        {
            if (data[i] != 0xFF && data[i] != 0x00) return false;
        }
        return true;
    }

    static uint16_t read16(const uint8_t* p)
    {
        return static_cast<uint16_t>(p[0] | (p[1] << 8));
    }

    static uint32_t read32(const uint8_t* p)
    {
        return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    }

    static void write16(uint8_t* p, uint16_t value)
    {
        p[0] = static_cast<uint8_t>(value);
        p[1] = static_cast<uint8_t>(value >> 8);
    }

    static void write32(uint8_t* p, uint32_t value)
    {
        for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(value >> (8 * i));
    }
};
