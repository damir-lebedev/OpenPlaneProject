// Реализация storage/KeyValueStore.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "storage/KeyValueStore.h"


KeyValueStore::KeyValueStore(IFlashStorage& storageRef)
: storage(storageRef)
{
}

auto KeyValueStore::mount() -> void
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

auto KeyValueStore::get(const char* space, const char* key, size_t* length) -> const uint8_t*
{
    mount();
    const long at = find(space, key);
    if (at < 0) return nullptr;

    const uint8_t* record = records() + at;
    if (length) *length = read16(record + 2);
    return record + 4 + record[0] + record[1];
}

auto KeyValueStore::contains(const char* space, const char* key) -> bool
{
    return get(space, key, nullptr) != nullptr;
}

auto KeyValueStore::hasNamespace(const char* space) -> bool
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

auto KeyValueStore::put(const char* space, const char* key, const void* data, size_t length) -> bool
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

auto KeyValueStore::remove(const char* space, const char* key) -> bool
{
    mount();
    const long at = find(space, key);
    if (at < 0) return false;
    erase(static_cast<size_t>(at));
    dirty = true;
    return true;
}

auto KeyValueStore::clear(const char* space) -> bool
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

auto KeyValueStore::commit() -> bool
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

auto KeyValueStore::crc32(const uint8_t* data, size_t length) -> uint32_t
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

auto KeyValueStore::magic() -> const uint8_t*
{
    static const uint8_t value[4] = { 'O', 'P', 'K', 'V' };
    return value;
}

auto KeyValueStore::recordSize(const uint8_t* record) -> size_t
{
    return 4 + record[0] + record[1] + read16(record + 2);
}

auto KeyValueStore::nameLength(const char* name) -> size_t
{
    if (!name) return 0;
    size_t length = 0;
    while (name[length] != '\0')
    {
        if (++length > MAX_NAME_LENGTH) return 0;
    }
    return length;
}

auto KeyValueStore::find(const char* space, const char* key) -> long
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

auto KeyValueStore::erase(size_t at) -> void
{
    const size_t size = recordSize(records() + at);
    memmove(records() + at, records() + at + size, used - at - size);
    used -= size;
}

auto KeyValueStore::recordsValid(size_t length) const -> bool
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

auto KeyValueStore::isErased(const uint8_t* data, size_t length) -> bool
{
    for (size_t i = 0; i < length; ++i)
    {
        if (data[i] != 0xFF && data[i] != 0x00) return false;
    }
    return true;
}

auto KeyValueStore::read16(const uint8_t* p) -> uint16_t
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

auto KeyValueStore::read32(const uint8_t* p) -> uint32_t
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

auto KeyValueStore::write16(uint8_t* p, uint16_t value) -> void
{
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
}

auto KeyValueStore::write32(uint8_t* p, uint32_t value) -> void
{
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(value >> (8 * i));
}
