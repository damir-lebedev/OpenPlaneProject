// Реализация storage/KvPreferences.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "storage/KvPreferences.h"


KvPreferences::KvPreferences(KeyValueStore& storeRef)
: store(storeRef)
{
}

auto KvPreferences::begin(const char* name, bool readOnly, const char* partitionLabel) -> bool
{
    (void)partitionLabel;
    if (started || !name || name[0] == '\0' || strlen(name) > KeyValueStore::MAX_NAME_LENGTH) return false;
    if (readOnly && !store.hasNamespace(name)) return false;

    strncpy(space, name, sizeof(space) - 1);
    space[sizeof(space) - 1] = '\0';
    readOnlyMode = readOnly;
    started = true;
    return true;
}

auto KvPreferences::end() -> void
{
    if (!started) return;
    started = false;
    if (!readOnlyMode) store.commit();
}

auto KvPreferences::putBytes(const char* key, const void* value, size_t length) -> size_t
{
    if (!writable() || !key || (!value && length > 0)) return 0;
    return store.put(space, key, value, length) ? length : 0;
}

auto KvPreferences::getBool(const char* key, bool defaultValue) -> bool
{
    return getValue<uint8_t>(key, defaultValue ? 1 : 0) != 0;
}

auto KvPreferences::getBytesLength(const char* key) -> size_t
{
    size_t length = 0;
    return lookup(key, &length) ? length : 0;
}

auto KvPreferences::getBytes(const char* key, void* buffer, size_t maxLength) -> size_t
{
    size_t length = 0;
    const uint8_t* data = lookup(key, &length);
    if (!data || !buffer || length > maxLength) return 0;
    memcpy(buffer, data, length);
    return length;
}

auto KvPreferences::lookup(const char* key, size_t* length) -> const uint8_t*
{
    if (!started || !key) return nullptr;
    return store.get(space, key, length);
}
