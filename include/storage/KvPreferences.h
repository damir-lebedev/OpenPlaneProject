#pragma once
#include <math.h>

#include "storage/KeyValueStore.h"

// ============================================================
// API класса Preferences (ESP32 Arduino core) поверх KeyValueStore.
//
// Код проекта хранит калибровки и настройки через Preferences — на
// ESP32 это NVS. На платах без NVS (STM32) заголовок
// hal/stm32/compat/Preferences.h подставляет вместо него наследника
// этого класса, и драйверы датчиков, автотриммер и настройки лога
// собираются без единого #ifdef.
//
// Поведение как у оригинала там, где на него опирается код:
//   - begin(name, true) для пространства, которого ещё нет, — false;
//   - get*() без ключа или с ключом другого размера — значение по
//     умолчанию (getFloat — NAN);
//   - getBytes() — 0, если буфер меньше значения;
//   - запись на носитель — в end() (и в деструкторе), только если
//     что-то поменялось.
// Только то подмножество методов, которым пользуется проект.
// ============================================================

class KvPreferences
{
public:

    explicit KvPreferences(KeyValueStore& storeRef)
        : store(storeRef)
    {
    }

    ~KvPreferences() { end(); }

    KvPreferences(const KvPreferences&) = delete;
    KvPreferences& operator=(const KvPreferences&) = delete;

    bool begin(const char* name, bool readOnly = false, const char* partitionLabel = nullptr)
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

    void end()
    {
        if (!started) return;
        started = false;
        if (!readOnlyMode) store.commit();
    }

    bool clear() { return writable() && store.clear(space); }
    bool remove(const char* key) { return writable() && store.remove(space, key); }
    bool isKey(const char* key) { return started && store.contains(space, key); }

    size_t putBool(const char* key, bool value) { return putValue(key, static_cast<uint8_t>(value ? 1 : 0)); }
    size_t putUChar(const char* key, uint8_t value) { return putValue(key, value); }
    size_t putInt(const char* key, int32_t value) { return putValue(key, value); }
    size_t putUInt(const char* key, uint32_t value) { return putValue(key, value); }
    size_t putFloat(const char* key, float value) { return putValue(key, value); }

    size_t putBytes(const char* key, const void* value, size_t length)
    {
        if (!writable() || !key || (!value && length > 0)) return 0;
        return store.put(space, key, value, length) ? length : 0;
    }

    bool getBool(const char* key, bool defaultValue = false)
    {
        return getValue<uint8_t>(key, defaultValue ? 1 : 0) != 0;
    }
    uint8_t getUChar(const char* key, uint8_t defaultValue = 0) { return getValue(key, defaultValue); }
    int32_t getInt(const char* key, int32_t defaultValue = 0) { return getValue(key, defaultValue); }
    uint32_t getUInt(const char* key, uint32_t defaultValue = 0) { return getValue(key, defaultValue); }
    float getFloat(const char* key, float defaultValue = NAN) { return getValue(key, defaultValue); }

    size_t getBytesLength(const char* key)
    {
        size_t length = 0;
        return lookup(key, &length) ? length : 0;
    }

    size_t getBytes(const char* key, void* buffer, size_t maxLength)
    {
        size_t length = 0;
        const uint8_t* data = lookup(key, &length);
        if (!data || !buffer || length > maxLength) return 0;
        memcpy(buffer, data, length);
        return length;
    }


private:

    KeyValueStore& store;
    char space[KeyValueStore::MAX_NAME_LENGTH + 1] = {};
    bool started = false;
    bool readOnlyMode = false;

    bool writable() const { return started && !readOnlyMode; }

    const uint8_t* lookup(const char* key, size_t* length)
    {
        if (!started || !key) return nullptr;
        return store.get(space, key, length);
    }

    template <typename T>
    size_t putValue(const char* key, T value)
    {
        return putBytes(key, &value, sizeof(value));
    }

    template <typename T>
    T getValue(const char* key, T defaultValue)
    {
        size_t length = 0;
        const uint8_t* data = lookup(key, &length);
        if (!data || length != sizeof(T)) return defaultValue;
        T value;
        memcpy(&value, data, sizeof(T));
        return value;
    }
};
