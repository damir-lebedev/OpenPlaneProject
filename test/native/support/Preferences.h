#pragma once

// ============================================================
// Нативная замена Preferences (NVS) из Arduino core ESP32 2.0.x.
//
// Хранилище — в памяти (fake::nvs()), общее для всех объектов
// Preferences, как флеш на плате. Поведение повторяет оригинал:
// begin() только на чтение не создаёт отсутствующее пространство
// имён, get*() без ключа возвращают значение по умолчанию,
// getBytes() — 0, если буфер меньше значения.
// ============================================================

#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace fake
{
    struct NvsState
    {
        std::map<std::string, std::map<std::string, std::vector<uint8_t>>> spaces;
        bool failBegin = false;
        uint32_t commits = 0;   // сколько раз что-то записали (end() после put*)
    };

    inline NvsState& nvs()
    {
        static NvsState state;
        return state;
    }

    inline void resetNvs() { nvs() = NvsState(); }
}

class Preferences
{
public:
    ~Preferences() { end(); }

    bool begin(const char* name, bool readOnly = false, const char* partitionLabel = nullptr)
    {
        (void)partitionLabel;
        if (started || fake::nvs().failBegin || !name) return false;
        auto& spaces = fake::nvs().spaces;
        if (readOnly && spaces.find(name) == spaces.end()) return false;
        space = &spaces[name];
        readOnlyMode = readOnly;
        started = true;
        dirty = false;
        return true;
    }

    void end()
    {
        if (started && dirty) fake::nvs().commits++;
        started = false;
        space = nullptr;
    }

    bool clear()
    {
        if (!writable()) return false;
        space->clear();
        dirty = true;
        return true;
    }

    bool remove(const char* key)
    {
        if (!writable()) return false;
        dirty = true;
        return space->erase(key) > 0;
    }

    bool isKey(const char* key) const { return started && space->count(key) > 0; }

    size_t putBool(const char* key, bool value) { return putValue(key, static_cast<uint8_t>(value ? 1 : 0)); }
    size_t putUChar(const char* key, uint8_t value) { return putValue(key, value); }
    size_t putFloat(const char* key, float value) { return putValue(key, value); }
    size_t putBytes(const char* key, const void* value, size_t length)
    {
        if (!writable() || !key || (!value && length)) return 0;
        const uint8_t* bytes = static_cast<const uint8_t*>(value);
        (*space)[key] = std::vector<uint8_t>(bytes, bytes + length);
        dirty = true;
        return length;
    }

    bool getBool(const char* key, bool defaultValue = false) const
    {
        return getValue<uint8_t>(key, defaultValue ? 1 : 0) != 0;
    }
    uint8_t getUChar(const char* key, uint8_t defaultValue = 0) const { return getValue(key, defaultValue); }
    float getFloat(const char* key, float defaultValue = NAN) const { return getValue(key, defaultValue); }

    size_t getBytesLength(const char* key) const
    {
        const std::vector<uint8_t>* v = lookup(key);
        return v ? v->size() : 0;
    }

    size_t getBytes(const char* key, void* buffer, size_t maxLength) const
    {
        const std::vector<uint8_t>* v = lookup(key);
        if (!v || !buffer || v->size() > maxLength) return 0;
        memcpy(buffer, v->data(), v->size());
        return v->size();
    }

private:
    std::map<std::string, std::vector<uint8_t>>* space = nullptr;
    bool started = false;
    bool readOnlyMode = false;
    bool dirty = false;

    bool writable() const { return started && !readOnlyMode; }

    const std::vector<uint8_t>* lookup(const char* key) const
    {
        if (!started || !key) return nullptr;
        auto it = space->find(key);
        return it == space->end() ? nullptr : &it->second;
    }

    template <typename T>
    size_t putValue(const char* key, T value)
    {
        return putBytes(key, &value, sizeof(value));
    }

    template <typename T>
    T getValue(const char* key, T defaultValue) const
    {
        const std::vector<uint8_t>* v = lookup(key);
        if (!v || v->size() != sizeof(T)) return defaultValue;
        T value;
        memcpy(&value, v->data(), sizeof(T));
        return value;
    }
};
