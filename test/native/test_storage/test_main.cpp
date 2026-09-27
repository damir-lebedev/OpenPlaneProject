// ============================================================
// STORAGE: хранилище настроек для плат без NVS (STM32) —
// KeyValueStore (образ во флеше с CRC) и KvPreferences (API
// Preferences ESP32 поверх него). Носитель — массив в памяти;
// на STM32 это сектор флеша (hal/stm32/Stm32FlashStorage.h).
//
// Запуск: pio test -e native -f native/test_storage
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "storage/KeyValueStore.h"
#include "storage/KvPreferences.h"

void setUp() {}
void tearDown() {}

namespace
{
    // "Флеш" в памяти: 0xFF после стирания, счётчик записей.
    class RamFlash : public IFlashStorage
    {
    public:
        explicit RamFlash(size_t size = 8192) : bytes(size, 0xFF) {}

        size_t capacity() const override { return bytes.size(); }

        void read(uint8_t* destination, size_t size) override
        {
            memcpy(destination, bytes.data(), size);
            reads++;
        }

        bool write(const uint8_t* source, size_t size) override
        {
            if (failWrites) return false;
            std::fill(bytes.begin(), bytes.end(), 0xFF);   // стирание сектора
            memcpy(bytes.data(), source, size);
            writes++;
            return true;
        }

        std::vector<uint8_t> bytes;
        unsigned reads = 0;
        unsigned writes = 0;
        bool failWrites = false;
    };

    float readFloat(KeyValueStore& store, const char* ns, const char* key)
    {
        size_t length = 0;
        const uint8_t* data = store.get(ns, key, &length);
        TEST_ASSERT_NOT_NULL(data);
        TEST_ASSERT_EQUAL(4, length);
        float v;
        memcpy(&v, data, 4);
        return v;
    }
}


// ---------------- KeyValueStore ----------------

void test_blank_flash_is_empty_not_corrupt()
{
    RamFlash flash;
    KeyValueStore store(flash);
    store.mount();

    TEST_ASSERT_FALSE(store.wasCorrupt());
    TEST_ASSERT_EQUAL(KeyValueStore::HEADER_SIZE, store.bytesUsed());
    TEST_ASSERT_NULL(store.get("imu", "bias", nullptr));
    TEST_ASSERT_FALSE(store.hasNamespace("imu"));
    TEST_ASSERT_EQUAL(1, flash.reads);
}

void test_put_get_and_survive_reboot()
{
    RamFlash flash;
    {
        KeyValueStore store(flash);
        const float trim = 12.5f;
        TEST_ASSERT_TRUE(store.put("autotrim", "roll", &trim, sizeof(trim)));
        const uint8_t level = 3;
        TEST_ASSERT_TRUE(store.put("log", "imu", &level, 1));
        TEST_ASSERT_TRUE(store.isDirty());
        TEST_ASSERT_TRUE(store.commit());
        TEST_ASSERT_FALSE(store.isDirty());
        TEST_ASSERT_EQUAL(1, flash.writes);
    }

    // "Перезагрузка": новый объект читает тот же флеш.
    KeyValueStore store(flash);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, readFloat(store, "autotrim", "roll"));
    size_t length = 0;
    const uint8_t* level = store.get("log", "imu", &length);
    TEST_ASSERT_NOT_NULL(level);
    TEST_ASSERT_EQUAL(1, length);
    TEST_ASSERT_EQUAL(3, level[0]);
    TEST_ASSERT_TRUE(store.hasNamespace("autotrim"));
    TEST_ASSERT_FALSE(store.hasNamespace("auto"));   // префикс — не совпадение
}

void test_same_value_does_not_touch_flash()
{
    RamFlash flash;
    KeyValueStore store(flash);
    const float value = 1.0f;
    store.put("a", "k", &value, 4);
    store.commit();
    TEST_ASSERT_EQUAL(1, flash.writes);

    TEST_ASSERT_TRUE(store.put("a", "k", &value, 4));
    TEST_ASSERT_FALSE(store.isDirty());
    TEST_ASSERT_TRUE(store.commit());
    TEST_ASSERT_EQUAL(1, flash.writes);   // износ флеша не растёт
}

void test_overwrite_with_different_size_and_remove()
{
    RamFlash flash;
    KeyValueStore store(flash);
    const uint8_t one = 1;
    const float two = 2.0f;
    store.put("ns", "a", &one, 1);
    store.put("ns", "b", &two, 4);
    store.put("ns", "a", &two, 4);   // тот же ключ, другой размер

    TEST_ASSERT_EQUAL_FLOAT(2.0f, readFloat(store, "ns", "a"));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, readFloat(store, "ns", "b"));

    TEST_ASSERT_TRUE(store.remove("ns", "a"));
    TEST_ASSERT_FALSE(store.remove("ns", "a"));
    TEST_ASSERT_NULL(store.get("ns", "a", nullptr));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, readFloat(store, "ns", "b"));
}

void test_clear_removes_only_its_namespace()
{
    RamFlash flash;
    KeyValueStore store(flash);
    const uint8_t v = 7;
    store.put("mag", "x", &v, 1);
    store.put("imu", "x", &v, 1);
    store.put("mag", "y", &v, 1);

    TEST_ASSERT_TRUE(store.clear("mag"));
    TEST_ASSERT_FALSE(store.hasNamespace("mag"));
    TEST_ASSERT_TRUE(store.contains("imu", "x"));
    TEST_ASSERT_FALSE(store.clear(""));
}

void test_names_are_limited_like_nvs()
{
    RamFlash flash;
    KeyValueStore store(flash);
    const uint8_t v = 1;
    TEST_ASSERT_TRUE(store.put("123456789012345", "k", &v, 1));    // 15 — можно
    TEST_ASSERT_FALSE(store.put("1234567890123456", "k", &v, 1));  // 16 — нельзя
    TEST_ASSERT_FALSE(store.put("ns", "", &v, 1));
    TEST_ASSERT_FALSE(store.put(nullptr, "k", &v, 1));
    TEST_ASSERT_FALSE(store.put("ns", "k", nullptr, 1));
    TEST_ASSERT_TRUE(store.put("ns", "empty", nullptr, 0));
    size_t length = 99;
    TEST_ASSERT_NOT_NULL(store.get("ns", "empty", &length));
    TEST_ASSERT_EQUAL(0, length);
}

void test_full_store_rejects_without_losing_data()
{
    RamFlash flash;
    KeyValueStore store(flash);
    uint8_t blob[200];
    memset(blob, 0xAB, sizeof(blob));

    char key[8];
    int stored = 0;
    for (int i = 0; i < 20; ++i)
    {
        snprintf(key, sizeof(key), "k%d", i);
        if (store.put("big", key, blob, sizeof(blob))) stored++;
    }
    TEST_ASSERT_TRUE(stored >= 9 && stored < 20);
    TEST_ASSERT_TRUE(store.bytesUsed() <= KeyValueStore::CAPACITY);

    // Перезапись существующего ключа тем же размером при полном образе — можно.
    memset(blob, 0xCD, sizeof(blob));
    TEST_ASSERT_TRUE(store.put("big", "k0", blob, sizeof(blob)));
    size_t length = 0;
    const uint8_t* data = store.get("big", "k0", &length);
    TEST_ASSERT_EQUAL(200, length);
    TEST_ASSERT_EQUAL_HEX8(0xCD, data[199]);

    // Больше, чем влезает, — отказ, старое значение цело.
    uint8_t huge[KeyValueStore::CAPACITY];
    TEST_ASSERT_FALSE(store.put("big", "k1", huge, sizeof(huge)));
    TEST_ASSERT_NOT_NULL(store.get("big", "k1", &length));
    TEST_ASSERT_EQUAL(200, length);
}

void test_corrupted_image_reads_as_empty()
{
    RamFlash flash;
    {
        KeyValueStore store(flash);
        const float v = 3.0f;
        store.put("imu", "bias", &v, 4);
        store.commit();
    }
    flash.bytes[KeyValueStore::HEADER_SIZE + 5] ^= 0x10;   // бит в данных — CRC не сойдётся

    KeyValueStore store(flash);
    store.mount();
    TEST_ASSERT_TRUE(store.wasCorrupt());
    TEST_ASSERT_NULL(store.get("imu", "bias", nullptr));

    // Хранилище после этого работает как пустое.
    const float v = 4.0f;
    TEST_ASSERT_TRUE(store.put("imu", "bias", &v, 4));
    TEST_ASSERT_TRUE(store.commit());
    KeyValueStore again(flash);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, readFloat(again, "imu", "bias"));
}

void test_power_loss_mid_erase_and_garbage()
{
    // Питание пропало после стирания: всё 0xFF — пусто, не ошибка.
    RamFlash erased;
    KeyValueStore a(erased);
    a.mount();
    TEST_ASSERT_FALSE(a.wasCorrupt());

    // Мусор без магии — ошибка, но не падение.
    RamFlash garbage;
    for (size_t i = 0; i < garbage.bytes.size(); ++i) garbage.bytes[i] = static_cast<uint8_t>(i * 37);
    KeyValueStore b(garbage);
    b.mount();
    TEST_ASSERT_TRUE(b.wasCorrupt());

    // Правильная магия и CRC, но длина больше образа — отвергается.
    RamFlash lying;
    const uint8_t header[12] = { 'O', 'P', 'K', 'V', 1, 0, 0xFF, 0x7F, 0, 0, 0, 0 };
    memcpy(lying.bytes.data(), header, sizeof(header));
    KeyValueStore c(lying);
    c.mount();
    TEST_ASSERT_TRUE(c.wasCorrupt());
}

void test_record_structure_is_validated()
{
    // Запись с нулевой длиной имени и верной CRC — всё равно битый образ.
    RamFlash flash;
    uint8_t records[5] = { 0, 1, 1, 0, 'k' };
    const uint32_t crc = KeyValueStore::crc32(records, sizeof(records));
    uint8_t header[12] = { 'O', 'P', 'K', 'V', 1, 0, sizeof(records), 0,
                           static_cast<uint8_t>(crc), static_cast<uint8_t>(crc >> 8),
                           static_cast<uint8_t>(crc >> 16), static_cast<uint8_t>(crc >> 24) };
    memcpy(flash.bytes.data(), header, sizeof(header));
    memcpy(flash.bytes.data() + 12, records, sizeof(records));

    KeyValueStore store(flash);
    store.mount();
    TEST_ASSERT_TRUE(store.wasCorrupt());

    // Другая версия формата — тоже не читаем.
    RamFlash other;
    KeyValueStore writer(other);
    const uint8_t v = 1;
    writer.put("a", "b", &v, 1);
    writer.commit();
    other.bytes[4] = 2;
    KeyValueStore reader(other);
    reader.mount();
    TEST_ASSERT_TRUE(reader.wasCorrupt());
}

void test_small_or_failing_medium()
{
    RamFlash tiny(8);   // меньше заголовка
    KeyValueStore store(tiny);
    store.mount();
    TEST_ASSERT_FALSE(store.wasCorrupt());

    RamFlash failing;
    KeyValueStore s2(failing);
    const uint8_t v = 1;
    s2.put("a", "b", &v, 1);
    failing.failWrites = true;
    TEST_ASSERT_FALSE(s2.commit());
    TEST_ASSERT_TRUE(s2.isDirty());   // попробует снова в следующий раз
    failing.failWrites = false;
    TEST_ASSERT_TRUE(s2.commit());
    TEST_ASSERT_EQUAL(1, s2.commitCount());
}

void test_crc32_reference_value()
{
    // CRC-32/ISO-HDLC("123456789") = 0xCBF43926.
    const uint8_t text[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, KeyValueStore::crc32(text, sizeof(text)));
}


// ---------------- KvPreferences (API Preferences) ----------------

void test_preferences_roundtrip_all_types()
{
    RamFlash flash;
    KeyValueStore store(flash);
    {
        KvPreferences prefs(store);
        TEST_ASSERT_TRUE(prefs.begin("calib", false));
        TEST_ASSERT_EQUAL(1, prefs.putBool("done", true));
        TEST_ASSERT_EQUAL(1, prefs.putUChar("level", 200));
        TEST_ASSERT_EQUAL(4, prefs.putFloat("gain", 0.75f));
        TEST_ASSERT_EQUAL(4, prefs.putInt("offset", -123456));
        TEST_ASSERT_EQUAL(4, prefs.putUInt("count", 4000000000u));
        const float matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        TEST_ASSERT_EQUAL(sizeof(matrix), prefs.putBytes("matrix", matrix, sizeof(matrix)));
        prefs.end();
    }
    TEST_ASSERT_EQUAL(1, flash.writes);

    KeyValueStore rebooted(flash);
    KvPreferences prefs(rebooted);
    TEST_ASSERT_TRUE(prefs.begin("calib", true));
    TEST_ASSERT_TRUE(prefs.getBool("done"));
    TEST_ASSERT_EQUAL(200, prefs.getUChar("level"));
    TEST_ASSERT_EQUAL_FLOAT(0.75f, prefs.getFloat("gain"));
    TEST_ASSERT_EQUAL(-123456, prefs.getInt("offset"));
    TEST_ASSERT_EQUAL_UINT32(4000000000u, prefs.getUInt("count"));
    TEST_ASSERT_EQUAL(36, prefs.getBytesLength("matrix"));
    float matrix[9] = {};
    TEST_ASSERT_EQUAL(36, prefs.getBytes("matrix", matrix, sizeof(matrix)));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, matrix[8]);
    TEST_ASSERT_TRUE(prefs.isKey("gain"));
    TEST_ASSERT_FALSE(prefs.isKey("nope"));
}

void test_preferences_defaults_like_esp32()
{
    RamFlash flash;
    KeyValueStore store(flash);
    KvPreferences prefs(store);

    // Только чтение, пространства ещё нет — как у NVS.
    TEST_ASSERT_FALSE(prefs.begin("autotrim", true));
    TEST_ASSERT_TRUE(prefs.begin("autotrim", false));
    TEST_ASSERT_FALSE(prefs.begin("autotrim", false));   // уже открыто

    TEST_ASSERT_TRUE(std::isnan(prefs.getFloat("roll")));
    TEST_ASSERT_EQUAL_FLOAT(5.0f, prefs.getFloat("roll", 5.0f));
    TEST_ASSERT_FALSE(prefs.getBool("x"));
    TEST_ASSERT_TRUE(prefs.getBool("x", true));
    TEST_ASSERT_EQUAL(0, prefs.getBytesLength("x"));

    // Ключ другого размера — значение по умолчанию.
    prefs.putUChar("small", 9);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, prefs.getFloat("small", 1.5f));

    // Буфер меньше значения — 0 байт.
    const uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    prefs.putBytes("blob", data, sizeof(data));
    uint8_t small[4];
    TEST_ASSERT_EQUAL(0, prefs.getBytes("blob", small, sizeof(small)));
    TEST_ASSERT_EQUAL(0, prefs.getBytes("blob", nullptr, 100));
    prefs.end();

    // Без begin() ничего не читается и не пишется.
    TEST_ASSERT_EQUAL(0, prefs.putFloat("roll", 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, prefs.getFloat("roll", 2.0f));
    TEST_ASSERT_FALSE(prefs.begin(nullptr));
    TEST_ASSERT_FALSE(prefs.begin(""));
    TEST_ASSERT_FALSE(prefs.begin("name-longer-than-15"));
}

void test_preferences_read_only_cannot_write()
{
    RamFlash flash;
    KeyValueStore store(flash);
    {
        KvPreferences prefs(store);
        prefs.begin("log");
        prefs.putUChar("imu", 1);
    }   // деструктор = end()
    TEST_ASSERT_EQUAL(1, flash.writes);

    KvPreferences prefs(store);
    TEST_ASSERT_TRUE(prefs.begin("log", true));
    TEST_ASSERT_EQUAL(0, prefs.putUChar("imu", 2));
    TEST_ASSERT_FALSE(prefs.remove("imu"));
    TEST_ASSERT_FALSE(prefs.clear());
    prefs.end();
    TEST_ASSERT_EQUAL(1, flash.writes);

    TEST_ASSERT_TRUE(prefs.begin("log"));
    TEST_ASSERT_TRUE(prefs.remove("imu"));
    TEST_ASSERT_TRUE(prefs.clear());
    prefs.end();
    TEST_ASSERT_EQUAL(2, flash.writes);
}

void test_preferences_unchanged_values_skip_commit()
{
    RamFlash flash;
    KeyValueStore store(flash);
    for (int boot = 0; boot < 3; ++boot)
    {
        KvPreferences prefs(store);
        prefs.begin("autotrim");
        prefs.putFloat("roll", 10.0f);
        prefs.putFloat("pitch", -4.0f);
        prefs.end();
    }
    TEST_ASSERT_EQUAL(1, flash.writes);
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_blank_flash_is_empty_not_corrupt);
    RUN_TEST(test_put_get_and_survive_reboot);
    RUN_TEST(test_same_value_does_not_touch_flash);
    RUN_TEST(test_overwrite_with_different_size_and_remove);
    RUN_TEST(test_clear_removes_only_its_namespace);
    RUN_TEST(test_names_are_limited_like_nvs);
    RUN_TEST(test_full_store_rejects_without_losing_data);
    RUN_TEST(test_corrupted_image_reads_as_empty);
    RUN_TEST(test_power_loss_mid_erase_and_garbage);
    RUN_TEST(test_record_structure_is_validated);
    RUN_TEST(test_small_or_failing_medium);
    RUN_TEST(test_crc32_reference_value);
    RUN_TEST(test_preferences_roundtrip_all_types);
    RUN_TEST(test_preferences_defaults_like_esp32);
    RUN_TEST(test_preferences_read_only_cannot_write);
    RUN_TEST(test_preferences_unchanged_values_skip_commit);
    return UNITY_END();
}
