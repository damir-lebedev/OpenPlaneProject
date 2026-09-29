#pragma once

// ============================================================
// Нативная замена esp_partition.h (ESP-IDF 4.4): разделы флеша.
//
// Раздел — вектор байт с поведением NOR-флеша: стирание — только
// секторами 4 КБ (иначе ошибка), стёртое = 0xFF, запись только
// опускает биты (новое & старое) — попытку поднять бит без стирания
// фейк считает (bitRaises), как на железе она молча портит данные.
//
// Тест регистрирует раздел (fake::addPartition), читает и портит его
// байты напрямую и считает операции. beforeWrite — крючок "пропало
// питание": вернул false — запись не выполняется.
// ============================================================

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x104

typedef enum
{
    ESP_PARTITION_TYPE_APP = 0x00,
    ESP_PARTITION_TYPE_DATA = 0x01,
} esp_partition_type_t;

typedef enum
{
    ESP_PARTITION_SUBTYPE_ANY = 0xff,
} esp_partition_subtype_t;

typedef struct
{
    esp_partition_type_t type;
    esp_partition_subtype_t subtype;
    uint32_t address;
    uint32_t size;
    char label[17];
    bool encrypted;
} esp_partition_t;

namespace fake
{
    struct FlashPartition
    {
        esp_partition_t info = {};
        std::vector<uint8_t> data;

        uint32_t reads = 0;
        uint32_t writes = 0;
        uint32_t bytesWritten = 0;
        uint32_t erases = 0;          // вызовов erase_range
        uint32_t sectorsErased = 0;
        uint32_t bitRaises = 0;       // запись пыталась поднять бит 0 -> 1
        std::function<bool(uint32_t offset, size_t length)> beforeWrite;
    };

    inline std::vector<std::unique_ptr<FlashPartition>>& partitions()
    {
        static std::vector<std::unique_ptr<FlashPartition>> all;
        return all;
    }

    // Новый раздел данных; fill — начальное содержимое (0xFF — стёрт).
    inline FlashPartition& addPartition(const char* label, uint32_t size, uint8_t fill = 0xFF)
    {
        std::unique_ptr<FlashPartition> p(new FlashPartition());
        p->info.type = ESP_PARTITION_TYPE_DATA;
        p->info.subtype = static_cast<esp_partition_subtype_t>(0x40);
        p->info.address = 0x210000;
        p->info.size = size;
        strncpy(p->info.label, label, sizeof(p->info.label) - 1);
        p->data.assign(size, fill);
        partitions().push_back(std::move(p));
        return *partitions().back();
    }

    inline FlashPartition* partition(const char* label)
    {
        for (auto& p : partitions())
        {
            if (strcmp(p->info.label, label) == 0) return p.get();
        }
        return nullptr;
    }

    inline FlashPartition* partitionOf(const esp_partition_t* info)
    {
        for (auto& p : partitions())
        {
            if (&p->info == info) return p.get();
        }
        return nullptr;
    }

    inline void resetPartitions() { partitions().clear(); }
}

inline const esp_partition_t* esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype,
                                                       const char* label)
{
    for (auto& p : fake::partitions())
    {
        if (p->info.type != type) continue;
        if (subtype != ESP_PARTITION_SUBTYPE_ANY && p->info.subtype != subtype) continue;
        if (label && strcmp(p->info.label, label) != 0) continue;
        return &p->info;
    }
    return nullptr;
}

inline esp_err_t esp_partition_read(const esp_partition_t* partition, size_t offset, void* dst, size_t size)
{
    fake::FlashPartition* p = fake::partitionOf(partition);
    if (!p || offset + size > p->data.size()) return ESP_ERR_INVALID_SIZE;
    memcpy(dst, p->data.data() + offset, size);
    p->reads++;
    return ESP_OK;
}

inline esp_err_t esp_partition_write(const esp_partition_t* partition, size_t offset, const void* src, size_t size)
{
    fake::FlashPartition* p = fake::partitionOf(partition);
    if (!p || offset + size > p->data.size()) return ESP_ERR_INVALID_SIZE;
    if (p->beforeWrite && !p->beforeWrite(static_cast<uint32_t>(offset), size)) return ESP_FAIL;
    const uint8_t* bytes = static_cast<const uint8_t*>(src);
    for (size_t i = 0; i < size; ++i)
    {
        uint8_t& cell = p->data[offset + i];
        if (bytes[i] & ~cell) p->bitRaises++;
        cell &= bytes[i];
    }
    p->writes++;
    p->bytesWritten += static_cast<uint32_t>(size);
    return ESP_OK;
}

inline esp_err_t esp_partition_erase_range(const esp_partition_t* partition, size_t offset, size_t size)
{
    fake::FlashPartition* p = fake::partitionOf(partition);
    if (!p || offset + size > p->data.size()) return ESP_ERR_INVALID_SIZE;
    if (offset % 4096 != 0 || size % 4096 != 0) return ESP_ERR_INVALID_ARG;
    memset(p->data.data() + offset, 0xFF, size);
    p->erases++;
    p->sectorsErased += static_cast<uint32_t>(size / 4096);
    return ESP_OK;
}
