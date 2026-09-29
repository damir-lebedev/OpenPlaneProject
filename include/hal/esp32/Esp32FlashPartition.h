#pragma once
#include <esp_partition.h>

#include "hal/IFlashRegion.h"

// ============================================================
// Раздел данных на встроенном флеше ESP32 по имени (esp_partition).
// Таблица разделов — partitions_blackbox.csv (platformio.ini,
// board_build.partitions). Раздела нет (другая таблица, другая
// плата) — size() == 0, и тот, кто им пользуется, выключается.
// ============================================================

class Esp32FlashPartition : public IFlashRegion
{
public:

    explicit Esp32FlashPartition(const char* partitionName)
        : name(partitionName)
    {
    }

    // Найти раздел. Отдельно от конструктора: таблица разделов
    // доступна только после старта ядра, а объекты в main.cpp —
    // глобальные.
    bool begin()
    {
        partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, name);
        return partition != nullptr;
    }

    uint32_t size() const override { return partition ? partition->size : 0; }

    bool read(uint32_t offset, void* data, size_t length) override
    {
        return partition && esp_partition_read(partition, offset, data, length) == ESP_OK;
    }

    bool write(uint32_t offset, const void* data, size_t length) override
    {
        return partition && esp_partition_write(partition, offset, data, length) == ESP_OK;
    }

    bool erase(uint32_t offset, uint32_t length) override
    {
        return partition && esp_partition_erase_range(partition, offset, length) == ESP_OK;
    }


private:

    const char* name;
    const esp_partition_t* partition = nullptr;
};
