#pragma once
#include "hal/ISpiBus.h"
#include "hal/Rtos.h"

// ============================================================
// Шина SPI, которой пользуются две задачи (у F405 флеш ящика и OSD
// бывают на одной SPI): каждая транзакция — под мьютексом. begin() —
// один раз, до задач; мьютекс создаётся там же.
// ============================================================

class LockedSpiBus : public ISpiBus
{
public:
    explicit LockedSpiBus(ISpiBus& inner)
        : bus(inner)
    {
    }

    void begin() override
    {
        if (!lock) lock = xSemaphoreCreateMutex();
        if (!started) bus.begin();
        started = true;
    }

    void beginTransaction(uint32_t clockHz, uint8_t spiMode) override
    {
        if (lock) xSemaphoreTake(lock, portMAX_DELAY);
        bus.beginTransaction(clockHz, spiMode);
    }

    uint8_t transfer(uint8_t data) override { return bus.transfer(data); }

    void endTransaction() override
    {
        bus.endTransaction();
        if (lock) xSemaphoreGive(lock);
    }

private:
    ISpiBus& bus;
    SemaphoreHandle_t lock = nullptr;
    bool started = false;
};
