#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "hal/ISpiBus.h"

// ============================================================
// Реализация ISpiBus для ESP32 (Arduino core) — обёртка над
// глобальным SPI. Пины SCK/MISO/MOSI фиксируются в конструкторе;
// CS каждое устройство держит и переключает само (см. ISpiBus.h).
// ============================================================

class Esp32SpiBus : public ISpiBus
{
public:
    Esp32SpiBus(int8_t sckPin, int8_t misoPin, int8_t mosiPin);

    void begin() override;

    void beginTransaction(uint32_t clockHz, uint8_t spiMode) override;

    uint8_t transfer(uint8_t data) override { return SPI.transfer(data); }
    void endTransaction() override { SPI.endTransaction(); }


private:
    int8_t sck;
    int8_t miso;
    int8_t mosi;

    static uint8_t spiModeOf(uint8_t mode);
};
