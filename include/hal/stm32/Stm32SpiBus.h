#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "hal/ISpiBus.h"

// ============================================================
// Реализация ISpiBus для STM32 (STM32duino 3.x) — обёртка над
// SPIClass. ЗАГОТОВКА: на железе не проверялась.
//
// Пины SCK/MISO/MOSI фиксируются в конструкторе и применяются в
// begin(); контроллер (SPI1, SPI2...) ядро выбирает по ним само.
// Аппаратный NSS не используется: CS каждого датчика переключает
// SpiRegisterDevice через digitalWrite (см. ISpiBus.h) — так же,
// как на ESP32.
// ============================================================

class Stm32SpiBus : public ISpiBus
{
public:
    Stm32SpiBus(SPIClass& bus, pin_size_t sckPin, pin_size_t misoPin, pin_size_t mosiPin);

    void begin() override;

    void beginTransaction(uint32_t clockHz, uint8_t spiMode) override;

    uint8_t transfer(uint8_t data) override { return spi.transfer(data); }
    void endTransaction() override { spi.endTransaction(); }


private:
    SPIClass& spi;
    pin_size_t sck;
    pin_size_t miso;
    pin_size_t mosi;

    static SPIMode spiModeOf(uint8_t mode);
};
