// Реализация hal/stm32/Stm32SpiBus.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32SpiBus.h"


Stm32SpiBus::Stm32SpiBus(SPIClass& bus, pin_size_t sckPin, pin_size_t misoPin, pin_size_t mosiPin)
: spi(bus), sck(sckPin), miso(misoPin), mosi(mosiPin)
{
}

auto Stm32SpiBus::begin() -> void
{
    // setSCLK()/setMISO()/setMOSI() действуют только до begin().
    spi.setSCLK(sck);
    spi.setMISO(miso);
    spi.setMOSI(mosi);
    spi.begin();
}

auto Stm32SpiBus::beginTransaction(uint32_t clockHz, uint8_t spiMode) -> void
{
    spi.beginTransaction(SPISettings(clockHz, MSBFIRST, spiModeOf(spiMode)));
}

auto Stm32SpiBus::spiModeOf(uint8_t mode) -> SPIMode
{
    switch (mode)
    {
        case 1: return SPI_MODE1;
        case 2: return SPI_MODE2;
        case 3: return SPI_MODE3;
        default: return SPI_MODE0;
    }
}
