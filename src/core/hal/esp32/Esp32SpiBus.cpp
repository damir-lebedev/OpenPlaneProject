// Реализация hal/esp32/Esp32SpiBus.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/esp32/Esp32SpiBus.h"


Esp32SpiBus::Esp32SpiBus(int8_t sckPin, int8_t misoPin, int8_t mosiPin)
: sck(sckPin), miso(misoPin), mosi(mosiPin)
{
}

auto Esp32SpiBus::begin() -> void
{
    SPI.begin(sck, miso, mosi, -1);  // CS = -1: им управляют сами датчики
}

auto Esp32SpiBus::beginTransaction(uint32_t clockHz, uint8_t spiMode) -> void
{
    SPI.beginTransaction(SPISettings(clockHz, MSBFIRST, spiModeOf(spiMode)));
}

auto Esp32SpiBus::spiModeOf(uint8_t mode) -> uint8_t
{
    switch (mode)
    {
        case 1: return SPI_MODE1;
        case 2: return SPI_MODE2;
        case 3: return SPI_MODE3;
        default: return SPI_MODE0;
    }
}
