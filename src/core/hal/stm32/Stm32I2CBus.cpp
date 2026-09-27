// Реализация hal/stm32/Stm32I2CBus.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32I2CBus.h"


Stm32I2CBus::Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz)
: wire(bus),
      sda(sdaPin),
      scl(sclPin),
      frequency(frequencyHz)
{
}

auto Stm32I2CBus::begin() -> void
{
    // setSDA()/setSCL() действуют только до begin().
    wire.setSDA(sda);
    wire.setSCL(scl);
    wire.begin();
    wire.setClock(frequency);
}

auto Stm32I2CBus::requestFrom(uint8_t address, uint8_t quantity) -> uint8_t
{
    return static_cast<uint8_t>(wire.requestFrom(address, static_cast<size_t>(quantity)));
}
