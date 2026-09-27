// Реализация hal/esp32/Esp32I2CBus.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/esp32/Esp32I2CBus.h"


Esp32I2CBus::Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz)
: wire(bus),
      sda(sdaPin),
      scl(sclPin),
      frequency(frequencyHz)
{
}

auto Esp32I2CBus::begin() -> void
{
    wire.begin(sda, scl, frequency);
    wire.setTimeOut(TIMEOUT_MS);
}

auto Esp32I2CBus::requestFrom(uint8_t address, uint8_t quantity) -> uint8_t
{
    return wire.requestFrom(address, quantity);
}
