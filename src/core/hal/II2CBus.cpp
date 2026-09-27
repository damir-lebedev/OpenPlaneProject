// Реализация hal/II2CBus.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/II2CBus.h"


auto II2CBus::writeRegister(uint8_t address, uint8_t reg, uint8_t value) -> bool
{
    beginTransmission(address);
    write(reg);
    write(value);
    return endTransmission() == 0;
}

auto II2CBus::writeRegisters(uint8_t address, uint8_t reg, const uint8_t* data, uint8_t count) -> bool
{
    beginTransmission(address);
    write(reg);
    write(data, count);
    return endTransmission() == 0;
}

auto II2CBus::readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t count) -> bool
{
    beginTransmission(address);
    write(reg);
    if (endTransmission(false) != 0) return false;

    if (requestFrom(address, count) != count) return false;

    for (uint8_t i = 0; i < count; ++i)
    {
        buffer[i] = static_cast<uint8_t>(read());
    }
    return true;
}

auto II2CBus::readRegister(uint8_t address, uint8_t reg) -> int
{
    uint8_t value;
    return readRegisters(address, reg, &value, 1) ? value : -1;
}

auto II2CBus::probe(uint8_t address) -> bool
{
    beginTransmission(address);
    return endTransmission() == 0;
}
