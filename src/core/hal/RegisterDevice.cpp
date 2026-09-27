// Реализация hal/RegisterDevice.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/RegisterDevice.h"


auto IRegisterDevice::readRegister(uint8_t reg) -> int
{
    uint8_t value;
    return readRegisters(reg, &value, 1) ? value : -1;
}

I2cRegisterDevice::I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress)
: bus(i2cBus),
      address(deviceAddress),
      alternate(alternateAddress)
{
}

auto I2cRegisterDevice::begin() -> void
{
    if (alternate != 0 && !bus.probe(address) && bus.probe(alternate))
    {
        const uint8_t primary = address;
        address = alternate;
        alternate = primary;
    }
}

auto I2cRegisterDevice::probe() -> bool
{
    return bus.probe(address);
}

auto I2cRegisterDevice::writeRegister(uint8_t reg, uint8_t value) -> bool
{
    return bus.writeRegister(address, reg, value);
}

auto I2cRegisterDevice::writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) -> bool
{
    return bus.writeRegisters(address, reg, data, count);
}

auto I2cRegisterDevice::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) -> bool
{
    return bus.readRegisters(address, reg, buffer, count);
}

SpiRegisterDevice::SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin,
                      uint32_t clockFrequencyHz, uint8_t dummyBytesBeforeData,
                      uint8_t mode)
: bus(spiBus),
      csPin(chipSelectPin),
      clockHz(clockFrequencyHz),
      dummyReadBytes(dummyBytesBeforeData),
      spiMode(mode)
{
}

auto SpiRegisterDevice::begin() -> void
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}

auto SpiRegisterDevice::probe() -> bool
{
    return true;
}

auto SpiRegisterDevice::writeRegister(uint8_t reg, uint8_t value) -> bool
{
    return writeRegisters(reg, &value, 1);
}

auto SpiRegisterDevice::writeRegisters(uint8_t reg, const uint8_t* data, uint8_t count) -> bool
{
    select();
    bus.transfer(reg & 0x7F);
    for (uint8_t i = 0; i < count; ++i)
    {
        bus.transfer(data[i]);
    }
    deselect();
    return true;
}

auto SpiRegisterDevice::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) -> bool
{
    select();
    bus.transfer(reg | 0x80);
    for (uint8_t i = 0; i < dummyReadBytes; ++i)
    {
        bus.transfer(0x00);
    }
    for (uint8_t i = 0; i < count; ++i)
    {
        buffer[i] = bus.transfer(0x00);
    }
    deselect();
    return true;
}

auto SpiRegisterDevice::select() -> void
{
    bus.beginTransaction(clockHz, spiMode);
    digitalWrite(csPin, LOW);
}

auto SpiRegisterDevice::deselect() -> void
{
    digitalWrite(csPin, HIGH);
    bus.endTransaction();
}
