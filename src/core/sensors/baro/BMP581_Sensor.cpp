// Реализация sensors/baro/BMP581_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/baro/BMP581_Sensor.h"


auto BMP581_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
}

BMP581_Sensor::BMP581_Sensor(IRegisterDevice& registerDevice, const char* sensorName)
: BarometerBase(sensorName, POLL_PERIOD_US),
      device(registerDevice)
{
}

auto BMP581_Sensor::begin() -> bool
{
    device.begin();
    delay(2);

    device.readRegister(REG_CHIP_ID);  // SPI: первое чтение переводит интерфейс в SPI
    const int chipId = device.readRegister(REG_CHIP_ID);
    if (chipId != CHIP_ID_581 && chipId != CHIP_ID_585)
    {
        printName();
        if (chipId < 0) Serial.println(": не отвечает");
        else { Serial.print(": неверный chip ID 0x"); Serial.println(chipId, HEX); }
        return fail();
    }

    device.writeRegister(REG_CMD, CMD_SOFT_RESET);
    delay(RESET_TIME_MS);
    device.readRegister(REG_CHIP_ID);  // SPI: то же после сброса

    const int intStatus = device.readRegister(REG_INT_STATUS);
    const int status = device.readRegister(REG_STATUS);
    if (intStatus < 0 || !(intStatus & INT_POR_COMPLETE) ||
        status < 0 || !(status & STATUS_NVM_RDY) || (status & STATUS_NVM_ERR))
    {
        printName();
        Serial.println(": не вышел из сброса (NVM)");
        return fail();
    }

    const int dsp = device.readRegister(REG_DSP_CONFIG);
    if (dsp < 0) return fail();

    const bool ok =
        device.writeRegister(REG_ODR_CONFIG, ODR_DEEP_DISABLE | (ODR_50HZ << 2) | PWR_STANDBY) &&
        device.writeRegister(REG_OSR_CONFIG, OSR_PRESS_EN | (OSR_16X << 3) | OSR_2X) &&
        device.writeRegister(REG_DSP_CONFIG, static_cast<uint8_t>(dsp | DSP_SHDW_IIR_T | DSP_SHDW_IIR_P)) &&
        device.writeRegister(REG_DSP_IIR, IIR_COEF_3 << 3) &&
        device.writeRegister(REG_INT_SOURCE, INT_SOURCE_DRDY) &&
        device.writeRegister(REG_ODR_CONFIG, ODR_DEEP_DISABLE | (ODR_50HZ << 2) | PWR_NORMAL);

    if (!ok)
    {
        printName();
        Serial.println(": ошибка записи регистров");
        return fail();
    }

    const int osrEff = device.readRegister(REG_OSR_EFF);
    if (osrEff >= 0 && !(osrEff & OSR_EFF_ODR_VALID))
    {
        printName();
        Serial.println(": предупреждение — ODR не успевает за оверсэмплингом");
    }

    printName();
    Serial.println(": подключён");
    setAvailable(true);
    return true;
}

auto BMP581_Sensor::isNewSampleReady(bool& ready) -> bool
{
    const int intStatus = device.readRegister(REG_INT_STATUS);  // флаги сбрасываются чтением
    if (intStatus < 0) return false;

    // На случай, если флаг готовности на конкретном чипе не
    // выставляется, отсчёт всё равно читается раз в два периода ODR.
    const uint32_t now = micros();
    ready = (intStatus & INT_DRDY) != 0 || now - lastSampleUs >= FALLBACK_PERIOD_US;
    if (ready) lastSampleUs = now;
    return true;
}

auto BMP581_Sensor::readSample(float& pressurePa, float& temperatureC) -> bool
{
    uint8_t raw[6];  // температура XLSB..MSB, давление XLSB..MSB
    if (!device.readRegisters(REG_TEMP_XLSB, raw, sizeof(raw))) return false;

    uint32_t t = static_cast<uint32_t>(raw[0]) | (static_cast<uint32_t>(raw[1]) << 8) |
                 (static_cast<uint32_t>(raw[2]) << 16);
    if (t & 0x800000u) t |= 0xFF000000u;  // знак int24
    const uint32_t p = static_cast<uint32_t>(raw[3]) | (static_cast<uint32_t>(raw[4]) << 8) |
                       (static_cast<uint32_t>(raw[5]) << 16);

    temperatureC = static_cast<float>(static_cast<int32_t>(t)) / 65536.0f;
    pressurePa = static_cast<float>(p) / 64.0f;
    return true;
}

auto BMP581_Sensor::printName() const -> void
{
    Serial.print(getSensorType());
}

auto BMP581_Sensor::fail() -> bool
{
    setAvailable(false);
    return false;
}
