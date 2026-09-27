// Реализация sensors/baro/SPL06_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/baro/SPL06_Sensor.h"


auto SPL06_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
}

SPL06_Sensor::SPL06_Sensor(IRegisterDevice& registerDevice, const char* sensorName)
: BarometerBase(sensorName, POLL_PERIOD_US),
      device(registerDevice),
      coef()
{
}

auto SPL06_Sensor::begin() -> bool
{
    device.begin();
    delay(5);

    const int id = device.readRegister(REG_ID);
    if (id != ID_VALUE)
    {
        printName();
        if (id < 0) Serial.println(": не отвечает");
        else if (id == ID_SPA06) Serial.println(": это SPA06 (ID 0x11) — другие коэффициенты, не поддержан");
        else { Serial.print(": неверный ID 0x"); Serial.println(id, HEX); }
        setAvailable(false);
        return false;
    }

    device.writeRegister(REG_RESET, RESET_SOFT);
    delay(RESET_TIME_MS);  // коэффициенты доступны через 40 мс после сброса

    if (!waitReady() || !readCoefficients())
    {
        printName();
        Serial.println(": не удалось прочитать коэффициенты");
        setAvailable(false);
        return false;
    }

    // Температура — тем датчиком, которым чип калибровали на заводе.
    const int source = device.readRegister(REG_COEF_SRCE);
    const uint8_t tmpExt = (source < 0 || (source & 0x80)) ? TMP_EXT : 0;

    const bool ok =
        device.writeRegister(REG_PRS_CFG, (RATE_32 << 4) | PRC_16) &&
        device.writeRegister(REG_TMP_CFG, tmpExt | (RATE_4 << 4) | PRC_1) &&
        device.writeRegister(REG_CFG, CFG_P_SHIFT) &&
        device.writeRegister(REG_MEAS_CFG, MEAS_CONTINUOUS_PT);

    if (!ok)
    {
        printName();
        Serial.println(": ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    printName();
    Serial.println(": подключён");
    setAvailable(true);
    return true;
}

auto SPL06_Sensor::isNewSampleReady(bool& ready) -> bool
{
    const int meas = device.readRegister(REG_MEAS_CFG);
    if (meas < 0) return false;

    ready = (meas & MEAS_PRS_RDY) != 0;
    return true;
}

auto SPL06_Sensor::readSample(float& pressurePa, float& temperatureC) -> bool
{
    uint8_t raw[6];  // PSR_B2..B0, TMP_B2..B0
    if (!device.readRegisters(REG_PSR_B2, raw, sizeof(raw))) return false;

    const float pressureScaled = static_cast<float>(int24(raw)) / SCALE_16X;
    const float temperatureScaled = static_cast<float>(int24(raw + 3)) / SCALE_1X;

    temperatureC = coef.c0 * 0.5f + coef.c1 * temperatureScaled;
    pressurePa = coef.c00
               + pressureScaled * (coef.c10 + pressureScaled * (coef.c20 + pressureScaled * coef.c30))
               + temperatureScaled * coef.c01
               + temperatureScaled * pressureScaled * (coef.c11 + pressureScaled * coef.c21);
    return true;
}

auto SPL06_Sensor::printName() const -> void
{
    Serial.print(getSensorType());
}

auto SPL06_Sensor::waitReady() -> bool
{
    for (uint8_t i = 0; i < READY_POLL_TRIES; ++i)
    {
        const int meas = device.readRegister(REG_MEAS_CFG);
        if (meas >= 0 && (meas & (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) == (MEAS_COEF_RDY | MEAS_SENSOR_RDY))
        {
            return true;
        }
        delay(5);
    }
    return false;
}

auto SPL06_Sensor::signExtend(uint32_t value, uint8_t bits) -> int32_t
{
    const uint32_t signBit = 1u << (bits - 1);
    return static_cast<int32_t>((value ^ signBit) - signBit);
}

auto SPL06_Sensor::int24(const uint8_t* p) -> int32_t
{
    return signExtend((static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2], 24);
}

auto SPL06_Sensor::readCoefficients() -> bool
{
    uint8_t c[18];
    if (!device.readRegisters(REG_COEF, c, sizeof(c))) return false;

    auto s16 = [&](int i) { return static_cast<float>(static_cast<int16_t>((c[i] << 8) | c[i + 1])); };

    coef.c0  = static_cast<float>(signExtend((static_cast<uint32_t>(c[0]) << 4) | (c[1] >> 4), 12));
    coef.c1  = static_cast<float>(signExtend((static_cast<uint32_t>(c[1] & 0x0F) << 8) | c[2], 12));
    coef.c00 = static_cast<float>(signExtend((static_cast<uint32_t>(c[3]) << 12) |
                                             (static_cast<uint32_t>(c[4]) << 4) | (c[5] >> 4), 20));
    coef.c10 = static_cast<float>(signExtend((static_cast<uint32_t>(c[5] & 0x0F) << 16) |
                                             (static_cast<uint32_t>(c[6]) << 8) | c[7], 20));
    coef.c01 = s16(8);
    coef.c11 = s16(10);
    coef.c20 = s16(12);
    coef.c21 = s16(14);
    coef.c30 = s16(16);
    return true;
}
