// Реализация sensors/baro/BMP388_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/baro/BMP388_Sensor.h"


auto BMP388_Sensor::spiDevice(ISpiBus& bus, uint8_t chipSelectPin) -> SpiRegisterDevice
{
    return SpiRegisterDevice(bus, chipSelectPin, 8000000, 1);
}

BMP388_Sensor::BMP388_Sensor(IRegisterDevice& registerDevice)
: BarometerBase("BMP388", POLL_PERIOD_US),
      device(registerDevice),
      calib()
{
}

auto BMP388_Sensor::begin() -> bool
{
    device.begin();
    delay(5);

    const int chipId = device.readRegister(REG_CHIP_ID);
    if (chipId != CHIP_ID_VALUE)
    {
        Serial.print("BMP388: ");
        if (chipId < 0)
        {
            Serial.println("не отвечает (I2C: CSB -> VCC, SDO -> GND)");
        }
        else
        {
            Serial.print("неверный chip ID 0x");
            Serial.println(chipId, HEX);
        }
        setAvailable(false);
        return false;
    }

    device.writeRegister(REG_CMD, 0xB6);  // soft reset
    delay(10);

    if (!readCalibration())
    {
        Serial.println("BMP388: не удалось прочитать калибровку NVM");
        setAvailable(false);
        return false;
    }

    const bool ok =
        device.writeRegister(REG_OSR, 0x03) &&       // давление ×8 (011), температура ×1 (000)
        device.writeRegister(REG_ODR, 0x02) &&       // 50 Гц (odr_sel=2)
        device.writeRegister(REG_CONFIG, 0x04) &&    // IIR-фильтр, коэффициент 3
        device.writeRegister(REG_PWR_CTRL, 0x33);    // давление + температура, normal mode

    if (!ok)
    {
        Serial.println("BMP388: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    Serial.println("BMP388: подключён");
    setAvailable(true);
    return true;
}

auto BMP388_Sensor::isNewSampleReady(bool& ready) -> bool
{
    const int status = device.readRegister(REG_STATUS);
    if (status < 0) return false;

    ready = (status & STATUS_DRDY_PRESS) != 0;
    return true;
}

auto BMP388_Sensor::readSample(float& pressurePa, float& temperatureC) -> bool
{
    uint8_t raw[6];  // давление XLSB..MSB, температура XLSB..MSB
    if (!device.readRegisters(REG_DATA_0, raw, sizeof(raw))) return false;

    const uint32_t uncompPress = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
    const uint32_t uncompTemp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

    // Температура — первой: её промежуточный член tLin нужен для давления.
    const double tLin = compensateTemperature(uncompTemp);
    temperatureC = static_cast<float>(tLin);
    pressurePa = static_cast<float>(compensatePressure(uncompPress, tLin));
    return true;
}

auto BMP388_Sensor::readCalibration() -> bool
{
    uint8_t r[21];
    if (!device.readRegisters(REG_NVM_PAR, r, sizeof(r))) return false;

    auto u16 = [&](int i) { return (double)(uint16_t)(r[i] | (r[i + 1] << 8)); };
    auto s16 = [&](int i) { return (double)(int16_t)(r[i] | (r[i + 1] << 8)); };
    auto s8  = [&](int i) { return (double)(int8_t)r[i]; };

    calib.t1  = u16(0) * 256.0;                              // / 2^-8
    calib.t2  = u16(2) / 1073741824.0;                       // / 2^30
    calib.t3  = s8(4) / 281474976710656.0;                   // / 2^48

    calib.p1  = (s16(5) - 16384.0) / 1048576.0;              // / 2^20
    calib.p2  = (s16(7) - 16384.0) / 536870912.0;            // / 2^29
    calib.p3  = s8(9) / 4294967296.0;                        // / 2^32
    calib.p4  = s8(10) / 137438953472.0;                     // / 2^37
    calib.p5  = u16(11) * 8.0;                               // / 2^-3
    calib.p6  = u16(13) / 64.0;                              // / 2^6
    calib.p7  = s8(15) / 256.0;                              // / 2^8
    calib.p8  = s8(16) / 32768.0;                            // / 2^15
    calib.p9  = s16(17) / 281474976710656.0;                 // / 2^48
    calib.p10 = s8(19) / 281474976710656.0;                  // / 2^48
    calib.p11 = s8(20) / 36893488147419103232.0;             // / 2^65
    return true;
}

auto BMP388_Sensor::compensateTemperature(uint32_t uncompTemp) const -> double
{
    const double d1 = (double)uncompTemp - calib.t1;
    const double d2 = d1 * calib.t2;
    return d2 + d1 * d1 * calib.t3;
}

auto BMP388_Sensor::compensatePressure(uint32_t uncompPress, double tLin) const -> double
{
    const double p = (double)uncompPress;
    const double t2 = tLin * tLin;
    const double t3 = t2 * tLin;

    const double out1 = calib.p5 + calib.p6 * tLin + calib.p7 * t2 + calib.p8 * t3;
    const double out2 = p * (calib.p1 + calib.p2 * tLin + calib.p3 * t2 + calib.p4 * t3);
    const double out3 = p * p * (calib.p9 + calib.p10 * tLin) + p * p * p * calib.p11;

    return out1 + out2 + out3;
}
