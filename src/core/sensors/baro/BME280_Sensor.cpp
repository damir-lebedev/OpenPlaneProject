// Реализация sensors/baro/BME280_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/baro/BME280_Sensor.h"


BME280_Sensor::BME280_Sensor(IRegisterDevice& registerDevice)
: BarometerBase("BME280", POLL_PERIOD_US),
      device(registerDevice),
      calib()
{
}

auto BME280_Sensor::begin() -> bool
{
    device.begin();
    delay(5);

    const int chipId = device.readRegister(REG_CHIP_ID);
    if (chipId != CHIP_ID_BME280 && chipId != CHIP_ID_BMP280)
    {
        Serial.print("BME280: ");
        if (chipId < 0)
        {
            Serial.println("не отвечает");
        }
        else
        {
            Serial.print("неверный chip ID 0x");
            Serial.println(chipId, HEX);
        }
        setAvailable(false);
        return false;
    }

    device.writeRegister(REG_RESET, 0xB6);
    delay(5);

    if (!readCalibration())
    {
        Serial.println("BME280: не удалось прочитать калибровку");
        setAvailable(false);
        return false;
    }

    // CTRL_HUM применяется только после записи CTRL_MEAS, поэтому
    // пишется первым (раньше порядок был обратный).
    bool ok = true;
    if (chipId == CHIP_ID_BME280)
    {
        ok &= device.writeRegister(REG_CTRL_HUM, 0x00);  // влажность не измеряем
    }
    ok &= device.writeRegister(REG_CONFIG, 0x08);        // пауза 0.5 мс, IIR 4
    ok &= device.writeRegister(REG_CTRL_MEAS, 0x53);     // T ×2 (010), P ×8 (100), normal (11)

    if (!ok)
    {
        Serial.println("BME280: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    Serial.println(chipId == CHIP_ID_BME280 ? "BME280: подключён" : "BME280: подключён (BMP280)");
    setAvailable(true);
    return true;
}

auto BME280_Sensor::isNewSampleReady(bool& ready) -> bool
{
    ready = true;  // флага нет — период опроса не короче измерения
    return true;
}

auto BME280_Sensor::readSample(float& pressurePa, float& temperatureC) -> bool
{
    uint8_t r[6];  // press MSB/LSB/XLSB, temp MSB/LSB/XLSB
    if (!device.readRegisters(REG_PRESS_MSB, r, sizeof(r))) return false;

    const int32_t adcP = ((int32_t)r[0] << 12) | ((int32_t)r[1] << 4) | (r[2] >> 4);
    const int32_t adcT = ((int32_t)r[3] << 12) | ((int32_t)r[4] << 4) | (r[5] >> 4);

    double tFine;
    temperatureC = static_cast<float>(compensateTemperature(adcT, tFine));
    pressurePa = static_cast<float>(compensatePressure(adcP, tFine));
    return true;
}

auto BME280_Sensor::readCalibration() -> bool
{
    uint8_t r[24];
    if (!device.readRegisters(REG_CALIB_00, r, sizeof(r))) return false;

    auto u16 = [&](int i) { return (double)(uint16_t)(r[i] | (r[i + 1] << 8)); };
    auto s16 = [&](int i) { return (double)(int16_t)(r[i] | (r[i + 1] << 8)); };

    calib.t1 = u16(0);  calib.t2 = s16(2);  calib.t3 = s16(4);
    calib.p1 = u16(6);  calib.p2 = s16(8);  calib.p3 = s16(10);
    calib.p4 = s16(12); calib.p5 = s16(14); calib.p6 = s16(16);
    calib.p7 = s16(18); calib.p8 = s16(20); calib.p9 = s16(22);
    return true;
}

auto BME280_Sensor::compensateTemperature(int32_t adcT, double& tFine) const -> double
{
    const double v1 = (adcT / 16384.0 - calib.t1 / 1024.0) * calib.t2;
    const double d  = adcT / 131072.0 - calib.t1 / 8192.0;
    const double v2 = d * d * calib.t3;
    tFine = v1 + v2;
    return tFine / 5120.0;
}

auto BME280_Sensor::compensatePressure(int32_t adcP, double tFine) const -> double
{
    double v1 = tFine / 2.0 - 64000.0;
    double v2 = v1 * v1 * calib.p6 / 32768.0;
    v2 = v2 + v1 * calib.p5 * 2.0;
    v2 = v2 / 4.0 + calib.p4 * 65536.0;
    v1 = (calib.p3 * v1 * v1 / 524288.0 + calib.p2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * calib.p1;

    if (v1 == 0.0) return 0.0;  // защита от деления на ноль (датащит)

    double p = 1048576.0 - adcP;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = calib.p9 * p * p / 2147483648.0;
    v2 = p * calib.p8 / 32768.0;
    return p + (v1 + v2 + calib.p7) / 16.0;
}
