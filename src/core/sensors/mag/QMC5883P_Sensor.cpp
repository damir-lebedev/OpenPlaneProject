// Реализация sensors/mag/QMC5883P_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/mag/QMC5883P_Sensor.h"


QMC5883P_Sensor::QMC5883P_Sensor(IRegisterDevice& registerDevice)
: MagnetometerBase("QMC5883P", "qmc5883p"),
      device(registerDevice)
{
}

auto QMC5883P_Sensor::begin() -> bool
{
    device.begin();

    const int chipId = device.readRegister(REG_CHIP_ID);
    if (chipId != CHIP_ID_VALUE)
    {
        Serial.print("QMC5883P: не отвечает (chip ID ");
        Serial.print(chipId < 0 ? String("нет ответа") : String("0x") + String(chipId, HEX));
        Serial.println("), компас недоступен");
        setAvailable(false);
        return false;
    }

    device.writeRegister(REG_CONTROL2, 0x80);  // SOFT_RST
    delay(10);

    // По примеру из датащита: знаки осей; SET/RESET включён,
    // диапазон ±8 Гс; режим normal, ODR 200 Гц, OSR1=8, OSR2=8.
    const bool ok =
        device.writeRegister(REG_AXIS_SIGN, 0x06) &&
        device.writeRegister(REG_CONTROL2, 0x08) &&
        device.writeRegister(REG_CONTROL1, 0xCD);

    if (!ok)
    {
        Serial.println("QMC5883P: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    setAvailable(true);
    return true;
}

auto QMC5883P_Sensor::readRaw(int16_t raw[3]) -> bool
{
    uint8_t b[6];
    if (!device.readRegisters(REG_DATA_X_LSB, b, sizeof(b))) return false;

    raw[0] = (int16_t)((b[1] << 8) | b[0]);
    raw[1] = (int16_t)((b[3] << 8) | b[2]);
    raw[2] = (int16_t)((b[5] << 8) | b[4]);
    return true;
}
