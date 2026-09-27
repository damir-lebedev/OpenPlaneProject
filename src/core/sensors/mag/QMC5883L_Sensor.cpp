// Реализация sensors/mag/QMC5883L_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/mag/QMC5883L_Sensor.h"


QMC5883L_Sensor::QMC5883L_Sensor(IRegisterDevice& registerDevice)
: MagnetometerBase("QMC5883L", "qmc5883l"),
      device(registerDevice)
{
}

auto QMC5883L_Sensor::begin() -> bool
{
    device.begin();

    if (!device.probe())
    {
        Serial.println("QMC5883L: не отвечает, компас недоступен");
        setAvailable(false);
        return false;
    }

    // SET/RESET period = 0x01 (датащит); MODE=continuous (01),
    // ODR=200 Гц (11), RNG=±8 Гс (01), OSR=512 (00) -> 0x1D.
    const bool ok =
        device.writeRegister(REG_SET_RESET, 0x01) &&
        device.writeRegister(REG_CONTROL1, 0x1D);

    if (!ok)
    {
        Serial.println("QMC5883L: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    setAvailable(true);
    return true;
}

auto QMC5883L_Sensor::readRaw(int16_t raw[3]) -> bool
{
    uint8_t b[6];
    if (!device.readRegisters(REG_DATA_X_LSB, b, sizeof(b))) return false;

    raw[0] = (int16_t)((b[1] << 8) | b[0]);
    raw[1] = (int16_t)((b[3] << 8) | b[2]);
    raw[2] = (int16_t)((b[5] << 8) | b[4]);
    return true;
}
