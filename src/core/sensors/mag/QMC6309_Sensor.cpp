// Реализация sensors/mag/QMC6309_Sensor.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/mag/QMC6309_Sensor.h"


QMC6309_Sensor::QMC6309_Sensor(IRegisterDevice& registerDevice)
: MagnetometerBase("QMC6309", "qmc6309"),
      device(registerDevice)
{
}

auto QMC6309_Sensor::begin() -> bool
{
    device.begin();

    const int chipId = device.readRegister(REG_CHIP_ID);
    if (chipId != CHIP_ID_VALUE)
    {
        Serial.print("QMC6309: не отвечает (chip ID ");
        Serial.print(chipId < 0 ? String("нет ответа") : String("0x") + String(chipId, HEX));
        Serial.println("), компас недоступен");
        setAvailable(false);
        return false;
    }

    device.writeRegister(REG_CONTROL2, CTRL2_SOFT_RST);
    device.writeRegister(REG_CONTROL2, 0x00);

    if (!waitNvmReady())
    {
        Serial.println("QMC6309: NVM не загрузилась после сброса");
        setAvailable(false);
        return false;
    }

    const bool ok =
        device.writeRegister(REG_CONTROL2, (ODR_200HZ << 4) | (RANGE_8G << 2) | SET_RESET_ON) &&
        device.writeRegister(REG_CONTROL1, (LPF_DEPTH_16 << 5) | (OSR_8 << 3) | MODE_NORMAL);

    if (!ok)
    {
        Serial.println("QMC6309: ошибка записи регистров");
        setAvailable(false);
        return false;
    }

    setAvailable(true);
    return true;
}

auto QMC6309_Sensor::readRaw(int16_t raw[3]) -> bool
{
    uint8_t b[6];
    if (!device.readRegisters(REG_DATA_X_LSB, b, sizeof(b))) return false;

    raw[0] = static_cast<int16_t>((b[1] << 8) | b[0]);
    raw[1] = static_cast<int16_t>((b[3] << 8) | b[2]);
    raw[2] = static_cast<int16_t>((b[5] << 8) | b[4]);
    return true;
}

auto QMC6309_Sensor::waitNvmReady() -> bool
{
    for (uint8_t i = 0; i < NVM_POLL_TRIES; ++i)
    {
        const int status = device.readRegister(REG_STATUS);
        if (status >= 0 && (status & STATUS_NVM_READY) == STATUS_NVM_READY) return true;
        delay(1);
    }
    return false;
}
