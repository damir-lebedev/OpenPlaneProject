#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/mag/MagnetometerBase.h"

// ============================================================
// QMC5883P (плата GY-273, маркировка чипа "5883P"/"HP5883") — компас
//
// I2C, адрес 0x2C. Раскладка регистров сверена с датащитом и
// проверена вживую на GY-273 (chip ID 0x80). Она НЕ совпадает с
// QMC5883L (адрес 0x0D, данные с 0x00, управление в 0x09) — драйверы
// не взаимозаменяемы:
//   0x00      CHIP_ID = 0x80
//   0x01-0x06 X/Y/Z, int16 little-endian
//   0x0A      CONTROL1: [1:0] MODE, [3:2] ODR, [5:4] OSR1, [7:6] OSR2
//   0x0B      CONTROL2: [1:0] SET/RESET, [3:2] RNG, [7] SOFT_RST
//   0x29      знаки осей (датащит рекомендует 0x06)
//
// Калибровка, курс, опрос, ошибки — в MagnetometerBase.
// ============================================================

class QMC5883P_Sensor : public MagnetometerBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS = 0x2C;

    explicit QMC5883P_Sensor(IRegisterDevice& registerDevice)
        : MagnetometerBase("QMC5883P", "qmc5883p"),
          device(registerDevice)
    {
    }

    bool begin() override
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


protected:

    bool readRaw(int16_t raw[3]) override
    {
        uint8_t b[6];
        if (!device.readRegisters(REG_DATA_X_LSB, b, sizeof(b))) return false;

        raw[0] = (int16_t)((b[1] << 8) | b[0]);
        raw[1] = (int16_t)((b[3] << 8) | b[2]);
        raw[2] = (int16_t)((b[5] << 8) | b[4]);
        return true;
    }

    // ±8 Гс: 3750 LSB/Гс, 1 Гс = 100 мкТл.
    float lsbPerMicroTesla() const override { return 37.5f; }


private:

    static constexpr uint8_t REG_CHIP_ID    = 0x00;
    static constexpr uint8_t REG_DATA_X_LSB = 0x01;
    static constexpr uint8_t REG_CONTROL1   = 0x0A;
    static constexpr uint8_t REG_CONTROL2   = 0x0B;
    static constexpr uint8_t REG_AXIS_SIGN  = 0x29;
    static constexpr int CHIP_ID_VALUE = 0x80;

    IRegisterDevice& device;
};
