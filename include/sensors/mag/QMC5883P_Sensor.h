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

    explicit QMC5883P_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readRaw(int16_t raw[3]) override;

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
