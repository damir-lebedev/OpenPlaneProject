#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/mag/MagnetometerBase.h"

// ============================================================
// QMC5883L (плата GY-273) — компас
//
// I2C, адрес 0x0D (фиксированный). Раскладка по датащиту QMC5883L:
//   0x00-0x05 X/Y/Z, int16 little-endian
//   0x09      CONTROL1: [1:0] MODE, [3:2] ODR, [5:4] RNG, [7:6] OSR
//   0x0B      SET/RESET PERIOD — датащит требует записать 0x01
//   0x0D      CHIP_ID = 0xFF
//
// На части плат "GY-273" стоит QMC5883P с другими регистрами (см.
// QMC5883P_Sensor.h) — какой чип на вашей, видно по адресу при
// сканировании шины.
//
// Калибровка, курс, опрос, ошибки — в MagnetometerBase.
// ============================================================

class QMC5883L_Sensor : public MagnetometerBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS = 0x0D;

    explicit QMC5883L_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readRaw(int16_t raw[3]) override;

    // ±8 Гс: 3000 LSB/Гс = 30 LSB/мкТл.
    float lsbPerMicroTesla() const override { return 30.0f; }


private:

    static constexpr uint8_t REG_DATA_X_LSB = 0x00;
    static constexpr uint8_t REG_CONTROL1   = 0x09;
    static constexpr uint8_t REG_SET_RESET  = 0x0B;

    IRegisterDevice& device;
};
