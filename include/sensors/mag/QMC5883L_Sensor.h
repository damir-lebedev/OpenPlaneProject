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

    explicit QMC5883L_Sensor(IRegisterDevice& registerDevice)
        : MagnetometerBase("QMC5883L", "qmc5883l"),
          device(registerDevice)
    {
    }

    bool begin() override
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

    // ±8 Гс: 3000 LSB/Гс = 30 LSB/мкТл.
    float lsbPerMicroTesla() const override { return 30.0f; }


private:

    static constexpr uint8_t REG_DATA_X_LSB = 0x00;
    static constexpr uint8_t REG_CONTROL1   = 0x09;
    static constexpr uint8_t REG_SET_RESET  = 0x0B;

    IRegisterDevice& device;
};
