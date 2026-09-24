#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/baro/BarometerBase.h"

// ============================================================
// BME280 / BMP280 (Bosch) — барометр/термометр, I2C или SPI
//
// I2C — адрес 0x76 (SDO=GND) или 0x77 (SDO=VDD); по SPI фиктивного
// байта нет (в отличие от BMP388). Влажность BME280 не читается —
// автопилоту она не нужна, поэтому BMP280 (chip ID 0x58) работает
// тем же драйвером.
//
// Компенсация — формулы Bosch с плавающей точкой (датащит BME280
// §8.1, коэффициенты dig_T1..dig_P9 с регистра 0x88). Раньше здесь
// стояла "аппроксимация" P = 100000 + (adc − 100000)/1000 без
// коэффициентов — давление и высота были бессмысленными числами.
//
// Режим: температура ×2, давление ×8, IIR 4, normal mode, пауза
// 0.5 мс — измерение ~24 мс. Флага "новый отсчёт" у чипа нет,
// поэтому опрос раз в 25 мс.
// ============================================================

class BME280_Sensor : public BarometerBase
{
public:

    explicit BME280_Sensor(IRegisterDevice& device)
        : BarometerBase("BME280", POLL_PERIOD_US),
          device(device)
    {
        memset(&calib, 0, sizeof(calib));
    }

    bool begin() override
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


protected:

    bool isNewSampleReady(bool& ready) override
    {
        ready = true;  // флага нет — период опроса не короче измерения
        return true;
    }

    bool readSample(float& pressurePa, float& temperatureC) override
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


private:

    static constexpr uint8_t REG_CALIB_00  = 0x88;  // dig_T1..dig_P9, 24 байта
    static constexpr uint8_t REG_CHIP_ID   = 0xD0;
    static constexpr uint8_t REG_RESET     = 0xE0;
    static constexpr uint8_t REG_CTRL_HUM  = 0xF2;
    static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t REG_CONFIG    = 0xF5;
    static constexpr uint8_t REG_PRESS_MSB = 0xF7;

    static constexpr int CHIP_ID_BME280 = 0x60;
    static constexpr int CHIP_ID_BMP280 = 0x58;

    static constexpr uint32_t POLL_PERIOD_US = 25000;

    IRegisterDevice& device;

    struct
    {
        double t1, t2, t3;
        double p1, p2, p3, p4, p5, p6, p7, p8, p9;
    } calib;

    bool readCalibration()
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

    // Датащит BME280 §8.1 (double). tFine нужен для давления.
    double compensateTemperature(int32_t adcT, double& tFine) const
    {
        const double v1 = (adcT / 16384.0 - calib.t1 / 1024.0) * calib.t2;
        const double d  = adcT / 131072.0 - calib.t1 / 8192.0;
        const double v2 = d * d * calib.t3;
        tFine = v1 + v2;
        return tFine / 5120.0;
    }

    double compensatePressure(int32_t adcP, double tFine) const
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
};
