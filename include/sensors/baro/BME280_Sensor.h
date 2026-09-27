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

    explicit BME280_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool isNewSampleReady(bool& ready) override;

    bool readSample(float& pressurePa, float& temperatureC) override;


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

    bool readCalibration();

    // Датащит BME280 §8.1 (double). tFine нужен для давления.
    double compensateTemperature(int32_t adcT, double& tFine) const;

    double compensatePressure(int32_t adcP, double tFine) const;
};
