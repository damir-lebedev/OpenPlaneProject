#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/baro/BarometerBase.h"

// ============================================================
// BMP388 (Bosch) — барометр/термометр, I2C или SPI
//
// Один драйвер для обеих шин (IRegisterDevice):
//   • I2C — I2cRegisterDevice, адрес 0x76 (SDO=GND) или 0x77
//     (SDO=VDD). CSB модуля — на VCC: если при включении он на
//     земле, чип уходит в SPI и по I2C не отвечает;
//   • SPI — BMP388_Sensor::spiDevice(): в режиме SPI чип перед
//     данными отдаёт один фиктивный байт (датащит §5.3.2). Прежний
//     SPI-драйвер его не пропускал и читал всё со сдвигом — даже
//     chip ID не проходил проверку.
//
// Компенсация — формула Bosch с плавающей точкой (датащит §9.3),
// 21 байт NVM-коэффициентов с регистра 0x31. Высота, вертикальная
// скорость, калибровка базы — в BarometerBase.
//
// Режим: давление ×8, температура ×1 (~20 мс на измерение), ODR
// 50 Гц, IIR-фильтр 3; новый отсчёт — по флагу drdy_press в STATUS.
// ============================================================

class BMP388_Sensor : public BarometerBase
{
public:

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin);

    explicit BMP388_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool isNewSampleReady(bool& ready) override;

    bool readSample(float& pressurePa, float& temperatureC) override;


private:

    static constexpr uint8_t REG_CHIP_ID  = 0x00;
    static constexpr uint8_t REG_STATUS   = 0x03;
    static constexpr uint8_t REG_DATA_0   = 0x04;
    static constexpr uint8_t REG_PWR_CTRL = 0x1B;
    static constexpr uint8_t REG_OSR      = 0x1C;
    static constexpr uint8_t REG_ODR      = 0x1D;
    static constexpr uint8_t REG_CONFIG   = 0x1F;
    static constexpr uint8_t REG_NVM_PAR  = 0x31;  // 21 байт калибровки
    static constexpr uint8_t REG_CMD      = 0x7E;

    static constexpr int CHIP_ID_VALUE = 0x50;
    static constexpr uint8_t STATUS_DRDY_PRESS = 0x20;

    // Опрос флага готовности в 4 раза чаще ODR — отсчёт читается не
    // позже чем через 5 мс после измерения.
    static constexpr uint32_t POLL_PERIOD_US = 5000;

    IRegisterDevice& device;

    // Коэффициенты уже с масштабами из датащита (§9.1).
    struct
    {
        double t1, t2, t3;
        double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    } calib;

    bool readCalibration();

    // Датащит BMP388 §9.3 — возвращает tLin (это и есть температура, °C).
    double compensateTemperature(uint32_t uncompTemp) const;

    double compensatePressure(uint32_t uncompPress, double tLin) const;
};
