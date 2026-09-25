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

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin)
    {
        return SpiRegisterDevice(bus, chipSelectPin, 8000000, 1);
    }

    explicit BMP388_Sensor(IRegisterDevice& registerDevice)
        : BarometerBase("BMP388", POLL_PERIOD_US),
          device(registerDevice),
          calib()
    {
    }

    bool begin() override
    {
        device.begin();
        delay(5);

        const int chipId = device.readRegister(REG_CHIP_ID);
        if (chipId != CHIP_ID_VALUE)
        {
            Serial.print("BMP388: ");
            if (chipId < 0)
            {
                Serial.println("не отвечает (I2C: CSB -> VCC, SDO -> GND)");
            }
            else
            {
                Serial.print("неверный chip ID 0x");
                Serial.println(chipId, HEX);
            }
            setAvailable(false);
            return false;
        }

        device.writeRegister(REG_CMD, 0xB6);  // soft reset
        delay(10);

        if (!readCalibration())
        {
            Serial.println("BMP388: не удалось прочитать калибровку NVM");
            setAvailable(false);
            return false;
        }

        const bool ok =
            device.writeRegister(REG_OSR, 0x03) &&       // давление ×8 (011), температура ×1 (000)
            device.writeRegister(REG_ODR, 0x02) &&       // 50 Гц (odr_sel=2)
            device.writeRegister(REG_CONFIG, 0x04) &&    // IIR-фильтр, коэффициент 3
            device.writeRegister(REG_PWR_CTRL, 0x33);    // давление + температура, normal mode

        if (!ok)
        {
            Serial.println("BMP388: ошибка записи регистров");
            setAvailable(false);
            return false;
        }

        Serial.println("BMP388: подключён");
        setAvailable(true);
        return true;
    }


protected:

    bool isNewSampleReady(bool& ready) override
    {
        const int status = device.readRegister(REG_STATUS);
        if (status < 0) return false;

        ready = (status & STATUS_DRDY_PRESS) != 0;
        return true;
    }

    bool readSample(float& pressurePa, float& temperatureC) override
    {
        uint8_t raw[6];  // давление XLSB..MSB, температура XLSB..MSB
        if (!device.readRegisters(REG_DATA_0, raw, sizeof(raw))) return false;

        const uint32_t uncompPress = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
        const uint32_t uncompTemp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

        // Температура — первой: её промежуточный член tLin нужен для давления.
        const double tLin = compensateTemperature(uncompTemp);
        temperatureC = static_cast<float>(tLin);
        pressurePa = static_cast<float>(compensatePressure(uncompPress, tLin));
        return true;
    }


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

    bool readCalibration()
    {
        uint8_t r[21];
        if (!device.readRegisters(REG_NVM_PAR, r, sizeof(r))) return false;

        auto u16 = [&](int i) { return (double)(uint16_t)(r[i] | (r[i + 1] << 8)); };
        auto s16 = [&](int i) { return (double)(int16_t)(r[i] | (r[i + 1] << 8)); };
        auto s8  = [&](int i) { return (double)(int8_t)r[i]; };

        calib.t1  = u16(0) * 256.0;                              // / 2^-8
        calib.t2  = u16(2) / 1073741824.0;                       // / 2^30
        calib.t3  = s8(4) / 281474976710656.0;                   // / 2^48

        calib.p1  = (s16(5) - 16384.0) / 1048576.0;              // / 2^20
        calib.p2  = (s16(7) - 16384.0) / 536870912.0;            // / 2^29
        calib.p3  = s8(9) / 4294967296.0;                        // / 2^32
        calib.p4  = s8(10) / 137438953472.0;                     // / 2^37
        calib.p5  = u16(11) * 8.0;                               // / 2^-3
        calib.p6  = u16(13) / 64.0;                              // / 2^6
        calib.p7  = s8(15) / 256.0;                              // / 2^8
        calib.p8  = s8(16) / 32768.0;                            // / 2^15
        calib.p9  = s16(17) / 281474976710656.0;                 // / 2^48
        calib.p10 = s8(19) / 281474976710656.0;                  // / 2^48
        calib.p11 = s8(20) / 36893488147419103232.0;             // / 2^65
        return true;
    }

    // Датащит BMP388 §9.3 — возвращает tLin (это и есть температура, °C).
    double compensateTemperature(uint32_t uncompTemp) const
    {
        const double d1 = (double)uncompTemp - calib.t1;
        const double d2 = d1 * calib.t2;
        return d2 + d1 * d1 * calib.t3;
    }

    double compensatePressure(uint32_t uncompPress, double tLin) const
    {
        const double p = (double)uncompPress;
        const double t2 = tLin * tLin;
        const double t3 = t2 * tLin;

        const double out1 = calib.p5 + calib.p6 * tLin + calib.p7 * t2 + calib.p8 * t3;
        const double out2 = p * (calib.p1 + calib.p2 * tLin + calib.p3 * t2 + calib.p4 * t3);
        const double out3 = p * p * (calib.p9 + calib.p10 * tLin) + p * p * p * calib.p11;

        return out1 + out2 + out3;
    }
};
