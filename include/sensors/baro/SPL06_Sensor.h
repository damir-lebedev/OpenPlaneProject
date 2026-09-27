#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/baro/BarometerBase.h"

// ============================================================
// SPL06-001 (Goertek) — барометр/термометр, I2C или SPI
//
// I2C: адрес 0x76 (SDO = GND) или 0x77 (SDO = VDD) — у модулей
// бывает по-разному, I2cRegisterDevice с запасным адресом находит
// любой. SPI — SPL06_Sensor::spiDevice() (режим 0, без лишних байт).
//
// Регистры сверены с датащитом и драйвером ArduPilot
// (AP_Baro_SPL06.cpp):
//   0x00-0x02 PSR_B2..B0, 0x03-0x05 TMP_B2..B0 — int24 big-endian
//   0x06 PRS_CFG  [6:4] PM_RATE, [3:0] PM_PRC (оверсэмплинг 2^n)
//   0x07 TMP_CFG  [7] TMP_EXT, [6:4] TMP_RATE, [3:0] TMP_PRC
//   0x08 MEAS_CFG [7] COEF_RDY, [6] SENSOR_RDY, [5] TMP_RDY,
//                 [4] PRS_RDY, [2:0] MEAS_CTRL (7 = непрерывно P+T)
//   0x09 CFG_REG  [3] T_SHIFT, [2] P_SHIFT (обязательны при ×16 и выше)
//   0x0C RESET = 0x09 — мягкий сброс
//   0x0D ID = 0x10
//   0x10-0x21 18 байт коэффициентов c0, c1, c00, c10, c01, c11, c20, c21, c30
//   0x28 COEF_SRCE [7] — каким датчиком температуры калиброван чип
//
// Компенсация (датащит, §4.9):
//   Praw_sc = Praw / kP, Traw_sc = Traw / kT
//   T = c0/2 + c1·Traw_sc
//   P = c00 + Praw_sc·(c10 + Praw_sc·(c20 + Praw_sc·c30))
//       + Traw_sc·c01 + Traw_sc·Praw_sc·(c11 + Praw_sc·c21)
//   kP/kT зависят от оверсэмплинга (таблица "Compensation Scale Factors").
//
// Режим: давление 32 изм/с × 16 (27.6 мс на измерение — 0.88 с из
// каждой секунды), температура 4 изм/с × 1. Сумма времени измерений
// должна быть меньше секунды — иначе чип не успевает. Новый отсчёт —
// по флагу PRS_RDY.
// ============================================================

class SPL06_Sensor : public BarometerBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS   = 0x76;  // SDO = GND
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x77;  // SDO = VDD

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin)
    {
        return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
    }

    explicit SPL06_Sensor(IRegisterDevice& registerDevice, const char* sensorName = "SPL06")
        : BarometerBase(sensorName, POLL_PERIOD_US),
          device(registerDevice),
          coef()
    {
    }

    bool begin() override
    {
        device.begin();
        delay(5);

        const int id = device.readRegister(REG_ID);
        if (id != ID_VALUE)
        {
            printName();
            if (id < 0) Serial.println(": не отвечает");
            else if (id == ID_SPA06) Serial.println(": это SPA06 (ID 0x11) — другие коэффициенты, не поддержан");
            else { Serial.print(": неверный ID 0x"); Serial.println(id, HEX); }
            setAvailable(false);
            return false;
        }

        device.writeRegister(REG_RESET, RESET_SOFT);
        delay(RESET_TIME_MS);  // коэффициенты доступны через 40 мс после сброса

        if (!waitReady() || !readCoefficients())
        {
            printName();
            Serial.println(": не удалось прочитать коэффициенты");
            setAvailable(false);
            return false;
        }

        // Температура — тем датчиком, которым чип калибровали на заводе.
        const int source = device.readRegister(REG_COEF_SRCE);
        const uint8_t tmpExt = (source < 0 || (source & 0x80)) ? TMP_EXT : 0;

        const bool ok =
            device.writeRegister(REG_PRS_CFG, (RATE_32 << 4) | PRC_16) &&
            device.writeRegister(REG_TMP_CFG, tmpExt | (RATE_4 << 4) | PRC_1) &&
            device.writeRegister(REG_CFG, CFG_P_SHIFT) &&
            device.writeRegister(REG_MEAS_CFG, MEAS_CONTINUOUS_PT);

        if (!ok)
        {
            printName();
            Serial.println(": ошибка записи регистров");
            setAvailable(false);
            return false;
        }

        printName();
        Serial.println(": подключён");
        setAvailable(true);
        return true;
    }


protected:

    bool isNewSampleReady(bool& ready) override
    {
        const int meas = device.readRegister(REG_MEAS_CFG);
        if (meas < 0) return false;

        ready = (meas & MEAS_PRS_RDY) != 0;
        return true;
    }

    bool readSample(float& pressurePa, float& temperatureC) override
    {
        uint8_t raw[6];  // PSR_B2..B0, TMP_B2..B0
        if (!device.readRegisters(REG_PSR_B2, raw, sizeof(raw))) return false;

        const float pressureScaled = static_cast<float>(int24(raw)) / SCALE_16X;
        const float temperatureScaled = static_cast<float>(int24(raw + 3)) / SCALE_1X;

        temperatureC = coef.c0 * 0.5f + coef.c1 * temperatureScaled;
        pressurePa = coef.c00
                   + pressureScaled * (coef.c10 + pressureScaled * (coef.c20 + pressureScaled * coef.c30))
                   + temperatureScaled * coef.c01
                   + temperatureScaled * pressureScaled * (coef.c11 + pressureScaled * coef.c21);
        return true;
    }


private:

    static constexpr uint8_t REG_PSR_B2    = 0x00;
    static constexpr uint8_t REG_PRS_CFG   = 0x06;
    static constexpr uint8_t REG_TMP_CFG   = 0x07;
    static constexpr uint8_t REG_MEAS_CFG  = 0x08;
    static constexpr uint8_t REG_CFG       = 0x09;
    static constexpr uint8_t REG_RESET     = 0x0C;
    static constexpr uint8_t REG_ID        = 0x0D;
    static constexpr uint8_t REG_COEF      = 0x10;
    static constexpr uint8_t REG_COEF_SRCE = 0x28;

    static constexpr int ID_VALUE = 0x10;
    static constexpr int ID_SPA06 = 0x11;

    static constexpr uint8_t RESET_SOFT = 0x09;
    static constexpr uint8_t TMP_EXT = 0x80;
    static constexpr uint8_t RATE_4  = 0x02;
    static constexpr uint8_t RATE_32 = 0x05;
    static constexpr uint8_t PRC_1   = 0x00;
    static constexpr uint8_t PRC_16  = 0x04;
    static constexpr uint8_t CFG_P_SHIFT = 0x04;
    static constexpr uint8_t MEAS_CONTINUOUS_PT = 0x07;
    static constexpr uint8_t MEAS_COEF_RDY   = 0x80;
    static constexpr uint8_t MEAS_SENSOR_RDY = 0x40;
    static constexpr uint8_t MEAS_PRS_RDY    = 0x10;

    // Таблица "Compensation Scale Factors": ×1 -> 524288, ×16 -> 253952.
    static constexpr float SCALE_1X  = 524288.0f;
    static constexpr float SCALE_16X = 253952.0f;

    static constexpr uint32_t RESET_TIME_MS = 40;
    static constexpr uint8_t READY_POLL_TRIES = 20;

    // Новый отсчёт раз в ~31 мс; флаг опрашиваем раз в 5 мс.
    static constexpr uint32_t POLL_PERIOD_US = 5000;

    IRegisterDevice& device;

    struct
    {
        float c0, c1, c00, c10, c01, c11, c20, c21, c30;
    } coef;

    void printName() const
    {
        Serial.print(getSensorType());
    }

    bool waitReady()
    {
        for (uint8_t i = 0; i < READY_POLL_TRIES; ++i)
        {
            const int meas = device.readRegister(REG_MEAS_CFG);
            if (meas >= 0 && (meas & (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) == (MEAS_COEF_RDY | MEAS_SENSOR_RDY))
            {
                return true;
            }
            delay(5);
        }
        return false;
    }

    // Знаковое число из bits младших бит.
    static int32_t signExtend(uint32_t value, uint8_t bits)
    {
        const uint32_t signBit = 1u << (bits - 1);
        return static_cast<int32_t>((value ^ signBit) - signBit);
    }

    static int32_t int24(const uint8_t* p)
    {
        return signExtend((static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2], 24);
    }

    bool readCoefficients()
    {
        uint8_t c[18];
        if (!device.readRegisters(REG_COEF, c, sizeof(c))) return false;

        auto s16 = [&](int i) { return static_cast<float>(static_cast<int16_t>((c[i] << 8) | c[i + 1])); };

        coef.c0  = static_cast<float>(signExtend((static_cast<uint32_t>(c[0]) << 4) | (c[1] >> 4), 12));
        coef.c1  = static_cast<float>(signExtend((static_cast<uint32_t>(c[1] & 0x0F) << 8) | c[2], 12));
        coef.c00 = static_cast<float>(signExtend((static_cast<uint32_t>(c[3]) << 12) |
                                                 (static_cast<uint32_t>(c[4]) << 4) | (c[5] >> 4), 20));
        coef.c10 = static_cast<float>(signExtend((static_cast<uint32_t>(c[5] & 0x0F) << 16) |
                                                 (static_cast<uint32_t>(c[6]) << 8) | c[7], 20));
        coef.c01 = s16(8);
        coef.c11 = s16(10);
        coef.c20 = s16(12);
        coef.c21 = s16(14);
        coef.c30 = s16(16);
        return true;
    }
};
