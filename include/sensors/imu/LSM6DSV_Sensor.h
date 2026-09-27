#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST) — гироскоп + акселерометр
//
// Модуль "LSM6DSV + QMC6309" — I2C, адрес 0x6A (SA0 = GND) или 0x6B
// (SA0 = VDD); I2cRegisterDevice с запасным адресом находит любой.
// По SPI — LSM6DSV_Sensor::spiDevice() (режим 0, без лишних байт).
//
// Регистры сверены с драйвером ST (STMicroelectronics/lsm6dsv-pid,
// lsm6dsv_reg.h) и с драйвером ArduPilot:
//   0x0F WHO_AM_I = 0x70 (у 16X и 32X одинаковый)
//   0x10 CTRL1  [3:0] ODR_XL, [6:4] OP_MODE_XL (0 — high performance)
//   0x11 CTRL2  [3:0] ODR_G,  [6:4] OP_MODE_G
//   0x12 CTRL3  [0] SW_RESET, [2] IF_INC, [6] BDU
//   0x15 CTRL6  [3:0] FS_G (4 = ±2000 °/с), [6:4] LPF1_G_BW
//   0x16 CTRL7  [0] LPF1_G_EN
//   0x17 CTRL8  [1:0] FS_XL, [2] признак 32X, [7:5] HP_LPF2_XL_BW
//   0x18 CTRL9  [3] LPF2_XL_EN
//   0x20..0x2D  температура, gyro XYZ, accel XYZ — int16 little-endian
//
// 16X и 32X различаются бит 2 в CTRL8 после сброса (у 32X он 1):
// у 32X коды диапазона акселерометра сдвинуты (00 = ±4g ... 11 =
// ±32g), и тот же код, что даёт ±16g у 16X, дал бы у 32X ±32g —
// ускорения читались бы вдвое меньше. Поэтому вариант определяется
// и код диапазона выбирается под него (так же делает ArduPilot).
//
// Настройка: ±2000 °/с и ±16g (как у остальных IMU проекта), ODR
// 960 Гц в high-performance, ФНЧ гироскопа LPF1 и акселерометра LPF2
// (ODR/20 ≈ 48 Гц) — полётный цикл читает IMU на 500 Гц, частоты выше
// половины этой частоты надо срезать в самом чипе.
//
// Калибровка, поворот осей, знаки и фильтр ориентации — в
// ImuSensorBase. Оси чипа стандартные: X, Y в плоскости, Z вверх.
//
// Не проверен на железе: при подключении — WHO_AM_I и вариант в
// логе, знаки наклоном (нос вверх -> P > 0, правое крыло вниз -> R > 0).
// ============================================================

class LSM6DSV_Sensor : public ImuSensorBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS   = 0x6A;  // SA0 = GND
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x6B;  // SA0 = VDD

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin)
    {
        return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
    }

    explicit LSM6DSV_Sensor(IRegisterDevice& registerDevice)
        : ImuSensorBase("LSM6DSV", "imu_lsm6dsv"),
          device(registerDevice)
    {
    }

    bool begin() override
    {
        device.begin();
        delay(BOOT_TIME_MS);

        const int whoAmI = device.readRegister(REG_WHO_AM_I);
        if (whoAmI != WHO_AM_I_VALUE)
        {
            Serial.print("LSM6DSV: неверный WHO_AM_I ");
            Serial.println(whoAmI < 0 ? String("(нет ответа)") : String("0x") + String(whoAmI, HEX));
            return fail();
        }

        if (!softReset())
        {
            Serial.println("LSM6DSV: чип не вышел из сброса");
            return fail();
        }

        const int ctrl8 = device.readRegister(REG_CTRL8);
        if (ctrl8 < 0) return fail();
        is32x = (ctrl8 & CTRL8_VARIANT_32X) != 0;

        const uint8_t fsXl = is32x ? FS_XL_16G_ON_32X : FS_XL_16G_ON_16X;

        const bool ok =
            device.writeRegister(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC) &&
            device.writeRegister(REG_CTRL6, FS_G_2000DPS | (GYRO_LPF1_BW << 4)) &&
            device.writeRegister(REG_CTRL7, CTRL7_LPF1_G_EN) &&
            device.writeRegister(REG_CTRL8, fsXl | (ACCEL_LPF2_BW << 5)) &&
            device.writeRegister(REG_CTRL9, CTRL9_LPF2_XL_EN) &&
            device.writeRegister(REG_CTRL1, ODR_960HZ) &&   // OP_MODE = 0: high performance
            device.writeRegister(REG_CTRL2, ODR_960HZ);

        if (!ok)
        {
            Serial.println("LSM6DSV: ошибка записи регистров");
            return fail();
        }

        Serial.print("LSM6DSV: подключён (");
        Serial.print(is32x ? "LSM6DSV32X" : "LSM6DSV/16X");
        Serial.println(")");
        setAvailable(true);
        return true;
    }

    bool isVariant32x() const { return is32x; }


protected:

    bool readSample(RawImuSample& s) override
    {
        uint8_t b[14];  // температура, gyro XYZ, accel XYZ — little-endian
        if (!device.readRegisters(REG_OUT_TEMP_L, b, sizeof(b))) return false;

        s.temperature = le16(b + 0);
        s.gyroX       = le16(b + 2);
        s.gyroY       = le16(b + 4);
        s.gyroZ       = le16(b + 6);
        s.accelX      = le16(b + 8);
        s.accelY      = le16(b + 10);
        s.accelZ      = le16(b + 12);
        return true;
    }

    // ±16g: 0.488 mg/LSB; ±2000 °/с: 70 mdps/LSB (lsm6dsv_reg.c).
    float accelLsbPerG() const override { return 1000.0f / 0.488f; }
    float gyroLsbPerDps() const override { return 1000.0f / 70.0f; }

    float temperatureC(int16_t raw) const override
    {
        return raw / 256.0f + 25.0f;
    }


private:

    static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
    static constexpr uint8_t REG_CTRL1      = 0x10;
    static constexpr uint8_t REG_CTRL2      = 0x11;
    static constexpr uint8_t REG_CTRL3      = 0x12;
    static constexpr uint8_t REG_CTRL6      = 0x15;
    static constexpr uint8_t REG_CTRL7      = 0x16;
    static constexpr uint8_t REG_CTRL8      = 0x17;
    static constexpr uint8_t REG_CTRL9      = 0x18;
    static constexpr uint8_t REG_OUT_TEMP_L = 0x20;

    static constexpr int WHO_AM_I_VALUE = 0x70;

    static constexpr uint8_t CTRL3_SW_RESET = 0x01;
    static constexpr uint8_t CTRL3_IF_INC   = 0x04;
    static constexpr uint8_t CTRL3_BDU      = 0x40;
    static constexpr uint8_t CTRL7_LPF1_G_EN = 0x01;
    static constexpr uint8_t CTRL8_VARIANT_32X = 0x04;
    static constexpr uint8_t CTRL9_LPF2_XL_EN = 0x08;

    static constexpr uint8_t ODR_960HZ    = 0x09;
    static constexpr uint8_t FS_G_2000DPS = 0x04;
    static constexpr uint8_t FS_XL_16G_ON_16X = 0x03;
    static constexpr uint8_t FS_XL_16G_ON_32X = 0x02 | CTRL8_VARIANT_32X;

    // LPF1 гироскопа — ступень "strong" (lsm6dsv_filt_gy_lp1_bandwidth_t),
    // срез — десятки Гц при ODR 960 Гц (таблица LPF1 в датащите).
    // LPF2 акселерометра — код 2 = ODR/20 ≈ 48 Гц.
    static constexpr uint8_t GYRO_LPF1_BW  = 0x04;
    static constexpr uint8_t ACCEL_LPF2_BW = 0x02;

    static constexpr uint32_t BOOT_TIME_MS = 10;
    static constexpr uint8_t RESET_POLL_TRIES = 50;

    IRegisterDevice& device;
    bool is32x = false;

    bool fail()
    {
        setAvailable(false);
        return false;
    }

    // SW_RESET сбрасывается чипом сам, когда сброс завершён.
    bool softReset()
    {
        if (!device.writeRegister(REG_CTRL3, CTRL3_SW_RESET)) return false;

        for (uint8_t i = 0; i < RESET_POLL_TRIES; ++i)
        {
            delay(1);
            const int ctrl3 = device.readRegister(REG_CTRL3);
            if (ctrl3 >= 0 && (ctrl3 & CTRL3_SW_RESET) == 0) return true;
        }
        return false;
    }

    static int16_t le16(const uint8_t* p)
    {
        return static_cast<int16_t>((p[1] << 8) | p[0]);
    }
};
