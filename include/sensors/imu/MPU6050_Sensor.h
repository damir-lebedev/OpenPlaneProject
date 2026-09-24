#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// MPU6050 / MPU6500 (платы GY-521 и их клоны) — гироскоп + акселерометр
//
// Обычно I2C, адрес 0x68 (AD0=GND) или 0x69 (AD0=VCC). Драйвер
// работает через IRegisterDevice, поэтому MPU6500 можно посадить и
// на SPI (SpiRegisterDevice), если плата это позволяет.
//
// На платах "GY-521" часто стоит не MPU6050, а MPU6500 или клон
// (WHO_AM_I = 0x70 вместо 0x68 — так и на текущем стенде). Регистры
// данных и диапазонов у них совпадают; отличаются отдельный фильтр
// акселерометра (ACCEL_CONFIG2, есть только у 6500) и формула
// температуры — драйвер определяет чип по WHO_AM_I.
//
// Калибровка, поворот осей, знаки и фильтр ориентации — в
// ImuSensorBase, здесь только регистры.
// ============================================================

class MPU6050_Sensor : public ImuSensorBase
{
public:

    explicit MPU6050_Sensor(IRegisterDevice& device)
        : ImuSensorBase("MPU6050"),
          device(device)
    {
    }

    bool begin() override
    {
        device.begin();

        const int whoAmI = device.readRegister(REG_WHO_AM_I);
        if (whoAmI < 0)
        {
            Serial.println("MPU6050: датчик не отвечает, IMU недоступен");
            setAvailable(false);
            return false;
        }

        isMpu6500Family = (whoAmI != WHO_AM_I_MPU6050);
        setName(chipName(whoAmI));

        Serial.print("MPU6050: WHO_AM_I=0x");
        Serial.print(whoAmI, HEX);
        Serial.print(" -> ");
        Serial.println(chipName(whoAmI));

        if (!configure())
        {
            Serial.println("MPU6050: ошибка записи регистров, IMU недоступен");
            setAvailable(false);
            return false;
        }

        setAvailable(true);
        return true;
    }


protected:

    bool readSample(RawImuSample& s) override
    {
        uint8_t b[14];  // accel XYZ, temp, gyro XYZ — big-endian
        if (!device.readRegisters(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

        s.accelX      = be16(b + 0);
        s.accelY      = be16(b + 2);
        s.accelZ      = be16(b + 4);
        s.temperature = be16(b + 6);
        s.gyroX       = be16(b + 8);
        s.gyroY       = be16(b + 10);
        s.gyroZ       = be16(b + 12);
        return true;
    }

    // ±16g и ±2000°/с (см. configure()) — датащит MPU6050 §4.17/4.19.
    float accelLsbPerG() const override { return 2048.0f; }
    float gyroLsbPerDps() const override { return 16.4f; }

    float temperatureC(int16_t raw) const override
    {
        return isMpu6500Family
            ? raw / 333.87f + 21.0f     // датащит MPU6500
            : raw / 340.0f + 36.53f;    // датащит MPU6050
    }


private:

    static constexpr uint8_t REG_SMPLRT_DIV    = 0x19;
    static constexpr uint8_t REG_CONFIG        = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // только MPU6500-семейство
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I      = 0x75;

    static constexpr int WHO_AM_I_MPU6050 = 0x68;

    IRegisterDevice& device;
    bool isMpu6500Family = false;

    bool configure()
    {
        device.writeRegister(REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET
        delay(100);

        bool ok = true;
        ok &= device.writeRegister(REG_PWR_MGMT_1, 0x01);    // выход из sleep, такт от PLL гироскопа
        delay(10);

        // Широкие диапазоны: в резком манёвре и на вибрации узкие
        // (±250°/с, ±2g) насыщаются и дают мусор в углы.
        ok &= device.writeRegister(REG_GYRO_CONFIG, 0x18);   // ±2000°/с (FS_SEL=3)
        ok &= device.writeRegister(REG_ACCEL_CONFIG, 0x18);  // ±16g (AFS_SEL=3)

        // Цифровой ФНЧ гироскопа DLPF_CFG=3: ~42 Гц (MPU6050) / 41 Гц
        // (MPU6500), задержка ~5 мс. Срезает вибрацию мотора и винта
        // (10x5 на ~8000 об/мин — ~130 Гц), не добавляя заметного
        // запаздывания для стабилизации самолёта.
        ok &= device.writeRegister(REG_CONFIG, 0x03);
        ok &= device.writeRegister(REG_SMPLRT_DIV, 0x00);    // 1 кГц — быстрее цикла

        // У MPU6500 фильтр акселерометра отдельный и по умолчанию
        // почти выключен (218 Гц) — ставим те же ~41 Гц.
        if (isMpu6500Family)
        {
            ok &= device.writeRegister(REG_ACCEL_CONFIG2, 0x03);
        }

        return ok;
    }

    static const char* chipName(int whoAmI)
    {
        switch (whoAmI)
        {
            case 0x68: return "MPU6050";
            case 0x70: return "MPU6500";
            case 0x71: return "MPU9250";
            case 0x73: return "MPU9255";
            default:   return "MPU6500-совместимый клон";
        }
    }

    static int16_t be16(const uint8_t* p)
    {
        return static_cast<int16_t>((p[0] << 8) | p[1]);
    }
};
