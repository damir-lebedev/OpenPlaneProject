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

    explicit MPU6050_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readSample(RawImuSample& s) override;

    // ±16g и ±2000°/с (см. configure()) — датащит MPU6050 §4.17/4.19.
    float accelLsbPerG() const override { return 2048.0f; }
    float gyroLsbPerDps() const override { return 16.4f; }

    float temperatureC(int16_t raw) const override;


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

    bool configure();

    static const char* chipName(int whoAmI);

    static int16_t be16(const uint8_t* p);
};
