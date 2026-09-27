#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// ICM-42688-P (плата "601N1") — гироскоп + акселерометр
//
// Обычно SPI (SpiRegisterDevice, свой CS — Config::PIN_SPI_CS_IMU),
// чип умеет и I2C (адрес 0x68/0x69) — драйвер работает через
// IRegisterDevice и шины не различает.
//
// Калибровка, поворот осей (Config::IMU_ROTATION_CW_DEG), знаки и
// фильтр ориентации — в ImuSensorBase, одинаково с MPU6050. Оси чипа
// те же: X, Y в плоскости, Z вверх из микросхемы.
//
// Настройка: ±2000°/с и ±16g (как у MPU6050 — узкие диапазоны
// насыщаются в манёвре), ODR 1 кГц, режим Low Noise, фильтр
// UI ~50 Гц. Регистры банковые (REG_BANK_SEL) — работаем в банке 0.
// Порядок burst-чтения: температура, accel XYZ, gyro XYZ (у MPU6050
// accel идёт первым), big-endian.
//
// Не проверен на железе: при подключении — WHO_AM_I в логе, знаки
// наклоном (нос вверх -> P > 0, правое крыло вниз -> R > 0).
// ============================================================

class ICM42688_Sensor : public ImuSensorBase
{
public:

    // По SPI до 24 МГц; 8 МГц — с запасом для проводов на макетке.
    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin);

    explicit ICM42688_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readSample(RawImuSample& s) override;

    float accelLsbPerG() const override { return 2048.0f; }   // ±16g
    float gyroLsbPerDps() const override { return 16.4f; }    // ±2000°/с

    float temperatureC(int16_t raw) const override;


private:

    // Банк 0.
    static constexpr uint8_t REG_DEVICE_CONFIG       = 0x11;
    static constexpr uint8_t REG_TEMP_DATA1          = 0x1D;
    static constexpr uint8_t REG_PWR_MGMT0           = 0x4E;
    static constexpr uint8_t REG_GYRO_CONFIG0        = 0x4F;
    static constexpr uint8_t REG_ACCEL_CONFIG0       = 0x50;
    static constexpr uint8_t REG_GYRO_ACCEL_CONFIG0  = 0x52;
    static constexpr uint8_t REG_WHO_AM_I            = 0x75;
    static constexpr uint8_t REG_BANK_SEL            = 0x76;

    static constexpr int WHO_AM_I_VALUE = 0x47;

    IRegisterDevice& device;

    static int16_t be16(const uint8_t* p);
};
