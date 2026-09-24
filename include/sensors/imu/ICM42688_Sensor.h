#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// ICM-42688-P (плата "601N1") — гироскоп + акселерометр
//
// Обычно SPI (SpiRegisterDevice, свой CS — Config::PIN_SPI_CS_ICM42688),
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
    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin)
    {
        return SpiRegisterDevice(bus, chipSelectPin, 8000000, 0);
    }

    explicit ICM42688_Sensor(IRegisterDevice& device)
        : ImuSensorBase("ICM42688"),
          device(device)
    {
    }

    bool begin() override
    {
        device.begin();
        delay(10);

        device.writeRegister(REG_BANK_SEL, 0x00);        // банк 0 — чип может включиться в другом
        device.writeRegister(REG_DEVICE_CONFIG, 0x01);   // SOFT_RESET
        delay(2);                                        // датащит: ≥1 мс после сброса
        device.writeRegister(REG_BANK_SEL, 0x00);

        const int whoAmI = device.readRegister(REG_WHO_AM_I);
        if (whoAmI != WHO_AM_I_VALUE)
        {
            Serial.print("ICM42688: неверный WHO_AM_I ");
            Serial.println(whoAmI < 0 ? String("(нет ответа)") : String("0x") + String(whoAmI, HEX));
            setAvailable(false);
            return false;
        }

        bool ok = true;

        // GYRO_MODE=11 и ACCEL_MODE=11 (Low Noise). Датащит требует
        // ≥200 мкс после включения режимов до других записей.
        ok &= device.writeRegister(REG_PWR_MGMT0, 0x0F);
        delay(1);

        // FS_SEL=000 -> ±2000°/с / ±16g; ODR=0110 -> 1 кГц.
        ok &= device.writeRegister(REG_GYRO_CONFIG0, 0x06);
        ok &= device.writeRegister(REG_ACCEL_CONFIG0, 0x06);

        // Полоса UI-фильтров: 6 = ODR/20 = 50 Гц для accel и gyro —
        // по смыслу как DLPF ~41 Гц у MPU6050.
        ok &= device.writeRegister(REG_GYRO_ACCEL_CONFIG0, 0x66);

        if (!ok)
        {
            Serial.println("ICM42688: ошибка записи регистров");
            setAvailable(false);
            return false;
        }

        Serial.println("ICM42688: подключён");
        setAvailable(true);
        return true;
    }


protected:

    bool readSample(RawImuSample& s) override
    {
        uint8_t b[14];  // temp, accel XYZ, gyro XYZ — big-endian
        if (!device.readRegisters(REG_TEMP_DATA1, b, sizeof(b))) return false;

        s.temperature = be16(b + 0);
        s.accelX      = be16(b + 2);
        s.accelY      = be16(b + 4);
        s.accelZ      = be16(b + 6);
        s.gyroX       = be16(b + 8);
        s.gyroY       = be16(b + 10);
        s.gyroZ       = be16(b + 12);
        return true;
    }

    float accelLsbPerG() const override { return 2048.0f; }   // ±16g
    float gyroLsbPerDps() const override { return 16.4f; }    // ±2000°/с

    float temperatureC(int16_t raw) const override
    {
        return raw / 132.48f + 25.0f;  // датащит ICM-42688-P
    }


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

    static int16_t be16(const uint8_t* p)
    {
        return static_cast<int16_t>((p[0] << 8) | p[1]);
    }
};
