#pragma once
#include <Arduino.h>

// ============================================================
// ICM-42688-P (плата "601N1") — гироскоп + акселерометр
//
// SPI, свой CS-пин (см. Config::PIN_SPI_CS_ICM42688). Регистры
// читаются напрямую через ISpiBus — собственная минимальная
// реализация, без vendor-библиотек, по структуре и математике
// (комплементарный фильтр roll/pitch, интеграция yaw, калибровка
// усреднением 200 отсчётов) сознательно скопирована с
// MPU6050_Sensor.h — принцип ориентации тот же, чип другой.
//
// Диапазоны выбраны так же, как у MPU6050 (±250°/сек, ±2g), чтобы
// переиспользовать те же коэффициенты масштаба (131 LSB/(°/сек),
// 16384 LSB/g) — это не совпадение, а сознательный выбор FS_SEL
// при инициализации (см. initialize()).
//
// Отличие от MPU6050 по железу: регистры банковые (REG_BANK_SEL),
// и порядок burst-чтения другой — сначала температура, потом
// accel, потом gyro (у MPU6050 наоборот, accel первым).
// ============================================================

#include "SensorInterface.h"
#include "../hal/ISpiBus.h"

class ICM42688_Sensor : public ImuSensor
{
public:

    ICM42688_Sensor(ISpiBus& bus, uint8_t chipSelectPin)
        : spi(bus),
          csPin(chipSelectPin),
          available(false),
          calibrationDone(false)
    {
        memset(&imuData, 0, sizeof(imuData));
        memset(&calibration, 0, sizeof(calibration));
    }

    bool begin() override
    {
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);

        delay(100);

        const uint8_t whoAmI = readRegister(REG_WHO_AM_I);
        if (whoAmI != 0x47)
        {
            Serial.print("ICM42688: неверный WHO_AM_I 0x");
            Serial.println(whoAmI, HEX);
            available = false;
            return false;
        }

        initialize();

        available = true;
        Serial.println("ICM42688: подключён");

        return true;
    }

    bool isAvailable() const override
    {
        return available;
    }

    // Вызывать регулярно из loop()/Autopilot::update(); не делает
    // ничего, если датчик недоступен (imuData остаётся нулевым).
    void update() override
    {
        if (!available) return;

        readRawData();
        applyCalibration();
        calculateAngles();

        imuData.timestamp = micros();
    }

    const ImuData& getImuData() const override
    {
        return imuData;
    }

    // Нужно вызывать на земле, пока самолёт неподвижен (~2 секунды).
    // Усредняет 200 сырых отсчётов и сохраняет их как нулевое смещение.
    void calibrate() override
    {
        if (!available) return;

        Serial.println("ICM42688: калибровка...");

        const int SAMPLE_COUNT = 200;
        float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
        float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;

        for (int i = 0; i < SAMPLE_COUNT; i++)
        {
            readRawData();

            sumGyroX += rawGx;
            sumGyroY += rawGy;
            sumGyroZ += rawGz;

            sumAccelX += rawAx;
            sumAccelY += rawAy;
            sumAccelZ += rawAz;

            delay(10);
        }

        calibration.gyroOffsetX = sumGyroX / SAMPLE_COUNT;
        calibration.gyroOffsetY = sumGyroY / SAMPLE_COUNT;
        calibration.gyroOffsetZ = sumGyroZ / SAMPLE_COUNT;

        calibration.accelOffsetX = sumAccelX / SAMPLE_COUNT;
        calibration.accelOffsetY = sumAccelY / SAMPLE_COUNT;
        calibration.accelOffsetZ = sumAccelZ / SAMPLE_COUNT - 16384;  // компенсация -1g по Z

        calibrationDone = true;

        Serial.print("ICM42688: калибровка завершена, offset gyro=");
        Serial.print(calibration.gyroOffsetX); Serial.print(",");
        Serial.print(calibration.gyroOffsetY); Serial.print(",");
        Serial.println(calibration.gyroOffsetZ);
    }

    // Используется, чтобы сбросить направление yaw в начале полёта.
    void setYaw(float yawDegrees) override
    {
        imuData.yaw = yawDegrees;
        yawIntegral = yawDegrees;
    }

    const char* getSensorType() const override
    {
        return "ICM42688 (601N1)";
    }

    void printStatus() const override
    {
        Serial.print("ICM42688: available=");
        Serial.print(available ? "YES" : "NO");
        Serial.print(" calibrated=");
        Serial.print(calibrationDone ? "YES" : "NO");
        Serial.print(" gyro(dps)="); Serial.print(imuData.gyroX, 2);
        Serial.print(","); Serial.print(imuData.gyroY, 2);
        Serial.print(","); Serial.print(imuData.gyroZ, 2);
        Serial.print(" accel(g)="); Serial.print(imuData.accelX, 2);
        Serial.print(","); Serial.print(imuData.accelY, 2);
        Serial.print(","); Serial.print(imuData.accelZ, 2);
        Serial.print(" roll="); Serial.print(imuData.roll, 1);
        Serial.print(" pitch="); Serial.print(imuData.pitch, 1);
        Serial.print(" yaw="); Serial.println(imuData.yaw, 1);
    }


private:

    // Регистры банка 0 (REG_BANK_SEL=0 — выставляется в initialize()).
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t REG_BANK_SEL     = 0x76;
    static constexpr uint8_t REG_PWR_MGMT0    = 0x4E;
    static constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
    static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
    static constexpr uint8_t REG_TEMP_DATA1   = 0x1D;  // burst-чтение: temp, accel, gyro — 14 байт

    ISpiBus& spi;
    uint8_t csPin;
    bool available;
    bool calibrationDone;

    int16_t rawAx, rawAy, rawAz;
    int16_t rawGx, rawGy, rawGz;
    int16_t rawTemp;

    struct
    {
        float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
        float accelOffsetX, accelOffsetY, accelOffsetZ;
    } calibration;

    ImuData imuData;

    // Интегрируем yaw просто из гироскопа — та же оговорка, что и
    // в MPU6050_Sensor.h: без абсолютной опоры, будет уплывать.
    float yawIntegral = 0;

    void initialize()
    {
        writeRegister(REG_BANK_SEL, 0x00);  // явно банк 0 — чип может включиться в другом

        // GYRO_MODE=11 (Low Noise), ACCEL_MODE=11 (Low Noise).
        writeRegister(REG_PWR_MGMT0, 0x0F);
        delay(1);  // датащит требует паузу после включения режимов перед чтением данных

        // FS_SEL=3 -> ±250°/сек (совпадает по масштабу с MPU6050: 131 LSB/(°/сек)).
        // ODR=0110 -> 1 kHz.
        writeRegister(REG_GYRO_CONFIG0, 0x66);

        // FS_SEL=3 -> ±2g (совпадает по масштабу с MPU6050: 16384 LSB/g).
        // ODR=0110 -> 1 kHz.
        writeRegister(REG_ACCEL_CONFIG0, 0x66);
    }

    void readRawData()
    {
        uint8_t buf[14];

        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);

        spi.transfer(REG_TEMP_DATA1 | 0x80);
        for (uint8_t i = 0; i < 14; ++i)
        {
            buf[i] = spi.transfer(0x00);
        }

        spi.endTransaction();
        digitalWrite(csPin, HIGH);

        // Порядок в этом чипе: temp, accel(x,y,z), gyro(x,y,z) —
        // не как у MPU6050, где accel идёт первым.
        rawTemp = (buf[0] << 8) | buf[1];
        rawAx = (buf[2] << 8) | buf[3];
        rawAy = (buf[4] << 8) | buf[5];
        rawAz = (buf[6] << 8) | buf[7];
        rawGx = (buf[8] << 8) | buf[9];
        rawGy = (buf[10] << 8) | buf[11];
        rawGz = (buf[12] << 8) | buf[13];
    }

    void applyCalibration()
    {
        imuData.accelX = (rawAx - calibration.accelOffsetX) / 16384.0f;
        imuData.accelY = (rawAy - calibration.accelOffsetY) / 16384.0f;
        imuData.accelZ = (rawAz - calibration.accelOffsetZ) / 16384.0f;

        imuData.gyroX = (rawGx - calibration.gyroOffsetX) / 131.0f;
        imuData.gyroY = (rawGy - calibration.gyroOffsetY) / 131.0f;
        imuData.gyroZ = (rawGz - calibration.gyroOffsetZ) / 131.0f;

        // Формула ICM42688 из датащита, отличается от MPU6050.
        imuData.temperature = (rawTemp / 132.48f) + 25.0f;
    }

    // Комплементарный фильтр — идентичен MPU6050_Sensor.h: 70% гироскоп
    // (интеграция) + 30% акселерометр (абсолютный угол, но шумный).
    void calculateAngles()
    {
        float accelRoll = atan2(imuData.accelY, imuData.accelZ) * 57.2958f;

        float accelPitch = atan2(-imuData.accelX,
                                 sqrt(imuData.accelY * imuData.accelY +
                                      imuData.accelZ * imuData.accelZ)) * 57.2958f;

        static unsigned long lastTime = 0;
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        if (dt > 0 && dt < 0.1f)  // защита от скачка dt после паузы
        {
            imuData.roll = imuData.roll * 0.7f + accelRoll * 0.3f +
                          imuData.gyroX * dt;

            imuData.pitch = imuData.pitch * 0.7f + accelPitch * 0.3f +
                           imuData.gyroY * dt;

            yawIntegral += imuData.gyroZ * dt;
            imuData.yaw = yawIntegral;
        }

        if (imuData.roll > 180) imuData.roll -= 360;
        if (imuData.roll < -180) imuData.roll += 360;

        if (imuData.pitch > 180) imuData.pitch -= 360;
        if (imuData.pitch < -180) imuData.pitch += 360;

        if (imuData.yaw > 180) imuData.yaw -= 360;
        if (imuData.yaw < -180) imuData.yaw += 360;
    }

    uint8_t readRegister(uint8_t reg)
    {
        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);

        spi.transfer(reg | 0x80);
        uint8_t value = spi.transfer(0x00);

        spi.endTransaction();
        digitalWrite(csPin, HIGH);

        return value;
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);

        spi.transfer(reg & 0x7F);
        spi.transfer(value);

        spi.endTransaction();
        digitalWrite(csPin, HIGH);
    }
};
