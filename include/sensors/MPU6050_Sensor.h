#pragma once
#include <Arduino.h>

// ============================================================
// MPU6050 GY-521 (гироскоп + акселерометр)
//
// I2C, адрес 0x68 (AD0=GND) или 0x69 (AD0=VCC). Регистры читаются
// напрямую через Wire — это НЕ обёртка над библиотекой jrowberg/
// MPU6050 (её и нет в platformio.ini), а собственная минимальная
// реализация под то, что нужно автопилоту: углы roll/pitch через
// комплементарный фильтр + сырой yaw-рейт.
// ============================================================

#include "SensorInterface.h"
#include <Wire.h>

class MPU6050_Sensor : public ImuSensor
{
public:

    // address: 0x68 (AD0=GND, по умолчанию) или 0x69 (AD0=VCC).
    explicit MPU6050_Sensor(uint8_t address = 0x68)
        : i2cAddress(address),
          available(false),
          calibrationDone(false)
    {
        memset(&imuData, 0, sizeof(imuData));
        memset(&calibration, 0, sizeof(calibration));
    }

    bool begin() override
    {
        Wire.begin();          // SCL=GPIO22, SDA=GPIO21 (стандартные пины ESP32)
        Wire.setClock(400000);

        delay(100);

        if (!checkConnection())
        {
            Serial.println("MPU6050: датчик не отвечает на I2C, IMU недоступен");
            available = false;
            return false;
        }

        if (!initialize())
        {
            Serial.println("MPU6050: ошибка инициализации регистров");
            available = false;
            return false;
        }

        available = true;
        Serial.println("MPU6050: подключён");

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

        Serial.println("MPU6050: калибровка...");

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

        Serial.print("MPU6050: калибровка завершена, offset gyro=");
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
        return "MPU6050 GY-521";
    }

    void printStatus() const override
    {
        Serial.print("MPU6050: available=");
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

    uint8_t i2cAddress;
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

    // Интегрируем yaw просто из гироскопа — Z-ось не имеет
    // абсолютной опорной точки (в отличие от roll/pitch по акселерометру),
    // поэтому будет медленно "уплывать".
    float yawIntegral = 0;

    bool checkConnection()
    {
        Wire.beginTransmission(i2cAddress);
        return (Wire.endTransmission() == 0);
    }

    bool initialize()
    {
        writeRegister(0x6B, 0x00);  // PWR_MGMT_1: выход из sleep
        writeRegister(0x1B, 0x00);  // GYRO_CONFIG: диапазон ±250°/сек
        writeRegister(0x1C, 0x00);  // ACCEL_CONFIG: диапазон ±2g
        writeRegister(0x19, 0x07);  // SMPLRT_DIV: 1kHz / (1+7) = 125 Hz
        writeRegister(0x1A, 0x05);  // CONFIG: температурная компенсация гироскопа

        return true;
    }

    void readRawData()
    {
        Wire.beginTransmission(i2cAddress);
        Wire.write(0x3B);  // ACCEL_XOUT_H — далее 14 байт: accel, temp, gyro
        Wire.endTransmission(false);

        Wire.requestFrom(i2cAddress, (uint8_t)14);

        rawAx = (Wire.read() << 8) | Wire.read();
        rawAy = (Wire.read() << 8) | Wire.read();
        rawAz = (Wire.read() << 8) | Wire.read();
        rawTemp = (Wire.read() << 8) | Wire.read();
        rawGx = (Wire.read() << 8) | Wire.read();
        rawGy = (Wire.read() << 8) | Wire.read();
        rawGz = (Wire.read() << 8) | Wire.read();
    }

    void applyCalibration()
    {
        // Масштаб для диапазона ±2g / ±250°/сек (см. initialize()).
        imuData.accelX = (rawAx - calibration.accelOffsetX) / 16384.0f;
        imuData.accelY = (rawAy - calibration.accelOffsetY) / 16384.0f;
        imuData.accelZ = (rawAz - calibration.accelOffsetZ) / 16384.0f;

        imuData.gyroX = (rawGx - calibration.gyroOffsetX) / 131.0f;
        imuData.gyroY = (rawGy - calibration.gyroOffsetY) / 131.0f;
        imuData.gyroZ = (rawGz - calibration.gyroOffsetZ) / 131.0f;

        imuData.temperature = (rawTemp / 340.0f) + 36.53f;  // формула из датащита
    }

    // Комплементарный фильтр: акселерометр даёт абсолютный угол,
    // но шумит; гироскоп даёт гладкую скорость без абсолютной опоры.
    // Смешиваем 70% гироскопа (интеграция) + 30% акселерометра.
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

    void writeRegister(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(i2cAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};
