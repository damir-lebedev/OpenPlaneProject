#pragma once
#include <Arduino.h>

#include "config/Config.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorMounting.h"
#include "sensors/imu/AttitudeEstimator.h"

// ============================================================
// IMU SENSOR BASE — общая часть всех IMU проекта
//
// Драйвер конкретного чипа (MPU6050_Sensor, ICM42688_Sensor)
// реализует только работу с железом: begin() (опознать чип,
// настроить регистры) и readSample() (сырые отсчёты), плюс
// масштабы. Всё остальное — здесь, одинаково для любого чипа:
//
//   сырые отсчёты -> калибровка (смещения в единицах АЦП)
//     -> масштаб в g и °/с -> поворот осей чипа в оси самолёта
//     (Config::IMU_ROTATION_CW_DEG) -> авиационные знаки
//     -> AttitudeEstimator (крен/тангаж/рысканье).
//
// Оси чипа (у MPU6050/6500 и ICM42688 одинаковые): X, Y в
// плоскости платы, Z вверх из микросхемы — правая тройка. После
// поворота это оси самолёта X к носу, Y влево, Z вверх, и в
// авиационных знаках крен = вращение вокруг X как есть, а тангаж и
// рысканье — с обратным знаком (Y влево, Z вверх).
//
// Ошибки шины: чтение не удалось — данные не затираются мусором,
// растёт счётчик; MAX_CONSECUTIVE_ERRORS подряд — isAvailable()
// становится false, и автопилот перестаёт давать коррекции по
// устаревшим углам. Как только чтения восстанавливаются — IMU снова
// доступен.
// ============================================================

// Один отсчёт в "сырых" единицах АЦП, в осях чипа.
struct RawImuSample
{
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temperature;
};

class ImuSensorBase : public ImuSensor
{
public:

    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    void update() override
    {
        if (!available) return;

        RawImuSample sample;
        if (!readSample(sample))
        {
            if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
            errorCount++;
            return;
        }

        consecutiveErrors = 0;
        process(sample);
    }

    const ImuData& getImuData() const override
    {
        return imuData;
    }

    // Самолёт неподвижен ~2 с: гироскоп -> нулевое смещение,
    // акселерометр -> текущее положение принимается за горизонт
    // (roll = pitch = 0). Поэтому калибровать надо, когда самолёт
    // стоит так, как должен лететь ровно.
    void calibrate() override
    {
        if (!available) return;

        Serial.print(name);
        Serial.println(": калибровка...");

        float sum[6] = {};
        int samples = 0;

        for (int i = 0; i < CALIBRATION_SAMPLES; i++)
        {
            RawImuSample s;
            if (readSample(s))
            {
                sum[0] += s.gyroX;  sum[1] += s.gyroY;  sum[2] += s.gyroZ;
                sum[3] += s.accelX; sum[4] += s.accelY; sum[5] += s.accelZ;
                samples++;
            }
            delay(10);
        }

        if (samples < CALIBRATION_SAMPLES / 2)
        {
            Serial.print(name);
            Serial.println(": калибровка не удалась — датчик не отвечает");
            return;
        }

        // Смещение акселерометра по Z — всё, что сверх 1g (плата
        // лежит микросхемой вверх).
        gyroOffset[0] = sum[0] / samples;
        gyroOffset[1] = sum[1] / samples;
        gyroOffset[2] = sum[2] / samples;
        accelOffset[0] = sum[3] / samples;
        accelOffset[1] = sum[4] / samples;
        accelOffset[2] = sum[5] / samples - accelLsbPerG();

        calibrated = true;
        estimator.reset();

        Serial.print(name);
        Serial.print(": калибровка завершена, смещение гироскопа=");
        Serial.print(gyroOffset[0]); Serial.print(",");
        Serial.print(gyroOffset[1]); Serial.print(",");
        Serial.println(gyroOffset[2]);
    }

    void setYaw(float yawDegrees) override
    {
        estimator.setYaw(yawDegrees);
        imuData.yaw = estimator.getYaw();
    }

    const char* getSensorType() const override
    {
        return name;
    }

    void printStatus() const override
    {
        Serial.print(name);
        Serial.print(": available="); Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" calibrated="); Serial.print(calibrated ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
        Serial.print(" gyro(dps)="); Serial.print(imuData.gyroX, 2);
        Serial.print(","); Serial.print(imuData.gyroY, 2);
        Serial.print(","); Serial.print(imuData.gyroZ, 2);
        Serial.print(" accel(g)="); Serial.print(imuData.accelX, 2);
        Serial.print(","); Serial.print(imuData.accelY, 2);
        Serial.print(","); Serial.print(imuData.accelZ, 2);
        Serial.print(" roll="); Serial.print(imuData.roll, 1);
        Serial.print(" pitch="); Serial.print(imuData.pitch, 1);
        Serial.print(" yaw="); Serial.print(imuData.yaw, 1);
        Serial.print(" temp="); Serial.println(imuData.temperature, 1);
    }


protected:

    explicit ImuSensorBase(const char* name)
        : name(name)
    {
        memset(&imuData, 0, sizeof(imuData));
    }

    // --- то, что реализует драйвер конкретного чипа ---

    // Один отсчёт в осях чипа. false — чип не ответил.
    virtual bool readSample(RawImuSample& sample) = 0;

    virtual float accelLsbPerG() const = 0;
    virtual float gyroLsbPerDps() const = 0;
    virtual float temperatureC(int16_t raw) const = 0;

    // Вызывает begin() драйвера: чип опознан и настроен (или нет).
    void setAvailable(bool isAvailable)
    {
        available = isAvailable;
    }

    // Имя чипа для лога — драйвер может уточнить после опознания
    // (MPU6050 vs MPU6500).
    void setName(const char* chipName)
    {
        name = chipName;
    }


private:

    static constexpr int CALIBRATION_SAMPLES = 200;

    // ~0.1 с подряд без ответа при цикле 2 мс -> датчик недоступен.
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 50;

    const char* name;
    bool available = false;
    bool calibrated = false;

    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    float gyroOffset[3] = {};
    float accelOffset[3] = {};

    AttitudeEstimator estimator;
    ImuData imuData;

    void process(const RawImuSample& s)
    {
        const float accelScale = accelLsbPerG();
        const float gyroScale = gyroLsbPerDps();

        // Калибровка и масштаб — в осях чипа.
        const float chipAx = (s.accelX - accelOffset[0]) / accelScale;
        const float chipAy = (s.accelY - accelOffset[1]) / accelScale;
        const float az     = (s.accelZ - accelOffset[2]) / accelScale;
        const float chipGx = (s.gyroX - gyroOffset[0]) / gyroScale;
        const float chipGy = (s.gyroY - gyroOffset[1]) / gyroScale;
        const float gz     = (s.gyroZ - gyroOffset[2]) / gyroScale;

        // Оси чипа -> оси самолёта (X к носу, Y влево, Z вверх).
        float ax, ay, gx, gy;
        SensorMounting::rotateToBody(Config::IMU_ROTATION_CW_DEG, chipAx, chipAy, ax, ay);
        SensorMounting::rotateToBody(Config::IMU_ROTATION_CW_DEG, chipGx, chipGy, gx, gy);

        imuData.accelX = ax;
        imuData.accelY = ay;
        imuData.accelZ = az;

        // Авиационные знаки угловых скоростей.
        imuData.gyroX = gx;   // крен:     + правое крыло вниз
        imuData.gyroY = -gy;  // тангаж:   + нос вверх
        imuData.gyroZ = -gz;  // рысканье: + нос вправо

        imuData.temperature = temperatureC(s.temperature);

        const uint32_t now = micros();
        estimator.update(ax, ay, az, imuData.gyroX, imuData.gyroY, imuData.gyroZ, now);

        imuData.roll = estimator.getRoll();
        imuData.pitch = estimator.getPitch();
        imuData.yaw = estimator.getYaw();
        imuData.timestamp = now;
    }
};
