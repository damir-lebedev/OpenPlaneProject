#pragma once
#include <Arduino.h>

// ============================================================
// MPU6050 / MPU6500 (платы GY-521 и их клоны) — гироскоп + акселерометр
//
// I2C, адрес 0x68 (AD0=GND) или 0x69 (AD0=VCC). Регистры читаются
// напрямую через II2CBus (см. hal/II2CBus.h) — это НЕ обёртка над
// библиотекой jrowberg/MPU6050, а собственная минимальная реализация
// под то, что нужно автопилоту: углы roll/pitch через комплементарный
// фильтр + yaw, проинтегрированный из гироскопа.
//
// На платах "GY-521" часто стоит не MPU6050, а MPU6500 или клон
// (WHO_AM_I = 0x70 вместо 0x68 — так и на текущем стенде). Регистры
// данных/диапазонов у них совпадают, отличаются только отдельный
// фильтр акселерометра (ACCEL_CONFIG2, есть только у 6500) и формула
// температуры — драйвер определяет чип по WHO_AM_I.
//
// СИСТЕМА КООРДИНАТ (авиационная, как у ArduPilot/PX4):
//   roll  > 0 — правое крыло вниз
//   pitch > 0 — нос вверх
//   yaw   > 0 — нос вправо (по часовой, если смотреть сверху)
//   gyroX/gyroY/gyroZ — угловые скорости в тех же знаках.
// Сначала оси чипа поворачиваются в оси самолёта (Config::
// IMU_ROTATION_CW_DEG — чип лежит микросхемой вверх, но может быть
// повёрнут вокруг вертикали) — получаются X к носу, Y влево, Z вверх.
// Затем pitch и yaw берутся с обратным знаком относительно gyroY/
// gyroZ в этих осях (правая тройка "влево/вверх" -> авиационные знаки).
// ============================================================

#include "SensorInterface.h"
#include "../Config.h"
#include "../hal/II2CBus.h"

class MPU6050_Sensor : public ImuSensor
{
public:

    // address: 0x68 (AD0=GND, по умолчанию) или 0x69 (AD0=VCC).
    explicit MPU6050_Sensor(II2CBus& bus, uint8_t address = 0x68)
        : i2c(bus),
          i2cAddress(address)
    {
        memset(&imuData, 0, sizeof(imuData));
        memset(&calibration, 0, sizeof(calibration));
    }

    bool begin() override
    {
        const int whoAmI = i2c.readRegister(i2cAddress, REG_WHO_AM_I);

        if (whoAmI < 0)
        {
            Serial.println("MPU6050: датчик не отвечает на I2C, IMU недоступен");
            available = false;
            return false;
        }

        isMpu6500Family = (whoAmI != 0x68);

        Serial.print("MPU6050: WHO_AM_I=0x");
        Serial.print(whoAmI, HEX);
        Serial.print(" (");
        Serial.print(chipName(whoAmI));
        Serial.println(")");

        if (!initialize())
        {
            Serial.println("MPU6050: ошибка записи регистров, IMU недоступен");
            available = false;
            return false;
        }

        available = true;
        return true;
    }

    // true, пока датчик отвечает: если чтения подряд проваливаются
    // (отпал провод, шина зависла), IMU считается недоступным, и
    // автопилот перестаёт давать коррекции по устаревшим углам.
    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    // Вызывать регулярно из Autopilot::update(); не делает ничего,
    // если датчик недоступен (imuData остаётся прежним).
    void update() override
    {
        if (!available) return;

        if (!readRawData())
        {
            if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
            errorCount++;
            return;  // остаются прошлые данные, а не мусор
        }

        consecutiveErrors = 0;
        applyCalibration();
        calculateAngles();

        imuData.timestamp = micros();
    }

    const ImuData& getImuData() const override
    {
        return imuData;
    }

    // Нужно вызывать на земле, пока самолёт неподвижен (~2 секунды).
    // Усредняет 200 сырых отсчётов: гироскоп -> нулевое смещение,
    // акселерометр -> текущее положение принимается за горизонт
    // (roll = pitch = 0). Поэтому включать нужно, когда самолёт стоит
    // так, как он должен лететь ровно.
    void calibrate() override
    {
        if (!available) return;

        Serial.println("MPU6050: калибровка...");

        const int SAMPLE_COUNT = 200;
        float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
        float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
        int samples = 0;

        for (int i = 0; i < SAMPLE_COUNT; i++)
        {
            if (readRawData())
            {
                sumGyroX += rawGx;
                sumGyroY += rawGy;
                sumGyroZ += rawGz;

                sumAccelX += rawAx;
                sumAccelY += rawAy;
                sumAccelZ += rawAz;
                samples++;
            }

            delay(10);
        }

        if (samples < SAMPLE_COUNT / 2)
        {
            Serial.println("MPU6050: калибровка не удалась — датчик не отвечает");
            return;
        }

        calibration.gyroOffsetX = sumGyroX / samples;
        calibration.gyroOffsetY = sumGyroY / samples;
        calibration.gyroOffsetZ = sumGyroZ / samples;

        calibration.accelOffsetX = sumAccelX / samples;
        calibration.accelOffsetY = sumAccelY / samples;
        calibration.accelOffsetZ = sumAccelZ / samples - ACCEL_SCALE_LSB_PER_G;  // компенсация -1g по Z

        calibrationDone = true;
        anglesInitialized = false;  // углы заново стартуют с акселерометра

        Serial.print("MPU6050: калибровка завершена, offset gyro=");
        Serial.print(calibration.gyroOffsetX); Serial.print(",");
        Serial.print(calibration.gyroOffsetY); Serial.print(",");
        Serial.println(calibration.gyroOffsetZ);
    }

    // Используется, чтобы сбросить направление yaw в начале полёта.
    void setYaw(float yawDegrees) override
    {
        imuData.yaw = wrap180(yawDegrees);
        yawIntegral = imuData.yaw;
    }

    const char* getSensorType() const override
    {
        return isMpu6500Family ? "MPU6500 (GY-521)" : "MPU6050 GY-521";
    }

    void printStatus() const override
    {
        Serial.print("MPU6050: available=");
        Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" calibrated=");
        Serial.print(calibrationDone ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
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

    static constexpr uint8_t REG_SMPLRT_DIV    = 0x19;
    static constexpr uint8_t REG_CONFIG        = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // только MPU6500-семейство
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I      = 0x75;

    // Масштабы под диапазон, прописанный в initialize()
    // (GYRO_CONFIG/ACCEL_CONFIG = ±2000°/сек / ±16g) — датащит MPU6050
    // §4.19/4.17. Раньше диапазон был ±250°/сек / ±2g (масштабы
    // 131 / 16384) — при резком манёвре/вибрации гироскоп/акселерометр
    // легко насыщались (клиппинг сырых данных), давая мусор в углы.
    static constexpr float GYRO_SCALE_LSB_PER_DPS = 16.4f;
    static constexpr float ACCEL_SCALE_LSB_PER_G = 2048.0f;

    // ~0.1 с подряд без ответа (при цикле ~2.5 мс) -> датчик недоступен.
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 40;

    II2CBus& i2c;
    uint8_t i2cAddress;
    bool available = false;
    bool calibrationDone = false;
    bool isMpu6500Family = false;

    int16_t rawAx = 0, rawAy = 0, rawAz = 0;
    int16_t rawGx = 0, rawGy = 0, rawGz = 0;
    int16_t rawTemp = 0;

    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

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

    uint32_t lastUpdateUs = 0;
    bool anglesInitialized = false;

    static const char* chipName(int whoAmI)
    {
        switch (whoAmI)
        {
            case 0x68: return "MPU6050";
            case 0x70: return "MPU6500";
            case 0x71: return "MPU9250";
            case 0x73: return "MPU9255";
            default:   return "неизвестный клон, работаем как с MPU6500";
        }
    }

    bool initialize()
    {
        i2c.writeRegister(i2cAddress, REG_PWR_MGMT_1, 0x80);  // DEVICE_RESET
        delay(100);

        bool ok = true;
        ok &= i2c.writeRegister(i2cAddress, REG_PWR_MGMT_1, 0x01);   // выход из sleep, такт от PLL гироскопа
        delay(10);
        ok &= i2c.writeRegister(i2cAddress, REG_GYRO_CONFIG, 0x18);  // ±2000°/сек (FS_SEL=3)
        ok &= i2c.writeRegister(i2cAddress, REG_ACCEL_CONFIG, 0x18); // ±16g (AFS_SEL=3)

        // Цифровой ФНЧ гироскопа (DLPF_CFG=3): ~42 Гц у MPU6050 / 41 Гц
        // у MPU6500, задержка ~5 мс. Срезает вибрацию мотора/винта
        // (10x5 на ~8000 об/мин даёт ~130 Гц) и при этом не добавляет
        // заметного запаздывания для стабилизации самолёта. Раньше тут
        // стоял DLPF_CFG=5 (10 Гц, ~14 мс задержки) с неверным
        // комментарием "температурная компенсация".
        ok &= i2c.writeRegister(i2cAddress, REG_CONFIG, 0x03);
        ok &= i2c.writeRegister(i2cAddress, REG_SMPLRT_DIV, 0x00);   // 1 кГц — быстрее цикла, данные всегда свежие

        // У MPU6500 фильтр акселерометра отдельный и по умолчанию
        // почти выключен (218 Гц) — ставим те же ~41 Гц.
        if (isMpu6500Family)
        {
            ok &= i2c.writeRegister(i2cAddress, REG_ACCEL_CONFIG2, 0x03);
        }

        return ok;
    }

    bool readRawData()
    {
        uint8_t b[14];  // accel XYZ, temp, gyro XYZ — big-endian
        if (!i2c.readRegisters(i2cAddress, REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

        // Раньше тут было "(read() << 8) | read()" — порядок вычисления
        // операндов "|" в C++ не определён, и компилятор вправе прочитать
        // младший байт первым. Сборка из буфера от этого не зависит.
        rawAx   = (int16_t)((b[0] << 8) | b[1]);
        rawAy   = (int16_t)((b[2] << 8) | b[3]);
        rawAz   = (int16_t)((b[4] << 8) | b[5]);
        rawTemp = (int16_t)((b[6] << 8) | b[7]);
        rawGx   = (int16_t)((b[8] << 8) | b[9]);
        rawGy   = (int16_t)((b[10] << 8) | b[11]);
        rawGz   = (int16_t)((b[12] << 8) | b[13]);
        return true;
    }

    // Поворот вектора из осей чипа в оси самолёта (X к носу, Y влево)
    // по Config::IMU_ROTATION_CW_DEG. Z (вверх) не меняется.
    static void rotateToBody(float chipX, float chipY, float& bodyX, float& bodyY)
    {
        switch (Config::IMU_ROTATION_CW_DEG)
        {
            case 90:   // ось X чипа смотрит вправо, ось Y чипа — к носу
                bodyX = chipY;   bodyY = -chipX; break;
            case 180:  // ось X чипа к хвосту
                bodyX = -chipX;  bodyY = -chipY; break;
            case 270:  // ось X чипа влево, ось Y чипа — к хвосту
                bodyX = -chipY;  bodyY = chipX;  break;
            default:   // 0: ось X чипа к носу
                bodyX = chipX;   bodyY = chipY;  break;
        }
    }

    // Калибровка (в осях чипа) -> поворот в оси самолёта (X к носу,
    // Y влево, Z вверх) -> авиационные знаки (см. заголовок): крен —
    // как есть, тангаж и рысканье — с обратным знаком.
    void applyCalibration()
    {
        const float chipAx = (rawAx - calibration.accelOffsetX) / ACCEL_SCALE_LSB_PER_G;
        const float chipAy = (rawAy - calibration.accelOffsetY) / ACCEL_SCALE_LSB_PER_G;
        const float az = (rawAz - calibration.accelOffsetZ) / ACCEL_SCALE_LSB_PER_G;

        const float chipGx = (rawGx - calibration.gyroOffsetX) / GYRO_SCALE_LSB_PER_DPS;
        const float chipGy = (rawGy - calibration.gyroOffsetY) / GYRO_SCALE_LSB_PER_DPS;
        const float gz = (rawGz - calibration.gyroOffsetZ) / GYRO_SCALE_LSB_PER_DPS;

        float ax, ay, gx, gy;
        rotateToBody(chipAx, chipAy, ax, ay);
        rotateToBody(chipGx, chipGy, gx, gy);

        // Акселерометр — в осях самолёта X к носу, Y влево, Z вверх
        // ("сырые" g), углы из него считаются в calculateAngles().
        imuData.accelX = ax;
        imuData.accelY = ay;
        imuData.accelZ = az;

        imuData.gyroX = gx;   // roll rate:  + правое крыло вниз
        imuData.gyroY = -gy;  // pitch rate: + нос вверх
        imuData.gyroZ = -gz;  // yaw rate:   + нос вправо

        imuData.temperature = isMpu6500Family
            ? rawTemp / 333.87f + 21.0f    // датащит MPU6500
            : rawTemp / 340.0f + 36.53f;   // датащит MPU6050
    }

    // Комплементарный фильтр: акселерометр даёт абсолютный угол, но
    // шумит; гироскоп даёт гладкую скорость без абсолютной опоры.
    // Обе части (прошлая оценка+гироскоп и акселерометр) взвешены
    // одним ALPHA, так что вся "память" фильтра — одно понятное число.
    //
    // ALPHA=0.98 при цикле ~2.5 мс даёт tau = dt*ALPHA/(1-ALPHA) ≈ 120 мс.
    // Требует проверки в полёте — если углы "тормозят" относительно
    // реального движения, ALPHA можно уменьшить.
    static constexpr float COMPLEMENTARY_FILTER_ALPHA = 0.98f;

    void calculateAngles()
    {
        // Углы по акселерометру (оси самолёта: X к носу, Y влево, Z вверх).
        // Нос вверх -> проекция "верха" на X положительна -> pitch > 0.
        // Правое крыло вниз -> проекция "верха" на Y (влево) > 0 -> roll > 0.
        const float accelRoll = atan2f(imuData.accelY, imuData.accelZ) * RAD_TO_DEG;
        const float accelPitch = atan2f(imuData.accelX,
                                        sqrtf(imuData.accelY * imuData.accelY +
                                              imuData.accelZ * imuData.accelZ)) * RAD_TO_DEG;

        const uint32_t now = micros();
        const float dt = (now - lastUpdateUs) / 1000000.0f;
        lastUpdateUs = now;

        // Первый отсчёт (или после долгой паузы/калибровки): стартуем
        // сразу с угла по акселерометру, а не с нуля — иначе фильтр
        // ~0.5 с "доезжает" до реального угла.
        if (!anglesInitialized || dt <= 0 || dt > 0.1f)
        {
            if (!anglesInitialized)
            {
                imuData.roll = accelRoll;
                imuData.pitch = accelPitch;
                anglesInitialized = true;
            }
            return;
        }

        imuData.roll = COMPLEMENTARY_FILTER_ALPHA * (imuData.roll + imuData.gyroX * dt) +
                       (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accelRoll;

        imuData.pitch = COMPLEMENTARY_FILTER_ALPHA * (imuData.pitch + imuData.gyroY * dt) +
                        (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;

        yawIntegral = wrap180(yawIntegral + imuData.gyroZ * dt);
        imuData.yaw = yawIntegral;

        imuData.roll = wrap180(imuData.roll);
        imuData.pitch = wrap180(imuData.pitch);
    }

    static float wrap180(float angle)
    {
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
};
