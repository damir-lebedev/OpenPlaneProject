#pragma once
#include <Arduino.h>

#include "config/Config.h"
#include "sensors/SensorInterface.h"
#include "sensors/imu/AttitudeEstimator.h"
#include "sensors/imu/ImuOrientation.h"

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
//     (ImuOrientation) -> авиационные знаки
//     -> AttitudeEstimator (крен/тангаж/рысканье).
//
// Оси чипа (у MPU6050/6500 и ICM42688 одинаковые): X, Y в
// плоскости платы, Z вверх из микросхемы — правая тройка. После
// поворота это оси самолёта X к носу, Y влево, Z вверх, и в
// авиационных знаках крен = вращение вокруг X как есть, а тангаж и
// рысканье — с обратным знаком (Y влево, Z вверх).
//
// Установка платы в самолёте:
//   • откалибрована (команда 'o', calibrateOrientation()) — плата
//     может стоять как угодно; поворот и горизонт хранятся в NVS;
//   • нет — поворот из Config::IMU_ROTATION_CW_DEG, плата обязана
//     лежать чипом вверх, горизонт — положение при включении.
//
// Предполётная проверка при каждой калибровке гироскопа (включение,
// команда 'i'): самолёт неподвижен, акселерометр видит 1g, "верх"
// совпадает с установкой. Не прошла — getPreflightProblem().
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

    // Калибровка гироскопа и предполётная проверка: самолёт должен
    // стоять неподвижно ~2 с. Ровно — только пока установка не
    // откалибрована (тогда горизонт = положение сейчас).
    void calibrate() override
    {
        if (!available) return;

        Serial.print(name);
        Serial.println(": калибровка гироскопа...");

        float sum[6] = {};
        float gyroSquares[3] = {};
        int samples = 0;

        for (int i = 0; i < CALIBRATION_SAMPLES; i++)
        {
            RawImuSample s;
            if (readSample(s))
            {
                const int16_t gyro[3] = { s.gyroX, s.gyroY, s.gyroZ };
                for (uint8_t axis = 0; axis < 3; ++axis)
                {
                    sum[axis] += gyro[axis];
                    gyroSquares[axis] += float(gyro[axis]) * gyro[axis];
                }
                sum[3] += s.accelX; sum[4] += s.accelY; sum[5] += s.accelZ;
                samples++;
            }
            delay(10);
        }

        if (samples < CALIBRATION_SAMPLES / 2)
        {
            Serial.print(name);
            Serial.println(": калибровка не удалась — датчик не отвечает");
            preflightProblem = PreflightProblem::NotResponding;
            return;
        }

        // Шум гироскопа за время калибровки — двигали ли самолёт.
        float gyroNoiseDps = 0;
        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            gyroOffset[axis] = sum[axis] / samples;
            const float variance = gyroSquares[axis] / samples - gyroOffset[axis] * gyroOffset[axis];
            gyroNoiseDps = max(gyroNoiseDps, sqrtf(max(variance, 0.0f)) / gyroLsbPerDps());
        }

        const float accelScale = accelLsbPerG();
        const float chipUp[3] = { sum[3] / samples / accelScale, sum[4] / samples / accelScale,
                                  sum[5] / samples / accelScale };

        if (orientationCalibrated)
        {
            // Горизонт — из калибровки установки, смещение нуля
            // акселерометра уже учтено в ней.
            accelOffset[0] = accelOffset[1] = accelOffset[2] = 0;
        }
        else
        {
            // Горизонт = положение сейчас: всё, что сверх (0, 0, 1g)
            // (плата лежит микросхемой вверх).
            accelOffset[0] = sum[3] / samples;
            accelOffset[1] = sum[4] / samples;
            accelOffset[2] = sum[5] / samples - accelScale;
        }

        calibrated = true;
        estimator.reset();
        runPreflightCheck(gyroNoiseDps, chipUp);

        Serial.print(name);
        Serial.print(": калибровка завершена, смещение гироскопа=");
        Serial.print(gyroOffset[0]); Serial.print(",");
        Serial.print(gyroOffset[1]); Serial.print(",");
        Serial.print(gyroOffset[2]);
        Serial.print(", шум=");
        Serial.print(gyroNoiseDps, 2);
        Serial.println(" °/с");
        printPreflightResult();
    }

    // Установка платы в самолёте, 3 позы (см. ImuOrientation.h).
    // Блокирует до ~1.5 мин, пока пилот ставит самолёт.
    void calibrateOrientation() override
    {
        if (!available)
        {
            Serial.print(name);
            Serial.println(": недоступен, калибровка установки невозможна");
            return;
        }

        Serial.print(name);
        Serial.println(": калибровка установки — плата может стоять в самолёте как угодно.");
        Serial.println("  В каждом шаге держите самолёт неподвижно ~1 с, шаг засчитается сам.");

        float level[3], noseUp[3], rightWingDown[3];

        Serial.println("  Шаг 1/3: поставьте самолёт ровно, как в горизонтальном полёте.");
        delay(POSE_SETTLE_MS);
        if (!capturePose(nullptr, nullptr, level)) return;

        Serial.println("  Шаг 2/3: поднимите НОС на 30-60° (крылья ровно) и держите.");
        if (!capturePose(level, nullptr, noseUp)) return;

        Serial.println("  Шаг 3/3: нос обратно, опустите ПРАВОЕ крыло на 30-60° и держите.");
        if (!capturePose(level, noseUp, rightWingDown)) return;

        ImuOrientation result;
        const char* error = ImuOrientation::fromPoses(level, noseUp, rightWingDown, result);
        if (error)
        {
            Serial.print(name);
            Serial.print(": калибровка установки отклонена — ");
            Serial.println(error);
            return;
        }

        orientation = result;
        orientationCalibrated = true;
        orientation.save(nvsNamespace);
        accelOffset[0] = accelOffset[1] = accelOffset[2] = 0;
        estimator.reset();
        // Установка только что измерена — проблемы с ней сняты; шум
        // гироскопа при включении — нет (калибровку гироскопа это не
        // повторяет).
        if (preflightProblem == PreflightProblem::MountingMismatch ||
            preflightProblem == PreflightProblem::NotChipUp)
        {
            preflightProblem = PreflightProblem::None;
        }

        Serial.print(name);
        Serial.print(": установка сохранена: ");
        orientation.describe(Serial);
        Serial.println();
        Serial.println("  Проверка: нос вверх -> P растёт, правое крыло вниз -> R растёт.");
    }

    const char* getPreflightProblem() const override
    {
        switch (preflightProblem)
        {
            case PreflightProblem::None:
                return nullptr;
            case PreflightProblem::NotResponding:
                return "IMU: калибровка не удалась, датчик не отвечает";
            case PreflightProblem::Moved:
                return "IMU: самолёт двигали во время калибровки гироскопа — "
                       "поставьте неподвижно и перезагрузите (или 'i')";
            case PreflightProblem::NotOneG:
                return "IMU: акселерометр в покое показывает не 1g";
            case PreflightProblem::MountingMismatch:
                return "IMU: \"верх\" не совпадает с калибровкой установки "
                       "(плату переставили?) — откалибруйте заново, 'o'";
            case PreflightProblem::NotChipUp:
                return "IMU: плата не лежит чипом вверх, а установка не "
                       "откалибрована — команда 'o'";
        }
        return nullptr;
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
        Serial.print(" mounting="); Serial.print(orientationCalibrated ? "CALIBRATED" : "CONFIG");
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
        Serial.print(" temp="); Serial.print(imuData.temperature, 1);
        Serial.print(" preflight=");
        Serial.println(getPreflightProblem() ? getPreflightProblem() : "OK");
    }


protected:

    // nvsNamespace — своё пространство NVS у каждого драйвера: оси
    // разных чипов не обязаны совпадать.
    ImuSensorBase(const char* name, const char* nvsNamespace)
        : name(name),
          nvsNamespace(nvsNamespace)
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
        if (available) loadOrientation();
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

    // Предполётная проверка. Шум гироскопа неподвижной платы ~0.05-0.1
    // °/с; самолёт в руках или стол, который трогают, — больше 0.5.
    static constexpr float STILL_GYRO_NOISE_DPS = 0.5f;
    static constexpr float ONE_G_TOLERANCE = 0.2f;
    // "Верх" при включении дальше этого от сохранённого — плату
    // переставили (самолёт на хвостовом колесе или на склоне — ~15°).
    static constexpr float MAX_BOOT_TILT_DEG = 45.0f;

    // Калибровка установки: поза засчитывается после ~1 с неподвижности.
    static constexpr uint32_t POSE_SETTLE_MS = 2000;
    static constexpr uint32_t POSE_TIMEOUT_MS = 30000;
    static constexpr int POSE_SAMPLES = 100;           // × 10 мс
    static constexpr float POSE_STILL_DPS = 3.0f;
    static constexpr float POSE_MIN_CHANGE_DEG = 20.0f;

    const char* name;
    const char* nvsNamespace;
    bool available = false;
    bool calibrated = false;

    ImuOrientation orientation;
    bool orientationCalibrated = false;
    enum class PreflightProblem : uint8_t
    {
        None, NotResponding, Moved, NotOneG, MountingMismatch, NotChipUp
    };
    PreflightProblem preflightProblem = PreflightProblem::None;

    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    float gyroOffset[3] = {};
    float accelOffset[3] = {};

    AttitudeEstimator estimator;
    ImuData imuData;

    void loadOrientation()
    {
        orientationCalibrated = orientation.load(nvsNamespace);
        if (!orientationCalibrated)
        {
            orientation = ImuOrientation::fromYawSteps(Config::IMU_ROTATION_CW_DEG);
        }

        Serial.print(name);
        if (orientationCalibrated)
        {
            Serial.print(": установка из калибровки: ");
            orientation.describe(Serial);
            Serial.println();
        }
        else
        {
            Serial.print(": установка из Config (поворот ");
            Serial.print(Config::IMU_ROTATION_CW_DEG);
            Serial.println("°, плата чипом вверх); поставить как угодно — команда 'o'");
        }
    }

    void runPreflightCheck(float gyroNoiseDps, const float chipUp[3])
    {
        preflightProblem = PreflightProblem::None;

        if (gyroNoiseDps > STILL_GYRO_NOISE_DPS)
        {
            preflightProblem = PreflightProblem::Moved;
            return;
        }

        const float g = sqrtf(chipUp[0] * chipUp[0] + chipUp[1] * chipUp[1] + chipUp[2] * chipUp[2]);
        if (fabsf(g - 1.0f) > ONE_G_TOLERANCE)
        {
            preflightProblem = PreflightProblem::NotOneG;
            return;
        }

        if (orientationCalibrated)
        {
            if (orientation.tiltFromLevelDeg(chipUp) > MAX_BOOT_TILT_DEG)
            {
                preflightProblem = PreflightProblem::MountingMismatch;
            }
        }
        else if (chipUp[2] < 0.5f * g)
        {
            // Без калибровки установки горизонт берётся "чип вверх" —
            // перевёрнутая или стоящая на боку плата дала бы углы с
            // перепутанными знаками.
            preflightProblem = PreflightProblem::NotChipUp;
        }
    }

    void printPreflightResult() const
    {
        Serial.print(name);
        if (getPreflightProblem())
        {
            Serial.print(": ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — ");
            Serial.println(getPreflightProblem());
        }
        else
        {
            Serial.println(": предполётная проверка пройдена");
        }
    }

    // Ждёт позу и возвращает средний "верх" (g, оси чипа) за ~1 с
    // неподвижности. differFromA/B — прошлые позы, от которых новая
    // должна отличаться хотя бы на POSE_MIN_CHANGE_DEG (иначе засчитался
    // бы прошлый шаг, пока пилот ещё не наклонил самолёт).
    bool capturePose(const float* differFromA, const float* differFromB, float out[3])
    {
        const float accelScale = accelLsbPerG();
        const float gyroScale = gyroLsbPerDps();
        const uint32_t start = millis();

        float sum[3] = {};
        int count = 0;

        while (millis() - start < POSE_TIMEOUT_MS)
        {
            delay(10);

            RawImuSample s;
            if (!readSample(s)) continue;

            const float up[3] = { s.accelX / accelScale, s.accelY / accelScale, s.accelZ / accelScale };
            const float rate = max(max(fabsf((s.gyroX - gyroOffset[0]) / gyroScale),
                                       fabsf((s.gyroY - gyroOffset[1]) / gyroScale)),
                                   fabsf((s.gyroZ - gyroOffset[2]) / gyroScale));
            const float g = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);

            const bool still = rate < POSE_STILL_DPS && fabsf(g - 1.0f) < ONE_G_TOLERANCE;
            const bool changed = farFrom(up, differFromA) && farFrom(up, differFromB);
            if (!still || !changed)
            {
                count = 0;
                sum[0] = sum[1] = sum[2] = 0;
                continue;
            }

            for (uint8_t i = 0; i < 3; ++i) sum[i] += up[i];
            if (++count >= POSE_SAMPLES)
            {
                for (uint8_t i = 0; i < 3; ++i) out[i] = sum[i] / count;
                Serial.println("    засчитано");
                return true;
            }
        }

        Serial.print(name);
        Serial.println(": поза не дождалась за 30 с — калибровка установки отменена");
        return false;
    }

    static bool farFrom(const float up[3], const float* previous)
    {
        if (!previous) return true;
        const float dotProduct = up[0] * previous[0] + up[1] * previous[1] + up[2] * previous[2];
        const float lengths = sqrtf((up[0] * up[0] + up[1] * up[1] + up[2] * up[2]) *
                                    (previous[0] * previous[0] + previous[1] * previous[1] +
                                     previous[2] * previous[2]));
        if (lengths < 1e-6f) return false;
        const float angle = acosf(constrain(dotProduct / lengths, -1.0f, 1.0f)) * RAD_TO_DEG;
        return angle >= POSE_MIN_CHANGE_DEG;
    }

    void process(const RawImuSample& s)
    {
        const float accelScale = accelLsbPerG();
        const float gyroScale = gyroLsbPerDps();

        // Калибровка и масштаб — в осях чипа.
        const float chipAccel[3] = { (s.accelX - accelOffset[0]) / accelScale,
                                     (s.accelY - accelOffset[1]) / accelScale,
                                     (s.accelZ - accelOffset[2]) / accelScale };
        const float chipGyro[3] = { (s.gyroX - gyroOffset[0]) / gyroScale,
                                    (s.gyroY - gyroOffset[1]) / gyroScale,
                                    (s.gyroZ - gyroOffset[2]) / gyroScale };

        // Оси чипа -> оси самолёта (X к носу, Y влево, Z вверх).
        float accel[3], gyro[3];
        orientation.apply(chipAccel, accel);
        orientation.apply(chipGyro, gyro);

        imuData.accelX = accel[0];
        imuData.accelY = accel[1];
        imuData.accelZ = accel[2];

        // Авиационные знаки угловых скоростей.
        imuData.gyroX = gyro[0];    // крен:     + правое крыло вниз
        imuData.gyroY = -gyro[1];   // тангаж:   + нос вверх
        imuData.gyroZ = -gyro[2];   // рысканье: + нос вправо

        imuData.temperature = temperatureC(s.temperature);

        const uint32_t now = micros();
        estimator.update(accel[0], accel[1], accel[2], imuData.gyroX, imuData.gyroY, imuData.gyroZ, now);

        imuData.roll = estimator.getRoll();
        imuData.pitch = estimator.getPitch();
        imuData.yaw = estimator.getYaw();
        imuData.timestamp = now;
    }
};
