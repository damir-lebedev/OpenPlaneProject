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

    bool isAvailable() const override;

    void update() override;

    const ImuData& getImuData() const override;

    // Калибровка гироскопа и предполётная проверка: самолёт должен
    // стоять неподвижно ~2 с. Ровно — только пока установка не
    // откалибрована (тогда горизонт = положение сейчас).
    void calibrate() override;

    // Установка платы в самолёте, 3 позы (см. ImuOrientation.h).
    // Блокирует до ~1.5 мин, пока пилот ставит самолёт.
    void calibrateOrientation() override;

    const char* getPreflightProblem() const override;

    void setYaw(float yawDegrees) override;

    const char* getSensorType() const override;

    void printStatus() const override;


protected:

    // nvsNamespace — своё пространство NVS у каждого драйвера: оси
    // разных чипов не обязаны совпадать.
    ImuSensorBase(const char* sensorName, const char* nvsName);

    // --- то, что реализует драйвер конкретного чипа ---

    // Один отсчёт в осях чипа. false — чип не ответил.
    virtual bool readSample(RawImuSample& sample) = 0;

    virtual float accelLsbPerG() const = 0;
    virtual float gyroLsbPerDps() const = 0;
    virtual float temperatureC(int16_t raw) const = 0;

    // Вызывает begin() драйвера: чип опознан и настроен (или нет).
    void setAvailable(bool isAvailable);

    // Имя чипа для лога — драйвер может уточнить после опознания
    // (MPU6050 vs MPU6500).
    void setName(const char* chipName);


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

    void loadOrientation();

    void runPreflightCheck(float gyroNoiseDps, const float chipUp[3]);

    void printPreflightResult() const;

    // Ждёт позу и возвращает средний "верх" (g, оси чипа) за ~1 с
    // неподвижности. differFromA/B — прошлые позы, от которых новая
    // должна отличаться хотя бы на POSE_MIN_CHANGE_DEG (иначе засчитался
    // бы прошлый шаг, пока пилот ещё не наклонил самолёт).
    bool capturePose(const float* differFromA, const float* differFromB, float out[3]);

    static bool farFrom(const float up[3], const float* previous);

    void process(const RawImuSample& s);
};
