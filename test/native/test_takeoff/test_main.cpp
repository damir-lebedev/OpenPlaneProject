// ============================================================
// АВТОВЗЛЁТ С ХВОСТОВЫМ И НОСОВЫМ КОЛЕСОМ, НАЗЕМНАЯ КАЛИБРОВКА, СИГНАЛЫ
//
//   • ImuOrientation::withLevel()/attitudeOf() — горизонт по позе
//     "хвост поднят", стоянка;
//   • AttitudeEstimator без акселерометра — разгон не задирает нос;
//   • ImuSensorBase — калибровка шасси во флеше, горизонт после
//     включения на колёсах, 'o' стирает стоянку, уточнение нуля гироскопа;
//   • GroundCalibration — два шага, проверки, жест тумблером;
//   • SignalPlayer — кивки, пауза импульсов ESC, счёт;
//   • TakeoffController через Autopilot — этапы, законы рулей, защиты,
//     потеря связи на полосе, окно "только гироскоп".
// Полёты целиком (с моделью шасси) — test_sim.
//
// Запуск: pio test -e native -f native/test_takeoff
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <math.h>

#include "autopilot/Autopilot.h"
#include "control/GroundCalibration.h"
#include "control/SignalPlayer.h"
#include "sensors/imu/AttitudeEstimator.h"
#include "sensors/imu/ImuOrientation.h"
#include "sensors/imu/ImuSensorBase.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    constexpr float DEG = 0.017453293f;
    using Stage = TakeoffController::Stage;

    // "Верх" (g) в осях самолёта при тангаже/крене — то, что показал бы
    // акселерометр в покое.
    void bodyUp(float pitchDeg, float rollDeg, float up[3])
    {
        up[0] = sinf(pitchDeg * DEG);
        up[1] = cosf(pitchDeg * DEG) * sinf(rollDeg * DEG);
        up[2] = cosf(pitchDeg * DEG) * cosf(rollDeg * DEG);
    }

    // Самолёт стоит с этими углами (FakeImu: углы + акселерометр в покое).
    void setAttitude(FakeImu& imu, float pitchDeg, float rollDeg = 0.0f)
    {
        float up[3];
        bodyUp(pitchDeg, rollDeg, up);
        imu.data.pitch = pitchDeg;
        imu.data.roll = rollDeg;
        imu.data.accelX = up[0];
        imu.data.accelY = up[1];
        imu.data.accelZ = up[2];
        // Установка FakeImu — тождественная: "верх" чипа = "верх" самолёта.
        for (int i = 0; i < 3; ++i) imu.chipUp[i] = up[i];
    }

    struct TakeoffRig
    {
        FakeImu imu;
        FakeBaro baro;
        FakeAirspeed airspeed;
        Autopilot autopilot;
        uint16_t throttle = 1000;
        bool armed = true;
        bool linkLost = false;

        explicit TakeoffRig(Config::LandingGear gear = Config::LandingGear::TAIL_WHEEL, bool withAirspeed = false)
            : autopilot(&imu, &baro, nullptr, nullptr, withAirspeed ? &airspeed : nullptr)
        {
            autopilot.setLandingGear(gear);
            autopilot.setMode(MODE_AUTO_TAKEOFF);
            if (gear == Config::LandingGear::TAIL_WHEEL)
            {
                imu.park(10.0f);
                setAttitude(imu, 10.0f);
            }
            else
            {
                setAttitude(imu, 0.0f);
            }
        }

        void run(uint32_t ms)
        {
            for (uint32_t t = 0; t < ms; t += 10)
            {
                fake::advanceMs(10);
                autopilot.update(armed, linkLost, throttle);
            }
        }

        // Газ внизу, затем вверх и проверка до старта мотора.
        void start()
        {
            throttle = 1000;
            run(20);
            throttle = 1600;
            run(Config::TAKEOFF_CHECK_MS + 50);
        }

        Stage stage() const { return autopilot.getTakeoffStage(); }
    };

    // Настоящий ImuSensorBase с "чипом", который отдаёт заданный отсчёт —
    // проверяется хранение калибровки, горизонт и ноль гироскопа.
    class ScriptedImu : public ImuSensorBase
    {
    public:
        static constexpr float ACCEL_LSB = 4096.0f;
        static constexpr float GYRO_LSB = 65.5f;

        ScriptedImu() : ImuSensorBase("SCRIPTED", "imu_test") {}

        // Поза по времени (millis): калибровка установки 'o' ведёт самолёт
        // по позам сама.
        void (*poseAt)(uint32_t ms, float up[3]) = nullptr;
        float up[3] = { 0.0f, 0.0f, 1.0f };
        float gyroDps[3] = {};   // в осях чипа

        bool begin() override
        {
            setAvailable(true);
            return true;
        }

    protected:
        bool readSample(RawImuSample& s) override
        {
            float u[3] = { up[0], up[1], up[2] };
            if (poseAt) poseAt(millis(), u);
            s.accelX = static_cast<int16_t>(lroundf(u[0] * ACCEL_LSB));
            s.accelY = static_cast<int16_t>(lroundf(u[1] * ACCEL_LSB));
            s.accelZ = static_cast<int16_t>(lroundf(u[2] * ACCEL_LSB));
            s.gyroX = static_cast<int16_t>(lroundf(gyroDps[0] * GYRO_LSB));
            s.gyroY = static_cast<int16_t>(lroundf(gyroDps[1] * GYRO_LSB));
            s.gyroZ = static_cast<int16_t>(lroundf(gyroDps[2] * GYRO_LSB));
            s.temperature = 0;
            return true;
        }

        float accelLsbPerG() const override { return ACCEL_LSB; }
        float gyroLsbPerDps() const override { return GYRO_LSB; }
        float temperatureC(int16_t) const override { return 25.0f; }
    };

    // Плата наклонена в самолёте на +6° по тангажу: "верх" чипа при
    // тангаже самолёта pitchDeg. mounting — как оси чипа повёрнуты
    // вокруг вертикали (у ScriptedImu без калибровки — Config).
    void chipUpWithTiltedBoard(float pitchDeg, float up[3],
                               const ImuOrientation& mounting = ImuOrientation::fromYawSteps(0))
    {
        float board[3];
        bodyUp(pitchDeg + 6.0f, 0.0f, board);
        mounting.applyInverse(board, up);
    }

    const ImuOrientation& configMounting()
    {
        static const ImuOrientation m = ImuOrientation::fromYawSteps(Config::IMU_ROTATION_CW_DEG);
        return m;
    }

    void updateFor(ImuSensorBase& imu, uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 2)
        {
            fake::advanceMs(2);
            imu.update();
        }
    }
}

// ------------------------------------------------------------
// Горизонт и стоянка: ImuOrientation
// ------------------------------------------------------------

void test_level_pose_refines_horizon_and_keeps_nose()
{
    const ImuOrientation base = ImuOrientation::fromYawSteps(0);
    float levelUp[3], parkUp[3];
    chipUpWithTiltedBoard(0.0f, levelUp);    // хвост поднят, фюзеляж горизонтально
    chipUpWithTiltedBoard(10.0f, parkUp);    // на колёсах — нос вверх на 10°

    float pitch, roll;
    base.attitudeOf(levelUp, pitch, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.0f, pitch);   // без поправки — наклон платы

    ImuOrientation level;
    TEST_ASSERT_NULL(base.withLevel(levelUp, level));
    level.attitudeOf(levelUp, pitch, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    level.attitudeOf(parkUp, pitch, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, pitch);   // стояночный угол — от нового горизонта

    // Нос остался носом: X самолёта в осях чипа почти тот же.
    const float chipX[3] = { 1.0f, 0.0f, 0.0f };
    float body[3];
    level.apply(chipX, body);
    TEST_ASSERT_GREATER_THAN_FLOAT(cosf(7.0f * DEG), body[0]);

    // Обратный поворот возвращает исходный вектор.
    float back[3];
    level.applyInverse(body, back);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, back[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, back[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, back[2]);

    // Больше 45° — это не "почти ровно".
    float tilted[3];
    bodyUp(50.0f, 0.0f, tilted);
    ImuOrientation unused;
    TEST_ASSERT_NOT_NULL(base.withLevel(tilted, unused));
    const float zero[3] = {};
    TEST_ASSERT_NOT_NULL(base.withLevel(zero, unused));
}

void test_estimator_without_accelerometer_ignores_runway_acceleration()
{
    AttitudeEstimator estimator;
    uint32_t t = 0;
    auto feed = [&](float ax, float az, float pitchRate, int steps) {
        for (int i = 0; i < steps; ++i)
        {
            t += 2000;
            estimator.update(ax, 0.0f, az, 0.0f, pitchRate, 0.0f, t);
        }
    };
    const float s10 = sinf(10.0f * DEG), c10 = cosf(10.0f * DEG);
    feed(s10, c10, 0.0f, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, estimator.getPitch());

    // Разгон 0.6 g: с акселерометром нос "задирается" за доли секунды.
    AttitudeEstimator withAccel = estimator;
    for (int i = 0; i < 250; ++i)
    {
        t += 2000;
        withAccel.update(s10 + 0.6f, 0.0f, c10, 0.0f, 0.0f, 0.0f, t);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(30.0f, withAccel.getPitch());

    // Без акселерометра — только гироскоп: тот же разгон угол не трогает,
    // а настоящий поворот (хвост поднимается) виден.
    estimator.setAccelCorrection(false);
    TEST_ASSERT_FALSE(estimator.isAccelCorrectionEnabled());
    t -= 500000;   // тот же отрезок времени
    feed(s10 + 0.6f, c10, 0.0f, 250);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, estimator.getPitch());
    feed(s10 + 0.6f, c10, -8.0f, 500);   // −8°/с за 1 с
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, estimator.getPitch());

    // Включили обратно — подтягивается к акселерометру.
    estimator.setAccelCorrection(true);
    feed(s10, c10, 0.0f, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, estimator.getPitch());
}

// ------------------------------------------------------------
// ImuSensorBase: калибровка шасси во флеше
// ------------------------------------------------------------

void test_imu_stores_ground_calibration_and_uses_it_after_power_on()
{
    float levelUp[3], parkUp[3];
    chipUpWithTiltedBoard(0.0f, levelUp, configMounting());
    chipUpWithTiltedBoard(10.0f, parkUp, configMounting());

    {
        // Включили, держа самолёт почти стоймя: без калибровки установки
        // "чип вверх" не подтверждён — предполётная проверка не пройдена.
        ScriptedImu imu;
        bodyUp(70.0f, 0.0f, imu.up);
        imu.begin();
        TEST_ASSERT_TRUE(contains(takeSerial(), "калибровки шасси нет"));
        imu.calibrate();
        TEST_ASSERT_NOT_NULL(imu.getPreflightProblem());
        float p, r;
        TEST_ASSERT_FALSE(imu.getParkingAttitude(p, r));

        ImuOrientation current, level;
        TEST_ASSERT_TRUE(imu.getOrientation(current));
        TEST_ASSERT_NULL(current.withLevel(levelUp, level));
        TEST_ASSERT_TRUE(imu.setGroundCalibration(level, parkUp));
        // Горизонт только что измерен — проблема установки снята.
        TEST_ASSERT_NULL(imu.getPreflightProblem());
        TEST_ASSERT_TRUE(imu.getParkingAttitude(p, r));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, p);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, r);
    }

    // Включили снова, стоя на колёсах: горизонт — из калибровки, а не
    // "как стоит" — тангаж честные +10°.
    ScriptedImu imu;
    for (int i = 0; i < 3; ++i) imu.up[i] = parkUp[i];
    imu.begin();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "стоянка на колёсах — тангаж 10.0"), log.c_str());
    imu.calibrate();
    TEST_ASSERT_NULL(imu.getPreflightProblem());
    float readUp[3];
    updateFor(imu, 200);
    TEST_ASSERT_TRUE(imu.readChipUp(readUp));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, parkUp[0], readUp[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, imu.getImuData().pitch);

    imu.printStatus();
    TEST_ASSERT_TRUE(contains(takeSerial(), "park(pitch,roll)=10.0"));
}

// 'o' задаёт свой горизонт — стоянка от прежнего стирается.
uint32_t mountingPosesStartMs = 0;

void posesForMounting(uint32_t ms, float up[3])
{
    const uint32_t t = ms - mountingPosesStartMs;
    if (t < 4000) bodyUp(0.0f, 0.0f, up);
    else if (t < 8000) bodyUp(45.0f, 0.0f, up);
    else bodyUp(0.0f, 45.0f, up);
}

void test_mounting_calibration_forgets_parking()
{
    ScriptedImu imu;
    imu.begin();
    float levelUp[3], parkUp[3];
    bodyUp(0.0f, 0.0f, levelUp);
    bodyUp(10.0f, 0.0f, parkUp);
    ImuOrientation current, level;
    imu.getOrientation(current);
    current.withLevel(levelUp, level);
    imu.setGroundCalibration(level, parkUp);
    takeSerial();

    mountingPosesStartMs = millis();
    imu.poseAt = posesForMounting;
    imu.calibrateOrientation();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE_MESSAGE(contains(log, "установка сохранена"), log.c_str());
    TEST_ASSERT_TRUE(contains(log, "Калибровка шасси сброшена"));
    float p, r;
    TEST_ASSERT_FALSE(imu.getParkingAttitude(p, r));

    ScriptedImu again;
    again.begin();
    TEST_ASSERT_FALSE(again.getParkingAttitude(p, r));   // и во флеше тоже
}

void test_gyro_bias_trim_removes_residual()
{
    ScriptedImu imu;
    imu.begin();
    imu.calibrate();
    // Гироскоп "уплыл" после калибровки при включении.
    imu.gyroDps[0] = 0.5f;
    imu.gyroDps[1] = 0.2f;
    imu.gyroDps[2] = -0.3f;
    updateFor(imu, 20);
    const ImuData before = imu.getImuData();
    const float drift = sqrtf(before.gyroX * before.gyroX + before.gyroY * before.gyroY + before.gyroZ * before.gyroZ);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, sqrtf(0.25f + 0.04f + 0.09f), drift);

    const float residual[3] = { before.gyroX, before.gyroY, before.gyroZ };
    imu.trimGyroBias(residual);
    updateFor(imu, 20);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, imu.getImuData().gyroX);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, imu.getImuData().gyroY);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, imu.getImuData().gyroZ);
}

// ------------------------------------------------------------
// GroundCalibration
// ------------------------------------------------------------

namespace
{
    // Шаг калибровки: signalMs — звучит сигнал, затем ms тишины.
    void calibrationTicks(GroundCalibration& cal, FakeImu& imu, uint32_t signalMs, uint32_t ms)
    {
        for (uint32_t t = 0; t < signalMs + ms; t += 10)
        {
            fake::advanceMs(10);
            cal.update(&imu, false, t < signalMs, millis());
        }
    }

    // Самолёт переставляют: вращение.
    void moveIt(GroundCalibration& cal, FakeImu& imu)
    {
        imu.data.gyroY = 30.0f;
        calibrationTicks(cal, imu, 0, 200);
        imu.data.gyroY = 0.0f;
    }
}

void test_ground_calibration_tail_wheel_two_steps()
{
    FakeImu imu;
    GroundCalibration cal(Config::LandingGear::TAIL_WHEEL);
    setAttitude(imu, 0.0f);   // шаг 1: хвост поднят — горизонтально

    TEST_ASSERT_TRUE(cal.start(&imu, millis(), "тест"));
    TEST_ASSERT_TRUE(contains(takeSerial(), "поднимите хвост"));
    TEST_ASSERT_EQUAL_UINT8(1, cal.takeSignal());
    TEST_ASSERT_EQUAL_UINT8(0, cal.takeSignal());
    TEST_ASSERT_TRUE(cal.isActive());

    // Пока сигнал звучит — не засчитывается.
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, 0);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::LEVEL, cal.getStage());
    calibrationTicks(cal, imu, 0, Config::CAL_STILL_MS + 20);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::PARK, cal.getStage());
    TEST_ASSERT_EQUAL_UINT8(1, cal.takeSignal());
    TEST_ASSERT_EQUAL(0u, imu.groundCalibrationsSaved);   // ещё ничего не сохранено

    // Шаг 2: хвост опустили — нос вверх на 9°, крен 1°.
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, 0);
    moveIt(cal, imu);
    setAttitude(imu, 9.0f, 1.0f);
    calibrationTicks(cal, imu, 0, Config::CAL_STILL_MS + 20);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::DONE, cal.getStage());
    TEST_ASSERT_EQUAL_UINT8(2, cal.takeSignal());
    TEST_ASSERT_EQUAL(1u, imu.groundCalibrationsSaved);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 9.0f, imu.parkingPitch);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, imu.parkingRoll);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 9.0f, cal.getParkPitch());
    TEST_ASSERT_TRUE(contains(takeSerial(), "нос вверх на 9.0°"));
    TEST_ASSERT_FALSE(cal.isActive());
}

void test_ground_calibration_rejects_tail_not_lowered()
{
    FakeImu imu;
    GroundCalibration cal(Config::LandingGear::TAIL_WHEEL);
    setAttitude(imu, 0.0f);
    cal.start(&imu, millis(), "тест");
    cal.takeSignal();
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_STILL_MS + 20);
    cal.takeSignal();
    // Хвост так и держат поднятым: шаг 2 ждёт CAL_PARK_WAIT_MS и меряет то же.
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_PARK_WAIT_MS + Config::CAL_STILL_MS + 50);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, cal.getStage());
    TEST_ASSERT_EQUAL_UINT8(3, cal.takeSignal());
    TEST_ASSERT_EQUAL(0u, imu.groundCalibrationsSaved);
    TEST_ASSERT_TRUE(contains(cal.getFailure(), "хвост не опустили"));
    TEST_ASSERT_TRUE(contains(takeSerial(), "Ничего не сохранено"));
}

void test_ground_calibration_checks_parking_angles()
{
    struct Case { Config::LandingGear gear; float pitch, roll; const char* failure; };
    const Case cases[] = {
        { Config::LandingGear::TAIL_WHEEL, 30.0f, 0.0f, "больше CAL_TAIL_PARK_MAX_DEG" },
        { Config::LandingGear::TAIL_WHEEL, 10.0f, 12.0f, "большой крен" },
        { Config::LandingGear::NOSE_WHEEL, 9.0f, 0.0f, "LANDING_GEAR" },
        { Config::LandingGear::NOSE_WHEEL, 1.5f, 0.0f, nullptr },
    };
    for (const Case& c : cases)
    {
        FakeImu imu;
        GroundCalibration cal(c.gear);
        setAttitude(imu, 0.0f);
        cal.start(&imu, millis(), "тест");
        calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_STILL_MS + 20);
        TEST_ASSERT_EQUAL(GroundCalibration::Stage::PARK, cal.getStage());
        calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, 0);
        moveIt(cal, imu);
        setAttitude(imu, c.pitch, c.roll);
        calibrationTicks(cal, imu, 0, Config::CAL_STILL_MS + 20);
        if (c.failure)
        {
            TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, cal.getStage());
            TEST_ASSERT_TRUE_MESSAGE(contains(cal.getFailure(), c.failure), cal.getFailure());
        }
        else
        {
            TEST_ASSERT_EQUAL(GroundCalibration::Stage::DONE, cal.getStage());
        }
    }
}

void test_ground_calibration_nose_wheel_same_pose_after_wait()
{
    FakeImu imu;
    GroundCalibration cal(Config::LandingGear::NOSE_WHEEL);
    setAttitude(imu, 0.5f);
    cal.start(&imu, millis(), "тест");
    TEST_ASSERT_TRUE(contains(takeSerial(), "поставьте самолёт ровно"));
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_STILL_MS + 20);
    // Самолёт не трогали: шаг 2 всё равно засчитается после ожидания.
    calibrationTicks(cal, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_PARK_WAIT_MS - 100);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::PARK, cal.getStage());
    calibrationTicks(cal, imu, 0, Config::CAL_STILL_MS + 200);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::DONE, cal.getStage());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, imu.parkingPitch);
}

void test_ground_calibration_cancel_timeout_and_missing_imu()
{
    FakeImu imu;
    setAttitude(imu, 0.0f);

    GroundCalibration noImu;
    TEST_ASSERT_FALSE(noImu.start(nullptr, millis(), "тест"));
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, noImu.getStage());
    TEST_ASSERT_EQUAL_UINT8(3, noImu.takeSignal());

    imu.supportsGround = false;
    GroundCalibration unsupported;
    TEST_ASSERT_FALSE(unsupported.start(&imu, millis(), "тест"));
    TEST_ASSERT_TRUE(contains(unsupported.getFailure(), "не умеет"));
    imu.supportsGround = true;

    // ARM — отмена без сигнала.
    GroundCalibration armed;
    armed.start(&imu, millis(), "тест");
    armed.takeSignal();
    armed.update(&imu, true, false, millis());
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::IDLE, armed.getStage());
    TEST_ASSERT_EQUAL_UINT8(0, armed.takeSignal());
    TEST_ASSERT_TRUE(contains(takeSerial(), "отменена: ARM"));

    // Самолёт всё время в руках — не замер за CAL_STEP_TIMEOUT_MS.
    GroundCalibration shaking;
    shaking.start(&imu, millis(), "тест");
    imu.data.gyroX = 20.0f;
    calibrationTicks(shaking, imu, 0, Config::CAL_STEP_TIMEOUT_MS + 100);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, shaking.getStage());
    TEST_ASSERT_TRUE(contains(shaking.getFailure(), "не замер"));
    imu.data.gyroX = 0.0f;

    // "Горизонт" дальше 45° от установки — это не горизонт.
    GroundCalibration steep;
    setAttitude(imu, 50.0f);
    steep.start(&imu, millis(), "тест");
    steep.takeSignal();
    calibrationTicks(steep, imu, 0, Config::CAL_STILL_MS + 20);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, steep.getStage());
    TEST_ASSERT_TRUE_MESSAGE(contains(steep.getFailure(), "больше 45°"), steep.getFailure());
    TEST_ASSERT_EQUAL_UINT8(3, steep.takeSignal());
    setAttitude(imu, 0.0f);

    // IMU не смог сохранить — ошибка, а не "готово".
    GroundCalibration unsaved(Config::LandingGear::TAIL_WHEEL);
    imu.refuseGroundCalibration = true;
    unsaved.start(&imu, millis(), "тест");
    calibrationTicks(unsaved, imu, Config::SIGNAL_PERIOD_MS, Config::CAL_STILL_MS + 20);
    calibrationTicks(unsaved, imu, Config::SIGNAL_PERIOD_MS, 0);
    moveIt(unsaved, imu);
    setAttitude(imu, 9.0f);
    calibrationTicks(unsaved, imu, 0, Config::CAL_STILL_MS + 20);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, unsaved.getStage());
    TEST_ASSERT_TRUE(contains(unsaved.getFailure(), "не сохранил"));
    TEST_ASSERT_EQUAL(0u, imu.groundCalibrationsSaved);
    imu.refuseGroundCalibration = false;
    setAttitude(imu, 0.0f);

    // IMU пропал посреди калибровки.
    GroundCalibration lost;
    lost.start(&imu, millis(), "тест");
    imu.available = false;
    lost.update(&imu, false, false, millis());
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::FAILED, lost.getStage());

    TEST_ASSERT_EQUAL_STRING("CAL1", GroundCalibration::stageShort(GroundCalibration::Stage::LEVEL));
    TEST_ASSERT_EQUAL_STRING("CAL2", GroundCalibration::stageShort(GroundCalibration::Stage::PARK));
    TEST_ASSERT_EQUAL_STRING("CALOK", GroundCalibration::stageShort(GroundCalibration::Stage::DONE));
    TEST_ASSERT_EQUAL_STRING("CALER", GroundCalibration::stageShort(GroundCalibration::Stage::FAILED));
    TEST_ASSERT_EQUAL_STRING("", GroundCalibration::stageShort(GroundCalibration::Stage::IDLE));
}

void test_calibration_switch_gesture()
{
    GroundCalibration cal;
    RcChannelState rc;
    rc.set(Channels::THROTTLE, 1000);
    rc.set(Config::CAL_SWITCH, 1000);
    uint32_t now = 1000;
    TEST_ASSERT_FALSE(cal.watchSwitch(rc, now));

    // Вниз-вверх: засчитывается возврат вверх.
    auto flip = [&](uint32_t gapMs) {
        now += gapMs;
        rc.set(Config::CAL_SWITCH, 2000);
        const bool a = cal.watchSwitch(rc, now);
        now += 200;
        rc.set(Config::CAL_SWITCH, 1000);
        return a || cal.watchSwitch(rc, now);
    };
    TEST_ASSERT_FALSE(flip(300));
    TEST_ASSERT_FALSE(flip(300));
    TEST_ASSERT_TRUE(flip(300));     // третий раз за окно — жест

    // Медленно — не жест.
    TEST_ASSERT_FALSE(flip(3000));
    TEST_ASSERT_FALSE(flip(3000));
    TEST_ASSERT_FALSE(flip(3000));

    // Газ не внизу — не считается.
    rc.set(Channels::THROTTLE, 1400);
    TEST_ASSERT_FALSE(flip(100));
    TEST_ASSERT_FALSE(flip(100));
    TEST_ASSERT_FALSE(flip(100));

    // Тумблер уже был включён при первом взгляде — это не щелчок.
    GroundCalibration fresh;
    rc.set(Channels::THROTTLE, 1000);
    rc.set(Config::CAL_SWITCH, 2000);
    TEST_ASSERT_FALSE(fresh.watchSwitch(rc, now));
    fresh.forgetSwitch();
    rc.set(Config::CAL_SWITCH, 1000);
    TEST_ASSERT_FALSE(fresh.watchSwitch(rc, now));
}

// ------------------------------------------------------------
// SignalPlayer
// ------------------------------------------------------------

void test_signal_player_timeline()
{
    SignalPlayer s;
    TEST_ASSERT_FALSE(s.isBusy(0));
    s.play(0, 100, true);
    TEST_ASSERT_FALSE(s.isBusy(100));

    s.play(2, 1000, true);
    TEST_ASSERT_TRUE(s.isBusy(1000));
    TEST_ASSERT_EQUAL_UINT8(2, s.getCount(1000));
    TEST_ASSERT_EQUAL_INT16(Config::SIGNAL_NOD_US, s.nodUs(1000));
    TEST_ASSERT_EQUAL_INT16(-Config::SIGNAL_NOD_US, s.nodUs(1000 + Config::SIGNAL_NOD_MS));
    TEST_ASSERT_EQUAL_INT16(0, s.nodUs(1000 + 2 * Config::SIGNAL_NOD_MS));
    TEST_ASSERT_TRUE(s.light(1000));
    TEST_ASSERT_FALSE(s.light(1000 + 2 * Config::SIGNAL_NOD_MS));
    TEST_ASSERT_EQUAL(Config::SIGNAL_MOTOR, s.escPulsesOff(1000));
    TEST_ASSERT_FALSE(s.escPulsesOff(1000 + Config::SIGNAL_ESC_PAUSE_MS));

    // Второй сигнал — через период, так же.
    const uint32_t second = 1000 + Config::SIGNAL_PERIOD_MS;
    TEST_ASSERT_EQUAL_INT16(Config::SIGNAL_NOD_US, s.nodUs(second));
    TEST_ASSERT_EQUAL(Config::SIGNAL_MOTOR, s.escPulsesOff(second + 10));
    TEST_ASSERT_TRUE(s.isBusy(second + Config::SIGNAL_PERIOD_MS - 1));
    TEST_ASSERT_FALSE(s.isBusy(second + Config::SIGNAL_PERIOD_MS));
    TEST_ASSERT_EQUAL_UINT8(0, s.getCount(second + Config::SIGNAL_PERIOD_MS));

    // Без мотора (заармлено) — только рули.
    s.play(3, 50000, false);
    TEST_ASSERT_FALSE(s.escPulsesOff(50000));
    TEST_ASSERT_EQUAL_INT16(Config::SIGNAL_NOD_US, s.nodUs(50000));
    s.stop();
    TEST_ASSERT_FALSE(s.isBusy(50001));
}

// ------------------------------------------------------------
// Автовзлёт: хвостовое колесо
// ------------------------------------------------------------

void test_tail_wheel_takeoff_stages()
{
    TakeoffRig rig;
    TEST_ASSERT_EQUAL(Stage::IDLE, rig.stage());

    // Газ сразу вверх без "внизу" после смены режима — не старт.
    rig.throttle = 1600;
    rig.run(100);
    TEST_ASSERT_EQUAL(Stage::IDLE, rig.stage());

    rig.throttle = 1000;
    rig.run(20);
    rig.throttle = 1600;
    rig.run(20);
    TEST_ASSERT_EQUAL(Stage::CHECK, rig.stage());
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, rig.autopilot.applyThrottle(1600));   // мотор стоит
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getPitchCorrection());              // рули = стики

    rig.run(Config::TAKEOFF_CHECK_MS + 20);
    TEST_ASSERT_EQUAL(Stage::TAIL_UP, rig.stage());
    TEST_ASSERT_EQUAL(1u, rig.imu.gyroTrims);
    TEST_ASSERT_FALSE(rig.imu.accelCorrection);   // разгон — горизонт только по гироскопу
    TEST_ASSERT_TRUE(rig.autopilot.isGyroOnlyAttitude());

    // Хвост вверх: руль высоты от себя, но не больше TD_TAIL_UP_ELEVATOR_US.
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -Config::TD_TAIL_UP_ELEVATOR_US, rig.autopilot.getPitchCorrection());

    // Газ — плавно: за половину разгона — половина.
    rig.run(Config::TAKEOFF_THROTTLE_RAMP_MS / 2);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, Config::TAKEOFF_THROTTLE_PCT / 2, rig.autopilot.getThrottleCorrection());
    // Газ — программы, а не стика: стик вверху не обгоняет плавный разгон.
    TEST_ASSERT_UINT16_WITHIN(25, 1500, rig.autopilot.applyThrottle(2000));

    // Хвост почти поднят — отдача меньше; вращение гасится.
    setAttitude(rig.imu, 4.0f);
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Config::TD_PITCH_KP * (Config::TD_ROLL_PITCH_DEG - 4.0f),
                             rig.autopilot.getPitchCorrection());
    rig.imu.data.gyroY = -10.0f;   // хвост быстро идёт вверх
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Config::TD_PITCH_KP * (Config::TD_ROLL_PITCH_DEG - 4.0f) + Config::TD_PITCH_KD * 10.0f,
                             rig.autopilot.getPitchCorrection());
    rig.imu.data.gyroY = 0.0f;

    // Тангаж у горизонта — разбег ровно.
    setAttitude(rig.imu, 2.5f);
    rig.run(10);
    TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Config::TD_ROLL_PITCH_KP * (Config::TD_ROLL_PITCH_DEG - 2.5f),
                             rig.autopilot.getPitchCorrection());

    // Без датчика скорости — разбег по времени, затем отрыв.
    rig.run(Config::TD_ROLL_MS + 20);
    TEST_ASSERT_EQUAL(Stage::LIFTOFF, rig.stage());

    // Цель растёт на TD_ROTATE_RATE_DPS, но у земли — не выше стоянки
    // минус TD_TAIL_CLEARANCE_DEG (10 − 3 = 7°).
    rig.run(1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Config::TD_ROLL_PITCH_DEG + Config::TD_ROTATE_RATE_DPS, rig.autopilot.getDesiredPitch());
    rig.run(3000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f - Config::TD_TAIL_CLEARANCE_DEG, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL(Stage::LIFTOFF, rig.stage());   // с барометром ждёт отрыва

    // Отрыв по барометру — предел у земли снят, набор.
    rig.baro.data.altitude = 1.5f;
    rig.run(Config::TAKEOFF_LIFTOFF_CONFIRM_MS + 20);
    TEST_ASSERT_EQUAL(Stage::CLIMB, rig.stage());
    rig.run(1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Config::TAKEOFF_CLIMB_PITCH_DEG, rig.autopilot.getDesiredPitch());

    // Через TAKEOFF_GYRO_ONLY_CLIMB_MS набора — горизонт снова по акселерометру.
    rig.run(Config::TAKEOFF_GYRO_ONLY_CLIMB_MS);
    TEST_ASSERT_TRUE(rig.imu.accelCorrection);

    rig.baro.data.altitude = Config::TAKEOFF_SAFE_ALTITUDE_M + 1.0f;
    rig.run(20);
    TEST_ASSERT_EQUAL(Stage::DONE, rig.stage());
    rig.run(20);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Config::TAKEOFF_SAFE_ALTITUDE_M + 1.0f, rig.autopilot.getTargetAltitude());
    TEST_ASSERT_EQUAL_STRING("AUTO_TAKEOFF", rig.autopilot.getModeName());
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "хвост поднят"));
    TEST_ASSERT_TRUE(contains(log, "набор окончен"));
}

void test_takeoff_refuses_when_not_standing_like_calibration()
{
    TakeoffRig rig;
    setAttitude(rig.imu, 2.0f);   // хвост на подставке / сломано колесо
    rig.start();
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
    TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "тангаж не стояночный"));
    TEST_ASSERT_TRUE(rig.autopilot.takeTakeoffRefusal());
    TEST_ASSERT_FALSE(rig.autopilot.takeTakeoffRefusal());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_TRUE(rig.imu.accelCorrection);   // мотор не стартовал — окна нет
    TEST_ASSERT_EQUAL(0u, rig.imu.gyroTrims);

    TakeoffRig tilted;
    setAttitude(tilted.imu, 10.0f, 7.0f);
    tilted.start();
    TEST_ASSERT_TRUE(contains(tilted.autopilot.getTakeoffAbortReason(), "крен не стояночный"));

    TakeoffRig uncalibrated;
    uncalibrated.imu.parkingValid = false;
    uncalibrated.start();
    TEST_ASSERT_TRUE(contains(uncalibrated.autopilot.getTakeoffAbortReason(), "нет калибровки шасси"));

    TakeoffRig noImu;
    noImu.imu.available = false;
    noImu.start();
    TEST_ASSERT_EQUAL(Stage::ABORTED, noImu.stage());
    TEST_ASSERT_TRUE(contains(noImu.autopilot.getTakeoffAbortReason(), "IMU не отвечает"));
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, noImu.autopilot.applyThrottle(1600));

    // Самолёт держат в руках — проверка ждёт, потом отказ.
    TakeoffRig shaking;
    shaking.imu.data.gyroZ = 10.0f;
    shaking.start();
    TEST_ASSERT_EQUAL(Stage::CHECK, shaking.stage());
    shaking.run(Config::TAKEOFF_CHECK_TIMEOUT_MS);
    TEST_ASSERT_TRUE(contains(shaking.autopilot.getTakeoffAbortReason(), "не стоит неподвижно"));

    // Передумал на проверке — снова ждёт, без отказа.
    TakeoffRig changedMind;
    changedMind.throttle = 1000;
    changedMind.run(20);
    changedMind.throttle = 1600;
    changedMind.run(100);
    changedMind.throttle = 1000;
    changedMind.run(20);
    TEST_ASSERT_EQUAL(Stage::IDLE, changedMind.stage());
    TEST_ASSERT_FALSE(changedMind.autopilot.takeTakeoffRefusal());
}

void test_throttle_cut_aborts_and_restart_needs_throttle_down()
{
    TakeoffRig rig;
    rig.start();
    setAttitude(rig.imu, 2.0f);
    rig.run(100);
    TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());

    rig.throttle = 1000;
    rig.run(10);
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
    TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "газ убран"));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, rig.autopilot.applyThrottle(1000));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getPitchCorrection());   // руль высоты в нейтраль
    TEST_ASSERT_TRUE(contains(takeSerial(), "Автовзлёт ПРЕРВАН: газ убран"));

    // Газ снова вверх сразу — всё ещё прерван, мотор стоит.
    rig.throttle = 1600;
    rig.run(500);
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, rig.autopilot.applyThrottle(1600));

    // Газ вниз на секунду — заново.
    rig.throttle = 1000;
    rig.run(1100);
    TEST_ASSERT_EQUAL(Stage::IDLE, rig.stage());
    setAttitude(rig.imu, 10.0f);
    rig.throttle = 1600;
    rig.run(20);
    TEST_ASSERT_EQUAL(Stage::CHECK, rig.stage());
}

void test_link_loss_on_runway_aborts_instead_of_failsafe()
{
    TakeoffRig rig;
    rig.start();
    setAttitude(rig.imu, 2.0f);
    rig.run(100);
    TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());

    rig.linkLost = true;
    rig.run(10);
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeActive());   // не RTH и не планирование с полосы
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "связь потеряна"));

    // Без связи газ failsafe (внизу) — это не пилот: заново не начинается.
    rig.throttle = 1000;
    rig.run(3000);
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeActive());

    // Связь вернулась, стик газа вверху — мотор не стартует.
    rig.linkLost = false;
    rig.throttle = 1600;
    rig.run(2000);
    TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());

    // На месте, до старта мотора, потеря связи — тоже не failsafe.
    TakeoffRig waiting;
    waiting.throttle = 1000;
    waiting.run(20);
    waiting.linkLost = true;
    waiting.run(100);
    TEST_ASSERT_EQUAL(Stage::IDLE, waiting.stage());
    TEST_ASSERT_FALSE(waiting.autopilot.isFailsafeActive());
}

void test_ground_aborts_roll_heading_and_nose_down()
{
    {
        TakeoffRig rig;
        rig.start();
        setAttitude(rig.imu, 10.0f, Config::TAKEOFF_ABORT_ROLL_DEG + 1.0f);
        rig.run(10);
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "крен на разбеге"));
    }
    {
        TakeoffRig rig;
        rig.start();
        rig.imu.data.yaw = Config::TAKEOFF_ABORT_HEADING_DEG + 1.0f;
        rig.run(10);
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "ушёл с курса"));
    }
    {
        TakeoffRig rig;
        rig.start();
        setAttitude(rig.imu, Config::TD_MIN_PITCH_DEG - 1.0f);
        rig.run(Config::TAKEOFF_NOSE_DOWN_MS - 50);
        TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());   // коротко — не страшно
        // Ниже горизонта — руль высоты тянет нос вверх.
        TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, rig.autopilot.getPitchCorrection());
        rig.run(100);
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "винт у земли"));
    }
    {
        TakeoffRig rig;
        rig.start();
        rig.run(Config::TD_TAIL_UP_TIMEOUT_MS + 20);   // тангаж так и 10°
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "хвост не поднялся"));
    }
    {
        // IMU пропал на разбеге — держать нечем: прерван, мотор стоп.
        TakeoffRig rig;
        rig.start();
        setAttitude(rig.imu, 2.0f);
        rig.run(100);
        TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());
        rig.imu.available = false;
        rig.run(10);
        TEST_ASSERT_EQUAL(Stage::ABORTED, rig.stage());
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "IMU не отвечает"));
        TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, rig.autopilot.applyThrottle(1600));
    }
    {
        // С датчиком скорости разбег ждёт скорость отрыва — не набрал за
        // TAKEOFF_GROUND_TIMEOUT_MS — прерван.
        TakeoffRig rig(Config::LandingGear::TAIL_WHEEL, true);
        rig.airspeed.setSpeed(5.0f);
        rig.start();
        setAttitude(rig.imu, 2.0f);
        rig.run(Config::TAKEOFF_GROUND_TIMEOUT_MS + 50);
        TEST_ASSERT_TRUE(contains(rig.autopilot.getTakeoffAbortReason(), "нет отрыва"));
    }
}

void test_rudder_holds_heading_and_sticks_add_on_top()
{
    TakeoffRig rig;
    rig.start();
    rig.imu.data.yaw = 5.0f;   // нос ушёл вправо
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -Config::TAKEOFF_HEADING_KP * 5.0f, rig.autopilot.getYawCorrection());
    rig.imu.data.gyroZ = 10.0f;   // и продолжает поворачивать
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -Config::TAKEOFF_HEADING_KP * 5.0f - Config::TAKEOFF_YAW_RATE_KD * 10.0f,
                             rig.autopilot.getYawCorrection());
    rig.imu.data.gyroZ = 0.0f;

    // Стик пилота — поверх программы.
    ControlCommand sticks;
    sticks.yaw = 100;
    sticks.pitch = 50;
    fake::advanceMs(10);
    rig.autopilot.update(true, false, 1600, sticks);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -Config::TAKEOFF_HEADING_KP * 5.0f, rig.autopilot.getYawCorrection());
    TEST_ASSERT_EQUAL_INT16(-Config::TD_TAIL_UP_ELEVATOR_US + 50, rig.autopilot.getCommand().pitch);

    // Крутилка STAB_GAIN ослабляет законы взлёта.
    PilotInputs in;
    in.knobBound[static_cast<uint8_t>(Knob::STAB_GAIN)] = true;
    in.knobs[static_cast<uint8_t>(Knob::STAB_GAIN)] = -1.0f;   // ×0.25
    rig.autopilot.setInputs(in);
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -Config::TAKEOFF_HEADING_KP * 5.0f * Config::STAB_GAIN_MIN,
                             rig.autopilot.getYawCorrection());
}

void test_takeoff_without_barometer_climbs_by_time()
{
    TakeoffRig rig;
    rig.baro.available = false;
    rig.start();
    setAttitude(rig.imu, 2.0f);
    rig.run(Config::TD_ROLL_MS + 20);
    TEST_ASSERT_EQUAL(Stage::LIFTOFF, rig.stage());

    // До предела у земли (7°) 3 с, плюс TAKEOFF_LIFTOFF_MS — набор.
    rig.run(3000 + Config::TAKEOFF_LIFTOFF_MS + 100);
    TEST_ASSERT_EQUAL(Stage::CLIMB, rig.stage());
    rig.run(3000);
    // Отрыв не виден — предел у земли остаётся и в наборе.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f - Config::TD_TAIL_CLEARANCE_DEG, rig.autopilot.getDesiredPitch());

    rig.run(Config::TAKEOFF_CLIMB_MS);
    TEST_ASSERT_EQUAL(Stage::DONE, rig.stage());
    // Дальше — горизонт и курс, газ круиза.
    rig.run(20);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, Config::CRUISE_THROTTLE_PCT, rig.autopilot.getThrottleCorrection());
}

void test_takeoff_rotates_on_airspeed()
{
    TakeoffRig rig(Config::LandingGear::TAIL_WHEEL, true);
    rig.airspeed.setSpeed(0.0f);
    rig.start();
    setAttitude(rig.imu, 2.0f);
    rig.run(Config::TD_ROLL_MS + 500);
    TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());   // по времени — нет, ждёт скорость
    rig.airspeed.setSpeed(Config::TAKEOFF_ROTATE_SPEED_MS + 0.5f);
    rig.run(20);
    TEST_ASSERT_EQUAL(Stage::LIFTOFF, rig.stage());
}

void test_nose_wheel_takeoff()
{
    TakeoffRig rig(Config::LandingGear::NOSE_WHEEL);
    rig.start();
    TEST_ASSERT_EQUAL(Stage::ROLL, rig.stage());   // без "хвост вверх"
    rig.run(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, Config::TRI_ROLL_ELEVATOR_US, rig.autopilot.getPitchCorrection());

    rig.run(Config::TRI_ROLL_MS);
    TEST_ASSERT_EQUAL(Stage::LIFTOFF, rig.stage());
    rig.run(1000);   // ~1 с подъёма носа (и пара тактов от входа в этап)
    TEST_ASSERT_FLOAT_WITHIN(0.3f, Config::TRI_ROTATE_RATE_DPS, rig.autopilot.getDesiredPitch());
    rig.run(5000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, min(Config::TAKEOFF_CLIMB_PITCH_DEG, Config::TRI_MAX_GROUND_PITCH_DEG),
                             rig.autopilot.getDesiredPitch());

    // Носовому калибровка не обязательна, но стоять он должен ровно.
    TakeoffRig tilted(Config::LandingGear::NOSE_WHEEL);
    setAttitude(tilted.imu, Config::TAKEOFF_UNCALIBRATED_TILT_DEG + 2.0f);
    tilted.start();
    TEST_ASSERT_TRUE(contains(tilted.autopilot.getTakeoffAbortReason(), "стоит не ровно"));
}

void test_gyro_only_window_survives_mode_switch_and_ends_on_disarm()
{
    TakeoffRig rig;
    rig.start();
    TEST_ASSERT_FALSE(rig.imu.accelCorrection);

    // Пилот переключился в STABILIZE на разбеге — разгон тот же, окно держится.
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.run(5000);
    TEST_ASSERT_FALSE(rig.imu.accelCorrection);
    rig.run(Config::TAKEOFF_GYRO_ONLY_MAX_MS);
    TEST_ASSERT_TRUE(rig.imu.accelCorrection);   // но не дольше предела
    TEST_ASSERT_TRUE(contains(takeSerial(), "горизонт снова по акселерометру"));

    TakeoffRig disarm;
    disarm.start();
    TEST_ASSERT_FALSE(disarm.imu.accelCorrection);
    disarm.armed = false;
    disarm.run(10);
    TEST_ASSERT_TRUE(disarm.imu.accelCorrection);
    TEST_ASSERT_EQUAL(Stage::IDLE, disarm.stage());

    // Прерванный взлёт: самолёт тормозит — окно ещё TAKEOFF_GYRO_ONLY_ABORT_MS.
    TakeoffRig aborted;
    aborted.start();
    aborted.throttle = 1000;
    aborted.run(10);
    TEST_ASSERT_FALSE(aborted.imu.accelCorrection);
    aborted.run(Config::TAKEOFF_GYRO_ONLY_ABORT_MS);
    TEST_ASSERT_TRUE(aborted.imu.accelCorrection);
}

void test_takeoff_names()
{
    TEST_ASSERT_EQUAL_STRING("TAIL_UP", TakeoffController::stageName(Stage::TAIL_UP));
    TEST_ASSERT_EQUAL_STRING("ABORTED", TakeoffController::stageName(Stage::ABORTED));
    TEST_ASSERT_EQUAL_STRING("TAIL", TakeoffController::stageShort(Stage::TAIL_UP));
    TEST_ASSERT_EQUAL_STRING("WAIT", TakeoffController::stageShort(Stage::IDLE));
    TEST_ASSERT_EQUAL_STRING("CLIMB", TakeoffController::stageShort(Stage::CLIMB));

    // Все этапы названы и различимы, короткое имя влезает в строку OLED.
    const Stage all[] = { Stage::IDLE, Stage::CHECK, Stage::TAIL_UP, Stage::ROLL,
                          Stage::LIFTOFF, Stage::CLIMB, Stage::DONE, Stage::ABORTED };
    const size_t count = sizeof(all) / sizeof(all[0]);
    for (size_t i = 0; i < count; ++i)
    {
        const char* name = TakeoffController::stageName(all[i]);
        const char* shortName = TakeoffController::stageShort(all[i]);
        TEST_ASSERT_TRUE_MESSAGE(strcmp(name, "?") != 0 && strcmp(shortName, "?") != 0, name);
        TEST_ASSERT_TRUE_MESSAGE(strlen(shortName) <= 5, shortName);
        for (size_t j = 0; j < i; ++j)
        {
            TEST_ASSERT_TRUE(strcmp(name, TakeoffController::stageName(all[j])) != 0);
            TEST_ASSERT_TRUE(strcmp(shortName, TakeoffController::stageShort(all[j])) != 0);
        }
    }
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_pose_refines_horizon_and_keeps_nose);
    RUN_TEST(test_estimator_without_accelerometer_ignores_runway_acceleration);
    RUN_TEST(test_imu_stores_ground_calibration_and_uses_it_after_power_on);
    RUN_TEST(test_mounting_calibration_forgets_parking);
    RUN_TEST(test_gyro_bias_trim_removes_residual);
    RUN_TEST(test_ground_calibration_tail_wheel_two_steps);
    RUN_TEST(test_ground_calibration_rejects_tail_not_lowered);
    RUN_TEST(test_ground_calibration_checks_parking_angles);
    RUN_TEST(test_ground_calibration_nose_wheel_same_pose_after_wait);
    RUN_TEST(test_ground_calibration_cancel_timeout_and_missing_imu);
    RUN_TEST(test_calibration_switch_gesture);
    RUN_TEST(test_signal_player_timeline);
    RUN_TEST(test_tail_wheel_takeoff_stages);
    RUN_TEST(test_takeoff_refuses_when_not_standing_like_calibration);
    RUN_TEST(test_throttle_cut_aborts_and_restart_needs_throttle_down);
    RUN_TEST(test_link_loss_on_runway_aborts_instead_of_failsafe);
    RUN_TEST(test_ground_aborts_roll_heading_and_nose_down);
    RUN_TEST(test_rudder_holds_heading_and_sticks_add_on_top);
    RUN_TEST(test_takeoff_without_barometer_climbs_by_time);
    RUN_TEST(test_takeoff_rotates_on_airspeed);
    RUN_TEST(test_nose_wheel_takeoff);
    RUN_TEST(test_gyro_only_window_survives_mode_switch_and_ends_on_disarm);
    RUN_TEST(test_takeoff_names);
    return UNITY_END();
}
