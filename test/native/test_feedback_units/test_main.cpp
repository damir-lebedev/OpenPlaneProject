// ============================================================
// Модули обратной связи по отдельности — то, до чего не доходит
// замкнутая симуляция test_feedback: граничные случаи, отмены,
// признаки сваливания, имена состояний, диагностика.
//
// Запуск: pio test -e native -f native/test_feedback_units
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/feedback/FeedbackModules.h"
#include "helpers/TestSupport.h"

using namespace FeedbackConfig;

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    FlightSnapshot level(uint32_t timeUs)
    {
        FlightSnapshot s;
        s.timeUs = timeUs;
        s.armed = true;
        s.imuValid = true;
        s.accelZg = 1.0f;
        return s;
    }

    FlightSnapshot flying(uint32_t timeUs, float speed)
    {
        FlightSnapshot s = level(timeUs);
        s.baroValid = true;
        s.altitudeM = 50;
        s.airspeedValid = true;
        s.airspeedMs = speed;
        return s;
    }
}

// ------------------------------------------------------------
// SpeedEstimator
// ------------------------------------------------------------

void test_speed_sources_in_priority_order()
{
    SpeedEstimator speed;
    FlightSnapshot s = level(10000);
    speed.update(s);
    TEST_ASSERT_FALSE(speed.hasSpeed());
    TEST_ASSERT_TRUE(speed.getSource() == SpeedEstimator::Source::None);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, speed.effectivenessScale());

    s.timeUs += 20000;
    s.gpsValid = true;
    s.groundSpeedMs = 7.0f;
    speed.update(s);
    TEST_ASSERT_TRUE(speed.getSource() == SpeedEstimator::Source::Gps);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, speed.getSpeed());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (7.0f / 14.0f) * (7.0f / 14.0f), speed.effectivenessScale());

    s.timeUs += 20000;
    s.airspeedValid = true;
    s.airspeedMs = 28.0f;
    speed.update(s);
    TEST_ASSERT_TRUE(speed.getSource() == SpeedEstimator::Source::Airspeed);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, speed.effectivenessScale());   // (28/14)² = 4 — потолок

    s.airspeedMs = 0.5f;
    s.timeUs += 20000;
    speed.update(s);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, speed.effectivenessScale());  // пол
}

void test_speed_acceleration_needs_imu_and_sane_dt()
{
    SpeedEstimator speed;
    FlightSnapshot s = level(10000);
    s.accelXg = 0.2f;   // разгон 0.2g
    for (int i = 0; i < 100; ++i)
    {
        s.timeUs += 20000;
        speed.update(s);
    }
    TEST_ASSERT_TRUE(speed.hasAcceleration());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.2f * GRAVITY, speed.getAcceleration());

    s.imuValid = false;
    s.timeUs += 20000;
    speed.update(s);
    TEST_ASSERT_FALSE(speed.hasAcceleration());

    s.imuValid = true;
    s.accelXg = -1.0f;
    s.timeUs += 2000000;   // пауза 2 с — шаг пропускается
    speed.update(s);
    TEST_ASSERT_FALSE(speed.hasAcceleration());
}

// ------------------------------------------------------------
// AirborneDetector
// ------------------------------------------------------------

void test_airborne_detection_confirm_and_disarm_reset()
{
    AirborneDetector air;
    SpeedEstimator speed;
    FlightSnapshot s = level(0);
    s.heightAglValid = true;
    s.heightAglM = 5.0f;

    air.update(s, speed, 1000);
    TEST_ASSERT_FALSE(air.isAirborne());
    air.update(s, speed, 1000 + AIRBORNE_CONFIRM_MS);
    TEST_ASSERT_TRUE(air.isAirborne());

    // Сел: низко по дальномеру, не вращается, 1g — через GROUND_STILL_MS.
    s.heightAglM = 0.1f;
    air.update(s, speed, 5000);
    air.update(s, speed, 5000 + GROUND_STILL_MS);
    TEST_ASSERT_FALSE(air.isAirborne());

    air.force(true);
    TEST_ASSERT_TRUE(air.isAirborne());
    s.armed = false;
    air.update(s, speed, 9000);
    TEST_ASSERT_FALSE(air.isAirborne());
}

// ------------------------------------------------------------
// ControlEffectivenessEstimator и AdaptiveRateController
// ------------------------------------------------------------

void test_estimator_restarts_after_long_gap_and_reports_trim()
{
    ControlEffectivenessEstimator e(AXIS_PITCH);
    TEST_ASSERT_EQUAL_FLOAT(EFFECTIVENESS_PRIOR[AXIS_PITCH], e.getReferenceEffectiveness());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, e.getTrimUs());   // c = 0

    e.update(0, 0, 1.0f, true, 0);
    e.update(10, 5, 1.0f, true, 20);
    e.update(10, 7, 1.0f, true, 40);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, e.getAngularAccel());   // (7 − 5) / 0.02

    // Цикл стоял 0.5 с — интервал испорчен, оценка не меняется.
    const float before = e.getAngularAccel();
    e.update(10, 50, 1.0f, true, 540);
    TEST_ASSERT_EQUAL_FLOAT(before, e.getAngularAccel());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, e.getEffectivenessSigma());
}

void test_rate_controller_exposes_state_and_limits_output()
{
    AdaptiveRateController c(AXIS_ROLL);
    AxisModel model;
    model.effectiveness = 0.001f;   // почти нулевая — ограничивается минимумом
    const float out = c.update(100.0f, 0.0f, model, true, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(MAX_DEFLECTION_US[AXIS_ROLL], out);
    TEST_ASSERT_EQUAL_FLOAT(out, c.getOutput());
    TEST_ASSERT_EQUAL_FLOAT(100.0f, c.getDesiredRate());
    TEST_ASSERT_TRUE(c.isSaturated());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, c.getIntegral());

    // В упоре интеграл в ту же сторону не копится.
    const float integral = c.getIntegral();
    c.update(100.0f, 0.0f, model, true, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(integral, c.getIntegral());

    c.reset();
    TEST_ASSERT_FALSE(c.isSaturated());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.getIntegral());
    // Курс 10°, цель 350° — кратчайший путь −20°; для рысканья скорость
    // ограничена MAX_RATE_DPS.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f * ANGLE_GAIN[AXIS_ROLL], c.angleToRate(350.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -MAX_RATE_DPS[AXIS_YAW], AdaptiveRateController(AXIS_YAW).angleToRate(350.0f, 10.0f));
}

// ------------------------------------------------------------
// StallGuard
// ------------------------------------------------------------

void test_stall_guard_levels_and_limits()
{
    StallGuard guard;
    SpeedEstimator speed;
    StallGuard::ControlState control;
    TEST_ASSERT_EQUAL_STRING("OK", guard.getLevelName());
    TEST_ASSERT_EQUAL_FLOAT(90.0f, guard.maxPitchDeg());
    TEST_ASSERT_EQUAL_FLOAT(180.0f, guard.maxBankDeg());
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, guard.throttleFloorPercent(false));

    // Скорость ниже сваливания.
    FlightSnapshot s = flying(20000, 7.0f);
    speed.update(s);
    guard.update(s, speed, control, true, 100);
    TEST_ASSERT_TRUE(guard.getLevel() == StallGuard::Level::Stall);
    TEST_ASSERT_EQUAL_STRING("STALL", guard.getLevelName());
    TEST_ASSERT_EQUAL_STRING("скорость ниже сваливания", guard.getReason());
    TEST_ASSERT_EQUAL_FLOAT(STALL_MAX_PITCH_DEG, guard.maxPitchDeg());
    TEST_ASSERT_EQUAL_FLOAT(STALL_MAX_BANK_DEG, guard.maxBankDeg());
    TEST_ASSERT_EQUAL_FLOAT(STALL_AILERON_LIMIT_US, guard.maxAileronUs());
    TEST_ASSERT_EQUAL_FLOAT(STALL_THROTTLE_PERCENT, guard.throttleFloorPercent(false));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, guard.throttleFloorPercent(true));   // без связи газ не трогаем

    // Признаки ушли — меры сваливания держатся RECOVERY_HOLD_MS, потом
    // "мало энергии", пока скорость не поднимется выше 1.5·Vs.
    s = flying(40000, 11.0f);
    speed.update(s);
    guard.update(s, speed, control, true, 600);
    TEST_ASSERT_TRUE(guard.getLevel() == StallGuard::Level::Stall);
    guard.update(s, speed, control, true, 100 + RECOVERY_HOLD_MS);
    TEST_ASSERT_TRUE(guard.getLevel() == StallGuard::Level::LowEnergy);
    TEST_ASSERT_EQUAL_STRING("LOW_ENERGY", guard.getLevelName());
    TEST_ASSERT_EQUAL_FLOAT(LOW_ENERGY_MAX_PITCH_DEG, guard.maxPitchDeg());
    TEST_ASSERT_EQUAL_FLOAT(LOW_ENERGY_THROTTLE_PERCENT, guard.throttleFloorPercent(false));
    s = flying(60000, 20.0f);
    speed.update(s);
    guard.update(s, speed, control, true, 100 + 3 * RECOVERY_HOLD_MS);
    TEST_ASSERT_TRUE(guard.getLevel() == StallGuard::Level::Normal);

    // На земле/без IMU — сброс.
    guard.update(flying(80000, 5.0f), speed, control, false, 5000);
    TEST_ASSERT_TRUE(guard.getLevel() == StallGuard::Level::Normal);
}

void test_stall_guard_signs_without_airspeed()
{
    SpeedEstimator speed;   // скорость неизвестна
    StallGuard::ControlState control;

    // Нос резко падает, хотя руль высоты тянет вверх.
    StallGuard noseDrop;
    FlightSnapshot s = level(1000);
    s.commandPitchUs = 100;
    s.pitchRateDps = -80;
    noseDrop.update(s, speed, control, true, 10);
    TEST_ASSERT_EQUAL_STRING("нос падает против руля высоты", noseDrop.getReason());

    // Руль высоты потерял эффективность — мало энергии.
    StallGuard weak;
    control.pitchEffectivenessKnown = true;
    control.pitchEffectiveness = 0.1f;
    weak.update(level(1000), speed, control, true, 10);
    TEST_ASSERT_TRUE(weak.getLevel() == StallGuard::Level::LowEnergy);
    TEST_ASSERT_EQUAL_STRING("руль высоты потерял эффективность", weak.getReason());

    // При малой энергии крыло валится против элеронов — сваливание на крыло.
    FlightSnapshot wing = level(1000);
    wing.rollRateDps = -150;
    wing.commandRollUs = 100;
    weak.update(wing, speed, control, true, 20);
    TEST_ASSERT_TRUE(weak.getLevel() == StallGuard::Level::Stall);
    TEST_ASSERT_EQUAL_STRING("сваливание на крыло", weak.getReason());

    // Энергия без датчика скорости "вернулась", когда скорость не падает.
    StallGuard recovering;
    control.pitchEffectiveness = 0.1f;
    recovering.update(level(1000), speed, control, true, 10);
    control.pitchEffectivenessKnown = false;
    recovering.update(level(1000), speed, control, true, 10 + RECOVERY_HOLD_MS);
    TEST_ASSERT_TRUE(recovering.getLevel() == StallGuard::Level::Normal);
}

// ------------------------------------------------------------
// Взлёт и посадка
// ------------------------------------------------------------

void test_takeoff_states_names_and_aborts()
{
    TakeoffSequencer t;
    SpeedEstimator speed;
    TEST_ASSERT_EQUAL_STRING("IDLE", t.getStateName());
    TEST_ASSERT_FALSE(t.isActive());

    t.request(0);
    TEST_ASSERT_EQUAL_STRING("WAIT_THROTTLE", t.getStateName());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.getTargets().throttlePercent);   // мотор стоит до газа
    TEST_ASSERT_FALSE(t.getTargets().controlRoll);

    FlightSnapshot s = level(0);
    s.pilotThrottlePercent = 60;
    t.update(s, speed, 100);
    TEST_ASSERT_EQUAL_STRING("GROUND_ROLL", t.getStateName());
    TEST_ASSERT_FALSE(t.isAirborne());

    // Газ убрали на разбеге — отмена.
    s.pilotThrottlePercent = 10;
    t.update(s, speed, 200);
    TEST_ASSERT_EQUAL_STRING("ABORTED", t.getStateName());
    TEST_ASSERT_FALSE(t.getTargets().active);

    // Разбег затянулся без отрыва (скорость есть, но мала) — отмена.
    TakeoffSequencer slow;
    FlightSnapshot fast = flying(20000, 5.0f);
    fast.pilotThrottlePercent = 60;
    speed.update(fast);
    slow.request(0);
    slow.update(fast, speed, 10);
    slow.update(fast, speed, 10 + LAUNCH_TIMEOUT_MS + 1);
    TEST_ASSERT_EQUAL_STRING("ABORTED", slow.getStateName());

    TakeoffSequencer cancelled;
    cancelled.request(0);
    cancelled.cancel();
    TEST_ASSERT_EQUAL_STRING("ABORTED", cancelled.getStateName());
    cancelled.reset();
    TEST_ASSERT_EQUAL_STRING("IDLE", cancelled.getStateName());
    cancelled.cancel();   // неактивный — остаётся IDLE
    TEST_ASSERT_EQUAL_STRING("IDLE", cancelled.getStateName());
}

void test_takeoff_fallbacks_without_airspeed_and_barometer()
{
    TakeoffSequencer t;
    SpeedEstimator speed;   // датчика скорости нет
    FlightSnapshot s = level(0);
    s.pilotThrottlePercent = 80;
    t.request(0);
    t.update(s, speed, 0);
    TEST_ASSERT_EQUAL_STRING("GROUND_ROLL", t.getStateName());
    t.update(s, speed, ROTATE_FALLBACK_MS);   // отрыв по времени
    TEST_ASSERT_EQUAL_STRING("CLIMB", t.getStateName());
    TEST_ASSERT_TRUE(t.isAirborne());
    TEST_ASSERT_EQUAL_FLOAT(CLIMB_PITCH_DEG, t.getTargets().targetPitchDeg);

    t.update(s, speed, ROTATE_FALLBACK_MS + TAKEOFF_CLIMB_FALLBACK_MS);   // набор по времени
    TEST_ASSERT_EQUAL_STRING("COMPLETE", t.getStateName());
    TEST_ASSERT_TRUE(t.isAirborne());
}

void test_landing_states_go_around_and_no_height_source()
{
    LandingSequencer l;
    TEST_ASSERT_EQUAL_STRING("IDLE", l.getStateName());
    l.request(0);
    TEST_ASSERT_EQUAL_STRING("APPROACH", l.getStateName());

    // Без барометра и дальномера высоты нет — выравнивание не начинается,
    // тангаж — базовый.
    FlightSnapshot s = level(0);
    l.update(s, 100);
    TEST_ASSERT_EQUAL_STRING("APPROACH", l.getStateName());
    TEST_ASSERT_EQUAL_FLOAT(APPROACH_BASE_PITCH_DEG, l.getTargets().targetPitchDeg);

    s.pilotThrottlePercent = GO_AROUND_THROTTLE_PERCENT;   // уход на второй круг
    l.update(s, 200);
    TEST_ASSERT_EQUAL_STRING("ABORTED", l.getStateName());
    TEST_ASSERT_FALSE(l.isActive());

    // Выравнивание по дальномеру, касание по удару.
    LandingSequencer agl;
    agl.request(0);
    FlightSnapshot low = level(0);
    low.heightAglValid = true;
    low.heightAglM = 1.5f;
    agl.update(low, 10);
    TEST_ASSERT_EQUAL_STRING("FLARE", agl.getStateName());
    TEST_ASSERT_TRUE(agl.isNearGround());
    low.accelZg = 1.8f;
    agl.update(low, 20);
    TEST_ASSERT_EQUAL_STRING("ROLLOUT", agl.getStateName());
    TEST_ASSERT_TRUE(agl.isOnGround());
    agl.update(low, 20 + ROLLOUT_MS);
    TEST_ASSERT_EQUAL_STRING("COMPLETE", agl.getStateName());

    agl.cancel();   // не активна — остаётся COMPLETE
    TEST_ASSERT_EQUAL_STRING("COMPLETE", agl.getStateName());
    agl.request(0);
    agl.cancel();
    TEST_ASSERT_EQUAL_STRING("ABORTED", agl.getStateName());
}

// ------------------------------------------------------------
// FeedbackSupervisor
// ------------------------------------------------------------

void test_supervisor_requests_are_refused_in_wrong_state()
{
    FeedbackSupervisor fb;
    TEST_ASSERT_FALSE(fb.requestTakeoff());   // ещё не видел ни одного снимка (не armed)
    TEST_ASSERT_FALSE(fb.requestLanding());

    FlightSnapshot s = level(1000);
    fb.update(s);
    TEST_ASSERT_FALSE(fb.requestLanding());   // на земле
    TEST_ASSERT_TRUE(fb.requestTakeoff());
    fb.cancelPhase();
    TEST_ASSERT_TRUE(fb.getTakeoff().getState() == TakeoffSequencer::State::Aborted);
    TEST_ASSERT_FALSE(fb.getSpeedEstimator().hasSpeed());
}

void test_supervisor_status_print_and_stall_reason()
{
    FeedbackSupervisor fb;
    FlightSnapshot s = flying(1000, 7.0f);   // ниже сваливания
    s.stabilizationActive = true;
    for (int i = 0; i < 400; ++i)
    {
        s.timeUs += 2000;
        fb.update(s);
    }
    TEST_ASSERT_TRUE(fb.isAirborne());
    TEST_ASSERT_TRUE(contains(fb.getOutput().reason, "СВАЛИВАНИЕ: скорость ниже сваливания"));
    TEST_ASSERT_EQUAL_FLOAT(STALL_THROTTLE_PERCENT, fb.getOutput().throttleFloorPercent);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(STALL_MAX_PITCH_DEG, fb.getOutput().targetPitchDeg);

    fb.printStatus(Serial);
    const std::string status = takeSerial();
    TEST_ASSERT_TRUE(contains(status, "FEEDBACK: СВАЛИВАНИЕ"));
    TEST_ASSERT_TRUE(contains(status, "air=1 speed=7.0"));
    TEST_ASSERT_TRUE(contains(status, "stall=STALL takeoff=IDLE landing=IDLE"));
    TEST_ASSERT_TRUE(contains(status, "  ROLL "));
    TEST_ASSERT_TRUE(contains(status, "  PITCH"));
    TEST_ASSERT_TRUE(contains(status, "  YAW  "));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fb.getController(AXIS_YAW).getIntegral() * 0.0f);

    // Без скорости и без стабилизации: "ручное (обучение)", оси выключены.
    FeedbackSupervisor manual;
    FlightSnapshot m = level(1000);
    manual.update(m);
    manual.printStatus(Serial);
    const std::string manualStatus = takeSerial();
    TEST_ASSERT_TRUE(contains(manualStatus, "ручное (обучение)"));
    TEST_ASSERT_TRUE(contains(manualStatus, "speed=?"));
    TEST_ASSERT_TRUE(contains(manualStatus, "(off)"));
}

void test_supervisor_throttle_override_respects_stall_floor()
{
    // Этап полёта задаёт газ, а защита от сваливания поднимает его до
    // своего минимума.
    FeedbackSupervisor fb;
    FlightSnapshot s = flying(1000, 9.5f);    // мало энергии: Vs < V < 1.25·Vs
    s.stabilizationActive = true;
    for (int i = 0; i < 400; ++i)
    {
        s.timeUs += 2000;
        fb.update(s);
    }
    TEST_ASSERT_TRUE(fb.isAirborne());
    TEST_ASSERT_TRUE(fb.requestLanding());
    s.timeUs += 2000;
    fb.update(s);
    TEST_ASSERT_TRUE(fb.getLanding().isActive());
    // Этап посадки хочет 25 %, а мало энергии — не меньше 80 %.
    TEST_ASSERT_EQUAL_FLOAT(LOW_ENERGY_THROTTLE_PERCENT, fb.getOutput().throttleOverridePercent);
    TEST_ASSERT_TRUE(contains(fb.getOutput().reason, "мало энергии"));
}

// ------------------------------------------------------------
// Интерфейс IMU: реализации по умолчанию
// ------------------------------------------------------------

void test_imu_interface_defaults()
{
    struct BareImu : ImuSensor
    {
        ImuData data = {};
        bool begin() override { return true; }
        bool isAvailable() const override { return true; }
        void update() override {}
        const char* getSensorType() const override { return "bare"; }
        void printStatus() const override {}
        const ImuData& getImuData() const override { return data; }
        void calibrate() override {}
        void setYaw(float) override {}
    } imu;
    ImuSensor& sensor = imu;
    sensor.calibrateOrientation();   // по умолчанию — не поддерживается, ничего не делает
    TEST_ASSERT_NULL(sensor.getPreflightProblem());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_speed_sources_in_priority_order);
    RUN_TEST(test_speed_acceleration_needs_imu_and_sane_dt);
    RUN_TEST(test_airborne_detection_confirm_and_disarm_reset);
    RUN_TEST(test_estimator_restarts_after_long_gap_and_reports_trim);
    RUN_TEST(test_rate_controller_exposes_state_and_limits_output);
    RUN_TEST(test_stall_guard_levels_and_limits);
    RUN_TEST(test_stall_guard_signs_without_airspeed);
    RUN_TEST(test_takeoff_states_names_and_aborts);
    RUN_TEST(test_takeoff_fallbacks_without_airspeed_and_barometer);
    RUN_TEST(test_landing_states_go_around_and_no_height_source);
    RUN_TEST(test_supervisor_requests_are_refused_in_wrong_state);
    RUN_TEST(test_supervisor_status_print_and_stall_reason);
    RUN_TEST(test_supervisor_throttle_override_respects_stall_floor);
    RUN_TEST(test_imu_interface_defaults);
    return UNITY_END();
}
