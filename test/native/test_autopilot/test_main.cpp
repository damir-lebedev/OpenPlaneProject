// ============================================================
// AUTOPILOT: ПИД (D по скорости датчика, anti-windup, заморозка
// интегратора, dt), режимы автопилота с фейковыми датчиками и выбор
// режима тумблером CH7.
//
// Запуск: pio test -e native -f native/test_autopilot
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotModeSelector.h"
#include "autopilot/PidController.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

// ------------------------------------------------------------
// PidController
// ------------------------------------------------------------

void test_pid_proportional_and_rate_damping()
{
    PidController pid(2.0f, 0.0f, 0.5f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.getKp());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getKi());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.getKd());

    // P = 2·(10 − 4) = 12; D = −0.5·20 = −10 (по скорости датчика).
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, pid.calculate(10.0f, 4.0f, 20.0f));
}

void test_pid_integrates_with_measured_dt_and_limits_windup()
{
    PidController pid(0.0f, 1.0f, 0.0f);
    pid.setLimits(-1000.0f, 1000.0f);
    pid.reset();

    // Ошибка 10 в течение 0.5 с (по 10 мс) — интеграл 5.
    float out = 0;
    for (int i = 0; i < 50; ++i)
    {
        fake::advanceMs(10);
        out = pid.calculate(10.0f, 0.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 5.0f, out);

    // Долгая ошибка — интеграл упирается в INTEGRAL_LIMIT = 100.
    for (int i = 0; i < 5000; ++i)
    {
        fake::advanceMs(10);
        out = pid.calculate(10.0f, 0.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, out);
}

void test_pid_frozen_integrator_and_nominal_dt_after_pause()
{
    PidController pid(0.0f, 1.0f, 0.0f);
    pid.setLimits(-1000.0f, 1000.0f);
    pid.reset();
    fake::advanceMs(10);
    pid.calculate(5.0f, 0.0f, 0.0f);

    // integrate = false сбрасывает накопленное (не заармлен).
    fake::advanceMs(10);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.calculate(5.0f, 0.0f, 0.0f, false));

    // Пауза дольше 0.1 с — шаг считается номинальным периодом цикла.
    fake::advanceMs(1000);
    const float out = pid.calculate(5.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f * Config::LOOP_PERIOD_MS / 1000.0f, out);
}

void test_pid_output_is_limited()
{
    PidController pid(100.0f);
    pid.setLimits(-50.0f, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pid.calculate(10.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, pid.calculate(-10.0f, 0.0f, 0.0f));
    pid.setGains(1.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.calculate(1.0f, 0.0f, 0.0f));
}

// ------------------------------------------------------------
// Autopilot
// ------------------------------------------------------------

struct Rig
{
    FakeImu imu;
    FakeBaro baro;
    FakeMag mag;
    FakeGps gps;
    Autopilot autopilot{ &imu, &baro, &mag, &gps };
};

void test_begin_reports_missing_core_sensors()
{
    Rig rig;
    TEST_ASSERT_TRUE(rig.autopilot.begin());

    Autopilot noSensors;
    TEST_ASSERT_FALSE(noSensors.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "IMU или барометр не подключены"));
}

void test_update_reads_all_sensors_every_cycle()
{
    Rig rig;
    rig.autopilot.update(false, false, 1000);
    rig.autopilot.update(false, true, 1000);   // и без связи тоже
    TEST_ASSERT_EQUAL(2, rig.imu.updates);
    TEST_ASSERT_EQUAL(2, rig.baro.updates);
    TEST_ASSERT_EQUAL(2, rig.mag.updates);
    TEST_ASSERT_EQUAL(2, rig.gps.updates);
    TEST_ASSERT_TRUE(rig.autopilot.getMagnetometerSensor() == &rig.mag);
    TEST_ASSERT_TRUE(rig.autopilot.getGpsSensor() == &rig.gps);
}

void test_manual_mode_gives_no_corrections()
{
    Rig rig;
    rig.imu.data.roll = 30;
    rig.autopilot.update(true, false, 1500);
    TEST_ASSERT_EQUAL(MODE_MANUAL, rig.autopilot.getMode());
    TEST_ASSERT_EQUAL_STRING("MANUAL", rig.autopilot.getModeName());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getRollCorrection());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getPitchCorrection());
    TEST_ASSERT_EQUAL_UINT16(1500, rig.autopilot.applyThrottle(1500));
}

// Крен вправо и нос вниз -> коррекция влево и нос вверх (знаки
// ControlCommand) — выравнивание к горизонту.
void test_stabilize_corrects_toward_level()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.roll = 10;
    rig.imu.data.pitch = -8;
    rig.autopilot.update(false, false, 1000);

    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, rig.autopilot.getRollCorrection());
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, rig.autopilot.getPitchCorrection());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -50.0f, rig.autopilot.getRollCorrection());   // Kp = 5
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredRoll());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredPitch());
}

void test_stabilize_rate_term_damps_rotation()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.gyroX = 100;   // крен вправо растёт
    rig.autopilot.update(false, false, 1000);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -50.0f, rig.autopilot.getRollCorrection());   // Kd 0.5 · 100
}

void test_integrator_only_accumulates_when_armed()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.roll = 10;
    for (int i = 0; i < 100; ++i)
    {
        fake::advanceMs(10);
        rig.autopilot.update(false, false, 1000);
    }
    const float disarmed = rig.autopilot.getRollCorrection();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -50.0f, disarmed);   // только P

    for (int i = 0; i < 100; ++i)
    {
        fake::advanceMs(10);
        rig.autopilot.update(true, false, 1000);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(disarmed - 1.0f, rig.autopilot.getRollCorrection());
}

void test_no_corrections_without_ready_imu()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.roll = 20;

    rig.imu.available = false;
    rig.autopilot.update(true, false, 1000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getRollCorrection());

    // Предполётная проверка не пройдена — углам верить нельзя.
    rig.imu.available = true;
    rig.imu.preflightProblem = "IMU: плата не лежит чипом вверх";
    rig.autopilot.update(true, false, 1000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getRollCorrection());

    Autopilot noImu;
    noImu.setMode(MODE_STABILIZE);
    noImu.update(true, false, 1000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, noImu.getRollCorrection());
}

void test_auto_takeoff_program()
{
    Rig rig;
    rig.autopilot.setMode(MODE_AUTO_TAKEOFF);
    TEST_ASSERT_EQUAL_STRING("AUTO_TAKEOFF", rig.autopilot.getModeName());

    // Не заармлен или газ ниже порога — программа не стартует.
    rig.autopilot.update(false, false, 1800);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    rig.autopilot.update(true, false, 1400);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_UINT16(1400, rig.autopilot.applyThrottle(1400));

    rig.autopilot.update(true, false, 1500);   // старт
    TEST_ASSERT_TRUE(contains(takeSerial(), "автовзлёт — старт"));

    fake::advanceMs(500);
    rig.autopilot.update(true, false, 1500);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, rig.autopilot.getThrottleCorrection());   // разгон газа
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(1500, rig.autopilot.applyThrottle(1500));   // max(пилот, программа)
    TEST_ASSERT_EQUAL_UINT16(1700, rig.autopilot.applyThrottle(1700));

    fake::advanceMs(1000);
    rig.autopilot.update(true, false, 1500);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_FLOAT(15.0f, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_UINT16(2000, rig.autopilot.applyThrottle(1000));

    fake::advanceMs(2000);
    rig.autopilot.update(true, false, 1000);   // газ убран — программа идёт дальше
    TEST_ASSERT_EQUAL_FLOAT(10.0f, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredRoll());

    // DISARM сбрасывает программу.
    rig.autopilot.update(false, false, 1000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
}

void test_alt_hold_holds_altitude_at_mode_entry()
{
    Rig rig;
    rig.baro.data.altitude = 12.0f;
    rig.autopilot.setMode(MODE_ALT_HOLD);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, rig.autopilot.getTargetAltitude());

    // Ниже цели на 2 м — газ добавляется (Kp 10 %/м).
    rig.baro.data.altitude = 10.0f;
    rig.autopilot.update(false, false, 1500);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_UINT16(1700, rig.autopilot.applyThrottle(1500));

    // Выше цели — газ убирается, но не ниже PWM_MIN; поправка ≤ 50 %.
    rig.baro.data.altitude = 30.0f;
    rig.autopilot.update(false, false, 1500);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_UINT16(1000, rig.autopilot.applyThrottle(1200));

    rig.baro.available = false;
    rig.autopilot.update(false, false, 1500);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());

    Autopilot noBaro;
    noBaro.setMode(MODE_ALT_HOLD);
    noBaro.update(true, false, 1500);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, noBaro.getThrottleCorrection());
}

void test_set_mode_resets_state_only_on_real_change()
{
    Rig rig;
    rig.autopilot.setMode(MODE_STABILIZE);
    rig.imu.data.roll = 10;
    rig.autopilot.update(false, false, 1000);
    TEST_ASSERT_NOT_EQUAL(0, static_cast<int>(rig.autopilot.getRollCorrection()));

    takeSerial();
    rig.autopilot.setMode(MODE_STABILIZE);   // тот же — ничего не сбрасывается
    TEST_ASSERT_EQUAL(0, static_cast<int>(takeSerial().size()));
    TEST_ASSERT_NOT_EQUAL(0, static_cast<int>(rig.autopilot.getRollCorrection()));

    rig.autopilot.setMode(MODE_MANUAL);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getRollCorrection());
    TEST_ASSERT_TRUE(contains(takeSerial(), "STABILIZE -> MANUAL"));
}

void test_link_loss_while_armed_glides_in_any_mode()
{
    Rig rig;
    rig.imu.data.roll = 5;
    rig.autopilot.update(true, true, 1000);

    TEST_ASSERT_TRUE(rig.autopilot.isFailsafeGliding());
    TEST_ASSERT_EQUAL_STRING("FAILSAFE_GLIDE", rig.autopilot.getModeName());
    TEST_ASSERT_EQUAL_FLOAT(Config::FAILSAFE_GLIDE_ROLL_DEG, rig.autopilot.getDesiredRoll());
    TEST_ASSERT_EQUAL_FLOAT(Config::FAILSAFE_GLIDE_PITCH_DEG, rig.autopilot.getDesiredPitch());
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, rig.autopilot.getRollCorrection());   // выравнивает даже из MANUAL
    TEST_ASSERT_EQUAL(MODE_MANUAL, rig.autopilot.getMode());                // выбранный режим сохранён
    TEST_ASSERT_TRUE(contains(takeSerial(), "связь потеряна"));

    rig.autopilot.update(true, false, 1000);
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeGliding());
    TEST_ASSERT_EQUAL_STRING("MANUAL", rig.autopilot.getModeName());
    TEST_ASSERT_TRUE(contains(takeSerial(), "связь восстановлена"));
}

void test_link_loss_on_ground_does_not_glide()
{
    Rig rig;
    rig.autopilot.update(false, true, 1000);
    TEST_ASSERT_FALSE(rig.autopilot.isFailsafeGliding());
}

void test_glide_restarts_takeoff_only_anew()
{
    Rig rig;
    rig.autopilot.setMode(MODE_AUTO_TAKEOFF);
    rig.autopilot.update(true, false, 1600);
    fake::advanceMs(2000);
    rig.autopilot.update(true, false, 1600);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, rig.autopilot.getDesiredPitch());

    rig.autopilot.update(true, true, 1000);   // связь пропала — планирование
    rig.autopilot.update(true, false, 1000);  // вернулась, газ пилота внизу
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getThrottleCorrection());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.autopilot.getDesiredPitch());
}

void test_set_pid_gains_from_dashboard()
{
    Rig rig;
    rig.autopilot.setPIDGains(1, 2, 3, 4, 5, 6);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rig.autopilot.getRollPid().getKp());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rig.autopilot.getRollPid().getKi());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, rig.autopilot.getRollPid().getKd());
    TEST_ASSERT_EQUAL_FLOAT(4.0f, rig.autopilot.getPitchPid().getKp());
    TEST_ASSERT_EQUAL_FLOAT(5.0f, rig.autopilot.getPitchPid().getKi());
    TEST_ASSERT_EQUAL_FLOAT(6.0f, rig.autopilot.getPitchPid().getKd());
}

void test_unknown_mode_value_is_reported_as_unknown()
{
    Rig rig;
    rig.autopilot.setMode(static_cast<AutopilotMode>(7));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", rig.autopilot.getModeName());
    rig.autopilot.update(true, false, 1000);   // неизвестный режим — ничего не делает
    TEST_ASSERT_EQUAL_UINT16(1234, rig.autopilot.applyThrottle(1234));
}

// ------------------------------------------------------------
// AutopilotModeSelector
// ------------------------------------------------------------

static RcChannelState ch7(uint16_t us)
{
    RcChannelState rc;
    rc.set(Channels::AUX_2, us);
    return rc;
}

void test_selector_zones()
{
    Autopilot autopilot;
    AutopilotModeSelector selector(&autopilot);

    selector.update(ch7(1300));
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());
    selector.update(ch7(1749));
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());
    selector.update(ch7(1750));
    TEST_ASSERT_EQUAL(MODE_AUTO_TAKEOFF, autopilot.getMode());
    selector.update(ch7(1249));
    TEST_ASSERT_EQUAL(MODE_MANUAL, autopilot.getMode());
}

// ALT_HOLD с дашборда держится, пока тумблер не сменит зону.
void test_selector_changes_mode_only_on_zone_transition()
{
    Autopilot autopilot;
    AutopilotModeSelector selector(&autopilot);
    selector.update(ch7(1000));
    autopilot.setMode(MODE_ALT_HOLD);

    for (int i = 0; i < 10; ++i) selector.update(ch7(1000));
    TEST_ASSERT_EQUAL(MODE_ALT_HOLD, autopilot.getMode());

    selector.update(ch7(1500));
    TEST_ASSERT_EQUAL(MODE_STABILIZE, autopilot.getMode());
}

void test_selector_without_autopilot_is_noop()
{
    AutopilotModeSelector selector;
    selector.update(ch7(2000));   // не падает
    TEST_PASS();
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_proportional_and_rate_damping);
    RUN_TEST(test_pid_integrates_with_measured_dt_and_limits_windup);
    RUN_TEST(test_pid_frozen_integrator_and_nominal_dt_after_pause);
    RUN_TEST(test_pid_output_is_limited);
    RUN_TEST(test_begin_reports_missing_core_sensors);
    RUN_TEST(test_update_reads_all_sensors_every_cycle);
    RUN_TEST(test_manual_mode_gives_no_corrections);
    RUN_TEST(test_stabilize_corrects_toward_level);
    RUN_TEST(test_stabilize_rate_term_damps_rotation);
    RUN_TEST(test_integrator_only_accumulates_when_armed);
    RUN_TEST(test_no_corrections_without_ready_imu);
    RUN_TEST(test_auto_takeoff_program);
    RUN_TEST(test_alt_hold_holds_altitude_at_mode_entry);
    RUN_TEST(test_set_mode_resets_state_only_on_real_change);
    RUN_TEST(test_link_loss_while_armed_glides_in_any_mode);
    RUN_TEST(test_link_loss_on_ground_does_not_glide);
    RUN_TEST(test_glide_restarts_takeoff_only_anew);
    RUN_TEST(test_set_pid_gains_from_dashboard);
    RUN_TEST(test_unknown_mode_value_is_reported_as_unknown);
    RUN_TEST(test_selector_zones);
    RUN_TEST(test_selector_changes_mode_only_on_zone_transition);
    RUN_TEST(test_selector_without_autopilot_is_noop);
    return UNITY_END();
}
