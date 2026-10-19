// ============================================================
// FlightController: полный такт на настоящих RcReceiver,
// ControlMixer, ThrottleManager, ArmingManager, FlightOutputs,
// Autopilot и PilotSwitches; железо — FakeBoard, датчики —
// фейки. Проверяется порядок приоритетов: потеря связи > ARM > стики
// и автопилот > газ.
//
// Запуск: pio test -e native -f native/test_flight_controller
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/Autopilot.h"
#include "autopilot/PilotSwitches.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/RcReceiver.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

namespace
{
    struct Plane
    {
        FakeBoard board;
        FakeImu imu;
        FakeBaro baro;
        RcReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro };
        PilotSwitches selector{ &autopilot };
        ArmingManager arming{ &autopilot };
        FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &selector };

        Plane()
        {
            outputs.begin();
            controller.begin();
        }

        // Кадр пульта + один такт цикла (2 мс).
        void tick(const RcChannels& rc)
        {
            board.rc.push(ibusFrame(rc));
            fake::advanceMs(2);
            controller.update();
        }

        void tickWithoutFrame()
        {
            fake::advanceMs(2);
            controller.update();
        }

        void arm()
        {
            RcChannels rc;
            tick(rc);
            rc.set(Channels::ARM, 2000);
            tick(rc);
        }

        // Стоит на колёсах, калибровка шасси есть (нос вверх на 10°).
        void standOnWheels()
        {
            imu.park(10.0f);
            imu.data.pitch = 10.0f;
            imu.data.accelX = sinf(10.0f * 0.0174533f);
            imu.data.accelZ = cosf(10.0f * 0.0174533f);
        }

        uint16_t pwm(uint8_t channel) const { return board.servos[channel].lastUs; }
    };
}

void test_begin_puts_outputs_to_failsafe_and_opens_receiver()
{
    Plane plane;
    TEST_ASSERT_EQUAL_UINT32(Config::IBUS_BAUDRATE, plane.board.rc.baud);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_AILERON, plane.pwm(ServoChannel::AILERON_LEFT));
}

void test_no_frames_yet_means_failsafe_outputs()
{
    Plane plane;
    plane.tickWithoutFrame();
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_ELEVATOR, plane.pwm(ServoChannel::ELEVATOR));
}

void test_sticks_drive_surfaces_but_throttle_is_blocked_until_armed()
{
    Plane plane;
    RcChannels rc;
    rc.set(Channels::AILERON, 2000).set(Channels::THROTTLE, 1800);
    plane.tick(rc);

    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::PWM_MIN), plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_LEFT_REVERSED ? 1000 : 2000, plane.pwm(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_LEFT_REVERSED ? 1000 : 2000,
                             plane.controller.getOutputState().aileronLeft);
    TEST_ASSERT_EQUAL_UINT16(2000, plane.controller.getRcState().get(Channels::AILERON));
}

void test_armed_passes_pilot_throttle()
{
    Plane plane;
    plane.arm();
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    TEST_ASSERT_TRUE(plane.controller.getArming().isArmed());

    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1650);
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(escUs(cappedThrottleUs(1650)), plane.pwm(ServoChannel::ESC));
}

void test_link_loss_on_ground_neutralises_everything()
{
    Plane plane;
    RcChannels rc;
    rc.set(Channels::AILERON, 2000);
    plane.tick(rc);

    fake::advanceMs(600);   // кадров нет — обрыв провода
    plane.tickWithoutFrame();
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_AILERON, plane.pwm(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
}

// В воздухе (armed) без связи: мотор выключен, автопилот держит
// планирование — рули отвечают на крен, а не стоят в нейтрали.
void test_link_loss_in_air_glides_with_motor_off()
{
    Plane plane;
    plane.arm();
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1700).set(Channels::SWB, 2000);
    for (int i = 0; i < 10; ++i) plane.tick(rc);

    plane.imu.data.roll = 20;   // правое крыло вниз
    RcChannels lost = rc;
    lost.set(Channels::THROTTLE, 900);   // failsafe-значение от приёмника
    plane.tick(lost);

    TEST_ASSERT_TRUE(plane.autopilot.isFailsafeGliding());
    TEST_ASSERT_TRUE(plane.controller.isArmed());   // ARM не сбрасывается
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
    // Коррекция крена влево: левый элерон вверх (закрылки убраны).
    const uint16_t left = plane.pwm(ServoChannel::AILERON_LEFT);
    TEST_ASSERT_TRUE(Config::AILERON_LEFT_REVERSED ? left > 1500 : left < 1500);
}

void test_link_loss_without_imu_neutralises_even_when_armed()
{
    Plane plane;
    plane.arm();
    plane.imu.available = false;
    RcChannels lost;
    lost.set(Channels::ARM, 2000).set(Channels::THROTTLE, 900).set(Channels::AILERON, 2000);
    plane.tick(lost);
    TEST_ASSERT_EQUAL_UINT16(1500, plane.pwm(ServoChannel::AILERON_LEFT));
    TEST_ASSERT_EQUAL_UINT16(1500, plane.pwm(ServoChannel::ELEVATOR));
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
}

void test_mode_switch_is_ignored_during_link_loss()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_MANUAL, plane.autopilot.getMode());

    RcChannels lost;
    lost.set(Channels::THROTTLE, 900).set(Channels::SWC, 2000);   // CH7 в failsafe-кадре
    plane.tick(lost);
    TEST_ASSERT_EQUAL(MODE_MANUAL, plane.autopilot.getMode());

    rc.set(Channels::SWC, 1500);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, plane.autopilot.getMode());
}

void test_autopilot_corrections_are_added_to_sticks_and_clamped()
{
    Plane plane;
    plane.imu.data.pitch = -10;   // нос вниз -> коррекция "нос вверх"
    RcChannels rc;
    rc.set(Channels::SWC, 1500);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, plane.autopilot.getMode());

    const uint16_t elevator = plane.pwm(ServoChannel::ELEVATOR);
    // Нос вверх = руль высоты вверх; в PWM знак зависит от реверса.
    TEST_ASSERT_TRUE(Config::ELEVATOR_REVERSED ? elevator < 1500 : elevator > 1500);

    // STABILIZE — угол по стику: стик на себя до упора задаёт +25°, а
    // нос опущен на 90° — ошибка больше хода руля, но не дальше упора.
    plane.imu.data.pitch = -90;
    rc.set(Channels::ELEVATOR, 1000);   // на себя — нос вверх
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(Config::ELEVATOR_REVERSED ? 1000 : 2000, plane.pwm(ServoChannel::ELEVATOR));
}

void test_takeoff_throttle_still_requires_arm()
{
    Plane plane;
    plane.standOnWheels();
    RcChannels rc;
    rc.set(Channels::SWC, 2000).set(Channels::THROTTLE, 1000);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_AUTO_TAKEOFF, plane.autopilot.getMode());

    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    rc.set(Channels::THROTTLE, 1600);
    // Проверка перед стартом — мотор стоит; потом разгон газа программы.
    const int checkTicks = Config::TAKEOFF_CHECK_MS / 2 - 20;
    for (int i = 0; i < checkTicks; ++i) plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::PWM_MIN), plane.pwm(ServoChannel::ESC));
    for (int i = 0; i < static_cast<int>(Config::TAKEOFF_THROTTLE_RAMP_MS / 2) + 60; ++i) plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(escUs(cappedThrottleUs(2000)), plane.pwm(ServoChannel::ESC));   // 100% программы

    rc.set(Channels::ARM, 1000);   // DISARM — газ сразу в ноль, хотя режим хочет 100 %
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::PWM_MIN), plane.pwm(ServoChannel::ESC));
}

void test_flaps_follow_switch_smoothly()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);   // первый такт — закрылки убраны
    rc.set(Channels::SWB, 2000);
    for (int i = 0; i < 10; ++i) plane.tick(rc);   // 20 мс — ~4 мкс хода
    const int16_t first = plane.controller.getFlapsUs();
    TEST_ASSERT_GREATER_THAN_INT16(0, first);
    TEST_ASSERT_LESS_THAN_INT16(Config::FLAPS_DEPLOYED_US, first);   // не ступенькой
    for (int i = 0; i < 100; ++i) plane.tick(rc);
    TEST_ASSERT_GREATER_THAN_INT16(first, plane.controller.getFlapsUs());
    TEST_ASSERT_TRUE(plane.controller.getOutputs().isAttached(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT32(111, plane.controller.getReceiver().getGoodFrameCount());
}

void test_manual_only_controller_without_autopilot()
{
    FakeBoard board;
    RcReceiver receiver(board.rcUart());
    ControlMixer mixer;
    ThrottleManager throttle;
    ArmingManager arming;
    FlightOutputs outputs(board);
    FlightController controller(receiver, mixer, throttle, arming, outputs);
    outputs.begin();
    controller.begin();

    RcChannels rc;
    board.rc.push(ibusFrame(rc));
    controller.update();
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1000);
    board.rc.push(ibusFrame(rc));
    controller.update();
    rc.set(Channels::THROTTLE, 1400);
    board.rc.push(ibusFrame(rc));
    controller.update();
    TEST_ASSERT_EQUAL_UINT16(escUs(cappedThrottleUs(1400)), board.servos[ServoChannel::ESC].lastUs);

    fake::advanceMs(600);
    controller.update();
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), board.servos[ServoChannel::ESC].lastUs);
}

// Реверс (SwD, Feature::MOTOR_REVERSE в Controls.h) — только в DISARM.
void test_motor_reverse_switches_only_when_disarmed()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);
    rc.set(Channels::SWD, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.getOutputState().motorReverse);
    TEST_ASSERT_EQUAL_UINT16(Config::ESC_NEUTRAL_US, plane.pwm(ServoChannel::ESC));   // стоит
    TEST_ASSERT_TRUE(contains(takeSerial(), "MotorReverse: мотор НАЗАД"));

    // Сразу после смены направления ARM нельзя — мотор мог ещё крутиться.
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_TRUE(contains(takeSerial(), "направление мотора только что сменилось"));

    rc.set(Channels::ARM, 1000);
    plane.tick(rc);
    fake::advanceMs(Config::REVERSE_SETTLE_MS);
    plane.tick(rc);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());

    // Полный газ назад — REVERSE_THROTTLE_LIMIT_PCT мощности.
    rc.set(Channels::THROTTLE, 2000);
    plane.tick(rc);
    const uint16_t fullBack = Config::PWM_MIN + 10 * Config::REVERSE_THROTTLE_LIMIT_PCT;
    TEST_ASSERT_EQUAL_UINT16(fullBack, plane.controller.getOutputState().throttle);
    TEST_ASSERT_EQUAL_UINT16(escUs(fullBack, true), plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_LESS_THAN_UINT16(Config::ESC_NEUTRAL_US, plane.pwm(ServoChannel::ESC));

    // Тумблер назад в ARM: направление не меняется, ждёт DISARM.
    rc.set(Channels::SWD, 1000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.getOutputState().motorReverse);
    TEST_ASSERT_TRUE(plane.controller.getMotorReverse().isPending());
    TEST_ASSERT_EQUAL_UINT16(escUs(fullBack, true), plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_TRUE(contains(takeSerial(), "реверс НЕ переключён"));

    rc.set(Channels::ARM, 1000);
    plane.tick(rc);
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_FALSE(plane.controller.getOutputState().motorReverse);
    TEST_ASSERT_FALSE(plane.controller.getMotorReverse().isPending());
    TEST_ASSERT_EQUAL_UINT16(Config::ESC_NEUTRAL_US, plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_TRUE(contains(takeSerial(), "MotorReverse: мотор вперёд"));

    // ARM и реверс щёлкнули разом — ARM в старом направлении, реверс ждёт.
    fake::advanceMs(Config::REVERSE_SETTLE_MS);
    plane.tick(rc);
    rc.set(Channels::ARM, 2000).set(Channels::SWD, 2000).set(Channels::THROTTLE, 1000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    TEST_ASSERT_FALSE(plane.controller.getOutputState().motorReverse);
    TEST_ASSERT_TRUE(plane.controller.getMotorReverse().isPending());
    takeSerial();
}

// В реверсе газ только от пилота: автопилот видит мотор выключенным,
// а при потере связи мотор глохнет, даже если автопилот ведёт домой.
void test_motor_reverse_ignores_autopilot_throttle_and_stops_on_link_loss()
{
    Plane plane;
    plane.standOnWheels();
    RcChannels rc;
    rc.set(Channels::SWD, 2000);
    plane.tick(rc);
    fake::advanceMs(Config::REVERSE_SETTLE_MS);
    plane.tick(rc);
    rc.set(Channels::ARM, 2000).set(Channels::SWC, 2000);   // AUTO_TAKEOFF
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    TEST_ASSERT_EQUAL(MODE_AUTO_TAKEOFF, plane.autopilot.getMode());

    rc.set(Channels::THROTTLE, 1600);
    for (int i = 0; i < 10; ++i) plane.tick(rc);
    const uint16_t back = Config::PWM_MIN + 600 * Config::REVERSE_THROTTLE_LIMIT_PCT / 100;
    TEST_ASSERT_EQUAL_UINT16(escUs(back, true), plane.pwm(ServoChannel::ESC));

    RcChannels lost = rc;
    lost.set(Channels::THROTTLE, 900);
    plane.tick(lost);
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL_UINT16(Config::ESC_NEUTRAL_US, plane.pwm(ServoChannel::ESC));
}

// Крутилка Knob::THROTTLE_LIMIT: полный стик = предел с крутилки,
// разрешённое растянуто на весь ход стика.
void test_throttle_limit_knob_stretches_allowed_throttle_over_the_stick()
{
    static constexpr Binding TABLE[] = { Bind::knob(Channels::VRB, Knob::THROTTLE_LIMIT) };
    FakeBoard board;
    FakeImu imu;
    FakeBaro baro;
    RcReceiver receiver{ board.rcUart() };
    ControlMixer mixer;
    ThrottleManager throttle;
    FlightOutputs outputs{ board };
    Autopilot autopilot{ &imu, &baro };
    PilotSwitches selector{ &autopilot, TABLE };
    ArmingManager arming{ &autopilot };
    FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &selector };
    outputs.begin();
    controller.begin();
    auto tick = [&](const RcChannels& rc) { board.rc.push(ibusFrame(rc)); fake::advanceMs(2); controller.update(); };
    auto thr = [&]() { return controller.getOutputState().throttle; };

    RcChannels rc;
    tick(rc);
    rc.set(Channels::ARM, 2000);
    tick(rc);
    TEST_ASSERT_TRUE(controller.isArmed());

    rc.set(Channels::THROTTLE, 2000).set(Channels::VRB, 1000);   // крутилка вниз — 10 %
    tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1000 + 10 * Config::THROTTLE_KNOB_MIN_PCT, thr());
    rc.set(Channels::THROTTLE, 1500);                              // полстика — половина от 10 %
    tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1000 + 5 * Config::THROTTLE_KNOB_MIN_PCT, thr());

    rc.set(Channels::VRB, 1500).set(Channels::THROTTLE, 2000);   // центр — 55 %
    tick(rc);
    TEST_ASSERT_EQUAL_UINT16(1550, thr());
    TEST_ASSERT_EQUAL_UINT16(escUs(1550), board.servos[ServoChannel::ESC].lastUs);

    rc.set(Channels::VRB, 2000);                                   // вверх — без предела
    tick(rc);
    TEST_ASSERT_EQUAL_UINT16(cappedThrottleUs(2000), thr());
}

// ------------------------------------------------------------
// Наземная калибровка и сигналы
// ------------------------------------------------------------

namespace
{
    // Элерон/руль высоты: отклонение от нейтрали в физическом знаке (+ — вверх).
    int elevatorNod(const Plane& plane)
    {
        const int d = static_cast<int>(plane.pwm(ServoChannel::ELEVATOR)) - 1500;
        return Config::ELEVATOR_REVERSED ? -d : d;
    }

    // Тумблер калибровки вниз-вверх count раз (по 100 мс).
    void flipCalibrationSwitch(Plane& plane, RcChannels& rc, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            rc.set(Config::CAL_SWITCH, 2000);
            for (int t = 0; t < 50; ++t) plane.tick(rc);
            rc.set(Config::CAL_SWITCH, 1000);
            for (int t = 0; t < 50; ++t) plane.tick(rc);
        }
    }

    void ticks(Plane& plane, const RcChannels& rc, uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 2) plane.tick(rc);
    }
}

void test_switch_gesture_starts_calibration_with_motor_and_surface_signal()
{
    Plane plane;
    plane.imu.data.accelZ = 1.0f;   // стоит ровно
    RcChannels rc;
    plane.tick(rc);

    flipCalibrationSwitch(plane, rc, Config::CAL_SWITCH_FLIPS - 1);
    TEST_ASSERT_FALSE(plane.controller.getCalibration().isActive());

    // Последний щелчок: вниз, а на возврате вверх — старт и сигнал.
    rc.set(Config::CAL_SWITCH, 2000);
    for (int t = 0; t < 50; ++t) plane.tick(rc);
    rc.set(Config::CAL_SWITCH, 1000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.getCalibration().isActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "Калибровка самолёта (пульт)"));
    TEST_ASSERT_EQUAL_UINT8(1, plane.controller.getSignals().getCount(millis()));

    // Мотор: импульсов на ESC нет (регулятор потом сыграет мелодию готовности).
    TEST_ASSERT_TRUE(plane.board.servos[ServoChannel::ESC].pulsesOff);
    TEST_ASSERT_EQUAL_UINT16(0, FlightOutputs::valueOf(plane.outputs.getLastState(), ServoChannel::ESC));
    // Рули кивают: вверх, потом вниз; руль направления — тоже.
    TEST_ASSERT_EQUAL_INT(Config::SIGNAL_NOD_US, elevatorNod(plane));
    TEST_ASSERT_NOT_EQUAL(1500, plane.pwm(ServoChannel::RUDDER));
    ticks(plane, rc, Config::SIGNAL_NOD_MS);
    TEST_ASSERT_EQUAL_INT(-Config::SIGNAL_NOD_US, elevatorNod(plane));
    ticks(plane, rc, Config::SIGNAL_NOD_MS);
    TEST_ASSERT_EQUAL_INT(0, elevatorNod(plane));

    // Пауза кончилась — импульсы снова, нейтраль (мотор стоит).
    ticks(plane, rc, Config::SIGNAL_ESC_PAUSE_MS);
    TEST_ASSERT_FALSE(plane.board.servos[ServoChannel::ESC].pulsesOff);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::PWM_MIN), plane.pwm(ServoChannel::ESC));

    // ARM во время калибровки: щелчок отменяет её, но сам не армит.
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_FALSE(plane.controller.getCalibration().isActive());
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_TRUE(contains(plane.controller.getArming().getLastRefusalReason(), "отменена калибровка"));

    // Сигнал ещё звучит — ARM нельзя; отзвучал — можно.
    rc.set(Channels::ARM, 1000);
    plane.tick(rc);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_TRUE(contains(plane.controller.getArming().getLastRefusalReason(), "звучит сигнал"));
    rc.set(Channels::ARM, 1000);
    ticks(plane, rc, Config::SIGNAL_PERIOD_MS);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());

    // С ARM жест не работает (и щелчки не копятся).
    flipCalibrationSwitch(plane, rc, Config::CAL_SWITCH_FLIPS);
    TEST_ASSERT_FALSE(plane.controller.getCalibration().isActive());
}

void test_full_ground_calibration_from_transmitter_enables_tail_wheel_takeoff()
{
    Plane plane;
    plane.imu.data.accelZ = 1.0f;
    RcChannels rc;
    rc.set(Channels::SWC, 2000);   // AUTO_TAKEOFF
    plane.tick(rc);

    // Без калибровки шасси хвостовому колесу ARM в AUTO_TAKEOFF нельзя.
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_TRUE(contains(plane.controller.getArming().getLastRefusalReason(), "нет калибровки шасси"));
    rc.set(Channels::ARM, 1000);
    plane.tick(rc);

    flipCalibrationSwitch(plane, rc, Config::CAL_SWITCH_FLIPS);
    TEST_ASSERT_TRUE(plane.controller.getCalibration().isActive());

    // Шаг 1: хвост поднят — горизонтально, неподвижно.
    ticks(plane, rc, Config::SIGNAL_PERIOD_MS + Config::CAL_STILL_MS + 100);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::PARK, plane.controller.getCalibration().getStage());
    TEST_ASSERT_EQUAL_UINT8(1, plane.controller.getSignals().getCount(millis()));

    // Шаг 2: на колёса — нос вверх на 9°.
    ticks(plane, rc, Config::SIGNAL_PERIOD_MS);
    plane.imu.data.gyroY = 20.0f;
    ticks(plane, rc, 200);
    plane.imu.data.gyroY = 0.0f;
    const float up[3] = { sinf(9.0f * 0.0174533f), 0.0f, cosf(9.0f * 0.0174533f) };
    for (int i = 0; i < 3; ++i) plane.imu.chipUp[i] = up[i];
    plane.imu.data.pitch = 9.0f;
    plane.imu.data.accelX = up[0];
    plane.imu.data.accelZ = up[2];
    ticks(plane, rc, Config::CAL_STILL_MS + 100);
    TEST_ASSERT_EQUAL(GroundCalibration::Stage::DONE, plane.controller.getCalibration().getStage());
    TEST_ASSERT_EQUAL_UINT8(2, plane.controller.getSignals().getCount(millis()));
    TEST_ASSERT_TRUE(plane.imu.parkingValid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 9.0f, plane.imu.parkingPitch);

    // Отзвучало — ARM в AUTO_TAKEOFF разрешён.
    ticks(plane, rc, 2 * Config::SIGNAL_PERIOD_MS);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());
}

void test_takeoff_refusal_nods_surfaces_but_leaves_esc_alone()
{
    Plane plane;
    plane.standOnWheels();
    RcChannels rc;
    rc.set(Channels::SWC, 2000);
    plane.tick(rc);
    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());

    plane.imu.data.pitch = 1.0f;   // стоит не как на калибровке
    rc.set(Channels::THROTTLE, 1600);
    ticks(plane, rc, Config::TAKEOFF_CHECK_MS + 100);
    TEST_ASSERT_EQUAL(TakeoffController::Stage::ABORTED, plane.autopilot.getTakeoffStage());
    TEST_ASSERT_EQUAL_UINT8(3, plane.controller.getSignals().getCount(millis()));
    TEST_ASSERT_EQUAL_UINT32(0u, plane.board.servos[ServoChannel::ESC].stops);   // заармлено — ESC не трогаем

    // Кивок следующего сигнала виден на руле высоты.
    bool nodded = false;
    for (uint32_t t = 0; t < Config::SIGNAL_PERIOD_MS; t += 2)
    {
        plane.tick(rc);
        nodded = nodded || elevatorNod(plane) == Config::SIGNAL_NOD_US;
    }
    TEST_ASSERT_TRUE(nodded);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::PWM_MIN), plane.pwm(ServoChannel::ESC));
}

void test_signal_plays_on_without_link()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);
    plane.controller.playSignal(2);
    // Пульт выключили: кадров нет, но сигнал звучит — второй кивок и пауза ESC.
    for (uint32_t t = 0; t < Config::SIGNAL_PERIOD_MS; t += 2) plane.tickWithoutFrame();
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL_INT(Config::SIGNAL_NOD_US, elevatorNod(plane));
    TEST_ASSERT_TRUE(plane.board.servos[ServoChannel::ESC].pulsesOff);

    // Отзвучал — обычный failsafe: нейтраль, мотор стоит.
    for (uint32_t t = 0; t < Config::SIGNAL_PERIOD_MS; t += 2) plane.tickWithoutFrame();
    TEST_ASSERT_EQUAL_INT(0, elevatorNod(plane));
    TEST_ASSERT_FALSE(plane.board.servos[ServoChannel::ESC].pulsesOff);
    TEST_ASSERT_EQUAL_UINT16(escUs(Config::FAILSAFE_THROTTLE), plane.pwm(ServoChannel::ESC));
}

void test_console_calibration_toggle()
{
    Plane plane;
    plane.imu.data.accelZ = 1.0f;
    RcChannels rc;
    plane.tick(rc);
    plane.controller.toggleGroundCalibration();
    TEST_ASSERT_TRUE(plane.controller.getCalibration().isActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "Калибровка самолёта (консоль)"));
    plane.controller.toggleGroundCalibration();
    TEST_ASSERT_FALSE(plane.controller.getCalibration().isActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "отменена: из консоли"));

    // Заармлено — нельзя.
    ticks(plane, rc, Config::SIGNAL_PERIOD_MS);
    plane.arm();
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    plane.controller.toggleGroundCalibration();
    TEST_ASSERT_FALSE(plane.controller.getCalibration().isActive());
    TEST_ASSERT_TRUE(contains(takeSerial(), "только без ARM"));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_puts_outputs_to_failsafe_and_opens_receiver);
    RUN_TEST(test_no_frames_yet_means_failsafe_outputs);
    RUN_TEST(test_sticks_drive_surfaces_but_throttle_is_blocked_until_armed);
    RUN_TEST(test_armed_passes_pilot_throttle);
    RUN_TEST(test_link_loss_on_ground_neutralises_everything);
    RUN_TEST(test_link_loss_in_air_glides_with_motor_off);
    RUN_TEST(test_link_loss_without_imu_neutralises_even_when_armed);
    RUN_TEST(test_mode_switch_is_ignored_during_link_loss);
    RUN_TEST(test_autopilot_corrections_are_added_to_sticks_and_clamped);
    RUN_TEST(test_takeoff_throttle_still_requires_arm);
    RUN_TEST(test_flaps_follow_switch_smoothly);
    RUN_TEST(test_manual_only_controller_without_autopilot);
    RUN_TEST(test_throttle_limit_knob_stretches_allowed_throttle_over_the_stick);
    RUN_TEST(test_motor_reverse_switches_only_when_disarmed);
    RUN_TEST(test_motor_reverse_ignores_autopilot_throttle_and_stops_on_link_loss);
    RUN_TEST(test_switch_gesture_starts_calibration_with_motor_and_surface_signal);
    RUN_TEST(test_full_ground_calibration_from_transmitter_enables_tail_wheel_takeoff);
    RUN_TEST(test_takeoff_refusal_nods_surfaces_but_leaves_esc_alone);
    RUN_TEST(test_signal_plays_on_without_link);
    RUN_TEST(test_console_calibration_toggle);
    return UNITY_END();
}
