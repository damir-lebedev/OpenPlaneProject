// ============================================================
// FlightController: полный такт на настоящих IBusReceiver,
// ControlMixer, ThrottleManager, ArmingManager, FlightOutputs,
// Autopilot и AutopilotModeSelector; железо — FakeBoard, датчики —
// фейки. Проверяется порядок приоритетов: потеря связи > ARM > стики
// и автопилот > газ.
//
// Запуск: pio test -e native -f native/test_flight_controller
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotModeSelector.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
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
        IBusReceiver receiver{ board.rcUart() };
        ControlMixer mixer;
        ThrottleManager throttle;
        FlightOutputs outputs{ board };
        Autopilot autopilot{ &imu, &baro };
        AutopilotModeSelector selector{ &autopilot };
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

        uint16_t pwm(uint8_t channel) const { return board.servos[channel].lastUs; }
    };
}

void test_begin_puts_outputs_to_failsafe_and_opens_receiver()
{
    Plane plane;
    TEST_ASSERT_EQUAL_UINT32(Config::IBUS_BAUDRATE, plane.board.rc.baud);
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_AILERON, plane.pwm(ServoChannel::AILERON_LEFT));
}

void test_no_frames_yet_means_failsafe_outputs()
{
    Plane plane;
    plane.tickWithoutFrame();
    TEST_ASSERT_TRUE(plane.controller.isReceiverFailsafe());
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, plane.pwm(ServoChannel::ESC));
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_ELEVATOR, plane.pwm(ServoChannel::ELEVATOR));
}

void test_sticks_drive_surfaces_but_throttle_is_blocked_until_armed()
{
    Plane plane;
    RcChannels rc;
    rc.set(Channels::AILERON, 2000).set(Channels::THROTTLE, 1800);
    plane.tick(rc);

    TEST_ASSERT_FALSE(plane.controller.isArmed());
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, plane.pwm(ServoChannel::ESC));
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
    TEST_ASSERT_EQUAL_UINT16(1650, plane.pwm(ServoChannel::ESC));
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
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, plane.pwm(ServoChannel::ESC));
}

// В воздухе (armed) без связи: мотор выключен, автопилот держит
// планирование — рули отвечают на крен, а не стоят в нейтрали.
void test_link_loss_in_air_glides_with_motor_off()
{
    Plane plane;
    plane.arm();
    RcChannels rc;
    rc.set(Channels::ARM, 2000).set(Channels::THROTTLE, 1700).set(Channels::FLAPS, 2000);
    for (int i = 0; i < 10; ++i) plane.tick(rc);

    plane.imu.data.roll = 20;   // правое крыло вниз
    RcChannels lost = rc;
    lost.set(Channels::THROTTLE, 900);   // failsafe-значение от приёмника
    plane.tick(lost);

    TEST_ASSERT_TRUE(plane.autopilot.isFailsafeGliding());
    TEST_ASSERT_TRUE(plane.controller.isArmed());   // ARM не сбрасывается
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, plane.pwm(ServoChannel::ESC));
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
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, plane.pwm(ServoChannel::ESC));
}

void test_mode_switch_is_ignored_during_link_loss()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_MANUAL, plane.autopilot.getMode());

    RcChannels lost;
    lost.set(Channels::THROTTLE, 900).set(Channels::AUX_2, 2000);   // CH7 в failsafe-кадре
    plane.tick(lost);
    TEST_ASSERT_EQUAL(MODE_MANUAL, plane.autopilot.getMode());

    rc.set(Channels::AUX_2, 1500);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, plane.autopilot.getMode());
}

void test_autopilot_corrections_are_added_to_sticks_and_clamped()
{
    Plane plane;
    plane.imu.data.pitch = -10;   // нос вниз -> коррекция "нос вверх"
    RcChannels rc;
    rc.set(Channels::AUX_2, 1500);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_STABILIZE, plane.autopilot.getMode());

    const uint16_t elevator = plane.pwm(ServoChannel::ELEVATOR);
    // Нос вверх = руль высоты вверх; в PWM знак зависит от реверса.
    TEST_ASSERT_TRUE(Config::ELEVATOR_REVERSED ? elevator < 1500 : elevator > 1500);

    // Стик до упора + коррекция в ту же сторону — не дальше хода.
    plane.imu.data.pitch = -60;
    rc.set(Channels::ELEVATOR, 1000);   // на себя — нос вверх
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(Config::ELEVATOR_REVERSED ? 1000 : 2000, plane.pwm(ServoChannel::ELEVATOR));
}

void test_takeoff_throttle_still_requires_arm()
{
    Plane plane;
    RcChannels rc;
    rc.set(Channels::AUX_2, 2000).set(Channels::THROTTLE, 1000);
    plane.tick(rc);
    TEST_ASSERT_EQUAL(MODE_AUTO_TAKEOFF, plane.autopilot.getMode());

    rc.set(Channels::ARM, 2000);
    plane.tick(rc);
    TEST_ASSERT_TRUE(plane.controller.isArmed());
    rc.set(Channels::THROTTLE, 1600);
    for (int i = 0; i < 600; ++i) plane.tick(rc);   // 1.2 с программы
    TEST_ASSERT_EQUAL_UINT16(2000, plane.pwm(ServoChannel::ESC));

    rc.set(Channels::ARM, 1000);   // DISARM — газ сразу в ноль, хотя режим хочет 100 %
    plane.tick(rc);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, plane.pwm(ServoChannel::ESC));
}

void test_flaps_follow_switch_smoothly()
{
    Plane plane;
    RcChannels rc;
    plane.tick(rc);   // первый такт — закрылки убраны
    rc.set(Channels::FLAPS, 2000);
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
    IBusReceiver receiver(board.rcUart());
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
    TEST_ASSERT_EQUAL_UINT16(1400, board.servos[ServoChannel::ESC].lastUs);

    fake::advanceMs(600);
    controller.update();
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, board.servos[ServoChannel::ESC].lastUs);
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
    return UNITY_END();
}
