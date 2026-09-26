// ============================================================
// CONTROL: закрылки, микшер (знаки, реверс, флапероны), газ, ARM
// (автомат тумблера и проверки датчиков режима), таблица выходов.
//
// Запуск: pio test -e native -f native/test_control
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/Autopilot.h"
#include "control/ArmingManager.h"
#include "control/ControlCommand.h"
#include "control/ControlMixer.h"
#include "control/FlapsController.h"
#include "control/FlightOutputState.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/RcChannelState.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

static RcChannelState rcWith(uint8_t channel, uint16_t value)
{
    RcChannelState rc;
    rc.set(channel, value);
    return rc;
}

// ------------------------------------------------------------
// Структуры данных
// ------------------------------------------------------------

void test_command_and_output_state_defaults()
{
    const ControlCommand command;
    TEST_ASSERT_EQUAL_INT16(0, command.roll);
    TEST_ASSERT_EQUAL_INT16(0, command.pitch);
    TEST_ASSERT_EQUAL_INT16(0, command.yaw);
    TEST_ASSERT_EQUAL_INT16(0, command.flaps);

    const FlightOutputState state;
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.aileronLeft);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.aileronRight);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.elevator);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.rudder);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, state.throttle);
}

// ------------------------------------------------------------
// FlapsController
// ------------------------------------------------------------

void test_flaps_first_call_jumps_to_target()
{
    FlapsController deployedAtBoot;
    TEST_ASSERT_EQUAL_INT16(Config::FLAPS_DEPLOYED_US, deployedAtBoot.update(true, 5000));

    FlapsController retractedAtBoot;
    TEST_ASSERT_EQUAL_INT16(0, retractedAtBoot.update(false, 5000));
}

void test_flaps_move_at_limited_rate()
{
    FlapsController flaps;
    flaps.update(false, 0);

    // Полный ход за FLAPS_TRANSITION_MS: за 100 мс — десятая часть.
    uint32_t now = 0;
    for (int i = 0; i < 10; ++i)
    {
        now += 10;
        flaps.update(true, now);
    }
    TEST_ASSERT_INT16_WITHIN(1, Config::FLAPS_DEPLOYED_US / 10, flaps.getPosition());

    for (int i = 0; i < 200; ++i)
    {
        now += 10;
        flaps.update(true, now);
    }
    TEST_ASSERT_EQUAL_INT16(Config::FLAPS_DEPLOYED_US, flaps.getPosition());   // не дальше цели

    now += 10;
    flaps.update(false, now);
    TEST_ASSERT_LESS_THAN_INT16(Config::FLAPS_DEPLOYED_US, flaps.getPosition());
}

void test_flaps_do_not_jump_after_long_pause()
{
    FlapsController flaps;
    flaps.update(false, 0);
    // 10 с без вызовов (калибровка из консоли) — один шаг не больше 20 мс хода.
    const int16_t position = flaps.update(true, 10000);
    TEST_ASSERT_INT16_WITHIN(1, Config::FLAPS_DEPLOYED_US * 20 / Config::FLAPS_TRANSITION_MS, position);
}

// ------------------------------------------------------------
// ControlMixer
// ------------------------------------------------------------

void test_sticks_to_command_signs()
{
    ControlMixer mixer;

    ControlCommand c = mixer.fromSticks(rcWith(Channels::AILERON, 2000), 0);
    TEST_ASSERT_EQUAL_INT16(500, c.roll);          // стик вправо — крен вправо

    c = mixer.fromSticks(rcWith(Channels::ELEVATOR, 2000), 0);
    TEST_ASSERT_EQUAL_INT16(-500, c.pitch);        // от себя — нос вниз

    c = mixer.fromSticks(rcWith(Channels::RUDDER, 1000), 0);
    TEST_ASSERT_EQUAL_INT16(-500, c.yaw);          // влево — нос влево

    c = mixer.fromSticks(RcChannelState(), 0);
    TEST_ASSERT_EQUAL_INT16(0, c.roll);
    TEST_ASSERT_EQUAL_INT16(0, c.pitch);
    TEST_ASSERT_EQUAL_INT16(0, c.yaw);
    TEST_ASSERT_EQUAL_INT16(0, c.flaps);
}

void test_flaps_switch_threshold_and_smooth_deploy()
{
    ControlMixer mixer;
    // Середина (1500, значение до первого кадра) — закрылки не выпускаются.
    TEST_ASSERT_EQUAL_INT16(0, mixer.fromSticks(rcWith(Channels::FLAPS, 1500), 0).flaps);

    const RcChannelState down = rcWith(Channels::FLAPS, Config::FLAPS_SWITCH_ON_US);
    uint32_t now = 0;
    int16_t flaps = 0;
    for (int i = 0; i < 150; ++i)
    {
        now += 10;
        flaps = mixer.fromSticks(down, now).flaps;
    }
    TEST_ASSERT_EQUAL_INT16(Config::FLAPS_DEPLOYED_US, flaps);
    TEST_ASSERT_EQUAL_INT16(Config::FLAPS_DEPLOYED_US, mixer.getFlaps());
}

void test_mix_applies_physical_signs_and_servo_reversal()
{
    ControlMixer mixer;
    ControlCommand c;

    c.roll = 200;   // крен вправо: левый элерон вниз, правый вверх
    FlightOutputState out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_LEFT_REVERSED ? 1300 : 1700, out.aileronLeft);
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_RIGHT_REVERSED ? 1700 : 1300, out.aileronRight);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, out.elevator);

    c = ControlCommand();
    c.pitch = 100;  // руль высоты реверсирован в Config
    out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(Config::ELEVATOR_REVERSED ? 1400 : 1600, out.elevator);

    c = ControlCommand();
    c.yaw = -300;
    out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(Config::RUDDER_REVERSED ? 1800 : 1200, out.rudder);
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, out.throttle);   // газ микшер не трогает
}

void test_mix_clamps_commands_and_outputs()
{
    ControlMixer mixer;
    ControlCommand c;
    c.roll = 900;
    c.pitch = -900;
    c.yaw = 900;
    const FlightOutputState out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(Config::AILERON_LEFT_REVERSED ? 1000 : 2000, out.aileronLeft);
    TEST_ASSERT_EQUAL_UINT16(Config::ELEVATOR_REVERSED ? 2000 : 1000, out.elevator);
    TEST_ASSERT_EQUAL_UINT16(Config::RUDDER_REVERSED ? 1000 : 2000, out.rudder);
}

// Флапероны: при полном крене опускающийся элерон упирается в край,
// а поднимающийся продолжает — дифференциал элеронов.
void test_flaperons_add_roll_on_top_of_flaps()
{
    ControlMixer mixer;
    ControlCommand c;
    c.flaps = Config::FLAPS_DEPLOYED_US;
    FlightOutputState out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(1500 + Config::FLAPS_DEPLOYED_US, out.aileronLeft);
    TEST_ASSERT_EQUAL_UINT16(1500 + Config::FLAPS_DEPLOYED_US, out.aileronRight);

    c.roll = 500;
    out = mixer.mix(c);
    TEST_ASSERT_EQUAL_UINT16(2000, out.aileronLeft);                                 // упор
    TEST_ASSERT_EQUAL_UINT16(1500 + Config::FLAPS_DEPLOYED_US - 500, out.aileronRight);
}

// ------------------------------------------------------------
// ThrottleManager
// ------------------------------------------------------------

void test_throttle_passes_clamped_stick_or_failsafe()
{
    ThrottleManager throttle;
    TEST_ASSERT_EQUAL_UINT16(1400, throttle.update(rcWith(Channels::THROTTLE, 1400), false));
    TEST_ASSERT_EQUAL_UINT16(2000, throttle.update(rcWith(Channels::THROTTLE, 2100), false));
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, throttle.update(rcWith(Channels::THROTTLE, 1800), true));
}

// ------------------------------------------------------------
// ArmingManager
// ------------------------------------------------------------

static RcChannelState armRc(bool switchOn, uint16_t throttle = 1000)
{
    RcChannelState rc;
    rc.set(Channels::ARM, switchOn ? 2000 : 1000);
    rc.set(Channels::THROTTLE, throttle);
    return rc;
}

void test_switch_on_at_boot_does_not_arm()
{
    ArmingManager arming;
    arming.update(armRc(true), false);
    TEST_ASSERT_FALSE(arming.isArmed());

    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());
    TEST_ASSERT_NULL(arming.getLastRefusalReason());
    TEST_ASSERT_TRUE(contains(takeSerial(), "ArmingManager: ARM"));
}

void test_disarm_immediately_when_switch_off()
{
    ArmingManager arming;
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());

    arming.update(armRc(false, 1800), false);   // газ не важен для DISARM
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_TRUE(contains(takeSerial(), "DISARM"));
}

void test_throttle_up_refuses_and_needs_new_off_on_cycle()
{
    ArmingManager arming;
    arming.update(armRc(false), false);
    arming.update(armRc(true, 1500), false);
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_EQUAL_STRING("газ не на минимуме", arming.getLastRefusalReason());

    // Газ убрали, тумблер не трогали — ARM не происходит.
    arming.update(armRc(true, 1000), false);
    TEST_ASSERT_FALSE(arming.isArmed());

    arming.update(armRc(false), false);
    TEST_ASSERT_NULL(arming.getLastRefusalReason());
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());
}

void test_link_loss_neither_arms_nor_disarms()
{
    ArmingManager arming;
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());

    arming.update(armRc(false), true);   // failsafe-кадр — тумблер не читается
    TEST_ASSERT_TRUE(arming.isArmed());
}

void test_stabilized_modes_require_responding_imu_without_preflight_problem()
{
    FakeImu imu;
    FakeBaro baro;
    Autopilot autopilot(&imu, &baro);
    ArmingManager arming(&autopilot);

    autopilot.setMode(MODE_STABILIZE);
    imu.available = false;
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_TRUE(contains(arming.getLastRefusalReason(), "IMU не отвечает"));

    imu.available = true;
    imu.preflightProblem = "IMU: самолёт двигали";
    arming.update(armRc(false), false);
    autopilot.setMode(MODE_AUTO_TAKEOFF);
    arming.update(armRc(true), false);
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_EQUAL_STRING("IMU: самолёт двигали", arming.getLastRefusalReason());

    imu.preflightProblem = nullptr;
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());
}

void test_alt_hold_requires_barometer_and_manual_needs_nothing()
{
    FakeImu imu;
    FakeBaro baro;
    Autopilot autopilot(&imu, &baro);
    ArmingManager arming(&autopilot);

    autopilot.setMode(MODE_ALT_HOLD);
    baro.available = false;
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_TRUE(contains(arming.getLastRefusalReason(), "барометр"));

    // MANUAL армится без единого рабочего датчика.
    imu.available = false;
    autopilot.setMode(MODE_MANUAL);
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());
}

void test_sensors_absent_from_build_do_not_block_arming()
{
    Autopilot autopilot;   // без датчиков
    ArmingManager arming(&autopilot);
    autopilot.setMode(MODE_STABILIZE);
    arming.update(armRc(false), false);
    arming.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming.isArmed());

    Autopilot altHold;
    altHold.setMode(MODE_ALT_HOLD);
    ArmingManager arming2(&altHold);
    arming2.update(armRc(false), false);
    arming2.update(armRc(true), false);
    TEST_ASSERT_TRUE(arming2.isArmed());
}

// ------------------------------------------------------------
// FlightOutputs
// ------------------------------------------------------------

void test_output_table_matches_servo_channels()
{
    TEST_ASSERT_EQUAL_STRING("aileronLeft", FlightOutputs::outputInfo(ServoChannel::AILERON_LEFT).key);
    TEST_ASSERT_EQUAL_STRING("aileronRight", FlightOutputs::outputInfo(ServoChannel::AILERON_RIGHT).key);
    TEST_ASSERT_EQUAL_STRING("elevator", FlightOutputs::outputInfo(ServoChannel::ELEVATOR).key);
    TEST_ASSERT_EQUAL_STRING("esc", FlightOutputs::outputInfo(ServoChannel::ESC).key);
    TEST_ASSERT_EQUAL_STRING("rudder", FlightOutputs::outputInfo(ServoChannel::RUDDER).key);
    TEST_ASSERT_FALSE(FlightOutputs::outputInfo(ServoChannel::RUDDER).required);

    FlightOutputState s;
    s.aileronLeft = 1001;
    s.aileronRight = 1002;
    s.elevator = 1003;
    s.throttle = 1004;
    s.rudder = 1005;
    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        TEST_ASSERT_EQUAL_UINT16(1001 + ch, FlightOutputs::valueOf(s, ch));
    }
}

void test_outputs_begin_attaches_all_and_reports_required()
{
    FakeBoard board;
    FlightOutputs outputs(board);
    TEST_ASSERT_TRUE(outputs.begin());
    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        TEST_ASSERT_TRUE(outputs.isAttached(ch));
        TEST_ASSERT_EQUAL_UINT16(Config::PWM_MIN, board.servos[ch].minUs);
        TEST_ASSERT_EQUAL_UINT16(Config::PWM_MAX, board.servos[ch].maxUs);
    }
    TEST_ASSERT_FALSE(outputs.isAttached(ServoChannel::COUNT));
    TEST_ASSERT_TRUE(contains(takeSerial(), "aileronLeft(GPIO4)=OK"));

    // Необязательный руль направления не мешает, обязательный ESC — да.
    FakeBoard noRudder;
    noRudder.servos[ServoChannel::RUDDER].attachResult = false;
    FlightOutputs outputs2(noRudder);
    TEST_ASSERT_TRUE(outputs2.begin());

    FakeBoard noEsc;
    noEsc.servos[ServoChannel::ESC].attachResult = false;
    FlightOutputs outputs3(noEsc);
    TEST_ASSERT_FALSE(outputs3.begin());
    TEST_ASSERT_TRUE(contains(takeSerial(), "esc(GPIO7)=FAIL"));
}

void test_outputs_write_and_failsafe()
{
    FakeBoard board;
    FlightOutputs outputs(board);
    outputs.begin();

    FlightOutputState s;
    s.aileronLeft = 1100;
    s.throttle = 1600;
    outputs.write(s);
    TEST_ASSERT_EQUAL_UINT16(1100, board.servos[ServoChannel::AILERON_LEFT].lastUs);
    TEST_ASSERT_EQUAL_UINT16(1600, board.servos[ServoChannel::ESC].lastUs);
    TEST_ASSERT_EQUAL_UINT16(1600, outputs.getLastState().throttle);

    outputs.setFailsafe();
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_AILERON, board.servos[ServoChannel::AILERON_LEFT].lastUs);
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_AILERON, board.servos[ServoChannel::AILERON_RIGHT].lastUs);
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_ELEVATOR, board.servos[ServoChannel::ELEVATOR].lastUs);
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_RUDDER, board.servos[ServoChannel::RUDDER].lastUs);
    TEST_ASSERT_EQUAL_UINT16(Config::FAILSAFE_THROTTLE, board.servos[ServoChannel::ESC].lastUs);
}

void test_pulse_self_test_reports_match_mismatch_and_missing()
{
    FakeBoard board;
    FlightOutputs outputs(board);
    outputs.begin();
    outputs.setFailsafe();
    takeSerial();

    board.servos[ServoChannel::AILERON_LEFT].pulse = 1505;   // в допуске ±15
    board.servos[ServoChannel::AILERON_RIGHT].pulse = 1000;  // провод не туда
    board.servos[ServoChannel::ELEVATOR].pulse = -1;         // нет импульса
    board.servos[ServoChannel::ESC].pulse = 1000;
    board.servos[ServoChannel::RUDDER].pulse = 1500;
    outputs.printPulseSelfTest();

    const std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "элерон L GPIO4: 1505 / 1500  OK"));
    TEST_ASSERT_TRUE(contains(log, "элерон R GPIO5: 1000 / 1500  НЕ СОВПАДАЕТ"));
    TEST_ASSERT_TRUE(contains(log, "руль выс GPIO6: нет импульса / 1500  НЕ СОВПАДАЕТ"));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_command_and_output_state_defaults);
    RUN_TEST(test_flaps_first_call_jumps_to_target);
    RUN_TEST(test_flaps_move_at_limited_rate);
    RUN_TEST(test_flaps_do_not_jump_after_long_pause);
    RUN_TEST(test_sticks_to_command_signs);
    RUN_TEST(test_flaps_switch_threshold_and_smooth_deploy);
    RUN_TEST(test_mix_applies_physical_signs_and_servo_reversal);
    RUN_TEST(test_mix_clamps_commands_and_outputs);
    RUN_TEST(test_flaperons_add_roll_on_top_of_flaps);
    RUN_TEST(test_throttle_passes_clamped_stick_or_failsafe);
    RUN_TEST(test_switch_on_at_boot_does_not_arm);
    RUN_TEST(test_disarm_immediately_when_switch_off);
    RUN_TEST(test_throttle_up_refuses_and_needs_new_off_on_cycle);
    RUN_TEST(test_link_loss_neither_arms_nor_disarms);
    RUN_TEST(test_stabilized_modes_require_responding_imu_without_preflight_problem);
    RUN_TEST(test_alt_hold_requires_barometer_and_manual_needs_nothing);
    RUN_TEST(test_sensors_absent_from_build_do_not_block_arming);
    RUN_TEST(test_output_table_matches_servo_channels);
    RUN_TEST(test_outputs_begin_attaches_all_and_reports_required);
    RUN_TEST(test_outputs_write_and_failsafe);
    RUN_TEST(test_pulse_self_test_reports_match_mismatch_and_missing);
    return UNITY_END();
}
