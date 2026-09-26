// ============================================================
// RC: снимок каналов, преобразования RcInput и разбор iBUS
// (кадры, CRC, 12-битные значения, потеря связи, пересинхронизация).
//
// Запуск: pio test -e native -f native/test_rc
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "rc/IBusReceiver.h"
#include "rc/RcChannelState.h"
#include "rc/RcInput.h"
#include "helpers/TestSupport.h"

void setUp() { resetWorld(); }
void tearDown() {}

// ------------------------------------------------------------
// RcChannelState
// ------------------------------------------------------------

void test_channel_state_defaults_are_safe()
{
    RcChannelState state;
    for (uint8_t ch = 0; ch < Config::IBUS_CHANNELS; ++ch)
    {
        const uint16_t expected = ch == Channels::THROTTLE ? Config::PWM_MIN : Config::PWM_CENTER;
        TEST_ASSERT_EQUAL_UINT16(expected, state.get(ch));
    }
}

void test_channel_state_ignores_out_of_range_indices()
{
    RcChannelState state;
    state.set(Channels::AILERON, 1900);
    state.set(Config::IBUS_CHANNELS, 1234);   // игнорируется
    TEST_ASSERT_EQUAL_UINT16(1900, state.get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.get(Config::IBUS_CHANNELS));
    TEST_ASSERT_EQUAL_UINT16(1900, state.data()[Channels::AILERON]);

    state.reset();
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.get(Channels::AILERON));
}

// ------------------------------------------------------------
// RcInput
// ------------------------------------------------------------

void test_clamp_limits_to_standard_pulse_range()
{
    TEST_ASSERT_EQUAL_UINT16(1000, RcInput::clamp(900));
    TEST_ASSERT_EQUAL_UINT16(1500, RcInput::clamp(1500));
    TEST_ASSERT_EQUAL_UINT16(2000, RcInput::clamp(2100));
}

void test_centered_maps_linearly_and_reverses()
{
    TEST_ASSERT_EQUAL_INT16(-500, RcInput::centered(1000, 500));
    TEST_ASSERT_EQUAL_INT16(0, RcInput::centered(1500, 500));
    TEST_ASSERT_EQUAL_INT16(250, RcInput::centered(1750, 500));
    TEST_ASSERT_EQUAL_INT16(500, RcInput::centered(2000, 500));
    TEST_ASSERT_EQUAL_INT16(-250, RcInput::centered(1750, 500, true));
    TEST_ASSERT_EQUAL_INT16(110, RcInput::centered(1750, 220));

    // Вход за пределами ограничивается до преобразования.
    TEST_ASSERT_EQUAL_INT16(500, RcInput::centered(2400, 500));
    TEST_ASSERT_EQUAL_INT16(-500, RcInput::centered(0, 500));
}

// ------------------------------------------------------------
// IBusReceiver
// ------------------------------------------------------------

void test_receiver_opens_uart_at_ibus_speed()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();
    TEST_ASSERT_EQUAL_UINT32(115200, uart.baud);
}

void test_signal_is_lost_until_first_frame()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();
    receiver.update();

    // Каналы по умолчанию (1500) не должны приниматься за команды пульта.
    TEST_ASSERT_TRUE(receiver.isFrameTimeout());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
    TEST_ASSERT_FALSE(receiver.isFailsafeReported());
}

void test_valid_frame_updates_channels_and_counters()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1900).set(Channels::THROTTLE, 1200).set(Channels::AUX_5, 1100);
    fake::advanceMs(5);
    uart.push(ibusFrame(rc));
    receiver.update();

    TEST_ASSERT_FALSE(receiver.isSignalLost());
    TEST_ASSERT_EQUAL_UINT16(1900, receiver.getState().get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(1200, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_EQUAL_UINT16(1100, receiver.getState().get(Channels::AUX_5));
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getBadFrameCount());
    TEST_ASSERT_EQUAL_UINT32(micros(), receiver.getLastFrameTime());
}

void test_frame_split_across_reads_is_assembled()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::ELEVATOR, 1300).set(Channels::THROTTLE, 1100);
    const std::string frame = ibusFrame(rc);

    uart.push(frame.substr(0, 7));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getGoodFrameCount());

    uart.push(frame.substr(7));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1300, receiver.getState().get(Channels::ELEVATOR));
}

void test_bad_crc_frame_is_counted_and_ignored()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels good;
    good.set(Channels::AILERON, 1600).set(Channels::THROTTLE, 1100);
    uart.push(ibusFrame(good));
    RcChannels noisy;
    noisy.set(Channels::AILERON, 1999).set(Channels::THROTTLE, 1100);
    uart.push(ibusFrame(noisy, 0, true));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getBadFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1600, receiver.getState().get(Channels::AILERON));
}

// В старших 4 битах FS-iA6B шлёт служебные данные: без маски failsafe
// по газу (0x2384 -> 900 мкс) читался как 9092 и не срабатывал.
void test_only_low_12_bits_are_channel_value()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::THROTTLE, 900);
    uart.push(ibusFrame(rc, 0x2));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT16(900, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_TRUE(receiver.isFailsafeReported());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
    TEST_ASSERT_FALSE(receiver.isFrameTimeout());
}

void test_failsafe_clears_on_next_normal_frame()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels lost;
    lost.set(Channels::THROTTLE, 940);
    uart.push(ibusFrame(lost));
    receiver.update();
    TEST_ASSERT_TRUE(receiver.isSignalLost());

    RcChannels back;
    back.set(Channels::THROTTLE, 1000);
    uart.push(ibusFrame(back));
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());
}

void test_frame_timeout_after_500ms_without_frames()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    uart.push(ibusFrame(RcChannels()));
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());

    fake::advanceUs(Config::RX_TIMEOUT_US);
    TEST_ASSERT_FALSE(receiver.isFrameTimeout());   // ровно 500 мс — ещё нет
    fake::advanceUs(1);
    TEST_ASSERT_TRUE(receiver.isFrameTimeout());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
}

void test_timeout_survives_micros_overflow()
{
    fake::setTimeUs(0xFFFFFFFFULL - 100000);   // за 0.1 с до переполнения micros()
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();
    uart.push(ibusFrame(RcChannels()));
    receiver.update();

    fake::advanceMs(300);   // micros() перешёл через 0
    TEST_ASSERT_FALSE(receiver.isSignalLost());
    fake::advanceMs(300);
    TEST_ASSERT_TRUE(receiver.isSignalLost());
}

void test_garbage_before_header_is_skipped()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::RUDDER, 1700).set(Channels::THROTTLE, 1100);
    uart.push(std::string("\x00\x13\x40\x20\x11", 5) + ibusFrame(rc));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1700, receiver.getState().get(Channels::RUDDER));
}

// Одиночный 0x20 прямо перед настоящим кадром: второй 0x20 не 0x40,
// но сам начинает кадр — кадр не должен теряться.
void test_resync_when_stray_header_byte_precedes_frame()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1234).set(Channels::THROTTLE, 1100);
    uart.push(std::string("\x20", 1) + ibusFrame(rc));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1234, receiver.getState().get(Channels::AILERON));
}

void test_consecutive_frames_are_all_parsed()
{
    FakeUart uart;
    IBusReceiver receiver(uart);
    receiver.begin();

    for (uint16_t i = 0; i < 5; ++i)
    {
        RcChannels rc;
        rc.set(Channels::AILERON, static_cast<uint16_t>(1000 + i * 100)).set(Channels::THROTTLE, 1100);
        uart.push(ibusFrame(rc));
    }
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(5, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1400, receiver.getState().get(Channels::AILERON));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_channel_state_defaults_are_safe);
    RUN_TEST(test_channel_state_ignores_out_of_range_indices);
    RUN_TEST(test_clamp_limits_to_standard_pulse_range);
    RUN_TEST(test_centered_maps_linearly_and_reverses);
    RUN_TEST(test_receiver_opens_uart_at_ibus_speed);
    RUN_TEST(test_signal_is_lost_until_first_frame);
    RUN_TEST(test_valid_frame_updates_channels_and_counters);
    RUN_TEST(test_frame_split_across_reads_is_assembled);
    RUN_TEST(test_bad_crc_frame_is_counted_and_ignored);
    RUN_TEST(test_only_low_12_bits_are_channel_value);
    RUN_TEST(test_failsafe_clears_on_next_normal_frame);
    RUN_TEST(test_frame_timeout_after_500ms_without_frames);
    RUN_TEST(test_timeout_survives_micros_overflow);
    RUN_TEST(test_garbage_before_header_is_skipped);
    RUN_TEST(test_resync_when_stray_header_byte_precedes_frame);
    RUN_TEST(test_consecutive_frames_are_all_parsed);
    return UNITY_END();
}
