// ============================================================
// RC: снимок каналов, преобразования RcInput и приёмник: iBUS (кадры,
// CRC, 12-битные значения, потеря связи, пересинхронизация), SBUS
// (8E2 с инверсией, флаг failsafe, SBUS2) и CRSF (CRC8, качество связи).
//
// Запуск: pio test -e native -f native/test_rc
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "rc/Packed11.h"
#include "rc/RcReceiver.h"
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
    for (uint8_t ch = 0; ch < Config::RC_CHANNELS; ++ch)
    {
        const uint16_t expected = ch == Channels::THROTTLE ? Config::PWM_MIN : Config::PWM_CENTER;
        TEST_ASSERT_EQUAL_UINT16(expected, state.get(ch));
    }
}

void test_channel_state_ignores_out_of_range_indices()
{
    RcChannelState state;
    state.set(Channels::AILERON, 1900);
    state.set(Config::RC_CHANNELS, 1234);   // игнорируется
    TEST_ASSERT_EQUAL_UINT16(1900, state.get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(Config::PWM_CENTER, state.get(Config::RC_CHANNELS));
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
// RcReceiver: iBUS
// ------------------------------------------------------------

void test_receiver_opens_uart_at_ibus_speed()
{
    FakeUart uart;
    RcReceiver receiver(uart);
    receiver.begin();
    TEST_ASSERT_EQUAL_UINT32(115200, uart.baud);
    TEST_ASSERT_TRUE(uart.format == UartFormat::STANDARD_8N1);
    TEST_ASSERT_TRUE(receiver.isPortReady());
    TEST_ASSERT_EQUAL_STRING("iBUS", receiver.getProtocolName());
    TEST_ASSERT_EQUAL_INT16(-1, receiver.getLinkQuality());   // iBUS качество связи не сообщает
}

void test_signal_is_lost_until_first_frame()
{
    FakeUart uart;
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1900).set(Channels::THROTTLE, 1200).set(Channels::VRB, 1100);
    fake::advanceMs(5);
    uart.push(ibusFrame(rc));
    receiver.update();

    TEST_ASSERT_FALSE(receiver.isSignalLost());
    TEST_ASSERT_EQUAL_UINT16(1900, receiver.getState().get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(1200, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_EQUAL_UINT16(1100, receiver.getState().get(Channels::VRB));
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getBadFrameCount());
    TEST_ASSERT_EQUAL_UINT32(micros(), receiver.getLastFrameTime());
}

void test_frame_split_across_reads_is_assembled()
{
    FakeUart uart;
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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
    RcReceiver receiver(uart);
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

// ------------------------------------------------------------
// 16 каналов по 11 бит (SBUS, CRSF)
// ------------------------------------------------------------

void test_packed11_maps_endpoints_and_center()
{
    TEST_ASSERT_EQUAL_UINT16(988, Packed11::toMicros(172));
    TEST_ASSERT_EQUAL_UINT16(1500, Packed11::toMicros(992));
    TEST_ASSERT_EQUAL_UINT16(2012, Packed11::toMicros(1811));
}

void test_packed11_unpacks_every_channel_in_order()
{
    // Каналам — разные значения через все границы байтов.
    uint16_t raw[16];
    for (uint8_t ch = 0; ch < 16; ++ch) raw[ch] = static_cast<uint16_t>(172 + ch * 101);
    uint8_t data[22] = {};
    uint32_t bits = 0;
    uint8_t count = 0, next = 0;
    for (uint16_t value : raw)
    {
        bits |= static_cast<uint32_t>(value) << count;
        count += 11;
        while (count >= 8)
        {
            data[next++] = static_cast<uint8_t>(bits & 0xFF);
            bits >>= 8;
            count -= 8;
        }
    }
    TEST_ASSERT_EQUAL_UINT8(22, next);

    uint16_t us[16];
    Packed11::unpack(data, us);
    for (uint8_t ch = 0; ch < 16; ++ch) TEST_ASSERT_EQUAL_UINT16(Packed11::toMicros(raw[ch]), us[ch]);
}

// ------------------------------------------------------------
// RcReceiver: SBUS
// ------------------------------------------------------------

void test_sbus_opens_uart_8e2_inverted_at_100000()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();
    TEST_ASSERT_EQUAL_UINT32(100000, uart.baud);
    TEST_ASSERT_TRUE(uart.format == UartFormat::SBUS_8E2_INVERTED);
    TEST_ASSERT_TRUE(receiver.isPortReady());
    TEST_ASSERT_EQUAL_STRING("SBUS", receiver.getProtocolName());
}

// Порт без инверсии: кадров не будет — приёмник это знает, связь «потеряна».
void test_sbus_on_port_without_inversion_is_not_ready()
{
    FakeUart uart;
    uart.supportsSbus = false;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();
    TEST_ASSERT_FALSE(receiver.isPortReady());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
}

void test_sbus_frame_updates_channels()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1900).set(Channels::THROTTLE, 1200).set(Channels::VRB, 1100).set(Channels::SWC, 2000);
    uart.push(sbusFrame(rc));
    receiver.update();

    TEST_ASSERT_FALSE(receiver.isSignalLost());
    TEST_ASSERT_EQUAL_UINT16(1900, receiver.getState().get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(1200, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_EQUAL_UINT16(1100, receiver.getState().get(Channels::VRB));
    TEST_ASSERT_EQUAL_UINT16(2000, receiver.getState().get(Channels::SWC));
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getBadFrameCount());
}

void test_sbus_failsafe_flag_means_link_lost_but_lost_frame_does_not()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::THROTTLE, 1100);
    uart.push(sbusFrame(rc, SbusParser::FLAG_FRAME_LOST));   // один кадр по радио потерян — не беда
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());

    uart.push(sbusFrame(rc, SbusParser::FLAG_FAILSAFE | SbusParser::FLAG_FRAME_LOST));
    receiver.update();
    TEST_ASSERT_TRUE(receiver.isFailsafeReported());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
    TEST_ASSERT_FALSE(receiver.isFrameTimeout());

    uart.push(sbusFrame(rc));
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());
}

// Failsafe приёмника «газ ниже нормы» работает и для SBUS.
void test_sbus_low_throttle_failsafe_counts_too()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::THROTTLE, 900);
    uart.push(sbusFrame(rc));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT16(900, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_TRUE(receiver.isFailsafeReported());
}

void test_sbus2_end_bytes_are_accepted_and_bad_end_is_not()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::ELEVATOR, 1300).set(Channels::THROTTLE, 1100);
    uart.push(sbusFrame(rc, 0, 0x14));
    uart.push(sbusFrame(rc, 0, 0x34));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(2, receiver.getGoodFrameCount());

    uart.push(sbusFrame(rc, 0, 0x55));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(2, receiver.getGoodFrameCount());
    TEST_ASSERT_TRUE(receiver.getBadFrameCount() >= 1);
}

// Подключились посреди потока: ложное начало (0x0F в данных) не мешает
// поймать следующие кадры.
void test_sbus_resyncs_after_joining_mid_stream()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::RUDDER, 1700).set(Channels::THROTTLE, 1100);
    const std::string frame = sbusFrame(rc);
    uart.push(std::string("\x0F\x11\x22\x33\x44\x55\x66\x77", 8) + frame + frame);
    receiver.update();

    TEST_ASSERT_TRUE(receiver.getGoodFrameCount() >= 1);
    TEST_ASSERT_EQUAL_UINT16(1700, receiver.getState().get(Channels::RUDDER));
    TEST_ASSERT_FALSE(receiver.isSignalLost());
}

void test_sbus_frame_split_across_reads_is_assembled()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::SBUS);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1250).set(Channels::THROTTLE, 1100);
    const std::string frame = sbusFrame(rc);
    uart.push(frame.substr(0, 10));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getGoodFrameCount());
    uart.push(frame.substr(10));
    receiver.update();
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1250, receiver.getState().get(Channels::AILERON));
}

// ------------------------------------------------------------
// RcReceiver: CRSF
// ------------------------------------------------------------

void test_crsf_opens_uart_8n1_at_420000()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();
    TEST_ASSERT_EQUAL_UINT32(420000, uart.baud);
    TEST_ASSERT_TRUE(uart.format == UartFormat::STANDARD_8N1);
    TEST_ASSERT_EQUAL_STRING("CRSF", receiver.getProtocolName());
    TEST_ASSERT_EQUAL_INT16(-1, receiver.getLinkQuality());   // статистики ещё не было
}

// CRC8 DVB-S2 (полином 0xD5): проверочное значение для "123456789".
void test_crsf_crc8_matches_reference()
{
    const uint8_t text[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX8(0xBC, CrsfParser::crc8(text, sizeof(text)));
}

void test_crsf_channels_frame_updates_channels()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::AILERON, 1100).set(Channels::THROTTLE, 1300).set(Channels::ARM, 2000);
    uart.push(crsfChannels(rc));
    receiver.update();

    TEST_ASSERT_FALSE(receiver.isSignalLost());
    TEST_ASSERT_EQUAL_UINT16(1100, receiver.getState().get(Channels::AILERON));
    TEST_ASSERT_EQUAL_UINT16(1300, receiver.getState().get(Channels::THROTTLE));
    TEST_ASSERT_EQUAL_UINT16(2000, receiver.getState().get(Channels::ARM));
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
}

void test_crsf_bad_crc_is_counted_and_ignored()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();

    RcChannels good;
    good.set(Channels::AILERON, 1600).set(Channels::THROTTLE, 1100);
    RcChannels noisy;
    noisy.set(Channels::AILERON, 1900).set(Channels::THROTTLE, 1100);
    uart.push(crsfChannels(good) + crsfChannels(noisy, true));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getBadFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1600, receiver.getState().get(Channels::AILERON));
}

// ExpressLRS в режиме «последние значения»: каналы идут, но LQ = 0.
void test_crsf_zero_link_quality_means_link_lost()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::THROTTLE, 1100);
    uart.push(crsfLinkStats(96) + crsfChannels(rc));
    receiver.update();
    TEST_ASSERT_EQUAL_INT16(96, receiver.getLinkQuality());
    TEST_ASSERT_FALSE(receiver.isSignalLost());

    uart.push(crsfLinkStats(0) + crsfChannels(rc));
    receiver.update();
    TEST_ASSERT_TRUE(receiver.isFailsafeReported());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
    TEST_ASSERT_FALSE(receiver.isFrameTimeout());

    uart.push(crsfLinkStats(80));
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());
}

// ExpressLRS по умолчанию: при потере связи каналы просто перестают идти.
void test_crsf_silence_is_frame_timeout()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();
    uart.push(crsfChannels(RcChannels()));
    receiver.update();
    TEST_ASSERT_FALSE(receiver.isSignalLost());

    fake::advanceUs(Config::RX_TIMEOUT_US + 1);
    uart.push(crsfLinkStats(0));   // статистика без каналов связь не «держит»
    receiver.update();
    TEST_ASSERT_TRUE(receiver.isFrameTimeout());
    TEST_ASSERT_TRUE(receiver.isSignalLost());
}

void test_crsf_other_frames_and_garbage_are_skipped()
{
    FakeUart uart;
    RcReceiver receiver(uart, Config::RxProtocol::CRSF);
    receiver.begin();

    RcChannels rc;
    rc.set(Channels::ELEVATOR, 1750).set(Channels::THROTTLE, 1100);
    const std::string deviceInfo = crsfFrame(0x29, std::string("OpenPlane\0", 10));
    uart.push(std::string("\x00\x13\xEE\x01", 4) + deviceInfo + std::string("\xC8\x00", 2) + crsfChannels(rc));
    receiver.update();

    TEST_ASSERT_EQUAL_UINT32(1, receiver.getGoodFrameCount());
    TEST_ASSERT_EQUAL_UINT16(1750, receiver.getState().get(Channels::ELEVATOR));
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
    RUN_TEST(test_packed11_maps_endpoints_and_center);
    RUN_TEST(test_packed11_unpacks_every_channel_in_order);
    RUN_TEST(test_sbus_opens_uart_8e2_inverted_at_100000);
    RUN_TEST(test_sbus_on_port_without_inversion_is_not_ready);
    RUN_TEST(test_sbus_frame_updates_channels);
    RUN_TEST(test_sbus_failsafe_flag_means_link_lost_but_lost_frame_does_not);
    RUN_TEST(test_sbus_low_throttle_failsafe_counts_too);
    RUN_TEST(test_sbus2_end_bytes_are_accepted_and_bad_end_is_not);
    RUN_TEST(test_sbus_resyncs_after_joining_mid_stream);
    RUN_TEST(test_sbus_frame_split_across_reads_is_assembled);
    RUN_TEST(test_crsf_opens_uart_8n1_at_420000);
    RUN_TEST(test_crsf_crc8_matches_reference);
    RUN_TEST(test_crsf_channels_frame_updates_channels);
    RUN_TEST(test_crsf_bad_crc_is_counted_and_ignored);
    RUN_TEST(test_crsf_zero_link_quality_means_link_lost);
    RUN_TEST(test_crsf_silence_is_frame_timeout);
    RUN_TEST(test_crsf_other_frames_and_garbage_are_skipped);
    return UNITY_END();
}
