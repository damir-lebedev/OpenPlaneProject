// ============================================================
// COPTER: блоки мультикоптера по отдельности — рамы, смеситель,
// фильтры, протоколы регуляторов, модель винтомоторной группы,
// настройка по описанию сборки, контур угловой скорости, ориентация,
// высота, фильтр гироскопа, ARM и защиты контроллера, реестр настроек
// (единицы и дефолты Betaflight, профили, флеш) и CLI.
//
// Полёты целиком (модель коптера + прошивка) — в test_copter_sim.
//
// Запуск: pio test -e native -f native/test_copter
// ============================================================

#include <unity.h>

#include <math.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#include "hal/Max7456.h"
#include "helpers/Osd7456Emulator.h"
#include "helpers/VtxEmulator.h"

#include "copter/AltitudeHold.h"
#include "copter/Beeper.h"
#include "copter/Battery.h"
#include "copter/Attitude.h"
#include "copter/CopterCli.h"
#include "copter/CopterController.h"
#include "copter/CrsfTelemetry.h"
#include "copter/GpsHome.h"
#include "copter/Osd.h"
#include "copter/EscProtocol.h"
#include "copter/Filters.h"
#include "copter/Frames.h"
#include "copter/GyroFilter.h"
#include "copter/Mixer.h"
#include "copter/Params.h"
#include "copter/Presets.h"
#include "copter/Propulsion.h"
#include "copter/RateController.h"
#include "copter/RcSetup.h"
#include "copter/Vtx.h"

using namespace copter;

void setUp() {}
void tearDown() {}

// ---------------- рамы ----------------

static const FrameType ALL_FRAMES[] = {
    FrameType::QUAD_X, FrameType::QUAD_X_STRETCHED, FrameType::QUAD_H, FrameType::QUAD_DEADCAT,
    FrameType::QUAD_PLUS, FrameType::HEX_X, FrameType::HEX_PLUS, FrameType::Y6,
    FrameType::OCTO_X, FrameType::OCTO_PLUS, FrameType::X8,
};

void test_frames_have_expected_motor_counts()
{
    const uint8_t expected[] = { 4, 4, 4, 4, 4, 6, 6, 6, 8, 8, 8 };
    for (uint8_t i = 0; i < sizeof(expected); ++i)
    {
        const Frame f = buildFrame(ALL_FRAMES[i]);
        TEST_ASSERT_EQUAL_UINT8(expected[i], f.count);
        TEST_ASSERT_NOT_EQUAL(0, strlen(frameName(ALL_FRAMES[i])));
    }
    TEST_ASSERT_TRUE(buildFrame(FrameType::Y6).coaxial);
    TEST_ASSERT_TRUE(buildFrame(FrameType::X8).coaxial);
    TEST_ASSERT_FALSE(buildFrame(FrameType::HEX_X).coaxial);
}

// Симметричная рама: на висении команды по осям не тянут ни в какую сторону,
// моторы по часовой и против уравновешены, у каждой оси максимум ровно 1.
void test_frames_are_balanced_and_normalized()
{
    for (FrameType type : ALL_FRAMES)
    {
        const Frame f = buildFrame(type);
        float sumYaw = 0, maxRoll = 0, maxPitch = 0, sumRollTorque = 0, sumPitchTorque = 0;
        for (uint8_t i = 0; i < f.count; ++i)
        {
            sumYaw += f.mix[i].yaw;
            maxRoll = fmaxf(maxRoll, fabsf(f.mix[i].roll));
            maxPitch = fmaxf(maxPitch, fabsf(f.mix[i].pitch));
            // момент от равной тяги всех моторов
            sumRollTorque += -f.place[i].y;
            sumPitchTorque += f.place[i].x;
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, sumYaw);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, maxRoll);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, maxPitch);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, sumRollTorque);
        if (type != FrameType::QUAD_DEADCAT) TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, sumPitchTorque);
    }
}

// Нумерация и вращение — как в Betaflight: 1 задний правый по часовой,
// 2 передний правый против, 3 задний левый против, 4 передний левый по часовой.
void test_quad_x_follows_betaflight_order()
{
    const Frame f = buildFrame(FrameType::QUAD_X);
    TEST_ASSERT_TRUE(f.place[0].x < 0 && f.place[0].y > 0);
    TEST_ASSERT_TRUE(f.place[1].x > 0 && f.place[1].y > 0);
    TEST_ASSERT_TRUE(f.place[2].x < 0 && f.place[2].y < 0);
    TEST_ASSERT_TRUE(f.place[3].x > 0 && f.place[3].y < 0);
    TEST_ASSERT_EQUAL_INT8(-1, static_cast<int8_t>(f.place[0].spin));
    TEST_ASSERT_EQUAL_INT8(1, static_cast<int8_t>(f.place[1].spin));
    TEST_ASSERT_EQUAL_INT8(1, static_cast<int8_t>(f.place[2].spin));
    TEST_ASSERT_EQUAL_INT8(-1, static_cast<int8_t>(f.place[3].spin));
    // крен вправо — левые моторы сильнее; нос вверх — передние; нос вправо — CCW
    TEST_ASSERT_TRUE(f.mix[2].roll > 0 && f.mix[3].roll > 0 && f.mix[0].roll < 0);
    TEST_ASSERT_TRUE(f.mix[1].pitch > 0 && f.mix[3].pitch > 0 && f.mix[0].pitch < 0);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.mix[1].yaw);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, f.mix[0].yaw);
}

void test_props_out_reverses_every_motor()
{
    const Frame in = buildFrame(FrameType::HEX_X);
    const Frame out = buildFrame(FrameType::HEX_X, true);
    for (uint8_t i = 0; i < in.count; ++i)
    {
        TEST_ASSERT_EQUAL_FLOAT(-in.mix[i].yaw, out.mix[i].yaw);
        TEST_ASSERT_EQUAL_FLOAT(in.mix[i].roll, out.mix[i].roll);
    }
}

void test_coaxial_pairs_share_an_arm_and_counter_rotate()
{
    const Frame f = buildFrame(FrameType::X8);
    for (uint8_t i = 0; i < 4; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.place[i].x, f.place[i + 4].x);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.place[i].y, f.place[i + 4].y);
        TEST_ASSERT_EQUAL_FLOAT(-f.mix[i].yaw, f.mix[i + 4].yaw);
        TEST_ASSERT_TRUE(f.place[i + 4].bottom);
    }
}

void test_describe_motor_in_words()
{
    char text[96];   // кириллица — по 2 байта на букву
    const Frame f = buildFrame(FrameType::QUAD_X);
    describeMotor(f, 0, text, sizeof(text));
    TEST_ASSERT_EQUAL_STRING("задний правый, по часовой", text);
    describeMotor(f, 3, text, sizeof(text));
    TEST_ASSERT_EQUAL_STRING("передний левый, по часовой", text);
    const Frame x8 = buildFrame(FrameType::X8);
    describeMotor(x8, 5, text, sizeof(text));
    TEST_ASSERT_EQUAL_STRING("передний правый, нижний, по часовой", text);
    describeMotor(f, 9, text, sizeof(text));
    TEST_ASSERT_EQUAL_STRING("", text);
}

// ---------------- смеситель ----------------

void test_mixer_passes_small_commands_unchanged()
{
    const Frame f = buildFrame(FrameType::QUAD_X);
    Mixer m;
    MixInput in;
    in.throttle = 0.5f;
    in.roll = 0.1f;
    in.pitch = -0.05f;
    in.yaw = 0.02f;
    const MixResult r = m.mix(f, in);
    TEST_ASSERT_FALSE(r.saturated);
    for (uint8_t i = 0; i < 4; ++i)
    {
        const float expected = 0.5f + 0.1f * f.mix[i].roll - 0.05f * f.mix[i].pitch + 0.02f * f.mix[i].yaw;
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected, r.motor[i]);
    }
}

// Airmode: на нулевом газе разница между моторами всё равно есть.
void test_mixer_airmode_keeps_authority_at_zero_throttle()
{
    const Frame f = buildFrame(FrameType::QUAD_X);
    Mixer m;
    MixInput in;
    in.throttle = 0.0f;
    in.roll = 0.3f;
    const MixResult r = m.mix(f, in);
    float lo = 1, hi = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        lo = fminf(lo, r.motor[i]);
        hi = fmaxf(hi, r.motor[i]);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, lo);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.6f, hi - lo);
    TEST_ASSERT_TRUE(r.throttle > 0.0f);

    m.setAirmode(false);
    TEST_ASSERT_FALSE(m.isAirmode());
    const MixResult off = m.mix(f, in);
    for (uint8_t i = 0; i < 4; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, off.motor[i]);
}

// Не влезает — сначала урезается рысканье, крен с тангажом сохраняются.
void test_mixer_cuts_yaw_before_roll_and_pitch()
{
    const Frame f = buildFrame(FrameType::QUAD_X);
    Mixer m;
    MixInput in;
    in.throttle = 0.5f;
    in.roll = 0.4f;
    in.yaw = 0.8f;
    const MixResult r = m.mix(f, in);
    TEST_ASSERT_TRUE(r.saturated);
    // крен: левые минус правые = 2·0.4 по нормированным коэффициентам
    const float rollSpan = (r.motor[2] + r.motor[3] - r.motor[0] - r.motor[1]) * 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.8f, rollSpan);
    for (uint8_t i = 0; i < 4; ++i)
    {
        TEST_ASSERT_TRUE(r.motor[i] >= 0.0f && r.motor[i] <= 1.0f);
    }
}

void test_mixer_scales_roll_and_pitch_together()
{
    const Frame f = buildFrame(FrameType::QUAD_X);
    Mixer m;
    MixInput in;
    in.throttle = 0.5f;
    in.roll = 1.0f;
    in.pitch = 0.5f;
    const MixResult r = m.mix(f, in);
    TEST_ASSERT_TRUE(r.saturated);
    const float roll = (r.motor[2] + r.motor[3] - r.motor[0] - r.motor[1]);
    const float pitch = (r.motor[1] + r.motor[3] - r.motor[0] - r.motor[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, roll / pitch);   // направление не искажено
}

void test_thrust_linearization_and_idle()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, Mixer::linearize(0.0f, 0.3f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Mixer::linearize(1.0f, 0.3f));
    TEST_ASSERT_TRUE(Mixer::linearize(0.25f, 0.3f) > 0.25f);    // на малом газе сигнал поднят
    TEST_ASSERT_EQUAL_FLOAT(0.25f, Mixer::linearize(0.25f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.05f, Mixer::withIdle(0.0f, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Mixer::withIdle(1.0f, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Mixer::withIdle(2.0f, 0.05f));
}

// ---------------- фильтры ----------------

static float sineGain(float hz, float fs, float (*filter)(float))
{
    float peak = 0;
    for (int i = 0; i < 4000; ++i)
    {
        const float y = filter(sinf(2.0f * PI_F * hz * i / fs));
        if (i > 3000) peak = fmaxf(peak, fabsf(y));
    }
    return peak;
}

static Pt2 g_pt2;
static Biquad g_notch;
static Biquad g_lowpass;

void test_pt1_reaches_63_percent_in_one_time_constant()
{
    Pt1 f;
    f.configure(10.0f, 0.001f);   // τ = 15.9 мс
    float y = 0;
    for (int i = 0; i < 16; ++i) y = f.apply(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.63f, y);
    f.reset(0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.value());
    Pt1 pass;
    pass.configure(0.0f, 0.001f);
    TEST_ASSERT_EQUAL_FLOAT(0.7f, pass.apply(0.7f));
}

void test_pt2_is_minus_3db_at_cutoff()
{
    g_pt2.configure(50.0f, 0.001f);
    g_pt2.reset();
    const float gain = sineGain(50.0f, 1000.0f, [](float x) { return g_pt2.apply(x); });
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.707f, gain);
}

void test_notch_removes_center_and_keeps_low_frequencies()
{
    g_notch.notch(150.0f, 1000.0f, 4.0f);
    TEST_ASSERT_TRUE(g_notch.isEnabled());
    TEST_ASSERT_TRUE(sineGain(150.0f, 1000.0f, [](float x) { return g_notch.apply(x); }) < 0.05f);
    g_notch.reset();
    TEST_ASSERT_TRUE(sineGain(30.0f, 1000.0f, [](float x) { return g_notch.apply(x); }) > 0.95f);
    // центр едет на ходу без скачков
    g_notch.setCenter(160.0f);
    TEST_ASSERT_TRUE(isfinite(g_notch.apply(1.0f)));
    // выше Найквиста — выключен
    g_notch.setCenter(600.0f);
    TEST_ASSERT_FALSE(g_notch.isEnabled());
    TEST_ASSERT_EQUAL_FLOAT(0.3f, g_notch.apply(0.3f));
}

void test_biquad_lowpass()
{
    g_lowpass.lowpass(100.0f, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.707f, sineGain(100.0f, 1000.0f, [](float x) { return g_lowpass.apply(x); }));
    g_lowpass.lowpass(700.0f, 1000.0f);
    TEST_ASSERT_FALSE(g_lowpass.isEnabled());
}

// ---------------- регуляторы ----------------

void test_analog_esc_pulses()
{
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, pulseUs(EscProtocol::PWM, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, pulseUs(EscProtocol::PWM, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(187.5f, pulseUs(EscProtocol::ONESHOT125, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(25.0f, pulseUs(EscProtocol::MULTISHOT, 5.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 41.667f, pulseUs(EscProtocol::ONESHOT42, -1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pulseUs(EscProtocol::DSHOT600, 0.5f));
    TEST_ASSERT_FALSE(isDshot(EscProtocol::ONESHOT125));
    TEST_ASSERT_TRUE(isDshot(EscProtocol::DSHOT300));
    for (uint8_t p = 0; p <= static_cast<uint8_t>(EscProtocol::DSHOT600); ++p)
    {
        TEST_ASSERT_NOT_EQUAL(0, strcmp("?", escProtocolName(static_cast<EscProtocol>(p))));
    }
}

// Кадр DShot: значение 1046 без телеметрии — 0x82C6 (пример из описания протокола).
void test_dshot_frame_and_crc()
{
    TEST_ASSERT_EQUAL_HEX16(0x82C6, dshot::frame(1046, false, false));
    TEST_ASSERT_EQUAL_HEX16(0x82C9, dshot::frame(1046, false, true));   // двунаправленный: CRC инверсна
    TEST_ASSERT_TRUE(dshot::frameValid(0x82C6, false));
    TEST_ASSERT_FALSE(dshot::frameValid(0x82C7, false));
    TEST_ASSERT_TRUE(dshot::frameValid(dshot::frame(dshot::BEEP1, true, true), true));
    TEST_ASSERT_EQUAL_UINT16(0, dshot::throttleValue(0.7f, false));
    TEST_ASSERT_EQUAL_UINT16(48, dshot::throttleValue(0.0f, true));
    TEST_ASSERT_EQUAL_UINT16(2047, dshot::throttleValue(1.5f, true));
}

void test_dshot_bit_timing()
{
    const dshot::BitTicks t = dshot::bitTicks(EscProtocol::DSHOT600, 240000000);
    TEST_ASSERT_EQUAL_UINT32(400, t.period);
    TEST_ASSERT_EQUAL_UINT32(300, t.one);
    TEST_ASSERT_EQUAL_UINT32(150, t.zero);
    TEST_ASSERT_EQUAL_UINT32(0, dshot::bitTicks(EscProtocol::PWM, 240000000).period);
    uint32_t pulses[17];
    dshot::frameToPulses(0x8001, t, pulses);
    TEST_ASSERT_EQUAL_UINT32(300, pulses[0]);
    TEST_ASSERT_EQUAL_UINT32(150, pulses[1]);
    TEST_ASSERT_EQUAL_UINT32(300, pulses[15]);
    TEST_ASSERT_EQUAL_UINT32(0, pulses[16]);
    TEST_ASSERT_EQUAL_UINT32(150000, dshot::bitRate(EscProtocol::DSHOT150));
}

// ---------------- винтомоторная группа ----------------

// 5" 3-лопастной на 2207 1950KV, 6S: на полном газе ~28 тыс. об/мин,
// ~1.6 кг тяги и ~45 А — как на стендах.
void test_propulsion_matches_bench_data_for_5_inch()
{
    const Propulsion p = Propulsion::from({ 5.1f, 4.3f, 3, 0 }, { 22, 7, 1950, 0 });
    const float w = p.steadyOmega(1.0f, 22.5f);
    const float rpm = w * 60.0f / (2.0f * PI_F);
    TEST_ASSERT_TRUE(rpm > 24000 && rpm < 32000);
    const float grams = p.thrust(w) / GRAVITY * 1000.0f;
    TEST_ASSERT_TRUE(grams > 1300 && grams < 2100);
    const float amps = p.current(1.0f, 22.5f, w);
    TEST_ASSERT_TRUE(amps > 30 && amps < 65);
    TEST_ASSERT_TRUE(p.timeConstant(w) > 0.005f && p.timeConstant(w) < 0.06f);
    TEST_ASSERT_TRUE(p.steadyOmega(0.5f, 22.5f) < w);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.omegaDot(1.0f, 22.5f, w) / 1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.steadyOmega(0.0f, 22.5f));
    TEST_ASSERT_TRUE(Propulsion::bladeFactor(2) < Propulsion::bladeFactor(3));
    TEST_ASSERT_TRUE(Propulsion::bladeFactor(1) < Propulsion::bladeFactor(2));
    TEST_ASSERT_TRUE(Propulsion::bladeFactor(8) > Propulsion::bladeFactor(6));
}

// ---------------- настройка по сборке ----------------

void test_every_catalog_build_gets_a_sane_tune()
{
    uint8_t n = 0;
    const BuildSpec* cat = buildCatalog(n);
    TEST_ASSERT_TRUE(n >= 12);
    for (uint8_t i = 0; i < n; ++i)
    {
        const CopterTune t = deriveTune(cat[i], 1000.0f, EscProtocol::DSHOT600);
        TEST_ASSERT_TRUE_MESSAGE(t.thrustToWeight > 2.0f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.hoverThrust > 0.03f && t.hoverThrust < 0.6f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.phaseMarginDeg >= 45.0f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.gainMarginDb >= GAIN_MARGIN_DB - 0.1f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.crossoverHzRP > 5.0f && t.crossoverHzRP < 60.0f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.roll.p > 0 && t.roll.i > 0 && t.roll.ff > 0 && t.yaw.p > 0, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.gyroLpfHz >= 60.0f && t.gyroLpfHz <= 400.0f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.dtermLpfHz <= t.gyroLpfHz, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.rotHzIdle < t.rotHzHover && t.rotHzHover < t.rotHzFull, cat[i].id);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 1.0f, t.tpaAt(t.hoverThrust), cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE(t.maxAngleDeg >= 30.0f && t.maxAngleDeg <= 60.0f, cat[i].id);
        TEST_ASSERT_TRUE_MESSAGE((t.warnings & (WARN_NO_LIFT_MARGIN | WARN_MOTOR_OVERLOAD | WARN_PROP_OVERLAP)) == 0, cat[i].id);
        TEST_ASSERT_TRUE(findBuild(cat[i].id) == &cat[i]);
    }
    TEST_ASSERT_NULL(findBuild("no-such-build"));
}

// Вупы крутят моторы быстрее половины частоты цикла 1 кГц — об этом предупреждение.
void test_whoop_warns_about_vibration_alias_at_1khz()
{
    const CopterTune t1 = deriveTune(*findBuild("whoop65"), 1000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_TRUE(t1.warnings & WARN_VIBRATION_ALIAS);
    const CopterTune t4 = deriveTune(*findBuild("whoop65"), 4000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_FALSE(t4.warnings & WARN_VIBRATION_ALIAS);
    TEST_ASSERT_TRUE(t4.crossoverHzRP > t1.crossoverHzRP);   // цикл быстрее — контур шире
}

void test_bad_builds_get_warnings()
{
    // 7" винт на моторе 1104 и 4S — мотор не тянет, тяги мало
    BuildSpec weak = *findBuild("freestyle5");
    weak.prop = { 7.0f, 4.0f, 3, 0 };
    weak.motor = { 11, 4, 4000, 0 };
    weak.massG = 900;
    const CopterTune tw = deriveTune(weak, 1000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_TRUE(tw.warnings & (WARN_LOW_THRUST | WARN_NO_LIFT_MARGIN));
    TEST_ASSERT_TRUE(tw.warnings & WARN_MOTOR_OVERLOAD);
    TEST_ASSERT_TRUE(tw.warnings & WARN_HEAVY_HOVER);

    // 7" винты на раме 180 мм задевают друг друга
    BuildSpec tight = *findBuild("longrange7");
    tight.wheelbaseMm = 180;
    TEST_ASSERT_TRUE(deriveTune(tight, 1000.0f, EscProtocol::DSHOT600).warnings & WARN_PROP_OVERLAP);

    // массу можно не знать — оценка по раме, батарее и моторам
    BuildSpec unknown = *findBuild("freestyle5");
    unknown.massG = 0;
    const CopterTune tu = deriveTune(unknown, 1000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_TRUE(tu.massKg > 0.45f && tu.massKg < 0.9f);
}

// PWM-регулятор отвечает медленнее — контур становится уже, но остаётся с запасами.
void test_slower_esc_protocol_narrows_the_loop()
{
    const BuildSpec& b = *findBuild("freestyle5");
    const CopterTune fast = deriveTune(b, 1000.0f, EscProtocol::DSHOT600);
    const CopterTune slow = deriveTune(b, 1000.0f, EscProtocol::PWM);
    TEST_ASSERT_TRUE(slow.crossoverHzRP < fast.crossoverHzRP);
    TEST_ASSERT_TRUE(slow.gainMarginDb >= GAIN_MARGIN_DB - 0.1f);
    TEST_ASSERT_TRUE(escDelaySeconds(EscProtocol::PWM) > escDelaySeconds(EscProtocol::ONESHOT125));
}

void test_rate_curve()
{
    const RateCurve c{ 200, 700, 0.5f };
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.apply(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 700.0f, c.apply(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -700.0f, c.apply(-2.0f));
    TEST_ASSERT_TRUE(c.apply(0.1f) > 19.0f && c.apply(0.1f) < 21.0f);   // у центра — наклон center
    for (uint8_t s = 0; s < 6; ++s)
    {
        TEST_ASSERT_NOT_EQUAL(0, strcmp("?", flyStyleName(static_cast<FlyStyle>(s))));
    }
}

// ---------------- контур угловой скорости ----------------

static RateAxis makeAxis(float p, float i, float d, float ff)
{
    RateAxis a;
    AxisGains g;
    g.p = p;
    g.i = i;
    g.d = d;
    g.ff = ff;
    a.configure(g, 0.0f, 0.0f, 0.3f, 0.001f);
    return a;
}

void test_rate_axis_p_and_i()
{
    RateAxis a = makeAxis(0.01f, 0.1f, 0.0f, 0.0f);
    float out = a.update(10.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.1f, out);
    for (int i = 0; i < 1000; ++i) a.update(10.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_TRUE(a.lastTerms().i > 0.1f);   // ~0.1·10·1 с, после relax-окна
    for (int i = 0; i < 5000; ++i) a.update(10.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, a.lastTerms().i);   // предел интеграла
    a.resetIntegral();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, a.update(0.0f, 0.0f, 1.0f, 1.0f, false, false));
    TEST_ASSERT_EQUAL_FLOAT(0.01f, a.getGains().p);
}

void test_rate_axis_integral_holds_on_ground_and_in_saturation()
{
    RateAxis a = makeAxis(0.01f, 0.1f, 0.0f, 0.0f);
    for (int i = 0; i < 500; ++i) a.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a.lastTerms().i);
    for (int i = 0; i < 500; ++i) a.update(10.0f, 0.0f, 1.0f, 1.0f, false, true);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a.lastTerms().i);   // упор в сторону ошибки — не копит
    // anti-gravity: с усилением интеграл растёт быстрее
    RateAxis b = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    RateAxis c = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    for (int i = 0; i < 50; ++i)
    {
        b.update(10.0f, 0.0f, 1.0f, 1.0f, false, false);
        c.update(10.0f, 0.0f, 1.0f, 3.0f, false, false);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f * b.lastTerms().i, c.lastTerms().i);
}

// I-relax: большая ошибка разгона (флип) не копится — нет «отскока».
// Наш relax по ошибке: коптер догоняет новую уставку (флип) — большая
// ошибка разгона в I не копится; та же ошибка без движения уставки (сбило,
// перекос тяги на рывке газом) — копится как есть.
void test_rate_axis_relaxes_integral_on_large_error()
{
    PidExtras x;
    x.relax = ItermRelax::OFF;   // только relax по ошибке
    const auto flip = [&x](bool relaxError) {
        x.relaxError = relaxError;
        RateAxis a = makeAxis(0.0f, 0.01f, 0.0f, 0.0f);
        a.setExtras(x, 0);
        a.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
        float w = 0.0f;
        for (int i = 0; i < 300; ++i)
        {
            w += (500.0f - w) * 0.025f;   // коптер выходит на 500 °/с с постоянной 40 мс
            a.update(500.0f, w, 1.0f, 1.0f, false, false);
        }
        return a.integral();
    };
    const float relaxed = flip(true), plain = flip(false);
    TEST_ASSERT_TRUE(plain > 0.15f);           // ∫ошибки ≈ 20 (°/с)·с
    TEST_ASSERT_TRUE(relaxed < 0.15f * plain);

    // уставка стоит, коптер сбило на 100 °/с — копится сразу и целиком
    x.relaxError = true;
    RateAxis b = makeAxis(0.0f, 0.01f, 0.0f, 0.0f);
    b.setExtras(x, 0);
    for (int i = 0; i < 100; ++i) b.update(0.0f, -100.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.01f * 100.0f * 0.1f, b.integral());
}

namespace
{
    CopterParams& five5Params();   // ниже, у реестра настроек
}

// 011: D на резком движении растёт до d_max, в покое — базовый.
void test_rate_axis_d_max_rises_on_fast_motion()
{
    PidExtras x;
    x.dMax[0] = 0.002f;   // вдвое больше базового D
    RateAxis a = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    a.setExtras(x, 0);
    RateAxis base = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    for (int i = 0; i < 200; ++i)
    {
        a.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
        base.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, a.dBoost());
    // гироскоп разгоняется 10000 °/с² (пропвош, удар) — D почти вдвое
    float m = 0.0f;
    for (int i = 0; i < 50; ++i)
    {
        m += 10.0f;
        a.update(0.0f, m, 1.0f, 1.0f, false, false);
        base.update(0.0f, m, 1.0f, 1.0f, false, false);
    }
    TEST_ASSERT_TRUE(a.dBoost() > 0.95f);
    TEST_ASSERT_TRUE(fabsf(a.lastTerms().d) > 1.9f * fabsf(base.lastTerms().d));
    // движение успокоилось — D обратно к базовому
    for (int i = 0; i < 300; ++i) a.update(0.0f, m, 1.0f, 1.0f, false, false);
    TEST_ASSERT_TRUE(a.dBoost() < 0.01f);

    // d_max_advance: стик рывком — D поднимается заранее, гироскоп ещё стоит
    RateAxis s = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    s.setExtras(x, 0);
    for (int i = 0; i < 50; ++i) s.update(20.0f * (i + 1), 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_TRUE(s.dBoost() > 0.9f);
    x.dMaxAdvance = 0.0f;
    RateAxis noAdvance = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    noAdvance.setExtras(x, 0);
    for (int i = 0; i < 50; ++i) noAdvance.update(20.0f * (i + 1), 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, noAdvance.dBoost());

    // d_max не больше D — D постоянный
    x.dMax[0] = 0.0005f;
    RateAxis off = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    off.setExtras(x, 0);
    off.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
    off.update(0.0f, 10.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -10.0f, off.lastTerms().d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, off.dBoost());
}

namespace
{
    // Ось с одним I (без нашего relax по ошибке) и заданным iterm_relax.
    RateAxis relaxAxis(ItermRelax relax, ItermRelaxType type, uint8_t axis)
    {
        PidExtras x;
        x.relax = relax;
        x.relaxType = type;
        x.relaxError = false;
        RateAxis a = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
        a.setExtras(x, axis);
        a.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
        return a;
    }

    // Уставка прыгает на value и держится ms миллисекунд, гироскоп стоит.
    float stepIntegral(RateAxis& a, float value, int ms)
    {
        for (int i = 0; i < ms; ++i) a.update(value, 0.0f, 1.0f, 1.0f, false, false);
        return a.integral();
    }
}

// 012: iterm_relax — оси, SETPOINT/GYRO, *_INC.
void test_iterm_relax_axes_types_and_inc()
{
    const ItermRelaxType sp = ItermRelaxType::SETPOINT;
    // OFF: ступенька 100 °/с копится сразу: 0.1·100·0.005 = 0.05
    RateAxis off = relaxAxis(ItermRelax::OFF, sp, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f, stepIntegral(off, 100.0f, 5));
    // RP: крен ждёт, рысканье — нет; RPY — ждут все
    RateAxis roll = relaxAxis(ItermRelax::RP, sp, 0);
    RateAxis yawRp = relaxAxis(ItermRelax::RP, sp, 2);
    RateAxis yawRpy = relaxAxis(ItermRelax::RPY, sp, 2);
    TEST_ASSERT_TRUE(stepIntegral(roll, 100.0f, 5) < 0.002f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f, stepIntegral(yawRp, 100.0f, 5));
    TEST_ASSERT_TRUE(stepIntegral(yawRpy, 100.0f, 5) < 0.002f);
    // уставка встала — relax отпускает, интеграл копится
    TEST_ASSERT_TRUE(stepIntegral(roll, 100.0f, 500) > 0.2f);

    // *_INC: накопленный интеграл уменьшается без задержки, рост — придерживается
    RateAxis inc = relaxAxis(ItermRelax::RP_INC, sp, 0);
    RateAxis plain = relaxAxis(ItermRelax::RP, sp, 0);
    inc.setIntegral(0.2f);
    plain.setIntegral(0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.15f, stepIntegral(inc, -100.0f, 5));
    TEST_ASSERT_TRUE(stepIntegral(plain, -100.0f, 5) > 0.198f);
    RateAxis incUp = relaxAxis(ItermRelax::RP_INC, sp, 0);
    TEST_ASSERT_TRUE(stepIntegral(incUp, 100.0f, 5) < 0.002f);

    // GYRO: ошибка — к сглаженной уставке, в полосе «насколько убежала» — ноль
    RateAxis gyro = relaxAxis(ItermRelax::RP, ItermRelaxType::GYRO, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, stepIntegral(gyro, 100.0f, 5));
    TEST_ASSERT_TRUE(stepIntegral(gyro, 100.0f, 500) > 0.2f);

    // срез выше — ждёт меньше
    PidExtras fast;
    fast.relax = ItermRelax::RP;
    fast.relaxError = false;
    fast.relaxCutoffHz = 40.0f;
    RateAxis quick = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    quick.setExtras(fast, 0);
    quick.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
    RateAxis slow = relaxAxis(ItermRelax::RP, sp, 0);
    TEST_ASSERT_TRUE(stepIntegral(quick, 100.0f, 20) > 2.0f * stepIntegral(slow, 100.0f, 20));
}

// 013: iterm_rotation — пируэт: интеграл крена переходит в тангаж.
void test_iterm_rotation_turns_with_the_frame()
{
    // рысканье +90° (нос вправо): то, что было «вперёд», стало «влево»
    float v[3] = { 0.1f, 0.0f, 0.0f };
    const float yaw[3] = { 0.0f, 0.0f, 90.0f };
    for (int i = 0; i < 1000; ++i) rotateIterm(v, yaw, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, v[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, -0.1f, v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, v[2]);
    // крен +90°: тангаж уходит в рысканье
    float w[3] = { 0.0f, 0.1f, 0.0f };
    const float roll[3] = { 90.0f, 0.0f, 0.0f };
    for (int i = 0; i < 1000; ++i) rotateIterm(w, roll, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, w[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, -0.1f, w[2]);
    // без вращения — как был
    float s[3] = { 0.1f, -0.05f, 0.02f };
    const float still[3] = { 0.0f, 0.0f, 0.0f };
    rotateIterm(s, still, 0.001f);
    TEST_ASSERT_EQUAL_FLOAT(-0.05f, s[1]);

    // в контроллере — только с настройкой: на земле интегралы нулевые, тут — сама функция
    RateAxis a = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    a.setIntegral(1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, a.integral());   // предел соблюдается и при повороте
}

// 014: iterm_windup — моторы разошлись больше порога: I копится слабее, на упоре стоит.
void test_iterm_windup_scales_integral()
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, itermWindupScale(0.5f, 0.85f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, itermWindupScale(0.925f, 0.85f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, itermWindupScale(1.0f, 0.85f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, itermWindupScale(1.0f, 1.0f));   // 100 % — выкл
    RateAxis a = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    for (int i = 0; i < 100; ++i) a.update(10.0f, 0.0f, 1.0f, itermWindupScale(1.0f, 0.85f), false, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a.integral());
}

// 015 и реестр: тонкости PID — настройки профиля; модель — как было, BF 4.5 — свои.
void test_pid_extras_from_params()
{
    CopterParams& p = five5Params();
    char err[96];
    CopterTune t = p.tune();
    // умолчания модели: поведение как до этих настроек
    TEST_ASSERT_EQUAL(static_cast<int>(ItermRelax::RPY), static_cast<int>(t.pid.relax));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, t.pid.relaxCutoffHz);
    TEST_ASSERT_TRUE(t.pid.relaxError);
    TEST_ASSERT_FALSE(t.pid.itermRotation);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, t.pid.itermWindup);
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().iLimit, t.iLimit);
    TEST_ASSERT_EQUAL_FLOAT(roundf(p.modelTune().iLimit * 1000.0f), p.defaultValue(PARAM_ITERM_LIMIT));
    for (uint8_t a = 0; a < 3; ++a) TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.dMax[a]);

    // iterm_limit в единицах BF: 200 — пятая часть хода моторов
    TEST_ASSERT_TRUE(p.parse(PARAM_ITERM_LIMIT, 0, "200", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ITERM_RELAX, 0, "RP_INC", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ITERM_RELAX_TYPE, 0, "GYRO", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ITERM_WINDUP, 0, "85", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ITERM_ROTATION, 0, "ON", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_ITERM_WINDUP, 0, "20", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(0.2f, t.iLimit);
    TEST_ASSERT_EQUAL(static_cast<int>(ItermRelax::RP_INC), static_cast<int>(t.pid.relax));
    TEST_ASSERT_EQUAL(static_cast<int>(ItermRelaxType::GYRO), static_cast<int>(t.pid.relaxType));
    TEST_ASSERT_EQUAL_FLOAT(0.85f, t.pid.itermWindup);
    TEST_ASSERT_TRUE(t.pid.itermRotation);
    // другой PID-профиль — свои значения
    TEST_ASSERT_TRUE(p.selectPidProfile(1));
    TEST_ASSERT_FALSE(p.tune().pid.itermRotation);
    TEST_ASSERT_TRUE(p.selectPidProfile(0));

    // d_max (BF 2025.12): D из таблицы — в покое
    const float d = p.tune().roll.d;
    const float dBf = roundf(gainToBf(PidTerm::D, 0, d));
    char text[16];
    snprintf(text, sizeof(text), "%d", static_cast<int>(dBf) + 10);
    TEST_ASSERT_TRUE(p.parse(PARAM_D_MAX_ROLL, 0, text, err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, d, t.roll.d);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(PidTerm::D, 0, dBf + 10.0f), t.pid.dMax[0]);
    p.reset(PARAM_D_MAX_ROLL, 0);
    // d_min (BF 4.5): в покое — d_min, D из таблицы — на движении
    TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 0, "45", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_D_ROLL, 0, "40", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_D_MIN_ROLL, 0, "30", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(PidTerm::D, 0, 30.0f), t.roll.d);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(PidTerm::D, 0, 40.0f), t.pid.dMax[0]);
    // d_min не меньше D — D постоянный
    TEST_ASSERT_TRUE(p.parse(PARAM_D_MIN_ROLL, 0, "45", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tune().pid.dMax[0]);
    // старые имена BF 4.5 — те же настройки
    TEST_ASSERT_EQUAL(PARAM_D_MAX_GAIN, findParam("d_min_gain"));
    TEST_ASSERT_EQUAL(PARAM_D_MAX_ADVANCE, findParam("D_MIN_ADVANCE"));
    TEST_ASSERT_EQUAL(-1, findParam("d_min_gainx"));
}

void test_rate_axis_d_on_measurement_and_feedforward()
{
    RateAxis a = makeAxis(0.0f, 0.0f, 0.001f, 0.0f);
    a.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
    a.update(0.0f, 10.0f, 1.0f, 1.0f, false, false);   // разгон +10 °/с за 1 мс
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -10.0f, a.lastTerms().d);
    // движение стика не даёт пинка по D, только FF
    RateAxis b = makeAxis(0.0f, 0.0f, 0.001f, 0.0001f);
    b.update(0.0f, 0.0f, 1.0f, 1.0f, false, false);
    for (int i = 0; i < 20; ++i) b.update(100.0f * (i + 1), 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.lastTerms().d);
    TEST_ASSERT_TRUE(b.lastTerms().ff > 0.5f);
}

// ---------------- ориентация ----------------

static void accelFor(float rollDeg, float pitchDeg, float out[3])
{
    const float r = rollDeg * PI_F / 180.0f, p = pitchDeg * PI_F / 180.0f;
    out[0] = sinf(p);
    out[1] = sinf(r) * cosf(p);   // Y влево: правый борт вниз -> «верх» влево
    out[2] = cosf(r) * cosf(p);
}

void test_attitude_starts_from_accelerometer()
{
    AttitudeQuat a;
    float acc[3], gyro[3] = { 0, 0, 0 };
    accelFor(30.0f, -20.0f, acc);
    a.update(gyro, acc, 0.001f);
    TEST_ASSERT_TRUE(a.isInitialized());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, a.rollDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -20.0f, a.pitchDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, a.yawDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, acosf(cosf(30 * PI_F / 180) * cosf(20 * PI_F / 180)) * 180 / PI_F, a.tiltDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, a.verticalAccel(acc));
}

// Флип по крену на 360° гироскопом — ориентация возвращается (кватернион не «теряется»).
void test_attitude_integrates_full_flip_and_yaw()
{
    AttitudeQuat a;
    a.configure(0.0f, 0.0f);   // без акселерометра — только гироскоп
    float acc[3] = { 0, 0, 1 }, gyro[3] = { 720, 0, 0 };
    a.update(gyro, acc, 0.001f);
    for (int i = 0; i < 250; ++i) a.update(gyro, acc, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 180.0f, fabsf(a.rollDeg()));
    TEST_ASSERT_TRUE(a.cosTilt() < -0.99f);
    for (int i = 0; i < 250; ++i) a.update(gyro, acc, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, a.rollDeg());
    float yaw[3] = { 0, 0, 90 };
    for (int i = 0; i < 1000; ++i) a.update(yaw, acc, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, a.yawDeg());
}

void test_attitude_accelerometer_pulls_slowly_and_only_near_1g()
{
    AttitudeQuat a;
    a.configure(2.0f, 0.2f);
    float level[3] = { 0, 0, 1 }, gyro[3] = { 0, 0, 0 }, tilted[3];
    a.update(gyro, level, 0.001f);
    accelFor(20.0f, 0.0f, tilted);
    a.setGrounded(false);
    for (int i = 0; i < 1000; ++i) a.update(gyro, tilted, 0.001f);   // 1 с при τ = 2 с
    TEST_ASSERT_TRUE(a.rollDeg() > 5.0f && a.rollDeg() < 12.0f);
    // 1.5 g (разгон) — не опора
    AttitudeQuat b;
    b.configure(2.0f, 0.2f);
    b.update(gyro, level, 0.001f);
    b.setGrounded(false);
    float hard[3] = { tilted[0] * 1.5f, tilted[1] * 1.5f, tilted[2] * 1.5f };
    for (int i = 0; i < 1000; ++i) b.update(gyro, hard, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, b.rollDeg());
    // на земле — быстро
    AttitudeQuat c;
    c.configure(2.0f, 0.2f);
    c.update(gyro, level, 0.001f);
    c.setGrounded(true);
    for (int i = 0; i < 1000; ++i) c.update(gyro, tilted, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, c.rollDeg());
    c.reset();
    TEST_ASSERT_FALSE(c.isInitialized());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.qw());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.qx() + c.qy() + c.qz());
}

// ---------------- высота ----------------

void test_vertical_estimator_fuses_baro_and_accel()
{
    VerticalEstimator v;
    v.configure(2.5f);
    v.update(0.0f, 10.0f, true, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, v.altitude());
    // подъём 1 м/с: акселерометр дал скорость, барометр ведёт высоту
    float alt = 10.0f;
    for (int i = 0; i < 3000; ++i)
    {
        const float a = i < 100 ? 10.0f : 0.0f;   // разгон до 1 м/с за 0.1 с
        alt += (i < 100 ? 0.0005f * (i + 1) * 0.02f : 0.001f);
        v.update(a, alt, true, 0.001f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 1.0f, v.climbRate());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, alt, v.altitude());
    TEST_ASSERT_TRUE(v.baroUsed());
    // смещение акселерометра выучивается
    VerticalEstimator b;
    b.configure(2.5f);
    for (int i = 0; i < 60000; ++i) b.update(0.3f, 0.0f, true, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.3f, b.accelBias());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, b.climbRate());
}

void test_vertical_estimator_without_baro_learns_bias_on_ground()
{
    VerticalEstimator v;
    v.configure(2.5f);
    v.setBiasLearning(true);
    for (int i = 0; i < 5000; ++i) v.update(0.2f, 0.0f, false, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, v.accelBias());
    TEST_ASSERT_FALSE(v.baroUsed());
    v.setBiasLearning(false);
    v.reset(0.0f);
    for (int i = 0; i < 1000; ++i) v.update(0.2f - 1.0f, 0.0f, false, 0.001f);   // 1 с вниз с 1 м/с²
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -1.0f, v.climbRate());
}

void test_altitude_controller()
{
    AltitudeController c;
    AltitudeController::Settings s;
    c.configure(s, 0.3f);
    c.reset(5.0f, 0.3f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, c.update(5.0f, 0.0f, 0.0f, true, 1.0f, 0.001f));
    TEST_ASSERT_TRUE(c.update(4.0f, 0.0f, 0.0f, true, 1.0f, 0.001f) > 0.3f);   // ниже цели — газ больше
    TEST_ASSERT_TRUE(c.update(5.0f, 0.0f, 0.0f, true, 0.7f, 0.001f) > 0.4f);   // наклон — больше тяги
    // пилот просит набор — цель едет вместе с коптером
    c.update(6.0f, 2.0f, 2.0f, true, 1.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, c.getTarget());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, c.climbTarget());
    c.setTarget(3.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, c.getTarget());
    // новая оценка висения не меняет текущий газ
    const float before = c.learnedHover();
    c.setHover(0.25f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, before, c.learnedHover());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, c.getHover());
    // долго тянет вверх (тяжелее, чем думали) — интеграл учит висение
    AltitudeController h;
    h.configure(s, 0.3f);
    h.reset(0.0f, 0.3f);
    for (int i = 0; i < 20000; ++i) h.update(0.0f, -0.2f, 0.0f, false, 1.0f, 0.001f);
    TEST_ASSERT_TRUE(h.learnedHover() > 0.33f);
}

// ---------------- фильтр гироскопа ----------------

void test_notch_centers_follow_rotation_and_blades()
{
    float c[NOTCH_HARMONICS];
    notchCenters(150.0f, 3, 80.0f, c);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, c[0]);
    TEST_ASSERT_EQUAL_FLOAT(300.0f, c[1]);
    TEST_ASSERT_EQUAL_FLOAT(450.0f, c[2]);
    notchCenters(60.0f, 2, 80.0f, c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c[0]);     // ниже минимума — выключен
    TEST_ASSERT_EQUAL_FLOAT(120.0f, c[1]);
    TEST_ASSERT_EQUAL_FLOAT(180.0f, c[2]);   // у двухлопастных — 3×
    notchCenters(0.0f, 3, 80.0f, c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c[1]);
}

void test_gyro_filter_removes_motor_vibration()
{
    GyroFilter f;
    f.configure(1000.0f, 200.0f, 4.0f, 3, 80.0f);
    f.setRotationHz(150.0f);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, f.notchCenter(0));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.notchCenter(5));
    float peak = 0;
    for (int i = 0; i < 3000; ++i)
    {
        float r[3] = { 50.0f * sinf(2 * PI_F * 150.0f * i / 1000.0f) + 20.0f, 0, 0 };
        f.apply(r);
        if (i > 2000) peak = fmaxf(peak, fabsf(r[0] - 20.0f));
    }
    TEST_ASSERT_TRUE(peak < 2.0f);   // вибрация 50 °/с срезана, вращение 20 °/с осталось
    f.setRotationHz(0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.notchCenter(0));
}

// ---------------- контроллер: ARM и защиты ----------------

struct Bench
{
    CopterController ctl;
    CopterSensors sens;
    CopterRc rc;
    CopterOutputs out;

    explicit Bench(const CopterSettings& s = CopterSettings())
    {
        ctl.configure(deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600), s);
        sens.accel[2] = 1.0f;
        sens.batteryVolts = 25.0f;
        rc.linkOk = true;
        rc.mode = CopterMode::ANGLE;
    }

    void run(float seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 1000.0f); ++i) out = ctl.update(sens, rc);
    }

    bool armAfterBoot()
    {
        run(2.5f);
        rc.armSwitch = true;
        run(0.01f);
        return ctl.isArmed();
    }
};

void test_arm_requires_settling_then_arms_with_switch()
{
    Bench b;
    b.run(0.5f);
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_SETTLING);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    b.run(2.5f);
    // тумблер так и включён — нужен новый щелчок
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_SWITCH);
    b.rc.armSwitch = false;
    b.run(0.01f);
    TEST_ASSERT_EQUAL_HEX32(0, b.ctl.getStatus().armBlockers);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());
    TEST_ASSERT_EQUAL_UINT32(1, b.ctl.getStatus().armCount);
    TEST_ASSERT_TRUE(b.out.spinning);
    for (uint8_t i = 0; i < b.out.count; ++i) TEST_ASSERT_FLOAT_WITHIN(0.01f, b.ctl.getTune().idle, b.out.motor[i]);
    // выключил — моторы стоп сразу
    b.rc.armSwitch = false;
    b.run(0.001f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    TEST_ASSERT_FALSE(b.out.spinning);
}

void test_arm_blockers()
{
    {
        Bench b;
        b.rc.throttle = 0.3f;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_THROTTLE);
    }
    {
        Bench b;
        b.rc.mode = CopterMode::ALT_HOLD;
        b.rc.throttle = 0.3f;   // в ALT_HOLD газ у центра ниже середины — можно
        TEST_ASSERT_TRUE(b.armAfterBoot());
    }
    {
        Bench b;
        b.sens.accel[0] = 0.5f;   // стоит под 30°
        b.sens.accel[2] = 0.866f;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_TILT);
    }
    {
        Bench b;
        b.sens.gyro[1] = 40.0f;   // качают в руках
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_MOVING);
    }
    {
        Bench b;
        b.sens.batteryVolts = 19.0f;   // 6S по 3.17 В
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_BATTERY);
    }
    {
        Bench b;
        b.sens.imuOk = false;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_IMU);
    }
    {
        // снаружи — только свои биты: CLI, приложение, плата
        Bench b;
        b.ctl.setExternalBlockers(ARM_BLOCK_CLI | ARM_BLOCK_MSP | ARM_BLOCK_TILT);
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_CLI);
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_MSP);
        TEST_ASSERT_FALSE(b.ctl.getStatus().armBlockers & ARM_BLOCK_TILT);
        b.ctl.setExternalBlockers(0);
        b.rc.armSwitch = false;
        b.run(0.1f);
        TEST_ASSERT_TRUE(b.armAfterBoot());
    }
    {
        // режим FAILSAFE с тумблера, PREARM, ARM не назначен
        Bench b;
        b.rc.failsafeSwitch = true;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE_SWITCH);
        TEST_ASSERT_FALSE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_LINK);
    }
    {
        Bench b;
        b.rc.prearm = false;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_PREARM);
    }
    {
        Bench b;
        b.rc.armAssigned = false;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_ARM_SWITCH);
    }
    {
        Bench b;
        b.rc.linkOk = false;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_LINK);
    }
    // у каждой причины — слова и код, коды как в status Betaflight
    for (uint32_t bit = 1; bit <= ARM_BLOCK_LAST; bit <<= 1)
    {
        TEST_ASSERT_NOT_EQUAL(0, strcmp("?", armBlockText(bit)));
        TEST_ASSERT_NOT_EQUAL(0, strcmp("?", armBlockCode(bit)));
    }
    TEST_ASSERT_EQUAL_STRING("?", armBlockText(0));
    TEST_ASSERT_EQUAL_STRING("RXLOSS", armBlockCode(ARM_BLOCK_NO_LINK));
    TEST_ASSERT_EQUAL_STRING("ANGLE", armBlockCode(ARM_BLOCK_TILT));
    TEST_ASSERT_EQUAL_STRING("THROTTLE", armBlockCode(ARM_BLOCK_THROTTLE));
    TEST_ASSERT_EQUAL_STRING("CLI", armBlockCode(ARM_BLOCK_CLI));
    TEST_ASSERT_EQUAL_STRING("MSP", armBlockCode(ARM_BLOCK_MSP));
    TEST_ASSERT_EQUAL_STRING("BOXFAILSAFE", armBlockCode(ARM_BLOCK_FAILSAFE_SWITCH));
}

void test_motor_test_only_disarmed_and_times_out()
{
    Bench b;
    b.run(2.5f);
    TEST_ASSERT_FALSE(b.ctl.startMotorTest(9, 0.1f, 1.0f));   // нет такого мотора
    TEST_ASSERT_TRUE(b.ctl.startMotorTest(2, 0.9f, 1.0f));    // доля тяги ограничена 0.3
    b.run(0.1f);
    TEST_ASSERT_TRUE(b.ctl.motorTestActive());
    TEST_ASSERT_TRUE(b.out.spinning);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, b.ctl.getTune().motorCommand(0.3f), b.out.motor[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.out.motor[0]);
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_MOTOR_TEST);
    b.run(1.0f);
    TEST_ASSERT_FALSE(b.ctl.motorTestActive());
    TEST_ASSERT_FALSE(b.out.spinning);
    TEST_ASSERT_TRUE(b.ctl.startMotorTest(0, 0.1f, 3.0f));
    b.ctl.stopMotorTest();
    TEST_ASSERT_FALSE(b.ctl.motorTestActive());
    // ARM прекращает проверку, заармленному — не запускается
    b.rc.armSwitch = false;
    b.run(0.01f);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());
    TEST_ASSERT_FALSE(b.ctl.startMotorTest(0, 0.1f, 1.0f));
}

void test_link_loss_on_ground_disarms_and_needs_switch_toggle()
{
    Bench b;
    TEST_ASSERT_TRUE(b.armAfterBoot());
    b.rc.linkOk = false;
    b.run(0.1f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());   // короткий провал — ждём
    b.run(0.5f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    b.rc.linkOk = true;
    b.run(0.5f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());   // тумблер так и стоит на ARM — сам не армится
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NOT_DISARMED);
    b.rc.armSwitch = false;
    b.run(0.01f);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());
    b.ctl.disarm();
    TEST_ASSERT_FALSE(b.ctl.isArmed());
}

// Биты причин — номера флагов запрета ARM Betaflight 4.5 (конфигуратор
// читает маску по номерам), наши — после них.
void test_arm_block_bits_are_betaflight_flags()
{
    TEST_ASSERT_EQUAL_HEX32(1u << 0, ARM_BLOCK_IMU);
    TEST_ASSERT_EQUAL_HEX32(1u << 1, ARM_BLOCK_FAILSAFE);
    TEST_ASSERT_EQUAL_HEX32(1u << 2, ARM_BLOCK_NO_LINK);
    TEST_ASSERT_EQUAL_HEX32(1u << 3, ARM_BLOCK_NOT_DISARMED);
    TEST_ASSERT_EQUAL_HEX32(1u << 4, ARM_BLOCK_FAILSAFE_SWITCH);
    TEST_ASSERT_EQUAL_HEX32(1u << 5, ARM_BLOCK_RUNAWAY);
    TEST_ASSERT_EQUAL_HEX32(1u << 6, ARM_BLOCK_CRASH);
    TEST_ASSERT_EQUAL_HEX32(1u << 7, ARM_BLOCK_THROTTLE);
    TEST_ASSERT_EQUAL_HEX32(1u << 8, ARM_BLOCK_TILT);
    TEST_ASSERT_EQUAL_HEX32(1u << 11, ARM_BLOCK_LOAD);
    TEST_ASSERT_EQUAL_HEX32(1u << 12, ARM_BLOCK_SETTLING);
    TEST_ASSERT_EQUAL_HEX32(1u << 13, ARM_BLOCK_CLI);
    TEST_ASSERT_EQUAL_HEX32(1u << 16, ARM_BLOCK_MSP);
    TEST_ASSERT_EQUAL_HEX32(1u << 21, ARM_BLOCK_REBOOT);
    TEST_ASSERT_EQUAL_HEX32(1u << 23, ARM_BLOCK_NO_ACC_CAL);
    TEST_ASSERT_EQUAL_HEX32(1u << 25, ARM_BLOCK_SWITCH);
    const char* bf[ARM_BLOCK_BF_COUNT] = { "NOGYRO", "FAILSAFE", "RXLOSS", "NOT_DISARMED", "BOXFAILSAFE", "RUNAWAY",
                                           "CRASH", "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB",
                                           "CLI", "CMS", "BST", "MSP", "PARALYZE", "GPS", "RESC", "RPMFILTER",
                                           "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH" };
    for (uint8_t i = 0; i < ARM_BLOCK_BF_COUNT; ++i) TEST_ASSERT_EQUAL_STRING(bf[i], armBlockCode(1u << i));
    // наши — вне маски BF, коды не повторяются
    TEST_ASSERT_FALSE(ARM_BLOCK_MOVING & ARM_BLOCK_BF_MASK);
    TEST_ASSERT_FALSE(ARM_BLOCK_CONFIG & ARM_BLOCK_BF_MASK);
    for (uint32_t a = 1; a && a <= ARM_BLOCK_LAST; a <<= 1)
        for (uint32_t b = a << 1; b && b <= ARM_BLOCK_LAST; b <<= 1) TEST_ASSERT_NOT_EQUAL(0, strcmp(armBlockCode(a), armBlockCode(b)));
}

// small_angle: стоит под 30° — с 25 не армится, с 35 и 180 — армится.
void test_small_angle_setting()
{
    const float limits[3] = { 25.0f, 35.0f, 180.0f };
    for (int k = 0; k < 3; ++k)
    {
        CopterSettings s;
        s.smallAngleDeg = limits[k];
        Bench b(s);
        b.sens.accel[0] = 0.5f;
        b.sens.accel[2] = 0.866f;
        TEST_ASSERT_EQUAL(k > 0, b.armAfterBoot());
        if (k == 0) TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_TILT);
    }
}

// auto_disarm_delay: заармлен на земле с газом внизу — DISARM через 5 с,
// газ двигали — отсчёт заново; 0 — не разоружается.
void test_auto_disarm_on_the_ground()
{
    {
        Bench b;
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.run(4.0f);
        b.rc.throttle = 0.04f;   // в пределах «газ внизу» — отсчёт идёт
        b.run(0.9f);
        TEST_ASSERT_TRUE(b.ctl.isArmed());
        b.run(0.2f);
        TEST_ASSERT_FALSE(b.ctl.isArmed());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NOT_DISARMED);
        b.rc.armSwitch = false;
        b.run(0.01f);
        TEST_ASSERT_EQUAL_HEX32(0, b.ctl.getStatus().armBlockers);
        b.rc.armSwitch = true;
        b.run(0.01f);
        TEST_ASSERT_TRUE(b.ctl.isArmed());
        b.run(3.0f);
        b.rc.throttle = 0.08f;   // приподнял газ (коптер ещё не оторвался) — отсчёт заново
        b.run(0.05f);
        b.rc.throttle = 0.0f;
        b.run(3.0f);
        TEST_ASSERT_TRUE(b.ctl.isArmed());
    }
    {
        CopterSettings s;
        s.autoDisarmS = 0.0f;
        Bench b(s);
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.run(10.0f);
        TEST_ASSERT_TRUE(b.ctl.isArmed());
    }
}

// После DISARM failsafe'ом ARM ждёт, пока связь продержится
// failsafe_recovery_delay (FAILSAFE), и выключения тумблера (NOT_DISARMED).
void test_arm_after_failsafe_waits_for_recovery_delay()
{
    CopterSettings s;
    s.failsafeRecoveryS = 1.0f;
    Bench b(s);
    TEST_ASSERT_TRUE(b.armAfterBoot());
    b.rc.linkOk = false;
    b.run(0.5f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_LINK);
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_FAILSAFE);
    b.rc.linkOk = true;
    b.rc.armSwitch = false;
    b.run(0.5f);
    TEST_ASSERT_EQUAL_HEX32(ARM_BLOCK_FAILSAFE, b.ctl.getStatus().armBlockers);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_FALSE(b.ctl.isArmed());
    b.rc.armSwitch = false;
    b.run(0.6f);
    TEST_ASSERT_EQUAL_HEX32(0, b.ctl.getStatus().armBlockers);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());
}

// Настройки безопасности и failsafe — в реестре под именами BF, в
// контроллер — в его единицах.
void test_safety_params_reach_the_controller()
{
    CopterParams p;
    TEST_ASSERT_TRUE(p.setValue(PARAM_SMALL_ANGLE, 0, 40.0f));
    TEST_ASSERT_TRUE(p.setValue(PARAM_AUTO_DISARM_DELAY, 0, 0.0f));
    TEST_ASSERT_TRUE(p.setValue(PARAM_FAILSAFE_DELAY, 0, 8.0f));
    TEST_ASSERT_TRUE(p.setValue(PARAM_FAILSAFE_THROTTLE, 0, 1350.0f));
    TEST_ASSERT_TRUE(p.setValue(PARAM_FAILSAFE_RECOVERY_DELAY, 0, 10.0f));
    TEST_ASSERT_TRUE(p.setValue(PARAM_RUNAWAY_TAKEOFF_DEACTIVATE_DELAY, 0, 300.0f));
    char err[64];
    TEST_ASSERT_TRUE(p.parse(PARAM_FAILSAFE_PROCEDURE, 0, "DROP", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_FAILSAFE_SWITCH_MODE, 0, "kill", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_RUNAWAY_TAKEOFF_PREVENTION, 0, "OFF", err, sizeof(err)));
    CopterSettings s;
    p.applyTo(s);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, s.smallAngleDeg);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.autoDisarmS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, s.failsafeDelayS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.35f, s.failsafeThrottle);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, s.failsafeRecoveryS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, s.runawayDeactivateS);
    TEST_ASSERT_FALSE(s.runawayPrevention);
    TEST_ASSERT_EQUAL(static_cast<int>(FailsafeProcedure::DROP), static_cast<int>(s.failsafeProcedure));
    TEST_ASSERT_EQUAL(static_cast<int>(FailsafeSwitchMode::KILL), static_cast<int>(s.failsafeSwitchMode));
    // умолчания — как в BF 4.5, кроме процедуры (у нас AUTO-LAND) и времени посадки
    CopterParams d;
    CopterSettings ds;
    d.applyTo(ds);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, ds.smallAngleDeg);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, ds.autoDisarmS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f, ds.failsafeDelayS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, ds.failsafeRecoveryS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, ds.failsafeThrottleLowS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ds.failsafeThrottle);
    TEST_ASSERT_TRUE(ds.runawayPrevention);
    TEST_ASSERT_EQUAL(static_cast<int>(FailsafeProcedure::AUTO_LAND), static_cast<int>(ds.failsafeProcedure));
    TEST_ASSERT_EQUAL_STRING("safety", paramGroupName(paramInfo(PARAM_FAILSAFE_DELAY).group));
}

void test_mode_names_and_settings()
{
    TEST_ASSERT_EQUAL_STRING("ACRO", copterModeName(CopterMode::ACRO));
    TEST_ASSERT_EQUAL_STRING("HORIZON", copterModeName(CopterMode::HORIZON));
    TEST_ASSERT_EQUAL_STRING("ANGLE", copterModeName(CopterMode::ANGLE));
    TEST_ASSERT_EQUAL_STRING("ALT_HOLD", copterModeName(CopterMode::ALT_HOLD));
    Bench b;
    TEST_ASSERT_TRUE(b.ctl.getSettings().airmode);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.ctl.getGyroFilter().notchCenter(0));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.ctl.axisTerms(7).p);
}

// ---------------- реестр настроек, единицы BF, профили, CLI ----------------

namespace
{
    class RamFlash : public IFlashStorage
    {
    public:
        size_t capacity() const override { return KeyValueStore::CAPACITY; }
        void read(uint8_t* destination, size_t size) override { memcpy(destination, bytes, size); }
        bool write(const uint8_t* source, size_t size) override
        {
            memcpy(bytes, source, size);
            return true;
        }
        uint8_t bytes[KeyValueStore::CAPACITY] = {};
    };

    struct CliCapture : public CliPort
    {
        std::string text;
        int statusCalls = 0;
        void write(const char* s) override { text += s; }
        void printStatus() override { ++statusCalls; }
        std::string take()
        {
            std::string t = text;
            text.clear();
            return t;
        }
    };

    bool has(const std::string& text, const char* what) { return text.find(what) != std::string::npos; }

    CopterParams& five5Params()
    {
        static CopterParams p;
        p.resetAll();
        p.begin(*findBuild("freestyle5"), 1000.0f, EscProtocol::ONESHOT125);
        return p;
    }
}

void test_params_table_is_consistent()
{
    for (int i = 0; i < PARAM_COUNT; ++i)
    {
        const ParamInfo& a = paramInfo(static_cast<ParamId>(i));
        TEST_ASSERT_EQUAL_INT_MESSAGE(i, findParam(a.name), a.name);
        TEST_ASSERT_TRUE_MESSAGE(strlen(a.text) > 0 && a.min <= a.max, a.name);
        for (int j = i + 1; j < PARAM_COUNT; ++j)
        {
            const ParamInfo& b = paramInfo(static_cast<ParamId>(j));
            TEST_ASSERT_TRUE_MESSAGE(strcmp(a.name, b.name) != 0, a.name);
            TEST_ASSERT_TRUE_MESSAGE(paramNameHash(a.name) != paramNameHash(b.name), a.name);   // ключи во флеше
        }
    }
    TEST_ASSERT_EQUAL_INT(PARAM_P_ROLL, findParam("P_Roll"));
    TEST_ASSERT_EQUAL_INT(-1, findParam("p_rol"));
    TEST_ASSERT_EQUAL_INT(-1, findParam("no_such_param"));
    TEST_ASSERT_EQUAL_INT(PARAM_F_YAW, pidParam(2, PidTerm::F));
    TEST_ASSERT_EQUAL_STRING("pid", paramGroupName(paramInfo(PARAM_P_ROLL).group));
    TEST_ASSERT_EQUAL_STRING("rates", paramGroupName(paramInfo(PARAM_ROLL_SRATE).group));

    // умолчания из модели — в границах для всего каталога
    uint8_t n = 0;
    const BuildSpec* cat = buildCatalog(n);
    CopterParams p;
    for (uint8_t b = 0; b < n; ++b)
    {
        p.begin(cat[b], 1000.0f, EscProtocol::DSHOT600);
        for (int i = 0; i < PARAM_COUNT; ++i)
        {
            const ParamInfo& info = paramInfo(static_cast<ParamId>(i));
            if (info.type == ParamType::STRING) continue;
            const float d = p.defaultValue(static_cast<ParamId>(i));
            TEST_ASSERT_TRUE_MESSAGE(d >= info.min && d <= info.max, info.name);
        }
    }
}

void test_params_set_get_bounds_and_enums()
{
    CopterParams& p = five5Params();
    char err[96] = {}, out[48];
    TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 0, " 50 ", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, p.value(PARAM_P_ROLL));
    TEST_ASSERT_TRUE(p.isSet(PARAM_P_ROLL, 0));
    p.format(PARAM_P_ROLL, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("50", out);

    TEST_ASSERT_FALSE(p.parse(PARAM_P_ROLL, 0, "251", err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("допустимо 0 - 250", err);
    TEST_ASSERT_FALSE(p.parse(PARAM_P_ROLL, 0, "5x", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_ROLL_RC_RATE, 0, "0", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.setValue(PARAM_P_ROLL, CopterParams::PROFILES, 40.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, p.value(PARAM_P_ROLL));

    // равное умолчанию — снова умолчание
    TEST_ASSERT_TRUE(p.setValue(PARAM_P_ROLL, 0, p.defaultValue(PARAM_P_ROLL)));
    TEST_ASSERT_FALSE(p.isSet(PARAM_P_ROLL, 0));

    // выбор из списка — по имени, без учёта регистра
    TEST_ASSERT_EQUAL(static_cast<int>(TuneSource::MODEL), static_cast<int>(p.source()));
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "betaflight", err, sizeof(err)));
    TEST_ASSERT_EQUAL(static_cast<int>(TuneSource::BETAFLIGHT), static_cast<int>(p.source()));
    p.format(PARAM_TUNE_SOURCE, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("BETAFLIGHT", out);
    TEST_ASSERT_FALSE(p.parse(PARAM_TUNE_SOURCE, 0, "KISS", err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("допустимо: MODEL, BETAFLIGHT", err);

    // строки: до 8 символов, «-» — пусто
    TEST_ASSERT_TRUE(p.parse(PARAM_PROFILE_NAME, 1, "race", err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("race", p.text(PARAM_PROFILE_NAME, 1));
    TEST_ASSERT_EQUAL_STRING("", p.text(PARAM_PROFILE_NAME, 0));
    TEST_ASSERT_FALSE(p.parse(PARAM_PROFILE_NAME, 1, "123456789", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_PROFILE_NAME, 1, "-", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.isSet(PARAM_PROFILE_NAME, 1));
}

void test_params_survive_flash_and_skip_unknown()
{
    RamFlash flash;
    {
        KeyValueStore kv(flash);
        kv.put("imu", "mount", "x", 1);   // чужие ключи рядом не мешают
        CopterParams& p = five5Params();
        char err[96];
        TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 0, "52", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 2, "61", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.parse(PARAM_PROFILE_NAME, 2, "wind", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.parse(PARAM_YAW_SRATE, 3, "50", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.selectPidProfile(2));
        TEST_ASSERT_TRUE(p.selectRateProfile(3));
        TEST_ASSERT_TRUE(p.save(kv));
        TEST_ASSERT_TRUE(kv.commit());
    }
    {
        KeyValueStore kv(flash);   // «перезагрузка»
        CopterParams p;
        p.begin(*findBuild("freestyle5"), 1000.0f, EscProtocol::ONESHOT125);
        TEST_ASSERT_TRUE(p.load(kv));
        TEST_ASSERT_EQUAL(static_cast<int>(TuneSource::BETAFLIGHT), static_cast<int>(p.source()));
        TEST_ASSERT_EQUAL_UINT8(2, p.pidProfile());
        TEST_ASSERT_EQUAL_UINT8(3, p.rateProfile());
        TEST_ASSERT_EQUAL_FLOAT(52.0f, p.valueIn(PARAM_P_ROLL, 0));
        TEST_ASSERT_EQUAL_FLOAT(61.0f, p.value(PARAM_P_ROLL));
        TEST_ASSERT_EQUAL_FLOAT(45.0f, p.valueIn(PARAM_P_ROLL, 1));   // не задан — дефолт BF
        TEST_ASSERT_EQUAL_STRING("wind", p.text(PARAM_PROFILE_NAME, 2));
        TEST_ASSERT_EQUAL_FLOAT(50.0f, p.value(PARAM_YAW_SRATE));
        TEST_ASSERT_TRUE(kv.contains("imu", "mount"));

        // defaults — ключа во флеше больше нет
        p.resetAll();
        TEST_ASSERT_TRUE(p.save(kv));
        TEST_ASSERT_FALSE(kv.contains(CopterParams::STORE_SPACE, CopterParams::STORE_KEY));
    }
    {
        // настройка, которой больше нет, пропускается; за границей — прижимается
        KeyValueStore kv(flash);
        uint8_t blob[3 + 7 + 7] = { CopterParams::STORE_VERSION, 0, 0 };
        const uint16_t gone = paramNameHash("no_such_param"), pr = paramNameHash("p_roll");
        const float big = 999.0f, any = 7.0f;
        blob[3] = static_cast<uint8_t>(gone);
        blob[4] = static_cast<uint8_t>(gone >> 8);
        blob[5] = 0;
        memcpy(blob + 6, &any, 4);
        blob[10] = static_cast<uint8_t>(pr);
        blob[11] = static_cast<uint8_t>(pr >> 8);
        blob[12] = 0;
        memcpy(blob + 13, &big, 4);
        TEST_ASSERT_TRUE(kv.put(CopterParams::STORE_SPACE, CopterParams::STORE_KEY, blob, sizeof(blob)));
        CopterParams p;
        p.begin(*findBuild("freestyle5"), 1000.0f, EscProtocol::ONESHOT125);
        TEST_ASSERT_TRUE(p.load(kv));
        TEST_ASSERT_EQUAL_FLOAT(250.0f, p.value(PARAM_P_ROLL));
        // другая версия схемы — всё по умолчанию
        blob[0] = CopterParams::STORE_VERSION + 1;
        TEST_ASSERT_TRUE(kv.put(CopterParams::STORE_SPACE, CopterParams::STORE_KEY, blob, sizeof(blob)));
        TEST_ASSERT_FALSE(p.load(kv));
        TEST_ASSERT_FALSE(p.isSet(PARAM_P_ROLL, 0));
    }
}

// 003: модельная настройка 5" в единицах Betaflight и обратно.
void test_pid_in_betaflight_units()
{
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 45.0f * 0.032029f / 1000.0f, bfToGain(PidTerm::P, 0, 45.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 80.0f * 0.244381f * 2.5f / 1000.0f, bfToGain(PidTerm::I, 2, 80.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 30.0f * 0.000529f / 1000.0f, bfToGain(PidTerm::D, 1, 30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 120.0f * 0.013754f * 0.01f / 1000.0f, bfToGain(PidTerm::F, 0, 120.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 37.0f, gainToBf(PidTerm::D, 0, bfToGain(PidTerm::D, 0, 37.0f)));

    CopterParams& p = five5Params();
    const CopterTune& m = p.modelTune();
    char msg[160];
    snprintf(msg, sizeof(msg), "freestyle5 в BF: P %.0f I %.0f D %.0f F %.0f; рысканье P %.0f I %.0f", p.defaultValue(PARAM_P_ROLL),
             p.defaultValue(PARAM_I_ROLL), p.defaultValue(PARAM_D_ROLL), p.defaultValue(PARAM_F_ROLL),
             p.defaultValue(PARAM_P_YAW), p.defaultValue(PARAM_I_YAW));
    TEST_MESSAGE(msg);
    // модель и BF — числа одного порядка (BF 4.5: P 45, I 80, D 40, F 120)
    TEST_ASSERT_TRUE_MESSAGE(p.defaultValue(PARAM_P_ROLL) >= 15.0f && p.defaultValue(PARAM_P_ROLL) <= 90.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(p.defaultValue(PARAM_I_ROLL) >= 10.0f && p.defaultValue(PARAM_I_ROLL) <= 160.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(p.defaultValue(PARAM_D_ROLL) >= 5.0f && p.defaultValue(PARAM_D_ROLL) <= 80.0f, msg);
    TEST_ASSERT_TRUE_MESSAGE(p.defaultValue(PARAM_F_ROLL) >= 5.0f && p.defaultValue(PARAM_F_ROLL) <= 250.0f, msg);
    TEST_ASSERT_EQUAL_FLOAT(roundf(gainToBf(PidTerm::P, 0, m.roll.p)), p.defaultValue(PARAM_P_ROLL));

    // не задано — модель как есть (без округления до целых BF)
    CopterTune t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(m.roll.p, t.roll.p);
    TEST_ASSERT_TRUE(t.gainsFromModel);
    // своё число — только этот член
    char err[96];
    TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 0, "50", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 50.0f * 0.032029f / 1000.0f, t.roll.p);
    TEST_ASSERT_EQUAL_FLOAT(m.roll.i, t.roll.i);
    TEST_ASSERT_EQUAL_FLOAT(m.pitch.p, t.pitch.p);
    TEST_ASSERT_FALSE(t.gainsFromModel);
    // расходы по стилю «фристайл»: 180..720 °/с, экспо 0.5 — в единицах BF Actual
    TEST_ASSERT_EQUAL_FLOAT(18.0f, p.value(PARAM_ROLL_RC_RATE));
    TEST_ASSERT_EQUAL_FLOAT(72.0f, p.value(PARAM_ROLL_SRATE));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, p.value(PARAM_ROLL_EXPO));
    TEST_ASSERT_EQUAL_FLOAT(m.rates[0].apply(0.7f), t.rates[0].apply(0.7f));
    TEST_ASSERT_TRUE(p.parse(PARAM_ROLL_SRATE, 0, "100", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, p.tune().rates[0].apply(1.0f));
}

// 004: tune_source = BETAFLIGHT — дефолты BF 4.5 целиком.
void test_betaflight_source_takes_bf_45_defaults()
{
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    const uint16_t expect[3][4] = { { 45, 80, 40, 120 }, { 47, 84, 46, 125 }, { 45, 80, 0, 120 } };
    const uint16_t dMin[3] = { 30, 34, 0 };
    const CopterTune t = p.tune();
    const AxisGains* g[3] = { &t.roll, &t.pitch, &t.yaw };
    for (uint8_t a = 0; a < 3; ++a)
    {
        for (uint8_t k = 0; k < 4; ++k)
        {
            const PidTerm term = static_cast<PidTerm>(k);
            TEST_ASSERT_EQUAL_FLOAT(expect[a][k], p.defaultValue(pidParam(a, term)));
            // D из таблицы BF 4.5 — на резком движении, в покое — d_min
            const float want = term == PidTerm::D && dMin[a] ? dMin[a] : expect[a][k];
            TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(term, a, want), gainTerm(*g[a], term));
        }
        TEST_ASSERT_EQUAL_FLOAT(dMin[a], p.defaultValue(static_cast<ParamId>(PARAM_D_MIN_ROLL + a)));
        TEST_ASSERT_FLOAT_WITHIN(1e-9f, dMin[a] ? bfToGain(PidTerm::D, a, expect[a][2]) : 0.0f, t.pid.dMax[a]);
    }
    // I: relax RP 15 Гц SETPOINT, windup 85 %, limit 400
    TEST_ASSERT_EQUAL(static_cast<int>(ItermRelax::RP), static_cast<int>(t.pid.relax));
    TEST_ASSERT_EQUAL(static_cast<int>(ItermRelaxType::SETPOINT), static_cast<int>(t.pid.relaxType));
    TEST_ASSERT_EQUAL_FLOAT(15.0f, t.pid.relaxCutoffHz);
    TEST_ASSERT_EQUAL_FLOAT(0.85f, t.pid.itermWindup);
    TEST_ASSERT_EQUAL_FLOAT(400.0f, p.defaultValue(PARAM_ITERM_LIMIT));
    TEST_ASSERT_FALSE(t.gainsFromModel);
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::D), static_cast<int>(t.pid.tpaMode));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, t.tpaAt(0.3f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.35f, t.tpaAt(1.0f));
    TEST_ASSERT_EQUAL_FLOAT(250.0f, t.gyroLpfHz);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, t.yawLpfHz);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, t.iLimit);
    // расходы Actual 70/670/0
    TEST_ASSERT_EQUAL_FLOAT(70.0f, t.rates[0].centerDps);
    TEST_ASSERT_EQUAL_FLOAT(670.0f, t.rates[2].maxDps);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.rates[1].expo);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 70.0f * 0.5f + 600.0f * 0.25f, t.rates[0].apply(0.5f));
    // модель осталась на месте: физика та же, что без BF
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().hoverThrust, t.hoverThrust);
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().minNotchHz, t.minNotchHz);

    // TPA только на D: P на полном газе не слабеет
    RateAxis axis;
    axis.configure(t.roll, 0.0f, 0.0f, 0.3f, 0.001f);
    axis.setTpaMode(TpaMode::D);
    axis.update(0.0f, 0.0f, 0.35f, 1.0f, true, false);
    axis.update(100.0f, 0.0f, 0.35f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, t.roll.p * 100.0f, axis.lastTerms().p);

    // обратно на модель
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "MODEL", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().roll.d, p.tune().roll.d);
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::MODEL), static_cast<int>(p.tune().pid.tpaMode));
}

// 005: профили независимы, переключаются на ходу — и в полёте.
void test_profiles_are_independent_and_switch_on_the_fly()
{
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_TRUE(p.parse(PARAM_P_ROLL, 1, "70", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ROLL_SRATE, 2, "90", err, sizeof(err)));
    const CopterTune t0 = p.tune();
    TEST_ASSERT_FALSE(p.isSet(PARAM_P_ROLL, 0));
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().roll.p, t0.roll.p);
    TEST_ASSERT_FALSE(p.selectPidProfile(4));

    Bench b;
    b.ctl.configure(t0);
    TEST_ASSERT_TRUE(b.armAfterBoot());
    b.rc.throttle = 0.5f;
    b.run(0.5f);
    const uint32_t loops = b.ctl.getStatus().loops;

    TEST_ASSERT_TRUE(p.selectPidProfile(1));
    TEST_ASSERT_TRUE(p.selectRateProfile(2));
    b.ctl.applyProfile(p.tune());
    TEST_ASSERT_TRUE(b.ctl.isArmed());                       // ARM, ориентация, счёт циклов — те же
    TEST_ASSERT_EQUAL_UINT32(loops, b.ctl.getStatus().loops);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(PidTerm::P, 0, 70.0f), b.ctl.getTune().roll.p);
    TEST_ASSERT_EQUAL_FLOAT(900.0f, b.ctl.getTune().rates[0].maxDps);
    TEST_ASSERT_EQUAL_FLOAT(t0.rates[1].maxDps, b.ctl.getTune().rates[1].maxDps);
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());

    // профиль 0 не тронут
    TEST_ASSERT_TRUE(p.selectPidProfile(0));
    TEST_ASSERT_EQUAL_FLOAT(t0.roll.p, p.tune().roll.p);
}

// ---------------- газ: пределы суммы, anti-gravity, throttle boost, TPA (016–020) ----------------

namespace
{
    // 5" из модели с заданными тонкостями PID-профиля.
    CopterTune fiveWith(const PidExtras& x)
    {
        CopterTune t = deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600);
        t.pid = x;
        return t;
    }

    // ACRO, ARM, газ throttle 1 с — в воздухе.
    void armInAcro(Bench& b, float throttle)
    {
        b.rc.mode = CopterMode::ACRO;
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.rc.throttle = throttle;
        b.run(1.0f);
        TEST_ASSERT_TRUE(b.ctl.getStatus().airborne);
    }

    float motorAverage(const CopterOutputs& out)
    {
        float s = 0.0f;
        for (uint8_t i = 0; i < out.count; ++i) s += out.motor[i];
        return out.count ? s / out.count : 0.0f;
    }

    // Без TPA (кривая BF с точкой на полном газе): P — только ошибка и множители.
    PidExtras flatTpa()
    {
        PidExtras x;
        x.tpaMode = TpaMode::PD;
        x.tpaBreakpoint = 1.0f;
        return x;
    }
}

// 016: предел суммы PID — свой у крена и тангажа и у рысканья; в пределе I не копится.
void test_pidsum_limit_per_axis()
{
    PidExtras x;
    x.pidSumLimit = 0.5f;
    x.pidSumLimitYaw = 0.4f;
    RateAxis roll = makeAxis(0.01f, 0.0f, 0.0f, 0.0f);
    RateAxis yaw = makeAxis(0.01f, 0.0f, 0.0f, 0.0f);
    roll.setExtras(x, 0);
    yaw.setExtras(x, 2);
    // ошибка 100 °/с × P 0.01 = 1 — каждая ось упирается в свой предел
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, roll.update(100.0f, 0.0f, 1.0f, 1.0f, true, false));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.4f, yaw.update(-100.0f, 0.0f, 1.0f, 1.0f, true, false));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, roll.update(20.0f, 0.0f, 1.0f, 1.0f, true, false));
    RateAxis plain = makeAxis(0.01f, 0.0f, 0.0f, 0.0f);   // без настроек — ±1, как было
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, plain.update(300.0f, 0.0f, 1.0f, 1.0f, true, false));

    // P 0.3 + I дорастает до 0.2 — сумма в пределе, дальше I стоит; назад — сразу
    PidExtras y;
    y.relax = ItermRelax::OFF;
    y.relaxError = false;
    y.pidSumLimit = 0.5f;
    RateAxis pi = makeAxis(0.01f, 0.1f, 0.0f, 0.0f);
    pi.setExtras(y, 0);
    for (int i = 0; i < 300; ++i) pi.update(30.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.2f, pi.integral());
    const float held = pi.integral();
    pi.update(-30.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_TRUE(pi.integral() < held - 0.002f);

    // реестр: модель — ±1000 (как было), BF — 500 и 400
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, p.defaultValue(PARAM_PIDSUM_LIMIT));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.tune().pid.pidSumLimitYaw);
    TEST_ASSERT_TRUE(p.parse(PARAM_PIDSUM_LIMIT_YAW, 0, "300", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_PIDSUM_LIMIT, 0, "50", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.3f, p.tune().pid.pidSumLimitYaw);
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(500.0f, p.defaultValue(PARAM_PIDSUM_LIMIT));
    TEST_ASSERT_EQUAL_FLOAT(400.0f, p.defaultValue(PARAM_PIDSUM_LIMIT_YAW));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, p.tune().pid.pidSumLimit);
}

// 017: anti-gravity — рывок газом: I сильнее до 1 + gain/10 раз, P крена и
// тангажа — до +50 % при p_gain 100; газ ровный — всё как было.
void test_anti_gravity_boosts_i_and_p_on_throttle_punch()
{
    struct Punch
    {
        float p0 = 0, pMax = 0, pAfter = 0, iSteady = 0, iPunch = 0;
    };
    const auto punch = [](const PidExtras& x) {
        Punch r;
        Bench b;
        b.ctl.configure(fiveWith(x));
        armInAcro(b, 0.25f);
        b.sens.gyro[0] = -10.0f;   // ошибка крена +10 °/с
        b.run(0.05f);
        r.p0 = b.ctl.axisTerms(0).p;
        float i0 = b.ctl.axisTerms(0).i;
        b.run(0.01f);
        r.iSteady = b.ctl.axisTerms(0).i - i0;
        i0 = b.ctl.axisTerms(0).i;
        b.rc.throttle = 0.75f;
        for (int i = 0; i < 10; ++i)
        {
            b.run(0.001f);
            r.pMax = fmaxf(r.pMax, b.ctl.axisTerms(0).p);
        }
        r.iPunch = b.ctl.axisTerms(0).i - i0;
        b.run(1.5f);
        r.pAfter = b.ctl.axisTerms(0).p;
        return r;
    };

    PidExtras bf = flatTpa();   // дефолты BF 4.3+: 80, 5 Гц, 100 (у модели — то же без P)
    bf.antiGravityP = 1.0f;
    const Punch a = punch(bf);
    TEST_ASSERT_TRUE(a.p0 > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * a.p0, 1.5f * a.p0, a.pMax);
    TEST_ASSERT_TRUE(a.iPunch > 5.0f * a.iSteady);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * a.p0, a.p0, a.pAfter);

    PidExtras off = flatTpa();
    off.antiGravityI = 0.0f;
    const Punch o = punch(off);
    // P не растёт (0.5 % — переходный процесс режекторов, газ сдвинул обороты)
    TEST_ASSERT_FLOAT_WITHIN(0.005f * o.p0, o.p0, o.pMax);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * o.iSteady, o.iSteady, o.iPunch);

    // прежняя (I до ×4, ФНЧ 15 Гц) — рывок за 10 мс резче полного: сразу ×4, P не трогается
    PidExtras old = flatTpa();
    old.antiGravityI = 3.0f;
    old.antiGravityHz = 15.0f;
    const Punch m = punch(old);
    TEST_ASSERT_FLOAT_WITHIN(0.005f * m.p0, m.p0, m.pMax);
    char msg[160];
    snprintf(msg, sizeof(msg), "anti-gravity I in 10 ms punch / steady: BF %.2f, off %.2f, old %.2f", a.iPunch / a.iSteady,
             o.iPunch / o.iSteady, m.iPunch / m.iSteady);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(m.iPunch > 2.0f * m.iSteady && m.iPunch < 4.05f * m.iSteady, msg);

    // реестр: модель 80 / 5 Гц / 0, BF 80 / 5 Гц / 100
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(80.0f, p.defaultValue(PARAM_ANTI_GRAVITY_GAIN));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.defaultValue(PARAM_ANTI_GRAVITY_P_GAIN));
    CopterTune t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(8.0f, t.pid.antiGravityI);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, t.pid.antiGravityHz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.antiGravityP);
    TEST_ASSERT_TRUE(p.parse(PARAM_ANTI_GRAVITY_P_GAIN, 0, "50", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_ANTI_GRAVITY_CUTOFF_HZ, 0, "1", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, p.tune().pid.antiGravityP);
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(8.0f, t.pid.antiGravityI);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, t.pid.antiGravityHz);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, t.pid.antiGravityP);   // задано руками — и в BF
    TEST_ASSERT_EQUAL_FLOAT(100.0f, p.defaultValue(PARAM_ANTI_GRAVITY_P_GAIN));
}

// 018: throttle_boost — к газу со стика добавляется его быстрая часть:
// моторы догоняют рывок раньше, на ровном газе — как без добавки.
void test_throttle_boost_adds_the_fast_part_of_the_stick()
{
    PidExtras x;
    x.throttleBoost = 1.0f;   // throttle_boost 10
    Bench plain, boosted;
    boosted.ctl.configure(fiveWith(x));
    armInAcro(plain, 0.3f);
    armInAcro(boosted, 0.3f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, motorAverage(plain.out), motorAverage(boosted.out));
    plain.rc.throttle = boosted.rc.throttle = 0.5f;
    plain.run(0.01f);
    boosted.run(0.01f);
    TEST_ASSERT_TRUE(motorAverage(boosted.out) > motorAverage(plain.out) + 0.03f);
    TEST_ASSERT_EQUAL_FLOAT(plain.ctl.getStatus().throttle, boosted.ctl.getStatus().throttle);   // сам газ — без добавки
    plain.run(0.5f);
    boosted.run(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, motorAverage(plain.out), motorAverage(boosted.out));
    // вниз — так же быстрее
    plain.rc.throttle = boosted.rc.throttle = 0.3f;
    plain.run(0.01f);
    boosted.run(0.01f);
    TEST_ASSERT_TRUE(motorAverage(boosted.out) < motorAverage(plain.out) - 0.03f);

    // реестр: модель 0, BF 5 (×0.1)
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tune().pid.throttleBoost);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, p.tune().pid.throttleBoostHz);
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, p.tune().pid.throttleBoost);
    TEST_ASSERT_TRUE(p.parse(PARAM_THROTTLE_BOOST_CUTOFF, 0, "25", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(25.0f, p.tune().pid.throttleBoostHz);
}

// 019: TPA как в BF — PD или только D выше точки излома; MODEL — таблица модели.
void test_tpa_modes_like_betaflight()
{
    CopterTune t = deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::MODEL), static_cast<int>(t.pid.tpaMode));
    TEST_ASSERT_EQUAL_FLOAT(t.tpa[5], t.tpaAt(0.5f));
    t.pid.tpaMode = TpaMode::D;
    t.pid.tpaRate = 0.5f;
    t.pid.tpaBreakpoint = 0.5f;
    TEST_ASSERT_EQUAL_FLOAT(1.0f, t.tpaAt(0.4f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f, t.tpaAt(0.75f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, t.tpaAt(1.0f));

    // P слабеет в PD и MODEL, в D — нет; D — во всех
    const TpaMode modes[] = { TpaMode::PD, TpaMode::D, TpaMode::MODEL };
    for (TpaMode mode : modes)
    {
        PidExtras x;
        x.tpaMode = mode;
        RateAxis a = makeAxis(0.001f, 0.0f, 0.001f, 0.0f);
        a.setExtras(x, 0);
        a.update(0.0f, 0.0f, 0.5f, 1.0f, true, false);
        a.update(100.0f, 10.0f, 0.5f, 1.0f, true, false);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (mode == TpaMode::D ? 1.0f : 0.5f) * 0.09f, a.lastTerms().p);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f * 10.0f, a.lastTerms().d);
    }

    // реестр: модель — MODEL, BF — D 65 % от 1350; точка в мкс газа
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(TpaMode::MODEL), p.defaultValue(PARAM_TPA_MODE));
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_MODE, 0, "PD", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_RATE, 0, "50", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_BREAKPOINT, 0, "1500", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_TPA_RATE, 0, "101", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::PD), static_cast<int>(t.pid.tpaMode));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, t.pid.tpaBreakpoint);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, t.tpaAt(1.0f));
    p.reset(PARAM_TPA_MODE, 0);
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(TpaMode::D), p.defaultValue(PARAM_TPA_MODE));
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_MODE, 0, "MODEL", err, sizeof(err)));   // модельная таблица и на BF
    t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().tpa[10], t.tpaAt(1.0f));
}

// 020: tpa_low — на малом газе PD (D) слабее на tpa_low_rate; по умолчанию
// только до первого подъёма газа после ARM, с tpa_low_always — всегда.
void test_tpa_low_until_takeoff_or_always()
{
    CopterTune t = deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, t.tpaLowAt(0.0f));   // модель: выкл
    t.pid.tpaLowRate = 0.2f;
    t.pid.tpaLowBreakpoint = 0.05f;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.tpaLowAt(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.9f, t.tpaLowAt(0.025f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, t.tpaLowAt(0.05f));
    t.pid.tpaLowRate = -0.5f;   // меньше 0 — сильнее
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, t.tpaLowAt(0.0f));

    // в контроллере: P на нуле газа (ошибка постоянная) против P без поправки
    const auto pAtZero = [](bool always, bool rearm) {
        PidExtras x = flatTpa();
        x.tpaLowRate = 0.2f;
        x.tpaLowBreakpoint = 0.05f;
        x.tpaLowAlways = always;
        Bench b;
        b.ctl.configure(fiveWith(x));
        b.rc.mode = CopterMode::ACRO;
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.rc.throttle = 0.3f;   // взлёт
        b.run(0.2f);
        b.rc.throttle = 0.0f;
        b.run(0.3f);
        if (rearm)
        {
            b.rc.armSwitch = false;
            b.run(0.01f);
            b.rc.armSwitch = true;
            b.run(0.01f);
            TEST_ASSERT_TRUE(b.ctl.isArmed());
        }
        // ошибка маленькая: airmode поднимает газ смесителя на размах моторов
        b.sens.gyro[0] = -2.0f;
        b.run(0.05f);   // режекторы гироскопа отзвенели
        return b.ctl.axisTerms(0).p / (b.ctl.getTune().roll.p * 2.0f);
    };
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, pAtZero(false, false));   // взлетели — выкл
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.8f, pAtZero(false, true));    // заново ARM, газ не поднимали
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.8f, pAtZero(true, false));    // always — и после взлёта

    // реестр: модель — 0, BF — 20 % ниже 1050, OFF
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tune().pid.tpaLowRate);
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_LOW_RATE, 0, "-30", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_TPA_LOW_ALWAYS, 0, "ON", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.3f, t.pid.tpaLowRate);
    TEST_ASSERT_TRUE(t.pid.tpaLowAlways);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, t.pid.tpaLowBreakpoint);
    p.reset(PARAM_TPA_LOW_RATE, 0);
    TEST_ASSERT_TRUE(p.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, p.tune().pid.tpaLowRate);
}

// ---------------- feedforward (021–025) ----------------

namespace
{
    // Одна ось, только FF: 1e-4 на °/с², I стоит (земля). Возвращает FF в °/с².
    RateAxis ffAxis(const PidExtras& x)
    {
        RateAxis a = makeAxis(0.0f, 0.0f, 0.0f, 1e-4f);
        a.setExtras(x, 0);
        return a;
    }

    float ffStep(RateAxis& a, float stickSp, float deflection, float raw, bool frame, float period)
    {
        FeedforwardInput in;
        in.stickSetpoint = stickSp;
        in.deflection = deflection;
        in.rawStick = raw;
        in.newFrame = frame;
        in.framePeriodS = period;
        a.setFeedforwardInput(in);
        a.update(stickSp, 0.0f, 1.0f, 1.0f, true, false);
        return a.lastTerms().ff / 1e-4f;
    }

    // «Лесенка» кадров 50 Гц: стик меняется только с кадром, за кадр +step
    // °/с (у alternate — то 0.5, то 1.5 шага: кадры приходят неровно).
    // Размах FF относительно среднего и среднее за последние 0.1 с из 0.4 с.
    void ffStairs(const PidExtras& x, bool alternate, float& ripple, float& mean)
    {
        RateAxis a = ffAxis(x);
        float lo = 1e9f, hi = -1e9f, sum = 0.0f, sp = 0.0f;
        int n = 0;
        for (int i = 0; i < 400; ++i)
        {
            const bool frame = i % 20 == 0;
            if (frame) sp += alternate ? ((i / 20) % 2 ? 15.0f : 5.0f) : 10.0f;
            const float f = ffStep(a, sp, 0.5f, sp / 670.0f, frame, 0.02f);
            if (i >= 300)
            {
                lo = fminf(lo, f);
                hi = fmaxf(hi, f);
                sum += f;
                ++n;
            }
        }
        mean = sum / n;
        ripple = (hi - lo) / mean;
    }
}

// 021: feedforward_transition — у центра стика FF слабее, линейно к нулю;
// дальше точки — как без неё.
void test_feedforward_transition_softens_center()
{
    PidExtras on;
    on.ffTransition = 0.5f;
    PidExtras off;
    RateAxis a = ffAxis(on), b = ffAxis(off);
    float at01 = 0, at01off = 0, at07 = 0, at07off = 0;
    for (int i = 0; i <= 200; ++i)
    {
        const float s = i / 200.0f;   // стик 0 -> 1 за 0.2 с
        const float fa = ffStep(a, 670.0f * s, s, s, true, 0.0f);
        const float fb = ffStep(b, 670.0f * s, s, s, true, 0.0f);
        if (i == 20) { at01 = fa; at01off = fb; }
        if (i == 140) { at07 = fa; at07off = fb; }
    }
    // на 0.1 — около 0.1/0.5 = 0.2 (ФНЧ помнит ещё меньшее отклонение)
    TEST_ASSERT_TRUE(at01off > 1000.0f);
    TEST_ASSERT_TRUE(at01 / at01off > 0.1f && at01 / at01off < 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, at07 / at07off);
}

// 022: feedforward_averaging — скорость стика средняя за N кадров: «лесенка»
// кадров 50 Гц без пульсаций, и неровные кадры — тоже; среднее то же.
void test_feedforward_averaging_smooths_frame_steps()
{
    float rOff, mOff, r2, m2, r4, m4, rAlt2, mAlt2, rAltOff, mAltOff;
    PidExtras off;
    ffStairs(off, false, rOff, mOff);
    PidExtras two;
    two.ffAveraging = 2;
    ffStairs(two, false, r2, m2);
    PidExtras four;
    four.ffAveraging = 4;
    ffStairs(four, false, r4, m4);
    ffStairs(two, true, rAlt2, mAlt2);
    ffStairs(off, true, rAltOff, mAltOff);
    char line[200];
    snprintf(line, sizeof(line), "ripple off %.2f, 2pt %.3f, 4pt %.3f, uneven off %.2f, uneven 2pt %.3f; mean %.0f %.0f %.0f",
             rOff, r2, r4, rAltOff, rAlt2, mOff, m2, m4);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(rOff > 2.0f && rAltOff > 2.0f, line);
    TEST_ASSERT_TRUE_MESSAGE(r2 < 0.05f && r4 < 0.05f && rAlt2 < 0.05f, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0f, 500.0f, mOff, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0f, 500.0f, m2, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0f, 500.0f, m4, line);
}

// 023: feedforward_smooth_factor — сглаживание между кадрами: пульсации
// «лесенки» меньше, среднее то же.
void test_feedforward_smooth_factor()
{
    float rOff, mOff, rS, mS, rStrong, mStrong;
    PidExtras off;
    ffStairs(off, false, rOff, mOff);
    PidExtras s;
    s.ffSmoothFactor = 0.25f;   // BF
    ffStairs(s, false, rS, mS);
    PidExtras strong;
    strong.ffSmoothFactor = 0.65f;
    ffStairs(strong, false, rStrong, mStrong);
    char line[160];
    snprintf(line, sizeof(line), "ripple off %.2f, 25 %.2f, 65 %.2f; mean %.0f %.0f %.0f", rOff, rS, rStrong, mOff, mS, mStrong);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(rS < rOff * 0.9f && rStrong < rS * 0.6f, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0f, 500.0f, mS, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0f, 500.0f, mStrong, line);
}

// 024: feedforward_jitter_factor — медленное дрожание стика ±2 % (кадры
// 250 Гц) почти не даёт FF; резкий флик — как без него.
void test_feedforward_jitter_factor()
{
    const auto run = [](float jitter, bool flick) {
        PidExtras x;
        x.ffJitterFactor = jitter;
        RateAxis a = ffAxis(x);
        float raw = 0.0f, sum2 = 0.0f, peak = 0.0f;
        for (int i = 0; i < 1000; ++i)
        {
            const bool frame = i % 4 == 0;
            if (frame) raw = flick ? fminf(1.0f, i / 40.0f) : 0.02f * sinf(2.0f * PI_F * 2.0f * i / 1000.0f);
            const float f = ffStep(a, 670.0f * raw, fabsf(raw), raw, frame, 0.004f);
            sum2 += f * f;
            peak = fmaxf(peak, f);
        }
        return flick ? peak : sqrtf(sum2 / 1000.0f);
    };
    const float tremble = run(7.0f, false), trembleOff = run(0.0f, false);
    const float flick = run(7.0f, true), flickOff = run(0.0f, true);
    char line[160];
    snprintf(line, sizeof(line), "tremble rms %.1f vs %.1f; flick peak %.0f vs %.0f", tremble, trembleOff, flick, flickOff);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(trembleOff > 50.0f && tremble < 0.25f * trembleOff, line);
    TEST_ASSERT_TRUE_MESSAGE(flick > 0.97f * flickOff, line);
}

// 025: feedforward_boost — добавка по ускорению стика: пока стик разгоняется,
// FF больше; на ровном движении — как без неё. Не больше самой скорости.
void test_feedforward_boost()
{
    PidExtras x;
    x.ffBoost = 15.0f;
    PidExtras off;
    RateAxis a = ffAxis(x), b = ffAxis(off);
    float sp = 0.0f, speed = 0.0f, accelRatio = 0, cruiseRatio = 0;
    for (int i = 0; i < 150; ++i)
    {
        if (i < 30) speed += 20.0f;   // разгон 20000 °/с³ 30 мс, дальше ровно
        sp += speed * 0.001f;
        const float fa = ffStep(a, sp, 0.5f, sp / 670.0f, true, 0.0f);
        const float fb = ffStep(b, sp, 0.5f, sp / 670.0f, true, 0.0f);
        if (i == 29) accelRatio = fa / fb;
        if (i == 149) cruiseRatio = fa / fb;
    }
    char line[120];
    snprintf(line, sizeof(line), "boost: accel x%.3f, cruise x%.4f", accelRatio, cruiseRatio);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(accelRatio > 1.05f && accelRatio < 1.25f, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 1.0f, cruiseRatio, line);

    // ступенька стика: добавка не больше самой скорости — FF не больше чем вдвое
    RateAxis c = ffAxis(x), d = ffAxis(off);
    ffStep(c, 0.0f, 0.0f, 0.0f, true, 0.0f);
    ffStep(d, 0.0f, 0.0f, 0.0f, true, 0.0f);
    const float fc = ffStep(c, 300.0f, 0.5f, 0.5f, true, 0.0f);
    const float fd = ffStep(d, 300.0f, 0.5f, 0.5f, true, 0.0f);
    TEST_ASSERT_TRUE(fc > fd && fc <= 2.0f * fd * 1.0001f);
}

// 021–025 в контроллере: jitter и прочее — только на часть уставки от
// стика; в ANGLE поправка горизонта идёт в FF как была. Кадры — по
// счётчику rc.frame.
void test_feedforward_touches_only_the_stick_part()
{
    const auto ffWhileTilting = [](float jitter) {
        PidExtras x = flatTpa();
        x.ffJitterFactor = jitter;
        x.ffSmoothFactor = 0.25f;
        Bench b;
        b.ctl.configure(fiveWith(x));
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.rc.throttle = 0.4f;
        b.run(0.3f);
        // стик неподвижен, коптер наклоняет — горизонт гонит уставку
        b.sens.gyro[0] = 40.0f;
        float sumFf = 0.0f;
        for (int i = 0; i < 100; ++i)
        {
            if (i % 4 == 0) ++b.rc.frame;
            b.out = b.ctl.update(b.sens, b.rc);
            sumFf += b.ctl.axisTerms(0).ff;
        }
        return sumFf;
    };
    const float plain = ffWhileTilting(0.0f), jitter = ffWhileTilting(7.0f);
    TEST_ASSERT_TRUE(fabsf(plain) > 1e-4f);
    TEST_ASSERT_FLOAT_WITHIN(fabsf(plain) * 1e-3f, plain, jitter);

    // ACRO: стик стоит на месте между кадрами (дрожь пальца ±1 %) — с jitter FF почти нет
    const auto trembleFf = [](float factor) {
        PidExtras x = flatTpa();
        x.ffJitterFactor = factor;
        Bench b;
        b.ctl.configure(fiveWith(x));
        b.rc.mode = CopterMode::ACRO;
        TEST_ASSERT_TRUE(b.armAfterBoot());
        b.rc.throttle = 0.4f;
        b.run(0.3f);
        float sum2 = 0.0f;
        for (int i = 0; i < 1000; ++i)
        {
            if (i % 4 == 0)
            {
                ++b.rc.frame;
                b.rc.roll = 0.01f * sinf(2.0f * PI_F * 1.5f * i / 1000.0f);
            }
            b.out = b.ctl.update(b.sens, b.rc);
            sum2 += b.ctl.axisTerms(0).ff * b.ctl.axisTerms(0).ff;
        }
        return sqrtf(sum2 / 1000.0f);
    };
    const float tremble = trembleFf(7.0f), trembleOff = trembleFf(0.0f);
    TEST_ASSERT_TRUE(trembleOff > 0.0f && tremble < 0.25f * trembleOff);
}

// 021–025 в реестре: имена BF, модель — всё выкл, BETAFLIGHT — 25 / 7 / 15.
void test_feedforward_params()
{
    CopterParams& p = five5Params();
    char err[96];
    CopterTune t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.ffTransition);
    TEST_ASSERT_EQUAL(1, t.pid.ffAveraging);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.ffSmoothFactor);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.ffJitterFactor);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.ffBoost);
    TEST_ASSERT_TRUE(p.parse(static_cast<ParamId>(findParam("feedforward_transition")), 0, "40", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(static_cast<ParamId>(findParam("feedforward_averaging")), 0, "3_POINT", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(static_cast<ParamId>(findParam("feedforward_smooth_factor")), 0, "65", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(static_cast<ParamId>(findParam("feedforward_jitter_factor")), 0, "10", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(static_cast<ParamId>(findParam("feedforward_boost")), 0, "20", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(static_cast<ParamId>(findParam("feedforward_smooth_factor")), 0, "96", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(static_cast<ParamId>(findParam("feedforward_averaging")), 0, "5_POINT", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, t.pid.ffTransition);
    TEST_ASSERT_EQUAL(3, t.pid.ffAveraging);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.65f, t.pid.ffSmoothFactor);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, t.pid.ffJitterFactor);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, t.pid.ffBoost);
    // во втором профиле — свои
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.valueIn(PARAM_FEEDFORWARD_BOOST, 1));

    CopterParams& q = five5Params();
    TEST_ASSERT_TRUE(q.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    t = q.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, t.pid.ffSmoothFactor);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, t.pid.ffJitterFactor);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, t.pid.ffBoost);
    TEST_ASSERT_EQUAL(1, t.pid.ffAveraging);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.ffTransition);
}

// 026: feedforward_max_rate_limit — у края расхода упреждение наружу не
// больше P на оставшийся запас, за краем — ноль; к центру — как есть.
void test_feedforward_max_rate_limit()
{
    const auto ffAt = [](float limit, float from, float to) {
        RateAxis a = makeAxis(0.001f, 0.0f, 0.0f, 0.0001f);
        PidExtras x;
        x.ffMaxRateLimit = limit;
        a.setExtras(x, 0);
        a.setMaxRate(600.0f);
        a.update(from, from, 1.0f, 1.0f, true, false);
        a.update(to, from, 1.0f, 1.0f, true, false);
        return a.lastTerms().ff;
    };
    // без предела: FF = 0.0001 × (10 °/с за 1 мс) × доля ФНЧ 25 Гц
    const float free = ffAt(0.0f, 500.0f, 510.0f);
    TEST_ASSERT_TRUE(free > 0.1f);
    // предел 90 % — 540 °/с: запас 30 °/с, P 0.001 -> не больше 0.03
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.03f, ffAt(0.9f, 500.0f, 510.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ffAt(0.9f, 545.0f, 555.0f));       // за краем
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f, ffAt(0.9f, -500.0f, -510.0f));
    // далеко от края и к центру — не трогается
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, ffAt(0.0f, 100.0f, 110.0f), ffAt(0.9f, 100.0f, 110.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, ffAt(0.0f, 560.0f, 550.0f), ffAt(0.9f, 560.0f, 550.0f));

    // реестр: модель — 0 (перелёта нет и так), BF — 90; расход — из профиля расходов
    CopterParams& p = five5Params();
    char err[96];
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tune().pid.ffMaxRateLimit);
    TEST_ASSERT_TRUE(p.parse(PARAM_FEEDFORWARD_MAX_RATE_LIMIT, 0, "100", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_FEEDFORWARD_MAX_RATE_LIMIT, 0, "151", err, sizeof(err)));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, p.tune().pid.ffMaxRateLimit);
    CopterParams& q = five5Params();
    TEST_ASSERT_TRUE(q.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(90.0f, q.defaultValue(PARAM_FEEDFORWARD_MAX_RATE_LIMIT));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.9f, q.tune().pid.ffMaxRateLimit);
}

// 027: yaw_lowpass_hz — ФНЧ выхода рысканья настройкой; не задан — частота
// модели (BF — 100 Гц); 0 — без фильтра; меняется со сменой профиля.
void test_yaw_lowpass_hz()
{
    CopterParams& p = five5Params();
    char err[96];
    const float model = p.modelTune().yawLpfHz;
    TEST_ASSERT_EQUAL_FLOAT(model, p.tune().yawLpfHz);
    TEST_ASSERT_EQUAL_FLOAT(roundf(model), p.defaultValue(PARAM_YAW_LOWPASS_HZ));
    TEST_ASSERT_TRUE(p.parse(PARAM_YAW_LOWPASS_HZ, 1, "0", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_YAW_LOWPASS_HZ, 0, "501", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.selectPidProfile(1));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tune().yawLpfHz);
    CopterParams& q = five5Params();
    TEST_ASSERT_TRUE(q.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, q.tune().yawLpfHz);

    // выход рысканья через ФНЧ: 10 Гц — первый цикл ступеньки P — 6 %, 0 — весь
    const auto firstStep = [](float hz) {
        CopterTune t = deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600);
        t.yawLpfHz = 100.0f;
        CopterController ctl;
        ctl.configure(t);
        t.yawLpfHz = hz;
        ctl.applyProfile(t);   // на ходу, как смена профиля
        RateAxis a;
        a.configure(t.yaw, 0.0f, ctl.getTune().yawLpfHz, 0.3f, 0.001f);
        a.update(0.0f, 0.0f, 1.0f, 1.0f, true, false);
        a.update(0.0f, -10.0f, 1.0f, 1.0f, true, false);
        return a.lastTerms().sum / a.lastTerms().p;
    };
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.0f - expf(-2.0f * 3.14159265f * 10.0f * 0.001f), firstStep(10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, firstStep(0.0f));
}

// 028: integrated yaw — выход рысканья = интеграл суммы PID с утечкой;
// PID из модели пересчитаны (тот же контур), числа BF — как в BF (I без ×2.5).
void test_integrated_yaw()
{
    PidExtras x;
    x.integratedYaw = true;
    x.integratedYawRelax = 0.0f;
    RateAxis yaw = makeAxis(0.001f, 0.0f, 0.0f, 0.0f);
    yaw.setExtras(x, 2);
    // постоянная ошибка 10 °/с: P 0.01 -> выход растёт на 100 × 0.01 за секунду
    for (int i = 0; i < 100; ++i) yaw.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.1f, yaw.lastTerms().sum);
    for (int i = 0; i < 2000; ++i) yaw.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, yaw.lastTerms().sum);   // предел суммы
    // утечка 200 (16 1/с): выход встаёт на 100/16 × сумма
    x.integratedYawRelax = 200.0f;
    RateAxis leak = makeAxis(0.0001f, 0.0f, 0.0f, 0.0f);
    leak.setExtras(x, 2);
    for (int i = 0; i < 1000; ++i) leak.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.001f * 100.0f / 16.0f, leak.lastTerms().sum);
    // крен и тангаж — без интеграла
    RateAxis roll = makeAxis(0.001f, 0.0f, 0.0f, 0.0f);
    roll.setExtras(x, 0);
    roll.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, roll.lastTerms().sum);

    // реестр
    CopterParams& p = five5Params();
    char err[96];
    const AxisGains model = p.modelTune().yaw;
    TEST_ASSERT_FALSE(p.tune().pid.integratedYaw);
    TEST_ASSERT_TRUE(p.parse(PARAM_USE_INTEGRATED_YAW, 0, "ON", err, sizeof(err)));
    CopterTune t = p.tune();
    TEST_ASSERT_TRUE(t.pid.integratedYaw);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, t.pid.integratedYawRelax);
    const AxisGains conv = integratedYawGains(model, 200.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, (model.i + 16.0f * model.p) / 100.0f, conv.p);
    TEST_ASSERT_EQUAL_FLOAT(conv.p, t.yaw.p);
    TEST_ASSERT_EQUAL_FLOAT(conv.i, t.yaw.i);
    TEST_ASSERT_EQUAL_FLOAT(conv.d, t.yaw.d);
    TEST_ASSERT_EQUAL_FLOAT(conv.ff, t.yaw.ff);
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().roll.p, t.roll.p);   // крен не трогается
    // своё число — как в BF: i_yaw без множителя 2.5
    TEST_ASSERT_TRUE(p.parse(PARAM_I_YAW, 0, "80", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_P_YAW, 0, "45", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 80.0f * betaflight::I_SCALE / 1000.0f, t.yaw.i);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bfToGain(PidTerm::P, 2, 45.0f), t.yaw.p);
    TEST_ASSERT_EQUAL_FLOAT(conv.d, t.yaw.d);   // незаданные — из модели, пересчитанные
    TEST_ASSERT_TRUE(p.parse(PARAM_INTEGRATED_YAW_RELAX, 0, "100", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_INTEGRATED_YAW_RELAX, 0, "256", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, p.tune().pid.integratedYawRelax);
    TEST_ASSERT_EQUAL_FLOAT(integratedYawGains(model, 100.0f).d, p.tune().yaw.d);
    TEST_ASSERT_TRUE(p.parse(PARAM_USE_INTEGRATED_YAW, 0, "OFF", err, sizeof(err)));
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bfToGain(PidTerm::I, 2, 80.0f), p.tune().yaw.i);

    // включили на ходу (смена профиля) — выход без рывка
    RateAxis live = makeAxis(0.001f, 0.0f, 0.0f, 0.0f);
    live.setExtras(PidExtras(), 2);
    live.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    x.integratedYawRelax = 200.0f;
    live.setExtras(x, 2);
    live.update(10.0f, 0.0f, 1.0f, 1.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.01f, live.lastTerms().sum);
}

// 029: absolute control — потерянный угол копится и возвращается поправкой
// уставки; пока стик быстро движется, отставание рамы не копится.
void test_absolute_control()
{
    AbsoluteControl ac;
    ac.configure(11.0f, 0.001f);
    // уставка 0, рама 0.1 с крутится 10 °/с — потеряно 1°: поправка −gain × 1
    float corr = 0.0f;
    for (int i = 0; i < 100; ++i) corr = ac.update(0.0f, 10.0f, 10.0f, 90.0f, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, ac.error());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, -10.0f, corr);
    // пределы: ошибка — error_limit, поправка — limit
    for (int i = 0; i < 5000; ++i) corr = ac.update(0.0f, 10.0f, 10.0f, 90.0f, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f, ac.error());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -90.0f, corr);
    // рама вернулась (уставка стоит, рама на месте) — ошибка та же: угол ещё не вернули
    ac.update(0.0f, 0.0f, 10.0f, 90.0f, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -20.0f, ac.error());

    // быстрый стик: уставка 0 -> 500 °/с за 50 мс, рама отстаёт на 5 мс — не копится
    AbsoluteControl fast;
    fast.configure(11.0f, 0.001f);
    float peak = 0.0f;
    for (int i = 0; i < 50; ++i)
    {
        const float sp = 10.0f * i, gyro = i >= 5 ? 10.0f * (i - 5) : 0.0f;
        fast.update(sp, gyro, 10.0f, 90.0f, 20.0f);
        peak = fmaxf(peak, fabsf(fast.error()));
    }
    TEST_ASSERT_TRUE(peak < 0.05f);   // без полосы было бы ~2°

    // в контроллере: только в воздухе, крен — в ACRO; реестр
    CopterParams& p = five5Params();
    char err[96];
    CopterTune t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pid.absGain);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, t.pid.absLimit);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, t.pid.absErrorLimit);
    TEST_ASSERT_EQUAL_FLOAT(11.0f, t.pid.absCutoffHz);
    TEST_ASSERT_TRUE(p.parse(PARAM_ABS_CONTROL_GAIN, 0, "10", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ABS_CONTROL_LIMIT, 0, "120", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ABS_CONTROL_ERROR_LIMIT, 0, "30", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_ABS_CONTROL_CUTOFF, 0, "6", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_ABS_CONTROL_GAIN, 0, "21", err, sizeof(err)));
    t = p.tune();
    TEST_ASSERT_EQUAL_FLOAT(10.0f, t.pid.absGain);
    TEST_ASSERT_EQUAL_FLOAT(120.0f, t.pid.absLimit);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, t.pid.absErrorLimit);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, t.pid.absCutoffHz);

    // в воздухе: рама 0.2 с крутится по крену 10 °/с при стике в центре —
    // в ACRO потеряно 2°, уставка тянет назад на 20 °/с (P втрое больше);
    // в ANGLE угол держит горизонт — поправки нет
    const auto pWith = [](CopterMode mode, float gain) {
        PidExtras x;
        x.absGain = gain;
        Bench b;
        b.ctl.configure(fiveWith(x));
        armInAcro(b, 0.5f);
        b.rc.mode = mode;
        b.run(0.05f);
        b.sens.gyro[0] = 10.0f;
        b.run(0.2f);
        return b.ctl.axisTerms(0).p;
    };
    const float acroOff = pWith(CopterMode::ACRO, 0.0f), acroOn = pWith(CopterMode::ACRO, 10.0f);
    TEST_ASSERT_TRUE(acroOff < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f * fabsf(acroOff), 3.0f * acroOff, acroOn);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, pWith(CopterMode::ANGLE, 0.0f), pWith(CopterMode::ANGLE, 10.0f));
}

// 030: thrust_linear и motor_output_limit — настройкой; предел 80 % —
// команда регулятору не выше холостых + 80 % хода.
void test_thrust_linear_and_motor_output_limit()
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f + 0.95f * 0.8f, Mixer::withIdle(1.0f, 0.05f, 0.8f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, Mixer::withIdle(0.0f, 0.05f, 0.8f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, Mixer::withIdle(1.0f, 0.05f));

    CopterParams& p = five5Params();
    char err[96];
    const float model = p.modelTune().thrustLinear;
    TEST_ASSERT_EQUAL_FLOAT(roundf(model * 100.0f), p.defaultValue(PARAM_THRUST_LINEAR));
    TEST_ASSERT_EQUAL_FLOAT(model, p.tune().thrustLinear);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.tune().motorOutputLimit);
    TEST_ASSERT_TRUE(p.parse(PARAM_THRUST_LINEAR, 0, "40", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_MOTOR_OUTPUT_LIMIT, 0, "80", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_MOTOR_OUTPUT_LIMIT, 0, "0", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_THRUST_LINEAR, 0, "151", err, sizeof(err)));
    CopterTune t = p.tune();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, t.thrustLinear);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.motorOutputLimit);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, t.idle + (1.0f - t.idle) * 0.8f, t.motorCommand(1.0f));
    CopterParams& q = five5Params();
    TEST_ASSERT_TRUE(q.parse(PARAM_TUNE_SOURCE, 0, "BETAFLIGHT", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.defaultValue(PARAM_THRUST_LINEAR));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.tune().thrustLinear);

    // в полёте: полный газ и рывок стиком — ни один мотор не выше предела
    Bench b;
    b.ctl.configure(t);
    armInAcro(b, 1.0f);
    b.rc.roll = 1.0f;
    const float top = t.idle + (1.0f - t.idle) * 0.8f;
    float highest = 0.0f;
    for (int i = 0; i < 300; ++i)
    {
        b.run(0.001f);
        for (uint8_t m = 0; m < b.out.count; ++m) highest = fmaxf(highest, b.out.motor[m]);
    }
    TEST_ASSERT_TRUE(highest <= top + 1e-6f);
    TEST_ASSERT_TRUE(highest > top - 0.01f);
}

void test_cli_set_get_diff_save_and_paste()
{
    RamFlash flash;
    KeyValueStore kv(flash);
    CopterParams& p = five5Params();
    Bench b;
    b.ctl.configure(p.tune());
    CliCapture port;
    CopterCli cli(p, b.ctl, port);
    cli.setStore(&kv);
    cli.setVersion("OpenPlane copter / test");

    TEST_ASSERT_TRUE(cli.enter());
    TEST_ASSERT_TRUE(has(port.take(), "help"));
    for (const char c : std::string("set p_roll = 50\r\n")) cli.feed(c);
    std::string out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "set p_roll = 50"), out.c_str());   // эхо
    TEST_ASSERT_TRUE_MESSAGE(has(out, "p_roll = 50\r\n# "), out.c_str());
    TEST_ASSERT_TRUE(cli.hasUnsaved());
    // до save контроллер работает на старом
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().roll.p, b.ctl.getTune().roll.p);

    cli.execute("get roll");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "p_roll = 50\r\nДопустимо: 0 - 250, BF\r\nПо умолчанию: "), out.c_str());
    TEST_ASSERT_TRUE(has(out, "(из модели коптера)"));
    TEST_ASSERT_TRUE(has(out, "roll_srate = 72"));
    TEST_ASSERT_TRUE(has(out, "# P крена [PID-профиль]"));
    TEST_ASSERT_FALSE(has(out, "pitch"));

    cli.execute("set p_roll = 300");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: p_roll: допустимо 0 - 250###"));
    cli.execute("set nonsense = 1");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: нет настройки nonsense###"));
    cli.execute("set p_roll 5");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: set имя = значение###"));
    cli.execute("frobnicate");
    TEST_ASSERT_TRUE(has(port.take(), "нет команды frobnicate"));

    cli.execute("diff");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "# diff\r\n\r\n# version\r\n# OpenPlane copter / test"), out.c_str());
    TEST_ASSERT_TRUE(has(out, "\r\nprofile 0\r\n\r\n# profile 0\r\nset p_roll = 50\r\n"));
    TEST_ASSERT_TRUE(has(out, "rateprofile 0"));
    TEST_ASSERT_FALSE(has(out, "i_roll"));
    TEST_ASSERT_FALSE(has(out, "profile 1"));
    cli.execute("dump all");
    out = port.take();
    TEST_ASSERT_TRUE(has(out, "set tune_source = MODEL"));
    TEST_ASSERT_TRUE(has(out, "# profile 3\r\n"));
    TEST_ASSERT_TRUE(has(out, "set i_roll = "));
    TEST_ASSERT_TRUE(has(out, "# выбранные профили\r\nprofile 0\r\nrateprofile 0"));

    // вставка diff из Betaflight: комментарии, batch, профили, чужие команды
    const char* bfDiff[] = {
        "# diff all",
        "# version",
        "# Betaflight / STM32F7X2 (S7X2) 4.5.1",
        "batch start",
        "feature -AIRMODE",
        "set tune_source = BETAFLIGHT",
        "profile 1",
        "# profile 1",
        "set profile_name = wind",
        "set d_pitch = 38",
        "set d_max_pitch = 50",
        "set no_such_bf_setting = 1",
        "rateprofile 1",
        "set roll_srate = 80",
        "batch end",
    };
    for (const char* line : bfDiff) cli.execute(line);
    out = port.take();
    TEST_ASSERT_TRUE(has(out, "###ОШИБКА: нет команды feature"));
    TEST_ASSERT_TRUE(has(out, "###ОШИБКА: нет настройки no_such_bf_setting###"));
    TEST_ASSERT_FALSE(has(out, "d_max_pitch###"));
    cli.execute("profile");
    TEST_ASSERT_TRUE(has(port.take(), "profile 1 # wind"));
    TEST_ASSERT_EQUAL_UINT8(1, p.pidProfile());
    TEST_ASSERT_EQUAL_FLOAT(38.0f, p.value(PARAM_D_PITCH));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, p.value(PARAM_D_MAX_PITCH));
    TEST_ASSERT_EQUAL_FLOAT(80.0f, p.value(PARAM_ROLL_SRATE));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, p.valueIn(PARAM_P_ROLL, 0));
    // профиль переключился сразу (на дефолты BF профиля 1), set — заработает после save
    TEST_ASSERT_EQUAL_FLOAT(670.0f, b.ctl.getTune().rates[0].maxDps);

    cli.execute("status");
    TEST_ASSERT_EQUAL_INT(1, port.statusCalls);
    cli.execute("version");
    TEST_ASSERT_TRUE(has(port.take(), "# OpenPlane copter / test"));

    // save: во флеш, настройка пересчитана, CLI закрыт
    cli.execute("save");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "Сохранено"), out.c_str());
    TEST_ASSERT_FALSE(cli.active());
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::D), static_cast<int>(b.ctl.getTune().pid.tpaMode));
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(PidTerm::D, 1, 38.0f), b.ctl.getTune().pitch.d);
    TEST_ASSERT_TRUE(kv.contains(CopterParams::STORE_SPACE, CopterParams::STORE_KEY));

    // exit без save — отмена
    TEST_ASSERT_TRUE(cli.enter());
    cli.execute("set i_roll = 99");
    cli.execute("profile 0");
    cli.execute("exit");
    TEST_ASSERT_TRUE(has(port.take(), "Несохранённые изменения отменены."));
    TEST_ASSERT_EQUAL_UINT8(1, p.pidProfile());
    TEST_ASSERT_FALSE(p.isSet(PARAM_I_ROLL, 1));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, b.ctl.getTune().rates[0].maxDps);

    // defaults nosave — в ОЗУ; defaults — и во флеш
    TEST_ASSERT_TRUE(cli.enter());
    cli.execute("defaults nosave");
    TEST_ASSERT_TRUE(cli.active());
    TEST_ASSERT_EQUAL(static_cast<int>(TuneSource::MODEL), static_cast<int>(p.source()));
    cli.execute("defaults");
    TEST_ASSERT_FALSE(cli.active());
    TEST_ASSERT_FALSE(kv.contains(CopterParams::STORE_SPACE, CopterParams::STORE_KEY));
    TEST_ASSERT_EQUAL_FLOAT(p.modelTune().roll.p, b.ctl.getTune().roll.p);

    // профиль за границей, длинная строка, заармленному — нельзя
    TEST_ASSERT_TRUE(cli.enter());
    cli.execute("profile 4");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: profile 0-3###"));
    for (int i = 0; i < 200; ++i) cli.feed('x');
    cli.feed('\n');
    TEST_ASSERT_TRUE(has(port.take(), "строка длиннее"));
    cli.feed('s');
    cli.feed('x');
    cli.feed('\b');
    cli.feed('e');
    cli.feed('t');
    cli.feed('\r');
    TEST_ASSERT_TRUE(has(port.take(), "tune_source = MODEL"));   // «set» после забоя
    cli.execute("exit");
    TEST_ASSERT_TRUE(b.armAfterBoot());
    TEST_ASSERT_FALSE(cli.enter());
    TEST_ASSERT_TRUE(has(port.take(), "только без ARM"));
}

// ---- пульт: map, rxrange, aux (copter/RcSetup.h) ----

void test_rc_map_and_rxrange()
{
    RcConfig c;
    char err[96] = {}, m[9];
    c.formatMap(m, sizeof(m));
    TEST_ASSERT_EQUAL_STRING("AETR1234", m);
    TEST_ASSERT_TRUE(c.mapIsDefault());
    // по умолчанию: крен CH1, тангаж CH2, рысканье CH4, газ CH3, AUX1 — CH5
    TEST_ASSERT_EQUAL_UINT8(0, c.channelFor(0));
    TEST_ASSERT_EQUAL_UINT8(3, c.channelFor(2));
    TEST_ASSERT_EQUAL_UINT8(2, c.channelFor(3));
    TEST_ASSERT_EQUAL_UINT8(4, c.auxChannel(0));
    TEST_ASSERT_EQUAL_UINT8(8, c.auxChannel(4));   // после восьмого — по порядку

    TEST_ASSERT_TRUE(c.setMap("taer", err, sizeof(err)));   // 4 буквы, регистр не важен
    c.formatMap(m, sizeof(m));
    TEST_ASSERT_EQUAL_STRING("TAER1234", m);
    TEST_ASSERT_EQUAL_UINT8(0, c.channelFor(3));   // газ — CH1
    TEST_ASSERT_EQUAL_UINT8(1, c.channelFor(0));   // крен — CH2
    TEST_ASSERT_TRUE(c.setMap("AETR4321", err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(7, c.auxChannel(0));
    TEST_ASSERT_FALSE(c.setMap("AATR1234", err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("буква A дважды", err);
    TEST_ASSERT_FALSE(c.setMap("AETX1234", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.setMap("AET", err, sizeof(err)));
    c.formatMap(m, sizeof(m));
    TEST_ASSERT_EQUAL_STRING("AETR4321", m);   // ошибка не портит

    TEST_ASSERT_TRUE(c.setRange(3, 1020, 1980, err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT16(1020, c.rangeLow(3));
    TEST_ASSERT_FALSE(c.rangeIsDefault(3));
    TEST_ASSERT_FALSE(c.setRange(4, 1000, 2000, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.setRange(0, 2000, 1000, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.setRange(0, 700, 2000, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.isDefault());
    c.reset();
    TEST_ASSERT_TRUE(c.isDefault());
}

namespace
{
    // Каналы приёмника: стики в центре, газ внизу, AUX — 1000.
    struct RxBench
    {
        uint16_t ch[10] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000 };
        RcConfig cfg;
        RxInput in;
        CopterRc read(bool link = true)
        {
            in.configure(cfg, 885, 2115);
            return in.read(ch, 10, link);
        }
    };

    void useDefaultSwitches(RcConfig& c)
    {
        AuxRange fb[RcConfig::AUX_SLOTS];
        // как CopterConfig::defaultAux: ARM — CH5 1700..2100, CH7: ANGLE / ALTHOLD / ACRO
        fb[0] = { BOX_ARM, 0, 1700, 2100, 0, 0 };
        fb[1] = { BOX_ANGLE, 2, 900, 1300, 0, 0 };
        fb[2] = { BOX_ALTHOLD, 2, 1300, 1700, 0, 0 };
        c.setFallback(fb, 3);
    }
}

void test_rx_input_modes_like_betaflight()
{
    RxBench b;
    useDefaultSwitches(b.cfg);
    char err[96];

    // пустая таблица — тумблеры прошивки
    CopterRc rc = b.read();
    TEST_ASSERT_TRUE(rc.linkOk);
    TEST_ASSERT_FALSE(rc.armSwitch);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));
    TEST_ASSERT_TRUE(rc.armAssigned);
    b.ch[4] = 2000;
    b.ch[6] = 1500;
    rc = b.read();
    TEST_ASSERT_TRUE(rc.armSwitch);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ALT_HOLD), static_cast<int>(rc.mode));
    b.ch[6] = 2000;
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ACRO), static_cast<int>(b.read().mode));
    b.ch[6] = 2150;   // выше 2100 — как 2099 (BF прижимает)
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ACRO), static_cast<int>(b.read().mode));

    // своя строка заменяет всю таблицу: ARM на CH6, ANGLE на CH7 посередине
    TEST_ASSERT_TRUE(b.cfg.parseAux("0 0 1 1700 2100 0 0", err, sizeof(err)));
    TEST_ASSERT_TRUE(b.cfg.parseAux("1 1 2 1300 1700", err, sizeof(err)));
    b.ch[4] = 2000;
    b.ch[5] = 1000;
    b.ch[6] = 1000;
    rc = b.read();
    TEST_ASSERT_FALSE(rc.armSwitch);   // CH5 больше не ARM
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ACRO), static_cast<int>(rc.mode));
    b.ch[5] = 1900;
    b.ch[6] = 1500;
    rc = b.read();
    TEST_ASSERT_TRUE(rc.armSwitch);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));
    TEST_ASSERT_TRUE(b.in.isActive(BOX_ANGLE));

    // приоритет: ALTHOLD > ANGLE > HORIZON; HORIZON — когда ANGLE нет
    TEST_ASSERT_TRUE(b.cfg.parseAux("2 2 2 900 2100", err, sizeof(err)));
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(b.read().mode));
    b.ch[6] = 1900;
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::HORIZON), static_cast<int>(b.read().mode));
    TEST_ASSERT_TRUE(b.cfg.parseAux("3 3 3 1700 2100", err, sizeof(err)));
    b.ch[7] = 2000;
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ALT_HOLD), static_cast<int>(b.read().mode));

    // шаг 25 мкс вниз, как в BF; границы
    TEST_ASSERT_TRUE(b.cfg.parseAux("4 28 4 1310 1999", err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT16(1300, b.cfg.aux(4).startUs);
    TEST_ASSERT_EQUAL_UINT16(1975, b.cfg.aux(4).endUs);
    TEST_ASSERT_FALSE(b.cfg.parseAux("20 0 0 1700 2100", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 64 0 1700 2100", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 0 6 1700 2100", err, sizeof(err)));   // AUX7 — у приёмника 10 каналов
    TEST_ASSERT_EQUAL_STRING("канал 0-5 (AUX1-AUX6, у приёмника 10 каналов)", err);
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 0 0 800 2100", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 0 0 1700", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 0 0 1700 2100 2", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("0 0 0 1700 2100 0 0 9", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.cfg.parseAux("x", err, sizeof(err)));

    // AIR MODE, FAILSAFE, PREARM
    b.ch[8] = 1500;
    rc = b.read();
    TEST_ASSERT_TRUE(rc.airmode);
    TEST_ASSERT_FALSE(rc.failsafeSwitch);
    TEST_ASSERT_TRUE(b.cfg.parseAux("5 27 5 1700 2100", err, sizeof(err)));
    b.ch[9] = 2000;
    TEST_ASSERT_TRUE(b.read().failsafeSwitch);
    TEST_ASSERT_TRUE(b.read().prearm);   // PREARM нет в таблице
    TEST_ASSERT_TRUE(b.cfg.parseAux("6 36 5 900 1300", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.read().prearm);
    b.ch[9] = 1000;
    TEST_ASSERT_TRUE(b.read().prearm);

    // ARM нет в таблице
    TEST_ASSERT_TRUE(b.cfg.parseAux("0 0 0 900 900", err, sizeof(err)));
    TEST_ASSERT_FALSE(b.read().armAssigned);

    // импульс вне rx_min_usec..rx_max_usec: стик — связь потеряна, AUX — прежнее
    b.ch[7] = 1000;
    b.ch[6] = 1500;
    TEST_ASSERT_TRUE(b.read().linkOk);
    b.ch[6] = 600;
    rc = b.read();
    TEST_ASSERT_TRUE(rc.linkOk);
    TEST_ASSERT_EQUAL_UINT16(1500, b.in.auxUs(2));
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));
    b.ch[0] = 2200;
    TEST_ASSERT_FALSE(b.read().linkOk);
    b.ch[0] = 1500;
    TEST_ASSERT_FALSE(b.read(false).linkOk);
}

void test_rx_input_logic_linked_map_and_rxrange()
{
    // И: нужны обе строки; ИЛИ: хватит любой
    AuxRange t[RcConfig::AUX_SLOTS];
    t[0] = { BOX_ANGLE, 0, 1700, 2100, 1, 0 };
    t[1] = { BOX_ANGLE, 1, 1700, 2100, 1, 0 };
    t[2] = { BOX_HORIZON, 0, 1700, 2100, 0, 0 };
    t[3] = { BOX_HORIZON, 1, 1700, 2100, 0, 0 };
    t[4] = { BOX_AIRMODE, 0, 900, 900, 0, BOX_ANGLE };   // связан с ANGLE
    t[4].endUs = 2100;
    uint16_t aux[6] = { 2000, 1000, 1000, 1000, 1000, 1000 };
    uint64_t m = RxInput::evaluate(t, aux, 6);
    TEST_ASSERT_FALSE((m >> BOX_ANGLE) & 1u);
    TEST_ASSERT_TRUE((m >> BOX_HORIZON) & 1u);
    TEST_ASSERT_FALSE((m >> BOX_AIRMODE) & 1u);
    aux[1] = 2000;
    m = RxInput::evaluate(t, aux, 6);
    TEST_ASSERT_TRUE((m >> BOX_ANGLE) & 1u);
    TEST_ASSERT_TRUE((m >> BOX_AIRMODE) & 1u);

    // map TAER: газ с CH1, крен с CH2; rxrange растягивает концы
    RxBench b;
    char err[96];
    TEST_ASSERT_TRUE(b.cfg.setMap("TAER1234", err, sizeof(err)));
    b.ch[0] = 2000;   // газ вверх
    b.ch[1] = 2000;   // крен вправо
    b.ch[2] = 1500;
    CopterRc rc = b.read();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, rc.throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, rc.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rc.pitch);
    TEST_ASSERT_EQUAL_UINT8(0, b.in.throttleChannel());
    TEST_ASSERT_TRUE(b.cfg.setRange(3, 1100, 1900, err, sizeof(err)));   // газ 1100..1900
    b.ch[0] = 1500;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, b.read().throttle);
    b.ch[0] = 1900;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, b.read().throttle);
    b.ch[0] = 1100;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, b.read().throttle);
    b.ch[1] = 1000;   // тангаж в BF-порядке — функция E, у TAER это CH3
    b.ch[2] = 2000;
    rc = b.read();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, rc.pitch);   // от себя — нос вниз
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, rc.roll);
}

void test_rc_tables_survive_flash()
{
    RamFlash flash;
    {
        KeyValueStore kv(flash);
        CopterParams& p = five5Params();
        char err[96];
        TEST_ASSERT_TRUE(p.rc().setMap("TAER1234", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.rc().setRange(1, 990, 2010, err, sizeof(err)));
        TEST_ASSERT_TRUE(p.rc().parseAux("0 0 1 1700 2100 0 0", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.rc().parseAux("7 36 3 1300 1700 1 1", err, sizeof(err)));
        TEST_ASSERT_TRUE(p.save(kv));
        TEST_ASSERT_TRUE(kv.commit());
        TEST_ASSERT_TRUE(kv.contains(CopterParams::STORE_SPACE, CopterParams::RC_KEY));
        TEST_ASSERT_FALSE(kv.contains(CopterParams::STORE_SPACE, CopterParams::STORE_KEY));   // set не трогали
    }
    {
        KeyValueStore kv(flash);
        CopterParams p;
        p.begin(*findBuild("freestyle5"), 1000.0f, EscProtocol::ONESHOT125);
        TEST_ASSERT_TRUE(p.load(kv));
        char m[9];
        p.rc().formatMap(m, sizeof(m));
        TEST_ASSERT_EQUAL_STRING("TAER1234", m);
        TEST_ASSERT_EQUAL_UINT16(990, p.rc().rangeLow(1));
        TEST_ASSERT_EQUAL_UINT16(2010, p.rc().rangeHigh(1));
        TEST_ASSERT_EQUAL_UINT8(1, p.rc().aux(0).channel);
        const AuxRange& a = p.rc().aux(7);
        TEST_ASSERT_EQUAL_UINT8(BOX_PREARM, a.mode);
        TEST_ASSERT_EQUAL_UINT16(1300, a.startUs);
        TEST_ASSERT_EQUAL_UINT16(1700, a.endUs);
        TEST_ASSERT_EQUAL_UINT8(1, a.logic);
        TEST_ASSERT_EQUAL_UINT8(1, a.linked);
        // всё по умолчанию — ключа нет
        p.resetAll();
        TEST_ASSERT_TRUE(p.save(kv));
        TEST_ASSERT_FALSE(kv.contains(CopterParams::STORE_SPACE, CopterParams::RC_KEY));
    }
    // порченые данные — по умолчанию
    RcConfig c;
    uint8_t bad[1 + 8 + 16 + 1] = { RcConfig::STORE_VERSION, 0, 0, 3, 2, 4, 5, 6, 7 };   // CH1 дважды
    TEST_ASSERT_FALSE(c.deserialize(bad, sizeof(bad)));
    TEST_ASSERT_TRUE(c.isDefault());
    TEST_ASSERT_FALSE(c.deserialize(nullptr, 0));
}

// ---- описание коптера — тоже настройки ----

void test_copter_description_is_params()
{
    CopterParams& p = five5Params();
    char err[96], out[48];
    const CopterTune five = p.modelTune();
    TEST_ASSERT_EQUAL_STRING("freestyle5", p.buildSpec().id);
    TEST_ASSERT_EQUAL_FLOAT(680.0f, p.value(PARAM_COPTER_MASS_G));
    TEST_ASSERT_EQUAL_FLOAT(5.1f, p.value(PARAM_PROP_DIAMETER_IN));
    p.format(PARAM_PROP_DIAMETER_IN, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("5.1", out);   // не 5.100
    p.format(PARAM_MOTOR_STATOR_MM, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("22", out);
    TEST_ASSERT_EQUAL_STRING("copter", paramGroupName(paramInfo(PARAM_COPTER_BUILD).group));

    // другая сборка: умолчания описания — из неё, заданное руками остаётся
    TEST_ASSERT_TRUE(p.parse(PARAM_BATTERY_MAH, 0, "1500", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_COPTER_BUILD, 0, "race5", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(560.0f, p.value(PARAM_COPTER_MASS_G));
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, p.value(PARAM_BATTERY_MAH));
    BuildSpec b = p.buildSpec();
    TEST_ASSERT_EQUAL_STRING("race5", b.id);
    TEST_ASSERT_EQUAL(static_cast<int>(FrameType::QUAD_X_STRETCHED), static_cast<int>(b.frame));
    TEST_ASSERT_EQUAL_UINT16(1500, b.battery.capacityMah);
    // модель — только по updateModel (save)
    TEST_ASSERT_EQUAL_FLOAT(five.massKg, p.modelTune().massKg);
    TEST_ASSERT_TRUE(p.updateModel());
    TEST_ASSERT_FALSE(p.updateModel());   // уже
    TEST_ASSERT_EQUAL(static_cast<int>(FrameType::QUAD_X_STRETCHED), static_cast<int>(p.modelTune().frame.type));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.560f, p.modelTune().massKg);

    // масса тяжелее — висение выше; другой винт — его масса заново по размеру
    TEST_ASSERT_TRUE(p.parse(PARAM_COPTER_MASS_G, 0, "900", err, sizeof(err)));
    const float hoverBefore = p.modelTune().hoverThrust;
    TEST_ASSERT_TRUE(p.updateModel());
    TEST_ASSERT_TRUE(p.modelTune().hoverThrust > hoverBefore);
    TEST_ASSERT_TRUE(p.buildSpec().prop.massG > 0.0f);
    TEST_ASSERT_TRUE(p.parse(PARAM_PROP_DIAMETER_IN, 0, "5.5", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.buildSpec().prop.massG);
    TEST_ASSERT_FALSE(p.parse(PARAM_COPTER_BUILD, 0, "dji", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_BATTERY_CELLS, 0, "15", err, sizeof(err)));

    // поправки прошивки (CopterConfig.h) — и к другой сборке
    CopterParams q;
    BuildSpec mine = *findBuild("freestyle5");
    mine.massG = 742;
    q.begin(mine, 1000.0f, EscProtocol::ONESHOT125, [](BuildSpec& s) { s.massG = 742; });
    TEST_ASSERT_EQUAL_FLOAT(742.0f, q.value(PARAM_COPTER_MASS_G));
    char g[64];
    TEST_ASSERT_TRUE(q.parse(PARAM_COPTER_BUILD, 0, "longrange7", g, sizeof(g)));
    TEST_ASSERT_EQUAL_FLOAT(742.0f, q.value(PARAM_COPTER_MASS_G));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, q.value(PARAM_PROP_DIAMETER_IN));

    // описание из флеша — модель при загрузке
    RamFlash flash;
    KeyValueStore kv(flash);
    TEST_ASSERT_TRUE(q.save(kv));
    CopterParams r;
    r.begin(*findBuild("freestyle5"), 1000.0f, EscProtocol::ONESHOT125);
    TEST_ASSERT_TRUE(r.load(kv));
    TEST_ASSERT_EQUAL_STRING("longrange7", r.buildSpec().id);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 7.0f, r.modelTune().prop.prop.diameterIn);

    // протокол приёмника: умолчание — от прошивки
    p.setTableDefault(PARAM_SERIALRX_PROVIDER, 2);
    p.format(PARAM_SERIALRX_PROVIDER, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("CRSF", out);
    p.setTableDefault(PARAM_SERIALRX_PROVIDER, 0);
    TEST_ASSERT_EQUAL_FLOAT(885.0f, p.value(PARAM_RX_MIN_USEC));
}

void test_cli_aux_map_rxrange_and_diff()
{
    CopterParams& p = five5Params();
    useDefaultSwitches(p.rc());
    Bench b;
    b.ctl.configure(p.tune());
    CliCapture port;
    CopterCli cli(p, b.ctl, port);
    TEST_ASSERT_TRUE(cli.enter());
    port.take();

    // пустая таблица: подсказка с тумблерами прошивки строками aux
    cli.execute("aux");
    std::string out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "# таблица aux пуста"), out.c_str());
    TEST_ASSERT_TRUE(has(out, "# aux 0 0 0 1700 2100 0 0   — ARM"));
    TEST_ASSERT_TRUE(has(out, "# aux 2 3 2 1300 1700 0 0   — ALTHOLD"));

    cli.execute("aux 1 1 2 1300 1700 0 0");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "aux 1 1 2 1300 1700 0 0\r\n"), out.c_str());
    TEST_ASSERT_TRUE(has(out, "# в таблице нет ARM — коптер не армится"));
    TEST_ASSERT_TRUE(cli.hasUnsaved());
    cli.execute("aux 0 0 1 1700 2100 0 0");
    TEST_ASSERT_FALSE(has(port.take(), "нет ARM"));
    cli.execute("aux 2 46 3 1700 2100 0 0");
    TEST_ASSERT_TRUE(has(port.take(), "# режим 46 GPS RESCUE у нас пока не действует — строка хранится"));
    cli.execute("aux 3 0 9 1700 2100 0 0");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: канал 0-5"));
    cli.execute("aux help");
    TEST_ASSERT_TRUE(has(port.take(), "# 28 — AIR MODE\r\n"));
    cli.execute("aux");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "aux 0 0 1 1700 2100 0 0   # ARM\r\n"), out.c_str());
    TEST_ASSERT_FALSE(has(out, "пуста"));

    cli.execute("map");
    TEST_ASSERT_TRUE(has(port.take(), "map AETR1234\r\n"));
    cli.execute("map TAER1234");
    TEST_ASSERT_TRUE(has(port.take(), "map TAER1234\r\n"));
    cli.execute("map TTER1234");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: буква T дважды###"));
    cli.execute("rxrange 3 1050 1950");
    TEST_ASSERT_TRUE(has(port.take(), "rxrange 3 1050 1950\r\n"));
    cli.execute("rxrange 3 1050");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: rxrange 0-3 мин макс###"));
    cli.execute("rxrange 5 1000 2000");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: rxrange 0-3"));
    cli.execute("rxrange");
    TEST_ASSERT_TRUE(has(port.take(), "rxrange 0 1000 2000\r\nrxrange 1 1000 2000\r\nrxrange 2 1000 2000\r\nrxrange 3 1050 1950\r\n"));

    // diff — как в BF: map, aux, rxrange перед # master
    cli.execute("set copter_mass_g = 742");
    port.take();
    cli.execute("diff");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "\r\n# map\r\nmap TAER1234\r\n\r\n# aux\r\naux 0 0 1 1700 2100 0 0\r\naux 1 1 2 1300 1700 0 0\r\n"
                                      "aux 2 46 3 1700 2100 0 0\r\n\r\n# rxrange\r\nrxrange 3 1050 1950\r\n\r\n# master\r\n"
                                      "set copter_mass_g = 742\r\n"),
                             out.c_str());
    cli.execute("dump");
    out = port.take();
    TEST_ASSERT_TRUE(has(out, "aux 19 0 0 900 900 0 0\r\n"));
    TEST_ASSERT_TRUE(has(out, "rxrange 0 1000 2000\r\n"));
    TEST_ASSERT_TRUE(has(out, "set copter_build = freestyle5\r\n"));
    TEST_ASSERT_TRUE(has(out, "set serialrx_provider = IBUS\r\n"));
    cli.execute("get copter_mass");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "По умолчанию: 680 (из сборки freestyle5)"), out.c_str());

    // defaults nosave — таблицы пусты
    cli.execute("defaults nosave");
    TEST_ASSERT_TRUE(p.rc().isDefault());
    cli.execute("exit");
    port.take();
}

// ---- 104: rxfail ----

// Строки rxfail как в BF: стики a/h, AUX h/s; умолчания, ошибки, флеш.
void test_rxfail_table_like_betaflight()
{
    RcConfig c;
    char err[96], row[32];
    TEST_ASSERT_EQUAL(static_cast<int>(RxfailMode::AUTO), static_cast<int>(c.rxfail(0).mode));
    TEST_ASSERT_EQUAL(static_cast<int>(RxfailMode::AUTO), static_cast<int>(c.rxfail(3).mode));
    TEST_ASSERT_EQUAL(static_cast<int>(RxfailMode::HOLD), static_cast<int>(c.rxfail(4).mode));
    TEST_ASSERT_EQUAL(static_cast<int>(RxfailMode::HOLD), static_cast<int>(c.rxfail(17).mode));
    c.formatRxfail(0, row, sizeof(row));
    TEST_ASSERT_EQUAL_STRING("rxfail 0 a", row);
    c.formatRxfail(5, row, sizeof(row));
    TEST_ASSERT_EQUAL_STRING("rxfail 5 h", row);
    TEST_ASSERT_FALSE(c.anyRxfail());
    TEST_ASSERT_EQUAL_UINT8(0, c.stickHoldMask());

    // стик — a или h; AUX — h или s значение (шаг 25 вниз)
    TEST_ASSERT_TRUE(c.parseRxfail("3 h", err, sizeof(err)));
    TEST_ASSERT_TRUE(c.parseRxfail("1 H", err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(0x0A, c.stickHoldMask());
    TEST_ASSERT_FALSE(c.parseRxfail("0 s 1500", err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("у стиков режим a (авто) или h (держать)", err);
    TEST_ASSERT_FALSE(c.parseRxfail("4 a", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseRxfail("4 s", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseRxfail("4 s 2300", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseRxfail("18 h", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseRxfail("4 s 1500 x", err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseRxfail("x", err, sizeof(err)));
    TEST_ASSERT_TRUE(c.parseRxfail("4 s 1990", err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT16(1975, c.rxfail(4).us);
    c.formatRxfail(4, row, sizeof(row));
    TEST_ASSERT_EQUAL_STRING("rxfail 4 s 1975", row);
    TEST_ASSERT_TRUE(c.anyRxfail());
    TEST_ASSERT_FALSE(c.isDefault());

    // во флеш и обратно; старая запись без rxfail — умолчания
    uint8_t buf[RcConfig::STORE_MAX];
    const size_t n = c.serialize(buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    RcConfig back;
    TEST_ASSERT_TRUE(back.deserialize(buf, n));
    TEST_ASSERT_EQUAL(static_cast<int>(RxfailMode::SET), static_cast<int>(back.rxfail(4).mode));
    TEST_ASSERT_EQUAL_UINT16(1975, back.rxfail(4).us);
    TEST_ASSERT_EQUAL_UINT8(0x0A, back.stickHoldMask());
    TEST_ASSERT_TRUE(back.deserialize(buf, n - 1 - 3 * 3));   // без блока rxfail
    TEST_ASSERT_FALSE(back.anyRxfail());
    c.reset();
    TEST_ASSERT_TRUE(c.isDefault());
}

// AUX без связи: 0.3 с — прежнее, затем rxfail (HOLD — прежнее, SET —
// заданное); связь вернулась — снова с пульта. Испорченный импульс AUX при
// живой связи — так же.
void test_rx_input_rxfail_values()
{
    RxBench b;
    useDefaultSwitches(b.cfg);
    char err[96];
    TEST_ASSERT_TRUE(b.cfg.parseRxfail("6 s 1500", err, sizeof(err)));   // AUX3 (CH7): ALTHOLD
    TEST_ASSERT_TRUE(b.cfg.parseRxfail("2 h", err, sizeof(err)));
    b.in.configure(b.cfg, 885, 2115);
    b.ch[4] = 2000;   // ARM
    b.ch[5] = 1800;   // AUX2
    CopterRc rc = b.in.read(b.ch, 10, true, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));
    TEST_ASSERT_EQUAL_UINT8(0x04, rc.stickHold);

    // приёмник без связи шлёт что попало — не верим
    b.ch[5] = 1100;
    b.ch[6] = 1000;
    rc = b.in.read(b.ch, 10, false, 1100);
    TEST_ASSERT_FALSE(rc.linkOk);
    TEST_ASSERT_EQUAL_UINT16(1800, b.in.auxUs(1));
    rc = b.in.read(b.ch, 10, false, 1399);
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));
    rc = b.in.read(b.ch, 10, false, 1400);   // 0.3 с — значение rxfail
    TEST_ASSERT_EQUAL_UINT16(1500, b.in.auxUs(2));
    TEST_ASSERT_EQUAL_UINT16(1800, b.in.auxUs(1));   // HOLD
    TEST_ASSERT_EQUAL_UINT16(2000, b.in.auxUs(0));   // ARM — держится
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ALT_HOLD), static_cast<int>(rc.mode));

    // связь вернулась
    rc = b.in.read(b.ch, 10, true, 1500);
    TEST_ASSERT_TRUE(rc.linkOk);
    TEST_ASSERT_EQUAL_UINT16(1100, b.in.auxUs(1));
    TEST_ASSERT_EQUAL(static_cast<int>(CopterMode::ANGLE), static_cast<int>(rc.mode));

    // испорченный импульс AUX3 при живой связи: таймер — заново
    b.ch[6] = 3000;
    rc = b.in.read(b.ch, 10, true, 2000);
    TEST_ASSERT_TRUE(rc.linkOk);
    TEST_ASSERT_EQUAL_UINT16(1000, b.in.auxUs(2));
    rc = b.in.read(b.ch, 10, true, 2300);
    TEST_ASSERT_EQUAL_UINT16(1500, b.in.auxUs(2));

    // без времени (nowMs = 0) — просто держится
    RxBench plain;
    TEST_ASSERT_TRUE(plain.cfg.parseRxfail("4 s 2000", err, sizeof(err)));
    plain.read(true);
    plain.ch[4] = 1700;
    plain.read(false);
    plain.read(false);
    TEST_ASSERT_EQUAL_UINT16(1000, plain.in.auxUs(0));
}

void test_cli_rxfail_and_diff()
{
    CopterParams& p = five5Params();
    Bench b;
    b.ctl.configure(p.tune());
    CliCapture port;
    CopterCli cli(p, b.ctl, port);
    TEST_ASSERT_TRUE(cli.enter());
    port.take();
    cli.execute("rxfail");
    std::string out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "rxfail 0 a\r\nrxfail 1 a\r\nrxfail 2 a\r\nrxfail 3 a\r\nrxfail 4 h\r\n"), out.c_str());
    TEST_ASSERT_TRUE(has(out, "rxfail 17 h\r\n"));
    cli.execute("rxfail 7 s 1000");
    TEST_ASSERT_TRUE(has(port.take(), "rxfail 7 s 1000\r\n"));
    TEST_ASSERT_TRUE(cli.hasUnsaved());
    cli.execute("rxfail 3 h");
    port.take();
    cli.execute("rxfail 7");
    TEST_ASSERT_TRUE(has(port.take(), "rxfail 7 s 1000\r\n"));
    cli.execute("rxfail 0 s 1500");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: у стиков режим a (авто) или h (держать)###"));
    cli.execute("diff");
    out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "\r\n# rxfail\r\nrxfail 3 h\r\nrxfail 7 s 1000\r\n"), out.c_str());
    cli.execute("dump");
    TEST_ASSERT_TRUE(has(port.take(), "rxfail 0 a\r\n"));
    cli.execute("defaults nosave");
    TEST_ASSERT_TRUE(p.rc().isDefault());
    cli.execute("exit");
    port.take();
}

// ---- батарея (copter/Battery.h) ----

namespace
{
    // мВ на пине при напряжении батареи v (делитель 1:11)
    float pinMv(float v) { return v * 1000.0f / 11.0f; }

    void feed(BatteryMonitor& b, float volts, float currentMv, float seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 100.0f); ++i) b.update(pinMv(volts), currentMv, 0.01f);
    }

    BatteryConfig liion6s()
    {
        BatteryConfig c;
        c.vbatScale = 110;
        c.ibataScale = 100;   // 10 мВ на ампер (4-в-1 BLS 100A)
        c.currentSensor = true;
        c.cells = 6;
        c.capacityMah = 4000;
        c.minCellVolts = 2.80f;
        c.warningCellVolts = 3.10f;
        c.maxCellVolts = 4.25f;
        return c;
    }
}

void test_battery_voltage_current_and_mah()
{
    BatteryMonitor b;
    b.configure(liion6s());
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::NOT_PRESENT), static_cast<int>(b.state()));
    feed(b, 25.2f, 100.0f, 36.0f);   // 10 А, 36 с — 100 мА·ч
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.2f, b.volts());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.2f, b.cellVolts());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, b.amps());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, b.mahDrawn());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 97.5f, b.percentLeft());
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(b.state()));
    TEST_ASSERT_EQUAL_STRING("OK", batteryStateName(b.state()));

    // смещение датчика: 50 мВ при нуле ампер; ниже нуля — не бывает
    BatteryConfig c = liion6s();
    c.ibataOffsetMv = 50.0f;
    b.configure(c);
    feed(b, 25.2f, 150.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, b.amps());
    feed(b, 25.2f, 0.0f, 2.0f);   // ibat_lpf_period 10 — ФНЧ 1 Гц
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, b.amps());
}

void test_battery_warning_critical_hysteresis_and_sag()
{
    BatteryMonitor b;
    b.configure(liion6s());
    feed(b, 22.0f, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(b.state()));

    // провал под газом на 0.3 с — не тревога: ниже порога нужно 2 с подряд
    // (vbat_duration_for_warning 20)
    feed(b, 15.0f, 0.0f, 0.3f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(b.state()));
    feed(b, 22.0f, 0.0f, 3.0f);

    feed(b, 18.0f, 0.0f, 6.0f);   // 3.00 В на банку
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(b.state()));
    feed(b, 16.2f, 0.0f, 6.0f);   // 2.70
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::CRITICAL), static_cast<int>(b.state()));
    feed(b, 16.79f, 0.0f, 6.0f);  // не выше минимума 16.8 — всё ещё CRITICAL
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::CRITICAL), static_cast<int>(b.state()));
    feed(b, 18.0f, 0.0f, 6.0f);   // 3.00 — выше минимума: WARNING (как в BF)
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(b.state()));
    feed(b, 18.6f, 0.0f, 6.0f);   // 3.10 — ровно порог, не выше
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(b.state()));
    feed(b, 19.5f, 0.0f, 6.0f);   // 3.25
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(b.state()));

    // только USB — батареи нет; подключили полную — сразу OK, без «критично» по дороге
    feed(b, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::NOT_PRESENT), static_cast<int>(b.state()));
    feed(b, 25.0f, 0.0f, 0.05f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(b.state()));

    // без датчика тока — ампер нет, остаток — по напряжению
    BatteryConfig c = liion6s();
    c.currentSensor = false;
    b.configure(c);
    feed(b, 6.0f * 3.525f, 300.0f, 20.0f);
    TEST_ASSERT_FALSE(b.hasCurrent());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, b.amps());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 50.0f, b.percentLeft());
}

// 124–126: настройки батареи BF — делитель, батарея есть или USB, «не
// полная», задержки и гистерезис, тревога по расходу, use_vbat_alerts,
// vbat_cutoff_percent, калибровка по мультиметру.
void test_battery_betaflight_settings()
{
    BatteryMonitor b;
    BatteryConfig c = liion6s();
    c.vbatDivider = 20;   // вдвое больший делитель — вдвое меньше вольт
    b.configure(c);
    feed(b, 25.2f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.6f, b.volts());
    c.vbatDivider = 10;
    c.vbatMultiplier = 2;
    b.configure(c);
    feed(b, 50.4f, 0.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 25.2f, b.volts());

    // battery_meter NONE — напряжения нет
    c = liion6s();
    c.voltageSensor = false;
    b.configure(c);
    feed(b, 25.2f, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::NOT_PRESENT), static_cast<int>(b.state()));

    // 1S: 4.2 В — батарея; 5 В USB через делитель — нет
    BatteryMonitor one;
    c = liion6s();
    c.cells = 1;
    one.configure(c);
    feed(one, 4.2f, 0.0f, 0.5f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(one.state()));
    feed(one, 5.0f, 0.0f, 0.5f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::NOT_PRESENT), static_cast<int>(one.state()));
    TEST_ASSERT_EQUAL_UINT8(1, one.cells());

    // полная ли при подключении (vbat_full_cell_voltage 4.10)
    BatteryMonitor f;
    f.configure(liion6s());
    feed(f, 25.0f, 0.0f, 0.5f);
    TEST_ASSERT_TRUE(f.wasFullWhenPluggedIn());
    feed(f, 0.0f, 0.0f, 0.5f);
    TEST_ASSERT_FALSE(f.wasFullWhenPluggedIn());
    feed(f, 24.0f, 0.0f, 0.5f);
    TEST_ASSERT_FALSE(f.wasFullWhenPluggedIn());

    // без задержек (как у BF) — сразу, порог — минус vbat_hysteresis
    BatteryMonitor d;
    c = liion6s();
    c.warningDelayS = c.criticalDelayS = 0.0f;
    c.hysteresisVolts = 0.3f;
    d.configure(c);
    feed(d, 20.0f, 0.0f, 0.5f);
    feed(d, 18.4f, 0.0f, 10.0f);   // ниже 18.6, но не ниже 18.3
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(d.state()));
    feed(d, 18.2f, 0.0f, 10.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(d.state()));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, d.throttleLimit());   // vbat_cutoff_percent 100 — нет

    // по расходу: 100 мА·ч при 10 А; use_vbat_alerts OFF — молчит
    BatteryMonitor m;
    c = liion6s();
    c.capacityMah = 100.0f;
    c.useCbatAlerts = true;
    c.useVbatAlerts = false;
    m.configure(c);
    feed(m, 25.2f, 100.0f, 31.0f);   // 86 мА·ч — осталось 14 %
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(m.state()));
    feed(m, 25.2f, 100.0f, 2.0f);    // 92 — 8 %
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(m.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(m.stateByConsumption()));
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(m.stateByVoltage()));
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::OK), static_cast<int>(m.alertState()));
    feed(m, 25.2f, 100.0f, 4.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::CRITICAL), static_cast<int>(m.state()));

    // vbat_cutoff_percent 50: с первого CRITICAL газ за 10 с до половины
    BatteryMonitor l;
    c = liion6s();
    c.lvcPercent = 50.0f;
    l.configure(c);
    feed(l, 20.0f, 0.0f, 0.5f);
    feed(l, 16.0f, 0.0f, 3.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::WARNING), static_cast<int>(l.state()));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, l.throttleLimit());
    feed(l, 16.0f, 0.0f, 3.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryState::CRITICAL), static_cast<int>(l.state()));
    feed(l, 16.0f, 0.0f, 5.0f);
    TEST_ASSERT_TRUE(l.throttleLimit() < 0.85f && l.throttleLimit() > 0.6f);
    feed(l, 18.0f, 0.0f, 10.0f);   // поднялось — ограничение остаётся до новой батареи
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, l.throttleLimit());

    // калибровка: делитель на деле 1:10, а стоит 110 — мультиметр 22.9 В
    BatteryMonitor k;
    k.configure(liion6s());
    for (int i = 0; i < 300; ++i) k.update(22.9f * 1000.0f / 10.0f, 50.0f, 0.01f);   // 5 А при 100
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, k.vbatScaleFor(22.9f));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, k.ibataScaleFor(10.0f));   // на деле 10 А — 5 мВ/А
    TEST_ASSERT_EQUAL_FLOAT(0.0f, k.ibataScaleFor(0.0f));
}

// Реестр -> монитор: force_battery_cell_count, bat_capacity, делитель,
// current_meter из BF (VIRTUAL принимается, тока не даёт), задержки.
void test_battery_params_like_betaflight()
{
    CopterParams& p = five5Params();
    char err[96];
    BatteryConfig c = p.batteryConfig();
    TEST_ASSERT_EQUAL_UINT8(p.buildSpec().battery.cells, c.cells);
    TEST_ASSERT_EQUAL_FLOAT(p.value(PARAM_BATTERY_MAH), c.capacityMah);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.10f, c.fullCellVolts);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, c.warningDelayS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, c.detectCellVolts);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, c.displayLpfPeriod);
    TEST_ASSERT_TRUE(c.voltageSensor);
    TEST_ASSERT_TRUE(p.parse(PARAM_FORCE_BATTERY_CELL_COUNT, 0, "6", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_BAT_CAPACITY, 0, "2200", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_VBAT_DIVIDER, 0, "12", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_CURRENT_METER, 0, "VIRTUAL", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_BATTERY_METER, 0, "NONE", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_VBAT_DURATION_FOR_WARNING, 0, "0", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_VBAT_CUTOFF_PERCENT, 0, "60", err, sizeof(err)));
    c = p.batteryConfig();
    TEST_ASSERT_EQUAL_UINT8(6, c.cells);
    TEST_ASSERT_EQUAL_FLOAT(2200.0f, c.capacityMah);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, c.vbatDivider);
    TEST_ASSERT_FALSE(c.currentSensor);
    TEST_ASSERT_FALSE(c.voltageSensor);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.warningDelayS);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, c.lvcPercent);
    // полная банка — по химии
    TEST_ASSERT_TRUE(p.parse(PARAM_BATTERY_CHEMISTRY, 0, "LIHV", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(425.0f, p.value(PARAM_VBAT_FULL_CELL_VOLTAGE));
    p.resetAll();
}

// vbat_cutoff_percent в контроллере: газ со стика ×, в ALT_HOLD — нет.
void test_throttle_limit_scales_stick_throttle()
{
    Bench b;
    TEST_ASSERT_TRUE(b.armAfterBoot());
    b.rc.throttle = 0.6f;
    b.run(0.5f);
    const float full = b.ctl.getStatus().throttle;
    b.ctl.setThrottleLimit(0.5f);
    b.run(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, full * 0.5f, b.ctl.getStatus().throttle);
    b.ctl.setThrottleLimit(2.0f);   // прижимается к 1
    b.run(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, full, b.ctl.getStatus().throttle);
}

void test_battery_chemistry_params()
{
    CopterParams& p = five5Params();
    char err[96], out[48];
    TEST_ASSERT_EQUAL(static_cast<int>(BatteryChemistry::LIPO), static_cast<int>(p.chemistry()));
    TEST_ASSERT_EQUAL_FLOAT(330.0f, p.value(PARAM_VBAT_MIN_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_FLOAT(350.0f, p.value(PARAM_VBAT_WARNING_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_FLOAT(430.0f, p.value(PARAM_VBAT_MAX_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_STRING("power", paramGroupName(paramInfo(PARAM_IBATA_SCALE).group));

    // Li-ion: пороги — его, модель тяги — 3.5 В на банку под нагрузкой
    const float liPoVolts = p.modelTune().volts;
    TEST_ASSERT_TRUE(p.parse(PARAM_BATTERY_CHEMISTRY, 0, "liion", err, sizeof(err)));
    TEST_ASSERT_EQUAL_FLOAT(280.0f, p.value(PARAM_VBAT_MIN_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_FLOAT(310.0f, p.value(PARAM_VBAT_WARNING_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_FLOAT(425.0f, p.value(PARAM_VBAT_MAX_CELL_VOLTAGE));
    TEST_ASSERT_EQUAL_FLOAT(3.5f, p.buildSpec().battery.voltsPerCellLoaded);
    TEST_ASSERT_TRUE(p.updateModel());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.0f, p.modelTune().volts);
    TEST_ASSERT_TRUE(p.modelTune().volts < liPoVolts);
    TEST_ASSERT_EQUAL_UINT8(6, p.modelTune().cells);

    // заданный руками порог остаётся при смене химии
    TEST_ASSERT_TRUE(p.parse(PARAM_VBAT_WARNING_CELL_VOLTAGE, 0, "320", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_IBATA_SCALE, 0, "100", err, sizeof(err)));
    p.setTableDefault(PARAM_CURRENT_METER, 1);
    const BatteryConfig c = p.batteryConfig();
    TEST_ASSERT_EQUAL_FLOAT(100.0f, c.ibataScale);
    TEST_ASSERT_TRUE(c.currentSensor);
    TEST_ASSERT_EQUAL_UINT8(6, c.cells);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.80f, c.minCellVolts);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.20f, c.warningCellVolts);
    CopterSettings s;
    p.applyTo(s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.20f, s.minCellVolts);   // ниже без нагрузки — не армить
    p.format(PARAM_BATTERY_CHEMISTRY, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("LIION", out);
    TEST_ASSERT_EQUAL_STRING("Li-ion", chemistryName(p.chemistry()));
    p.setTableDefault(PARAM_CURRENT_METER, 0);
}

// ---- видеопередатчик (copter/Vtx.h) ----

namespace
{
    // Провод управления с передатчиком-эмулятором (helpers/VtxEmulator.h).
    struct FakeVtx : public VtxPort
    {
        VtxEmulator dev;
        bool echo = false;   // линия возвращает наши же байты (полудуплекс без отключения приёма)
        uint32_t baud = 0;
        bool twoStop = false;
        std::vector<uint8_t> toFc;
        size_t readPos = 0;

        void open(uint32_t b, bool two) override
        {
            baud = b;
            twoStop = two;
        }

        void send(const uint8_t* data, uint8_t n) override
        {
            if (echo) toFc.insert(toFc.end(), data, data + n);
            const std::vector<uint8_t> r = dev.receive(data, n, baud, twoStop);
            toFc.insert(toFc.end(), r.begin(), r.end());
        }

        int read() override { return readPos < toFc.size() ? toFc[readPos++] : -1; }
    };

    void spin(VtxController& c, uint32_t& now, uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 10)
        {
            now += 10;
            c.update(now);
        }
    }
}

void test_vtx_band_table_is_this_transmitter()
{
    TEST_ASSERT_EQUAL_UINT16(5865, vtxFrequency(1, 1));   // A1
    TEST_ASSERT_EQUAL_UINT16(5945, vtxFrequency(3, 8));   // E8
    TEST_ASSERT_EQUAL_UINT16(5917, vtxFrequency(5, 8));   // R8
    TEST_ASSERT_EQUAL_UINT16(5362, vtxFrequency(6, 1));   // L1 (у продавца D)
    TEST_ASSERT_EQUAL_UINT16(4990, vtxFrequency(7, 1));   // X1
    TEST_ASSERT_EQUAL_UINT16(6100, vtxFrequency(9, 8));   // G8
    TEST_ASSERT_EQUAL_UINT16(5613, vtxFrequency(10, 8));  // S8
    TEST_ASSERT_EQUAL_UINT16(0, vtxFrequency(11, 1));
    TEST_ASSERT_EQUAL_UINT16(0, vtxFrequency(1, 9));
    uint8_t band = 0, ch = 0;
    TEST_ASSERT_TRUE(vtxBandChannel(5658, band, ch));
    TEST_ASSERT_EQUAL_UINT8(5, band);
    TEST_ASSERT_EQUAL_UINT8(1, ch);
    TEST_ASSERT_FALSE(vtxBandChannel(5999, band, ch));
    TEST_ASSERT_TRUE(vtxBandCommonGoggles(6));    // L — Cobra X принимает
    TEST_ASSERT_FALSE(vtxBandCommonGoggles(7));   // X — нет
    TEST_ASSERT_EQUAL_UINT16(4000, dbmToMw(36));
    TEST_ASSERT_EQUAL_UINT16(25, dbmToMw(14));
}

void test_vtx_smartaudio_21_power_cap_and_pit()
{
    FakeVtx vtx;
    VtxController c(vtx);
    c.setProtocol(VtxProtocol::AUTO);
    uint32_t now = 0;
    spin(c, now, 300);
    TEST_ASSERT_TRUE(c.state().online);
    TEST_ASSERT_EQUAL(static_cast<int>(VtxProtocol::SMARTAUDIO), static_cast<int>(c.state().protocol));
    TEST_ASSERT_EQUAL_UINT8(21, c.state().version);
    TEST_ASSERT_EQUAL_UINT8(6, c.levelCount());            // таблица — от самого VTX
    TEST_ASSERT_EQUAL_UINT16(4000, c.levelMw(6));
    TEST_ASSERT_EQUAL_UINT16(400, c.levelMw(3));

    // R1 и самый сильный уровень, но потолок «в городе» 400 мВт
    VtxRequest r;
    r.band = 5;
    r.channel = 1;
    r.frequencyMhz = vtxFrequency(5, 1);
    r.powerLevel = 6;
    c.setMaxPowerMw(400);
    c.request(r);
    spin(c, now, 2000);
    TEST_ASSERT_EQUAL_UINT16(5658, vtx.dev.freq);
    TEST_ASSERT_FALSE(vtx.dev.userFreq);                       // A–R — номером канала
    TEST_ASSERT_EQUAL_UINT8(26, vtx.dev.dbm);                  // 400 мВт, не 4 Вт
    TEST_ASSERT_EQUAL_UINT16(400, c.state().powerMw);
    TEST_ASSERT_EQUAL_UINT8(3, c.effectiveLevel());

    // кнопкой на передатчике подняли до 4 Вт — прошивка опускает обратно
    vtx.dev.dbm = 36;
    spin(c, now, 2500);
    TEST_ASSERT_EQUAL_UINT8(26, vtx.dev.dbm);

    // за городом — без потолка
    c.setMaxPowerMw(0);
    spin(c, now, 2000);
    TEST_ASSERT_EQUAL_UINT8(36, vtx.dev.dbm);

    // PIT и обратно
    r.pit = true;
    c.request(r);
    spin(c, now, 2000);
    TEST_ASSERT_TRUE(vtx.dev.pit);
    TEST_ASSERT_TRUE(c.state().pit);
    r.pit = false;
    c.request(r);
    spin(c, now, 2000);
    TEST_ASSERT_FALSE(vtx.dev.pit);

    // X-полоса (4.9 ГГц) — номера канала у SmartAudio для неё нет: частотой
    r.band = 7;
    r.channel = 3;
    r.frequencyMhz = vtxFrequency(7, 3);
    c.request(r);
    spin(c, now, 2000);
    TEST_ASSERT_EQUAL_UINT16(5050, vtx.dev.freq);
    TEST_ASSERT_TRUE(vtx.dev.userFreq);
    TEST_ASSERT_EQUAL_UINT16(5050, c.state().frequencyMhz);

    // VTX CONTROL DISABLE — ничего не шлём
    const int framesBefore = vtx.dev.frames;
    r.control = false;
    r.powerLevel = 1;
    c.request(r);
    spin(c, now, 3000);
    TEST_ASSERT_EQUAL(framesBefore, vtx.dev.frames);
    TEST_ASSERT_EQUAL_UINT8(36, vtx.dev.dbm);
}

void test_vtx_smartaudio_v1_and_echo_on_the_wire()
{
    FakeVtx vtx;
    vtx.dev.saVersion = 1;
    vtx.echo = true;
    VtxController c(vtx);
    c.setProtocol(VtxProtocol::SMARTAUDIO);
    uint32_t now = 0;
    spin(c, now, 300);
    TEST_ASSERT_TRUE(c.state().online);
    TEST_ASSERT_EQUAL_UINT8(1, c.state().version);
    TEST_ASSERT_EQUAL_UINT8(4, c.levelCount());   // 25, 200, 500, 800
    VtxRequest r;
    r.band = 1;
    r.channel = 4;
    r.frequencyMhz = vtxFrequency(1, 4);
    r.powerLevel = 3;
    c.request(r);
    spin(c, now, 3000);
    TEST_ASSERT_EQUAL_UINT16(5805, vtx.dev.freq);
    TEST_ASSERT_EQUAL_UINT8(2, vtx.dev.powerIndex);   // 500 мВт — значение 25 у v1
    TEST_ASSERT_EQUAL_UINT16(500, c.state().powerMw);
}

void test_vtx_auto_finds_tramp_and_offline_without_vtx()
{
    FakeVtx vtx;
    vtx.dev.kind = VtxEmulator::TRAMP;
    vtx.dev.trampMax = 1600;
    VtxController c(vtx);
    c.setProtocol(VtxProtocol::AUTO);
    uint32_t now = 0;
    spin(c, now, 300);
    TEST_ASSERT_FALSE(c.state().online);   // сначала пробует SmartAudio
    spin(c, now, 1500);
    TEST_ASSERT_TRUE(c.state().online);
    TEST_ASSERT_EQUAL(static_cast<int>(VtxProtocol::TRAMP), static_cast<int>(c.state().protocol));
    TEST_ASSERT_EQUAL_UINT8(8, c.levelCount());   // 25 … 1600: не выше, чем умеет
    TEST_ASSERT_EQUAL_UINT16(1600, c.levelMw(8));
    VtxRequest r;
    r.band = 4;
    r.channel = 2;
    r.frequencyMhz = vtxFrequency(4, 2);
    r.powerLevel = 8;
    c.setMaxPowerMw(600);
    c.request(r);
    spin(c, now, 4000);
    TEST_ASSERT_EQUAL_UINT16(5760, vtx.dev.freq);
    TEST_ASSERT_EQUAL_UINT16(600, vtx.dev.trampMw);
    TEST_ASSERT_EQUAL_UINT16(600, c.state().powerMw);
    r.pit = true;
    c.request(r);
    spin(c, now, 3000);
    TEST_ASSERT_TRUE(vtx.dev.pit);

    // передатчика нет — не в сети, но и не виснет, пробует дальше
    FakeVtx none;
    none.dev.kind = VtxEmulator::ABSENT;
    VtxController d(none);
    d.setProtocol(VtxProtocol::AUTO);
    spin(d, now, 5000);
    TEST_ASSERT_FALSE(d.state().online);
    TEST_ASSERT_EQUAL_UINT8(0, d.levelCount());
    // NONE — порт не трогается
    FakeVtx untouched;
    VtxController e(untouched);
    e.setProtocol(VtxProtocol::NONE);
    spin(e, now, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, untouched.baud);
}

void test_vtx_ranges_from_switches()
{
    RcConfig c;
    char err[96];
    // мощность с трёхпозиционного AUX3: вверх 2, вниз 6; полоса R — с AUX4
    TEST_ASSERT_TRUE(c.parseVtx("0 2 0 0 2 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_TRUE(c.parseVtx("1 2 0 0 6 1700 2100", 10, 12, err, sizeof(err)));
    TEST_ASSERT_TRUE(c.parseVtx("2 3 5 3 0 1500 2100", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("10 2 0 0 2 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("0 9 0 0 2 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("0 2 11 0 2 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("0 2 0 9 2 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("0 2 0 0 13 900 1300", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.parseVtx("0 2 0 0 2 900", 10, 12, err, sizeof(err)));
    TEST_ASSERT_FALSE(c.isDefault());
    char row[48];
    RcConfig::formatVtx(1, c.vtx(1), row, sizeof(row));
    TEST_ASSERT_EQUAL_STRING("vtx 1 2 0 0 6 1700 2100", row);

    RxInput in;
    in.configure(c, 885, 2115);
    uint16_t ch[10] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000 };
    in.read(ch, 10, true);
    VtxSelection sel = in.vtxSelection();
    TEST_ASSERT_EQUAL_UINT8(2, sel.power);
    TEST_ASSERT_EQUAL_UINT8(0, sel.band);
    ch[6] = 2000;   // AUX3 вниз
    ch[7] = 1800;   // AUX4 — полоса R, канал 3
    in.read(ch, 10, true);
    sel = in.vtxSelection();
    TEST_ASSERT_EQUAL_UINT8(6, sel.power);
    TEST_ASSERT_EQUAL_UINT8(5, sel.band);
    TEST_ASSERT_EQUAL_UINT8(3, sel.channel);
    ch[6] = 1500;   // середина — ни одна строка мощности
    in.read(ch, 10, true);
    TEST_ASSERT_EQUAL_UINT8(0, in.vtxSelection().power);

    // во флеш и обратно; старая запись (без строк vtx) читается
    uint8_t blob[RcConfig::STORE_MAX];
    const size_t n = c.serialize(blob, sizeof(blob));
    RcConfig d;
    TEST_ASSERT_TRUE(d.deserialize(blob, n));
    TEST_ASSERT_EQUAL_UINT8(6, d.vtx(1).power);
    TEST_ASSERT_EQUAL_UINT16(1500, d.vtx(2).startUs);
    TEST_ASSERT_TRUE(d.deserialize(blob, n - (1 + 3 * 7)));   // обрезано до таблицы aux
    TEST_ASSERT_FALSE(d.anyVtx());

    // USER1–USER4, VTX PIT MODE и VTX CONTROL DISABLE — действуют
    TEST_ASSERT_TRUE(findBox(BOX_USER1)->works);
    TEST_ASSERT_TRUE(findBox(BOX_USER4)->works);
    TEST_ASSERT_TRUE(findBox(BOX_VTX_PIT_MODE)->works);
    TEST_ASSERT_TRUE(findBox(BOX_VTX_CONTROL_DISABLE)->works);
}

void test_cli_vtx_and_params()
{
    CopterParams& p = five5Params();
    TEST_ASSERT_EQUAL_FLOAT(5.0f, p.value(PARAM_VTX_BAND));   // R1, самый слабый, без потолка
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.value(PARAM_VTX_CHANNEL));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p.value(PARAM_VTX_POWER));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.value(PARAM_VTX_MAX_POWER_MW));
    TEST_ASSERT_EQUAL_STRING("vtx", paramGroupName(paramInfo(PARAM_VTX_POWER).group));
    Bench b;
    b.ctl.configure(p.tune());
    CliCapture port;
    CopterCli cli(p, b.ctl, port);
    TEST_ASSERT_TRUE(cli.enter());
    port.take();
    cli.execute("vtx");
    TEST_ASSERT_TRUE(has(port.take(), "# строк vtx нет"));
    cli.execute("vtx 0 2 0 0 6 1700 2100");
    TEST_ASSERT_TRUE(has(port.take(), "vtx 0 2 0 0 6 1700 2100\r\n"));
    cli.execute("vtx 0 2 0 0 99 1700 2100");
    TEST_ASSERT_TRUE(has(port.take(), "###ОШИБКА: полоса 0-10, канал 0-8, мощность 0-12"));
    cli.execute("set vtx_max_power_mw = 400");
    port.take();
    cli.execute("diff");
    const std::string out = port.take();
    TEST_ASSERT_TRUE_MESSAGE(has(out, "\r\n# vtx\r\nvtx 0 2 0 0 6 1700 2100\r\n\r\n# master\r\nset vtx_max_power_mw = 400\r\n"),
                             out.c_str());
    char err[96];
    TEST_ASSERT_TRUE(p.parse(PARAM_VTX_PROTOCOL, 0, "smartaudio", err, sizeof(err)));
    TEST_ASSERT_FALSE(p.parse(PARAM_VTX_PROTOCOL, 0, "msp", err, sizeof(err)));
    TEST_ASSERT_TRUE(p.parse(PARAM_VTX_LOW_POWER_DISARM, 0, "until_first_arm", err, sizeof(err)));
    cli.execute("defaults nosave");
    TEST_ASSERT_FALSE(p.rc().anyVtx());
    cli.execute("exit");
}

// ---- дом по GPS и телеметрия CRSF ----

namespace
{
    GpsFix fixAt(double lat, double lon, uint8_t sats = 12)
    {
        GpsFix f;
        f.valid = true;
        f.satellites = sats;
        f.latitude = lat;
        f.longitude = lon;
        f.altitudeM = 150.0f;
        return f;
    }
}

void test_gps_home_distance_and_direction()
{
    GpsHome h;
    h.setMinimumSats(8);
    const double lat0 = 55.75, lon0 = 37.61;
    h.update(fixAt(lat0, lon0), false);
    TEST_ASSERT_FALSE(h.hasHome());   // дом — при ARM
    h.update(fixAt(lat0, lon0, 6), true);
    TEST_ASSERT_FALSE(h.hasHome());   // спутников мало
    h.update(fixAt(lat0, lon0), true);
    TEST_ASSERT_TRUE(h.hasHome());
    TEST_ASSERT_TRUE(h.homeSetInFlight());   // фиксы были и до него — поставлен уже заармленным

    // 100 м на север: дом — на юге (180°)
    const double dLat = 100.0 / 6371000.0 * 180.0 / 3.14159265358979;
    h.update(fixAt(lat0 + dLat, lon0), true);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, h.distanceM());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, h.bearingHomeDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, fabsf(h.relativeHomeDeg(0.0f)));   // нос на север — дом за спиной
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, h.relativeHomeDeg(180.0f));          // нос на юг — дом прямо

    // 200 м на восток от дома: дом — на западе (270°)
    const double dLon = 200.0 / (6371000.0 * cos(lat0 * 3.14159265358979 / 180.0)) * 180.0 / 3.14159265358979;
    h.update(fixAt(lat0, lon0 + dLon), true);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 200.0f, h.distanceM());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 270.0f, h.bearingHomeDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -90.0f, h.relativeHomeDeg(0.0f));   // налево

    // DISARM дом не стирает; новый ARM — новый дом, сразу (фикс хороший)
    h.update(fixAt(lat0, lon0 + dLon), false);
    TEST_ASSERT_TRUE(h.hasHome());
    h.update(fixAt(lat0, lon0 + dLon), true);
    TEST_ASSERT_TRUE(h.hasHome());
    TEST_ASSERT_FALSE(h.homeSetInFlight());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, h.distanceM());
}

void test_crsf_telemetry_frames()
{
    CrsfTelemetryData d;
    d.volts = 25.2f;
    d.amps = 12.3f;
    d.mahDrawn = 1234;
    d.percentLeft = 69;
    uint8_t f[CrsfTelemetry::MAX_FRAME];

    // батарея: C8 0A 08 | 00 FC | 00 7B | 00 04 D2 | 45 | CRC
    uint8_t n = CrsfTelemetry::build(0, d, f);
    TEST_ASSERT_EQUAL_UINT8(12, n);
    const uint8_t battery[] = { 0xC8, 0x0A, 0x08, 0x00, 0xFC, 0x00, 0x7B, 0x00, 0x04, 0xD2, 0x45 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(battery, f, sizeof(battery));
    TEST_ASSERT_EQUAL_HEX8(crsfCrc(f + 2, 9), f[11]);

    // ориентация: тангаж 10° = 0.1745 рад -> 1745; крен −20° -> −3491; курс 270° -> −90° -> −15708
    d.pitchDeg = 10;
    d.rollDeg = -20;
    d.yawDeg = 270;
    n = CrsfTelemetry::build(1, d, f);
    TEST_ASSERT_EQUAL_UINT8(10, n);
    TEST_ASSERT_EQUAL_HEX8(0x1E, f[2]);
    TEST_ASSERT_EQUAL_INT16(1745, static_cast<int16_t>((f[3] << 8) | f[4]));
    TEST_ASSERT_EQUAL_INT16(-3491, static_cast<int16_t>((f[5] << 8) | f[6]));
    TEST_ASSERT_EQUAL_INT16(-15708, static_cast<int16_t>((f[7] << 8) | f[8]));

    // режим: строка с нулём; без ARM — «*», failsafe — «!FS!»
    CrsfTelemetry::modeName(d.mode, sizeof(d.mode), "ANGL", false, false);
    TEST_ASSERT_EQUAL_STRING("ANGL*", d.mode);
    n = CrsfTelemetry::build(2, d, f);
    TEST_ASSERT_EQUAL_UINT8(4 + 6, n);
    TEST_ASSERT_EQUAL_HEX8(0x21, f[2]);
    TEST_ASSERT_EQUAL_STRING("ANGL*", reinterpret_cast<const char*>(f + 3));
    CrsfTelemetry::modeName(d.mode, sizeof(d.mode), "ACRO", true, true);
    TEST_ASSERT_EQUAL_STRING("!FS!", d.mode);

    // вариометр: см/с
    d.climbMs = -1.5f;
    n = CrsfTelemetry::build(3, d, f);
    TEST_ASSERT_EQUAL_HEX8(0x07, f[2]);
    TEST_ASSERT_EQUAL_INT16(-150, static_cast<int16_t>((f[3] << 8) | f[4]));

    // GPS — только с фиксом
    TEST_ASSERT_EQUAL_UINT8(0, CrsfTelemetry::build(4, d, f));
    d.gps = true;
    d.latitude = 55.75;
    d.longitude = 37.61;
    d.speedMs = 10.0f;      // 36 км/ч -> 360
    d.courseDeg = 90.0f;    // 9000
    d.altitudeM = 150.0f;   // 1150
    d.satellites = 12;
    n = CrsfTelemetry::build(4, d, f);
    TEST_ASSERT_EQUAL_UINT8(19, n);
    TEST_ASSERT_EQUAL_HEX8(0x02, f[2]);
    TEST_ASSERT_EQUAL_INT32(557500000, static_cast<int32_t>((f[3] << 24) | (f[4] << 16) | (f[5] << 8) | f[6]));
    TEST_ASSERT_EQUAL_UINT16(360, static_cast<uint16_t>((f[11] << 8) | f[12]));
    TEST_ASSERT_EQUAL_UINT16(9000, static_cast<uint16_t>((f[13] << 8) | f[14]));
    TEST_ASSERT_EQUAL_UINT16(1150, static_cast<uint16_t>((f[15] << 8) | f[16]));
    TEST_ASSERT_EQUAL_UINT8(12, f[17]);
    TEST_ASSERT_EQUAL_HEX8(crsfCrc(f + 2, 16), f[18]);

    // по кругу: без GPS — четыре вида, пятый пропускается
    CrsfTelemetry t;
    d.gps = false;
    uint8_t types[8];
    for (int i = 0; i < 8; ++i)
    {
        TEST_ASSERT_TRUE(t.next(d, f) > 0);
        types[i] = f[2];
    }
    const uint8_t expected[8] = { 0x08, 0x1E, 0x21, 0x07, 0x08, 0x1E, 0x21, 0x07 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, types, 8);
}

// ---- OSD (copter/Osd.h, hal/Max7456.h) ----

void test_osd_arrows_and_warnings()
{
    TEST_ASSERT_EQUAL_HEX8(0x68, osdArrow(0.0f));     // вперёд — «север» шрифта
    TEST_ASSERT_EQUAL_HEX8(0x64, osdArrow(90.0f));    // направо
    TEST_ASSERT_EQUAL_HEX8(0x60, osdArrow(180.0f));   // назад
    TEST_ASSERT_EQUAL_HEX8(0x60, osdArrow(-180.0f));
    TEST_ASSERT_EQUAL_HEX8(0x6C, osdArrow(-90.0f));   // налево
    TEST_ASSERT_EQUAL_HEX8(0x67, osdArrow(20.0f));    // чуть правее — соседняя

    OsdData d;
    d.armed = true;
    TEST_ASSERT_NULL(OsdScreen::warningText(d));
    d.batteryState = 2;
    TEST_ASSERT_EQUAL_STRING("LOW BATTERY", OsdScreen::warningText(d));
    d.batteryState = 3;
    TEST_ASSERT_EQUAL_STRING("BATT CRITICAL", OsdScreen::warningText(d));
    d.linkLost = true;
    TEST_ASSERT_EQUAL_STRING("RXLOSS", OsdScreen::warningText(d));
    d.failsafe = true;
    TEST_ASSERT_EQUAL_STRING("FAILSAFE", OsdScreen::warningText(d));
    OsdData ground;
    ground.armed = false;
    TEST_ASSERT_EQUAL_STRING("DISARMED", OsdScreen::warningText(ground));
    ground.armBlock = "THROTTLE";
    TEST_ASSERT_EQUAL_STRING("THROTTLE", OsdScreen::warningText(ground));
}

void test_osd_screen_layout()
{
    OsdScreen s;
    s.setRows(16);
    OsdData d;
    d.armed = true;
    d.mode = "ANGL";
    d.linkQuality = 100;
    d.volts = 25.2f;
    d.cellVolts = 4.2f;
    d.hasCurrent = true;
    d.amps = 12.3f;
    d.mahDrawn = 1234;
    d.flightSeconds = 133;
    d.altitudeM = 12.4f;
    d.gpsPresent = true;
    d.satellites = 12;
    d.home = true;
    d.homeDistanceM = 1260;
    d.homeRelativeDeg = 90;
    d.vtxMw = 400;
    s.compose(d);
    auto row = [&s](uint8_t r) {
        std::string t;
        for (uint8_t c = 0; c < OsdScreen::COLS; ++c) t += static_cast<char>(s.at(r, c));
        return t;
    };
    TEST_ASSERT_EQUAL_HEX8(osdsym::LINK_QUALITY, s.at(0, 1));
    TEST_ASSERT_TRUE(has(row(0), "100"));
    TEST_ASSERT_TRUE(has(row(0), "ANGL"));
    TEST_ASSERT_EQUAL_HEX8(osdsym::SAT_L, s.at(0, 24));
    TEST_ASSERT_TRUE(has(row(0), "12"));
    TEST_ASSERT_TRUE(has(row(1), "400MW"));
    TEST_ASSERT_EQUAL_HEX8(osdsym::CROSSHAIR, s.at(8, 14));
    TEST_ASSERT_EQUAL_HEX8(osdArrow(90), s.at(13, 1));
    TEST_ASSERT_TRUE(has(row(13), "1.3KM"));
    TEST_ASSERT_TRUE(has(row(13), "12"));
    TEST_ASSERT_TRUE_MESSAGE(has(row(14), "25.2"), row(14).c_str());
    TEST_ASSERT_TRUE(has(row(14), "4.20"));
    TEST_ASSERT_TRUE(has(row(14), "02:13"));
    TEST_ASSERT_TRUE(has(row(15), "12.3"));
    TEST_ASSERT_TRUE(has(row(15), "1234"));
    // текст — заглавными (в шрифте только они; 0x60–0x6F — это стрелки)
    OsdScreen t;
    t.print(0, 0, "Angl");
    TEST_ASSERT_EQUAL_HEX8('A', t.at(0, 0));
    TEST_ASSERT_EQUAL_HEX8('N', t.at(0, 1));

    // NTSC — 13 строк: низ экрана выше
    s.setRows(13);
    d.batteryState = 2;
    s.compose(d);
    TEST_ASSERT_TRUE(has(row(11), "25.2"));
    TEST_ASSERT_TRUE(has(row(8), "LOW BATTERY"));
}

void test_osd_chip_driver()
{
    Osd7456Emulator chip;
    Max7456 osd(chip, 10);
    TEST_ASSERT_TRUE(osd.begin());
    TEST_ASSERT_TRUE(osd.isPal());
    TEST_ASSERT_EQUAL_UINT8(16, osd.rows());
    TEST_ASSERT_EQUAL_HEX8(0x48, chip.vm0);         // PAL + OSD
    TEST_ASSERT_EQUAL_HEX8(0x0F, chip.osdbl);       // уровень чёрного — автоматически
    TEST_ASSERT_EQUAL(1, chip.resets);

    OsdScreen s;
    s.setRows(osd.rows());
    s.print(0, 3, "ANGL");
    uint16_t total = 0;
    for (int i = 0; i < 20; ++i) total = static_cast<uint16_t>(total + osd.flush(s, 40));
    TEST_ASSERT_EQUAL_UINT16(480, total);           // первый раз — весь экран (пробелы тоже)
    TEST_ASSERT_EQUAL_STRING("   ANGL                       ", chip.row(0).c_str());
    TEST_ASSERT_EQUAL_UINT16(0, osd.flush(s, 40));  // ничего не поменялось — ничего не пишем
    s.print(0, 3, "ACRO");
    const int before = chip.charWrites;
    for (int i = 0; i < 20; ++i) osd.flush(s, 40);
    TEST_ASSERT_EQUAL(3, chip.charWrites - before); // A остаётся, CRO — три символа
    TEST_ASSERT_EQUAL_STRING("   ACRO                       ", chip.row(0).c_str());

    // камера NTSC — заметит и перенастроит
    chip.palSignal = false;
    osd.check();
    TEST_ASSERT_FALSE(osd.isPal());
    TEST_ASSERT_EQUAL_UINT8(13, osd.rows());
    TEST_ASSERT_EQUAL_HEX8(0x08, chip.vm0);

    // чипа нет
    Osd7456Emulator none;
    none.connected = false;
    Max7456 absent(none, 10);
    TEST_ASSERT_FALSE(absent.begin());
    TEST_ASSERT_EQUAL_UINT16(0, absent.flush(s, 40));
    Max7456 noCs(chip, -1);
    TEST_ASSERT_FALSE(noCs.begin());
}

// ---- пищалка (copter/Beeper.h) ----

namespace
{
    // Сколько тиков по 10 мс пищит из n (и какой рисунок, строкой «#.»).
    std::string listen(Beeper& b, const BeeperInputs& in, int ticks)
    {
        std::string s;
        for (int i = 0; i < ticks; ++i) s += b.tick(in) ? '#' : '.';
        return s;
    }
}

void test_beeper_events_and_alarms()
{
    Beeper b;
    BeeperInputs quiet;
    TEST_ASSERT_EQUAL_STRING("....", listen(b, quiet, 4).c_str());

    b.event(BeeperEvent::ARMED);   // один длинный — 0.5 с
    const std::string arm = listen(b, quiet, 60);
    TEST_ASSERT_EQUAL(50, static_cast<int>(std::count(arm.begin(), arm.end(), '#')));
    TEST_ASSERT_FALSE(b.busy());

    b.event(BeeperEvent::DISARMED);   // два коротких
    TEST_ASSERT_EQUAL_STRING("############..........############......", listen(b, quiet, 40).c_str());

    // батарея: WARNING — короткий раз в 2 с, CRITICAL — часто
    BeeperInputs low;
    low.batteryWarning = true;
    const std::string warn = listen(b, low, 400);
    TEST_ASSERT_EQUAL(20, static_cast<int>(std::count(warn.begin(), warn.end(), '#')));
    low.batteryCritical = true;
    const std::string crit = listen(b, low, 200);
    TEST_ASSERT_TRUE(std::count(crit.begin(), crit.end(), '#') >= 90);
    // всё хорошо — замолчал
    TEST_ASSERT_EQUAL_STRING("..........", listen(b, quiet, 10).c_str());

    // «поиск модели» главнее всего и не глушится; BEEPER MUTE глушит остальное
    BeeperInputs findMe;
    findMe.findMe = true;
    findMe.muted = true;
    findMe.batteryCritical = true;
    const std::string loud = listen(b, findMe, 100);
    TEST_ASSERT_TRUE(std::count(loud.begin(), loud.end(), '#') >= 40);
    BeeperInputs muted;
    muted.muted = true;
    muted.batteryCritical = true;
    TEST_ASSERT_EQUAL_STRING("..........", listen(b, muted, 10).c_str());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_frames_have_expected_motor_counts);
    RUN_TEST(test_frames_are_balanced_and_normalized);
    RUN_TEST(test_quad_x_follows_betaflight_order);
    RUN_TEST(test_props_out_reverses_every_motor);
    RUN_TEST(test_coaxial_pairs_share_an_arm_and_counter_rotate);
    RUN_TEST(test_describe_motor_in_words);
    RUN_TEST(test_mixer_passes_small_commands_unchanged);
    RUN_TEST(test_mixer_airmode_keeps_authority_at_zero_throttle);
    RUN_TEST(test_mixer_cuts_yaw_before_roll_and_pitch);
    RUN_TEST(test_mixer_scales_roll_and_pitch_together);
    RUN_TEST(test_thrust_linearization_and_idle);
    RUN_TEST(test_pt1_reaches_63_percent_in_one_time_constant);
    RUN_TEST(test_pt2_is_minus_3db_at_cutoff);
    RUN_TEST(test_notch_removes_center_and_keeps_low_frequencies);
    RUN_TEST(test_biquad_lowpass);
    RUN_TEST(test_analog_esc_pulses);
    RUN_TEST(test_dshot_frame_and_crc);
    RUN_TEST(test_dshot_bit_timing);
    RUN_TEST(test_propulsion_matches_bench_data_for_5_inch);
    RUN_TEST(test_every_catalog_build_gets_a_sane_tune);
    RUN_TEST(test_whoop_warns_about_vibration_alias_at_1khz);
    RUN_TEST(test_bad_builds_get_warnings);
    RUN_TEST(test_slower_esc_protocol_narrows_the_loop);
    RUN_TEST(test_rate_curve);
    RUN_TEST(test_rate_axis_p_and_i);
    RUN_TEST(test_rate_axis_integral_holds_on_ground_and_in_saturation);
    RUN_TEST(test_rate_axis_relaxes_integral_on_large_error);
    RUN_TEST(test_rate_axis_d_max_rises_on_fast_motion);
    RUN_TEST(test_iterm_relax_axes_types_and_inc);
    RUN_TEST(test_iterm_rotation_turns_with_the_frame);
    RUN_TEST(test_iterm_windup_scales_integral);
    RUN_TEST(test_pid_extras_from_params);
    RUN_TEST(test_rate_axis_d_on_measurement_and_feedforward);
    RUN_TEST(test_attitude_starts_from_accelerometer);
    RUN_TEST(test_attitude_integrates_full_flip_and_yaw);
    RUN_TEST(test_attitude_accelerometer_pulls_slowly_and_only_near_1g);
    RUN_TEST(test_vertical_estimator_fuses_baro_and_accel);
    RUN_TEST(test_vertical_estimator_without_baro_learns_bias_on_ground);
    RUN_TEST(test_altitude_controller);
    RUN_TEST(test_notch_centers_follow_rotation_and_blades);
    RUN_TEST(test_gyro_filter_removes_motor_vibration);
    RUN_TEST(test_arm_requires_settling_then_arms_with_switch);
    RUN_TEST(test_arm_blockers);
    RUN_TEST(test_motor_test_only_disarmed_and_times_out);
    RUN_TEST(test_link_loss_on_ground_disarms_and_needs_switch_toggle);
    RUN_TEST(test_arm_block_bits_are_betaflight_flags);
    RUN_TEST(test_small_angle_setting);
    RUN_TEST(test_auto_disarm_on_the_ground);
    RUN_TEST(test_arm_after_failsafe_waits_for_recovery_delay);
    RUN_TEST(test_safety_params_reach_the_controller);
    RUN_TEST(test_mode_names_and_settings);
    RUN_TEST(test_params_table_is_consistent);
    RUN_TEST(test_params_set_get_bounds_and_enums);
    RUN_TEST(test_params_survive_flash_and_skip_unknown);
    RUN_TEST(test_pid_in_betaflight_units);
    RUN_TEST(test_betaflight_source_takes_bf_45_defaults);
    RUN_TEST(test_profiles_are_independent_and_switch_on_the_fly);
    RUN_TEST(test_pidsum_limit_per_axis);
    RUN_TEST(test_anti_gravity_boosts_i_and_p_on_throttle_punch);
    RUN_TEST(test_throttle_boost_adds_the_fast_part_of_the_stick);
    RUN_TEST(test_tpa_modes_like_betaflight);
    RUN_TEST(test_tpa_low_until_takeoff_or_always);
    RUN_TEST(test_feedforward_transition_softens_center);
    RUN_TEST(test_feedforward_averaging_smooths_frame_steps);
    RUN_TEST(test_feedforward_smooth_factor);
    RUN_TEST(test_feedforward_jitter_factor);
    RUN_TEST(test_feedforward_boost);
    RUN_TEST(test_feedforward_touches_only_the_stick_part);
    RUN_TEST(test_feedforward_params);
    RUN_TEST(test_feedforward_max_rate_limit);
    RUN_TEST(test_yaw_lowpass_hz);
    RUN_TEST(test_integrated_yaw);
    RUN_TEST(test_absolute_control);
    RUN_TEST(test_thrust_linear_and_motor_output_limit);
    RUN_TEST(test_cli_set_get_diff_save_and_paste);
    RUN_TEST(test_rc_map_and_rxrange);
    RUN_TEST(test_rx_input_modes_like_betaflight);
    RUN_TEST(test_rx_input_logic_linked_map_and_rxrange);
    RUN_TEST(test_rc_tables_survive_flash);
    RUN_TEST(test_copter_description_is_params);
    RUN_TEST(test_cli_aux_map_rxrange_and_diff);
    RUN_TEST(test_rxfail_table_like_betaflight);
    RUN_TEST(test_rx_input_rxfail_values);
    RUN_TEST(test_cli_rxfail_and_diff);
    RUN_TEST(test_battery_voltage_current_and_mah);
    RUN_TEST(test_battery_warning_critical_hysteresis_and_sag);
    RUN_TEST(test_battery_chemistry_params);
    RUN_TEST(test_battery_betaflight_settings);
    RUN_TEST(test_battery_params_like_betaflight);
    RUN_TEST(test_throttle_limit_scales_stick_throttle);
    RUN_TEST(test_vtx_band_table_is_this_transmitter);
    RUN_TEST(test_vtx_smartaudio_21_power_cap_and_pit);
    RUN_TEST(test_vtx_smartaudio_v1_and_echo_on_the_wire);
    RUN_TEST(test_vtx_auto_finds_tramp_and_offline_without_vtx);
    RUN_TEST(test_vtx_ranges_from_switches);
    RUN_TEST(test_cli_vtx_and_params);
    RUN_TEST(test_gps_home_distance_and_direction);
    RUN_TEST(test_crsf_telemetry_frames);
    RUN_TEST(test_osd_arrows_and_warnings);
    RUN_TEST(test_osd_screen_layout);
    RUN_TEST(test_osd_chip_driver);
    RUN_TEST(test_beeper_events_and_alarms);
    return UNITY_END();
}
