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

#include <string>

#include "copter/AltitudeHold.h"
#include "copter/Attitude.h"
#include "copter/CopterCli.h"
#include "copter/CopterController.h"
#include "copter/EscProtocol.h"
#include "copter/Filters.h"
#include "copter/Frames.h"
#include "copter/GyroFilter.h"
#include "copter/Mixer.h"
#include "copter/Params.h"
#include "copter/Presets.h"
#include "copter/Propulsion.h"
#include "copter/RateController.h"

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
void test_rate_axis_relaxes_integral_on_large_error()
{
    RateAxis a = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    for (int i = 0; i < 1000; ++i) a.update(500.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, a.lastTerms().i);
    RateAxis b = makeAxis(0.0f, 0.1f, 0.0f, 0.0f);
    for (int i = 0; i < 1000; ++i) b.update(50.0f, 0.0f, 1.0f, 1.0f, false, false);
    TEST_ASSERT_TRUE(b.lastTerms().i > 0.0f && b.lastTerms().i < 0.1f * 50.0f * 1.0f);
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

    Bench()
    {
        ctl.configure(deriveTune(*findBuild("freestyle5"), 1000.0f, EscProtocol::DSHOT600));
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
    TEST_ASSERT_EQUAL_HEX16(0, b.ctl.getStatus().armBlockers);
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
        Bench b;
        b.ctl.setExternalBlock(true);
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_EXTERNAL);
    }
    {
        Bench b;
        b.rc.linkOk = false;
        TEST_ASSERT_FALSE(b.armAfterBoot());
        TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_NO_LINK);
    }
    for (uint16_t bit = 1; bit <= ARM_BLOCK_EXTERNAL; bit <<= 1)
    {
        TEST_ASSERT_NOT_EQUAL(0, strcmp("?", armBlockText(bit)));
    }
    TEST_ASSERT_EQUAL_STRING("?", armBlockText(0));
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
    TEST_ASSERT_TRUE(b.ctl.getStatus().armBlockers & ARM_BLOCK_SWITCH);
    b.rc.armSwitch = false;
    b.run(0.01f);
    b.rc.armSwitch = true;
    b.run(0.01f);
    TEST_ASSERT_TRUE(b.ctl.isArmed());
    b.ctl.disarm();
    TEST_ASSERT_FALSE(b.ctl.isArmed());
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
    const CopterTune t = p.tune();
    const AxisGains* g[3] = { &t.roll, &t.pitch, &t.yaw };
    for (uint8_t a = 0; a < 3; ++a)
    {
        for (uint8_t k = 0; k < 4; ++k)
        {
            const PidTerm term = static_cast<PidTerm>(k);
            TEST_ASSERT_EQUAL_FLOAT(expect[a][k], p.defaultValue(pidParam(a, term)));
            TEST_ASSERT_FLOAT_WITHIN(1e-9f, bfToGain(term, a, expect[a][k]), gainTerm(*g[a], term));
        }
    }
    TEST_ASSERT_FALSE(t.gainsFromModel);
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::D), static_cast<int>(t.tpaMode));
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
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::PD), static_cast<int>(p.tune().tpaMode));
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
        "rateprofile 1",
        "set roll_srate = 80",
        "batch end",
    };
    for (const char* line : bfDiff) cli.execute(line);
    out = port.take();
    TEST_ASSERT_TRUE(has(out, "###ОШИБКА: нет команды feature"));
    TEST_ASSERT_TRUE(has(out, "###ОШИБКА: нет настройки d_max_pitch###"));
    cli.execute("profile");
    TEST_ASSERT_TRUE(has(port.take(), "profile 1 # wind"));
    TEST_ASSERT_EQUAL_UINT8(1, p.pidProfile());
    TEST_ASSERT_EQUAL_FLOAT(38.0f, p.value(PARAM_D_PITCH));
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
    TEST_ASSERT_EQUAL(static_cast<int>(TpaMode::D), static_cast<int>(b.ctl.getTune().tpaMode));
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
    RUN_TEST(test_mode_names_and_settings);
    RUN_TEST(test_params_table_is_consistent);
    RUN_TEST(test_params_set_get_bounds_and_enums);
    RUN_TEST(test_params_survive_flash_and_skip_unknown);
    RUN_TEST(test_pid_in_betaflight_units);
    RUN_TEST(test_betaflight_source_takes_bf_45_defaults);
    RUN_TEST(test_profiles_are_independent_and_switch_on_the_fly);
    RUN_TEST(test_cli_set_get_diff_save_and_paste);
    return UNITY_END();
}
