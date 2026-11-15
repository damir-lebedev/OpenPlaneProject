// ============================================================
// Фильтр ориентации в замкнутом полёте: MEKF (copter/Attitude.h) против
// прежнего Махони — на одной и той же модели, одним и тем же полётом.
//
// Мерило — ошибка горизонта: угол между «вниз» по оценке и настоящим
// «вниз» модели (один угол, годится и вверх ногами). Сценарии — то, на
// чём фильтр с постоянным усилением и без оценки смещения врёт:
//   • нуль гироскопа ушёл на 2 °/с (с включения и постепенно в полёте);
//   • флипы при неточном масштабе гироскопа (1 %);
//   • сильная вибрация, рывки газом и стиком;
//   • долгий наклон и долгий разгон (акселерометр «видит» тягу, а не тяжесть).
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "helpers/QuadHarness.h"
#include "helpers/QuadVariants.h"

using namespace copter;

namespace
{
    double horizonErrorDeg(const QuadHarness& h)
    {
        const Vec3 d = h.ctl.getAttitude().downBody();
        const double r = h.sim.rollDeg() * M_PI / 180.0, p = h.sim.pitchDeg() * M_PI / 180.0;
        const double c = d.x * -std::sin(p) + d.y * std::sin(r) * std::cos(p) + d.z * std::cos(r) * std::cos(p);
        return std::acos(std::fmax(-1.0, std::fmin(1.0, c))) * 57.29578;
    }

    struct Horizon
    {
        double sum2 = 0, worst = 0, last = 0;
        long n = 0;
        bool flew = false;
        void add(double e)
        {
            sum2 += e * e;
            worst = std::fmax(worst, e);
            last += (e - last) * 0.002;   // сглаженная за ~0.5 с: чем кончилось
            ++n;
        }
        double rms() const { return n ? std::sqrt(sum2 / n) : 0; }
    };

    CopterSettings filterSettings(bool ekf)
    {
        CopterSettings s;
        s.attitudeEkf = ekf;
        return s;
    }

    const BuildSpec& five() { return *findBuild("freestyle5"); }

    // Нуль гироскопа ушёл с самого включения: взлёт, 20 с висения.
    Horizon biasFromBoot(bool ekf)
    {
        QuadHarness h(five(), QuadPerturb(), 101, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        h.gyroBiasDps[0] = 2.0;
        h.gyroBiasDps[1] = -2.0;
        h.gyroBiasDps[2] = 1.0;
        Horizon r;
        r.flew = h.takeoffTo(5.0);
        for (int i = 0; i < 20000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        return r;
    }

    // Нуль уходит в полёте — гироскоп греется: 0 -> 2 °/с за 2 минуты (у
    // MPU6000 по паспорту до 0.16 °/с на градус), ещё 20 с.
    Horizon biasDrift(bool ekf)
    {
        QuadHarness h(five(), QuadPerturb(), 102, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        Horizon r;
        r.flew = h.takeoffTo(5.0);
        for (int i = 0; i < 140000; ++i)
        {
            const double k = std::fmin(1.0, i / 120000.0);
            h.gyroBiasDps[0] = 2.0 * k;
            h.gyroBiasDps[1] = -2.0 * k;
            h.step();
            r.add(horizonErrorDeg(h));
        }
        return r;
    }

    // Три флипа по крену в ACRO; масштаб гироскопа врёт на 1 % (за 1080° — 11°).
    // Ошибка — за 6 с после флипов: как быстро горизонт возвращается. Без GPS
    // акселерометр ошибку почти не видит: он показывает тягу, а она идёт по
    // оси рамы, как и оценка; тяжесть проявится, только когда встанет
    // сопротивление воздуха (секунды). С GPS видно боковое ускорение.
    Horizon flips(bool ekf, bool gps)
    {
        QuadHarness h(five(), QuadPerturb(), 103, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        h.gyroScale = 1.01;
        h.gpsOn = gps;
        Horizon r;
        r.flew = h.takeoffTo(40.0);
        h.rc.mode = CopterMode::ACRO;
        h.rc.throttle = h.ctl.getStatus().hoverLearned;
        h.run(0.3);
        h.rc.roll = 1.0f;
        const double rate = h.tune.rates[0].apply(1.0f);
        h.run(1080.0 / rate + 0.05);
        h.rc.roll = 0.0f;
        h.rc.mode = CopterMode::ALT_HOLD;
        h.rc.throttle = 0.5f;
        for (int i = 0; i < 6000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        r.flew = r.flew && h.ctl.isArmed() && h.sim.height() > 5.0;
        return r;
    }

    // Втрое больше вибрации, рывки газом и стиком крена 10 с (ANGLE).
    Horizon vibration(bool ekf)
    {
        QuadPerturb p;
        p.vibration = 3.0;
        QuadHarness h(five(), p, 104, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        Horizon r;
        r.flew = h.takeoffTo(8.0);
        h.rc.mode = CopterMode::ANGLE;
        const float hover = h.ctl.getStatus().hoverLearned;
        for (int i = 0; i < 10000; ++i)
        {
            const bool punch = (i / 400) % 2 == 0;
            h.rc.throttle = hover * (punch ? 1.6f : 0.6f);
            h.rc.roll = 0.5f * static_cast<float>(std::sin(2.0 * M_PI * i / 1000.0));
            h.step();
            r.add(horizonErrorDeg(h));
        }
        r.flew = r.flew && h.ctl.isArmed();
        return r;
    }

    // Наклон на полстика и 4 с так: акселерометр показывает «ровно» (тягу).
    Horizon tiltHold(bool ekf)
    {
        QuadHarness h(five(), QuadPerturb(), 105, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        Horizon r;
        r.flew = h.takeoffTo(10.0);
        h.rc.roll = 0.5f;
        for (int i = 0; i < 4000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        h.rc.roll = 0.0f;
        for (int i = 0; i < 3000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        return r;
    }

    // Вираж с GPS и компасом: 12 с по кругу (крен 0.6 стика и рысканье) —
    // «верх» акселерометра наклонён на центростремительное ускорение.
    Horizon turn(bool ekf)
    {
        QuadHarness h(five(), QuadPerturb(), 106, EscProtocol::DSHOT600, 1000.0f, filterSettings(ekf));
        h.gpsOn = true;
        Horizon r;
        r.flew = h.takeoffTo(15.0);
        h.rc.pitch = -0.5f;   // вперёд
        h.run(3.0);
        h.rc.roll = 0.6f;
        h.rc.yaw = 0.25f;
        for (int i = 0; i < 12000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        h.rc.roll = h.rc.pitch = h.rc.yaw = 0.0f;
        for (int i = 0; i < 4000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        r.flew = r.flew && h.ctl.isArmed();
        return r;
    }

    void report(const char* name, const Horizon& mahony, const Horizon& ekf, char* line, size_t size)
    {
        snprintf(line, size, "horizon error, %-36s Mahony rms %.2f max %.2f end %.2f | MEKF rms %.2f max %.2f end %.2f deg", name,
                 mahony.rms(), mahony.worst, mahony.last, ekf.rms(), ekf.worst, ekf.last);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(mahony.flew && ekf.flew, line);
    }
}

// Смещение гироскопа 2 °/с: Махони стоит с ошибкой смещение/усиление (4°),
// MEKF находит смещение — на земле за секунды, в полёте — за десятки секунд.
void test_attitude_ekf_learns_gyro_bias()
{
    char line[240];
    const Horizon m = biasFromBoot(false), e = biasFromBoot(true);
    report("gyro bias 2 deg/s from boot:", m, e, line, sizeof(line));
    TEST_ASSERT_TRUE_MESSAGE(m.rms() > 2.5, line);     // «до»: ошибка есть
    // на земле до взлёта — всего 2.5 с: смещение найдено с точностью ~0.15 °/с,
    // остаток доучивается в полёте
    TEST_ASSERT_TRUE_MESSAGE(e.rms() < 1.0, line);
    TEST_ASSERT_TRUE_MESSAGE(e.worst < 2.0, line);

    const Horizon md = biasDrift(false), ed = biasDrift(true);
    report("gyro bias drifts to 2 deg/s in 2 min:", md, ed, line, sizeof(line));
    TEST_ASSERT_TRUE_MESSAGE(md.worst > 3.0, line);
    // уход нуля 1 °/с за минуту: ошибка — скорость ухода / усиление по
    // смещению (0.017 °/с² / 0.01 1/с² ≈ 1.7°), с запасом на опоздание
    // сопротивления воздуха
    TEST_ASSERT_TRUE_MESSAGE(ed.worst < 3.5, line);
    TEST_ASSERT_TRUE_MESSAGE(ed.rms() < 0.7 * md.rms(), line);

    // смещение найдено: то, что вложили
    QuadHarness h(five(), QuadPerturb(), 107);
    h.gyroBiasDps[0] = 2.0;
    h.gyroBiasDps[1] = -2.0;
    TEST_ASSERT_TRUE(h.takeoffTo(5.0));
    h.run(10.0);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 2.0f, h.ctl.getAttitude().gyroBiasDps(0));
    TEST_ASSERT_FLOAT_WITHIN(0.3f, -2.0f, h.ctl.getAttitude().gyroBiasDps(1));
}

// Флипы, вибрация, манёвры: MEKF не хуже Махони нигде и лучше там, где
// акселерометр показывает тягу вместо тяжести.
void test_attitude_ekf_against_mahony_in_manoeuvres()
{
    char line[240];
    char all[1600] = "";
    auto note = [&]() {
        strncat(all, line, sizeof(all) - strlen(all) - 2);
        strncat(all, "\n", sizeof(all) - strlen(all) - 1);
    };
    const Horizon mf = flips(false, false), ef = flips(true, false);
    report("3 flips, gyro scale 1% off, no GPS:", mf, ef, line, sizeof(line));
    note();
    const Horizon mg = flips(false, true), eg = flips(true, true);
    report("3 flips, gyro scale 1% off, GPS:", mg, eg, line, sizeof(line));
    note();
    const Horizon mv = vibration(false), ev = vibration(true);
    report("vibration x3, throttle punches:", mv, ev, line, sizeof(line));
    note();
    const Horizon mt = tiltHold(false), et = tiltHold(true);
    report("half-stick tilt held for 4 s:", mt, et, line, sizeof(line));
    note();
    const Horizon mc = turn(false), ec = turn(true);
    report("12 s turn with GPS and compass:", mc, ec, line, sizeof(line));
    note();

    // после флипов ошибка 11° (1 % от 1080°) — у обоих; дальше MEKF её убирает:
    // с GPS — по ускорению, без — по сопротивлению воздуха (медленнее)
    TEST_ASSERT_TRUE_MESSAGE(ef.rms() < mf.rms() + 0.3, all);
    TEST_ASSERT_TRUE_MESSAGE(eg.rms() < mg.rms() + 0.3, all);
    TEST_ASSERT_TRUE_MESSAGE(ef.last < 3.0 && eg.last < 3.0, all);
    // вибрация: оба держат доли градуса
    TEST_ASSERT_TRUE_MESSAGE(ev.rms() < 0.6, all);
    TEST_ASSERT_TRUE_MESSAGE(ev.worst < 1.5, all);
    // наклон и вираж: Махони тянет горизонт к «ровно» по тяге, MEKF — нет
    TEST_ASSERT_TRUE_MESSAGE(mt.worst > 4.0, all);
    TEST_ASSERT_TRUE_MESSAGE(et.worst < 1.0, all);
    TEST_ASSERT_TRUE_MESSAGE(ec.rms() < 0.6 * mc.rms(), all);
    TEST_ASSERT_TRUE_MESSAGE(ec.worst < 1.5, all);
}

// Посадка автоматом на склон (у модели земля ровная — «ровно» сдвинуто
// подстройкой горизонта: то же самое): интеграл не раскачивает моторы,
// коптер не прыгает, моторы выключаются. Раньше с 1° не выключались вовсе.
void test_failsafe_landing_on_a_slope_disarms()
{
    const char* ids[] = { "freestyle5", "longrange7", "cinelifter8" };
    for (const char* id : ids)
    {
        const BuildSpec* b = findBuild(id);
        if (!b) continue;
        for (double slope : { 1.0, 3.0 })
        {
            CopterSettings s;
            s.levelTrimRollDeg = static_cast<float>(slope);
            QuadHarness h(*b, QuadPerturb(), 108, EscProtocol::DSHOT600, 1000.0f, s);
            TEST_ASSERT_TRUE(h.takeoffTo(8.0));
            h.rc.linkOk = false;
            const double t0 = h.sim.now();
            double touch = -1;
            while (h.ctl.isArmed() && h.sim.now() - t0 < 40.0)
            {
                h.step();
                if (touch < 0 && h.sim.grounded()) touch = h.sim.now() - t0;
            }
            char line[160];
            snprintf(line, sizeof(line), "failsafe landing, %s, level off by %.0f deg: touched at %.1f s, disarmed %s at %.1f s", id, slope,
                     touch, h.ctl.isArmed() ? "NO" : "yes", h.sim.now() - t0);
            TEST_MESSAGE(line);
            TEST_ASSERT_FALSE_MESSAGE(h.ctl.isArmed(), line);
            TEST_ASSERT_TRUE_MESSAGE(h.sim.now() - t0 < touch + 10.0, line);
        }
    }
}
