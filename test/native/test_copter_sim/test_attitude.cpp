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
// Ниже — расширение фильтра: сопротивление винтов и ветер (учатся по GPS),
// компас (рысканье и смещение гироскопа по нему), проверка согласованности.
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

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

// ============================================================
// Остатки 067: сопротивление винтов и ветер — состояния фильтра (учатся в
// полёте по GPS), компас — рысканье и смещение гироскопа по нему.
// ============================================================

namespace
{
    // Манёвры «как летают»: вперёд, вправо, назад, влево по 4 с, между ними —
    // пауза или разворот.
    void manoeuvre(QuadHarness& h, double t, float stick)
    {
        const double ph = std::fmod(t, 24.0);
        h.rc.pitch = h.rc.roll = h.rc.yaw = 0.0f;
        if (ph < 4.0) h.rc.pitch = -stick;
        else if (ph < 6.0) {}
        else if (ph < 10.0) h.rc.roll = stick;
        else if (ph < 12.0) h.rc.yaw = 0.3f;
        else if (ph < 16.0) h.rc.pitch = stick;
        else if (ph < 18.0) {}
        else if (ph < 22.0) h.rc.roll = -stick;
        else h.rc.yaw = -0.3f;
    }

    double wrap180(double d)
    {
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }

    // Ошибка горизонта в осях ошибки фильтра (δθx, δθy — земные оси, рад).
    void tiltError(const QuadHarness& h, double& ex, double& ey)
    {
        const double r = h.sim.rollDeg() * M_PI / 180.0, p = h.sim.pitchDeg() * M_PI / 180.0;
        const Vec3 down = { static_cast<float>(-std::sin(p)), static_cast<float>(std::sin(r) * std::cos(p)),
                            static_cast<float>(std::cos(r) * std::cos(p)) };
        const Vec3 v = h.ctl.getAttitude().toEarth(down);
        ex = v.y;
        ey = -v.x;
    }

    // eᵀ·P⁻¹·e для блока 2 × 2.
    double nees2(double e0, double e1, double p00, double p01, double p11)
    {
        return (e0 * e0 * p11 - 2.0 * e0 * e1 * p01 + e1 * e1 * p00) / (p00 * p11 - p01 * p01);
    }

    // Квантиль χ² (приближение Уилсона — Хилферти), z — квантиль нормального.
    double chi2(double dof, double z)
    {
        const double a = 2.0 / (9.0 * dof);
        const double c = 1.0 - a + z * std::sqrt(a);
        return dof * c * c * c;
    }

    // Долгий наклон без GPS: что даёт верный k в модели фильтра.
    Horizon longTiltWithoutGps(double dragLin, float dragSetting)
    {
        QuadPerturb p;
        p.dragLin = dragLin;
        CopterSettings s;
        s.dragCoef = dragSetting;
        QuadHarness h(five(), p, 131, EscProtocol::DSHOT600, 1000.0f, s);
        Horizon r;
        r.flew = h.takeoffTo(15.0);
        h.rc.roll = 0.5f;
        for (int i = 0; i < 12000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        h.rc.roll = 0.0f;
        for (int i = 0; i < 6000; ++i)
        {
            h.step();
            r.add(horizonErrorDeg(h));
        }
        return r;
    }
}

// Рама с настоящим сопротивлением винтов 0.2 и 0.45 1/с (фильтр начинает с
// 0.3): за полминуты полёта с манёврами оценка сходится — к «эффективному»
// сопротивлению на скорости полёта (у модели, как у настоящего коптера, есть
// ещё квадратичное сопротивление рамы). На висении оценка и её дисперсия не
// двигаются. Выученное — в настройку: следующий полёт без GPS держит
// горизонт на долгом наклоне.
void test_attitude_ekf_learns_rotor_drag_in_flight()
{
    for (double drag : { 0.2, 0.45 })
    {
        QuadPerturb p;
        p.dragLin = drag;
        QuadHarness h(five(), p, drag < 0.3 ? 111 : 112);
        h.gpsOn = true;
        h.gpsDelayS = 0.1;
        h.compassNoiseDeg = 2.0;
        h.sim.setWind(3.0, -2.5, 0.5);   // 4 м/с с порывами
        TEST_ASSERT_TRUE(h.takeoffTo(15.0));
        const AttitudeQuat& a = h.ctl.getAttitude();
        double learnedAt = -1.0, sumLog = 0.0, sumTrue = 0.0, worst = 0.0, at20 = 0.0;
        long n = 0;
        for (long i = 0; i < 48000; ++i)
        {
            manoeuvre(h, i * 0.001, 0.5f);
            h.step();
            if (learnedAt < 0.0 && a.dragLearned()) learnedAt = i * 0.001;
            if (i == 20000) at20 = a.dragCoef();
            worst = std::fmax(worst, horizonErrorDeg(h));
            if (i >= 24000)
            {
                // истина — сопротивление на текущей скорости (линейное + квадратичное)
                sumLog += std::log(a.dragCoef() / h.sim.dragNow());
                sumTrue += h.sim.dragNow();
                ++n;
            }
        }
        const double kEnd = a.dragCoef(), kTrue = sumTrue / n, bias = sumLog / n;
        char line[260];
        snprintf(line, sizeof(line),
                 "rotor drag %.2f (+ frame drag: %.3f at flight speed): k %.3f after 20 s, %.3f after 48 s (sigma %.2f), mean error %+.1f %%, "
                 "ready to save after %.0f s; wind %.1f %.1f (true 3.0 -2.5); horizon max %.2f deg",
                 drag, kTrue, at20, kEnd, a.dragSigma(), bias * 100.0, learnedAt, a.windNorth(), a.windEast(), worst);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed(), line);
        TEST_ASSERT_TRUE_MESSAGE(learnedAt > 0.0 && learnedAt < 30.0, line);
        // после 20 с — в пределах 15 % от настоящего на этой скорости, в среднем за вторую половину — 8 %
        TEST_ASSERT_TRUE_MESSAGE(std::fabs(std::log(at20 / kTrue)) < 0.15, line);
        TEST_ASSERT_TRUE_MESSAGE(std::fabs(bias) < 0.08, line);
        // и не дальше линейной части с одной стороны и «полного» на макс. скорости — с другой
        TEST_ASSERT_TRUE_MESSAGE(kEnd > 0.95 * drag && kEnd < 1.6 * drag, line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.2f, 3.0f, a.windNorth(), line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.2f, -2.5f, a.windEast(), line);
        TEST_ASSERT_TRUE_MESSAGE(worst < 1.5, line);

        // висение (держит точку по GPS): k не наблюдаем — оценка и дисперсия стоят побитно
        h.rc.pitch = h.rc.roll = h.rc.yaw = 0.0f;
        h.rc.mode = CopterMode::POS_HOLD;
        h.run(12.0);   // остановился
        const float k0 = a.dragCoef(), var0 = a.dragCovariance(0, 0);
        double fastest = 0.0;
        for (int i = 0; i < 30000; ++i)
        {
            h.step();
            fastest = std::fmax(fastest, std::hypot(h.sim.velocity().x, h.sim.velocity().y));
        }
        snprintf(line, sizeof(line), "rotor drag %.2f: 30 s of position hold in gusty wind (ground speed up to %.1f m/s): k %.4f -> %.4f, variance %.5f -> %.5f",
                 drag, fastest, k0, a.dragCoef(), var0, a.dragCovariance(0, 0));
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(fastest < 2.5, line);
        TEST_ASSERT_TRUE_MESSAGE(a.dragCoef() == k0 && a.dragCovariance(0, 0) == var0, line);

        // следующий полёт — без GPS, долгий наклон: модель с выученным k
        if (drag > 0.3)
        {
            const Horizon fresh = longTiltWithoutGps(drag, 0.0f), learned = longTiltWithoutGps(drag, k0);
            snprintf(line, sizeof(line), "12 s half-stick tilt without GPS, rotor drag %.2f: horizon error max %.2f deg (rms %.2f) with k 0.30, "
                     "%.2f deg (rms %.2f) with learned k %.3f",
                     drag, fresh.worst, fresh.rms(), learned.worst, learned.rms(), k0);
            TEST_MESSAGE(line);
            TEST_ASSERT_TRUE_MESSAGE(fresh.flew && learned.flew, line);
            TEST_ASSERT_TRUE_MESSAGE(fresh.worst > 1.8, line);            // «до»: те самые 2°
            TEST_ASSERT_TRUE_MESSAGE(learned.worst < 0.8, line);
            TEST_ASSERT_TRUE_MESSAGE(learned.rms() < 0.5 * fresh.rms(), line);
        }
    }
}

// Согласованность фильтра: нормированные квадраты ошибок (NEES — против
// истины модели) и невязок (NIS — против их дисперсии) по восьми полётам с
// разными рамами, ветром, смещениями гироскопа и шумами.
//
// Воздух — такой, на какой фильтр рассчитан: порывы 1.3 м/с, компас с
// «цветной» ошибкой 4°. Тогда у новых состояний (ln k, ветер) и у невязок
// сопротивления и компаса NEES и NIS должны лежать в доверительных границах
// χ² с обеих сторон: фильтр не самоуверен и не перестраховывается.
//
// У горизонта и смещений гироскопа проверяется только верхняя граница
// (фильтр не самоуверен). Нижнюю они не проходят намеренно: шумы там
// заложены под настоящий коптер — вибрация, наложение частот, рывки, — а
// модель чище, и заявленная неуверенность (2–3° в манёвре) выше ошибки
// (0.3°). Подогнать её под модель — значит получить самоуверенный фильтр в
// поле.
void test_attitude_ekf_consistency_nees_nis()
{
    const int runs = 8;
    double sTilt = 0, sBias = 0, sBz = 0, sK = 0, sWind = 0;
    double sAccelNis = 0, sDragNis = 0, sMagNis = 0;
    long cAccel = 0, cDrag = 0, cMag = 0;
    for (int r = 0; r < runs; ++r)
    {
        QuadPerturb p;
        p.dragLin = r % 2 ? 0.45 : 0.2;
        QuadHarness h(five(), p, 200 + r);
        h.gpsOn = true;
        h.gpsDelayS = 0.1;
        h.gpsSeed = 999u + 77u * static_cast<uint32_t>(r);
        h.compassSeed = 5u + 13u * static_cast<uint32_t>(r);
        h.compassNoiseDeg = 4.0;
        h.compassNoiseTauS = 0.5;
        const double wN = 2.0 * std::cos(r * 1.1), wE = 2.0 * std::sin(r * 1.1);
        h.sim.setWind(wN, wE, 1.3);
        const double b[3] = { 0.6 * std::sin(r * 2.3), 0.6 * std::cos(r * 1.7), 0.8 * std::sin(r * 0.9 + 1.0) };
        for (int axis = 0; axis < 3; ++axis) h.gyroBiasDps[axis] = b[axis];
        TEST_ASSERT_TRUE(h.takeoffTo(15.0));
        const AttitudeQuat& a = h.ctl.getAttitude();
        double tilt = 0, bias = 0, bz = 0, k = 0, wind = 0;
        int samples = 0;
        float lastDrag = -1.0f, lastMag = -1.0f;
        for (long i = 0; i < 60000; ++i)
        {
            manoeuvre(h, i * 0.001, 0.5f);
            h.step();
            sAccelNis += a.accelNis();
            ++cAccel;
            if (a.dragNis() != lastDrag)
            {
                lastDrag = a.dragNis();
                sDragNis += lastDrag;
                ++cDrag;
            }
            if (a.compassNis() != lastMag)
            {
                lastMag = a.compassNis();
                sMagNis += lastMag;
                ++cMag;
            }
            if (i >= 25000 && i % 5000 == 0)
            {
                double ex, ey;
                tiltError(h, ex, ey);
                tilt += nees2(ex, ey, a.covariance(0, 0), a.covariance(0, 1), a.covariance(1, 1));
                const Vec3 est = a.gyroBias();
                const double d2r = M_PI / 180.0;
                bias += nees2(b[0] * d2r - est.x, b[1] * d2r - est.y, a.covariance(3, 3), a.covariance(3, 4), a.covariance(4, 4));
                const double ez = b[2] * d2r - est.z;
                bz += ez * ez / a.covariance(5, 5);
                // истина для k — сопротивление на текущей скорости; для ветра — средний (порывы — шум измерения)
                const double ek = std::log(h.sim.dragNow() / a.dragCoef());
                k += ek * ek / a.dragCovariance(0, 0);
                wind += nees2(wN - a.windNorth(), wE - a.windEast(), a.dragCovariance(1, 1), a.dragCovariance(1, 2), a.dragCovariance(2, 2));
                ++samples;
            }
        }
        TEST_ASSERT_TRUE(h.ctl.isArmed());
        // среднее за полёт — одно независимое наблюдение на полёт
        sTilt += tilt / samples;
        sBias += bias / samples;
        sBz += bz / samples;
        sK += k / samples;
        sWind += wind / samples;
    }
    // на степень свободы; границы — χ² с dof·runs степенями, уровень 1 % с каждой стороны
    const double tilt = sTilt / runs / 2, bias = sBias / runs / 2, bz = sBz / runs, k = sK / runs, wind = sWind / runs / 2;
    const double accelNis = sAccelNis / cAccel / 2, dragNis = sDragNis / cDrag / 2, magNis = sMagNis / cMag;
    const double z = 2.326;
    const double lo1 = chi2(runs, -z) / runs, hi1 = chi2(runs, z) / runs;
    const double lo2 = chi2(2 * runs, -z) / (2 * runs), hi2 = chi2(2 * runs, z) / (2 * runs);
    char line[420];
    snprintf(line, sizeof(line),
             "consistency over %d flights, per degree of freedom (1 = honest): NEES ln k %.2f, wind %.2f [bounds %.2f..%.2f and %.2f..%.2f]; "
             "NIS drag %.2f, compass %.2f | conservative by design: NEES tilt %.3f, roll/pitch bias %.3f, yaw bias %.3f, NIS accel %.2f",
             runs, k, wind, lo1, hi1, lo2, hi2, dragNis, magNis, tilt, bias, bz, accelNis);
    TEST_MESSAGE(line);
    // новые состояния и новые измерения — в границах с обеих сторон
    TEST_ASSERT_TRUE_MESSAGE(k > lo1 && k < hi1, line);
    TEST_ASSERT_TRUE_MESSAGE(wind > lo2 && wind < hi2, line);
    TEST_ASSERT_TRUE_MESSAGE(dragNis > lo2 && dragNis < hi2, line);
    TEST_ASSERT_TRUE_MESSAGE(magNis > lo1 && magNis < hi1, line);
    // прежние — не самоуверенны (верхняя граница)
    TEST_ASSERT_TRUE_MESSAGE(tilt < hi2 && bias < hi2 && bz < hi1 && accelNis < hi2, line);
}

// Компас в полёте: смещение гироскопа по рысканью (1.5 °/с) находится, и
// горизонт в манёврах от этого точнее (на наклоне ось рысканья рамы не
// вертикальна — её смещение уводит горизонт). Помеха от моторов — скачок
// курса на 40° — отбрасывается проверкой невязки: ни рысканье, ни смещение,
// ни горизонт не уходят. Без компаса — как было.
void test_attitude_ekf_compass_yaw_bias_and_interference_in_flight()
{
    Horizon hz[2];
    float biasZ[2] = {};
    for (int compass = 0; compass < 2; ++compass)
    {
        QuadHarness h(five(), QuadPerturb(), 121);
        h.gpsOn = true;
        h.compass = compass != 0;
        h.compassNoiseDeg = 2.0;
        h.gyroBiasDps[2] = 1.5;
        hz[compass].flew = h.takeoffTo(12.0);
        for (long i = 0; i < 48000; ++i)
        {
            manoeuvre(h, i * 0.001, 0.5f);
            h.step();
            if (i >= 12000) hz[compass].add(horizonErrorDeg(h));
        }
        biasZ[compass] = h.ctl.getAttitude().gyroBiasDps(2);
        TEST_ASSERT_EQUAL(compass != 0, h.ctl.getAttitude().compassUsed());
        if (!compass) TEST_ASSERT_EQUAL_UINT32(0, h.ctl.getAttitude().compassRejected());
    }
    char line[260];
    snprintf(line, sizeof(line), "yaw gyro bias 1.5 deg/s, 48 s of manoeuvres: no compass - bias found %.2f, horizon rms %.2f max %.2f deg; "
             "compass - bias found %.2f, horizon rms %.2f max %.2f deg",
             biasZ[0], hz[0].rms(), hz[0].worst, biasZ[1], hz[1].rms(), hz[1].worst);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(hz[0].flew && hz[1].flew, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, 1.5f, biasZ[1], line);
    TEST_ASSERT_TRUE_MESSAGE(hz[1].rms() < 0.6 * hz[0].rms(), line);
    TEST_ASSERT_TRUE_MESSAGE(hz[1].worst < 1.2, line);

    // помеха: 10 с компас врёт на 40° (в висении и на развороте), потом перестаёт
    QuadHarness h(five(), QuadPerturb(), 122);
    h.gpsOn = true;
    h.compassNoiseDeg = 2.0;
    h.gyroBiasDps[2] = 1.0;
    TEST_ASSERT_TRUE(h.takeoffTo(10.0));
    h.run(25.0);
    const AttitudeQuat& a = h.ctl.getAttitude();
    const double yaw0 = wrap180(a.yawDeg() - h.sim.yawDeg());
    const float bias0 = a.gyroBiasDps(2);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 1.0f, bias0);
    TEST_ASSERT_EQUAL_UINT32(0, a.compassRejected());
    double worstYaw = 0, worstBias = 0, worstHorizon = 0;
    for (int i = 0; i < 20000; ++i)
    {
        h.compassErrorDeg = i < 10000 ? 40.0 : 0.0;
        h.rc.yaw = (i > 4000 && i < 6000) ? 0.3f : 0.0f;
        h.step();
        worstYaw = std::fmax(worstYaw, std::fabs(wrap180(a.yawDeg() - h.sim.yawDeg() - yaw0)));
        worstBias = std::fmax(worstBias, std::fabs(a.gyroBiasDps(2) - bias0));
        worstHorizon = std::fmax(worstHorizon, horizonErrorDeg(h));
    }
    snprintf(line, sizeof(line), "compass off by 40 deg for 10 s in flight: rejected %u samples; yaw moved by %.2f deg, yaw bias by %.3f deg/s, horizon error max %.2f deg",
             static_cast<unsigned>(a.compassRejected()), worstYaw, worstBias, worstHorizon);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(a.compassRejected() >= 230, line);
    TEST_ASSERT_TRUE_MESSAGE(worstYaw < 1.5, line);
    TEST_ASSERT_TRUE_MESSAGE(worstBias < 0.1, line);
    TEST_ASSERT_TRUE_MESSAGE(worstHorizon < 0.5, line);
    TEST_ASSERT_TRUE_MESSAGE(h.ctl.isArmed(), line);
}
