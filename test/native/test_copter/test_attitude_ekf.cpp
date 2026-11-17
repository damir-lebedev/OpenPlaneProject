// ============================================================
// MEKF (copter/Attitude.h), расширение: сопротивление винтов и ветер как
// состояния фильтра (учатся по скорости GPS), курс компаса — рысканье и
// смещение гироскопа по нему.
//
// Истина — маленькая точная модель: коптер держит высоту, тяга по оси рамы,
// сопротивление винтов лежит в их плоскости и равно −k·(скорость
// относительно воздуха). Модель фильтра совпадает с ней точно — поэтому
// проверки строгие: k с точностью до процентов, «не двигается» — побитно.
// Замкнутые полёты на 6DOF-модели с шумами — в test_copter_sim.
// ============================================================

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "copter/Attitude.h"

using namespace copter;

namespace
{
    constexpr double G = 9.80665;
    constexpr double D2R = M_PI / 180.0;

    struct PointCopter
    {
        double q[4] = { 1, 0, 0, 0 };   // рама (FRD) -> земля (NED)
        double vN = 0, vE = 0;
        double k = 0.3;                 // сопротивление винтов, 1/с
        double windN = 0, windE = 0;
        double rollT = 0, pitchT = 0;   // куда наклоняться, рад
        double yawRate = 0;             // рад/с
        double flipRate = 0;            // рад/с по крену поверх всего (флип)
        double biasDps[3] = {};
        double gyroScale = 1.0;
        bool holdPlace = false;         // держать нулевую скорость по земле (висение, и на ветру)
        double R[3][3] = {};
        float gyro[3] = {}, accel[3] = {};

        void setYawDeg(double deg)
        {
            q[0] = std::cos(0.5 * deg * D2R);
            q[1] = q[2] = 0;
            q[3] = std::sin(0.5 * deg * D2R);
        }
        void rotation()
        {
            const double w = q[0], x = q[1], y = q[2], z = q[3];
            R[0][0] = 1 - 2 * (y * y + z * z);
            R[0][1] = 2 * (x * y - w * z);
            R[0][2] = 2 * (x * z + w * y);
            R[1][0] = 2 * (x * y + w * z);
            R[1][1] = 1 - 2 * (x * x + z * z);
            R[1][2] = 2 * (y * z - w * x);
            R[2][0] = 2 * (x * z - w * y);
            R[2][1] = 2 * (y * z + w * x);
            R[2][2] = 1 - 2 * (x * x + y * y);
        }
        double rollRad() const { return std::atan2(R[2][1], R[2][2]); }
        double pitchRad() const { return -std::asin(std::fmax(-1.0, std::fmin(1.0, R[2][0]))); }
        double yawDeg() const { return std::atan2(R[1][0], R[0][0]) / D2R; }
        double headingDeg() const { return yawDeg(); }

        void step(double dt)
        {
            rotation();
            if (holdPlace)
            {
                // висение: наклон против скорости по земле (в осях носа)
                const double yaw = yawDeg() * D2R, c = std::cos(yaw), s = std::sin(yaw);
                const double fwd = vN * c + vE * s, right = -vN * s + vE * c;
                pitchT = std::atan(2.0 * fwd / G);
                rollT = -std::atan(2.0 * right / G);
            }
            auto clamp = [](double v, double lim) { return std::fmax(-lim, std::fmin(lim, v)); };
            const bool flipping = flipRate != 0.0;
            const double p = flipping ? flipRate : clamp(6.0 * (rollT - rollRad()), 3.0);
            const double qq = flipping ? 0.0 : clamp(6.0 * (pitchT - pitchRad()), 3.0);
            const double r = yawRate;

            // сопротивление винтов: в плоскости рамы, против скорости в воздухе
            const double relN = vN - windN, relE = vE - windE;
            const double ux = R[0][0] * relN + R[1][0] * relE, uy = R[0][1] * relN + R[1][1] * relE;
            const double dx = -k * ux, dy = -k * uy;
            // тяга держит высоту (вверх ногами и на боку — просто вес)
            const double thrust = R[2][2] > 0.3 ? (G + R[2][0] * dx + R[2][1] * dy) / R[2][2] : G;
            vN += (R[0][0] * dx + R[0][1] * dy - R[0][2] * thrust) * dt;
            vE += (R[1][0] * dx + R[1][1] * dy - R[1][2] * thrust) * dt;

            // датчики: акселерометр X нос, Y влево, Z вверх (g); гироскоп °/с
            accel[0] = static_cast<float>(dx / G);
            accel[1] = static_cast<float>(-dy / G);
            accel[2] = static_cast<float>(thrust / G);
            gyro[0] = static_cast<float>(p / D2R * gyroScale + biasDps[0]);
            gyro[1] = static_cast<float>(qq / D2R * gyroScale + biasDps[1]);
            gyro[2] = static_cast<float>(r / D2R * gyroScale + biasDps[2]);

            const double hw = 0.5 * dt;
            const double w = q[0], x = q[1], y = q[2], z = q[3];
            double n[4] = { w + hw * (-x * p - y * qq - z * r), x + hw * (w * p + y * r - z * qq), y + hw * (w * qq - x * r + z * p),
                            z + hw * (w * r + x * qq - y * p) };
            const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2] + n[3] * n[3]);
            for (int i = 0; i < 4; ++i) q[i] = n[i] / len;
        }
    };

    // Фильтр, которому скармливают то же, что контроллер: GPS 10 Гц (фикс
    // описывает то, что было 0.1 с назад, — как у настоящего приёмника), компас.
    struct Fed
    {
        AttitudeQuat a;
        GpsAccel accelGps;
        bool gps = true;
        bool compass = false;
        double compassErrorDeg = 0.0;
        long n = 0;
        static constexpr int GPS_DELAY = 100;   // шагов
        double pastN[GPS_DELAY] = {}, pastE[GPS_DELAY] = {};

        Fed()
        {
            a.setEkf(true);
            const float gyro[3] = { 0, 0, 0 }, level[3] = { 0, 0, 1 };
            for (int i = 0; i < 500; ++i) a.update(gyro, level, 0.001f);   // на земле встал в горизонт
            a.setGrounded(false);
        }

        void step(PointCopter& c, double dt = 0.001)
        {
            c.step(dt);
            const bool fresh = gps && n % 100 == 0;
            const float velN = static_cast<float>(pastN[n % GPS_DELAY]), velE = static_cast<float>(pastE[n % GPS_DELAY]);
            pastN[n % GPS_DELAY] = c.vN;
            pastE[n % GPS_DELAY] = c.vE;
            accelGps.update(fresh, gps, velN, velE, static_cast<float>(dt));
            a.setGroundAccel(accelGps.accel());
            const float d = static_cast<float>(c.headingDeg()) - a.yawDeg();
            if (accelGps.accel() >= 0.0f) a.setGroundAccelVector(accelGps.accelN(), accelGps.accelE(), d);
            else a.clearGroundAccelVector();
            if (fresh) a.setDragVelocity(velN, velE, 0.0f, d);
            if (compass) a.setCompass(static_cast<float>(c.headingDeg() + compassErrorDeg) - a.yawDeg());
            else a.clearCompass();
            a.update(c.gyro, c.accel, static_cast<float>(dt));
            ++n;
        }

        // Угол между «вниз» по оценке и настоящим, °.
        double horizonErrorDeg(const PointCopter& c) const
        {
            const Vec3 d = a.downBody();
            const double dot = d.x * c.R[2][0] + d.y * c.R[2][1] + d.z * c.R[2][2];
            return std::acos(std::fmax(-1.0, std::fmin(1.0, dot))) / D2R;
        }
    };

    // Полёт с манёврами: каждые 4 с — новый наклон (до 25°), иногда разворот.
    void fly(PointCopter& c, Fed& f, double seconds, double tiltDeg = 25.0)
    {
        const long steps = std::lround(seconds * 1000.0);
        for (long i = 0; i < steps; ++i)
        {
            const int leg = static_cast<int>(f.n / 4000) % 6;
            const double tilt = tiltDeg * D2R;
            c.rollT = leg == 1 ? tilt : leg == 4 ? -tilt : 0.0;
            c.pitchT = leg == 0 ? -tilt : leg == 3 ? tilt : 0.0;
            c.yawRate = leg == 2 ? 0.5 : leg == 5 ? -0.3 : 0.0;
            f.step(c);
        }
        c.rollT = c.pitchT = c.yawRate = 0.0;
    }

    void hover(PointCopter& c, Fed& f, double seconds)
    {
        c.holdPlace = true;
        const long steps = std::lround(seconds * 1000.0);
        for (long i = 0; i < steps; ++i) f.step(c);
        c.holdPlace = false;
    }

    // Матрица — ковариация: симметрична, конечна, положительно определена
    // (разложение Холецкого проходит).
    bool isCovariance(const double* m, int n)
    {
        double L[6][6] = {};
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                if (!(m[i * n + j] == m[j * n + i]) || !std::isfinite(m[i * n + j])) return false;
                double sum = m[i * n + j];
                for (int k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
                if (i == j)
                {
                    if (!(sum > 0.0)) return false;
                    L[i][i] = std::sqrt(sum);
                }
                else
                {
                    L[i][j] = sum / L[j][j];
                }
            }
        }
        return true;
    }

    bool covariancesOk(const AttitudeQuat& a)
    {
        double P[36], D[9];
        for (uint8_t i = 0; i < 6; ++i)
        {
            for (uint8_t j = 0; j < 6; ++j) P[i * 6 + j] = a.covariance(i, j);
        }
        for (uint8_t i = 0; i < 3; ++i)
        {
            for (uint8_t j = 0; j < 3; ++j) D[i * 3 + j] = a.dragCovariance(i, j);
        }
        return isCovariance(P, 6) && isCovariance(D, 3);
    }
}

// Сопротивление винтов находится по скорости GPS на раме с k = 0.2 и 0.45
// (фильтр начинает с 0.3), при ветре и при курсе, не совпадающем с рысканьем
// фильтра (оно — от включения). Ветер находится заодно.
void test_attitude_ekf_learns_rotor_drag_from_gps_velocity()
{
    for (double k : { 0.2, 0.45 })
    {
        PointCopter c;
        c.k = k;
        c.windN = 3.0;
        c.windE = -2.0;
        c.setYawDeg(70.0);   // нос на восток-северо-восток, фильтр считает рысканье от нуля
        Fed f;
        TEST_ASSERT_EQUAL_FLOAT(0.3f, f.a.dragCoef());
        TEST_ASSERT_EQUAL_FLOAT(0.5f, f.a.dragSigma());
        TEST_ASSERT_FALSE(f.a.dragLearned());

        fly(c, f, 20.0);
        char line[200];
        snprintf(line, sizeof(line), "rotor drag %.2f: after 20 s k %.3f (sigma %.2f), wind %.2f %.2f; rejected %u", k, f.a.dragCoef(),
                 f.a.dragSigma(), f.a.windNorth(), f.a.windEast(), static_cast<unsigned>(f.a.dragRejected()));
        TEST_MESSAGE(line);
        // 20 с манёвров: уже в пределах 12 % (было 33–50 %), и фильтр это знает
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(static_cast<float>(0.12 * k), static_cast<float>(k), f.a.dragCoef(), line);
        TEST_ASSERT_TRUE_MESSAGE(f.a.dragLearned(), line);
        fly(c, f, 28.0);
        snprintf(line, sizeof(line), "rotor drag %.2f: after 48 s k %.3f (sigma %.2f), wind %.2f %.2f", k, f.a.dragCoef(),
                 f.a.dragSigma(), f.a.windNorth(), f.a.windEast());
        TEST_MESSAGE(line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(static_cast<float>(0.06 * k), static_cast<float>(k), f.a.dragCoef(), line);
        // дальше k и ветер «расцепляются» (в полёте в одну сторону их не различить): 2.5 %
        fly(c, f, 48.0);
        snprintf(line, sizeof(line), "rotor drag %.2f: after 96 s k %.3f (sigma %.2f), wind %.2f %.2f; horizon error %.2f deg", k,
                 f.a.dragCoef(), f.a.dragSigma(), f.a.windNorth(), f.a.windEast(), f.horizonErrorDeg(c));
        TEST_MESSAGE(line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(static_cast<float>(0.025 * k), static_cast<float>(k), f.a.dragCoef(), line);
        TEST_ASSERT_TRUE_MESSAGE(f.a.dragSigma() <= 0.12f && f.a.dragSigma() >= 0.0999f, line);   // пол ±10 %
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, 3.0f, f.a.windNorth(), line);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, -2.0f, f.a.windEast(), line);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, f.a.dragRejected(), line);
        TEST_ASSERT_TRUE_MESSAGE(f.horizonErrorDeg(c) < 1.0, line);
        TEST_ASSERT_TRUE(covariancesOk(f.a));
    }
}

// Наблюдаемость. В висении (и в висении на ветру) k не виден — измерение даёт
// произведение k·ветер: оценка k и её дисперсия не двигаются ни на бит, пока
// скорость по земле меньше 3 м/с; ветер при этом находится. Без GPS k не
// учится вовсе. Начальное значение — из настройки (attitude_drag).
void test_attitude_ekf_rotor_drag_stands_still_in_hover_and_without_gps()
{
    // висение на ветру 4 м/с сразу после взлёта: k как был 0.3 ± 50 %
    PointCopter c;
    c.k = 0.45;
    c.windN = -2.4;
    c.windE = 3.2;
    Fed f;
    hover(c, f, 30.0);
    TEST_ASSERT_TRUE(std::hypot(c.vN, c.vE) < 1.0);   // держится на месте, наклонившись на ветер
    TEST_ASSERT_EQUAL_FLOAT(0.3f, f.a.dragCoef());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, f.a.dragCovariance(0, 0));
    // ветер виден с точностью до k: фильтр думает k = 0.3, настоящий 0.45 —
    // «ветер» в полтора раза сильнее (произведение k·ветер верное)
    char line[160];
    snprintf(line, sizeof(line), "hover in wind (-2.4, 3.2), true k 0.45, filter k 0.30: wind estimate %.2f %.2f", f.a.windNorth(),
             f.a.windEast());
    TEST_MESSAGE(line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, -2.4f * 1.5f, f.a.windNorth(), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, 3.2f * 1.5f, f.a.windEast(), line);

    // полетал — выучил; снова висение: оценка и дисперсия стоят побитно
    fly(c, f, 48.0);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.45f, f.a.dragCoef());
    hover(c, f, 20.0);   // остановился
    const float k0 = f.a.dragCoef(), var0 = f.a.dragCovariance(0, 0);
    hover(c, f, 60.0);
    TEST_ASSERT_TRUE(f.a.dragCoef() == k0);
    TEST_ASSERT_TRUE(f.a.dragCovariance(0, 0) == var0);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, -2.4f, f.a.windNorth());   // а ветер теперь настоящий
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 3.2f, f.a.windEast());
    TEST_ASSERT_TRUE(covariancesOk(f.a));

    // без GPS: те же манёвры — k и дисперсия те же, что при включении
    PointCopter c2;
    c2.k = 0.45;
    Fed blind;
    blind.gps = false;
    blind.a.setDragCoef(0.0f);
    fly(c2, blind, 48.0);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, blind.a.dragCoef());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, blind.a.dragCovariance(0, 0));
    TEST_ASSERT_FALSE(blind.a.dragLearned());

    // настройка: измеренному раньше — вера ±15 %; вне разумного — в границы
    AttitudeQuat a;
    a.setEkf(true);
    a.setDragCoef(0.45f);
    TEST_ASSERT_EQUAL_FLOAT(0.45f, a.dragCoef());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, a.dragSigma());
    a.setDragCoef(7.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, a.dragCoef());
    a.setDragCoef(0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, a.dragCoef());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, a.dragSigma());
    a.setDragCoef(0.45f);
    a.reset();   // настройка контроллера заново — значение из настроек остаётся
    TEST_ASSERT_EQUAL_FLOAT(0.45f, a.dragCoef());
}

// Зачем это всё: без GPS долгий наклон на раме с k = 0.45 при модели 0.3
// уводил горизонт на градусы; с выученным (сохранённым) k — нет.
void test_attitude_ekf_learned_drag_keeps_horizon_in_long_tilt_without_gps()
{
    double worst[2] = {};
    for (int learned = 0; learned < 2; ++learned)
    {
        PointCopter c;
        c.k = 0.45;
        Fed f;
        f.gps = false;
        f.a.setDragCoef(learned ? 0.45f : 0.0f);
        c.rollT = 25.0 * D2R;
        for (int i = 0; i < 15000; ++i)
        {
            f.step(c);
            worst[learned] = std::fmax(worst[learned], f.horizonErrorDeg(c));
        }
        c.rollT = 0.0;
        for (int i = 0; i < 6000; ++i)
        {
            f.step(c);
            worst[learned] = std::fmax(worst[learned], f.horizonErrorDeg(c));
        }
    }
    char line[160];
    snprintf(line, sizeof(line), "15 s tilt without GPS, true k 0.45: horizon error max %.2f deg with k 0.30, %.2f deg with learned k",
             worst[0], worst[1]);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(worst[0] > 1.0, line);
    TEST_ASSERT_TRUE_MESSAGE(worst[1] < 0.3, line);
}

// Компас: рысканье перестаёт уплывать, смещение гироскопа по нему находится.
// Без компаса его не найти (и не выдумать) — как раньше.
void test_attitude_ekf_compass_finds_yaw_bias()
{
    PointCopter c1, c2;
    c1.biasDps[2] = c2.biasDps[2] = 1.0;
    c1.holdPlace = c2.holdPlace = true;
    Fed with, without;
    with.compass = true;
    with.gps = without.gps = false;
    float after10 = 0.0f;
    for (int i = 0; i < 30000; ++i)
    {
        // компас шумит: ±4.5°, медленно (детерминированно)
        with.compassErrorDeg = 3.0 * std::sin(i * 0.004) + 1.5 * std::sin(i * 0.031);
        with.step(c1);
        without.step(c2);
        if (i == 10000) after10 = with.a.gyroBiasDps(2);
    }
    char line[220];
    snprintf(line, sizeof(line),
             "yaw gyro bias 1 deg/s, hover: with compass bias z %.3f after 10 s, %.3f after 30 s, yaw %.2f; without: bias z %.3f, yaw %.2f",
             after10, with.a.gyroBiasDps(2), with.a.yawDeg(), without.a.gyroBiasDps(2), without.a.yawDeg());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE(with.a.compassUsed());
    TEST_ASSERT_FALSE(without.a.compassUsed());
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, 1.0f, after10, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, 1.0f, with.a.gyroBiasDps(2), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(4.0f, 0.0f, with.a.yawDeg(), line);          // стоит (ошибка компаса ±4.5°)
    // без компаса: курс уплыл на 30°, смещение по рысканью — ноль
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 0.0f, without.a.gyroBiasDps(2), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, 30.0f, without.a.yawDeg(), line);
    TEST_ASSERT_EQUAL_UINT32(0, with.a.compassRejected());
    TEST_ASSERT_TRUE(covariancesOk(with.a));
}

// Из компаса берётся только курс, и усиление — только у рысканья и смещения
// гироскопа по оси Z. Два фильтра на одном и том же полёте: один с компасом,
// который врёт (шум, сдвиги на 8°, скачки на 50°), другой без компаса.
//   • В ровном висении крен, тангаж и их смещения совпадают до тысячных:
//     прямого пути от компаса к горизонту нет.
//   • В манёврах остаётся косвенный: сдвиг компаса на 8° (в пределах шума — он
//     проходит) качает оценку смещения по Z на ~0.25 °/с, а на наклоне ось Z
//     рамы уже не вертикальна. Это доли градуса — и это же смещение, найденное
//     верно, убирает градусы (тест выше и test_copter_sim).
void test_attitude_ekf_compass_never_touches_the_horizon()
{
    for (int manoeuvres = 0; manoeuvres < 2; ++manoeuvres)
    {
        PointCopter c1, c2;
        c1.holdPlace = c2.holdPlace = !manoeuvres;
        Fed with, without;
        with.compass = true;
        double worstRoll = 0.0, worstPitch = 0.0, worstBias = 0.0, worstYaw = 0.0;
        for (int i = 0; i < 60000; ++i)
        {
            const int leg = (i / 5000) % 4;
            with.compassErrorDeg = (leg == 1 ? 8.0 : leg == 3 ? 50.0 : 0.0) + 3.0 * std::sin(i * 0.004);
            if (manoeuvres)
            {
                for (PointCopter* c : { &c1, &c2 })
                {
                    c->rollT = leg == 0 ? 0.3 : leg == 2 ? -0.2 : 0.0;
                    c->pitchT = leg == 1 ? -0.3 : 0.0;
                }
            }
            with.step(c1);
            without.step(c2);
            worstRoll = std::fmax(worstRoll, std::fabs(with.a.rollDeg() - without.a.rollDeg()));
            worstPitch = std::fmax(worstPitch, std::fabs(with.a.pitchDeg() - without.a.pitchDeg()));
            worstBias = std::fmax(worstBias, std::fabs(with.a.gyroBiasDps(0) - without.a.gyroBiasDps(0)));
            worstBias = std::fmax(worstBias, std::fabs(with.a.gyroBiasDps(1) - without.a.gyroBiasDps(1)));
            worstYaw = std::fmax(worstYaw, std::fabs(with.a.yawDeg() - without.a.yawDeg()));
        }
        char line[240];
        snprintf(line, sizeof(line),
                 "lying compass against no compass, 60 s %s: yaw differs by up to %.1f deg, roll by %.4f, pitch by %.4f deg, roll/pitch bias by %.4f deg/s",
                 manoeuvres ? "of manoeuvres" : "of hover", worstYaw, worstRoll, worstPitch, worstBias);
        TEST_MESSAGE(line);
        TEST_ASSERT_TRUE_MESSAGE(worstYaw > 4.0, line);       // компас рысканье двигал
        TEST_ASSERT_TRUE_MESSAGE(with.a.compassRejected() > 100, line);
        if (!manoeuvres)
        {
            TEST_ASSERT_TRUE_MESSAGE(worstRoll < 0.005 && worstPitch < 0.005, line);
            TEST_ASSERT_TRUE_MESSAGE(worstBias < 0.002, line);
        }
        else
        {
            TEST_ASSERT_TRUE_MESSAGE(worstRoll < 1.0 && worstPitch < 1.0, line);
            TEST_ASSERT_TRUE_MESSAGE(worstBias < 0.15, line);
        }
    }
}

// Помеха от моторов — скачок курса компаса на десятки градусов: проверка
// невязки по χ² отбрасывает такие отсчёты, рысканье и смещение не уходят.
// Помеха в пределах шума компаса проходит, но и она горизонт не двигает.
void test_attitude_ekf_compass_interference_is_gated()
{
    PointCopter c;
    c.holdPlace = true;
    Fed f;
    f.compass = true;
    f.gps = false;
    for (int i = 0; i < 20000; ++i) f.step(c);
    TEST_ASSERT_EQUAL_UINT32(0, f.a.compassRejected());
    const float yaw0 = f.a.yawDeg(), bias0 = f.a.gyroBiasDps(2), roll0 = f.a.rollDeg(), pitch0 = f.a.pitchDeg();

    // 10 с помехи +40° (газ в пол рядом с силовыми проводами)
    f.compassErrorDeg = 40.0;
    for (int i = 0; i < 10000; ++i) f.step(c);
    char line[200];
    snprintf(line, sizeof(line), "compass off by 40 deg for 10 s: rejected %u samples, yaw moved %.2f deg, yaw bias %.3f deg/s, NIS %.0f",
             static_cast<unsigned>(f.a.compassRejected()), f.a.yawDeg() - yaw0, f.a.gyroBiasDps(2) - bias0, f.a.compassNis());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(f.a.compassRejected() >= 240, line);   // 25 Гц × 10 с, все
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, yaw0, f.a.yawDeg(), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, bias0, f.a.gyroBiasDps(2), line);
    TEST_ASSERT_TRUE_MESSAGE(f.a.compassNis() > 9.0f, line);

    // помеха ушла — компас снова в деле, без отброшенных
    f.compassErrorDeg = 0.0;
    const uint32_t rejected = f.a.compassRejected();
    for (int i = 0; i < 5000; ++i) f.step(c);
    TEST_ASSERT_EQUAL_UINT32(rejected, f.a.compassRejected());

    // +8° — в пределах шума компаса: проходит, рысканье идёт за ним (медленно),
    // крен и тангаж — на месте
    f.compassErrorDeg = 8.0;
    for (int i = 0; i < 10000; ++i) f.step(c);
    snprintf(line, sizeof(line), "compass off by 8 deg for 10 s: yaw moved %.2f deg, yaw bias %.3f deg/s, roll %.4f pitch %.4f",
             f.a.yawDeg() - yaw0, f.a.gyroBiasDps(2) - bias0, f.a.rollDeg() - roll0, f.a.pitchDeg() - pitch0);
    TEST_MESSAGE(line);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(rejected, f.a.compassRejected(), line);
    TEST_ASSERT_TRUE_MESSAGE(f.a.yawDeg() - yaw0 > 4.0f && f.a.yawDeg() - yaw0 < 8.5f, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, roll0, f.a.rollDeg(), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, pitch0, f.a.pitchDeg(), line);

    // компас «переехал» насовсем (+60°): пока неуверенность в рысканье мала —
    // отброшен; она растёт, и фильтр сам возвращается к компасу
    f.compassErrorDeg = 60.0;
    for (int i = 0; i < 5000; ++i) f.step(c);
    TEST_ASSERT_TRUE(f.a.compassRejected() > rejected + 100);
    TEST_ASSERT_TRUE(f.a.yawDeg() - yaw0 < 12.0f);
    for (int i = 0; i < 900000 && f.a.yawDeg() - yaw0 < 50.0f; ++i) f.step(c);
    TEST_ASSERT_TRUE(f.a.yawDeg() - yaw0 >= 50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, roll0, f.a.rollDeg());
}

// Компаса долго не было (отвалился, не было веры): рысканье уплыло на сотни
// градусов. Невязка, свёрнутая в ±180°, для смещения ничего не значит —
// опора берётся заново: ни скачка рысканья, ни толчка смещению не в ту
// сторону; дальше смещение находится как обычно.
void test_attitude_ekf_compass_returns_after_long_outage()
{
    PointCopter c;
    c.holdPlace = true;
    Fed f;
    f.compass = true;
    f.gps = false;
    for (int i = 0; i < 20000; ++i) f.step(c);
    TEST_ASSERT_TRUE(f.a.compassUsed());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, f.a.gyroBiasDps(2));

    // 400 с без компаса; нуль гироскопа по рысканью тем временем ушёл на 0.7 °/с:
    // рысканье фильтра уплыло на 280° — «минус 80°», если свернуть
    f.compass = false;
    c.biasDps[2] = 0.7;
    for (int i = 0; i < 400000; ++i) f.step(c);
    const float yawBefore = f.a.yawDeg(), biasBefore = f.a.gyroBiasDps(2);
    TEST_ASSERT_TRUE(f.a.covariance(2, 2) > 0.3f);

    f.compass = true;
    for (int i = 0; i < 1000; ++i) f.step(c);
    float moved = f.a.yawDeg() - yawBefore - 0.7f;   // за секунду гироскоп сам ушёл на 0.7°
    while (moved > 180.0f) moved -= 360.0f;
    while (moved < -180.0f) moved += 360.0f;
    char line[200];
    snprintf(line, sizeof(line), "compass back after 400 s (yaw drifted 280 deg): yaw jumped by %.2f deg, yaw bias %.3f -> %.3f deg/s", moved,
             biasBefore, f.a.gyroBiasDps(2));
    TEST_MESSAGE(line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, 0.0f, moved, line);
    TEST_ASSERT_TRUE_MESSAGE(f.a.gyroBiasDps(2) > biasBefore - 0.03f, line);   // не в минус
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, f.a.compassRejected(), line);
    // и за полминуты смещение найдено
    for (int i = 0; i < 30000; ++i) f.step(c);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.7f, f.a.gyroBiasDps(2));
    TEST_ASSERT_TRUE(covariancesOk(f.a));
}

// Чего не видно — то стоит: без компаса дисперсия смещения по рысканью не
// растёт выше начальной (и оценка — ноль), дисперсия рысканья упирается в
// потолок, а не уходит в бесконечность; блок сопротивления без GPS — как
// при включении. Пять минут ровного висения.
void test_attitude_ekf_unobservable_states_do_not_drift()
{
    AttitudeQuat a;
    a.setEkf(true);
    const float gyro[3] = { 0, 0, 0 }, level[3] = { 0, 0, 1 };
    a.update(gyro, level, 0.001f);
    a.setGrounded(false);
    const float biasVar0 = a.covariance(5, 5);
    TEST_ASSERT_TRUE(biasVar0 > 0.0f);
    float worstBiasVar = 0.0f, worstYawVar = 0.0f;
    for (int i = 0; i < 300000; ++i)
    {
        a.update(gyro, level, 0.001f);
        worstBiasVar = fmaxf(worstBiasVar, a.covariance(5, 5));
        worstYawVar = fmaxf(worstYawVar, a.covariance(2, 2));
    }
    char line[160];
    snprintf(line, sizeof(line), "5 min hover without compass: yaw bias sigma %.3f deg/s (start %.3f), yaw sigma %.0f deg",
             sqrtf(a.covariance(5, 5)) * 57.29578f, sqrtf(biasVar0) * 57.29578f, sqrtf(a.covariance(2, 2)) * 57.29578f);
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE_MESSAGE(worstBiasVar <= biasVar0 * 1.0001f, line);
    TEST_ASSERT_TRUE_MESSAGE(a.covariance(5, 5) >= biasVar0 * 0.999f, line);   // и не падает: информации нет
    TEST_ASSERT_TRUE_MESSAGE(worstYawVar <= 10.0f * 1.0001f, line);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a.gyroBiasDps(2));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, a.yawDeg());
    TEST_ASSERT_EQUAL_FLOAT(0.3f, a.dragCoef());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, a.dragCovariance(0, 0));
    TEST_ASSERT_FALSE(a.compassUsed());
    // крен и тангаж при этом наблюдаемы: их неуверенность — установившаяся
    TEST_ASSERT_TRUE(a.tiltSigmaDeg() < 2.0f);
}

// Числа на float: десять минут полёта со всем сразу — манёвры, флипы с
// ошибкой масштаба гироскопа (большие поправки: доворот ковариации), смещение
// гироскопа, ветер, скачки компаса, пропадающий GPS. Ковариации остаются
// ковариациями: симметричны, конечны, положительно определены; оценки не
// расходятся.
void test_attitude_ekf_covariance_stays_valid_in_float()
{
    PointCopter c;
    c.k = 0.35;
    c.windN = 2.0;
    c.windE = 1.0;
    c.biasDps[0] = 1.5;
    c.biasDps[1] = -1.0;
    c.biasDps[2] = 0.7;
    c.gyroScale = 1.01;
    c.setYawDeg(-120.0);
    Fed f;
    f.compass = true;
    uint32_t seed = 2026;
    auto random = [&seed]() {
        seed = seed * 1103515245u + 12345u;
        return static_cast<double>((seed >> 8) & 0xFFFF) / 65536.0;
    };
    long bad = 0;
    double worstAfterFlip = 0.0, worstLater = 0.0;
    for (long i = 0; i < 600000; ++i)
    {
        const long t = i % 60000;   // минута: манёвры, флип, тишина без GPS
        if (i % 4000 == 0)
        {
            c.rollT = (random() - 0.5) * 60.0 * D2R;
            c.pitchT = (random() - 0.5) * 60.0 * D2R;
            c.yawRate = random() < 0.3 ? (random() - 0.5) * 2.0 : 0.0;
            f.compassErrorDeg = random() < 0.2 ? (random() - 0.5) * 120.0 : (random() - 0.5) * 6.0;
        }
        c.flipRate = (t >= 30000 && t < 30000 + 1500) ? 2.0 * M_PI * 2.0 / 1.5 * (1.0 / 1.0) : 0.0;   // два оборота за 1.5 с
        f.gps = !(t >= 40000 && t < 52000);
        f.step(c);
        if (i % 250 == 0 && !covariancesOk(f.a)) ++bad;
        if (t == 31600) worstAfterFlip = std::fmax(worstAfterFlip, f.horizonErrorDeg(c));
        if (t == 39900) worstLater = std::fmax(worstLater, f.horizonErrorDeg(c));
    }
    char line[220];
    snprintf(line, sizeof(line),
             "10 min of everything: bad covariance checks %ld, horizon error right after flips up to %.2f deg, 8 s later %.2f, at the end %.2f deg; "
             "k %.3f (true 0.35), gyro bias %.2f %.2f %.2f",
             bad, worstAfterFlip, worstLater, f.horizonErrorDeg(c), f.a.dragCoef(), f.a.gyroBiasDps(0), f.a.gyroBiasDps(1), f.a.gyroBiasDps(2));
    TEST_MESSAGE(line);
    TEST_ASSERT_EQUAL_INT64_MESSAGE(0, bad, line);
    TEST_ASSERT_TRUE_MESSAGE(f.a.qw() == f.a.qw(), line);
    // 1 % от 720° — 7° сразу после флипов (их даёт гироскоп); дальше ошибка
    // убирается, а не растёт
    TEST_ASSERT_TRUE_MESSAGE(worstAfterFlip < 11.0, line);
    TEST_ASSERT_TRUE_MESSAGE(worstLater < 0.6 * worstAfterFlip, line);
    TEST_ASSERT_TRUE_MESSAGE(f.horizonErrorDeg(c) < 3.0, line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 0.35f, f.a.dragCoef(), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, 1.5f, f.a.gyroBiasDps(0), line);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.3f, -1.0f, f.a.gyroBiasDps(1), line);
}
