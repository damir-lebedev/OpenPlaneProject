#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"

// ============================================================
// Ориентация мультикоптера: кватернион + поправка по акселерометру.
// Два фильтра на одном кватернионе: расширенный фильтр Калмана (MEKF,
// основной) и фильтр Махони (запасной, `attitude_filter = MAHONY`).
//
// Самолётный AttitudeEstimator складывает угловые скорости прямо с
// углами Эйлера — для самолёта, который не крутит сальто, этого
// хватает. Коптер в акро делает флипы и висит на любом крене, поэтому
// здесь — кватернион: повороты складываются точно при любой
// ориентации, без «замка» на тангаже 90°.
//
// Акселерометр на коптере — плохая опора: тяга всегда вдоль оси Z
// рамы, и при разгоне он «видит» горизонт там, где его нет. Верен он
// только в среднем (в установившемся полёте сопротивление уравновешивает
// тягу). Поэтому поправка медленная — постоянная времени ~2 с в полёте
// (на земле до взлёта — 0.2 с, чтобы быстро встать в горизонт), и
// выключается совсем, когда модуль ускорения далёк от 1 g (вираж,
// рывок газом, удар).
//
// Разгон с удержанием высоты этим не поймать: тяга подстраивается, модуль
// остаётся около 1 g, а «верх» акселерометра наклонён меньше настоящего —
// поправка тянула горизонт, контроллер доворачивал, и за 3 с разгона
// реальный наклон уходил на 10° дальше заданного (а вместе с ним врала
// и высота — P32). Поэтому в полёте поправка ещё и гаснет: при
// горизонтальном ускорении по GPS (GpsAccel — курс не нужен, только
// модуль) или, без GPS, при большом наклоне.
//
// Оси — авиационные, как во всей прошивке: гироскоп — крен (+ правый
// борт вниз), тангаж (+ нос вверх), рысканье (+ нос вправо), °/с;
// акселерометр — g, X к носу, Y влево, Z вверх. Внутри — связанная
// система FRD (X вперёд, Y вправо, Z вниз): угловые скорости в ней
// совпадают с авиационными знаками.
//
// ---- MEKF (мультипликативный расширенный фильтр Калмана) ----
//
// Махони — поправка с постоянным усилением и без памяти о гироскопе:
// смещение нуля гироскопа b он не убирает, а терпит — горизонт стоит с
// ошибкой b/k (2 °/с при k = 0.5 1/с в полёте — 4°), и чем слабее вера
// акселерометру, тем ошибка больше. MEKF оценивает само смещение.
//
// Состояние ошибки — 6 чисел: δθ (поворот оценки относительно истины, в
// земных осях, рад) и δb (ошибка смещения гироскопа, рад/с, оси рамы).
// Сам кватернион q и смещение b хранятся отдельно; после каждой поправки
// ошибка «вливается» в них и снова равна нулю (поэтому «мультипликативный»:
// кватернион не складывается с ошибкой, а умножается на малый поворот).
//
//   модель:     δθ' = −R·(δb + шум гироскопа),  δb' = шум ухода нуля;
//               Φ = [[I, −R·T], [0, I]],  P ← Φ·P·Φᵀ + Q
//   измерение:  горизонтальная удельная сила в земных осях z = (R·a)xy —
//               это горизонтальное ускорение коптера (тяжесть вертикальна) —
//               против ожидаемого f:
//                 • есть GPS и курс — f = ускорение по GPS (вираж, разгон,
//                   торможение: центростремительное и любое другое);
//                 • нет — по модели: тяга идёт точно по оси рамы, сопротивление
//                   винтов лежит в их плоскости и равно −c·v (v — скорость
//                   относительно воздуха в этой плоскости; считается из той же
//                   модели: v' = тяга_гориз + сопротивление). Это известное
//                   свойство мультикоптера: акселерометр по осям X и Y рамы
//                   меряет именно сопротивление винтов (Martin, Salaün, «The
//                   true role of accelerometer feedback in quadrotor control»,
//                   ICRA 2010);
//               z.x − f.x = h·δθy,  z.y − f.y = −h·δθx  (h — вертикальная
//               удельная сила, в висении 1 g) — два скалярных шага Калмана,
//               обращать матрицы не нужно.
//
// Ошибка — в земных осях, поэтому ненаблюдаемое рысканье (вокруг вертикали)
// всегда одна и та же строка P, и матрицу не надо поворачивать вместе с рамой.
//
// Почему модель, а не просто «верх = куда показывает акселерометр»: на
// коптере акселерометр показывает тягу, а она всегда по оси рамы — при любом
// наклоне он «видит ровно», пока сопротивление воздуха не уравновесит тягу
// (секунды). Простой фильтр на наклоне тянет горизонт к «ровно», а после
// разгона, когда коптер уже ровно и тормозится о воздух, — в другую сторону.
// С моделью поправка на манёвре почти нулевая (остаётся ошибка в c), а
// настоящая ошибка горизонта проявляется как положено: коптер с кривым
// горизонтом разгоняется, и сопротивление расходится с ожидаемым.
//
// Доверие акселерометру — шум измерения σ (в g ≈ рад) растёт:
//   • с угловой скоростью (флип: рычаг до центра масс, масштаб гироскопа);
//   • с ускорением по GPS (опоздание фикса, ошибка курса) или, без GPS, со
//     скоростью по модели (c известен примерно); GPS без курса — с разницей
//     между его модулем ускорения и моделью;
//   • с модулем далеко от 1 g (удар, зашкал).
// Усиление получается само: P растёт, пока акселерометру веры нет (и с
// угловой скоростью — масштаб гироскопа неточен на ~1 %), после манёвра
// поправка быстрая и затихает. В спокойном висении установившиеся усиления —
// 0.25 1/с по углу и 0.01 1/с² по смещению: без GPS ошибка видна через
// скорость с опозданием 1/c ≈ 3–4 с, и усиление по углу выше ~0.3 1/с такой
// контур раскачивает (условие устойчивости: Kθ > Kb/c).
//
// Цена: ковариация и два шага — ~240 умножений, ~210 сложений, 4 корня,
// 3 деления за шаг; шаг не чаще 1 кГц (цикл быстрее — копит время).
// На STM32F405 (168 МГц, FPU, ~3.5 такта на операцию с загрузкой) —
// около 10 мкс. Память: 36 + 3 float сверх кватерниона. Динамической нет.
// ============================================================

namespace copter
{
    struct Vec3
    {
        float x = 0, y = 0, z = 0;
    };

    inline Vec3 cross(const Vec3& a, const Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float length(const Vec3& a) { return sqrtf(dot(a, a)); }

    class AttitudeQuat
    {
    public:
        // Постоянные времени поправки по акселерометру, с.
        void configure(float flightTauS, float groundTauS)
        {
            kFlight = flightTauS > 0.0f ? 1.0f / flightTauS : 0.0f;
            kGround = groundTauS > 0.0f ? 1.0f / groundTauS : 0.0f;
        }

        // Фильтр: MEKF (true) или Махони. Переключение на ходу — без скачка:
        // кватернион общий, ковариация начинается заново.
        void setEkf(bool on)
        {
            if (on && !ekf) ekfReset();
            ekf = on;
        }
        bool ekfEnabled() const { return ekf; }

        void reset()
        {
            w = 1.0f;
            x = y = z = 0.0f;
            initialized = false;
            ekfReset();
        }

        // На земле до взлёта поправка быстрая.
        void setGrounded(bool g) { grounded = g; }

        // Горизонтальное ускорение по GPS, м/с²; < 0 — GPS нет (тогда — по наклону).
        void setGroundAccel(float ms2)
        {
            groundAccel = ms2;
            if (ms2 < 0.0f) accelVector = false;
        }

        // То же вектором (север, восток) — когда известен курс носа:
        // headingMinusYawDeg — курс минус рысканье этого фильтра. MEKF вычитает
        // его из акселерометра (вираж, разгон); Махони берёт только модуль.
        void setGroundAccelVector(float northMs2, float eastMs2, float headingMinusYawDeg)
        {
            const float d = headingMinusYawDeg * PI_F / 180.0f;
            const float c = cosf(d), sn = sinf(d);
            // земные оси фильтра повёрнуты от настоящих на (курс − рысканье)
            accelX = c * northMs2 + sn * eastMs2;
            accelY = -sn * northMs2 + c * eastMs2;
            accelVector = true;
        }
        void clearGroundAccelVector() { accelVector = false; }

        // Доля поправки в полёте по ускорению GPS или по наклону, 0…1.
        float flightTrust() const
        {
            if (groundAccel >= 0.0f) return clamp01(1.0f - (groundAccel - GPS_ACCEL_OK) / GPS_ACCEL_SPAN);
            return clamp01(1.0f - (tiltDeg() - TILT_OK_DEG) / TILT_SPAN_DEG);
        }

        // gyroDps — крен/тангаж/рысканье, °/с; accelG — X нос, Y влево, Z вверх.
        void update(const float gyroDps[3], const float accelG[3], float dtS)
        {
            const Vec3 a = { accelG[0], -accelG[1], -accelG[2] };   // FRD
            const float aLen = length(a);
            if (!initialized)
            {
                // первый годный отсчёт акселерометра (сразу после включения чип
                // может отдать нули): без него ориентации нет — и ARM нет
                if (aLen > 0.5f)
                {
                    levelFrom(a);
                    initialized = true;
                }
                return;
            }

            const float d2r = PI_F / 180.0f;
            float p = gyroDps[0] * d2r, q = gyroDps[1] * d2r, r = gyroDps[2] * d2r;

            if (ekf)
            {
                p -= bias.x;
                q -= bias.y;
                r -= bias.z;
                integrate(p, q, r, dtS);
                ekfTime += dtS;
                if (ekfTime >= EKF_STEP_S * 0.999f)
                {
                    ekfStep(a, aLen, sqrtf(p * p + q * q + r * r), ekfTime);
                    ekfTime = 0.0f;
                }
                return;
            }

            // поправка: измеренный «верх» (удельная сила) против оценки. В полёте
            // акселерометр видит направление тяги — с тяжестью оно совпадает
            // только в спокойном полёте: не при рывке (модуль не 1 g), не во
            // вращении (флип на газе висения — ровно 1 g, но «вверх» крутится
            // вместе с рамой) и не тогда, когда расходится с гироскопом на
            // десятки градусов (так далеко гироскоп не уплывает).
            float trust = fmaxf(0.0f, 1.0f - fabsf(aLen - 1.0f) / 0.15f);
            if (!grounded)
            {
                const float spinDps = sqrtf(gyroDps[0] * gyroDps[0] + gyroDps[1] * gyroDps[1] + gyroDps[2] * gyroDps[2]);
                trust *= fminf(1.0f, fmaxf(0.0f, 1.0f - (spinDps - 30.0f) / 70.0f));
                trust *= flightTrust();
            }
            const float k = (grounded ? kGround : kFlight) * trust;
            if (k > 0.0f && aLen > 0.1f)
            {
                const Vec3 measured = { a.x / aLen, a.y / aLen, a.z / aLen };
                const Vec3 estimated = upBody();
                const float gain = (!grounded && dot(measured, estimated) < COS_25) ? 0.0f : k;
                const Vec3 e = cross(measured, estimated);
                p += gain * e.x;
                q += gain * e.y;
                r += gain * e.z;
            }

            integrate(p, q, r, dtS);
        }

        // ---- что MEKF знает о себе (Махони — нули) ----

        // Смещение нуля гироскопа, °/с: крен, тангаж, рысканье.
        float gyroBiasDps(uint8_t axis) const
        {
            const float b = axis == 0 ? bias.x : axis == 1 ? bias.y : bias.z;
            return ekf ? b * 180.0f / PI_F : 0.0f;
        }
        // Неуверенность в горизонте, ° (1σ, худшая из двух осей).
        float tiltSigmaDeg() const { return ekf ? sqrtf(fmaxf(P[0][0], P[1][1])) * 180.0f / PI_F : 0.0f; }
        // Шум, приписанный акселерометру на последнем шаге, g (≈ рад): чем больше — тем меньше веры.
        float accelSigma() const { return ekf ? lastSigma : 0.0f; }
        // Вера акселерометру, 0…1: 1 — спокойно висит или стоит, 0 — манёвр (поправки почти нет).
        float accelTrust() const
        {
            if (!ekf || lastSigma <= 0.0f) return 0.0f;
            const float base = grounded ? EKF_SIGMA_GROUND : EKF_SIGMA_FLIGHT;
            return clamp01(base * base / (lastSigma * lastSigma));
        }

        // Направление «вниз» Земли в осях рамы (FRD), единичный вектор.
        Vec3 downBody() const
        {
            return { 2.0f * (x * z - w * y), 2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y) };
        }

        Vec3 upBody() const
        {
            const Vec3 d = downBody();
            return { -d.x, -d.y, -d.z };
        }

        // Ось Z рамы (вниз), спроецированная на вертикаль: 1 — ровно, 0 — на боку, −1 — вверх ногами.
        float cosTilt() const { return 1.0f - 2.0f * (x * x + y * y); }
        float tiltDeg() const { return acosf(fminf(1.0f, fmaxf(-1.0f, cosTilt()))) * 180.0f / PI_F; }

        float rollDeg() const { return atan2f(2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y)) * 180.0f / PI_F; }
        float pitchDeg() const
        {
            const float s = fminf(1.0f, fmaxf(-1.0f, 2.0f * (w * y - x * z)));
            return asinf(s) * 180.0f / PI_F;
        }
        float yawDeg() const { return atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / PI_F; }

        // Вектор из рамы (FRD) в земные оси (NED).
        Vec3 toEarth(const Vec3& v) const
        {
            return {
                (1.0f - 2.0f * (y * y + z * z)) * v.x + 2.0f * (x * y - w * z) * v.y + 2.0f * (x * z + w * y) * v.z,
                2.0f * (x * y + w * z) * v.x + (1.0f - 2.0f * (x * x + z * z)) * v.y + 2.0f * (y * z - w * x) * v.z,
                2.0f * (x * z - w * y) * v.x + 2.0f * (y * z + w * x) * v.y + (1.0f - 2.0f * (x * x + y * y)) * v.z,
            };
        }

        // Вертикальное ускорение, м/с² (+ вверх), из показаний акселерометра (g).
        float verticalAccel(const float accelG[3]) const
        {
            const Vec3 f = toEarth({ accelG[0], -accelG[1], -accelG[2] });
            return (-f.z - 1.0f) * 9.80665f;
        }

        bool isInitialized() const { return initialized; }

        float qw() const { return w; }
        float qx() const { return x; }
        float qy() const { return y; }
        float qz() const { return z; }

    private:
        // q̇ = ½·q ⊗ (0, ω), ω — рад/с в осях рамы.
        void integrate(float p, float q, float r, float dtS)
        {
            const float hw = 0.5f * dtS;
            const float nw = w + hw * (-x * p - y * q - z * r);
            const float nx = x + hw * (w * p + y * r - z * q);
            const float ny = y + hw * (w * q - x * r + z * p);
            const float nz = z + hw * (w * r + x * q - y * p);
            const float n = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
            w = nw / n;
            x = nx / n;
            y = ny / n;
            z = nz / n;
        }

        void ekfReset()
        {
            for (auto& row : P)
            {
                for (float& v : row) v = 0.0f;
            }
            P[0][0] = P[1][1] = EKF_TILT_VAR0;
            P[3][3] = P[4][4] = P[5][5] = EKF_BIAS_VAR0;
            bias = Vec3();
            airX = airY = 0.0f;
            ekfTime = 0.0f;
            lastSigma = 0.0f;
        }

        // Шаг MEKF: a — удельная сила (g, оси рамы FRD), spin — модуль угловой
        // скорости без смещения (рад/с), T — время с прошлого шага.
        void ekfStep(const Vec3& a, float aLen, float spin, float T)
        {
            // матрица поворота рама -> земля
            const float R[3][3] = {
                { 1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z), 2.0f * (x * z + w * y) },
                { 2.0f * (x * y + w * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x) },
                { 2.0f * (x * z - w * y), 2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y) },
            };

            // ---- прогноз ковариации: A = Pθθ, B = Pθb, C = Pbb ----
            // A ← A − T·(B·Rᵀ + R·Bᵀ) + qθ·T;  B ← B − T·R·C;  C ← C + qb·T
            // (член T²·R·C·Rᵀ на три порядка меньше qθ·T — опущен)
            float BRt[3][3], RC[3][3];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    BRt[i][j] = P[i][3] * R[j][0] + P[i][4] * R[j][1] + P[i][5] * R[j][2];
                    RC[i][j] = R[i][0] * P[3][3 + j] + R[i][1] * P[4][3 + j] + R[i][2] * P[5][3 + j];
                }
            }
            // гироскоп: шум и вибрация + ошибка масштаба на быстром вращении
            const float scale = EKF_GYRO_SCALE * spin;
            const float qTheta = ((grounded ? EKF_Q_ANGLE_GROUND : EKF_Q_ANGLE) + scale * scale * EKF_SCALE_TAU_S) * T;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = i; j < 3; ++j)
                {
                    P[i][j] -= T * (BRt[i][j] + BRt[j][i]);
                    P[j][i] = P[i][j];
                }
                P[i][i] += qTheta;
                for (int j = 0; j < 3; ++j)
                {
                    P[i][3 + j] -= T * RC[i][j];
                    P[3 + j][i] = P[i][3 + j];
                }
                P[3 + i][3 + i] += EKF_Q_BIAS * T;
            }
            // рысканье акселерометр не видит: дисперсия растёт без конца — потолок
            if (P[2][2] > EKF_YAW_VAR_MAX)
            {
                const float s = sqrtf(EKF_YAW_VAR_MAX / P[2][2]);
                for (int k = 0; k < 6; ++k)
                {
                    P[2][k] *= s;
                    P[k][2] *= s;
                }
            }

            // ---- скорость относительно воздуха по модели ----
            // Тяга идёт точно по оси рамы: в земных осях её горизонтальная часть
            // разгоняет коптер. Сопротивление винтов лежит в их плоскости и
            // пропорционально скорости в ней: в осях рамы −c·(vx, vy, 0).
            const float thrustX = 9.80665f * a.z * R[0][2], thrustY = 9.80665f * a.z * R[1][2];   // м/с²
            // ожидаемое сопротивление в осях рамы, g
            const float dragBx = -EKF_DRAG * (R[0][0] * airX + R[1][0] * airY) / 9.80665f;
            const float dragBy = -EKF_DRAG * (R[0][1] * airX + R[1][1] * airY) / 9.80665f;
            if (grounded)
            {
                airX = airY = 0.0f;
            }
            else
            {
                airX += (thrustX + 9.80665f * (R[0][0] * dragBx + R[0][1] * dragBy)) * T;
                airY += (thrustY + 9.80665f * (R[1][0] * dragBx + R[1][1] * dragBy)) * T;
            }

            // ---- измерение ----
            const float k = grounded ? kGround : kFlight;   // 0 — без акселерометра (только гироскоп)
            if (k <= 0.0f || aLen < 0.1f) return;

            // удельная сила в земных осях, g; её горизонтальная часть — это
            // горизонтальное ускорение коптера (тяжесть вертикальна)
            const float zx = R[0][0] * a.x + R[0][1] * a.y + R[0][2] * a.z;
            const float zy = R[1][0] * a.x + R[1][1] * a.y + R[1][2] * a.z;
            const float h = -(R[2][0] * a.x + R[2][1] * a.y + R[2][2] * a.z);   // вверх; в висении 1
            if (h < EKF_MIN_LIFT)
            {
                // в полёте: падает или летит вверх ногами — опоры нет. На земле
                // акселерометр — чистая тяжесть, и «верх» в другой стороне от
                // оценки значит, что ориентация потеряна (кувырок с зашкалом
                // гироскопа): заново по акселерометру, курс сохраняется
                if (grounded && fabsf(aLen - 1.0f) < 0.2f)
                {
                    levelFrom(a, atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z)));
                    P[0][0] = P[1][1] = EKF_TILT_VAR0;
                    P[0][1] = P[1][0] = 0.0f;
                }
                return;
            }

            // ожидаемое горизонтальное ускорение, g
            float fx = 0.0f, fy = 0.0f;
            float sigma = grounded ? EKF_SIGMA_GROUND : EKF_SIGMA_FLIGHT;
            sigma += EKF_K_MODULE * fmaxf(0.0f, fabsf(aLen - 1.0f) - EKF_MODULE_NOISE);
            if (!grounded)
            {
                sigma += EKF_K_SPIN * spin;
                if (accelVector)
                {
                    // ускорение известно по GPS: ошибка горизонта видна сразу;
                    // не верим доле самого ускорения — опоздание фикса, ошибка курса
                    fx = accelX / 9.80665f;
                    fy = accelY / 9.80665f;
                    sigma += EKF_K_GPS_RESIDUAL * sqrtf(fx * fx + fy * fy);
                }
                else
                {
                    // по модели: тяга по оси рамы плюс сопротивление. Акселерометр
                    // тогда не «видит ровно» на любом наклоне (тяга всегда по оси
                    // рамы), а сравнивает сопротивление с ожидаемым: ошибка горизонта
                    // проявляется через скорость, с опозданием 1/c секунд.
                    // Коэффициент c известен примерно — не верим доле сопротивления
                    fx = thrustX / 9.80665f + R[0][0] * dragBx + R[0][1] * dragBy;
                    fy = thrustY / 9.80665f + R[1][0] * dragBx + R[1][1] * dragBy;
                    sigma += EKF_K_DRAG * sqrtf(dragBx * dragBx + dragBy * dragBy);
                    // GPS без курса: модуль ускорения должен сойтись с моделью
                    if (groundAccel >= 0.0f) sigma += fabsf(groundAccel / 9.80665f - sqrtf(fx * fx + fy * fy));
                }
            }
            lastSigma = sigma;
            // шум акселерометра «цветной» (манёвр длится доли секунды): его
            // плотность σ²·τ, на шаг длиной T — дисперсия σ²·τ/T
            const float Rk = sigma * sigma * EKF_NOISE_TAU_S / T;

            const float nuX = zx - fx, nuY = zy - fy;
            // в полёте расхождение больше 25° (и больше 4σ) — не опора:
            // так далеко гироскоп не уплывает
            const float gate = fmaxf(SIN_25 * SIN_25, 16.0f * (P[0][0] + P[1][1])) * h * h;
            if (!grounded && nuX * nuX + nuY * nuY > gate) return;

            float dx[6] = {};
            scalarUpdate(1, h, nuX, Rk, dx);    // z.x − f.x = h·δθy
            scalarUpdate(0, -h, nuY, Rk, dx);   // z.y − f.y = −h·δθx

            // ошибка -> в кватернион (малый поворот слева: земные оси) и в смещение
            const float hx = 0.5f * dx[0], hy = 0.5f * dx[1], hz = 0.5f * dx[2];
            const float nw = w - hx * x - hy * y - hz * z;
            const float nx = x + hx * w + hy * z - hz * y;
            const float ny = y + hy * w + hz * x - hx * z;
            const float nz = z + hz * w + hx * y - hy * x;
            const float n = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
            w = nw / n;
            x = nx / n;
            y = ny / n;
            z = nz / n;
            bias.x = fminf(EKF_BIAS_MAX, fmaxf(-EKF_BIAS_MAX, bias.x + dx[3]));
            bias.y = fminf(EKF_BIAS_MAX, fmaxf(-EKF_BIAS_MAX, bias.y + dx[4]));
            bias.z = fminf(EKF_BIAS_MAX, fmaxf(-EKF_BIAS_MAX, bias.z + dx[5]));
        }

        // Один скалярный шаг Калмана: измерение nu = h·δx[j] + шум (дисперсия Rk).
        // dx — накопленная поправка состояния (второй шаг учитывает первый).
        void scalarUpdate(int j, float h, float nu, float Rk, float dx[6])
        {
            const float s = h * h * P[j][j] + Rk;
            if (s <= 0.0f) return;
            float K[6];
            for (int i = 0; i < 6; ++i) K[i] = h * P[i][j] / s;
            const float innovation = nu - h * dx[j];
            for (int i = 0; i < 6; ++i) dx[i] += K[i] * innovation;
            // P ← P − K·s·Kᵀ (симметрично)
            for (int i = 0; i < 6; ++i)
            {
                for (int c = i; c < 6; ++c)
                {
                    P[i][c] -= K[i] * s * K[c];
                    P[c][i] = P[i][c];
                }
            }
        }

        // Ориентация сразу по акселерометру; рысканье — заданное (в начале 0).
        void levelFrom(const Vec3& a, float yawRad = 0.0f)
        {
            const float roll = atan2f(-a.y, -a.z);
            const float pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z));
            const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
            const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
            const float cy = cosf(yawRad * 0.5f), sy = sinf(yawRad * 0.5f);
            w = cr * cp * cy + sr * sp * sy;
            x = sr * cp * cy - cr * sp * sy;
            y = cr * sp * cy + sr * cp * sy;
            z = cr * cp * sy - sr * sp * cy;
        }

        static float clamp01(float v) { return fminf(1.0f, fmaxf(0.0f, v)); }

        static constexpr float COS_25 = 0.9063078f;
        static constexpr float SIN_25 = 0.4226183f;

        // ---- MEKF: шумы (обоснование — docs/COPTER.md «Фильтр ориентации») ----
        static constexpr float EKF_STEP_S = 0.001f;            // шаг ковариации и поправки не чаще
        // гироскоп: случайное блуждание угла, рад²/с. В полёте — с запасом на
        // вибрацию и наложение частот (0.57 °/√с: без поправки — 4° за минуту);
        // на земле гироскоп тих (0.11 °/√с) — и смещение видно за секунды
        static constexpr float EKF_Q_ANGLE = 1.0e-4f;
        static constexpr float EKF_Q_ANGLE_GROUND = 4.0e-6f;
        static constexpr float EKF_GYRO_SCALE = 0.01f;         // ошибка масштаба гироскопа (1 %)
        static constexpr float EKF_SCALE_TAU_S = 0.25f;        // манёвр длится столько: ошибка копится согласованно
        static constexpr float EKF_Q_BIAS = 2.5e-7f;           // уход нуля, (рад/с)²/с: 0.03 °/с за √с
        static constexpr float EKF_BIAS_MAX = 0.0873f;         // 5 °/с: больше — это не уход нуля
        static constexpr float EKF_TILT_VAR0 = 2.7e-3f;        // после включения: 3° (1σ)
        static constexpr float EKF_BIAS_VAR0 = 3.0e-4f;        // и 1 °/с
        static constexpr float EKF_YAW_VAR_MAX = 10.0f;
        // акселерометр: σ, g (≈ рад)
        static constexpr float EKF_SIGMA_FLIGHT = 0.05f;       // спокойный полёт: 3° (вибрация, сопротивление, ветер)
        static constexpr float EKF_SIGMA_GROUND = 0.003f;      // на земле: 0.2°
        static constexpr float EKF_NOISE_TAU_S = 1.0f;         // время корреляции ошибки акселерометра × 2
        static constexpr float EKF_K_MODULE = 0.5f;            // удар, зашкал: модуль 2 g — σ +0.45
        static constexpr float EKF_MODULE_NOISE = 0.1f;        // рывок газом — не в счёт: сравниваются горизонтальные части
        static constexpr float EKF_MIN_LIFT = 0.3f;            // вертикальная удельная сила меньше (g) — опоры нет
        static constexpr float EKF_K_SPIN = 0.25f;             // вращение: 100 °/с — σ +0.44
        static constexpr float EKF_K_GPS_RESIDUAL = 0.3f;      // доля ускорения GPS, которой нельзя верить (опоздание, курс)
        // сопротивление воздуха, 1/с: (м/с²) на (м/с). У винтов оно почти
        // линейно по скорости: 0.2–0.4 у коптеров 3–7". Точное не нужно —
        // даже грубое убирает большую часть ложной поправки на манёвре
        static constexpr float EKF_DRAG = 0.3f;
        static constexpr float EKF_K_DRAG = 0.5f;              // доля сопротивления, которой нельзя верить (c известен ±50 %)
        // GPS: до 0.7 м/с² — полная поправка, от 2.2 — никакой (шум скорости
        // M10 на базе 0.4 с — около 0.3 м/с²)
        static constexpr float GPS_ACCEL_OK = 0.7f, GPS_ACCEL_SPAN = 1.5f;
        // без GPS: до 15° — полная, от 35° — никакой (висение в ветер — меньше 15°)
        static constexpr float TILT_OK_DEG = 15.0f, TILT_SPAN_DEG = 20.0f;

        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
        float kFlight = 0.5f, kGround = 5.0f;
        bool grounded = true;
        bool initialized = false;
        float groundAccel = -1.0f;
        // MEKF
        bool ekf = false;
        bool accelVector = false;
        float accelX = 0.0f, accelY = 0.0f;   // ускорение по GPS в земных осях фильтра, м/с²
        float airX = 0.0f, airY = 0.0f;       // скорость относительно воздуха по модели, м/с (те же оси)
        Vec3 bias;                            // смещение гироскопа, рад/с
        float P[6][6] = {};                   // ковариация ошибки: δθ (земля), δb (рама)
        float ekfTime = 0.0f;
        float lastSigma = 0.0f;
    };

    // Горизонтальное ускорение по скорости GPS: разность скоростей на базе
    // в несколько фиксов (на соседних шум скорости делится на 0.1 с).
    class GpsAccel
    {
    public:
        static constexpr uint8_t SPAN = 5;            // фиксов в базе (0.4 с при 10 Гц)
        static constexpr float STALE_S = 0.5f;        // фикса нет дольше — GPS «нет»

        void reset()
        {
            count = 0;
            sinceFix = STALE_S + 1.0f;
            value = -1.0f;
            an = ae = 0.0f;
        }

        // Каждый цикл: fresh — пришёл новый фикс со скоростью (м/с, север/восток).
        void update(bool fresh, bool fix, float velN, float velE, float dt)
        {
            now += dt;
            sinceFix += dt;
            if (fresh && fix)
            {
                sinceFix = 0.0f;
                head = static_cast<uint8_t>((head + 1) % SPAN);
                vn[head] = velN;
                ve[head] = velE;
                t[head] = now;
                if (count < SPAN) ++count;
                if (count >= 2)
                {
                    const uint8_t old = static_cast<uint8_t>((head + SPAN - (count - 1)) % SPAN);
                    const float span = t[head] - t[old];
                    if (span > 0.05f)
                    {
                        an = (vn[head] - vn[old]) / span;
                        ae = (ve[head] - ve[old]) / span;
                        value = sqrtf(an * an + ae * ae);
                    }
                }
            }
            if (sinceFix > STALE_S) reset();
        }

        // м/с², −1 — неизвестно.
        float accel() const { return value; }
        // То же вектором: север, восток (годятся, пока accel() >= 0).
        float accelN() const { return an; }
        float accelE() const { return ae; }

    private:
        float vn[SPAN] = {}, ve[SPAN] = {}, t[SPAN] = {};
        uint8_t head = 0, count = 0;
        float now = 0.0f, sinceFix = STALE_S + 1.0f;
        float value = -1.0f;
        float an = 0.0f, ae = 0.0f;
    };
}
