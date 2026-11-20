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
// Состояние ошибки — 6 чисел (ниже — расширение: ещё сопротивление винтов
// и ветер): δθ (поворот истины относительно оценки, в земных осях, рад) и
// δb (ошибка смещения гироскопа, рад/с, оси рамы).
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
//                   винтов лежит в их плоскости и равно −c·v (c — оценка
//                   фильтра, см. «Расширение» ниже; v — скорость
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
// ---- Расширение: сопротивление винтов, ветер, компас ----
//
// Вектор ошибки — 6 + 3 числа:
//   δθ (3)  поворот истины относительно оценки, земные оси, рад:  R = Exp(δθ)·R̂
//   δb (3)  ошибка смещения гироскопа, оси рамы, рад/с:           b = b̂ + δb
//   δκ      κ = ln k,  k — сопротивление винтов, 1/с ((м/с²) на (м/с))
//   δw (2)  ветер, м/с: на север, на восток
//
// Почему ln k, а не k или 1/k. k положителен — логарифму не нужны
// ограничения; рамы различаются в разы — ошибка относительная (σ = 0.5 —
// это «±50 %» у любой рамы); и ∂(k·u)/∂ ln k = k·u — само измеряемое
// сопротивление, то есть шаг линеен там же, где линейно измерение. У 1/k
// (постоянная времени разгона) измерение −k·u нелинейно.
//
// Модель:  δθ' = −R·(δb + шум),  δb' = шум ухода,  δκ' = 0,  δw' = шум ветра
//   F = I + T·[[0, −R], [0, 0]] для (δθ, δb) и единица для (δκ, δw);
//   Q = diag(qθ·I, qb·I, 0, qw·I)·T. У k шума процесса нет — это постоянная
//   рамы: пока его не видно, дисперсия стоит на месте, а не растёт.
//
// Измерения и их якобианы (столбцы: δθ | δb | δκ | δw):
//  1. Горизонтальная удельная сила, каждый шаг — как раньше:
//       z = (R·a)xy − f;   H = [0 h 0 | 0 | 0 | 0],  [−h 0 0 | 0 | 0 | 0]
//     С ускорением по GPS (f = a_GPS) столбец δκ — ровно ноль: (R·a)xy есть
//     горизонтальное ускорение при любом k (тяга и сопротивление входят в a
//     вместе). Из обновления по ускорению GPS k не достать — для него нужно
//     измерение 2. Без GPS f = тяга + сопротивление по модели, ∂f/∂κ есть,
//     но k там неотличим от δθ (в установившемся наклоне k·v = g·tg φ при
//     любом k): он не учится, а неточность модели идёт в шум.
//  2. Сопротивление винтов, на каждый фикс GPS с курсом (10 Гц).
//     Акселерометр по осям X и Y рамы меряет его само:
//       y = a_xy = −(k/g)·u,   u = [Rᵀ·Rz(−d)·(v_GPS − w)]xy
//       H = [≈0 | 0 | −(k/g)·u | +(k/g)·[Rᵀ·Rz(−d)]xy←NE]
//     (d — курс минус рысканье фильтра). ∂y/∂δθ = −(k/g)·[Rᵀ·(v×δθ)]xy —
//     порядка (k/g)·|v|·sin φ·δθ: при 10 м/с, наклоне 30° и ошибке 1° это
//     0.003 g против шума 0.02 g — отнесено к шуму. Поэтому блоки (δθ, δb) и
//     (δκ, δw) не связаны ни прогнозом, ни измерениями, ковариация
//     блочно-диагональна точно: хранится как 6 × 6 и 3 × 3, и шаг 1 кГц от
//     новых состояний не дорожает.
//  3. Компас, 25 Гц:  z = (курс − рысканье) − опора = δθz + шум,
//       H = [0 0 1 | 0 | 0 | 0]
//     Из компаса берётся одно число — курс, то есть горизонтальная проекция
//     поля: наклонение и помехи по вертикали в измерение не входят. Усиление
//     оставлено только у δθz и δb_z; у крена, тангажа и их смещений оно
//     обнулено (ковариация тогда — в форме Джозефа, она верна при любом
//     усилении): компас не сдвинет горизонт ни при какой помехе.
//     Опора — первый отсчёт (рысканье фильтра считается от включения);
//     заново она берётся, когда неуверенность в рысканье больше 30°.
//     Помехи от моторов и силовых проводов — скачки курса: невязка
//     проверяется по χ² против своей дисперсии (P[2][2] + σ²), больше 3σ —
//     отсчёт отброшен. То же у измерения 2 (2 степени свободы).
//
// Наблюдаемость.
//   • k виден только при скорости относительно воздуха в плоскости винтов
//     (наклон + скорость): информация ∝ (k·|u|/g)². В висении u = −w, и
//     измерение — произведение k·w: раздельно k и ветер не видны. Поэтому при
//     скорости по земле меньше 3 м/с усиление по κ — ноль (κ «учитывается»,
//     но не правится — фильтр Шмидта): оценка и дисперсия k стоят точно,
//     измерение целиком идёт в ветер. В полёте в одну сторону k отличим от
//     ветра вдоль скорости лишь с точностью σ_w/|v| — ковариация это знает
//     (связь κ–w); разворот или смена скорости их разделяет.
//   • Без GPS (и без курса) k не учится — работает сохранённый.
//   • δθz и δb_z без компаса не наблюдаемы (в наклоне δb_z виден слабо —
//     через уход горизонта на sin φ): дисперсия рысканья упирается в
//     потолок, дисперсия смещения — в начальную (стоит, а не растёт с шумом
//     ухода), оценка в ровном полёте не двигается (усиление идёт через
//     Pθ,b_z, а оно без наклона ноль). С компасом наблюдаемы оба.
//
// Числа на float (FPU одинарной точности, double в цикле нет).
//   • Измерение 1 — полное усиление Калмана, P ← P − K·s·Kᵀ по верхнему
//     треугольнику с копией вниз: симметрия по построению. Шум «цветной» —
//     каждое измерение слабое (K ~ 10⁻³), вычитание близких чисел не грозит.
//   • Измерения 2 и 3 — усиление урезано, поэтому форма Джозефа
//     (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ: верна при любом K и не теряет
//     положительность на округлении.
//   • Нижние пределы дисперсий (на 2–3 порядка ниже рабочих) и потолки у
//     ненаблюдаемого; не число в ковариации — она начинается заново.
//   • Сброс ошибки: после поправки δθ̂ новая ошибка меряется от повёрнутой
//     оценки, δθ⁺ = (I + ½[δθ̂×])·(δθ − δθ̂), значит P ← G·P·Gᵀ. Считается,
//     когда поправка больше 1 мрад (после флипов, потери опоры); меньшая
//     меняет P слабее округления float.
//
// Цена. Шаг 1 кГц: ковариация 6 × 6 и два скалярных шага — ~260 умножений,
// ~230 сложений, 4 корня, 4 деления (сверх прежнего — сглаживание
// акселерометра, проверка невязки, пределы: ~25 операций и 1 деление).
// На STM32F405 (168 МГц, FPU, ~3.5 такта на операцию с загрузкой) — около
// 11 мкс против 10 прежних: +0.1 % загрузки при цикле 1 кГц. Компас (25 Гц,
// ~250 операций) и сопротивление (10 Гц, ~300 операций, exp и 3 корня) в
// среднем дают ещё 0.03 %; шаг, на который пришлись оба, длиннее на ~20 мкс.
// Память: 36 + 9 float ковариаций и ~25 float состояния. Динамической нет.
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
            mahonyFlight = kFlight;
            mahonyGround = kGround;
        }

        // Махони отдельно (imu_dcm_kp / imu_dcm_ki, имена BF): сила поправки по
        // акселерометру в полёте и на земле, 1/с, и интеграл — сам выбирает
        // смещение гироскопа. MEKF их не берёт (у него своя модель шумов).
        void setMahonyGains(float flightPerS, float groundPerS, float integralPerS2)
        {
            mahonyFlight = fmaxf(0.0f, flightPerS);
            mahonyGround = fmaxf(0.0f, groundPerS);
            mahonyKi = fmaxf(0.0f, integralPerS2);
            if (mahonyKi <= 0.0f) mahonyI = Vec3();
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
            mahonyI = Vec3();
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

        // Сопротивление винтов для модели, 1/с: значение из настроек
        // (attitude_drag) — с него начинает оценка. 0 — не измерено: берётся
        // EKF_DRAG_DEFAULT с неуверенностью ±50 %; измеренному вера ±15 %.
        void setDragCoef(float perS)
        {
            dragKnown = perS > 0.0f;
            dragInit = dragKnown ? fminf(EKF_DRAG_MAX, fmaxf(EKF_DRAG_MIN, perS)) : EKF_DRAG_DEFAULT;
            dragReset();
        }

        // Скорость по GPS с нового фикса, м/с (север, восток, вниз), и курс
        // минус рысканье этого фильтра: по ней и по акселерометру MEKF учит
        // сопротивление винтов и ветер. Звать один раз на фикс.
        void setDragVelocity(float northMs, float eastMs, float downMs, float headingMinusYawDeg)
        {
            const float d = headingMinusYawDeg * PI_F / 180.0f;
            dragCos = cosf(d);
            dragSin = sinf(d);
            dragVn = northMs;
            dragVe = eastMs;
            dragVd = downMs;
            dragFresh = true;
        }

        // Курс носа по компасу минус рысканье этого фильтра, °: каждый цикл,
        // пока компасу есть вера. MEKF берёт из него только поворот вокруг
        // вертикали: держит рысканье и находит смещение гироскопа по нему.
        void setCompass(float headingMinusYawDeg)
        {
            magOffset = headingMinusYawDeg * PI_F / 180.0f;
            magValid = true;
        }
        void clearCompass() { magValid = false; }

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
            const float k = (grounded ? mahonyGround : mahonyFlight) * trust;
            if (k > 0.0f && aLen > 0.1f)
            {
                const Vec3 measured = { a.x / aLen, a.y / aLen, a.z / aLen };
                const Vec3 estimated = upBody();
                const float gain = (!grounded && dot(measured, estimated) < COS_25) ? 0.0f : k;
                const Vec3 e = cross(measured, estimated);
                p += gain * e.x;
                q += gain * e.y;
                r += gain * e.z;
                // интеграл (imu_dcm_ki): копится, только пока поправке верим
                if (mahonyKi > 0.0f && gain > 0.0f)
                {
                    const float ki = mahonyKi * trust * dtS;
                    mahonyI.x = clampAbs(mahonyI.x + ki * e.x, MAHONY_I_MAX);
                    mahonyI.y = clampAbs(mahonyI.y + ki * e.y, MAHONY_I_MAX);
                    mahonyI.z = clampAbs(mahonyI.z + ki * e.z, MAHONY_I_MAX);
                }
            }
            p += mahonyI.x;
            q += mahonyI.y;
            r += mahonyI.z;

            integrate(p, q, r, dtS);
        }

        // Нуль гироскопа откалиброван заново (клавиша 'i', стики): выученная
        // поправка относилась к прежнему нулю и теперь вычиталась бы лишний
        // раз. Курсовую её часть без компаса фильтр сам не забудет никогда —
        // курс уплывал на неё (стол, GETFUN F405V3: −4.56 °/с). Ориентация
        // остаётся, неуверенность в смещении — как после включения.
        void forgetGyroBias()
        {
            bias = Vec3();
            mahonyI = Vec3();
            for (int i = 0; i < 6; ++i)
            {
                for (int j = 3; j < 6; ++j) P[i][j] = P[j][i] = 0.0f;
            }
            P[3][3] = P[4][4] = P[5][5] = EKF_BIAS_VAR0;
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

        // Сопротивление винтов, 1/с: оценка (начальная — пока не было GPS) и её
        // неуверенность — доля (σ логарифма: 0.1 — ±10 %).
        float dragCoef() const { return dragK; }
        float dragSigma() const { return dragSig; }
        // Оценка годится, чтобы её сохранить: набрано измерений на скорости и σ мала.
        bool dragLearned() const { return ekf && dragCount >= EKF_DRAG_SAVE_COUNT && dragSig <= EKF_DRAG_SAVE_SIGMA; }
        // Ветер по оценке, м/с: куда дует — на север, на восток.
        float windNorth() const { return windN; }
        float windEast() const { return windE; }
        // его неуверенность, м/с (1σ, худшая из двух осей)
        float windSigma() const { return ekf ? sqrtf(fmaxf(D[1][1], D[2][2])) : 0.0f; }
        // Скорость относительно воздуха по модели (тяга − сопротивление), м/с,
        // в осях рысканья гироскопа (X — рысканье 0): для полёта вслепую
        // (copter/DeadReckonHome.h)
        float airVelX() const { return ekf ? airX : 0.0f; }
        float airVelY() const { return ekf ? airY : 0.0f; }
        // Компас: принят ли фильтром (опора рысканья есть), сколько отсчётов
        // отброшено проверкой невязки.
        bool compassUsed() const { return ekf && magAnchored; }
        uint32_t compassRejected() const { return magRejects; }
        uint32_t dragRejected() const { return dragRejects; }
        // Нормированные квадраты невязок последнего шага (NIS; дисперсия — без
        // запаса на «цветной» шум): акселерометр и сопротивление — по 2 степени
        // свободы, компас — 1. Для проверки согласованности фильтра.
        float accelNis() const { return lastAccelNis; }
        float dragNis() const { return lastDragNis; }
        float compassNis() const { return lastMagNis; }
        // Ковариация: ошибка ориентации и смещения (6 × 6: δθ в земных осях,
        // рад; δb в осях рамы, рад/с) и блок сопротивления (3 × 3: δ ln k,
        // ветер на север и восток, м/с).
        float covariance(uint8_t i, uint8_t j) const { return i < 6 && j < 6 ? P[i][j] : 0.0f; }
        float dragCovariance(uint8_t i, uint8_t j) const { return i < 3 && j < 3 ? D[i][j] : 0.0f; }
        // Смещение гироскопа, рад/с, оси рамы (для тестов — без округления в °/с).
        Vec3 gyroBias() const { return ekf ? bias : Vec3(); }

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
            lastAccelNis = lastDragNis = lastMagNis = 0.0f;
            magAnchored = false;
            magSince = 0.0f;
            magRejects = 0;
            dragReset();
        }

        // Блок сопротивления — как после включения: k из настроек, ветра нет.
        void dragReset()
        {
            dragLog = logf(dragInit);
            dragK = dragInit;
            windN = windE = 0.0f;
            for (auto& row : D)
            {
                for (float& v : row) v = 0.0f;
            }
            D[0][0] = dragKnown ? EKF_DRAG_VAR_KNOWN : EKF_DRAG_VAR0;
            D[1][1] = D[2][2] = EKF_WIND_VAR0;
            dragSig = sqrtf(D[0][0]);
            dragFresh = false;
            dragSince = 0.0f;
            dragCount = 0;
            dragRejects = 0;
            dragRejectRun = 0;
            dragAx = dragAy = 0.0f;
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

            propagate(R, spin, T);

            // ---- скорость относительно воздуха по модели ----
            // Тяга идёт точно по оси рамы: в земных осях её горизонтальная часть
            // разгоняет коптер. Сопротивление винтов лежит в их плоскости и
            // пропорционально скорости в ней: в осях рамы −k·(vx, vy, 0).
            const float thrustX = 9.80665f * a.z * R[0][2], thrustY = 9.80665f * a.z * R[1][2];   // м/с²
            // ожидаемое сопротивление в осях рамы, g
            const float dragBx = -dragK * (R[0][0] * airX + R[1][0] * airY) / 9.80665f;
            const float dragBy = -dragK * (R[0][1] * airX + R[1][1] * airY) / 9.80665f;
            if (grounded)
            {
                airX = airY = 0.0f;
            }
            else
            {
                airX += (thrustX + 9.80665f * (R[0][0] * dragBx + R[0][1] * dragBy)) * T;
                airY += (thrustY + 9.80665f * (R[1][0] * dragBx + R[1][1] * dragBy)) * T;
            }

            // акселерометр в плоскости винтов, сглаженный на опоздание фикса
            // GPS, — измерение сопротивления (сравнивается со скоростью по GPS)
            const float lp = fminf(1.0f, T * (1.0f / EKF_DRAG_LAG_S));
            dragAx += lp * (a.x - dragAx);
            dragAy += lp * (a.y - dragAy);
            dragSince += T;
            magSince += T;

            // вертикальная удельная сила, g (вверх; в висении 1)
            const float h = -(R[2][0] * a.x + R[2][1] * a.y + R[2][2] * a.z);

            accelUpdate(R, a, aLen, h, spin, T, thrustX, thrustY, dragBx, dragBy);
            if (dragFresh) dragUpdate(R, aLen, h, spin);
            if (magValid && magSince >= EKF_MAG_STEP_S) magUpdate(R, spin);
            floorVariances();
        }

        // Прогноз ковариации: A = Pθθ, B = Pθb, C = Pbb.
        // A ← A − T·(B·Rᵀ + R·Bᵀ) + qθ·T;  B ← B − T·R·C;  C ← C + qb·T
        // (член T²·R·C·Rᵀ на три порядка меньше qθ·T — опущен). Блок
        // сопротивления (ln k, ветер) с этим не связан: у k шума процесса нет
        // (постоянная рамы), у ветра он добавляется при его измерении.
        void propagate(const float R[3][3], float spin, float T)
        {
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
            // Ненаблюдаемое не «расползается». Рысканье без компаса акселерометр
            // не видит: его дисперсия росла бы без конца — потолок. То же со
            // смещением, которого сейчас не видно (по рысканью без компаса, по
            // всем осям — пока акселерометру нет веры): дисперсия стоит на
            // начальной, а не уходит с шумом ухода нуля.
            if (P[2][2] > EKF_YAW_VAR_MAX) shrinkTo(2, EKF_YAW_VAR_MAX);
            for (int i = 3; i < 6; ++i)
            {
                if (P[i][i] > EKF_BIAS_VAR0) shrinkTo(i, EKF_BIAS_VAR0);
            }
        }

        // Дисперсию состояния i — к потолку: строка и столбец умножаются на одно
        // число (преобразование D·P·D — матрица остаётся ковариацией).
        void shrinkTo(int i, float limit)
        {
            const float s = sqrtf(limit / P[i][i]);
            for (int k = 0; k < 6; ++k)
            {
                P[i][k] *= s;
                P[k][i] *= s;
            }
        }

        // Измерение 1 (каждый шаг): горизонтальная удельная сила против
        // ожидаемого ускорения — горизонт и смещение гироскопа.
        void accelUpdate(const float R[3][3], const Vec3& a, float aLen, float h, float spin, float T, float thrustX,
                         float thrustY, float dragBx, float dragBy)
        {
            const float k = grounded ? kGround : kFlight;   // 0 — без акселерометра (только гироскоп)
            if (k <= 0.0f || aLen < 0.1f) return;

            // удельная сила в земных осях, g; её горизонтальная часть — это
            // горизонтальное ускорение коптера (тяжесть вертикальна)
            const float zx = R[0][0] * a.x + R[0][1] * a.y + R[0][2] * a.z;
            const float zy = R[1][0] * a.x + R[1][1] * a.y + R[1][2] * a.z;
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
                    // не верим доле самого ускорения — опоздание фикса, ошибка курса.
                    // Сопротивление сюда не входит: (R·a)xy — ускорение при любом k
                    fx = accelX / 9.80665f;
                    fy = accelY / 9.80665f;
                    sigma += EKF_K_GPS_RESIDUAL * sqrtf(fx * fx + fy * fy);
                }
                else
                {
                    // по модели: тяга по оси рамы плюс сопротивление. Акселерометр
                    // тогда не «видит ровно» на любом наклоне (тяга всегда по оси
                    // рамы), а сравнивает сопротивление с ожидаемым: ошибка горизонта
                    // проявляется через скорость, с опозданием 1/k секунд.
                    // Здесь k не наблюдаем (в установившемся наклоне k·v = g·tg φ при
                    // любом k) и не учится — только используется. Половине
                    // сопротивления не верим и при измеренном k: модель линейная, а
                    // у рамы оно квадратичное, плюс порывы (проверено на модели:
                    // выигрыш даёт верный k, а не большая вера ему)
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
            const float sX = h * h * P[1][1] + sigma * sigma, sY = h * h * P[0][0] + sigma * sigma;
            lastAccelNis = (nuX * nuX * sY + nuY * nuY * sX) / (sX * sY);
            // в полёте расхождение больше 25° (и больше 4σ) — не опора:
            // так далеко гироскоп не уплывает
            const float gate = fmaxf(SIN_25 * SIN_25, 16.0f * (P[0][0] + P[1][1])) * h * h;
            if (!grounded && nuX * nuX + nuY * nuY > gate) return;

            float dx[6] = {};
            scalarUpdate(1, h, nuX, Rk, dx);    // z.x − f.x = h·δθy
            scalarUpdate(0, -h, nuY, Rk, dx);   // z.y − f.y = −h·δθx
            inject(dx);
        }

        // Измерение 2 (на каждый фикс GPS с курсом): сопротивление винтов.
        // Акселерометр по осям X и Y рамы меряет его само: y = −(k/g)·u, где
        // u — скорость относительно воздуха в плоскости винтов,
        // u = [Rᵀ·Rz(−d)·(v_GPS − ветер)]xy (d — курс минус рысканье фильтра).
        //   ∂y/∂ ln k = −(k/g)·u = y;   ∂y/∂ветер = +(k/g)·[Rᵀ·Rz(−d)]
        //   ∂y/∂δθ ~ (k/g)·|v|·sin φ·δθ — на два порядка меньше шума: в R.
        // Поэтому блок (ln k, ветер) с ориентацией не связан, его ковариация
        // D — отдельные 3 × 3, и шаг 1 кГц от него не дорожает.
        void dragUpdate(const float R[3][3], float aLen, float h, float spin)
        {
            dragFresh = false;
            const float since = dragSince;
            dragSince = 0.0f;
            // ветер за это время мог смениться; потолок — начальная неуверенность
            for (int i = 1; i < 3; ++i)
            {
                D[i][i] += EKF_Q_WIND * since;
                if (D[i][i] > EKF_WIND_VAR0)
                {
                    const float s = sqrtf(EKF_WIND_VAR0 / D[i][i]);
                    for (int k = 0; k < 3; ++k)
                    {
                        D[i][k] *= s;
                        D[k][i] *= s;
                    }
                }
            }
            // на земле, в падении, на ударе сопротивление не меряется
            if (grounded || h < EKF_MIN_LIFT || fabsf(aLen - 1.0f) > 1.0f) return;

            const float c = dragCos, sn = dragSin;
            // скорость по земле и ветер — в земные оси фильтра, потом в оси рамы
            const float gex = c * dragVn + sn * dragVe, gey = -sn * dragVn + c * dragVe;
            const float wex = c * windN + sn * windE, wey = -sn * windN + c * windE;
            const float gbx = R[0][0] * gex + R[1][0] * gey + R[2][0] * dragVd;
            const float gby = R[0][1] * gex + R[1][1] * gey + R[2][1] * dragVd;
            const float ux = gbx - (R[0][0] * wex + R[1][0] * wey);
            const float uy = gby - (R[0][1] * wex + R[1][1] * wey);
            // k отличим от ветра только на скорости по земле: в висении (и в
            // висении на ветру) измерение — произведение k·ветер, и поправка k
            // была бы выдумкой. Усиление по k тогда — ноль: оценка и её
            // дисперсия стоят, измерение целиком идёт в ветер
            const bool learn = gbx * gbx + gby * gby >= EKF_DRAG_MIN_SPEED * EKF_DRAG_MIN_SPEED;

            const float kg = dragK / 9.80665f;
            const float Hx[3] = { -kg * ux, kg * (R[0][0] * c - R[1][0] * sn), kg * (R[0][0] * sn + R[1][0] * c) };
            const float Hy[3] = { -kg * uy, kg * (R[0][1] * c - R[1][1] * sn), kg * (R[0][1] * sn + R[1][1] * c) };
            const float nuX = dragAx - Hx[0], nuY = dragAy - Hy[0];

            // шум: акселерометр, ошибка курса и вращение за время опоздания
            // (скорость повёрнута не туда), разгон за время опоздания
            const float speed = sqrtf(ux * ux + uy * uy);
            float sigma = EKF_DRAG_SIGMA + kg * speed * (EKF_DRAG_HEADING + EKF_DRAG_LAG_S * spin);
            if (groundAccel > 0.0f) sigma += kg * EKF_DRAG_LAG_S * groundAccel;
            const float var = sigma * sigma;

            // стробирование: невязка против своей дисперсии (χ², 2 степени свободы)
            lastDragNis = nuX * nuX / (quadD(Hx) + var) + nuY * nuY / (quadD(Hy) + var);
            if (lastDragNis > EKF_CHI2_2)
            {
                if (dragRejects < 0xFFFFFFFFu) ++dragRejects;
                // секунду подряд — это не выброс, а другой ветер (поднялся выше,
                // вылетел из-за деревьев): его дисперсия — снова начальная.
                // Прибавка к диагонали связь ветра с k только ослабляет
                if (++dragRejectRun >= EKF_DRAG_REJECT_RUN)
                {
                    dragRejectRun = 0;
                    D[1][1] = fmaxf(D[1][1], EKF_WIND_VAR0);
                    D[2][2] = fmaxf(D[2][2], EKF_WIND_VAR0);
                }
                return;
            }
            dragRejectRun = 0;

            // ошибки измерения держатся секунды (порыв, перекос, курс): как у
            // акселерометра — дисперсия на отсчёт σ²·τ/T
            const float Rk = var * fmaxf(1.0f, EKF_DRAG_TAU_S / fmaxf(since, 0.05f));
            float dx[3] = {};
            dragScalar(Hx, nuX, Rk, learn, dx);
            dragScalar(Hy, nuY, Rk, learn, dx);

            dragLog = fminf(logf(EKF_DRAG_MAX), fmaxf(logf(EKF_DRAG_MIN), dragLog + dx[0]));
            dragK = expf(dragLog);
            windN = fminf(EKF_WIND_MAX, fmaxf(-EKF_WIND_MAX, windN + dx[1]));
            windE = fminf(EKF_WIND_MAX, fmaxf(-EKF_WIND_MAX, windE + dx[2]));
            if (learn && dragCount < 0xFFFF) ++dragCount;

            // пределы дисперсий: k измерен не точнее ±3 % (он «дышит» с газом и
            // скоростью) — и оценка не «засыпает»; потолок — начальная
            D[0][0] = fminf(EKF_DRAG_VAR0, fmaxf(EKF_DRAG_VAR_MIN, D[0][0]));
            for (int i = 1; i < 3; ++i) D[i][i] = fmaxf(EKF_WIND_VAR_MIN, D[i][i]);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = i + 1; j < 3; ++j)
                {
                    const float lim = sqrtf(D[i][i] * D[j][j]);
                    D[i][j] = D[j][i] = fminf(lim, fmaxf(-lim, D[i][j]));
                }
            }
            dragSig = sqrtf(D[0][0]);
        }

        float quadD(const float H[3]) const
        {
            float q = 0.0f;
            for (int i = 0; i < 3; ++i) q += H[i] * (D[i][0] * H[0] + D[i][1] * H[1] + D[i][2] * H[2]);
            return q;
        }

        // Скалярный шаг блока сопротивления в форме Джозефа:
        // D ← (I − K·H)·D·(I − K·H)ᵀ + K·Rk·Kᵀ — верна при любом усилении (и
        // когда строка k обнулена: тогда D[0][0] не меняется вовсе) и не даёт
        // матрице потерять положительность на округлении float.
        void dragScalar(const float H[3], float nu, float Rk, bool learn, float dx[3])
        {
            float DH[3];
            for (int i = 0; i < 3; ++i) DH[i] = D[i][0] * H[0] + D[i][1] * H[1] + D[i][2] * H[2];
            const float s = H[0] * DH[0] + H[1] * DH[1] + H[2] * DH[2] + Rk;
            if (!(s > 0.0f)) return;
            const float K[3] = { learn ? DH[0] / s : 0.0f, DH[1] / s, DH[2] / s };
            const float innovation = nu - (H[0] * dx[0] + H[1] * dx[1] + H[2] * dx[2]);
            for (int i = 0; i < 3; ++i) dx[i] += K[i] * innovation;

            float A[3][3], AD[3][3];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j) A[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
            }
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j) AD[i][j] = A[i][0] * D[0][j] + A[i][1] * D[1][j] + A[i][2] * D[2][j];
            }
            for (int i = 0; i < 3; ++i)
            {
                for (int j = i; j < 3; ++j)
                {
                    D[i][j] = AD[i][0] * A[j][0] + AD[i][1] * A[j][1] + AD[i][2] * A[j][2] + K[i] * Rk * K[j];
                    D[j][i] = D[i][j];
                }
            }
        }

        // Измерение 3 (25 Гц, пока компасу есть вера): курс. Из компаса
        // берётся одно число — поворот вокруг вертикали: z = (курс − рысканье)
        // − опора = δθz + шум, H = (0, 0, 1, 0, 0, 0). Вертикальная часть поля
        // (наклонение, помехи по вертикали) сюда не входит вовсе, а усиление
        // оставлено только рысканью и смещению гироскопа по оси Z рамы: крен,
        // тангаж и их смещения компас не трогает ни при каких помехах.
        // Опора — первый отсчёт: рысканье фильтра считается от включения, и
        // настоящий курс ему не нужен — только чтобы разность не уплывала.
        void magUpdate(const float R[3][3], float spin)
        {
            const float since = magSince;
            magSince = 0.0f;
            // нос почти вертикально — рысканье не определено
            if (R[0][0] * R[0][0] + R[1][0] * R[1][0] < 0.25f) return;
            // Опора — первый отсчёт. И заново — если рысканье уплыло неизвестно
            // куда (компаса долго не было, или он долго отбрасывался): шаг
            // Калмана линеен, а невязка свёрнута в ±180° — уход на 280° выглядел
            // бы как −80° и толкнул бы смещение не в ту сторону
            if (!magAnchored || P[2][2] > EKF_YAW_REANCHOR_VAR)
            {
                magRef = magOffset;
                magAnchored = true;
                // ошибка рысканья теперь считается от этой опоры: она — шум одного отсчёта
                for (int k = 0; k < 6; ++k) P[2][k] = P[k][2] = 0.0f;
                P[2][2] = EKF_MAG_SIGMA * EKF_MAG_SIGMA;
                return;
            }
            float nu = magOffset - magRef;
            while (nu > PI_F) nu -= 2.0f * PI_F;
            while (nu < -PI_F) nu += 2.0f * PI_F;

            // шум: сам компас (с ошибкой горизонта, которая через наклонение
            // поля переходит в курс) и опоздание отсчёта на вращении
            const float sigma = EKF_MAG_SIGMA + EKF_MAG_LAG_S * spin;
            const float var = sigma * sigma;

            // стробирование: помеха от моторов и силовых проводов — скачок
            // курса; невязка против своей дисперсии (χ², 1 степень свободы).
            // Отброшенный отсчёт ничего не меняет; дисперсия рысканья тем
            // временем растёт, и к компасу, который «переехал» насовсем,
            // фильтр вернётся сам
            lastMagNis = nu * nu / (P[2][2] + var);
            if (lastMagNis > EKF_CHI2_1)
            {
                if (magRejects < 0xFFFFFFFFu) ++magRejects;
                return;
            }

            const float Rk = var * fmaxf(1.0f, EKF_MAG_TAU_S / fmaxf(since, EKF_MAG_STEP_S));
            const float s = P[2][2] + Rk;
            const float K[6] = { 0.0f, 0.0f, P[2][2] / s, 0.0f, 0.0f, P[5][2] / s };
            joseph(2, K, Rk);
            const float dx[6] = { 0.0f, 0.0f, K[2] * nu, 0.0f, 0.0f, K[5] * nu };
            inject(dx);
        }

        // Ковариация после скалярного измерения состояния j (H = e_j) с любым
        // усилением K — форма Джозефа: P ← (I − K·H)·P·(I − K·H)ᵀ + K·Rk·Kᵀ.
        // Короткая P − K·s·Kᵀ верна только для полного усиления Калмана.
        void joseph(int j, const float K[6], float Rk)
        {
            float line[6];
            for (int c = 0; c < 6; ++c) line[c] = P[j][c];
            for (int i = 0; i < 6; ++i)
            {
                if (K[i] == 0.0f) continue;
                for (int c = 0; c < 6; ++c) P[i][c] -= K[i] * line[c];
            }
            for (int i = 0; i < 6; ++i) line[i] = P[i][j];
            for (int c = 0; c < 6; ++c)
            {
                if (K[c] == 0.0f) continue;
                for (int i = 0; i < 6; ++i) P[i][c] += (K[i] * Rk - line[i]) * K[c];
            }
            symmetrize();
        }

        void symmetrize()
        {
            for (int i = 0; i < 6; ++i)
            {
                for (int j = i + 1; j < 6; ++j) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
            }
        }

        // Ошибка -> в кватернион (малый поворот слева: земные оси) и в
        // смещение; после этого ошибка снова ноль.
        void inject(const float dx[6])
        {
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

            // Сброс ошибки (reset MEKF). Новая ошибка меряется от повёрнутой
            // оценки: δθ⁺ = (I + ½[δθ̂×])·(δθ − δθ̂), значит P ← G·P·Gᵀ,
            // G = I + ½[δθ̂×] в блоке δθ. Поправка шага — доли миллирадиана,
            // и G·P·Gᵀ отличается от P меньше, чем на округление float; считать
            // стоит только большую (после флипов, после потери опоры)
            if (4.0f * (hx * hx + hy * hy + hz * hz) > EKF_RESET_MIN)
            {
                for (int c = 0; c < 6; ++c)
                {
                    const float vx = P[0][c], vy = P[1][c], vz = P[2][c];
                    P[0][c] = vx + hy * vz - hz * vy;
                    P[1][c] = vy + hz * vx - hx * vz;
                    P[2][c] = vz + hx * vy - hy * vx;
                }
                for (int i = 0; i < 6; ++i)
                {
                    const float vx = P[i][0], vy = P[i][1], vz = P[i][2];
                    P[i][0] = vx + hy * vz - hz * vy;
                    P[i][1] = vy + hz * vx - hx * vz;
                    P[i][2] = vz + hx * vy - hy * vx;
                }
                symmetrize();
            }
        }

        // Нижние пределы дисперсий: на float вычитание K·s·Kᵀ может увести
        // диагональ в ноль и ниже (тогда усиление меняет знак, фильтр
        // расходится). Пол — на два-три порядка ниже рабочих значений, в работе
        // не достигается. Не число (NaN) в ковариации — начать её заново.
        void floorVariances()
        {
            for (int i = 0; i < 6; ++i)
            {
                const float lo = i < 3 ? EKF_ANGLE_VAR_MIN : EKF_BIAS_VAR_MIN;
                if (P[i][i] >= lo) continue;
                if (P[i][i] != P[i][i])
                {
                    covarianceRestart();
                    return;
                }
                P[i][i] = lo;
                for (int j = 0; j < 6; ++j)
                {
                    if (j == i) continue;
                    const float lim = sqrtf(lo * fmaxf(P[j][j], 0.0f));
                    P[i][j] = P[j][i] = fminf(lim, fmaxf(-lim, P[i][j]));
                }
            }
        }

        void covarianceRestart()
        {
            for (auto& row : P)
            {
                for (float& v : row) v = 0.0f;
            }
            P[0][0] = P[1][1] = EKF_TILT_VAR0;
            P[2][2] = magAnchored ? EKF_TILT_VAR0 : 0.0f;
            P[3][3] = P[4][4] = P[5][5] = EKF_BIAS_VAR0;
        }

        // Один скалярный шаг Калмана: измерение nu = h·δx[j] + шум (дисперсия Rk).
        // dx — накопленная поправка состояния (второй шаг учитывает первый).
        // Усиление полное, поэтому P ← P − K·s·Kᵀ; симметрия — по построению
        // (считается верхний треугольник, нижний — его копия).
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
        // сопротивление винтов k, 1/с: (м/с²) на (м/с). У винтов оно почти
        // линейно по скорости: 0.2–0.4 у коптеров 3–7". 0.3 — пока не измерено;
        // дальше — оценка по GPS (состояние фильтра ln k), между полётами
        // хранится в настройке attitude_drag
        static constexpr float EKF_DRAG_DEFAULT = 0.3f;
        static constexpr float EKF_DRAG_MIN = 0.05f, EKF_DRAG_MAX = 1.5f;
        static constexpr float EKF_K_DRAG = 0.5f;              // доля сопротивления по модели, которой нельзя верить
        static constexpr float EKF_DRAG_VAR0 = 0.25f;          // δ ln k после включения: ±50 % (1σ); он же потолок
        static constexpr float EKF_DRAG_VAR_KNOWN = 0.0225f;   // k из настроек (измерен в прошлых полётах): ±15 %
        // пол ±10 %: k — линейная замена сопротивлению, в котором есть и
        // квадратичная часть (рама): «эффективный» k растёт со скоростью, и
        // точнее его знать нельзя. С полом оценка не «засыпает» и идёт за ним
        static constexpr float EKF_DRAG_VAR_MIN = 0.01f;
        static constexpr float EKF_WIND_VAR0 = 25.0f;          // ветер после включения: 5 м/с (1σ); он же потолок
        static constexpr float EKF_WIND_VAR_MIN = 0.01f;       // пол: 0.1 м/с
        static constexpr float EKF_Q_WIND = 0.04f;             // ветер меняется: 0.2 м/с за √с
        static constexpr float EKF_WIND_MAX = 30.0f;
        static constexpr float EKF_DRAG_SIGMA = 0.02f;         // акселерометр в плоскости винтов, g: вибрация, порывы, перекос платы 1°
        static constexpr float EKF_DRAG_HEADING = 0.1f;        // ошибка курса, рад: скорость повёрнута на неё
        static constexpr float EKF_DRAG_LAG_S = 0.1f;          // опоздание фикса GPS; акселерометр сглажен на столько же
        static constexpr float EKF_DRAG_TAU_S = 2.0f;          // время корреляции ошибок измерения сопротивления
        static constexpr float EKF_DRAG_MIN_SPEED = 3.0f;      // скорость по земле в плоскости винтов меньше — k не трогать
        static constexpr float EKF_DRAG_SAVE_SIGMA = 0.12f;    // оценку можно сохранять: ±12 %
        static constexpr uint16_t EKF_DRAG_SAVE_COUNT = 100;   // и не меньше 10 с измерений на скорости
        static constexpr uint8_t EKF_DRAG_REJECT_RUN = 10;     // столько отброшенных подряд (1 с) — ветер сменился
        // компас: σ курса, рад
        static constexpr float EKF_MAG_STEP_S = 0.04f;         // поправка 25 Гц: датчик даёт 50 — каждый отсчёт в дело один раз
        static constexpr float EKF_MAG_SIGMA = 0.07f;          // 4°: железо рядом, калибровка, ошибка горизонта 1° × tg наклонения поля
        static constexpr float EKF_MAG_LAG_S = 0.02f;          // отсчёт опаздывает: на вращении 200 °/с — ещё 4°
        static constexpr float EKF_MAG_TAU_S = 1.0f;           // время корреляции ошибки компаса × 2
        static constexpr float EKF_YAW_REANCHOR_VAR = 0.3f;    // неуверенность в рысканье больше 30° — опору взять заново
        // стробирование — пороги χ²: честный датчик даёт такую невязку реже
        // чем раз из 370 (1 степень свободы, 3σ) и раз из 100 (2 степени)
        static constexpr float EKF_CHI2_1 = 9.0f, EKF_CHI2_2 = 9.21f;
        // пределы дисперсий (float): углы — (0.002°)², смещения — (0.00006 °/с)²
        static constexpr float EKF_ANGLE_VAR_MIN = 1.0e-9f;
        static constexpr float EKF_BIAS_VAR_MIN = 1.0e-12f;
        static constexpr float EKF_RESET_MIN = 1.0e-6f;        // поправка угла больше 1 мрад — ковариацию довернуть
        // GPS: до 0.7 м/с² — полная поправка, от 2.2 — никакой (шум скорости
        // M10 на базе 0.4 с — около 0.3 м/с²)
        static constexpr float GPS_ACCEL_OK = 0.7f, GPS_ACCEL_SPAN = 1.5f;
        // без GPS: до 15° — полная, от 35° — никакой (висение в ветер — меньше 15°)
        static constexpr float TILT_OK_DEG = 15.0f, TILT_SPAN_DEG = 20.0f;

        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
        float kFlight = 0.5f, kGround = 5.0f;
        float mahonyFlight = 0.5f, mahonyGround = 5.0f, mahonyKi = 0.0f;
        Vec3 mahonyI;                                    // интеграл Махони, рад/с
        static constexpr float MAHONY_I_MAX = 0.35f;     // ±20 °/с: дальше гироскоп не уплывает
        static float clampAbs(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }
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
        float lastAccelNis = 0.0f, lastDragNis = 0.0f, lastMagNis = 0.0f;
        // сопротивление винтов и ветер
        float dragInit = EKF_DRAG_DEFAULT;    // с чего начинать (настройка или умолчание)
        bool dragKnown = false;               // настройка задана — k измерен раньше
        float dragLog = -1.2039728f;          // ln k
        float dragK = EKF_DRAG_DEFAULT;       // k = exp(ln k), 1/с
        float dragSig = 0.5f;                 // σ(ln k)
        float windN = 0.0f, windE = 0.0f;     // ветер, м/с (на север, на восток)
        float D[3][3] = {};                   // ковариация: δ ln k, ветер N, E
        float dragAx = 0.0f, dragAy = 0.0f;   // акселерометр по X, Y рамы, сглаженный, g
        float dragVn = 0.0f, dragVe = 0.0f, dragVd = 0.0f;   // скорость по GPS с последнего фикса
        float dragCos = 1.0f, dragSin = 0.0f;                // курс минус рысканье
        bool dragFresh = false;
        float dragSince = 0.0f;
        uint16_t dragCount = 0;               // измерений, которые учили k
        uint32_t dragRejects = 0;
        uint8_t dragRejectRun = 0;            // отброшено подряд
        // компас
        bool magValid = false, magAnchored = false;
        float magOffset = 0.0f, magRef = 0.0f;   // курс минус рысканье: сейчас и опорный, рад
        float magSince = 0.0f;
        uint32_t magRejects = 0;
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
