#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Подмена GPS (спуфинг): борт сам замечает, что GPS врёт (P43 а).
//
// Подменщик шлёт сигнал спутников, по которому приёмник «видит» себя не
// там, где он есть: точка прыгает на сотни метров или — хитрее — медленно
// «едет», и удержание позиции само уводит коптер в сторону (над деревьями
// в городе так и бывает). Отдельного антиспуфера нет — сверяем GPS с тем,
// что подменить нельзя: акселерометром, ветром, барометром.
//
// Курс сверке не нужен: ускорение — сырое горизонтальное, в осях «от
// включения» (рысканье гироскопа). Оценка положения учит по GPS смещение
// акселерометра и курс — подменённый GPS научил бы её «не замечать»
// подмену, а врущий компас выглядел бы как подмена. Поэтому сравниваются
// только модули — их поворот осей не меняет:
//
//   • JUMP — скачок. От прошлого фикса точка и скорость не могли уйти
//     дальше, чем позволяет ∫|a| (положение — двойной интеграл), плюс шум.
//     После пропажи фикса (глушение перед подменой) допуск растёт с
//     полётом, но в висении — на метры, и точка в километре не пройдёт.
//     Ориентации ещё нет — допуск по наибольшему ускорению (A_MAX).
//   • VELOCITY — увод. За окно WINDOW_S |приращение скорости GPS| против
//     |приращения по акселерометру|: невязка против своей дисперсии (χ²:
//     шум скорости GPS, ошибка горизонта, ошибка масштаба). GPS
//     «разгоняется», а коптер нет — подмена; удержание тянет коптер за
//     ложной точкой — коптер разгоняется, а GPS стоит — тоже.
//   • WIND — медленный увод тоньше акселерометра. Фильтр ориентации учит
//     сопротивление винтов и ветер по скорости GPS; подменённая скорость
//     выглядит как резкая смена ветра. Ветер дальше WIND_JUMP_MS от своего
//     среднего за минуту дольше WIND_HOLD_S — подмена.
//   • ALTITUDE — высота GPS против барометра: разность учится на земле и
//     медленно в полёте (уход барометра), скачок больше ALT_M — подмена.
//   • RECEIVER — сам приёмник: u-blox M10 сообщает подмену
//     (UBX-NAV-STATUS spoofDetState ≥ 2) дольше RECEIVER_S.
//   Доверие (не защёлка) снижает и путь по GPS за 8 с против пути по модели
//   фильтра (наклон → скорость относительно воздуха + ветер,
//   copter/DeadReckonHome.h): он видит увод, который GPS Rescue сам
//   наполовину гасит (модули разгонов совпадают — VELOCITY молчит), но так
//   же выглядит резкая смена ветра на 4–5 м/с — защёлкивать нельзя, пока нет
//   признака по переходному процессу (сопротивление по акселерометру).
//
// Подмена защёлкивается: GPS выключен из навигации до посадки — в воздухе
// сам не возвращается (подменщик может «отпустить» и поймать снова).
// Сброс — только без ARM, после CLEAR_S без признаков. gps_spoof_detect =
// OFF — проверки считают (доверие видно), но не защёлкивают.
//
// Чувствительность увода — разгон ложной точки от ~1.5 м/с² (порог —
// ошибка горизонта ≈ 2°); медленнее — ловит ветер (от WIND_JUMP_MS).
// Без Arduino — тестируется на ПК (test_copter, test_copter_sim).
// ============================================================

namespace copter
{
    enum class GpsSpoof : uint8_t { NONE, JUMP, VELOCITY, WIND, ALTITUDE, RECEIVER };

    inline const char* gpsSpoofName(GpsSpoof s)
    {
        switch (s)
        {
            case GpsSpoof::JUMP: return "JUMP";
            case GpsSpoof::VELOCITY: return "VELOCITY";
            case GpsSpoof::WIND: return "WIND";
            case GpsSpoof::ALTITUDE: return "ALTITUDE";
            case GpsSpoof::RECEIVER: return "RECEIVER";
            case GpsSpoof::NONE:
            default: return "NONE";
        }
    }

    struct GpsIntegrityInput
    {
        bool fresh = false;            // в этом цикле — новый фикс
        bool fix = false;              // 3D
        double latitude = 0.0, longitude = 0.0;   // градусы
        float velN = 0.0f, velE = 0.0f;           // м/с по земле
        float altitudeM = 0.0f;        // над уровнем моря
        uint8_t receiverSpoof = 0;     // spoofDetState приёмника: 0 — не знает, 1 — нет, 2 — подмена, 3 — несколько признаков
        float baroAltM = 0.0f;
        bool baroOk = false;
        bool accelOk = false;          // ориентация известна: ускорение ниже годится
        float accelX = 0.0f, accelY = 0.0f;       // м/с², горизонтальное, в осях «от включения», без тяжести
        bool windOk = false;           // фильтр выучил сопротивление: ветер ниже ему верен
        float windN = 0.0f, windE = 0.0f;         // м/с
        bool armed = false;
        // путь по GPS против пути по модели (copter/DeadReckonHome.h): отношение
        // к допуску (< 0 — не сравнить) — только в доверие, без защёлки
        float pathRatio = -1.0f;
    };

    class GpsIntegrity
    {
    public:
        static constexpr float A_MAX = 20.0f;          // м/с²: быстрее коптер не разгоняется (ориентации нет — допуск по нему)
        static constexpr float JUMP_M = 8.0f;          // скачок точки: допуск на шум и смену созвездия
        static constexpr float JUMP_V = 3.0f;          // скачок скорости, м/с
        static constexpr float JUMP_MARGIN = 1.5f;     // запас к ∫|a|
        static constexpr float ACC_SIGMA = 0.3f;       // м/с²: ошибка горизонта и смещение акселерометра (≈ 2°)
        static constexpr float SCALE_SIGMA = 0.1f;     // доля разгона: ошибка масштаба и горизонта в вираже
        static constexpr float VEL_SIGMA = 0.2f;       // м/с: шум скорости GPS (M10 по Доплеру — 0.05…0.2)
        static constexpr float GAP_MAX_S = 60.0f;      // фикса не было дольше — сверять не с чем, начать заново
        static constexpr float WINDOW_S = 2.0f;        // окно сверки разгона
        static constexpr float CHI2_GATE = 12.0f;      // χ² (1 степень свободы ≈ 3.5σ)
        static constexpr uint8_t CHI2_COUNT = 5;       // столько фиксов подряд за порогом
        static constexpr float WIND_JUMP_MS = 5.0f;
        static constexpr float WIND_HOLD_S = 5.0f;
        static constexpr float WIND_TAU_S = 60.0f;
        static constexpr float WIND_WARMUP_S = 10.0f;  // столько ветру верить, прежде чем сверять
        static constexpr float ALT_M = 30.0f;
        static constexpr float ALT_HOLD_S = 2.0f;
        static constexpr float ALT_TAU_GROUND_S = 5.0f;
        static constexpr float ALT_TAU_AIR_S = 300.0f;
        static constexpr float RECEIVER_S = 3.0f;
        static constexpr float CLEAR_S = 10.0f;        // без ARM: столько чисто — подмена снята
        static constexpr uint8_t HIST = 16;            // прошлые фиксы для окна — не чаще HIST_DT (ОЗУ F405)
        static constexpr float HIST_DT = 0.15f;

        void configure(bool enabled) { on = enabled; }
        bool enabled() const { return on; }

        // Всё забыть (новая сессия; подмена тоже снимается).
        void reset()
        {
            latched = GpsSpoof::NONE;
            synced = false;
            originSet = false;
            sinceFix = 1e6f;
            now = 0.0f;
            uX = uY = 0.0f;
            blindTicks = 0;
            histCount = 0;
            chi2Count = 0;
            windSet = false;
            windAge = 0.0f;
            windTimer = 0.0f;
            altSet = false;
            altTimer = 0.0f;
            receiverTimer = 0.0f;
            cleanTimer = 0.0f;
            worst = 0.0f;
            lastChi2 = 0.0f;
        }

        void update(const GpsIntegrityInput& in, float dt)
        {
            now += dt;
            sinceFix += dt;
            // ---- сколько коптер мог уйти от прошлого фикса: ∫|a| и ∫∫|a| ----
            const float a = in.accelOk ? sqrtf(in.accelX * in.accelX + in.accelY * in.accelY) : A_MAX;
            reachP += reachV * dt + 0.5f * a * dt * dt;
            reachV += a * dt;
            if (in.accelOk)
            {
                uX += in.accelX * dt;
                uY += in.accelY * dt;
            }
            else
            {
                ++blindTicks;
            }

            bool hit = false;
            GpsSpoof why = GpsSpoof::NONE;
            const auto flag = [&](GpsSpoof s) {
                if (!hit) why = s;
                hit = true;
            };

            // ---- приёмник ----
            receiverTimer = in.receiverSpoof >= 2 ? receiverTimer + dt : 0.0f;
            if (receiverTimer > RECEIVER_S) flag(GpsSpoof::RECEIVER);

            // ---- ветер ----
            if (in.windOk)
            {
                windAge += dt;
                if (!windSet && windAge >= WIND_WARMUP_S)
                {
                    windSlowN = in.windN;
                    windSlowE = in.windE;
                    windSet = true;
                }
                if (windSet)
                {
                    const float dN = in.windN - windSlowN, dE = in.windE - windSlowE;
                    const float d = sqrtf(dN * dN + dE * dE);
                    windRatio = d / WIND_JUMP_MS;
                    windTimer = d > WIND_JUMP_MS ? windTimer + dt : 0.0f;
                    if (windTimer > WIND_HOLD_S) flag(GpsSpoof::WIND);
                    if (latched == GpsSpoof::NONE)   // подменённый ветер в среднее не пускать
                    {
                        const float k = fminf(1.0f, dt / WIND_TAU_S);
                        windSlowN += dN * k;
                        windSlowE += dE * k;
                    }
                }
            }
            else
            {
                windAge = 0.0f;
                windTimer = 0.0f;
                windRatio = 0.0f;
            }

            // ---- фикс ----
            if (in.fresh && in.fix)
            {
                float fixWorst = 0.0f;
                // своя точка отсчёта — дом ставится только при ARM, а сверять надо и до него
                if (!originSet)
                {
                    originLat = in.latitude;
                    originLon = in.longitude;
                    cosLat = cosf(static_cast<float>(in.latitude) * 0.017453293f);
                    originSet = true;
                }
                const float n = static_cast<float>((in.latitude - originLat) * 111194.93);
                const float e = static_cast<float>((in.longitude - originLon) * 111194.93) * cosLat;

                // скачок: от прошлого фикса (с его скоростью) не дальше, чем позволяет ∫|a|
                if (synced && sinceFix <= GAP_MAX_S)
                {
                    const float t = sinceFix;
                    const float tolP = JUMP_M + JUMP_MARGIN * reachP + 3.0f * (VEL_SIGMA * t + 0.5f * ACC_SIGMA * t * t);
                    const float tolV = JUMP_V + JUMP_MARGIN * reachV + 3.0f * ACC_SIGMA * t;
                    const float eP = hypot2(n - (pN + vN * t), e - (pE + vE * t));
                    const float eV = hypot2(in.velN - vN, in.velE - vE);
                    jumpRatio = fmaxf(eP / tolP, eV / tolV);
                    fixWorst = fmaxf(fixWorst, jumpRatio);
                    if (jumpRatio > 1.0f) flag(GpsSpoof::JUMP);
                }
                // подмене не верим: отсчёт остаётся от последнего честного фикса
                if (latched == GpsSpoof::NONE && !hit)
                {
                    pN = n;
                    pE = e;
                    vN = in.velN;
                    vE = in.velE;
                    synced = true;
                    sinceFix = 0.0f;
                    reachP = reachV = 0.0f;
                }

                // увод: |разгон по GPS| против |разгона по акселерометру| за окно
                pushHist(in.velN, in.velE);
                const Hist* old = windowStart();
                if (old && old->blind == blindTicks && in.accelOk)
                {
                    const float gps = hypot2(in.velN - old->vN, in.velE - old->vE);
                    const float acc = hypot2(uX - old->uX, uY - old->uY);
                    const float w = now - old->t;
                    const float var = 2.0f * VEL_SIGMA * VEL_SIGMA + (ACC_SIGMA * w) * (ACC_SIGMA * w) +
                                      SCALE_SIGMA * SCALE_SIGMA * acc * acc;
                    lastChi2 = (gps - acc) * (gps - acc) / var;
                    chi2Count = lastChi2 > CHI2_GATE ? static_cast<uint8_t>(chi2Count < 255 ? chi2Count + 1 : 255) : 0;
                    fixWorst = fmaxf(fixWorst, sqrtf(lastChi2 / CHI2_GATE));
                    if (chi2Count >= CHI2_COUNT) flag(GpsSpoof::VELOCITY);
                }
                else
                {
                    chi2Count = 0;
                }

                // высота против барометра
                if (in.baroOk)
                {
                    const float d = in.altitudeM - in.baroAltM;
                    if (!altSet)
                    {
                        altOffset = d;
                        altSet = true;
                    }
                    const float dev = d - altOffset;
                    altRatio = fabsf(dev) / ALT_M;
                    fixWorst = fmaxf(fixWorst, altRatio);
                    altTimer = fabsf(dev) > ALT_M ? altTimer + fixPeriod() : 0.0f;
                    if (altTimer > ALT_HOLD_S) flag(GpsSpoof::ALTITUDE);
                    if (latched == GpsSpoof::NONE && fabsf(dev) <= ALT_M)
                        altOffset += dev * fminf(1.0f, fixPeriod() / (in.armed ? ALT_TAU_AIR_S : ALT_TAU_GROUND_S));
                }
                lastFixAt = now;
                worst = fmaxf(fmaxf(fixWorst, windRatio), in.pathRatio);
            }

            // ---- защёлка ----
            if (hit)
            {
                cleanTimer = 0.0f;
                if (on && latched == GpsSpoof::NONE)
                {
                    latched = why;
                    if (events < 0xFFFF) ++events;
                }
            }
            else if (latched != GpsSpoof::NONE && !in.armed)
            {
                cleanTimer += dt;
                if (cleanTimer > CLEAR_S)
                {
                    latched = GpsSpoof::NONE;
                    synced = false;   // начать сверку заново с честного фикса
                    chi2Count = 0;
                }
            }
            if (!on) latched = GpsSpoof::NONE;
        }

        bool spoofed() const { return latched != GpsSpoof::NONE; }
        GpsSpoof reason() const { return latched; }
        // Доверие GPS, 0…100: насколько далеко от порогов худшая проверка (подмена — 0).
        uint8_t trust() const
        {
            if (spoofed()) return 0;
            const float t = 100.0f * (1.0f - fminf(1.0f, worst));
            return static_cast<uint8_t>(t + 0.5f);
        }
        uint16_t detections() const { return events; }
        float chi2() const { return lastChi2; }

    private:
        struct Hist
        {
            float t, vN, vE, uX, uY;
            uint32_t blind;
        };

        static float hypot2(float x, float y) { return sqrtf(x * x + y * y); }
        float fixPeriod() const { return fminf(1.0f, fmaxf(0.02f, now - lastFixAt)); }

        void pushHist(float velN, float velE)
        {
            if (histCount && now - hist[head].t < HIST_DT) return;
            head = static_cast<uint8_t>((head + 1) % HIST);
            hist[head] = { now, velN, velE, uX, uY, blindTicks };
            if (histCount < HIST) ++histCount;
        }

        // Самый свежий фикс не моложе окна; окна не набралось — старейший, если ему хотя бы полокна.
        const Hist* windowStart() const
        {
            const Hist* best = nullptr;
            for (uint8_t i = 0; i < histCount; ++i)
            {
                const Hist& h = hist[(head + HIST - i) % HIST];
                best = &h;
                if (now - h.t >= WINDOW_S) return &h;
            }
            return best && now - best->t >= 0.5f * WINDOW_S ? best : nullptr;
        }

        bool on = true;
        GpsSpoof latched = GpsSpoof::NONE;
        uint16_t events = 0;
        float now = 0.0f;
        // последний честный фикс и сколько коптер мог уйти от него
        bool synced = false;
        float pN = 0.0f, pE = 0.0f, vN = 0.0f, vE = 0.0f;
        float sinceFix = 1e6f;
        float reachP = 0.0f, reachV = 0.0f;
        bool originSet = false;
        double originLat = 0.0, originLon = 0.0;
        float cosLat = 1.0f;
        // окно разгона
        float uX = 0.0f, uY = 0.0f;
        uint32_t blindTicks = 0;
        Hist hist[HIST] = {};
        uint8_t head = 0, histCount = 0;
        uint8_t chi2Count = 0;
        float lastChi2 = 0.0f;
        // ветер
        bool windSet = false;
        float windAge = 0.0f, windTimer = 0.0f;
        float windSlowN = 0.0f, windSlowE = 0.0f;
        // высота
        bool altSet = false;
        float altOffset = 0.0f, altTimer = 0.0f;
        float receiverTimer = 0.0f;
        float cleanTimer = 0.0f;
        float lastFixAt = 0.0f;
        // доверие
        float worst = 0.0f, jumpRatio = 0.0f, windRatio = 0.0f, altRatio = 0.0f;
    };
}
