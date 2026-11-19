#pragma once

// ============================================================
// Выбор гироскопа, когда их на плате два (Matek H743-SLIM V4: второй
// ICM-42688-P на SPI4). Настройка gyro_to_use — как у Betaflight:
//   FIRST  — первый, SECOND — второй, BOTH — среднее двух.
// В отличие от BF второй гироскоп здесь ещё и запасной: выбранный
// «умер» — берётся живой (вместо падения коптера). «Умер» = не ответил
// при включении или выдаёт одно и то же по всем шести осям подряд (у
// живого MEMS младшие разряды шумят всегда; мёртвый чип на SPI читается
// одинаковыми 0x00/0xFF). Ожил — возвращается сам.
//
// Оси второго чипа берутся те же, что у первого (одна калибровка
// установки). Повёрнут ли он на плате иначе — неизвестно, поэтому пока
// первый жив, второй идёт в дело только после проверки: при первом же
// заметном вращении (≥ 30 °/с) оба должны показать одно и то же. Не
// совпали — второй до перезагрузки не используется (среднее «правильного»
// и «повёрнутого» перевернуло бы коптер). Первый мёртв с самого начала —
// второй просто единственный гироскоп.
// Без Arduino: проверяется тестами на ПК.
// ============================================================

#include <math.h>
#include <stdint.h>

namespace copter
{
    enum class GyroToUse : uint8_t
    {
        FIRST = 0,
        SECOND = 1,
        BOTH = 2,
    };

    enum class GyroAgree : uint8_t
    {
        UNKNOWN = 0,   // коптер ещё не вращали
        YES = 1,       // оси совпадают
        NO = 2,        // второй повёрнут или врёт — не использовать
    };

    struct GyroSample
    {
        float gyro[3] = { 0, 0, 0 };    // °/с
        float accel[3] = { 0, 0, 0 };   // g
        bool ok = false;                // чип ответил при включении
    };

    // Застывшие показания: столько одинаковых чтений подряд — гироскоп мёртв
    // (при 1–4 кГц это 25–100 мс; у живого чипа совпадение всех 6 осей подряд
    // даже дважды почти невозможно).
    class GyroStuckDetector
    {
    public:
        static constexpr uint16_t STUCK_SAMPLES = 100;

        bool update(const GyroSample& s)
        {
            bool same = havePrev;
            for (int i = 0; i < 3 && same; ++i) same = s.gyro[i] == prev.gyro[i] && s.accel[i] == prev.accel[i];
            prev = s;
            havePrev = true;
            if (!same) repeats = 0;
            else if (repeats < STUCK_SAMPLES) ++repeats;
            return alive(s.ok);
        }

        bool alive(bool ok) const { return ok && repeats < STUCK_SAMPLES; }

    private:
        GyroSample prev;
        bool havePrev = false;
        uint16_t repeats = 0;
    };

    // Смотрят ли два гироскопа в одну сторону: решение — один раз до перезагрузки.
    class GyroAgreement
    {
    public:
        static constexpr float MIN_RATE_DPS = 30.0f;    // медленнее — шум и смещение нуля решают
        static constexpr float MAX_ERROR = 0.25f;       // |a − b| не больше четверти |a|
        static constexpr uint16_t DECIDE_SAMPLES = 200;

        GyroAgree update(const float a[3], const float b[3])
        {
            if (result != GyroAgree::UNKNOWN) return result;
            const float na = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if (na < MIN_RATE_DPS) return result;
            const float d0 = a[0] - b[0], d1 = a[1] - b[1], d2 = a[2] - b[2];
            const float d = sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
            if (d <= MAX_ERROR * na)
            {
                if (++good >= DECIDE_SAMPLES) result = GyroAgree::YES;
            }
            else if (++bad >= DECIDE_SAMPLES)
            {
                result = GyroAgree::NO;
            }
            return result;
        }

        GyroAgree state() const { return result; }

    private:
        GyroAgree result = GyroAgree::UNKNOWN;
        uint16_t good = 0, bad = 0;
    };

    struct GyroPick
    {
        float gyro[3] = { 0, 0, 0 };
        float accel[3] = { 0, 0, 0 };
        bool ok = false;          // есть хоть один живой
        uint8_t used = 0;         // бит 0 — первый, бит 1 — второй
        bool fallback = false;    // выбранный по настройке мёртв, взят другой
    };

    // Чистая функция: какой гироскоп (или среднее) отдать контуру.
    // second == nullptr — второго гироскопа на плате нет.
    inline GyroPick pickGyro(GyroToUse mode, const GyroSample& first, bool firstAlive, const GyroSample* second, bool secondAlive,
                             GyroAgree agree)
    {
        GyroPick p;
        const bool alive2 = second != nullptr && secondAlive;
        // пока первый жив — второй только проверенный; первый мёртв — любой не «повёрнутый»
        const bool eligible2 = alive2 && (firstAlive ? agree == GyroAgree::YES : agree != GyroAgree::NO);
        uint8_t want = mode == GyroToUse::BOTH ? 3u : mode == GyroToUse::SECOND ? 2u : 1u;
        if (second == nullptr) want = 1u;   // второго нет — настройка ни при чём
        const uint8_t eligible = static_cast<uint8_t>((firstAlive ? 1u : 0u) | (eligible2 ? 2u : 0u));
        const uint8_t aliveMask = static_cast<uint8_t>((firstAlive ? 1u : 0u) | (alive2 ? 2u : 0u));
        uint8_t use = want & eligible;
        if (use == 0) use = eligible;   // выбранный не в деле — любой годный
        p.fallback = use != 0 && (want & ~aliveMask) != 0;
        if (use == 0)
        {
            // годных нет: отдаём первый как есть, ok = false (запрет ARM, failsafe)
            for (int i = 0; i < 3; ++i)
            {
                p.gyro[i] = first.gyro[i];
                p.accel[i] = first.accel[i];
            }
            return p;
        }
        p.ok = true;
        p.used = use;
        if (use == 3u)
        {
            for (int i = 0; i < 3; ++i)
            {
                p.gyro[i] = 0.5f * (first.gyro[i] + second->gyro[i]);
                p.accel[i] = 0.5f * (first.accel[i] + second->accel[i]);
            }
        }
        else
        {
            const GyroSample& s = use == 2u ? *second : first;
            for (int i = 0; i < 3; ++i)
            {
                p.gyro[i] = s.gyro[i];
                p.accel[i] = s.accel[i];
            }
        }
        return p;
    }
}
