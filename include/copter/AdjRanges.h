#pragma once

// Подстройки в полёте (121/122, как `adjrange` в BF): пока AUX «диапазона»
// в своих границах, канал «подстройки» меняет настройку:
//   - шагом: тумблер/крутилка вверх (> 1700 мкс) — +1, вниз (< 1300) — −1,
//     держать — ещё шаг каждые 0.5 с, середина — ничего;
//   - абсолютно (центр ≠ 0): значение = центр + (мкс − 1500) · масштаб / 500;
//   - выбором (RATE PROFILE, PID PROFILE): три положения
//     тумблера — профили 1–3.
// Строка CLI — как в BF 4.x:
//   adjrange № 0 канал_диапазона от до функция канал_подстройки центр масштаб
// (второе число — старое поле BF, всегда 0; каналы — 0 = AUX1). Стереть
// строку — «adjrange № 0 0 900 900 0 0 0 0». Хранится во флеше рядом с
// настройками (ключ "adj"); что делать с ней в полёте — copter/Adjustments.h.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace copter
{
    // Номера функций — как adjustmentFunction_e в BF (их пишет конфигуратор и diff).
    enum AdjFunction : uint8_t
    {
        ADJ_NONE = 0,
        ADJ_RC_RATE,
        ADJ_RC_EXPO,
        ADJ_THROTTLE_EXPO,
        ADJ_PITCH_ROLL_RATE,
        ADJ_YAW_RATE,
        ADJ_PITCH_ROLL_P,
        ADJ_PITCH_ROLL_I,
        ADJ_PITCH_ROLL_D,
        ADJ_YAW_P,
        ADJ_YAW_I,
        ADJ_YAW_D,
        ADJ_RATE_PROFILE,
        ADJ_PITCH_RATE,
        ADJ_ROLL_RATE,
        ADJ_PITCH_P,
        ADJ_PITCH_I,
        ADJ_PITCH_D,
        ADJ_ROLL_P,
        ADJ_ROLL_I,
        ADJ_ROLL_D,
        ADJ_RC_RATE_YAW,
        ADJ_PITCH_ROLL_F,
        ADJ_FEEDFORWARD_TRANSITION,
        ADJ_HORIZON_STRENGTH,
        ADJ_ROLL_RC_RATE,
        ADJ_PITCH_RC_RATE,
        ADJ_ROLL_RC_EXPO,
        ADJ_PITCH_RC_EXPO,
        ADJ_PID_AUDIO,
        ADJ_PITCH_F,
        ADJ_ROLL_F,
        ADJ_YAW_F,
        ADJ_OSD_PROFILE,
        ADJ_LED_PROFILE,
        ADJ_LED_DIMMER,
        ADJ_PID_PROFILE,   // своё (в BF PID-профиль тумблером не выбрать)
        ADJ_FUNCTION_COUNT
    };

    enum class AdjKind : uint8_t
    {
        UNSUPPORTED,   // у нас этого нет (PID AUDIO, LED) — строка хранится, но ничего не делает
        STEP,          // шагом или абсолютно — настройки
        SELECT,        // выбор профиля положением тумблера
    };

    struct AdjFunctionInfo
    {
        const char* name;   // как на OSD BF
        AdjKind kind;
    };

    inline const AdjFunctionInfo& adjFunctionInfo(uint8_t f)
    {
        static const AdjFunctionInfo table[ADJ_FUNCTION_COUNT] = {
            { "NONE", AdjKind::UNSUPPORTED },
            { "RC RATE", AdjKind::STEP },
            { "RC EXPO", AdjKind::STEP },
            { "THROTTLE EXPO", AdjKind::STEP },
            { "ROLL/PITCH RATE", AdjKind::STEP },
            { "YAW RATE", AdjKind::STEP },
            { "ROLL/PITCH P", AdjKind::STEP },
            { "ROLL/PITCH I", AdjKind::STEP },
            { "ROLL/PITCH D", AdjKind::STEP },
            { "YAW P", AdjKind::STEP },
            { "YAW I", AdjKind::STEP },
            { "YAW D", AdjKind::STEP },
            { "RATE PROFILE", AdjKind::SELECT },
            { "PITCH RATE", AdjKind::STEP },
            { "ROLL RATE", AdjKind::STEP },
            { "PITCH P", AdjKind::STEP },
            { "PITCH I", AdjKind::STEP },
            { "PITCH D", AdjKind::STEP },
            { "ROLL P", AdjKind::STEP },
            { "ROLL I", AdjKind::STEP },
            { "ROLL D", AdjKind::STEP },
            { "RC RATE YAW", AdjKind::STEP },
            { "PITCH/ROLL F", AdjKind::STEP },
            { "FF TRANSITION", AdjKind::STEP },
            { "HORIZON STRENGTH", AdjKind::STEP },
            { "ROLL RC RATE", AdjKind::STEP },
            { "PITCH RC RATE", AdjKind::STEP },
            { "ROLL RC EXPO", AdjKind::STEP },
            { "PITCH RC EXPO", AdjKind::STEP },
            { "PID AUDIO", AdjKind::UNSUPPORTED },
            { "PITCH F", AdjKind::STEP },
            { "ROLL F", AdjKind::STEP },
            { "YAW F", AdjKind::STEP },
            { "OSD PROFILE", AdjKind::UNSUPPORTED },   // профилей OSD у нас нет — элемент виден во всех
            { "LED PROFILE", AdjKind::UNSUPPORTED },
            { "LED DIMMER", AdjKind::UNSUPPORTED },
            { "PID PROFILE", AdjKind::SELECT },
        };
        return table[f < ADJ_FUNCTION_COUNT ? f : 0];
    }

    struct AdjRange
    {
        uint8_t rangeChannel = 0;   // AUX, что включает строку (0 — AUX1)
        uint16_t startUs = 900;
        uint16_t endUs = 900;
        uint8_t function = ADJ_NONE;
        uint8_t adjChannel = 0;     // AUX, которым подстраивают
        uint16_t center = 0;        // 0 — шагом; иначе — абсолютно от центра
        uint16_t scale = 0;         // на сколько меняется от центра до края
        bool used() const { return endUs > startUs; }
    };

    class AdjRangeTable
    {
    public:
        static constexpr uint8_t SLOTS = 30;   // как MAX_ADJUSTMENT_RANGE_COUNT в BF
        static constexpr uint16_t MIN_US = 900, MAX_US = 2100, STEP_US = 25;
        static constexpr uint16_t VALUE_MAX = 2000;   // центр и масштаб
        static constexpr uint8_t STORE_VERSION = 1;
        // [версия][число строк][строки: №, канал диапазона, от, до (шаги 25 мкс от 900),
        // функция, канал подстройки, центр u16, масштаб u16]
        static constexpr size_t ROW_BYTES = 10;
        static constexpr size_t STORE_MAX = 2 + SLOTS * ROW_BYTES;

        const AdjRange& range(uint8_t slot) const { return rows[slot < SLOTS ? slot : 0]; }

        bool any() const
        {
            for (const auto& r : rows)
            {
                if (r.used()) return true;
            }
            return false;
        }
        bool isDefault() const { return !any(); }
        void clear()
        {
            for (auto& r : rows) r = AdjRange();
        }

        // «№ 0 канал от до функция канал центр масштаб» (как `adjrange` в BF).
        bool parse(const char* args, uint8_t auxCount, char* error, size_t errorSize)
        {
            long v[9] = {};
            int n = 0;
            const char* p = args ? args : "";
            while (n < 9)
            {
                while (*p == ' ' || *p == '\t') ++p;
                if (!*p) break;
                char* end = nullptr;
                v[n] = strtol(p, &end, 10);
                if (end == p) break;
                p = end;
                ++n;
            }
            while (*p == ' ' || *p == '\t') ++p;
            if (n != 9 || *p)
            {
                snprintf(error, errorSize, "adjrange № 0 канал от до функция канал_подстройки центр масштаб");
                return false;
            }
            if (v[0] < 0 || v[0] >= SLOTS)
            {
                snprintf(error, errorSize, "№ строки 0-%u", SLOTS - 1);
                return false;
            }
            const AdjRange cleared;
            const bool clearing = v[3] == v[4];   // «900 900» — стереть
            if (!clearing && (v[2] < 0 || v[2] >= auxCount || v[6] < 0 || v[6] >= auxCount))
            {
                snprintf(error, errorSize, "каналы 0-%u (AUX1-AUX%u)", auxCount - 1, auxCount);
                return false;
            }
            if (v[3] < MIN_US || v[3] > MAX_US || v[4] < MIN_US || v[4] > MAX_US)
            {
                snprintf(error, errorSize, "от и до — %u..%u мкс", MIN_US, MAX_US);
                return false;
            }
            if (v[5] < 0 || v[5] >= ADJ_FUNCTION_COUNT)
            {
                snprintf(error, errorSize, "функция 0-%u (adjrange help — список)", ADJ_FUNCTION_COUNT - 1);
                return false;
            }
            if (v[7] < 0 || v[7] > VALUE_MAX || v[8] < 0 || v[8] > VALUE_MAX)
            {
                snprintf(error, errorSize, "центр и масштаб 0-%u", VALUE_MAX);
                return false;
            }
            AdjRange& r = rows[v[0]];
            if (clearing)
            {
                r = cleared;
                return true;
            }
            r.rangeChannel = static_cast<uint8_t>(v[2]);
            r.startUs = toStep(v[3]);
            r.endUs = toStep(v[4]);
            r.function = static_cast<uint8_t>(v[5]);
            r.adjChannel = static_cast<uint8_t>(v[6]);
            r.center = static_cast<uint16_t>(v[7]);
            r.scale = static_cast<uint16_t>(v[8]);
            return true;
        }

        static void format(uint8_t slot, const AdjRange& r, char* out, size_t size)
        {
            snprintf(out, size, "adjrange %u 0 %u %u %u %u %u %u %u", slot, r.rangeChannel, r.startUs, r.endUs, r.function,
                     r.adjChannel, r.center, r.scale);
        }

        size_t serialize(uint8_t* out, size_t capacity) const
        {
            if (capacity < STORE_MAX) return 0;
            size_t n = 0;
            out[n++] = STORE_VERSION;
            const size_t countAt = n++;
            uint8_t count = 0;
            for (uint8_t slot = 0; slot < SLOTS; ++slot)
            {
                const AdjRange& r = rows[slot];
                if (!r.used()) continue;
                out[n++] = slot;
                out[n++] = r.rangeChannel;
                out[n++] = static_cast<uint8_t>((r.startUs - MIN_US) / STEP_US);
                out[n++] = static_cast<uint8_t>((r.endUs - MIN_US) / STEP_US);
                out[n++] = r.function;
                out[n++] = r.adjChannel;
                out[n++] = static_cast<uint8_t>(r.center);
                out[n++] = static_cast<uint8_t>(r.center >> 8);
                out[n++] = static_cast<uint8_t>(r.scale);
                out[n++] = static_cast<uint8_t>(r.scale >> 8);
                ++count;
            }
            out[countAt] = count;
            return n;
        }

        // Не та версия или порченые данные — false, строки, что не прочлись, пустые.
        bool deserialize(const uint8_t* data, size_t length)
        {
            clear();
            if (!data || length < 2 || data[0] != STORE_VERSION) return false;
            const uint8_t count = data[1];
            size_t at = 2;
            for (uint8_t k = 0; k < count && at + ROW_BYTES <= length; ++k, at += ROW_BYTES)
            {
                const uint8_t* d = data + at;
                const uint8_t maxStep = (MAX_US - MIN_US) / STEP_US;
                if (d[0] >= SLOTS || d[2] > maxStep || d[3] > maxStep || d[4] >= ADJ_FUNCTION_COUNT) continue;
                AdjRange& r = rows[d[0]];
                r.rangeChannel = d[1];
                r.startUs = static_cast<uint16_t>(MIN_US + d[2] * STEP_US);
                r.endUs = static_cast<uint16_t>(MIN_US + d[3] * STEP_US);
                r.function = d[4];
                r.adjChannel = d[5];
                const uint16_t center = static_cast<uint16_t>(d[6] | (d[7] << 8));
                const uint16_t scale = static_cast<uint16_t>(d[8] | (d[9] << 8));
                r.center = center <= VALUE_MAX ? center : 0;
                r.scale = scale <= VALUE_MAX ? scale : 0;
            }
            return true;
        }

        // MSP_SET_ADJUSTMENT_RANGE: строка целиком (шаги 25 мкс, как в BF).
        bool setRaw(uint8_t slot, uint8_t rangeChannel, uint8_t startStep, uint8_t endStep, uint8_t function,
                    uint8_t adjChannel, uint16_t center, uint16_t scale, uint8_t auxCount)
        {
            char line[64], error[96];
            snprintf(line, sizeof(line), "%u 0 %u %u %u %u %u %u %u", slot, rangeChannel, MIN_US + startStep * STEP_US,
                     MIN_US + endStep * STEP_US, function, adjChannel, center, scale);
            return parse(line, auxCount, error, sizeof(error));
        }

    private:
        static uint16_t toStep(long us) { return static_cast<uint16_t>(MIN_US + (us - MIN_US) / STEP_US * STEP_US); }

        AdjRange rows[SLOTS];
    };
}
