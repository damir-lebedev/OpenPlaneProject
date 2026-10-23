#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "copter/CopterController.h"

// ============================================================
// Пульт коптера, как в Betaflight: карта каналов (`map`), калибровка
// концов стиков (`rxrange`), допустимые импульсы (`rx_min_usec`,
// `rx_max_usec`) и режимы на тумблерах (`aux`).
//
//   map AETR1234 — что приходит в каждом канале по порядку: A — крен,
//       E — тангаж, T — газ, R — рысканье, 1..4 — AUX1..AUX4 (CH5…CH8
//       по умолчанию). Каналы после восьмого — AUX5… по порядку.
//   rxrange 0..3 мин макс — концы стика крена/тангажа/рысканья/газа
//       (порядок BF) растягиваются на 1000..2000.
//   aux № режим канал от до [логика [связь]] — режим включён, пока AUX
//       «канал» (0 — AUX1) в [от, до) мкс, шаг 25, 900..2100. У режима
//       может быть несколько строк: логика 0 — хватит любой (ИЛИ), 1 —
//       нужны все (И); связь — режим включается вместе с другим.
//
// Таблица aux пуста (так после `defaults`) — работают тумблеры прошивки
// (config/CopterConfig.h: ARM — SwA, режимы — SwC). Есть хоть одна
// строка — работает только таблица, как в BF: вставленный diff из BF
// описывает её целиком, и умолчания прошивки к нему не примешиваются.
//
// Номера режимов — постоянные номера BF (ARM 0, ANGLE 1, HORIZON 2,
// ALTHOLD 3, AIRMODE 28…). Режимы, которых у нас пока нет (BEEPER,
// GPS RESCUE…), принимаются и хранятся (diff из BF вставляется без
// ошибок), но ничего не делают — `aux` об этом пишет.
//
// Приоритет полётных режимов: ALTHOLD, затем ANGLE, затем HORIZON,
// иначе ACRO. FAILSAFE — как потеря связи; AIRMODE — airmode и при
// выключенном в настройках; PREARM — есть в таблице: ARM только при
// включённом PREARM. USER1…USER4 — ножки PINIO платы (камера CAM1/CAM2,
// питание 12 В видеопередатчика…); VTX PIT MODE и VTX CONTROL DISABLE —
// видеопередатчику (copter/Vtx.h).
//
//   vtx № канал полоса канал_VTX мощность от до — как `vtx` в BF: пока AUX
//       «канал» в [от, до), у видеопередатчика эта полоса, канал и уровень
//       мощности (0 — не трогать). Так мощность — с тумблера или крутилки.
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    // Постоянные номера режимов BF (permanentId).
    enum BoxId : uint8_t
    {
        BOX_ARM = 0,
        BOX_ANGLE = 1,
        BOX_HORIZON = 2,
        BOX_ALTHOLD = 3,
        BOX_ANTIGRAVITY = 4,
        BOX_MAG = 5,
        BOX_HEADFREE = 6,
        BOX_BEEPER = 13,
        BOX_OSD = 19,
        BOX_TELEMETRY = 20,
        BOX_BLACKBOX = 26,
        BOX_FAILSAFE = 27,
        BOX_AIRMODE = 28,
        BOX_FLIP_OVER_AFTER_CRASH = 35,
        BOX_PREARM = 36,
        BOX_VTX_PIT_MODE = 39,
        BOX_USER1 = 40,
        BOX_USER2 = 41,
        BOX_USER3 = 42,
        BOX_USER4 = 43,
        BOX_PARALYZE = 45,
        BOX_GPS_RESCUE = 46,
        BOX_ACRO_TRAINER = 47,
        BOX_VTX_CONTROL_DISABLE = 48,
        BOX_LAUNCH_CONTROL = 49,
        BOX_BEEPER_MUTE = 52,
        BOX_COUNT = 64,   // номера 0..63
    };

    struct BoxInfo
    {
        uint8_t id;
        const char* name;   // как в конфигураторе BF
        bool works;         // у нас уже действует
    };

    inline const BoxInfo* boxTable(uint8_t& count)
    {
        static const BoxInfo table[] = {
            { BOX_ARM, "ARM", true },
            { BOX_ANGLE, "ANGLE", true },
            { BOX_HORIZON, "HORIZON", true },
            { BOX_ALTHOLD, "ALTHOLD", true },
            { BOX_AIRMODE, "AIR MODE", true },
            { BOX_FAILSAFE, "FAILSAFE", true },
            { BOX_PREARM, "PREARM", true },
            { BOX_USER1, "USER1", true },
            { BOX_USER2, "USER2", true },
            { BOX_USER3, "USER3", true },
            { BOX_USER4, "USER4", true },
            { BOX_VTX_PIT_MODE, "VTX PIT MODE", true },
            { BOX_VTX_CONTROL_DISABLE, "VTX CONTROL DISABLE", true },
            { BOX_ANTIGRAVITY, "ANTI GRAVITY", false },
            { BOX_MAG, "MAG", false },
            { BOX_HEADFREE, "HEADFREE", false },
            { BOX_BEEPER, "BEEPER", false },
            { BOX_OSD, "OSD DISABLE", false },
            { BOX_TELEMETRY, "TELEMETRY", false },
            { BOX_BLACKBOX, "BLACKBOX", false },
            { BOX_FLIP_OVER_AFTER_CRASH, "FLIP OVER AFTER CRASH", false },
            { BOX_PARALYZE, "PARALYZE", false },
            { BOX_GPS_RESCUE, "GPS RESCUE", false },
            { BOX_ACRO_TRAINER, "ACRO TRAINER", false },
            { BOX_LAUNCH_CONTROL, "LAUNCH CONTROL", false },
            { BOX_BEEPER_MUTE, "BEEPER MUTE", false },
        };
        count = sizeof(table) / sizeof(table[0]);
        return table;
    }

    inline const BoxInfo* findBox(uint8_t id)
    {
        uint8_t n = 0;
        const BoxInfo* t = boxTable(n);
        for (uint8_t i = 0; i < n; ++i)
        {
            if (t[i].id == id) return &t[i];
        }
        return nullptr;
    }

    inline uint8_t boxForMode(CopterMode m)
    {
        switch (m)
        {
            case CopterMode::ANGLE: return BOX_ANGLE;
            case CopterMode::HORIZON: return BOX_HORIZON;
            case CopterMode::ALT_HOLD: return BOX_ALTHOLD;
            case CopterMode::ACRO:
            default: return BOX_COUNT;   // ACRO — когда ни один не включён
        }
    }

    struct AuxRange
    {
        uint8_t mode = 0;
        uint8_t channel = 0;     // 0 — AUX1
        uint16_t startUs = 900;
        uint16_t endUs = 900;
        uint8_t logic = 0;       // 0 — ИЛИ, 1 — И
        uint8_t linked = 0;      // режим, за которым следует (0 — нет)
        bool used() const { return endUs > startUs; }
    };

    // Строка `vtx`: AUX в диапазоне — видеопередатчику полоса, канал, мощность.
    struct VtxRange
    {
        uint8_t channel = 0;     // 0 — AUX1
        uint8_t band = 0;        // 0 — не трогать
        uint8_t vtxChannel = 0;  // 0 — не трогать
        uint8_t power = 0;       // уровень, 0 — не трогать
        uint16_t startUs = 900;
        uint16_t endUs = 900;
        bool used() const { return endUs > startUs; }
    };

    // Что строки `vtx` выбрали сейчас (0 — ничего).
    struct VtxSelection
    {
        uint8_t band = 0, channel = 0, power = 0;
    };

    class RcConfig
    {
    public:
        static constexpr uint8_t AUX_SLOTS = 20;
        static constexpr uint8_t VTX_SLOTS = 10;
        static constexpr uint8_t MAPPABLE = 8;            // AERT1234
        static constexpr uint8_t STICKS = 4;
        static constexpr uint16_t AUX_MIN_US = 900, AUX_MAX_US = 2100, AUX_STEP_US = 25;
        static constexpr uint16_t PULSE_MIN_US = 750, PULSE_MAX_US = 2250;
        static constexpr uint8_t STORE_VERSION = 1;
        // Буквы функций в порядке BF (номер = функция): крен, тангаж, рысканье, газ, AUX1..4.
        static constexpr const char* LETTERS = "AERT1234";

        RcConfig() { reset(); }

        void reset()
        {
            static const uint8_t AETR[MAPPABLE] = { 0, 1, 3, 2, 4, 5, 6, 7 };   // R — CH4, T — CH3
            memcpy(map, AETR, sizeof(map));
            for (uint8_t i = 0; i < STICKS; ++i)
            {
                rangeMin[i] = 1000;
                rangeMax[i] = 2000;
            }
            for (auto& a : table) a = AuxRange();
            for (auto& v : vtxTable) v = VtxRange();
        }

        // Сколько каналов у приёмника: AUX — все после четырёх стиков.
        void setChannelCount(uint8_t n) { channels = n < STICKS + 1 ? STICKS + 1 : n; }
        uint8_t channelCount() const { return channels; }
        uint8_t auxCount() const { return static_cast<uint8_t>(channels - STICKS); }

        // ---- map ----

        // Канал приёмника (0 — CH1) для функции 0..7 (A, E, R, T, AUX1..4).
        uint8_t channelFor(uint8_t function) const { return function < MAPPABLE ? map[function] : function; }

        // Канал приёмника для AUX n (0 — AUX1).
        uint8_t auxChannel(uint8_t n) const
        {
            return n < MAPPABLE - STICKS ? map[STICKS + n] : static_cast<uint8_t>(STICKS + n);
        }

        // «TAER1234» (или 4 буквы — AUX как были): каждая буква — один раз.
        bool setMap(const char* text, char* error, size_t errorSize)
        {
            char buf[MAPPABLE + 1] = "AETR1234";
            const size_t n = text ? strlen(text) : 0;
            if (n != STICKS && n != MAPPABLE)
            {
                snprintf(error, errorSize, "map — 8 букв из AETR1234, например TAER1234");
                return false;
            }
            for (size_t i = 0; i < n; ++i) buf[i] = static_cast<char>(toupper(static_cast<unsigned char>(text[i])));
            uint8_t next[MAPPABLE];
            uint8_t seen = 0;
            for (uint8_t pos = 0; pos < MAPPABLE; ++pos)
            {
                const char* at = strchr(LETTERS, buf[pos]);
                if (!buf[pos] || !at)
                {
                    snprintf(error, errorSize, "буква %c — не из AETR1234", buf[pos]);
                    return false;
                }
                const uint8_t f = static_cast<uint8_t>(at - LETTERS);
                if (seen & (1u << f))
                {
                    snprintf(error, errorSize, "буква %c дважды", buf[pos]);
                    return false;
                }
                seen |= static_cast<uint8_t>(1u << f);
                next[f] = pos;
            }
            memcpy(map, next, sizeof(map));
            return true;
        }

        // Как в BF: буква функции на месте каждого канала, «AETR1234».
        void formatMap(char* out, size_t size) const
        {
            char s[MAPPABLE + 1] = {};
            for (uint8_t f = 0; f < MAPPABLE; ++f)
            {
                if (map[f] < MAPPABLE) s[map[f]] = LETTERS[f];
            }
            snprintf(out, size, "%s", s);
        }

        bool mapIsDefault() const
        {
            char s[MAPPABLE + 1];
            formatMap(s, sizeof(s));
            return strcmp(s, "AETR1234") == 0;
        }

        // ---- rxrange ----

        uint16_t rangeLow(uint8_t i) const { return i < STICKS ? rangeMin[i] : 1000; }
        uint16_t rangeHigh(uint8_t i) const { return i < STICKS ? rangeMax[i] : 2000; }
        bool rangeIsDefault(uint8_t i) const { return rangeLow(i) == 1000 && rangeHigh(i) == 2000; }

        bool setRange(long i, long lo, long hi, char* error, size_t errorSize)
        {
            if (i < 0 || i >= STICKS)
            {
                snprintf(error, errorSize, "rxrange 0-3 (крен, тангаж, рысканье, газ) мин макс");
                return false;
            }
            if (lo < PULSE_MIN_US || hi > PULSE_MAX_US || lo >= hi)
            {
                snprintf(error, errorSize, "концы %u..%u мкс, мин меньше макс", PULSE_MIN_US, PULSE_MAX_US);
                return false;
            }
            rangeMin[i] = static_cast<uint16_t>(lo);
            rangeMax[i] = static_cast<uint16_t>(hi);
            return true;
        }

        // ---- aux ----

        const AuxRange& aux(uint8_t slot) const { return table[slot < AUX_SLOTS ? slot : 0]; }

        bool anyAux() const
        {
            for (const auto& a : table)
            {
                if (a.used()) return true;
            }
            return false;
        }

        // Таблица тумблеров прошивки — работает, пока своя пуста.
        void setFallback(const AuxRange* ranges, uint8_t count)
        {
            for (auto& a : fallback) a = AuxRange();
            for (uint8_t i = 0; i < count && i < AUX_SLOTS; ++i) fallback[i] = ranges[i];
        }
        const AuxRange* fallbackAux() const { return fallback; }
        const AuxRange* effectiveAux() const { return anyAux() ? table : fallback; }

        // Есть ли в работающей таблице строка с этим режимом.
        bool hasMode(uint8_t mode) const
        {
            const AuxRange* t = effectiveAux();
            for (uint8_t i = 0; i < AUX_SLOTS; ++i)
            {
                if (t[i].used() && t[i].mode == mode) return true;
            }
            return false;
        }

        // «№ режим канал от до [логика [связь]]», числа через пробел.
        bool parseAux(const char* args, char* error, size_t errorSize)
        {
            long v[7] = { 0, 0, 0, 0, 0, 0, 0 };
            int n = 0;
            const char* p = args ? args : "";
            while (n < 7)
            {
                while (*p == ' ' || *p == '\t') ++p;
                if (!*p) break;
                char* end = nullptr;
                v[n] = strtol(p, &end, 10);
                if (end == p)
                {
                    snprintf(error, errorSize, "aux № режим канал от до [логика связь] — только числа");
                    return false;
                }
                p = end;
                ++n;
            }
            while (*p == ' ' || *p == '\t') ++p;
            if (n < 5 || *p)
            {
                snprintf(error, errorSize, "aux № режим канал от до [логика связь]");
                return false;
            }
            if (v[0] < 0 || v[0] >= AUX_SLOTS)
            {
                snprintf(error, errorSize, "№ строки 0-%u", AUX_SLOTS - 1);
                return false;
            }
            if (v[1] < 0 || v[1] >= BOX_COUNT || v[6] < 0 || v[6] >= BOX_COUNT)
            {
                snprintf(error, errorSize, "режим 0-%u", BOX_COUNT - 1);
                return false;
            }
            if (v[2] < 0 || v[2] >= auxCount())
            {
                snprintf(error, errorSize, "канал 0-%u (AUX1-AUX%u, у приёмника %u каналов)", auxCount() - 1, auxCount(),
                         channels);
                return false;
            }
            if (v[3] < AUX_MIN_US || v[3] > AUX_MAX_US || v[4] < AUX_MIN_US || v[4] > AUX_MAX_US)
            {
                snprintf(error, errorSize, "от и до — %u..%u мкс", AUX_MIN_US, AUX_MAX_US);
                return false;
            }
            if (v[5] < 0 || v[5] > 1)
            {
                snprintf(error, errorSize, "логика 0 (ИЛИ) или 1 (И)");
                return false;
            }
            AuxRange& a = table[v[0]];
            a.mode = static_cast<uint8_t>(v[1]);
            a.channel = static_cast<uint8_t>(v[2]);
            a.startUs = toStep(v[3]);
            a.endUs = toStep(v[4]);
            a.logic = static_cast<uint8_t>(v[5]);
            a.linked = static_cast<uint8_t>(v[6]);
            return true;
        }

        static void formatAux(uint8_t slot, const AuxRange& a, char* out, size_t size)
        {
            snprintf(out, size, "aux %u %u %u %u %u %u %u", slot, a.mode, a.channel, a.startUs, a.endUs, a.logic, a.linked);
        }

        // ---- vtx ----

        const VtxRange& vtx(uint8_t slot) const { return vtxTable[slot < VTX_SLOTS ? slot : 0]; }

        bool anyVtx() const
        {
            for (const auto& v : vtxTable)
            {
                if (v.used()) return true;
            }
            return false;
        }

        // «№ канал полоса канал_VTX мощность от до» (как `vtx` в BF).
        bool parseVtx(const char* args, uint8_t bands, uint8_t levels, char* error, size_t errorSize)
        {
            long v[7] = {};
            int n = 0;
            const char* p = args ? args : "";
            while (n < 7)
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
            if (n != 7 || *p)
            {
                snprintf(error, errorSize, "vtx № канал полоса канал_VTX мощность от до");
                return false;
            }
            if (v[0] < 0 || v[0] >= VTX_SLOTS)
            {
                snprintf(error, errorSize, "№ строки 0-%u", VTX_SLOTS - 1);
                return false;
            }
            if (v[1] < 0 || v[1] >= auxCount())
            {
                snprintf(error, errorSize, "канал 0-%u (AUX1-AUX%u)", auxCount() - 1, auxCount());
                return false;
            }
            if (v[2] < 0 || v[2] > bands || v[3] < 0 || v[3] > 8 || v[4] < 0 || v[4] > levels)
            {
                snprintf(error, errorSize, "полоса 0-%u, канал 0-8, мощность 0-%u (0 — не трогать)", bands, levels);
                return false;
            }
            if (v[5] < AUX_MIN_US || v[5] > AUX_MAX_US || v[6] < AUX_MIN_US || v[6] > AUX_MAX_US)
            {
                snprintf(error, errorSize, "от и до — %u..%u мкс", AUX_MIN_US, AUX_MAX_US);
                return false;
            }
            VtxRange& r = vtxTable[v[0]];
            r.channel = static_cast<uint8_t>(v[1]);
            r.band = static_cast<uint8_t>(v[2]);
            r.vtxChannel = static_cast<uint8_t>(v[3]);
            r.power = static_cast<uint8_t>(v[4]);
            r.startUs = toStep(v[5]);
            r.endUs = toStep(v[6]);
            return true;
        }

        static void formatVtx(uint8_t slot, const VtxRange& r, char* out, size_t size)
        {
            snprintf(out, size, "vtx %u %u %u %u %u %u %u", slot, r.channel, r.band, r.vtxChannel, r.power, r.startUs, r.endUs);
        }

        // ---- флеш ----

        bool isDefault() const
        {
            if (!mapIsDefault() || anyAux() || anyVtx()) return false;
            for (uint8_t i = 0; i < STICKS; ++i)
            {
                if (!rangeIsDefault(i)) return false;
            }
            return true;
        }

        // [версия][map 8][rxrange 4×(мин, макс u16)][число строк][строки: №, режим,
        // канал, от, до (шаги 25 мкс от 900), логика, связь][число строк vtx][строки:
        // №, канал, полоса, канал VTX, мощность, от, до (шаги)]. Строк vtx в старой
        // записи нет — их просто не будет.
        static constexpr size_t STORE_MAX = 1 + MAPPABLE + STICKS * 4 + 1 + AUX_SLOTS * 7 + 1 + VTX_SLOTS * 7;

        size_t serialize(uint8_t* out, size_t capacity) const
        {
            if (capacity < STORE_MAX) return 0;
            size_t n = 0;
            out[n++] = STORE_VERSION;
            memcpy(out + n, map, MAPPABLE);
            n += MAPPABLE;
            for (uint8_t i = 0; i < STICKS; ++i)
            {
                out[n++] = static_cast<uint8_t>(rangeMin[i]);
                out[n++] = static_cast<uint8_t>(rangeMin[i] >> 8);
                out[n++] = static_cast<uint8_t>(rangeMax[i]);
                out[n++] = static_cast<uint8_t>(rangeMax[i] >> 8);
            }
            const size_t countAt = n++;
            uint8_t count = 0;
            for (uint8_t slot = 0; slot < AUX_SLOTS; ++slot)
            {
                const AuxRange& a = table[slot];
                if (!a.used()) continue;
                out[n++] = slot;
                out[n++] = a.mode;
                out[n++] = a.channel;
                out[n++] = static_cast<uint8_t>((a.startUs - AUX_MIN_US) / AUX_STEP_US);
                out[n++] = static_cast<uint8_t>((a.endUs - AUX_MIN_US) / AUX_STEP_US);
                out[n++] = a.logic;
                out[n++] = a.linked;
                ++count;
            }
            out[countAt] = count;
            const size_t vtxCountAt = n++;
            uint8_t vtxCount = 0;
            for (uint8_t slot = 0; slot < VTX_SLOTS; ++slot)
            {
                const VtxRange& r = vtxTable[slot];
                if (!r.used()) continue;
                out[n++] = slot;
                out[n++] = r.channel;
                out[n++] = r.band;
                out[n++] = r.vtxChannel;
                out[n++] = r.power;
                out[n++] = static_cast<uint8_t>((r.startUs - AUX_MIN_US) / AUX_STEP_US);
                out[n++] = static_cast<uint8_t>((r.endUs - AUX_MIN_US) / AUX_STEP_US);
                ++vtxCount;
            }
            out[vtxCountAt] = vtxCount;
            return n;
        }

        // Не та версия или порченые данные — false и всё по умолчанию.
        bool deserialize(const uint8_t* data, size_t length)
        {
            reset();
            const size_t head = 1 + MAPPABLE + STICKS * 4 + 1;
            if (!data || length < head || data[0] != STORE_VERSION) return false;
            char letters[MAPPABLE + 1] = {};
            for (uint8_t f = 0; f < MAPPABLE; ++f)
            {
                const uint8_t ch = data[1 + f];
                if (ch >= MAPPABLE || letters[ch]) return false;
                letters[ch] = LETTERS[f];
            }
            memcpy(map, data + 1, MAPPABLE);
            size_t at = 1 + MAPPABLE;
            char error[64];
            for (uint8_t i = 0; i < STICKS; ++i, at += 4)
            {
                const long lo = data[at] | (data[at + 1] << 8);
                const long hi = data[at + 2] | (data[at + 3] << 8);
                setRange(i, lo, hi, error, sizeof(error));   // за границами — остаётся 1000..2000
            }
            const uint8_t count = data[at++];
            for (uint8_t k = 0; k < count && at + 7 <= length; ++k, at += 7)
            {
                const uint8_t slot = data[at];
                if (slot >= AUX_SLOTS || data[at + 1] >= BOX_COUNT || data[at + 4] > (AUX_MAX_US - AUX_MIN_US) / AUX_STEP_US)
                    continue;
                AuxRange& a = table[slot];
                a.mode = data[at + 1];
                a.channel = data[at + 2];
                a.startUs = static_cast<uint16_t>(AUX_MIN_US + data[at + 3] * AUX_STEP_US);
                a.endUs = static_cast<uint16_t>(AUX_MIN_US + data[at + 4] * AUX_STEP_US);
                a.logic = data[at + 5] ? 1 : 0;
                a.linked = data[at + 6] < BOX_COUNT ? data[at + 6] : 0;
            }
            if (at < length)
            {
                const uint8_t vtxCount = data[at++];
                for (uint8_t k = 0; k < vtxCount && at + 7 <= length; ++k, at += 7)
                {
                    const uint8_t slot = data[at];
                    if (slot >= VTX_SLOTS || data[at + 6] > (AUX_MAX_US - AUX_MIN_US) / AUX_STEP_US) continue;
                    VtxRange& r = vtxTable[slot];
                    r.channel = data[at + 1];
                    r.band = data[at + 2];
                    r.vtxChannel = data[at + 3];
                    r.power = data[at + 4];
                    r.startUs = static_cast<uint16_t>(AUX_MIN_US + data[at + 5] * AUX_STEP_US);
                    r.endUs = static_cast<uint16_t>(AUX_MIN_US + data[at + 6] * AUX_STEP_US);
                }
            }
            return true;
        }

    private:
        // Как в BF: вниз к шагу 25 мкс.
        static uint16_t toStep(long us) { return static_cast<uint16_t>(AUX_MIN_US + (us - AUX_MIN_US) / AUX_STEP_US * AUX_STEP_US); }

        uint8_t map[MAPPABLE];   // функция -> канал приёмника
        uint16_t rangeMin[STICKS], rangeMax[STICKS];
        AuxRange table[AUX_SLOTS];
        AuxRange fallback[AUX_SLOTS];
        VtxRange vtxTable[VTX_SLOTS];
        uint8_t channels = 10;
    };

    // Каналы приёмника (мкс) -> ручки контроллера. Хранит последнее
    // верное значение AUX: импульс вне rx_min_usec..rx_max_usec на AUX —
    // держится прежнее (rxfail «hold» в BF), на стике — связь потеряна.
    class RxInput
    {
    public:
        static constexpr uint8_t MAX_CHANNELS = 18;

        void configure(const RcConfig& c, uint16_t minUs, uint16_t maxUs)
        {
            cfg = c;
            rxMinUs = minUs;
            rxMaxUs = maxUs;
        }

        const RcConfig& config() const { return cfg; }

        // Канал газа у приёмника (порог failsafe по газу — на нём).
        uint8_t throttleChannel() const { return cfg.channelFor(3); }

        CopterRc read(const uint16_t* raw, uint8_t count, bool linkOk)
        {
            CopterRc rc;
            bool sticksValid = true;
            float v[RcConfig::STICKS];
            for (uint8_t f = 0; f < RcConfig::STICKS; ++f)
            {
                const uint8_t ch = cfg.channelFor(f);
                const uint16_t us = ch < count ? raw[ch] : 0;
                if (us < rxMinUs || us > rxMaxUs) sticksValid = false;
                // rxrange: концы стика -> 1000..2000
                const float lo = cfg.rangeLow(f), hi = cfg.rangeHigh(f);
                const float scaled = 1000.0f + (static_cast<float>(us) - lo) * 1000.0f / (hi - lo);
                v[f] = fminf(static_cast<float>(RcConfig::PULSE_MAX_US), fmaxf(static_cast<float>(RcConfig::PULSE_MIN_US), scaled));
            }
            auto stick = [](float us) { return fminf(1.0f, fmaxf(-1.0f, (us - 1500.0f) / 500.0f)); };
            rc.linkOk = linkOk && sticksValid;
            rc.roll = stick(v[0]);
            rc.pitch = -stick(v[1]);   // от себя — нос вниз
            rc.yaw = stick(v[2]);
            rc.throttle = fminf(1.0f, fmaxf(0.0f, (v[3] - 1000.0f) / 1000.0f));

            const uint8_t auxN = cfg.auxCount() < MAX_CHANNELS - RcConfig::STICKS ? cfg.auxCount()
                                                                                    : MAX_CHANNELS - RcConfig::STICKS;
            for (uint8_t n = 0; n < auxN; ++n)
            {
                const uint8_t ch = cfg.auxChannel(n);
                const uint16_t us = ch < count ? raw[ch] : 0;
                if (us >= rxMinUs && us <= rxMaxUs) aux[n] = us;
            }
            active = evaluate(cfg.effectiveAux(), aux, auxN);

            rc.armSwitch = isActive(BOX_ARM);
            if (isActive(BOX_ALTHOLD)) rc.mode = CopterMode::ALT_HOLD;
            else if (isActive(BOX_ANGLE)) rc.mode = CopterMode::ANGLE;
            else if (isActive(BOX_HORIZON)) rc.mode = CopterMode::HORIZON;
            else rc.mode = CopterMode::ACRO;
            rc.airmode = isActive(BOX_AIRMODE);
            rc.failsafeSwitch = isActive(BOX_FAILSAFE);
            rc.prearm = !cfg.hasMode(BOX_PREARM) || isActive(BOX_PREARM);
            rc.armAssigned = cfg.hasMode(BOX_ARM);
            return rc;
        }

        bool isActive(uint8_t box) const { return box < BOX_COUNT && ((active >> box) & 1u); }

        // Строки `vtx`, чей диапазон сейчас включён: что они задают (позже — главнее).
        VtxSelection vtxSelection() const
        {
            VtxSelection sel;
            const uint8_t auxN = cfg.auxCount() < MAX_CHANNELS - RcConfig::STICKS ? cfg.auxCount()
                                                                                    : MAX_CHANNELS - RcConfig::STICKS;
            for (uint8_t i = 0; i < RcConfig::VTX_SLOTS; ++i)
            {
                const VtxRange& r = cfg.vtx(i);
                if (!r.used() || r.channel >= auxN) continue;
                uint16_t us = aux[r.channel];
                if (us < RcConfig::AUX_MIN_US) us = RcConfig::AUX_MIN_US;
                if (us >= RcConfig::AUX_MAX_US) us = RcConfig::AUX_MAX_US - 1;
                if (us < r.startUs || us >= r.endUs) continue;
                if (r.band) sel.band = r.band;
                if (r.vtxChannel) sel.channel = r.vtxChannel;
                if (r.power) sel.power = r.power;
            }
            return sel;
        }
        uint64_t activeModes() const { return active; }
        uint16_t auxUs(uint8_t n) const { return n < MAX_CHANNELS - RcConfig::STICKS ? aux[n] : 0; }

        // Какие режимы включены при этих значениях AUX (мкс, 0 — AUX1).
        static uint64_t evaluate(const AuxRange* t, const uint16_t* auxValues, uint8_t auxN)
        {
            uint64_t orOn = 0, andSeen = 0, andOff = 0, linked = 0;
            for (uint8_t i = 0; i < RcConfig::AUX_SLOTS; ++i)
            {
                const AuxRange& a = t[i];
                if (!a.used() || a.mode >= BOX_COUNT) continue;
                const uint64_t bit = 1ull << a.mode;
                if (a.linked)
                {
                    linked |= bit;
                    continue;
                }
                bool on = false;
                if (a.channel < auxN)
                {
                    // как в BF: значение прижато к 900..2099
                    uint16_t us = auxValues[a.channel];
                    if (us < RcConfig::AUX_MIN_US) us = RcConfig::AUX_MIN_US;
                    if (us >= RcConfig::AUX_MAX_US) us = RcConfig::AUX_MAX_US - 1;
                    on = us >= a.startUs && us < a.endUs;
                }
                if (a.logic)
                {
                    andSeen |= bit;
                    if (!on) andOff |= bit;
                }
                else if (on)
                {
                    orOn |= bit;
                }
            }
            uint64_t mask = orOn | (andSeen & ~andOff);
            // связанные: режим включён, когда включён тот, за которым он следует
            for (uint8_t i = 0; i < RcConfig::AUX_SLOTS; ++i)
            {
                const AuxRange& a = t[i];
                if (a.used() && a.linked && a.mode < BOX_COUNT && ((linked >> a.mode) & 1u) && ((mask >> a.linked) & 1u))
                {
                    mask |= 1ull << a.mode;
                }
            }
            return mask;
        }

    private:
        RcConfig cfg;
        uint16_t rxMinUs = 885, rxMaxUs = 2115;
        uint16_t aux[MAX_CHANNELS - RcConfig::STICKS] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                                                          1500, 1500, 1500, 1500, 1500, 1500, 1500 };
        uint64_t active = 0;
    };
}
