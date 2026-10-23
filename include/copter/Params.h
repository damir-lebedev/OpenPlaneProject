#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "copter/Presets.h"
#include "storage/KeyValueStore.h"

// ============================================================
// Реестр настроек коптера — то, что в Betaflight задаётся строкой
// `set имя = значение` в CLI.
//
// У каждой настройки: имя как в BF, тип (целое, дробное, выбор из
// списка, строка), границы, значение по умолчанию, единицы, группа
// (= вкладка приложения) и описание по-русски одной строкой. Значения —
// в единицах BF, где они у нас имеют смысл (P/I/D/F, расходы).
//
// Где хранится:
//   • общие (MASTER) — одно значение;
//   • PID-профиль и профиль расходов — по 4 значения (profile 0..3,
//     rateprofile 0..3, как в BF), работает выбранный профиль.
// Настройка, которую не меняли, берёт умолчание — а оно у многих не
// число из таблицы, а вывод из описания коптера (PID, расходы — по модели
// и стилю полёта) или дефолт BF 4.5 (tune_source = BETAFLIGHT). Поэтому
// смена сборки или источника двигает всё, что пилот не трогал, а то, что
// он задал руками, остаётся. Задать значение, равное умолчанию, — то же,
// что вернуть умолчание (в diff его нет, как в BF).
//
// Во флеш (KeyValueStore, пространство "copter", ключ "params") пишутся
// только заданные руками значения: [версия][профиль][профиль расходов],
// затем записи [хеш имени u16][профиль u8][значение f32] — строки
// [хеш][0x80|профиль][длина][символы]. Хеш, а не номер, — чтобы новые
// настройки и перестановка таблицы не сдвигали сохранённые; настройку,
// которой больше нет, загрузка пропускает, значение за границами —
// прижимает к ним.
//
// Код переносимый (без Arduino) — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    enum class ParamType : uint8_t
    {
        INT,
        FLOAT,
        ENUM,
        STRING,
    };

    enum class ParamScope : uint8_t
    {
        MASTER,
        PID_PROFILE,
        RATE_PROFILE,
    };

    // Группа = вкладка приложения.
    enum class ParamGroup : uint8_t
    {
        PID,
        RATES,
        FILTERS,
        MOTORS,
        SAFETY,
        RECEIVER,
        MODES,
        COPTER,
        OTHER,
    };

    inline const char* paramGroupName(ParamGroup g)
    {
        switch (g)
        {
            case ParamGroup::PID: return "pid";
            case ParamGroup::RATES: return "rates";
            case ParamGroup::FILTERS: return "filters";
            case ParamGroup::MOTORS: return "motors";
            case ParamGroup::SAFETY: return "safety";
            case ParamGroup::RECEIVER: return "receiver";
            case ParamGroup::MODES: return "modes";
            case ParamGroup::COPTER: return "copter";
            case ParamGroup::OTHER: return "other";
        }
        return "other";
    }

    enum class TuneSource : uint8_t
    {
        MODEL,        // PID, фильтры, TPA — из описания коптера (наш вывод)
        BETAFLIGHT,   // дефолты Betaflight 4.5
    };

    // Порядок — как в таблице paramTable(): P, I, D, F по осям подряд.
    enum ParamId : uint8_t
    {
        PARAM_TUNE_SOURCE,
        PARAM_PROFILE_NAME,
        PARAM_P_ROLL,
        PARAM_I_ROLL,
        PARAM_D_ROLL,
        PARAM_F_ROLL,
        PARAM_P_PITCH,
        PARAM_I_PITCH,
        PARAM_D_PITCH,
        PARAM_F_PITCH,
        PARAM_P_YAW,
        PARAM_I_YAW,
        PARAM_D_YAW,
        PARAM_F_YAW,
        PARAM_RATEPROFILE_NAME,
        PARAM_ROLL_RC_RATE,
        PARAM_PITCH_RC_RATE,
        PARAM_YAW_RC_RATE,
        PARAM_ROLL_EXPO,
        PARAM_PITCH_EXPO,
        PARAM_YAW_EXPO,
        PARAM_ROLL_SRATE,
        PARAM_PITCH_SRATE,
        PARAM_YAW_SRATE,
        PARAM_COUNT
    };

    struct ParamInfo
    {
        const char* name;            // как в CLI Betaflight
        ParamType type;
        ParamScope scope;
        ParamGroup group;
        float min, max;              // STRING: max — наибольшая длина
        float def;                   // умолчание, если не fromModel
        bool fromModel;              // умолчание — из описания коптера / tune_source
        const char* units;
        const char* const* options;  // ENUM: имена значений 0, 1, …
        uint8_t optionCount;
        const char* text;            // что это, по-русски
    };

    inline const ParamInfo* paramTable()
    {
        static const char* const TUNE_SOURCES[] = { "MODEL", "BETAFLIGHT" };
        constexpr ParamType kInt = ParamType::INT, kEnum = ParamType::ENUM, kText = ParamType::STRING;
        constexpr ParamScope kMaster = ParamScope::MASTER, kPid = ParamScope::PID_PROFILE, kRate = ParamScope::RATE_PROFILE;
        constexpr ParamGroup kGroupPid = ParamGroup::PID, kGroupRates = ParamGroup::RATES;
        static const ParamInfo table[] = {
            { "tune_source", kEnum, kMaster, kGroupPid, 0, 1, 0, false, "", TUNE_SOURCES, 2,
              "откуда PID и фильтры: MODEL — из описания коптера, BETAFLIGHT — дефолты BF 4.5" },
            { "profile_name", kText, kPid, kGroupPid, 0, 8, 0, false, "", nullptr, 0, "имя PID-профиля" },
            { "p_roll", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "P крена" },
            { "i_roll", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "I крена" },
            { "d_roll", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "D крена" },
            { "f_roll", kInt, kPid, kGroupPid, 0, 1000, 0, true, "BF", nullptr, 0, "упреждение (feedforward) крена" },
            { "p_pitch", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "P тангажа" },
            { "i_pitch", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "I тангажа" },
            { "d_pitch", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "D тангажа" },
            { "f_pitch", kInt, kPid, kGroupPid, 0, 1000, 0, true, "BF", nullptr, 0, "упреждение (feedforward) тангажа" },
            { "p_yaw", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "P рысканья" },
            { "i_yaw", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "I рысканья" },
            { "d_yaw", kInt, kPid, kGroupPid, 0, 250, 0, true, "BF", nullptr, 0, "D рысканья" },
            { "f_yaw", kInt, kPid, kGroupPid, 0, 1000, 0, true, "BF", nullptr, 0, "упреждение (feedforward) рысканья" },
            { "rateprofile_name", kText, kRate, kGroupRates, 0, 8, 0, false, "", nullptr, 0, "имя профиля расходов" },
            { "roll_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, true, "x10 °/с", nullptr, 0, "расход крена у центра стика" },
            { "pitch_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, true, "x10 °/с", nullptr, 0, "расход тангажа у центра стика" },
            { "yaw_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, true, "x10 °/с", nullptr, 0, "расход рысканья у центра стика" },
            { "roll_expo", kInt, kRate, kGroupRates, 0, 100, 0, true, "%", nullptr, 0, "экспо крена: мягче у центра" },
            { "pitch_expo", kInt, kRate, kGroupRates, 0, 100, 0, true, "%", nullptr, 0, "экспо тангажа: мягче у центра" },
            { "yaw_expo", kInt, kRate, kGroupRates, 0, 100, 0, true, "%", nullptr, 0, "экспо рысканья: мягче у центра" },
            { "roll_srate", kInt, kRate, kGroupRates, 0, 255, 0, true, "x10 °/с", nullptr, 0, "наибольший расход крена (стик до упора)" },
            { "pitch_srate", kInt, kRate, kGroupRates, 0, 255, 0, true, "x10 °/с", nullptr, 0, "наибольший расход тангажа (стик до упора)" },
            { "yaw_srate", kInt, kRate, kGroupRates, 0, 255, 0, true, "x10 °/с", nullptr, 0, "наибольший расход рысканья (стик до упора)" },
        };
        static_assert(sizeof(table) / sizeof(table[0]) == PARAM_COUNT, "таблица настроек и ParamId разошлись");
        return table;
    }

    inline const ParamInfo& paramInfo(ParamId id) { return paramTable()[id < PARAM_COUNT ? id : 0]; }

    // Настройка по имени (без учёта регистра); -1 — нет такой.
    inline int findParam(const char* name)
    {
        if (!name) return -1;
        for (int i = 0; i < PARAM_COUNT; ++i)
        {
            const char* a = paramTable()[i].name;
            const char* b = name;
            while (*a && tolower(static_cast<unsigned char>(*b)) == *a)
            {
                ++a;
                ++b;
            }
            if (*a == '\0' && *b == '\0') return i;
        }
        return -1;
    }

    // FNV-1a, свёрнутый до 16 бит: ключ настройки во флеше.
    inline uint16_t paramNameHash(const char* name)
    {
        uint32_t h = 2166136261u;
        for (const char* p = name; *p; ++p)
        {
            h ^= static_cast<uint8_t>(*p);
            h *= 16777619u;
        }
        return static_cast<uint16_t>(h ^ (h >> 16));
    }

    inline ParamId pidParam(uint8_t axis, PidTerm term)
    {
        return static_cast<ParamId>(PARAM_P_ROLL + axis * 4 + static_cast<uint8_t>(term));
    }


    class CopterParams
    {
    public:
        static constexpr uint8_t PROFILES = 4;
        static constexpr uint8_t NAME_LENGTH = 8;
        static constexpr uint8_t STORE_VERSION = 1;
        static constexpr const char* STORE_SPACE = "copter";
        static constexpr const char* STORE_KEY = "params";

        CopterParams()
        {
            clearValues();
            refreshDefaults();
        }

        // Описание коптера -> модельная настройка (перебор контура — долго,
        // один раз) и умолчания. Заданное руками не трогается.
        void begin(const BuildSpec& b, float loopHz, EscProtocol esc)
        {
            model = deriveTune(b, loopHz, esc);
            refreshDefaults();
        }

        const CopterTune& modelTune() const { return model; }

        // ---- профили ----
        uint8_t pidProfile() const { return pidIndex; }
        uint8_t rateProfile() const { return rateIndex; }
        bool selectPidProfile(uint8_t n)
        {
            if (n >= PROFILES) return false;
            pidIndex = n;
            return true;
        }
        bool selectRateProfile(uint8_t n)
        {
            if (n >= PROFILES) return false;
            rateIndex = n;
            return true;
        }

        // Профиль, в котором сейчас работает настройка (общие — 0).
        uint8_t activeSlot(ParamId id) const
        {
            switch (paramInfo(id).scope)
            {
                case ParamScope::PID_PROFILE: return pidIndex;
                case ParamScope::RATE_PROFILE: return rateIndex;
                case ParamScope::MASTER:
                default: return 0;
            }
        }

        // ---- значения ----
        float defaultValue(ParamId id) const { return defaults[id]; }
        bool isSet(ParamId id, uint8_t slot) const
        {
            if (paramInfo(id).type == ParamType::STRING) return text(id, slot)[0] != '\0';
            return slot < PROFILES && (setMask[id] >> slot) & 1u;
        }
        float valueIn(ParamId id, uint8_t slot) const { return isSet(id, slot) ? values[id][slot] : defaults[id]; }
        float value(ParamId id) const { return valueIn(id, activeSlot(id)); }

        const char* text(ParamId id, uint8_t slot) const
        {
            const char* s = stringSlot(id, slot);
            return s ? s : "";
        }

        // Задать число (в границах, ENUM — номер значения). Равное
        // умолчанию — то же, что вернуть умолчание.
        bool setValue(ParamId id, uint8_t slot, float v)
        {
            const ParamInfo& info = paramInfo(id);
            if (slot >= PROFILES || info.type == ParamType::STRING || !(v >= info.min && v <= info.max)) return false;
            if (info.type != ParamType::FLOAT) v = roundf(v);
            if (v == defaults[id])
            {
                setMask[id] &= static_cast<uint8_t>(~(1u << slot));
            }
            else
            {
                values[id][slot] = v;
                setMask[id] |= static_cast<uint8_t>(1u << slot);
            }
            if (id == PARAM_TUNE_SOURCE) refreshDefaults();
            return true;
        }

        bool setText(ParamId id, uint8_t slot, const char* s)
        {
            char* dst = stringSlot(id, slot);
            if (!dst || !s || strlen(s) > NAME_LENGTH) return false;
            strcpy(dst, s);
            return true;
        }

        void reset(ParamId id, uint8_t slot)
        {
            char* s = stringSlot(id, slot);
            if (s) s[0] = '\0';
            if (slot < PROFILES) setMask[id] &= static_cast<uint8_t>(~(1u << slot));
            if (id == PARAM_TUNE_SOURCE) refreshDefaults();
        }

        // Всё по умолчанию, профили 0 (как `defaults` в BF).
        void resetAll()
        {
            clearValues();
            refreshDefaults();
        }

        TuneSource source() const { return static_cast<TuneSource>(static_cast<uint8_t>(value(PARAM_TUNE_SOURCE))); }

        // ---- текст, как в CLI ----

        // Разобрать и задать. false — в error почему (по-русски).
        bool parse(ParamId id, uint8_t slot, const char* textIn, char* error, size_t errorSize)
        {
            const ParamInfo& info = paramInfo(id);
            char buf[48];
            trimCopy(textIn, buf, sizeof(buf));
            switch (info.type)
            {
                case ParamType::STRING:
                {
                    const char* s = strcmp(buf, "-") == 0 ? "" : buf;
                    if (setText(id, slot, s)) return true;
                    snprintf(error, errorSize, "не длиннее %d символов", static_cast<int>(info.max));
                    return false;
                }
                case ParamType::ENUM:
                {
                    for (uint8_t i = 0; i < info.optionCount; ++i)
                    {
                        if (sameText(buf, info.options[i])) return setValue(id, slot, i);
                    }
                    int n = snprintf(error, errorSize, "допустимо:");
                    for (uint8_t i = 0; i < info.optionCount && n > 0 && static_cast<size_t>(n) < errorSize; ++i)
                    {
                        n += snprintf(error + n, errorSize - n, "%s %s", i ? "," : "", info.options[i]);
                    }
                    return false;
                }
                case ParamType::INT:
                case ParamType::FLOAT:
                default:
                {
                    char* end = nullptr;
                    const float v = info.type == ParamType::INT ? static_cast<float>(strtol(buf, &end, 10)) : strtof(buf, &end);
                    if (end != buf && *end == '\0' && setValue(id, slot, v)) return true;
                    char lo[16], hi[16];
                    formatNumber(info, info.min, lo, sizeof(lo));
                    formatNumber(info, info.max, hi, sizeof(hi));
                    snprintf(error, errorSize, "допустимо %s - %s", lo, hi);
                    return false;
                }
            }
        }

        void format(ParamId id, uint8_t slot, char* out, size_t size) const
        {
            const ParamInfo& info = paramInfo(id);
            if (info.type == ParamType::STRING)
            {
                snprintf(out, size, "%s", text(id, slot));
                return;
            }
            formatNumber(info, valueIn(id, slot), out, size);
        }

        void formatDefault(ParamId id, char* out, size_t size) const
        {
            const ParamInfo& info = paramInfo(id);
            if (info.type == ParamType::STRING)
            {
                snprintf(out, size, "%s", "");
                return;
            }
            formatNumber(info, defaults[id], out, size);
        }

        static void formatNumber(const ParamInfo& info, float v, char* out, size_t size)
        {
            if (info.type == ParamType::ENUM)
            {
                const uint8_t i = static_cast<uint8_t>(v);
                snprintf(out, size, "%s", i < info.optionCount ? info.options[i] : "?");
            }
            else if (info.type == ParamType::FLOAT)
            {
                snprintf(out, size, "%.3f", static_cast<double>(v));
            }
            else
            {
                snprintf(out, size, "%ld", lroundf(v));
            }
        }

        // ---- настройка контроллеру ----

        // Модельная настройка + источник + выбранные профили.
        CopterTune tune() const
        {
            CopterTune t = model;
            const bool bf = source() == TuneSource::BETAFLIGHT;
            if (bf) applyBetaflightBase(t);
            AxisGains* gains[3] = { &t.roll, &t.pitch, &t.yaw };
            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                for (uint8_t k = 0; k < 4; ++k)
                {
                    const PidTerm term = static_cast<PidTerm>(k);
                    const ParamId id = pidParam(axis, term);
                    // своё число BF — и в модельном режиме: задан руками, значит так и надо
                    if (bf || isSet(id, pidIndex))
                    {
                        gainTerm(*gains[axis], term) = bfToGain(term, axis, valueIn(id, pidIndex));
                        t.gainsFromModel = false;
                    }
                }
                const float center = 10.0f * valueIn(static_cast<ParamId>(PARAM_ROLL_RC_RATE + axis), rateIndex);
                const float top = 10.0f * valueIn(static_cast<ParamId>(PARAM_ROLL_SRATE + axis), rateIndex);
                t.rates[axis].centerDps = center;
                t.rates[axis].maxDps = fmaxf(center, top);
                t.rates[axis].expo = valueIn(static_cast<ParamId>(PARAM_ROLL_EXPO + axis), rateIndex) / 100.0f;
            }
            return t;
        }

        // ---- флеш ----

        // Записать заданное руками (commit — за вызывающим).
        bool save(KeyValueStore& kv) const
        {
            uint8_t blob[3 + PARAM_COUNT * PROFILES * (4 + NAME_LENGTH)];
            size_t n = 0;
            blob[n++] = STORE_VERSION;
            blob[n++] = pidIndex;
            blob[n++] = rateIndex;
            bool any = pidIndex != 0 || rateIndex != 0;
            for (uint8_t i = 0; i < PARAM_COUNT; ++i)
            {
                const ParamId id = static_cast<ParamId>(i);
                const ParamInfo& info = paramInfo(id);
                const uint16_t h = paramNameHash(info.name);
                const uint8_t slots = info.scope == ParamScope::MASTER ? 1 : PROFILES;
                for (uint8_t slot = 0; slot < slots; ++slot)
                {
                    if (!isSet(id, slot)) continue;
                    any = true;
                    blob[n++] = static_cast<uint8_t>(h);
                    blob[n++] = static_cast<uint8_t>(h >> 8);
                    if (info.type == ParamType::STRING)
                    {
                        const char* s = text(id, slot);
                        const size_t len = strlen(s);
                        blob[n++] = static_cast<uint8_t>(0x80u | slot);
                        blob[n++] = static_cast<uint8_t>(len);
                        memcpy(blob + n, s, len);
                        n += len;
                    }
                    else
                    {
                        blob[n++] = slot;
                        memcpy(blob + n, &values[id][slot], 4);
                        n += 4;
                    }
                }
            }
            if (!any)
            {
                kv.remove(STORE_SPACE, STORE_KEY);
                return true;
            }
            return kv.put(STORE_SPACE, STORE_KEY, blob, n);
        }

        // Прочитать из флеша. false — там ничего нет или не та версия: всё по умолчанию.
        bool load(KeyValueStore& kv)
        {
            clearValues();
            size_t length = 0;
            const uint8_t* data = kv.get(STORE_SPACE, STORE_KEY, &length);
            const bool ok = data && length >= 3 && data[0] == STORE_VERSION;
            if (ok)
            {
                pidIndex = data[1] < PROFILES ? data[1] : 0;
                rateIndex = data[2] < PROFILES ? data[2] : 0;
                size_t at = 3;
                while (at + 3 <= length)
                {
                    const uint16_t h = static_cast<uint16_t>(data[at] | (data[at + 1] << 8));
                    const uint8_t tag = data[at + 2];
                    const bool isText = (tag & 0x80u) != 0;
                    const uint8_t slot = tag & 0x7Fu;
                    at += 3;
                    const int found = findHash(h);
                    if (isText)
                    {
                        if (at >= length) break;
                        const uint8_t len = data[at++];
                        if (at + len > length) break;
                        char s[NAME_LENGTH + 1] = {};
                        if (found >= 0 && len <= NAME_LENGTH)
                        {
                            memcpy(s, data + at, len);
                            setText(static_cast<ParamId>(found), slot, s);
                        }
                        at += len;
                    }
                    else
                    {
                        if (at + 4 > length) break;
                        float v;
                        memcpy(&v, data + at, 4);
                        at += 4;
                        if (found < 0 || slot >= PROFILES) continue;   // настройки больше нет
                        const ParamInfo& info = paramInfo(static_cast<ParamId>(found));
                        if (info.type == ParamType::STRING || isnan(v)) continue;
                        if (info.type == ParamType::ENUM && (v < 0 || v >= info.optionCount)) continue;
                        values[found][slot] = fminf(info.max, fmaxf(info.min, v));
                        setMask[found] |= static_cast<uint8_t>(1u << slot);
                    }
                }
            }
            refreshDefaults();
            return ok;
        }

    private:
        void clearValues()
        {
            memset(values, 0, sizeof(values));
            memset(setMask, 0, sizeof(setMask));
            memset(names, 0, sizeof(names));
            pidIndex = 0;
            rateIndex = 0;
        }

        // Умолчания: из таблицы, PID и расходы — из модели или BF 4.5.
        void refreshDefaults()
        {
            for (uint8_t i = 0; i < PARAM_COUNT; ++i) defaults[i] = paramTable()[i].def;
            const bool bf = source() == TuneSource::BETAFLIGHT;
            const AxisGains* gains[3] = { &model.roll, &model.pitch, &model.yaw };
            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                for (uint8_t k = 0; k < 4; ++k)
                {
                    const PidTerm term = static_cast<PidTerm>(k);
                    const ParamId id = pidParam(axis, term);
                    const ParamInfo& info = paramInfo(id);
                    const float v = bf ? betaflight::DEFAULT_PID[axis][k] : roundf(gainToBf(term, axis, gainTerm(*gains[axis], term)));
                    defaults[id] = fminf(info.max, fmaxf(info.min, v));
                }
                const RateCurve& r = model.rates[axis];
                defaults[PARAM_ROLL_RC_RATE + axis] = bf ? betaflight::DEFAULT_RC_RATE : fmaxf(1.0f, roundf(r.centerDps / 10.0f));
                defaults[PARAM_ROLL_SRATE + axis] = bf ? betaflight::DEFAULT_SRATE : fminf(255.0f, roundf(r.maxDps / 10.0f));
                defaults[PARAM_ROLL_EXPO + axis] = bf ? betaflight::DEFAULT_EXPO : roundf(r.expo * 100.0f);
            }
        }

        int findHash(uint16_t h) const
        {
            for (int i = 0; i < PARAM_COUNT; ++i)
            {
                if (paramNameHash(paramTable()[i].name) == h) return i;
            }
            return -1;
        }

        char* stringSlot(ParamId id, uint8_t slot)
        {
            if (slot >= PROFILES) return nullptr;
            if (id == PARAM_PROFILE_NAME) return names[0][slot];
            if (id == PARAM_RATEPROFILE_NAME) return names[1][slot];
            return nullptr;
        }
        const char* stringSlot(ParamId id, uint8_t slot) const { return const_cast<CopterParams*>(this)->stringSlot(id, slot); }

        static bool sameText(const char* a, const char* b)
        {
            while (*a && *b && tolower(static_cast<unsigned char>(*a)) == tolower(static_cast<unsigned char>(*b)))
            {
                ++a;
                ++b;
            }
            return *a == '\0' && *b == '\0';
        }

        static void trimCopy(const char* in, char* out, size_t size)
        {
            if (!in) in = "";
            while (*in == ' ' || *in == '\t') ++in;
            size_t n = 0;
            while (in[n] && n + 1 < size)
            {
                out[n] = in[n];
                ++n;
            }
            while (n > 0 && (out[n - 1] == ' ' || out[n - 1] == '\t' || out[n - 1] == '\r' || out[n - 1] == '\n')) --n;
            out[n] = '\0';
        }

        CopterTune model;
        float defaults[PARAM_COUNT] = {};
        float values[PARAM_COUNT][PROFILES];
        uint8_t setMask[PARAM_COUNT];
        char names[2][PROFILES][NAME_LENGTH + 1];
        uint8_t pidIndex = 0;
        uint8_t rateIndex = 0;
    };
}
