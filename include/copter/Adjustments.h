#pragma once

// Подстройки в полёте (121/122): строки `adjrange` (copter/AdjRanges.h) ->
// настройки текущего профиля. Только в RAM: склейка пересчитывает контроллер
// (applyProfile) и пишет во флеш после DISARM (как INAV/старый BF).
// Без связи с пультом, при открытом CLI и меню в очках — ничего не делает.

#include <stdint.h>
#include "copter/AdjRanges.h"
#include "copter/Params.h"

namespace copter
{
    // Какие настройки меняет функция (до двух: крен и тангаж вместе).
    struct AdjTargets
    {
        ParamId ids[2] = { PARAM_COUNT, PARAM_COUNT };
        uint8_t count = 0;
    };

    inline AdjTargets adjTargets(uint8_t f)
    {
        AdjTargets t;
        auto one = [&](ParamId a) { t.ids[0] = a, t.count = 1; };
        auto two = [&](ParamId a, ParamId b) { t.ids[0] = a, t.ids[1] = b, t.count = 2; };
        auto rp = [&](PidTerm term) { two(pidParam(0, term), pidParam(1, term)); };
        switch (f)
        {
            case ADJ_RC_RATE: two(PARAM_ROLL_RC_RATE, PARAM_PITCH_RC_RATE); break;
            case ADJ_RC_EXPO: two(PARAM_ROLL_EXPO, PARAM_PITCH_EXPO); break;
            case ADJ_THROTTLE_EXPO: one(PARAM_THR_EXPO); break;
            case ADJ_PITCH_ROLL_RATE: two(PARAM_ROLL_SRATE, PARAM_PITCH_SRATE); break;
            case ADJ_YAW_RATE: one(PARAM_YAW_SRATE); break;
            case ADJ_PITCH_ROLL_P: rp(PidTerm::P); break;
            case ADJ_PITCH_ROLL_I: rp(PidTerm::I); break;
            case ADJ_PITCH_ROLL_D: rp(PidTerm::D); break;
            case ADJ_YAW_P: one(pidParam(2, PidTerm::P)); break;
            case ADJ_YAW_I: one(pidParam(2, PidTerm::I)); break;
            case ADJ_YAW_D: one(pidParam(2, PidTerm::D)); break;
            case ADJ_PITCH_RATE: one(PARAM_PITCH_SRATE); break;
            case ADJ_ROLL_RATE: one(PARAM_ROLL_SRATE); break;
            case ADJ_PITCH_P: one(pidParam(1, PidTerm::P)); break;
            case ADJ_PITCH_I: one(pidParam(1, PidTerm::I)); break;
            case ADJ_PITCH_D: one(pidParam(1, PidTerm::D)); break;
            case ADJ_ROLL_P: one(pidParam(0, PidTerm::P)); break;
            case ADJ_ROLL_I: one(pidParam(0, PidTerm::I)); break;
            case ADJ_ROLL_D: one(pidParam(0, PidTerm::D)); break;
            case ADJ_RC_RATE_YAW: one(PARAM_YAW_RC_RATE); break;
            case ADJ_PITCH_ROLL_F: rp(PidTerm::F); break;
            case ADJ_FEEDFORWARD_TRANSITION: one(PARAM_FEEDFORWARD_TRANSITION); break;
            case ADJ_HORIZON_STRENGTH: one(PARAM_HORIZON_LEVEL_STRENGTH); break;
            case ADJ_ROLL_RC_RATE: one(PARAM_ROLL_RC_RATE); break;
            case ADJ_PITCH_RC_RATE: one(PARAM_PITCH_RC_RATE); break;
            case ADJ_ROLL_RC_EXPO: one(PARAM_ROLL_EXPO); break;
            case ADJ_PITCH_RC_EXPO: one(PARAM_PITCH_EXPO); break;
            case ADJ_PITCH_F: one(pidParam(1, PidTerm::F)); break;
            case ADJ_ROLL_F: one(pidParam(0, PidTerm::F)); break;
            case ADJ_YAW_F: one(pidParam(2, PidTerm::F)); break;
            default: break;
        }
        return t;
    }

    class InflightAdjuster
    {
    public:
        static constexpr uint16_t STEP_UP_US = 1700, STEP_DOWN_US = 1300;
        static constexpr uint32_t REPEAT_MS = 500;      // держат — ещё шаг
        static constexpr uint8_t SELECT_POSITIONS = 3;  // профиль: три положения тумблера

        struct Event
        {
            bool shown = false;     // было что показать в очках (шаг упёрся в границу — тоже)
            bool changed = false;   // настройки или профиль поменялись
            bool profile = false;   // выбран профиль (value — его номер с 1)
            uint8_t function = ADJ_NONE;
            int value = 0;
        };

        void reset()
        {
            for (auto& s : state) s = SlotState();
        }

        Event update(const AdjRangeTable& table, CopterParams& params, const uint16_t* aux, uint8_t auxN, bool enabled,
                     uint32_t nowMs)
        {
            Event ev;
            if (!enabled || !aux)
            {
                reset();
                return ev;
            }
            for (uint8_t i = 0; i < AdjRangeTable::SLOTS; ++i)
            {
                const AdjRange& r = table.range(i);
                SlotState& s = state[i];
                const AdjKind kind = adjFunctionInfo(r.function).kind;
                if (!r.used() || kind == AdjKind::UNSUPPORTED || r.rangeChannel >= auxN || r.adjChannel >= auxN ||
                    !inRange(aux[r.rangeChannel], r))
                {
                    s = SlotState();
                    continue;
                }
                const uint16_t us = aux[r.adjChannel];
                if (kind == AdjKind::SELECT) select(r.function, us, params, ev);
                else if (r.center) absolute(r, us, s, params, ev);
                else step(r.function, us, s, params, nowMs, ev);
            }
            return ev;
        }

        // Положение тумблера -> номер профиля (0..2), как в BF: 900..2100 на три части.
        static uint8_t selectPosition(uint16_t us)
        {
            const uint16_t width = (AdjRangeTable::MAX_US - AdjRangeTable::MIN_US) / SELECT_POSITIONS;
            if (us < AdjRangeTable::MIN_US) us = AdjRangeTable::MIN_US;
            if (us >= AdjRangeTable::MAX_US) us = AdjRangeTable::MAX_US - 1;
            return static_cast<uint8_t>((us - AdjRangeTable::MIN_US) / width);
        }

        // Абсолютно: центр + (мкс − 1500) · масштаб / 500.
        static int absoluteValue(const AdjRange& r, uint16_t us)
        {
            int u = us < 1000 ? 1000 : us > 2000 ? 2000 : us;
            return static_cast<int>(r.center) + (u - 1500) * static_cast<int>(r.scale) / 500;
        }

    private:
        struct SlotState
        {
            bool ready = true;        // тумблер был в середине — следующий шаг сразу
            uint32_t repeatAtMs = 0;
            bool hasAbs = false;
            int lastAbs = 0;
        };

        static bool inRange(uint16_t us, const AdjRange& r)
        {
            if (us < AdjRangeTable::MIN_US) us = AdjRangeTable::MIN_US;
            if (us >= AdjRangeTable::MAX_US) us = AdjRangeTable::MAX_US - 1;
            return us >= r.startUs && us < r.endUs;
        }

        // Записать значение (или прибавить шаг) во все настройки функции, в границах; true — что-то поменялось.
        static bool apply(const AdjTargets& t, int value, bool relative, CopterParams& params, int& shown)
        {
            bool changed = false;
            for (uint8_t k = 0; k < t.count; ++k)
            {
                const ParamId id = t.ids[k];
                const ParamInfo& info = paramInfo(id);
                float v = relative ? params.value(id) + static_cast<float>(value) : static_cast<float>(value);
                if (v < info.min) v = info.min;
                if (v > info.max) v = info.max;
                const float old = params.value(id);
                if (v != old && params.setValue(id, params.activeSlot(id), v)) changed = true;
                if (k == 0) shown = static_cast<int>(params.value(id));
            }
            return changed;
        }

        static void select(uint8_t f, uint16_t us, CopterParams& params, Event& ev)
        {
            const uint8_t pos = selectPosition(us);
            const bool rate = f == ADJ_RATE_PROFILE;
            const uint8_t current = rate ? params.rateProfile() : params.pidProfile();
            if (pos == current || pos >= CopterParams::PROFILES) return;
            if (!(rate ? params.selectRateProfile(pos) : params.selectPidProfile(pos))) return;
            ev.shown = ev.changed = ev.profile = true;
            ev.function = f;
            ev.value = pos + 1;
        }

        static void absolute(const AdjRange& r, uint16_t us, SlotState& s, CopterParams& params, Event& ev)
        {
            const int value = absoluteValue(r, us);
            if (s.hasAbs && value == s.lastAbs) return;
            s.hasAbs = true;
            s.lastAbs = value;
            int shown = 0;
            const bool changed = apply(adjTargets(r.function), value, false, params, shown);
            if (!changed) return;   // ручка на том же, что уже стоит, — молчим
            ev.shown = ev.changed = true;
            ev.profile = false;
            ev.function = r.function;
            ev.value = shown;
        }

        static void step(uint8_t f, uint16_t us, SlotState& s, CopterParams& params, uint32_t nowMs, Event& ev)
        {
            const int delta = us > STEP_UP_US ? 1 : us < STEP_DOWN_US ? -1 : 0;
            if (!delta)
            {
                s.ready = true;
                return;
            }
            if (!s.ready && static_cast<int32_t>(nowMs - s.repeatAtMs) < 0) return;
            s.ready = false;
            s.repeatAtMs = nowMs + REPEAT_MS;
            const AdjTargets t = adjTargets(f);
            if (!t.count) return;
            int shown = 0;
            const bool changed = apply(t, delta, true, params, shown);
            ev.shown = true;
            ev.changed = ev.changed || changed;
            ev.profile = false;
            ev.function = f;
            ev.value = shown;
        }

        SlotState state[AdjRangeTable::SLOTS];
    };
}
