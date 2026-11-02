#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "copter/CopterController.h"
#include "copter/Params.h"

// ============================================================
// Меню настроек в очках стиками (CMS в Betaflight) — PID, расходы,
// ползунки, VTX и учебные ограничения без компьютера.
//
// Вход — без ARM, связь есть: газ в центре, рыск влево, тангаж от себя,
// крен в центре — 0.5 с (как в BF). Дальше стики сначала отпустить в центр
// (газ — как угодно), потом:
//   тангаж от себя / на себя — строка выше / ниже (держать — повтор);
//   крен вправо — войти в раздел / значение больше;
//   крен влево  — значение меньше;
//   рыск влево  — назад (из главного — выйти без сохранения).
// «SAVE+EXIT» — записать во флеш и применить, «EXIT» — отменить всё, что
// изменено в меню (снова из флеша). Пока меню открыто — ARM запрещён (CMS
// в причинах), команды стиками молчат; ARM или потеря связи — меню
// закрывается без сохранения.
//
// Код свой (поведение — по описанию CMS в документации BF). Без Arduino —
// тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    class OsdMenu
    {
    public:
        enum class Exit : uint8_t
        {
            NONE,
            SAVE,      // записать и применить (как save в CLI)
            DISCARD,   // отменить изменения (снова из флеша)
        };

        static constexpr float ENTER_DELAY_S = 0.5f;
        static constexpr float REPEAT_FIRST_S = 0.4f;   // держать стик — первый повтор
        static constexpr float REPEAT_S = 0.15f;        // дальше
        static constexpr float REPEAT_FAST_S = 0.05f;   // держать дольше 2 с
        static constexpr float FAST_AFTER_S = 2.0f;

        // min_check / max_check, мкс — те же, что у команд стиками.
        void configure(uint16_t minCheckUs, uint16_t maxCheckUs)
        {
            minCheck = minCheckUs;
            maxCheck = maxCheckUs;
        }

        bool isOpen() const { return open; }

        // Каждый цикл (или 50 Гц). Возвращает, чем закрылось меню.
        Exit update(const CopterRc& rc, bool armed, float dt, CopterParams& params)
        {
            if (!open)
            {
                const bool entering = !armed && rc.linkOk && zone(1000.0f + rc.throttle * 1000.0f) == MID &&
                                      zone(1500.0f + rc.yaw * 500.0f) == LO &&
                                      zone(1500.0f - rc.pitch * 500.0f) == HI && zone(1500.0f + rc.roll * 500.0f) == MID;
                enterHeld = entering ? enterHeld + dt : 0.0f;
                if (enterHeld < ENTER_DELAY_S) return Exit::NONE;
                open = true;
                page = PAGE_MAIN;
                cursor = 0;
                top = 0;
                waitCenter = true;
                key = NONE;
                return Exit::NONE;
            }
            if (armed || !rc.linkOk) return close(Exit::DISCARD);

            const Key k = keyOf(rc);
            if (waitCenter)
            {
                if (k != NONE) return Exit::NONE;
                waitCenter = false;
            }
            if (k != key)
            {
                key = k;
                held = 0.0f;
                nextAt = 0.0f;
            }
            else
            {
                held += dt;
            }
            if (k == NONE || held < nextAt) return Exit::NONE;
            nextAt = held + (held == 0.0f ? REPEAT_FIRST_S : held >= FAST_AFTER_S ? REPEAT_FAST_S : REPEAT_S);
            return press(k, held > 0.0f, params);
        }

        // Экран меню вместо обычного OSD.
        void draw(OsdScreen& s, const CopterParams& params) const
        {
            s.clear();
            if (!open) return;
            char title[OsdScreen::COLS + 1];
            pageTitle(params, title, sizeof(title));
            s.print(0, static_cast<uint8_t>((OsdScreen::COLS - strlen(title)) / 2), title);

            const uint8_t visible = visibleRows(s.rowCount());
            const uint8_t n = itemCount(page);
            for (uint8_t i = 0; i < visible && top + i < n; ++i)
            {
                const uint8_t row = static_cast<uint8_t>(FIRST_ROW + i);
                const Item& it = items(page)[top + i];
                if (top + i == cursor) s.print(row, 1, ">");
                s.print(row, 2, it.label);
                char v[12];
                if (valueText(it, params, v, sizeof(v)))
                {
                    s.print(row, static_cast<uint8_t>(OsdScreen::COLS - 1 - strlen(v)), v);
                }
            }
            if (top > 0) s.print(FIRST_ROW - 1, OsdScreen::COLS - 2, "^");
            if (top + visible < n) s.print(static_cast<uint8_t>(FIRST_ROW + visible), OsdScreen::COLS - 2, "v");
            if (s.rowCount() > FIRST_ROW + visible + 1)
            {
                s.print(static_cast<uint8_t>(s.rowCount() - 1), 0, "PITCH MOVE  ROLL SET  YAW BACK");
            }
        }

        // Для тестов и приложения: где курсор.
        uint8_t currentPage() const { return page; }
        uint8_t cursorRow() const { return cursor; }

        enum Page : uint8_t
        {
            PAGE_MAIN,
            PAGE_PROFILES,
            PAGE_PID,
            PAGE_RATES,
            PAGE_SLIDERS,
            PAGE_VTX,
            PAGE_TRAINING,
            PAGE_COUNT
        };

    private:
        enum Zone : uint8_t { LO, MID, HI };
        enum Key : uint8_t { NONE, UP, DOWN, RIGHT, LEFT, BACK };

        enum class Kind : uint8_t
        {
            PAGE,           // value — номер страницы
            PARAM,          // id — настройка в её активном профиле
            PID_PROFILE,
            RATE_PROFILE,
            SAVE_EXIT,
            EXIT,
        };

        struct Item
        {
            const char* label;
            Kind kind;
            uint16_t value;   // PAGE: страница, PARAM: ParamId
        };

        static constexpr uint8_t FIRST_ROW = 2;

        static const Item* items(uint8_t p)
        {
            static const Item MAIN[] = {
                { "PROFILES", Kind::PAGE, PAGE_PROFILES },
                { "PID", Kind::PAGE, PAGE_PID },
                { "RATES", Kind::PAGE, PAGE_RATES },
                { "SLIDERS", Kind::PAGE, PAGE_SLIDERS },
                { "VTX", Kind::PAGE, PAGE_VTX },
                { "TRAINING", Kind::PAGE, PAGE_TRAINING },
                { "SAVE+EXIT", Kind::SAVE_EXIT, 0 },
                { "EXIT", Kind::EXIT, 0 },
            };
            static const Item PROFILES[] = {
                { "PID PROFILE", Kind::PID_PROFILE, 0 },
                { "RATE PROFILE", Kind::RATE_PROFILE, 0 },
            };
            static const Item PID[] = {
                { "ROLL P", Kind::PARAM, PARAM_P_ROLL },   { "ROLL I", Kind::PARAM, PARAM_I_ROLL },
                { "ROLL D", Kind::PARAM, PARAM_D_ROLL },   { "ROLL F", Kind::PARAM, PARAM_F_ROLL },
                { "PITCH P", Kind::PARAM, PARAM_P_PITCH }, { "PITCH I", Kind::PARAM, PARAM_I_PITCH },
                { "PITCH D", Kind::PARAM, PARAM_D_PITCH }, { "PITCH F", Kind::PARAM, PARAM_F_PITCH },
                { "YAW P", Kind::PARAM, PARAM_P_YAW },     { "YAW I", Kind::PARAM, PARAM_I_YAW },
                { "YAW D", Kind::PARAM, PARAM_D_YAW },     { "YAW F", Kind::PARAM, PARAM_F_YAW },
            };
            static const Item RATES[] = {
                { "ROLL RC RATE", Kind::PARAM, PARAM_ROLL_RC_RATE },   { "ROLL SUPER", Kind::PARAM, PARAM_ROLL_SRATE },
                { "ROLL EXPO", Kind::PARAM, PARAM_ROLL_EXPO },         { "PITCH RC RATE", Kind::PARAM, PARAM_PITCH_RC_RATE },
                { "PITCH SUPER", Kind::PARAM, PARAM_PITCH_SRATE },     { "PITCH EXPO", Kind::PARAM, PARAM_PITCH_EXPO },
                { "YAW RC RATE", Kind::PARAM, PARAM_YAW_RC_RATE },     { "YAW SUPER", Kind::PARAM, PARAM_YAW_SRATE },
                { "YAW EXPO", Kind::PARAM, PARAM_YAW_EXPO },
            };
            static const Item SLIDERS[] = {
                { "PIDS MODE", Kind::PARAM, PARAM_SIMPLIFIED_PIDS_MODE },
                { "MASTER", Kind::PARAM, PARAM_SIMPLIFIED_MASTER_MULTIPLIER },
                { "PI GAIN", Kind::PARAM, PARAM_SIMPLIFIED_PI_GAIN },
                { "I GAIN", Kind::PARAM, PARAM_SIMPLIFIED_I_GAIN },
                { "D GAIN", Kind::PARAM, PARAM_SIMPLIFIED_D_GAIN },
                { "D MAX GAIN", Kind::PARAM, PARAM_SIMPLIFIED_DMAX_GAIN },
                { "FF GAIN", Kind::PARAM, PARAM_SIMPLIFIED_FEEDFORWARD_GAIN },
                { "PITCH D", Kind::PARAM, PARAM_SIMPLIFIED_PITCH_D_GAIN },
                { "PITCH PI", Kind::PARAM, PARAM_SIMPLIFIED_PITCH_PI_GAIN },
                { "GYRO FILTER", Kind::PARAM, PARAM_SIMPLIFIED_GYRO_FILTER_MULTIPLIER },
                { "DTERM FILTER", Kind::PARAM, PARAM_SIMPLIFIED_DTERM_FILTER_MULTIPLIER },
            };
            static const Item VTX[] = {
                { "BAND", Kind::PARAM, PARAM_VTX_BAND },
                { "CHANNEL", Kind::PARAM, PARAM_VTX_CHANNEL },
                { "POWER", Kind::PARAM, PARAM_VTX_POWER },
                { "LOW POWER DISARM", Kind::PARAM, PARAM_VTX_LOW_POWER_DISARM },
            };
            static const Item TRAINING[] = {
                { "LIMITS", Kind::PARAM, PARAM_TRAINING_LIMITS },
                { "MAX ALT M", Kind::PARAM, PARAM_TRAINING_MAX_ALT },
                { "MAX DIST M", Kind::PARAM, PARAM_TRAINING_MAX_DIST },
                { "MAX ANGLE", Kind::PARAM, PARAM_TRAINING_MAX_ANGLE },
                { "MAX KMH", Kind::PARAM, PARAM_TRAINING_MAX_SPEED },
            };
            switch (p)
            {
                case PAGE_PROFILES: return PROFILES;
                case PAGE_PID: return PID;
                case PAGE_RATES: return RATES;
                case PAGE_SLIDERS: return SLIDERS;
                case PAGE_VTX: return VTX;
                case PAGE_TRAINING: return TRAINING;
                default: return MAIN;
            }
        }

        static uint8_t itemCount(uint8_t p)
        {
            switch (p)
            {
                case PAGE_PROFILES: return 2;
                case PAGE_PID: return 12;
                case PAGE_RATES: return 9;
                case PAGE_SLIDERS: return 11;
                case PAGE_VTX: return 4;
                case PAGE_TRAINING: return 5;
                default: return 8;
            }
        }

        static const char* pageName(uint8_t p)
        {
            static const char* const names[PAGE_COUNT] = { "MENU", "PROFILES", "PID", "RATES",
                                                           "SLIDERS", "VTX", "TRAINING" };
            return p < PAGE_COUNT ? names[p] : "MENU";
        }

        void pageTitle(const CopterParams& params, char* out, size_t n) const
        {
            if (page == PAGE_PID || page == PAGE_SLIDERS)
                snprintf(out, n, "%s  PROFILE %u", pageName(page), params.pidProfile() + 1);
            else if (page == PAGE_RATES)
                snprintf(out, n, "%s  PROFILE %u", pageName(page), params.rateProfile() + 1);
            else
                snprintf(out, n, "%s", pageName(page));
        }

        static bool valueText(const Item& it, const CopterParams& params, char* out, size_t n)
        {
            switch (it.kind)
            {
                case Kind::PID_PROFILE: snprintf(out, n, "%u", params.pidProfile() + 1); return true;
                case Kind::RATE_PROFILE: snprintf(out, n, "%u", params.rateProfile() + 1); return true;
                case Kind::PARAM:
                {
                    const ParamId id = static_cast<ParamId>(it.value);
                    if (id == PARAM_VTX_BAND)
                    {
                        const uint8_t b = static_cast<uint8_t>(params.value(id));
                        snprintf(out, n, "%c", b >= 1 && b <= VTX_BANDS ? vtxBandLetters()[b - 1] : 'U');
                        return true;
                    }
                    params.format(id, params.activeSlot(id), out, n);
                    return true;
                }
                case Kind::PAGE: snprintf(out, n, "%s", ">"); return true;
                default: return false;
            }
        }

        uint8_t visibleRows(uint8_t screenRows) const
        {
            // заголовок, пустая строка, подсказка внизу и строка «ещё ниже»
            const int v = static_cast<int>(screenRows) - FIRST_ROW - 2;
            return static_cast<uint8_t>(v < 1 ? 1 : v);
        }

        Exit press(Key k, bool repeat, CopterParams& params)
        {
            const uint8_t n = itemCount(page);
            switch (k)
            {
                case UP:
                    cursor = cursor == 0 ? static_cast<uint8_t>(n - 1) : static_cast<uint8_t>(cursor - 1);
                    scroll();
                    return Exit::NONE;
                case DOWN:
                    cursor = static_cast<uint8_t>((cursor + 1) % n);
                    scroll();
                    return Exit::NONE;
                case BACK:
                    if (repeat) return Exit::NONE;
                    if (page == PAGE_MAIN) return close(Exit::DISCARD);
                    cursor = mainRowOf(page);
                    page = PAGE_MAIN;
                    scroll();
                    return Exit::NONE;
                case RIGHT:
                case LEFT:
                {
                    const Item& it = items(page)[cursor];
                    const int dir = k == RIGHT ? 1 : -1;
                    switch (it.kind)
                    {
                        case Kind::PAGE:
                            if (k == RIGHT && !repeat)
                            {
                                page = static_cast<uint8_t>(it.value);
                                cursor = 0;
                                top = 0;
                            }
                            return Exit::NONE;
                        case Kind::SAVE_EXIT: return k == RIGHT && !repeat ? close(Exit::SAVE) : Exit::NONE;
                        case Kind::EXIT: return k == RIGHT && !repeat ? close(Exit::DISCARD) : Exit::NONE;
                        case Kind::PID_PROFILE:
                            if (!repeat) params.selectPidProfile(step(params.pidProfile(), dir));
                            return Exit::NONE;
                        case Kind::RATE_PROFILE:
                            if (!repeat) params.selectRateProfile(step(params.rateProfile(), dir));
                            return Exit::NONE;
                        case Kind::PARAM:
                        {
                            const ParamId id = static_cast<ParamId>(it.value);
                            const ParamInfo& info = paramInfo(id);
                            const float inc = info.type == ParamType::FLOAT ? 0.1f : 1.0f;
                            const float v = fminf(info.max, fmaxf(info.min, params.value(id) + dir * inc));
                            params.setValue(id, params.activeSlot(id), v);
                            return Exit::NONE;
                        }
                    }
                    return Exit::NONE;
                }
                default: return Exit::NONE;
            }
        }

        static uint8_t step(uint8_t profile, int dir)
        {
            const int n = CopterParams::PROFILES;
            return static_cast<uint8_t>((profile + dir + n) % n);
        }

        static uint8_t mainRowOf(uint8_t p)
        {
            for (uint8_t i = 0; i < itemCount(PAGE_MAIN); ++i)
            {
                const Item& it = items(PAGE_MAIN)[i];
                if (it.kind == Kind::PAGE && it.value == p) return i;
            }
            return 0;
        }

        void scroll()
        {
            // строки на экране NTSC (13) — самый тесный случай
            const uint8_t visible = visibleRows(13);
            if (cursor < top) top = cursor;
            if (cursor >= top + visible) top = static_cast<uint8_t>(cursor - visible + 1);
        }

        Exit close(Exit how)
        {
            open = false;
            enterHeld = 0.0f;
            key = NONE;
            return how;
        }

        Zone zone(float us) const { return us < minCheck ? LO : us > maxCheck ? HI : MID; }

        Key keyOf(const CopterRc& rc) const
        {
            const Zone pitch = zone(1500.0f - rc.pitch * 500.0f);   // HI — стик от себя
            const Zone roll = zone(1500.0f + rc.roll * 500.0f);
            const Zone yaw = zone(1500.0f + rc.yaw * 500.0f);
            if (yaw == LO) return BACK;
            if (pitch == HI) return UP;
            if (pitch == LO) return DOWN;
            if (roll == HI) return RIGHT;
            if (roll == LO) return LEFT;
            return NONE;
        }

        uint16_t minCheck = 1050, maxCheck = 1900;
        bool open = false;
        bool waitCenter = false;
        float enterHeld = 0.0f;
        uint8_t page = PAGE_MAIN;
        uint8_t cursor = 0;
        uint8_t top = 0;
        Key key = NONE;
        float held = 0.0f;
        float nextAt = 0.0f;
    };
}
