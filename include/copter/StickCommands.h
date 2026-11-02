#pragma once
#include <stdint.h>

#include "copter/CopterController.h"

// ============================================================
// Команды стиками — как в Betaflight (только без ARM, связь есть):
//
//   газ  рыск  тангаж крен
//   низ  лево  низ    центр — калибровка гироскопа (и барометра, дом GPS)
//   низ  лево  центр  лево  — PID-профиль 1   ┐ подтверждение — столько
//   низ  лево  верх   центр — PID-профиль 2   │ коротких писков
//   низ  лево  центр  право — PID-профиль 3   ┘
//   низ  лево  низ    право — сохранить настройки (save)
//   верх лево  низ    центр — калибровка акселерометра (горизонт = как
//                             стоит сейчас: acc_trim_roll/pitch)
//   верх центр …            — ANGLE/HORIZON: подстройка горизонта
//                             (acc_trim_*) на 0.1° туда, куда стик тангажа
//                             или крена, повтор каждые 0.25 с; ACRO —
//                             профиль расходов 1–4 (тангаж верх, низ,
//                             крен право, лево)
// «Низ/верх» — дальше min_check/max_check (1050/1900 мкс, как в BF).
// Команда — когда положение продержалось 50 мс, один раз до смены
// положения (подстройка — с повтором). Режим STICK COMMANDS DISABLE — нет
// команд (ARM/DISARM стиками остаются).
//
// ARM стиками — если ARM нет в таблице режимов (как в BF): газ низ + рыск
// вправо 0.5 с — ARM (все проверки те же, что у тумблера), газ низ + рыск
// влево 0.5 с — DISARM. Отличие от BF: DISARM стиками — только на земле
// (в воздухе газ вниз и рыск влево — обычный манёвр, а не «выключить моторы»).
//
// Без Arduino — тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    enum class StickAction : uint8_t
    {
        NONE,
        CALIBRATE_GYRO,
        CALIBRATE_ACC,
        PID_PROFILE,    // value — номер 0..2
        RATE_PROFILE,   // value — номер 0..3
        SAVE,
        TRIM,           // trimRoll / trimPitch: ±1 (0.1°), + — вправо / нос вниз (как acc_trim_* в BF)
    };

    struct StickCommand
    {
        StickAction action = StickAction::NONE;
        uint8_t value = 0;
        int8_t trimRoll = 0;
        int8_t trimPitch = 0;
    };

    class StickCommands
    {
    public:
        static constexpr float COMMAND_DELAY_S = 0.05f;   // STICK_DELAY_MS BF
        static constexpr float ARM_DELAY_S = 0.5f;        // ARM_DELAY_MS BF
        static constexpr float REPEAT_S = 0.25f;          // STICK_AUTOREPEAT_MS BF

        // min_check / max_check, мкс (стик 1000…2000).
        void configure(uint16_t minCheckUs, uint16_t maxCheckUs)
        {
            minCheck = minCheckUs;
            maxCheck = maxCheckUs;
        }

        // Каждый цикл, до контроллера. ARM нет в таблице — rc.armSwitch
        // заменяется «тумблером из стиков» (и rc.armAssigned = true).
        // levelMode — ANGLE/HORIZON (подстройка горизонта, иначе расходы),
        // disabled — режим STICK COMMANDS DISABLE.
        StickCommand update(CopterRc& rc, bool armed, bool airborne, bool levelMode, bool disabled, float dt)
        {
            StickCommand out;
            const uint8_t pos = rc.linkOk ? position(rc) : 0xFF;
            if (pos == lastPos) held += dt;
            else
            {
                held = 0.0f;
                done = false;
                repeatAt = 0.0f;
            }
            lastPos = pos;

            stickArming = !rc.armAssigned;
            disarmHeld = false;
            if (stickArming)
            {
                if (!armed) stickArm = false;   // DISARM защитой — тумблер «выключен», ARM — заново стиками
                if (pos == (THR_LO | YAW_HI | PIT_CE | ROL_CE) && held >= ARM_DELAY_S && !armed) stickArm = true;
                if (pos == (THR_LO | YAW_LO | PIT_CE | ROL_CE))
                {
                    disarmHeld = !armed;
                    if (held >= ARM_DELAY_S && armed && !airborne) stickArm = false;
                }
                rc.armSwitch = stickArm;
                rc.armAssigned = true;
            }

            if (armed || disabled || pos == 0xFF || held < COMMAND_DELAY_S) return out;
            const bool trim = levelMode && (pos & THR_MASK) == THR_HI && (pos & YAW_MASK) == YAW_CE;
            if (done && !(trim && held >= repeatAt)) return out;
            done = true;

            switch (pos)
            {
                case THR_LO | YAW_LO | PIT_LO | ROL_CE: out.action = StickAction::CALIBRATE_GYRO; return out;
                case THR_LO | YAW_LO | PIT_CE | ROL_LO: return profile(out, 0);
                case THR_LO | YAW_LO | PIT_HI | ROL_CE: return profile(out, 1);
                case THR_LO | YAW_LO | PIT_CE | ROL_HI: return profile(out, 2);
                case THR_LO | YAW_LO | PIT_LO | ROL_HI: out.action = StickAction::SAVE; return out;
                case THR_HI | YAW_LO | PIT_LO | ROL_CE: out.action = StickAction::CALIBRATE_ACC; return out;
                default: break;
            }
            if ((pos & THR_MASK) != THR_HI || (pos & YAW_MASK) != YAW_CE) return out;
            const uint8_t rp = pos & (PIT_MASK | ROL_MASK);
            if (trim)
            {
                // стик тангажа от себя (верх) — горизонт ниже носом, крен вправо — правее
                if (rp == (PIT_HI | ROL_CE)) out.trimPitch = 1;
                else if (rp == (PIT_LO | ROL_CE)) out.trimPitch = -1;
                else if (rp == (PIT_CE | ROL_HI)) out.trimRoll = 1;
                else if (rp == (PIT_CE | ROL_LO)) out.trimRoll = -1;
                if (out.trimPitch || out.trimRoll)
                {
                    out.action = StickAction::TRIM;
                    repeatAt = held + REPEAT_S;
                }
                return out;
            }
            uint8_t rate = 0xFF;
            if (rp == (PIT_HI | ROL_CE)) rate = 0;
            else if (rp == (PIT_LO | ROL_CE)) rate = 1;
            else if (rp == (PIT_CE | ROL_HI)) rate = 2;
            else if (rp == (PIT_CE | ROL_LO)) rate = 3;
            if (rate != 0xFF)
            {
                out.action = StickAction::RATE_PROFILE;
                out.value = rate;
            }
            return out;
        }

        bool usingSticksToArm() const { return stickArming; }
        // Стики в положении DISARM без ARM — для писка DISARM_REPEAT.
        bool disarmPosition() const { return disarmHeld; }

    private:
        // Положение стиков: по 2 бита на ось (низ / центр / верх), как rcSticks в BF.
        enum : uint8_t
        {
            ROL_LO = 1 << 0, ROL_CE = 3 << 0, ROL_HI = 2 << 0, ROL_MASK = 3 << 0,
            PIT_LO = 1 << 2, PIT_CE = 3 << 2, PIT_HI = 2 << 2, PIT_MASK = 3 << 2,
            YAW_LO = 1 << 4, YAW_CE = 3 << 4, YAW_HI = 2 << 4, YAW_MASK = 3 << 4,
            THR_LO = 1 << 6, THR_CE = 3 << 6, THR_HI = 2 << 6, THR_MASK = 3 << 6,
        };

        uint8_t zone(float us) const { return us < minCheck ? 1 : us > maxCheck ? 2 : 3; }

        uint8_t position(const CopterRc& rc) const
        {
            // стик тангажа «верх» — от себя (у нас + — на себя)
            return static_cast<uint8_t>(zone(1500.0f + rc.roll * 500.0f) | zone(1500.0f - rc.pitch * 500.0f) << 2 |
                                        zone(1500.0f + rc.yaw * 500.0f) << 4 | zone(1000.0f + rc.throttle * 1000.0f) << 6);
        }

        static StickCommand profile(StickCommand& out, uint8_t n)
        {
            out.action = StickAction::PID_PROFILE;
            out.value = n;
            return out;
        }

        uint16_t minCheck = 1050, maxCheck = 1900;
        uint8_t lastPos = 0xFF;
        float held = 0.0f;
        float repeatAt = 0.0f;
        bool done = false;
        bool stickArming = false;
        bool stickArm = false;
        bool disarmHeld = false;
    };
}
