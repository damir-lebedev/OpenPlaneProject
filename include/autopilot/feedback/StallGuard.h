#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/SpeedEstimator.h"

// ============================================================
// STALL GUARD — защита от потери скорости и сваливания
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Автопилот может честно выровнять нос — и при этом потерять
// скорость: задрал нос, газа мало, крыло перестало держать. Поэтому
// сверху над выравниванием стоит контроль энергии. Два уровня:
//
//   LowEnergy — скорость уходит: быстро падает (по IMU) при поднятом
//     носе, или близка к сваливанию, или рули по тангажу потеряли
//     эффективность (оценка b ≪ ожидаемой на крейсерской — это
//     признак малой скорости даже без датчика скорости).
//     Меры: газ не меньше LOW_ENERGY_THROTTLE_PERCENT, тангаж не
//     выше LOW_ENERGY_MAX_PITCH_DEG — не даём задирать нос дальше.
//
//   Stall — самолёт уже сваливается: скорость ниже сваливания, нос
//     резко падает, хотя руль высоты тянет вверх, или крыло резко
//     валится против элеронов при малой энергии.
//     Меры: полный газ, нос вниз (не выше STALL_MAX_PITCH_DEG),
//     крылья почти ровно, элероны ограничены — на сваливании большой
//     элерон срывает законцовку и усугубляет сваливание на крыло.
//
// Меры держатся RECOVERY_HOLD_MS после исчезновения признаков и пока
// скорость не поднимется до STALL_SPEED × LOW_SPEED_EXIT_MARGIN (без
// датчика скорости — пока она не перестанет падать), чтобы не
// дёргаться на границе. Газ не поднимается при потере связи — там
// действует failsafe (мотор выключен, планирование).
//
// Это не управление энергией целиком (как TECS в ArduPlane), а
// защита: если режим упорно просит невозможное (крутой набор на
// малом газу), самолёт будет ходить между LowEnergy и Normal, но
// скорость сваливания не потеряет.
// ============================================================

class StallGuard
{
public:

    enum class Level : uint8_t { Normal, LowEnergy, Stall };

    // Что известно о тангаже и крене от других модулей.
    struct ControlState
    {
        int8_t pitchSign = 1;            // знаки осей (ControlDirectionGuard)
        int8_t rollSign = 1;
        bool pitchEffectivenessKnown = false;
        float pitchEffectiveness = 0;    // модуль оценки b по тангажу
    };

    void reset()
    {
        level = Level::Normal;
        decelerating = false;
        reason = "";
    }

    void update(const FlightSnapshot& s, const SpeedEstimator& speed,
                const ControlState& control, bool airborne, uint32_t nowMs)
    {
        if (!airborne || !s.imuValid)
        {
            reset();
            return;
        }

        const bool lowEnergy = detectLowEnergy(s, speed, control, nowMs);
        const bool stall = detectStall(s, speed, control, lowEnergy);

        if (stall)
        {
            level = Level::Stall;
            lastStallMs = nowMs;
            lastLowEnergyMs = nowMs;
        }
        else if (level == Level::Stall && nowMs - lastStallMs < FeedbackConfig::RECOVERY_HOLD_MS)
        {
            // держим меры сваливания
        }
        else if (lowEnergy)
        {
            level = Level::LowEnergy;
            lastLowEnergyMs = nowMs;
        }
        else if (level != Level::Normal &&
                 (nowMs - lastLowEnergyMs < FeedbackConfig::RECOVERY_HOLD_MS || !energyRecovered(speed)))
        {
            level = Level::LowEnergy;   // признаки ушли, но энергия ещё не вернулась
        }
        else
        {
            level = Level::Normal;
            reason = "";
        }
    }

    Level getLevel() const { return level; }
    const char* getLevelName() const
    {
        switch (level)
        {
            case Level::Normal:    return "OK";
            case Level::LowEnergy: return "LOW_ENERGY";
            case Level::Stall:     return "STALL";
        }
        return "?";
    }

    // Почему сработало (последний признак) — для лога.
    const char* getReason() const { return reason; }

    // --- Ограничения для FeedbackSupervisor ---

    float maxPitchDeg() const
    {
        if (level == Level::Stall) return FeedbackConfig::STALL_MAX_PITCH_DEG;
        if (level == Level::LowEnergy) return FeedbackConfig::LOW_ENERGY_MAX_PITCH_DEG;
        return 90.0f;
    }

    float maxBankDeg() const
    {
        return level == Level::Stall ? FeedbackConfig::STALL_MAX_BANK_DEG : 180.0f;
    }

    // Предел отклонения элеронов, мкс.
    float maxAileronUs() const
    {
        return level == Level::Stall ? FeedbackConfig::STALL_AILERON_LIMIT_US
                                     : FeedbackConfig::MAX_DEFLECTION_US[FeedbackConfig::AXIS_ROLL];
    }

    // Нижняя граница газа, %; −1 — не ограничивать.
    float throttleFloorPercent(bool linkLost) const
    {
        if (linkLost) return -1.0f;
        if (level == Level::Stall) return FeedbackConfig::STALL_THROTTLE_PERCENT;
        if (level == Level::LowEnergy) return FeedbackConfig::LOW_ENERGY_THROTTLE_PERCENT;
        return -1.0f;
    }


private:

    Level level = Level::Normal;
    const char* reason = "";

    bool decelerating = false;
    uint32_t decelSinceMs = 0;
    uint32_t lastStallMs = 0;
    uint32_t lastLowEnergyMs = 0;

    bool detectLowEnergy(const FlightSnapshot& s, const SpeedEstimator& speed,
                         const ControlState& control, uint32_t nowMs)
    {
        // Скорость быстро падает — подтверждаем, чтобы не реагировать
        // на тряску.
        const bool decelNow = speed.hasAcceleration() &&
                              speed.getAcceleration() < -FeedbackConfig::DECEL_WARN_MS2;
        if (decelNow && !decelerating) decelSinceMs = nowMs;
        decelerating = decelNow;
        const bool decelConfirmed = decelerating &&
                                    nowMs - decelSinceMs >= FeedbackConfig::DECEL_CONFIRM_MS;

        if (decelConfirmed && s.pitchDeg > FeedbackConfig::LOW_ENERGY_PITCH_DEG)
        {
            reason = "скорость падает при поднятом носе";
            return true;
        }

        if (speed.hasSpeed() &&
            speed.getSpeed() < FeedbackConfig::STALL_SPEED_MS * FeedbackConfig::LOW_SPEED_MARGIN)
        {
            reason = "скорость близка к сваливанию";
            return true;
        }

        if (control.pitchEffectivenessKnown &&
            control.pitchEffectiveness < FeedbackConfig::LOW_EFFECTIVENESS_RATIO *
                                         FeedbackConfig::EFFECTIVENESS_PRIOR[FeedbackConfig::AXIS_PITCH])
        {
            reason = "руль высоты потерял эффективность";
            return true;
        }

        return false;
    }

    // Можно снимать меры: скорость поднялась с запасом (гистерезис
    // относительно порога входа), а без датчика скорости — хотя бы
    // перестала падать.
    static bool energyRecovered(const SpeedEstimator& speed)
    {
        if (speed.hasSpeed())
        {
            return speed.getSpeed() >= FeedbackConfig::STALL_SPEED_MS * FeedbackConfig::LOW_SPEED_EXIT_MARGIN;
        }
        return !speed.hasAcceleration() || speed.getAcceleration() >= 0.0f;
    }

    bool detectStall(const FlightSnapshot& s, const SpeedEstimator& speed,
                     const ControlState& control, bool lowEnergy)
    {
        if (speed.hasSpeed() && speed.getSpeed() < FeedbackConfig::STALL_SPEED_MS)
        {
            reason = "скорость ниже сваливания";
            return true;
        }

        // Нос резко падает, хотя руль высоты тянет вверх.
        const bool elevatorUp =
            s.commandPitchUs * control.pitchSign > FeedbackConfig::STALL_NOSE_UP_COMMAND_US;
        if (elevatorUp && s.pitchRateDps < -FeedbackConfig::NOSE_DROP_RATE_DPS)
        {
            reason = "нос падает против руля высоты";
            return true;
        }

        // Крыло резко валится против элеронов — только при малой
        // энергии: на скорости это скорее порыв.
        const float rollCommand = s.commandRollUs * control.rollSign;
        if (lowEnergy && fabsf(s.rollRateDps) > FeedbackConfig::WING_DROP_RATE_DPS &&
            s.rollRateDps * rollCommand < 0)
        {
            reason = "сваливание на крыло";
            return true;
        }

        return false;
    }
};
