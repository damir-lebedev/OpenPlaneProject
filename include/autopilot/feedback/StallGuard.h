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

    // Что известно о руле высоты от оценки эффективности.
    struct ControlState
    {
        bool pitchEffectivenessKnown = false;
        float pitchEffectiveness = 0;    // модуль оценки b по тангажу
    };

    void reset();

    void update(const FlightSnapshot& s, const SpeedEstimator& speed,
                const ControlState& control, bool airborne, uint32_t nowMs);

    Level getLevel() const { return level; }
    const char* getLevelName() const;

    // Почему сработало (последний признак) — для лога.
    const char* getReason() const { return reason; }

    // --- Ограничения для FeedbackSupervisor ---

    float maxPitchDeg() const;

    float maxBankDeg() const;

    // Предел отклонения элеронов, мкс.
    float maxAileronUs() const;

    // Нижняя граница газа, %; −1 — не ограничивать.
    float throttleFloorPercent(bool linkLost) const;


private:

    Level level = Level::Normal;
    const char* reason = "";

    bool decelerating = false;
    uint32_t decelSinceMs = 0;
    uint32_t lastStallMs = 0;
    uint32_t lastLowEnergyMs = 0;

    bool detectLowEnergy(const FlightSnapshot& s, const SpeedEstimator& speed,
                         const ControlState& control, uint32_t nowMs);

    // Можно снимать меры: скорость поднялась с запасом (гистерезис
    // относительно порога входа), а без датчика скорости — хотя бы
    // перестала падать.
    static bool energyRecovered(const SpeedEstimator& speed);

    bool detectStall(const FlightSnapshot& s, const SpeedEstimator& speed, bool lowEnergy);
};
