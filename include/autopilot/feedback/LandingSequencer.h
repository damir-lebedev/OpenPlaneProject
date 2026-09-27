#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/PhaseTargets.h"

// ============================================================
// LANDING SEQUENCER — автоматическая посадка по датчикам
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
//   Approach: газ APPROACH_THROTTLE_PERCENT, снижение с постоянной
//     вертикальной скоростью APPROACH_SINK_RATE_MS. Тангаж — от
//     реального снижения: падаем быстрее нужного — нос чуть вверх,
//     медленнее — вниз. Крен — от пилота (он доворачивает на полосу),
//     не больше APPROACH_MAX_BANK_DEG.
//       ── высота ≤ FLARE_HEIGHT_M ──►
//   Flare (выравнивание): газ 0, крылья ровно, снижение гасится до
//     FLARE_SINK_RATE_MS — тем же правилом "тангаж от вертикальной
//     скорости", нос от 0 до FLARE_MAX_PITCH_DEG. По мере потери
//     скорости нос поднимается сам — ровно настолько, насколько нужно.
//       ── касание: удар по акселерометру или высота ≈ 0 и самолёт
//          не вращается TOUCHDOWN_STILL_MS ──►
//   Rollout (пробег): газ 0, крылья ровно, курс держат руль
//     направления и колесо, ROLLOUT_MS ──► Complete
//
// Высота: дальномер (heightAgl), если есть; иначе барометр от точки
// включения — только если садимся туда же, откуда взлетали, и с
// ошибкой ~метр, поэтому выравнивание по барометру грубое.
//
// Уход на второй круг: пилот даёт газ ≥ GO_AROUND_THROTTLE_PERCENT
// или cancel() — посадка отменяется, управляет пилот.
// ============================================================

class LandingSequencer
{
public:

    enum class State : uint8_t { Idle, Approach, Flare, Rollout, Complete, Aborted };

    void reset();

    void request(uint32_t nowMs) { enter(State::Approach, nowMs); }

    void cancel();

    // Сначала переход (не больше одного за такт), потом цели того
    // этапа, в котором оказались, — так в такт смены этапа не уходят
    // цели прошлого (например, газ снижения в первом такте выравнивания).
    void update(const FlightSnapshot& s, uint32_t nowMs);

    const PhaseTargets& getTargets() const { return targets; }
    State getState() const { return state; }

    bool isActive() const;

    // Выравнивание и пробег — у самой земли, где сваливание — это
    // и есть посадка: защите от сваливания здесь не место.
    bool isNearGround() const { return state == State::Flare || state == State::Rollout; }

    // Колёса уже на земле.
    bool isOnGround() const { return state == State::Rollout || state == State::Complete; }

    const char* getStateName() const;


private:

    State state = State::Idle;
    uint32_t stateSinceMs = 0;
    float headingDeg = 0;
    PhaseTargets targets;

    bool stillPending = false;
    uint32_t stillSinceMs = 0;

    void enter(State next, uint32_t nowMs);

    void advance(const FlightSnapshot& s, uint32_t nowMs);

    PhaseTargets targetsFor(State st, const FlightSnapshot& s) const;

    static bool heightAboveGround(const FlightSnapshot& s, float& height);

    // Тангаж по ошибке вертикальной скорости: снижаемся быстрее
    // нужного (climbRate −2 при нужных −1) — нос вверх на
    // SINK_TO_PITCH_GAIN° за каждый лишний м/с. Без барометра —
    // базовый угол.
    static float sinkRatePitch(const FlightSnapshot& s, float sinkRateMs,
                               float basePitchDeg, float minPitchDeg);

    bool touchdownDetected(const FlightSnapshot& s, uint32_t nowMs);
};
