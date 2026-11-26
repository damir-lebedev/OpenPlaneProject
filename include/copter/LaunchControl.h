#pragma once
// Launch control (BF: режим LAUNCH CONTROL, launch_*): гоночный старт.
//
// ARM с включённым режимом (или газ вниз на земле при
// launch_trigger_allow_reset) — коптер стоит на земле, газ не поднимает его,
// а стик тангажа медленно наклоняет раму вперёд (не дальше
// launch_angle_limit) и держит её так интегралом (launch_control_gain). Рывок
// газом выше launch_trigger_throttle_percent — старт: обычный полёт ACRO с
// уже наклонённой рамой, без кувырка назад и без потери первых метров.
//
// Режимы (launch_control_mode):
//   NORMAL    — стики всех трёх осей (медленно), I — только у тангажа;
//   PITCHONLY — только тангаж и только носом вниз: работают задние моторы,
//               передние на холостых;
//   FULL      — стики и I всех трёх осей.
//
// Только в ACRO: в ANGLE/HORIZON и автоматах (failsafe, возврат) — нет.
// Код свой, поведение — по описанию BF.

#include <math.h>
#include <stdint.h>

namespace copter
{
    enum class LaunchControlMode : uint8_t { NORMAL, PITCHONLY, FULL };
    enum class LaunchState : uint8_t { OFF, ACTIVE, TRIGGERED };

    // Имя для живого кадра Studio.
    inline const char* launchStateName(LaunchState s)
    {
        return s == LaunchState::ACTIVE ? "ACTIVE" : s == LaunchState::TRIGGERED ? "TRIGGERED" : "OFF";
    }

    struct LaunchControlSettings
    {
        LaunchControlMode mode = LaunchControlMode::NORMAL;   // launch_control_mode
        bool allowReset = true;          // launch_trigger_allow_reset: после старта — снова, газ вниз на земле
        float triggerThrottle = 0.20f;   // launch_trigger_throttle_percent / 100: газ выше — старт
        float angleLimitDeg = 0.0f;      // launch_angle_limit: наклон носом вниз не больше (0 — без предела)
        float gainBf = 40.0f;            // launch_control_gain: I на время ожидания, число BF
    };

    class LaunchControl
    {
    public:
        static constexpr float MAX_RATE_DPS = 100.0f;    // на половине хода стика и дальше
        static constexpr float MIN_RATE_DPS = 5.0f;      // у самого предела наклона
        static constexpr float ANGLE_WINDOW_DEG = 10.0f; // за столько до предела скорость спадает
        static constexpr float RESET_THROTTLE = 0.05f;   // газ «внизу» для повторного взвода
        static constexpr float BACK_GAIN = 4.0f;         // за пределом: (°/с) назад на градус

        void configure(const LaunchControlSettings& s) { set = s; }
        const LaunchControlSettings& settings() const { return set; }

        // ARM: режим включён — ждать рывка.
        void arm(bool modeOn, bool usable) { st = modeOn && usable ? LaunchState::ACTIVE : LaunchState::OFF; }
        void disarm() { st = LaunchState::OFF; }

        // Раз за цикл, после ARM. modeOn — тумблер; usable — можно (ACRO, не
        // failsafe/возврат/turtle); throttle — стик газа 0..1; airborne — в воздухе.
        void update(bool modeOn, bool usable, float throttle, bool airborne)
        {
            if (!modeOn || !usable)
            {
                // выключили тумблер или ушли из ACRO — до следующего взвода
                st = modeOn ? (st == LaunchState::OFF ? LaunchState::OFF : LaunchState::TRIGGERED) : LaunchState::OFF;
                return;
            }
            if (st == LaunchState::ACTIVE && throttle > set.triggerThrottle) st = LaunchState::TRIGGERED;
            // повторный взвод: после старта (или тумблер включили уже заармленным)
            // газ вниз на земле
            if (st != LaunchState::ACTIVE && set.allowReset && !airborne && throttle <= RESET_THROTTLE)
                st = LaunchState::ACTIVE;
        }

        LaunchState state() const { return st; }
        bool active() const { return st == LaunchState::ACTIVE; }

        // Уставки угловой скорости, °/с, по стикам −1..1 (+ тангаж — нос
        // вверх). Скорость постоянная, от расходов не зависит: 100 °/с на
        // половине хода. pitchDeg — тангаж рамы (+ нос вверх).
        void setpoints(const float sticks[3], float pitchDeg, float sp[3]) const
        {
            for (uint8_t a = 0; a < 3; ++a)
            {
                const bool used = a == 1 || set.mode != LaunchControlMode::PITCHONLY;
                const float s = fminf(0.5f, fmaxf(-0.5f, sticks[a]));
                sp[a] = used ? 2.0f * MAX_RATE_DPS * s : 0.0f;
            }
            // PITCHONLY — только носом вниз (задние моторы)
            if (set.mode == LaunchControlMode::PITCHONLY && sp[1] > 0.0f) sp[1] = 0.0f;
            // предел наклона вперёд: у предела — медленнее, за ним — назад к
            // пределу (своё: в BF только стоп, и накопленный I перекатывает раму)
            if (set.angleLimitDeg > 0.0f)
            {
                const float forward = -pitchDeg;
                const float left = set.angleLimitDeg - forward;
                if (left <= 0.0f)
                    sp[1] = fmaxf(sp[1], fminf(MAX_RATE_DPS, -left * BACK_GAIN));
                else if (sp[1] < 0.0f && left < ANGLE_WINDOW_DEG)
                {
                    const float k = left / ANGLE_WINDOW_DEG;
                    const float fast = -sp[1];
                    sp[1] = -(MIN_RATE_DPS + k * (fmaxf(fast, MIN_RATE_DPS) - MIN_RATE_DPS));
                }
            }
        }

        // Копить ли интеграл оси, пока ждём рывка.
        bool integrates(uint8_t axis) const { return axis == 1 || set.mode == LaunchControlMode::FULL; }

    private:
        LaunchControlSettings set;
        LaunchState st = LaunchState::OFF;
    };
}
