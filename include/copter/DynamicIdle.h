#pragma once
// Dynamic idle (BF: dyn_idle_*): холостые по оборотам.
//
// Обычные холостые (dshot_idle_value) — постоянная доля команды: на газу
// внизу в кувырке или на спуске винт попадает в собственный поток,
// обороты проваливаются, регулятор теряет синхронизацию, а мотору нечем
// тормозить раму. Dynamic idle держит самый медленный мотор не ниже
// dyn_idle_min_rpm (×100 об/мин): регулятор PI(D) по ошибке оборотов
// поднимает нижний край хода моторов, не больше dyn_idle_max_increase.
// До airmode (на земле, газ ни разу не поднимали) предел —
// dyn_idle_start_increase: коптер на столе не прыгает.
//
// Работает только по настоящим оборотам (двунаправленный DShot): модель
// оборотов считает их из той же команды, и подъём холостых сам себе
// «подтвердил» бы обороты. Нет телеметрии — добавка 0.
// Код свой, поведение — по описанию BF.

#include <math.h>
#include <stdint.h>

namespace copter
{
    struct DynIdleSettings
    {
        float minRpm = 0.0f;           // dyn_idle_min_rpm × 100: об/мин самого медленного мотора; 0 — выключен
        float pGain = 50.0f;           // dyn_idle_p_gain, число BF
        float iGain = 50.0f;           // dyn_idle_i_gain
        float dGain = 50.0f;           // dyn_idle_d_gain
        float maxIncrease = 0.150f;    // dyn_idle_max_increase / 1000: предел подъёма холостых
        float startIncrease = 0.050f;  // dyn_idle_start_increase / 1000: предел до airmode
        bool enabled() const { return minRpm > 0.0f; }
    };

    class DynamicIdle
    {
    public:
        // Числа BF -> доли команды на Гц ошибки (масштаб — как у BF по смыслу:
        // p 50 и провал на 10 Гц дают ~7 % хода; i 50 — 0,5 доли/с на Гц).
        static constexpr float P_SCALE = 0.00015f;   // 1/Гц
        static constexpr float I_SCALE = 0.01f;      // 1/(Гц·с)
        static constexpr float D_SCALE = 0.000003f;  // с/Гц

        // motorHz — обороты моторов (об/с), telemetry — они настоящие (DShot),
        // airmode — airmode уже включился (газ поднимали). Возвращает добавку
        // к холостым, доля команды регулятору.
        float update(const DynIdleSettings& s, const float* motorHz, uint8_t motors, bool telemetry, bool airmode,
                     float dt)
        {
            if (!s.enabled() || !telemetry || motors == 0 || dt <= 0.0f)
            {
                reset();
                return 0.0f;
            }
            float slowest = motorHz[0];
            for (uint8_t i = 1; i < motors; ++i) slowest = fminf(slowest, motorHz[i]);
            minHz = slowest;
            const float limit = airmode ? s.maxIncrease : s.startIncrease;
            const float error = s.minRpm / 60.0f - slowest;   // Гц; > 0 — мотор ниже минимума
            const float p = error * s.pGain * P_SCALE;
            integral = clamp(integral + error * s.iGain * I_SCALE * dt, 0.0f, limit);
            // D — по скорости провала оборотов (сглажено: телеметрия DShot ступенчатая)
            const float rate = havePrev ? (slowest - prevHz) / dt : 0.0f;
            rateLpf += (rate - rateLpf) * fminf(1.0f, dt * 2.0f * 3.14159265f * D_LPF_HZ);
            prevHz = slowest;
            havePrev = true;
            const float d = -rateLpf * s.dGain * D_SCALE;
            out = clamp(p + integral + d, 0.0f, limit);
            return out;
        }

        void reset()
        {
            integral = out = rateLpf = minHz = 0.0f;
            havePrev = false;
        }

        float output() const { return out; }        // добавка к холостым сейчас, доля
        float slowestHz() const { return minHz; }   // самый медленный мотор, об/с

    private:
        static constexpr float D_LPF_HZ = 30.0f;
        static float clamp(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

        float integral = 0.0f, out = 0.0f, rateLpf = 0.0f, prevHz = 0.0f, minHz = 0.0f;
        bool havePrev = false;
    };
}
