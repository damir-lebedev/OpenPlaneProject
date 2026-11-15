#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"

// ============================================================
// Chirp — качание частоты для измерения коптера в полёте (пункт 168).
//
// Синус, частота которого растёт по экспоненте от chirp_frequency_start до
// chirp_frequency_end за chirp_time_seconds (логарифмический свип: на каждую
// октаву — одно и то же время). Сигнал добавляется к команде одной оси на
// входе смесителя — поверх работающей стабилизации: коптер висит как висел,
// а по записи «возбуждение → команда → гироскоп» в чёрном ящике считается
// его частотная характеристика (copter/SysId.h, вкладка «Измерение» в
// OpenPlane Studio).
//
// Почему в команду оси, а не в уставку: размах на моторах задан прямо
// (доля хода смесителя) и не зависит от текущих PID; на низких частотах
// контур сам гасит раскачку (угол не уходит), на высоких — возбуждение
// идёт на моторы как есть.
//
// Амплитуда по частоте: A·|(1 + jf/lead)/(1 + jf/lag)|. Объект — интегратор
// с запаздыванием мотора, его ответ падает как 1/f²; при lead < lag
// возбуждение растёт между ними в lag/lead раз, и на высоких частотах ответ
// остаётся выше шума гироскопа (там и видны задержки). Не больше MAX_AMPLITUDE
// и не больше половины текущего газа: мотор с командой оси ±1 качается от
// половины до полутора своей доли тяги и не упирается в ноль (иначе airmode
// поднимет общий газ, и коптер уйдёт вверх).
// В начале и в конце — плавный вход и выход (без щелчка по моторам).
//
// Имена настроек — как в Betaflight (chirp_*); смысл амплитуды свой: десятые
// доли процента хода смесителя, а не °/с.
// ============================================================

namespace copter
{
    struct ChirpSettings
    {
        float amplitude[3] = { 0.06f, 0.06f, 0.10f };   // chirp_amplitude_*: доля хода смесителя (крен, тангаж, рысканье)
        float startHz = 1.0f;                           // chirp_frequency_start_deci_hz
        float endHz = 200.0f;                           // chirp_frequency_end_deci_hz (не выше 0.4 частоты цикла)
        float seconds = 20.0f;                          // chirp_time_seconds
        float leadHz = 20.0f;                           // chirp_lead_freq_hz: отсюда амплитуда растёт
        float lagHz = 60.0f;                            // chirp_lag_freq_hz: досюда
    };

    // Чем кончился свип.
    enum class ChirpEnd : uint8_t
    {
        NONE,
        DONE,         // прошёл целиком
        SWITCH,       // пилот выключил тумблер
        MODE,         // режим не ANGLE/ACRO, failsafe, возврат домой
        TILT,         // наклон больше предела
        SATURATED,    // моторы упёрлись
        NOT_FLYING,   // сел, DISARM
    };

    inline const char* chirpEndName(ChirpEnd e)
    {
        switch (e)
        {
            case ChirpEnd::NONE: return "NONE";
            case ChirpEnd::DONE: return "DONE";
            case ChirpEnd::SWITCH: return "SWITCH";
            case ChirpEnd::MODE: return "MODE";
            case ChirpEnd::TILT: return "TILT";
            case ChirpEnd::SATURATED: return "SATURATED";
            case ChirpEnd::NOT_FLYING: return "NOT_FLYING";
        }
        return "?";
    }

    class Chirp
    {
    public:
        static constexpr float MAX_AMPLITUDE = 0.3f;    // доля хода смесителя
        static constexpr float FADE_IN_S = 0.5f, FADE_OUT_S = 0.25f;

        void configure(const ChirpSettings& s, float loopHz)
        {
            set = s;
            f0 = fmaxf(0.1f, s.startHz);
            f1 = fminf(0.4f * loopHz, fmaxf(f0 * 1.5f, s.endHz));
            total = fmaxf(1.0f, s.seconds);
            stop(ChirpEnd::NONE);
        }

        // Начать свип по оси (0 крен, 1 тангаж, 2 рысканье). false — амплитуда оси 0.
        bool start(uint8_t whichAxis)
        {
            if (whichAxis > 2 || set.amplitude[whichAxis] <= 0.0f) return false;
            ax = whichAxis;
            t = 0.0f;
            phase = 0.0f;
            freq = f0;
            running = true;
            ended = ChirpEnd::NONE;
            ++runs;
            return true;
        }

        void stop(ChirpEnd why)
        {
            running = false;
            ended = why;
            value = 0.0f;
        }

        // Один цикл: возбуждение, доля хода смесителя. ceiling — потолок размаха
        // в этом цикле (доля газа). Свип кончился — 0 и DONE.
        float update(float dt, float ceiling = MAX_AMPLITUDE)
        {
            if (!running) return 0.0f;
            // f(t) = f0·(f1/f0)^(t/T): за цикл частота растёт в одно и то же число раз
            freq = f0 * powf(f1 / f0, t / total);
            phase += 2.0f * PI_F * freq * dt;
            if (phase > 2.0f * PI_F) phase -= 2.0f * PI_F;
            float gain = 1.0f;
            if (set.leadHz > 0.0f && set.lagHz > 0.0f)
            {
                const float a = freq / set.leadHz, b = freq / set.lagHz;
                gain = sqrtf((1.0f + a * a) / (1.0f + b * b));
            }
            const float fade = fminf(1.0f, fminf(t / FADE_IN_S, (total - t) / FADE_OUT_S));
            const float top = fminf(MAX_AMPLITUDE, fmaxf(0.0f, ceiling));
            const float amp = fminf(top, set.amplitude[ax] * gain) * fmaxf(0.0f, fade);
            value = amp * sinf(phase);
            t += dt;
            if (t >= total) stop(ChirpEnd::DONE);
            return value;
        }

        bool active() const { return running; }
        uint8_t axis() const { return ax; }
        float hz() const { return running ? freq : 0.0f; }
        float progress() const { return running ? t / total : 0.0f; }
        float excitation() const { return value; }
        ChirpEnd lastEnd() const { return ended; }
        uint8_t runCount() const { return runs; }   // сколько свипов начато (метка в чёрном ящике)
        float startHz() const { return f0; }
        float endHz() const { return f1; }

    private:
        ChirpSettings set;
        float f0 = 1.0f, f1 = 200.0f, total = 20.0f;
        float t = 0.0f, phase = 0.0f, freq = 0.0f, value = 0.0f;
        uint8_t ax = 0, runs = 0;
        bool running = false;
        ChirpEnd ended = ChirpEnd::NONE;
    };
}
