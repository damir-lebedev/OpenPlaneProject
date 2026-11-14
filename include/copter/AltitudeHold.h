#pragma once
#include <math.h>

// ============================================================
// Высота мультикоптера: оценка и удержание.
//
// Оценка — комплементарный фильтр третьего порядка: акселерометр
// (вертикальное ускорение в земных осях) интегрируется в скорость и
// высоту, барометр медленно подтягивает высоту, скорость и смещение
// акселерометра. Барометр шумит на ±0.3 м и «проваливается» от потока
// винтов, акселерометр дрейфует — вместе они дают и высоту, и гладкую
// вертикальную скорость. Нет барометра — только акселерометр с утечкой:
// высота не годится, скорость — годится на десятки секунд (снижение
// при потере связи).
//
// Удержание — каскад: высота -> вертикальная скорость -> ускорение ->
// доля тяги. Тяга висения известна из настройки (Presets.h) и
// уточняется в полёте (интеграл контура скорости): тяжелее батарея,
// подвесили камеру, просела батарея — контур сам найдёт газ висения.
// На наклоне тяга делится на косинус крена: высота не теряется в
// повороте.
//
// Настройки — как у ALTHOLD Betaflight 2025 (alt_hold_*, autopilot_altitude_*,
// autopilot_throttle_*): усиления P/I/D/F в единицах BF, 15 — наши расчётные.
// Отличия: середина зоны удержания — центр стика газа (у BF — газ висения),
// газ внизу — снижение (у BF — висение); газ висения берётся из описания
// коптера и уточняется в полёте (autopilot_hover_throttle — только если задан).
// ============================================================

namespace copter
{
    class VerticalEstimator
    {
    public:
        // tauS — насколько медленно барометр поправляет акселерометр.
        void configure(float tauS)
        {
            k1 = 3.0f / tauS;
            k2 = 3.0f / (tauS * tauS);
            k3 = 1.0f / (tauS * tauS * tauS);
        }

        // Высота известна, скорость ноль (стоит). Смещение акселерометра не забывается.
        void reset(float altitude)
        {
            h = altitude;
            v = 0.0f;
            initialized = true;
        }

        // Без барометра: учить смещение акселерометра (только когда коптер стоит).
        void setBiasLearning(bool on) { learnBias = on; }

        // accelUp — вертикальное ускорение, м/с² (+ вверх), без g.
        void update(float accelUp, float baroAlt, bool baroOk, float dt)
        {
            if (!initialized) reset(baroOk ? baroAlt : 0.0f);
            const float a = accelUp - bias;
            if (baroOk)
            {
                const float err = baroAlt - h;
                bias -= k3 * err * dt;
                v += (a + k2 * err) * dt;
                h += (v + k1 * err) * dt;
                hasBaro = true;
            }
            else
            {
                if (learnBias) bias += (accelUp - bias) * dt / BIAS_TAU_S;
                v += a * dt;
                v -= v * dt / LEAK_S;
                h += v * dt;
                hasBaro = false;
            }
        }

        float altitude() const { return h; }
        float climbRate() const { return v; }
        float accelBias() const { return bias; }
        bool baroUsed() const { return hasBaro; }

    private:
        static constexpr float LEAK_S = 20.0f;
        static constexpr float BIAS_TAU_S = 1.0f;
        float k1 = 1.5f, k2 = 0.75f, k3 = 0.125f;
        float h = 0.0f, v = 0.0f, bias = 0.0f;
        bool initialized = false;
        bool hasBaro = false;
        bool learnBias = false;
    };

    // Контур вертикальной скорости и высоты. Выход — доля тяги (вход смесителя).
    class AltitudeController
    {
    public:
        struct Settings
        {
            float posGain = 1.2f;        // (м/с) на метр ошибки высоты
            float velGain = 2.5f;        // (м/с²) на м/с ошибки скорости
            float hoverIGain = 0.1f;     // доля газа висения на (м/с ошибки)·с
            float ffGain = 1.0f;         // упреждение: доля ускорения заданной скорости (autopilot_altitude_F)
            float maxClimb = 5.0f;       // м/с при газе до упора (alt_hold_adjust_rate)
            float maxDescent = 3.0f;     // м/с: в собственном потоке винтов — не быстрее 60 % набора
            float maxAccel = 5.0f;       // м/с², ограничение запроса вверх/вниз
            float deadband = 0.10f;      // alt_hold_deadband: столько хода газа в обе стороны от центра — держать
            // autopilot_throttle_min/max — доля тяги; не выше четверти и не ниже 1.4
            // газа висения: иначе коптер не сядет или не удержит высоту
            float throttleMin = 0.10f;
            float throttleMax = 0.70f;
        };

        void configure(const Settings& s, float hoverThrust)
        {
            set = s;
            hover = hoverThrust;
            reset(0.0f, hoverThrust);
        }

        // Начать удерживать: цель — текущая высота, газ — текущий.
        void reset(float altitude, float currentThrust)
        {
            target = altitude;
            // интеграл подхватывает текущий газ: без рывка при включении
            iHover = hover > 0.01f ? currentThrust / hover - 1.0f : 0.0f;
            iHover = clampHover(iHover);
            lastCmd = 0.0f;
            ffAccel = 0.0f;
            braking = true;
            brakeTimer = 0.0f;
        }

        void setTarget(float altitude)
        {
            target = altitude;
            braking = false;
        }
        float getTarget() const { return target; }

        // Новая оценка газа висения: выученная интегралом поправка
        // переносится в неё, итоговый газ не прыгает.
        void setHover(float h)
        {
            if (h <= 0.01f) return;
            const float effective = hover * (1.0f + iHover);
            hover = h;
            iHover = clampHover(effective / hover - 1.0f);
        }

        float getHover() const { return hover; }

        // climbCmd — запрос скорости от пилота, м/с (0 — держать высоту).
        // holdAltitude — false: только скорость (без барометра, посадка).
        // cosTilt — для поправки тяги на наклон.
        float update(float altitude, float climbRate, float climbCmd, bool holdAltitude, float cosTilt, float dt)
        {
            float vTarget;
            if (holdAltitude && fabsf(climbCmd) < 0.01f)
            {
                // стик отпустили на скорости (или удержание включили в наборе):
                // сначала остановиться, высоту запомнить там, где встал, — иначе
                // коптер проскочит точку и будет к ней возвращаться с перелётом
                if (braking)
                {
                    brakeTimer += dt;
                    target = altitude;
                    if (fabsf(climbRate) < BRAKE_DONE_MS || brakeTimer > BRAKE_MAX_S) braking = false;
                }
                vTarget = set.posGain * (target - altitude);
            }
            else
            {
                vTarget = climbCmd;
                // цель едет с пилотом; отпустил стик — тормозит и держится, где встал
                target = altitude;
                braking = true;
                brakeTimer = 0.0f;
            }
            vTarget = fminf(set.maxClimb, fmaxf(-set.maxDescent, vTarget));
            lastClimbTarget = vTarget;

            // упреждение: стик газа двинули — тяга меняется сразу, не дожидаясь ошибки скорости
            const float cmd = fabsf(climbCmd) < 0.01f ? 0.0f : fminf(set.maxClimb, fmaxf(-set.maxDescent, climbCmd));
            if (dt > 0.0f) ffAccel += ((cmd - lastCmd) / dt - ffAccel) * fminf(1.0f, dt / FF_SMOOTH_S);
            lastCmd = cmd;
            const float ff = set.ffGain * fminf(0.5f * set.maxAccel, fmaxf(-0.5f * set.maxAccel, ffAccel));

            const float err = vTarget - climbRate;
            const float accel = fminf(set.maxAccel, fmaxf(-set.maxAccel, set.velGain * err + ff));
            const float c = fmaxf(0.5f, cosTilt);
            const float learned = hover * (1.0f + iHover);
            const float raw = learned * (1.0f + accel / 9.80665f) / c;
            const float lo = fmaxf(0.0f, fminf(set.throttleMin, 0.25f * learned));
            const float hi = fminf(1.0f, fmaxf(set.throttleMax, 1.4f * learned));
            // интеграл не копится в сторону упора
            const bool stuck = (raw >= hi && err > 0.0f) || (raw <= lo && err < 0.0f);
            if (!stuck) iHover = clampHover(iHover + set.hoverIGain * err * dt);
            return fminf(hi, fmaxf(lo, raw));
        }

        // Газ висения с поправкой интеграла (что контур выучил).
        float learnedHover() const { return hover * (1.0f + iHover); }
        float climbTarget() const { return lastClimbTarget; }

    private:
        // −60 %…+100 % от расчётного висения: легче/мощнее или вдвое тяжелее
        static float clampHover(float v) { return fminf(1.0f, fmaxf(-0.6f, v)); }
        static constexpr float FF_SMOOTH_S = 0.1f;

        Settings set;
        float lastCmd = 0.0f;
        float ffAccel = 0.0f;
        bool braking = false;        // стик отпущен, скорость ещё гасится — цель не запомнена
        float brakeTimer = 0.0f;
        static constexpr float BRAKE_DONE_MS = 0.3f;   // м/с
        static constexpr float BRAKE_MAX_S = 3.0f;
        float hover = 0.4f;
        float target = 0.0f;
        float iHover = 0.0f;
        float lastClimbTarget = 0.0f;
    };
}
