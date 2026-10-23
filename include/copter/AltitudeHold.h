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
            float maxClimb = 3.0f;       // м/с
            float maxDescent = 2.0f;     // м/с
            float maxAccel = 5.0f;       // м/с², ограничение запроса вверх/вниз
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
        }

        void setTarget(float altitude) { target = altitude; }
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
                vTarget = set.posGain * (target - altitude);
            }
            else
            {
                vTarget = climbCmd;
                // цель едет с пилотом; отпустил стик — держится, где был
                target = altitude;
            }
            vTarget = fminf(set.maxClimb, fmaxf(-set.maxDescent, vTarget));
            lastClimbTarget = vTarget;

            const float err = vTarget - climbRate;
            const float accel = fminf(set.maxAccel, fmaxf(-set.maxAccel, set.velGain * err));
            const float c = fmaxf(0.5f, cosTilt);
            const float raw = hover * (1.0f + iHover) * (1.0f + accel / 9.80665f) / c;
            // интеграл не копится в сторону упора (газ 0 или 1)
            const bool stuck = (raw >= 1.0f && err > 0.0f) || (raw <= 0.0f && err < 0.0f);
            if (!stuck) iHover = clampHover(iHover + set.hoverIGain * err * dt);
            return fminf(1.0f, fmaxf(0.0f, raw));
        }

        // Газ висения с поправкой интеграла (что контур выучил).
        float learnedHover() const { return hover * (1.0f + iHover); }
        float climbTarget() const { return lastClimbTarget; }

    private:
        // −60 %…+100 % от расчётного висения: легче/мощнее или вдвое тяжелее
        static float clampHover(float v) { return fminf(1.0f, fmaxf(-0.6f, v)); }

        Settings set;
        float hover = 0.4f;
        float target = 0.0f;
        float iHover = 0.0f;
        float lastClimbTarget = 0.0f;
    };
}
