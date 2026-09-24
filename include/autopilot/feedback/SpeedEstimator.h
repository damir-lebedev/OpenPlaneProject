#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"

// ============================================================
// SPEED ESTIMATOR — скорость и её изменение
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Скорость: воздушная (трубка Пито) лучше всего; путевая по GPS —
// запасной вариант (при ветре ошибается на скорость ветра); без них
// скорость неизвестна (hasSpeed() == false).
//
// Продольное ускорение (как быстро скорость растёт или падает)
// есть всегда, пока жив IMU. Акселерометр меряет не ускорение, а
// "перегрузку": по оси X к носу это dV/dt + g·sin(тангаж). Отсюда
//     dV/dt = g · (ax − sin(тангаж)),
// ax в g. Этого хватает, чтобы заметить "скорость быстро падает"
// раньше, чем её покажет GPS, — даже без датчика скорости вообще.
// Точность ограничена калибровкой акселерометра (0.02g ≈ 0.2 м/с²)
// и вибрацией — поэтому ФНЧ.
// ============================================================

class SpeedEstimator
{
public:

    enum class Source : uint8_t { None, Gps, Airspeed };

    void update(const FlightSnapshot& s)
    {
        const float dt = (s.timeUs - lastUs) / 1000000.0f;
        lastUs = s.timeUs;
        if (dt <= 0.0f || dt > 0.5f) return;

        // Продольное ускорение по IMU.
        if (s.imuValid)
        {
            const float kinematic =
                FeedbackConfig::GRAVITY * (s.accelXg - sinf(s.pitchDeg * DEG_TO_RAD));
            const float alpha = dt / (FeedbackConfig::ACCEL_FILTER_TAU_S + dt);
            accelMs2 += alpha * (kinematic - accelMs2);
            accelValid = true;
        }
        else
        {
            accelValid = false;
        }

        // Скорость: лучший доступный источник.
        if (s.airspeedValid)
        {
            speedMs = s.airspeedMs;
            source = Source::Airspeed;
        }
        else if (s.gpsValid)
        {
            speedMs = s.groundSpeedMs;
            source = Source::Gps;
        }
        else
        {
            source = Source::None;
        }
    }

    bool hasSpeed() const { return source != Source::None; }
    float getSpeed() const { return speedMs; }
    Source getSource() const { return source; }

    bool hasAcceleration() const { return accelValid; }
    float getAcceleration() const { return accelMs2; }  // м/с², + разгон

    // Во сколько раз эффективность рулей сейчас отличается от опорной:
    // аэродинамическая сила ∝ V². 1 — скорость неизвестна.
    float effectivenessScale() const
    {
        if (!hasSpeed()) return 1.0f;
        const float ratio = speedMs / FeedbackConfig::REFERENCE_SPEED_MS;
        return constrain(ratio * ratio, 0.05f, 4.0f);
    }


private:

    uint32_t lastUs = 0;
    float speedMs = 0;
    Source source = Source::None;
    float accelMs2 = 0;
    bool accelValid = false;
};
