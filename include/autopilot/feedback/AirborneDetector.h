#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/SpeedEstimator.h"

// ============================================================
// AIRBORNE DETECTOR — самолёт в воздухе или на земле
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// От этого зависит почти всё: учить эффективность рулей, копить
// интеграл, искать сваливание и перепутанный знак имеет смысл только
// в полёте — на колёсах рули самолёт не вращают.
//
// В воздухе: высота над точкой включения (или над землёй по
// дальномеру) выше AIRBORNE_HEIGHT_M, или скорость выше скорости
// отрыва — дольше AIRBORNE_CONFIRM_MS.
// На земле: низко, не вращается и перегрузка ≈ 1g дольше
// GROUND_STILL_MS (самолёт стоит или катится ровно).
//
// Взлёт и посадка знают лучше — FeedbackSupervisor может сказать
// явно через force(): после отрыва — в воздухе, после касания — на
// земле.
// ============================================================

class AirborneDetector
{
public:

    void reset();

    void force(bool isAirborne);

    void update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs);

    bool isAirborne() const { return airborne; }


private:

    bool airborne = false;
    bool pending = false;
    uint32_t pendingSinceMs = 0;

    static bool looksAirborne(const FlightSnapshot& s, const SpeedEstimator& speed);

    static bool looksLanded(const FlightSnapshot& s);
};
