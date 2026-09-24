#pragma once
#include <Arduino.h>

#include "sensors/SensorInterface.h"

// ============================================================
// AIRSPEED SENSOR — интерфейс датчика воздушной скорости
//
// ⚠️ ЗАГОТОВКА: реализаций пока нет, в SensorSelection.h категории
// нет. Нужна контуру обратной связи (autopilot/feedback/): от
// воздушной скорости зависят и эффективность рулей, и сваливание —
// путевая скорость GPS при ветре врёт на скорость ветра.
//
// Кандидаты: MS4525DO (I2C, цифровой) или MPXV7002DP (аналоговый)
// + трубка Пито. Скорость по перепаду давлений:
// V = sqrt(2·ΔP / ρ), ρ ≈ 1.225 кг/м³ на уровне моря.
// ============================================================

struct AirspeedData
{
    float differentialPressurePa;  // после вычета нуля
    float indicatedMs;             // приборная скорость (ρ уровня моря)
    uint32_t timestamp;
};

class AirspeedSensor : public Sensor
{
public:
    virtual ~AirspeedSensor() = default;

    virtual const AirspeedData& getAirspeedData() const = 0;

    // Запомнить текущий перепад как ноль — на земле, без ветра в
    // трубку (накрыть рукой/колпачком).
    virtual void calibrateZero() = 0;
};
