#pragma once
#include <Arduino.h>

#include "sensors/SensorInterface.h"

// ============================================================
// AIRSPEED SENSOR — интерфейс датчика воздушной скорости
//
// Реализация: самодельная трубка Пито на двух барометрах
// (PitotDualBaroAirspeed.h) — полное давление в трубке минус
// статическое в фюзеляже. Выбор — SENSOR_AIRSPEED в SensorSelection.h.
//
// Воздушная скорость нужна автопилоту: от неё зависят и сваливание
// (путевая скорость GPS при ветре врёт на скорость ветра), и
// эффективность рулей, и газ в круизе.
//
// Скорость по перепаду давлений: V = sqrt(2·ΔP / ρ).
//   • приборная (IAS) — с ρ0 = 1.225 кг/м³ (уровень моря, 15 °C):
//     по ней сваливание одинаково на любой высоте и в любую погоду;
//   • истинная (TAS) — с ρ по давлению и температуре статики:
//     по ней навигация (TAS + ветер = путевая скорость).
// ============================================================

struct AirspeedData
{
    float differentialPressurePa;  // после вычета нуля и фильтра
    float indicatedMs;             // приборная скорость (ρ0)
    float trueMs;                  // истинная скорость (ρ по статике)
    float airDensity;              // кг/м³
    uint32_t timestamp;
};

class AirspeedSensor : public Sensor
{
public:
    ~AirspeedSensor() override = default;

    virtual const AirspeedData& getAirspeedData() const = 0;

    // Запомнить текущий перепад как ноль — на земле, без ветра в
    // трубку (накрыть рукой/колпачком или развернуть по ветру).
    virtual void calibrateZero() = 0;

    // Идёт ли сейчас обнуление (скорость в это время не выдаётся).
    virtual bool isZeroing() const { return false; }
};
