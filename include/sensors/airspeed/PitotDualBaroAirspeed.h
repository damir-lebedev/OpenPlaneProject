#pragma once
#include <Arduino.h>

#include "config/Config.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"

// ============================================================
// САМОДЕЛЬНАЯ ТРУБКА ПИТО НА ДВУХ БАРОМЕТРАХ
//
//   ┌──────── крыло/нос ─────────┐
//   │  ═══►  трубка ─► BMP581     │   полное давление  Pt = Ps + ½ρV²
//   └─────────────────────────────┘
//   фюзеляж:  SPL06 (основной барометр)  статическое давление Ps
//
//   ΔP = Pt − Ps − ноль,   V = sqrt(2·ΔP / ρ)
//
// Готового дифференциального датчика (MS4525, MPXV7002) нет —
// перепад считается как разность двух абсолютных барометров. Это
// работает, потому что у BMP581 шум ~0.1 Па, а на 10 м/с перепад
// ~61 Па, на 5 м/с — ~15 Па. Но у двух разных чипов свои заводские
// смещения (до сотен Па), поэтому:
//
//   • ноль — среднее Pt − Ps за PITOT_ZERO_SAMPLES отсчётов при
//     включении (самолёт стоит, трубка накрыта или по ветру) и по
//     команде calibrateZero(); пока ноль не набран, скорость не
//     выдаётся (isAvailable() == false);
//   • перепад сглаживается ФНЧ с τ = PITOT_FILTER_TAU_S: барометры
//     меряют не одновременно и с разной частотой;
//   • плотность ρ = Ps / (R·T) — по статике и её температуре, отсюда
//     истинная скорость; приборная — с ρ0 = 1.225;
//   • PITOT_SCALE — поправочный множитель скорости: давление внутри
//     фюзеляжа не равно статическому (зависит от щелей и скорости),
//     трубка может стоять под углом. Подбирается полётом по GPS в
//     безветрие туда-обратно (см. docs/AUTOPILOT_GUIDE.md);
//   • здоровье: оба барометра отвечают, отсчёт трубки свежий, и
//     перепад не уходит сильно в минус дольше PITOT_NEGATIVE_FAULT_MS
//     (так выглядят перепутанные/пережатые шланги или вода в трубке).
//
// Барометр фюзеляжа опрашивает Autopilot (он же основной барометр);
// этот класс опрашивает только барометр трубки.
// ============================================================

class PitotDualBaroAirspeed : public AirspeedSensor
{
public:

    PitotDualBaroAirspeed(BarometerSensor& pitotBaro, BarometerSensor& staticBaro);

    bool begin() override;

    bool isAvailable() const override;

    void update() override;

    const AirspeedData& getAirspeedData() const override { return data; }

    void calibrateZero() override;

    bool isZeroing() const override { return zeroing; }
    bool hasFault() const { return fault; }
    float getZeroOffset() const { return zeroOffsetPa; }

    const char* getSensorType() const override { return "PITOT"; }

    void printStatus() const override;

    // Скорость по перепаду (Па) и плотности (кг/м³) с поправкой
    // PITOT_SCALE; отрицательный перепад — 0.
    static float speedFrom(float differentialPa, float density);

    // Плотность сухого воздуха по давлению (Па) и температуре (°C).
    static float densityOf(float pressurePa, float temperatureC);

    static constexpr float SEA_LEVEL_DENSITY = 1.225f;


private:

    static constexpr float GAS_CONSTANT_AIR = 287.05f;  // Дж/(кг·К)

    BarometerSensor& total;
    BarometerSensor& stat;

    AirspeedData data;
    bool begun = false;

    bool zeroing = false;
    float zeroSum = 0;
    uint16_t zeroCount = 0;
    float zeroOffsetPa = 0;

    float filteredPa = 0;
    uint32_t lastTotalUs = 0;

    bool fault = false;
    uint32_t negativeSinceUs = 0;

    void accumulateZero(float raw);

    void updateFault(float dp, uint32_t now);
};
