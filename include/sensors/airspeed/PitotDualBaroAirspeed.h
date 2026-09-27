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

    PitotDualBaroAirspeed(BarometerSensor& pitotBaro, BarometerSensor& staticBaro)
        : total(pitotBaro),
          stat(staticBaro),
          data()
    {
    }

    bool begin() override
    {
        begun = total.begin();
        if (!begun)
        {
            Serial.println("Pitot: барометр трубки не отвечает — воздушной скорости нет");
            return false;
        }

        calibrateZero();
        Serial.println("Pitot: обнуление — трубку накрыть или развернуть по ветру на ~1 с");
        return true;
    }

    bool isAvailable() const override
    {
        return begun && !zeroing && !fault &&
               total.isAvailable() && stat.isAvailable() &&
               data.timestamp != 0 && micros() - data.timestamp < Config::PITOT_STALE_US;
    }

    void update() override
    {
        if (!begun) return;

        total.update();
        if (!total.isAvailable() || !stat.isAvailable()) return;

        const BarometerData& t = total.getBarometerData();
        const BarometerData& s = stat.getBarometerData();
        if (t.timestamp == 0 || s.timestamp == 0 || t.timestamp == lastTotalUs) return;

        const float dtS = lastTotalUs == 0 ? 0.0f : (t.timestamp - lastTotalUs) / 1000000.0f;
        lastTotalUs = t.timestamp;

        const float raw = t.pressure - s.pressure;

        if (zeroing)
        {
            accumulateZero(raw);
            return;
        }

        const float dp = raw - zeroOffsetPa;
        if (dtS <= 0.0f || dtS > 0.5f)
        {
            filteredPa = dp;  // первый отсчёт после обнуления/паузы
        }
        else
        {
            filteredPa += dtS / (Config::PITOT_FILTER_TAU_S + dtS) * (dp - filteredPa);
        }

        updateFault(dp, t.timestamp);

        data.differentialPressurePa = filteredPa;
        data.airDensity = densityOf(s.pressure, s.temperature);
        data.indicatedMs = speedFrom(filteredPa, SEA_LEVEL_DENSITY);
        data.trueMs = speedFrom(filteredPa, data.airDensity);
        data.timestamp = t.timestamp;
    }

    const AirspeedData& getAirspeedData() const override { return data; }

    void calibrateZero() override
    {
        zeroing = true;
        zeroSum = 0;
        zeroCount = 0;
        fault = false;
        negativeSinceUs = 0;
    }

    bool isZeroing() const override { return zeroing; }
    bool hasFault() const { return fault; }
    float getZeroOffset() const { return zeroOffsetPa; }

    const char* getSensorType() const override { return "PITOT"; }

    void printStatus() const override
    {
        Serial.print("PITOT: available="); Serial.print(isAvailable() ? "YES" : "NO");
        if (zeroing) Serial.print(" (обнуление)");
        if (fault) Serial.print(" (отрицательный перепад — шланги/вода?)");
        Serial.print(" dP="); Serial.print(data.differentialPressurePa, 1); Serial.print("Pa");
        Serial.print(" IAS="); Serial.print(data.indicatedMs, 1); Serial.print("m/s");
        Serial.print(" TAS="); Serial.print(data.trueMs, 1); Serial.print("m/s");
        Serial.print(" rho="); Serial.print(data.airDensity, 3);
        Serial.print(" zero="); Serial.print(zeroOffsetPa, 1); Serial.println("Pa");
    }

    // Скорость по перепаду (Па) и плотности (кг/м³) с поправкой
    // PITOT_SCALE; отрицательный перепад — 0.
    static float speedFrom(float differentialPa, float density)
    {
        if (differentialPa <= 0.0f || density <= 0.0f) return 0.0f;
        return Config::PITOT_SCALE * sqrtf(2.0f * differentialPa / density);
    }

    // Плотность сухого воздуха по давлению (Па) и температуре (°C).
    static float densityOf(float pressurePa, float temperatureC)
    {
        const float kelvin = constrain(temperatureC, -40.0f, 60.0f) + 273.15f;
        return pressurePa / (GAS_CONSTANT_AIR * kelvin);
    }

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

    void accumulateZero(float raw)
    {
        zeroSum += raw;
        if (++zeroCount < Config::PITOT_ZERO_SAMPLES) return;

        zeroOffsetPa = zeroSum / zeroCount;
        zeroing = false;
        filteredPa = 0;
        lastTotalUs = 0;

        Serial.print("Pitot: ноль ");
        Serial.print(zeroOffsetPa, 1);
        Serial.println(" Па (смещение между барометрами)");
    }

    void updateFault(float dp, uint32_t now)
    {
        if (dp > -Config::PITOT_NEGATIVE_FAULT_PA)
        {
            negativeSinceUs = 0;
            return;
        }

        if (negativeSinceUs == 0) negativeSinceUs = now;
        if (!fault && now - negativeSinceUs >= Config::PITOT_NEGATIVE_FAULT_MS * 1000UL)
        {
            fault = true;
            Serial.println("Pitot: перепад долго отрицательный — трубка неисправна, скорость отключена");
        }
    }
};
