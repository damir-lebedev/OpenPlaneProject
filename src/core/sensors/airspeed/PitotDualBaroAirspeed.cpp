// Реализация sensors/airspeed/PitotDualBaroAirspeed.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/airspeed/PitotDualBaroAirspeed.h"


PitotDualBaroAirspeed::PitotDualBaroAirspeed(BarometerSensor& pitotBaro, BarometerSensor& staticBaro)
: total(pitotBaro),
      stat(staticBaro),
      data()
{
}

auto PitotDualBaroAirspeed::begin() -> bool
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

auto PitotDualBaroAirspeed::isAvailable() const -> bool
{
    return begun && !zeroing && !fault &&
           total.isAvailable() && stat.isAvailable() &&
           data.timestamp != 0 && micros() - data.timestamp < Config::PITOT_STALE_US;
}

auto PitotDualBaroAirspeed::update() -> void
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

auto PitotDualBaroAirspeed::calibrateZero() -> void
{
    zeroing = true;
    zeroSum = 0;
    zeroCount = 0;
    fault = false;
    negativeSinceUs = 0;
}

auto PitotDualBaroAirspeed::printStatus() const -> void
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

auto PitotDualBaroAirspeed::speedFrom(float differentialPa, float density) -> float
{
    if (differentialPa <= 0.0f || density <= 0.0f) return 0.0f;
    return Config::PITOT_SCALE * sqrtf(2.0f * differentialPa / density);
}

auto PitotDualBaroAirspeed::densityOf(float pressurePa, float temperatureC) -> float
{
    const float kelvin = constrain(temperatureC, -40.0f, 60.0f) + 273.15f;
    return pressurePa / (GAS_CONSTANT_AIR * kelvin);
}

auto PitotDualBaroAirspeed::accumulateZero(float raw) -> void
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

auto PitotDualBaroAirspeed::updateFault(float dp, uint32_t now) -> void
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
