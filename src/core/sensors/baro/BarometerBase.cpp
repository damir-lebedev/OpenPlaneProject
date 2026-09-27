// Реализация sensors/baro/BarometerBase.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/baro/BarometerBase.h"


auto BarometerBase::isAvailable() const -> bool
{
    return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
}

auto BarometerBase::update() -> void
{
    if (!available) return;

    const uint32_t now = micros();
    if (now - lastPollUs < pollPeriodUs) return;
    lastPollUs = now;

    bool ready = false;
    if (!isNewSampleReady(ready))
    {
        onReadError();
        return;
    }
    if (!ready) return;

    float pressurePa, temperatureC;
    if (!readSample(pressurePa, temperatureC))
    {
        onReadError();
        return;
    }

    consecutiveErrors = 0;
    baroData.pressure = pressurePa;
    baroData.temperature = temperatureC;
    updateAltitude(now);
    baroData.timestamp = now;
}

auto BarometerBase::getBarometerData() const -> const BarometerData&
{
    return baroData;
}

auto BarometerBase::calibrateAltitude() -> void
{
    if (!available) return;

    Serial.print(name);
    Serial.println(": калибровка высоты...");

    float sum = 0;
    int samples = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        delay(CALIBRATION_INTERVAL_MS);

        float pressurePa, temperatureC;
        if (readSample(pressurePa, temperatureC))
        {
            sum += absoluteAltitude(pressurePa);
            samples++;
        }
    }

    if (samples == 0)
    {
        Serial.print(name);
        Serial.println(": калибровка не удалась — датчик не отвечает");
        return;
    }

    baseAltitude = sum / samples;

    baroData.altitude = 0;
    baroData.verticalSpeed = 0;
    previousAltitude = 0;
    lastSampleUs = 0;  // следующий отсчёт не считает скорость от старой базы

    Serial.print(name);
    Serial.print(": калибровка завершена, база=");
    Serial.print(baseAltitude);
    Serial.println(" м над уровнем моря (по стандартной атмосфере)");
}

auto BarometerBase::setSeaLevelPressure(float pressurePa) -> void
{
    seaLevelPressure = pressurePa;
}

auto BarometerBase::getSensorType() const -> const char*
{
    return name;
}

auto BarometerBase::printStatus() const -> void
{
    Serial.print(name);
    Serial.print(": available="); Serial.print(isAvailable() ? "YES" : "NO");
    Serial.print(" errors="); Serial.print(errorCount);
    Serial.print(" pressure="); Serial.print(baroData.pressure / 100.0f); Serial.print("hPa");
    Serial.print(" altitude="); Serial.print(baroData.altitude); Serial.print("m");
    Serial.print(" climb="); Serial.print(baroData.verticalSpeed, 2); Serial.print("m/s");
    Serial.print(" temp="); Serial.print(baroData.temperature); Serial.println("C");
}

BarometerBase::BarometerBase(const char* sensorName, uint32_t pollIntervalUs)
: name(sensorName),
      pollPeriodUs(pollIntervalUs),
      baroData()
{
}

auto BarometerBase::setAvailable(bool isAvailable) -> void
{
    available = isAvailable;
}

auto BarometerBase::onReadError() -> void
{
    if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
    errorCount++;
}

auto BarometerBase::absoluteAltitude(float pressurePa) const -> float
{
    return 44330.0f * (1.0f - powf(pressurePa / seaLevelPressure, 0.1903f));
}

auto BarometerBase::updateAltitude(uint32_t now) -> void
{
    baroData.altitude = absoluteAltitude(baroData.pressure) - baseAltitude;

    const float dt = (now - lastSampleUs) / 1000000.0f;

    if (lastSampleUs != 0 && dt > 0 && dt < 0.5f)
    {
        const float rawClimb = (baroData.altitude - previousAltitude) / dt;
        const float alpha = dt / (CLIMB_FILTER_TAU_S + dt);
        baroData.verticalSpeed += alpha * (rawClimb - baroData.verticalSpeed);
    }

    previousAltitude = baroData.altitude;
    lastSampleUs = now;
}
