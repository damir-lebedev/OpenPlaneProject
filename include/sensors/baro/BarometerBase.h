#pragma once
#include <Arduino.h>

#include "sensors/SensorInterface.h"

// ============================================================
// BAROMETER BASE — общая часть всех барометров проекта
//
// Драйвер конкретного чипа (BMP388_Sensor, BME280_Sensor)
// реализует только работу с железом: begin() (опознать и настроить
// чип), isNewSampleReady() (есть ли новый отсчёт) и readSample()
// (давление, температура). Здесь — одинаково для любого чипа:
//
//   • опрос не чаще pollPeriodUs и чтение только нового отсчёта:
//     update() вызывается ~500 раз в секунду, а чип меряет десятки
//     раз в секунду. Если читать каждый цикл, вертикальная скорость
//     считается по одному и тому же отсчёту (0 м/с) вперемешку со
//     скачками (Δh / 2 мс) — шум в десятки м/с;
//   • высота по международной стандартной атмосфере
//     h = 44330 · (1 − (P/P0)^(1/5.255)) — относительно точки
//     калибровки (при включении);
//   • вертикальная скорость — производная высоты по реальным
//     отсчётам через ФНЧ (τ = CLIMB_FILTER_TAU_S);
//   • счёт ошибок шины и isAvailable() по ним.
// ============================================================

class BarometerBase : public BarometerSensor
{
public:

    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    void update() override
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

    const BarometerData& getBarometerData() const override
    {
        return baroData;
    }

    // На земле перед полётом: среднее из 20 отсчётов — нулевая высота.
    void calibrateAltitude() override
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

    void setSeaLevelPressure(float pressurePa) override
    {
        seaLevelPressure = pressurePa;
    }

    const char* getSensorType() const override
    {
        return name;
    }

    void printStatus() const override
    {
        Serial.print(name);
        Serial.print(": available="); Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
        Serial.print(" pressure="); Serial.print(baroData.pressure / 100.0f); Serial.print("hPa");
        Serial.print(" altitude="); Serial.print(baroData.altitude); Serial.print("m");
        Serial.print(" climb="); Serial.print(baroData.verticalSpeed, 2); Serial.print("m/s");
        Serial.print(" temp="); Serial.print(baroData.temperature); Serial.println("C");
    }


protected:

    BarometerBase(const char* sensorName, uint32_t pollIntervalUs)
        : name(sensorName),
          pollPeriodUs(pollIntervalUs),
          baroData()
    {
    }

    // --- то, что реализует драйвер конкретного чипа ---

    // ready = есть новый отсчёт с прошлого readSample(). false — чип
    // не ответил. Чипам без флага готовности достаточно ready = true
    // и периода опроса не короче времени измерения.
    virtual bool isNewSampleReady(bool& ready) = 0;

    // Давление, Па, и температура, °C. false — чип не ответил.
    virtual bool readSample(float& pressurePa, float& temperatureC) = 0;

    void setAvailable(bool isAvailable)
    {
        available = isAvailable;
    }


private:

    static constexpr int CALIBRATION_SAMPLES = 20;
    static constexpr uint32_t CALIBRATION_INTERVAL_MS = 50;

    // ~0.5 с подряд без ответа (при опросе раз в 5 мс) -> недоступен.
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 100;

    // ФНЧ вертикальной скорости: производная высоты шумит (0.1 м
    // шума за 0.02 с = 5 м/с), сглаживание ~0.5 с оставляет полезный
    // сигнал для ALT_HOLD.
    static constexpr float CLIMB_FILTER_TAU_S = 0.5f;

    const char* name;
    uint32_t pollPeriodUs;
    bool available = false;

    float seaLevelPressure = 101325.0f;
    float baseAltitude = 0;
    float previousAltitude = 0;

    uint32_t lastPollUs = 0;
    uint32_t lastSampleUs = 0;
    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    BarometerData baroData;

    void onReadError()
    {
        if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
        errorCount++;
    }

    float absoluteAltitude(float pressurePa) const
    {
        return 44330.0f * (1.0f - powf(pressurePa / seaLevelPressure, 0.1903f));
    }

    void updateAltitude(uint32_t now)
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
};
