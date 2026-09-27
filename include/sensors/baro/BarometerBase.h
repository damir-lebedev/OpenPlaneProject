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

    bool isAvailable() const override;

    void update() override;

    const BarometerData& getBarometerData() const override;

    // На земле перед полётом: среднее из 20 отсчётов — нулевая высота.
    void calibrateAltitude() override;

    void setSeaLevelPressure(float pressurePa) override;

    const char* getSensorType() const override;

    void printStatus() const override;


protected:

    BarometerBase(const char* sensorName, uint32_t pollIntervalUs);

    // --- то, что реализует драйвер конкретного чипа ---

    // ready = есть новый отсчёт с прошлого readSample(). false — чип
    // не ответил. Чипам без флага готовности достаточно ready = true
    // и периода опроса не короче времени измерения.
    virtual bool isNewSampleReady(bool& ready) = 0;

    // Давление, Па, и температура, °C. false — чип не ответил.
    virtual bool readSample(float& pressurePa, float& temperatureC) = 0;

    void setAvailable(bool isAvailable);


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

    void onReadError();

    float absoluteAltitude(float pressurePa) const;

    void updateAltitude(uint32_t now);
};
