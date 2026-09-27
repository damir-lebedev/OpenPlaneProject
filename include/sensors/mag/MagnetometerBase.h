#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "config/Config.h"
#include "sensors/SensorInterface.h"
#include "sensors/SensorMounting.h"

// ============================================================
// MAGNETOMETER BASE — общая часть всех компасов проекта
//
// Драйвер конкретного чипа (QMC5883P_Sensor, QMC5883L_Sensor)
// реализует только begin() (опознать и настроить чип),
// readRaw() (сырые X/Y/Z) и чувствительность. Здесь — одинаково
// для любого чипа:
//
//   • опрос 50 Гц: для курса больше не нужно, а шина общая с IMU;
//   • hard-iron калибровка (min/max по осям при вращении) с
//     хранением смещений в NVS — переживает перезагрузку, поэтому
//     15 секунд вращения не нужны при каждом включении;
//   • поворот осей чипа в оси самолёта (Config::MAG_ROTATION_CW_DEG)
//     и курс atan2(Y, X) в осях X к носу, Y влево;
//   • счёт ошибок шины и isAvailable() по ним.
//
// Курс без компенсации наклона: верен, пока самолёт почти в
// горизонте. Направление отсчёта (растёт ли курс при повороте по
// часовой) надо проверить на собранном самолёте — оно зависит и от
// установки, и от того, как оси чипа разведены на конкретной плате.
// ============================================================

class MagnetometerBase : public MagnetometerSensor
{
public:

    bool isAvailable() const override;

    void update() override;

    const MagData& getMagData() const override;

    // Hard-iron калибровка: 15 с вращать датчик вокруг всех осей.
    // offset = (min+max)/2 по каждой оси — компенсирует постоянные
    // магнитные помехи рядом с датчиком (магниты моторов, провода),
    // но не soft-iron искажения (эллипс вместо окружности).
    // Результат сохраняется в NVS.
    void calibrate() override;

    const char* getSensorType() const override;

    void printStatus() const override;


protected:

    // nvsNamespace — своё пространство NVS у каждого чипа: смещения
    // в единицах АЦП разных чипов несовместимы.
    MagnetometerBase(const char* sensorName, const char* nvsName);

    // --- то, что реализует драйвер конкретного чипа ---

    // Сырые X/Y/Z в осях чипа. false — чип не ответил.
    virtual bool readRaw(int16_t raw[3]) = 0;

    virtual float lsbPerMicroTesla() const = 0;

    // Вызывает begin() драйвера после успешной настройки чипа.
    void setAvailable(bool isAvailable);


private:

    static constexpr uint32_t READ_PERIOD_US = 20000;          // 50 Гц
    static constexpr uint32_t CALIBRATION_MS = 15000;
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 25;      // ~0.5 с без ответа

    const char* name;
    const char* nvsNamespace;
    bool available = false;
    bool calibrated = false;

    float offset[3] = {};

    uint32_t lastReadUs = 0;
    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    MagData magData;

    void process(const int16_t raw[3]);

    void loadCalibration();

    void saveCalibration();
};
