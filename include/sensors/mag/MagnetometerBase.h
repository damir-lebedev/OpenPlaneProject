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

    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    void update() override
    {
        if (!available) return;

        const uint32_t now = micros();
        if (now - lastReadUs < READ_PERIOD_US) return;
        lastReadUs = now;

        int16_t raw[3];
        if (!readRaw(raw))
        {
            if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
            errorCount++;
            return;  // остаются прошлые данные, а не мусор
        }

        consecutiveErrors = 0;
        process(raw);
        magData.timestamp = now;
    }

    const MagData& getMagData() const override
    {
        return magData;
    }

    // Hard-iron калибровка: 15 с вращать датчик вокруг всех осей.
    // offset = (min+max)/2 по каждой оси — компенсирует постоянные
    // магнитные помехи рядом с датчиком (магниты моторов, провода),
    // но не soft-iron искажения (эллипс вместо окружности).
    // Результат сохраняется в NVS.
    void calibrate() override
    {
        if (!available) return;

        Serial.print(name);
        Serial.println(": калибровка 15 с — вращайте датчик по всем осям...");

        int16_t minV[3] = { INT16_MAX, INT16_MAX, INT16_MAX };
        int16_t maxV[3] = { INT16_MIN, INT16_MIN, INT16_MIN };

        const uint32_t start = millis();
        while (millis() - start < CALIBRATION_MS)
        {
            int16_t raw[3];
            if (readRaw(raw))
            {
                for (int i = 0; i < 3; i++)
                {
                    minV[i] = min(minV[i], raw[i]);
                    maxV[i] = max(maxV[i], raw[i]);
                }
            }
            delay(20);
        }

        for (int i = 0; i < 3; i++)
        {
            offset[i] = (minV[i] + maxV[i]) / 2.0f;
        }
        calibrated = true;
        saveCalibration();

        Serial.print(name);
        Serial.print(": калибровка сохранена, смещения=");
        Serial.print(offset[0]); Serial.print(",");
        Serial.print(offset[1]); Serial.print(",");
        Serial.println(offset[2]);
    }

    const char* getSensorType() const override
    {
        return name;
    }

    void printStatus() const override
    {
        Serial.print(name);
        Serial.print(": available="); Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" calibrated="); Serial.print(calibrated ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
        Serial.print(" mag(uT)="); Serial.print(magData.magX, 1);
        Serial.print(","); Serial.print(magData.magY, 1);
        Serial.print(","); Serial.print(magData.magZ, 1);
        Serial.print(" heading="); Serial.println(magData.headingDegrees, 1);
    }


protected:

    // nvsNamespace — своё пространство NVS у каждого чипа: смещения
    // в единицах АЦП разных чипов несовместимы.
    MagnetometerBase(const char* name, const char* nvsNamespace)
        : name(name),
          nvsNamespace(nvsNamespace)
    {
        memset(&magData, 0, sizeof(magData));
    }

    // --- то, что реализует драйвер конкретного чипа ---

    // Сырые X/Y/Z в осях чипа. false — чип не ответил.
    virtual bool readRaw(int16_t raw[3]) = 0;

    virtual float lsbPerMicroTesla() const = 0;

    // Вызывает begin() драйвера после успешной настройки чипа.
    void setAvailable(bool isAvailable)
    {
        available = isAvailable;
        if (available)
        {
            loadCalibration();
            Serial.print(name);
            Serial.print(": подключён, калибровка ");
            Serial.println(calibrated ? "загружена из NVS" : "НЕ выполнена (курс неточный, команда 'm')");
        }
    }


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

    void process(const int16_t raw[3])
    {
        const float scale = lsbPerMicroTesla();
        const float chipX = (raw[0] - offset[0]) / scale;
        const float chipY = (raw[1] - offset[1]) / scale;

        float x, y;
        SensorMounting::rotateToBody(Config::MAG_ROTATION_CW_DEG, chipX, chipY, x, y);

        magData.magX = x;
        magData.magY = y;
        magData.magZ = (raw[2] - offset[2]) / scale;

        // Оси самолёта X к носу, Y влево: нос на север — поле вдоль +X
        // (0°), нос на восток — север слева, поле вдоль +Y (90°).
        float heading = atan2f(magData.magY, magData.magX) * RAD_TO_DEG;
        if (heading < 0) heading += 360.0f;
        magData.headingDegrees = heading;
    }

    void loadCalibration()
    {
        // На запись, хотя только читаем: в режиме "только чтение"
        // отсутствующее пространство имён (калибровку ещё ни разу не
        // сохраняли) Preferences печатает как ошибку в лог.
        Preferences prefs;
        if (!prefs.begin(nvsNamespace, false)) return;

        calibrated = prefs.getBool("ok", false);
        if (calibrated)
        {
            offset[0] = prefs.getFloat("x", 0);
            offset[1] = prefs.getFloat("y", 0);
            offset[2] = prefs.getFloat("z", 0);
        }
        prefs.end();
    }

    void saveCalibration()
    {
        Preferences prefs;
        if (!prefs.begin(nvsNamespace, false)) return;

        prefs.putFloat("x", offset[0]);
        prefs.putFloat("y", offset[1]);
        prefs.putFloat("z", offset[2]);
        prefs.putBool("ok", true);
        prefs.end();
    }
};
