// Реализация sensors/mag/MagnetometerBase.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/mag/MagnetometerBase.h"


auto MagnetometerBase::isAvailable() const -> bool
{
    return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
}

auto MagnetometerBase::update() -> void
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

auto MagnetometerBase::getMagData() const -> const MagData&
{
    return magData;
}

auto MagnetometerBase::calibrate() -> void
{
    if (!available) return;

    Serial.print(name);
    Serial.println(": калибровка 15 с — вращайте датчик по всем осям...");

    int16_t minV[3] = { INT16_MAX, INT16_MAX, INT16_MAX };
    int16_t maxV[3] = { INT16_MIN, INT16_MIN, INT16_MIN };
    uint32_t samples = 0;

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
            samples++;
        }
        delay(20);
    }

    // Ни одного отсчёта — min/max остались INT16_MAX/INT16_MIN, и
    // "смещение" из них испортило бы прежнюю калибровку в NVS.
    if (samples == 0)
    {
        Serial.print(name);
        Serial.println(": калибровка не удалась — датчик не отвечает, прежняя калибровка сохранена");
        return;
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

auto MagnetometerBase::getSensorType() const -> const char*
{
    return name;
}

auto MagnetometerBase::printStatus() const -> void
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

MagnetometerBase::MagnetometerBase(const char* sensorName, const char* nvsName)
: name(sensorName),
      nvsNamespace(nvsName),
      magData()
{
}

auto MagnetometerBase::setAvailable(bool isAvailable) -> void
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

auto MagnetometerBase::process(const int16_t raw[3]) -> void
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
    float heading = atan2f(magData.magY, magData.magX) * static_cast<float>(RAD_TO_DEG);
    if (heading < 0) heading += 360.0f;
    magData.headingDegrees = heading;
}

auto MagnetometerBase::loadCalibration() -> void
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

auto MagnetometerBase::saveCalibration() -> void
{
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, false)) return;

    prefs.putFloat("x", offset[0]);
    prefs.putFloat("y", offset[1]);
    prefs.putFloat("z", offset[2]);
    prefs.putBool("ok", true);
    prefs.end();
}
