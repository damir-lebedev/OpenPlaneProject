#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/SensorInterface.h"

// ============================================================
// НЕСКОЛЬКО ДАТЧИКОВ ОДНОГО ТИПА (sensors/SensorSelection.h:
// SENSOR_IMU, SENSOR_IMU_2, SENSOR_IMU_3 и так же у барометра и компаса)
//
// Цепочка — сама датчик своего типа (ImuSensor, BarometerSensor,
// MagnetometerSensor): автопилот, консоль и чёрный ящик видят один
// датчик и не знают, что за ним несколько чипов. begin() опознаёт чипы
// по порядку выбора и берёт первый ответивший — он и работает весь
// полёт, остальные не опрашиваются. Так одна прошивка летает с любым из
// выбранных модулей: снял GY-521, поставил LSM6DSV — перепрошивать не
// надо.
//
// Каждый чип ищется на всех шинах I2C платы (I2cRegisterDevice): ответил
// по адресу чужой чип — пробуется следующее место
// (IRegisterDevice::nextPlacement()).
//
// Переключения на запасной чип в полёте нет: смена IMU на лету — скачок
// углов и нулей гироскопа. Отказ виден до взлёта: в консоли ('s') и в
// приложении написано, какой чип нашёлся и где.
// ============================================================

class SensorChainCore
{
public:

    static constexpr uint8_t MAX = 3;

    struct Entry
    {
        Sensor* sensor = nullptr;
        IRegisterDevice* device = nullptr;   // nullptr — у датчика нет регистров (так не бывает, но не падаем)

        // Конструкторы явно: ядро ESP32 собирается в C++11, там структура с
        // инициализаторами полей — не агрегат и {a, b} не принимает.
        Entry() = default;
        Entry(Sensor* s, IRegisterDevice* d) : sensor(s), device(d) {}
    };

    // label — тип для консоли: "IMU", "Барометр", "Компас".
    SensorChainCore(const char* label, Entry a, Entry b, Entry c)
        : what(label),
          entries{ a, b, c }
    {
    }

    // Сколько чипов выбрано (0 — тип выключен, SENSOR_*_NONE).
    uint8_t configuredCount() const
    {
        uint8_t n = 0;
        for (const Entry& e : entries) n += e.sensor ? 1 : 0;
        return n;
    }

    // Опознать по порядку; true — какой-то ответил, он и работает.
    bool beginFirst()
    {
        active = -1;
        for (uint8_t i = 0; i < MAX; ++i)
        {
            Entry& e = entries[i];
            if (!e.sensor) continue;
            for (;;)
            {
                if (e.sensor->begin())
                {
                    active = static_cast<int8_t>(i);
                    report();
                    return true;
                }
                if (!e.device || !e.device->nextPlacement()) break;
            }
        }
        report();
        return false;
    }

    Sensor* activeSensor() const { return active >= 0 ? entries[active].sensor : nullptr; }

    // Номер работающего чипа в порядке выбора (0 — основной), -1 — ни один.
    int8_t activeIndex() const { return active; }

    // Где работающий чип: "I2C PB9/PB8 0x6A"; пусто — не найден.
    void describeActive(char* out, size_t size) const
    {
        if (size == 0) return;
        out[0] = '\0';
        if (active >= 0 && entries[active].device) entries[active].device->describe(out, size);
    }

    // Имя для лога и чёрного ящика: работающий чип, иначе первый выбранный.
    const char* typeName() const
    {
        if (const Sensor* s = activeSensor()) return s->getSensorType();
        for (const Entry& e : entries)
        {
            if (e.sensor) return e.sensor->getSensorType();
        }
        return "нет";
    }

    // Что выбрано и что нашлось — строка в консоль.
    void report() const
    {
        if (configuredCount() == 0) return;
        Serial.print(what);
        Serial.print(": ");
        if (active < 0)
        {
            Serial.print("не найден ни один из выбранных (");
            bool first = true;
            for (const Entry& e : entries)
            {
                if (!e.sensor) continue;
                if (!first) Serial.print(", ");
                first = false;
                Serial.print(e.sensor->getSensorType());
            }
            Serial.println(")");
            return;
        }
        char where[32];
        describeActive(where, sizeof(where));
        Serial.print(entries[active].sensor->getSensorType());
        Serial.print(" — ");
        Serial.print(where);
        if (configuredCount() > 1) Serial.print(active == firstConfigured() ? " (основной)" : " (запасной: основной не ответил)");
        Serial.println();
    }


private:

    const char* what;
    Entry entries[MAX];
    int8_t active = -1;

    int8_t firstConfigured() const
    {
        for (uint8_t i = 0; i < MAX; ++i)
        {
            if (entries[i].sensor) return static_cast<int8_t>(i);
        }
        return -1;
    }
};


// ------------------------------------------------------------
// IMU
// ------------------------------------------------------------
class ImuChain : public ImuSensor
{
public:

    ImuChain(SensorChainCore::Entry a, SensorChainCore::Entry b = SensorChainCore::Entry(), SensorChainCore::Entry c = SensorChainCore::Entry())
        : core("IMU", a, b, c)
    {
    }

    bool begin() override { return core.beginFirst(); }
    bool isAvailable() const override { return imu() && imu()->isAvailable(); }
    void update() override { if (imu()) imu()->update(); }
    const char* getSensorType() const override { return core.typeName(); }
    void printStatus() const override
    {
        core.report();   // какой чип из выбранных и на какой шине
        if (imu()) imu()->printStatus();
    }

    const ImuData& getImuData() const override { return imu() ? imu()->getImuData() : empty; }
    void calibrate() override { if (imu()) imu()->calibrate(); }
    void setYaw(float yaw) override { if (imu()) imu()->setYaw(yaw); }
    void calibrateOrientation() override { if (imu()) imu()->calibrateOrientation(); }
    bool isMountCalibrated() const override { return imu() && imu()->isMountCalibrated(); }
    const char* getPreflightProblem() const override { return imu() ? imu()->getPreflightProblem() : nullptr; }
    bool readChipUp(float chipUp[3]) const override { return imu() && imu()->readChipUp(chipUp); }
    bool getOrientation(ImuOrientation& out) const override { return imu() && imu()->getOrientation(out); }
    bool setGroundCalibration(const ImuOrientation& level, const float parkChipUp[3]) override
    {
        return imu() && imu()->setGroundCalibration(level, parkChipUp);
    }
    bool getParkingAttitude(float& pitchDeg, float& rollDeg) const override
    {
        return imu() && imu()->getParkingAttitude(pitchDeg, rollDeg);
    }
    void setAccelCorrection(bool enabled) override { if (imu()) imu()->setAccelCorrection(enabled); }
    void trimGyroBias(const float residualDps[3]) override { if (imu()) imu()->trimGyroBias(residualDps); }
    II2CBus* auxI2c() override { return imu() ? imu()->auxI2c() : nullptr; }

    const SensorChainCore& chain() const { return core; }


private:

    SensorChainCore core;
    ImuData empty{};

    ImuSensor* imu() const { return static_cast<ImuSensor*>(core.activeSensor()); }
};


// ------------------------------------------------------------
// Барометр
// ------------------------------------------------------------
class BaroChain : public BarometerSensor
{
public:

    BaroChain(SensorChainCore::Entry a, SensorChainCore::Entry b = SensorChainCore::Entry(), SensorChainCore::Entry c = SensorChainCore::Entry())
        : core("Барометр", a, b, c)
    {
    }

    bool begin() override { return core.beginFirst(); }
    bool isAvailable() const override { return baro() && baro()->isAvailable(); }
    void update() override { if (baro()) baro()->update(); }
    const char* getSensorType() const override { return core.typeName(); }
    void printStatus() const override
    {
        core.report();   // какой чип из выбранных и на какой шине
        if (baro()) baro()->printStatus();
    }

    const BarometerData& getBarometerData() const override { return baro() ? baro()->getBarometerData() : empty; }
    void calibrateAltitude() override { if (baro()) baro()->calibrateAltitude(); }
    void setSeaLevelPressure(float pressure) override { if (baro()) baro()->setSeaLevelPressure(pressure); }

    const SensorChainCore& chain() const { return core; }


private:

    SensorChainCore core;
    BarometerData empty{};

    BarometerSensor* baro() const { return static_cast<BarometerSensor*>(core.activeSensor()); }
};


// ------------------------------------------------------------
// Компас
// ------------------------------------------------------------
class MagChain : public MagnetometerSensor
{
public:

    MagChain(SensorChainCore::Entry a, SensorChainCore::Entry b = SensorChainCore::Entry(), SensorChainCore::Entry c = SensorChainCore::Entry())
        : core("Компас", a, b, c)
    {
    }

    bool begin() override { return core.beginFirst(); }
    bool isAvailable() const override { return mag() && mag()->isAvailable(); }
    void update() override { if (mag()) mag()->update(); }
    const char* getSensorType() const override { return core.typeName(); }
    void printStatus() const override
    {
        core.report();   // какой чип из выбранных и на какой шине
        if (mag()) mag()->printStatus();
    }

    const MagData& getMagData() const override { return mag() ? mag()->getMagData() : empty; }
    void calibrate() override { if (mag()) mag()->calibrate(); }
    bool isCalibrated() const override { return mag() && mag()->isCalibrated(); }

    const SensorChainCore& chain() const { return core; }


private:

    SensorChainCore core;
    MagData empty{};

    MagnetometerSensor* mag() const { return static_cast<MagnetometerSensor*>(core.activeSensor()); }
};
