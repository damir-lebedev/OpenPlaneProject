#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// QMC5883P (плата GY-273, на чипе бывает маркировка "5883P"/
// "HP5883") — 3-осевой магнитометр QST
//
// I2C, адрес 0x2C (фиксированный). Собственная минимальная
// реализация через II2CBus, без vendor-библиотек.
//
// Раскладка регистров сверена с даташитом QMC5883P и проверена
// вживую на плате GY-273 (chip ID 0x80 по адресу 0x2C). Она НЕ
// совпадает с QMC5883L (адрес 0x0D, данные с 0x00, управление в
// 0x09) — драйверы не взаимозаменяемы:
//   0x00      CHIP_ID = 0x80
//   0x01-0x06 X/Y/Z, int16 little-endian
//   0x09      STATUS: бит0 DRDY, бит1 OVFL
//   0x0A      CONTROL1: [1:0] MODE, [3:2] ODR, [5:4] OSR1, [7:6] OSR2
//   0x0B      CONTROL2: [1:0] SET/RESET, [3:2] RNG, [7] SOFT_RST
//   0x29      знаки осей (даташит рекомендует 0x06)
//
// Калибровка hard-iron (смещения по осям) хранится в NVS и
// переживает перезагрузку — поэтому calibrate() (15 секунд
// вращения) не нужно запускать при каждом включении.
// ============================================================

#include "SensorInterface.h"
#include "../hal/II2CBus.h"

class QMC5883P_Sensor : public MagnetometerSensor
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS = 0x2C;

    explicit QMC5883P_Sensor(II2CBus& bus, uint8_t address = DEFAULT_ADDRESS)
        : i2c(bus),
          i2cAddress(address)
    {
        memset(&magData, 0, sizeof(magData));
    }

    bool begin() override
    {
        const int chipId = i2c.readRegister(i2cAddress, REG_CHIP_ID);

        if (chipId != CHIP_ID_VALUE)
        {
            Serial.print("QMC5883P: не отвечает (chip ID ");
            Serial.print(chipId < 0 ? String("нет ответа") : String("0x") + String(chipId, HEX));
            Serial.println("), магнитометр недоступен");
            available = false;
            return false;
        }

        i2c.writeRegister(i2cAddress, REG_CONTROL2, 0x80);  // SOFT_RST
        delay(10);

        // Инициализация по примеру из даташита: знаки осей, SET/RESET
        // включён + диапазон ±8 Гс, режим normal, ODR 200 Гц, OSR1=8, OSR2=8.
        const bool ok =
            i2c.writeRegister(i2cAddress, REG_AXIS_SIGN, 0x06) &&
            i2c.writeRegister(i2cAddress, REG_CONTROL2, 0x08) &&
            i2c.writeRegister(i2cAddress, REG_CONTROL1, 0xCD);

        if (!ok)
        {
            Serial.println("QMC5883P: ошибка записи регистров, магнитометр недоступен");
            available = false;
            return false;
        }

        loadCalibration();

        available = true;
        Serial.print("QMC5883P: подключён, калибровка ");
        Serial.println(calibrated ? "загружена из NVS" : "НЕ выполнена (курс будет неточным)");

        return true;
    }

    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    // Чип обновляет данные с ODR 200 Гц, но для курса хватает 50 Гц —
    // чаще не читаем, чтобы не занимать общую с IMU шину.
    void update() override
    {
        if (!available) return;

        const uint32_t now = micros();
        if (now - lastReadUs < READ_PERIOD_US) return;
        lastReadUs = now;

        if (!readRawData())
        {
            if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
            errorCount++;
            return;  // остаются прошлые данные, а не мусор
        }

        consecutiveErrors = 0;
        applyCalibration();
        calculateHeading();

        magData.timestamp = now;
    }

    const MagData& getMagData() const override
    {
        return magData;
    }

    // Hard-iron калибровка: вызвать и в течение 15 сек вращать плату
    // вокруг всех осей. offset = (min+max)/2 по каждой оси —
    // компенсирует постоянные магнитные помехи рядом с датчиком
    // (моторы, провода), но НЕ soft-iron (эллипс вместо окружности).
    // Результат сохраняется в NVS.
    void calibrate() override
    {
        if (!available) return;

        Serial.println("QMC5883P: калибровка 15 сек — вращайте датчик по всем осям...");

        int16_t minX = INT16_MAX, maxX = INT16_MIN;
        int16_t minY = INT16_MAX, maxY = INT16_MIN;
        int16_t minZ = INT16_MAX, maxZ = INT16_MIN;

        const uint32_t startTime = millis();

        while (millis() - startTime < 15000)
        {
            if (readRawData())
            {
                minX = min(minX, rawX); maxX = max(maxX, rawX);
                minY = min(minY, rawY); maxY = max(maxY, rawY);
                minZ = min(minZ, rawZ); maxZ = max(maxZ, rawZ);
            }

            delay(20);
        }

        offsetX = (minX + maxX) / 2.0f;
        offsetY = (minY + maxY) / 2.0f;
        offsetZ = (minZ + maxZ) / 2.0f;
        calibrated = true;

        saveCalibration();

        Serial.print("QMC5883P: калибровка сохранена, offset=");
        Serial.print(offsetX); Serial.print(",");
        Serial.print(offsetY); Serial.print(",");
        Serial.println(offsetZ);
    }

    const char* getSensorType() const override
    {
        return "QMC5883P (GY-273)";
    }

    void printStatus() const override
    {
        Serial.print("QMC5883P: available=");
        Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" calibrated=");
        Serial.print(calibrated ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
        Serial.print(" mag(uT)="); Serial.print(magData.magX, 1);
        Serial.print(","); Serial.print(magData.magY, 1);
        Serial.print(","); Serial.print(magData.magZ, 1);
        Serial.print(" heading="); Serial.println(magData.headingDegrees, 1);
    }


private:

    static constexpr uint8_t REG_CHIP_ID    = 0x00;
    static constexpr uint8_t REG_DATA_X_LSB = 0x01;  // далее X,Y,Z по 2 байта LE
    static constexpr uint8_t REG_CONTROL1   = 0x0A;
    static constexpr uint8_t REG_CONTROL2   = 0x0B;
    static constexpr uint8_t REG_AXIS_SIGN  = 0x29;
    static constexpr uint8_t CHIP_ID_VALUE  = 0x80;

    // Диапазон ±8 Гс: 3750 LSB/Гс, 1 Гс = 100 мкТл -> 37.5 LSB/мкТл.
    static constexpr float SENSITIVITY_LSB_PER_UT = 37.5f;

    static constexpr uint32_t READ_PERIOD_US = 20000;  // 50 Гц
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 25;  // ~0.5 с без ответа

    II2CBus& i2c;
    uint8_t i2cAddress;
    bool available = false;
    bool calibrated = false;

    int16_t rawX = 0, rawY = 0, rawZ = 0;
    float offsetX = 0, offsetY = 0, offsetZ = 0;

    uint32_t lastReadUs = 0;
    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    MagData magData;

    bool readRawData()
    {
        uint8_t b[6];
        if (!i2c.readRegisters(i2cAddress, REG_DATA_X_LSB, b, sizeof(b))) return false;

        rawX = (int16_t)((b[1] << 8) | b[0]);
        rawY = (int16_t)((b[3] << 8) | b[2]);
        rawZ = (int16_t)((b[5] << 8) | b[4]);
        return true;
    }

    void applyCalibration()
    {
        magData.magX = (rawX - offsetX) / SENSITIVITY_LSB_PER_UT;
        magData.magY = (rawY - offsetY) / SENSITIVITY_LSB_PER_UT;
        magData.magZ = (rawZ - offsetZ) / SENSITIVITY_LSB_PER_UT;
    }

    // 2D-курс без компенсации наклона (roll/pitch) — датчик
    // предполагается установленным горизонтально. Направление
    // (растёт ли курс при повороте по часовой) зависит от того, как
    // плата GY-273 повёрнута относительно самолёта, — проверить
    // вживую перед тем, как на курс будет опираться какой-либо режим.
    void calculateHeading()
    {
        float heading = atan2f(magData.magY, magData.magX) * RAD_TO_DEG;
        if (heading < 0) heading += 360.0f;

        magData.headingDegrees = heading;
    }

    void loadCalibration()
    {
        // Открываем на запись, хотя только читаем: в режиме "только
        // чтение" отсутствующее пространство имён (калибровку ещё ни
        // разу не сохраняли) Preferences печатает как ошибку в лог.
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) return;

        calibrated = prefs.getBool("ok", false);
        if (calibrated)
        {
            offsetX = prefs.getFloat("x", 0);
            offsetY = prefs.getFloat("y", 0);
            offsetZ = prefs.getFloat("z", 0);
        }
        prefs.end();
    }

    void saveCalibration()
    {
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) return;

        prefs.putFloat("x", offsetX);
        prefs.putFloat("y", offsetY);
        prefs.putFloat("z", offsetZ);
        prefs.putBool("ok", true);
        prefs.end();
    }

    static constexpr const char* NVS_NAMESPACE = "qmc5883p";
};
