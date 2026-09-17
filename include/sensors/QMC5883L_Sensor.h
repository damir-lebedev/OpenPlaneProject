#pragma once
#include <Arduino.h>

// ============================================================
// QMC5883L (плата GY-273) — 3-осевой магнитометр
//
// I2C. Собственная минимальная реализация через II2CBus, без
// vendor-библиотек — по структуре аналогична MPU6050_Sensor.h.
//
// В отличие от QMC5883P_Sensor.h (см. предупреждение в том файле)
// адрес и регистры здесь — это проверенная по датащиту QMC5883L
// раскладка, а не заготовка: 0x0D — стандартный 7-битный I2C-адрес
// этого чипа, регистр 0x0B (SET/RESET Period) обязателен к записи
// 0x01 по датащиту, а 0x09 (Control Register 1) с MODE=continuous,
// ODR=200Hz, RNG=±8Гаусс, OSR=512 даёт ровно 0x1D. QMC5883L — самый
// распространённый чип на платах "GY-273", в отличие от более
// редкого QMC5883P.
// ============================================================

#include "SensorInterface.h"
#include "../hal/II2CBus.h"

class QMC5883L_Sensor : public MagnetometerSensor
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS = 0x0D;

    explicit QMC5883L_Sensor(II2CBus& bus, uint8_t address = DEFAULT_ADDRESS)
        : i2c(bus),
          i2cAddress(address),
          available(false)
    {
        memset(&magData, 0, sizeof(magData));
        calibration.offsetX = 0;
        calibration.offsetY = 0;
        calibration.offsetZ = 0;
    }

    bool begin() override
    {
        delay(100);

        if (!checkConnection())
        {
            Serial.println("QMC5883L: датчик не отвечает на I2C, магнитометр недоступен");
            available = false;
            return false;
        }

        initialize();

        available = true;
        Serial.println("QMC5883L: подключён");

        return true;
    }

    bool isAvailable() const override
    {
        return available;
    }

    void update() override
    {
        if (!available) return;

        readRawData();
        applyCalibration();
        calculateHeading();

        magData.timestamp = micros();
    }

    const MagData& getMagData() const override
    {
        return magData;
    }

    // Offset-калибровка (hard-iron): вызвать и в течение ~10-20 сек
    // вращать плату вокруг всех осей. Берёт min/max по каждой оси,
    // offset = (min+max)/2 — компенсирует постоянные магнитные
    // помехи рядом с датчиком (моторы, провода), но НЕ soft-iron
    // искажения (эллипс вместо окружности) — этого намеренно нет.
    void calibrate() override
    {
        if (!available) return;

        Serial.println("QMC5883L: калибровка (вращайте датчик по всем осям)...");

        float minX = 32767, maxX = -32768;
        float minY = 32767, maxY = -32768;
        float minZ = 32767, maxZ = -32768;

        const uint32_t durationMs = 15000;
        const uint32_t startTime = millis();

        while (millis() - startTime < durationMs)
        {
            readRawData();

            if (rawX < minX) minX = rawX;
            if (rawX > maxX) maxX = rawX;
            if (rawY < minY) minY = rawY;
            if (rawY > maxY) maxY = rawY;
            if (rawZ < minZ) minZ = rawZ;
            if (rawZ > maxZ) maxZ = rawZ;

            delay(20);
        }

        calibration.offsetX = (minX + maxX) / 2.0f;
        calibration.offsetY = (minY + maxY) / 2.0f;
        calibration.offsetZ = (minZ + maxZ) / 2.0f;

        Serial.print("QMC5883L: калибровка завершена, offset=");
        Serial.print(calibration.offsetX); Serial.print(",");
        Serial.print(calibration.offsetY); Serial.print(",");
        Serial.println(calibration.offsetZ);
    }

    const char* getSensorType() const override
    {
        return "QMC5883L (GY-273)";
    }

    void printStatus() const override
    {
        Serial.print("QMC5883L: available=");
        Serial.print(available ? "YES" : "NO");
        Serial.print(" mag(uT)="); Serial.print(magData.magX, 1);
        Serial.print(","); Serial.print(magData.magY, 1);
        Serial.print(","); Serial.print(magData.magZ, 1);
        Serial.print(" heading="); Serial.println(magData.headingDegrees, 1);
    }


private:

    static constexpr uint8_t REG_DATA_X_LSB = 0x00;  // далее X,Y,Z по 2 байта LE
    static constexpr uint8_t REG_STATUS     = 0x06;  // бит0 = данные готовы (не используется здесь)
    static constexpr uint8_t REG_CONTROL1   = 0x09;  // MODE/ODR/RNG/OSR
    static constexpr uint8_t REG_SET_RESET  = 0x0B;

    // Диапазон ±8 Гаусс: 3000 LSB/Гаусс = 30 LSB/µT (датащит QMC5883L).
    static constexpr float SENSITIVITY_LSB_PER_UT = 30.0f;

    II2CBus& i2c;
    uint8_t i2cAddress;
    bool available;

    int16_t rawX, rawY, rawZ;

    struct
    {
        float offsetX, offsetY, offsetZ;
    } calibration;

    MagData magData;

    bool checkConnection()
    {
        i2c.beginTransmission(i2cAddress);
        return (i2c.endTransmission() == 0);
    }

    void initialize()
    {
        writeRegister(REG_SET_RESET, 0x01);
        // MODE=continuous(01), ODR=200Hz(11), RNG=8G(01), OSR=512(00) -> 0x1D.
        writeRegister(REG_CONTROL1, 0x1D);
    }

    void readRawData()
    {
        i2c.beginTransmission(i2cAddress);
        i2c.write(REG_DATA_X_LSB);
        i2c.endTransmission(false);

        i2c.requestFrom(i2cAddress, (uint8_t)6);

        const uint8_t xl = i2c.read(), xh = i2c.read();
        const uint8_t yl = i2c.read(), yh = i2c.read();
        const uint8_t zl = i2c.read(), zh = i2c.read();

        rawX = (int16_t)((xh << 8) | xl);
        rawY = (int16_t)((yh << 8) | yl);
        rawZ = (int16_t)((zh << 8) | zl);
    }

    void applyCalibration()
    {
        magData.magX = (rawX - calibration.offsetX) / SENSITIVITY_LSB_PER_UT;
        magData.magY = (rawY - calibration.offsetY) / SENSITIVITY_LSB_PER_UT;
        magData.magZ = (rawZ - calibration.offsetZ) / SENSITIVITY_LSB_PER_UT;
    }

    // 2D-курс без компенсации наклона (roll/pitch) — датчик
    // предполагается установленным горизонтально. Тilt-компенсация
    // потребовала бы данных IMU и сюда намеренно не добавлена.
    void calculateHeading()
    {
        float heading = atan2(magData.magY, magData.magX) * 57.2958f;
        if (heading < 0) heading += 360.0f;

        magData.headingDegrees = heading;
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        i2c.beginTransmission(i2cAddress);
        i2c.write(reg);
        i2c.write(value);
        i2c.endTransmission();
    }
};
