#pragma once
#include <Arduino.h>

// ============================================================
// BME280 (барометр/термометр)
//
// I2C, адрес 0x76 или 0x77. Как и MPU6050_Sensor, это своя
// минимальная реализация через II2CBus (см. hal/II2CBus.h), а не
// обёртка над Adafruit_BME280 (её нет в platformio.ini).
//
// ВАЖНО: readCalibration()/calculateAltitude() ниже — грубая
// аппроксимация, а не настоящая формула компенсации BME280 из
// датащита (там 26 калибровочных коэффициентов на чип). Значения
// давления/высоты будут не точными в абсолютных числах, но
// разница (climb rate) для ALT_HOLD достаточно стабильна.
// Точная компенсация по датащиту реализована для BMP388
// (см. BMP388_Sensor.h) — используйте его, если нужна абсолютная
// точность, BME280 остаётся как более простая I2C-альтернатива.
// ============================================================

#include "SensorInterface.h"
#include "../Config.h"
#include "../hal/II2CBus.h"

class BME280_Sensor : public BarometerSensor
{
public:

    explicit BME280_Sensor(II2CBus& bus, uint8_t address = 0x76)
        : i2c(bus),
          i2cAddress(address),
          available(false),
          seaLevelPressure(101325.0f)
    {
        memset(&baroData, 0, sizeof(baroData));
        calibrationAltitude = 0;
        previousAltitude = 0;
    }

    bool begin() override
    {
        delay(100);

        if (!checkConnection())
        {
            Serial.println("BME280: датчик не отвечает на I2C, барометр недоступен");
            available = false;
            return false;
        }

        if (!initialize())
        {
            Serial.println("BME280: ошибка инициализации");
            available = false;
            return false;
        }

        available = true;
        Serial.println("BME280: подключён");

        return true;
    }

    bool isAvailable() const override
    {
        return available;
    }

    void update() override
    {
        if (!available) return;

        readSensorData();
        calculateAltitude();

        baroData.timestamp = micros();
    }

    const BarometerData& getBarometerData() const override
    {
        return baroData;
    }

    // Вызывать на земле перед полётом — берёт среднее из 20
    // отсчётов и запоминает его как нулевую высоту.
    void calibrateAltitude() override
    {
        Serial.println("BME280: калибровка высоты...");

        float sumAltitude = 0;

        for (int i = 0; i < 20; i++)
        {
            readSensorData();
            calculateAltitude();
            sumAltitude += baroData.altitude;
            delay(50);
        }

        calibrationAltitude = sumAltitude / 20.0f;

        Serial.print("BME280: калибровка завершена, база=");
        Serial.print(calibrationAltitude);
        Serial.println(" м");
    }

    void setSeaLevelPressure(float pressure) override
    {
        seaLevelPressure = pressure;
    }

    const char* getSensorType() const override
    {
        return "BME280";
    }

    void printStatus() const override
    {
        Serial.print("BME280: available=");
        Serial.print(available ? "YES" : "NO");
        Serial.print(" pressure="); Serial.print(baroData.pressure / 100.0f); Serial.print("hPa");
        Serial.print(" altitude="); Serial.print(baroData.altitude); Serial.print("m");
        Serial.print(" climb="); Serial.print(baroData.verticalSpeed, 2); Serial.print("m/s");
        Serial.print(" temp="); Serial.print(baroData.temperature); Serial.println("C");
    }


private:

    II2CBus& i2c;
    uint8_t i2cAddress;
    bool available;

    float seaLevelPressure;
    float calibrationAltitude;
    float previousAltitude;

    BarometerData baroData;

    // Заглушка под реальные калибровочные коэффициенты чипа (не заполняется).
    struct
    {
        uint16_t dig_T1;
        int16_t dig_T2, dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
        uint8_t dig_H1;
        int16_t dig_H2;
        uint8_t dig_H3;
    } calibration;

    bool checkConnection()
    {
        i2c.beginTransmission(i2cAddress);
        return (i2c.endTransmission() == 0);
    }

    bool initialize()
    {
        uint8_t chipId = readRegister(0xD0);
        if (chipId != 0x60)
        {
            Serial.print("BME280: неверный chip ID 0x");
            Serial.println(chipId, HEX);
            return false;
        }

        writeRegister(0xE0, 0xB6);  // soft reset
        delay(100);

        readCalibration();

        writeRegister(0xF5, 0x00);  // CONFIG: без IIR-фильтра
        writeRegister(0xF4, 0x37);  // CTRL_MEAS: normal mode, oversampling x1
        writeRegister(0xF2, 0x02);  // CTRL_HUM: oversampling x2

        return true;
    }

    void readSensorData()
    {
        i2c.beginTransmission(i2cAddress);
        i2c.write(0xF7);  // PRESS_MSB
        i2c.endTransmission(false);
        i2c.requestFrom(i2cAddress, (uint8_t)3);

        uint32_t adc_P = ((uint32_t)i2c.read() << 12) |
                         ((uint32_t)i2c.read() << 4) |
                         ((uint32_t)i2c.read() >> 4);

        i2c.beginTransmission(i2cAddress);
        i2c.write(0xFA);  // TEMP_MSB
        i2c.endTransmission(false);
        i2c.requestFrom(i2cAddress, (uint8_t)3);

        uint32_t adc_T = ((uint32_t)i2c.read() << 12) |
                         ((uint32_t)i2c.read() << 4) |
                         ((uint32_t)i2c.read() >> 4);

        // Аппроксимация вместо формулы компенсации из датащита (см. заголовок файла).
        baroData.temperature = 25.0f + ((int32_t)adc_T - 100000) / 100000.0f;
        baroData.pressure = 100000.0f + ((int32_t)adc_P - 100000) / 1000.0f;
    }

    // Барометрическая формула высоты: h = 44330 * (1 - (P/P0)^(1/5.255)).
    void calculateAltitude()
    {
        float ratio = seaLevelPressure / baroData.pressure;
        float altitude = 44330.0f * (1.0f - pow(ratio, 0.1903f));

        baroData.altitude = altitude - calibrationAltitude;

        static unsigned long lastTime = 0;
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        if (dt > 0 && dt < 1.0f)
        {
            baroData.verticalSpeed = (baroData.altitude - previousAltitude) / dt;
        }

        previousAltitude = baroData.altitude;
    }

    void readCalibration()
    {
        // Реальный чип отдаёт 26 байт калибровочных коэффициентов по 0x88;
        // здесь не читаются и не используются (см. заголовок файла).
    }

    uint8_t readRegister(uint8_t reg)
    {
        i2c.beginTransmission(i2cAddress);
        i2c.write(reg);
        i2c.endTransmission(false);

        i2c.requestFrom(i2cAddress, (uint8_t)1);
        return i2c.read();
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        i2c.beginTransmission(i2cAddress);
        i2c.write(reg);
        i2c.write(value);
        i2c.endTransmission();
    }
};
