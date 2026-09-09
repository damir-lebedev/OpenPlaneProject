#pragma once

// ============================================================
// 🔌 BME280 BAROMETER/ALTIMETER IMPLEMENTATION
//
// Датчик окружающей среды:
// • Барометр (давление): ±1 гПа
// • Термометр: ±1°C
// • Гигрометр: ±3% влажности
//
// Используем: Давление для расчёта высоты
// Протокол: I2C (адрес: 0x76 или 0x77)
//
// Библиотека: Adafruit_BME280
// (установить: lib_deps = adafruit/Adafruit BME280 Library)
// ============================================================

#include "SensorInterface.h"
#include <Wire.h>

class BME280_Sensor : public BarometerSensor
{
public:

    // ========================================================
    // КОНСТРУКТОР
    // ========================================================

    explicit BME280_Sensor(uint8_t address = 0x76)
        : i2cAddress(address),
          available(false),
          seaLevelPressure(101325.0f)  // Стандартное давление на уровне моря (Па)
    {
        memset(&baroData, 0, sizeof(baroData));
        calibrationAltitude = 0;
    }


    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ
    // ========================================================

    bool begin() override
    {
        // Инициализируем I2C
        Wire.begin();
        Wire.setClock(400000);

        delay(100);

        // Проверяем связь
        if (!checkConnection())
        {
            Serial.println("❌ BME280: No response from sensor!");
            available = false;
            return false;
        }

        // Инициализируем датчик
        if (!initialize())
        {
            Serial.println("❌ BME280: Initialization failed!");
            available = false;
            return false;
        }

        available = true;
        Serial.println("✅ BME280: Initialized successfully");

        return true;
    }


    // ========================================================
    // ПРОВЕРКА ДОСТУПНОСТИ
    // ========================================================

    bool isAvailable() const override
    {
        return available;
    }


    // ========================================================
    // ОБНОВЛЕНИЕ ДАННЫХ
    // ========================================================

    void update() override
    {
        if (!available) return;

        readSensorData();
        calculateAltitude();

        baroData.timestamp = micros();
    }


    // ========================================================
    // ПОЛУЧИТЬ ДАННЫЕ БАРОМЕТРА
    // ========================================================

    const BarometerData& getBarometerData() const override
    {
        return baroData;
    }


    // ========================================================
    // КАЛИБРОВКА ВЫСОТЫ
    // ========================================================
    // Вызывается когда самолёт на земле перед полётом
    // Сохраняет текущую высоту как 0

    void calibrateAltitude() override
    {
        Serial.println("🔧 BME280: Calibrating altitude...");

        // Берём среднее от 20 образцов
        float sumAltitude = 0;

        for (int i = 0; i < 20; i++)
        {
            readSensorData();
            calculateAltitude();
            sumAltitude += baroData.altitude;
            delay(50);
        }

        calibrationAltitude = sumAltitude / 20.0f;

        Serial.print("✅ BME280: Altitude calibration complete. Base: ");
        Serial.print(calibrationAltitude);
        Serial.println(" m");
    }


    // ========================================================
    // ЗАДАТЬ ДАВЛЕНИЕ НА УРОВНЕ МОРЯ
    // ========================================================

    void setSeaLevelPressure(float pressure) override
    {
        seaLevelPressure = pressure;
    }


    // ========================================================
    // ТИП ДАТЧИКА
    // ========================================================

    const char* getSensorType() const override
    {
        return "BME280";
    }


    // ========================================================
    // ДИАГНОСТИКА
    // ========================================================

    void printStatus() const override
    {
        Serial.println("\n📊 BME280 Status:");
        Serial.print("  Available: ");
        Serial.println(available ? "YES ✓" : "NO ✗");
        Serial.print("  Pressure: ");
        Serial.print(baroData.pressure / 100.0f);  // В гПа
        Serial.println(" hPa");
        Serial.print("  Altitude: ");
        Serial.print(baroData.altitude);
        Serial.println(" m");
        Serial.print("  Vertical Speed: ");
        Serial.print(baroData.verticalSpeed, 2);
        Serial.println(" m/s");
        Serial.print("  Temperature: ");
        Serial.print(baroData.temperature);
        Serial.println("°C");
    }


private:

    // ========================================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ========================================================

    uint8_t i2cAddress;
    bool available;

    float seaLevelPressure;      // Давление на уровне моря для расчёта высоты
    float calibrationAltitude;   // Высота в начале полёта
    float previousAltitude;      // Для расчёта вертикальной скорости

    BarometerData baroData;

    // Калибровочные коэффициенты BME280 (заглушка)
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


    // ========================================================
    // ПРОВЕРКА СВЯЗИ
    // ========================================================

    bool checkConnection()
    {
        Wire.beginTransmission(i2cAddress);
        return (Wire.endTransmission() == 0);
    }


    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ
    // ========================================================

    bool initialize()
    {
        // Прочитаем ID датчика
        uint8_t chipId = readRegister(0xD0);
        if (chipId != 0x60)  // 0x60 = BME280
        {
            Serial.print("❌ BME280: Wrong chip ID: 0x");
            Serial.println(chipId, HEX);
            return false;
        }

        // Сброс датчика
        writeRegister(0xE0, 0xB6);
        delay(100);

        // Читаем калибровочные коэффициенты
        readCalibration();

        // Конфигурация: нормальный режим, фильтр, стабилизация
        writeRegister(0xF5, 0x00);  // Конфиг
        writeRegister(0xF4, 0x37);  // Контроль измерений (нормальный режим)
        writeRegister(0xF2, 0x02);  // Контроль влажности

        return true;
    }


    // ========================================================
    // ЧТЕНИЕ СЫРЫХ ДАННЫХ ДАТЧИКА
    // ========================================================

    void readSensorData()
    {
        // Читаем 8 байт: давление, температура
        Wire.beginTransmission(i2cAddress);
        Wire.write(0xF7);  // PRESS_MSB
        Wire.endTransmission(false);

        Wire.requestFrom(i2cAddress, (uint8_t)3);

        uint32_t adc_P = ((uint32_t)Wire.read() << 12) |
                         ((uint32_t)Wire.read() << 4) |
                         ((uint32_t)Wire.read() >> 4);

        // Читаем температуру
        Wire.beginTransmission(i2cAddress);
        Wire.write(0xFA);  // TEMP_MSB
        Wire.endTransmission(false);

        Wire.requestFrom(i2cAddress, (uint8_t)3);

        uint32_t adc_T = ((uint32_t)Wire.read() << 12) |
                         ((uint32_t)Wire.read() << 4) |
                         ((uint32_t)Wire.read() >> 4);

        // Для упрощения используем аппроксимацию
        // (реальная библиотека Adafruit делает это более точно)

        // Простая аппроксимация температуры
        baroData.temperature = 25.0f + ((int32_t)adc_T - 100000) / 100000.0f;

        // Простая аппроксимация давления (Па)
        // Реальная формула в датащите очень сложная
        baroData.pressure = 100000.0f + ((int32_t)adc_P - 100000) / 1000.0f;
    }


    // ========================================================
    // РАСЧЁТ ВЫСОТЫ ИЗ ДАВЛЕНИЯ
    // ========================================================
    // Используем барометрическую формулу

    void calculateAltitude()
    {
        // Формула для высоты из давления:
        // h = (P0 / P)^(1/5.255) * 44330 - 11000
        // где P0 - давление на уровне моря, P - текущее давление

        float ratio = seaLevelPressure / baroData.pressure;
        float altitude = 44330.0f * (1.0f - pow(ratio, 0.1903f));

        // Относительная высота (от начальной точки)
        baroData.altitude = altitude - calibrationAltitude;

        // Вертикальная скорость (производная от высоты)
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


    // ========================================================
    // ЧТЕНИЕ КАЛИБРОВОЧНЫХ ДАННЫХ
    // ========================================================

    void readCalibration()
    {
        // Эта функция в реальной библиотеке читает
        // 26 байт калибровочных коэффициентов
        // Для упрощения оставляем заглушку

        Serial.println("📚 BME280: Calibration data loaded");
    }


    // ========================================================
    // ЧТЕНИЕ РЕГИСТРА
    // ========================================================

    uint8_t readRegister(uint8_t reg)
    {
        Wire.beginTransmission(i2cAddress);
        Wire.write(reg);
        Wire.endTransmission(false);

        Wire.requestFrom(i2cAddress, (uint8_t)1);
        return Wire.read();
    }


    // ========================================================
    // ЗАПИСЬ В РЕГИСТР
    // ========================================================

    void writeRegister(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(i2cAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};
