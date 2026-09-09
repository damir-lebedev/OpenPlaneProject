// ============================================================
// 🔌 MPU6050 GY-521 SENSOR IMPLEMENTATION
//
// Датчик: 6-осевой инерциальный блок
// • Гироскоп (3 оси): точность до 0.01°/сек
// • Акселерометр (3 оси): точность до 0.01g
// • Встроенный термометр
//
// Протокол: I2C (адрес: 0x68 или 0x69)
// Частота: до 1 kHz (читаем с ~200 Hz)
//
// Библиотека: MPU6050 от jrowberg
// (установить через PlatformIO: lib_deps = jrowberg/MPU6050)
// ============================================================

#pragma once
#include "SensorInterface.h"
#include <Wire.h>

// Заглушка для MPU6050 класса (может быть реальная библиотека)
class MPU6050_Sensor : public ImuSensor
{
public:

    // ========================================================
    // КОНСТРУКТОР
    // ========================================================
    // address: 0x68 (AD0=GND, по умолчанию) или 0x69 (AD0=VCC)
    // ========================================================

    explicit MPU6050_Sensor(uint8_t address = 0x68)
        : i2cAddress(address),
          available(false),
          calibrationDone(false)
    {
        // Инициализация структур данных нулями
        memset(&imuData, 0, sizeof(imuData));
        memset(&calibration, 0, sizeof(calibration));
    }


    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ ДАТЧИКА
    // ========================================================
    // Возвращает true если датчик успешно инициализирован

    bool begin() override
    {
        // Инициализируем I2C (SCL=GPIO22, SDA=GPIO21 для ESP32)
        Wire.begin();
        Wire.setClock(400000);  // 400 kHz

        delay(100);

        // Проверяем связь с датчиком
        if (!checkConnection())
        {
            Serial.println("❌ MPU6050: No response from sensor!");
            available = false;
            return false;
        }

        // Инициализируем MPU6050
        if (!initialize())
        {
            Serial.println("❌ MPU6050: Initialization failed!");
            available = false;
            return false;
        }

        available = true;
        Serial.println("✅ MPU6050: Initialized successfully");

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
    // Вызывается регулярно из loop() (~200 Hz)

    void update() override
    {
        if (!available) return;

        // Читаем сырые данные с датчика
        readRawData();

        // Применяем калибровку
        applyCalibration();

        // Считаем углы из акселерометра
        calculateAngles();

        // Обновляем временную метку
        imuData.timestamp = micros();
    }


    // ========================================================
    // ПОЛУЧИТЬ ДАННЫЕ IMU
    // ========================================================

    const ImuData& getImuData() const override
    {
        return imuData;
    }


    // ========================================================
    // КАЛИБРОВКА ДАТЧИКА
    // ========================================================
    // Должна вызваться когда самолёт неподвижен на земле
    // Будет выполняться ~2 секунды

    void calibrate() override
    {
        if (!available) return;

        Serial.println("🔧 MPU6050: Starting calibration...");

        // Берём 200 образцов для усреднения
        const int SAMPLE_COUNT = 200;
        float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
        float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;

        for (int i = 0; i < SAMPLE_COUNT; i++)
        {
            readRawData();

            sumGyroX += rawGx;
            sumGyroY += rawGy;
            sumGyroZ += rawGz;

            sumAccelX += rawAx;
            sumAccelY += rawAy;
            sumAccelZ += rawAz;

            delay(10);
        }

        // Сохраняем средние значения (смещение)
        calibration.gyroOffsetX = sumGyroX / SAMPLE_COUNT;
        calibration.gyroOffsetY = sumGyroY / SAMPLE_COUNT;
        calibration.gyroOffsetZ = sumGyroZ / SAMPLE_COUNT;

        calibration.accelOffsetX = sumAccelX / SAMPLE_COUNT;
        calibration.accelOffsetY = sumAccelY / SAMPLE_COUNT;
        calibration.accelOffsetZ = sumAccelZ / SAMPLE_COUNT - 16384;  // -1g

        calibrationDone = true;

        Serial.println("✅ MPU6050: Calibration complete");
        Serial.print("  Gyro offset: ");
        Serial.print(calibration.gyroOffsetX);
        Serial.print(", ");
        Serial.print(calibration.gyroOffsetY);
        Serial.print(", ");
        Serial.println(calibration.gyroOffsetZ);
    }


    // ========================================================
    // ЗАДАТЬ YAW (рысканье)
    // ========================================================
    // Используется для сброса направления в начале полёта

    void setYaw(float yawDegrees) override
    {
        imuData.yaw = yawDegrees;
    }


    // ========================================================
    // ТИП ДАТЧИКА
    // ========================================================

    const char* getSensorType() const override
    {
        return "MPU6050 GY-521";
    }


    // ========================================================
    // ДИАГНОСТИКА
    // ========================================================

    void printStatus() const override
    {
        Serial.println("\n📊 MPU6050 Status:");
        Serial.print("  Available: ");
        Serial.println(available ? "YES ✓" : "NO ✗");
        Serial.print("  Calibrated: ");
        Serial.println(calibrationDone ? "YES ✓" : "NO ✗");
        Serial.print("  Gyro (°/s): ");
        Serial.print(imuData.gyroX, 2);
        Serial.print(", ");
        Serial.print(imuData.gyroY, 2);
        Serial.print(", ");
        Serial.println(imuData.gyroZ, 2);
        Serial.print("  Accel (g): ");
        Serial.print(imuData.accelX, 2);
        Serial.print(", ");
        Serial.print(imuData.accelY, 2);
        Serial.print(", ");
        Serial.println(imuData.accelZ, 2);
        Serial.print("  Angles (°): ");
        Serial.print("Roll=");
        Serial.print(imuData.roll, 1);
        Serial.print(", Pitch=");
        Serial.print(imuData.pitch, 1);
        Serial.print(", Yaw=");
        Serial.println(imuData.yaw, 1);
    }


private:

    // ========================================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ========================================================

    uint8_t i2cAddress;
    bool available;
    bool calibrationDone;

    // Сырые данные с датчика
    int16_t rawAx, rawAy, rawAz;  // Акселерометр
    int16_t rawGx, rawGy, rawGz;  // Гироскоп
    int16_t rawTemp;              // Температура

    // Структура калибровки
    struct
    {
        float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
        float accelOffsetX, accelOffsetY, accelOffsetZ;
    } calibration;

    // Результирующие данные
    ImuData imuData;

    // Интегратор для рысканья (так как гироскоп Z не откалиброван стабильно)
    float yawIntegral = 0;


    // ========================================================
    // ПРОВЕРКА СВЯЗИ С ДАТЧИКОМ
    // ========================================================

    bool checkConnection()
    {
        Wire.beginTransmission(i2cAddress);
        return (Wire.endTransmission() == 0);
    }


    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ MPU6050
    // ========================================================

    bool initialize()
    {
        // Выход из режима sleep
        writeRegister(0x6B, 0x00);

        // Установка диапазона гироскопа: ±250°/сек (0x00)
        writeRegister(0x1B, 0x00);

        // Установка диапазона акселерометра: ±2g (0x00)
        writeRegister(0x1C, 0x00);

        // Установка частоты дискретизации: 1kHz
        writeRegister(0x19, 0x07);  // Делитель: 8 (1kHz / 8 = 125 Hz)

        // Включение компенсации температуры гироскопа
        writeRegister(0x1A, 0x05);

        return true;
    }


    // ========================================================
    // ЧТЕНИЕ СЫРЫХ ДАННЫХ
    // ========================================================

    void readRawData()
    {
        Wire.beginTransmission(i2cAddress);
        Wire.write(0x3B);  // Регистр ACCEL_XOUT_H
        Wire.endTransmission(false);

        Wire.requestFrom(i2cAddress, (uint8_t)14);

        // Читаем 14 байт:
        // 0-1: ACCEL_X
        // 2-3: ACCEL_Y
        // 4-5: ACCEL_Z
        // 6-7: TEMP
        // 8-9: GYRO_X
        // 10-11: GYRO_Y
        // 12-13: GYRO_Z

        rawAx = (Wire.read() << 8) | Wire.read();
        rawAy = (Wire.read() << 8) | Wire.read();
        rawAz = (Wire.read() << 8) | Wire.read();
        rawTemp = (Wire.read() << 8) | Wire.read();
        rawGx = (Wire.read() << 8) | Wire.read();
        rawGy = (Wire.read() << 8) | Wire.read();
        rawGz = (Wire.read() << 8) | Wire.read();
    }


    // ========================================================
    // ПРИМЕНЕНИЕ КАЛИБРОВКИ
    // ========================================================

    void applyCalibration()
    {
        // Преобразуем сырые значения в физические единицы

        // Акселерометр: сырое значение / 16384 = g
        imuData.accelX = (rawAx - calibration.accelOffsetX) / 16384.0f;
        imuData.accelY = (rawAy - calibration.accelOffsetY) / 16384.0f;
        imuData.accelZ = (rawAz - calibration.accelOffsetZ) / 16384.0f;

        // Гироскоп: сырое значение / 131 = °/сек
        imuData.gyroX = (rawGx - calibration.gyroOffsetX) / 131.0f;
        imuData.gyroY = (rawGy - calibration.gyroOffsetY) / 131.0f;
        imuData.gyroZ = (rawGz - calibration.gyroOffsetZ) / 131.0f;

        // Температура: сырое значение / 340 + 36.53 = °C
        imuData.temperature = (rawTemp / 340.0f) + 36.53f;
    }


    // ========================================================
    // РАСЧЁТ УГЛОВ ИЗ АКСЕЛЕРОМЕТРА
    // ========================================================
    // Complementary filter: акселерометр даёт абсолютную позицию,
    // гироскоп даёт скорость (производную).

    void calculateAngles()
    {
        // Угол крена из акселерометра
        float accelRoll = atan2(imuData.accelY, imuData.accelZ) * 57.2958f;

        // Угол тангажа из акселерометра
        float accelPitch = atan2(-imuData.accelX,
                                 sqrt(imuData.accelY * imuData.accelY +
                                      imuData.accelZ * imuData.accelZ)) * 57.2958f;

        // Complementary filter (70% гироскоп, 30% акселерометр)
        static unsigned long lastTime = 0;
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        if (dt > 0 && dt < 0.1f)  // Защита от больших прыжков
        {
            imuData.roll = imuData.roll * 0.7f + accelRoll * 0.3f +
                          imuData.gyroX * dt;

            imuData.pitch = imuData.pitch * 0.7f + accelPitch * 0.3f +
                           imuData.gyroY * dt;

            // Yaw интегрируем просто из гироскопа
            yawIntegral += imuData.gyroZ * dt;
            imuData.yaw = yawIntegral;
        }

        // Ограничиваем углы
        if (imuData.roll > 180) imuData.roll -= 360;
        if (imuData.roll < -180) imuData.roll += 360;

        if (imuData.pitch > 180) imuData.pitch -= 360;
        if (imuData.pitch < -180) imuData.pitch += 360;

        if (imuData.yaw > 180) imuData.yaw -= 360;
        if (imuData.yaw < -180) imuData.yaw += 360;
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
