#pragma once
#include <Arduino.h>

// ============================================================
// BMP388 (барометр/термометр, Bosch) — I2C-вариант
//
// Тот же чип и та же компенсация по датащиту Bosch BMP388 (§9.3),
// что и в BMP388_Sensor.h (SPI), но регистры читаются через
// II2CBus вместо ISpiBus — для модулей, где SDO/CSB разведены на
// I2C-режим, либо когда физически удобнее посадить датчик на ту
// же шину, что и IMU, а не заводить отдельную SPI-шину.
//
// Адрес: 0x76 (SDO=GND, по умолчанию) или 0x77 (SDO=VDD) — как и
// у BME280. CSB модуля должен быть подтянут к VCC: если при
// включении он на земле, чип уходит в SPI и по I2C не отвечает.
//
// Чип меряет с ODR 50 Гц, а update() вызывается ~500 раз в секунду —
// поэтому данные читаются только когда готов новый отсчёт (флаг
// drdy_press в STATUS). Раньше данные читались каждый цикл, и
// вертикальная скорость считалась по одному и тому же отсчёту
// (0 м/с) вперемешку со скачками (Δh / 2 мс) — шум в десятки м/с.
// ============================================================

#include "SensorInterface.h"
#include "../hal/II2CBus.h"

class BMP388_I2C_Sensor : public BarometerSensor
{
public:

    explicit BMP388_I2C_Sensor(II2CBus& bus, uint8_t address = 0x76)
        : i2c(bus),
          i2cAddress(address)
    {
        memset(&baroData, 0, sizeof(baroData));
        memset(&calib, 0, sizeof(calib));
    }

    bool begin() override
    {
        const int chipId = i2c.readRegister(i2cAddress, REG_CHIP_ID);
        if (chipId != 0x50)
        {
            Serial.print("BMP388: ");
            if (chipId < 0)
            {
                Serial.println("не отвечает на I2C (проверьте CSB -> VCC, SDO -> GND)");
            }
            else
            {
                Serial.print("неверный chip ID 0x");
                Serial.println(chipId, HEX);
            }
            available = false;
            return false;
        }

        i2c.writeRegister(i2cAddress, REG_CMD, 0xB6);  // soft reset
        delay(10);

        if (!readCalibration())
        {
            Serial.println("BMP388: не удалось прочитать калибровку NVM");
            available = false;
            return false;
        }

        const bool ok =
            // OSR: пресс x8 (011), темп x1 (000) — ~20 мс на измерение, влезает в 50 Гц.
            i2c.writeRegister(i2cAddress, REG_OSR, 0x03) &&
            // ODR: 50 Hz (см. датащит §4.3.19, odr_sel=0x02).
            i2c.writeRegister(i2cAddress, REG_ODR, 0x02) &&
            // IIR-фильтр: коэффициент 3 — гасит шум и порывы, задержка ~0.1 с.
            i2c.writeRegister(i2cAddress, REG_CONFIG, 0x04) &&
            // PWR_CTRL: press_en=1, temp_en=1, mode=normal(11).
            i2c.writeRegister(i2cAddress, REG_PWR_CTRL, 0x33);

        if (!ok)
        {
            Serial.println("BMP388: ошибка записи регистров");
            available = false;
            return false;
        }

        available = true;
        Serial.println("BMP388: подключён (I2C)");

        return true;
    }

    bool isAvailable() const override
    {
        return available && consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
    }

    void update() override
    {
        if (!available) return;

        const uint32_t now = micros();
        if (now - lastPollUs < POLL_PERIOD_US) return;
        lastPollUs = now;

        const int status = i2c.readRegister(i2cAddress, REG_STATUS);
        if (status < 0)
        {
            onReadError();
            return;
        }

        if (!(status & STATUS_DRDY_PRESS)) return;  // нового отсчёта ещё нет

        if (!readSensorData())
        {
            onReadError();
            return;
        }

        consecutiveErrors = 0;
        updateAltitude(now);
        baroData.timestamp = now;
    }

    const BarometerData& getBarometerData() const override
    {
        return baroData;
    }

    // Вызывать на земле перед полётом — берёт среднее из 20
    // отсчётов и запоминает его как нулевую высоту.
    void calibrateAltitude() override
    {
        if (!available) return;

        Serial.println("BMP388: калибровка высоты...");

        float sumAltitude = 0;
        int samples = 0;

        for (int i = 0; i < 20; i++)
        {
            delay(50);
            if (readSensorData())
            {
                sumAltitude += absoluteAltitude();
                samples++;
            }
        }

        if (samples == 0)
        {
            Serial.println("BMP388: калибровка не удалась — датчик не отвечает");
            return;
        }

        calibrationAltitude = sumAltitude / samples;

        baroData.altitude = 0;
        baroData.verticalSpeed = 0;
        previousAltitude = 0;
        lastSampleUs = 0;  // следующий отсчёт не считает скорость от старой базы

        Serial.print("BMP388: калибровка завершена, база=");
        Serial.print(calibrationAltitude);
        Serial.println(" м над уровнем моря (по стандартной атмосфере)");
    }

    void setSeaLevelPressure(float pressure) override
    {
        seaLevelPressure = pressure;
    }

    const char* getSensorType() const override
    {
        return "BMP388 (I2C)";
    }

    void printStatus() const override
    {
        Serial.print("BMP388: available=");
        Serial.print(isAvailable() ? "YES" : "NO");
        Serial.print(" errors="); Serial.print(errorCount);
        Serial.print(" pressure="); Serial.print(baroData.pressure / 100.0f); Serial.print("hPa");
        Serial.print(" altitude="); Serial.print(baroData.altitude); Serial.print("m");
        Serial.print(" climb="); Serial.print(baroData.verticalSpeed, 2); Serial.print("m/s");
        Serial.print(" temp="); Serial.print(baroData.temperature); Serial.println("C");
    }


private:

    static constexpr uint8_t REG_CHIP_ID  = 0x00;
    static constexpr uint8_t REG_STATUS   = 0x03;
    static constexpr uint8_t REG_DATA_0   = 0x04;  // burst: press(3) + temp(3), XLSB..MSB
    static constexpr uint8_t REG_PWR_CTRL = 0x1B;
    static constexpr uint8_t REG_OSR      = 0x1C;
    static constexpr uint8_t REG_ODR      = 0x1D;
    static constexpr uint8_t REG_CONFIG   = 0x1F;
    static constexpr uint8_t REG_CMD      = 0x7E;
    static constexpr uint8_t REG_NVM_PAR  = 0x31;  // 21 байт калибровки

    static constexpr uint8_t STATUS_DRDY_PRESS = 0x20;

    // Опрос флага готовности — в 4 раза чаще ODR, чтобы задержка
    // между измерением и чтением была не больше 5 мс.
    static constexpr uint32_t POLL_PERIOD_US = 5000;

    // ~0.5 с подряд без ответа -> барометр недоступен.
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 100;

    // Постоянная времени ФНЧ вертикальной скорости: производная от
    // высоты шумит (0.1 м шума / 0.02 с = 5 м/с), сглаживание ~0.5 с
    // оставляет полезный сигнал для ALT_HOLD.
    static constexpr float CLIMB_FILTER_TAU_S = 0.5f;

    II2CBus& i2c;
    uint8_t i2cAddress;
    bool available = false;

    float seaLevelPressure = 101325.0f;
    float calibrationAltitude = 0;
    float previousAltitude = 0;

    uint32_t lastPollUs = 0;
    uint32_t lastSampleUs = 0;
    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;

    BarometerData baroData;

    // "Квантованные" коэффициенты по формуле Bosch (датащит §9.3) —
    // уже с применёнными масштабами (степени двойки из датащита),
    // используются в compensateTemperature()/compensatePressure()
    // напрямую, без пересчёта каждый раз.
    struct
    {
        double par_t1, par_t2, par_t3;
        double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    } calib;

    double tLin = 0;  // компенсированная температура, нужна и для давления (порядок вычисления важен)

    void onReadError()
    {
        if (consecutiveErrors < MAX_CONSECUTIVE_ERRORS) consecutiveErrors++;
        errorCount++;
    }

    bool readCalibration()
    {
        uint8_t raw[21];
        if (!i2c.readRegisters(i2cAddress, REG_NVM_PAR, raw, sizeof(raw))) return false;

        const uint16_t t1 = (uint16_t)(raw[0] | (raw[1] << 8));
        const uint16_t t2 = (uint16_t)(raw[2] | (raw[3] << 8));
        const int8_t   t3 = (int8_t)raw[4];
        const int16_t  p1 = (int16_t)(raw[5] | (raw[6] << 8));
        const int16_t  p2 = (int16_t)(raw[7] | (raw[8] << 8));
        const int8_t   p3 = (int8_t)raw[9];
        const int8_t   p4 = (int8_t)raw[10];
        const uint16_t p5 = (uint16_t)(raw[11] | (raw[12] << 8));
        const uint16_t p6 = (uint16_t)(raw[13] | (raw[14] << 8));
        const int8_t   p7 = (int8_t)raw[15];
        const int8_t   p8 = (int8_t)raw[16];
        const int16_t  p9 = (int16_t)(raw[17] | (raw[18] << 8));
        const int8_t   p10 = (int8_t)raw[19];
        const int8_t   p11 = (int8_t)raw[20];

        // Масштабы — степени двойки из датащита Bosch BMP388 §9.1.
        calib.par_t1 = (double)t1 * 256.0;                       // / 2^-8
        calib.par_t2 = (double)t2 / 1073741824.0;                 // / 2^30
        calib.par_t3 = (double)t3 / 281474976710656.0;            // / 2^48

        calib.par_p1 = ((double)p1 - 16384.0) / 1048576.0;        // / 2^20
        calib.par_p2 = ((double)p2 - 16384.0) / 536870912.0;      // / 2^29
        calib.par_p3 = (double)p3 / 4294967296.0;                 // / 2^32
        calib.par_p4 = (double)p4 / 137438953472.0;               // / 2^37
        calib.par_p5 = (double)p5 * 8.0;                          // / 2^-3
        calib.par_p6 = (double)p6 / 64.0;                         // / 2^6
        calib.par_p7 = (double)p7 / 256.0;                        // / 2^8
        calib.par_p8 = (double)p8 / 32768.0;                      // / 2^15
        calib.par_p9 = (double)p9 / 281474976710656.0;            // / 2^48
        calib.par_p10 = (double)p10 / 281474976710656.0;          // / 2^48
        calib.par_p11 = (double)p11 / 36893488147419103232.0;     // / 2^65

        return true;
    }

    bool readSensorData()
    {
        uint8_t raw[6];
        if (!i2c.readRegisters(i2cAddress, REG_DATA_0, raw, sizeof(raw))) return false;

        const uint32_t uncompPress = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
        const uint32_t uncompTemp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

        baroData.temperature = (float)compensateTemperature(uncompTemp);  // обновляет tLin
        baroData.pressure = (float)compensatePressure(uncompPress);
        return true;
    }

    // Формула Bosch BMP388, датащит §9.3. tLin — промежуточный член,
    // обязателен для compensatePressure(), поэтому температура
    // всегда считается первой (см. readSensorData()).
    double compensateTemperature(uint32_t uncompTemp)
    {
        const double partialData1 = (double)uncompTemp - calib.par_t1;
        const double partialData2 = partialData1 * calib.par_t2;

        tLin = partialData2 + (partialData1 * partialData1) * calib.par_t3;
        return tLin;
    }

    double compensatePressure(uint32_t uncompPress)
    {
        const double p = (double)uncompPress;

        double partialData1 = calib.par_p6 * tLin;
        double partialData2 = calib.par_p7 * tLin * tLin;
        double partialData3 = calib.par_p8 * tLin * tLin * tLin;
        const double partialOut1 = calib.par_p5 + partialData1 + partialData2 + partialData3;

        partialData1 = calib.par_p2 * tLin;
        partialData2 = calib.par_p3 * tLin * tLin;
        partialData3 = calib.par_p4 * tLin * tLin * tLin;
        const double partialOut2 = p * (calib.par_p1 + partialData1 + partialData2 + partialData3);

        partialData1 = p * p;
        partialData2 = calib.par_p9 + calib.par_p10 * tLin;
        partialData3 = partialData1 * partialData2;
        const double partialData4 = partialData3 + (p * p * p) * calib.par_p11;

        return partialOut1 + partialOut2 + partialData4;
    }

    // Барометрическая формула (международная стандартная атмосфера):
    // h = 44330 * (1 - (P/P0)^(1/5.255)). Раньше в скобках стояло
    // P0/P — высота выходила с обратным знаком (подъём -> "минус"),
    // и ALT_HOLD, поднявшись выше цели, добавлял бы газ.
    float absoluteAltitude() const
    {
        return 44330.0f * (1.0f - powf(baroData.pressure / seaLevelPressure, 0.1903f));
    }

    void updateAltitude(uint32_t now)
    {
        baroData.altitude = absoluteAltitude() - calibrationAltitude;

        const float dt = (now - lastSampleUs) / 1000000.0f;

        if (lastSampleUs != 0 && dt > 0 && dt < 0.5f)
        {
            const float rawClimb = (baroData.altitude - previousAltitude) / dt;
            const float alpha = dt / (CLIMB_FILTER_TAU_S + dt);
            baroData.verticalSpeed += alpha * (rawClimb - baroData.verticalSpeed);
        }

        previousAltitude = baroData.altitude;
        lastSampleUs = now;
    }
};
