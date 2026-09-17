#pragma once
#include <Arduino.h>

// ============================================================
// BMP388 (барометр/термометр, Bosch)
//
// SPI, свой CS-пин (см. Config::PIN_SPI_CS_BMP388). Как и
// BME280_Sensor, это собственная минимальная реализация без
// vendor-библиотек — но, в отличие от неё, здесь честная
// компенсация по формуле из датащита Bosch BMP388 (§9.3,
// floating-point вариант) с 21 байтом NVM-коэффициентов с
// регистра 0x31. Высота считается той же барометрической
// формулой, что и в BME280_Sensor.h — она не специфична для чипа.
// ============================================================

#include "SensorInterface.h"
#include "../hal/ISpiBus.h"

class BMP388_Sensor : public BarometerSensor
{
public:

    BMP388_Sensor(ISpiBus& bus, uint8_t chipSelectPin)
        : spi(bus),
          csPin(chipSelectPin),
          available(false),
          seaLevelPressure(101325.0f)
    {
        memset(&baroData, 0, sizeof(baroData));
        memset(&calib, 0, sizeof(calib));
        calibrationAltitude = 0;
        previousAltitude = 0;
        tLin = 0;
    }

    bool begin() override
    {
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);

        delay(100);

        const uint8_t chipId = readRegister(REG_CHIP_ID);
        if (chipId != 0x50)
        {
            Serial.print("BMP388: неверный chip ID 0x");
            Serial.println(chipId, HEX);
            available = false;
            return false;
        }

        writeRegister(REG_CMD, 0xB6);  // soft reset
        delay(10);

        readCalibration();

        // OSR: пресс x8 (011), темп x1 (000).
        writeRegister(REG_OSR, 0x03);
        // ODR: 50 Hz (см. датащит §4.3.19, odr_sel=0x02).
        writeRegister(REG_ODR, 0x02);
        // IIR-фильтр: коэффициент 1 (лёгкое сглаживание).
        writeRegister(REG_CONFIG, 0x02);
        // PWR_CTRL: press_en=1, temp_en=1, mode=normal(11).
        writeRegister(REG_PWR_CTRL, 0x33);

        available = true;
        Serial.println("BMP388: подключён");

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
        Serial.println("BMP388: калибровка высоты...");

        float sumAltitude = 0;

        for (int i = 0; i < 20; i++)
        {
            readSensorData();
            calculateAltitude();
            sumAltitude += baroData.altitude;
            delay(50);
        }

        calibrationAltitude = sumAltitude / 20.0f;

        Serial.print("BMP388: калибровка завершена, база=");
        Serial.print(calibrationAltitude);
        Serial.println(" м");
    }

    void setSeaLevelPressure(float pressure) override
    {
        seaLevelPressure = pressure;
    }

    const char* getSensorType() const override
    {
        return "BMP388";
    }

    void printStatus() const override
    {
        Serial.print("BMP388: available=");
        Serial.print(available ? "YES" : "NO");
        Serial.print(" pressure="); Serial.print(baroData.pressure / 100.0f); Serial.print("hPa");
        Serial.print(" altitude="); Serial.print(baroData.altitude); Serial.print("m");
        Serial.print(" climb="); Serial.print(baroData.verticalSpeed, 2); Serial.print("m/s");
        Serial.print(" temp="); Serial.print(baroData.temperature); Serial.println("C");
    }


private:

    static constexpr uint8_t REG_CHIP_ID  = 0x00;
    static constexpr uint8_t REG_DATA_0   = 0x04;  // burst: press(3) + temp(3), XLSB..MSB
    static constexpr uint8_t REG_PWR_CTRL = 0x1B;
    static constexpr uint8_t REG_OSR      = 0x1C;
    static constexpr uint8_t REG_ODR      = 0x1D;
    static constexpr uint8_t REG_CONFIG   = 0x1F;
    static constexpr uint8_t REG_CMD      = 0x7E;
    static constexpr uint8_t REG_NVM_PAR  = 0x31;  // 21 байт калибровки

    ISpiBus& spi;
    uint8_t csPin;
    bool available;

    float seaLevelPressure;
    float calibrationAltitude;
    float previousAltitude;

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

    double tLin;  // компенсированная температура, нужна и для давления (порядок вычисления важен)

    void readCalibration()
    {
        uint8_t raw[21];

        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);
        spi.transfer(REG_NVM_PAR | 0x80);
        for (uint8_t i = 0; i < 21; ++i)
        {
            raw[i] = spi.transfer(0x00);
        }
        spi.endTransaction();
        digitalWrite(csPin, HIGH);

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
    }

    void readSensorData()
    {
        uint8_t raw[6];

        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);
        spi.transfer(REG_DATA_0 | 0x80);
        for (uint8_t i = 0; i < 6; ++i)
        {
            raw[i] = spi.transfer(0x00);
        }
        spi.endTransaction();
        digitalWrite(csPin, HIGH);

        const uint32_t uncompPress = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
        const uint32_t uncompTemp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

        baroData.temperature = (float)compensateTemperature(uncompTemp);  // обновляет tLin
        baroData.pressure = (float)compensatePressure(uncompPress);
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

    // Барометрическая формула высоты — та же, что в BME280_Sensor.h,
    // от конкретного чипа не зависит.
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

    uint8_t readRegister(uint8_t reg)
    {
        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);

        spi.transfer(reg | 0x80);
        uint8_t value = spi.transfer(0x00);

        spi.endTransaction();
        digitalWrite(csPin, HIGH);

        return value;
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        digitalWrite(csPin, LOW);
        spi.beginTransaction(8000000, 0);

        spi.transfer(reg & 0x7F);
        spi.transfer(value);

        spi.endTransaction();
        digitalWrite(csPin, HIGH);
    }
};
