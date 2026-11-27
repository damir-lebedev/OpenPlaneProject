#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/Bmi270Config.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// BMI270 (Bosch) — гироскоп + акселерометр, SPI (SpeedyBee F405 V3 и
// другие полётники под BF).
//
// Особенность чипа: после включения он «пустой» — пока в него не залита
// микропрограмма (файл конфигурации Bosch, 8 КБ, Bmi270Config.h), данные
// не идут. Порядок по датащиту (§4.5 «Quick start»):
//   1. фронт CS переводит чип из I2C в SPI — холостое чтение CHIP_ID;
//   2. мягкий сброс CMD=0xB6, снова холостое чтение;
//   3. CHIP_ID = 0x24;
//   4. PWR_CONF=0 (выкл. advanced power save), ждать ≥450 мкс;
//   5. INIT_CTRL=0, файл — кусками в INIT_DATA (адрес куска в словах —
//      INIT_ADDR_0/1), INIT_CTRL=1;
//   6. через ≥20 мс INTERNAL_STATUS.message = 1 («init ok»);
//   7. ODR/полосы/диапазоны, PWR_CTRL — включить акселерометр, гироскоп,
//      температуру.
// По SPI чип перед данными отдаёт один мусорный байт (dummy).
//
// Настройка: ±2000°/с и ±16g (как у остальных IMU), ODR 1600 Гц.
// Полоса: у коптера (Config::IMU_WIDE_BANDWIDTH) режим normal (~300 Гц у
// гироскопа), иначе OSR4 (~75 Гц). Данные LE с 0x0C: accel XYZ, gyro XYZ;
// температура 0x22 (1/512 °C, 0 = 23 °C, 0x8000 — нет данных).
//
// Не проверен на железе: при подключении — «CHIP_ID=0x24 -> BMI270» и
// «файл конфигурации загружен» в логе, знаки наклоном.
// ============================================================

class BMI270_Sensor : public ImuSensorBase
{
public:

    // До 10 МГц по датащиту; 8 МГц — с запасом. Один мусорный байт перед данными.
    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin)
    {
        return SpiRegisterDevice(bus, chipSelectPin, SPI_HZ, 1);
    }

    explicit BMI270_Sensor(IRegisterDevice& registerDevice)
        : ImuSensorBase("BMI270", "imu_bmi270"),
          device(registerDevice)
    {
    }

    bool begin() override
    {
        device.begin();
        delay(2);

        device.readRegister(REG_CHIP_ID);               // фронт CS: I2C -> SPI
        device.writeRegister(REG_CMD, CMD_SOFT_RESET);
        delay(RESET_MS);
        device.readRegister(REG_CHIP_ID);               // после сброса чип снова в I2C

        const int chipId = device.readRegister(REG_CHIP_ID);
        if (chipId != CHIP_ID_VALUE)
        {
            if (chipId < 0 || chipId == 0x00 || chipId == 0xFF)
                Serial.println("BMI270: датчик не отвечает, IMU недоступен");
            else
            {
                Serial.print("BMI270: неверный CHIP_ID 0x");
                Serial.println(chipId, HEX);
            }
            setAvailable(false);
            return false;
        }
        Serial.println("BMI270: CHIP_ID=0x24 -> BMI270");

        if (!uploadConfig())
        {
            setAvailable(false);
            return false;
        }

        bool ok = true;
        const bool wide = Config::IMU_WIDE_BANDWIDTH;
        ok &= device.writeRegister(REG_ACC_CONF, wide ? ACC_CONF_WIDE : ACC_CONF_NARROW);
        ok &= device.writeRegister(REG_ACC_RANGE, ACC_RANGE_16G);
        ok &= device.writeRegister(REG_GYR_CONF, wide ? (imu::hardwareLpfIndex(true) == 0 ? GYR_CONF_WIDE : GYR_CONF_3200)
                                                      : GYR_CONF_NARROW);
        ok &= device.writeRegister(REG_GYR_RANGE, GYR_RANGE_2000);
        ok &= device.writeRegister(REG_PWR_CTRL, PWR_CTRL_ACC_GYR_TEMP);
        ok &= device.writeRegister(REG_PWR_CONF, PWR_CONF_FIFO_WAKEUP);   // без энергосбережения
        if (!ok)
        {
            Serial.println("BMI270: ошибка записи регистров, IMU недоступен");
            setAvailable(false);
            return false;
        }

        // Гироскоп после включения запускается ~45 мс (датащит, start-up time):
        // до этого в регистрах нули — калибровка не должна их брать.
        delay(STARTUP_MS);

        Serial.println("BMI270: подключён");
        setAvailable(true);
        return true;
    }


    // gyro_hardware_lpf (sensors/imu/HardwareLpf.h): NORMAL, OPTION_1, OPTION_2,
    // EXPERIMENTAL — GYR_CONF: normal при ODR 1600 Гц,
    // иначе normal при ODR 3200 Гц (шире чип не умеет — остальные пункты так же).
    static constexpr imu::HardwareLpfSpec HW_LPF[imu::HARDWARE_LPF_COUNT] = { { 300, 800 }, { 750, 350 }, { 750, 350 }, { 750, 350 } };
    const imu::HardwareLpfSpec* hardwareLpfTable() const override { return HW_LPF; }

protected:

    bool readSample(RawImuSample& s) override
    {
        uint8_t b[DATA_BYTES];   // 0x0C…0x23: accel, gyro, …, температура
        if (!device.readRegisters(REG_DATA_START, b, sizeof(b))) return false;

        s.accelX = le16(b + 0);
        s.accelY = le16(b + 2);
        s.accelZ = le16(b + 4);
        s.gyroX  = le16(b + 6);
        s.gyroY  = le16(b + 8);
        s.gyroZ  = le16(b + 10);
        const int16_t t = le16(b + (REG_TEMPERATURE - REG_DATA_START));
        if (t != INT16_MIN) lastTemperature = t;   // 0x8000 — «нет данных», держим прежнюю
        s.temperature = lastTemperature;
        return true;
    }

    float accelLsbPerG() const override { return 2048.0f; }   // ±16g
    float gyroLsbPerDps() const override { return 16.4f; }    // ±2000°/с

    float temperatureC(int16_t raw) const override
    {
        return raw / 512.0f + 23.0f;   // датащит BMI270, TEMPERATURE
    }


private:

    bool uploadConfig()
    {
        bool ok = device.writeRegister(REG_PWR_CONF, 0x00);   // advanced power save выкл.
        delayMicroseconds(450);
        ok &= device.writeRegister(REG_INIT_CTRL, 0x00);

        for (uint16_t index = 0; index < bmi270::CONFIG_SIZE; index += CHUNK)
        {
            // Адрес куска — в 16-битных словах: биты 3:0 в INIT_ADDR_0, 11:4 в INIT_ADDR_1.
            const uint16_t word = index / 2;
            const uint8_t addr[2] = { static_cast<uint8_t>(word & 0x0F), static_cast<uint8_t>(word >> 4) };
            ok &= device.writeRegisters(REG_INIT_ADDR_0, addr, 2);
            ok &= device.writeRegisters(REG_INIT_DATA, bmi270::CONFIG_FILE + index, CHUNK);
        }
        ok &= device.writeRegister(REG_INIT_CTRL, 0x01);

        // Чип проверяет файл ~20 мс; ждём «init ok» до 150 мс.
        int status = -1;
        for (uint8_t i = 0; i < 15 && ok; ++i)
        {
            delay(10);
            status = device.readRegister(REG_INTERNAL_STATUS);
            if (status >= 0 && (status & 0x0F) == STATUS_INIT_OK) break;
        }
        if (!ok || status < 0 || (status & 0x0F) != STATUS_INIT_OK)
        {
            Serial.print("BMI270: файл конфигурации не принят (INTERNAL_STATUS=0x");
            Serial.print(status < 0 ? 0 : status, HEX);
            Serial.println("), IMU недоступен");
            return false;
        }
        Serial.println("BMI270: файл конфигурации загружен");
        return true;
    }

    static constexpr uint32_t SPI_HZ   = 8000000;
    static constexpr uint32_t RESET_MS = 10;     // датащит: 2 мс после сброса, с запасом
    static constexpr uint32_t STARTUP_MS = 50;   // запуск гироскопа ~45 мс
    static constexpr uint16_t CHUNK    = 128;    // байт за обмен (чётное, ≤ 255)
    static_assert(bmi270::CONFIG_SIZE % CHUNK == 0, "файл конфигурации — целыми кусками");

    static constexpr uint8_t REG_CHIP_ID         = 0x00;
    static constexpr uint8_t REG_DATA_START      = 0x0C;   // ACC_X_LSB
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
    static constexpr uint8_t REG_TEMPERATURE     = 0x22;
    static constexpr uint8_t DATA_BYTES          = REG_TEMPERATURE + 2 - REG_DATA_START;
    static constexpr uint8_t REG_ACC_CONF        = 0x40;
    static constexpr uint8_t REG_ACC_RANGE       = 0x41;
    static constexpr uint8_t REG_GYR_CONF        = 0x42;
    static constexpr uint8_t REG_GYR_RANGE       = 0x43;
    static constexpr uint8_t REG_INIT_CTRL       = 0x59;
    static constexpr uint8_t REG_INIT_ADDR_0     = 0x5B;
    static constexpr uint8_t REG_INIT_DATA       = 0x5E;
    static constexpr uint8_t REG_PWR_CONF        = 0x7C;
    static constexpr uint8_t REG_PWR_CTRL        = 0x7D;
    static constexpr uint8_t REG_CMD             = 0x7E;

    static constexpr int     CHIP_ID_VALUE       = 0x24;
    static constexpr uint8_t CMD_SOFT_RESET      = 0xB6;
    static constexpr uint8_t STATUS_INIT_OK      = 0x01;

    // ACC_CONF: bit7 filter_perf, 6:4 bwp (2 — normal, 0 — OSR4), 3:0 odr (0x0C — 1600 Гц).
    static constexpr uint8_t ACC_CONF_WIDE   = 0xAC;
    static constexpr uint8_t ACC_CONF_NARROW = 0x8C;
    // GYR_CONF: bit7 filter_perf, bit6 noise_perf, 5:4 bwp, 3:0 odr (0x0C — 1600 Гц).
    static constexpr uint8_t GYR_CONF_WIDE   = 0xEC;
    static constexpr uint8_t GYR_CONF_NARROW = 0xCC;
    static constexpr uint8_t GYR_CONF_3200   = 0xED;   // normal, 3200 Гц (gyro_hardware_lpf не NORMAL)
    static constexpr uint8_t ACC_RANGE_16G   = 0x03;
    static constexpr uint8_t GYR_RANGE_2000  = 0x00;
    static constexpr uint8_t PWR_CTRL_ACC_GYR_TEMP = 0x0E;
    static constexpr uint8_t PWR_CONF_FIFO_WAKEUP  = 0x02;

    IRegisterDevice& device;
    int16_t lastTemperature = 0;

    static int16_t le16(const uint8_t* p)
    {
        return static_cast<int16_t>(p[0] | (p[1] << 8));
    }
};
