#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/imu/ImuSensorBase.h"

// ============================================================
// ICM-45686 (TDK InvenSense) — гироскоп + акселерометр
//
// Замена модуля LSM6DSV: I2C, адрес 0x68 (AP_AD0 = GND) или 0x69; по
// SPI — ICM45686_Sensor::spiDevice() (режим 0, без лишних байт).
//
// Регистры сверены с драйвером TDK (motion.arduino.ICM45686,
// inv_imu_regmap_le.h / inv_imu_defs.h), драйвером Zephyr и ArduPilot
// (AP_InertialSensor_Invensensev3). Это НЕ раскладка ICM-42688:
//   0x00..0x0D  accel XYZ, gyro XYZ, температура — int16 little-endian
//               (порядок байт по умолчанию, SREG_CTRL не трогаем)
//   0x10 PWR_MGMT0      [1:0] ACCEL_MODE, [3:2] GYRO_MODE (3 = Low Noise)
//   0x1B ACCEL_CONFIG0  [3:0] ACCEL_ODR, [6:4] ACCEL_UI_FS_SEL (1 = ±16g)
//   0x1C GYRO_CONFIG0   [3:0] GYRO_ODR,  [7:4] GYRO_UI_FS_SEL (1 = ±2000 °/с)
//   0x72 WHO_AM_I = 0xE9
//   0x7C..0x7E  IREG_ADDR_15_8 / IREG_ADDR_7_0 / IREG_DATA — косвенный
//               доступ к внутренним регистрам (фильтры)
//   0x7F REG_MISC2      [1] SOFT_RST
//
// Фильтры UI (как ~50 Гц у ICM-42688 в проекте) — только косвенно:
// IPREG_SYS1_REG_172 (0xA4AC) [2:0] GYRO_UI_LPFBW_SEL и
// IPREG_SYS2_REG_131 (0xA583) [2:0] ACCEL_UI_LPFBW_SEL, код 4 =
// ODR/32 = 50 Гц при ODR 1.6 кГц. Запись — пакетом "адрес + данные" в
// IREG_ADDR (так пишет драйвер TDK), чтение-изменение-запись, чтобы не
// трогать соседние биты; между косвенными записями — пауза 1 мс.
//
// Калибровка, поворот осей, знаки и фильтр ориентации — в ImuSensorBase.
//
// Не проверен на железе: при подключении — WHO_AM_I в логе, знаки
// наклоном (нос вверх -> P > 0, правое крыло вниз -> R > 0).
// ============================================================

class ICM45686_Sensor : public ImuSensorBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS   = 0x68;
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x69;

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin);

    explicit ICM45686_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readSample(RawImuSample& s) override;

    float accelLsbPerG() const override { return 2048.0f; }   // ±16g
    float gyroLsbPerDps() const override { return 16.4f; }    // ±2000 °/с

    float temperatureC(int16_t raw) const override;


private:

    static constexpr uint8_t REG_ACCEL_DATA_X1_UI = 0x00;
    static constexpr uint8_t REG_PWR_MGMT0        = 0x10;
    static constexpr uint8_t REG_ACCEL_CONFIG0    = 0x1B;
    static constexpr uint8_t REG_GYRO_CONFIG0     = 0x1C;
    static constexpr uint8_t REG_WHO_AM_I         = 0x72;
    static constexpr uint8_t REG_IREG_ADDR_15_8   = 0x7C;
    static constexpr uint8_t REG_IREG_DATA        = 0x7E;
    static constexpr uint8_t REG_MISC2            = 0x7F;

    static constexpr int WHO_AM_I_VALUE = 0xE9;

    static constexpr uint8_t MISC2_SOFT_RST = 0x02;
    static constexpr uint8_t PWR_ACCEL_LN   = 0x03;
    static constexpr uint8_t PWR_GYRO_LN    = 0x03 << 2;
    static constexpr uint8_t ODR_1600HZ     = 0x05;
    static constexpr uint8_t FS_16G         = 0x01;
    static constexpr uint8_t FS_2000DPS     = 0x01;

    static constexpr uint16_t IPREG_GYRO_LPF  = 0xA4AC;  // IPREG_SYS1_REG_172
    static constexpr uint16_t IPREG_ACCEL_LPF = 0xA583;  // IPREG_SYS2_REG_131
    static constexpr uint8_t LPF_MASK         = 0x07;
    static constexpr uint8_t LPF_ODR_DIV_32   = 0x04;

    static constexpr uint32_t GYRO_STARTUP_MS = 45;

    IRegisterDevice& device;

    bool fail();

    // Чтение-изменение-запись внутреннего регистра через IREG.
    bool setIndirectBits(uint16_t address, uint8_t mask, uint8_t value);

    static int16_t le16(const uint8_t* p);
};
