#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/mag/MagnetometerBase.h"

// ============================================================
// QMC6309 (QST) — компас модулей "LSM6DSV + QMC6309" и
// "ICM-45686 + QMC6309"
//
// I2C, адрес 0x7C. Раскладка сверена с драйвером qmc6309 (crates.io,
// 0.4.0, регистровая карта по датащиту QST); она отличается и от
// QMC5883L, и от QMC5883P (другие адрес, chip ID и поля CTRL):
//   0x00      CHIP_ID = 0x90
//   0x01-0x06 X/Y/Z, int16 little-endian
//   0x09      STATUS: [0] DRDY, [1] OVFL, [3] NVM_RDY, [4] NVM_LOAD_DONE
//   0x0A      CTRL1: [1:0] MODE (1 = normal), [4:3] OSR1 (0 = ×8),
//                    [7:5] OSR2/LPF (4 = глубина 16)
//   0x0B      CTRL2: [1:0] SET/RESET (0 = оба), [3:2] RNG (2 = ±8 Гс),
//                    [6:4] ODR (4 = 200 Гц), [7] SOFT_RST
//
// После SOFT_RST (бит пишется 1, затем 0 вручную) чип перечитывает
// NVM — настраивать можно, когда в STATUS выставлены NVM_RDY и
// NVM_LOAD_DONE. Сначала CTRL2 (частота, диапазон), режим — последним.
//
// ±8 Гс -> 32768/8 = 4096 LSB/Гс = 40.96 LSB/мкТл.
//
// Калибровка, курс, опрос, ошибки — в MagnetometerBase.
// ============================================================

class QMC6309_Sensor : public MagnetometerBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS = 0x7C;

    explicit QMC6309_Sensor(IRegisterDevice& registerDevice);

    bool begin() override;


protected:

    bool readRaw(int16_t raw[3]) override;

    float lsbPerMicroTesla() const override { return 40.96f; }


private:

    static constexpr uint8_t REG_CHIP_ID    = 0x00;
    static constexpr uint8_t REG_DATA_X_LSB = 0x01;
    static constexpr uint8_t REG_STATUS     = 0x09;
    static constexpr uint8_t REG_CONTROL1   = 0x0A;
    static constexpr uint8_t REG_CONTROL2   = 0x0B;
    static constexpr int CHIP_ID_VALUE = 0x90;

    static constexpr uint8_t STATUS_NVM_READY = 0x18;  // NVM_RDY | NVM_LOAD_DONE
    static constexpr uint8_t CTRL2_SOFT_RST   = 0x80;

    static constexpr uint8_t MODE_NORMAL   = 0x01;
    static constexpr uint8_t OSR_8         = 0x00;
    static constexpr uint8_t LPF_DEPTH_16  = 0x04;
    static constexpr uint8_t SET_RESET_ON  = 0x00;
    static constexpr uint8_t RANGE_8G      = 0x02;
    static constexpr uint8_t ODR_200HZ     = 0x04;

    static constexpr uint8_t NVM_POLL_TRIES = 50;

    IRegisterDevice& device;

    bool waitNvmReady();
};
