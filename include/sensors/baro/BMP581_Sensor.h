#pragma once
#include <Arduino.h>

#include "hal/RegisterDevice.h"
#include "sensors/baro/BarometerBase.h"

// ============================================================
// BMP581 / BMP580 / BMP585 (Bosch) — барометр/термометр, I2C или SPI
//
// Самый малошумящий барометр проекта (~0.1 Па с оверсэмплингом),
// поэтому он и стоит в самодельной трубке Пито (см.
// sensors/airspeed/PitotDualBaroAirspeed.h). Может быть и основным
// барометром.
//
// I2C: 0x46 (SDO = GND) или 0x47 (SDO = VDD). Если BMP581 два (трубка
// + фюзеляж) — у них должны быть разные адреса. SPI —
// BMP581_Sensor::spiDevice() (режим 0, без лишних байт); после
// сброса по SPI нужно одно фиктивное чтение — оно делается всегда.
//
// Регистры и последовательность сверены с официальным API Bosch
// (boschsensortec/BMP5_SensorAPI: bmp5_defs.h, bmp5.c):
//   0x01 CHIP_ID = 0x50 (BMP580/581) или 0x51 (BMP585)
//   0x15 INT_SOURCE [0] drdy_data_reg_en
//   0x1D-0x1F температура int24 LE, /65536 -> °C
//   0x20-0x22 давление uint24 LE, /64 -> Па
//   0x27 INT_STATUS [0] drdy, [4] сброс/включение завершены
//   0x28 STATUS [1] NVM_RDY, [2] NVM_ERR
//   0x30 DSP_CONFIG [3] shdw_sel_iir_t, [5] shdw_sel_iir_p — читать
//        отфильтрованные IIR значения
//   0x31 DSP_IIR [2:0] set_iir_t, [5:3] set_iir_p
//   0x36 OSR_CONFIG [2:0] osr_t, [5:3] osr_p, [6] press_en
//   0x37 ODR_CONFIG [1:0] pwr_mode, [6:2] odr, [7] deep_dis
//   0x38 OSR_EFF [7] odr_is_valid — хватает ли времени на измерение
//   0x7E CMD = 0xB6 — мягкий сброс (2 мс)
//
// Режим: давление ×16, температура ×2, ODR 50 Гц, IIR давления с
// коэффициентом 3; normal mode (deep standby выключен). Настройки
// меняются только в standby — после сброса чип в нём и находится.
// ============================================================

class BMP581_Sensor : public BarometerBase
{
public:

    static constexpr uint8_t DEFAULT_ADDRESS   = 0x46;  // SDO = GND
    static constexpr uint8_t ALTERNATE_ADDRESS = 0x47;  // SDO = VDD

    static SpiRegisterDevice spiDevice(ISpiBus& bus, uint8_t chipSelectPin);

    explicit BMP581_Sensor(IRegisterDevice& registerDevice, const char* sensorName = "BMP581");

    bool begin() override;


protected:

    bool isNewSampleReady(bool& ready) override;

    bool readSample(float& pressurePa, float& temperatureC) override;


private:

    static constexpr uint8_t REG_CHIP_ID    = 0x01;
    static constexpr uint8_t REG_INT_SOURCE = 0x15;
    static constexpr uint8_t REG_TEMP_XLSB  = 0x1D;
    static constexpr uint8_t REG_INT_STATUS = 0x27;
    static constexpr uint8_t REG_STATUS     = 0x28;
    static constexpr uint8_t REG_DSP_CONFIG = 0x30;
    static constexpr uint8_t REG_DSP_IIR    = 0x31;
    static constexpr uint8_t REG_OSR_CONFIG = 0x36;
    static constexpr uint8_t REG_ODR_CONFIG = 0x37;
    static constexpr uint8_t REG_OSR_EFF    = 0x38;
    static constexpr uint8_t REG_CMD        = 0x7E;

    static constexpr int CHIP_ID_581 = 0x50;
    static constexpr int CHIP_ID_585 = 0x51;

    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
    static constexpr uint8_t INT_DRDY = 0x01;
    static constexpr uint8_t INT_POR_COMPLETE = 0x10;
    static constexpr uint8_t INT_SOURCE_DRDY = 0x01;
    static constexpr uint8_t STATUS_NVM_RDY = 0x02;
    static constexpr uint8_t STATUS_NVM_ERR = 0x04;
    static constexpr uint8_t DSP_SHDW_IIR_T = 0x08;
    static constexpr uint8_t DSP_SHDW_IIR_P = 0x20;
    static constexpr uint8_t IIR_COEF_3 = 0x02;
    static constexpr uint8_t OSR_PRESS_EN = 0x40;
    static constexpr uint8_t OSR_16X = 0x04;
    static constexpr uint8_t OSR_2X = 0x01;
    static constexpr uint8_t ODR_DEEP_DISABLE = 0x80;
    static constexpr uint8_t ODR_50HZ = 0x0F;
    static constexpr uint8_t PWR_STANDBY = 0x00;
    static constexpr uint8_t PWR_NORMAL = 0x01;
    static constexpr uint8_t OSR_EFF_ODR_VALID = 0x80;

    static constexpr uint32_t RESET_TIME_MS = 2;

    // Флаг опрашивается раз в 5 мс (ODR 50 Гц = 20 мс).
    static constexpr uint32_t POLL_PERIOD_US = 5000;
    static constexpr uint32_t FALLBACK_PERIOD_US = 40000;

    IRegisterDevice& device;
    uint32_t lastSampleUs = 0;

    void printName() const;

    bool fail();
};
