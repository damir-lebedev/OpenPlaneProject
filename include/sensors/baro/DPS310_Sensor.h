#pragma once
#include <Arduino.h>

#include "sensors/baro/SPL06_Sensor.h"

// ============================================================
// DPS310 (Infineon) — барометр на готовых полётниках (Matek H743,
// F405-WSE, MicoAir743), I2C 0x76/0x77 или SPI
//
// Регистры, ID (0x10), коэффициенты и формулы — те же, что у SPL06-001
// (Goertek сделал SPL06 по образу DPS310), поэтому драйвер — SPL06_Sensor.
// Отличие одно: у части DPS310 термодатчик после включения врёт
// (десятки градусов), а с ним и поправка давления. Лечит последовательность
// записей в служебные регистры — её публикует Infineon («Temperature
// measurement issue» в README драйвера DPS310), так же делает ArduPilot
// (AP_Baro_DPS280). Пишется после сброса, до настройки измерений.
//
// По ID DPS310 от SPL06 не отличить — чип выбирается явно:
// SENSOR_BARO_DPS310 в SensorSelection.h.
// ============================================================

class DPS310_Sensor : public SPL06_Sensor
{
public:

    explicit DPS310_Sensor(IRegisterDevice& registerDevice, const char* sensorName = "DPS310")
        : SPL06_Sensor(registerDevice, sensorName)
    {
    }


protected:

    bool afterReset(IRegisterDevice& dev) override
    {
        return dev.writeRegister(0x0E, 0xA5) &&
               dev.writeRegister(0x0F, 0x96) &&
               dev.writeRegister(0x62, 0x02) &&
               dev.writeRegister(0x0E, 0x00) &&
               dev.writeRegister(0x0F, 0x00);
    }
};
