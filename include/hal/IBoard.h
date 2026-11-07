#pragma once
#include <stdint.h>

// Типы, которые возвращают методы IBoard, — часть его API.
#include "hal/II2CBus.h"       // IWYU pragma: export
#include "hal/IServoOutput.h"  // IWYU pragma: export
#include "hal/ISpiBus.h"       // IWYU pragma: export
#include "hal/IUartPort.h"     // IWYU pragma: export

// ============================================================
// 🧠 АБСТРАКЦИЯ "МОЗГА" (MCU)
//
// Единственная точка входа в железо для всего остального кода
// (main.cpp, FlightOutputs, RcReceiver, драйверы датчиков).
// Ничего выше этого интерфейса не должно включать <Wire.h>,
// <SPI.h>, HardwareSerial или LEDC напрямую — только
// IBoard и его under-интерфейсы (II2CBus/ISpiBus/IUartPort/
// IServoOutput).
//
// Реализации: Esp32Board (hal/esp32/) — основная, и Stm32Board
// (hal/stm32/) — заготовка под STM32H743, на железе пока не
// проверенная. Обе реализуют одни и те же методы поверх своих
// Wire/SPI/UART/PWM — остальной код (сенсоры, автопилот,
// FlightController) от MCU не зависит, потому что он написан против
// IBoard/II2CBus/ISpiBus/IUartPort/IServoOutput, а не против
// конкретных API.
// ============================================================

// Индекс серво-канала для IBoard::servo(channel). Плоский список,
// а не 4 именованных метода — чтобы добавление нового выхода
// в будущем не меняло сам интерфейс IBoard.
namespace ServoChannel
{
    constexpr uint8_t AILERON_LEFT  = 0;
    constexpr uint8_t AILERON_RIGHT = 1;
    constexpr uint8_t ELEVATOR      = 2;
    constexpr uint8_t ESC           = 3;
    constexpr uint8_t RUDDER        = 4;
    constexpr uint8_t AUX1          = 5;   // сброс груза (Feature::PAYLOAD_DROP)
    constexpr uint8_t AUX2          = 6;   // камера (Knob::CAMERA_TILT, Feature::CAMERA_STAB)
    constexpr uint8_t COUNT         = 7;
}

class IBoard
{
public:
    virtual ~IBoard() = default;

    // Разовая инициализация платы: шины I2C/SPI. UART-порты
    // открывают их владельцы (RcReceiver, GPS-драйвер) со своей
    // скоростью, PWM-выходы настраивает FlightOutputs::begin().
    virtual void begin() = 0;

    virtual II2CBus& i2c() = 0;          // шина датчиков
    virtual ISpiBus& spi() = 0;

    // Вторая шина I2C (Config::PIN_I2C2_*): экран и/или датчики — что
    // на неё повесили. nullptr, если на плате её нет.
    virtual II2CBus* secondI2c() = 0;

    // Шина внешнего разъёма. У самодельных плат — та же шина датчиков;
    // у готовых полётников (Matek H743) барометр распаян на плате на
    // одной шине, а разъём для внешних — на другой.
    virtual II2CBus& externalI2c() { return i2c(); }

    // Все шины I2C платы по номеру: 0 — основная (i2c()), 1 — вторая,
    // 2 — внешний разъём, если он отдельный, 3 — шина за IMU (sensor hub
    // LSM6DSV, setAuxI2c()); nullptr — такой шины нет. Датчики ищутся на
    // всех (I2cRegisterDevice), так что какой модуль куда повесить — всё равно.
    static constexpr uint8_t I2C_BUS_COUNT = 4;

    // Шина за IMU: её даёт драйвер IMU (ImuSensor::auxI2c()), SelectedSensors
    // добавляет её сюда до того, как создаются компас и барометр.
    void setAuxI2c(II2CBus* bus) { auxBus = bus; }

    II2CBus* i2cBus(uint8_t index)
    {
        switch (index)
        {
            case 0: return &i2c();
            case 1: return secondI2c();
            case 2: return &externalI2c() != &i2c() ? &externalI2c() : nullptr;
            case 3: return auxBus;
            default: return nullptr;
        }
    }

    // Все шины, начиная с preferred, — для I2cRegisterDevice: датчик
    // ищется сначала там, где он обычно висит, потом на остальных.
    I2cBusList i2cBuses(II2CBus& preferred)
    {
        I2cBusList list;
        list.add(&preferred);
        for (uint8_t i = 0; i < I2C_BUS_COUNT; ++i) list.add(i2cBus(i));
        return list;
    }

    virtual IUartPort& rcUart() = 0;   // приёмник пульта: iBUS, SBUS или CRSF
    virtual IUartPort& gpsUart() = 0;  // новый UART для GPS

    // UART радиомодема (MAVLink, Config::PIN_TELEM_RX/TX). nullptr —
    // на плате свободного UART нет (ESP32: там дашборд по Wi-Fi).
    virtual IUartPort* telemetryUart() { return nullptr; }

    virtual IServoOutput& servo(uint8_t channel) = 0;  // см. ServoChannel::*

    // Пищалка (Config::PIN_BUZZER через транзистор). Платам без неё
    // реализовывать не нужно.
    virtual void setBuzzer(bool on) { (void)on; }


private:

    II2CBus* auxBus = nullptr;
};
