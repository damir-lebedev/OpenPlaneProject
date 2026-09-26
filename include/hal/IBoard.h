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
// (main.cpp, FlightOutputs, IBusReceiver, драйверы датчиков).
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
    constexpr uint8_t COUNT         = 5;
}

class IBoard
{
public:
    virtual ~IBoard() = default;

    // Разовая инициализация платы: шины I2C/SPI. UART-порты
    // открывают их владельцы (IBusReceiver, GPS-драйвер) со своей
    // скоростью, PWM-выходы настраивает FlightOutputs::begin().
    virtual void begin() = 0;

    virtual II2CBus& i2c() = 0;          // шина датчиков
    virtual ISpiBus& spi() = 0;

    // Вторая шина I2C — только для экрана, чтобы отрисовка не
    // задерживала опрос датчиков. nullptr, если на плате её нет.
    virtual II2CBus* displayI2c() = 0;

    virtual IUartPort& rcUart() = 0;   // существующий iBUS UART
    virtual IUartPort& gpsUart() = 0;  // новый UART для GPS

    virtual IServoOutput& servo(uint8_t channel) = 0;  // см. ServoChannel::*
};
