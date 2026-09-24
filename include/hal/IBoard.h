#pragma once

#include "II2CBus.h"
#include "ISpiBus.h"
#include "IUartPort.h"
#include "IServoOutput.h"

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
// Сегодня единственная реализация — Esp32Board (hal/esp32/).
// Чтобы перейти на другой MCU (например STM32), нужно написать
// Stm32Board : public IBoard в hal/stm32/, реализующий те же
// методы поверх STM32-специфичных Wire/SPI/HardwareSerial/PWM —
// остальной код (сенсоры, автопилот, FlightController) не
// меняется вообще, потому что он написан против IBoard/II2CBus/
// ISpiBus/IUartPort/IServoOutput, а не против конкретных API.
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
    constexpr uint8_t COUNT         = 4;
}

class IBoard
{
public:
    virtual ~IBoard() = default;

    // Разовая инициализация платы (выделение PWM-таймеров и т.п.).
    // Шины (i2c/spi) и порты (rcUart/gpsUart) инициализируются
    // отдельно, каждый своим владельцем (см. Esp32Board.h) — так
    // же, как сегодня Wire.begin() дергают сами датчики.
    virtual void begin() = 0;

    virtual II2CBus& i2c() = 0;
    virtual ISpiBus& spi() = 0;

    virtual IUartPort& rcUart() = 0;   // существующий iBUS UART
    virtual IUartPort& gpsUart() = 0;  // новый UART для GPS

    virtual IServoOutput& servo(uint8_t channel) = 0;  // см. ServoChannel::*
};
