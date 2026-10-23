#pragma once
#include <Arduino.h>

#include "copter/Vtx.h"

// ============================================================
// Провод управления видеопередатчиком (SmartAudio, Tramp) на STM32: один
// провод — TX выбранного UART в полудуплексе (как площадка TX4 «VTX» у
// полётников под Betaflight). STM32duino сам выключает приём на время
// передачи; после отправки — дождаться, пока байты уйдут, и включить
// приём обратно (enableHalfDuplexRx), иначе ответ передатчика потеряется.
// ============================================================

class Stm32VtxPort : public copter::VtxPort
{
public:
    // pin — TX UART; −1 — провода нет (порт молчит).
    explicit Stm32VtxPort(int16_t pin)
        : serial(static_cast<pin_size_t>(pin < 0 ? 0 : pin)), connected(pin >= 0)
    {
    }

    void open(uint32_t baud, bool twoStopBits) override
    {
        if (!connected) return;
        serial.end();
        serial.begin(baud, twoStopBits ? SERIAL_8N2 : SERIAL_8N1);
        serial.enableHalfDuplexRx();
    }

    void send(const uint8_t* data, uint8_t length) override
    {
        if (!connected) return;
        serial.write(data, length);
        serial.enableHalfDuplexRx();   // ждёт, пока уйдёт последний байт
    }

    int read() override { return connected ? serial.read() : -1; }

    bool isConnected() const { return connected; }

private:
    Uart serial;   // один пин — полудуплекс
    bool connected;
};
