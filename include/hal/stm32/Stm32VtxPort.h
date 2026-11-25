#pragma once
#include <Arduino.h>
#include <new>

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
    explicit Stm32VtxPort(int16_t pin) { attach(pin); }

    Stm32VtxPort(const Stm32VtxPort&) = delete;
    Stm32VtxPort& operator=(const Stm32VtxPort&) = delete;

    // P63: провод на другом UART (настройка serial_vtx_uart), до open().
    void rebind(int16_t pin)
    {
        if (serial)
        {
            if (begun) serial->end();
            serial->~Uart();
            serial = nullptr;
        }
        begun = false;
        attach(pin);
    }

    void open(uint32_t baud, bool twoStopBits) override
    {
        if (!serial) return;
        if (begun) serial->end();
        serial->begin(baud, twoStopBits ? SERIAL_8N2 : SERIAL_8N1);
        serial->enableHalfDuplexRx();
        begun = true;
    }

    void send(const uint8_t* data, uint8_t length) override
    {
        if (!begun) return;
        serial->write(data, length);
        serial->enableHalfDuplexRx();   // ждёт, пока уйдёт последний байт
    }

    int read() override { return begun ? serial->read() : -1; }

    bool isConnected() const { return serial != nullptr; }

private:
    void attach(int16_t pin)
    {
        if (pin >= 0) serial = new (storage) Uart(static_cast<pin_size_t>(pin));   // один пин — полудуплекс
    }

    alignas(Uart) uint8_t storage[sizeof(Uart)];
    Uart* serial = nullptr;
    bool begun = false;
};
