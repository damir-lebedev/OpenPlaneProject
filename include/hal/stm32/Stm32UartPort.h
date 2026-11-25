#pragma once
#include <Arduino.h>
#include <new>

#include "hal/IUartPort.h"

// ============================================================
// Реализация IUartPort для STM32 (STM32duino 3.x) — обёртка над
// HardwareSerial. На железе (с датчиками) ещё не проверялась.
//
// В отличие от ESP32, пины UART на STM32 выбираются не в begin(),
// а при создании объекта Uart(rx, tx) — по ним ядро само находит
// периферию (USART3, UART7...) в таблицах PeripheralPins варианта.
// Поэтому порт сам создаёт Uart по паре ножек у себя внутри (буферы
// Uart — по 1.3 КБ, второй объект не держим), а в begin() задаёт только
// скорость и формат кадра. rebind() (P63, роли UART из настроек
// serial_*_uart) — создать его заново на других ножках, до begin().
// Обе ножки −1 — порта нет: begin() не удаётся, чтение пусто.
// SBUS: 8E2 (8 бит данных + чётность — ядро само берёт 9-битное
// слово) и инверсия RX средствами самого USART (RXINV) — внешний
// инвертор не нужен.
//
// Размер приёмного буфера — SERIAL_RX_BUFFER_SIZE (задан в
// platformio.ini, env stm32h743): кадр iBUS — 32 байта, NAV-PVT —
// 100, а стандартных 64 байт мало, если цикл один раз задержится.
// ============================================================

class Stm32UartPort : public IUartPort
{
public:
    Stm32UartPort(int16_t rx, int16_t tx) { attach(rx, tx); }

    Stm32UartPort(const Stm32UartPort&) = delete;
    Stm32UartPort& operator=(const Stm32UartPort&) = delete;

    void rebind(int16_t rx, int16_t tx)
    {
        if (serial)
        {
            if (begun) serial->end();
            serial->~Uart();
            serial = nullptr;
        }
        begun = false;
        attach(rx, tx);
    }

    bool isAttached() const { return serial != nullptr; }

    void begin(uint32_t baud) override
    {
        if (!serial) return;
        serial->begin(baud, SERIAL_8N1);
        begun = true;
    }

    bool begin(uint32_t baud, UartFormat format) override
    {
        if (!serial) return false;
        if (format == UartFormat::SBUS_8E2_INVERTED)
        {
            serial->setRxInvert();   // только до begin()
            serial->begin(baud, SERIAL_8E2);
        }
        else
        {
            serial->begin(baud, SERIAL_8N1);
        }
        begun = true;
        return true;
    }

    int available() override { return begun ? serial->available() : 0; }
    int read() override { return begun ? serial->read() : -1; }
    size_t write(uint8_t value) override { return begun ? serial->write(value) : 0; }
    size_t write(const uint8_t* buffer, size_t size) override { return begun ? serial->write(buffer, size) : 0; }
    int availableForWrite() override { return begun ? serial->availableForWrite() : 0; }


private:
    void attach(int16_t rx, int16_t tx)
    {
        if (rx < 0 && tx < 0) return;
        serial = new (storage) Uart(static_cast<pin_size_t>(rx), static_cast<pin_size_t>(tx));
    }

    alignas(Uart) uint8_t storage[sizeof(Uart)];
    Uart* serial = nullptr;
    bool begun = false;
};
