#pragma once

// ============================================================
// Нативная замена HardwareSerial (UART) из Arduino core ESP32 2.0.x.
//
// Принятые байты тест кладёт через pushRx(), переданные забирает
// через tx()/takeTx(). Каждый порт регистрируется по номеру UART
// (fake::uart(n)), поэтому тест может "подключить провод" к порту,
// который создан внутри Esp32Board, не зная про его поля.
// ============================================================

#include <cstdint>
#include <cstdio>
#include <string>

#include "Stream.h"

#define SERIAL_8N1 0x800001c

class HardwareSerial;

namespace fake
{
    constexpr uint8_t UART_COUNT = 3;

    struct UartRegistry
    {
        HardwareSerial* port[UART_COUNT] = {};
        bool echoSerial0 = false;   // дублировать вывод UART0 (Serial) в stdout
    };

    inline UartRegistry& uarts()
    {
        static UartRegistry registry;
        return registry;
    }

    inline HardwareSerial* uart(uint8_t number)
    {
        return number < UART_COUNT ? uarts().port[number] : nullptr;
    }

    inline void setSerialEcho(bool echo) { uarts().echoSerial0 = echo; }
}

class HardwareSerial : public Stream
{
public:
    explicit HardwareSerial(int uartNumber)
        : number(uartNumber)
    {
        if (number >= 0 && number < fake::UART_COUNT) fake::uarts().port[number] = this;
    }

    ~HardwareSerial() override
    {
        if (number >= 0 && number < fake::UART_COUNT && fake::uarts().port[number] == this)
        {
            fake::uarts().port[number] = nullptr;
        }
    }

    HardwareSerial(const HardwareSerial&) = delete;
    HardwareSerial& operator=(const HardwareSerial&) = delete;

    void begin(unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1,
               bool invert = false, unsigned long timeoutMs = 20000UL, uint8_t rxfifoFullThreshold = 112)
    {
        (void)invert;
        (void)timeoutMs;
        (void)rxfifoFullThreshold;
        baudRate = baud;
        frameConfig = config;
        rx = rxPin;
        tx = txPin;
        started = true;
        beginCalls++;
    }

    void end() { started = false; }

    // Как в Arduino core: после begin() размер буфера менять нельзя.
    size_t setTxBufferSize(size_t size)
    {
        if (started) return 0;
        txBufferSize = size;
        return size;
    }

    int available() override { return static_cast<int>(rxData.size() - rxPosition); }

    int read() override
    {
        if (rxPosition >= rxData.size()) return -1;
        return static_cast<uint8_t>(rxData[rxPosition++]);
    }

    int peek() override
    {
        return rxPosition < rxData.size() ? static_cast<uint8_t>(rxData[rxPosition]) : -1;
    }

    size_t write(uint8_t c) override
    {
        txData.push_back(static_cast<char>(c));
        if (number == 0 && fake::uarts().echoSerial0) fputc(c, stdout);
        return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) override
    {
        for (size_t i = 0; i < size; ++i) write(buffer[i]);
        return size;
    }

    using Print::write;

    explicit operator bool() const { return true; }

    // --- управление из тестов ---

    void pushRx(const uint8_t* data, size_t size)
    {
        compactRx();
        rxData.append(reinterpret_cast<const char*>(data), size);
    }

    void pushRx(const std::string& data) { pushRx(reinterpret_cast<const uint8_t*>(data.data()), data.size()); }

    const std::string& txBytes() const { return txData; }
    std::string takeTx()
    {
        std::string out;
        out.swap(txData);
        return out;
    }
    void clearTx() { txData.clear(); }

    int uartNumber() const { return number; }
    unsigned long baud() const { return baudRate; }
    uint32_t config() const { return frameConfig; }
    int8_t rxPin() const { return rx; }
    int8_t txPin() const { return tx; }
    bool isStarted() const { return started; }
    unsigned beginCount() const { return beginCalls; }
    size_t txBuffer() const { return txBufferSize; }

    void resetFake()
    {
        rxData.clear();
        rxPosition = 0;
        txData.clear();
        baudRate = 0;
        frameConfig = 0;
        rx = tx = -1;
        started = false;
        beginCalls = 0;
        txBufferSize = 0;
    }

private:
    int number;
    std::string rxData;
    size_t rxPosition = 0;
    std::string txData;
    unsigned long baudRate = 0;
    uint32_t frameConfig = 0;
    int8_t rx = -1;
    int8_t tx = -1;
    bool started = false;
    unsigned beginCalls = 0;
    size_t txBufferSize = 0;

    void compactRx()
    {
        if (rxPosition > 0 && rxPosition == rxData.size())
        {
            rxData.clear();
            rxPosition = 0;
        }
    }
};

// На ESP32-S3 с ARDUINO_USB_CDC_ON_BOOT=0 Serial — это UART0.
inline HardwareSerial Serial(0);
