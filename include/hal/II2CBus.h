#pragma once
#include <Arduino.h>

// ============================================================
// 🔌 АБСТРАКЦИЯ ШИНЫ I2C
//
// По форме — тонкая обёртка над Wire (begin/beginTransmission/
// write/endTransmission/requestFrom/read). Датчики работают с ней
// через I2cRegisterDevice (hal/RegisterDevice.h), OLED — напрямую;
// никто из них не знает, что за интерфейсом — Wire/Wire1 ESP32 или
// будущая реализация под другой MCU.
//
// begin()/setClock() без аргументов: пины и частота фиксируются
// один раз, когда конкретная реализация (например Esp32I2CBus)
// создаётся платой (IBoard) — так шина инициализируется ровно
// один раз, даже если её использует несколько датчиков.
// ============================================================

class II2CBus
{
public:
    virtual ~II2CBus() = default;

    virtual void begin() = 0;
    virtual void setClock(uint32_t hz) = 0;

    virtual void beginTransmission(uint8_t address) = 0;
    virtual size_t write(uint8_t data) = 0;
    virtual size_t write(const uint8_t* data, size_t length) = 0;
    virtual uint8_t endTransmission(bool sendStop = true) = 0;

    virtual uint8_t requestFrom(uint8_t address, uint8_t quantity) = 0;
    virtual int available() = 0;
    virtual int read() = 0;

    // --------------------------------------------------------
    // Общие помощники поверх примитивов выше — регистровый доступ,
    // одинаковый у всех I2C-датчиков проекта. Возвращают false, если
    // устройство не ответило (NACK) или пришло меньше байт, чем
    // запрошено; буфер при этом не трогается, так что драйвер может
    // оставить прошлые данные вместо мусора (0xFF от read() на
    // пустом буфере).
    // --------------------------------------------------------

    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
    {
        beginTransmission(address);
        write(reg);
        write(value);
        return endTransmission() == 0;
    }

    bool readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t count)
    {
        beginTransmission(address);
        write(reg);
        if (endTransmission(false) != 0) return false;

        if (requestFrom(address, count) != count) return false;

        for (uint8_t i = 0; i < count; ++i)
        {
            buffer[i] = static_cast<uint8_t>(read());
        }
        return true;
    }

    // -1, если устройство не ответило.
    int readRegister(uint8_t address, uint8_t reg)
    {
        uint8_t value;
        return readRegisters(address, reg, &value, 1) ? value : -1;
    }

    bool probe(uint8_t address)
    {
        beginTransmission(address);
        return endTransmission() == 0;
    }
};
