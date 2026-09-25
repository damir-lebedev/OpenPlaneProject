#pragma once

// ============================================================
// Нативная замена TwoWire (I2C) из Arduino core ESP32 2.0.x.
//
// К шине подключаются симулированные устройства fake::I2cDevice по
// 7-битному адресу (attach()). Транзакции ведут себя как у Wire:
//   beginTransmission / write... / endTransmission — запись; 0 = ACK,
//   2 = NACK на адрес (устройства нет), 3 = NACK на данные;
//   requestFrom(addr, n) — сколько байт реально пришло (0 — NACK).
//
// fake::RegisterMapDevice — типовой датчик "набор 8-битных
// регистров" с автоинкрементом адреса, журналом записей и
// внедрением сбоев (NACK, короткое чтение).
// ============================================================

#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "Stream.h"

namespace fake
{
    class I2cDevice
    {
    public:
        virtual ~I2cDevice() = default;

        // Транзакция записи; обычно data[0] — номер регистра. length == 0
        // — пустая транзакция (проверка присутствия). false — NACK.
        virtual bool onWrite(const uint8_t* data, size_t length) = 0;

        // Чтение count байт. Возвращает, сколько байт устройство
        // отдало (0 — NACK).
        virtual size_t onRead(uint8_t* out, size_t count) = 0;
    };

    class RegisterMapDevice : public I2cDevice
    {
    public:
        uint8_t regs[256] = {};
        uint8_t pointer = 0;

        bool present = true;         // false — не отвечает вовсе
        bool failWrites = false;     // NACK на запись данных
        int failReads = 0;           // столько следующих чтений — NACK (-1 — всегда)
        size_t shortRead = 0;        // > 0 — отдавать не больше стольких байт

        std::function<bool(uint8_t firstReg)> failReadIf;  // NACK на чтение с этого регистра

        std::vector<std::pair<uint8_t, uint8_t>> writes;   // журнал (регистр, значение)
        std::function<void(uint8_t reg, uint8_t value)> onRegisterWrite;
        std::function<void(uint8_t firstReg, size_t count)> beforeRead;

        uint32_t readTransactions = 0;

        bool onWrite(const uint8_t* data, size_t length) override
        {
            if (!present) return false;
            if (length == 0) return true;
            if (failWrites && length > 1) return false;
            pointer = data[0];
            for (size_t i = 1; i < length; ++i)
            {
                writeRegister(pointer++, data[i]);
            }
            return true;
        }

        size_t onRead(uint8_t* out, size_t count) override
        {
            if (!present) return 0;
            if (failReadIf && failReadIf(pointer)) return 0;
            if (failReads != 0)
            {
                if (failReads > 0) failReads--;
                return 0;
            }
            if (beforeRead) beforeRead(pointer, count);
            readTransactions++;
            const size_t n = shortRead > 0 && shortRead < count ? shortRead : count;
            for (size_t i = 0; i < n; ++i) out[i] = regs[pointer++];
            return n;
        }

        void writeRegister(uint8_t reg, uint8_t value)
        {
            regs[reg] = value;
            writes.emplace_back(reg, value);
            if (onRegisterWrite) onRegisterWrite(reg, value);
        }

        // Последнее значение, записанное прошивкой в регистр; -1 — не писали.
        int lastWrite(uint8_t reg) const
        {
            for (auto it = writes.rbegin(); it != writes.rend(); ++it)
            {
                if (it->first == reg) return it->second;
            }
            return -1;
        }

        void setBigEndian16(uint8_t reg, int16_t value)
        {
            regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
            regs[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>(value & 0xFF);
        }

        void setLittleEndian16(uint8_t reg, int16_t value)
        {
            regs[reg] = static_cast<uint8_t>(value & 0xFF);
            regs[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
        }
    };
}

class TwoWire : public Stream
{
public:
    explicit TwoWire(uint8_t busNumber) : number(busNumber) {}

    bool begin(int sdaPin = -1, int sclPin = -1, uint32_t frequencyHz = 0)
    {
        sda = sdaPin;
        scl = sclPin;
        frequency = frequencyHz;
        started = true;
        beginCalls++;
        return true;
    }

    bool setClock(uint32_t frequencyHz)
    {
        frequency = frequencyHz;
        return true;
    }

    void setTimeOut(uint16_t timeoutMs) { timeout = timeoutMs; }
    uint16_t getTimeOut() const { return timeout; }

    void beginTransmission(uint16_t address)
    {
        txAddress = static_cast<uint8_t>(address);
        txBuffer.clear();
        transmitting = true;
    }
    void beginTransmission(uint8_t address) { beginTransmission(static_cast<uint16_t>(address)); }
    void beginTransmission(int address) { beginTransmission(static_cast<uint16_t>(address)); }

    uint8_t endTransmission(bool sendStop = true)
    {
        (void)sendStop;
        transmitting = false;
        transactions++;
        fake::I2cDevice* device = find(txAddress);
        if (!device) return 2;
        if (!device->onWrite(txBuffer.data(), txBuffer.size())) return txBuffer.empty() ? 2 : 3;
        return 0;
    }

    uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop)
    {
        (void)sendStop;
        rxBuffer.assign(size, 0);
        rxPosition = 0;
        fake::I2cDevice* device = find(static_cast<uint8_t>(address));
        const size_t n = device ? device->onRead(rxBuffer.data(), size) : 0;
        rxBuffer.resize(n);
        return static_cast<uint8_t>(n);
    }
    uint8_t requestFrom(uint8_t address, uint8_t size) { return requestFrom(static_cast<uint16_t>(address), size, true); }
    uint8_t requestFrom(int address, int size)
    {
        return requestFrom(static_cast<uint16_t>(address), static_cast<uint8_t>(size), true);
    }

    size_t write(uint8_t data) override
    {
        if (!transmitting) return 0;
        txBuffer.push_back(data);
        return 1;
    }

    size_t write(const uint8_t* data, size_t length) override
    {
        for (size_t i = 0; i < length; ++i) write(data[i]);
        return transmitting ? length : 0;
    }

    using Print::write;

    int available() override { return static_cast<int>(rxBuffer.size() - rxPosition); }
    int read() override { return rxPosition < rxBuffer.size() ? rxBuffer[rxPosition++] : -1; }
    int peek() override { return rxPosition < rxBuffer.size() ? rxBuffer[rxPosition] : -1; }

    // --- управление из тестов ---

    void attach(uint8_t address, fake::I2cDevice* device) { devices[address] = device; }
    void detach(uint8_t address) { devices.erase(address); }

    int sdaPin() const { return sda; }
    int sclPin() const { return scl; }
    uint32_t clockHz() const { return frequency; }
    bool isStarted() const { return started; }
    unsigned beginCount() const { return beginCalls; }
    uint32_t transactionCount() const { return transactions; }

    void resetFake()
    {
        devices.clear();
        txBuffer.clear();
        rxBuffer.clear();
        rxPosition = 0;
        transmitting = false;
        sda = scl = -1;
        frequency = 0;
        timeout = 50;
        started = false;
        beginCalls = 0;
        transactions = 0;
    }

private:
    uint8_t number;
    std::map<uint8_t, fake::I2cDevice*> devices;
    uint8_t txAddress = 0;
    std::vector<uint8_t> txBuffer;
    std::vector<uint8_t> rxBuffer;
    size_t rxPosition = 0;
    bool transmitting = false;

    int sda = -1;
    int scl = -1;
    uint32_t frequency = 0;
    uint16_t timeout = 50;
    bool started = false;
    unsigned beginCalls = 0;
    uint32_t transactions = 0;

    fake::I2cDevice* find(uint8_t address)
    {
        auto it = devices.find(address);
        return it == devices.end() ? nullptr : it->second;
    }
};

inline TwoWire Wire(0);
inline TwoWire Wire1(1);
