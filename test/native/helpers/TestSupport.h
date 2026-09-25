#pragma once

// ============================================================
// Общие помощники нативных тестов (test/native/test_*).
//
//   resetWorld()     — вернуть весь симулированный мир (время, GPIO,
//                      LEDC, задачи, NVS, Wi-Fi, шины, Serial) в
//                      исходное состояние; зовётся из setUp();
//   Fake*            — управляемые реализации интерфейсов проекта
//                      (IUartPort, IServoOutput, IBoard, датчики);
//   ibusFrame()      — корректный кадр iBUS с нужными каналами;
//   I2cRig/SpiRig    — драйвер датчика поверх настоящих Esp32*Bus и
//                      *RegisterDevice с симулированным чипом.
// ============================================================

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

#include <deque>
#include <string>
#include <vector>

#include "config/Channels.h"
#include "config/Config.h"
#include "hal/IBoard.h"
#include "hal/RegisterDevice.h"
#include "hal/esp32/Esp32I2CBus.h"
#include "hal/esp32/Esp32SpiBus.h"
#include "sensors/SensorInterface.h"

inline void resetWorld()
{
    fake::resetHal();
    fake::resetNvs();
    fake::wifi() = fake::WifiState();
    fake::setSerialEcho(false);
    Serial.resetFake();
    Wire.resetFake();
    Wire1.resetFake();
    SPI.resetFake();
}

inline bool contains(const std::string& text, const char* fragment)
{
    return text.find(fragment) != std::string::npos;
}

// Вывод Serial с прошлого вызова.
inline std::string takeSerial()
{
    return Serial.takeTx();
}

// ------------------------------------------------------------
// Кадры iBUS
// ------------------------------------------------------------

struct RcChannels
{
    uint16_t value[Config::IBUS_CHANNELS];

    RcChannels()
    {
        for (uint16_t& v : value) v = Config::PWM_CENTER;
        value[Channels::THROTTLE] = Config::PWM_MIN;
        value[Channels::ARM] = Config::PWM_MIN;
        value[Channels::FLAPS] = Config::PWM_MIN;
        value[Channels::AUX_2] = Config::PWM_MIN;
    }

    RcChannels& set(uint8_t channel, uint16_t us)
    {
        value[channel] = us;
        return *this;
    }
};

// Кадр из 32 байт: заголовок, 14 каналов (10 наших + 4 заполнителя),
// CRC. upperBits — служебные старшие 4 бита каждого канала (FS-iA6B
// кладёт туда данные каналов 15-18).
inline std::string ibusFrame(const RcChannels& rc, uint8_t upperBits = 0, bool corruptCrc = false)
{
    std::string frame;
    frame.push_back(static_cast<char>(Config::IBUS_HEADER_0));
    frame.push_back(static_cast<char>(Config::IBUS_HEADER_1));
    for (uint8_t ch = 0; ch < 14; ++ch)
    {
        const uint16_t us = ch < Config::IBUS_CHANNELS ? rc.value[ch] : 1500;
        const uint16_t raw = static_cast<uint16_t>(us | (static_cast<uint16_t>(upperBits & 0x0F) << 12));
        frame.push_back(static_cast<char>(raw & 0xFF));
        frame.push_back(static_cast<char>(raw >> 8));
    }
    uint16_t checksum = 0xFFFF;
    for (char c : frame) checksum -= static_cast<uint8_t>(c);
    if (corruptCrc) checksum ^= 0x0101;
    frame.push_back(static_cast<char>(checksum & 0xFF));
    frame.push_back(static_cast<char>(checksum >> 8));
    return frame;
}

// ------------------------------------------------------------
// Дублёры HAL
// ------------------------------------------------------------

class FakeUart : public IUartPort
{
public:
    uint32_t baud = 0;
    unsigned beginCalls = 0;
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;

    void begin(uint32_t baudRate) override
    {
        baud = baudRate;
        beginCalls++;
    }

    int available() override { return static_cast<int>(rx.size()); }

    int read() override
    {
        if (rx.empty()) return -1;
        const uint8_t b = rx.front();
        rx.pop_front();
        return b;
    }

    size_t write(uint8_t value) override
    {
        tx.push_back(value);
        return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) override
    {
        tx.insert(tx.end(), buffer, buffer + size);
        return size;
    }

    void push(const std::string& bytes)
    {
        for (char c : bytes) rx.push_back(static_cast<uint8_t>(c));
    }
};

class FakeServo : public IServoOutput
{
public:
    bool attachResult = true;
    bool attached = false;
    uint16_t minUs = 0, maxUs = 0;
    uint16_t lastUs = 0;
    unsigned writes = 0;
    int32_t pulse = -1;

    bool attach(uint16_t min, uint16_t max) override
    {
        minUs = min;
        maxUs = max;
        attached = attachResult;
        return attached;
    }

    void writeMicroseconds(uint16_t us) override
    {
        lastUs = us;
        writes++;
    }

    bool isAttached() const override { return attached; }
    int32_t measurePulseUs() override { return pulse; }
};

class FakeI2cBus : public II2CBus
{
public:
    void begin() override {}
    void setClock(uint32_t) override {}
    void beginTransmission(uint8_t) override {}
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t length) override { return length; }
    uint8_t endTransmission(bool) override { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t quantity) override { return quantity; }
    int available() override { return 0; }
    int read() override { return 0; }
};

class FakeSpiBus : public ISpiBus
{
public:
    void begin() override {}
    void beginTransaction(uint32_t, uint8_t) override {}
    uint8_t transfer(uint8_t) override { return 0; }
    void endTransaction() override {}
};

class FakeBoard : public IBoard
{
public:
    FakeServo servos[ServoChannel::COUNT];
    FakeUart rc;
    FakeUart gps;
    FakeI2cBus sensorBus;
    FakeI2cBus screenBus;
    FakeSpiBus spiBus;
    bool hasScreen = true;
    unsigned beginCalls = 0;

    void begin() override { beginCalls++; }
    II2CBus& i2c() override { return sensorBus; }
    ISpiBus& spi() override { return spiBus; }
    II2CBus* displayI2c() override { return hasScreen ? &screenBus : nullptr; }
    IUartPort& rcUart() override { return rc; }
    IUartPort& gpsUart() override { return gps; }
    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }
};

// Драйвер поверх настоящего I2C-стека: TwoWire (фейк) -> Esp32I2CBus
// -> I2cRegisterDevice, чип — fake::RegisterMapDevice.
struct I2cRig
{
    TwoWire wire{ 7 };
    Esp32I2CBus bus{ wire, 8, 9 };
    fake::RegisterMapDevice chip;
    I2cRegisterDevice device;

    explicit I2cRig(uint8_t address) : device(bus, address)
    {
        wire.attach(address, &chip);
        bus.begin();
    }
};

// То же для SPI: SPIClass (фейк) -> Esp32SpiBus -> SpiRegisterDevice.
// Esp32SpiBus работает с глобальным SPI, поэтому устройство
// подключается к нему.
struct SpiRig
{
    Esp32SpiBus bus{ 12, 13, 11 };
    fake::SpiRegisterMapDevice chip;
    uint8_t csPin;

    explicit SpiRig(uint8_t cs, uint8_t dummyBytes) : csPin(cs)
    {
        chip.dummyBytes = dummyBytes;
        SPI.attach(cs, &chip);
        bus.begin();
    }
};

// ------------------------------------------------------------
// Дублёры датчиков (для автопилота, ARM, телеметрии)
// ------------------------------------------------------------

class FakeImu : public ImuSensor
{
public:
    ImuData data = {};
    bool available = true;
    bool beginResult = true;
    const char* preflightProblem = nullptr;
    unsigned updates = 0, calibrations = 0, orientationCalibrations = 0;
    mutable unsigned statusPrints = 0;
    float yaw = 0;

    bool begin() override { return beginResult; }
    bool isAvailable() const override { return available; }
    void update() override { updates++; }
    const char* getSensorType() const override { return "FakeIMU"; }
    void printStatus() const override
    {
        statusPrints++;
        Serial.println("FakeIMU status");
    }
    const ImuData& getImuData() const override { return data; }
    void calibrate() override { calibrations++; }
    void setYaw(float y) override { yaw = y; }
    void calibrateOrientation() override { orientationCalibrations++; }
    const char* getPreflightProblem() const override { return preflightProblem; }
};

class FakeBaro : public BarometerSensor
{
public:
    BarometerData data = {};
    bool available = true;
    unsigned updates = 0, calibrations = 0;
    float seaLevel = 0;

    bool begin() override { return true; }
    bool isAvailable() const override { return available; }
    void update() override { updates++; }
    const char* getSensorType() const override { return "FakeBaro"; }
    void printStatus() const override { Serial.println("FakeBaro status"); }
    const BarometerData& getBarometerData() const override { return data; }
    void calibrateAltitude() override { calibrations++; }
    void setSeaLevelPressure(float p) override { seaLevel = p; }
};

class FakeMag : public MagnetometerSensor
{
public:
    MagData data = {};
    bool available = true;
    unsigned updates = 0, calibrations = 0;

    bool begin() override { return true; }
    bool isAvailable() const override { return available; }
    void update() override { updates++; }
    const char* getSensorType() const override { return "FakeMag"; }
    void printStatus() const override { Serial.println("FakeMag status"); }
    const MagData& getMagData() const override { return data; }
    void calibrate() override { calibrations++; }
};

class FakeGps : public GpsSensor
{
public:
    GpsData data = {};
    bool available = true;
    unsigned updates = 0;

    bool begin() override { return true; }
    bool isAvailable() const override { return available; }
    void update() override { updates++; }
    const char* getSensorType() const override { return "FakeGPS"; }
    void printStatus() const override { Serial.println("FakeGPS status"); }
    const GpsData& getGpsData() const override { return data; }
    bool hasFix() const override { return available && data.fixType >= 2; }
};
