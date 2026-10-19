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
//   sbusFrame(), crsfChannels(), crsfLinkStats() — то же для SBUS и CRSF;
//   I2cRig/SpiRig    — драйвер датчика поверх настоящих Esp32*Bus и
//                      *RegisterDevice с симулированным чипом.
// ============================================================

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#if defined(BOARD_STM32H743)
#include <EEPROM.h>
#include <STM32FreeRTOS.h>
#else
#include <WiFi.h>
#endif

#include <deque>
#include <set>
#include <string>
#include <vector>

#include "config/Channels.h"
#include "config/Config.h"
#include "control/EscSignal.h"
#include "hal/IBoard.h"
#include "hal/RegisterDevice.h"
#include "rc/CrsfParser.h"
#if !defined(BOARD_STM32H743)
#include "hal/esp32/Esp32I2CBus.h"
#include "hal/esp32/Esp32SpiBus.h"
#endif
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "sensors/imu/ImuOrientation.h"

// Что выйдет на ESC при стике газа stickUs: верх хода ограничен
// Config::THROTTLE_LIMIT_PCT (FlightController::capThrottle) — тесты не
// привязаны к текущему значению лимита.
inline uint16_t cappedThrottleUs(uint16_t stickUs)
{
    const long top = Config::PWM_MIN + (Config::PWM_MAX - Config::PWM_MIN) * Config::THROTTLE_LIMIT_PCT / 100;
    return static_cast<uint16_t>(map(stickUs, Config::PWM_MIN, Config::PWM_MAX, Config::PWM_MIN, top));
}

// Импульс на пине ESC при газе throttleUs (1000..2000 = 0..100%): у
// двунаправленного ESC (Config::ESC_BIDIRECTIONAL) 1500 = мотор стоит.
inline uint16_t escUs(uint16_t throttleUs, bool reverse = false)
{
    return EscSignal::pulseUs(throttleUs, reverse);
}

inline void resetWorld()
{
    fake::resetHal();
#if defined(BOARD_STM32H743)
    // Среда native-stm32: настройки — во "флеше" EEPROM-эмуляции, Wi-Fi нет.
    fake::resetEeprom();
    fake::timerPulses().clear();
    fake::schedulerStarted() = false;
    fake::resetSdCard();
    fake::rcc().RSR = RCC_RSR_PORRSTF | RCC_RSR_PINRSTF;
    fake::adcBits() = 10;
#else
    fake::resetNvs();
    fake::wifi() = fake::WifiState();
#endif
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
    uint16_t value[Config::RC_CHANNELS];

    RcChannels()
    {
        for (uint16_t& v : value) v = Config::PWM_CENTER;
        value[Channels::THROTTLE] = Config::PWM_MIN;
        value[Channels::ARM] = Config::PWM_MIN;
        value[Channels::SWB] = Config::PWM_MIN;
        value[Channels::SWC] = Config::PWM_MIN;
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
        const uint16_t us = ch < Config::RC_CHANNELS ? rc.value[ch] : 1500;
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

// 16 каналов по 11 бит (SBUS, CRSF): наши RC_CHANNELS + остальные в центре.
// мкс -> значение: обратное Packed11::toMicros (точно — для мкс, кратных 5).
inline std::string packed11(const RcChannels& rc)
{
    std::string out;
    uint32_t bits = 0;
    uint8_t count = 0;
    for (uint8_t ch = 0; ch < 16; ++ch)
    {
        const uint16_t us = ch < Config::RC_CHANNELS ? rc.value[ch] : 1500;
        const uint32_t raw = (static_cast<uint32_t>(us) - 880) * 8 / 5;
        bits |= (raw & 0x7FF) << count;
        count += 11;
        while (count >= 8)
        {
            out.push_back(static_cast<char>(bits & 0xFF));
            bits >>= 8;
            count -= 8;
        }
    }
    return out;
}

// Кадр SBUS: 0x0F, каналы, флаги (0x08 — failsafe, 0x04 — потерян кадр), конец.
inline std::string sbusFrame(const RcChannels& rc, uint8_t flags = 0, uint8_t end = 0x00)
{
    std::string frame(1, '\x0F');
    frame += packed11(rc);
    frame.push_back(static_cast<char>(flags));
    frame.push_back(static_cast<char>(end));
    return frame;
}

// Кадр CRSF полётнику: 0xC8, длина, тип, данные, CRC8.
inline std::string crsfFrame(uint8_t type, const std::string& payload, bool corruptCrc = false)
{
    std::string body(1, static_cast<char>(type));
    body += payload;
    uint8_t crc = CrsfParser::crc8(reinterpret_cast<const uint8_t*>(body.data()), static_cast<uint8_t>(body.size()));
    if (corruptCrc) crc ^= 0x5A;
    std::string frame;
    frame.push_back(static_cast<char>(CrsfParser::ADDRESS_FLIGHT_CONTROLLER));
    frame.push_back(static_cast<char>(body.size() + 1));
    frame += body;
    frame.push_back(static_cast<char>(crc));
    return frame;
}

inline std::string crsfChannels(const RcChannels& rc, bool corruptCrc = false)
{
    return crsfFrame(CrsfParser::TYPE_RC_CHANNELS, packed11(rc), corruptCrc);
}

// Статистика связи: RSSI 1/2, LQ, SNR, антенна, режим, мощность, обратный канал.
inline std::string crsfLinkStats(uint8_t linkQuality)
{
    const char payload[10] = { 60, 62, static_cast<char>(linkQuality), 9, 0, 4, 2, 70, 100, 8 };
    return crsfFrame(CrsfParser::TYPE_LINK_STATISTICS, std::string(payload, sizeof(payload)));
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
    int room = -1;   // availableForWrite(): −1 — без ограничения

    UartFormat format = UartFormat::STANDARD_8N1;
    bool supportsSbus = true;

    void begin(uint32_t baudRate) override
    {
        baud = baudRate;
        format = UartFormat::STANDARD_8N1;
        beginCalls++;
    }

    bool begin(uint32_t baudRate, UartFormat frameFormat) override
    {
        if (frameFormat == UartFormat::SBUS_8E2_INVERTED && !supportsSbus) return false;
        baud = baudRate;
        format = frameFormat;
        beginCalls++;
        return true;
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

    int availableForWrite() override { return room; }

    void push(const std::string& bytes)
    {
        for (char c : bytes) rx.push_back(static_cast<uint8_t>(c));
    }

    void push(const std::vector<uint8_t>& bytes)
    {
        rx.insert(rx.end(), bytes.begin(), bytes.end());
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
    bool pulsesOff = false;   // stopPulses() — импульсов нет до следующей записи
    unsigned stops = 0;

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
        pulsesOff = false;
    }

    void stopPulses() override
    {
        pulsesOff = true;
        stops++;
    }

    bool isAttached() const override { return attached; }
    int32_t measurePulseUs() override { return pulse; }
};

class FakeI2cBus : public II2CBus
{
public:
    // Пусто — отвечает любой адрес; иначе — только перечисленные.
    std::set<uint8_t> devices;

    void begin() override {}
    void setClock(uint32_t) override {}
    void beginTransmission(uint8_t address) override { current = address; }
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t length) override { return length; }
    uint8_t endTransmission(bool) override { return devices.empty() || devices.count(current) ? 0 : 2; }
    uint8_t requestFrom(uint8_t, uint8_t quantity) override { return quantity; }
    int available() override { return 0; }
    int read() override { return 0; }

private:
    uint8_t current = 0;
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
    FakeUart telemetry;
    bool hasTelemetry = false;
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
    IUartPort* telemetryUart() override { return hasTelemetry ? &telemetry : nullptr; }
    IServoOutput& servo(uint8_t channel) override { return servos[channel]; }

    bool buzzer = false;
    unsigned buzzerChanges = 0;
    void setBuzzer(bool on) override
    {
        if (on != buzzer) buzzerChanges++;
        buzzer = on;
    }
};

#if !defined(BOARD_STM32H743)
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
#endif

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
    bool mountCalibrated = false;
    bool isMountCalibrated() const override { return mountCalibrated; }
    const char* getPreflightProblem() const override { return preflightProblem; }

    // Наземная калибровка: "верх" в осях чипа (тест задаёт), установка,
    // стоянка — и что сохранено.
    float chipUp[3] = { 0.0f, 0.0f, 1.0f };
    bool hasChipUp = true;
    bool supportsGround = true;
    bool refuseGroundCalibration = false;   // setGroundCalibration() не смог сохранить
    ImuOrientation orientation;
    bool parkingValid = false;
    float parkingPitch = 0, parkingRoll = 0;
    unsigned groundCalibrationsSaved = 0;
    float savedPark[3] = {};

    // Автовзлёт: опора горизонта и уточнения нуля гироскопа.
    bool accelCorrection = true;
    unsigned accelCorrectionChanges = 0;
    unsigned gyroTrims = 0;
    float lastGyroTrim[3] = {};

    bool readChipUp(float up[3]) const override
    {
        if (!hasChipUp) return false;
        for (int i = 0; i < 3; ++i) up[i] = chipUp[i];
        return true;
    }

    bool getOrientation(ImuOrientation& out) const override
    {
        if (!supportsGround) return false;
        out = orientation;
        return true;
    }

    bool setGroundCalibration(const ImuOrientation& level, const float parkChipUp[3]) override
    {
        if (refuseGroundCalibration) return false;
        orientation = level;
        for (int i = 0; i < 3; ++i) savedPark[i] = parkChipUp[i];
        level.attitudeOf(parkChipUp, parkingPitch, parkingRoll);
        parkingValid = true;
        groundCalibrationsSaved++;
        return true;
    }

    bool getParkingAttitude(float& pitchDeg, float& rollDeg) const override
    {
        if (!parkingValid) return false;
        pitchDeg = parkingPitch;
        rollDeg = parkingRoll;
        return true;
    }

    void setAccelCorrection(bool enabled) override
    {
        if (enabled != accelCorrection) accelCorrectionChanges++;
        accelCorrection = enabled;
    }

    void trimGyroBias(const float residualDps[3]) override
    {
        gyroTrims++;
        for (int i = 0; i < 3; ++i) lastGyroTrim[i] = residualDps[i];
    }

    // Поставить стоянку (как будто калибровка уже была).
    void park(float pitchDeg, float rollDeg = 0.0f)
    {
        parkingValid = true;
        parkingPitch = pitchDeg;
        parkingRoll = rollDeg;
    }
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
    bool calibrated = false;
    bool isCalibrated() const override { return calibrated; }
};

class FakeAirspeed : public AirspeedSensor
{
public:
    AirspeedData data = {};
    bool available = true;
    unsigned updates = 0, zeroCalls = 0;

    bool begin() override { return true; }
    bool isAvailable() const override { return available; }
    void update() override { updates++; }
    const char* getSensorType() const override { return "FakeAirspeed"; }
    void printStatus() const override { Serial.println("FakeAirspeed status"); }
    const AirspeedData& getAirspeedData() const override { return data; }
    void calibrateZero() override { zeroCalls++; }

    void setSpeed(float ms)
    {
        data.indicatedMs = ms;
        data.trueMs = ms;
        data.airDensity = 1.225f;
        data.differentialPressurePa = 0.5f * 1.225f * ms * ms;
        data.timestamp = micros();
    }
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
