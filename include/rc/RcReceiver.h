#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "hal/IUartPort.h"
#include "rc/CrsfParser.h"
#include "rc/IBusParser.h"
#include "rc/RcChannelState.h"
#include "rc/RcInput.h"
#include "rc/SbusParser.h"

// ============================================================
// ПРИЁМНИК ПУЛЬТА
//
// Кадры из UART -> RC_CHANNELS каналов (мкс) + признак «связи нет».
// Ничего не знает про failsafe-поведение самолёта, Servo или ARM.
// Протокол — Config::RX_PROTOCOL:
//   iBUS — FlySky;  SBUS — FrSky, Futaba, RadioMaster, Jumper...;
//   CRSF — ExpressLRS, TBS Crossfire.
// Разбор кадров — IBusParser / SbusParser / CrsfParser; здесь общее:
// таймаут кадров, failsafe, реверс каналов, счётчики. Сам UART
// спрятан за IUartPort: пины — в реализации, сюда приходят скорость
// и формат кадра (у SBUS — 8E2 с инверсией).
//
// Потеря связи — любой из признаков (isSignalLost()):
//   • кадров нет дольше RX_TIMEOUT_US — обрыв провода, приёмник умер
//     или по потере связи молчит (так ExpressLRS по умолчанию);
//   • приёмник сам сообщает о failsafe: флаг в кадре SBUS, качество
//     связи 0 % в статистике CRSF;
//   • газ ниже RX_FAILSAFE_THROTTLE_US — так failsafe сообщает FS-iA6B
//     (он кадры слать не перестаёт, см. комментарий у
//     RX_FAILSAFE_THROTTLE_US в Config.h) и любой приёмник, у которого
//     failsafe настроен «газ ниже минимума».
// ============================================================

static_assert(!Config::CHANNEL_REVERSED[Channels::THROTTLE],
              "CHANNEL_REVERSED: газ (CH3) реверсировать нельзя — потеря связи определяется по газу < RX_FAILSAFE_THROTTLE_US");

inline const char* rxProtocolName(Config::RxProtocol protocol)
{
    switch (protocol)
    {
        case Config::RxProtocol::SBUS: return "SBUS";
        case Config::RxProtocol::CRSF: return "CRSF";
        default:                       return "iBUS";
    }
}

class RcReceiver
{
public:

    explicit RcReceiver(IUartPort& port, Config::RxProtocol rxProtocol = Config::RX_PROTOCOL)
        : serial(port), protocol(rxProtocol)
    {
    }

    void begin()
    {
        switch (protocol)
        {
            case Config::RxProtocol::SBUS:
                portReady = serial.begin(Config::SBUS_BAUDRATE, UartFormat::SBUS_8E2_INVERTED);
                break;
            case Config::RxProtocol::CRSF:
                portReady = serial.begin(Config::CRSF_BAUDRATE, UartFormat::STANDARD_8N1);
                break;
            default:
                portReady = serial.begin(Config::IBUS_BAUDRATE, UartFormat::STANDARD_8N1);
                break;
        }

        lastFrameTime = micros();
    }

    // Вызывать каждый цикл: вычитывает всё, что накопилось в UART-буфере.
    void update()
    {
        while (serial.available())
        {
            const uint8_t value = static_cast<uint8_t>(serial.read());
            switch (protocol)
            {
                case Config::RxProtocol::SBUS:
                    if (sbus.feed(value)) accept(sbus.values(), sbus.failsafe());
                    break;
                case Config::RxProtocol::CRSF:
                    if (crsf.feed(value)) accept(crsf.values(), false);
                    break;
                default:
                    if (ibus.feed(value)) accept(ibus.values(), false);
                    break;
            }
        }
    }

    const RcChannelState& getState() const
    {
        return state;
    }

    bool isSignalLost() const
    {
        return isFrameTimeout() || isFailsafeReported();
    }

    // Кадров нет дольше Config::RX_TIMEOUT_US — или не было ещё ни
    // одного: до первого кадра в каналах лежат значения по умолчанию
    // (все 1500), и принимать их за команды пульта нельзя (раньше CH7
    // = 1500 успевал включить STABILIZE до первого кадра).
    bool isFrameTimeout() const
    {
        return !receivedAnyFrame || (micros() - lastFrameTime) > Config::RX_TIMEOUT_US;
    }

    // Приёмник сообщает, что связи с пультом нет: флаг кадра, LQ = 0 или
    // failsafe-газ ниже нормы.
    bool isFailsafeReported() const
    {
        return failsafeReported || (protocol == Config::RxProtocol::CRSF && crsf.failsafe());
    }

    uint32_t getLastFrameTime() const
    {
        return lastFrameTime;
    }

    uint32_t getGoodFrameCount() const { return goodFrames; }

    uint32_t getBadFrameCount() const
    {
        switch (protocol)
        {
            case Config::RxProtocol::SBUS: return sbus.badFrames();
            case Config::RxProtocol::CRSF: return crsf.badFrames();
            default:                       return ibus.badFrames();
        }
    }

    Config::RxProtocol getProtocol() const { return protocol; }
    const char* getProtocolName() const { return rxProtocolName(protocol); }

    // Качество связи по радио, % — его сообщает только CRSF; −1 — неизвестно.
    int16_t getLinkQuality() const
    {
        return protocol == Config::RxProtocol::CRSF ? crsf.linkQuality() : -1;
    }

    // UART открылся в формате протокола. false — порт так не умеет
    // (SBUS без инверсии): кадров не будет, связь — «потеряна».
    bool isPortReady() const { return portReady; }


private:

    IUartPort& serial;
    Config::RxProtocol protocol;

    IBusParser ibus;
    SbusParser sbus;
    CrsfParser crsf;

    RcChannelState state;

    uint32_t lastFrameTime = 0;
    bool receivedAnyFrame = false;
    bool failsafeReported = false;
    bool portReady = false;
    uint32_t goodFrames = 0;

    void accept(const uint16_t* values, bool receiverFailsafe)
    {
        for (uint8_t channel = 0; channel < Config::RC_CHANNELS; ++channel)
        {
            const uint16_t value = values[channel];

            // Потеря связи по газу — по сырому значению, до реверса каналов.
            if (channel == Channels::THROTTLE)
            {
                failsafeReported = receiverFailsafe || value < Config::RX_FAILSAFE_THROTTLE_US;
            }

            state.set(channel, Config::CHANNEL_REVERSED[channel] ? RcInput::reversed(value) : value);
        }

        goodFrames++;
        receivedAnyFrame = true;
        lastFrameTime = micros();
    }
};
