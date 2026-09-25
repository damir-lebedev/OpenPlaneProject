#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "hal/IUartPort.h"
#include "rc/RcChannelState.h"

// ============================================================
// iBUS RECEIVER
//
// Парсит iBUS-кадры из UART в 10 RC-каналов. Ничего не знает
// про failsafe-поведение самолёта, Servo или ARM — только UART
// -> RcChannelState + признак "связи нет". Замена протокола
// (S-Bus, PWM) требует правки только этого файла. Сам UART спрятан
// за IUartPort — пины и формат кадра фиксированы в реализации (см.
// Esp32UartPort), сюда приходит только скорость.
//
// Формат кадра (32 байта):
//   [0x20][0x40] [CH1 low][CH1 high] ... [CH14 low][CH14 high] [CRC low][CRC high]
//   CRC = 0xFFFF - (сумма первых 30 байт). Берутся первые 10 каналов.
//
// Потеря связи — два независимых признака (см. isSignalLost()):
//   • кадров нет дольше RX_TIMEOUT_US — обрыв провода/приёмник умер;
//   • газ ниже RX_FAILSAFE_THROTTLE_US — так FS-iA6B сообщает о
//     потере связи с пультом (сам он кадры слать не перестаёт, см.
//     комментарий у RX_FAILSAFE_THROTTLE_US в Config.h).
// ============================================================

class IBusReceiver
{
public:

    explicit IBusReceiver(IUartPort& port)
        : serial(port)
    {
    }

    void begin()
    {
        serial.begin(Config::IBUS_BAUDRATE);

        lastFrameTime = micros();
    }

    // Вызывать каждый цикл: вычитывает всё, что накопилось в UART-буфере.
    void update()
    {
        while (serial.available())
        {
            processByte(static_cast<uint8_t>(serial.read()));
        }
    }

    const RcChannelState& getState() const
    {
        return state;
    }

    bool isSignalLost() const
    {
        return isFrameTimeout() || failsafeReported;
    }

    // Кадров нет дольше Config::RX_TIMEOUT_US — или не было ещё ни
    // одного: до первого кадра в каналах лежат значения по умолчанию
    // (все 1500), и принимать их за команды пульта нельзя (раньше CH7
    // = 1500 успевал включить STABILIZE до первого кадра).
    bool isFrameTimeout() const
    {
        return !receivedAnyFrame || (micros() - lastFrameTime) > Config::RX_TIMEOUT_US;
    }

    // Приёмник шлёт failsafe-значения (связи с пультом нет).
    bool isFailsafeReported() const
    {
        return failsafeReported;
    }

    uint32_t getLastFrameTime() const
    {
        return lastFrameTime;
    }

    uint32_t getGoodFrameCount() const { return goodFrames; }
    uint32_t getBadFrameCount() const { return badFrames; }


private:

    IUartPort& serial;

    RcChannelState state;

    uint8_t frame[Config::IBUS_FRAME_LENGTH] = {};

    uint8_t frameIndex = 0;

    uint32_t lastFrameTime = 0;
    bool receivedAnyFrame = false;
    bool failsafeReported = false;

    uint32_t goodFrames = 0;
    uint32_t badFrames = 0;

    // Побайтовый разбор кадра: байты могут приходить порциями,
    // поэтому нельзя ждать весь кадр за один serial.available().
    void processByte(uint8_t value)
    {
        if (frameIndex == 0)
        {
            if (value != Config::IBUS_HEADER_0)
            {
                return;
            }

            frame[frameIndex++] = value;
            return;
        }

        if (frameIndex == 1)
        {
            if (value != Config::IBUS_HEADER_1)
            {
                // 0x20 был случайным, начинаем заново — но этот байт сам
                // может быть началом кадра (0x20 0x20 0x40 ...).
                frameIndex = (value == Config::IBUS_HEADER_0) ? 1 : 0;
                return;
            }

            frame[frameIndex++] = value;
            return;
        }

        frame[frameIndex++] = value;

        if (frameIndex >= Config::IBUS_FRAME_LENGTH)
        {
            processFrame();
            frameIndex = 0;
        }
    }

    void processFrame()
    {
        uint16_t checksum = 0xFFFF;

        for (uint8_t i = 0; i < 30; ++i)
        {
            checksum -= frame[i];
        }

        const uint16_t receivedChecksum =
            static_cast<uint16_t>(frame[30]) |
            (static_cast<uint16_t>(frame[31]) << 8);

        if (checksum != receivedChecksum)
        {
            badFrames++;
            return;  // помехи на линии — игнорируем кадр
        }

        for (uint8_t channel = 0;
             channel < Config::IBUS_CHANNELS;
             ++channel)
        {
            const uint8_t lowByte  = frame[2 + channel * 2];
            const uint8_t highByte = frame[3 + channel * 2];

            // Значение канала — только младшие 12 бит. В старших 4 битах
            // FS-iA6B передаёт служебные данные (так кодируются каналы
            // 15-18), и они не нулевые, например, в failsafe: на стенде
            // CH3 = 0x2384 -> 900 мкс, а без маски читалось 9092, и
            // failsafe по газу (< 950) не срабатывал.
            const uint16_t value =
                (static_cast<uint16_t>(lowByte) |
                 (static_cast<uint16_t>(highByte) << 8)) & 0x0FFF;

            state.set(channel, value);
        }

        failsafeReported =
            state.get(Channels::THROTTLE) < Config::RX_FAILSAFE_THROTTLE_US;

        goodFrames++;
        receivedAnyFrame = true;
        lastFrameTime = micros();
    }
};
