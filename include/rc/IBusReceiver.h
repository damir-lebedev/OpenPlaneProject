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

    explicit IBusReceiver(IUartPort& port);

    void begin();

    // Вызывать каждый цикл: вычитывает всё, что накопилось в UART-буфере.
    void update();

    const RcChannelState& getState() const;

    bool isSignalLost() const;

    // Кадров нет дольше Config::RX_TIMEOUT_US — или не было ещё ни
    // одного: до первого кадра в каналах лежат значения по умолчанию
    // (все 1500), и принимать их за команды пульта нельзя (раньше CH7
    // = 1500 успевал включить STABILIZE до первого кадра).
    bool isFrameTimeout() const;

    // Приёмник шлёт failsafe-значения (связи с пультом нет).
    bool isFailsafeReported() const;

    uint32_t getLastFrameTime() const;

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
    void processByte(uint8_t value);

    void processFrame();
};
