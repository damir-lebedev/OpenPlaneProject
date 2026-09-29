#pragma once
#include <Arduino.h>

// ============================================================
// Бортовой самописец — то, что нужно от него консоли (меню 'k' и
// команды "bb ..." от ПК). Реализация — BlackBox (ESP32-S3); консоль
// от неё не зависит и собирается на любой плате.
// ============================================================

class IFlightRecorder
{
public:
    virtual ~IFlightRecorder() = default;

    virtual bool isRecording() const = 0;
    virtual void requestManualStart() = 0;
    virtual void requestManualStop() = 0;

    virtual void printStatus(Print& out) const = 0;
    virtual void printFlights(Print& out) = 0;

    // Стереть все записи (блокирует; только без ARM и без записи).
    virtual bool eraseAll() = 0;

    // Строка команды с ПК после байта STX, например "bb list".
    virtual void handleHostCommand(const char* line) = 0;
};
