#pragma once
#include <Arduino.h>

#include "telemetry/BlackBoxFormat.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК: очередь записей между ядрами
//
// Полётный цикл (ядро 1) кладёт готовые записи, задача записи на
// флеш (ядро 0) забирает их по одной. Байтовое кольцо в PSRAM
// (мегабайты) — запас и на предзапись перед стартом, и на время,
// когда флеш не успевает или кончилось стёртое место.
//
// Переполнилась — выбрасываются САМЫЕ СТАРЫЕ записи: при падении
// важнее последние секунды, чем середина полёта. Каждая операция —
// короткая критическая секция (спинлок между ядрами), флеш под ней
// не трогается.
// ============================================================

class BlackBoxRing
{
public:

    void begin(uint8_t* memory, uint32_t size)
    {
        buffer = memory;
        capacity = memory ? size : 0;
        clear();
    }

    bool isReady() const { return capacity > 0; }
    uint32_t size() const { return capacity; }
    uint32_t used() const { return usedBytes; }
    uint32_t dropped() const { return droppedRecords; }

    void clear()
    {
        portENTER_CRITICAL(&lock);
        head = tail = usedBytes = 0;
        portEXIT_CRITICAL(&lock);
    }

    // Положить запись; места нет — выбросить старые. false — запись
    // больше всей очереди (не бывает) или очереди нет.
    bool push(const uint8_t* record, size_t length)
    {
        if (length > capacity || length < BlackBoxFormat::RECORD_HEADER) return false;

        portENTER_CRITICAL(&lock);
        while (capacity - usedBytes < length)
        {
            dropOldest();
            droppedRecords++;
        }
        copyIn(head, record, length);
        head = (head + length) % capacity;
        usedBytes += length;
        portEXIT_CRITICAL(&lock);
        return true;
    }

    // Забрать самую старую запись в out (не меньше MAX_RECORD байт).
    // 0 — очередь пуста.
    size_t pop(uint8_t* out)
    {
        portENTER_CRITICAL(&lock);
        size_t length = 0;
        if (usedBytes > 0)
        {
            length = recordLength(tail);
            copyOut(tail, out, length);
            tail = (tail + length) % capacity;
            usedBytes -= length;
        }
        portEXIT_CRITICAL(&lock);
        return length;
    }

    // Предзапись: выбросить записи старше maxAgeUs (по t_us записи).
    void trimOlderThan(uint32_t nowUs, uint32_t maxAgeUs)
    {
        portENTER_CRITICAL(&lock);
        while (usedBytes > 0 && nowUs - timestampAt(tail) > maxAgeUs)
        {
            dropOldest();
        }
        portEXIT_CRITICAL(&lock);
    }


private:

    uint8_t* buffer = nullptr;
    uint32_t capacity = 0;
    uint32_t head = 0;
    uint32_t tail = 0;
    volatile uint32_t usedBytes = 0;
    volatile uint32_t droppedRecords = 0;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

    uint8_t byteAt(uint32_t index) const { return buffer[index % capacity]; }

    size_t recordLength(uint32_t at) const
    {
        return BlackBoxFormat::RECORD_HEADER + byteAt(at + 1);
    }

    uint32_t timestampAt(uint32_t at) const
    {
        uint32_t t = 0;
        for (uint8_t i = 0; i < 4; ++i) t |= static_cast<uint32_t>(byteAt(at + 2 + i)) << (8 * i);
        return t;
    }

    void dropOldest()
    {
        const size_t length = recordLength(tail);
        tail = (tail + length) % capacity;
        usedBytes -= length;
    }

    void copyIn(uint32_t at, const uint8_t* data, size_t length)
    {
        const size_t first = min(length, static_cast<size_t>(capacity - at));
        memcpy(buffer + at, data, first);
        memcpy(buffer, data + first, length - first);
    }

    void copyOut(uint32_t at, uint8_t* data, size_t length) const
    {
        const size_t first = min(length, static_cast<size_t>(capacity - at));
        memcpy(data, buffer + at, first);
        memcpy(data + first, buffer, length - first);
    }
};
