#pragma once
#include <Arduino.h>

// EEPROM-эмуляция STM32duino: eeprom_buffer_*(). Сам объект EEPROM из
// заголовка не нужен (он static — предупреждение "не используется").
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <EEPROM.h>
#pragma GCC diagnostic pop

#include "storage/KeyValueStore.h"

// ============================================================
// Носитель настроек на STM32H743 — последний сектор флеша (банк 2)
// через EEPROM-эмуляцию STM32duino: образ 8 КБ в ОЗУ
// (eeprom_buffer_fill/flush), из которых KeyValueStore занимает
// первые CAPACITY байт.
//
// Стирание сектора 128 КБ у H743 длится секунды. Поэтому запись
// разделена на две части:
//   write()   — быстро: копирует образ в свой буфер и ставит флаг;
//               вызывается из KvPreferences::end() в полётной задаче;
//   service() — медленно: стирает сектор и пишет; вызывается ТОЛЬКО
//               из фоновой задачи низкого приоритета (src/stm32/main.cpp).
// Прошивка исполняется из банка 1, а сектор настроек — в банке 2, и
// флеш H7 умеет читать один банк, пока пишется другой: полётная
// задача вытесняет фоновую и продолжает работать во время стирания —
// даже сохранение триммера в воздухе не остановит цикл.
//
// Пропадёт питание посреди записи — CRC образа не сойдётся, и при
// следующем включении настройки будут по умолчанию (калибровки
// придётся повторить), но не мусором.
// ============================================================

class Stm32FlashStorage : public IFlashStorage
{
public:

    size_t capacity() const override;

    void read(uint8_t* destination, size_t size) override;

    bool write(const uint8_t* source, size_t size) override;

    // Перенести отложенную запись во флеш. Возвращает true, если писал.
    bool service();

    bool hasPending() const { return pending; }
    uint32_t flushCount() const { return flushes; }

    static Stm32FlashStorage& instance();

    static KeyValueStore& store();


private:

    uint8_t staged[KeyValueStore::CAPACITY] = {};
    size_t stagedSize = 0;
    volatile bool pending = false;
    uint32_t flushes = 0;
};
