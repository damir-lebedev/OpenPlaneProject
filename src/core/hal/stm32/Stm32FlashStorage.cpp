// Реализация hal/stm32/Stm32FlashStorage.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/Stm32FlashStorage.h"


auto Stm32FlashStorage::capacity() const -> size_t
{
    return KeyValueStore::CAPACITY < static_cast<size_t>(E2END) + 1 ? KeyValueStore::CAPACITY
                                                                     : static_cast<size_t>(E2END) + 1;
}

auto Stm32FlashStorage::read(uint8_t* destination, size_t size) -> void
{
    eeprom_buffer_fill();
    for (size_t i = 0; i < size && i < capacity(); ++i)
    {
        destination[i] = eeprom_buffered_read_byte(static_cast<uint32_t>(i));
    }
}

auto Stm32FlashStorage::write(const uint8_t* source, size_t size) -> bool
{
    if (size > capacity()) return false;

    noInterrupts();
    memcpy(staged, source, size);
    stagedSize = size;
    pending = true;
    interrupts();
    return true;
}

auto Stm32FlashStorage::service() -> bool
{
    if (!pending) return false;

    // Буфер EEPROM-эмуляции трогает только эта функция; снимок
    // берётся под запретом прерываний (десятки мкс), чтобы write()
    // из полётной задачи не подменил образ на середине.
    noInterrupts();
    for (size_t i = 0; i < stagedSize; ++i)
    {
        eeprom_buffered_write_byte(static_cast<uint32_t>(i), staged[i]);
    }
    pending = false;
    interrupts();

    eeprom_buffer_flush();
    ++flushes;
    return true;
}

auto Stm32FlashStorage::instance() -> Stm32FlashStorage&
{
    static Stm32FlashStorage storage;
    return storage;
}

auto Stm32FlashStorage::store() -> KeyValueStore&
{
    static KeyValueStore kv(instance());
    return kv;
}
