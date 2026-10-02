#pragma once
#include <stdint.h>

// ============================================================
// Блочное устройство — SD-карта как её видит прошивка: массив
// блоков по 512 байт, чтение и запись блоками. Стирания нет:
// перезаписывать блок можно сколько угодно (у карты внутри свой
// контроллер; байты "стёртого" состояния 0xFF пишет тот, кому они
// нужны — SdFileRegion).
//
// Реализации: Stm32SdCard (SDMMC1, hal/stm32/), FakeSdCard в тестах.
// Не потокобезопасно: владелец — одна задача (BlackBox держит мьютекс).
// ============================================================

class IBlockDevice
{
public:
    static constexpr uint32_t BLOCK_SIZE = 512;

    virtual ~IBlockDevice() = default;

    // Размер в блоках; 0 — карты нет или она не отвечает.
    virtual uint32_t blockCount() const = 0;

    // count блоков подряд, начиная с block. data — любой адрес.
    virtual bool read(uint32_t block, void* data, uint32_t count) = 0;
    virtual bool write(uint32_t block, const void* data, uint32_t count) = 0;
};
