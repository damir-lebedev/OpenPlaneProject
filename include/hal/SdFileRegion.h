#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hal/IBlockDevice.h"
#include "hal/IFlashRegion.h"
#include "storage/Fat32File.h"

// ============================================================
// Область для чёрного ящика на SD-карте: файл в корне FAT32 (по
// умолчанию BLACKBOX.BIN), который заранее создан на ПК одним куском
// и целиком заполнен 0xFF — tools/blackbox.py sd-prepare. Здесь файл
// только находится (Fat32::locate), дальше пишутся сырые блоки карты
// внутри него; таблицы FAT и каталог не трогаются никогда.
//
// Первый сектор файла (4 КБ) — служебный: метка "кольцо пусто" (см.
// IFlashRegion::ringMarkedEmpty). tools/blackbox.py sd-prepare ставит её
// сразу, поэтому новая карта загружается без сверки. Кольцо — остальное.
//
// Для BlackBoxStorage это тот же IFlashRegion, что раздел флеша ESP32:
//   • "сектор" — 4 КБ (кратно блоку карты 512 байт);
//   • стирание — запись 0xFF поверх сектора (у карты своё стирание
//     внутри, снаружи оно не нужно);
//   • запись кусками любого размера, в том числе по 256 байт:
//     неполный блок читается, дополняется и пишется целиком. Блок,
//     который только что писали, помнится — дописывание страницами
//     подряд не читает карту заново.
// Кэш сквозной (запись сразу уходит на карту), поэтому пропажа
// питания не теряет ничего, что уже вернулось из write().
// Не потокобезопасен: владелец — одна задача (BlackBox держит мьютекс).
// ============================================================

class SdFileRegion : public IFlashRegion
{
public:
    static constexpr uint32_t SECTOR_BYTES = 4096;
    // Стирание пишет 0xFF кусками по 16 КБ: SD-карта на мелких записях медленная
    // (4 КБ — ~3.7 мс, т.е. 1 МБ/с), на крупных быстрее.
    static constexpr uint32_t ERASE_CHUNK_BYTES = 16384;

    // maxBytes — потолок размера области, даже если файл больше: время
    // сверки секторов при включении растёт вместе с областью.
    SdFileRegion(IBlockDevice& blockDevice, const char* fileName, uint32_t maxBytes = 0xFFFFFFFFu)
        : device(blockDevice),
          name(fileName),
          limit(maxBytes)
    {
        memset(eraseData, 0xFF, sizeof(eraseData));
    }

    // Найти файл на карте. Ok — область готова, size() > 0.
    Fat32::Result begin()
    {
        ready = false;
        cachedBlock = NO_BLOCK;
        regionBytes = 0;

        Fat32::Extent extent;
        const Fat32::Result found = Fat32::locate(device, name, extent);
        if (found != Fat32::Result::Ok) return found;

        const uint32_t usable = (extent.bytes < limit ? extent.bytes : limit) / SECTOR_BYTES * SECTOR_BYTES;
        if (usable <= RESERVED_BYTES) return Fat32::Result::Empty;

        firstBlock = extent.firstBlock;
        fileBytes = extent.bytes;
        regionBytes = usable;
        ready = true;
        return Fat32::Result::Ok;
    }

    const char* fileName() const { return name; }
    uint32_t fileSize() const { return fileBytes; }
    uint32_t firstCardBlock() const { return firstBlock; }

    // Кольцо — без служебного сектора.
    uint32_t size() const override { return ready ? regionBytes - RESERVED_BYTES : 0; }

    bool read(uint32_t offset, void* data, size_t length) override
    {
        return inRing(offset, length) && rawRead(offset + RESERVED_BYTES, data, length);
    }

    bool write(uint32_t offset, const void* data, size_t length) override
    {
        return inRing(offset, length) && rawWrite(offset + RESERVED_BYTES, data, length);
    }

    // Целые сектора по 4 КБ: offset и length кратны SECTOR_BYTES.
    bool erase(uint32_t offset, uint32_t length) override
    {
        return inRing(offset, length) && rawErase(offset + RESERVED_BYTES, length);
    }

    bool ringMarkedEmpty() override
    {
        uint8_t mark[2 * MARK_LENGTH];
        if (!ready || !rawRead(0, mark, sizeof(mark))) return false;
        for (size_t i = 0; i < MARK_LENGTH; ++i)
        {
            if (mark[i] != markByte(i) || mark[MARK_LENGTH + i] != static_cast<uint8_t>(~markByte(i))) return false;
        }
        return true;
    }

    void markRingEmpty() override
    {
        uint8_t mark[2 * MARK_LENGTH];
        for (size_t i = 0; i < MARK_LENGTH; ++i)
        {
            mark[i] = markByte(i);
            mark[MARK_LENGTH + i] = static_cast<uint8_t>(~markByte(i));
        }
        if (ready) rawWrite(0, mark, sizeof(mark));
    }

    void clearRingMark() override
    {
        if (ready) rawErase(0, SECTOR_BYTES);
    }

    // Те же операции по файлу целиком (со служебным сектором): для самой области.
    bool rawRead(uint32_t offset, void* data, size_t length)
    {
        if (!inRange(offset, length)) return false;

        uint8_t* out = static_cast<uint8_t*>(data);
        while (length > 0)
        {
            const uint32_t block = offset / BLOCK;
            const uint32_t inBlock = offset % BLOCK;
            size_t chunk;
            if (inBlock == 0 && length >= BLOCK)
            {
                const uint32_t count = blocksFor(length);
                if (!device.read(firstBlock + block, out, count)) return false;
                chunk = static_cast<size_t>(count) * BLOCK;
            }
            else
            {
                if (!load(block)) return false;
                chunk = length < BLOCK - inBlock ? length : BLOCK - inBlock;
                memcpy(out, cache + inBlock, chunk);
            }
            out += chunk;
            offset += static_cast<uint32_t>(chunk);
            length -= chunk;
        }
        return true;
    }

    bool rawWrite(uint32_t offset, const void* data, size_t length)
    {
        if (!inRange(offset, length)) return false;

        const uint8_t* in = static_cast<const uint8_t*>(data);
        while (length > 0)
        {
            const uint32_t block = offset / BLOCK;
            const uint32_t inBlock = offset % BLOCK;
            size_t chunk;
            if (inBlock == 0 && length >= BLOCK)
            {
                const uint32_t count = blocksFor(length);
                cachedBlock = NO_BLOCK;
                if (!device.write(firstBlock + block, in, count)) return false;
                chunk = static_cast<size_t>(count) * BLOCK;
            }
            else
            {
                if (!load(block)) return false;
                chunk = length < BLOCK - inBlock ? length : BLOCK - inBlock;
                memcpy(cache + inBlock, in, chunk);
                if (!device.write(firstBlock + block, cache, 1))
                {
                    cachedBlock = NO_BLOCK;   // на карте теперь неизвестно что
                    return false;
                }
            }
            in += chunk;
            offset += static_cast<uint32_t>(chunk);
            length -= chunk;
        }
        return true;
    }

    bool rawErase(uint32_t offset, uint32_t length)
    {
        if (offset % SECTOR_BYTES != 0 || length % SECTOR_BYTES != 0 || !inRange(offset, length)) return false;

        cachedBlock = NO_BLOCK;
        for (uint32_t done = 0; done < length;)
        {
            const uint32_t chunk = length - done < ERASE_CHUNK_BYTES ? length - done : ERASE_CHUNK_BYTES;
            if (!device.write(firstBlock + (offset + done) / BLOCK, eraseData, chunk / BLOCK)) return false;
            done += chunk;
        }
        return true;
    }


private:

    static constexpr uint32_t RESERVED_BYTES = SECTOR_BYTES;   // служебный сектор в начале файла
    static constexpr size_t MARK_LENGTH = 4;
    static uint8_t markByte(size_t i) { return static_cast<uint8_t>("OPEM"[i]); }   // метка и она же инвертированная следом

    static constexpr uint32_t BLOCK = IBlockDevice::BLOCK_SIZE;
    static constexpr uint32_t NO_BLOCK = 0xFFFFFFFFu;
    static constexpr uint32_t MAX_IO_BLOCKS = SECTOR_BYTES / BLOCK;   // один сектор за обращение к карте

    IBlockDevice& device;
    const char* name;
    uint32_t limit;

    bool ready = false;
    uint32_t firstBlock = 0;    // блок карты, с которого начинается файл
    uint32_t fileBytes = 0;
    uint32_t regionBytes = 0;

    alignas(4) uint8_t cache[BLOCK] = {};   // выровнены: драйвер SDMMC читает и пишет FIFO словами
    uint32_t cachedBlock = NO_BLOCK;   // номер блока внутри файла
    alignas(4) uint8_t eraseData[ERASE_CHUNK_BYTES];   // 0xFF

    bool inRange(uint32_t offset, size_t length) const
    {
        return ready && offset <= regionBytes && length <= regionBytes - offset;
    }

    bool inRing(uint32_t offset, size_t length) const
    {
        return ready && offset <= regionBytes - RESERVED_BYTES && length <= regionBytes - RESERVED_BYTES - offset;
    }

    static uint32_t blocksFor(size_t length)
    {
        const size_t blocks = length / BLOCK;
        return static_cast<uint32_t>(blocks < MAX_IO_BLOCKS ? blocks : MAX_IO_BLOCKS);
    }

    bool load(uint32_t block)
    {
        if (cachedBlock == block) return true;
        cachedBlock = NO_BLOCK;
        if (!device.read(firstBlock + block, cache, 1)) return false;
        cachedBlock = block;
        return true;
    }
};
