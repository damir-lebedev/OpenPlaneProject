#pragma once

// ============================================================
// Образ FAT32 на фейковой SD-карте — ровно то, что нужно проверке
// Fat32::locate(): MBR (или без него), загрузочный сектор, две копии
// FAT, корневой каталог из одного-двух кластеров и файлы, которые
// лежат подряд или вразброс. Содержимое файлов — нули, тесты пишут
// туда сами.
// ============================================================

#include <stdint.h>
#include <string.h>

#include <map>
#include <string>
#include <vector>

#include "hal/IBlockDevice.h"
#include "storage/Fat32File.h"

namespace fat_image
{
    struct File
    {
        std::string name;          // "BLACKBOX.BIN"
        uint32_t bytes = 0;
        bool fragmented = false;   // кластеры через один
        bool deleted = false;      // запись каталога помечена 0xE5
    };

    struct Options
    {
        bool mbr = true;               // раздел с MBR (начало на блоке 2048), иначе FAT32 с нуля
        uint32_t perCluster = 1;       // блоков в кластере
        uint32_t rootClusters = 1;     // 2 — каталог на двух кластерах, целевые записи во втором
        uint8_t partitionType = 0x0C;  // тип раздела в MBR
    };

    struct Result
    {
        uint32_t volumeStart = 0;
        uint32_t dataStart = 0;
        std::map<std::string, uint32_t> firstBlock;   // имя файла -> блок карты, где он начинается
    };

    inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
    inline void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = (v >> (8 * i)) & 0xFF; }

    // Размер карты — как у card (card.data.size()). Старое содержимое затирается.
    template <typename Card>
    Result format(Card& card, const std::vector<File>& files, const Options& opt = Options())
    {
        constexpr uint32_t B = IBlockDevice::BLOCK_SIZE;
        const uint32_t total = static_cast<uint32_t>(card.data.size() / B);
        std::fill(card.data.begin(), card.data.end(), 0);

        Result result;
        const uint32_t start = opt.mbr ? 2048 : 0;
        const uint32_t volume = total - start;
        const uint32_t reserved = 32;
        const uint32_t fatSize = (volume / opt.perCluster + 2) * 4 / B + 1;
        const uint32_t fatStart = start + reserved;
        const uint32_t dataStart = fatStart + 2 * fatSize;
        result.volumeStart = start;
        result.dataStart = dataStart;
        const uint32_t clusterCount = (volume - reserved - 2 * fatSize) / opt.perCluster;

        auto block = [&](uint32_t n) { return card.data.data() + static_cast<size_t>(n) * B; };
        auto clusterBlock = [&](uint32_t c) { return dataStart + (c - 2) * opt.perCluster; };

        if (opt.mbr)
        {
            uint8_t* mbr = block(0);
            uint8_t* entry = mbr + 446;
            entry[4] = opt.partitionType;
            put32(entry + 8, start);
            put32(entry + 12, volume);
            mbr[510] = 0x55;
            mbr[511] = 0xAA;
        }

        uint8_t* boot = block(start);
        boot[0] = 0xEB;
        boot[1] = 0x58;
        boot[2] = 0x90;
        memcpy(boot + 3, "MSDOS5.0", 8);
        put16(boot + 11, B);
        boot[13] = static_cast<uint8_t>(opt.perCluster);
        put16(boot + 14, reserved);
        boot[16] = 2;
        put16(boot + 17, 0);
        put16(boot + 19, 0);
        boot[21] = 0xF8;
        put16(boot + 22, 0);
        put32(boot + 32, volume);
        put32(boot + 36, fatSize);
        put32(boot + 44, 2);   // корень — кластер 2
        memcpy(boot + 82, "FAT32   ", 8);
        boot[510] = 0x55;
        boot[511] = 0xAA;

        std::vector<uint32_t> fat(clusterCount + 2, 0);
        fat[0] = 0x0FFFFFF8;
        fat[1] = 0x0FFFFFFF;

        // Корень: один или два кластера.
        fat[2] = opt.rootClusters > 1 ? 3 : 0x0FFFFFFF;
        if (opt.rootClusters > 1) fat[3] = 0x0FFFFFFF;
        uint32_t next = 2 + opt.rootClusters;

        std::vector<uint8_t> entries;   // записи каталога подряд
        auto addEntry = [&](const char* name11, uint8_t attr, uint32_t cluster, uint32_t size, bool deleted)
        {
            uint8_t e[32] = {};
            memcpy(e, name11, 11);
            e[11] = attr;
            put16(e + 20, static_cast<uint16_t>(cluster >> 16));
            put16(e + 26, static_cast<uint16_t>(cluster & 0xFFFF));
            put32(e + 28, size);
            if (deleted) e[0] = 0xE5;
            entries.insert(entries.end(), e, e + 32);
        };

        // Шум перед нужными записями: метка тома, каталог, LFN, удалённый файл.
        addEntry("TESTCARD   ", 0x08, 0, 0, false);
        addEntry("SYSTEM~1   ", 0x10, 0, 0, false);
        addEntry("LFNPART1   ", 0x0F, 0, 0, false);
        addEntry("BLACKBOXBIN", 0x20, 0, 0, true);   // удалённый "старый" файл с тем же именем
        if (opt.rootClusters > 1)
        {
            // Заполнить первый кластер каталога, остальное — во втором.
            while (entries.size() < static_cast<size_t>(opt.perCluster) * B) addEntry("FILLER  TMP", 0x20, 0, 0, false);
        }

        for (const File& f : files)
        {
            uint8_t name11[11];
            Fat32::shortName(f.name.c_str(), name11);
            const uint32_t clusterBytes = opt.perCluster * B;
            const uint32_t clusters = (f.bytes + clusterBytes - 1) / clusterBytes;
            const uint32_t stride = f.fragmented ? 2 : 1;
            const uint32_t first = next;
            for (uint32_t i = 0; i < clusters; ++i)
            {
                const uint32_t c = first + i * stride;
                if (c >= fat.size()) break;   // файл больше тома: таблица "врёт", как в тесте "за краем карты"
                fat[c] = (i + 1 < clusters) ? c + stride : 0x0FFFFFFF;
            }
            next = first + clusters * stride;
            addEntry(reinterpret_cast<const char*>(name11), 0x20, clusters ? first : 0, f.bytes, f.deleted);
            result.firstBlock[f.name] = clusterBlock(first);
        }

        // Каталог в блоки.
        const size_t rootBytes = static_cast<size_t>(opt.rootClusters) * opt.perCluster * B;
        memcpy(block(clusterBlock(2)), entries.data(), std::min(entries.size(), rootBytes));

        // Обе копии FAT.
        for (uint32_t copy = 0; copy < 2; ++copy)
        {
            uint8_t* dst = block(fatStart + copy * fatSize);
            for (size_t i = 0; i < fat.size(); ++i) put32(dst + i * 4, fat[i]);
        }
        return result;
    }
}
