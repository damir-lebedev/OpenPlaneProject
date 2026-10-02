#pragma once
#include <stdint.h>
#include <string.h>

#include "hal/IBlockDevice.h"

// ============================================================
// FAT32, только чтение: найти файл в корне карты и сказать, где он
// лежит. Прошивка ничего не создаёт и не меняет в файловой системе —
// чёрный ящик пишет сырыми блоками ВНУТРИ заранее выделенного файла
// (tools/blackbox.py sd-prepare), поэтому карта остаётся обычной
// FAT32: файл можно скопировать на ПК, а при пропаже питания в полёте
// нечему портиться — таблицы FAT и каталог в полёте не трогаются.
//
// Условие одно: файл занимает кластеры подряд (так выглядит файл,
// созданный на пустой карте одним куском). Разбросанный файл —
// Result::Fragmented, запись выключена.
//
// Поддержано: MBR с разделом FAT32 (тип 0x0B/0x0C) или FAT32 без
// таблицы разделов; сектор 512 байт; имя файла 8.3 в корне.
// ============================================================

namespace Fat32
{
    enum class Result : uint8_t
    {
        Ok,
        NoCard,       // блочное устройство пустое
        ReadError,    // карта не прочиталась
        NotFat32,     // не FAT32 (exFAT, NTFS, пустая карта...)
        NotFound,     // в корне нет такого файла
        Fragmented,   // файл лежит не подряд
        Empty,        // файл нулевого размера
    };

    inline const char* describe(Result r)
    {
        switch (r)
        {
            case Result::Ok:         return "ок";
            case Result::NoCard:     return "карты нет";
            case Result::ReadError:  return "карта не читается";
            case Result::NotFat32:   return "не FAT32 (отформатируйте карту в FAT32)";
            case Result::NotFound:   return "файла нет в корне карты (tools/blackbox.py sd-prepare)";
            case Result::Fragmented: return "файл разбросан по карте — создайте его заново на пустой карте";
            case Result::Empty:      return "файл пустой";
        }
        return "?";
    }

    struct Extent
    {
        uint32_t firstBlock = 0;   // номер блока карты, где начинаются данные файла
        uint32_t bytes = 0;        // размер файла
    };

    // "BLACKBOX.BIN" -> "BLACKBOXBIN " (8 + 3, заглавные, пробелы). false —
    // имя не умещается в 8.3.
    inline bool shortName(const char* name, uint8_t out[11])
    {
        memset(out, ' ', 11);
        size_t at = 0;
        bool extension = false;
        for (const char* p = name; *p; ++p)
        {
            if (*p == '.')
            {
                if (extension || at == 0) return false;
                extension = true;
                at = 8;
                continue;
            }
            const size_t limit = extension ? 11 : 8;
            if (at >= limit) return false;
            out[at++] = static_cast<uint8_t>((*p >= 'a' && *p <= 'z') ? *p - 'a' + 'A' : *p);
        }
        return at > 0;
    }

    namespace detail
    {
        inline uint16_t le16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
        inline uint32_t le32(const uint8_t* p)
        {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
        }

        // Загрузочный сектор раздела FAT32 (BPB с нулевым корнем FAT16).
        inline bool isFat32Boot(const uint8_t* s)
        {
            if (s[510] != 0x55 || s[511] != 0xAA) return false;
            const uint8_t perCluster = s[13];
            return le16(s + 11) == IBlockDevice::BLOCK_SIZE && perCluster != 0 &&
                   (perCluster & (perCluster - 1)) == 0 && le16(s + 14) != 0 && s[16] != 0 &&
                   le16(s + 17) == 0 && le16(s + 22) == 0 && le32(s + 36) != 0 && le32(s + 44) >= 2;
        }
    }

    // Найти name в корне карты. На Ok в out — где лежит файл.
    inline Result locate(IBlockDevice& dev, const char* name, Extent& out)
    {
        using namespace detail;
        constexpr uint32_t END_OF_CHAIN = 0x0FFFFFF8;
        constexpr uint32_t BLOCK = IBlockDevice::BLOCK_SIZE;

        out = Extent();
        uint8_t wanted[11];
        if (!shortName(name, wanted)) return Result::NotFound;

        const uint32_t cardBlocks = dev.blockCount();
        if (cardBlocks == 0) return Result::NoCard;

        uint8_t sector[BLOCK];
        uint8_t fat[BLOCK];
        if (!dev.read(0, sector, 1)) return Result::ReadError;

        uint32_t volume = 0;
        if (!isFat32Boot(sector))
        {
            if (sector[510] != 0x55 || sector[511] != 0xAA) return Result::NotFat32;
            bool found = false;
            for (uint8_t i = 0; i < 4 && !found; ++i)
            {
                const uint8_t* entry = sector + 446 + 16 * i;
                const uint32_t start = le32(entry + 8);
                if ((entry[4] == 0x0B || entry[4] == 0x0C) && start > 0 && start < cardBlocks)
                {
                    volume = start;
                    found = true;
                }
            }
            if (!found) return Result::NotFat32;
            if (!dev.read(volume, sector, 1)) return Result::ReadError;
            if (!isFat32Boot(sector)) return Result::NotFat32;
        }

        const uint32_t perCluster = sector[13];
        const uint32_t reserved = le16(sector + 14);
        const uint32_t fatCount = sector[16];
        const uint32_t fatSize = le32(sector + 36);
        const uint32_t rootCluster = le32(sector + 44);
        const uint32_t total = le16(sector + 19) ? le16(sector + 19) : le32(sector + 32);
        const uint32_t fatStart = volume + reserved;
        const uint32_t dataStart = fatStart + fatCount * fatSize;
        if (total <= reserved + fatCount * fatSize) return Result::NotFat32;
        const uint32_t clusterCount = (total - reserved - fatCount * fatSize) / perCluster;
        const uint32_t lastCluster = clusterCount + 1;

        uint32_t fatBlock = 0xFFFFFFFF;
        // Следующий кластер цепочки; false — ошибка чтения или кластер вне тома.
        auto next = [&](uint32_t cluster, uint32_t& following, Result& error) -> bool
        {
            if (cluster < 2 || cluster > lastCluster)
            {
                error = Result::NotFat32;
                return false;
            }
            const uint32_t block = fatStart + cluster * 4 / BLOCK;
            if (block != fatBlock)
            {
                if (!dev.read(block, fat, 1))
                {
                    error = Result::ReadError;
                    return false;
                }
                fatBlock = block;
            }
            following = le32(fat + cluster * 4 % BLOCK) & 0x0FFFFFFF;
            return true;
        };
        auto clusterBlock = [&](uint32_t cluster) { return dataStart + (cluster - 2) * perCluster; };

        // Корневой каталог — цепочка кластеров; ищем запись с нужным именем.
        const uint8_t* entry = nullptr;
        uint32_t cluster = rootCluster;
        Result error = Result::NotFound;
        for (uint32_t hops = 0; hops <= lastCluster && !entry; ++hops)
        {
            for (uint32_t s = 0; s < perCluster && !entry; ++s)
            {
                if (!dev.read(clusterBlock(cluster) + s, sector, 1)) return Result::ReadError;
                for (uint32_t e = 0; e < BLOCK / 32; ++e)
                {
                    const uint8_t* candidate = sector + e * 32;
                    if (candidate[0] == 0x00) return Result::NotFound;
                    if (candidate[0] == 0xE5 || (candidate[11] & 0x3F) == 0x0F || (candidate[11] & 0x18)) continue;
                    if (memcmp(candidate, wanted, 11) == 0)
                    {
                        entry = candidate;
                        break;
                    }
                }
            }
            if (entry) break;
            uint32_t following = 0;
            if (!next(cluster, following, error)) return error;
            if (following >= END_OF_CHAIN) return Result::NotFound;
            cluster = following;
        }
        if (!entry) return Result::NotFound;

        const uint32_t size = le32(entry + 28);
        const uint32_t first = (static_cast<uint32_t>(le16(entry + 20)) << 16) | le16(entry + 26);
        if (size == 0 || first < 2) return Result::Empty;

        // Кластеры файла должны идти подряд.
        const uint32_t clusterBytes = perCluster * BLOCK;
        const uint32_t clusters = static_cast<uint32_t>((static_cast<uint64_t>(size) + clusterBytes - 1) / clusterBytes);
        cluster = first;
        for (uint32_t i = 1; i < clusters; ++i)
        {
            uint32_t following = 0;
            if (!next(cluster, following, error)) return error;
            if (following != cluster + 1) return Result::Fragmented;
            cluster = following;
        }

        const uint32_t firstBlock = clusterBlock(first);
        if (static_cast<uint64_t>(firstBlock) + (static_cast<uint64_t>(size) + BLOCK - 1) / BLOCK > cardBlocks)
        {
            return Result::NotFat32;   // файл "за краем" карты — таблица врёт
        }
        out.firstBlock = firstBlock;
        out.bytes = size;
        return Result::Ok;
    }
}
