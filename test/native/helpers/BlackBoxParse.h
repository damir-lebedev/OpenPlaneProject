#pragma once

// ============================================================
// Разбор записей чёрного ящика в тестах — то же, что делает
// tools/blackbox.py: записи сектора до первой стёртой (0xFF) или
// битой (CRC-8), полёт целиком по списку секторов хранилища.
// ============================================================

#include <stdint.h>
#include <string.h>

#include <string>
#include <vector>

#include "helpers/TestSupport.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"

namespace bbparse
{
    using namespace BlackBoxFormat;

    // --------------------------------------------------------
    // Разбор записей (как tools/blackbox.py)
    // --------------------------------------------------------

    struct Rec
    {
        uint8_t type = 0;
        std::vector<uint8_t> data;

        uint32_t t() const
        {
            uint32_t v = 0;
            memcpy(&v, data.data(), 4);
            return v;
        }
        std::string text() const { return std::string(data.begin() + 4, data.end()); }
        template <typename T> T as() const
        {
            T v;
            memcpy(&v, data.data(), sizeof(T));
            return v;
        }
    };

    inline std::vector<Rec> parseSector(const uint8_t* sector, size_t* torn = nullptr)
    {
        std::vector<Rec> out;
        size_t pos = SECTOR_HEADER_SIZE;
        while (pos + RECORD_HEADER <= SECTOR_SIZE)
        {
            const uint8_t type = sector[pos];
            const uint8_t length = sector[pos + 1];
            const size_t end = pos + RECORD_HEADER + length;
            if (type == REC_ERASED || end + RECORD_CHECK > SECTOR_SIZE) break;
            if (crc8(sector + pos, RECORD_HEADER + length) != sector[end])
            {
                if (torn) (*torn)++;
                break;
            }
            Rec r;
            r.type = type;
            r.data.assign(sector + pos + 2, sector + end);
            out.push_back(r);
            pos = end + RECORD_CHECK;
        }
        return out;
    }

    inline std::vector<Rec> readFlight(BlackBoxStorage& storage, uint16_t number, size_t* torn = nullptr)
    {
        std::vector<Rec> out;
        const BlackBoxStorage::Flight* f = storage.findFlight(number);
        if (!f) return out;
        std::vector<uint8_t> sector(SECTOR_SIZE);
        for (uint32_t k = 0; k < f->sectors; ++k)
        {
            storage.readSector(storage.sectorOf(*f, k), sector.data());
            const std::vector<Rec> part = parseSector(sector.data(), torn);
            out.insert(out.end(), part.begin(), part.end());
        }
        return out;
    }

    inline size_t countType(const std::vector<Rec>& recs, uint8_t type)
    {
        size_t n = 0;
        for (const Rec& r : recs) n += r.type == type;
        return n;
    }

    inline bool hasText(const std::vector<Rec>& recs, uint8_t type, const char* fragment)
    {
        for (const Rec& r : recs)
        {
            if (r.type == type && contains(r.text(), fragment)) return true;
        }
        return false;
    }

    inline std::vector<uint8_t> record(uint8_t type, uint32_t tUs, size_t payload, uint8_t fill)
    {
        std::vector<uint8_t> r(RECORD_HEADER + payload, fill);
        r[0] = type;
        r[1] = static_cast<uint8_t>(payload);
        memcpy(r.data() + 2, &tUs, 4);
        return r;
    }

}
