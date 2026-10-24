#pragma once

// ============================================================
// Разбор журнала Betaflight (.BFL) — для тестов telemetry/BflEncoder.h.
// Как Blackbox Explorer: заголовок H (поля, знаки, предсказатели и
// кодирования из самого журнала), кадры I/P/S/E. Умеет предсказатели 0
// (ноль) и 1 (предыдущее), кодирования 0 (со знаком, zigzag) и 1 (без
// знака) — то, что пишет наш кодировщик; другое — ошибка разбора.
// ============================================================

#include <stdint.h>
#include <stdlib.h>

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace bflparse
{
    struct Log
    {
        std::map<std::string, std::string> headers;
        std::vector<std::string> fields;
        std::vector<int> iPred, iEnc, pPred, pEnc;
        std::vector<std::vector<int32_t>> frames;     // главные кадры по порядку
        std::vector<char> frameTypes;                 // 'I' / 'P'
        std::vector<std::vector<uint32_t>> slow;      // кадры S
        bool ended = false;                           // E 255 "End of log"
        std::string error;
        std::string firstHeader;

        int field(const std::string& name) const
        {
            for (size_t i = 0; i < fields.size(); ++i)
            {
                if (fields[i] == name) return static_cast<int>(i);
            }
            return -1;
        }
    };

    inline std::vector<std::string> split(const std::string& s)
    {
        std::vector<std::string> out;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ',')) out.push_back(item);
        return out;
    }

    inline std::vector<int> ints(const std::string& s)
    {
        std::vector<int> out;
        for (const std::string& v : split(s)) out.push_back(atoi(v.c_str()));
        return out;
    }

    inline Log parse(const std::vector<uint8_t>& b)
    {
        Log log;
        size_t pos = 0;
        while (pos + 1 < b.size() && b[pos] == 'H' && b[pos + 1] == ' ')
        {
            size_t end = pos;
            while (end < b.size() && b[end] != '\n') ++end;
            const std::string line(b.begin() + static_cast<long>(pos) + 2, b.begin() + static_cast<long>(end));
            if (log.firstHeader.empty()) log.firstHeader = line;
            const size_t colon = line.find(':');
            if (colon != std::string::npos) log.headers[line.substr(0, colon)] = line.substr(colon + 1);
            pos = end + 1;
        }
        log.fields = split(log.headers["Field I name"]);
        log.iPred = ints(log.headers["Field I predictor"]);
        log.iEnc = ints(log.headers["Field I encoding"]);
        log.pPred = ints(log.headers["Field P predictor"]);
        log.pEnc = ints(log.headers["Field P encoding"]);
        const size_t n = log.fields.size();
        if (!n || log.iPred.size() != n || log.iEnc.size() != n || log.pPred.size() != n || log.pEnc.size() != n)
        {
            log.error = "поля заголовка не сходятся";
            return log;
        }
        const size_t slowCount = split(log.headers["Field S name"]).size();

        auto uvb = [&](uint32_t& v) {
            v = 0;
            for (int shift = 0; shift < 35; shift += 7)
            {
                if (pos >= b.size()) return false;
                const uint8_t c = b[pos++];
                v |= static_cast<uint32_t>(c & 0x7F) << shift;
                if (!(c & 0x80)) return true;
            }
            return false;
        };
        auto value = [&](int enc, int32_t& out) {
            uint32_t u = 0;
            if (!uvb(u)) return false;
            if (enc == 1) out = static_cast<int32_t>(u);
            else if (enc == 0) out = static_cast<int32_t>((u >> 1) ^ (~(u & 1) + 1));
            else return false;
            return true;
        };

        std::vector<int32_t> prev(n, 0);
        bool haveI = false;
        while (pos < b.size())
        {
            const char t = static_cast<char>(b[pos++]);
            if (t == 'I' || t == 'P')
            {
                if (t == 'P' && !haveI)
                {
                    log.error = "P до первого I";
                    return log;
                }
                const std::vector<int>& pred = t == 'I' ? log.iPred : log.pPred;
                const std::vector<int>& enc = t == 'I' ? log.iEnc : log.pEnc;
                std::vector<int32_t> v(n);
                for (size_t i = 0; i < n; ++i)
                {
                    int32_t raw = 0;
                    if (!value(enc[i], raw))
                    {
                        log.error = "обрыв или чужое кодирование";
                        return log;
                    }
                    if (pred[i] == 0) v[i] = raw;
                    else if (pred[i] == 1) v[i] = static_cast<int32_t>(static_cast<uint32_t>(prev[i]) + static_cast<uint32_t>(raw));
                    else
                    {
                        log.error = "чужой предсказатель";
                        return log;
                    }
                }
                prev = v;
                haveI = true;
                log.frames.push_back(v);
                log.frameTypes.push_back(t);
            }
            else if (t == 'S')
            {
                std::vector<uint32_t> s(slowCount);
                for (size_t i = 0; i < slowCount; ++i)
                {
                    if (!uvb(s[i]))
                    {
                        log.error = "обрыв кадра S";
                        return log;
                    }
                }
                log.slow.push_back(s);
            }
            else if (t == 'E')
            {
                if (pos < b.size() && b[pos] == 255)
                {
                    const std::string tail(b.begin() + static_cast<long>(pos) + 1, b.end());
                    log.ended = tail == std::string("End of log", 10) + '\0';
                    return log;
                }
                log.error = "чужое событие";
                return log;
            }
            else
            {
                log.error = std::string("чужой кадр ") + t;
                return log;
            }
        }
        return log;
    }
}
