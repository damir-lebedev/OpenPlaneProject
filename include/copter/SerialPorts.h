#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "copter/Params.h"

// ============================================================
// P63 · Порты: что на каком UART — настройками serial_*_uart, без
// пересборки (раньше — только заголовок платы). Прошивка раскладывает
// UART по ролям при старте: новое значение действует после save и
// перезагрузки, как у Betaflight.
//
// Команда `serial` — как у BF: `serial <id> <функции> <скорости…>`, id 0 —
// UART1 … 7 — UART8, 20 — USB. Функции — биты BF (serialPortFunction_e).
// Понимаем MSP (+VTX_MSP), GPS, приёмник, телеметрию ESC, SmartAudio и
// Tramp; остальные биты (FrSky, MAVLink…) — «# пропущено». Скорости
// читаются и не меняются: у приёмника скорость задаёт протокол, у GPS —
// автоподбор, у MSP — 115200. Строка задаёт всё, что на этом UART: роль,
// которой нет в маске, с него снимается (как у BF), поэтому строки diff
// можно вводить по одной в любом порядке.
// ============================================================

namespace copter
{
    enum class PortRole : uint8_t
    {
        RX,
        GPS,
        MSP,
        ESC_SENSOR,
        VTX,
    };
    constexpr uint8_t PORT_ROLES = 5;
    constexpr uint8_t MAX_UARTS = 8;        // UART1…UART8
    constexpr uint8_t SERIAL_VCP_ID = 20;   // USB в `serial` BF

    namespace SerialFunction
    {
        constexpr uint32_t MSP = 1u << 0;
        constexpr uint32_t GPS = 1u << 1;
        constexpr uint32_t RX_SERIAL = 1u << 6;
        constexpr uint32_t ESC_SENSOR = 1u << 10;
        constexpr uint32_t VTX_SMARTAUDIO = 1u << 11;
        constexpr uint32_t VTX_TRAMP = 1u << 13;
        constexpr uint32_t VTX_MSP = 1u << 17;
    }

    struct PortRoleInfo
    {
        const char* title;    // по-русски, для ошибок
        uint32_t function;    // бит BF при печати
        uint32_t accepts;     // какие биты BF дают эту роль
        bool needRx;          // без ножки RX роль не работает
        bool needTx;
    };

    inline const PortRoleInfo& portRoleInfo(PortRole r)
    {
        static const PortRoleInfo table[PORT_ROLES] = {
            { "приёмник", SerialFunction::RX_SERIAL, SerialFunction::RX_SERIAL, true, false },
            { "GPS", SerialFunction::GPS, SerialFunction::GPS, true, false },
            { "MSP (очки, OSD)", SerialFunction::MSP, SerialFunction::MSP | SerialFunction::VTX_MSP, true, true },
            { "телеметрия ESC", SerialFunction::ESC_SENSOR, SerialFunction::ESC_SENSOR, true, false },
            { "видеопередатчик (SmartAudio/Tramp)", SerialFunction::VTX_SMARTAUDIO,
              SerialFunction::VTX_SMARTAUDIO | SerialFunction::VTX_TRAMP, false, true },
        };
        const uint8_t i = static_cast<uint8_t>(r);
        return table[i < PORT_ROLES ? i : 0];
    }

    inline ParamId portParam(PortRole r) { return static_cast<ParamId>(PARAM_SERIAL_RX_UART + static_cast<uint8_t>(r)); }

    // Какие UART разведены на плате: есть ли ножка RX и TX.
    struct BoardUarts
    {
        bool rx[MAX_UARTS + 1] = {};
        bool tx[MAX_UARTS + 1] = {};

        bool exists(uint8_t n) const { return n >= 1 && n <= MAX_UARTS && (rx[n] || tx[n]); }
        bool any() const
        {
            for (uint8_t n = 1; n <= MAX_UARTS; ++n)
            {
                if (exists(n)) return true;
            }
            return false;
        }
    };

    // Роль → номер UART (1…8), 0 — роли нет.
    struct PortMap
    {
        uint8_t uart[PORT_ROLES] = {};

        uint8_t& operator[](PortRole r) { return uart[static_cast<uint8_t>(r)]; }
        uint8_t operator[](PortRole r) const { return uart[static_cast<uint8_t>(r)]; }

        // Маска BF для UART n; tramp — бит VTX как Tramp (vtx_protocol = TRAMP).
        uint32_t functions(uint8_t n, bool tramp) const
        {
            uint32_t mask = 0;
            for (uint8_t i = 0; i < PORT_ROLES; ++i)
            {
                if (n == 0 || uart[i] != n) continue;
                const PortRole r = static_cast<PortRole>(i);
                mask |= r == PortRole::VTX && tramp ? SerialFunction::VTX_TRAMP : portRoleInfo(r).function;
            }
            return mask;
        }

        bool operator==(const PortMap& o) const { return memcmp(uart, o.uart, sizeof(uart)) == 0; }
    };

    inline PortMap portMap(const CopterParams& p, bool defaults = false)
    {
        PortMap m;
        for (uint8_t i = 0; i < PORT_ROLES; ++i)
        {
            const ParamId id = portParam(static_cast<PortRole>(i));
            m.uart[i] = static_cast<uint8_t>(lroundf(defaults ? p.defaultValue(id) : p.value(id)));
        }
        return m;
    }

    inline void setPortMap(CopterParams& p, const PortMap& m)
    {
        for (uint8_t i = 0; i < PORT_ROLES; ++i) p.setValue(portParam(static_cast<PortRole>(i)), 0, m.uart[i]);
    }

    // Карта годна для платы? Нет — false и текст ошибки.
    inline bool checkPortMap(const PortMap& m, const BoardUarts& b, char* error, size_t size)
    {
        for (uint8_t i = 0; i < PORT_ROLES; ++i)
        {
            const uint8_t n = m.uart[i];
            if (n == 0) continue;
            const PortRoleInfo& info = portRoleInfo(static_cast<PortRole>(i));
            if (!b.exists(n))
            {
                snprintf(error, size, "UART%u нет на плате (%s)", n, info.title);
                return false;
            }
            if (info.needRx && !b.rx[n])
            {
                snprintf(error, size, "у UART%u не разведён RX — %s не сможет принимать", n, info.title);
                return false;
            }
            if (info.needTx && !b.tx[n])
            {
                snprintf(error, size, "у UART%u не разведён TX — %s не сможет отвечать", n, info.title);
                return false;
            }
            for (uint8_t j = 0; j < i; ++j)
            {
                if (m.uart[j] != n) continue;
                snprintf(error, size, "UART%u уже занят: %s; на одном UART — одна функция", n,
                         portRoleInfo(static_cast<PortRole>(j)).title);
                return false;
            }
        }
        return true;
    }

    // «serial 1 64 115200 57600 0 115200» для UART n (1…8), id 20 — USB.
    inline void formatSerialLine(const PortMap& m, uint8_t n, bool tramp, char* out, size_t size)
    {
        const unsigned id = n == SERIAL_VCP_ID ? SERIAL_VCP_ID : n - 1u;
        const unsigned long mask = n == SERIAL_VCP_ID ? SerialFunction::MSP : m.functions(n, tramp);
        snprintf(out, size, "serial %u %lu 115200 57600 0 115200", id, mask);
    }

    struct SerialLineResult
    {
        uint8_t uart = 0;       // 1…8, SERIAL_VCP_ID — USB
        uint32_t ignored = 0;   // биты BF, которых у нас нет
    };

    // Разбор аргументов `serial` (без слова serial) в карту m. Ошибка —
    // false, текст в error, карта не тронута.
    inline bool parseSerialLine(const char* args, const BoardUarts& b, PortMap& m, SerialLineResult& out, char* error,
                                size_t size)
    {
        char* end = nullptr;
        const long id = strtol(args, &end, 10);
        if (end == args)
        {
            snprintf(error, size, "serial № функции [скорости] — № 0 = UART1, 20 = USB");
            return false;
        }
        const char* rest = end;
        const unsigned long mask = strtoul(rest, &end, 10);
        if (end == rest)
        {
            snprintf(error, size, "serial %ld: нет числа функций (биты как в BF: 64 — приёмник, 2 — GPS, 1 — MSP)", id);
            return false;
        }
        out = SerialLineResult();
        if (id == SERIAL_VCP_ID)
        {
            out.uart = SERIAL_VCP_ID;   // USB — всегда MSP и CLI
            out.ignored = static_cast<uint32_t>(mask) & ~SerialFunction::MSP;
            return true;
        }
        if (id < 0 || id >= MAX_UARTS || !b.exists(static_cast<uint8_t>(id + 1)))
        {
            snprintf(error, size, "порта %ld нет на плате (софт-UART и LPUART не поддерживаем)", id);
            return false;
        }
        const uint8_t n = static_cast<uint8_t>(id + 1);
        PortMap next = m;
        uint32_t known = 0;
        for (uint8_t i = 0; i < PORT_ROLES; ++i)
        {
            const uint32_t accepts = portRoleInfo(static_cast<PortRole>(i)).accepts;
            known |= accepts;
            if (mask & accepts) next.uart[i] = n;
            else if (next.uart[i] == n) next.uart[i] = 0;
        }
        if (!checkPortMap(next, b, error, size)) return false;
        m = next;
        out.uart = n;
        out.ignored = static_cast<uint32_t>(mask) & ~known;
        return true;
    }
}
