#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// LOG SETTINGS — что DebugLogger выводит в монитор порта
//
// Вывод разбит на каналы, у каждого свой режим:
//   OFF       — не выводить;
//   ON_CHANGE — строка печатается, только когда значения изменились
//               больше допуска (дребезг стиков и шум датчиков не в счёт);
//   PERIODIC  — раз в periodMs, даже если ничего не изменилось.
// Системная строка (SYS) — только выкл/вкл, раз в 10 с.
//
// Настраивается в меню консоли (DebugConsole: 'l'), хранится в NVS —
// переживает перезагрузку. События (ARM/DISARM, отказы, калибровки)
// печатаются всегда, это не каналы.
// ============================================================

enum class LogChannel : uint8_t
{
    Status,     // связь, ARM, режим, закрылки, датчики
    Rc,         // каналы пульта
    Outputs,    // выходы на рули и ESC
    Attitude,   // крен, тангаж, курс
    Autopilot,  // цели и коррекции автопилота
    Altitude,   // высота, вертикальная скорость
    Heading,    // курс по компасу
    Gps,        // спутники, координаты
    Imu,        // гироскоп и акселерометр
    System,     // частота цикла, память (раз в 10 с)
    Count
};

enum class LogMode : uint8_t { Off, OnChange, Periodic };

struct LogChannelInfo
{
    const char* tag;           // префикс строки в логе
    const char* title;         // описание в меню
    bool periodicOnly;         // только выкл/вкл (SYS)
    LogMode defaultMode;
};

class LogSettings
{
public:

    static constexpr uint8_t COUNT = static_cast<uint8_t>(LogChannel::Count);

    // Допустимые периоды режима PERIODIC, мс — переключаются по кругу.
    static constexpr uint8_t PERIOD_OPTIONS = 4;

    static const LogChannelInfo& info(uint8_t channel)
    {
        static const LogChannelInfo table[COUNT] = {
            { "STAT", "связь, ARM, режим, датчики",       false, LogMode::OnChange },
            { "RC",   "каналы пульта",                    false, LogMode::Off },
            { "OUT",  "выходы на рули и ESC",             false, LogMode::Off },
            { "ATT",  "углы: крен, тангаж, курс",         false, LogMode::Off },
            { "AP",   "автопилот: цели и коррекции",      false, LogMode::Off },
            { "ALT",  "высота и вертикальная скорость",   false, LogMode::Off },
            { "MAG",  "курс по компасу",                  false, LogMode::Off },
            { "GPS",  "спутники, координаты",             false, LogMode::Off },
            { "IMU",  "гироскоп и акселерометр",          false, LogMode::Off },
            { "SYS",  "цикл, память (раз в 10 с)",        true,  LogMode::Periodic },
        };
        return table[channel];
    }

    static uint16_t periodOption(uint8_t index)
    {
        static const uint16_t options[PERIOD_OPTIONS] = { 200, 500, 1000, 2000 };
        return options[index % PERIOD_OPTIONS];
    }

    LogSettings()
    {
        setDefaults();
    }

    void setDefaults()
    {
        for (uint8_t i = 0; i < COUNT; ++i) modes[i] = info(i).defaultMode;
        periodIndex = 2;   // 1 с
    }

    LogMode mode(uint8_t channel) const { return modes[channel]; }
    LogMode mode(LogChannel channel) const { return modes[static_cast<uint8_t>(channel)]; }

    void setMode(uint8_t channel, LogMode newMode)
    {
        if (info(channel).periodicOnly && newMode == LogMode::OnChange) newMode = LogMode::Periodic;
        modes[channel] = newMode;
    }

    // выкл -> при изменении -> постоянно -> выкл (SYS: выкл <-> вкл).
    void cycleMode(uint8_t channel)
    {
        switch (modes[channel])
        {
            case LogMode::Off:      setMode(channel, LogMode::OnChange); break;
            case LogMode::OnChange: setMode(channel, LogMode::Periodic); break;
            case LogMode::Periodic: setMode(channel, LogMode::Off); break;
        }
    }

    void setAll(LogMode newMode)
    {
        for (uint8_t i = 0; i < COUNT; ++i)
        {
            // SYS не трогаем "всё при изменении": у него нет такого режима.
            if (info(i).periodicOnly && newMode == LogMode::OnChange) continue;
            setMode(i, newMode);
        }
    }

    uint16_t periodMs() const { return periodOption(periodIndex); }
    void cyclePeriod() { periodIndex = (periodIndex + 1) % PERIOD_OPTIONS; }

    static const char* modeName(LogMode m, bool periodicOnly)
    {
        switch (m)
        {
            case LogMode::Off:      return "выкл";
            case LogMode::OnChange: return "при изменении";
            case LogMode::Periodic: return periodicOnly ? "вкл" : "постоянно";
        }
        return "?";
    }

    // --- NVS ---

    void load()
    {
        // На запись, хотя только читаем: иначе отсутствующее
        // пространство имён Preferences печатает как ошибку.
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) return;

        uint8_t stored[COUNT];
        const bool ok = prefs.getUChar("v", 0) == VERSION &&
                        prefs.getBytes("modes", stored, sizeof(stored)) == sizeof(stored);
        if (ok)
        {
            for (uint8_t i = 0; i < COUNT; ++i)
            {
                setMode(i, stored[i] <= static_cast<uint8_t>(LogMode::Periodic)
                               ? static_cast<LogMode>(stored[i]) : info(i).defaultMode);
            }
            periodIndex = prefs.getUChar("period", 2) % PERIOD_OPTIONS;
        }
        prefs.end();
    }

    void save() const
    {
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) return;

        uint8_t stored[COUNT];
        for (uint8_t i = 0; i < COUNT; ++i) stored[i] = static_cast<uint8_t>(modes[i]);
        prefs.putBytes("modes", stored, sizeof(stored));
        prefs.putUChar("period", periodIndex);
        prefs.putUChar("v", VERSION);
        prefs.end();
    }


private:

    static constexpr const char* NVS_NAMESPACE = "debuglog";

    // Меняется вместе со списком каналов — старые настройки сбросятся.
    static constexpr uint8_t VERSION = 1;

    LogMode modes[COUNT] = {};
    uint8_t periodIndex = 2;
};
