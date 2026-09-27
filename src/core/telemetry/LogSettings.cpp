// Реализация telemetry/LogSettings.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/LogSettings.h"


auto LogSettings::info(uint8_t channel) -> const LogChannelInfo&
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
        { "NAV",  "дом, курс, скорость, функции",     false, LogMode::Off },
        { "SYS",  "цикл, память (раз в 10 с)",        true,  LogMode::Periodic },
    };
    return table[channel];
}

auto LogSettings::periodOption(uint8_t index) -> uint16_t
{
    static const uint16_t options[PERIOD_OPTIONS] = { 200, 500, 1000, 2000 };
    return options[index % PERIOD_OPTIONS];
}

LogSettings::LogSettings()
{
    setDefaults();
}

auto LogSettings::setDefaults() -> void
{
    for (uint8_t i = 0; i < COUNT; ++i) modes[i] = info(i).defaultMode;
    periodIndex = 2;   // 1 с
}

auto LogSettings::setMode(uint8_t channel, LogMode newMode) -> void
{
    if (info(channel).periodicOnly && newMode == LogMode::OnChange) newMode = LogMode::Periodic;
    modes[channel] = newMode;
}

auto LogSettings::cycleMode(uint8_t channel) -> void
{
    switch (modes[channel])
    {
        case LogMode::Off:      setMode(channel, LogMode::OnChange); break;
        case LogMode::OnChange: setMode(channel, LogMode::Periodic); break;
        case LogMode::Periodic: setMode(channel, LogMode::Off); break;
    }
}

auto LogSettings::setAll(LogMode newMode) -> void
{
    for (uint8_t i = 0; i < COUNT; ++i)
    {
        // SYS не трогаем "всё при изменении": у него нет такого режима.
        if (info(i).periodicOnly && newMode == LogMode::OnChange) continue;
        setMode(i, newMode);
    }
}

auto LogSettings::modeName(LogMode m, bool periodicOnly) -> const char*
{
    switch (m)
    {
        case LogMode::Off:      return "выкл";
        case LogMode::OnChange: return "при изменении";
        case LogMode::Periodic: return periodicOnly ? "вкл" : "постоянно";
    }
    return "?";
}

auto LogSettings::load() -> void
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

auto LogSettings::save() const -> void
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
