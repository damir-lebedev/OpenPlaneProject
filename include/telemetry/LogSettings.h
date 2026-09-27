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
    Nav,        // дом, курс, скорость, трубка Пито, тумблеры
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

    static const LogChannelInfo& info(uint8_t channel);

    static uint16_t periodOption(uint8_t index);

    LogSettings();

    void setDefaults();

    LogMode mode(uint8_t channel) const { return modes[channel]; }
    LogMode mode(LogChannel channel) const { return modes[static_cast<uint8_t>(channel)]; }

    void setMode(uint8_t channel, LogMode newMode);

    // выкл -> при изменении -> постоянно -> выкл (SYS: выкл <-> вкл).
    void cycleMode(uint8_t channel);

    void setAll(LogMode newMode);

    uint16_t periodMs() const { return periodOption(periodIndex); }
    void cyclePeriod() { periodIndex = (periodIndex + 1) % PERIOD_OPTIONS; }

    static const char* modeName(LogMode m, bool periodicOnly);

    // --- NVS ---

    void load();

    void save() const;


private:

    static constexpr const char* NVS_NAMESPACE = "debuglog";

    // Меняется вместе со списком каналов — старые настройки сбросятся.
    static constexpr uint8_t VERSION = 2;   // 2: канал NAV

    LogMode modes[COUNT] = {};
    uint8_t periodIndex = 2;
};
