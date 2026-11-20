#pragma once
#include <atomic>
#include <cstdint>
#include <cstdlib>

#include "config/Channels.h"
#include "config/Config.h"
#include "control/ControlCommand.h"
#include "rc/RcChannelState.h"

// ============================================================
// ОКНО ПРОШИВКИ ПО WIFI (ESP32, telemetry/WifiOta.h)
//
// Прошивка по воздуху принимается только на земле и только после
// явного «открыть окно»:
//   - клавиша 'u' в консоли или команда "ota" от OpenPlane Studio по USB;
//   - жест стиков без ARM: газ вниз, рысканье ВПРАВО, тангаж вниз (стик
//     на себя), элероны в центре — подержать OTA_STICK_HOLD_MS. Ответ —
//     кивки рулей: 4 — окно открыто, 3 — отказ (Config::OTA_WIFI выключен).
// Окно закрывается само через OTA_WINDOW_MS, по ARM и после прошивки.
// Приём начинается только без ARM и с газом внизу; ARM посреди приёма —
// приём обрывается (WifiOta зовёт Update.abort(), старая прошивка цела:
// новая пишется во вторую область app1, загрузчик переключится на неё
// только после проверки образа).
//
// Потоки: update() зовёт полётный цикл (ядро 1), beginUpload()/
// shouldAbort()/finishUpload() — веб-задача (ядро 0). Общие поля —
// атомики, блокировок нет.
// Без Arduino: тесты на ПК (test/native/test_telemetry).
// ============================================================

class OtaGate
{
public:

    enum class Result : uint8_t
    {
        Ok,
        Disabled,   // Config::OTA_WIFI = false
        Closed,     // окно не открыто или истекло
        Armed,      // заармлено
        Throttle,   // газ не внизу
        Busy,       // уже идёт приём
    };

    static const char* text(Result r)
    {
        switch (r)
        {
            case Result::Ok:       return "OK";
            case Result::Disabled: return "OTA выключена в настройках";
            case Result::Closed:   return "окно прошивки закрыто";
            case Result::Armed:    return "заармлено";
            case Result::Throttle: return "газ не внизу";
            case Result::Busy:     return "уже идёт прошивка";
        }
        return "?";
    }

    // Короткий код для Studio (ответ HTTP и строка "@ota").
    static const char* code(Result r)
    {
        switch (r)
        {
            case Result::Ok:       return "OK";
            case Result::Disabled: return "DISABLED";
            case Result::Closed:   return "CLOSED";
            case Result::Armed:    return "ARMED";
            case Result::Throttle: return "THROTTLE";
            case Result::Busy:     return "BUSY";
        }
        return "?";
    }

    // Открыть окно на OTA_WINDOW_MS (повторно — продлить).
    Result open(uint32_t nowMs)
    {
        if (!Config::OTA_WIFI) return Result::Disabled;
        if (armed.load()) return Result::Armed;
        if (!throttleLow.load()) return Result::Throttle;
        openedAtMs.store(nowMs);
        windowOpen.store(true);
        return Result::Ok;
    }

    void close()
    {
        windowOpen.store(false);
    }

    // Каждый такт полётного цикла. rcAlive = связь с пультом есть
    // (без неё жест не читается). true — окно только что открыто жестом.
    bool update(bool isArmed, const RcChannelState& rc, const ControlCommand& sticks, bool rcAlive, uint32_t nowMs)
    {
        armed.store(isArmed);
        throttleLow.store(!rcAlive || rc.get(Channels::THROTTLE) < Config::THROTTLE_LOW_US);
        lastNowMs.store(nowMs);

        if (isArmed && !receiving.load()) windowOpen.store(false);
        if (windowOpen.load() && !receiving.load() && nowMs - openedAtMs.load() >= Config::OTA_WINDOW_MS)
        {
            windowOpen.store(false);
        }

        gestureResult = Result::Ok;
        if (isArmed || !rcAlive)
        {
            holding = false;
            fired = false;
            return false;
        }
        const bool held = rc.get(Channels::THROTTLE) < Config::THROTTLE_LOW_US &&
                          sticks.yaw >= FULL * Config::RUDDER_MAX_US &&
                          sticks.pitch >= FULL * Config::ELEVATOR_MAX_US &&
                          std::abs(sticks.roll) <= CENTER * Config::AILERON_MAX_US;
        if (!held)
        {
            holding = false;
            fired = false;
            return false;
        }
        if (!holding)
        {
            holding = true;
            sinceMs = nowMs;
        }
        if (fired || nowMs - sinceMs < Config::OTA_STICK_HOLD_MS) return false;
        fired = true;
        gestureResult = open(nowMs);
        return true;
    }

    // Чем кончился жест, о котором update() вернул true.
    Result lastGesture() const { return gestureResult; }

    bool isOpen() const { return windowOpen.load(); }
    bool isReceiving() const { return receiving.load(); }
    bool isArmed() const { return armed.load(); }

    // Сколько осталось до закрытия окна (0 — закрыто).
    uint32_t remainingMs(uint32_t nowMs) const
    {
        if (!windowOpen.load()) return 0;
        const uint32_t passed = nowMs - openedAtMs.load();
        return passed >= Config::OTA_WINDOW_MS ? 0 : Config::OTA_WINDOW_MS - passed;
    }

    // Веб-задача: можно ли начинать приём образа.
    Result beginUpload()
    {
        if (!Config::OTA_WIFI) return Result::Disabled;
        if (!windowOpen.load()) return Result::Closed;
        if (armed.load()) return Result::Armed;
        if (!throttleLow.load()) return Result::Throttle;
        bool expected = false;
        if (!receiving.compare_exchange_strong(expected, true)) return Result::Busy;
        return Result::Ok;
    }

    // Посреди приёма: заармили — бросить.
    bool shouldAbort() const { return receiving.load() && armed.load(); }

    // Приём кончился. ok — образ записан и проверен: окно закрывается
    // (дальше перезагрузка); нет — окно остаётся, можно повторить.
    void finishUpload(bool ok)
    {
        receiving.store(false);
        if (ok || armed.load()) windowOpen.store(false);
        else openedAtMs.store(lastNowMs.load());   // неудача — снова полное окно
    }


private:

    static constexpr float FULL = 0.8f;     // «до упора» — 80 % хода
    static constexpr float CENTER = 0.3f;   // «в центре» — не дальше 30 %

    std::atomic<bool> armed{ false };
    std::atomic<bool> throttleLow{ false };
    std::atomic<bool> windowOpen{ false };
    std::atomic<bool> receiving{ false };
    std::atomic<uint32_t> openedAtMs{ 0 };
    std::atomic<uint32_t> lastNowMs{ 0 };

    // Жест — только полётный цикл.
    bool holding = false;
    bool fired = false;
    uint32_t sinceMs = 0;
    Result gestureResult = Result::Ok;
};
