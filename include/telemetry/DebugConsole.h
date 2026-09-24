#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/LogSettings.h"

// ============================================================
// DEBUG CONSOLE — текстовое меню в мониторе порта
//
// Горячие клавиши (работают всегда, Enter не обязателен):
//   h / ?  — главное меню
//   l      — меню "что выводить в лог"
//   пробел — пауза лога / продолжить
//   s      — статус датчиков
//   i      — калибровка гироскопа + предполётная проверка (2 с, не двигать)
//   o      — калибровка установки IMU (3 позы)
//   m      — калибровка компаса (15 с, вращать)
//   p      — проверка выходов (реальный импульс на каждом пине)
//
// Пока открыто меню, лог молчит (DebugLogger::suspend), чтобы меню
// не уезжало вверх; после закрытия все включённые каналы лога
// печатаются заново. Настройки лога действуют сразу, а в NVS
// записываются при закрытии меню и только без ARM: запись во флеш
// останавливает оба ядра на ~0.4 с (замерено на стенде) — заармлено,
// запись ждёт DISARM.
//
// Калибровки и проверка выходов блокируют полётный цикл на время
// выполнения, поэтому разрешены только когда мотор не заармлен.
// ============================================================

class DebugConsole
{
public:

    DebugConsole(FlightController& controller, FlightOutputs& outputs,
                 Autopilot& autopilot, DebugLogger& logger)
        : controller(controller),
          outputs(outputs),
          autopilot(autopilot),
          logger(logger)
    {
    }

    void printHint() const
    {
        Serial.println("Консоль: h — меню, l — что выводить в лог, пробел — пауза лога.");
    }

    // Вызывать каждый цикл: не блокирует, если ввода нет.
    void update()
    {
        while (Serial.available())
        {
            handle(static_cast<char>(Serial.read()));
        }

        if (settingsDirty && screen == Screen::None && !controller.isArmed())
        {
            logger.saveSettings();
            settingsDirty = false;
            Serial.println("Настройки лога сохранены.");
        }
    }


private:

    enum class Screen : uint8_t { None, Main, Log };

    static constexpr uint8_t MENU_WIDTH = 64;
    static constexpr uint8_t ITEM_WIDTH = 44;

    FlightController& controller;
    FlightOutputs& outputs;
    Autopilot& autopilot;
    DebugLogger& logger;

    Screen screen = Screen::None;
    bool settingsDirty = false;   // настройки лога изменены, в NVS ещё не записаны

    void handle(char key)
    {
        if (key == '\r' || key == '\n') return;

        switch (screen)
        {
            case Screen::Main: handleMainMenu(key); return;
            case Screen::Log:  handleLogMenu(key); return;
            case Screen::None: handleHotkey(key); return;
        }
    }

    // --- вне меню ---

    void handleHotkey(char key)
    {
        switch (key)
        {
            case 'h': case '?': openMainMenu(); break;
            case 'l':           openLogMenu(); break;
            case ' ':           togglePause(); break;
            case 's': case 'i': case 'o': case 'm': case 'p':
                runAction(key);
                break;
            default:
                printHint();
                break;
        }
    }

    // --- главное меню ---

    void openMainMenu()
    {
        screen = Screen::Main;
        logger.suspend(true);
        drawMainMenu();
    }

    void drawMainMenu() const
    {
        Serial.println();
        printRule("OpenPlane · консоль");
        printItem('1', "Лог: что выводить", 'l');
        printItem('2', "Статус датчиков", 's');
        printItem('3', "Калибровка гироскопа (2 с, не двигать)", 'i');
        printItem('4', "Калибровка установки IMU (3 позы)", 'o');
        printItem('5', "Калибровка компаса (15 с, вращать)", 'm');
        printItem('6', "Проверка выходов (импульсы на пинах)", 'p');
        Serial.println("  0  закрыть меню");
        printRule(logger.isPaused() ? "лог на паузе — пробел, чтобы продолжить"
                                    : "пробел — пауза лога");
    }

    void handleMainMenu(char key)
    {
        switch (key)
        {
            case '1': case 'l': openLogMenu(); return;
            case '2': case 's': closeMenu(); runAction('s'); return;
            case '3': case 'i': closeMenu(); runAction('i'); return;
            case '4': case 'o': closeMenu(); runAction('o'); return;
            case '5': case 'm': closeMenu(); runAction('m'); return;
            case '6': case 'p': closeMenu(); runAction('p'); return;
            case '0': case 'q': case 'h':
                closeMenu();
                return;
            case ' ':
                logger.setPaused(!logger.isPaused());
                drawMainMenu();
                return;
            default:
                drawMainMenu();
                return;
        }
    }

    void closeMenu()
    {
        screen = Screen::None;
        Serial.println("Меню закрыто (h — открыть).");
        logger.suspend(false);
    }

    // --- меню лога ---

    void openLogMenu()
    {
        screen = Screen::Log;
        logger.suspend(true);
        drawLogMenu();
    }

    static char channelKey(uint8_t channel)
    {
        // 1..9 — первые девять каналов, SYS — 's'.
        return channel < 9 ? static_cast<char>('1' + channel) : 's';
    }

    void drawLogMenu()
    {
        LogSettings& settings = logger.getSettings();

        Serial.println();
        printRule("Лог: что выводить");
        Serial.println("  клавиша канала: выкл -> при изменении -> постоянно");
        for (uint8_t channel = 0; channel < LogSettings::COUNT; ++channel)
        {
            const LogChannelInfo& info = LogSettings::info(channel);
            Serial.print("  ");
            Serial.print(channelKey(channel));
            Serial.print("  ");
            printPadded(info.tag, 5);
            printPadded(info.title, ITEM_WIDTH - 5);
            Serial.print("[");
            Serial.print(LogSettings::modeName(settings.mode(channel), info.periodicOnly));
            Serial.println("]");
        }
        Serial.println();
        Serial.print("  p  период для \"постоянно\": ");
        Serial.print(settings.periodMs() / 1000.0f, 1);
        Serial.println(" с");
        Serial.println("  a  всё \"при изменении\"    x  всё выкл    d  по умолчанию");
        Serial.println("  0  назад");
        printRule(controller.isArmed() ? "действует сразу, сохранится после DISARM"
                                       : "действует сразу, сохранится при выходе из меню");
    }

    void handleLogMenu(char key)
    {
        LogSettings& settings = logger.getSettings();

        if (key >= '1' && key <= '9' && static_cast<uint8_t>(key - '1') < LogSettings::COUNT)
        {
            settings.cycleMode(key - '1');
        }
        else if (key == 's' && LogSettings::COUNT > 9)
        {
            settings.cycleMode(9);
        }
        else if (key == 'p') settings.cyclePeriod();
        else if (key == 'a') settings.setAll(LogMode::OnChange);
        else if (key == 'x') settings.setAll(LogMode::Off);
        else if (key == 'd') settings.setDefaults();
        else if (key == '0' || key == 'q')
        {
            screen = Screen::Main;
            drawMainMenu();
            return;
        }
        else if (key == 'l' || key == 'h')
        {
            closeMenu();
            return;
        }
        else
        {
            drawLogMenu();
            return;
        }

        settingsDirty = true;
        drawLogMenu();
    }

    // --- действия ---

    void togglePause()
    {
        logger.setPaused(!logger.isPaused());
        Serial.println(logger.isPaused() ? "Лог: пауза (пробел — продолжить)." : "Лог: продолжен.");
    }

    static bool isBlocking(char action)
    {
        return action == 'i' || action == 'o' || action == 'm' || action == 'p';
    }

    void runAction(char action)
    {
        if (isBlocking(action) && controller.isArmed())
        {
            Serial.println("Консоль: команда недоступна, пока заармлено");
            return;
        }

        switch (action)
        {
            case 's': printSensorStatus(); break;
            case 'i': calibrate(autopilot.getImuSensor(), "IMU"); break;
            case 'o': calibrateImuMounting(); break;
            case 'm': calibrate(autopilot.getMagnetometerSensor(), "компас"); break;
            case 'p': outputs.printPulseSelfTest(); break;
            default: break;
        }
    }

    void printSensorStatus() const
    {
        const Sensor* sensors[] = {
            autopilot.getImuSensor(),
            autopilot.getBarometerSensor(),
            autopilot.getMagnetometerSensor(),
            autopilot.getGpsSensor(),
        };

        for (const Sensor* sensor : sensors)
        {
            if (sensor) sensor->printStatus();
        }
    }

    void calibrateImuMounting()
    {
        ImuSensor* imu = autopilot.getImuSensor();
        if (!imu)
        {
            Serial.println("Консоль: IMU не выбран в SensorSelection.h");
            return;
        }
        imu->calibrateOrientation();
    }

    template <typename SensorType>
    static void calibrate(SensorType* sensor, const char* what)
    {
        if (!sensor)
        {
            Serial.print("Консоль: ");
            Serial.print(what);
            Serial.println(" не выбран в SensorSelection.h");
            return;
        }
        sensor->calibrate();
    }

    // --- оформление ---

    // Ширина текста на экране: символы UTF-8, а не байты (кириллица —
    // 2 байта на букву).
    static uint8_t displayWidth(const char* text)
    {
        uint8_t width = 0;
        for (const char* p = text; *p; ++p)
        {
            if ((static_cast<uint8_t>(*p) & 0xC0) != 0x80) width++;
        }
        return width;
    }

    static void printPadded(const char* text, uint8_t width)
    {
        Serial.print(text);
        for (uint8_t i = displayWidth(text); i < width; ++i) Serial.print(' ');
    }

    static void printRule(const char* title)
    {
        Serial.print("══ ");
        Serial.print(title);
        Serial.print(' ');
        for (uint8_t i = displayWidth(title) + 4; i < MENU_WIDTH; ++i) Serial.print("═");
        Serial.println();
    }

    static void printItem(char key, const char* title, char hotkey)
    {
        Serial.print("  ");
        Serial.print(key);
        Serial.print("  ");
        printPadded(title, ITEM_WIDTH);
        Serial.print("[");
        Serial.print(hotkey);
        Serial.println("]");
    }
};
