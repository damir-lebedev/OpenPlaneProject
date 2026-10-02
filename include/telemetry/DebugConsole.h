#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "hal/IBoard.h"
#include "sensors/SensorInterface.h"
#include "telemetry/DebugLogger.h"
#include "telemetry/IFlightRecorder.h"
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
//   b      — опрос шин I2C: кто отвечает и что это за чип (если
//            консоли передана плата) — первое, что делать с новой платой
//   k      — чёрный ящик: полёты, запись вручную, стереть всё
//
// Команды от ПК (tools/blackbox.py): байт STX (0x02), затем строка
// "bb ..." до '\n' — передаётся чёрному ящику (выгрузка полётов).
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

    DebugConsole(FlightController& flightController, FlightOutputs& flightOutputs,
                 Autopilot& ap, DebugLogger& debugLogger, IBoard* boardForScan = nullptr,
                 IFlightRecorder* flightRecorder = nullptr)
        : controller(flightController),
          outputs(flightOutputs),
          autopilot(ap),
          logger(debugLogger),
          board(boardForScan),
          blackBox(flightRecorder)
    {
    }

    // Чип по адресу I2C — подсказка для опроса шин ('b').
    static const char* guessI2cDevice(uint8_t address)
    {
        switch (address)
        {
            case 0x0D: return "QMC5883L";
            case 0x2C: return "QMC5883P";
            case 0x3C: case 0x3D: return "OLED SSD1306";
            case 0x46: case 0x47: return "BMP581 (0x47 — трубка Пито)";
            case 0x68: case 0x69: return "MPU6050/6500, ICM-42688/45686";
            case 0x6A: case 0x6B: return "LSM6DSV";
            case 0x76: case 0x77: return "BME280/BMP388/SPL06";
            case 0x7C: return "QMC6309";
            default: return "?";
        }
    }

    void printHint() const
    {
        Serial.println("Консоль: h — меню, l — что выводить в лог, пробел — пауза лога.");
    }

    // Что делает клавиша 'D' (заглавная): перезагрузить плату в системный
    // загрузчик USB DFU, чтобы перепрошить без кнопки BOOT0. Задаёт только
    // прошивка STM32 (src/stm32/bootloader.cpp); без хука клавиша молчит.
    // Не работает при ARM.
    void setBootloaderHook(void (*hook)()) { bootloaderHook = hook; }

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

    enum class Screen : uint8_t { None, Main, Log, BlackBox };

    static constexpr uint8_t MENU_WIDTH = 64;
    static constexpr uint8_t ITEM_WIDTH = 44;

    FlightController& controller;
    FlightOutputs& outputs;
    Autopilot& autopilot;
    DebugLogger& logger;
    IBoard* board;
    IFlightRecorder* blackBox;
    void (*bootloaderHook)() = nullptr;

    Screen screen = Screen::None;
    bool settingsDirty = false;   // настройки лога изменены, в NVS ещё не записаны
    bool confirmErase = false;    // в меню ящика нажато 'e', ждём 'y'

    static constexpr char STX = 0x02;
    bool hostLine = false;        // после STX: собирается строка команды ПК
    char hostBuffer[48] = {};
    size_t hostLength = 0;

    void handle(char key)
    {
        if (key == STX || hostLine)
        {
            collectHostCommand(key);
            return;
        }
        if (key == '\r' || key == '\n') return;
        if (key == 'D')   // из любого меню: перепрошивка не должна зависеть от того, где консоль
        {
            rebootToBootloader();
            return;
        }

        switch (screen)
        {
            case Screen::Main:     handleMainMenu(key); return;
            case Screen::Log:      handleLogMenu(key); return;
            case Screen::BlackBox: handleBlackBoxMenu(key); return;
            case Screen::None:     handleHotkey(key); return;
        }
    }

    void collectHostCommand(char key)
    {
        if (key == STX)
        {
            hostLine = true;
            hostLength = 0;
            return;
        }
        if (key == '\r') return;
        if (key != '\n')
        {
            if (hostLength + 1 < sizeof(hostBuffer)) hostBuffer[hostLength++] = key;
            return;
        }

        hostLine = false;
        hostBuffer[hostLength] = '\0';
        if (blackBox) blackBox->handleHostCommand(hostBuffer);
        else Serial.println("BB:ERR чёрного ящика нет в этой сборке");
    }

    // --- вне меню ---

    void handleHotkey(char key)
    {
        switch (key)
        {
            case 'h': case '?': openMainMenu(); break;
            case 'l':           openLogMenu(); break;
            case 'k':           openBlackBoxMenu(); break;
            case ' ':           togglePause(); break;
            case 's': case 'i': case 'o': case 'm': case 'p': case 'b':
                runAction(key);
                break;
            default:
                printHint();
                break;
        }
    }

    void rebootToBootloader()
    {
        if (!bootloaderHook) return;
        if (controller.isArmed())
        {
            Serial.println("Консоль: загрузчик недоступен, пока заармлено.");
            return;
        }
        Serial.println("Перезагрузка в загрузчик USB DFU...");
        bootloaderHook();
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
        if (board) printItem('7', "Опрос шин I2C (кто отвечает)", 'b');
        if (blackBox) printItem('8', "Чёрный ящик: полёты, запись", 'k');
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
            case '7': case 'b':
                if (!board)
                {
                    drawMainMenu();
                    return;
                }
                closeMenu();
                runAction('b');
                return;
            case '8': case 'k':
                openBlackBoxMenu();
                return;
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
        // 1..9 — первые девять каналов, NAV — 'n', SYS — 's'.
        if (channel == static_cast<uint8_t>(LogChannel::System)) return 's';
        if (channel == static_cast<uint8_t>(LogChannel::Nav)) return 'n';
        return static_cast<char>('1' + channel);
    }

    // Обратное к channelKey(): канал по клавише или LogSettings::COUNT,
    // если такой клавиши у каналов нет.
    static uint8_t channelForKey(char key)
    {
        for (uint8_t channel = 0; channel < LogSettings::COUNT; ++channel)
        {
            if (channelKey(channel) == key) return channel;
        }
        return LogSettings::COUNT;
    }

    void drawLogMenu()
    {
        const LogSettings& settings = logger.getSettings();

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

        const uint8_t channel = channelForKey(key);
        if (channel < LogSettings::COUNT) settings.cycleMode(channel);
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

    // --- меню чёрного ящика ---

    void openBlackBoxMenu()
    {
        if (!blackBox)
        {
            Serial.println("Консоль: чёрного ящика нет в этой сборке");
            return;
        }
        screen = Screen::BlackBox;
        confirmErase = false;
        logger.suspend(true);
        drawBlackBoxMenu();
    }

    void drawBlackBoxMenu()
    {
        Serial.println();
        printRule("Чёрный ящик");
        Serial.print("  ");
        blackBox->printStatus(Serial);
        blackBox->printFlights(Serial);
        Serial.println();
        Serial.println(blackBox->isRecording() ? "  r  остановить запись" : "  r  начать запись вручную (стенд)");
        Serial.println("  e  стереть все полёты");
        Serial.println("  0  назад");
        printRule("выгрузка на ПК: python tools/blackbox.py download");
    }

    void handleBlackBoxMenu(char key)
    {
        if (confirmErase)
        {
            confirmErase = false;
            if (key != 'y')
            {
                Serial.println("Стирание отменено.");
                drawBlackBoxMenu();
                return;
            }
            if (controller.isArmed() || blackBox->isRecording())
            {
                Serial.println("Консоль: нельзя, пока заармлено или идёт запись");
                return;
            }
            Serial.println("Стираю весь раздел (до ~40 с)...");
            Serial.println(blackBox->eraseAll() ? "Стёрто." : "Не удалось стереть.");
            drawBlackBoxMenu();
            return;
        }

        switch (key)
        {
            case 'r':
                if (blackBox->isRecording()) blackBox->requestManualStop();
                else blackBox->requestManualStart();
                closeMenu();
                return;
            case 'e':
                confirmErase = true;
                Serial.println("Стереть ВСЕ полёты? y — да, любая другая клавиша — нет.");
                return;
            case '0': case 'q':
                screen = Screen::Main;
                drawMainMenu();
                return;
            case 'k': case 'h':
                closeMenu();
                return;
            default:
                drawBlackBoxMenu();
                return;
        }
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
            case 'b': scanBuses(); break;
            default: break;
        }
    }

    void scanBuses() const
    {
        if (!board)
        {
            Serial.println("Консоль: опрос шин недоступен в этой сборке");
            return;
        }
        scanBus("I2C датчиков", board->i2c());
        if (II2CBus* display = board->displayI2c()) scanBus("I2C экрана", *display);
    }

    static void scanBus(const char* name, II2CBus& bus)
    {
        Serial.print(name);
        Serial.println(':');
        uint8_t found = 0;
        // До 0x7F, а не до обычных 0x77: QMC6309 сидит на 0x7C (в
        // диапазоне, зарезервированном стандартом под 10-битные адреса).
        for (uint8_t address = 0x08; address <= 0x7F; ++address)
        {
            if (!bus.probe(address)) continue;
            Serial.print("  0x");
            if (address < 0x10) Serial.print('0');
            Serial.print(address, HEX);
            Serial.print("  ");
            Serial.println(guessI2cDevice(address));
            ++found;
        }
        if (!found) Serial.println("  никого (проверьте питание, SDA/SCL и подтяжки)");
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
