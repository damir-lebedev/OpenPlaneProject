#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "hal/IBoard.h"
#include "sensors/SensorInterface.h"
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
//   b      — опрос шин I2C: кто отвечает и что это за чип (если
//            консоли передана плата) — первое, что делать с новой платой
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
                 Autopilot& ap, DebugLogger& debugLogger, IBoard* boardForScan = nullptr);

    // Чип по адресу I2C — подсказка для опроса шин ('b').
    static const char* guessI2cDevice(uint8_t address);

    void printHint() const;

    // Вызывать каждый цикл: не блокирует, если ввода нет.
    void update();


private:

    enum class Screen : uint8_t { None, Main, Log };

    static constexpr uint8_t MENU_WIDTH = 64;
    static constexpr uint8_t ITEM_WIDTH = 44;

    FlightController& controller;
    FlightOutputs& outputs;
    Autopilot& autopilot;
    DebugLogger& logger;
    IBoard* board;

    Screen screen = Screen::None;
    bool settingsDirty = false;   // настройки лога изменены, в NVS ещё не записаны

    void handle(char key);

    // --- вне меню ---

    void handleHotkey(char key);

    // --- главное меню ---

    void openMainMenu();

    void drawMainMenu() const;

    void handleMainMenu(char key);

    void closeMenu();

    // --- меню лога ---

    void openLogMenu();

    static char channelKey(uint8_t channel);

    // Обратное к channelKey(): канал по клавише или LogSettings::COUNT,
    // если такой клавиши у каналов нет.
    static uint8_t channelForKey(char key);

    void drawLogMenu();

    void handleLogMenu(char key);

    // --- действия ---

    void togglePause();

    static bool isBlocking(char action);

    void runAction(char action);

    void scanBuses() const;

    static void scanBus(const char* name, II2CBus& bus);

    void printSensorStatus() const;

    void calibrateImuMounting();

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
    static uint8_t displayWidth(const char* text);

    static void printPadded(const char* text, uint8_t width);

    static void printRule(const char* title);

    static void printItem(char key, const char* title, char hotkey);
};
