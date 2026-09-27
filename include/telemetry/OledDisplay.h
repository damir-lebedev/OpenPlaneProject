#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "control/FlightController.h"
#include "control/FlightOutputState.h"
#include "hal/II2CBus.h"
#include "hal/Rtos.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/LoopStats.h"

// ============================================================
// OLED DISPLAY (SSD1306 128x64, I2C 0x3C)
//
// Отладочный экран состояния на отдельной шине I2C
// (IBoard::displayI2c()), чтобы отрисовка кадра (~25 мс на 400 кГц)
// не задерживала опрос датчиков. Рисуется в своей FreeRTOS-задаче
// раз в REFRESH_MS (ESP32 — ядро 0; STM32 — низкий приоритет, полётная
// задача её вытесняет, см. hal/Rtos.h); данные только читает
// (FlightController/Autopilot/LoopStats), ничего в них не меняет.
//
// U8g2 передаёт байты через собственную функцию поверх II2CBus
// (byteCallback), поэтому экран не знает, что за шиной стоит Wire1
// ESP32. Callback у U8g2 — обычная C-функция без контекста, а
// user_ptr в библиотеке включается только глобальным флагом сборки,
// меняющим её структуры, — поэтому шина хранится в статическом поле
// (экран на борту один).
//
// Контроллер проверен на стенде: SSD1306 (картинка встала ровно,
// без сдвига на 2 столбца, как было бы у SH1106).
//
//   RX ok ARM STAB FL       связь / ARM / режим / закрылки
//   R  +1.2 P  -0.4         крен / тангаж, °
//   Alt +0.3 Vz +0.1        высота, м / вертикальная скорость, м/с
//   H123 T1000 Y1500        курс / газ / руль направления, мкс
//   L1500 R1500 E1500       элероны / руль высоты, мкс
//   Loop 500Hz max 1100us   частота и худший такт цикла
// ============================================================

class OledDisplay
{
public:

    OledDisplay(FlightController& flightController, Autopilot* ap, const LoopStats& stats);

    // bus == nullptr (на плате нет второй шины) — экрана просто нет.
    bool begin(II2CBus* displayBus);


private:

    static constexpr uint8_t I2C_ADDRESS = 0x3C;
    static constexpr uint32_t REFRESH_MS = 200;

    FlightController& controller;
    Autopilot* autopilot;
    const LoopStats& loopStats;

    U8G2 display;

    // Шина экрана для byteCallback (C-колбэк U8g2 не знает об объекте).
    // Статическая локальная переменная, а не static inline член: тот
    // требует C++17, а ядро Arduino собирается с gnu++11.
    static II2CBus*& busSlot();

    // Передача байтов U8g2 поверх II2CBus. Сигнатура задана U8g2
    // (u8x8_msg_cb), поэтому u8x8 не const.
    // cppcheck-suppress constParameterCallback
    static uint8_t byteCallback(u8x8_t* u8x8, uint8_t msg, uint8_t argInt, void* argPtr);

    static void displayTask(void* arg);

    void draw();

    // Связь / ARM / режим / закрылки. Потеря связи — инверсией строки.
    void drawStatusLine();

    void drawLine(uint8_t baselineY, const String& text);

    String attitudeText() const;

    String baroText() const;

    String headingText() const;
};
