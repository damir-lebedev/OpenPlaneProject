// ============================================================
// Стендовый тест: программный стик руля высоты
//
// Вместо пульта прошивка сама "двигает" правый стик вверх-вниз
// (CH2, Channels::ELEVATOR) и пропускает его через тот же путь, что
// и живой стик в лётной прошивке:
//
//   RcChannelState (CH2) -> ControlMixer::fromSticks -> mix
//     -> FlightOutputs::write -> Esp32ServoOutput (LEDC, GPIO руля)
//
// Реверс, ход и пины — из Config.h. Если на пульте стик двигает
// руль, а здесь нет — разница в приёме iBUS, а не в выходах.
//
// Цикл: стик плавно на себя (руль вверх) -> постоять -> отпустить ->
// пауза -> от себя (руль вниз) -> постоять -> отпустить -> пауза.
// Так WORK_MS, потом COOL_MS стик в нейтрали — серва стоит без
// нагрузки и не греется. Ход — доля полного, чтобы тяга не упиралась
// в край. Остальные стики в нейтрали, газ на минимуме (как в лётной
// прошивке без ARM) — мотор не крутится.
//
// В крайних положениях измеряется реальный импульс на каждом выходе:
// "OK" на руле высоты, а серва стоит — дело в проводе/серве/питании.
//
// Заливка и возврат лётной прошивки — в platformio.ini рядом.
// ============================================================

#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "control/ControlMixer.h"
#include "control/FlightOutputs.h"
#include "hal/esp32/Esp32Board.h"
#include "rc/RcChannelState.h"

namespace
{
    // ---- Настройки теста -----------------------------------
    // Доля полного хода стика, отдельно в каждую сторону.
    constexpr float    UP_FRACTION   = 1.0f;   // на себя — руль вверх
    constexpr float    DOWN_FRACTION = 0.6f;   // от себя — руль вниз
    constexpr uint32_t RAMP_MS = 800;          // плавное движение стика
    constexpr uint32_t HOLD_MS = 400;          // держим в крайнем положении
    constexpr uint32_t REST_MS = 1000;         // стик отпущен между взмахами
    constexpr uint32_t WORK_MS = 20000;        // столько качаем...
    constexpr uint32_t COOL_MS = 20000;        // ...потом столько стик в нейтрали
    constexpr uint32_t STEP_MS = 20;           // один кадр, как у iBUS/серво

    // Стик "на себя" = меньше 1500 (CH2: 2000 = от себя, нос вниз).
    const uint16_t STICK_BACK    = static_cast<uint16_t>(Config::PWM_CENTER - (Config::PWM_CENTER - Config::PWM_MIN) * UP_FRACTION);
    const uint16_t STICK_FORWARD = static_cast<uint16_t>(Config::PWM_CENTER + (Config::PWM_MAX - Config::PWM_CENTER) * DOWN_FRACTION);

    Esp32Board board;
    FlightOutputs outputs(board);
    ControlMixer mixer;
    RcChannelState rc;

    uint16_t stick = Config::PWM_CENTER;
    uint32_t swings = 0;

    void applyStick(uint16_t value)
    {
        stick = value;
        rc.set(Channels::ELEVATOR, value);
        outputs.write(mixer.mix(mixer.fromSticks(rc)));
    }

    void moveStick(uint16_t target)
    {
        const int32_t from = stick;
        const int32_t steps = RAMP_MS / STEP_MS;
        for (int32_t i = 1; i <= steps; i++)
        {
            applyStick(static_cast<uint16_t>(from + (static_cast<int32_t>(target) - from) * i / steps));
            delay(STEP_MS);
        }
    }

    void holdAndReport(const char* what)
    {
        Serial.printf("  %s: стик CH2=%u -> руль высоты %u мкс\n",
                      what, stick, outputs.getLastState().elevator);
        const uint32_t start = millis();
        outputs.printPulseSelfTest();
        const uint32_t spent = millis() - start;
        if (spent < HOLD_MS) delay(HOLD_MS - spent);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("=== ТЕСТ: программный стик руля высоты (CH2) ===");

    board.begin();
    if (!outputs.begin())
    {
        Serial.println("ВНИМАНИЕ: не все обязательные выходы подключились к LEDC");
    }

    rc.reset();
    rc.set(Channels::THROTTLE, Config::PWM_MIN);
    applyStick(Config::PWM_CENTER);
    outputs.printStatus();

    Serial.printf("Стик на себя %u (%.0f%% хода), от себя %u (%.0f%%). Работа %lu с, нейтраль %lu с.\n",
                  STICK_BACK, UP_FRACTION * 100, STICK_FORWARD, DOWN_FRACTION * 100,
                  WORK_MS / 1000, COOL_MS / 1000);
    delay(1000);
}

void loop()
{
    Serial.printf("-- работа (взмахов всего: %lu)\n", swings);

    const uint32_t start = millis();
    while (millis() - start < WORK_MS)
    {
        moveStick(STICK_BACK);
        holdAndReport("на себя (руль вверх)");
        moveStick(Config::PWM_CENTER);
        delay(REST_MS);

        moveStick(STICK_FORWARD);
        holdAndReport("от себя (руль вниз)");
        moveStick(Config::PWM_CENTER);
        delay(REST_MS);

        swings++;
    }

    Serial.printf("-- отдых %lu с: стик в нейтрали\n", COOL_MS / 1000);
    delay(COOL_MS);
}
