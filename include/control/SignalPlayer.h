#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// СИГНАЛЫ ПИЛОТУ: мотор, рули, пищалка/светодиод
//
// Чтобы этапы калибровки на земле (control/GroundCalibration.h) и отказ
// автовзлёта было слышно и видно без ноутбука. Один сигнал:
//   • рули высоты и направления кивают: SIGNAL_NOD_US вверх, потом
//     вниз, по SIGNAL_NOD_MS (в шаге 1 калибровки хвост в руке — кивок
//     чувствуется);
//   • мотор (только без ARM): импульсы на ESC пропадают на
//     SIGNAL_ESC_PAUSE_MS. Регулятор решает, что сигнал потерян, а когда
//     импульсы — нейтраль — возвращаются, играет мелодию готовности, как
//     при включении. Мотор при этом не крутится: импульсов нет вовсе
//     или нейтраль. По ШИМ другой команды "пискни" у ESC нет;
//   • пищалка или светодиод на PIN_BUZZER — на время кивка.
// Следующий сигнал — через SIGNAL_PERIOD_MS: мелодия успевает доиграть.
//
// Счёт (одинаковый везде): 1 — началось / шаг засчитан; 2 — готово;
// 3 — ошибка.
//
// Всё — функция времени от старта: FlightController каждый такт
// спрашивает, что сейчас на выходах. Пока сигнал звучит, ARM запрещён
// (ESC мог не успеть снова взвестись).
// ============================================================

class SignalPlayer
{
public:

    // count сигналов с момента nowMs. motor — можно ли трогать ESC
    // (только без ARM; FlightController всё равно перепроверяет).
    void play(uint8_t count, uint32_t nowMs, bool motor)
    {
        if (count == 0) return;
        signals = count;
        startMs = nowMs;
        motorAllowed = motor && Config::SIGNAL_MOTOR;
    }

    void stop()
    {
        signals = 0;
    }

    bool isBusy(uint32_t nowMs) const
    {
        return signals > 0 && nowMs - startMs < signals * Config::SIGNAL_PERIOD_MS;
    }

    // Сколько сигналов в текущей серии (0 — тишина).
    uint8_t getCount(uint32_t nowMs) const { return isBusy(nowMs) ? signals : 0; }

    // Импульсы на ESC убрать сейчас.
    bool escPulsesOff(uint32_t nowMs) const
    {
        return motorAllowed && isBusy(nowMs) && sinceSignal(nowMs) < Config::SIGNAL_ESC_PAUSE_MS;
    }

    // Кивок рулей, мкс (+ — нос вверх и вправо), 0 — рули не трогать.
    int16_t nodUs(uint32_t nowMs) const
    {
        if (!Config::SIGNAL_SURFACES || !isBusy(nowMs)) return 0;
        const uint32_t t = sinceSignal(nowMs);
        if (t < Config::SIGNAL_NOD_MS) return Config::SIGNAL_NOD_US;
        if (t < 2 * Config::SIGNAL_NOD_MS) return static_cast<int16_t>(-Config::SIGNAL_NOD_US);
        return 0;
    }

    // Пищалка / светодиод.
    bool light(uint32_t nowMs) const
    {
        return isBusy(nowMs) && sinceSignal(nowMs) < 2 * Config::SIGNAL_NOD_MS;
    }


private:

    uint8_t signals = 0;
    uint32_t startMs = 0;
    bool motorAllowed = false;

    // Время от начала текущего сигнала серии.
    uint32_t sinceSignal(uint32_t nowMs) const
    {
        return (nowMs - startMs) % Config::SIGNAL_PERIOD_MS;
    }
};
