#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// РЕВЕРС МОТОРА (Feature::MOTOR_REVERSE, по умолчанию — SwD)
//
// Направление мотора меняется ТОЛЬКО без ARM. Перевернуть мотор на
// полном газу — это ток короткого замыкания через ключи ESC и обмотки,
// поэтому:
//   • DISARM — направление сразу следует за тумблером;
//   • ARM — тумблер не действует: направление защёлкнуто таким, каким
//     было в момент ARM. Щёлкнули тумблер в воздухе — в консоль
//     уходит предупреждение, мотор крутится как крутился; новое
//     направление включится только после DISARM;
//   • после смены направления ARM запрещён ещё REVERSE_SETTLE_MS
//     (armBlockReason()) — мотор, снятый с ARM на газу, успевает
//     остановиться.
//
// Сам газ назад ограничивает FlightController
// (Config::REVERSE_THROTTLE_LIMIT_PCT), автопилот в реверсе газом не
// управляет. С однонаправленным ESC (Config::ESC_BIDIRECTIONAL =
// false) реверс никогда не включается.
// ============================================================

class MotorReverse
{
public:

    // requested — тумблер реверса, armed — ARM на момент вызова.
    // Вызывается только при живой связи: в failsafe-кадре положение
    // тумблера не известно.
    void update(bool requested, bool armed, uint32_t nowMs)
    {
        if (!Config::ESC_BIDIRECTIONAL) requested = false;

        if (requested == reversed)
        {
            warnedWhileArmed = false;
            return;
        }

        if (armed)
        {
            if (!warnedWhileArmed)
            {
                Serial.println("MotorReverse: реверс НЕ переключён — направление меняется только после DISARM");
                warnedWhileArmed = true;
            }
            return;
        }

        reversed = requested;
        changedAtMs = nowMs;
        changed = true;
        warnedWhileArmed = false;
        Serial.println(reversed ? "MotorReverse: мотор НАЗАД (реверс)" : "MotorReverse: мотор вперёд");
    }

    bool isReversed() const { return reversed; }

    // Тумблер уже в другом положении, а направление ещё старое (ждёт DISARM).
    bool isPending() const { return warnedWhileArmed; }

    // Почему ARM сейчас нельзя (nullptr — можно) — для ArmingManager.
    const char* armBlockReason(uint32_t nowMs) const
    {
        if (changed && nowMs - changedAtMs < Config::REVERSE_SETTLE_MS)
        {
            return "направление мотора только что сменилось — подождите, пока он остановится";
        }
        return nullptr;
    }


private:

    bool reversed = false;
    bool changed = false;
    bool warnedWhileArmed = false;
    uint32_t changedAtMs = 0;
};
