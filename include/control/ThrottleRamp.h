#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// ПЛАВНЫЙ СПАД ГАЗА (Config::THROTTLE_DOWN_RAMP_MS)
//
// Газ на ESC падает не быстрее, чем 100 % за THROTTLE_DOWN_RAMP_MS;
// прибавка — сразу. Регулятор с активным торможением (двунаправленный
// BLHeli всегда такой) на резкий сброс газа тормозит винт обмотками:
// энергия винта возвращается в батарею скачком тока, напряжение на
// шине подпрыгивает, мотор встаёт рывком — бывало, что плата
// перезагружалась. С рампой винт замедляется своим сопротивлением
// воздуха, регулятору почти нечего тормозить.
//
// Рампа — в единицах газа PWM_MIN..PWM_MAX (0..100 %), без
// направления: направление меняется только без ARM, а DISARM и
// MOTOR_KILL режут газ мимо рампы (reset()) — это защита, она обязана
// останавливать мотор сразу.
// ============================================================

class ThrottleRamp
{
public:

    // targetUs — газ, который хотят пилот/автопилот; возвращает газ на ESC.
    uint16_t apply(uint16_t targetUs, uint32_t nowMs, uint32_t rampMs = Config::THROTTLE_DOWN_RAMP_MS)
    {
        const float target = static_cast<float>(targetUs);
        const uint32_t dtMs = started ? nowMs - lastMs : 0;
        started = true;
        lastMs = nowMs;

        if (rampMs == 0 || target >= current)
        {
            current = target;
        }
        else
        {
            const float step = static_cast<float>(Config::PWM_MAX - Config::PWM_MIN) * dtMs / rampMs;
            current = max(target, current - step);
        }
        return static_cast<uint16_t>(lroundf(current));
    }

    // Мотор остановлен мгновенно (DISARM, MOTOR_KILL): рампа с нуля.
    void reset()
    {
        current = Config::PWM_MIN;
    }

    // Газ ещё спадает к цели (для телеметрии и тестов).
    uint16_t currentUs() const { return static_cast<uint16_t>(lroundf(current)); }


private:

    float current = Config::PWM_MIN;
    uint32_t lastMs = 0;
    bool started = false;
};
