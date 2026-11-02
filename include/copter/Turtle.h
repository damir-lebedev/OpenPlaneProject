#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Frames.h"

// ============================================================
// Turtle mode — режим FLIP OVER AFTER CRASH, как в Betaflight (по описанию
// поведения; код BF не копирован): коптер лежит на спине, пилот армится с
// этим режимом — регуляторы получают команду DShot «крутить наоборот», и
// стик выбирает, какой стороной перевернуться: работают только моторы этой
// стороны (их обратная тяга поднимает её), остальные стоят.
//
//   крен вправо — моторы справа, тангаж от себя — передние, по диагонали
//   (стик ближе 30° к диагонали) — моторы этого угла; рысканье сильнее
//   крена и тангажа — пары моторов одного вращения (развернуть на месте).
//
// Сила — отклонение стика за мёртвой зоной 15 %, с экспо crashflip_expo;
// незадействованные моторы — crashflip_motor_percent от этой силы (0 —
// стоят). Нет DShot — нет и режима: команду реверса другие протоколы не знают.
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    struct TurtleSettings
    {
        float expo = 0.35f;          // crashflip_expo / 100
        float otherMotors = 0.0f;    // crashflip_motor_percent / 100
    };

    constexpr float TURTLE_STICK_DEADBAND = 0.15f;
    constexpr float MOTOR_STOP = -1.0f;   // команда «этот мотор стоит» (меньше нуля)

    // Стики −1…1 (крен + вправо, тангаж + на себя, рысканье + нос вправо) →
    // команды моторов 0…1 или MOTOR_STOP. true — хоть один крутится.
    inline bool turtleMix(const Frame& f, float roll, float pitch, float yaw, const TurtleSettings& s, float* motor)
    {
        const float ar = fabsf(roll), ap = fabsf(pitch), ay = fabsf(yaw);
        float sr = roll > 0.0f ? 1.0f : roll < 0.0f ? -1.0f : 0.0f;
        float sp = pitch < 0.0f ? 1.0f : pitch > 0.0f ? -1.0f : 0.0f;   // от себя — вперёд
        float sy = 0.0f;
        float length = sqrtf(ar * ar + ap * ap);
        if (ay > fmaxf(ar, ap))
        {
            sr = sp = 0.0f;
            sy = yaw > 0.0f ? 1.0f : -1.0f;
            length = ay;
        }
        else if (length > 0.0f && (ar + ap) / (1.41421356f * length) < 0.8660254f)
        {
            // не диагональ — одна ось, бо́льшая
            if (ar > ap) sp = 0.0f;
            else sr = 0.0f;
        }
        const float x = fmaxf(0.0f, (fminf(1.0f, length) - TURTLE_STICK_DEADBAND) / (1.0f - TURTLE_STICK_DEADBAND));
        const float power = (1.0f - s.expo) * x + s.expo * x * x * x;
        // вклад мотора: его место по выбранным осям (рысканье — по mix.yaw)
        float most = 0.0f;
        float drive[MAX_MOTORS] = {};
        for (uint8_t i = 0; i < f.count && i < MAX_MOTORS; ++i)
        {
            drive[i] = sr * f.place[i].y + sp * f.place[i].x - sy * f.mix[i].yaw;
            most = fmaxf(most, drive[i]);
        }
        bool any = false;
        for (uint8_t i = 0; i < f.count && i < MAX_MOTORS; ++i)
        {
            float c = most > 0.0f && drive[i] > 0.0f ? power * drive[i] / most : power * s.otherMotors;
            if (power <= 0.0f || c <= 0.0f)
            {
                motor[i] = MOTOR_STOP;
                continue;
            }
            motor[i] = fminf(1.0f, c);
            any = true;
        }
        return any;
    }
}
