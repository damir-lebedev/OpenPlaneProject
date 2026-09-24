#pragma once
#include <stdint.h>

// ============================================================
// SENSOR MOUNTING — поворот осей датчика в оси самолёта
//
// Датчик лежит микросхемой вверх, но может быть повёрнут вокруг
// вертикали — так бывает и из-за установки, и из-за самой платы
// (на клоне GY-521 чип запаян повёрнутым относительно стрелок на
// шелкографии). rotationCwDeg — куда смотрит ось X ЧИПА, если нос
// самолёта — "12 часов", по часовой стрелке при взгляде сверху:
//   0   — ось X чипа к носу
//   90  — ось X чипа вправо
//   180 — ось X чипа к хвосту
//   270 — ось X чипа влево
//
// Результат — оси самолёта X к носу, Y влево (Z вверх не меняется).
// ============================================================

namespace SensorMounting
{
    inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)
    {
        switch (rotationCwDeg)
        {
            case 90:   // ось X чипа вправо, ось Y чипа — к носу
                bodyX = chipY;   bodyY = -chipX; break;
            case 180:  // ось X чипа к хвосту
                bodyX = -chipX;  bodyY = -chipY; break;
            case 270:  // ось X чипа влево, ось Y чипа — к хвосту
                bodyX = -chipY;  bodyY = chipX;  break;
            default:   // 0: ось X чипа к носу
                bodyX = chipX;   bodyY = chipY;  break;
        }
    }
}
