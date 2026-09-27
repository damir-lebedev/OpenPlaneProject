// Реализация sensors/SensorMounting.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/SensorMounting.h"

namespace SensorMounting
{

void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)
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

}  // namespace SensorMounting
