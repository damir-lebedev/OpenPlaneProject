#pragma once
// ============================================================
// 6. FLIGHT OUTPUT STATE
//
// Это логическое представление того, что мы хотим отправить
// на физические исполнительные механизмы.
//
// Важный момент:
//
// ControlMixer НЕ должен знать о Servo.
//
// Он только рассчитывает:
//
//   left aileron
//   right aileron
//   elevator
//   throttle
//
// А FlightOutputs уже превращает это в PWM.
// ============================================================

struct FlightOutputState
{
    uint16_t aileronLeft  = Config::PWM_CENTER;
    uint16_t aileronRight = Config::PWM_CENTER;
    uint16_t elevator     = Config::PWM_CENTER;
    uint16_t throttle     = Config::PWM_MIN;
};