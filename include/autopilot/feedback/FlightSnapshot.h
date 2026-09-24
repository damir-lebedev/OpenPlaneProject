#pragma once
#include <stdint.h>

// ============================================================
// FLIGHT SNAPSHOT — всё, что обратная связь знает о самолёте за такт
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Единая точка стыковки: при подключении FlightController заполняет
// этот снимок каждый цикл (после чтения датчиков и расчёта команд) и
// отдаёт FeedbackSupervisor. Модули обратной связи не читают датчики
// и RC напрямую — только снимок, поэтому их можно гонять и на
// записанных логах, без железа.
//
// Знаки — как во всём проекте (см. DEVELOPER_GUIDE, "Соглашение о
// знаках"): крен + правое крыло вниз, тангаж + нос вверх, рысканье
// + нос вправо; команды — мкс отклонения в тех же знаках
// (ControlCommand).
// ============================================================

struct FlightSnapshot
{
    uint32_t timeUs = 0;

    bool armed = false;
    bool linkLost = false;

    // --- Ориентация (ImuData), авиационные знаки ---
    bool imuValid = false;
    float rollDeg = 0, pitchDeg = 0, yawDeg = 0;
    float rollRateDps = 0, pitchRateDps = 0, yawRateDps = 0;
    float accelXg = 0, accelYg = 0, accelZg = 0;   // X к носу, Y влево, Z вверх

    // --- Высота ---
    bool baroValid = false;
    float altitudeM = 0;          // от точки включения
    float climbRateMs = 0;

    // Высота над землёй — будущий дальномер (лидар/сонар). Барометр
    // для выравнивания перед касанием слишком груб (дрейф ~метр).
    bool heightAglValid = false;
    float heightAglM = 0;

    // --- Скорость ---
    bool airspeedValid = false;   // будущая трубка Пито (sensors/airspeed/)
    float airspeedMs = 0;
    bool gpsValid = false;
    float groundSpeedMs = 0;

    // --- Цели выбранного режима автопилота ---
    // stabilizationActive = false (MANUAL) — рулями управляет пилот:
    // обратная связь только учится и следит за сваливанием, регуляторы
    // сброшены и рули не трогают.
    bool stabilizationActive = false;
    float targetRollDeg = 0;
    float targetPitchDeg = 0;

    // --- Команды на рули, мкс отклонения ---
    // stick* — вклад пилота; command* — итог, реально ушедший на
    // поверхности (стики + коррекции). Обратной связи нужны оба:
    // реакция самолёта зависит от итога, а ручное управление пилотом
    // нельзя путать с работой автопилота.
    float stickRollUs = 0, stickPitchUs = 0, stickYawUs = 0;
    float commandRollUs = 0, commandPitchUs = 0, commandYawUs = 0;

    // --- Газ, % ---
    float pilotThrottlePercent = 0;
    float throttlePercent = 0;    // то, что реально ушло на ESC

    // --- Закрылки ---
    int16_t flapsUs = 0;
    bool flapsMoving = false;     // выпуск меняет балансировку по тангажу
};
