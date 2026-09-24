#pragma once
#include <stdint.h>

// ============================================================
// FEEDBACK CONFIG — настройки контура обратной связи
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА. Модули autopilot/feedback/
// пока не вызываются из FlightController/Autopilot (прототипа для
// лётных тестов ещё нет). При подключении эти константы переедут в
// config/Config.h — отдельный файл сейчас, чтобы не путать то, что
// реально летает, с тем, что ещё только готовится.
//
// Значения помеченные "прикидка" — оценки для модели ~1 кг с
// размахом 1.2 м; их надо уточнить на реальном самолёте. Модули
// обратной связи как раз и сделаны так, чтобы зависеть от этих
// чисел как можно меньше: эффективность рулей они изучают в полёте.
// ============================================================

namespace FeedbackConfig
{
    constexpr float GRAVITY = 9.80665f;

    // Оси управления: индексы в массивах по осям.
    constexpr uint8_t AXIS_ROLL  = 0;
    constexpr uint8_t AXIS_PITCH = 1;
    constexpr uint8_t AXIS_YAW   = 2;
    constexpr uint8_t AXIS_COUNT = 3;


    // --------------------------------------------------------
    // Скорость (SpeedEstimator)
    // --------------------------------------------------------

    // Скорость сваливания, м/с — прикидка; уточнить на самолёте
    // (минимальная скорость, на которой он ещё держится в горизонте).
    constexpr float STALL_SPEED_MS = 8.0f;

    // Опорная скорость, для которой задана априорная эффективность
    // рулей (EFFECTIVENESS_PRIOR ниже) — прикидка крейсерской.
    constexpr float REFERENCE_SPEED_MS = 14.0f;

    // ФНЧ продольного ускорения по IMU.
    constexpr float ACCEL_FILTER_TAU_S = 0.3f;


    // --------------------------------------------------------
    // В воздухе / на земле (AirborneDetector)
    // --------------------------------------------------------

    constexpr float AIRBORNE_HEIGHT_M = 3.0f;         // выше точки включения
    constexpr uint32_t AIRBORNE_CONFIRM_MS = 500;
    constexpr uint32_t GROUND_STILL_MS = 2000;        // стоит/катится ровно — на земле
    constexpr float GROUND_ACCEL_TOLERANCE_G = 0.1f;  // |перегрузка − 1g| на земле


    // --------------------------------------------------------
    // Каскад угол -> угловая скорость -> руль (AdaptiveRateController)
    // --------------------------------------------------------

    // Желаемая угловая скорость = ANGLE_GAIN × ошибка угла, 1/с:
    // 4 -> ошибка 10° даёт 40°/с — выравнивание примерно за 0.25 с.
    constexpr float ANGLE_GAIN[AXIS_COUNT] = { 4.0f, 4.0f, 2.0f };

    // Ограничение желаемой угловой скорости, °/с.
    constexpr float MAX_RATE_DPS[AXIS_COUNT] = { 120.0f, 60.0f, 30.0f };

    // За какое время контур скорости должен догнать желаемую скорость
    // вращения, с: желаемое угловое ускорение = ошибка скорости / TAU.
    constexpr float RATE_TAU_S[AXIS_COUNT] = { 0.15f, 0.20f, 0.30f };

    // Интегратор по ошибке угловой скорости, 1/с: "не доправил —
    // доправь ещё", пока самолёт реально не начнёт вращаться как надо.
    constexpr float RATE_INTEGRAL_GAIN[AXIS_COUNT] = { 2.0f, 2.0f, 1.0f };

    // Предел отклонения руля от контура обратной связи, мкс.
    constexpr float MAX_DEFLECTION_US[AXIS_COUNT] = { 400.0f, 400.0f, 400.0f };

    // Какую долю изученного демпфирования компенсировать рулём (0..1).
    // Полная компенсация при переоценке превращается в раскачку —
    // половина даёт запас, остальное доделывает интеграл.
    constexpr float DAMPING_COMPENSATION = 0.5f;


    // --------------------------------------------------------
    // Эффективность рулей (ControlEffectivenessEstimator)
    //
    // Модель оси: угловое ускорение = b × руль + a × угл.скорость + c.
    // b — сколько °/с² даёт 1 мкс руля ПРЯМО СЕЙЧАС (∝ V²), a —
    // демпфирование, c — постоянный момент (триммирование,
    // центровка). Всё изучается в полёте.
    // --------------------------------------------------------

    // Априорная эффективность на REFERENCE_SPEED_MS, °/с² на мкс —
    // прикидка, стартовая точка до того, как оценка наберёт данные.
    constexpr float EFFECTIVENESS_PRIOR[AXIS_COUNT] = { 3.0f, 1.5f, 0.6f };

    // Ниже MIN (на текущей скорости) рули считаются почти
    // неэффективными (сваливание, очень малая скорость); MAX — предел
    // оценки на опорной скорости.
    constexpr float EFFECTIVENESS_MIN[AXIS_COUNT] = { 0.2f, 0.1f, 0.05f };
    constexpr float EFFECTIVENESS_MAX[AXIS_COUNT] = { 30.0f, 15.0f, 6.0f };

    // Задержка между командой и откликом, мс, кратно ESTIMATOR_PERIOD_MS:
    // кадр PWM 20 мс (в среднем 10 мс ожидания) + ход серво (MG90S —
    // ~17 мс на 10°) + аэродинамика.
    constexpr uint32_t RESPONSE_DELAY_MS = 40;

    // Коэффициент забывания RLS на шаг 20 мс: 0.995 -> память ~4 с.
    constexpr float RLS_FORGETTING = 0.995f;

    // Шаг обновления оценки, мс (50 Гц).
    constexpr uint32_t ESTIMATOR_PERIOD_MS = 20;

    // Общий ФНЧ данных оценки, Гц: выше этой частоты модель "руль с
    // чистой задержкой" врёт из-за инерции серво.
    constexpr float ESTIMATOR_PREFILTER_HZ = 2.0f;

    // Минимальный размах команды за окно, чтобы оценка обучалась, мкс:
    // без "раскачки" рулём уравнение не определено, и оценка уплывёт.
    constexpr float MIN_EXCITATION_US = 30.0f;


    // --------------------------------------------------------
    // Защита от сваливания (StallGuard)
    // --------------------------------------------------------

    // Скорость падает быстрее, чем на столько м/с за секунду...
    constexpr float DECEL_WARN_MS2 = 2.0f;
    constexpr uint32_t DECEL_CONFIRM_MS = 300;

    // ...при тангаже выше этого — энергия уходит.
    constexpr float LOW_ENERGY_PITCH_DEG = 5.0f;

    // Признаки срыва: нос резко падает, хотя команда — "нос вверх";
    // крыло резко валится против команды.
    constexpr float NOSE_DROP_RATE_DPS = 60.0f;
    constexpr float WING_DROP_RATE_DPS = 120.0f;
    constexpr float STALL_NOSE_UP_COMMAND_US = 50.0f;  // руль высоты "вверх"

    // Рули почти потеряли эффективность (доля от ожидаемой на опорной
    // скорости) — ещё один признак малой скорости.
    constexpr float LOW_EFFECTIVENESS_RATIO = 0.35f;

    // Скорость ниже STALL_SPEED_MS × запас — энергии уже мало; меры
    // снимаются, только когда скорость поднялась выше второго запаса
    // (гистерезис — иначе газ дёргается на границе).
    constexpr float LOW_SPEED_MARGIN = 1.25f;
    constexpr float LOW_SPEED_EXIT_MARGIN = 1.5f;

    // Ответ на потерю энергии: газ не меньше, тангаж не выше.
    constexpr float LOW_ENERGY_THROTTLE_PERCENT = 80.0f;
    constexpr float LOW_ENERGY_MAX_PITCH_DEG = 5.0f;

    // Ответ на сваливание: полный газ, нос вниз, крылья почти ровно.
    constexpr float STALL_THROTTLE_PERCENT = 100.0f;
    constexpr float STALL_MAX_PITCH_DEG = -5.0f;
    constexpr float STALL_MAX_BANK_DEG = 10.0f;
    constexpr float STALL_AILERON_LIMIT_US = 150.0f;   // большой элерон срывает законцовку

    // Сколько держать меры после исчезновения признаков, мс.
    constexpr uint32_t RECOVERY_HOLD_MS = 1000;


    // --------------------------------------------------------
    // Взлёт (TakeoffSequencer)
    // --------------------------------------------------------

    // false — разбег по полосе (колесо и руль направления держат курс),
    // true — бросок с руки (мотор стартует только после броска).
    constexpr bool TAKEOFF_HAND_LAUNCH = false;

    constexpr float TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50.0f;  // пилот дал газ
    constexpr float TAKEOFF_THROTTLE_PERCENT = 100.0f;
    constexpr float LAUNCH_ACCEL_G = 1.0f;          // бросок с руки: продольное ускорение
    constexpr uint32_t LAUNCH_DETECT_MS = 50;
    constexpr float ROTATE_SPEED_MS = 10.0f;        // разбег: скорость отрыва — прикидка
    constexpr uint32_t ROTATE_FALLBACK_MS = 1500;   // без датчика скорости — по времени
    constexpr float CLIMB_PITCH_DEG = 12.0f;
    constexpr float TAKEOFF_TARGET_ALTITUDE_M = 30.0f;
    constexpr uint32_t TAKEOFF_CLIMB_FALLBACK_MS = 10000;   // без барометра — по времени
    constexpr uint32_t LAUNCH_TIMEOUT_MS = 8000;    // газ дали, а броска/разбега нет — отмена
    constexpr float HEADING_HOLD_GAIN = 2.0f;       // руление по курсу на разбеге, 1/с


    // --------------------------------------------------------
    // Посадка (LandingSequencer)
    // --------------------------------------------------------

    constexpr float APPROACH_SINK_RATE_MS = 1.0f;   // снижение на глиссаде
    constexpr float APPROACH_THROTTLE_PERCENT = 25.0f;
    constexpr float APPROACH_BASE_PITCH_DEG = -3.0f;
    constexpr float APPROACH_MIN_PITCH_DEG = -10.0f;
    constexpr float APPROACH_MAX_BANK_DEG = 20.0f;      // доворот пилотом на полосу
    constexpr float GO_AROUND_THROTTLE_PERCENT = 80.0f; // пилот дал газ — уход на второй круг
    constexpr float SINK_TO_PITCH_GAIN = 4.0f;      // ° тангажа на 1 м/с ошибки снижения
    constexpr float FLARE_HEIGHT_M = 2.0f;          // по дальномеру; по барометру — грубо
    // Выравнивание: снижение гасится до FLARE_SINK_RATE_MS тангажом от
    // 0 до FLARE_MAX_PITCH_DEG. Фиксированный угол не годится: самолёт
    // "вспухает", теряет скорость и падает с высоты.
    constexpr float FLARE_SINK_RATE_MS = 0.3f;
    constexpr float FLARE_MAX_PITCH_DEG = 8.0f;
    constexpr float TOUCHDOWN_ACCEL_G = 0.5f;       // удар колёс: отклонение |a| от 1g
    constexpr float TOUCHDOWN_HEIGHT_M = 0.3f;
    constexpr uint32_t TOUCHDOWN_STILL_MS = 500;
    constexpr float TOUCHDOWN_STILL_RATE_DPS = 5.0f;    // "не вращается"
    constexpr uint32_t ROLLOUT_MS = 5000;           // пробег до "посадка завершена"
}
