#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"

// ============================================================
// CONTROL EFFECTIVENESS ESTIMATOR — как самолёт реально слушается руля
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Одна ось (крен, тангаж или рысканье). Модель оси:
//
//     угловое ускорение = b · руль(t − задержка) + a · угл.скорость + c
//
//   b — эффективность руля, °/с² на мкс: сколько вращения даёт 1 мкс
//       отклонения ПРЯМО СЕЙЧАС. Почти пропадает на сваливании. Знак
//       b — направление реакции: b < 0 — руль вращает самолёт не туда
//       (перепутан реверс серво или ориентация IMU).
//   a — демпфирование, 1/с (обычно < 0): воздух тормозит вращение.
//       Без этого члена оценка b сильно врёт: при установившемся
//       вращении ускорение ≈ 0 при отклонённом руле, и регрессия
//       "ускорение от руля" дала бы b ≈ 0.
//   c — постоянный момент, °/с² (центровка, триммер, закрылки, винт):
//       руль −c/b его компенсирует — это автотриммирование.
//
// Скорость. Сила руля ∝ скоростному напору ρV²/2, демпфирование ∝ V.
// Поэтому учится не сам b, а b на опорной скорости REFERENCE_SPEED_MS:
//     b = b_опорн · (V/Vопорн)²,   a = a_опорн · (V/Vопорн).
// Разогнались — b сразу стал больше, без переобучения; обучение
// уточняет только то, чего формула не знает (конкретный самолёт,
// ветер для GPS-скорости). С трубкой Пито приборная скорость уже
// содержит плотность воздуха — высота учтена сама. Без датчика
// скорости масштаб = 1, и b просто учится как есть.
//
// Оценка — рекурсивный МНК (RLS) с забыванием: старые данные теряют
// вес (память ~4 с активного обучения).
//
// Данные — по интервалам ESTIMATOR_PERIOD_MS: среднее ускорение за
// интервал — это точно (разность гироскопа на концах) / длительность,
// и ему соответствуют СРЕДНИЕ руль и угловая скорость за тот же
// интервал. Руль берётся с задержкой RESPONSE_DELAY_MS. Затем ОБЕ
// стороны уравнения проходят через один и тот же ФНЧ
// (ESTIMATOR_PREFILTER_HZ): соотношение от этого не меняется, а
// высокие частоты, на которых модель "чистая задержка" врёт (у серво
// ещё и инерция), из оценки уходят. Фильтровать только ускорение
// нельзя: оно отстало бы от руля, и b "поехала" бы вниз.
//
// Учиться можно только в воздухе (на земле рули самолёт не вращают —
// b "выучится" нулевым) и только при "раскачке" руля: если руль стоит
// на месте, b из уравнения не определить. Без раскачки оценка просто
// замирает на последнем значении.
// ============================================================

class ControlEffectivenessEstimator
{
public:

    explicit ControlEffectivenessEstimator(uint8_t axisIndex = FeedbackConfig::AXIS_ROLL);

    // Начать заново от априорной оценки.
    void reset();

    // commandUs — отклонение руля этой оси, реально ушедшее на
    // поверхность (физические знаки ControlCommand, мкс);
    // rateDps — угловая скорость по гироскопу;
    // speedScale — (V/Vопорн)², SpeedEstimator::effectivenessScale();
    // learningAllowed — в воздухе, IMU жив, не сваливание, закрылки
    // не двигаются. Вызывать каждый цикл: интервалы оценка считает
    // сама.
    void update(float commandUs, float rateDps, float speedScale, bool learningAllowed, uint32_t nowMs);

    // Эффективность руля на текущей скорости, со знаком, °/с² на мкс.
    float getEffectiveness() const { return theta[0] * scale; }

    // Эффективность на опорной скорости — то, что изучено о самолёте.
    float getReferenceEffectiveness() const { return theta[0]; }

    // Демпфирование на текущей скорости, 1/с (обычно < 0).
    float getDamping() const { return theta[1] * sqrtf(scale); }

    // Постоянный момент, °/с².
    float getBias() const { return theta[2]; }

    // Руль, компенсирующий постоянный момент (автотриммирование), мкс.
    float getTrimUs() const;

    // Стандартное отклонение оценки b на текущей скорости.
    float getEffectivenessSigma() const { return sqrtf(noiseVariance * P[0][0]) * scale; }

    // Оценка набрала данные, её разброс мал относительно значения, и
    // рули не "мёртвые": знаку b можно верить.
    bool isConfident() const;

    // Угловое ускорение за последний интервал, °/с² — для отладки.
    float getAngularAccel() const { return angularAccel; }


private:

    static constexpr uint8_t N = 3;   // параметры: b, a, c (на опорной скорости)

    static constexpr float INITIAL_NOISE_VARIANCE = 10000.0f;   // (°/с²)², пока не измерена
    static constexpr float NOISE_FILTER_ALPHA = 0.05f;          // ~0.4 с на 50 Гц
    static constexpr float MAX_VARIANCE_GROWTH = 10.0f;         // потолок P от начальной
    static constexpr uint16_t MIN_UPDATES_FOR_CONFIDENCE = 50;  // ~1 с данных
    static constexpr float CONFIDENCE_RELATIVE_SIGMA = 0.3f;
    static constexpr uint32_t MAX_GAP_MS = 200;

    // Пределы демпфирования, 1/с: сильно отрицательное — норма
    // (крен у маленьких моделей гасится за 0.1–0.2 с), положительное
    // — неустойчивость, для самолёта маловероятна.
    static constexpr float DAMPING_MIN = -40.0f;
    static constexpr float DAMPING_MAX = 5.0f;

    // Буфер команд для задержки и оценки раскачки: 16 × 20 мс = 320 мс.
    static constexpr uint8_t HISTORY = 16;
    static constexpr uint8_t DELAY_STEPS =
        FeedbackConfig::RESPONSE_DELAY_MS / FeedbackConfig::ESTIMATOR_PERIOD_MS;
    static_assert(DELAY_STEPS < HISTORY, "RESPONSE_DELAY_MS не влезает в буфер команд");

    // Общий ФНЧ обеих сторон уравнения.
    static constexpr float PREFILTER_OMEGA_T =
        2.0f * 3.14159265f * FeedbackConfig::ESTIMATOR_PREFILTER_HZ *
        FeedbackConfig::ESTIMATOR_PERIOD_MS / 1000.0f;
    static constexpr float PREFILTER_ALPHA = PREFILTER_OMEGA_T / (1.0f + PREFILTER_OMEGA_T);

    uint8_t axis;
    float scale = 1.0f;         // (V/Vопорн)² последнего вызова

    float theta[N] = {};        // b, a, c на опорной скорости
    float P[N][N] = {};
    float maxVariance[N] = {};
    float noiseVariance = INITIAL_NOISE_VARIANCE;
    uint16_t updates = 0;

    float history[HISTORY] = {};   // средние команды интервалов, мкс
    uint8_t historyHead = 0;
    uint8_t historyCount = 0;

    // Текущий интервал.
    bool intervalStarted = false;
    uint32_t intervalStartMs = 0;
    float intervalStartRate = 0;
    float commandSum = 0;
    float rateSum = 0;
    uint16_t samples = 0;
    bool intervalLearnable = true;

    float angularAccel = 0;

    bool prefilterInitialized = false;
    float filteredCommand = 0;
    float filteredRate = 0;
    float filteredAccel = 0;

    // Данные прервались (старт, долгий пропуск цикла): задержка и
    // фильтр начинаются заново.
    void restartData(float rateDps, uint32_t nowMs);

    void startInterval(float rateDps, uint32_t nowMs);

    void prefilter(float command, float rate, float accel);

    void pushCommand(float commandUs);

    // Команда DELAY_STEPS шагов назад (или самая старая из имеющихся).
    float delayedCommand() const;

    // Руль за последние ~0.3 с менялся достаточно, чтобы было из чего
    // учиться.
    bool hasExcitation() const;

    // Один шаг RLS: θ += K·(y − φᵀθ), P = (P − K·φᵀP) / λ.
    void rlsStep(const float phi[N], float measured);
};
