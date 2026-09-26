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

    explicit ControlEffectivenessEstimator(uint8_t axisIndex = FeedbackConfig::AXIS_ROLL)
        : axis(axisIndex)
    {
        reset();
    }

    // Начать заново от априорной оценки.
    void reset()
    {
        const float prior = FeedbackConfig::EFFECTIVENESS_PRIOR[axis];

        theta[0] = prior;
        theta[1] = 0;
        theta[2] = 0;

        // P — ковариация оценки, нормированная на дисперсию шума.
        // Априорный разброс: b ± prior, a ± 5 1/с, c ± 100 °/с².
        for (uint8_t i = 0; i < N; ++i)
            for (uint8_t j = 0; j < N; ++j)
                P[i][j] = 0;
        P[0][0] = prior * prior / INITIAL_NOISE_VARIANCE;
        P[1][1] = 25.0f / INITIAL_NOISE_VARIANCE;
        P[2][2] = 10000.0f / INITIAL_NOISE_VARIANCE;
        for (uint8_t i = 0; i < N; ++i) maxVariance[i] = P[i][i] * MAX_VARIANCE_GROWTH;

        noiseVariance = INITIAL_NOISE_VARIANCE;
        updates = 0;
        historyCount = 0;
        intervalStarted = false;
    }

    // commandUs — отклонение руля этой оси, реально ушедшее на
    // поверхность (физические знаки ControlCommand, мкс);
    // rateDps — угловая скорость по гироскопу;
    // speedScale — (V/Vопорн)², SpeedEstimator::effectivenessScale();
    // learningAllowed — в воздухе, IMU жив, не сваливание, закрылки
    // не двигаются. Вызывать каждый цикл: интервалы оценка считает
    // сама.
    void update(float commandUs, float rateDps, float speedScale, bool learningAllowed, uint32_t nowMs)
    {
        scale = speedScale;

        if (!intervalStarted)
        {
            restartData(rateDps, nowMs);
            return;
        }

        commandSum += commandUs;
        rateSum += rateDps;
        samples++;
        intervalLearnable = intervalLearnable && learningAllowed;

        const uint32_t elapsedMs = nowMs - intervalStartMs;
        if (elapsedMs < FeedbackConfig::ESTIMATOR_PERIOD_MS) return;
        if (elapsedMs > MAX_GAP_MS)
        {
            // Долгий пропуск (цикл стоял) — интервал испорчен.
            restartData(rateDps, nowMs);
            return;
        }

        angularAccel = (rateDps - intervalStartRate) / (elapsedMs / 1000.0f);
        const float meanRate = rateSum / samples;
        pushCommand(commandSum / samples);
        const bool learn = intervalLearnable;
        startInterval(rateDps, nowMs);

        // Регрессоры в "опорных" единицах: руль × (V/Vопорн)²,
        // угловая скорость × V/Vопорн.
        prefilter(delayedCommand() * scale, meanRate * sqrtf(scale), angularAccel);
        if (!learn || !hasExcitation()) return;

        const float phi[N] = { filteredCommand, filteredRate, 1.0f };
        rlsStep(phi, filteredAccel);
    }

    // Эффективность руля на текущей скорости, со знаком, °/с² на мкс.
    float getEffectiveness() const { return theta[0] * scale; }

    // Эффективность на опорной скорости — то, что изучено о самолёте.
    float getReferenceEffectiveness() const { return theta[0]; }

    // Демпфирование на текущей скорости, 1/с (обычно < 0).
    float getDamping() const { return theta[1] * sqrtf(scale); }

    // Постоянный момент, °/с².
    float getBias() const { return theta[2]; }

    // Руль, компенсирующий постоянный момент (автотриммирование), мкс.
    float getTrimUs() const
    {
        const float b = getEffectiveness();
        return fabsf(b) >= FeedbackConfig::EFFECTIVENESS_MIN[axis] ? -theta[2] / b : 0.0f;
    }

    // Стандартное отклонение оценки b на текущей скорости.
    float getEffectivenessSigma() const { return sqrtf(noiseVariance * P[0][0]) * scale; }

    // Оценка набрала данные, её разброс мал относительно значения, и
    // рули не "мёртвые": знаку b можно верить.
    bool isConfident() const
    {
        const float magnitude = fabsf(getEffectiveness());
        return updates >= MIN_UPDATES_FOR_CONFIDENCE &&
               magnitude >= FeedbackConfig::EFFECTIVENESS_MIN[axis] &&
               getEffectivenessSigma() < CONFIDENCE_RELATIVE_SIGMA * magnitude;
    }

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
    void restartData(float rateDps, uint32_t nowMs)
    {
        startInterval(rateDps, nowMs);
        historyCount = 0;
        prefilterInitialized = false;
    }

    void startInterval(float rateDps, uint32_t nowMs)
    {
        intervalStarted = true;
        intervalStartMs = nowMs;
        intervalStartRate = rateDps;
        commandSum = 0;
        rateSum = 0;
        samples = 0;
        intervalLearnable = true;
    }

    void prefilter(float command, float rate, float accel)
    {
        if (!prefilterInitialized)
        {
            filteredCommand = command;
            filteredRate = rate;
            filteredAccel = accel;
            prefilterInitialized = true;
            return;
        }
        filteredCommand += PREFILTER_ALPHA * (command - filteredCommand);
        filteredRate += PREFILTER_ALPHA * (rate - filteredRate);
        filteredAccel += PREFILTER_ALPHA * (accel - filteredAccel);
    }

    void pushCommand(float commandUs)
    {
        history[historyHead] = commandUs;
        historyHead = (historyHead + 1) % HISTORY;
        if (historyCount < HISTORY) historyCount++;
    }

    // Команда DELAY_STEPS шагов назад (или самая старая из имеющихся).
    float delayedCommand() const
    {
        const uint8_t steps = historyCount > DELAY_STEPS ? DELAY_STEPS : historyCount - 1;
        return history[(historyHead + HISTORY - 1 - steps) % HISTORY];
    }

    // Руль за последние ~0.3 с менялся достаточно, чтобы было из чего
    // учиться.
    bool hasExcitation() const
    {
        if (historyCount < HISTORY) return false;

        float lo = history[0], hi = history[0];
        for (uint8_t i = 1; i < HISTORY; ++i)
        {
            lo = min(lo, history[i]);
            hi = max(hi, history[i]);
        }
        return hi - lo >= FeedbackConfig::MIN_EXCITATION_US;
    }

    // Один шаг RLS: θ += K·(y − φᵀθ), P = (P − K·φᵀP) / λ.
    void rlsStep(const float phi[N], float measured)
    {
        const float lambda = FeedbackConfig::RLS_FORGETTING;

        float pPhi[N];
        float denom = lambda;
        for (uint8_t i = 0; i < N; ++i)
        {
            pPhi[i] = 0;
            for (uint8_t j = 0; j < N; ++j) pPhi[i] += P[i][j] * phi[j];
            denom += phi[i] * pPhi[i];
        }
        if (denom < 1e-6f) return;

        float predicted = 0;
        for (uint8_t i = 0; i < N; ++i) predicted += theta[i] * phi[i];
        const float error = measured - predicted;

        // Дисперсия шума — по ошибке предсказания: от неё зависит,
        // насколько можно верить оценке (isConfident).
        noiseVariance += NOISE_FILTER_ALPHA * (error * error - noiseVariance);

        float gain[N];
        for (uint8_t i = 0; i < N; ++i)
        {
            gain[i] = pPhi[i] / denom;
            theta[i] += gain[i] * error;
        }

        for (uint8_t i = 0; i < N; ++i)
            for (uint8_t j = 0; j < N; ++j)
                P[i][j] = (P[i][j] - gain[i] * pPhi[j]) / lambda;

        // Численная гигиена float: симметрия, потолок дисперсий и
        // ковариации не больше, чем допускают дисперсии.
        for (uint8_t i = 0; i < N; ++i)
        {
            P[i][i] = constrain(P[i][i], 1e-12f, maxVariance[i]);
            for (uint8_t j = 0; j < i; ++j)
            {
                const float limit = sqrtf(P[i][i] * P[j][j]) * 0.999f;
                const float symmetric = constrain(0.5f * (P[i][j] + P[j][i]), -limit, limit);
                P[i][j] = P[j][i] = symmetric;
            }
        }

        const float bMax = FeedbackConfig::EFFECTIVENESS_MAX[axis];
        theta[0] = constrain(theta[0], -bMax, bMax);
        theta[1] = constrain(theta[1], DAMPING_MIN, DAMPING_MAX);

        if (updates < UINT16_MAX) updates++;
    }
};
