// Реализация autopilot/feedback/ControlEffectivenessEstimator.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/ControlEffectivenessEstimator.h"


ControlEffectivenessEstimator::ControlEffectivenessEstimator(uint8_t axisIndex)
: axis(axisIndex)
{
    reset();
}

auto ControlEffectivenessEstimator::reset() -> void
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

auto ControlEffectivenessEstimator::update(float commandUs, float rateDps, float speedScale, bool learningAllowed, uint32_t nowMs) -> void
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

auto ControlEffectivenessEstimator::getTrimUs() const -> float
{
    const float b = getEffectiveness();
    return fabsf(b) >= FeedbackConfig::EFFECTIVENESS_MIN[axis] ? -theta[2] / b : 0.0f;
}

auto ControlEffectivenessEstimator::isConfident() const -> bool
{
    const float magnitude = fabsf(getEffectiveness());
    return updates >= MIN_UPDATES_FOR_CONFIDENCE &&
           magnitude >= FeedbackConfig::EFFECTIVENESS_MIN[axis] &&
           getEffectivenessSigma() < CONFIDENCE_RELATIVE_SIGMA * magnitude;
}

auto ControlEffectivenessEstimator::restartData(float rateDps, uint32_t nowMs) -> void
{
    startInterval(rateDps, nowMs);
    historyCount = 0;
    prefilterInitialized = false;
}

auto ControlEffectivenessEstimator::startInterval(float rateDps, uint32_t nowMs) -> void
{
    intervalStarted = true;
    intervalStartMs = nowMs;
    intervalStartRate = rateDps;
    commandSum = 0;
    rateSum = 0;
    samples = 0;
    intervalLearnable = true;
}

auto ControlEffectivenessEstimator::prefilter(float command, float rate, float accel) -> void
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

auto ControlEffectivenessEstimator::pushCommand(float commandUs) -> void
{
    history[historyHead] = commandUs;
    historyHead = (historyHead + 1) % HISTORY;
    if (historyCount < HISTORY) historyCount++;
}

auto ControlEffectivenessEstimator::delayedCommand() const -> float
{
    const uint8_t steps = historyCount > DELAY_STEPS ? DELAY_STEPS : historyCount - 1;
    return history[(historyHead + HISTORY - 1 - steps) % HISTORY];
}

auto ControlEffectivenessEstimator::hasExcitation() const -> bool
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

auto ControlEffectivenessEstimator::rlsStep(const float phi[N], float measured) -> void
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
