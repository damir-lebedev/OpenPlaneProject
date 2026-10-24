#pragma once
#include <math.h>

#include "copter/Filters.h"
#include "copter/Presets.h"

// ============================================================
// Регулятор угловой скорости одной оси — внутренний контур, на нём
// держится любой полёт мультикоптера (и акро, и все режимы с
// автоматикой — они только задают ему уставку).
//
//   P  — ошибка «уставка − гироскоп»;
//   I  — интеграл ошибки: снос ветром, кривой центр тяжести. Не копится,
//        пока коптер на земле и пока смеситель или сумма PID упёрлись
//        (anti-windup), не выходит за предел (iterm_limit); на резком
//        манёвре придерживается (I-relax: после флипа нет «отскока»); при
//        рывке газом временно сильнее (anti-gravity — нос не клюёт, P —
//        тоже, множителем снаружи);
//   D  — по гироскопу, не по ошибке: движение стика не даёт пинка;
//        через свой ФНЧ — в D больше всего шума. На резком движении
//        (пропвош, флип) растёт до d_max — в покое меньше, и моторы не
//        греются от шума;
//   FF — упреждение от скорости изменения уставки: стик начал движение —
//        моторы уже отвечают, не дожидаясь ошибки. Часть от стика — с
//        настройками BF по кадрам приёмника (feedforward_transition,
//        _averaging, _smooth_factor, _jitter_factor, _boost); у предела
//        расхода гаснет (feedforward_max_rate_limit).
// Рысканье может идти через интеграл суммы PID (use_integrated_yaw): у
// больших винтов рысканье даёт разгон винта, момент ~ скорости изменения
// команды. Absolute control (AbsoluteControl ниже) возвращает в ACRO угол,
// потерянный, пока I догонял возмущение.
// P и D умножаются на поправку по газу (TPA, Presets.h); в режиме
// TpaMode::D — только D, как в Betaflight. Сумма — не больше своего
// предела (pidsum_limit, у рысканья — pidsum_limit_yaw).
// Тонкости I и D — настройки PID-профиля, как в BF (PidExtras).
// ============================================================

namespace copter
{
    struct AxisTerms
    {
        float p = 0, i = 0, d = 0, ff = 0, sum = 0;
    };

    // d_max: при каком угловом ускорении D дорастает до d_max целиком.
    // d_max_gain 37 — 5000 °/с² (резкий манёвр, пропвош; шум гироскопа на
    // висении после ФНЧ D — в разы меньше); d_max_advance 20 — уставка
    // меняется со скоростью 2500 °/с² (стик рывком).
    constexpr float D_MAX_GAIN_SCALE = 185000.0f;
    constexpr float D_MAX_ADVANCE_SCALE = 50000.0f;
    constexpr float D_MAX_BOOST_HZ = 20.0f;      // сглаживание «насколько резко»
    constexpr float ITERM_RELAX_DPS = 40.0f;     // уставка убежала от сглаженной на столько — I стоит
    // relax по ошибке: куда уставка шла последние ~0.2 с (флип у «вялой»
    // гексы выходит на расход за 0.13–0.15 с — память длиннее, иначе
    // перелёт растёт с 11.7 до 12–18 %)
    constexpr float RELAX_ERROR_TRACK_HZ = 0.7f;
    // feedforward_boost: 1 — упреждение по ускорению стика на столько секунд
    // (15 — 3 мс: на коротком рывке стиком моторы раньше берут разгон)
    constexpr float FF_BOOST_LEAD_S = 0.0002f;
    constexpr float FF_STICK_UNITS = 500.0f;   // BF: полстика — 500 (jitter_factor)
    constexpr uint8_t FF_HISTORY = 5;          // отметок кадров для feedforward_averaging (до 4 кадров)
    // integrated yaw: сумма PID рысканья копится со скоростью 100 1/с (на
    // частоте контура рысканья ~16 Гц — тот же масштаб, что без интеграла);
    // integrated_yaw_relax — утечка: 200 — 16 1/с (срез ~2.5 Гц)
    constexpr float INTEGRATED_YAW_GAIN = 100.0f;
    constexpr float INTEGRATED_YAW_LEAK = 0.08f;

    // Коэффициенты для интеграла суммы PID рысканья, с которыми контур тот
    // же, что у g без интеграла (интеграл с утечкой λ — K/(s+λ), значит
    // PID надо умножить на (s+λ)/K): P' = (I + λP)/K, I' = λI/K,
    // D' = (P + λD)/K, F' = (P + λF)/K. Пропадают только D·s² и F·s² — у
    // рысканья из модели D нет, упреждение остаётся по уставке.
    inline AxisGains integratedYawGains(const AxisGains& g, float relax)
    {
        const float k = INTEGRATED_YAW_GAIN, l = relax * INTEGRATED_YAW_LEAK;
        AxisGains r = g;
        r.p = (g.i + l * g.p) / k;
        r.i = l * g.i / k;
        r.d = (g.p + l * g.d) / k;
        r.ff = (g.p + l * g.ff) / k;
        return r;
    }

    // Упреждение с пульта (feedforward_*). Что из уставки задал стик и когда
    // пришёл кадр приёмника — настройки BF работают на этой части; остальное
    // (уставка от горизонта в ANGLE) идёт в FF как есть.
    struct FeedforwardInput
    {
        float stickSetpoint = 0.0f;   // °/с: часть уставки, которую задаёт стик
        float deflection = 1.0f;      // |стик| после сглаживания, 0..1 (feedforward_transition)
        float rawStick = 0.0f;        // стик как пришёл с пульта, −1..1 (feedforward_jitter_factor)
        bool newFrame = true;         // в этом цикле пришёл кадр приёмника
        float framePeriodS = 0.0f;    // период кадров, с (0 — каждый цикл)
    };

    class RateAxis
    {
    public:
        void configure(const AxisGains& g, float dtermLpfHz, float outLpfHz, float iLimit, float dtS)
        {
            gains = g;
            limit = iLimit;
            dt = dtS;
            dLpf.configure(dtermLpfHz, dtS);
            outLpf.configure(outLpfHz, dtS);
            ffLpf.configure(25.0f, dtS);
            ffShapedLpf.configure(25.0f, dtS);
            boostLpf.configure(D_MAX_BOOST_HZ, dtS);
            relaxLpf.configure(relaxCutoffHz, dtS);
            spTrack.configure(RELAX_ERROR_TRACK_HZ, dtS);
            reset();
        }

        // Тонкости PID-профиля для этой оси (0 крен, 1 тангаж, 2 рысканье); на
        // ходу — интеграл и фильтры не сбрасываются.
        void setExtras(const PidExtras& x, uint8_t axis)
        {
            dMax = x.dMax[axis < 3 ? axis : 2];
            dMaxGain = x.dMaxGain;
            dMaxAdvance = x.dMaxAdvance;
            const bool yaw = axis == 2;
            relaxOn = x.relax == ItermRelax::RPY || x.relax == ItermRelax::RPY_INC ||
                      (!yaw && (x.relax == ItermRelax::RP || x.relax == ItermRelax::RP_INC));
            relaxIncOnly = x.relax == ItermRelax::RP_INC || x.relax == ItermRelax::RPY_INC;
            relaxGyro = x.relaxType == ItermRelaxType::GYRO;
            relaxError = x.relaxError;
            relaxCutoffHz = x.relaxCutoffHz;
            relaxLpf.configure(relaxCutoffHz, dt);
            sumLimit = yaw ? x.pidSumLimitYaw : x.pidSumLimit;
            tpaOnP = x.tpaMode != TpaMode::D;
            ffTransition = x.ffTransition;
            ffAveraging = x.ffAveraging < 1 ? 1 : (x.ffAveraging > FF_HISTORY - 1 ? FF_HISTORY - 1 : x.ffAveraging);
            ffSmooth = fminf(0.95f, fmaxf(0.0f, x.ffSmoothFactor));
            ffJitter = x.ffJitterFactor;
            ffBoost = x.ffBoost;
            ffShaping = ffTransition > 0.0f || ffAveraging > 1 || ffSmooth > 0.0f || ffJitter > 0.0f || ffBoost > 0.0f;
            ffLimit = x.ffMaxRateLimit;
            const bool wasIntegrated = integrated;
            integrated = yaw && x.integratedYaw;
            integratedLeak = x.integratedYawRelax * INTEGRATED_YAW_LEAK;
            if (integrated && !wasIntegrated) integratedSum = terms.sum;   // включили на ходу — без рывка
        }

        // Расход при полном стике, °/с (feedforward_max_rate_limit).
        void setMaxRate(float dps) { maxRate = fabsf(dps); }

        // ФНЧ выхода (yaw_lowpass_hz) на ходу: 0 — без фильтра.
        void setOutputLpf(float hz) { outLpf.configure(hz, dt); }

        // Стик и кадр приёмника для упреждения — перед update(), каждый цикл.
        // Не задано — вся уставка от стика, кадр — каждый цикл, без
        // transition и jitter.
        void setFeedforwardInput(const FeedforwardInput& in)
        {
            ffIn = in;
            ffInKnown = true;
        }

        void reset()
        {
            terms = AxisTerms();
            iTerm = 0.0f;
            boost = 0.0f;
            sumClamped = false;
            prevMeasured = 0.0f;
            prevSetpoint = 0.0f;
            first = true;
            dLpf.reset();
            outLpf.reset();
            ffLpf.reset();
            ffShapedLpf.reset();
            ffFirst = true;
            integratedSum = 0.0f;
            boostLpf.reset();
            relaxLpf.reset();
            spTrack.reset();
        }

        void resetIntegral() { iTerm = 0.0f; }

        // Интеграл как есть — для поворота вместе с рамой (iterm_rotation).
        float integral() const { return iTerm; }
        void setIntegral(float v) { iTerm = fminf(limit, fmaxf(-limit, v)); }

        // Другие коэффициенты на ходу (смена профиля): интеграл и фильтры не сбрасываются.
        void setGains(const AxisGains& g) { gains = g; }
        void setLimit(float iLimit) { limit = iLimit; }
        void setTpaMode(TpaMode mode) { tpaOnP = mode != TpaMode::D; }

        // setpoint, measured — °/с. tpa — множитель D (и P, кроме TpaMode::D); iBoost —
        // множитель I (anti-gravity, iterm_windup); holdI — интеграл не копить
        // (земля); saturated — смеситель упёрся: интеграл не растёт в сторону упора;
        // pBoost — множитель P (anti-gravity).
        float update(float setpoint, float measured, float tpa, float iBoost, bool holdI, bool saturated,
                     float pBoost = 1.0f)
        {
            if (first)
            {
                prevMeasured = measured;
                prevSetpoint = setpoint;
                relaxLpf.reset(setpoint);
                spTrack.reset(setpoint);
                first = false;
            }
            const float error = setpoint - measured;
            const float smooth = relaxLpf.apply(setpoint);
            const float moved = fabsf(setpoint - spTrack.apply(setpoint));   // уставка недавно сдвинулась на столько

            terms.p = gains.p * error * (tpaOnP ? tpa : 1.0f) * pBoost;

            if (!holdI)
            {
                const float raw = gains.i * error * dt * iBoost;
                float delta = gains.i * relaxedError(setpoint, measured, error, smooth, moved) * dt * iBoost;
                // *_INC: уменьшаться интеграл может всегда, придерживается только рост
                if (relaxIncOnly && iTerm != 0.0f && (raw > 0.0f) != (iTerm > 0.0f)) delta = raw;
                const bool pushesIntoLimit = (saturated || sumClamped) && ((delta > 0.0f) == (terms.sum > 0.0f));
                if (!pushesIntoLimit) iTerm = fminf(limit, fmaxf(-limit, iTerm + delta));
            }
            terms.i = iTerm;

            const float spSpeed = (setpoint - prevSetpoint) / dt;   // °/с² уставки
            const float spRate = ffLpf.apply(spSpeed);
            prevSetpoint = setpoint;
            // упреждение: без настроек feedforward_* — та же скорость уставки
            const float ffSpeed = feedforwardSpeed(setpoint, spSpeed);
            float ffRate = spRate;
            if (ffShaping)
                ffRate = ffShapedLpf.apply(ffSpeed);
            else
                ffShapedLpf.reset(spRate);

            const float rate = dLpf.apply((measured - prevMeasured) / dt);       // °/с² гироскопа
            prevMeasured = measured;
            float d = gains.d;
            if (dMax > gains.d)
            {
                const float byGyro = fabsf(rate) * dMaxGain / D_MAX_GAIN_SCALE;
                const float bySetpoint = fabsf(spRate) * dMaxAdvance / D_MAX_ADVANCE_SCALE;
                boost = boostLpf.apply(fminf(1.0f, fmaxf(byGyro, bySetpoint)));
                d = gains.d + (dMax - gains.d) * boost;
            }
            else
            {
                boost = 0.0f;
            }
            terms.d = -d * rate * tpa;

            terms.ff = limitFeedforward(gains.ff * ffRate, setpoint);

            const float sum = terms.p + terms.i + terms.d + terms.ff;
            sumClamped = fabsf(sum) > sumLimit;
            float out = fminf(sumLimit, fmaxf(-sumLimit, sum));
            if (integrated)
            {
                // выход — интеграл суммы с утечкой, в пределе суммы
                integratedSum += (INTEGRATED_YAW_GAIN * sum - integratedLeak * integratedSum) * dt;
                integratedSum = fminf(sumLimit, fmaxf(-sumLimit, integratedSum));
                if (fabsf(integratedSum) >= sumLimit) sumClamped = true;
                out = integratedSum;
            }
            terms.sum = outLpf.apply(out);
            return terms.sum;
        }

        const AxisTerms& lastTerms() const { return terms; }
        const AxisGains& getGains() const { return gains; }
        // Насколько D сейчас поднят к d_max: 0 — базовый D, 1 — d_max.
        float dBoost() const { return boost; }

    private:
        // feedforward_max_rate_limit: уставка у края расхода — упреждение
        // наружу не больше того, что дал бы P на оставшийся до края запас;
        // за краем — ноль. Иначе на флипе полным стиком FF разгоняет раму
        // мимо расхода. К центру (стик отпускают) — как есть.
        float limitFeedforward(float ff, float setpoint) const
        {
            if (ffLimit <= 0.0f || maxRate <= 0.0f || ff * setpoint <= 0.0f) return ff;
            const float room = ffLimit * maxRate - fabsf(setpoint);
            if (room <= 0.0f) return 0.0f;
            return copysignf(fminf(fabsf(ff), gains.p * room), ff);
        }

        // Скорость уставки для упреждения (°/с²) с настройками feedforward_*.
        // Они действуют на часть уставки от стика: transition — у центра
        // стика FF слабее; averaging — скорость стика средняя за N кадров
        // приёмника (кадр, в котором стоим, — тоже); smooth_factor — сглаживание от кадра к кадру (доля
        // старого за кадр, в пересчёте на циклы); jitter_factor — стик за
        // кадр (в среднем за два) сдвинулся меньше порога — FF слабее,
        // плавно до нуля (дрожь пальца, медленный стик); boost — добавка по
        // ускорению стика. Уставка не от стика (горизонт) — как есть.
        float feedforwardSpeed(float setpoint, float spSpeed)
        {
            const float stickSp = ffInKnown ? ffIn.stickSetpoint : setpoint;
            if (ffFirst)
            {
                prevStickSp = stickSp;
                for (uint8_t k = 0; k < FF_HISTORY; ++k)
                {
                    ffHist[k] = stickSp;
                    ffHistAge[k] = 0.0f;
                }
                ffHistCount = 0;
                ffSmoothed = 0.0f;
                prevFfSmoothed = 0.0f;
                prevRawStick = ffIn.rawStick;
                prevStickDelta = FF_STICK_UNITS;
                jitterAtt = 1.0f;
                ffFirst = false;
            }
            const float stickSpeed = (stickSp - prevStickSp) / dt;
            prevStickSp = stickSp;
            if (!ffShaping) return spSpeed;
            for (uint8_t k = 0; k < FF_HISTORY; ++k) ffHistAge[k] += dt;

            const bool frame = !ffInKnown || ffIn.newFrame;
            if (frame)
            {
                for (uint8_t k = FF_HISTORY - 1; k > 0; --k)
                {
                    ffHist[k] = ffHist[k - 1];
                    ffHistAge[k] = ffHistAge[k - 1];
                }
                ffHist[0] = stickSp;
                ffHistAge[0] = 0.0f;
                if (ffHistCount < FF_HISTORY) ++ffHistCount;
                const float period = ffInKnown && ffIn.framePeriodS > dt ? ffIn.framePeriodS : dt;
                smoothAlpha = ffSmooth > 0.0f ? 1.0f - powf(ffSmooth, dt / period) : 1.0f;
                jitterAtt = 1.0f;
                if (ffInKnown)
                {
                    const float delta = fabsf(ffIn.rawStick - prevRawStick) * FF_STICK_UNITS;
                    const float moved = 0.5f * (delta + prevStickDelta);
                    prevRawStick = ffIn.rawStick;
                    prevStickDelta = delta;
                    if (ffJitter > 0.0f && moved < ffJitter)
                    {
                        const float q = 1.0f - moved / ffJitter;
                        jitterAtt = 1.0f - q * q;
                    }
                }
            }

            float speed = stickSpeed;
            if (ffAveraging > 1 && ffHistCount > ffAveraging)
            {
                // сдвиг за N кадров на время между их отметками: на «лесенке»
                // кадров — ровно средняя скорость, без пульсаций
                const uint8_t k = ffAveraging;
                const float span = ffHistAge[k] - ffHistAge[0];
                if (span > 0.5f * dt) speed = (stickSp - ffHist[k]) / span;
            }
            ffSmoothed += smoothAlpha * (speed - ffSmoothed);
            float out = ffSmoothed;
            if (ffBoost > 0.0f)
            {
                // не больше самой скорости стика: на ступеньке (стик — рывком
                // за кадр) ускорение огромно, и без предела перелёт растёт
                const float lim = fabsf(ffSmoothed);
                out += fminf(lim, fmaxf(-lim, ffBoost * FF_BOOST_LEAD_S * (ffSmoothed - prevFfSmoothed) / dt));
            }
            prevFfSmoothed = ffSmoothed;
            float k = jitterAtt;
            if (ffInKnown && ffTransition > 0.0f) k *= fminf(1.0f, ffIn.deflection / ffTransition);
            return out * k + (spSpeed - stickSpeed);
        }

        // Ошибка, по которой копится интеграл, с I-relax. smooth — уставка через
        // ФНЧ relax, moved — насколько уставка сдвинулась за последние ~0.2 с.
        float relaxedError(float setpoint, float measured, float error, float smooth, float moved) const
        {
            const float away = fabsf(setpoint - smooth);   // уставка быстро меняется
            float e = error;
            if (relaxOn)
            {
                if (relaxGyro)
                {
                    // к сглаженной уставке, в полосе ±away — ноль
                    const float toSmooth = smooth - measured;
                    e = fabsf(toSmooth) <= away ? 0.0f : toSmooth - copysignf(away, toSmooth);
                }
                else
                {
                    e *= fmaxf(0.0f, 1.0f - away / ITERM_RELAX_DPS);
                }
            }
            // наш relax по ошибке: коптер ещё разгоняется к новой уставке
            // (флип) — копить большую ошибку разгона значит проскочить
            // уставку. Придерживается только та доля ошибки, которую
            // объясняет недавнее движение уставки: ошибку без него (порыв,
            // перекос тяги на рывке газом — это тоже десятки °/с) интеграл
            // должен догнать, иначе её держит один P и коптер уходит на
            // десятки градусов
            if (relaxError)
            {
                const float f = fminf(1.0f, fmaxf(0.0f, 1.0f - (fabsf(error) - 20.0f) / 60.0f));
                const float w = fminf(1.0f, moved / fmaxf(1.0f, fabsf(error)));
                e *= 1.0f - w * (1.0f - f);
            }
            return e;
        }

        AxisGains gains;
        float limit = 0.3f;
        float sumLimit = 1.0f;           // pidsum_limit: доля смесителя
        bool sumClamped = false;         // сумма упёрлась в предел (прошлый цикл)
        float dt = 0.001f;
        bool tpaOnP = true;
        float dMax = 0.0f, dMaxGain = 37.0f, dMaxAdvance = 20.0f;
        bool relaxOn = true, relaxIncOnly = false, relaxGyro = false, relaxError = true;
        float relaxCutoffHz = 10.0f;
        Pt1 dLpf, outLpf, ffLpf, ffShapedLpf, boostLpf, relaxLpf, spTrack;
        // feedforward_*
        float ffTransition = 0.0f, ffSmooth = 0.0f, ffJitter = 0.0f, ffBoost = 0.0f;
        uint8_t ffAveraging = 1;
        bool ffShaping = false;
        FeedforwardInput ffIn;
        bool ffInKnown = false;
        bool ffFirst = true;
        float ffHist[FF_HISTORY] = {}, ffHistAge[FF_HISTORY] = {};   // уставка от стика в кадрах и сколько с тех пор, с
        uint8_t ffHistCount = 0;
        float ffSmoothed = 0.0f, prevFfSmoothed = 0.0f, smoothAlpha = 1.0f;
        float prevStickSp = 0.0f, prevRawStick = 0.0f, prevStickDelta = 0.0f, jitterAtt = 1.0f;
        float ffLimit = 0.0f, maxRate = 0.0f;
        bool integrated = false;
        float integratedLeak = 0.0f, integratedSum = 0.0f;
        AxisTerms terms;
        float iTerm = 0.0f;
        float boost = 0.0f;
        float prevMeasured = 0.0f;
        float prevSetpoint = 0.0f;
        bool first = true;
    };

    // Absolute control (abs_control_*): угол, который рама потеряла против
    // уставки (порыв, перекос тяги на рывке газом — пока I догонял), в ACRO
    // не пропадает: ошибка угловой скорости копится в ошибку угла, и к
    // уставке добавляется поправка gain × ошибка угла (не больше limit),
    // пока рама не вернётся. Пока стик быстро движется, отставание рамы —
    // не потеря: в полосе ±2·|уставка − её ФНЧ| вокруг ФНЧ уставки ошибка не
    // копится, а тает к нулю (не дальше нуля). Накопленное — не больше
    // error_limit градусов.
    class AbsoluteControl
    {
    public:
        void configure(float cutoffHz, float dtS)
        {
            dt = dtS;
            spLpf.configure(cutoffHz, dtS);
            reset();
        }

        void setCutoff(float cutoffHz) { spLpf.configure(cutoffHz, dt); }

        void reset()
        {
            angleError = 0.0f;
            first = true;
        }

        // setpoint, gyro — °/с; поправка уставки, °/с.
        float update(float setpoint, float gyro, float gain, float limit, float errorLimit)
        {
            if (first)
            {
                spLpf.reset(setpoint);
                first = false;
            }
            const float smooth = spLpf.apply(setpoint);
            const float band = 2.0f * fabsf(setpoint - smooth);
            const float hi = smooth + band, lo = smooth - band;
            float rateError;
            if (gyro > hi || gyro < lo)
            {
                rateError = (gyro > hi ? hi : lo) - gyro;
            }
            else
            {
                // в полосе: к нулю — со скоростью до края полосы, не дальше нуля
                rateError = angleError > 0.0f ? lo - gyro : hi - gyro;
                if (fabsf(rateError * dt) > fabsf(angleError)) rateError = -angleError / dt;
            }
            angleError = fminf(errorLimit, fmaxf(-errorLimit, angleError + rateError * dt));
            return fminf(limit, fmaxf(-limit, gain * angleError));
        }

        float error() const { return angleError; }

    private:
        Pt1 spLpf;
        float dt = 0.001f;
        float angleError = 0.0f;   // °
        bool first = true;
    };

    // iterm_rotation: рама повернулась на rates·dt — интеграл, который держит
    // неподвижное в мире (ветер), поворачивается вместе с ней: в пируэте
    // накопленное на крене переходит в тангаж. Оси FRD (крен, тангаж,
    // рысканье — правая тройка, Attitude.h): v' = v − ω × v · dt.
    inline void rotateIterm(float v[3], const float ratesDps[3], float dtS)
    {
        const float k = dtS * 0.017453293f;
        const float w0 = ratesDps[0] * k, w1 = ratesDps[1] * k, w2 = ratesDps[2] * k;
        const float c0 = w1 * v[2] - w2 * v[1];
        const float c1 = w2 * v[0] - w0 * v[2];
        const float c2 = w0 * v[1] - w1 * v[0];
        v[0] -= c0;
        v[1] -= c1;
        v[2] -= c2;
    }

    // iterm_windup: моторы разошлись больше порога (доля хода) — интеграл
    // копится слабее, на полном разбросе стоит. windup 1 — выключено.
    inline float itermWindupScale(float spread, float windup)
    {
        if (windup >= 1.0f || spread <= windup) return 1.0f;
        return fminf(1.0f, fmaxf(0.0f, (1.0f - spread) / (1.0f - windup)));
    }
}
