#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/GpsRescue.h"

// ============================================================
// Положение и скорость мультикоптера по горизонтали: GPS + акселерометр.
//
// GPS даёт точку 10 раз в секунду, с шумом в десятки сантиметров и с
// опозданием около 0.1 с (приёмник считает решение, потом шлёт его по
// UART). Удерживать по нему место напрямую — значит гоняться за шумом и
// всегда отставать. Акселерометр, повёрнутый в земные оси, знает ускорение
// сразу и гладко, но при интегрировании уплывает. Вместе: между фиксами
// скорость и положение ведёт акселерометр, GPS медленно подтягивает их и
// учит смещение акселерометра (ошибку горизонта) — как VerticalEstimator
// делает с барометром по вертикали.
//
// Задержка GPS: фикс сравнивается не с тем, где оценка сейчас, а с тем, где
// она была gpsDelayS назад (кольцо прошлых оценок). Поправка прибавляется
// ко всей истории и к текущему состоянию: коптер на скорости 15 м/с не
// «отстаёт» на полтора метра и не тормозит раньше времени.
//
// Курс. Акселерометр живёт в осях «от включения» (рысканье гироскопа), GPS —
// в осях север/восток. Поворот между ними (поправка курса):
//   • компас (GpsInput::headingTrusted) — сразу после взлёта;
//   • без компаса — по самому полёту: приращение скорости GPS за 0.4 с и
//     то же приращение по акселерометру — один и тот же вектор в двух осях,
//     угол между ними и есть поправка. Хватает одного энергичного разгона
//     или торможения (BF без компаса просит лететь прямо 50–100 м);
//   • оба источника сверяются: компас, который расходится с полётом больше
//     чем на 45° полсекунды, — врёт (не откалиброван, рядом силовой
//     провод). Тогда курс — по GPS, а компасу веры нет, пока не сойдутся.
// Курса нет совсем — оценка идёт по одному GPS (без акселерометра): для
// картинки и возврата домой годится, для удержания позиции — нет.
//
// Оси: север/восток от дома, м и м/с; курс 0 — север, по часовой.
// Без Arduino — тестируется на ПК (test_copter, test_copter_sim).
// ============================================================

namespace copter
{
    enum class HeadingSource : uint8_t { NONE, GPS, MAG };

    inline const char* headingSourceName(HeadingSource s)
    {
        switch (s)
        {
            case HeadingSource::GPS: return "GPS";
            case HeadingSource::MAG: return "MAG";
            case HeadingSource::NONE:
            default: return "NONE";
        }
    }

    class PositionEstimator
    {
    public:
        static constexpr float GPS_TIMEOUT_S = 1.5f;      // фикса нет дольше — GPS «потерян»
        static constexpr float SNAP_M = 25.0f;            // фикс дальше от оценки — начать заново (новый дом, сбой)
        static constexpr float MAG_DISAGREE_DEG = 45.0f;  // компас против полёта
        static constexpr float MAG_DISAGREE_S = 0.5f;
        static constexpr float MAG_AGREE_DEG = 20.0f;

        // gpsDelayS — насколько фикс старше момента, когда он пришёл.
        void configure(float gpsDelayS = 0.1f) { delay = fminf(0.3f, fmaxf(0.0f, gpsDelayS)); }
        void setMinSats(uint8_t n) { minSats = n; }

        void reset()
        {
            init = false;
            sinceFix = 1e6f;
            headN = headE = 0.0f;
            headKnown = false;
            magBad = false;
            magBadTimer = 0.0f;
            magOffsetValid = false;
            biasF = biasR = 0.0f;
            offPn = offPe = offVn = offVe = 0.0f;
            count = 0;
            gpsCount = 0;
            source = HeadingSource::NONE;
        }

        // Каждый цикл. accelX/accelY — горизонтальное ускорение в осях «от
        // включения» (ориентация гироскопа: X — туда, где рысканье 0), м/с²;
        // yawDeg — рысканье гироскопа; airborne — в воздухе (на земле курс
        // по полёту не учится).
        void update(float accelX, float accelY, float yawDeg, const GpsInput& g, bool airborne, float dt)
        {
            sinceFix += dt;
            yaw = yawDeg;

            // ---- курс: компас ----
            if (g.headingTrusted)
            {
                const float m = wrap180(g.headingDeg - yawDeg);
                magOffset = magOffsetValid ? wrap180(magOffset + wrap180(m - magOffset) * fminf(1.0f, dt / MAG_TAU_S)) : m;
                magOffsetValid = true;
            }
            else
            {
                magOffsetValid = false;
            }
            const float gpsOffset = atan2f(headE, headN) * RAD2DEG;
            const float gpsWeight = sqrtf(headN * headN + headE * headE);
            headKnown = gpsWeight >= HEAD_MIN;
            // компас против полёта
            if (magOffsetValid && headKnown)
            {
                const float d = fabsf(wrap180(magOffset - gpsOffset));
                if (d > MAG_DISAGREE_DEG) magBadTimer += dt;
                else magBadTimer = 0.0f;
                if (magBadTimer > MAG_DISAGREE_S) magBad = true;
                if (d < MAG_AGREE_DEG) magBad = false;
            }
            if (!magOffsetValid)
            {
                magBad = false;
                magBadTimer = 0.0f;
            }
            if (magOffsetValid && !magBad)
            {
                source = HeadingSource::MAG;
                offset = magOffset;
            }
            else if (headKnown)
            {
                source = HeadingSource::GPS;
                offset = gpsOffset;
            }
            else
            {
                source = HeadingSource::NONE;
            }

            // ---- ускорение в осях север/восток ----
            // u — чистый интеграл акселерометра в осях «от включения» (для курса по полёту)
            uX += accelX * dt;
            uY += accelY * dt;
            float aN = 0.0f, aE = 0.0f;
            if (source != HeadingSource::NONE)
            {
                const float h = headingDeg() * DEG2RAD, c = cosf(h), s = sinf(h);
                const float y = yawDeg * DEG2RAD, cy = cosf(y), sy = sinf(y);
                // в осях носа: вперёд и вправо (минус выученное смещение)
                const float fwd = accelX * cy + accelY * sy - biasF;
                const float right = -accelX * sy + accelY * cy - biasR;
                aN = fwd * c - right * s;
                aE = fwd * s + right * c;
            }
            accN = aN;
            accE = aE;
            if (init)
            {
                pn += (vn + 0.5f * aN * dt) * dt;
                pe += (ve + 0.5f * aE * dt) * dt;
                vn += aN * dt;
                ve += aE * dt;
            }
            // поправки фикса входят в выход плавно
            const float k = fminf(1.0f, dt / SMOOTH_S);
            offPn -= offPn * k;
            offPe -= offPe * k;
            offVn -= offVn * k;
            offVe -= offVe * k;

            // ---- история ----
            sinceHist += dt;
            if (sinceHist >= HIST_DT)
            {
                sinceHist -= HIST_DT;
                if (sinceHist > HIST_DT) sinceHist = 0.0f;   // цикл реже 50 Гц — не догонять
                head = static_cast<uint8_t>((head + 1) % HIST);
                hist[head] = { pn, pe, vn, ve, uX, uY };
                if (count < HIST) ++count;
                ++ticks;
                recenter();
            }

            if (g.fresh) onFix(g, airborne);
        }

        // Оценка годится: GPS живой (фикс, спутники, дом), фиксы идут.
        bool valid() const { return init && gpsGood && sinceFix <= GPS_TIMEOUT_S; }
        float gpsAgeS() const { return sinceFix; }

        float north() const { return originN + pn - offPn; }
        float east() const { return originE + pe - offPe; }
        float velN() const { return vn - offVn; }
        float velE() const { return ve - offVe; }
        float speed() const { return sqrtf(velN() * velN() + velE() * velE()); }
        // ускорение (без смещения), м/с²; курса нет — 0
        float accelN() const { return accN; }
        float accelE() const { return accE; }

        // Курс носа, 0…360 (0 — север); без источника — рысканье гироскопа.
        float headingDeg() const
        {
            float h = yaw + (source != HeadingSource::NONE ? offset : 0.0f);
            while (h < 0.0f) h += 360.0f;
            while (h >= 360.0f) h -= 360.0f;
            return h;
        }
        HeadingSource headingSource() const { return source; }
        // компас есть, но расходится с полётом
        bool magDisagrees() const { return magBad; }
        // курс по полёту выучен (с компасом — для сверки)
        bool gpsHeadingKnown() const { return headKnown; }
        float gpsHeadingOffsetDeg() const { return atan2f(headE, headN) * RAD2DEG; }

    private:
        static constexpr float DEG2RAD = 0.017453293f;
        static constexpr float RAD2DEG = 57.29578f;
        static constexpr uint8_t HIST = 32;
        static constexpr float HIST_DT = 0.02f;         // история — 0.64 с
        static constexpr uint8_t GPS_RING = 6;
        static constexpr float SMOOTH_S = 0.15f;
        static constexpr float MAG_TAU_S = 0.3f;
        // поправки GPS (1/с): положение, скорость, скорость по ошибке положения, смещение
        static constexpr float K_POS = 0.6f, K_VEL = 2.0f, K_POS_VEL = 0.15f, K_BIAS = 0.6f;
        // без акселерометра (курса нет): оценка идёт за GPS
        static constexpr float K_POS_GPS = 2.5f, K_VEL_GPS = 4.0f;
        static constexpr float BIAS_MAX = 1.5f;          // м/с² — около 9° ошибки горизонта
        // курс по полёту: приращения скорости на базе 0.25…0.5 с, не меньше 0.5 м/с
        static constexpr float HEAD_SPAN_MIN_S = 0.25f, HEAD_SPAN_MAX_S = 0.5f;
        static constexpr float HEAD_DV_MIN = 0.5f, HEAD_DV_MAX = 2.0f;
        static constexpr float HEAD_MIN = 3.0f, HEAD_MAX = 40.0f;
        static constexpr float RECENTER_M = 100.0f, RECENTER_U = 20.0f;

        struct Sample
        {
            float pn, pe, vn, ve, ux, uy;
        };
        struct GpsVel
        {
            float vn, ve;
            uint32_t tick;
        };

        static float wrap180(float d)
        {
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        }

        // Оценка ageS назад (между соседними записями — линейно).
        Sample past(float ageS) const
        {
            if (count == 0) return { pn, pe, vn, ve, uX, uY };
            float f = (ageS - sinceHist) / HIST_DT;
            if (f <= 0.0f)
            {
                // моложе последней записи: между ней и текущим состоянием
                const Sample& a = hist[head];
                const float w = sinceHist > 1e-6f ? fminf(1.0f, fmaxf(0.0f, ageS / sinceHist)) : 1.0f;
                return { pn + (a.pn - pn) * w, pe + (a.pe - pe) * w, vn + (a.vn - vn) * w,
                         ve + (a.ve - ve) * w, uX + (a.ux - uX) * w, uY + (a.uy - uY) * w };
            }
            const float top = static_cast<float>(count - 1);
            if (f > top) f = top;
            const uint8_t i = static_cast<uint8_t>(f);
            const float w = f - static_cast<float>(i);
            const Sample& a = hist[(head + HIST - i) % HIST];
            const Sample& b = hist[(head + HIST - (i + 1 < count ? i + 1 : i)) % HIST];
            return { a.pn + (b.pn - a.pn) * w, a.pe + (b.pe - a.pe) * w, a.vn + (b.vn - a.vn) * w,
                     a.ve + (b.ve - a.ve) * w, a.ux + (b.ux - a.ux) * w, a.uy + (b.uy - a.uy) * w };
        }

        void shift(float dPn, float dPe, float dVn, float dVe)
        {
            pn += dPn;
            pe += dPe;
            vn += dVn;
            ve += dVe;
            for (uint8_t i = 0; i < HIST; ++i)
            {
                hist[i].pn += dPn;
                hist[i].pe += dPe;
                hist[i].vn += dVn;
                hist[i].ve += dVe;
            }
            offPn += dPn;
            offPe += dPe;
            offVn += dVn;
            offVe += dVe;
        }

        void onFix(const GpsInput& g, bool airborne)
        {
            const float period = fminf(0.3f, fmaxf(0.05f, sinceFix));
            gpsGood = g.fix && g.sats >= minSats && g.home;
            if (!gpsGood)
            {
                gpsCount = 0;
                return;
            }
            sinceFix = 0.0f;
            const Sample was = past(delay);
            const float ePn = g.northM - originN - was.pn, ePe = g.eastM - originE - was.pe;
            if (!init || fabsf(ePn) > SNAP_M || fabsf(ePe) > SNAP_M)
            {
                // первый фикс (или дом переставили): встать в точку GPS с его скоростью
                init = true;
                originN = g.northM;
                originE = g.eastM;
                pn = g.velN * delay;
                pe = g.velE * delay;
                vn = g.velN;
                ve = g.velE;
                offPn = offPe = offVn = offVe = 0.0f;
                count = 0;
                gpsCount = 0;
                return;
            }
            const float eVn = g.velN - was.vn, eVe = g.velE - was.ve;
            if (source != HeadingSource::NONE)
            {
                shift(K_POS * period * ePn, K_POS * period * ePe, (K_VEL * eVn + K_POS_VEL * ePn) * period,
                      (K_VEL * eVe + K_POS_VEL * ePe) * period);
                // смещение акселерометра — в осях носа (ошибка горизонта поворачивается вместе с рамой)
                const float h = headingDeg() * DEG2RAD, c = cosf(h), s = sinf(h);
                biasF = clampBias(biasF - K_BIAS * period * (eVn * c + eVe * s));
                biasR = clampBias(biasR - K_BIAS * period * (-eVn * s + eVe * c));
            }
            else
            {
                shift(fminf(1.0f, K_POS_GPS * period) * ePn, fminf(1.0f, K_POS_GPS * period) * ePe,
                      fminf(1.0f, K_VEL_GPS * period) * eVn, fminf(1.0f, K_VEL_GPS * period) * eVe);
            }
            learnHeading(g, airborne);
        }

        // Курс по полёту: приращение скорости GPS против приращения по акселерометру.
        void learnHeading(const GpsInput& g, bool airborne)
        {
            gpsRing[gpsHead] = { g.velN, g.velE, ticks };
            const uint8_t newest = gpsHead;
            gpsHead = static_cast<uint8_t>((gpsHead + 1) % GPS_RING);
            if (gpsCount < GPS_RING) ++gpsCount;
            if (!airborne) return;
            // самый старый фикс, который ещё помещается в базу
            for (uint8_t back = static_cast<uint8_t>(gpsCount - 1); back >= 1; --back)
            {
                const GpsVel& old = gpsRing[(newest + GPS_RING - back) % GPS_RING];
                const float span = static_cast<float>(ticks - old.tick) * HIST_DT;
                if (span > HEAD_SPAN_MAX_S) continue;
                if (span < HEAD_SPAN_MIN_S) break;
                const Sample a = past(delay), b = past(delay + span);
                const float dux = a.ux - b.ux, duy = a.uy - b.uy;
                const float dvn = g.velN - old.vn, dve = g.velE - old.ve;
                const float du = sqrtf(dux * dux + duy * duy), dv = sqrtf(dvn * dvn + dve * dve);
                if (du < HEAD_DV_MIN || dv < HEAD_DV_MIN) break;
                // одинаковые по величине — тот же манёвр (не сбой GPS и не удар)
                if (du > 2.0f * dv || dv > 2.0f * du) break;
                const float m = fminf(HEAD_DV_MAX, fminf(du, dv));
                const float w = m * m / (du * dv);
                // поворот от du к dv
                headN += w * (dux * dvn + duy * dve);
                headE += w * (dux * dve - duy * dvn);
                const float len = sqrtf(headN * headN + headE * headE);
                if (len > HEAD_MAX)
                {
                    headN *= HEAD_MAX / len;
                    headE *= HEAD_MAX / len;
                }
                break;
            }
        }

        // float держит миллиметры только у нуля: положение считается от опорной
        // точки, которая переезжает за коптером, интеграл акселерометра — тоже
        void recenter()
        {
            if (fabsf(pn) > RECENTER_M || fabsf(pe) > RECENTER_M)
            {
                const float dn = pn, de = pe;
                originN += dn;
                originE += de;
                pn = pe = 0.0f;
                for (uint8_t i = 0; i < HIST; ++i)
                {
                    hist[i].pn -= dn;
                    hist[i].pe -= de;
                }
            }
            if (fabsf(uX) > RECENTER_U || fabsf(uY) > RECENTER_U)
            {
                const float dx = uX, dy = uY;
                uX = uY = 0.0f;
                for (uint8_t i = 0; i < HIST; ++i)
                {
                    hist[i].ux -= dx;
                    hist[i].uy -= dy;
                }
            }
        }

        static float clampBias(float v) { return fminf(BIAS_MAX, fmaxf(-BIAS_MAX, v)); }

        float delay = 0.1f;
        uint8_t minSats = 8;
        bool init = false;
        bool gpsGood = false;
        float sinceFix = 1e6f;
        float originN = 0.0f, originE = 0.0f;   // опорная точка; pn/pe — от неё
        float pn = 0.0f, pe = 0.0f, vn = 0.0f, ve = 0.0f;
        float accN = 0.0f, accE = 0.0f;
        float biasF = 0.0f, biasR = 0.0f;
        float offPn = 0.0f, offPe = 0.0f, offVn = 0.0f, offVe = 0.0f;
        float uX = 0.0f, uY = 0.0f;
        float yaw = 0.0f;

        Sample hist[HIST] = {};
        uint8_t head = 0, count = 0;
        float sinceHist = 0.0f;
        uint32_t ticks = 0;

        GpsVel gpsRing[GPS_RING] = {};
        uint8_t gpsHead = 0, gpsCount = 0;
        float headN = 0.0f, headE = 0.0f;   // сумма «поворотов» (вес — квадрат приращения скорости)
        bool headKnown = false;
        float magOffset = 0.0f;
        bool magOffsetValid = false;
        bool magBad = false;
        float magBadTimer = 0.0f;
        float offset = 0.0f;
        HeadingSource source = HeadingSource::NONE;
    };
}
