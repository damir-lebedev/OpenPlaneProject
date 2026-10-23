#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Filters.h"

// ============================================================
// Фильтр гироскопа мультикоптера.
//
// Главный шум гироскопа на коптере — вибрация от моторов: на частоте
// вращения, на её удвоенной и на частоте прохода лопастей (обороты ×
// число лопастей). Режекторы ставятся ровно туда и едут вслед за
// оборотами; обороты — по модели винтомоторной группы (Presets.h:
// доля тяги и напряжение -> частота вращения). Датчика оборотов не
// нужно (двунаправленный DShot даст точные — тогда центр берётся из
// него, setRotationHz). Дальше — ФНЧ второго порядка: всё, что выше
// полосы контура, но мимо режекторов.
//
// Режектор не опускается ниже minNotchHz: у больших винтов вращение
// медленное, и режектор у полосы контура съел бы запас по фазе.
// ============================================================

namespace copter
{
    constexpr uint8_t NOTCH_HARMONICS = 3;

    // Где стоят режекторы при частоте вращения rotHz: 1×, 2× и проход
    // лопастей (у двухлопастных — 3×). Ниже minHz — выключен (0).
    inline void notchCenters(float rotHz, uint8_t blades, float minHz, float out[NOTCH_HARMONICS])
    {
        const float mult[NOTCH_HARMONICS] = { 1.0f, 2.0f, blades > 2 ? static_cast<float>(blades) : 3.0f };
        for (uint8_t h = 0; h < NOTCH_HARMONICS; ++h)
        {
            const float f = rotHz * mult[h];
            out[h] = rotHz > 0.0f && f >= minHz ? f : 0.0f;
        }
    }

    class GyroFilter
    {
    public:
        void configure(float loopHz, float lpfHz, float notchQ, uint8_t blades, float minNotchHz)
        {
            fs = loopHz;
            q = notchQ;
            bladeCount = blades;
            minHz = minNotchHz;
            for (int a = 0; a < 3; ++a)
            {
                lpf[a].configure(lpfHz, 1.0f / loopHz);
                for (int h = 0; h < NOTCH_HARMONICS; ++h) notch[a][h].notch(0.0f, loopHz, q);
            }
            reset();
        }

        void reset()
        {
            for (int a = 0; a < 3; ++a)
            {
                lpf[a].reset();
                for (int h = 0; h < NOTCH_HARMONICS; ++h) notch[a][h].reset();
            }
        }

        // Частота вращения моторов, Гц (0 — моторы стоят, режекторы выключены).
        void setRotationHz(float hz)
        {
            notchCenters(hz, bladeCount, minHz, centers);
            for (int h = 0; h < NOTCH_HARMONICS; ++h)
            {
                for (int a = 0; a < 3; ++a) notch[a][h].setCenter(centers[h]);
            }
        }

        // rates — °/с по трём осям; фильтруется на месте.
        void apply(float rates[3])
        {
            for (int a = 0; a < 3; ++a)
            {
                float v = rates[a];
                for (int h = 0; h < NOTCH_HARMONICS; ++h) v = notch[a][h].apply(v);
                rates[a] = lpf[a].apply(v);
            }
        }

        float notchCenter(uint8_t harmonic) const { return harmonic < NOTCH_HARMONICS ? centers[harmonic] : 0.0f; }

    private:
        Pt2 lpf[3];
        Biquad notch[3][NOTCH_HARMONICS];
        float centers[NOTCH_HARMONICS] = {};
        float fs = 1000.0f, q = 3.0f, minHz = 80.0f;
        uint8_t bladeCount = 3;
    };
}
