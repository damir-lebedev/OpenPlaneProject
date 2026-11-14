#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Attitude.h"

// ============================================================
// Компас коптера: из сырого поля датчика — курс носа.
//
//   сырое поле (мкТл, оси датчика) -> калибровка (mag_zero_* — смещение от
//   железа рядом, mag_scale_* — растяжение по осям) -> установка датчика
//   (align_mag, как в BF) -> поле в осях рамы -> в земные оси по ориентации
//   коптера (наклон не мешает: курс верен и на крене 40°) -> направление на
//   магнитный север -> + mag_declination = курс от истинного севера.
//
// Курс хранится как поправка к рысканью гироскопа, посчитанная в момент
// отсчёта датчика (50 Гц): между отсчётами нос ведёт гироскоп, и быстрый
// поворот не «смазывает» курс.
//
// Калибровку делает приложение (мастер в OpenPlane Studio: коптер крутят во
// все стороны, точки ложатся на сферу, по ним считаются mag_zero_* и
// mag_scale_*, а по тому, как поле поворачивается вместе с рамой, —
// align_mag). Без калибровки компасу веры нет (trusted() = false): режимам по
// GPS он тогда не курс, а помеха.
//
// Поле не похоже на земное (15…90 мкТл) — рядом силовой провод или магнит:
// тоже не верим, пока не вернётся.
//
// Оси поля на входе — как у драйверов (sensors/mag/MagnetometerBase.h):
// X к носу, Y влево, Z вверх. Без Arduino — тестируется на ПК.
// ============================================================

namespace copter
{
    // align_mag — порядок как в BF. CWnn — датчик повёрнут на nn° по часовой
    // (вид сверху); FLIP — перевёрнут (через тангаж), потом повёрнут.
    enum class MagAlign : uint8_t { BOARD_DEFAULT, CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP };

    struct CompassSettings
    {
        MagAlign align = MagAlign::BOARD_DEFAULT;
        float zero[3] = { 0.0f, 0.0f, 0.0f };    // мкТл, в осях датчика
        float scale[3] = { 1.0f, 1.0f, 1.0f };
        float declinationDeg = 0.0f;             // + — магнитный север восточнее истинного
    };

    // Поле из осей датчика в оси рамы (X к носу, Y влево, Z вверх).
    inline void alignMag(MagAlign a, const float in[3], float out[3])
    {
        float x = in[0], y = in[1], z = in[2];
        uint8_t turns = 0;   // четверти оборота по часовой
        bool flip = false;
        switch (a)
        {
            case MagAlign::CW90: turns = 1; break;
            case MagAlign::CW180: turns = 2; break;
            case MagAlign::CW270: turns = 3; break;
            case MagAlign::CW0FLIP: flip = true; break;
            case MagAlign::CW90FLIP: turns = 1; flip = true; break;
            case MagAlign::CW180FLIP: turns = 2; flip = true; break;
            case MagAlign::CW270FLIP: turns = 3; flip = true; break;
            case MagAlign::BOARD_DEFAULT:
            case MagAlign::CW0:
            default: break;
        }
        if (flip)
        {
            // вверх ногами через тангаж: нос и верх меняют знак
            x = -x;
            z = -z;
        }
        for (uint8_t i = 0; i < turns; ++i)
        {
            // на 90° по часовой: что было слева — теперь спереди
            const float nx = y, ny = -x;
            x = nx;
            y = ny;
        }
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }

    class Compass
    {
    public:
        static constexpr float FIELD_MIN_UT = 15.0f, FIELD_MAX_UT = 90.0f;

        void configure(const CompassSettings& s) { set = s; }
        const CompassSettings& settings() const { return set; }

        // Калибровали ли (mag_zero_* не нулевые — как в BF).
        bool calibrated() const { return set.zero[0] != 0.0f || set.zero[1] != 0.0f || set.zero[2] != 0.0f; }

        // Датчик пропал — курса нет до следующего отсчёта.
        void lost() { fresh = false; }

        // Новый отсчёт датчика: rawUT — мкТл в осях датчика.
        void update(const float rawUT[3], const AttitudeQuat& att)
        {
            for (uint8_t i = 0; i < 3; ++i) raw[i] = rawUT[i];
            float c[3];
            for (uint8_t i = 0; i < 3; ++i) c[i] = (rawUT[i] - set.zero[i]) * (set.scale[i] > 0.0f ? set.scale[i] : 1.0f);
            float b[3];
            alignMag(set.align, c, b);
            const float len = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            field = fresh ? field + 0.1f * (len - field) : len;
            fresh = true;
            // в земные оси «от включения» (FRD -> север', восток', вниз)
            const Vec3 e = att.toEarth({ b[0], -b[1], -b[2] });
            horizontal = sqrtf(e.x * e.x + e.y * e.y);
            if (horizontal < 1.0f) return;   // поле вертикально (полюс или мусор) — курса из него нет
            // магнитный север — под углом atan2 к оси «рысканье 0»
            offset = wrap180(-atan2f(e.y, e.x) * RAD2DEG + set.declinationDeg);
            hasOffset = true;
        }

        // Курсу можно верить: откалиброван, отсчёты идут, поле — как у Земли.
        bool trusted() const
        {
            return fresh && hasOffset && calibrated() && field >= FIELD_MIN_UT && field <= FIELD_MAX_UT && horizontal >= 5.0f;
        }

        // Курс носа, 0…360, при рысканье гироскопа yawDeg.
        float headingDeg(float yawDeg) const
        {
            float h = yawDeg + offset;
            while (h < 0.0f) h += 360.0f;
            while (h >= 360.0f) h -= 360.0f;
            return h;
        }

        float fieldUT() const { return field; }              // модуль поля после калибровки
        const float* rawUT() const { return raw; }           // последний сырой отсчёт (мастеру калибровки)
        bool hasSample() const { return fresh; }

    private:
        static constexpr float RAD2DEG = 57.29578f;

        static float wrap180(float d)
        {
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        }

        CompassSettings set;
        float raw[3] = {};
        float field = 0.0f, horizontal = 0.0f;
        float offset = 0.0f;
        bool hasOffset = false;
        bool fresh = false;
    };
}
