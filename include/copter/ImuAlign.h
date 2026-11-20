#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// ============================================================
// Поворот платы и нули датчиков числами (имена BF):
//   • align_board_roll / _pitch / _yaw — плата стоит на раме повёрнутой
//     (полётник развёрнут на 90°, наклонён под камеру и т. п.); углы —
//     поворот платы относительно рамы: рысканье + по часовой сверху,
//     тангаж + передний край платы вверх, крен + правый край вниз;
//   • acc_calibration = "x,y,z,флаг" — нуль акселерометра в осях платы
//     (X к носу, Y влево, Z вверх), единицы 1/2048 g (как acc_1G у BF);
//     флаг 1 — откалибровано (стики «газ верх + рысканье влево + тангаж
//     на себя» на ровной земле, кнопка в Studio);
//   • gyro_offset_yaw — добавка к рысканью гироскопа, 0.01 °/с (как BF).
// Порядок как у BF: нуль акселерометра — в осях платы, потом поворот.
// Чистая логика, без Arduino: проверяется на ПК.
// ============================================================

namespace copter
{
    struct ImuAlignSettings
    {
        int16_t alignRollDeg = 0;     // align_board_roll
        int16_t alignPitchDeg = 0;    // align_board_pitch
        int16_t alignYawDeg = 0;      // align_board_yaw
        float accZeroG[3] = {};       // acc_calibration / 2048, оси платы (X нос, Y влево, Z вверх)
        float gyroOffsetYawDps = 0.0f;   // gyro_offset_yaw × 0.01
    };

    // acc_calibration: "x,y,z,флаг" (целые; x..z в пределах ±999) -> нули и
    // флаг; пусто — нули, флаг 0. false — написано не так.
    inline bool parseAccCalibration(const char* s, int16_t zero[3], bool& done)
    {
        zero[0] = zero[1] = zero[2] = 0;
        done = false;
        if (!s || !*s) return true;
        long v[4] = {};
        const char* p = s;
        for (int i = 0; i < 4; ++i)
        {
            char* end = nullptr;
            v[i] = strtol(p, &end, 10);
            if (end == p) return false;
            p = end;
            if (i < 3)
            {
                if (*p != ',') return false;
                ++p;
            }
        }
        if (*p != '\0') return false;
        for (int i = 0; i < 3; ++i)
        {
            if (v[i] < -999 || v[i] > 999) return false;
        }
        if (v[3] != 0 && v[3] != 1) return false;
        for (int i = 0; i < 3; ++i) zero[i] = static_cast<int16_t>(v[i]);
        done = v[3] == 1;
        return true;
    }

    // Нули -> текст для acc_calibration (буфер ≥ 17).
    inline void formatAccCalibration(const int16_t zero[3], bool done, char* out, size_t size)
    {
        snprintf(out, size, "%d,%d,%d,%d", zero[0], zero[1], zero[2], done ? 1 : 0);
    }

    class ImuAlign
    {
    public:
        void configure(const ImuAlignSettings& s)
        {
            set = s;
            const float d2r = 3.14159265f / 180.0f;
            const float r = s.alignRollDeg * d2r, p = s.alignPitchDeg * d2r, y = s.alignYawDeg * d2r;
            const float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
            // FRD: рама = Rz(рысканье) · Ry(тангаж) · Rx(крен) · плата
            m[0][0] = cy * cp; m[0][1] = cy * sp * sr - sy * cr; m[0][2] = cy * sp * cr + sy * sr;
            m[1][0] = sy * cp; m[1][1] = sy * sp * sr + cy * cr; m[1][2] = sy * sp * cr - cy * sr;
            m[2][0] = -sp;     m[2][1] = cp * sr;                m[2][2] = cp * cr;
            rotate = s.alignRollDeg % 360 != 0 || s.alignPitchDeg % 360 != 0 || s.alignYawDeg % 360 != 0;
        }

        bool rotates() const { return rotate; }

        // Гироскоп (°/с, авиационные знаки = FRD) и акселерометр (g, X нос,
        // Y влево, Z вверх) платы -> рамы. in и out могут совпадать.
        void apply(const float gyroIn[3], const float accIn[3], float gyroOut[3], float accOut[3]) const
        {
            float g[3] = { gyroIn[0], gyroIn[1], gyroIn[2] };
            // нуль — в осях платы; FLU -> FRD
            float a[3] = { accIn[0] - set.accZeroG[0], -(accIn[1] - set.accZeroG[1]), -(accIn[2] - set.accZeroG[2]) };
            if (rotate)
            {
                mul(g);
                mul(a);
            }
            gyroOut[0] = g[0];
            gyroOut[1] = g[1];
            gyroOut[2] = g[2] + set.gyroOffsetYawDps;
            accOut[0] = a[0];
            accOut[1] = -a[1];
            accOut[2] = -a[2];
        }

        // Какой «верх» (g, оси платы X нос, Y влево, Z вверх) видит плата, когда
        // рама стоит ровно: R^T · (0, 0, −1) FRD.
        void levelUpInBoard(float up[3]) const
        {
            // столбец 2 транспонированной = строка 2 матрицы, со знаком «вверх»
            const float frd[3] = { -m[2][0], -m[2][1], -m[2][2] };
            up[0] = frd[0];
            up[1] = -frd[1];
            up[2] = -frd[2];
        }

    private:
        void mul(float v[3]) const
        {
            const float x = v[0], y = v[1], z = v[2];
            v[0] = m[0][0] * x + m[0][1] * y + m[0][2] * z;
            v[1] = m[1][0] * x + m[1][1] * y + m[1][2] * z;
            v[2] = m[2][0] * x + m[2][1] * y + m[2][2] * z;
        }

        ImuAlignSettings set;
        float m[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        bool rotate = false;
    };

    // Калибровка нуля акселерометра (BF: «газ верх + рысканье влево + тангаж
    // на себя», консоль 'a', кнопка в Studio, MSP 205): рама стоит ровно и неподвижно ~0.5 с; нуль =
    // среднее (без старого нуля) − «верх», который плата видела бы на ровной
    // раме. Двигали, наклон больше 25° или нуль больше 0.49 g — не удалась.
    class AccZeroCalibrator
    {
    public:
        enum class State : uint8_t { IDLE, RUNNING, DONE, FAILED };

        void start(float seconds = 0.5f)
        {
            state = State::RUNNING;
            need = seconds;
            time = 0.0f;
            sum[0] = sum[1] = sum[2] = 0.0f;
            count = 0;
        }

        bool running() const { return state == State::RUNNING; }
        State getState() const { return state; }

        // accBoard — g в осях платы до вычитания нуля; gyro — °/с.
        State feed(const float accBoard[3], const float gyroDps[3], float dt, const ImuAlign& align)
        {
            if (state != State::RUNNING) return state;
            const float spin = sqrtf(gyroDps[0] * gyroDps[0] + gyroDps[1] * gyroDps[1] + gyroDps[2] * gyroDps[2]);
            const float g = sqrtf(accBoard[0] * accBoard[0] + accBoard[1] * accBoard[1] + accBoard[2] * accBoard[2]);
            if (spin > STILL_DPS || fabsf(g - 1.0f) > ONE_G_TOLERANCE)
            {
                state = State::FAILED;
                return state;
            }
            for (int i = 0; i < 3; ++i) sum[i] += accBoard[i];
            ++count;
            time += dt;
            if (time < need) return state;

            float up[3];
            align.levelUpInBoard(up);
            float avg[3];
            for (int i = 0; i < 3; ++i) avg[i] = sum[i] / count;
            const float len = sqrtf(avg[0] * avg[0] + avg[1] * avg[1] + avg[2] * avg[2]);
            const float cosTilt = (avg[0] * up[0] + avg[1] * up[1] + avg[2] * up[2]) / fmaxf(len, 1e-6f);
            if (cosTilt < COS_MAX_TILT)
            {
                state = State::FAILED;
                return state;
            }
            for (int i = 0; i < 3; ++i)
            {
                const long z = lroundf((avg[i] - up[i]) * 2048.0f);
                if (z < -999 || z > 999)
                {
                    state = State::FAILED;
                    return state;
                }
                zero[i] = static_cast<int16_t>(z);
            }
            state = State::DONE;
            return state;
        }

        // Нуль после DONE, 1/2048 g.
        const int16_t* result() const { return zero; }
        void clear() { state = State::IDLE; }

        static constexpr float STILL_DPS = 6.0f;
        static constexpr float ONE_G_TOLERANCE = 0.15f;
        static constexpr float COS_MAX_TILT = 0.9063f;   // 25°

    private:
        State state = State::IDLE;
        float need = 0.5f, time = 0.0f;
        float sum[3] = {};
        uint32_t count = 0;
        int16_t zero[3] = {};
    };
}
