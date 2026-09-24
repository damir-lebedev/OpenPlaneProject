#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "sensors/SensorMounting.h"

// ============================================================
// IMU ORIENTATION — как IMU стоит в самолёте
//
// Матрица поворота из осей чипа в оси самолёта (X к носу, Y влево,
// Z вверх): body = R · chip. Строки R — оси самолёта, записанные в
// осях чипа.
//
// Два источника:
//   • Config::IMU_ROTATION_CW_DEG — поворот вокруг вертикали шагами
//     по 90°, плата обязательно чипом вверх (fromYawSteps);
//   • калибровка по трём позам (fromPoses) — плата стоит КАК УГОДНО:
//     под любым углом, вверх ногами, на боку. Акселерометр в покое
//     показывает направление "вверх" в осях чипа:
//       ровно                 -> ось Z самолёта (и заодно горизонт);
//       нос поднят            -> "верх" отклонился к носу: его часть,
//                                перпендикулярная Z, — ось X;
//       правое крыло опущено  -> "верх" отклонился влево: ось Y.
//     X по носу и X по крылу (Y × Z) должны совпасть — это проверка,
//     что наклоняли то, что просили, и в ту сторону. Итог — среднее
//     двух оценок, то есть небольшой перекос в одной позе
//     наполовину гасится другой.
//
// Горизонт при этом тоже берётся из позы "ровно": смещение нуля
// акселерометра входит в неё и потому не мешает.
// ============================================================

class ImuOrientation
{
public:

    ImuOrientation()
    {
        *this = fromYawSteps(0);
    }

    // Поворот вокруг вертикали шагами по 90° (Config::IMU_ROTATION_CW_DEG),
    // плата чипом вверх.
    static ImuOrientation fromYawSteps(uint16_t rotationCwDeg)
    {
        // Столбец j матрицы — куда в осях самолёта смотрит ось j чипа.
        float chipXtoBodyX, chipXtoBodyY, chipYtoBodyX, chipYtoBodyY;
        SensorMounting::rotateToBody(rotationCwDeg, 1, 0, chipXtoBodyX, chipXtoBodyY);
        SensorMounting::rotateToBody(rotationCwDeg, 0, 1, chipYtoBodyX, chipYtoBodyY);

        ImuOrientation o(NoInit{});
        o.r[0][0] = chipXtoBodyX; o.r[0][1] = chipYtoBodyX; o.r[0][2] = 0;
        o.r[1][0] = chipXtoBodyY; o.r[1][1] = chipYtoBodyY; o.r[1][2] = 0;
        o.r[2][0] = 0;            o.r[2][1] = 0;            o.r[2][2] = 1;
        return o;
    }

    // level, noseUp, rightWingDown — "верх" (среднее показание
    // акселерометра в покое, g) в осях чипа в трёх позах.
    // nullptr — успех (результат в out), иначе — причина отказа.
    static const char* fromPoses(const float level[3], const float noseUp[3],
                                 const float rightWingDown[3], ImuOrientation& out)
    {
        float z[3], nose[3], wing[3];
        if (!normalize(level, z) || !normalize(noseUp, nose) || !normalize(rightWingDown, wing))
        {
            return "нет показаний акселерометра";
        }

        if (!tiltInRange(z, nose)) return "в шаге 2 нужен наклон 30-60°";
        if (!tiltInRange(z, wing)) return "в шаге 3 нужен наклон 30-60°";

        // Нос поднят: "верх" ушёл к носу — перпендикулярная Z часть и
        // есть направление носа.
        float xFromNose[3];
        perpendicularPart(nose, z, xFromNose);

        // Правое крыло опущено: "верх" ушёл влево (+Y); нос = Y × Z.
        float yFromWing[3], xFromWing[3];
        perpendicularPart(wing, z, yFromWing);
        cross(yFromWing, z, xFromWing);

        const float agreement = dot(xFromNose, xFromWing);   // cos угла между оценками
        if (agreement < -0.5f)
        {
            return "шаги 2 и 3 противоречат друг другу: нос опустили вместо подъёма "
                   "или опустили левое крыло вместо правого";
        }
        if (agreement < AGREEMENT_MIN_COS)
        {
            return "в шагах 2 и 3 наклоняли не те оси: в шаге 2 — только нос, "
                   "в шаге 3 — только правое крыло";
        }

        float x[3] = { xFromNose[0] + xFromWing[0], xFromNose[1] + xFromWing[1],
                       xFromNose[2] + xFromWing[2] };
        float xUnit[3], y[3];
        normalize(x, xUnit);
        cross(z, xUnit, y);

        ImuOrientation o(NoInit{});
        for (uint8_t i = 0; i < 3; ++i)
        {
            o.r[0][i] = xUnit[i];
            o.r[1][i] = y[i];
            o.r[2][i] = z[i];
        }
        out = o;
        return nullptr;
    }

    void apply(const float chip[3], float body[3]) const
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            body[i] = r[i][0] * chip[0] + r[i][1] * chip[1] + r[i][2] * chip[2];
        }
    }

    // Угол между измеренным "верхом" (оси чипа) и осью Z самолёта, °.
    float tiltFromLevelDeg(const float chipUp[3]) const
    {
        float body[3], unit[3];
        apply(chipUp, body);
        if (!normalize(body, unit)) return 180.0f;
        return acosf(constrain(unit[2], -1.0f, 1.0f)) * RAD_TO_DEG;
    }

    // --- NVS ---

    bool load(const char* nvsNamespace)
    {
        // На запись, хотя только читаем: иначе отсутствующее
        // пространство имён Preferences печатает как ошибку.
        Preferences prefs;
        if (!prefs.begin(nvsNamespace, false)) return false;

        bool ok = prefs.getBool("ok", false) && prefs.getBytesLength("r") == sizeof(r);
        ImuOrientation loaded(NoInit{});
        if (ok) ok = prefs.getBytes("r", loaded.r, sizeof(loaded.r)) == sizeof(loaded.r);
        prefs.end();

        if (!ok || !loaded.isRotation()) return false;
        *this = loaded;
        return true;
    }

    void save(const char* nvsNamespace) const
    {
        Preferences prefs;
        if (!prefs.begin(nvsNamespace, false)) return;
        prefs.putBytes("r", r, sizeof(r));
        prefs.putBool("ok", true);
        prefs.end();
    }

    // Словами: какая ось чипа смотрит в нос и какая вверх.
    void describe(Print& out) const
    {
        out.print("нос = ");
        describeAxis(out, r[0]);
        out.print(", верх = ");
        describeAxis(out, r[2]);
    }


private:

    struct NoInit {};
    explicit ImuOrientation(NoInit) {}

    static constexpr float MIN_TILT_DEG = 20.0f;
    static constexpr float MAX_TILT_DEG = 80.0f;

    // Оценки носа по шагам 2 и 3 должны совпасть с точностью ~25°.
    static constexpr float AGREEMENT_MIN_COS = 0.9f;

    float r[3][3];

    static float dot(const float a[3], const float b[3])
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static void cross(const float a[3], const float b[3], float out[3])
    {
        out[0] = a[1] * b[2] - a[2] * b[1];
        out[1] = a[2] * b[0] - a[0] * b[2];
        out[2] = a[0] * b[1] - a[1] * b[0];
    }

    static bool normalize(const float v[3], float out[3])
    {
        const float length = sqrtf(dot(v, v));
        if (length < 1e-6f) return false;
        for (uint8_t i = 0; i < 3; ++i) out[i] = v[i] / length;
        return true;
    }

    // Единичная часть v, перпендикулярная единичному axis.
    static void perpendicularPart(const float v[3], const float axis[3], float out[3])
    {
        const float along = dot(v, axis);
        const float p[3] = { v[0] - along * axis[0], v[1] - along * axis[1], v[2] - along * axis[2] };
        normalize(p, out);
    }

    static bool tiltInRange(const float levelUnit[3], const float poseUnit[3])
    {
        const float tilt = acosf(constrain(dot(levelUnit, poseUnit), -1.0f, 1.0f)) * RAD_TO_DEG;
        return tilt >= MIN_TILT_DEG && tilt <= MAX_TILT_DEG;
    }

    // Строки ортонормированы и тройка правая — иначе в NVS мусор.
    bool isRotation() const
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (fabsf(dot(r[i], r[i]) - 1.0f) > 0.01f) return false;
            if (fabsf(dot(r[i], r[(i + 1) % 3])) > 0.01f) return false;
        }
        float xy[3];
        cross(r[0], r[1], xy);
        return dot(xy, r[2]) > 0.99f;
    }

    static void describeAxis(Print& out, const float axis[3])
    {
        uint8_t best = 0;
        for (uint8_t i = 1; i < 3; ++i)
        {
            if (fabsf(axis[i]) > fabsf(axis[best])) best = i;
        }
        out.print(axis[best] >= 0 ? "+" : "-");
        out.print("XYZ"[best]);
        out.print(" чипа");
        if (fabsf(axis[best]) < 0.97f)
        {
            out.print(" (под углом ");
            out.print(acosf(fabsf(axis[best])) * RAD_TO_DEG, 0);
            out.print("°)");
        }
    }
};
