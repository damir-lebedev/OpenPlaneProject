// Реализация sensors/imu/ImuOrientation.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/imu/ImuOrientation.h"


ImuOrientation::ImuOrientation()
{
    *this = fromYawSteps(0);
}

auto ImuOrientation::fromYawSteps(uint16_t rotationCwDeg) -> ImuOrientation
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

auto ImuOrientation::fromPoses(const float level[3], const float noseUp[3],
                                 const float rightWingDown[3], ImuOrientation& out) -> const char*
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

auto ImuOrientation::apply(const float chip[3], float body[3]) const -> void
{
    for (uint8_t i = 0; i < 3; ++i)
    {
        body[i] = r[i][0] * chip[0] + r[i][1] * chip[1] + r[i][2] * chip[2];
    }
}

auto ImuOrientation::tiltFromLevelDeg(const float chipUp[3]) const -> float
{
    float body[3], unit[3];
    apply(chipUp, body);
    if (!normalize(body, unit)) return 180.0f;
    return acosf(constrain(unit[2], -1.0f, 1.0f)) * static_cast<float>(RAD_TO_DEG);
}

auto ImuOrientation::load(const char* nvsNamespace) -> bool
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

auto ImuOrientation::save(const char* nvsNamespace) const -> void
{
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, false)) return;
    prefs.putBytes("r", r, sizeof(r));
    prefs.putBool("ok", true);
    prefs.end();
}

auto ImuOrientation::describe(Print& out) const -> void
{
    out.print("нос = ");
    describeAxis(out, r[0]);
    out.print(", верх = ");
    describeAxis(out, r[2]);
}

auto ImuOrientation::dot(const float a[3], const float b[3]) -> float
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

auto ImuOrientation::cross(const float a[3], const float b[3], float out[3]) -> void
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

auto ImuOrientation::normalize(const float v[3], float out[3]) -> bool
{
    const float length = sqrtf(dot(v, v));
    if (length < 1e-6f) return false;
    for (uint8_t i = 0; i < 3; ++i) out[i] = v[i] / length;
    return true;
}

auto ImuOrientation::perpendicularPart(const float v[3], const float axis[3], float out[3]) -> void
{
    const float along = dot(v, axis);
    const float p[3] = { v[0] - along * axis[0], v[1] - along * axis[1], v[2] - along * axis[2] };
    normalize(p, out);
}

auto ImuOrientation::tiltInRange(const float levelUnit[3], const float poseUnit[3]) -> bool
{
    const float tilt = acosf(constrain(dot(levelUnit, poseUnit), -1.0f, 1.0f)) * static_cast<float>(RAD_TO_DEG);
    return tilt >= MIN_TILT_DEG && tilt <= MAX_TILT_DEG;
}

auto ImuOrientation::isRotation() const -> bool
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

auto ImuOrientation::describeAxis(Print& out, const float axis[3]) -> void
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
