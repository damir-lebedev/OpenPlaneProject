#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "copter/Filters.h"

// ============================================================
// Рамы мультикоптеров: где стоят моторы, куда крутятся, как в них
// раскладываются крен, тангаж и рысканье.
//
// Оси — как во всей прошивке: X к носу, Y вправо (здесь — для
// координат моторов), крен > 0 — правый борт вниз, тангаж > 0 — нос
// вверх, рысканье > 0 — нос вправо. Угол мотора — по часовой от носа,
// если смотреть сверху.
//
// Крен вправо — моторы слева сильнее: коэффициент крена = −y. Нос
// вверх — передние сильнее: тангаж = x. Пропеллер, крутящийся против
// часовой (сверху), разворачивает раму по часовой — нос вправо:
// рысканье = +1 у CCW, −1 у CW. Коэффициенты крена и тангажа приводятся
// к максимуму 1 по каждой оси; геометрия (плечи) остаётся для расчёта
// настройки (Presets.h) и симуляции.
//
// Нумерация квадрокоптеров — как в Betaflight (4-в-1 регуляторы и все
// схемы подключения — под неё): 1 — задний правый, 2 — передний правый,
// 3 — задний левый, 4 — передний левый. Пропеллеры внутрь (props in):
// 1 и 4 — по часовой, 2 и 3 — против. У рам с 6 и 8 моторами — по
// часовой сверху, начиная с правого переднего; у соосных (Y6, X8)
// верхний мотор плеча раньше нижнего.
// ============================================================

namespace copter
{
    constexpr uint8_t MAX_MOTORS = 8;

    enum class FrameType : uint8_t
    {
        QUAD_X,            // классика 3–10": все плечи под 45°
        QUAD_X_STRETCHED,  // вытянутый X: длиннее, чем шире (гонки)
        QUAD_H,            // H: шире, чем длиннее (синелифтеры)
        QUAD_DEADCAT,      // передние плечи разведены — пропеллеры не в кадре
        QUAD_PLUS,         // «+»: мотор спереди, сзади, слева, справа
        HEX_X,
        HEX_PLUS,
        Y6,                // три плеча, на каждом два мотора соосно
        OCTO_X,
        OCTO_PLUS,
        X8,                // квадро-X, на каждом плече два мотора соосно
    };

    enum class Spin : int8_t { CW = -1, CCW = 1 };

    struct MotorPlace
    {
        float x;        // вперёд, в долях плеча
        float y;        // вправо
        Spin spin;
        bool bottom;    // нижний в соосной паре
    };

    struct MotorMix
    {
        float roll;
        float pitch;
        float yaw;
    };

    struct Frame
    {
        FrameType type = FrameType::QUAD_X;
        uint8_t count = 0;
        MotorPlace place[MAX_MOTORS] = {};
        MotorMix mix[MAX_MOTORS] = {};
        bool coaxial = false;
    };

    inline const char* frameName(FrameType t)
    {
        switch (t)
        {
            case FrameType::QUAD_X: return "Quad X";
            case FrameType::QUAD_X_STRETCHED: return "Quad X (вытянутый)";
            case FrameType::QUAD_H: return "Quad H";
            case FrameType::QUAD_DEADCAT: return "Quad Deadcat";
            case FrameType::QUAD_PLUS: return "Quad +";
            case FrameType::HEX_X: return "Hex X";
            case FrameType::HEX_PLUS: return "Hex +";
            case FrameType::Y6: return "Y6";
            case FrameType::OCTO_X: return "Octo X";
            case FrameType::OCTO_PLUS: return "Octo +";
            case FrameType::X8: return "X8";
        }
        return "?";
    }

    namespace detail
    {
        inline MotorPlace polar(float angleDeg, Spin spin, bool bottom = false)
        {
            const float a = angleDeg * PI_F / 180.0f;
            return { cosf(a), sinf(a), spin, bottom };
        }

        inline void add(Frame& f, MotorPlace p)
        {
            if (f.count < MAX_MOTORS) f.place[f.count++] = p;
        }

        // Квадро по координатам (x вперёд, y вправо) переднего правого мотора:
        // порядок Betaflight, пропеллеры внутрь.
        inline void quad(Frame& f, float frontX, float frontY, float rearX, float rearY)
        {
            add(f, { -rearX, rearY, Spin::CW, false });     // 1 задний правый
            add(f, { frontX, frontY, Spin::CCW, false });   // 2 передний правый
            add(f, { -rearX, -rearY, Spin::CCW, false });   // 3 задний левый
            add(f, { frontX, -frontY, Spin::CW, false });   // 4 передний левый
        }
    }

    // Рама: места моторов и коэффициенты смешения. propsOut — пропеллеры
    // наружу: все моторы крутятся наоборот.
    inline Frame buildFrame(FrameType type, bool propsOut = false)
    {
        using namespace detail;
        Frame f;
        f.type = type;
        const float d = 0.70710678f;   // cos 45°
        switch (type)
        {
            case FrameType::QUAD_X: quad(f, d, d, d, d); break;
            case FrameType::QUAD_X_STRETCHED: quad(f, 0.78f, 0.63f, 0.78f, 0.63f); break;
            case FrameType::QUAD_H: quad(f, 0.55f, 0.835f, 0.55f, 0.835f); break;
            case FrameType::QUAD_DEADCAT: quad(f, 0.5f, 0.866f, 0.8f, 0.6f); break;
            case FrameType::QUAD_PLUS:
                add(f, polar(180, Spin::CW));    // 1 задний
                add(f, polar(90, Spin::CCW));    // 2 правый
                add(f, polar(270, Spin::CCW));   // 3 левый
                add(f, polar(0, Spin::CW));      // 4 передний
                break;
            case FrameType::HEX_X:
                for (int i = 0; i < 6; ++i) add(f, polar(30.0f + 60.0f * i, i % 2 ? Spin::CW : Spin::CCW));
                break;
            case FrameType::HEX_PLUS:
                for (int i = 0; i < 6; ++i) add(f, polar(60.0f * i, i % 2 ? Spin::CCW : Spin::CW));
                break;
            case FrameType::Y6:
                for (int i = 0; i < 3; ++i)
                {
                    const float a = 60.0f + 120.0f * i;   // 60° — правый передний, 180° — задний, 300° — левый передний
                    add(f, polar(a, Spin::CCW, false));
                    add(f, polar(a, Spin::CW, true));
                }
                f.coaxial = true;
                break;
            case FrameType::OCTO_X:
                for (int i = 0; i < 8; ++i) add(f, polar(22.5f + 45.0f * i, i % 2 ? Spin::CW : Spin::CCW));
                break;
            case FrameType::OCTO_PLUS:
                for (int i = 0; i < 8; ++i) add(f, polar(45.0f * i, i % 2 ? Spin::CCW : Spin::CW));
                break;
            case FrameType::X8:
            {
                Frame top;
                quad(top, d, d, d, d);
                for (int i = 0; i < 4; ++i) add(f, top.place[i]);
                for (int i = 0; i < 4; ++i)
                {
                    MotorPlace p = top.place[i];
                    p.spin = p.spin == Spin::CW ? Spin::CCW : Spin::CW;
                    p.bottom = true;
                    add(f, p);
                }
                f.coaxial = true;
                break;
            }
        }

        float maxRoll = 0.0f, maxPitch = 0.0f;
        for (uint8_t i = 0; i < f.count; ++i)
        {
            if (propsOut) f.place[i].spin = f.place[i].spin == Spin::CW ? Spin::CCW : Spin::CW;
            maxRoll = fmaxf(maxRoll, fabsf(f.place[i].y));
            maxPitch = fmaxf(maxPitch, fabsf(f.place[i].x));
        }
        for (uint8_t i = 0; i < f.count; ++i)
        {
            f.mix[i].roll = maxRoll > 0.0f ? -f.place[i].y / maxRoll : 0.0f;
            f.mix[i].pitch = maxPitch > 0.0f ? f.place[i].x / maxPitch : 0.0f;
            f.mix[i].yaw = static_cast<float>(static_cast<int8_t>(f.place[i].spin));
        }
        return f;
    }

    // Где мотор — словами, для консоли и проверки моторов.
    inline void describeMotor(const Frame& f, uint8_t index, char* out, uint8_t size)
    {
        if (index >= f.count || size < 2)
        {
            if (size) out[0] = 0;
            return;
        }
        const MotorPlace& p = f.place[index];
        const char* fb = p.x > 0.2f ? "передний" : (p.x < -0.2f ? "задний" : "");
        const char* lr = p.y > 0.2f ? "правый" : (p.y < -0.2f ? "левый" : "");
        const char* level = f.coaxial ? (p.bottom ? ", нижний" : ", верхний") : "";
        const char* spin = p.spin == Spin::CW ? "по часовой" : "против часовой";
        snprintf(out, size, "%s%s%s%s, %s", fb, (*fb && *lr) ? " " : "", lr, level, spin);
    }
}
