#pragma once
// Музыка моторами по оборотам (176c).
//
// Тоны DShot BEACON1..5 — пять фиксированных писков по ~0,26 с: мелодию они
// не передают (стенд 2026-11-27: «куча разнотоновых писков»). Произвольной
// ноты регуляторы (BLHeli_S/Bluejay, BLHeli_32, AM32) по DShot не играют —
// только стартовую мелодию из своей памяти при включении.
//
// Здесь высота звука — сами обороты: мотор без винта поёт на электрической
// частоте (обороты × пар полюсов) и её гармониках. Каждый мотор — голос: он
// держит заданную частоту по телеметрии DShot (dshot_bidir), без неё —
// по модели «Гц на единицу команды» из KV и напряжения. Пауза — холостые
// (0 у DShot — минимальный газ регулятора), «выкл» — мотор стоит.
//
// Безопасность — как у проверки моторов: только без ARM и только без винтов,
// команда не выше MAX_COMMAND, ноты не приходят HOLD_S — тишина и стоп,
// ARM и любая проверка моторов музыку останавливают.
//
// ОЗУ у F405 впритык: голосов четыре (у гекса и окто играют первые четыре
// мотора, остальные стоят), голос — 12 байт, модель «Гц на команду» — одна
// на все моторы (они одинаковые; разницу добирает интегратор).
// Код свой.

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "copter/Frames.h"
#include "copter/Turtle.h"

namespace copter
{
    class MotorMusic
    {
    public:
        static constexpr float MAX_COMMAND = 0.35f;   // скважность, без винтов
        static constexpr float HOLD_S = 1.0f;         // без нот дольше — стоп
        static constexpr float OFF = -1.0f;           // нота «мотор стоит»
        static constexpr float MAX_HZ = 16000.0f;
        static constexpr uint8_t ALL = 0xFF;
        static constexpr uint8_t VOICES = 4;

        // count — моторов; hzPerCommand — оборотов в секунду на единицу команды
        // без нагрузки (KV × В / 60); mult — звук / обороты (пар полюсов для
        // электрической частоты).
        bool start(uint8_t count, float hzPerCommand, float mult)
        {
            if (count == 0 || count > MAX_MOTORS || !(hzPerCommand > 1.0f) || !(mult >= 0.5f && mult <= 64.0f))
                return false;
            motors = count < VOICES ? count : VOICES;
            multiplier = mult;
            gainHz = hzPerCommand;
            for (Voice& s : v) s = Voice{};
            holdLeft = HOLD_S;
            clock = 0.0f;
            on = true;
            return true;
        }
        void stop()
        {
            on = false;
            holdLeft = 0.0f;
        }
        bool active() const { return on; }

        // Нота мотору (Гц звука): 0 — пауза на холостых, OFF — мотор стоит.
        // Любая нота продлевает музыку на HOLD_S.
        bool note(uint8_t motor, float soundHz)
        {
            if (!on) return false;
            if (motor != ALL && motor >= motors) return false;
            if (!(soundHz >= OFF) || soundHz > MAX_HZ) return false;
            const uint16_t t = soundHz < 0.0f ? T_OFF : static_cast<uint16_t>(soundHz * 4.0f + 0.5f);
            for (uint8_t i = 0; i < motors; ++i)
            {
                if (motor != ALL && i != motor) continue;
                // сменилась больше чем на 1 % — модель не учить, пока мотор не дойдёт
                const uint16_t was = v[i].target4;
                if (was == T_OFF || t == T_OFF || abs(static_cast<int>(t) - static_cast<int>(was)) * 100 > was)
                    v[i].changed10 = tick10();
                v[i].target4 = t;
            }
            holdLeft = HOLD_S;
            return true;
        }

        // Такт: motorHz — обороты, Гц (телеметрия, если hzOk); cmd — команды
        // моторам 0..MAX_COMMAND или MOTOR_STOP. false — музыка кончилась.
        bool update(float dt, const float* motorHz, bool hzOk, float* cmd)
        {
            if (!on) return false;
            holdLeft -= dt;
            if (holdLeft <= 0.0f)
            {
                stop();
                return false;
            }
            clock += dt;
            if (clock > 1000.0f) clock -= 1000.0f;   // точность float; tick10 — по модулю 256
            uint8_t playing = 0;   // модель общая: каждый звучащий голос учит её своей долей
            for (uint8_t i = 0; i < motors; ++i) playing += v[i].target4 != T_OFF && v[i].target4 > 0;
            if (playing == 0) playing = 1;
            for (uint8_t i = 0; i < MAX_MOTORS; ++i)
            {
                if (i >= motors)
                {
                    cmd[i] = MOTOR_STOP;
                    continue;
                }
                Voice& s = v[i];
                s.flags = 0;
                if (s.target4 == T_OFF)
                {
                    cmd[i] = MOTOR_STOP;
                    s.integral = 0.0f;
                    s.command = 0;
                    s.measured4 = 0;
                    continue;
                }
                const bool meas = hzOk && motorHz && motorHz[i] > 0.0f;
                s.measured4 = meas ? static_cast<uint16_t>(fminf(65000.0f, motorHz[i] * multiplier * 4.0f + 0.5f)) : 0;
                if (s.target4 == 0)
                {
                    // пауза: холостые регулятора
                    cmd[i] = 0.0f;
                    s.command = 0;
                    s.integral = 0.0f;
                    continue;
                }
                const float want = s.target4 * 0.25f / multiplier;   // обороты, Гц
                float c = want / gainHz;
                float dI = 0.0f;
                if (meas)
                {
                    const float err = want - motorHz[i];
                    // модель «Гц на команду» учится на установившемся звуке (раз
                    // в 2,56 с счётчик переходит через ноль — 80 мс без учёбы, не страшно)
                    const float prev = command(i);
                    const uint8_t since = static_cast<uint8_t>(tick10() - s.changed10);
                    if (prev > 0.03f && since > SETTLE_10MS && fabsf(err) < 0.15f * want)
                    {
                        const float g = motorHz[i] / prev;
                        gainHz += (g - gainHz) * fminf(1.0f, dt / (GAIN_TAU_S * playing));
                    }
                    dI = KI * err / gainHz * dt;
                    s.integral = fminf(I_LIMIT, fmaxf(-I_LIMIT, s.integral + dI));
                    c += KP * err / gainHz + s.integral;
                }
                const bool low = c < 0.0f, high = c > MAX_COMMAND;
                s.flags = static_cast<uint8_t>((low ? FLAG_LOW : 0) | (high ? FLAG_HIGH : 0));
                c = fminf(MAX_COMMAND, fmaxf(0.0f, c));
                if ((high && dI > 0.0f) || (low && dI < 0.0f)) s.integral -= dI;   // без накрутки в упоре
                s.command = static_cast<uint16_t>(c / MAX_COMMAND * 65535.0f + 0.5f);
                cmd[i] = c;
            }
            return true;
        }

        uint8_t count() const { return motors; }
        float mult() const { return multiplier; }
        float gain() const { return gainHz; }
        float targetHz(uint8_t i) const
        {
            return i >= VOICES || v[i].target4 == T_OFF ? OFF : v[i].target4 * 0.25f;
        }
        float measuredHz(uint8_t i) const { return i < VOICES ? v[i].measured4 * 0.25f : 0.0f; }
        float command(uint8_t i) const { return i < VOICES ? v[i].command * (MAX_COMMAND / 65535.0f) : 0.0f; }
        bool tooLow(uint8_t i) const { return i < VOICES && (v[i].flags & FLAG_LOW); }    // ниже холостых
        bool tooHigh(uint8_t i) const { return i < VOICES && (v[i].flags & FLAG_HIGH); }  // выше MAX_COMMAND

        // Самая высокая нота, которую моторы возьмут (по выученной модели).
        float maxSoundHz() const { return gainHz * MAX_COMMAND * multiplier; }

    private:
        static constexpr float KP = 0.6f;          // доля ошибки сразу в команду
        static constexpr float KI = 8.0f;          // 1/с
        static constexpr float I_LIMIT = 0.08f;    // доля команды
        static constexpr uint8_t SETTLE_10MS = 8;   // после смены ноты 80 мс модель не учить
        static constexpr float GAIN_TAU_S = 0.5f;   // на голос
        static constexpr uint16_t T_OFF = 0xFFFF;
        static constexpr uint8_t FLAG_LOW = 1, FLAG_HIGH = 2;

        struct Voice
        {
            float integral = 0.0f;
            uint16_t target4 = T_OFF;   // Гц × 4; 0 — пауза
            uint16_t command = 0;       // доля MAX_COMMAND × 65535
            uint16_t measured4 = 0;     // звук по оборотам, Гц × 4
            uint8_t changed10 = 0;      // когда сменилась нота, по 10 мс (tick10)
            uint8_t flags = 0;
        };
        uint8_t tick10() const { return static_cast<uint8_t>(static_cast<uint32_t>(clock * 100.0f) & 0xFF); }

        Voice v[VOICES];
        float multiplier = 7.0f;
        float gainHz = 0.0f;
        float holdLeft = 0.0f;
        float clock = 0.0f;   // с начала музыки, с (по модулю 1000)
        uint8_t motors = 0;
        bool on = false;
    };
}
