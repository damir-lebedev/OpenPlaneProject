#pragma once
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry/BlackBoxFormat.h"

// ============================================================
// Журнал полёта в формате Betaflight (.BFL) — открывается в Blackbox
// Explorer и PIDtoolbox. Своя реализация по описанию формата (данные v2):
//
//   заголовок — строки «H ключ:значение\n»: что это за журнал, поля
//     главного кадра (имена, знак, предсказатель, кодирование для кадров
//     I и P), поля медленного кадра S, настройки (PID, частота цикла,
//     диапазон моторов);
//   кадр I ('I' + значения) — каждые I_INTERVAL кадров, значения как есть;
//   кадр P ('P' + значения) — остальные: разность с прошлым кадром
//     (предсказатель 1 «предыдущее»);
//   кадр S ('S') — режимы, failsafe, связь — когда они меняются;
//   кадр E ('E', 255, "End of log") — конец журнала.
// Числа — переменной длины, по 7 бит в байте, младшие первыми (кодирование
// 1 — без знака, 0 — со знаком через zigzag: 0, −1, 1, −2 -> 0, 1, 2, 3).
//
// Поля и единицы — как у BF: axisP/I/D/F — сумма PID в тысячных хода
// моторов (у нас 1e-4 -> /10), rcCommand — стики ±500 и газ 1000–2000,
// setpoint — °/с и газ 0–1000, gyroADC и gyroUnfilt — °/с (gyro_scale 1),
// motor — 48–2047 (motorOutput), vbatLatest — 0.01 В, amperageLatest —
// 0.01 А. Заголовок представляется «Betaflight 4.5.0 (OpenPlane …)» —
// иначе Explorer не знает, как читать поля (то же решение, что для MSP).
//
// CopterBflConverter переводит записи нашего ящика (QGYRO, QCTRL, QMOT8,
// QSTATE, INFO — telemetry/CopterBlackBox.h) в кадры: один кадр на цикл,
// что записан (blackbox_sample_rate). Без Arduino — тесты на ПК.
// ============================================================

namespace bfl
{
    class Sink
    {
    public:
        virtual ~Sink() = default;
        virtual void put(const uint8_t* data, size_t length) = 0;
    };

    constexpr uint8_t ENC_SIGNED_VB = 0;
    constexpr uint8_t ENC_UNSIGNED_VB = 1;
    constexpr uint8_t PRED_ZERO = 0;
    constexpr uint8_t PRED_PREVIOUS = 1;
    constexpr uint8_t EVENT_LOG_END = 255;
    constexpr uint16_t I_INTERVAL = 32;
    constexpr uint8_t MAX_MOTORS = 8;

    inline uint32_t zigzag(int32_t v) { return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31); }

    // Поля главного кадра — по порядку.
    enum Field : uint8_t
    {
        F_LOOP,
        F_TIME,
        F_AXIS_P, F_AXIS_I = F_AXIS_P + 3, F_AXIS_D = F_AXIS_I + 3, F_AXIS_F = F_AXIS_D + 3,
        F_RC = F_AXIS_F + 3,
        F_SETPOINT = F_RC + 4,
        F_VBAT = F_SETPOINT + 4,
        F_AMPS,
        F_GYRO,
        F_GYRO_UNFILT = F_GYRO + 3,
        F_MOTOR = F_GYRO_UNFILT + 3,
        F_COUNT_NO_MOTORS = F_MOTOR,
    };

    struct Header
    {
        uint32_t frameIntervalUs = 1000;   // между кадрами журнала (цикл × blackbox_sample_rate)
        uint8_t motors = 4;
        char board[32] = "OpenPlane";
        char firmware[48] = "";            // коммит прошивки
        char craft[32] = "";               // сборка
        int pid[3][4] = {};                // P I D F по осям, единицы BF
        uint8_t motorPoles = 14;
    };

    class Writer
    {
    public:
        explicit Writer(Sink& out)
            : sink(out)
        {
        }

        uint8_t fieldCount() const { return static_cast<uint8_t>(F_COUNT_NO_MOTORS + motors); }

        void begin(const Header& h)
        {
            motors = h.motors > MAX_MOTORS ? MAX_MOTORS : (h.motors ? h.motors : 1);
            frames = 0;
            line("Product:Blackbox flight data recorder by Nicholas Sherlock");
            line("Data version:2");
            line("I interval:%u", I_INTERVAL);
            line("P interval:1/1");
            line("P ratio:%u", I_INTERVAL);
            line("Firmware type:Cleanflight");
            line("Firmware revision:Betaflight 4.5.0 (OpenPlane %s) %s", h.firmware, h.board);
            line("Firmware date:%s", "Jan  1 2026 00:00:00");
            line("Board information:OpenPlane %s", h.board);
            line("Log start datetime:0000-01-01T00:00:00.000+00:00");
            line("Craft name:%s", h.craft);
            // поля главного кадра
            char buf[640];
            fieldList(buf, sizeof(buf), [](uint8_t i, char* o, size_t n) { fieldName(i, o, n); });
            line("Field I name:%s", buf);
            fieldList(buf, sizeof(buf), [](uint8_t i, char* o, size_t n) {
                snprintf(o, n, "%u", (i >= F_AXIS_P && i != F_VBAT && i != F_AMPS) ? 1u : 0u);
            });
            line("Field I signed:%s", buf);
            fieldList(buf, sizeof(buf), [](uint8_t, char* o, size_t n) { snprintf(o, n, "%u", PRED_ZERO); });
            line("Field I predictor:%s", buf);
            fieldList(buf, sizeof(buf), [](uint8_t i, char* o, size_t n) {
                snprintf(o, n, "%u", (i == F_LOOP || i == F_TIME || i == F_VBAT || i == F_AMPS) ? ENC_UNSIGNED_VB : ENC_SIGNED_VB);
            });
            line("Field I encoding:%s", buf);
            fieldList(buf, sizeof(buf), [](uint8_t, char* o, size_t n) { snprintf(o, n, "%u", PRED_PREVIOUS); });
            line("Field P predictor:%s", buf);
            fieldList(buf, sizeof(buf), [](uint8_t, char* o, size_t n) { snprintf(o, n, "%u", ENC_SIGNED_VB); });
            line("Field P encoding:%s", buf);
            line("Field S name:flightModeFlags,stateFlags,failsafePhase,rxSignalReceived,rxFlightChannelsValid");
            line("Field S signed:0,0,0,0,0");
            line("Field S predictor:0,0,0,0,0");
            line("Field S encoding:1,1,1,1,1");
            // настройки
            line("looptime:%lu", static_cast<unsigned long>(h.frameIntervalUs));
            line("gyro_sync_denom:1");
            line("pid_process_denom:1");
            line("minthrottle:1070");
            line("maxthrottle:2000");
            line("motorOutput:48,2047");
            line("acc_1G:2048");
            line("gyro_scale:0x3f800000");
            line("vbatscale:110");
            line("vbatref:0");
            line("currentSensor:0,400");
            static const char* const AXES[3] = { "roll", "pitch", "yaw" };
            for (uint8_t a = 0; a < 3; ++a) line("%sPID:%d,%d,%d", AXES[a], h.pid[a][0], h.pid[a][1], h.pid[a][2]);
            line("ff_weight:%d,%d,%d", h.pid[0][3], h.pid[1][3], h.pid[2][3]);
            line("motor_poles:%u", h.motorPoles);
            line("debug_mode:0");
        }

        // Значения главного кадра по порядку полей (fieldCount()).
        void mainFrame(const int32_t* v)
        {
            const bool intra = frames % I_INTERVAL == 0;
            byte(intra ? 'I' : 'P');
            for (uint8_t i = 0; i < fieldCount(); ++i)
            {
                if (intra)
                {
                    const bool unsignedField = i == F_LOOP || i == F_TIME || i == F_VBAT || i == F_AMPS;
                    if (unsignedField) unsignedVb(static_cast<uint32_t>(v[i] < 0 ? 0 : v[i]));
                    else signedVb(v[i]);
                }
                else
                {
                    signedVb(static_cast<int32_t>(static_cast<uint32_t>(v[i]) - static_cast<uint32_t>(prev[i])));
                }
                prev[i] = v[i];
            }
            ++frames;
        }

        void slowFrame(uint32_t flightModes, uint32_t stateFlags, uint32_t failsafePhase, bool rxOk, bool rxValid)
        {
            byte('S');
            unsignedVb(flightModes);
            unsignedVb(stateFlags);
            unsignedVb(failsafePhase);
            unsignedVb(rxOk ? 1 : 0);
            unsignedVb(rxValid ? 1 : 0);
        }

        void end()
        {
            byte('E');
            byte(EVENT_LOG_END);
            static const char END[] = "End of log";
            sink.put(reinterpret_cast<const uint8_t*>(END), sizeof(END));   // с нулём в конце
        }

        uint32_t frameCount() const { return frames; }

        static void fieldName(uint8_t i, char* out, size_t n)
        {
            if (i == F_LOOP) snprintf(out, n, "loopIteration");
            else if (i == F_TIME) snprintf(out, n, "time");
            else if (i < F_AXIS_I) snprintf(out, n, "axisP[%u]", i - F_AXIS_P);
            else if (i < F_AXIS_D) snprintf(out, n, "axisI[%u]", i - F_AXIS_I);
            else if (i < F_AXIS_F) snprintf(out, n, "axisD[%u]", i - F_AXIS_D);
            else if (i < F_RC) snprintf(out, n, "axisF[%u]", i - F_AXIS_F);
            else if (i < F_SETPOINT) snprintf(out, n, "rcCommand[%u]", i - F_RC);
            else if (i < F_VBAT) snprintf(out, n, "setpoint[%u]", i - F_SETPOINT);
            else if (i == F_VBAT) snprintf(out, n, "vbatLatest");
            else if (i == F_AMPS) snprintf(out, n, "amperageLatest");
            else if (i < F_GYRO_UNFILT) snprintf(out, n, "gyroADC[%u]", i - F_GYRO);
            else if (i < F_MOTOR) snprintf(out, n, "gyroUnfilt[%u]", i - F_GYRO_UNFILT);
            else snprintf(out, n, "motor[%u]", i - F_MOTOR);
        }

    private:
        template <typename F> void fieldList(char* buf, size_t size, F item)
        {
            size_t at = 0;
            buf[0] = '\0';
            for (uint8_t i = 0; i < fieldCount() && at + 1 < size; ++i)
            {
                char one[24];
                item(i, one, sizeof(one));
                at += static_cast<size_t>(snprintf(buf + at, size - at, "%s%s", i ? "," : "", one));
            }
        }

        void line(const char* fmt, ...)
        {
            char text[700];
            text[0] = 'H';
            text[1] = ' ';
            va_list args;
            va_start(args, fmt);
            int n = vsnprintf(text + 2, sizeof(text) - 3, fmt, args);
            va_end(args);
            if (n < 0) return;
            size_t len = 2 + static_cast<size_t>(n) < sizeof(text) - 1 ? 2 + static_cast<size_t>(n) : sizeof(text) - 2;
            text[len++] = '\n';
            sink.put(reinterpret_cast<const uint8_t*>(text), len);
        }

        void byte(uint8_t b) { sink.put(&b, 1); }

        void unsignedVb(uint32_t v)
        {
            uint8_t out[5];
            size_t n = 0;
            while (v >= 0x80)
            {
                out[n++] = static_cast<uint8_t>(v | 0x80);
                v >>= 7;
            }
            out[n++] = static_cast<uint8_t>(v);
            sink.put(out, n);
        }

        void signedVb(int32_t v) { unsignedVb(zigzag(v)); }

        Sink& sink;
        uint8_t motors = 4;
        uint32_t frames = 0;
        int32_t prev[F_COUNT_NO_MOTORS + MAX_MOTORS] = {};
    };

    // Записи ящика коптера -> кадры BF. Заголовок — по INFO полёта (они идут
    // первыми), кадры — с первого QGYRO.
    class CopterBflConverter
    {
    public:
        explicit CopterBflConverter(Writer& w)
            : writer(w)
        {
        }

        void record(uint8_t type, const uint8_t* data, size_t length)
        {
            using namespace BlackBoxFormat;
            switch (type)
            {
                case REC_INFO:
                    if (length > 4) info(reinterpret_cast<const char*>(data + 4), length - 4);
                    break;
                case REC_QGYRO:
                    if (length < sizeof(CopterGyroRecord)) break;
                    flushFrame();
                    memcpy(&gyro, data, sizeof(gyro));
                    haveGyro = true;
                    break;
                case REC_QCTRL:
                    if (length < sizeof(CopterCtrlRecord)) break;
                    memcpy(&ctrl, data, sizeof(ctrl));
                    haveCtrl = true;
                    break;
                case REC_QMOT8:
                    if (length < sizeof(CopterMotor8Record)) break;
                    memcpy(&mot8, data, sizeof(mot8));
                    break;
                case REC_QSTATE:
                    if (length < sizeof(CopterStateRecord)) break;
                    memcpy(&state, data, sizeof(state));
                    haveState = true;
                    break;
                default:
                    break;
            }
        }

        // Последний кадр, конец журнала.
        void finish()
        {
            flushFrame();
            if (started) writer.end();
        }

        // Все записи сектора (4096 байт) по порядку — как в нашем формате.
        void sector(const uint8_t* s)
        {
            using namespace BlackBoxFormat;
            SectorHeader h;
            memcpy(&h, s, sizeof(h));
            if (!isValid(h)) return;
            size_t pos = SECTOR_HEADER_SIZE;
            while (pos + 2 <= SECTOR_SIZE)
            {
                const uint8_t kind = s[pos], len = s[pos + 1];
                const size_t end = pos + 2 + len;
                if (kind == 0xFF || end + 1 > SECTOR_SIZE) break;
                if (crc8(s + pos, 2 + len) != s[end]) break;
                record(kind, s + pos + 2, len);
                pos = end + 1;
            }
        }

        uint32_t frames() const { return writer.frameCount(); }
        const Header& header() const { return hdr; }

    private:
        void info(const char* text, size_t length)
        {
            char buf[256];
            const size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
            memcpy(buf, text, n);
            buf[n] = '\0';
            const char* eq = strchr(buf, '=');
            if (!eq) return;
            const char* v = eq + 1;
            const size_t keyLen = static_cast<size_t>(eq - buf);
            auto is = [&](const char* key) { return strlen(key) == keyLen && strncmp(buf, key, keyLen) == 0; };
            if (is("motors")) hdr.motors = static_cast<uint8_t>(atoi(v));
            else if (is("rate.gyro_hz")) { const float hz = static_cast<float>(atof(v)); if (hz > 0.0f) hdr.frameIntervalUs = static_cast<uint32_t>(1.0e6f / hz + 0.5f); }
            else if (is("board")) snprintf(hdr.board, sizeof(hdr.board), "%s", v);
            else if (is("firmware")) snprintf(hdr.firmware, sizeof(hdr.firmware), "%s", v);
            else if (is("build")) snprintf(hdr.craft, sizeof(hdr.craft), "%s", v);
            else if (is("set.motor_poles")) hdr.motorPoles = static_cast<uint8_t>(atoi(v));
            else if (is("pid.roll") || is("pid.pitch") || is("pid.yaw"))
            {
                const int a = is("pid.roll") ? 0 : is("pid.pitch") ? 1 : 2;
                float x[4] = {};
                if (sscanf(v, "%f %f %f %f", &x[0], &x[1], &x[2], &x[3]) >= 3)
                {
                    for (int k = 0; k < 4; ++k) hdr.pid[a][k] = static_cast<int>(lroundf(x[k]));
                }
            }
        }

        void flushFrame()
        {
            if (!haveGyro || !haveCtrl) return;
            if (!started)
            {
                writer.begin(hdr);
                started = true;
                firstUs = gyro.tUs;
            }
            int32_t v[F_COUNT_NO_MOTORS + MAX_MOTORS] = {};
            v[F_LOOP] = static_cast<int32_t>(loop++);
            v[F_TIME] = static_cast<int32_t>(gyro.tUs - firstUs);
            for (uint8_t a = 0; a < 3; ++a)
            {
                v[F_AXIS_P + a] = ctrl.pid[a][0] / 10;
                v[F_AXIS_I + a] = ctrl.pid[a][1] / 10;
                v[F_AXIS_D + a] = ctrl.pid[a][2] / 10;
                v[F_AXIS_F + a] = ctrl.pid[a][3] / 10;
                v[F_SETPOINT + a] = lroundf(gyro.setpoint[a] / 10.0f);
                v[F_GYRO + a] = lroundf(gyro.gyroF[a] / 10.0f);
                v[F_GYRO_UNFILT + a] = lroundf(gyro.gyro[a] / 10.0f);
            }
            v[F_SETPOINT + 3] = ctrl.thrust;
            if (haveState)
            {
                for (uint8_t a = 0; a < 3; ++a) v[F_RC + a] = state.rc[a] / 2;   // 1e-3 -> ±500
                v[F_RC + 3] = 1000 + state.rcThrottle;
                v[F_VBAT] = state.vbatMv / 10;
                v[F_AMPS] = state.ampsCa;
            }
            else
            {
                v[F_RC + 3] = 1000;
            }
            const uint8_t m = writer.fieldCount() - F_MOTOR;
            for (uint8_t i = 0; i < m; ++i)
            {
                const uint16_t cmd = i < 4 ? ctrl.motor[i] : mot8.motor[i - 4];
                v[F_MOTOR + i] = 48 + static_cast<int32_t>(cmd) * 1999 / 1000;
            }
            writer.mainFrame(v);
            haveCtrl = false;
            slowIfChanged();
        }

        // Режим, failsafe, связь — кадром S, когда поменялись. Флаги режимов —
        // как у BF: ANGLE 1, HORIZON 2, ALTHOLD 8; фаза failsafe — этап.
        void slowIfChanged()
        {
            if (!haveState) return;
            using namespace BlackBoxFormat;
            uint32_t modes = 0;
            if (state.mode == 2) modes |= 1u;        // ANGLE (copter::CopterMode)
            else if (state.mode == 1) modes |= 2u;   // HORIZON
            else if (state.mode == 3) modes |= 8u;   // ALT_HOLD
            const uint32_t phase = state.state == 2 ? 1u : 0u;   // FAILSAFE
            const bool rxOk = (state.flags & CopterFlag::RX_LOST) == 0;
            const uint32_t key = modes | (phase << 8) | (rxOk ? 1u << 12 : 0u);
            if (slowSent && key == lastSlow) return;
            writer.slowFrame(modes, (state.flags & CopterFlag::ARMED) ? 1u : 0u, phase, rxOk, rxOk);
            lastSlow = key;
            slowSent = true;
        }

        Writer& writer;
        Header hdr;
        BlackBoxFormat::CopterGyroRecord gyro{};
        BlackBoxFormat::CopterCtrlRecord ctrl{};
        BlackBoxFormat::CopterMotor8Record mot8{};
        BlackBoxFormat::CopterStateRecord state{};
        bool haveGyro = false, haveCtrl = false, haveState = false, started = false, slowSent = false;
        uint32_t loop = 0, firstUs = 0, lastSlow = 0;
    };
}
