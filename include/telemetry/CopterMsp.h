#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "copter/Battery.h"
#include "copter/CopterCli.h"
#include "copter/CopterController.h"
#include "copter/Params.h"
#include "copter/RcSetup.h"
#include "telemetry/Msp.h"

// ============================================================
// MSP коптера — чтобы работал конфигуратор Betaflight. Представляемся как
// BF 4.5 (FC_VARIANT «BTFL», MSP API 1.46, FC_VERSION 4.5.0): по ним
// конфигуратор решает, какие вкладки и поля показывать. В BOARD_INFO,
// BUILD_INFO и имени платы — честно OpenPlane.
//
// Ответы — по раскладкам BF 4.5 (номер команды, поля по порядку, младший
// байт первым), значения — из реестра настроек (copter/Params.h) и
// состояния контроллера. Записи (MSP_SET_*) меняют настройки в памяти,
// MSP_EEPROM_WRITE — сохраняет и применяет (как save в CLI); до него ARM
// запрещён (флаг MSP). Всё, что меняет настройки, — только без ARM.
//
// Чего у нас нет — отвечаем значением BF по умолчанию и при записи
// пропускаем: фильтры (кроме yaw_lowpass_hz — у нас они из модели),
// протокол регуляторов (выбирается сборкой), тип расходов (наша кривая —
// ACTUAL), features (у нас нет такого механизма). MSP_SET_MOTOR, то есть
// проверка моторов из конфигуратора, — пока нет: консоль, 'm'.
//
// Без Arduino — тесты на ПК (test_copter).
// ============================================================

namespace copter
{
    namespace mspcmd
    {
        enum : uint16_t
        {
            API_VERSION = 1, FC_VARIANT = 2, FC_VERSION = 3, BOARD_INFO = 4, BUILD_INFO = 5,
            NAME = 10, SET_NAME = 11,
            BATTERY_CONFIG = 32, SET_BATTERY_CONFIG = 33, MODE_RANGES = 34, SET_MODE_RANGE = 35,
            FEATURE_CONFIG = 36, SET_FEATURE_CONFIG = 37, MIXER_CONFIG = 42, RX_CONFIG = 44, SET_RX_CONFIG = 45,
            ARMING_CONFIG = 61, SET_ARMING_CONFIG = 62, RX_MAP = 64, SET_RX_MAP = 65, REBOOT = 68,
            FAILSAFE_CONFIG = 75, SET_FAILSAFE_CONFIG = 76, RXFAIL_CONFIG = 77, SET_RXFAIL_CONFIG = 78,
            ADVANCED_CONFIG = 90, SET_ADVANCED_CONFIG = 91, FILTER_CONFIG = 92, SET_FILTER_CONFIG = 93,
            PID_ADVANCED = 94, SET_PID_ADVANCED = 95, SET_ARMING_DISABLED = 99,
            STATUS = 101, RAW_IMU = 102, MOTOR = 104, RC = 105, ATTITUDE = 108, ALTITUDE = 109, ANALOG = 110,
            RC_TUNING = 111, PID = 112, BOXNAMES = 116, BOXIDS = 119, BATTERY_STATE = 130, MOTOR_CONFIG = 131,
            STATUS_EX = 150, UID = 160, SET_RAW_RC = 200, SET_PID = 202, SET_RC_TUNING = 204, SELECT_SETTING = 210,
            SET_MOTOR = 214, SET_MOTOR_CONFIG = 222, EEPROM_WRITE = 250,
        };
    }

    // Что MSP знает о плате и датчиках — main_copter.cpp.
    struct MspBoard
    {
        const char* boardName = "OpenPlane";   // OPB_BOARD_NAME / env
        const char* target = "OPENPLANE";
        const char* commit = "dev";            // OP_SOURCE_COMMIT
        uint8_t mcuType = 0;                   // как getMcuTypeId() у BF: F405 — 1, H743 — 12
        uint32_t uid[3] = {};
        uint16_t sensors = 0;                  // биты BF: 0 ACC, 1 BARO, 2 MAG, 3 GPS, 5 GYRO
        bool osd = false;                      // AT7456E на плате (board type 2)
        bool gps = false;
        uint8_t rcChannels = 10;
    };

    class CopterMsp
    {
    public:
        static constexpr uint32_t ARMING_HOLD_MS = 3000;   // конфигуратор молчит столько — флаг MSP снят

        CopterMsp(CopterController& controller, CopterParams& paramsRef, CopterCli& cliRef)
            : ctl(controller), params(paramsRef), cli(cliRef)
        {
        }

        void setBoard(const MspBoard& b) { board = b; }
        void setRx(const RxInput* input, const uint16_t* rawChannels, uint8_t count)
        {
            rx = input;
            raw = rawChannels;
            rawCount = count;
        }
        void setBattery(const BatteryMonitor* b) { battery = b; }
        void setOutputs(const CopterOutputs& out) { outputs = out; }
        void setImu(float gx, float gy, float gz, float ax, float ay, float az)
        {
            imu[0] = gx;
            imu[1] = gy;
            imu[2] = gz;
            imu[3] = ax;
            imu[4] = ay;
            imu[5] = az;
        }
        void setLoad(uint16_t percent) { loadPercent = percent; }
        void setLinkQuality(int16_t lq) { linkQuality = lq; }

        // Кадр пришёл — ответ в out (кадр целиком). 0 — отвечать нечем (не влезло).
        size_t handle(const msp::Parser& in, uint8_t* out, size_t capacity, uint32_t nowMs)
        {
            lastFrameMs = nowMs;
            seen = true;
            msp::Writer w;
            msp::Reader r(in.payload, in.size);
            const bool ok = command(in.cmd, r, w) && !w.overflow;
            ++frames;
            if (!ok) ++errors;
            return msp::frame(out, capacity, in.version, in.cmd, !ok, w.buf, ok ? w.length : 0);
        }

        // Конфигуратор запретил ARM (MSP_SET_ARMING_DISABLED) и ещё на связи,
        // или изменил настройки и не сохранил — ARM нельзя (флаг MSP, как у BF).
        bool blocksArming(uint32_t nowMs) const
        {
            if (unsaved) return true;
            return armingDisabled && seen && nowMs - lastFrameMs < ARMING_HOLD_MS;
        }
        bool hasUnsaved() const { return unsaved; }
        void settingsApplied() { unsaved = false; }

        // MSP_REBOOT: после отправки ответа — перезагрузка (0 — прошивка, 1 — загрузчик DFU).
        bool takeReboot(uint8_t& mode)
        {
            if (!rebootPending) return false;
            rebootPending = false;
            mode = rebootMode;
            return true;
        }

        uint32_t frames = 0, errors = 0;

    private:
        bool command(uint16_t cmd, msp::Reader& r, msp::Writer& w)
        {
            using namespace mspcmd;
            const bool armed = ctl.isArmed();
            switch (cmd)
            {
                case API_VERSION:
                    w.u8(0);
                    w.u8(1);
                    w.u8(46);
                    return true;
                case FC_VARIANT: w.text("BTFL"); return true;
                case FC_VERSION:
                    w.u8(4);
                    w.u8(5);
                    w.u8(0);
                    return true;
                case BOARD_INFO: boardInfo(w); return true;
                case BUILD_INFO: buildInfo(w); return true;
                case NAME: w.text(params.text(PARAM_CRAFT_NAME, 0)); return true;
                case UID:
                    for (uint32_t u : board.uid) w.u32(u);
                    return true;
                case STATUS:
                case STATUS_EX: status(w, cmd == STATUS_EX); return true;
                case RAW_IMU: rawImu(w); return true;
                case ATTITUDE: attitude(w); return true;
                case ALTITUDE:
                    w.u32(static_cast<uint32_t>(lroundf(ctl.getStatus().altitude * 100.0f)));
                    w.u16(static_cast<uint16_t>(static_cast<int16_t>(lroundf(ctl.getStatus().climbRate * 100.0f))));
                    return true;
                case ANALOG: analog(w); return true;
                case BATTERY_STATE: batteryState(w); return true;
                case RC: rc(w); return true;
                case MOTOR: motor(w); return true;
                case BOXNAMES: boxes(w, r.u8(), true); return true;
                case BOXIDS: boxes(w, r.u8(), false); return true;
                case FEATURE_CONFIG: w.u32(features()); return true;
                case MIXER_CONFIG:
                    w.u8(mixerMode());
                    w.u8(params.value(PARAM_PROPS_OUT) > 0.5f ? 1 : 0);
                    return true;
                case PID: pid(w); return true;
                case RC_TUNING: rcTuning(w); return true;
                case PID_ADVANCED: pidAdvanced(w); return true;
                case FILTER_CONFIG: filterConfig(w); return true;
                case ADVANCED_CONFIG: advancedConfig(w); return true;
                case MODE_RANGES: modeRanges(w); return true;
                case MOTOR_CONFIG: motorConfig(w); return true;
                case FAILSAFE_CONFIG: failsafeConfig(w); return true;
                case ARMING_CONFIG:
                    w.u8(static_cast<uint32_t>(params.value(PARAM_AUTO_DISARM_DELAY)));
                    w.u8(0);
                    w.u8(static_cast<uint32_t>(params.value(PARAM_SMALL_ANGLE)));
                    return true;
                case BATTERY_CONFIG: batteryConfig(w); return true;
                case RX_CONFIG: rxConfig(w); return true;
                case RX_MAP: rxMap(w); return true;
                case RXFAIL_CONFIG: rxfailConfig(w); return true;

                case SET_ARMING_DISABLED:
                    armingDisabled = r.u8() != 0;
                    return true;
                case SET_RAW_RC: return true;   // приёмника по MSP нет — как у BF без RX_MSP
                case REBOOT:
                    rebootMode = r.remaining() ? r.u8() : 0;
                    if (rebootMode > 1 || armed) return false;
                    w.u8(rebootMode);
                    rebootPending = true;
                    return true;
                default:
                    break;
            }
            // дальше — записи настроек: только без ARM и при закрытом CLI
            if (armed || cli.active()) return false;
            switch (cmd)
            {
                case SET_NAME:
                {
                    char name[17] = {};
                    for (size_t i = 0; i < 16 && r.remaining(); ++i) name[i] = static_cast<char>(r.u8());
                    params.setText(PARAM_CRAFT_NAME, 0, name);
                    break;
                }
                case SET_PID: setPid(r); break;
                case SET_RC_TUNING:
                    if (r.remaining() < 10) return false;
                    setRcTuning(r);
                    break;
                case SET_PID_ADVANCED: setPidAdvanced(r); break;
                case SET_FILTER_CONFIG: setFilterConfig(r); break;
                case SET_ADVANCED_CONFIG: break;   // протокол и делители — сборкой
                case SET_MODE_RANGE:
                    if (!setModeRange(r)) return false;
                    break;
                case SET_MOTOR_CONFIG:
                    r.u16();
                    r.u16();
                    r.u16();
                    if (r.remaining() >= 2)
                    {
                        set(PARAM_MOTOR_POLES, r.u8());
                        set(PARAM_DSHOT_BIDIR, r.u8() ? 1 : 0);
                    }
                    break;
                case SET_FAILSAFE_CONFIG: setFailsafeConfig(r); break;
                case SET_ARMING_CONFIG:
                    set(PARAM_AUTO_DISARM_DELAY, r.u8());
                    r.u8();
                    if (r.remaining()) set(PARAM_SMALL_ANGLE, r.u8());
                    break;
                case SET_BATTERY_CONFIG: setBatteryConfig(r); break;
                case SET_RX_CONFIG: setRxConfig(r); break;
                case SET_RX_MAP:
                    if (!setRxMap(r)) return false;
                    break;
                case SET_RXFAIL_CONFIG:
                {
                    const uint8_t ch = r.u8(), mode = r.u8();
                    const uint16_t us = r.u16();
                    char error[64];
                    const char letter = mode == 0 ? 'a' : mode == 1 ? 'h' : 's';
                    if (!params.rc().setRxfail(ch, letter, us, error, sizeof(error))) return false;
                    break;
                }
                case SET_FEATURE_CONFIG: r.u32(); return true;   // features у нас нет — ничего не меняется
                case SELECT_SETTING:
                {
                    const uint8_t v = r.u8();
                    const bool rate = (v & 0x80) != 0;
                    const uint8_t n = static_cast<uint8_t>(v & 0x7F);
                    const bool ok = rate ? params.selectRateProfile(n < CopterParams::PROFILES ? n : 0)
                                         : params.selectPidProfile(n < CopterParams::PROFILES ? n : 0);
                    if (!ok) return false;
                    ctl.applyProfile(params.tune());
                    break;
                }
                case EEPROM_WRITE:
                    cli.saveAndApply();
                    unsaved = false;
                    return true;
                default:
                    return false;   // команды не знаем — «!», как у BF
            }
            unsaved = true;
            return true;
        }

        // ---- ответы ----

        void boardInfo(msp::Writer& w)
        {
            w.text("OPLN");                 // boardIdentifier, 4 буквы
            w.u16(0);                       // hardware revision
            w.u8(board.osd ? 2 : 0);        // FC с MAX7456 — 2
            w.u8(1u | 8u);                  // VCP, загрузчик DFU
            w.lenText(board.target);
            w.lenText(board.boardName);
            w.lenText("OPNP");              // manufacturer
            for (int i = 0; i < 32; ++i) w.u8(0);   // подписи нет
            w.u8(board.mcuType);
            w.u8(2);                        // configurationState: CONFIGURED
            w.u16(static_cast<uint32_t>(ctl.getTune().loopHz));
            uint32_t problems = 0;
            if (ctl.getStatus().armBlockers & ARM_BLOCK_NO_ACC_CAL) problems |= 1u;
            w.u32(problems);
            w.u8(0);
            w.u8(0);
        }

        void buildInfo(msp::Writer& w)
        {
            char date[12] = "Jan  1 2026", time[9] = "00:00:00", rev[8] = "0000000";
#if defined(__DATE__) && defined(__TIME__)
            snprintf(date, sizeof(date), "%.11s", __DATE__);
            snprintf(time, sizeof(time), "%.8s", __TIME__);
#endif
            snprintf(rev, sizeof(rev), "%-7.7s", board.commit);
            w.data(date, 11);
            w.data(time, 8);
            w.data(rev, 7);
        }

        // Режимы на тумблерах: порядок и номера — как в нашей таблице (BOXIDS),
        // флаг i — i-й режим из неё включён.
        uint32_t modeFlags() const
        {
            uint8_t n = 0;
            const BoxInfo* boxesTable = boxTable(n);
            uint32_t flags = 0;
            for (uint8_t i = 0; i < n && i < 32; ++i)
            {
                bool on;
                if (boxesTable[i].id == BOX_ARM) on = ctl.isArmed();
                else if (boxesTable[i].id == BOX_FAILSAFE) on = ctl.getStatus().state == CopterState::FAILSAFE;
                else on = rx && rx->isActive(boxesTable[i].id);
                if (on) flags |= 1u << i;
            }
            return flags;
        }

        void status(msp::Writer& w, bool ex)
        {
            const CopterTune& t = ctl.getTune();
            w.u16(static_cast<uint32_t>(lroundf(1.0e6f / (t.loopHz > 0 ? t.loopHz : 1000.0f))));
            w.u16(0);
            w.u16(board.sensors);
            w.u32(modeFlags());
            w.u8(params.pidProfile());
            w.u16(loadPercent > 100 ? 100 : loadPercent);
            if (ex)
            {
                w.u8(CopterParams::PROFILES);
                w.u8(params.rateProfile());
            }
            else
            {
                w.u16(0);
            }
            w.u8(0);                         // флагов режимов сверх 32 нет
            w.u8(ARM_BLOCK_BF_COUNT);
            w.u32(ctl.getStatus().armBlockers & ARM_BLOCK_BF_MASK);
            w.u8((ctl.getStatus().armBlockers & ARM_BLOCK_REBOOT) ? 1 : 0);
            w.u16(0);                        // температура процессора
        }

        void rawImu(msp::Writer& w)
        {
            for (int a = 0; a < 3; ++a) w.u16(static_cast<uint16_t>(static_cast<int16_t>(lroundf(imu[3 + a] * 2048.0f))));
            for (int a = 0; a < 3; ++a) w.u16(static_cast<uint16_t>(static_cast<int16_t>(lroundf(imu[a]))));
            for (int a = 0; a < 3; ++a) w.u16(0);
        }

        void attitude(msp::Writer& w)
        {
            const CopterStatus& s = ctl.getStatus();
            w.u16(static_cast<uint16_t>(static_cast<int16_t>(lroundf(s.roll * 10.0f))));
            w.u16(static_cast<uint16_t>(static_cast<int16_t>(lroundf(s.pitch * 10.0f))));
            float yaw = fmodf(s.yaw, 360.0f);
            if (yaw < 0) yaw += 360.0f;
            w.u16(static_cast<uint16_t>(lroundf(yaw)));
        }

        void analog(msp::Writer& w)
        {
            const float v = battery ? battery->volts() : 0.0f;
            w.u8(static_cast<uint32_t>(fminf(255.0f, lroundf(v * 10.0f))));
            w.u16(battery ? static_cast<uint32_t>(fminf(65535.0f, battery->mahDrawn())) : 0);
            w.u16(linkQuality >= 0 ? static_cast<uint32_t>(linkQuality * 1023 / 100) : 0);
            w.u16(battery ? static_cast<uint16_t>(static_cast<int16_t>(lroundf(battery->amps() * 100.0f))) : 0);
            w.u16(static_cast<uint32_t>(lroundf(v * 100.0f)));
        }

        void batteryState(msp::Writer& w)
        {
            const float v = battery ? battery->volts() : 0.0f;
            w.u8(battery ? battery->cells() : 0);
            w.u16(battery ? static_cast<uint32_t>(battery->config().capacityMah) : 0);
            w.u8(static_cast<uint32_t>(fminf(255.0f, lroundf(v * 10.0f))));
            w.u16(battery ? static_cast<uint32_t>(fminf(65535.0f, battery->mahDrawn())) : 0);
            w.u16(battery ? static_cast<uint32_t>(lroundf(battery->amps() * 100.0f)) : 0);
            // BF: OK 0, WARNING 1, CRITICAL 2, NOT PRESENT 3
            uint8_t st = 3;
            if (battery)
            {
                switch (battery->state())
                {
                    case BatteryState::OK: st = 0; break;
                    case BatteryState::WARNING: st = 1; break;
                    case BatteryState::CRITICAL: st = 2; break;
                    default: st = 3; break;
                }
            }
            w.u8(st);
            w.u16(static_cast<uint32_t>(lroundf(v * 100.0f)));
        }

        // Каналы в порядке функций BF (крен, тангаж, рысканье, газ, AUX…) — после map.
        void rc(msp::Writer& w)
        {
            const uint8_t n = board.rcChannels;
            for (uint8_t f = 0; f < n; ++f)
            {
                uint8_t ch = f;
                if (rx) ch = f < RcConfig::STICKS ? rx->config().channelFor(f) : rx->config().auxChannel(f - RcConfig::STICKS);
                w.u16(raw && ch < rawCount ? raw[ch] : 0);
            }
        }

        void motor(msp::Writer& w)
        {
            for (uint8_t i = 0; i < 8; ++i)
            {
                if (i >= ctl.getTune().frame.count) w.u16(0);
                else w.u16(static_cast<uint32_t>(lroundf(1000.0f + (outputs.spinning ? outputs.motor[i] : 0.0f) * 1000.0f)));
            }
        }

        void boxes(msp::Writer& w, uint8_t page, bool names)
        {
            uint8_t n = 0;
            const BoxInfo* t = boxTable(n);
            for (uint8_t i = static_cast<uint8_t>(page * 32); i < n && i < (page + 1) * 32; ++i)
            {
                if (names)
                {
                    w.text(t[i].name);
                    w.u8(';');
                }
                else
                {
                    w.u8(t[i].id);
                }
            }
        }

        // Биты features BF: 3 RX_SERIAL, 7 GPS, 10 TELEMETRY, 18 OSD, 22 AIRMODE.
        uint32_t features() const
        {
            uint32_t f = 1u << 3;
            if (board.gps) f |= 1u << 7;
            if (params.value(PARAM_CRSF_TELEMETRY) > 0.5f) f |= 1u << 10;
            if (board.osd) f |= 1u << 18;
            f |= 1u << 22;   // airmode у нас всегда (и с тумблера)
            return f;
        }

        // mixerMode BF: QUADX 3, QUADP 2, HEX6 7, HEX6X 10, Y6 6, OCTOX8 11, OCTOFLATP 12, OCTOFLATX 13.
        uint8_t mixerMode() const
        {
            switch (ctl.getTune().frame.type)
            {
                case FrameType::QUAD_PLUS: return 2;
                case FrameType::HEX_PLUS: return 7;
                case FrameType::HEX_X: return 10;
                case FrameType::Y6: return 6;
                case FrameType::X8: return 11;
                case FrameType::OCTO_PLUS: return 12;
                case FrameType::OCTO_X: return 13;
                default: return 3;
            }
        }

        static uint32_t clampU8(float v) { return static_cast<uint32_t>(v < 0 ? 0 : v > 255 ? 255 : lroundf(v)); }

        void pid(msp::Writer& w)
        {
            static const ParamId P[3] = { PARAM_P_ROLL, PARAM_P_PITCH, PARAM_P_YAW };
            for (uint8_t a = 0; a < 3; ++a)
            {
                for (uint8_t k = 0; k < 3; ++k) w.u8(clampU8(params.value(static_cast<ParamId>(P[a] + k))));
            }
            // LEVEL и MAG — у нас нет: дефолты BF
            w.u8(50);
            w.u8(50);
            w.u8(75);
            w.u8(40);
            w.u8(0);
            w.u8(0);
        }

        void setPid(msp::Reader& r)
        {
            static const ParamId P[3] = { PARAM_P_ROLL, PARAM_P_PITCH, PARAM_P_YAW };
            for (uint8_t a = 0; a < 3; ++a)
            {
                for (uint8_t k = 0; k < 3; ++k) set(static_cast<ParamId>(P[a] + k), r.u8());
            }
        }

        void rcTuning(msp::Writer& w)
        {
            w.u8(clampU8(params.value(PARAM_ROLL_RC_RATE)));
            w.u8(clampU8(params.value(PARAM_ROLL_EXPO)));
            w.u8(clampU8(params.value(PARAM_ROLL_SRATE)));
            w.u8(clampU8(params.value(PARAM_PITCH_SRATE)));
            w.u8(clampU8(params.value(PARAM_YAW_SRATE)));
            w.u8(0);
            w.u8(50);   // thr_mid
            w.u8(0);    // thr_expo
            w.u16(0);
            w.u8(clampU8(params.value(PARAM_YAW_EXPO)));
            w.u8(clampU8(params.value(PARAM_YAW_RC_RATE)));
            w.u8(clampU8(params.value(PARAM_PITCH_RC_RATE)));
            w.u8(clampU8(params.value(PARAM_PITCH_EXPO)));
            w.u8(0);    // throttle_limit_type OFF
            w.u8(100);
            for (int a = 0; a < 3; ++a) w.u16(1998);   // rate_limit
            w.u8(3);    // rates_type ACTUAL
        }

        void setRcTuning(msp::Reader& r)
        {
            set(PARAM_ROLL_RC_RATE, r.u8());
            set(PARAM_ROLL_EXPO, r.u8());
            set(PARAM_ROLL_SRATE, r.u8());
            set(PARAM_PITCH_SRATE, r.u8());
            set(PARAM_YAW_SRATE, r.u8());
            r.u8();
            r.u8();
            r.u8();
            r.u16();
            if (r.remaining() >= 4)
            {
                set(PARAM_YAW_EXPO, r.u8());
                set(PARAM_YAW_RC_RATE, r.u8());
                set(PARAM_PITCH_RC_RATE, r.u8());
                set(PARAM_PITCH_EXPO, r.u8());
            }
            // throttle_limit, rate_limit, rates_type — у нас нет
        }

        void pidAdvanced(msp::Writer& w)
        {
            w.u16(0);
            w.u16(0);
            w.u16(0);
            w.u8(0);
            w.u8(0);
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_TRANSITION)));
            w.u8(0);
            w.u8(0);
            w.u8(0);
            w.u8(0);
            w.u16(0);   // rateAccelLimit
            w.u16(0);
            w.u8(60);   // angle_limit (у нас — из стиля полёта)
            w.u8(0);
            w.u16(0);
            w.u16(static_cast<uint32_t>(params.value(PARAM_ANTI_GRAVITY_GAIN)));
            w.u16(0);
            w.u8(clampU8(params.value(PARAM_ITERM_ROTATION)));
            w.u8(0);
            w.u8(clampU8(params.value(PARAM_ITERM_RELAX)));
            w.u8(clampU8(params.value(PARAM_ITERM_RELAX_TYPE)));
            w.u8(clampU8(params.value(PARAM_ABS_CONTROL_GAIN)));
            w.u8(clampU8(params.value(PARAM_THROTTLE_BOOST)));
            w.u8(0);    // acro trainer
            w.u16(static_cast<uint32_t>(params.value(PARAM_F_ROLL)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_F_PITCH)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_F_YAW)));
            w.u8(0);
            w.u8(clampU8(params.value(PARAM_D_MIN_ROLL)));
            w.u8(clampU8(params.value(PARAM_D_MIN_PITCH)));
            w.u8(clampU8(params.value(PARAM_D_MIN_YAW)));
            w.u8(clampU8(params.value(PARAM_D_MAX_GAIN)));
            w.u8(clampU8(params.value(PARAM_D_MAX_ADVANCE)));
            w.u8(clampU8(params.value(PARAM_USE_INTEGRATED_YAW)));
            w.u8(clampU8(params.value(PARAM_INTEGRATED_YAW_RELAX)));
            w.u8(clampU8(params.value(PARAM_ITERM_RELAX_CUTOFF)));
            w.u8(clampU8(params.value(PARAM_MOTOR_OUTPUT_LIMIT)));
            w.u8(0);    // auto_profile_cell_count
            w.u8(0);    // dyn_idle_min_rpm
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_AVERAGING)));
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_SMOOTH_FACTOR)));
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_BOOST)));
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_MAX_RATE_LIMIT)));
            w.u8(clampU8(params.value(PARAM_FEEDFORWARD_JITTER_FACTOR)));
            w.u8(clampU8(params.value(PARAM_VBAT_SAG_COMPENSATION)));
            w.u8(clampU8(params.value(PARAM_THRUST_LINEAR)));
            // tpa_mode: PD 0, D 1 (наш MODEL — как PD)
            const uint8_t tpaMode = static_cast<uint8_t>(params.value(PARAM_TPA_MODE));
            w.u8(tpaMode > 1 ? 0 : tpaMode);
            w.u8(clampU8(params.value(PARAM_TPA_RATE)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_TPA_BREAKPOINT)));
        }

        void setPidAdvanced(msp::Reader& r)
        {
            r.u16();
            r.u16();
            r.u16();
            r.u8();
            r.u8();
            set(PARAM_FEEDFORWARD_TRANSITION, r.u8());
            r.u8();
            r.u8();
            r.u8();
            r.u8();
            r.u16();
            r.u16();
            if (r.remaining() >= 2)
            {
                r.u8();   // angle_limit
                r.u8();
            }
            if (r.remaining() >= 4)
            {
                r.u16();
                set(PARAM_ANTI_GRAVITY_GAIN, r.u16());
            }
            if (r.remaining() >= 2) r.u16();
            if (r.remaining() >= 14)
            {
                set(PARAM_ITERM_ROTATION, r.u8());
                r.u8();
                set(PARAM_ITERM_RELAX, r.u8());
                set(PARAM_ITERM_RELAX_TYPE, r.u8());
                set(PARAM_ABS_CONTROL_GAIN, r.u8());
                set(PARAM_THROTTLE_BOOST, r.u8());
                r.u8();
                set(PARAM_F_ROLL, r.u16());
                set(PARAM_F_PITCH, r.u16());
                set(PARAM_F_YAW, r.u16());
                r.u8();
            }
            if (r.remaining() >= 7)
            {
                set(PARAM_D_MIN_ROLL, r.u8());
                set(PARAM_D_MIN_PITCH, r.u8());
                set(PARAM_D_MIN_YAW, r.u8());
                set(PARAM_D_MAX_GAIN, r.u8());
                set(PARAM_D_MAX_ADVANCE, r.u8());
                set(PARAM_USE_INTEGRATED_YAW, r.u8());
                set(PARAM_INTEGRATED_YAW_RELAX, r.u8());
            }
            if (r.remaining() >= 1) set(PARAM_ITERM_RELAX_CUTOFF, r.u8());
            if (r.remaining() >= 3)
            {
                set(PARAM_MOTOR_OUTPUT_LIMIT, r.u8());
                r.u8();
                r.u8();
            }
            if (r.remaining() >= 7)
            {
                set(PARAM_FEEDFORWARD_AVERAGING, r.u8());
                set(PARAM_FEEDFORWARD_SMOOTH_FACTOR, r.u8());
                set(PARAM_FEEDFORWARD_BOOST, r.u8());
                set(PARAM_FEEDFORWARD_MAX_RATE_LIMIT, r.u8());
                set(PARAM_FEEDFORWARD_JITTER_FACTOR, r.u8());
                set(PARAM_VBAT_SAG_COMPENSATION, r.u8());
                set(PARAM_THRUST_LINEAR, r.u8());
            }
            if (r.remaining() >= 4)
            {
                const uint8_t mode = r.u8();
                // наш MODEL конфигуратор не покажет: PD от него — не трогать
                if (static_cast<uint8_t>(params.value(PARAM_TPA_MODE)) <= 1 || mode != 0) set(PARAM_TPA_MODE, mode);
                set(PARAM_TPA_RATE, r.u8());
                set(PARAM_TPA_BREAKPOINT, r.u16());
            }
        }

        void filterConfig(msp::Writer& w)
        {
            const CopterTune& t = ctl.getTune();
            const uint32_t gyro = static_cast<uint32_t>(lroundf(t.gyroLpfHz));
            const uint32_t dterm = static_cast<uint32_t>(lroundf(t.dtermLpfHz));
            w.u8(gyro > 255 ? 255 : gyro);
            w.u16(dterm);
            w.u16(static_cast<uint32_t>(params.value(PARAM_YAW_LOWPASS_HZ)));
            for (int i = 0; i < 4; ++i) w.u16(0);   // notch 1, d-term notch
            w.u16(0);
            w.u16(0);
            w.u8(0);    // dterm_lpf1_type PT1
            w.u8(0);    // gyro_hardware_lpf
            w.u8(0);
            w.u16(gyro);
            w.u16(0);   // gyro_lpf2
            w.u8(2);    // gyro_lpf1_type PT2
            w.u8(0);
            w.u16(0);   // dterm_lpf2
            w.u8(0);
            for (int i = 0; i < 4; ++i) w.u16(0);   // dyn lpf
            w.u8(0);
            w.u8(0);
            w.u16(0);   // dyn notch q
            w.u16(0);
            w.u8(0);    // rpm_filter_harmonics (наши режекторы по оборотам — из модели)
            w.u8(0);
            w.u16(0);
            w.u8(0);
            w.u8(0);    // dyn_notch_count
        }

        void setFilterConfig(msp::Reader& r)
        {
            r.u8();
            r.u16();
            set(PARAM_YAW_LOWPASS_HZ, r.u16());   // остальное у нас — из модели
        }

        // motorPwmProtocol BF: PWM 0, ONESHOT125 1, ONESHOT42 2, MULTISHOT 3, DSHOT150 5, 300 6, 600 7.
        void advancedConfig(msp::Writer& w)
        {
            uint8_t proto = 0;
            switch (params.escProtocol())
            {
                case EscProtocol::ONESHOT125: proto = 1; break;
                case EscProtocol::ONESHOT42: proto = 2; break;
                case EscProtocol::MULTISHOT: proto = 3; break;
                case EscProtocol::DSHOT150: proto = 5; break;
                case EscProtocol::DSHOT300: proto = 6; break;
                case EscProtocol::DSHOT600: proto = 7; break;
                default: proto = 0; break;
            }
            w.u8(1);
            w.u8(1);       // pid_process_denom
            w.u8(0);
            w.u8(proto);
            w.u16(480);
            w.u16(static_cast<uint32_t>(lroundf(ctl.getTune().idle * 10000.0f)));   // dshot_idle_value
            w.u8(0);
            w.u8(0);
            w.u8(0);
            w.u8(0);
            w.u8(48);
            w.u16(125);
            w.u16(0);
            w.u8(0);
            w.u8(0);       // debug_mode NONE
            w.u8(1);       // DEBUG_COUNT
        }

        void modeRanges(msp::Writer& w)
        {
            const AuxRange* t = params.rc().effectiveAux();
            for (uint8_t i = 0; i < RcConfig::AUX_SLOTS; ++i)
            {
                const AuxRange& a = t[i];
                w.u8(a.used() ? a.mode : 0);
                w.u8(a.used() ? a.channel : 0);
                w.u8(a.used() ? (a.startUs - RcConfig::AUX_MIN_US) / RcConfig::AUX_STEP_US : 0);
                w.u8(a.used() ? (a.endUs - RcConfig::AUX_MIN_US) / RcConfig::AUX_STEP_US : 0);
            }
        }

        bool setModeRange(msp::Reader& r)
        {
            const uint8_t slot = r.u8(), mode = r.u8(), ch = r.u8(), start = r.u8(), end = r.u8();
            uint8_t logic = 0, linked = 0;
            if (r.remaining() >= 2)
            {
                logic = r.u8();
                linked = r.u8();
            }
            char line[64], error[96];
            snprintf(line, sizeof(line), "%u %u %u %u %u %u %u", slot, mode, ch, RcConfig::AUX_MIN_US + start * RcConfig::AUX_STEP_US,
                     RcConfig::AUX_MIN_US + end * RcConfig::AUX_STEP_US, logic, linked);
            return params.rc().parseAux(line, error, sizeof(error));
        }

        void motorConfig(msp::Writer& w)
        {
            w.u16(1070);
            w.u16(2000);
            w.u16(1000);
            w.u8(ctl.getTune().frame.count);
            w.u8(clampU8(params.value(PARAM_MOTOR_POLES)));
            w.u8(params.value(PARAM_DSHOT_BIDIR) > 0.5f ? 1 : 0);
            w.u8(0);
        }

        void failsafeConfig(msp::Writer& w)
        {
            w.u8(clampU8(params.value(PARAM_FAILSAFE_DELAY)));
            w.u8(clampU8(params.value(PARAM_FAILSAFE_LANDING_TIME)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_FAILSAFE_THROTTLE)));
            w.u8(clampU8(params.value(PARAM_FAILSAFE_SWITCH_MODE)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_FAILSAFE_THROTTLE_LOW_DELAY)));
            w.u8(clampU8(params.value(PARAM_FAILSAFE_PROCEDURE)));
        }

        void setFailsafeConfig(msp::Reader& r)
        {
            set(PARAM_FAILSAFE_DELAY, r.u8());
            set(PARAM_FAILSAFE_LANDING_TIME, r.u8());
            set(PARAM_FAILSAFE_THROTTLE, r.u16());
            set(PARAM_FAILSAFE_SWITCH_MODE, r.u8());
            set(PARAM_FAILSAFE_THROTTLE_LOW_DELAY, r.u16());
            set(PARAM_FAILSAFE_PROCEDURE, r.u8());
        }

        void batteryConfig(msp::Writer& w)
        {
            const float mn = params.value(PARAM_VBAT_MIN_CELL_VOLTAGE), mx = params.value(PARAM_VBAT_MAX_CELL_VOLTAGE),
                        wa = params.value(PARAM_VBAT_WARNING_CELL_VOLTAGE);
            w.u8(clampU8((mn + 5) / 10));
            w.u8(clampU8((mx + 5) / 10));
            w.u8(clampU8((wa + 5) / 10));
            w.u16(static_cast<uint32_t>(params.value(PARAM_BAT_CAPACITY)));
            w.u8(clampU8(params.value(PARAM_BATTERY_METER)));
            w.u8(clampU8(params.value(PARAM_CURRENT_METER)));
            w.u16(static_cast<uint32_t>(mn));
            w.u16(static_cast<uint32_t>(mx));
            w.u16(static_cast<uint32_t>(wa));
        }

        void setBatteryConfig(msp::Reader& r)
        {
            const uint8_t mn = r.u8(), mx = r.u8(), wa = r.u8();
            set(PARAM_BAT_CAPACITY, r.u16());
            set(PARAM_BATTERY_METER, r.u8());
            set(PARAM_CURRENT_METER, r.u8());
            if (r.remaining() >= 6)
            {
                set(PARAM_VBAT_MIN_CELL_VOLTAGE, r.u16());
                set(PARAM_VBAT_MAX_CELL_VOLTAGE, r.u16());
                set(PARAM_VBAT_WARNING_CELL_VOLTAGE, r.u16());
            }
            else
            {
                set(PARAM_VBAT_MIN_CELL_VOLTAGE, mn * 10.0f);
                set(PARAM_VBAT_MAX_CELL_VOLTAGE, mx * 10.0f);
                set(PARAM_VBAT_WARNING_CELL_VOLTAGE, wa * 10.0f);
            }
        }

        // serialrx_provider BF: SBUS 2, IBUS 7, CRSF 9 (наши 1, 0, 2).
        static uint8_t bfProvider(uint8_t ours) { return ours == 0 ? 7 : ours == 1 ? 2 : 9; }

        void rxConfig(msp::Writer& w)
        {
            w.u8(bfProvider(static_cast<uint8_t>(params.value(PARAM_SERIALRX_PROVIDER))));
            w.u16(1900);
            w.u16(1500);
            w.u16(1050);
            w.u8(0);
            w.u16(static_cast<uint32_t>(params.value(PARAM_RX_MIN_USEC)));
            w.u16(static_cast<uint32_t>(params.value(PARAM_RX_MAX_USEC)));
            w.u8(0);
            w.u8(0);
            w.u16(1250);   // airmode_start_throttle_percent 25
            w.u8(0);
            w.u32(0);
            w.u8(0);
            w.u8(0);       // fpv_mix_degrees
            w.u8(0);
            w.u8(0);
            w.u8(0);       // rc_smoothing_setpoint_cutoff: авто
            w.u8(0);
            w.u8(0);
            w.u8(0);
            w.u8(0);       // usb type
            w.u8(30);      // rc_smoothing_auto_factor
            w.u8(1);       // rc_smoothing ON
            for (int i = 0; i < 6; ++i) w.u8(0);   // UID ELRS
        }

        void setRxConfig(msp::Reader& r)
        {
            const uint8_t bf = r.u8();
            if (bf == 7) set(PARAM_SERIALRX_PROVIDER, 0);
            else if (bf == 2) set(PARAM_SERIALRX_PROVIDER, 1);
            else if (bf == 9) set(PARAM_SERIALRX_PROVIDER, 2);
            r.u16();
            r.u16();
            r.u16();
            r.u8();
            if (r.remaining() >= 4)
            {
                set(PARAM_RX_MIN_USEC, r.u16());
                set(PARAM_RX_MAX_USEC, r.u16());
            }
        }

        void rxMap(msp::Writer& w)
        {
            for (uint8_t f = 0; f < RcConfig::MAPPABLE; ++f) w.u8(params.rc().channelFor(f));
        }

        bool setRxMap(msp::Reader& r)
        {
            char letters[RcConfig::MAPPABLE + 1] = {};
            for (uint8_t f = 0; f < RcConfig::MAPPABLE; ++f)
            {
                const uint8_t ch = r.u8();
                if (ch >= RcConfig::MAPPABLE || letters[ch]) return false;
                letters[ch] = RcConfig::LETTERS[f];
            }
            char error[64];
            return params.rc().setMap(letters, error, sizeof(error));
        }

        void rxfailConfig(msp::Writer& w)
        {
            for (uint8_t i = 0; i < board.rcChannels && i < RcConfig::RXFAIL_CHANNELS; ++i)
            {
                const RxfailEntry& e = params.rc().rxfail(i);
                w.u8(static_cast<uint8_t>(e.mode));
                w.u16(e.us);
            }
        }

        // Число из конфигуратора -> настройка (прижать к её границам).
        void set(ParamId id, float v)
        {
            const ParamInfo& info = paramInfo(id);
            if (v < info.min) v = info.min;
            if (v > info.max) v = info.max;
            params.setValue(id, params.activeSlot(id), v);
        }

        CopterController& ctl;
        CopterParams& params;
        CopterCli& cli;
        MspBoard board;
        const RxInput* rx = nullptr;
        const uint16_t* raw = nullptr;
        uint8_t rawCount = 0;
        const BatteryMonitor* battery = nullptr;
        CopterOutputs outputs;
        float imu[6] = {};
        uint16_t loadPercent = 0;
        int16_t linkQuality = -1;
        bool armingDisabled = false, unsaved = false, seen = false, rebootPending = false;
        uint8_t rebootMode = 0;
        uint32_t lastFrameMs = 0;
    };
}
