#pragma once
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "copter/CopterCli.h"
#include "copter/Battery.h"
#include "copter/CopterController.h"
#include "copter/GpsHome.h"
#include "copter/GyroSelect.h"
#include "copter/Params.h"
#include "copter/RcSetup.h"
#include "copter/Rssi.h"
#include "copter/Vtx.h"
#include "hal/Platform.h"
#include "rc/RcReceiver.h"
#include "telemetry/FirmwareIdentity.h"
#include "copter/DisplayPort.h"
#include "telemetry/JsonWriter.h"

// ============================================================
// Связь коптера с приложением OpenPlane Studio: машинный протокол
// настроек (то, что конфигуратор BF делает MSP-запросами) и живые
// данные. Команды ПК — как у самолёта: STX (0x02), строка, '\n'
// (CopterConsole собирает строку и отдаёт сюда):
//
//   id                — "@id {...}": прошивка, плата, коммит, сборка;
//   params            — "@params {...}" (сколько настроек, профили,
//                       сборка), затем по строке "@param {...}" на
//                       настройку (имя, тип, группа = вкладка, границы,
//                       умолчание и откуда оно, единицы, описание,
//                       значение, у профильных — все 4), "@rc {...}" (map,
//                       rxrange, таблица aux и тумблеры прошивки, строки vtx,
//                       изменённые rxfail [канал, "a|h|s", мкс]) и
//                       "@modes [[номер, "имя", действует], …]" (режимы BF);
//   set имя значение  — "@set {...}": ok или ошибка, новое значение;
//   profile N · rateprofile N — "@profile {...}" (сразу, на ходу);
//   aux … · map … · rxrange … · vtx … · rxfail … — как в CLI; ответ — "@rc {...}";
//   geozone …         — поле и запретные зоны, как в CLI; ответ — "@fence {...}"
//                       на зону (в потоке params — по строке на каждую из 8);
//   calibrate vbat В · calibrate ibat А — по мультиметру: vbat_scale /
//                       ibata_scale, ответ — как на set;
//   save              — во флеш и в работу: "@save {...}" (и "flash":[занято, всего]);
//   revert            — отменить несохранённое: "@revert {...}";
//   live · live 0     — "@live {...}" 20 раз в секунду ещё 3 с / хватит;
//   motor N|all % · motor stop — проверка моторов без пропеллеров (до 30 %,
//                       держится 1 с — приложение повторяет): "@motor {...}";
//   dshot N|all команда… — команды регуляторам DShot (как dshotprog: 1 —
//                       пискнуть, 7/8 — направление, 12 — сохранить): "@dshot {...}";
//   beep УСЛОВИЕ       — «послушать», как пищит условие (имя BF: RX_LOST…), без ARM:
//                       "@beep {...}".
//
// Менять — только без ARM и при закрытом CLI; пока изменения не
// сохранены или не отменены, ARM запрещён (флаг MSP — как при
// подключённом конфигураторе BF). Ошибка — "@error {...}".
// Длинный ответ на params уходит по строке, когда в порту есть место
// под неё целиком: полётный цикл ПК не ждёт.
// ============================================================

class CopterLink
{
public:
#if defined(OP_STM32)
    static constexpr uint32_t FRAME_PERIOD_MS = 50;
#else
    static constexpr uint32_t FRAME_PERIOD_MS = 100;
#endif
    static constexpr uint32_t LIVE_HOLD_MS = 3000;
    // Строка уходит в порт, только если влезает в его очередь целиком (USB CDC у
    // STM32 — CDC_TRANSMIT_QUEUE_BUFFER_PACKET_NUMBER × 64 байта): строки ответа
    // короче килобайта, буфер — с запасом.
    static constexpr size_t LINE_CAPACITY = 1536;
    static constexpr uint8_t LINES_PER_UPDATE = 4;

    CopterLink(Stream& stream, copter::CopterController& controller, copter::CopterParams& paramsRef,
               copter::CopterCli& cliRef, RcReceiver& receiver)
        : io(stream), ctl(controller), params(paramsRef), cli(cliRef), rc(receiver)
    {
    }

    void setRxInput(const copter::RxInput* input) { rxInput = input; }
    void setBattery(const copter::BatteryMonitor* monitor) { battery = monitor; }
    void setGuard(const copter::EscGuard* g) { guard = g; }
    // обороты и EDT регуляторов (dshot_edt) — строка "esc"
    void setMotorTelemetry(const copter::IMotorTelemetry* t) { motorTelemetry = t; }
    void setRssi(const copter::Rssi* r) { rssi = r; }
    void setVtx(const copter::VtxController* controller) { vtx = controller; }
    // Куда рисуется OSD и что на порту очков (copter/DisplayPort.h) — в живые кадры.
    void setOsdOutput(const copter::OsdOutputStatus* status) { osdOut = status; }
    void setGps(const GpsSensor* sensor, const copter::GpsHome* home)
    {
        gps = sensor;
        gpsHome = home;
    }
    void setHeading(float deg) { headingDeg = deg; }
    // Имя модели пульта (MSP OP_TX_MODEL от Lua-скрипта) — "tx" в живом кадре.
    void setTxModel(const char* name) { txModel = name; }
    // Компас: датчик есть, и что из него получается (copter/Compass.h) — мастеру калибровки.
    void setCompass(const copter::Compass* c, bool present)
    {
        magCal = c;
        magPresent = present;
    }
    void setOutputs(const copter::CopterOutputs& out) { outputs = out; }
    // Датчики для вкладки «Датчики»: температура гироскопа, барометр (давление, температура, высота).
    void setSensors(const ImuSensor* imuSensor, const BarometerSensor* baroSensor)
    {
        imu = imuSensor;
        baro = baroSensor;
    }
    // Два гироскопа на плате (copter/GyroSelect.h): маска живых и маска взятых в контур.
    void setGyros(const copter::GyroPick* pick, const uint8_t* aliveMask, const copter::GyroAgreement* agreement)
    {
        gyroPick = pick;
        gyroAlive = aliveMask;
        gyroAgree = agreement;
    }
    void setLoop(uint32_t avgUs, uint32_t hz)
    {
        loopAvgUs = avgUs;
        loopHz = hz;
    }

    bool hasUnsaved() const { return dirty; }
    bool isLive() const { return live; }

    // Сохранено или отменено (и из CLI) — несохранённого больше нет.
    void settingsApplied() { dirty = false; }

    // Строка команды ПК (без STX и '\n'). true — команда наша.
    bool handle(const char* command, uint32_t nowMs)
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "%s", command ? command : "");
        char* args = buf;
        while (*args && *args != ' ') ++args;
        if (*args) *args++ = '\0';
        while (*args == ' ') ++args;
        const char* cmd = buf;

        if (strcmp(cmd, "live") == 0)
        {
            live = strcmp(args, "0") != 0;
            liveUntilMs = nowMs + LIVE_HOLD_MS;
            return true;
        }
        if (strcmp(cmd, "id") == 0)
        {
            sendId();
            return true;
        }
        if (strcmp(cmd, "params") == 0)
        {
            dumpStep = 0;
            return true;
        }
        if (strcmp(cmd, "motor") == 0)
        {
            doMotor(args);
            return true;
        }
        if (strcmp(cmd, "dshot") == 0)
        {
            doDshot(args);
            return true;
        }
        if (strcmp(cmd, "beep") == 0)
        {
            doBeep(args);
            return true;
        }
        const bool changes = strcmp(cmd, "set") == 0 || strcmp(cmd, "profile") == 0 || strcmp(cmd, "rateprofile") == 0 ||
                             strcmp(cmd, "aux") == 0 || strcmp(cmd, "map") == 0 || strcmp(cmd, "rxrange") == 0 ||
                             strcmp(cmd, "vtx") == 0 || strcmp(cmd, "rxfail") == 0 || strcmp(cmd, "calibrate") == 0 ||
                             strcmp(cmd, "save") == 0 || strcmp(cmd, "geozone") == 0 ||
                             strcmp(cmd, "revert") == 0;
        if (!changes) return false;
        if (ctl.isArmed() || cli.active())
        {
            sendError(cmd, ctl.isArmed() ? "только без ARM" : "открыт CLI");
            return true;
        }
        if (strcmp(cmd, "set") == 0) doSet(args);
        else if (strcmp(cmd, "calibrate") == 0) doCalibrate(args);
        else if (strcmp(cmd, "profile") == 0 || strcmp(cmd, "rateprofile") == 0) doProfile(cmd, args);
        else if (strcmp(cmd, "save") == 0) doSave();
        else if (strcmp(cmd, "revert") == 0) doRevert();
        else if (strcmp(cmd, "geozone") == 0) doGeozone(args);
        else doRcTable(cmd, args);
        return true;
    }

    // Каждый цикл: строки ответа на params и живые данные.
    void update(uint32_t nowMs)
    {
        for (uint8_t n = 0; n < LINES_PER_UPDATE && dumpStep >= 0; ++n)
        {
            char line[LINE_CAPACITY];
            const size_t length = formatDumpLine(dumpStep, line, sizeof(line));
            if (length == 0)
            {
                ++dumpStep;   // не влезла — пропустить (не бывает: проверено тестом)
            }
            else
            {
                if (io.availableForWrite() < static_cast<int>(length)) break;   // ждём место
                io.write(reinterpret_cast<const uint8_t*>(line), length);
                ++dumpStep;
            }
            if (dumpStep > DUMP_LAST) dumpStep = -1;
        }

        if (!live) return;
        if (static_cast<int32_t>(nowMs - liveUntilMs) >= 0)
        {
            live = false;
            return;
        }
        if (liveSent && nowMs - lastLiveMs < FRAME_PERIOD_MS) return;
        lastLiveMs = nowMs;
        liveSent = true;
        char line[LINE_CAPACITY];
        const size_t length = formatLive(line, sizeof(line), nowMs);
        if (length == 0 || io.availableForWrite() < static_cast<int>(length)) return;   // кадр пропущен
        io.write(reinterpret_cast<const uint8_t*>(line), length);
    }

    // ---- строки ----

    size_t formatParam(copter::ParamId id, char* buffer, size_t capacity) const
    {
        using namespace copter;
        const ParamInfo& info = paramInfo(id);
        JsonWriter w(buffer, capacity);
        w.add("@param {\"name\":");
        w.str(info.name);
        w.add(",\"type\":\"%s\",\"group\":\"%s\",\"scope\":\"%s\"", typeName(info.type), paramGroupName(info.group),
              scopeName(info.scope));
        if (info.type == ParamType::INT || info.type == ParamType::FLOAT)
        {
            w.add(",\"min\":");
            number(w, info, info.min);
            w.add(",\"max\":");
            number(w, info, info.max);
        }
        if (info.type == ParamType::STRING) w.add(",\"maxlen\":%d", static_cast<int>(info.max));
        if (info.type == ParamType::ENUM)
        {
            w.add(",\"options\":[");
            for (uint8_t k = 0; k < info.optionCount; ++k)
            {
                if (k) w.add(",");
                w.str(info.options[k]);
            }
            w.add("]");
        }
        w.add(",\"default\":");
        if (info.type == ParamType::STRING) w.str(CopterParams::textDefault(id));
        else number(w, info, params.defaultValue(id));
        w.add(",\"from\":\"%s\",\"units\":", fromName(info.from));
        w.str(info.units);
        w.add(",\"text\":");
        w.str(info.text);
        const uint8_t active = params.activeSlot(id);
        w.add(",\"value\":");
        value(w, id, active);
        w.add(",\"set\":%s", params.isSet(id, active) ? "true" : "false");
        if (info.scope != ParamScope::MASTER)
        {
            w.add(",\"profiles\":[");
            for (uint8_t s = 0; s < CopterParams::PROFILES; ++s)
            {
                if (s) w.add(",");
                value(w, id, s);
            }
            w.add("]");
        }
        w.add("}\n");
        return w.overflow ? 0 : w.length;
    }

    size_t formatRc(char* buffer, size_t capacity) const
    {
        using namespace copter;
        const RcConfig& c = params.rc();
        JsonWriter w(buffer, capacity);
        char m[RcConfig::MAPPABLE + 1];
        c.formatMap(m, sizeof(m));
        w.add("@rc {\"map\":\"%s\",\"channels\":%u,\"rxrange\":[", m, c.channelCount());
        for (uint8_t i = 0; i < RcConfig::STICKS; ++i) w.add("%s[%u,%u]", i ? "," : "", c.rangeLow(i), c.rangeHigh(i));
        w.add("],\"aux\":");
        auxList(w, [&c](uint8_t i) -> const AuxRange& { return c.aux(i); });
        w.add(",\"fallback\":");
        auxList(w, [&c](uint8_t i) -> const AuxRange& { return c.fallbackAux()[i]; });
        w.add(",\"usingFallback\":%s,\"vtx\":[", c.anyAux() ? "false" : "true");
        bool first = true;
        for (uint8_t i = 0; i < RcConfig::VTX_SLOTS; ++i)
        {
            const VtxRange& v = c.vtx(i);
            if (!v.used()) continue;
            w.add("%s[%u,%u,%u,%u,%u,%u,%u]", first ? "" : ",", i, v.channel, v.band, v.vtxChannel, v.power, v.startUs,
                  v.endUs);
            first = false;
        }
        w.add("],\"rxfail\":[");
        first = true;
        for (uint8_t i = 0; i < RcConfig::RXFAIL_CHANNELS; ++i)
        {
            if (c.rxfailIsDefault(i)) continue;
            const RxfailEntry& e = c.rxfail(i);
            const char* letter = e.mode == RxfailMode::AUTO ? "a" : e.mode == RxfailMode::HOLD ? "h" : "s";
            w.add("%s[%u,\"%s\",%u]", first ? "" : ",", i, letter, e.us);
            first = false;
        }
        w.add("]}\n");
        return w.overflow ? 0 : w.length;
    }

    // "@fence {"zone":0,"type":"FIELD","pts":[[563268710,440059870],…],"problem":null}" — точки в 1e-7 градуса.
    size_t formatFence(uint8_t i, char* buffer, size_t capacity) const
    {
        using namespace copter;
        const GeoZoneTable& t = params.zones();
        const GeoZoneTable::Zone& z = t.zone(i);
        JsonWriter w(buffer, capacity);
        w.add("@fence {\"zone\":%u,\"type\":\"%s\",\"pts\":[", i, geoZoneTypeName(z.type));
        for (uint8_t k = 0; k < z.count; ++k)
            w.add("%s[%ld,%ld]", k ? "," : "", static_cast<long>(z.pts[k].latE7), static_cast<long>(z.pts[k].lonE7));
        w.add("],\"problem\":");
        const char* p = t.used(i) ? t.problem(i) : nullptr;
        if (p) w.str(p);
        else w.add("null");
        w.add("}\n");
        return w.overflow ? 0 : w.length;
    }

    void doGeozone(char* args)
    {
        using namespace copter;
        char error[96] = {};
        if (!params.zones().parse(args, error, sizeof(error)))
        {
            sendError("geozone", error);
            return;
        }
        dirty = true;
        // ответ — изменённая зона; «geozone clear» — все
        char* end = nullptr;
        const long zi = strtol(args, &end, 10);
        char line[LINE_CAPACITY];
        for (uint8_t i = 0; i < GeoZoneTable::MAX_ZONES; ++i)
        {
            if (end != args && i != zi) continue;
            const size_t length = formatFence(i, line, sizeof(line));
            if (length) io.write(reinterpret_cast<const uint8_t*>(line), length);
        }
    }

    size_t formatModes(char* buffer, size_t capacity) const
    {
        using namespace copter;
        JsonWriter w(buffer, capacity);
        uint8_t n = 0;
        const BoxInfo* boxes = boxTable(n);
        w.add("@modes [");
        for (uint8_t i = 0; i < n; ++i)
        {
            w.add("%s[%u,", i ? "," : "", boxes[i].id);
            w.str(boxes[i].name);
            w.add(",%s]", boxes[i].works ? "true" : "false");
        }
        w.add("]\n");
        return w.overflow ? 0 : w.length;
    }

    size_t formatLive(char* buffer, size_t capacity, uint32_t nowMs) const
    {
        using namespace copter;
        const CopterStatus& s = ctl.getStatus();
        JsonWriter w(buffer, capacity);
        w.add("@live {\"t\":%lu,\"arm\":%s,\"state\":\"%s\",\"mode\":\"%s\"", static_cast<unsigned long>(nowMs),
              ctl.isArmed() ? "true" : "false", stateName(s.state), copterModeName(s.mode));
        if (s.turtle) w.add(",\"turtle\":true");   // моторы наоборот (FLIP OVER AFTER CRASH)
        // защиты, пока работают: crash recovery, yaw spin recovery, гироскоп в упоре
        if (s.crashRecovery || s.yawSpin || s.gyroSaturated)
        {
            const char* sep = "";
            w.add(",\"prot\":[");
            if (s.crashRecovery) { w.add("\"crash\""); sep = ","; }
            if (s.yawSpin) { w.add("%s\"yawspin\"", sep); sep = ","; }
            if (s.gyroSaturated) w.add("%s\"gyrosat\"", sep);
            w.add("]");
        }
        w.add(",\"rx\":\"%s\"", rc.isFrameTimeout() ? "lost" : rc.isFailsafeReported() ? "fs" : "ok");
        if (rc.getLinkQuality() >= 0) w.add(",\"lq\":%d", static_cast<int>(rc.getLinkQuality()));
        if (rssi && rssi->percent() >= 0)
        {
            w.add(",\"rssi\":%d,\"rssisrc\":\"%s\"", static_cast<int>(rssi->percent()), copter::rssiSourceName(rssi->source()));
        }
        if (rc.getRssiDbm() < 0) w.add(",\"dbm\":%d", static_cast<int>(rc.getRssiDbm()));
        if (txModel && *txModel) w.add(",\"tx\":\"%s\"", txModel);   // без кавычек и \\ — CopterMsp
        w.add(",\"rxhz\":%.0f,\"rcsp\":%.0f,\"rcth\":%.0f", s.rcFrameHz, s.rcSetpointCutoffHz, s.rcThrottleCutoffHz);
        w.add(",\"ch\":[");
        for (uint8_t i = 0; i < Config::RC_CHANNELS; ++i) w.add(i ? ",%u" : "%u", static_cast<unsigned>(rc.getState().get(i)));
        w.add("],\"att\":[%.1f,%.1f,%.1f],\"alt\":%.2f,\"vz\":%.2f,\"thr\":%.3f,\"v\":%.2f", s.roll, s.pitch, s.yaw,
              s.altitude, s.climbRate, s.throttle, s.volts);
        // вкладка «Датчики»: гироскоп (°/с, до фильтров) и акселерометр (g) в осях рамы, температура
        // гироскопа; барометр — давление (Па), температура, высота над морем (м)
        w.add(",\"sens\":{\"ok\":%s,\"g\":[%.1f,%.1f,%.1f],\"a\":[%.3f,%.3f,%.3f]", s.imuOk ? "true" : "false",
              s.gyroIn[0], s.gyroIn[1], s.gyroIn[2], s.accIn[0], s.accIn[1], s.accIn[2]);
        if (imu && imu->isAvailable()) w.add(",\"it\":%.1f", imu->getImuData().temperature);
        if (baro && baro->isAvailable())
        {
            const BarometerData& b = baro->getBarometerData();
            w.add(",\"baro\":[%.0f,%.1f,%.2f]", b.pressure, b.temperature, b.altitude);
        }
        w.add("}");
        // chirp: идёт ли качание, по какой оси (или какая следующая), частота, чем кончилось прошлое
        if (s.chirpActive || s.chirpRun > 0)
        {
            w.add(",\"chirp\":{\"on\":%s,\"axis\":%u,\"hz\":%.1f,\"run\":%u,\"end\":\"%s\"}", s.chirpActive ? "true" : "false",
                  static_cast<unsigned>(s.chirpAxis), s.chirpHz, static_cast<unsigned>(s.chirpRun), copter::chirpEndName(s.chirpEnd));
        }
        // фильтр ориентации MEKF: найденное смещение гироскопа (°/с) и вера акселерометру (%)
        if (ctl.getAttitude().ekfEnabled())
        {
            const copter::AttitudeQuat& at = ctl.getAttitude();
            w.add(",\"gbias\":[%.2f,%.2f,%.2f],\"atrust\":%d", at.gyroBiasDps(0), at.gyroBiasDps(1), at.gyroBiasDps(2),
                  static_cast<int>(lroundf(at.accelTrust() * 100.0f)));
            // сопротивление винтов по оценке фильтра (1/с) и её неуверенность (%); компас в деле
            w.add(",\"kdrag\":[%.3f,%d],\"amag\":%d", at.dragCoef(), static_cast<int>(lroundf(at.dragSigma() * 100.0f)),
                  at.compassUsed() ? 1 : 0);
        }
        if (gyroPick && gyroAlive)
        {
            w.add(",\"gyros\":[%u,%u,%u]", static_cast<unsigned>(*gyroAlive), static_cast<unsigned>(gyroPick->used),
                  static_cast<unsigned>(gyroAgree ? gyroAgree->state() : copter::GyroAgree::UNKNOWN));
        }
        if (battery)
        {
            w.add(",\"bat\":\"%s\",\"cell\":%.2f,\"full\":%s", copter::batteryStateName(battery->state()), battery->cellVolts(),
                  battery->wasFullWhenPluggedIn() ? "true" : "false");
            if (battery->hasCurrent()) w.add(",\"amps\":%.1f,\"mah\":%.0f", battery->amps(), battery->mahDrawn());
        }
        if (guard && battery && battery->hasCurrent())
        {
            // перегрузка сейчас (ESC, BATT, LIMIT — current_limit_a, или пусто) и наибольший ток на мотор за полёт
            const copter::Overload o = guard->state();
            w.add(",\"ovl\":\"%s\",\"mma\":%.1f",
                  o == copter::Overload::ESC ? "ESC" : o == copter::Overload::BATTERY ? "BATT" : guard->limiting() ? "LIMIT" : "",
                  guard->flightStats().maxMotorA);
        }
        escTelemetry(w);
        if (vtx)
        {
            const copter::VtxState& v = vtx->state();
            w.add(",\"vtx\":{\"on\":%s,\"mhz\":%u,\"mw\":%u,\"pit\":%s,\"cap\":%u}", v.online ? "true" : "false",
                  v.frequencyMhz, v.powerMw, v.pit ? "true" : "false", vtx->levelMw(vtx->topAllowedLevel()));
        }
        if (osdOut && (osdOut->port || osdOut->chip))
        {
            // OSD: чип и/или очки по MSP, экран (столбцы × строки, HD), порт очков:
            // есть ли он, спрашивают ли по нему, кто сказал размер, VTX на MSP
            const copter::OsdOutputStatus& o = *osdOut;
            w.add(",\"osd\":{\"chip\":%s,\"msp\":%s,\"c\":%u,\"r\":%u,\"hd\":%s,\"port\":%s,\"dev\":%s,\"devc\":%s,\"vtx\":%s,\"req\":%lu}",
                  o.chip ? "true" : "false", o.msp ? "true" : "false", o.cols, o.rows, o.hd ? "true" : "false",
                  o.port ? "true" : "false", o.deviceAlive ? "true" : "false", o.deviceCanvas ? "true" : "false",
                  o.mspVtx ? "true" : "false", static_cast<unsigned long>(o.requests));
        }
        w.add(",\"hdg\":%.0f,\"hsrc\":\"%s\"", headingDeg, copter::headingSourceName(s.headingSource));
        if (magPresent && magCal && magCal->hasSample())
        {
            // компас: сырое поле (мкТл, оси датчика), модуль после калибровки, откалиброван, верим ли,
            // расходится ли с курсом по полёту; склонение (°) и взято ли оно по GPS
            const float* m = magCal->rawUT();
            w.add(",\"mag\":{\"raw\":[%.1f,%.1f,%.1f],\"f\":%.1f,\"cal\":%s,\"ok\":%s,\"bad\":%s,\"decl\":%.1f,\"dgps\":%s}",
                  m[0], m[1], m[2], magCal->fieldUT(), magCal->calibrated() ? "true" : "false",
                  magCal->trusted() ? "true" : "false", s.magDisagrees ? "true" : "false", magCal->declinationDeg(),
                  magCal->declinationFromGps() ? "true" : "false");
        }
        if (s.posValid)
        {
            // оценка положения (от дома, м), скорость; удержание позиции: этап, точка, почему не держит
            w.add(",\"pos\":{\"n\":%.2f,\"e\":%.2f,\"vn\":%.2f,\"ve\":%.2f", s.posN, s.posE, s.velN, s.velE);
            if (s.posHoldPhase != copter::PosHoldPhase::OFF)
            {
                w.add(",\"hold\":\"%s\",\"tn\":%.2f,\"te\":%.2f", copter::posHoldPhaseName(s.posHoldPhase), s.posTargetN,
                      s.posTargetE);
            }
            w.add("}");
        }
        if (s.posHoldFail != copter::PosHoldFail::NONE) w.add(",\"posfail\":\"%s\"", copter::posHoldFailName(s.posHoldFail));
        spectrum(w);
        if (const char* tl = copter::trainingText(s.trainingFlags)) w.add(",\"train\":\"%s\"", tl);
        if (ctl.getGeofence().enabled()) w.add(",\"fence\":{\"f\":%u,\"m\":%.0f}", s.fenceFlags, s.fenceMarginM);
        if (s.rescuePhase != RescuePhase::IDLE)
        {
            w.add(",\"rescue\":\"%s\"", rescuePhaseName(s.rescuePhase));
            if (s.rescueFail != RescueFail::NONE) w.add(",\"rescueFail\":\"%s\"", rescueFailName(s.rescueFail));
        }
        // домой вслепую: этап, место по своим датчикам (м от дома) и его неуверенность
        if (s.deadReckPhase != DeadReckonPhase::IDLE)
            w.add(",\"blind\":{\"phase\":\"%s\",\"n\":%.0f,\"e\":%.0f,\"err\":%.0f}", deadReckonPhaseName(s.deadReckPhase),
                  static_cast<double>(s.deadReckN), static_cast<double>(s.deadReckE), static_cast<double>(s.deadReckErrM));
        if (gps && gpsHome && gps->isAvailable())
        {
            const copter::GpsFix& f = gpsHome->fix();
            w.add(",\"gps\":{\"fix\":%s,\"sats\":%u,\"lat\":%.7f,\"lon\":%.7f,\"spd\":%.1f", f.valid ? "true" : "false",
                  f.satellites, f.latitude, f.longitude, f.speedMs);
            if (gpsHome->hasHome()) w.add(",\"home\":%.0f,\"dir\":%.0f", gpsHome->distanceM(), gpsHome->bearingHomeDeg());
            if (!f.valid && gpsHome->hasLastGood())   // фикс пропал — где был (найти упавший)
                w.add(",\"last\":[%.7f,%.7f]", gpsHome->lastGood().latitude, gpsHome->lastGood().longitude);
            // подмена (copter/GpsIntegrity.h): доверие 0…100 и причина, если GPS выключен из навигации
            w.add(",\"trust\":%u", static_cast<unsigned>(s.gpsTrust));
            if (s.gpsSpoof != GpsSpoof::NONE) w.add(",\"spoof\":\"%s\"", gpsSpoofName(s.gpsSpoof));
            w.add("}");
        }
        w.add(",\"motors\":[");
        for (uint8_t i = 0; i < outputs.count && i < MAX_MOTORS; ++i) w.add(i ? ",%.3f" : "%.3f", outputs.motor[i]);
        w.add("],\"block\":[");
        bool first = true;
        for (uint32_t bit = 1; bit && bit <= ARM_BLOCK_LAST; bit <<= 1)
        {
            if (!(s.armBlockers & bit)) continue;
            w.add(first ? "\"%s\"" : ",\"%s\"", armBlockCode(bit));
            first = false;
        }
        w.add("],\"modes\":[");
        first = true;
        const uint64_t modes = rxInput ? rxInput->activeModes() : 0;
        for (uint8_t b = 0; b < BOX_COUNT; ++b)
        {
            if (!((modes >> b) & 1u)) continue;
            w.add(first ? "%u" : ",%u", b);
            first = false;
        }
        const unsigned cpu = loopHz ? static_cast<unsigned>(loopAvgUs * loopHz / 10000u) : 0u;
        w.add("],\"cpu\":%u,\"hz\":%lu,\"unsaved\":%s}\n", cpu, static_cast<unsigned long>(loopHz), dirty ? "true" : "false");
        return w.overflow ? 0 : w.length;
    }

private:
    // Спектр гироскопа (060–063): "spec":{"bw" — ширина полосы, Гц; "db" —
    // амплитуда полос 0, bw, 2bw… (сумма осей) в дБ над 0.01 °/с; "dn" —
    // центры динамических режекторов по осям; "rpm" — обороты моторов, Гц,
    // "src" — откуда (dshot | model)}.
    void spectrum(JsonWriter& w) const
    {
        using namespace copter;
        const GyroFilter& f = ctl.getGyroFilter();
        const GyroSpectrum& sp = f.getSpectrum();
        w.add(",\"spec\":{\"bw\":%.1f,\"db\":[", sp.binWidthHz());
        for (uint8_t k = 0; k < SDFT_BINS; ++k)
        {
            // у синуса амплитуды A пик окна Ханна — A·N/4
            const float amp = sqrtf(sp.binPower(k)) * 4.0f / SDFT_SIZE;
            const float db = amp > 0.01f ? 20.0f * log10f(amp * 100.0f) : 0.0f;
            w.add(k ? ",%d" : "%d", static_cast<int>(lroundf(db)));
        }
        w.add("],\"dn\":[");
        for (uint8_t a = 0; a < 3; ++a)
        {
            w.add(a ? ",[" : "[");
            for (uint8_t i = 0; i < f.dynNotchCount(); ++i) w.add(i ? ",%.0f" : "%.0f", f.dynNotchHz(a, i));
            w.add("]");
        }
        w.add("],\"rpm\":[");
        for (uint8_t m = 0; m < f.motorCount(); ++m) w.add(m ? ",%.0f" : "%.0f", f.motorRotationHz(m));
        w.add("],\"src\":\"%s\"}", ctl.getStatus().rpmTelemetry ? "dshot" : "model");
    }

    // EDT регуляторов (dshot_edt): "esc":[[°C, В, А, статус], …] по моторам,
    // null — значение ещё не приходило; нет EDT ни от одного — строки нет.
    void escTelemetry(JsonWriter& w) const
    {
        using copter::EscExtTelemetry;
        if (!motorTelemetry || !motorTelemetry->telemetryEnabled()) return;
        const uint8_t n = motorTelemetry->telemetryMotors();
        bool any = false;
        for (uint8_t i = 0; i < n && !any; ++i) any = motorTelemetry->extended(i) != nullptr;
        if (!any) return;
        w.add(",\"esc\":[");
        for (uint8_t i = 0; i < n; ++i)
        {
            const EscExtTelemetry* x = motorTelemetry->extended(i);
            w.add(i ? ",[" : "[");
            if (x && x->has(EscExtTelemetry::TEMPERATURE)) w.add("%d", x->temperatureC);
            else w.add("null");
            if (x && x->has(EscExtTelemetry::VOLTAGE)) w.add(",%.2f", x->volts);
            else w.add(",null");
            if (x && x->has(EscExtTelemetry::CURRENT)) w.add(",%.0f", x->amps);
            else w.add(",null");
            if (x && x->has(EscExtTelemetry::STATUS)) w.add(",%u]", static_cast<unsigned>(x->status));
            else w.add(",null]");
        }
        w.add("]");
    }

    // строки ответа на params: заголовок, настройки, @rc, @modes
    static constexpr int DUMP_LAST = copter::PARAM_COUNT + 2 + copter::GeoZoneTable::MAX_ZONES;

    size_t formatDumpLine(int step, char* buffer, size_t capacity) const
    {
        using namespace copter;
        if (step == 0)
        {
            JsonWriter w(buffer, capacity);
            const BuildSpec b = params.buildSpec();
            w.add("@params {\"count\":%d,\"profiles\":%u,\"profile\":%u,\"rateprofile\":%u,\"build\":", PARAM_COUNT,
                  CopterParams::PROFILES, params.pidProfile(), params.rateProfile());
            w.str(b.id ? b.id : "");
            w.add(",\"source\":\"%s\",\"unsaved\":%s", params.source() == TuneSource::BETAFLIGHT ? "BETAFLIGHT" : "MODEL",
                  dirty ? "true" : "false");
            flashUse(w);
            // рама для вкладки «Моторы»: где мотор (x — к носу, y — вправо) и куда крутится
            const Frame& f = ctl.getTune().frame;
            w.add(",\"frame\":{\"type\":\"%s\",\"motors\":[", frameName(f.type));
            for (uint8_t i = 0; i < f.count; ++i)
            {
                const MotorPlace& p = f.place[i];
                w.add("%s[%.2f,%.2f,\"%s\",%s]", i ? "," : "", p.x, p.y, p.spin == Spin::CW ? "cw" : "ccw",
                      p.bottom ? "true" : "false");
            }
            w.add("]},\"motorTestMax\":%.0f,\"dshotCommands\":%s", copter::CopterController::MOTOR_TEST_MAX * 100.0f,
                  cli.dshotCommandsAvailable() ? "true" : "false");
            // P63: UART платы для вкладки «Порты»: [номер, есть RX, есть TX]
            const BoardUarts& u = cli.boardUarts();
            w.add(",\"uarts\":[");
            bool first = true;
            for (uint8_t n = 1; n <= MAX_UARTS; ++n)
            {
                if (!u.exists(n)) continue;
                w.add("%s[%u,%u,%u]", first ? "" : ",", n, u.rx[n] ? 1u : 0u, u.tx[n] ? 1u : 0u);
                first = false;
            }
            w.add("]");
            margin(w);
            w.add("}\n");
            return w.overflow ? 0 : w.length;
        }
        if (step <= PARAM_COUNT) return formatParam(static_cast<ParamId>(step - 1), buffer, capacity);
        if (step == PARAM_COUNT + 1) return formatRc(buffer, capacity);
        // зоны — до @modes: по @modes приложение считает загрузку законченной
        if (step < DUMP_LAST) return formatFence(static_cast<uint8_t>(step - PARAM_COUNT - 2), buffer, capacity);
        return formatModes(buffer, capacity);
    }

    // «motor N проценты» (N — 0…, all — все) / «motor stop»: проверка моторов
    // без пропеллеров, до MOTOR_TEST_MAX; держится MOTOR_HOLD_S — приложение
    // повторяет команду, пока ползунок не на нуле: связь пропала — моторы встали.
    static constexpr float MOTOR_HOLD_S = 1.0f;

    void doMotor(char* args)
    {
        using copter::CopterController;
        if (ctl.isArmed() || cli.active())
        {
            sendError("motor", ctl.isArmed() ? "только без ARM" : "открыт CLI");
            return;
        }
        bool ok = true;
        if (strncmp(args, "stop", 4) == 0)
        {
            ctl.stopMotorTest();
        }
        else
        {
            char* end = nullptr;
            const bool all = strncmp(args, "all", 3) == 0;
            const long index = all ? 0 : strtol(args, &end, 10);
            if (all) end = args + 3;
            const float percent = strtof(end, nullptr);
            ok = (all || end != args) && percent >= 0.0f &&
                 ctl.setMotorTest(all ? CopterController::MOTOR_TEST_ALL : static_cast<uint8_t>(index < 0 ? 255 : index),
                                  percent / 100.0f, MOTOR_HOLD_S);
            if (!ok)
            {
                sendError("motor", "motor N|all проценты (0-30) · motor stop");
                return;
            }
        }
        char line[160];
        JsonWriter w(line, sizeof(line));
        w.add("@motor {\"ok\":true,\"max\":%.0f,\"m\":[", CopterController::MOTOR_TEST_MAX * 100.0f);
        for (uint8_t i = 0; i < ctl.getTune().frame.count && i < copter::MAX_MOTORS; ++i)
        {
            w.add(i ? ",%.1f" : "%.1f", ctl.motorTestValue(i) * 100.0f);
        }
        w.add("]}\n");
        send(w);
    }

    // «dshot N|all команда…» — команды регуляторам (мастер направления
    // моторов, «пискнуть»): только без ARM, при закрытом CLI и без проверки моторов.
    void doDshot(char* args)
    {
        if (cli.active())
        {
            sendError("dshot", "открыт CLI");
            return;
        }
        char* end = nullptr;
        const bool all = strncmp(args, "all", 3) == 0;
        const long index = all ? 255 : strtol(args, &end, 10);
        if (all) end = args + 3;
        uint16_t commands[8];
        uint8_t n = 0;
        const char* p = end;
        while (end != args && n < 8)
        {
            char* next = nullptr;
            const long c = strtol(p, &next, 10);
            if (next == p) break;
            commands[n++] = static_cast<uint16_t>(c < 0 || c > 0xFFFF ? 0 : c);
            p = next;
        }
        if (end == args || index < 0 || index > 255)
        {
            sendError("dshot", "dshot N|all команда [команда…]");
            return;
        }
        const char* error = cli.queueDshot(static_cast<uint8_t>(index), commands, n);
        if (error)
        {
            sendError("dshot", error);
            return;
        }
        char line[96];
        JsonWriter w(line, sizeof(line));
        w.add("@dshot {\"ok\":true,\"motor\":%ld,\"commands\":[", index);
        for (uint8_t i = 0; i < n; ++i) w.add(i ? ",%u" : "%u", commands[i]);
        w.add("]}\n");
        send(w);
    }

    void doBeep(const char* args)
    {
        const copter::BeeperCondition c = copter::beeperConditionByName(args);
        if (c == copter::BeeperCondition::NONE || c == copter::BeeperCondition::COUNT)
        {
            sendError("beep", "beep УСЛОВИЕ (как в beeper list)");
            return;
        }
        if (!cli.previewBeeper(c))
        {
            sendError("beep", ctl.isArmed() ? "только без ARM" : "пищалки нет или так не пищит");
            return;
        }
        char line[80];
        JsonWriter w(line, sizeof(line));
        w.add("@beep {\"ok\":true,\"name\":\"%s\"}\n", copter::beeperConditionName(c));
        send(w);
    }

    // «calibrate vbat 16.80» / «calibrate ibat 12.5» — по мультиметру: новый
    // vbat_scale / ibata_scale, ответ — как на set (сохранить — save).
    void doCalibrate(char* args)
    {
        char* what = args;
        char* value = args;
        while (*value && *value != ' ') ++value;
        if (*value) *value++ = '\0';
        const float measured = strtof(value, nullptr);
        const bool vbat = strcmp(what, "vbat") == 0;
        if (!battery || (!vbat && strcmp(what, "ibat") != 0))
        {
            sendError("calibrate", battery ? "calibrate vbat вольты | ibat амперы" : "батарея не измеряется");
            return;
        }
        const float scale = vbat ? battery->vbatScaleFor(measured) : battery->ibataScaleFor(measured);
        if (scale == 0.0f)
        {
            sendError("calibrate", vbat ? "нет напряжения батареи" : "нет тока (нужен датчик и хотя бы 0.5 А)");
            return;
        }
        char line[48];
        snprintf(line, sizeof(line), "%s %ld", vbat ? "vbat_scale" : "ibata_scale", lroundf(scale));
        doSet(line);
    }

    void doSet(char* args)
    {
        using namespace copter;
        // «имя значение» или «имя = значение»
        char* value = args;
        while (*value && *value != ' ' && *value != '=') ++value;
        if (*value) *value++ = '\0';
        while (*value == ' ' || *value == '=') ++value;
        const int found = findParam(args);
        char line[LINE_CAPACITY];
        JsonWriter w(line, sizeof(line));
        w.add("@set {\"name\":");
        w.str(args);
        char error[96] = {};
        if (found < 0)
        {
            w.add(",\"ok\":false,\"error\":");
            w.str("нет такой настройки");
        }
        else
        {
            const ParamId id = static_cast<ParamId>(found);
            const PortMap portsBefore = portMap(params);
            bool ok = params.parse(id, params.activeSlot(id), value, error, sizeof(error));
            // P63: UART, которого нет на плате, или занятый другой ролью — нельзя
            if (ok && paramInfo(id).group == ParamGroup::PORTS && cli.boardUarts().any() &&
                !checkPortMap(portMap(params), cli.boardUarts(), error, sizeof(error)))
            {
                setPortMap(params, portsBefore);
                ok = false;
            }
            if (ok) dirty = true;
            w.add(",\"ok\":%s", ok ? "true" : "false");
            if (!ok)
            {
                w.add(",\"error\":");
                w.str(error);
            }
            w.add(",\"value\":");
            this->value(w, id, params.activeSlot(id));
            // PID и фильтры: запас контура заново (полоса под ползунками)
            const ParamGroup g = paramInfo(id).group;
            if (ok && (g == ParamGroup::PID || g == ParamGroup::FILTERS)) margin(w);
        }
        w.add("}\n");
        send(w);
    }

    // ,"margin":{"gm":дБ,"pm":°,"hz":срез,"min":дБ,"model":{...}} — запасы контура
    // для PID, что работают, и модель для ступеньки во вкладке «PID» Studio:
    // "loop" Гц, "tau" — мотор, с, "delay" — задержка без фильтров гироскопа, с,
    // "iy" — integrated yaw (рысканье не моделируется), "ax" — по осям
    // [K (°/с²) на единицу команды, P, I, D (с d_max), FF] — коэффициенты в
    // миллионных долях единиц прошивки (без экспонент: %g на плате не нужен).
    // Флеш настроек: занято и всего, байт (вкладка «Прочее»).
    void flashUse(JsonWriter& w) const
    {
        const KeyValueStore* kv = cli.store();
        if (kv) w.add(",\"flash\":[%u,%u]", static_cast<unsigned>(kv->bytesUsed()), static_cast<unsigned>(KeyValueStore::CAPACITY));
    }

    void margin(JsonWriter& w) const
    {
        const copter::CopterTune t = params.tune();
        const copter::LoopCheck c = copter::checkLoop(t);
        w.add(",\"margin\":{\"gm\":%.1f,\"pm\":%.0f,\"hz\":%.1f,\"min\":%.0f", c.gainMarginDb, c.phaseMarginDeg,
              c.crossoverHz, copter::MIN_GAIN_MARGIN_DB);
        if (t.kRoll > 0.0f && t.loopHz > 0.0f)
        {
            // та же задержка, что у checkLoop: полтора цикла, ESC, полмиллисекунды на DShot
            w.add(",\"model\":{\"loop\":%.0f,\"tau\":%.4f,\"delay\":%.5f,\"iy\":%d,\"ax\":[", t.loopHz, t.motorTauS,
                  1.5f / t.loopHz + t.escDelayS + 0.0005f, t.pid.integratedYaw ? 1 : 0);
            const copter::AxisGains* g[3] = { &t.roll, &t.pitch, &t.yaw };
            const float k[3] = { t.kRoll, t.kPitch, t.kYaw };
            constexpr float DEG = 180.0f / 3.14159265f;
            for (uint8_t a = 0; a < 3; ++a)
            {
                constexpr float U = 1e6f;
                w.add("%s[%.0f,%.2f,%.2f,%.3f,%.3f]", a ? "," : "", k[a] * DEG, g[a]->p * U, g[a]->i * U,
                      fmaxf(g[a]->d, t.pid.dMax[a]) * U, g[a]->ff * U);
            }
            w.add("]}");
        }
        w.add("}");
    }

    void doProfile(const char* cmd, const char* args)
    {
        char* end = nullptr;
        const long n = strtol(args, &end, 10);
        const bool rates = strcmp(cmd, "rateprofile") == 0;
        const bool ok = end != args && n >= 0 &&
                        (rates ? params.selectRateProfile(static_cast<uint8_t>(n)) : params.selectPidProfile(static_cast<uint8_t>(n)));
        if (!ok)
        {
            sendError(cmd, "профиль 0-3");
            return;
        }
        dirty = true;
        ctl.applyProfile(params.tune());
        char line[96];
        JsonWriter w(line, sizeof(line));
        w.add("@profile {\"profile\":%u,\"rateprofile\":%u}\n", params.pidProfile(), params.rateProfile());
        send(w);
    }

    void doRcTable(const char* cmd, char* args)
    {
        using namespace copter;
        RcConfig& c = params.rc();
        char error[96] = {};
        bool ok = false;
        if (strcmp(cmd, "aux") == 0)
        {
            ok = c.parseAux(args, error, sizeof(error));
        }
        else if (strcmp(cmd, "map") == 0)
        {
            ok = c.setMap(args, error, sizeof(error));
        }
        else if (strcmp(cmd, "vtx") == 0)
        {
            ok = c.parseVtx(args, VTX_BANDS, VtxController::MAX_LEVELS, error, sizeof(error));
        }
        else if (strcmp(cmd, "rxfail") == 0)
        {
            ok = c.parseRxfail(args, error, sizeof(error));
        }
        else
        {
            char* end = nullptr;
            const long i = strtol(args, &end, 10);
            const long lo = strtol(end, &end, 10);
            const long hi = strtol(end, &end, 10);
            ok = c.setRange(i, lo, hi, error, sizeof(error));
        }
        if (!ok)
        {
            sendError(cmd, error);
            return;
        }
        dirty = true;
        char line[LINE_CAPACITY];
        const size_t length = formatRc(line, sizeof(line));
        if (length) io.write(reinterpret_cast<const uint8_t*>(line), length);
    }

    void doSave()
    {
        const copter::CopterCli::SaveResult r = cli.saveAndApply();
        char line[LINE_CAPACITY];   // запасы и модель для ступеньки (margin) — ~300 байт
        JsonWriter w(line, sizeof(line));
        w.add("@save {\"ok\":%s,\"stored\":%s", r == copter::CopterCli::SaveResult::FLASH_FULL ? "false" : "true",
              r == copter::CopterCli::SaveResult::SAVED ? "true" : "false");
        if (r == copter::CopterCli::SaveResult::NO_STORE)
        {
            w.add(",\"error\":");
            w.str("некуда сохранить — работает до перезагрузки");
        }
        if (r == copter::CopterCli::SaveResult::FLASH_FULL)
        {
            w.add(",\"error\":");
            w.str("флеш настроек переполнен");
        }
        margin(w);   // описание коптера сменилось — модель пересчитана при save
        flashUse(w);
        w.add("}\n");
        send(w);
    }

    void doRevert()
    {
        cli.revert();
        io.print("@revert {\"ok\":true}\n");
    }

    void sendId()
    {
        char line[448];   // имя модели из Project.h — до ~100 букв кириллицы
        JsonWriter w(line, sizeof(line));
        w.add("@id {\"fw\":\"openplane\",\"proto\":%d,\"vehicle\":\"copter\",\"board\":", FirmwareIdentity::PROTOCOL_VERSION);
        w.str(OP_BOARD_ENV);
        w.add(",\"commit\":");
        w.str(OP_SOURCE_COMMIT);
        w.add(",\"build\":");
        const copter::BuildSpec b = params.buildSpec();
        w.str(b.id ? b.id : "");
        // config и plane вшивает приложение при сборке (config/Project.h)
        w.add(",\"config\":");
        w.str(OP_CONFIG_HASH);
        w.add(",\"plane\":");
        w.str(OP_PLANE_NAME);
        w.add(",\"rx\":");
        w.str(rc.getProtocolName());
        w.add(",\"armed\":%s}\n", ctl.isArmed() ? "true" : "false");
        send(w);
    }

    void sendError(const char* cmd, const char* error)
    {
        char line[256];
        JsonWriter w(line, sizeof(line));
        w.add("@error {\"cmd\":");
        w.str(cmd);
        w.add(",\"error\":");
        w.str(error);
        w.add("}\n");
        send(w);
    }

    void send(const JsonWriter& w)
    {
        if (!w.overflow) io.write(reinterpret_cast<const uint8_t*>(w.buffer), w.length);
    }

    // Значение для JSON: числа — числом, выбор и строки — строкой.
    void value(JsonWriter& w, copter::ParamId id, uint8_t slot) const
    {
        using namespace copter;
        const ParamInfo& info = paramInfo(id);
        char text[48];
        params.format(id, slot, text, sizeof(text));
        if (info.type == ParamType::ENUM || info.type == ParamType::STRING) w.str(text);
        else w.add("%s", text);
    }

    static void number(JsonWriter& w, const copter::ParamInfo& info, float v)
    {
        char text[48];
        copter::CopterParams::formatNumber(info, v, text, sizeof(text));
        if (info.type == copter::ParamType::ENUM) w.str(text);
        else w.add("%s", text);
    }

    template <typename Get>
    static void auxList(JsonWriter& w, Get get)
    {
        w.add("[");
        bool first = true;
        for (uint8_t i = 0; i < copter::RcConfig::AUX_SLOTS; ++i)
        {
            const copter::AuxRange& a = get(i);
            if (!a.used()) continue;
            w.add("%s[%u,%u,%u,%u,%u,%u,%u]", first ? "" : ",", i, a.mode, a.channel, a.startUs, a.endUs, a.logic, a.linked);
            first = false;
        }
        w.add("]");
    }

    static const char* typeName(copter::ParamType t)
    {
        switch (t)
        {
            case copter::ParamType::FLOAT: return "float";
            case copter::ParamType::ENUM: return "enum";
            case copter::ParamType::STRING: return "string";
            case copter::ParamType::INT:
            default: return "int";
        }
    }

    static const char* scopeName(copter::ParamScope s)
    {
        switch (s)
        {
            case copter::ParamScope::PID_PROFILE: return "pid";
            case copter::ParamScope::RATE_PROFILE: return "rate";
            case copter::ParamScope::MASTER:
            default: return "master";
        }
    }

    static const char* fromName(copter::ParamDefault d)
    {
        switch (d)
        {
            case copter::ParamDefault::TUNE: return "tune";
            case copter::ParamDefault::BUILD: return "build";
            case copter::ParamDefault::BATTERY: return "battery";
            case copter::ParamDefault::TABLE:
            default: return "table";
        }
    }

    static const char* stateName(copter::CopterState st)
    {
        switch (st)
        {
            case copter::CopterState::ARMED: return "ARMED";
            case copter::CopterState::FAILSAFE: return "FAILSAFE";
            default: return "DISARMED";
        }
    }

    Stream& io;
    copter::CopterController& ctl;
    copter::CopterParams& params;
    copter::CopterCli& cli;
    RcReceiver& rc;
    const copter::RxInput* rxInput = nullptr;
    const copter::BatteryMonitor* battery = nullptr;
    const copter::EscGuard* guard = nullptr;   // перегрузка по току (ovl, mma)
    const copter::IMotorTelemetry* motorTelemetry = nullptr;   // EDT -> "esc"
    const copter::Rssi* rssi = nullptr;        // сила сигнала пульта (rssi, rssisrc)
    const copter::VtxController* vtx = nullptr;
    const copter::OsdOutputStatus* osdOut = nullptr;
    const GpsSensor* gps = nullptr;
    const copter::GpsHome* gpsHome = nullptr;
    float headingDeg = 0.0f;
    const char* txModel = nullptr;
    const copter::Compass* magCal = nullptr;
    const ImuSensor* imu = nullptr;
    const BarometerSensor* baro = nullptr;
    bool magPresent = false;
    const copter::GyroPick* gyroPick = nullptr;
    const uint8_t* gyroAlive = nullptr;
    const copter::GyroAgreement* gyroAgree = nullptr;
    copter::CopterOutputs outputs;
    uint32_t loopAvgUs = 0, loopHz = 0;
    bool dirty = false;
    int dumpStep = -1;
    bool live = false;
    bool liveSent = false;
    uint32_t liveUntilMs = 0;
    uint32_t lastLiveMs = 0;
};
