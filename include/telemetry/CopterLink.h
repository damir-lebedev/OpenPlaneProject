#pragma once
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "copter/CopterCli.h"
#include "copter/Battery.h"
#include "copter/CopterController.h"
#include "copter/GpsHome.h"
#include "copter/Params.h"
#include "copter/RcSetup.h"
#include "copter/Vtx.h"
#include "hal/Platform.h"
#include "rc/RcReceiver.h"
#include "telemetry/FirmwareIdentity.h"
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
//                       rxrange, таблица aux и тумблеры прошивки, строки vtx) и
//                       "@modes [[номер, "имя", действует], …]" (режимы BF);
//   set имя значение  — "@set {...}": ok или ошибка, новое значение;
//   profile N · rateprofile N — "@profile {...}" (сразу, на ходу);
//   aux … · map … · rxrange … — как в CLI; ответ — "@rc {...}";
//   save              — во флеш и в работу: "@save {...}";
//   revert            — отменить несохранённое: "@revert {...}";
//   live · live 0     — "@live {...}" 20 раз в секунду ещё 3 с / хватит.
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
    void setVtx(const copter::VtxController* controller) { vtx = controller; }
    void setGps(const GpsSensor* sensor, const copter::GpsHome* home)
    {
        gps = sensor;
        gpsHome = home;
    }
    void setHeading(float deg) { headingDeg = deg; }
    void setOutputs(const copter::CopterOutputs& out) { outputs = out; }
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
        const bool changes = strcmp(cmd, "set") == 0 || strcmp(cmd, "profile") == 0 || strcmp(cmd, "rateprofile") == 0 ||
                             strcmp(cmd, "aux") == 0 || strcmp(cmd, "map") == 0 || strcmp(cmd, "rxrange") == 0 ||
                             strcmp(cmd, "save") == 0 || strcmp(cmd, "revert") == 0;
        if (!changes) return false;
        if (ctl.isArmed() || cli.active())
        {
            sendError(cmd, ctl.isArmed() ? "только без ARM" : "открыт CLI");
            return true;
        }
        if (strcmp(cmd, "set") == 0) doSet(args);
        else if (strcmp(cmd, "profile") == 0 || strcmp(cmd, "rateprofile") == 0) doProfile(cmd, args);
        else if (strcmp(cmd, "save") == 0) doSave();
        else if (strcmp(cmd, "revert") == 0) doRevert();
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
        if (info.type == ParamType::STRING) w.add("\"\"");
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
        w.add("]}\n");
        return w.overflow ? 0 : w.length;
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
        w.add(",\"rx\":\"%s\"", rc.isFrameTimeout() ? "lost" : rc.isFailsafeReported() ? "fs" : "ok");
        if (rc.getLinkQuality() >= 0) w.add(",\"lq\":%d", static_cast<int>(rc.getLinkQuality()));
        w.add(",\"ch\":[");
        for (uint8_t i = 0; i < Config::RC_CHANNELS; ++i) w.add(i ? ",%u" : "%u", static_cast<unsigned>(rc.getState().get(i)));
        w.add("],\"att\":[%.1f,%.1f,%.1f],\"alt\":%.2f,\"vz\":%.2f,\"thr\":%.3f,\"v\":%.2f", s.roll, s.pitch, s.yaw,
              s.altitude, s.climbRate, s.throttle, s.volts);
        if (battery)
        {
            w.add(",\"bat\":\"%s\",\"cell\":%.2f", copter::batteryStateName(battery->state()), battery->cellVolts());
            if (battery->hasCurrent()) w.add(",\"amps\":%.1f,\"mah\":%.0f", battery->amps(), battery->mahDrawn());
        }
        if (vtx)
        {
            const copter::VtxState& v = vtx->state();
            w.add(",\"vtx\":{\"on\":%s,\"mhz\":%u,\"mw\":%u,\"pit\":%s,\"cap\":%u}", v.online ? "true" : "false",
                  v.frequencyMhz, v.powerMw, v.pit ? "true" : "false", vtx->levelMw(vtx->topAllowedLevel()));
        }
        w.add(",\"hdg\":%.0f", headingDeg);
        if (gps && gpsHome && gps->isAvailable())
        {
            const copter::GpsFix& f = gpsHome->fix();
            w.add(",\"gps\":{\"fix\":%s,\"sats\":%u,\"lat\":%.7f,\"lon\":%.7f,\"spd\":%.1f", f.valid ? "true" : "false",
                  f.satellites, f.latitude, f.longitude, f.speedMs);
            if (gpsHome->hasHome()) w.add(",\"home\":%.0f,\"dir\":%.0f", gpsHome->distanceM(), gpsHome->bearingHomeDeg());
            w.add("}");
        }
        w.add(",\"motors\":[");
        for (uint8_t i = 0; i < outputs.count && i < MAX_MOTORS; ++i) w.add(i ? ",%.3f" : "%.3f", outputs.motor[i]);
        w.add("],\"block\":[");
        bool first = true;
        for (uint32_t bit = 1; bit <= ARM_BLOCK_LAST; bit <<= 1)
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
    // строки ответа на params: заголовок, настройки, @rc, @modes
    static constexpr int DUMP_LAST = copter::PARAM_COUNT + 2;

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
            w.add(",\"source\":\"%s\",\"unsaved\":%s}\n", params.source() == TuneSource::BETAFLIGHT ? "BETAFLIGHT" : "MODEL",
                  dirty ? "true" : "false");
            return w.overflow ? 0 : w.length;
        }
        if (step <= PARAM_COUNT) return formatParam(static_cast<ParamId>(step - 1), buffer, capacity);
        if (step == PARAM_COUNT + 1) return formatRc(buffer, capacity);
        return formatModes(buffer, capacity);
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
            const bool ok = params.parse(id, params.activeSlot(id), value, error, sizeof(error));
            if (ok) dirty = true;
            w.add(",\"ok\":%s", ok ? "true" : "false");
            if (!ok)
            {
                w.add(",\"error\":");
                w.str(error);
            }
            w.add(",\"value\":");
            this->value(w, id, params.activeSlot(id));
        }
        w.add("}\n");
        send(w);
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
        char line[160];
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
        char line[320];
        JsonWriter w(line, sizeof(line));
        w.add("@id {\"fw\":\"openplane\",\"proto\":%d,\"vehicle\":\"copter\",\"board\":", FirmwareIdentity::PROTOCOL_VERSION);
        w.str(OP_BOARD_ENV);
        w.add(",\"commit\":");
        w.str(OP_SOURCE_COMMIT);
        w.add(",\"build\":");
        const copter::BuildSpec b = params.buildSpec();
        w.str(b.id ? b.id : "");
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
    const copter::VtxController* vtx = nullptr;
    const GpsSensor* gps = nullptr;
    const copter::GpsHome* gpsHome = nullptr;
    float headingDeg = 0.0f;
    copter::CopterOutputs outputs;
    uint32_t loopAvgUs = 0, loopHz = 0;
    bool dirty = false;
    int dumpStep = -1;
    bool live = false;
    bool liveSent = false;
    uint32_t liveUntilMs = 0;
    uint32_t lastLiveMs = 0;
};
