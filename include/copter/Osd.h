#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// OSD коптера — что рисуется поверх картинки камеры (чип AT7456E/MAX7456,
// hal/Max7456.h): экран 30 столбцов × 16 строк (PAL) или 13 (NTSC).
//
// Шрифт в чипе — шрифт Betaflight (его зашивает производитель полётника):
// латиница, цифры и значки (батарея, спутники, стрелки). Поэтому надписи —
// латиницей и короткие, как в BF: LOW BATTERY, RXLOSS, DISARMED…
//
// Раскладка — как в BF: у каждого элемента настройка osd_<имя>_pos (x + 32·y,
// бит 0x0800 — показан; биты 0x1000/0x2000 — профили 2 и 3 BF, у нас любой
// из трёх значит «показан»; бит 0x0400 — шестой бит x для HD). diff из BF
// переносится как есть. Умолчание — в пределах NTSC (13 строк):
//
//   ┌──────────────────────────────┐
//   │ LQ100        ANGL       ⊲12⊳ │ 0  связь · режим · спутники
//   │                       400MW  │ 1  мощность VTX
//   │                              │
//   │              +               │ 6  прицел
//   │         LOW BATTERY          │ 9  предупреждение (одно, главное)
//   │ 12.3A  1234MAH               │ 10 ток · мА·ч
//   │ ↑125M                 ▲12M   │ 11 домой: стрелка и расстояние · высота
//   │ 25.2V  4.20V          ⏱02:13 │ 12 батарея · банка · время в полёте
//   └──────────────────────────────┘
//
// После полёта (DISARM) — экран статистики (osd_stat_*), пока не двинули
// газ или тангаж вверх, не заармили снова или не прошла минута.
//
// Без Arduino: собирает экран в буфер, чип пишет драйвер — только то, что
// поменялось.
// ============================================================

namespace copter
{
    // Значки шрифта Betaflight (номера символов в чипе).
    namespace osdsym
    {
        constexpr uint8_t RSSI = 0x01;
        constexpr uint8_t THROTTLE = 0x04;
        constexpr uint8_t VOLT = 0x06;
        constexpr uint8_t MAH = 0x07;
        constexpr uint8_t METRE = 0x0C;
        constexpr uint8_t HOME = 0x11;
        constexpr uint8_t SAT_L = 0x1E;
        constexpr uint8_t SAT_R = 0x1F;
        constexpr uint8_t WATT = 0x57;
        constexpr uint8_t ARROW_SOUTH = 0x60;   // 16 стрелок: 0x60 юг, 0x64 восток, 0x68 север, 0x6C запад
        constexpr uint8_t SPEED = 0x70;
        constexpr uint8_t CROSSHAIR = 0x73;
        constexpr uint8_t LINK_QUALITY = 0x7B;
        constexpr uint8_t ALTITUDE = 0x7F;
        constexpr uint8_t LAT = 0x89;
        constexpr uint8_t LON = 0x98;
        constexpr uint8_t AMP = 0x9A;
        constexpr uint8_t FLY_TIME = 0x9C;
        constexpr uint8_t KPH = 0x9E;
        constexpr uint8_t MPS = 0x9F;
    }

    // Стрелка шрифта BF на угол относительно носа (0 — вперёд/вверх, по часовой).
    inline uint8_t osdArrow(float relativeDeg)
    {
        int k = static_cast<int>(lroundf(relativeDeg / 22.5f));
        k = ((8 - k) % 16 + 16) % 16;
        return static_cast<uint8_t>(osdsym::ARROW_SOUTH + k);
    }

    // ---- раскладка: позиции как в BF ----

    constexpr uint16_t OSD_VISIBLE = 0x0800;              // osd_*_pos: показан (профиль 1 BF)
    constexpr uint16_t OSD_PROFILE_MASK = 0x3800;         // профили 1–3 BF
    constexpr uint16_t OSD_POS_MAX = 0x3FFF;

    constexpr uint16_t osdPos(uint8_t x, uint8_t y, bool visible = true)
    {
        return static_cast<uint16_t>((x & 0x1F) | ((y & 0x1F) << 5) | ((x & 0x20) << 5) | (visible ? OSD_VISIBLE : 0));
    }
    inline uint8_t osdPosX(uint16_t p) { return static_cast<uint8_t>((p & 0x1F) | ((p >> 5) & 0x20)); }
    inline uint8_t osdPosY(uint16_t p) { return static_cast<uint8_t>((p >> 5) & 0x1F); }
    inline bool osdPosVisible(uint16_t p) { return (p & OSD_PROFILE_MASK) != 0; }

    // Элементы — по именам настроек BF (osd_<name>_pos).
    enum OsdElement : uint8_t
    {
        OSD_LINK_QUALITY,
        OSD_FLYMODE,
        OSD_GPS_SATS,
        OSD_VTX_CHANNEL,
        OSD_CROSSHAIRS,
        OSD_WARNINGS,
        OSD_CURRENT_DRAW,
        OSD_MAH_DRAWN,
        OSD_HOME_DIR,
        OSD_HOME_DIST,
        OSD_ALTITUDE,
        OSD_MAIN_BATT_VOLTAGE,
        OSD_AVG_CELL_VOLTAGE,
        OSD_FLYTIME,
        OSD_THROTTLE,
        OSD_GPS_SPEED,
        OSD_NUMERICAL_VARIO,
        OSD_CRAFT_NAME,
        OSD_POWER,
        OSD_GPS_LAT,
        OSD_GPS_LON,
        OSD_ELEMENT_COUNT
    };

    struct OsdElementInfo
    {
        const char* name;    // osd_<name>_pos
        uint16_t def;        // умолчание (с битом «показан» или без)
        const char* text;    // что это, по-русски
    };

    inline const OsdElementInfo& osdElementInfo(uint8_t e)
    {
        static const OsdElementInfo table[OSD_ELEMENT_COUNT] = {
            { "link_quality", osdPos(1, 0), "качество связи (LQ), %" },
            { "flymode", osdPos(13, 0), "режим полёта (ANGL, ACRO, RESC, !FS!)" },
            { "gps_sats", osdPos(24, 0), "спутники GPS" },
            { "vtx_channel", osdPos(23, 1), "мощность видеопередатчика (PIT — на земле)" },
            { "crosshairs", osdPos(14, 6), "прицел" },
            { "warnings", osdPos(9, 9), "главное предупреждение (по центру поля в 12 знаков)" },
            { "current", osdPos(1, 10), "ток, А" },
            { "mah_drawn", osdPos(8, 10), "израсходовано, мА·ч" },
            { "home_dir", osdPos(1, 11), "стрелка домой" },
            { "home_dist", osdPos(2, 11), "расстояние домой" },
            { "altitude", osdPos(23, 11), "высота от точки ARM" },
            { "vbat", osdPos(1, 12), "напряжение батареи" },
            { "avg_cell_voltage", osdPos(8, 12), "напряжение на банку" },
            { "tim_2", osdPos(22, 12), "время в полёте (таймер 2 BF)" },
            { "throttle", osdPos(1, 2, false), "газ, %" },
            { "gps_speed", osdPos(23, 2, false), "скорость по GPS, км/ч" },
            { "vario_num", osdPos(23, 10, false), "вертикальная скорость, м/с" },
            { "craft_name", osdPos(10, 12, false), "имя коптера (craft_name)" },
            { "power", osdPos(15, 10, false), "мощность, Вт" },
            { "gps_lat", osdPos(1, 3, false), "широта — найти упавший" },
            { "gps_lon", osdPos(1, 4, false), "долгота" },
        };
        return table[e < OSD_ELEMENT_COUNT ? e : 0];
    }

    // Строки статистики после полёта — по именам BF (osd_stat_<name>).
    enum OsdStat : uint8_t
    {
        OSD_STAT_TIM_2,
        OSD_STAT_MAX_ALT,
        OSD_STAT_MAX_SPD,
        OSD_STAT_MAX_DIST,
        OSD_STAT_MIN_BATT,
        OSD_STAT_END_BATT,
        OSD_STAT_USED_MAH,
        OSD_STAT_MAX_CURR,
        OSD_STAT_MIN_LINK_QUALITY,
        OSD_STAT_COUNT
    };

    struct OsdNamedFlag
    {
        const char* name;
        bool def;
        const char* text;
    };

    inline const OsdNamedFlag& osdStatInfo(uint8_t s)
    {
        static const OsdNamedFlag table[OSD_STAT_COUNT] = {
            { "tim_2", true, "время в полёте" },
            { "max_alt", true, "наибольшая высота" },
            { "max_spd", true, "наибольшая скорость по GPS" },
            { "max_dist", true, "дальше всего от дома" },
            { "min_batt", true, "наименьшее напряжение батареи" },
            { "endbatt", false, "напряжение после посадки" },
            { "used_mah", true, "израсходовано мА·ч" },
            { "max_curr", true, "наибольший ток" },
            { "min_link_quality", true, "наименьшее качество связи" },
        };
        return table[s < OSD_STAT_COUNT ? s : 0];
    }

    // Какие предупреждения показывать — по именам BF (osd_warn_<name>).
    enum OsdWarn : uint8_t
    {
        OSD_WARN_ARMING_DISABLE,
        OSD_WARN_BATT_NOT_FULL,
        OSD_WARN_BATT_WARNING,
        OSD_WARN_BATT_CRITICAL,
        OSD_WARN_FAIL_SAFE,
        OSD_WARN_LINK_QUALITY,
        OSD_WARN_COUNT
    };

    inline const OsdNamedFlag& osdWarnInfo(uint8_t w)
    {
        static const OsdNamedFlag table[OSD_WARN_COUNT] = {
            { "arming_disable", true, "без ARM — почему не армится (код BF)" },
            { "batt_not_full", true, "подключили не полную батарею (BATT < FULL)" },
            { "batt_warning", true, "LOW BATTERY" },
            { "batt_critical", true, "BATT CRITICAL" },
            { "fail_safe", true, "FAILSAFE и RXLOSS" },
            { "link_quality", true, "связь хуже osd_link_quality_alarm (LINK QUALITY)" },
        };
        return table[w < OSD_WARN_COUNT ? w : 0];
    }

    // Что и где показывать (из настроек; умолчание — как в таблицах выше).
    struct OsdLayout
    {
        uint16_t pos[OSD_ELEMENT_COUNT];
        uint16_t stats = 0;            // биты OsdStat
        uint8_t warnings = 0;          // биты OsdWarn
        uint16_t altAlarmM = 100;      // osd_alt_alarm: выше — высота мигает (0 — нет)
        uint16_t capAlarmMah = 2200;   // osd_cap_alarm: больше — мА·ч мигают (0 — нет)
        uint8_t linkQualityAlarm = 80; // osd_link_quality_alarm: ниже — LQ мигает и предупреждение

        OsdLayout()
        {
            for (uint8_t e = 0; e < OSD_ELEMENT_COUNT; ++e) pos[e] = osdElementInfo(e).def;
            for (uint8_t s = 0; s < OSD_STAT_COUNT; ++s)
            {
                if (osdStatInfo(s).def) stats = static_cast<uint16_t>(stats | (1u << s));
            }
            for (uint8_t w = 0; w < OSD_WARN_COUNT; ++w)
            {
                if (osdWarnInfo(w).def) warnings = static_cast<uint8_t>(warnings | (1u << w));
            }
        }

        bool stat(OsdStat s) const { return (stats >> s) & 1u; }
        bool warn(OsdWarn w) const { return (warnings >> w) & 1u; }
    };

    // Что показать.
    struct OsdData
    {
        bool armed = false;
        bool failsafe = false;
        const char* mode = "ACRO";          // ACRO, ANGL, HOR, ALTH
        bool rescue = false;                // GPS Rescue летит домой — режим RESC, как в BF
        bool rescueAbort = false;           // возврат невозможен — садится на месте
        int16_t linkQuality = -1;           // %, −1 — неизвестно
        bool linkLost = false;
        float volts = 0.0f, cellVolts = 0.0f;
        bool hasCurrent = false;
        float amps = 0.0f, mahDrawn = 0.0f;
        uint8_t batteryState = 1;           // BatteryState: 0 нет, 1 OK, 2 WARNING, 3 CRITICAL
        bool batteryNotFull = false;        // подключили не полную (vbat_full_cell_voltage)
        uint32_t flightSeconds = 0;
        float altitudeM = 0.0f;
        float climbMs = 0.0f;
        float throttle = 0.0f;              // 0…1
        bool gpsPresent = false;
        bool gpsFix = false;
        uint8_t satellites = 0;
        float speedKmh = 0.0f;
        double latitude = 0.0, longitude = 0.0;
        bool home = false;
        float homeDistanceM = 0.0f, homeRelativeDeg = 0.0f;
        uint16_t vtxMw = 0;                 // 0 — нет VTX
        bool vtxPit = false;
        const char* armBlock = nullptr;     // код BF, почему не армится (только без ARM)
        const char* craftName = nullptr;
        bool blink = true;                  // фаза мигания (2 Гц): false — тревожные значения гаснут
        bool exitStats = false;             // газ или тангаж вверх — убрать статистику
    };

    // Статистика полёта для экрана после DISARM.
    class OsdStats
    {
    public:
        static constexpr uint32_t SHOW_MS = 60000;      // не дольше минуты
        static constexpr uint32_t MIN_FLIGHT_S = 1;     // случайный ARM на столе — без статистики

        void update(const OsdData& d, uint32_t nowMs)
        {
            if (d.armed)
            {
                if (!wasArmed) reset();
                showing = false;
                flightSeconds = d.flightSeconds;
                maxAltM = fmaxf(maxAltM, d.altitudeM);
                if (d.gpsFix) maxSpeedKmh = fmaxf(maxSpeedKmh, d.speedKmh);
                if (d.home) maxDistM = fmaxf(maxDistM, d.homeDistanceM);
                if (d.batteryState != 0 && d.volts > 0.5f) minVolts = minVolts > 0.0f ? fminf(minVolts, d.volts) : d.volts;
                mah = d.mahDrawn;
                if (d.hasCurrent) maxAmps = fmaxf(maxAmps, d.amps);
                if (d.linkQuality >= 0) minLq = minLq < 0 ? d.linkQuality : (d.linkQuality < minLq ? d.linkQuality : minLq);
            }
            else if (wasArmed && flightSeconds >= MIN_FLIGHT_S)
            {
                showing = true;
                shownAtMs = nowMs;
            }
            if (!d.armed && showing)
            {
                endVolts = d.volts;
                if (d.exitStats || nowMs - shownAtMs > SHOW_MS) showing = false;
            }
            hasCurrent = hasCurrent || d.hasCurrent;
            wasArmed = d.armed;
        }

        bool isShowing() const { return showing; }

        uint32_t flightSeconds = 0;
        float maxAltM = 0.0f, maxSpeedKmh = 0.0f, maxDistM = 0.0f;
        float minVolts = 0.0f, endVolts = 0.0f;
        float mah = 0.0f, maxAmps = 0.0f;
        int16_t minLq = -1;
        bool hasCurrent = false;

    private:
        void reset()
        {
            const bool keepCurrent = hasCurrent;
            *this = OsdStats();
            hasCurrent = keepCurrent;
        }

        bool wasArmed = false;
        bool showing = false;
        uint32_t shownAtMs = 0;
    };

    class OsdScreen
    {
    public:
        static constexpr uint8_t COLS = 30;
        static constexpr uint8_t MAX_ROWS = 16;
        static constexpr uint8_t WARNING_FIELD = 12;   // поле предупреждений BF

        OsdScreen() { clear(); }

        void setRows(uint8_t n) { rows = n > MAX_ROWS ? MAX_ROWS : n; }
        uint8_t rowCount() const { return rows; }
        uint8_t at(uint8_t row, uint8_t col) const { return row < MAX_ROWS && col < COLS ? cells[row][col] : ' '; }

        void clear() { memset(cells, ' ', sizeof(cells)); }

        // Текст — заглавными (в шрифте только они).
        void print(uint8_t row, uint8_t col, const char* text)
        {
            for (; *text && col < COLS && row < rows; ++text, ++col)
            {
                cells[row][col] = static_cast<uint8_t>(toupper(static_cast<unsigned char>(*text)));
            }
        }

        void put(uint8_t row, uint8_t col, uint8_t symbol)
        {
            if (row < rows && col < COLS) cells[row][col] = symbol;
        }

        // Весь экран из данных: раскладка по умолчанию, без статистики.
        void compose(const OsdData& d)
        {
            static const OsdLayout layout;
            compose(d, layout, nullptr);
        }

        // Весь экран из данных по раскладке; статистика показывается — она вместо элементов.
        void compose(const OsdData& d, const OsdLayout& l, const OsdStats* stats)
        {
            clear();
            if (stats && stats->isShowing())
            {
                composeStats(*stats, l);
                return;
            }
            char s[32];
            for (uint8_t e = 0; e < OSD_ELEMENT_COUNT; ++e)
            {
                const uint16_t p = l.pos[e];
                if (!osdPosVisible(p)) continue;
                const uint8_t x = osdPosX(p), y = osdPosY(p);
                if (y >= rows || x >= COLS) continue;   // NTSC: строки 13–15 не видны
                drawElement(static_cast<OsdElement>(e), x, y, d, l, s, sizeof(s));
            }
        }

        // Самое важное, что сейчас не так (или nullptr). Все предупреждения включены.
        static const char* warningText(const OsdData& d)
        {
            static const OsdLayout all;
            return warningText(d, all);
        }

        static const char* warningText(const OsdData& d, const OsdLayout& l)
        {
            if (d.rescueAbort) return "RESCUE FAIL";
            if (l.warn(OSD_WARN_FAIL_SAFE) && d.failsafe) return "FAILSAFE";
            if (l.warn(OSD_WARN_FAIL_SAFE) && d.linkLost) return "RXLOSS";
            if (l.warn(OSD_WARN_BATT_CRITICAL) && d.batteryState == 3) return "BATT CRITICAL";
            if (l.warn(OSD_WARN_BATT_WARNING) && d.batteryState == 2) return "LOW BATTERY";
            if (l.warn(OSD_WARN_LINK_QUALITY) && d.armed && d.linkQuality >= 0 && l.linkQualityAlarm > 0 &&
                d.linkQuality < l.linkQualityAlarm)
            {
                return "LINK QUALITY";
            }
            if (l.warn(OSD_WARN_BATT_NOT_FULL) && !d.armed && d.batteryNotFull) return "BATT < FULL";
            if (l.warn(OSD_WARN_ARMING_DISABLE) && !d.armed && d.armBlock) return d.armBlock;
            if (!d.armed) return "DISARMED";
            return nullptr;
        }

    private:
        void drawElement(OsdElement e, uint8_t x, uint8_t y, const OsdData& d, const OsdLayout& l, char* s, size_t n)
        {
            switch (e)
            {
                case OSD_LINK_QUALITY:
                    if (d.linkQuality < 0) return;
                    if (!d.blink && l.linkQualityAlarm > 0 && d.linkQuality < l.linkQualityAlarm) return;
                    put(y, x, osdsym::LINK_QUALITY);
                    snprintf(s, n, "%d", d.linkQuality);
                    print(y, static_cast<uint8_t>(x + 1), s);
                    return;
                case OSD_FLYMODE:
                    print(y, x, d.rescue ? "RESC" : d.failsafe ? "!FS!" : d.mode);
                    return;
                case OSD_GPS_SATS:
                    if (!d.gpsPresent) return;
                    put(y, x, osdsym::SAT_L);
                    put(y, static_cast<uint8_t>(x + 1), osdsym::SAT_R);
                    snprintf(s, n, "%u", d.satellites);
                    print(y, static_cast<uint8_t>(x + 2), s);
                    return;
                case OSD_VTX_CHANNEL:
                    if (!d.vtxMw) return;
                    if (d.vtxPit) snprintf(s, n, "PIT");
                    else snprintf(s, n, "%uMW", d.vtxMw);
                    print(y, x, s);
                    return;
                case OSD_CROSSHAIRS:
                    put(y, x, osdsym::CROSSHAIR);
                    return;
                case OSD_WARNINGS:
                {
                    const char* w = warningText(d, l);
                    if (!w) return;
                    const int len = static_cast<int>(strlen(w));
                    const int col = x + (WARNING_FIELD > len ? (WARNING_FIELD - len) / 2 : 0);
                    print(y, static_cast<uint8_t>(col), w);
                    return;
                }
                case OSD_CURRENT_DRAW:
                    if (!d.hasCurrent) return;
                    snprintf(s, n, "%.1f", static_cast<double>(d.amps));
                    printWithSymbol(y, x, s, osdsym::AMP);
                    return;
                case OSD_MAH_DRAWN:
                    if (!d.hasCurrent) return;
                    if (!d.blink && l.capAlarmMah > 0 && d.mahDrawn >= l.capAlarmMah) return;
                    snprintf(s, n, "%d", static_cast<int>(d.mahDrawn));
                    printWithSymbol(y, x, s, osdsym::MAH);
                    return;
                case OSD_HOME_DIR:
                    if (d.home) put(y, x, osdArrow(d.homeRelativeDeg));
                    return;
                case OSD_HOME_DIST:
                    if (!d.home) return;
                    formatDistance(s, n, d.homeDistanceM);
                    print(y, x, s);
                    return;
                case OSD_ALTITUDE:
                    if (!d.blink && l.altAlarmM > 0 && d.altitudeM > l.altAlarmM) return;
                    put(y, x, osdsym::ALTITUDE);
                    snprintf(s, n, "%d", static_cast<int>(lroundf(d.altitudeM)));
                    printWithSymbol(y, static_cast<uint8_t>(x + 1), s, osdsym::METRE);
                    return;
                case OSD_MAIN_BATT_VOLTAGE:
                    if (d.batteryState == 0) return;
                    if (!d.blink && d.batteryState >= 2) return;
                    snprintf(s, n, "%.1f", static_cast<double>(d.volts));
                    printWithSymbol(y, x, s, osdsym::VOLT);
                    return;
                case OSD_AVG_CELL_VOLTAGE:
                    if (d.batteryState == 0) return;
                    snprintf(s, n, "%.2f", static_cast<double>(d.cellVolts));
                    printWithSymbol(y, x, s, osdsym::VOLT);
                    return;
                case OSD_FLYTIME:
                    put(y, x, osdsym::FLY_TIME);
                    formatTime(s, n, d.flightSeconds);
                    print(y, static_cast<uint8_t>(x + 1), s);
                    return;
                case OSD_THROTTLE:
                    put(y, x, osdsym::THROTTLE);
                    snprintf(s, n, "%d", static_cast<int>(lroundf(fmaxf(0.0f, fminf(1.0f, d.throttle)) * 100.0f)));
                    print(y, static_cast<uint8_t>(x + 1), s);
                    return;
                case OSD_GPS_SPEED:
                    if (!d.gpsFix) return;
                    put(y, x, osdsym::SPEED);
                    snprintf(s, n, "%d", static_cast<int>(lroundf(d.speedKmh)));
                    printWithSymbol(y, static_cast<uint8_t>(x + 1), s, osdsym::KPH);
                    return;
                case OSD_NUMERICAL_VARIO:
                    snprintf(s, n, "%+.1f", static_cast<double>(d.climbMs));
                    printWithSymbol(y, x, s, osdsym::MPS);
                    return;
                case OSD_CRAFT_NAME:
                    if (d.craftName && *d.craftName) print(y, x, d.craftName);
                    return;
                case OSD_POWER:
                    if (!d.hasCurrent) return;
                    snprintf(s, n, "%d", static_cast<int>(lroundf(d.volts * d.amps)));
                    printWithSymbol(y, x, s, osdsym::WATT);
                    return;
                case OSD_GPS_LAT:
                case OSD_GPS_LON:
                    if (!d.gpsFix) return;
                    put(y, x, e == OSD_GPS_LAT ? osdsym::LAT : osdsym::LON);
                    snprintf(s, n, "%.7f", e == OSD_GPS_LAT ? d.latitude : d.longitude);
                    print(y, static_cast<uint8_t>(x + 1), s);
                    return;
                case OSD_ELEMENT_COUNT:
                default:
                    return;
            }
        }

        void composeStats(const OsdStats& st, const OsdLayout& l)
        {
            char s[24];
            uint8_t row = rows > 13 ? 2 : 1;
            print(row++, 8, "--- STATS ---");
            auto line = [&](const char* label, const char* value) {
                if (row >= rows) return;
                print(row, 2, label);
                print(row, 18, value);
                ++row;
            };
            if (l.stat(OSD_STAT_TIM_2))
            {
                formatTime(s, sizeof(s), st.flightSeconds);
                line("FLY TIME", s);
            }
            if (l.stat(OSD_STAT_MAX_ALT))
            {
                snprintf(s, sizeof(s), "%dM", static_cast<int>(lroundf(st.maxAltM)));
                line("MAX ALTITUDE", s);
            }
            if (l.stat(OSD_STAT_MAX_SPD) && st.maxSpeedKmh > 0.0f)
            {
                snprintf(s, sizeof(s), "%dKM/H", static_cast<int>(lroundf(st.maxSpeedKmh)));
                line("MAX SPEED", s);
            }
            if (l.stat(OSD_STAT_MAX_DIST) && st.maxDistM > 0.0f)
            {
                formatDistance(s, sizeof(s), st.maxDistM);
                line("MAX DISTANCE", s);
            }
            if (l.stat(OSD_STAT_MIN_BATT) && st.minVolts > 0.0f)
            {
                snprintf(s, sizeof(s), "%.1fV", static_cast<double>(st.minVolts));
                line("MIN BATTERY", s);
            }
            if (l.stat(OSD_STAT_END_BATT) && st.endVolts > 0.5f)
            {
                snprintf(s, sizeof(s), "%.1fV", static_cast<double>(st.endVolts));
                line("END BATTERY", s);
            }
            if (l.stat(OSD_STAT_USED_MAH) && st.hasCurrent)
            {
                snprintf(s, sizeof(s), "%d", static_cast<int>(st.mah));
                line("USED MAH", s);
            }
            if (l.stat(OSD_STAT_MAX_CURR) && st.hasCurrent)
            {
                snprintf(s, sizeof(s), "%.1fA", static_cast<double>(st.maxAmps));
                line("MAX CURRENT", s);
            }
            if (l.stat(OSD_STAT_MIN_LINK_QUALITY) && st.minLq >= 0)
            {
                snprintf(s, sizeof(s), "%d", st.minLq);
                line("MIN LINK", s);
            }
        }

        void printWithSymbol(uint8_t row, uint8_t col, const char* text, uint8_t symbol)
        {
            print(row, col, text);
            put(row, static_cast<uint8_t>(col + strlen(text)), symbol);
        }

        static void formatTime(char* out, size_t size, uint32_t seconds)
        {
            snprintf(out, size, "%02lu:%02lu", static_cast<unsigned long>(seconds / 60 % 100),
                     static_cast<unsigned long>(seconds % 60));
        }

        static void formatDistance(char* out, size_t size, float m)
        {
            if (m < 1000.0f) snprintf(out, size, "%dM", static_cast<int>(lroundf(m)));
            else snprintf(out, size, "%.1fKM", static_cast<double>(m / 1000.0f));
        }

        uint8_t cells[MAX_ROWS][COLS];
        uint8_t rows = MAX_ROWS;
    };
}
