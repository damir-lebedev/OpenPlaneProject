#pragma once

// Поле и запретные зоны геозабора (P44 б, своё; в BF нет, в INAV 8 похожее —
// geozone, синтаксис CLI у нас свой). Многоугольники по точкам с карты:
//   FIELD — поле: летать только внутри (полей несколько — внутри любого);
//   NOFLY — запретная зона внутри поля (дом соседа, дорога, зрители).
// Дом (точка ARM) не обязательно в середине; ARM — только внутри поля и вне
// запретных зон. Хранится во флеше рядом с настройками (ключ "fence"), в CLI —
// строками geozone, в diff/dump — тоже ими:
//   geozone 0 FIELD                    — зона 0 — поле (точки не трогает)
//   geozone 0 3 56.3268710 44.0059870  — точка 3 зоны 0 (номер ≤ числа точек)
//   geozone 0 clear / geozone clear    — убрать зону / все
// Координаты — градусы с 7 знаками (1 см), внутри — целые 1e-7 градуса.
// Геометрия (внутри ли, до ближайшей стороны) — GeoPolygon ниже, в метрах
// от первой точки; считает Geofence.h.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace copter
{
    enum class GeoZoneType : uint8_t { NONE, FIELD, NOFLY };

    inline const char* geoZoneTypeName(GeoZoneType t)
    {
        return t == GeoZoneType::FIELD ? "FIELD" : t == GeoZoneType::NOFLY ? "NOFLY" : "NONE";
    }

    struct GeoPoint
    {
        int32_t latE7 = 0, lonE7 = 0;
    };

    class GeoZoneTable
    {
    public:
        static constexpr uint8_t MAX_ZONES = 8;
        static constexpr uint8_t MAX_POINTS = 16;
        static constexpr float MAX_SIZE_M = 20000.0f;   // точки дальше друг от друга — ошибка ввода
        static constexpr uint8_t STORE_VERSION = 1;
        static constexpr size_t STORE_MAX = 2 + MAX_ZONES * (2 + MAX_POINTS * 8);

        struct Zone
        {
            GeoZoneType type = GeoZoneType::NONE;
            uint8_t count = 0;
            GeoPoint pts[MAX_POINTS];
        };

        void reset()
        {
            for (Zone& z : zones) z = Zone();
        }
        bool isDefault() const
        {
            for (const Zone& z : zones)
            {
                if (z.type != GeoZoneType::NONE || z.count) return false;
            }
            return true;
        }
        const Zone& zone(uint8_t i) const { return zones[i < MAX_ZONES ? i : 0]; }
        bool used(uint8_t i) const { return i < MAX_ZONES && zones[i].type != GeoZoneType::NONE; }

        // Зона годится: тип есть, ≥ 3 точек, стороны не пересекаются, не больше 20 км.
        // nullptr — годится, иначе — почему нет.
        const char* problem(uint8_t i) const
        {
            if (!used(i)) return "зоны нет";
            const Zone& z = zones[i];
            if (z.count < 3) return "меньше 3 точек";
            float n[MAX_POINTS], e[MAX_POINTS];
            toMetres(z, n, e);
            for (uint8_t a = 0; a < z.count; ++a)
            {
                if (fabsf(n[a]) > MAX_SIZE_M || fabsf(e[a]) > MAX_SIZE_M) return "точки дальше 20 км друг от друга";
            }
            for (uint8_t a = 0; a < z.count; ++a)
            {
                const uint8_t a2 = static_cast<uint8_t>((a + 1) % z.count);
                for (uint8_t b = static_cast<uint8_t>(a + 1); b < z.count; ++b)
                {
                    const uint8_t b2 = static_cast<uint8_t>((b + 1) % z.count);
                    if (b == a2 || b2 == a) continue;   // соседние стороны
                    if (segmentsCross(n[a], e[a], n[a2], e[a2], n[b], e[b], n[b2], e[b2])) return "стороны пересекаются";
                }
            }
            return nullptr;
        }
        // Есть ли хоть одна годная зона (забор многоугольником включится).
        bool anyValid() const
        {
            for (uint8_t i = 0; i < MAX_ZONES; ++i)
            {
                if (!problem(i)) return true;
            }
            return false;
        }

        // Точки зоны в метрах от её первой точки: на север и на восток.
        static void toMetres(const Zone& z, float* north, float* east)
        {
            for (uint8_t i = 0; i < z.count; ++i) offsetM(z.pts[0], z.pts[i], north[i], east[i]);
        }
        // Смещение b от a, м (плоская Земля: до 20 км — ошибка сантиметры на метр).
        static void offsetM(const GeoPoint& a, const GeoPoint& b, float& north, float& east)
        {
            const double k = 1e-7 * 3.14159265358979323846 / 180.0 * 6371000.0;
            north = static_cast<float>((b.latE7 - a.latE7) * k);
            east = static_cast<float>((static_cast<double>(b.lonE7) - a.lonE7) * k * cos(a.latE7 * 1e-7 * 3.14159265358979323846 / 180.0));
        }
        static void offsetM(const GeoPoint& a, double lat, double lon, float& north, float& east)
        {
            const double toRad = 3.14159265358979323846 / 180.0;
            north = static_cast<float>((lat - a.latE7 * 1e-7) * toRad * 6371000.0);
            east = static_cast<float>((lon - a.lonE7 * 1e-7) * toRad * 6371000.0 * cos(a.latE7 * 1e-7 * toRad));
        }

        // ---- CLI ----

        // Строка после «geozone». false — ошибка (текст в error).
        bool parse(const char* args, char* error, size_t n)
        {
            while (*args == ' ') ++args;
            if (startsWord(args, "clear"))
            {
                reset();
                return true;
            }
            char* end = nullptr;
            const long zi = strtol(args, &end, 10);
            if (end == args || zi < 0 || zi >= MAX_ZONES) return fail(error, n, "номер зоны 0…7");
            Zone& z = zones[zi];
            const char* rest = skip(end);
            if (startsWord(rest, "clear"))
            {
                z = Zone();
                return true;
            }
            if (startsWord(rest, "FIELD") || startsWord(rest, "field"))
            {
                z.type = GeoZoneType::FIELD;
                return true;
            }
            if (startsWord(rest, "NOFLY") || startsWord(rest, "nofly"))
            {
                z.type = GeoZoneType::NOFLY;
                return true;
            }
            const long pi = strtol(rest, &end, 10);
            if (end == rest) return fail(error, n, "geozone <зона> FIELD|NOFLY|clear или geozone <зона> <точка> <широта> <долгота>");
            if (pi < 0 || pi >= MAX_POINTS) return fail(error, n, "номер точки 0…15");
            if (pi > z.count) return fail(error, n, "точки — по порядку: следующая — номер, равный числу точек");
            if (z.type == GeoZoneType::NONE) return fail(error, n, "сначала тип зоны: geozone <зона> FIELD или NOFLY");
            GeoPoint p;
            rest = skip(end);
            if (!parseDegrees(rest, 90, p.latE7, &end)) return fail(error, n, "широта: -90…90, градусы с точкой");
            rest = skip(end);
            if (!parseDegrees(rest, 180, p.lonE7, &end)) return fail(error, n, "долгота: -180…180, градусы с точкой");
            z.pts[pi] = p;
            if (pi == z.count) ++z.count;
            return true;
        }

        // Строки зоны для diff/dump: тип, затем точки. Вызывает out(line) на каждую.
        template <typename Out>
        void formatZone(uint8_t i, Out&& out) const
        {
            if (!used(i)) return;
            const Zone& z = zones[i];
            char line[64];
            snprintf(line, sizeof(line), "geozone %u %s", i, geoZoneTypeName(z.type));
            out(line);
            for (uint8_t k = 0; k < z.count; ++k)
            {
                char la[16], lo[16];
                formatDegrees(z.pts[k].latE7, la, sizeof(la));
                formatDegrees(z.pts[k].lonE7, lo, sizeof(lo));
                snprintf(line, sizeof(line), "geozone %u %u %s %s", i, k, la, lo);
                out(line);
            }
        }

        // Целые 1e-7 градуса → «-56.3268710» (без printf %f — его нет в newlib-nano).
        static void formatDegrees(int32_t e7, char* out, size_t n)
        {
            const uint32_t a = e7 < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(e7)) : static_cast<uint32_t>(e7);
            snprintf(out, n, "%s%lu.%07lu", e7 < 0 ? "-" : "", static_cast<unsigned long>(a / 10000000u),
                     static_cast<unsigned long>(a % 10000000u));
        }

        // ---- флеш ----

        size_t serialize(uint8_t* out, size_t capacity) const
        {
            if (capacity < 2) return 0;
            size_t at = 0;
            out[at++] = STORE_VERSION;
            uint8_t last = 0;
            for (uint8_t i = 0; i < MAX_ZONES; ++i)
            {
                if (used(i) || zones[i].count) last = static_cast<uint8_t>(i + 1);
            }
            out[at++] = last;
            for (uint8_t i = 0; i < last; ++i)
            {
                const Zone& z = zones[i];
                if (at + 2 + z.count * 8u > capacity) return 0;
                out[at++] = static_cast<uint8_t>(z.type);
                out[at++] = z.count;
                for (uint8_t k = 0; k < z.count; ++k)
                {
                    put32(out + at, z.pts[k].latE7);
                    put32(out + at + 4, z.pts[k].lonE7);
                    at += 8;
                }
            }
            return at;
        }

        bool deserialize(const uint8_t* data, size_t length)
        {
            reset();
            if (!data || length < 2 || data[0] != STORE_VERSION || data[1] > MAX_ZONES) return false;
            size_t at = 2;
            for (uint8_t i = 0; i < data[1]; ++i)
            {
                if (at + 2 > length) return fail();
                const uint8_t type = data[at], count = data[at + 1];
                at += 2;
                if (type > static_cast<uint8_t>(GeoZoneType::NOFLY) || count > MAX_POINTS || at + count * 8u > length) return fail();
                Zone& z = zones[i];
                z.type = static_cast<GeoZoneType>(type);
                z.count = count;
                for (uint8_t k = 0; k < count; ++k)
                {
                    z.pts[k].latE7 = get32(data + at);
                    z.pts[k].lonE7 = get32(data + at + 4);
                    at += 8;
                }
            }
            return true;
        }

    private:
        bool fail()
        {
            reset();
            return false;
        }
        static bool fail(char* error, size_t n, const char* text)
        {
            if (error && n) snprintf(error, n, "%s", text);
            return false;
        }
        static const char* skip(const char* s)
        {
            while (*s == ' ' || *s == '\t') ++s;
            return s;
        }
        static bool startsWord(const char* s, const char* w)
        {
            const size_t n = strlen(w);
            return strncmp(s, w, n) == 0 && (s[n] == '\0' || s[n] == ' ' || s[n] == '\r' || s[n] == '\n');
        }
        // «56.3268710» → 563268710; без strtod (точно до 1e-7, без double-округления).
        static bool parseDegrees(const char* s, int32_t limit, int32_t& outE7, char** end)
        {
            const char* p = s;
            bool neg = false;
            if (*p == '-' || *p == '+') neg = *p++ == '-';
            if (*p < '0' || *p > '9') return false;
            int64_t whole = 0;
            while (*p >= '0' && *p <= '9' && whole <= 1000) whole = whole * 10 + (*p++ - '0');
            int64_t frac = 0;
            int digits = 0;
            if (*p == '.')
            {
                ++p;
                while (*p >= '0' && *p <= '9')
                {
                    if (digits < 7)
                    {
                        frac = frac * 10 + (*p - '0');
                        ++digits;
                    }
                    else if (digits == 7)
                    {
                        if (*p >= '5') ++frac;   // округление 8-го знака
                        ++digits;
                    }
                    ++p;
                }
            }
            if (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') return false;
            while (digits < 7)
            {
                frac *= 10;
                ++digits;
            }
            const int64_t v = whole * 10000000 + frac;
            if (v > static_cast<int64_t>(limit) * 10000000) return false;
            outE7 = static_cast<int32_t>(neg ? -v : v);
            *end = const_cast<char*>(p);
            return true;
        }
        static bool segmentsCross(float an, float ae, float bn, float be, float cn, float ce, float dn, float de)
        {
            auto cross = [](float x1, float y1, float x2, float y2, float x3, float y3) {
                return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            };
            const float d1 = cross(cn, ce, dn, de, an, ae), d2 = cross(cn, ce, dn, de, bn, be);
            const float d3 = cross(an, ae, bn, be, cn, ce), d4 = cross(an, ae, bn, be, dn, de);
            return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
        }
        static void put32(uint8_t* p, int32_t v)
        {
            const uint32_t u = static_cast<uint32_t>(v);
            p[0] = static_cast<uint8_t>(u);
            p[1] = static_cast<uint8_t>(u >> 8);
            p[2] = static_cast<uint8_t>(u >> 16);
            p[3] = static_cast<uint8_t>(u >> 24);
        }
        static int32_t get32(const uint8_t* p)
        {
            return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 |
                                        static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24);
        }

        Zone zones[MAX_ZONES];
    };

    // Ближайшая сторона многоугольника к точке (метры, север/восток).
    struct PolygonNearest
    {
        bool inside = false;
        float distM = 1e9f;         // до ближайшей стороны
        float towardN = 0.0f, towardE = 0.0f;   // единичный вектор от точки к ближайшей точке стороны
    };

    inline PolygonNearest polygonNearest(const float* n, const float* e, uint8_t count, float pn, float pe)
    {
        PolygonNearest r;
        float bestN = pn, bestE = pe;
        for (uint8_t i = 0, j = static_cast<uint8_t>(count - 1); i < count; j = i++)
        {
            // внутри — луч на восток (чётность пересечений)
            if ((n[i] > pn) != (n[j] > pn) && pe < (e[j] - e[i]) * (pn - n[i]) / (n[j] - n[i]) + e[i]) r.inside = !r.inside;
            const float sn = n[i] - n[j], se = e[i] - e[j];
            const float len2 = sn * sn + se * se;
            float t = len2 > 1e-6f ? ((pn - n[j]) * sn + (pe - e[j]) * se) / len2 : 0.0f;
            t = fmaxf(0.0f, fminf(1.0f, t));
            const float qn = n[j] + t * sn, qe = e[j] + t * se;
            const float d = sqrtf((qn - pn) * (qn - pn) + (qe - pe) * (qe - pe));
            if (d < r.distM)
            {
                r.distM = d;
                bestN = qn;
                bestE = qe;
            }
        }
        if (r.distM > 1e-3f)
        {
            r.towardN = (bestN - pn) / r.distM;
            r.towardE = (bestE - pe) / r.distM;
        }
        return r;
    }

    // Расстояние от точки до отрезка (для радара в очках).
    inline float segmentDistance(float px, float py, float ax, float ay, float bx, float by)
    {
        const float sx = bx - ax, sy = by - ay;
        const float len2 = sx * sx + sy * sy;
        float t = len2 > 1e-6f ? ((px - ax) * sx + (py - ay) * sy) / len2 : 0.0f;
        t = fmaxf(0.0f, fminf(1.0f, t));
        const float dx = ax + t * sx - px, dy = ay + t * sy - py;
        return sqrtf(dx * dx + dy * dy);
    }
}
