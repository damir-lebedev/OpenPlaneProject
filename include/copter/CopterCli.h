#pragma once
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "copter/CopterController.h"
#include "copter/Params.h"
#include "storage/KeyValueStore.h"

// ============================================================
// CLI коптера — строковый режим консоли, как в Betaflight: вход по '#',
// дальше строки:
//
//   set                    — все настройки (выбранных профилей)
//   set имя = значение     — задать (в ОЗУ; работать начнёт после save)
//   get часть_имени        — значение, границы, умолчание, что это
//   diff [all]             — только изменённое; all — все профили
//   dump [all]             — всё
//   profile [0-3]          — PID-профиль; rateprofile [0-3] — профиль расходов
//                            (переключаются сразу, на ходу)
//   save                   — во флеш, настройка пересчитана, выход из CLI
//   defaults [nosave]      — всё по умолчанию (и сохранить)
//   status, version, help, exit (несохранённое отменяется)
//
// Вывод diff и dump — в формате BF (`set имя = значение`, `profile N`),
// и обратно: diff из Betaflight можно вставить целиком — строки
// комментариев, `batch start/end` пропускаются, известные настройки
// задаются, остальное — строкой ###ОШИБКА### и дальше. Пока CLI открыт,
// ARM запрещён (main_copter.cpp), войти в CLI заармленным нельзя.
//
// Без Arduino: вывод — через CliPort (консоль платы, тест на ПК).
// ============================================================

namespace copter
{
    class CliPort
    {
    public:
        virtual ~CliPort() = default;
        virtual void write(const char* text) = 0;
        virtual void printStatus() {}   // команда status — как 's' в консоли
    };

    class CopterCli
    {
    public:
        static constexpr size_t LINE_MAX = 160;

        CopterCli(CopterParams& paramsRef, CopterController& controller, CliPort& portRef)
            : params(paramsRef), ctl(controller), port(portRef)
        {
        }

        void setStore(KeyValueStore* store) { kv = store; }
        void setVersion(const char* text) { version = text; }

        bool active() const { return open; }
        bool hasUnsaved() const { return dirty; }

        // '#' в консоли. false — заармлен.
        bool enter()
        {
            if (ctl.isArmed())
            {
                print("CLI — только без ARM.\r\n");
                return false;
            }
            open = true;
            dirty = false;
            lineLength = 0;
            overflow = false;
            print("\r\nCLI коптера, как в Betaflight: help — команды, exit — выход. ARM запрещён, пока CLI открыт.\r\n# ");
            return true;
        }

        // Символ из порта: эхо, строка — по Enter.
        void feed(char c)
        {
            if (!open) return;
            if (c == '\r' || c == '\n')
            {
                if (c == '\n' && lastWasCr)
                {
                    lastWasCr = false;
                    return;   // \r\n — одна строка
                }
                lastWasCr = c == '\r';
                print("\r\n");
                line[lineLength] = '\0';
                if (overflow) printError("строка длиннее %u символов", static_cast<unsigned>(LINE_MAX - 1));
                else execute(line);
                lineLength = 0;
                overflow = false;
                if (open) print("# ");
                return;
            }
            lastWasCr = false;
            if (c == '\b' || c == 0x7F)
            {
                if (lineLength > 0)
                {
                    --lineLength;
                    print("\b \b");
                }
                return;
            }
            if (static_cast<unsigned char>(c) < 0x20 && c != '\t') return;
            if (lineLength + 1 >= LINE_MAX)
            {
                overflow = true;
                return;
            }
            line[lineLength++] = c;
            const char echo[2] = { c, '\0' };
            print(echo);
        }

        // Одна команда (без перевода строки).
        void execute(const char* text)
        {
            char buf[LINE_MAX];
            snprintf(buf, sizeof(buf), "%s", text ? text : "");
            char* cmd = skipSpaces(buf);
            trimRight(cmd);
            if (*cmd == '\0' || *cmd == '#') return;   // пусто, комментарий из diff

            char* args = cmd;
            while (*args && !isspace(static_cast<unsigned char>(*args))) ++args;
            if (*args) *args++ = '\0';
            args = skipSpaces(args);

            if (is(cmd, "set")) cmdSet(args);
            else if (is(cmd, "get")) cmdGet(args);
            else if (is(cmd, "diff")) cmdDump(args, true);
            else if (is(cmd, "dump")) cmdDump(args, false);
            else if (is(cmd, "profile")) cmdProfile(args, false);
            else if (is(cmd, "rateprofile")) cmdProfile(args, true);
            else if (is(cmd, "save")) cmdSave();
            else if (is(cmd, "defaults")) cmdDefaults(args);
            else if (is(cmd, "exit")) cmdExit();
            else if (is(cmd, "status")) port.printStatus();
            else if (is(cmd, "version")) print("# %s\r\n", version);
            else if (is(cmd, "help")) cmdHelp();
            else if (is(cmd, "batch")) return;   // batch start / end из diff BF
            else printError("нет команды %s (help — список)", cmd);
        }

    private:
        void cmdHelp()
        {
            print("set [имя = значение]  — без аргументов: все настройки\r\n");
            print("get часть_имени       — значение, границы, умолчание\r\n");
            print("diff [all] · dump [all] — изменённое · всё (all — все профили)\r\n");
            print("profile [0-3] · rateprofile [0-3] — профиль PID · расходов\r\n");
            print("save — сохранить и пересчитать настройку · defaults [nosave] — всё по умолчанию\r\n");
            print("status · version · exit (несохранённое отменяется)\r\n");
        }

        void cmdSet(char* args)
        {
            if (*args == '\0')
            {
                for (uint8_t i = 0; i < PARAM_COUNT; ++i) printValue(static_cast<ParamId>(i));
                return;
            }
            char* eq = strchr(args, '=');
            if (!eq)
            {
                printError("set имя = значение");
                return;
            }
            *eq = '\0';
            trimRight(args);
            const char* valueText = skipSpaces(eq + 1);
            const int found = findParam(args);
            if (found < 0)
            {
                printError("нет настройки %s", args);
                return;
            }
            const ParamId id = static_cast<ParamId>(found);
            char error[96];
            if (!params.parse(id, params.activeSlot(id), valueText, error, sizeof(error)))
            {
                printError("%s: %s", paramInfo(id).name, error);
                return;
            }
            dirty = true;
            printValue(id);
        }

        void cmdGet(const char* args)
        {
            bool any = false;
            for (uint8_t i = 0; i < PARAM_COUNT; ++i)
            {
                const ParamId id = static_cast<ParamId>(i);
                const ParamInfo& info = paramInfo(id);
                if (*args && !strstr(info.name, args)) continue;
                any = true;
                printValue(id);
                char def[16];
                params.formatDefault(id, def, sizeof(def));
                if (info.type == ParamType::ENUM)
                {
                    print("Допустимо:");
                    for (uint8_t k = 0; k < info.optionCount; ++k) print("%s %s", k ? "," : "", info.options[k]);
                    print("\r\nПо умолчанию: %s\r\n", def);
                }
                else if (info.type == ParamType::STRING)
                {
                    print("Строка до %d символов\r\n", static_cast<int>(info.max));
                }
                else
                {
                    char lo[16], hi[16];
                    CopterParams::formatNumber(info, info.min, lo, sizeof(lo));
                    CopterParams::formatNumber(info, info.max, hi, sizeof(hi));
                    print("Допустимо: %s - %s%s%s\r\n", lo, hi, *info.units ? ", " : "", info.units);
                    print("По умолчанию: %s%s\r\n", def, defaultOrigin(info));
                }
                print("# %s%s\r\n", info.text, scopeNote(info.scope));
            }
            if (!any) printError("нет настройки, содержащей «%s»", args);
        }

        // diff/dump: общие, затем профили; all — все, иначе выбранные.
        void cmdDump(const char* args, bool onlyChanged)
        {
            const bool all = is(args, "all");
            print("# %s%s\r\n\r\n# version\r\n# %s\r\n", onlyChanged ? "diff" : "dump", all ? " all" : "", version);
            print("\r\n# master\r\n");
            dumpScope(ParamScope::MASTER, 0, onlyChanged);
            const uint8_t pidActive = params.pidProfile(), rateActive = params.rateProfile();
            for (uint8_t n = 0; n < CopterParams::PROFILES; ++n)
            {
                if (!all && n != pidActive) continue;
                print("\r\nprofile %u\r\n\r\n# profile %u\r\n", n, n);
                dumpScope(ParamScope::PID_PROFILE, n, onlyChanged);
            }
            for (uint8_t n = 0; n < CopterParams::PROFILES; ++n)
            {
                if (!all && n != rateActive) continue;
                print("\r\nrateprofile %u\r\n\r\n# rateprofile %u\r\n", n, n);
                dumpScope(ParamScope::RATE_PROFILE, n, onlyChanged);
            }
            if (all)
            {
                print("\r\n# выбранные профили\r\nprofile %u\r\nrateprofile %u\r\n", pidActive, rateActive);
            }
            print("\r\n");
        }

        void dumpScope(ParamScope scope, uint8_t slot, bool onlyChanged)
        {
            for (uint8_t i = 0; i < PARAM_COUNT; ++i)
            {
                const ParamId id = static_cast<ParamId>(i);
                if (paramInfo(id).scope != scope) continue;
                if (onlyChanged && !changed(id, slot)) continue;
                char value[48];
                params.format(id, slot, value, sizeof(value));
                print("set %s = %s\r\n", paramInfo(id).name, value);
            }
        }

        bool changed(ParamId id, uint8_t slot) const
        {
            if (!params.isSet(id, slot)) return false;
            return paramInfo(id).type == ParamType::STRING || params.valueIn(id, slot) != params.defaultValue(id);
        }

        void cmdProfile(const char* args, bool rates)
        {
            const char* what = rates ? "rateprofile" : "profile";
            if (*args)
            {
                char* end = nullptr;
                const long n = strtol(args, &end, 10);
                const bool ok = end != args && *skipSpaces(end) == '\0' && n >= 0 &&
                                (rates ? params.selectRateProfile(static_cast<uint8_t>(n))
                                       : params.selectPidProfile(static_cast<uint8_t>(n)));
                if (!ok)
                {
                    printError("%s 0-%u", what, CopterParams::PROFILES - 1);
                    return;
                }
                dirty = true;
                ctl.applyProfile(params.tune());   // сразу, на ходу
            }
            const uint8_t n = rates ? params.rateProfile() : params.pidProfile();
            const char* name = params.text(rates ? PARAM_RATEPROFILE_NAME : PARAM_PROFILE_NAME, n);
            print("%s %u%s%s\r\n", what, n, *name ? " # " : "", name);
        }

        void cmdSave()
        {
            bool stored = false;
            if (kv) stored = params.save(*kv) && kv->commit();
            apply();
            if (!kv) printError("некуда сохранить — настройка работает до перезагрузки");
            else if (!stored) printError("флеш настроек переполнен — не сохранено");
            else print("Сохранено, настройка пересчитана.\r\n");
            leave();
        }

        void cmdDefaults(const char* args)
        {
            params.resetAll();
            dirty = true;
            if (is(args, "nosave"))
            {
                print("Всё по умолчанию (не сохранено: save — сохранить).\r\n");
                return;
            }
            print("Всё по умолчанию.\r\n");
            cmdSave();
        }

        void cmdExit()
        {
            if (dirty)
            {
                if (kv) params.load(*kv);
                ctl.applyProfile(params.tune());
                print("Несохранённые изменения отменены.\r\n");
            }
            leave();
        }

        void leave()
        {
            open = false;
            dirty = false;
            print("Выход из CLI.\r\n");
        }

        // Новая настройка контроллеру — как после перезагрузки (только без ARM).
        void apply()
        {
            if (ctl.isArmed()) return;
            ctl.configure(params.tune(), ctl.getSettings());
            dirty = false;
        }

        void printValue(ParamId id)
        {
            char value[48];
            params.format(id, params.activeSlot(id), value, sizeof(value));
            print("%s = %s\r\n", paramInfo(id).name, value);
        }

        const char* defaultOrigin(const ParamInfo& info) const
        {
            if (!info.fromModel) return "";
            return params.source() == TuneSource::BETAFLIGHT ? " (Betaflight 4.5)" : " (из модели коптера)";
        }

        static const char* scopeNote(ParamScope s)
        {
            switch (s)
            {
                case ParamScope::PID_PROFILE: return " [PID-профиль]";
                case ParamScope::RATE_PROFILE: return " [профиль расходов]";
                case ParamScope::MASTER:
                default: return "";
            }
        }

        void print(const char* fmt, ...)
        {
            char buf[192];
            va_list ap;
            va_start(ap, fmt);
            vsnprintf(buf, sizeof(buf), fmt, ap);
            va_end(ap);
            port.write(buf);
        }

        void printError(const char* fmt, ...)
        {
            char buf[160];
            va_list ap;
            va_start(ap, fmt);
            vsnprintf(buf, sizeof(buf), fmt, ap);
            va_end(ap);
            print("###ОШИБКА: %s###\r\n", buf);
        }

        static bool is(const char* a, const char* b)
        {
            while (*a && *b && tolower(static_cast<unsigned char>(*a)) == *b)
            {
                ++a;
                ++b;
            }
            return *a == '\0' && *b == '\0';
        }

        static char* skipSpaces(char* s)
        {
            while (*s == ' ' || *s == '\t') ++s;
            return s;
        }

        static void trimRight(char* s)
        {
            size_t n = strlen(s);
            while (n > 0 && isspace(static_cast<unsigned char>(s[n - 1]))) s[--n] = '\0';
        }

        CopterParams& params;
        CopterController& ctl;
        CliPort& port;
        KeyValueStore* kv = nullptr;
        const char* version = "OpenPlane copter";
        bool open = false;
        bool dirty = false;
        bool overflow = false;
        bool lastWasCr = false;
        char line[LINE_MAX] = {};
        size_t lineLength = 0;
    };
}
