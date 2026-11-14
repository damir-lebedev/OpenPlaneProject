#pragma once
#include <stdint.h>

#include <map>
#include <string>
#include <vector>

// ============================================================
// Цифровые очки (или видеопередатчик) на порту MSP — для тестов. Ведут
// себя, как настоящие, по описанию протокола MSP DisplayPort:
//
//   от полётника принимают кадры MSP v1 '$' 'M' '>' | размер | команда |
//   данные | xor:
//     команда 182 (DisplayPort), первый байт данных:
//       0 HEARTBEAT — отметить;
//       1 RELEASE — экран пуст и отдан очкам;
//       2 CLEAR_SCREEN — стереть набираемое;
//       3 WRITE_STRING — строка, столбец, атрибут, символы до конца кадра;
//       4 DRAW_SCREEN — набранное становится видимым;
//     остальные команды — ответы на запросы очков (по номеру команды);
//     '!' вместо '>' — ответ «ошибка».
//
// Сами спрашивают: request(команда, данные) — байты '$' 'M' '<' ….
//
// Строги: битая контрольная сумма, мусор между кадрами, строка за краем
// экрана, атрибут с запрещёнными битами — всё считается (bad*, stray,
// outOfRange): тест проверяет, что на проводе чисто.
// ============================================================

struct DisplayPortGoggles
{
    uint8_t cols = 30, rows = 16;

    std::vector<std::string> shown;     // видимый экран (после DRAW)
    std::vector<std::string> building;  // набирается после CLEAR
    bool released = false;

    int heartbeats = 0, clears = 0, draws = 0, strings = 0, releases = 0;
    int frames = 0, badChecksum = 0, stray = 0, outOfRange = 0, badAttribute = 0, unknownSub = 0;
    size_t longestString = 0;

    std::map<uint8_t, std::vector<uint8_t>> replies;   // последний ответ по команде
    std::map<uint8_t, int> replyCount;
    std::map<uint8_t, int> errorCount;                 // ответы '!'

    DisplayPortGoggles() { resize(30, 16); }

    void resize(uint8_t c, uint8_t r)
    {
        cols = c;
        rows = r;
        shown.assign(rows, std::string(cols, ' '));
        building.assign(rows, std::string(cols, ' '));
    }

    // Запрос очков полётнику.
    static std::vector<uint8_t> request(uint8_t cmd, const std::vector<uint8_t>& data = {})
    {
        std::vector<uint8_t> f = { '$', 'M', '<', static_cast<uint8_t>(data.size()), cmd };
        uint8_t x = static_cast<uint8_t>(data.size() ^ cmd);
        for (uint8_t b : data)
        {
            f.push_back(b);
            x ^= b;
        }
        f.push_back(x);
        return f;
    }

    void feed(const uint8_t* data, size_t n)
    {
        for (size_t i = 0; i < n; ++i) byte(data[i]);
    }
    void feed(const std::string& data) { feed(reinterpret_cast<const uint8_t*>(data.data()), data.size()); }
    void feed(const std::vector<uint8_t>& data) { feed(data.data(), data.size()); }

    std::string row(uint8_t r) const { return r < shown.size() ? shown[r] : std::string(); }

    // Видимый экран одной строкой с переводами строк — для сообщений тестов.
    std::string text() const
    {
        std::string t;
        for (const std::string& line : shown) t += line + "\n";
        return t;
    }

    // На проводе чисто: ни битых кадров, ни мусора, ни строк за экраном.
    bool clean() const { return badChecksum == 0 && stray == 0 && outOfRange == 0 && badAttribute == 0 && unknownSub == 0; }

private:
    enum class State { Idle, M, Direction, Size, Cmd, Data, Check };
    State state = State::Idle;
    bool error = false;
    uint8_t size = 0, cmd = 0, check = 0;
    std::vector<uint8_t> payload;

    void byte(uint8_t b)
    {
        switch (state)
        {
            case State::Idle:
                if (b == '$') state = State::M;
                else ++stray;
                return;
            case State::M:
                if (b == 'M') state = State::Direction;
                else
                {
                    ++stray;
                    state = State::Idle;
                }
                return;
            case State::Direction:
                if (b != '>' && b != '!')
                {
                    ++stray;
                    state = State::Idle;
                    return;
                }
                error = b == '!';
                state = State::Size;
                return;
            case State::Size:
                size = b;
                check = b;
                payload.clear();
                state = State::Cmd;
                return;
            case State::Cmd:
                cmd = b;
                check ^= b;
                state = size ? State::Data : State::Check;
                return;
            case State::Data:
                payload.push_back(b);
                check ^= b;
                if (payload.size() >= size) state = State::Check;
                return;
            case State::Check:
                state = State::Idle;
                if (b != check)
                {
                    ++badChecksum;
                    return;
                }
                ++frames;
                frame();
                return;
        }
    }

    void frame()
    {
        if (error)
        {
            ++errorCount[cmd];
            return;
        }
        if (cmd != 182)
        {
            replies[cmd] = payload;
            ++replyCount[cmd];
            return;
        }
        if (payload.empty())
        {
            ++unknownSub;
            return;
        }
        switch (payload[0])
        {
            case 0: ++heartbeats; break;
            case 1:
                ++releases;
                released = true;
                shown.assign(rows, std::string(cols, ' '));
                building.assign(rows, std::string(cols, ' '));
                break;
            case 2:
                ++clears;
                building.assign(rows, std::string(cols, ' '));
                break;
            case 3: writeString(); break;
            case 4:
                ++draws;
                released = false;
                shown = building;
                break;
            default: ++unknownSub; break;
        }
    }

    void writeString()
    {
        if (payload.size() < 4)
        {
            ++unknownSub;
            return;
        }
        ++strings;
        const uint8_t r = payload[1], c = payload[2], attribute = payload[3];
        if (attribute & 0xBC) ++badAttribute;   // бит 7 (версия) и биты 2–5 — только нули
        const size_t n = payload.size() - 4;
        if (n > longestString) longestString = n;
        if (n > 30) ++outOfRange;               // больше 30 символов за кадр нельзя
        for (size_t i = 0; i < n; ++i)
        {
            if (r >= rows || c + i >= cols)
            {
                ++outOfRange;
                break;
            }
            building[r][c + i] = static_cast<char>(payload[4 + i]);
        }
    }
};
