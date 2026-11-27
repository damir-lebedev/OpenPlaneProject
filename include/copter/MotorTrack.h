#pragma once
// Проигрыватель трека для музыки моторами (176c): ноты голосов по времени ->
// MotorMusic::note(). Треки — встроенные (copter/MotorTracks.h, генерирует
// tools/motor_tracks.py) и свой во флеше (слот custom, грузит
// tools/motor_music.py --upload).
//
// Формат: 'T', голосов 1..4, единица длительности, мс (uint16 LE), по голосу:
// число событий (uint16 LE) и события (нота MIDI, 0 — пауза; длительность в
// единицах 1..255). Байты читаются через fetch по кускам: встроенный трек —
// один кусок во флеше, свой — куски в хранилище настроек (ОЗУ на копию нет).
//
// Голосов меньше, чем моторов, — моторы дублируют голоса по кругу (мотор m
// играет голос m % голосов): громче. Октавы: весь трек сдвигается на целые
// октавы, чтобы больше нот попало в [lo, hi], выпавшие — своей октавой внутрь.
// Код свой.

#include <math.h>
#include <stdint.h>

#include "copter/MotorMusic.h"

namespace copter
{
    class MotorTrack
    {
    public:
        // Кусок номер chunk: указатель и длина; nullptr — нет.
        typedef const uint8_t* (*Fetch)(void* ctx, uint16_t chunk, uint16_t* length);
        static constexpr uint16_t MAX_SIZE = 1024;

        // Проверить трек и начать. lo/hi — диапазон звука, Гц. false — битый.
        bool start(Fetch f, void* c, uint16_t chunkBytes, uint16_t size, float lo, float hi, bool repeat)
        {
            stop();
            if (!f || chunkBytes == 0 || size < 4 || size > MAX_SIZE || !(hi > lo * 1.5f)) return false;
            fetch = f;
            ctx = c;
            chunk = chunkBytes;
            total = size;
            if (at(0) != 'T') return false;
            voices = at(1);
            unitMs = static_cast<uint16_t>(at(2) | (at(3) << 8));
            if (voices < 1 || voices > MotorMusic::VOICES || unitMs == 0) return false;
            uint16_t pos = 4;
            for (uint8_t v = 0; v < voices; ++v)
            {
                if (pos + 2 > total) return false;
                const uint16_t n = static_cast<uint16_t>(at(pos) | (at(pos + 1) << 8));
                pos = static_cast<uint16_t>(pos + 2);
                if (pos + 2u * n > total) return false;
                begin[v] = pos;
                end[v] = static_cast<uint16_t>(pos + 2 * n);
                pos = end[v];
            }
            if (pos != total) return false;
            loHz = lo;
            hiHz = hi;
            shift = bestShift();
            loop = repeat;
            rewind();
            playing = true;
            return true;
        }
        void stop() { playing = false; }
        bool isPlaying() const { return playing; }
        uint8_t voiceCount() const { return voices; }
        int8_t octaveShift() const { return static_cast<int8_t>(shift / 12); }

        // Такт: двигает ноты и держит музыку живой. false — трек кончился
        // (моторы — стоп, музыка выключена).
        bool tick(float dt, MotorMusic& music)
        {
            if (!playing) return false;
            if (!music.active())
            {
                playing = false;
                return false;
            }
            clockMs += dt * 1000.0f;
            const int32_t now = static_cast<int32_t>(clockMs);
            bool any = false;
            for (uint8_t v = 0; v < voices; ++v)
            {
                while (pos[v] < end[v] && now >= nextMs[v])
                {
                    const uint8_t note = at(pos[v]);
                    const uint8_t dur = at(pos[v] + 1);
                    pos[v] = static_cast<uint16_t>(pos[v] + 2);
                    nextMs[v] += static_cast<int32_t>(dur) * unitMs;
                    const float hz = note ? fold(note) : 0.0f;
                    for (uint8_t m = v; m < music.count(); m = static_cast<uint8_t>(m + voices)) music.note(m, hz);
                }
                if (pos[v] < end[v] || now < nextMs[v]) any = true;
                else if (!loop)
                    for (uint8_t m = v; m < music.count(); m = static_cast<uint8_t>(m + voices)) music.note(m, 0.0f);
            }
            if (!any)
            {
                if (loop)
                {
                    rewind();
                    return true;
                }
                music.note(MotorMusic::ALL, MotorMusic::OFF);
                music.stop();
                playing = false;
                return false;
            }
            music.keepAlive();
            return true;
        }

        // Звук ноты с переносом трека и октавами внутрь диапазона.
        float fold(uint8_t note) const
        {
            float hz = 440.0f * powf(2.0f, (static_cast<int>(note) + shift - 69) / 12.0f);
            while (hz > hiHz && hz * 0.5f >= loHz * 0.999f) hz *= 0.5f;
            while (hz < loHz && hz * 2.0f <= hiHz * 1.001f) hz *= 2.0f;
            return hz;
        }

    private:
        uint8_t at(uint16_t offset) const
        {
            if (offset >= total) return 0;
            uint16_t length = 0;
            const uint8_t* p = fetch(ctx, static_cast<uint16_t>(offset / chunk), &length);
            const uint16_t k = static_cast<uint16_t>(offset % chunk);
            return p && k < length ? p[k] : 0;
        }

        void rewind()
        {
            clockMs = 0.0f;
            for (uint8_t v = 0; v < voices; ++v)
            {
                pos[v] = begin[v];
                nextMs[v] = 0;
            }
        }

        // Сдвиг на целые октавы: больше всего нот в [lo, hi], при равенстве — меньший.
        int16_t bestShift() const
        {
            int16_t best = 0;
            int bestFit = -1;
            for (int octs = -4; octs <= 4; ++octs)
            {
                int fit = 0;
                for (uint8_t v = 0; v < voices; ++v)
                    for (uint16_t p = begin[v]; p < end[v]; p = static_cast<uint16_t>(p + 2))
                    {
                        const uint8_t n = at(p);
                        if (!n) continue;
                        const float hz = 440.0f * powf(2.0f, (n + 12 * octs - 69) / 12.0f);
                        fit += hz >= loHz && hz <= hiHz;
                    }
                if (fit > bestFit || (fit == bestFit && abs(octs) < abs(best / 12)))
                {
                    bestFit = fit;
                    best = static_cast<int16_t>(12 * octs);
                }
            }
            return best;
        }

        Fetch fetch = nullptr;
        void* ctx = nullptr;
        float clockMs = 0.0f, loHz = 200.0f, hiHz = 1000.0f;
        int32_t nextMs[MotorMusic::VOICES] = {};
        uint16_t begin[MotorMusic::VOICES] = {}, end[MotorMusic::VOICES] = {}, pos[MotorMusic::VOICES] = {};
        uint16_t chunk = 1, total = 0, unitMs = 0;
        int16_t shift = 0;
        uint8_t voices = 0;
        bool loop = false, playing = false;
    };

    // Музыка и проигрыватель вместе: на F405 — в CCM (copter/CopterController.h attachMusic).
    struct MotorMusicParts
    {
        MotorMusic music;
        MotorTrack track;
    };
}
