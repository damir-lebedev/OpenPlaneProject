#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "hal/IFlashRegion.h"
#include "telemetry/BlackBoxFormat.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК: кольцо секторов во флеше
//
// Сектора пишутся по кругу, по одному за другим: голова (head) —
// следующий сектор под запись, за ней freeCount сверенных стёртых
// секторов, дальше — старые полёты (от самого старого к новому) и
// снова голова. При включении голова находится по заголовкам: сектор
// после сектора с наибольшим seq. Карты в NVS нет — после пропажи
// питания в любой момент всё восстанавливается по самим секторам.
//
// Сверка при включении. Маленькая область (флеш ESP32, ~3.4 тыс. секторов,
// чтение ~0.1 мс) читается вся: заголовок каждого сектора. На SD-карте
// чтение случайного блока стоит ~0.6 мс, а область — десятки тысяч секторов:
// полная сверка 64 МБ шла бы 20 секунд. Поэтому большая область сверяется
// выборочно, по устройству кольца: настоящие сектора — одна непрерывная дуга
// (старые полёты стираются с её начала, запись идёт к её концу), остальное —
// стёртое место. Читается каждый S-й сектор (~500 проб), границы дуги и
// границы полётов внутри неё уточняются делением пополам. Результат тот же,
// что у полной сверки; если картина не сходится (дуг несколько, номера секторов
// не идут подряд, ни одной пробы не попало в дугу короче S секторов) —
// выполняется полная сверка.
//
// Стирание (eraseStep) — отдельно от записи, шагами, и только когда
// разрешил вызывающий (на земле без ARM): на ESP32 оно останавливает
// оба ядра на десятки-сотни мс. Запись во время полёта идёт только в
// заранее стёртые сектора. Старые полёты стираются ЦЕЛИКОМ, от самого
// старого, пока впереди не наберётся нужный запас; полёт protect
// (последний записанный) не стирается никогда.
//
// Запись страницами: байты копятся в буфере страницы (256 байт) и
// уходят во флеш, когда страница заполнилась, — одна запись флеша на
// страницу. flush() дописывает неполную страницу; дальше запись
// продолжается в неё же (NOR: стёртые байты можно дописать).
// Не потокобезопасен: владелец — одна задача (BlackBox держит мьютекс).
// ============================================================

class BlackBoxStorage
{
public:

    struct Flight
    {
        uint16_t number = 0;
        uint32_t firstSector = 0;   // индекс в кольце
        uint32_t sectors = 0;
        uint32_t startMs = 0;       // millis() при открытии первого сектора
        uint32_t lastMs = 0;        // ...и последнего
        bool hasStart = false;      // первый сектор (схема, параметры) на месте
    };

    static constexpr size_t MAX_FLIGHTS = 32;

    explicit BlackBoxStorage(IFlashRegion& region)
        : flash(region)
    {
    }

    // Области не меньше стольких секторов сверяются выборочно (см. выше).
    static constexpr uint32_t SPARSE_MIN_SECTORS = 4096;
    // Сколько проб берётся на область при выборочной сверке.
    static constexpr uint32_t SPARSE_PROBES = 512;

    // Для тестов: порог и шаг проб. 0 — по умолчанию.
    uint32_t sparseMinSectors = SPARSE_MIN_SECTORS;
    uint32_t sparseStride = 0;

    // Сколько заголовков прочитала последняя сверка и была ли она выборочной.
    uint32_t scanReads = 0;
    bool scanWasSparse = false;

    // Найти голову, номера и список полётов по заголовкам секторов.
    bool begin()
    {
        sectorCount = flash.size() / BlackBoxFormat::SECTOR_SIZE;
        flightTotal = 0;
        freeCount = 0;
        evicting = 0;
        flightOpen = false;
        sectorOpen = false;
        scanReads = 0;
        scanWasSparse = false;
        if (sectorCount < 2)
        {
            sectorCount = 0;
            return false;
        }

        markPresent = false;
        if (flash.ringMarkedEmpty())
        {
            // Кольцо пусто по метке — сверять нечего (стёртое впереди проверит eraseStep).
            head = 0;
            nextSeq = 1;
            nextFlight = 1;
            markPresent = true;
            scanReads = 1;
            return true;
        }

        if (sectorCount >= sparseMinSectors && scanSparse())
        {
            scanWasSparse = true;
            return true;
        }
        flightTotal = 0;
        scanFull();
        if (flightTotal == 0)
        {
            flash.markRingEmpty();   // следующая загрузка — без прохода по области
            markPresent = true;
        }
        return true;
    }

    // Прочитать заголовки всех секторов: голова, номера, список полётов.
    void scanFull()
    {
        // Первый проход читает заголовок каждого сектора и запоминает, какие
        // из них настоящие: второй проход (полёты) не тратит чтение на пустые.
        // На SD-карте чтение — десятки-сотни мкс, и область — десятки тысяч секторов.
        validMap.assign((sectorCount + 7) / 8, 0);

        bool any = false;
        uint32_t maxSeq = 0;
        uint32_t maxIndex = 0;
        uint16_t maxFlight = 0;
        for (uint32_t i = 0; i < sectorCount; ++i)
        {
            BlackBoxFormat::SectorHeader h;
            if (!readHeader(i, h)) continue;
            validMap[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
            if (!any || h.seq > maxSeq)
            {
                maxSeq = h.seq;
                maxIndex = i;
                maxFlight = h.flight;
            }
            any = true;
        }

        head = any ? (maxIndex + 1) % sectorCount : 0;
        nextSeq = any ? maxSeq + 1 : 1;
        nextFlight = any ? nextNumber(maxFlight) : 1;

        // Полёты — по кольцу от головы: сначала самые старые.
        for (uint32_t k = 0; k < sectorCount; ++k)
        {
            const uint32_t index = (head + k) % sectorCount;
            if (!(validMap[index / 8] & (1u << (index % 8)))) continue;
            BlackBoxFormat::SectorHeader h;
            if (readHeader(index, h)) noteSector(index, h, /*fromScan*/ true);
        }
    }

    bool isReady() const { return sectorCount > 0; }
    uint32_t totalSectors() const { return sectorCount; }
    uint32_t freeSectors() const { return freeCount; }
    uint32_t headSector() const { return head; }

    // Сколько данных ещё влезет в стёртое место, байт.
    uint32_t freeBytes() const { return freeCount * static_cast<uint32_t>(BlackBoxFormat::SECTOR_PAYLOAD); }

    // --------------------------------------------------------
    // Запись
    // --------------------------------------------------------

    // Начать полёт: номер выдаётся сразу, сектор — с первой записью.
    uint16_t openFlight()
    {
        closeFlight();
        flightOpen = true;
        flightNumber = nextFlight;
        nextFlight = nextNumber(nextFlight);
        return flightNumber;
    }

    bool isFlightOpen() const { return flightOpen; }
    uint16_t peekNextFlight() const { return nextFlight; }
    uint16_t currentFlight() const { return flightOpen ? flightNumber : 0; }

    // Дописать запись целиком ([тип][длина][данные]; CRC-8 добавляется
    // здесь). Возвращает число записей страниц во флеш (0..3), или -1,
    // если стёртых секторов не осталось (запись не принята — повторить
    // позже).
    int append(const uint8_t* data, size_t size, uint32_t nowMs)
    {
        if (!flightOpen || size == 0 || size > BlackBoxFormat::MAX_RECORD) return -1;

        uint8_t checked[BlackBoxFormat::MAX_RECORD + BlackBoxFormat::RECORD_CHECK];
        memcpy(checked, data, size);
        checked[size] = BlackBoxFormat::crc8(data, size);
        const uint8_t* record = checked;
        size_t length = size + BlackBoxFormat::RECORD_CHECK;

        int writes = 0;
        if (!sectorOpen || position + length > BlackBoxFormat::SECTOR_SIZE)
        {
            writes += flush();
            sectorOpen = false;
            if (!openSector(nowMs)) return -1;
        }

        while (length > 0)
        {
            const uint32_t offset = position % BlackBoxFormat::PAGE_SIZE;
            const size_t chunk = min32(static_cast<uint32_t>(length), BlackBoxFormat::PAGE_SIZE - offset);
            memcpy(page + offset, record, chunk);
            record += chunk;
            length -= chunk;
            position += static_cast<uint32_t>(chunk);
            if (position % BlackBoxFormat::PAGE_SIZE == 0) writes += flush();
        }
        return writes;
    }

    // Дописать во флеш накопленное (неполную страницу тоже).
    int flush()
    {
        if (!sectorOpen || written >= position) return 0;

        const uint32_t pageStart = written - written % BlackBoxFormat::PAGE_SIZE;
        const uint32_t from = written - pageStart;
        const uint32_t to = position - pageStart;   // <= PAGE_SIZE
        const bool ok = flash.write(sector * BlackBoxFormat::SECTOR_SIZE + written, page + from, to - from);
        if (!ok) writeErrors++;
        pageWrites++;
        written = position;
        if (position % BlackBoxFormat::PAGE_SIZE == 0) memset(page, 0xFF, sizeof(page));
        return 1;
    }

    void closeFlight()
    {
        flush();
        sectorOpen = false;
        flightOpen = false;
    }

    // --------------------------------------------------------
    // Стирание
    // --------------------------------------------------------

    // Один шаг: сверить или стереть следующий сектор (или блок 64 КБ)
    // впереди головы. Сектора без полёта (мусор, недостёртое) стираются
    // всегда — это бесплатное место; полёты — только пока стёртого
    // меньше targetFree секторов, и начатый полёт — до конца. protect —
    // полёт, который трогать нельзя (0 — нет). true — работа была (флеш
    // читался или стирался), false — делать нечего или дальше нельзя.
    //
    // allowErase = false — только сверка (чтение сектора, ~0.1 мс): так
    // можно и в воздухе, если стёртое место уже есть, но ещё не сверено.
    bool eraseStep(uint32_t targetFree, uint16_t protect, bool allowErase = true)
    {
        if (!isReady() || freeCount >= usableSectors()) return false;

        const uint32_t index = (head + freeCount) % sectorCount;
        BlackBoxFormat::SectorHeader h;
        const bool valid = readHeader(index, h);
        if (!allowErase)
        {
            if (valid || !isErased(index, 1)) return false;
            freeCount++;
            return true;
        }
        if (evicting && !(valid && h.flight == evicting)) evicting = 0;
        if (valid && !erasable(h, protect)) return false;
        const bool needSpace = freeCount < targetFree || evicting;
        if (valid && !needSpace) return false;

        uint32_t count = 1;
        if (index % BlackBoxFormat::SECTORS_PER_BLOCK == 0 && index + BlackBoxFormat::SECTORS_PER_BLOCK <= sectorCount &&
            usableSectors() - freeCount >= BlackBoxFormat::SECTORS_PER_BLOCK &&
            blockErasable(index, protect, needSpace))
        {
            count = BlackBoxFormat::SECTORS_PER_BLOCK;
        }

        if (!isErased(index, count))
        {
            if (!flash.erase(index * BlackBoxFormat::SECTOR_SIZE, count * BlackBoxFormat::SECTOR_SIZE))
            {
                eraseErrors++;
                return true;
            }
            eraseOps++;
        }

        for (uint32_t k = 0; k < count; ++k) forgetSector(index + k);
        freeCount += count;
        return true;
    }

    // Стереть всё (консоль, без ARM). Долго: до ~40 с на 14 МБ.
    bool eraseAll()
    {
        if (!isReady()) return false;
        closeFlight();
        const bool ok = flash.erase(0, sectorCount * BlackBoxFormat::SECTOR_SIZE);
        head = 0;
        freeCount = ok ? sectorCount : 0;
        flightTotal = 0;
        evicting = 0;
        if (ok)
        {
            flash.markRingEmpty();
            markPresent = true;
        }
        return ok;
    }

    // --------------------------------------------------------
    // Чтение
    // --------------------------------------------------------

    size_t flightCount() const { return flightTotal; }
    const Flight& flight(size_t i) const { return flights[i]; }   // 0 — самый старый

    const Flight* findFlight(uint16_t number) const
    {
        for (size_t i = 0; i < flightTotal; ++i)
        {
            if (flights[i].number == number) return &flights[i];
        }
        return nullptr;
    }

    const Flight* newestFlight() const { return flightTotal ? &flights[flightTotal - 1] : nullptr; }

    uint32_t sectorOf(const Flight& f, uint32_t k) const { return (f.firstSector + k) % sectorCount; }

    bool readSector(uint32_t index, uint8_t* buffer)
    {
        return index < sectorCount && flash.read(index * BlackBoxFormat::SECTOR_SIZE, buffer, BlackBoxFormat::SECTOR_SIZE);
    }

    // Статистика для SYS и консоли.
    uint32_t pageWrites = 0;
    uint32_t eraseOps = 0;
    uint32_t writeErrors = 0;
    uint32_t eraseErrors = 0;


private:

    IFlashRegion& flash;
    uint32_t sectorCount = 0;
    bool markPresent = false;   // метка "кольцо пусто" стоит
    uint32_t head = 0;
    uint32_t freeCount = 0;
    uint32_t nextSeq = 1;
    uint16_t nextFlight = 1;
    uint16_t evicting = 0;   // полёт, который стирается сейчас (стираем до конца)

    Flight flights[MAX_FLIGHTS];
    size_t flightTotal = 0;

    // Открытый полёт и сектор.
    bool flightOpen = false;
    uint16_t flightNumber = 0;
    bool sectorOpen = false;
    uint32_t sector = 0;
    uint32_t position = 0;   // байт в секторе занято (в буфере или во флеше)
    uint32_t written = 0;    // ...из них уже во флеше
    uint8_t page[BlackBoxFormat::PAGE_SIZE] = {};

    uint8_t scratch[BlackBoxFormat::SECTOR_SIZE] = {};
    std::vector<uint8_t> validMap;   // бит на сектор: заголовок настоящий (только внутри begin())

    static uint32_t min32(uint32_t a, uint32_t b) { return a < b ? a : b; }
    static uint16_t nextNumber(uint16_t n) { return n >= 0xFFFE ? 1 : static_cast<uint16_t>(n + 1); }

    // Открытый сектор стирать нельзя — сверка остановится перед ним.
    uint32_t usableSectors() const { return sectorOpen ? sectorCount - 1 : sectorCount; }

    bool readHeader(uint32_t index, BlackBoxFormat::SectorHeader& h)
    {
        scanReads++;
        return flash.read(index * BlackBoxFormat::SECTOR_SIZE, &h, sizeof(h)) && BlackBoxFormat::isValid(h);
    }

    bool erasable(const BlackBoxFormat::SectorHeader& h, uint16_t protect) const
    {
        return h.flight != protect && !(flightOpen && h.flight == flightNumber);
    }

    // Блок целиком — только если в нём один мусор или один и тот же
    // стираемый полёт: блок на стыке двух полётов задел бы начало
    // следующего, который можно было сохранить, — такой идёт по секторам.
    bool blockErasable(uint32_t first, uint16_t protect, bool mayEvict)
    {
        bool anyFlight = false;
        uint16_t flightInBlock = 0;
        for (uint32_t k = 0; k < BlackBoxFormat::SECTORS_PER_BLOCK; ++k)
        {
            if (sectorOpen && first + k == sector) return false;
            BlackBoxFormat::SectorHeader h;
            if (!readHeader(first + k, h)) continue;
            if (!mayEvict || !erasable(h, protect)) return false;
            if (anyFlight && h.flight != flightInBlock) return false;
            anyFlight = true;
            flightInBlock = h.flight;
        }
        return true;
    }

    // ----- выборочная сверка -----

    using Header = BlackBoxFormat::SectorHeader;

    struct Sample
    {
        uint32_t t;   // позиция в дуге настоящих секторов (0 — самый старый)
        Header h;
    };

    struct Run
    {
        Sample first;
        Sample last;
    };

    struct SparseScan
    {
        uint32_t arcStart = 0;
        std::vector<Run> runs;
        bool haveRun = false;
        Sample runStart = {};
    };

    uint32_t arcIndex(const SparseScan& scan, uint32_t t) const { return (scan.arcStart + t) % sectorCount; }

    // Между отсчётами t0 и t1 (оба настоящие) найти границы полётов.
    bool splitRange(SparseScan& scan, const Sample& a, const Sample& b)
    {
        if (a.h.flight == b.h.flight) return b.h.seq - a.h.seq == b.t - a.t;   // подряд — иначе картина не сходится
        if (b.t - a.t == 1)
        {
            scan.runs.push_back({ scan.runStart, a });
            scan.runStart = b;
            return true;
        }
        Sample mid;
        mid.t = (a.t + b.t) / 2;
        if (!readHeader(arcIndex(scan, mid.t), mid.h)) return false;
        return splitRange(scan, a, mid) && splitRange(scan, mid, b);
    }

    bool scanSparse()
    {
        const uint32_t N = sectorCount;
        const uint32_t S = sparseStride ? sparseStride : (N + SPARSE_PROBES - 1) / SPARSE_PROBES;
        const uint32_t K = (N + S - 1) / S;   // пробы в секторах 0, S, 2S...
        if (S < 2 || K < 4) return false;

        // 1. Пробы. Настоящие должны идти одной непрерывной дугой (по кругу).
        std::vector<Header> probe(K);
        std::vector<uint8_t> ok(K, 0);
        uint32_t valids = 0;
        for (uint32_t k = 0; k < K; ++k)
        {
            ok[k] = readHeader(k * S, probe[k]) ? 1 : 0;
            valids += ok[k];
        }
        if (valids == 0 || valids == K) return false;   // пусто (или короче S — не видно) / заполнено целиком

        uint32_t starts = 0, ends = 0, kFirst = 0, kLast = 0;
        for (uint32_t k = 0; k < K; ++k)
        {
            if (ok[k] && !ok[(k + K - 1) % K])
            {
                starts++;
                kFirst = k;
            }
            if (ok[k] && !ok[(k + 1) % K])
            {
                ends++;
                kLast = k;
            }
        }
        if (starts != 1 || ends != 1) return false;

        // 2. Края дуги — делением пополам между крайней настоящей пробой и соседней пустой.
        const uint32_t before = ((kFirst + K - 1) % K) * S;
        const uint32_t after = ((kLast + 1) % K) * S;
        Header h;
        uint32_t lo = 0, hi = (kFirst * S + N - before) % N;   // lo — пусто, hi — настоящий
        while (hi - lo > 1)
        {
            const uint32_t mid = (lo + hi) / 2;
            if (readHeader((before + mid) % N, h)) hi = mid;
            else lo = mid;
        }
        const uint32_t arcFirst = (before + hi) % N;

        lo = 0;
        hi = (after + N - kLast * S) % N;   // lo — настоящий, hi — пусто
        while (hi - lo > 1)
        {
            const uint32_t mid = (lo + hi) / 2;
            if (readHeader((kLast * S + mid) % N, h)) lo = mid;
            else hi = mid;
        }
        const uint32_t arcLast = (kLast * S + lo) % N;

        const uint32_t length = (arcLast + N - arcFirst) % N + 1;
        SparseScan scan;
        scan.arcStart = arcFirst;
        Sample oldest = { 0, {} };
        Sample newest = { length - 1, {} };
        if (!readHeader(arcFirst, oldest.h) || !readHeader(arcLast, newest.h)) return false;
        if (newest.h.seq - oldest.h.seq != length - 1) return false;   // номера секторов дуги идут подряд

        // 3. Полёты внутри дуги: между пробами смотрим номер полёта и ход seq.
        std::vector<Sample> samples;
        samples.push_back(oldest);
        for (uint32_t k = kFirst;; k = (k + 1) % K)
        {
            const uint32_t t = (k * S + N - arcFirst) % N;
            if (t > 0 && t < length - 1) samples.push_back({ t, probe[k] });
            if (k == kLast) break;
        }
        samples.push_back(newest);

        scan.runStart = oldest;
        for (size_t i = 0; i + 1 < samples.size(); ++i)
        {
            if (samples[i + 1].t <= samples[i].t) return false;
            if (!splitRange(scan, samples[i], samples[i + 1])) return false;
        }
        scan.runs.push_back({ scan.runStart, newest });

        // 4. Результат — как у полной сверки.
        const size_t keep = scan.runs.size() < MAX_FLIGHTS ? scan.runs.size() : MAX_FLIGHTS;
        flightTotal = 0;
        for (size_t i = scan.runs.size() - keep; i < scan.runs.size(); ++i)
        {
            const Run& run = scan.runs[i];
            Flight& f = flights[flightTotal++];
            const uint32_t index = arcIndex(scan, run.first.t);
            f.number = run.first.h.flight;
            f.firstSector = index;
            f.sectors = run.last.t - run.first.t + 1;
            f.startMs = run.first.h.startMs;
            f.lastMs = run.last.h.startMs;
            f.hasStart = !sameFlightBefore(index, run.first.h);
        }
        head = (arcLast + 1) % N;
        nextSeq = newest.h.seq + 1;
        nextFlight = nextNumber(newest.h.flight);
        return true;
    }

    bool isErased(uint32_t first, uint32_t count)
    {
        for (uint32_t k = 0; k < count; ++k)
        {
            if (!flash.read((first + k) * BlackBoxFormat::SECTOR_SIZE, scratch, sizeof(scratch))) return false;
            if (!std::all_of(std::begin(scratch), std::end(scratch), [](uint8_t b) { return b == 0xFF; })) return false;
        }
        return true;
    }

    bool openSector(uint32_t nowMs)
    {
        if (freeCount == 0) return false;

        if (markPresent)
        {
            flash.clearRingMark();   // до первой записи в кольцо: иначе обрыв питания оставил бы метку "пусто" над данными
            markPresent = false;
        }

        sector = head;
        head = (head + 1) % sectorCount;
        freeCount--;

        const BlackBoxFormat::SectorHeader h = BlackBoxFormat::makeHeader(nextSeq++, flightNumber, nowMs);
        memset(page, 0xFF, sizeof(page));
        memcpy(page, &h, sizeof(h));
        position = sizeof(h);
        written = 0;
        sectorOpen = true;
        noteSector(sector, h, false);
        return true;
    }

    // Сектор попал в список полётов (при включении или при записи).
    void noteSector(uint32_t index, const BlackBoxFormat::SectorHeader& h, bool fromScan)
    {
        Flight* last = flightTotal ? &flights[flightTotal - 1] : nullptr;
        if (last && last->number == h.flight && sectorOf(*last, last->sectors) == index)
        {
            last->sectors++;
            last->lastMs = h.startMs;
            return;
        }

        if (flightTotal == MAX_FLIGHTS)
        {
            memmove(&flights[0], &flights[1], sizeof(Flight) * (MAX_FLIGHTS - 1));
            flightTotal--;
        }

        Flight& f = flights[flightTotal++];
        f.number = h.flight;
        f.firstSector = index;
        f.sectors = 1;
        f.startMs = h.startMs;
        f.lastMs = h.startMs;
        // При включении начало полёта неизвестно, пока не видно
        // соседа: сектор без предшественника того же полёта — начало.
        f.hasStart = fromScan ? !sameFlightBefore(index, h) : true;
    }

    bool sameFlightBefore(uint32_t index, const BlackBoxFormat::SectorHeader& h)
    {
        const uint32_t before = (index + sectorCount - 1) % sectorCount;
        BlackBoxFormat::SectorHeader p;
        return readHeader(before, p) && p.flight == h.flight && p.seq + 1 == h.seq;
    }

    // Сектор стирается: убрать его из начала своего полёта.
    void forgetSector(uint32_t index)
    {
        for (size_t i = 0; i < flightTotal; ++i)
        {
            Flight& f = flights[i];
            if (f.sectors == 0 || f.firstSector != index) continue;

            evicting = f.number;
            f.firstSector = (f.firstSector + 1) % sectorCount;
            f.sectors--;
            f.hasStart = false;
            if (f.sectors == 0)
            {
                memmove(&flights[i], &flights[i + 1], sizeof(Flight) * (flightTotal - i - 1));
                flightTotal--;
                evicting = 0;
            }
            return;
        }
    }
};
