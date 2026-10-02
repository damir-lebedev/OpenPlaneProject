// ============================================================
// Выборочная сверка кольца чёрного ящика при включении
// (BlackBoxStorage::scanSparse) против полной (scanFull).
//
// Случайные истории кольца: полёты разной длины, стирание старых,
// переход через конец, перезагрузки. В конце одно и то же кольцо
// сверяется полностью и выборочно с разным шагом проб — голова,
// номера и список полётов обязаны совпасть, а когда выборочная сверка
// не уверена, она обязана уступить полной. Отдельно — стоимость
// на области SD-карты (16384 секторов): сколько заголовков прочитано.
//
// Запуск: pio test -e native -f native/test_blackbox_scan
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include <random>
#include <vector>

#include "hal/IFlashRegion.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxStorage.h"

using namespace BlackBoxFormat;

void setUp() {}
void tearDown() {}

namespace
{
    // Флеш в памяти: стёртое — 0xFF, запись просто копирует.
    class MemRegion : public IFlashRegion
    {
    public:
        explicit MemRegion(uint32_t sectors) : bytes(static_cast<size_t>(sectors) * SECTOR_SIZE, 0xFF) {}

        uint32_t size() const override { return static_cast<uint32_t>(bytes.size()); }
        bool read(uint32_t offset, void* data, size_t length) override
        {
            if (offset + length > bytes.size()) return false;
            memcpy(data, bytes.data() + offset, length);
            return true;
        }
        bool write(uint32_t offset, const void* data, size_t length) override
        {
            if (offset + length > bytes.size()) return false;
            memcpy(bytes.data() + offset, data, length);
            return true;
        }
        bool erase(uint32_t offset, uint32_t length) override
        {
            if (offset + length > bytes.size()) return false;
            memset(bytes.data() + offset, 0xFF, length);
            return true;
        }

        std::vector<uint8_t> bytes;
    };

    void verifyAhead(BlackBoxStorage& storage)
    {
        while (storage.eraseStep(storage.totalSectors(), 0, false)) {}
    }

    // Один полёт на ring: стереть место, записать sectors секторов записей.
    void writeFlight(BlackBoxStorage& storage, uint32_t sectors, uint32_t& clockMs)
    {
        const BlackBoxStorage::Flight* newest = storage.newestFlight();
        while (storage.eraseStep(sectors + 2, newest ? newest->number : 0)) {}
        storage.openFlight();
        uint8_t record[2 + 4 + 58];
        record[0] = REC_SYS;
        record[1] = 62;
        const uint32_t perSector = (SECTOR_PAYLOAD) / (sizeof(record) + RECORD_CHECK);
        for (uint32_t i = 0; i < sectors * perSector; ++i)
        {
            memcpy(record + 2, &i, 4);
            if (storage.append(record, sizeof(record), clockMs += 3) < 0) break;   // кольцо кончилось
        }
        storage.closeFlight();
    }

    void assertSameResult(BlackBoxStorage& full, BlackBoxStorage& sparse)
    {
        TEST_ASSERT_EQUAL_UINT32(full.headSector(), sparse.headSector());
        TEST_ASSERT_EQUAL_UINT16(full.peekNextFlight(), sparse.peekNextFlight());
        TEST_ASSERT_EQUAL_UINT32(full.flightCount(), sparse.flightCount());
        for (size_t i = 0; i < full.flightCount(); ++i)
        {
            const BlackBoxStorage::Flight& a = full.flight(i);
            const BlackBoxStorage::Flight& b = sparse.flight(i);
            TEST_ASSERT_EQUAL_UINT16(a.number, b.number);
            TEST_ASSERT_EQUAL_UINT32(a.firstSector, b.firstSector);
            TEST_ASSERT_EQUAL_UINT32(a.sectors, b.sectors);
            TEST_ASSERT_EQUAL_UINT32(a.startMs, b.startMs);
            TEST_ASSERT_EQUAL_UINT32(a.lastMs, b.lastMs);
            TEST_ASSERT_EQUAL(a.hasStart, b.hasStart);
        }
    }
}

void test_sparse_scan_gives_the_same_ring_as_the_full_scan_on_random_histories()
{
    std::mt19937 random(12345);
    int sparseUsed = 0, trials = 0;
    for (int trial = 0; trial < 300; ++trial)
    {
        const uint32_t sectors = 64 + random() % 700;
        MemRegion region(sectors);
        uint32_t clockMs = 1000;
        {
            BlackBoxStorage storage(region);
            storage.begin();
            verifyAhead(storage);
            const int flights = 1 + random() % 9;
            for (int f = 0; f < flights; ++f)
            {
                writeFlight(storage, 1 + random() % (sectors / 3), clockMs);
                if (random() % 3 == 0)   // перезагрузка между полётами
                {
                    storage.begin();
                    verifyAhead(storage);
                }
            }
        }

        BlackBoxStorage full(region);
        full.sparseMinSectors = 0xFFFFFFFFu;
        TEST_ASSERT_TRUE(full.begin());
        TEST_ASSERT_FALSE(full.scanWasSparse);

        for (const uint32_t stride : { 2u, 3u, 5u, 8u, 16u })
        {
            BlackBoxStorage sparse(region);
            sparse.sparseMinSectors = 0;
            sparse.sparseStride = stride;
            TEST_ASSERT_TRUE(sparse.begin());
            assertSameResult(full, sparse);
            sparseUsed += sparse.scanWasSparse;
            trials++;
        }
    }
    // Тест имеет смысл, только если выборочная сверка действительно работала, а не всегда уступала.
    TEST_ASSERT_TRUE_MESSAGE(sparseUsed > trials / 2, "выборочная сверка слишком часто уступает полной");
}

void test_sparse_scan_falls_back_when_the_ring_looks_empty_or_full_or_broken()
{
    MemRegion region(256);
    BlackBoxStorage empty(region);
    empty.sparseMinSectors = 0;
    empty.sparseStride = 4;
    TEST_ASSERT_TRUE(empty.begin());
    TEST_ASSERT_FALSE(empty.scanWasSparse);   // ничего не видно — полная сверка подтверждает "пусто"
    TEST_ASSERT_EQUAL_UINT32(0, empty.flightCount());
    TEST_ASSERT_TRUE(empty.scanReads >= 256);   // пробы + полный проход

    // Полёт короче шага проб: выборочная его не видит, полная находит.
    uint32_t clockMs = 0;
    {
        BlackBoxStorage storage(region);
        storage.begin();
        verifyAhead(storage);
        writeFlight(storage, 2, clockMs);
    }
    BlackBoxStorage shortOne(region);
    shortOne.sparseMinSectors = 0;
    shortOne.sparseStride = 32;
    TEST_ASSERT_TRUE(shortOne.begin());
    TEST_ASSERT_EQUAL_UINT32(1, shortOne.flightCount());

    // Две дуги настоящих секторов (чужой полёт в стёртой зоне) — кольцо "не сходится".
    MemRegion broken(256);
    {
        BlackBoxStorage storage(broken);
        storage.begin();
        verifyAhead(storage);
        writeFlight(storage, 20, clockMs);
    }
    const SectorHeader stray = makeHeader(9999, 7, 5);
    memcpy(broken.bytes.data() + 200 * SECTOR_SIZE, &stray, sizeof(stray));
    BlackBoxStorage a(broken), b(broken);
    a.sparseMinSectors = 0xFFFFFFFFu;
    b.sparseMinSectors = 0;
    b.sparseStride = 4;
    a.begin();
    b.begin();
    assertSameResult(a, b);
}

void test_sparse_scan_reads_a_few_hundred_headers_on_a_64_mb_card_region()
{
    constexpr uint32_t SECTORS = 16384;   // 64 МБ
    MemRegion region(SECTORS);
    uint32_t clockMs = 0;
    {
        BlackBoxStorage storage(region);
        storage.begin();
        verifyAhead(storage);
        writeFlight(storage, 300, clockMs);
        writeFlight(storage, 700, clockMs);
        writeFlight(storage, 120, clockMs);
        writeFlight(storage, 4000, clockMs);
    }

    BlackBoxStorage full(region);
    full.sparseMinSectors = 0xFFFFFFFFu;
    full.begin();

    BlackBoxStorage fast(region);   // порог и шаг — по умолчанию, как в прошивке
    TEST_ASSERT_TRUE(fast.begin());
    TEST_ASSERT_TRUE(fast.scanWasSparse);
    assertSameResult(full, fast);
    TEST_ASSERT_EQUAL_UINT32(4, fast.flightCount());
    TEST_ASSERT_TRUE(full.scanReads > 2u * SECTORS / 2);
    TEST_ASSERT_TRUE_MESSAGE(fast.scanReads < 700, "выборочная сверка читает слишком много заголовков");

    // Кольцо с переходом через конец и стёртыми старыми полётами.
    {
        BlackBoxStorage storage(region);
        storage.begin();
        verifyAhead(storage);
        for (int i = 0; i < 6; ++i) writeFlight(storage, 4500, clockMs);
    }
    BlackBoxStorage fullWrapped(region), fastWrapped(region);
    fullWrapped.sparseMinSectors = 0xFFFFFFFFu;
    fullWrapped.begin();
    TEST_ASSERT_TRUE(fastWrapped.begin());
    TEST_ASSERT_TRUE(fastWrapped.scanWasSparse);
    assertSameResult(fullWrapped, fastWrapped);
    TEST_ASSERT_TRUE(fastWrapped.scanReads < 700);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_sparse_scan_gives_the_same_ring_as_the_full_scan_on_random_histories);
    RUN_TEST(test_sparse_scan_falls_back_when_the_ring_looks_empty_or_full_or_broken);
    RUN_TEST(test_sparse_scan_reads_a_few_hundred_headers_on_a_64_mb_card_region);
    return UNITY_END();
}
