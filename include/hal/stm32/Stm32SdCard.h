#pragma once
#include <Arduino.h>

#include "hal/IBlockDevice.h"
#include "hal/Rtos.h"

// ============================================================
// SD-карта на STM32H743: SDMMC1, 4-битная шина, блоки по 512 байт
// (HAL_SD в режиме опроса — без DMA и прерываний).
//
// Выводы SDMMC1 — PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) — на платах
// DevEBox и WeAct именно они ведут к слоту µSD; настройка выводов и
// тактирования — HAL_SD_MspInit() в src/stm32/sd_msp.cpp. Ядро SDMMC
// тактируется от PLL1Q = 48 МГц (SystemClock_Config варианта платы), при
// ClockDiv = 1 шина идёт на 24 МГц — предел обычного режима SD (25 МГц).
// Карта на пониженных 12 и 6 МГц пробуется, если на 24 не прочиталось.
//
// Аппаратное управление потоком SDMMC включено: передача идёт опросом (CPU
// сам таскает слова FIFO), и полётная задача вытесняет задачу записи прямо
// посреди блока — без управления потоком FIFO за это время переполнялся
// бы (HAL_SD_ERROR_RX_OVERRUN, 0x20), на плате это было видно сразу. С ним
// SDMMC просто останавливает тактирование, пока FIFO не освободится.
//
// Не потокобезопасно: владелец — одна задача (BlackBox держит мьютекс).
// Ожидание готовности карты после записи отдаёт процессор другим
// задачам (Rtos::sleepMs): полётная задача вытесняет эту и так, но
// фоновым задачам низкого приоритета тоже нужно время.
// ============================================================

class Stm32SdCard : public IBlockDevice
{
public:

    // Поднять шину и опознать карту. false — карты нет или она не отвечает.
    bool begin()
    {
        blocks = 0;
        initErrorCode = 0;
        mayBeBusy = true;
        for (const uint32_t divider : { 1u, 2u, 4u })
        {
            if (!initAt(divider)) continue;

            // Пробное чтение — на этой скорости шина действительно держит?
            alignas(4) uint8_t probe[BLOCK_SIZE];
            blocks = info.LogBlockNbr;
            if (readChunk(0, probe, 1)) return true;
            blocks = 0;
        }
        return false;
    }

    uint32_t blockCount() const override { return blocks; }

    bool read(uint32_t block, void* data, uint32_t count) override
    {
        if (!inRange(block, count)) return false;

        uint8_t* out = static_cast<uint8_t*>(data);
        while (count > 0)
        {
            const uint32_t limit = aligned(out) ? DIRECT_CHUNK_BLOCKS : CHUNK_BLOCKS;
            const uint32_t n = count < limit ? count : limit;
            if (!readChunk(block, out, n)) return false;
            block += n;
            out += n * BLOCK_SIZE;
            count -= n;
        }
        return true;
    }

    bool write(uint32_t block, const void* data, uint32_t count) override
    {
        if (!inRange(block, count)) return false;

        const uint8_t* in = static_cast<const uint8_t*>(data);
        while (count > 0)
        {
            const uint32_t limit = aligned(in) ? DIRECT_CHUNK_BLOCKS : CHUNK_BLOCKS;
            const uint32_t n = count < limit ? count : limit;
            if (!writeChunk(block, in, n)) return false;
            block += n;
            in += n * BLOCK_SIZE;
            count -= n;
        }
        return true;
    }

    // Что за карта и как с ней общаемся — для строки состояния.
    uint32_t cardType() const { return info.CardType; }          // 0 — SDSC, 1 — SDHC/SDXC, 2 — SDHC/SDXC UHS-I...
    uint32_t cardClassSpeed() const { return info.CardSpeed; }
    uint32_t clockDivider() const { return clockDiv; }
    uint32_t lastErrorCode() const { return handle.ErrorCode; }
    uint32_t initError() const { return initErrorCode; }

    // Счётчики для SYS и отладки.
    uint32_t readOps = 0;
    uint32_t writeOps = 0;
    uint32_t errors = 0;
    uint32_t retries = 0;


private:

    static constexpr uint32_t CHUNK_BLOCKS = 8;       // ≤ 4 КБ через промежуточный буфер
    static constexpr uint32_t DIRECT_CHUNK_BLOCKS = 32;   // ≤ 16 КБ за обращение, если адрес выровнен: стирание быстрее
    static constexpr uint32_t IO_TIMEOUT_MS = 500;
    static constexpr uint32_t READY_TIMEOUT_MS = 1000;   // SD разрешает карте занимать шину до 250 мс на блок

    SD_HandleTypeDef handle = {};
    HAL_SD_CardInfoTypeDef info = {};
    uint32_t blocks = 0;
    uint32_t clockDiv = 0;
    uint32_t initErrorCode = 0;
    bool mayBeBusy = true;   // после записи (и после сбоя) карту нужно дождаться
    alignas(4) uint8_t bounce[CHUNK_BLOCKS * BLOCK_SIZE] = {};   // HAL читает FIFO словами: нужен адрес кратный 4

    bool initAt(uint32_t divider)
    {
        memset(&handle, 0, sizeof(handle));
        handle.Instance = SDMMC1;
        handle.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
        handle.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
        handle.Init.BusWide = SDMMC_BUS_WIDE_1B;
        handle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
        handle.Init.ClockDiv = divider;
        clockDiv = divider;

        if (HAL_SD_Init(&handle) != HAL_OK)
        {
            initErrorCode = handle.ErrorCode ? handle.ErrorCode : 1;
            return false;
        }
        if (HAL_SD_ConfigWideBusOperation(&handle, SDMMC_BUS_WIDE_4B) != HAL_OK)
        {
            initErrorCode = handle.ErrorCode ? handle.ErrorCode : 2;
            return false;
        }
        if (HAL_SD_GetCardInfo(&handle, &info) != HAL_OK || info.LogBlockSize != BLOCK_SIZE || info.LogBlockNbr == 0)
        {
            initErrorCode = 3;
            return false;
        }
        return true;
    }

    bool inRange(uint32_t block, uint32_t count) const
    {
        return blocks != 0 && count != 0 && block < blocks && count <= blocks - block;
    }

    // Дождаться, пока карта вернётся в состояние передачи (после записи
    // она какое-то время "программирует"). После чтения карта уже готова:
    // лишний запрос состояния (CMD13) стоил бы ~0.1 мс на каждое чтение, а
    // сверка при включении читает десятки тысяч секторов.
    bool waitReady()
    {
        if (!mayBeBusy) return true;
        const uint32_t start = millis();
        while (HAL_SD_GetCardState(&handle) != HAL_SD_CARD_TRANSFER)
        {
            if (millis() - start > READY_TIMEOUT_MS) return false;
            Rtos::sleepMs(1);
        }
        mayBeBusy = false;
        return true;
    }

    static bool aligned(const void* p) { return (reinterpret_cast<uintptr_t>(p) & 3u) == 0; }

    // Одно обращение, при сбое — одна повторная попытка.
    bool readChunk(uint32_t block, uint8_t* out, uint32_t n)
    {
        uint8_t* target = aligned(out) ? out : bounce;
        for (uint8_t attempt = 0; attempt < 2; ++attempt)
        {
            if (waitReady() && HAL_SD_ReadBlocks(&handle, target, block, n, IO_TIMEOUT_MS) == HAL_OK)
            {
                if (target != out) memcpy(out, target, n * BLOCK_SIZE);
                readOps++;
                return true;
            }
            mayBeBusy = true;
            errors++;
            if (attempt == 0) retries++;
        }
        return false;
    }

    bool writeChunk(uint32_t block, const uint8_t* in, uint32_t n)
    {
        const uint8_t* source = in;
        if (!aligned(in))
        {
            memcpy(bounce, in, n * BLOCK_SIZE);
            source = bounce;
        }
        for (uint8_t attempt = 0; attempt < 2; ++attempt)
        {
            if (waitReady() && HAL_SD_WriteBlocks(&handle, source, block, n, IO_TIMEOUT_MS) == HAL_OK)
            {
                writeOps++;
                mayBeBusy = true;
                return true;
            }
            mayBeBusy = true;
            errors++;
            if (attempt == 0) retries++;
        }
        return false;
    }
};
