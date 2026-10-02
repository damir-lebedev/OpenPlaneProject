#pragma once

// ============================================================
// Нативная замена HAL_SD (STM32H7) и сама "карта" для тестов.
//
//   fake::SdCardModel — SD-карта в памяти: блоки по 512 байт, сбои
//     по счётчику (чтение, запись, инициализация), "пропажа питания"
//     (все записи после N-й молча теряются), занятость карты после
//     записи (HAL_SD_GetCardState отвечает PROGRAMMING N раз подряд).
//     Это же IBlockDevice — тесты области и FAT берут её напрямую.
//   HAL_SD_* — ровно то подмножество, которое использует
//     Stm32SdCard: Init, ConfigWideBusOperation, GetCardInfo,
//     ReadBlocks, WriteBlocks, GetCardState. Они ходят в
//     fake::sdCard() — единственную "карту в слоте".
// ============================================================

#include <stdint.h>
#include <string.h>

#include <vector>

#include "hal/IBlockDevice.h"

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

#define HAL_SD_MODULE_ENABLED

struct SD_TypeDef
{
    int instance;
};

namespace fake
{
    inline SD_TypeDef& sdmmc1Instance()
    {
        static SD_TypeDef instance = { 1 };
        return instance;
    }
}
#define SDMMC1 (&::fake::sdmmc1Instance())

#define SDMMC_CLOCK_EDGE_RISING 0x00000000U
#define SDMMC_CLOCK_POWER_SAVE_DISABLE 0x00000000U
#define SDMMC_BUS_WIDE_1B 0x00000000U
#define SDMMC_BUS_WIDE_4B 0x00000001U
#define SDMMC_HARDWARE_FLOW_CONTROL_DISABLE 0x00000000U
#define SDMMC_HARDWARE_FLOW_CONTROL_ENABLE 0x00004000U

typedef struct
{
    uint32_t ClockEdge;
    uint32_t ClockPowerSave;
    uint32_t BusWide;
    uint32_t HardwareFlowControl;
    uint32_t ClockDiv;
} SDMMC_InitTypeDef;

typedef struct
{
    SD_TypeDef* Instance;
    SDMMC_InitTypeDef Init;
    uint32_t ErrorCode;
} SD_HandleTypeDef;

typedef struct
{
    uint32_t CardType;
    uint32_t CardVersion;
    uint32_t Class;
    uint32_t RelCardAdd;
    uint32_t BlockNbr;
    uint32_t BlockSize;
    uint32_t LogBlockNbr;
    uint32_t LogBlockSize;
    uint32_t CardSpeed;
} HAL_SD_CardInfoTypeDef;

typedef uint32_t HAL_SD_CardStateTypeDef;
#define HAL_SD_CARD_READY 0x00000001U
#define HAL_SD_CARD_TRANSFER 0x00000004U
#define HAL_SD_CARD_PROGRAMMING 0x00000007U

namespace fake
{
    class SdCardModel : public IBlockDevice
    {
    public:
        static constexpr uint32_t NEVER = 0xFFFFFFFFu;

        // Содержимое карты. Новая карта — вся из нулей, как после форматирования
        // (SD не обязана отдавать 0xFF).
        std::vector<uint8_t> data;
        bool present = true;

        // Сбои: сколько ближайших обращений провалить.
        uint32_t failInit = 0;
        uint32_t failReads = 0;
        uint32_t failWrites = 0;
        // Скорость шины, на которой карта "перестаёт держать": ClockDiv меньше
        // этого — чтение сбоит (HAL_ERROR).
        uint32_t minWorkingClockDiv = 0;

        // Пропажа питания: после этого числа записанных блоков все следующие
        // записи теряются молча (HAL отвечает OK — снаружи не видно).
        uint32_t cutPowerAfterBlocks = NEVER;
        uint32_t lostBlocks = 0;

        // После каждой записи карта "программирует" столько опросов состояния.
        uint32_t busyPollsAfterWrite = 0;
        uint32_t busyPolls = 0;

        // Счётчики.
        uint32_t readCalls = 0;
        uint32_t writeCalls = 0;
        uint32_t blocksRead = 0;
        uint32_t blocksWritten = 0;
        uint32_t maxBlocksPerCall = 0;
        uint32_t initCalls = 0;
        uint32_t lastClockDiv = 0;
        uint32_t lastBusWide = 0;
        uint32_t unalignedCalls = 0;   // обращения с адресом не кратным 4: настоящий HAL читает FIFO словами

        explicit SdCardModel(uint32_t blocks = 0) { resize(blocks); }

        void resize(uint32_t blocks) { data.assign(static_cast<size_t>(blocks) * BLOCK_SIZE, 0); }

        uint32_t blockCount() const override
        {
            return present ? static_cast<uint32_t>(data.size() / BLOCK_SIZE) : 0;
        }

        bool read(uint32_t block, void* out, uint32_t count) override
        {
            readCalls++;
            if (!present || block >= data.size() / BLOCK_SIZE || count > data.size() / BLOCK_SIZE - block) return false;
            if (failReads > 0)
            {
                failReads--;
                return false;
            }
            if (lastClockDiv != 0 && lastClockDiv < minWorkingClockDiv) return false;
            memcpy(out, data.data() + static_cast<size_t>(block) * BLOCK_SIZE, static_cast<size_t>(count) * BLOCK_SIZE);
            blocksRead += count;
            if (count > maxBlocksPerCall) maxBlocksPerCall = count;
            return true;
        }

        bool write(uint32_t block, const void* in, uint32_t count) override
        {
            writeCalls++;
            if (!present || block >= data.size() / BLOCK_SIZE || count > data.size() / BLOCK_SIZE - block) return false;
            if (failWrites > 0)
            {
                failWrites--;
                return false;
            }
            const uint8_t* src = static_cast<const uint8_t*>(in);
            for (uint32_t i = 0; i < count; ++i)
            {
                if (blocksWritten >= cutPowerAfterBlocks)
                {
                    lostBlocks++;
                }
                else
                {
                    memcpy(data.data() + static_cast<size_t>(block + i) * BLOCK_SIZE, src + static_cast<size_t>(i) * BLOCK_SIZE,
                           BLOCK_SIZE);
                    blocksWritten++;
                }
            }
            if (count > maxBlocksPerCall) maxBlocksPerCall = count;
            busyPolls = busyPollsAfterWrite;
            return true;
        }

        // Содержимое блока для проверок в тестах.
        const uint8_t* block(uint32_t index) const { return data.data() + static_cast<size_t>(index) * BLOCK_SIZE; }
    };

    // Карта в слоте: одна на весь тестовый мир.
    inline SdCardModel& sdCard()
    {
        static SdCardModel card;
        return card;
    }

    inline void resetSdCard()
    {
        sdCard() = SdCardModel();
    }
}

inline HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef* hsd)
{
    fake::SdCardModel& card = fake::sdCard();
    card.initCalls++;
    card.lastClockDiv = hsd->Init.ClockDiv;
    card.lastBusWide = hsd->Init.BusWide;
    if (!card.present || card.data.empty())
    {
        hsd->ErrorCode = 0x10;
        return HAL_ERROR;
    }
    if (card.failInit > 0)
    {
        card.failInit--;
        hsd->ErrorCode = 0x20;
        return HAL_ERROR;
    }
    hsd->ErrorCode = 0;
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef* hsd, uint32_t wideMode)
{
    hsd->Init.BusWide = wideMode;
    fake::sdCard().lastBusWide = wideMode;
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef*, HAL_SD_CardInfoTypeDef* info)
{
    const uint32_t blocks = fake::sdCard().blockCount();
    memset(info, 0, sizeof(*info));
    info->CardType = 1;   // SDHC
    info->BlockNbr = blocks;
    info->BlockSize = IBlockDevice::BLOCK_SIZE;
    info->LogBlockNbr = blocks;
    info->LogBlockSize = IBlockDevice::BLOCK_SIZE;
    return HAL_OK;
}

inline HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef*)
{
    fake::SdCardModel& card = fake::sdCard();
    if (!card.present) return 0xFF;
    if (card.busyPolls > 0)
    {
        card.busyPolls--;
        return HAL_SD_CARD_PROGRAMMING;
    }
    return HAL_SD_CARD_TRANSFER;
}

inline HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef* hsd, uint8_t* data, uint32_t block, uint32_t count,
                                           uint32_t /*timeout*/)
{
    fake::SdCardModel& card = fake::sdCard();
    if ((reinterpret_cast<uintptr_t>(data) & 3u) != 0) card.unalignedCalls++;
    // Скорость, на которой карта не держит, ломает передачу данных, а не опознание.
    card.lastClockDiv = hsd->Init.ClockDiv;
    if (!card.read(block, data, count))
    {
        hsd->ErrorCode = 0x40;
        return HAL_ERROR;
    }
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef* hsd, const uint8_t* data, uint32_t block, uint32_t count,
                                            uint32_t /*timeout*/)
{
    fake::SdCardModel& card = fake::sdCard();
    if ((reinterpret_cast<uintptr_t>(data) & 3u) != 0) card.unalignedCalls++;
    if (!card.write(block, data, count))
    {
        hsd->ErrorCode = 0x80;
        return HAL_ERROR;
    }
    return HAL_OK;
}
