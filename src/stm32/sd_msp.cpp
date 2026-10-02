// ============================================================
// Выводы и тактирование SDMMC1 для SD-карты (hal/stm32/Stm32SdCard.h).
//
// HAL_SD_Init() сам зовёт HAL_SD_MspInit() — слабая функция HAL, здесь
// её настоящее тело. На платах DevEBox H743 и WeAct MiniSTM32H743 слот
// µSD подключён одинаково:
//   PC8..PC11 — D0..D3, PC12 — CK, PD2 — CMD (все AF12 = SDMMC1).
// Подтяжки — на D0..D3 и CMD (так требует режим 4 бита и холостая шина);
// на CK подтяжка не нужна. Ядро SDMMC1 тактируется от PLL1Q: выбор
// источника делает SystemClock_Config() варианта платы.
//
// Отдельный .cpp (а не заголовок): определение функции с внешней связью,
// которое подхватывает HAL при компоновке. Нативные тесты его не собирают —
// там HAL_SD_* подменён фейком (test/native/support_stm32).
// ============================================================

#include <Arduino.h>

#if defined(BOARD_STM32H743) && defined(HAL_SD_MODULE_ENABLED)

extern "C" void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
    if (hsd->Instance != SDMMC1) return;

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef pins = {};
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pins.Alternate = GPIO_AF12_SDIO1;

    pins.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;   // D0..D3
    pins.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &pins);

    pins.Pin = GPIO_PIN_12;                                            // CK
    pins.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &pins);

    pins.Pin = GPIO_PIN_2;                                             // CMD
    pins.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &pins);
}

extern "C" void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
    if (hsd->Instance != SDMMC1) return;

    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}

#endif
