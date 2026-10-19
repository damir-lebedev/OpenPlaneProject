// ============================================================
// Такты Matek H743 (env matek-h743): кварц 8 МГц вместо 25 МГц у DevEBox.
//
// Сборка идёт с вариантом ядра DevEBox (тот же корпус STM32H743VI), а у
// него SystemClock_Config() — WEAK под кварц 25 МГц. Эта функция её
// заменяет и даёт то же дерево частот, что у DevEBox, — всё остальное
// (USB, SDMMC, UART, таймеры ШИМ) работает как там:
//   PLL1: 8 МГц / 1 × 60 = 480 МГц — ядро; Q / 10 = 48 МГц — USB и SDMMC;
//   PLL2, PLL3: 8 МГц / 5 × 100 / 2 = 80 МГц — АЦП, UART, SPI, I2C.
// HSE_VALUE = 8 МГц задаёт platformio.ini.
// ============================================================

#if defined(BOARD_MATEK_H743) && defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

static_assert(HSE_VALUE == 8000000UL, "Matek H743: кварц 8 МГц — флаг -D HSE_VALUE=8000000UL в platformio.ini");

extern "C" void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};
    RCC_PeriphCLKInitTypeDef periph = {};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);   // 480 МГц — только VOS0
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 1;                         // 8 МГц на входе PLL1
    osc.PLL.PLLN = 60;                        // VCO 480 МГц
    osc.PLL.PLLP = 1;                         // ядро 480 МГц (как у DevEBox)
    osc.PLL.PLLQ = 10;                        // 48 МГц: USB, SDMMC
    osc.PLL.PLLR = 10;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;      // вход 8–16 МГц
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                    RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) Error_Handler();

    periph.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_QSPI | RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_ADC |
                                  RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578 |
                                  RCC_PERIPHCLK_I2C123 | RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_SPI123 |
                                  RCC_PERIPHCLK_SPI45 | RCC_PERIPHCLK_SPI6;

    // PLL2 и PLL3: 8 / 5 = 1.6 МГц × 100 = 160 МГц, P/Q/R / 2 = 80 МГц.
    periph.PLL2.PLL2M = 5;
    periph.PLL2.PLL2N = 100;
    periph.PLL2.PLL2P = 2;
    periph.PLL2.PLL2Q = 2;
    periph.PLL2.PLL2R = 2;
    periph.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;  // вход 1–2 МГц
    periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    periph.PLL2.PLL2FRACN = 0;
    periph.PLL3.PLL3M = 5;
    periph.PLL3.PLL3N = 100;
    periph.PLL3.PLL3P = 2;
    periph.PLL3.PLL3Q = 2;
    periph.PLL3.PLL3R = 2;
    periph.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
    periph.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
    periph.PLL3.PLL3FRACN = 0;

    periph.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    periph.QspiClockSelection = RCC_QSPICLKSOURCE_PLL;
    periph.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
    periph.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PLL2;
    periph.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL2;
    periph.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
    periph.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;
    periph.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PLL3;
    periph.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    periph.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
    periph.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) Error_Handler();
}

#endif
