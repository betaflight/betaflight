/*
 * Minimal HAL MSP for the FSBL stub. Only XSPI2's MSP is non-trivial: it
 * enables the peripheral + XSPIM + GPION clocks and configures the eleven
 * GPIO N pins for XSPIM_P2. The boot ROM already picked a working XSPI2
 * clock source; we don't reconfigure it. 1S-1S-1S READ at the boot-ROM rate
 * is sufficient for XIP and works regardless of HSLV fuse state, so no BSEC
 * read is needed.
 */

#include "main.h"

void HAL_MspInit(void)
{
    HAL_PWREx_EnableVddIO3();
}

void HAL_XSPI_MspInit(XSPI_HandleTypeDef *hxspi)
{
    if (hxspi->Instance != XSPI2) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PWR clock is left on by the boot ROM. __HAL_RCC_PWR_CLK_ENABLE
     * lives behind CPU_IN_SECURE_STATE in stm32n6xx_hal_rcc.h, which pulls
     * in HAL_MPU_*_NS variants that require -mcmse. */
    HAL_PWREx_EnableVddIO3();
    HAL_PWREx_ConfigVddIORange(PWR_VDDIO3, PWR_VDDIO_RANGE_1V8);

    __HAL_RCC_XSPIM_CLK_ENABLE();
    __HAL_RCC_XSPI2_CLK_ENABLE();
    __HAL_RCC_GPION_CLK_ENABLE();

    /* PN0 -> DQS0, PN1 -> NCS1, PN2..PN5 -> IO0..IO3, PN6 -> CLK,
     * PN8..PN11 -> IO4..IO7. AF9 = XSPIM_P2 on every pin in this bank. */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                        | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8
                        | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P2;
    HAL_GPIO_Init(GPION, &GPIO_InitStruct);
}
