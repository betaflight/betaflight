/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"
#include "light_ws2811strip.h"
#include "drivers/nvic.h"
#include "dma.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "rcc.h"
#include "timer.h"

static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;

static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;

void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
    ws2811LedDataTransferInProgress = 0;
}

void ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return;
    }

    const timerHardware_t *timerHardware = timerGetByTag(ioTag, TIM_USE_ANY);
    TIM_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;

    if (timerHardware->dmaRef == NULL) {
        return;
    }
    TimHandle.Instance = timer;

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    TimHandle.Init.Prescaler = prescaler;
    TimHandle.Init.Period = period; // 800kHz
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    static DMA_HandleTypeDef hdma_tim;

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();

    /* Set the parameters to be configured */
    hdma_tim.Init.Channel = timerHardware->dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* Set hdma_tim instance */
    hdma_tim.Instance = timerHardware->dmaRef;

    uint16_t dmaIndex = timerDmaIndex(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&TimHandle, hdma[dmaIndex], hdma_tim);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_LED_STRIP, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(TimHandle.hdma[dmaIndex]) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (HAL_TIMEx_PWMN_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    } else {
        if (HAL_TIM_PWM_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    }
    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised) {
        ws2811LedDataTransferInProgress = 0;
        return;
    }

    if (DMA_SetCurrDataCounter(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = 0;
        return;
    }
    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(&TimHandle,0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TimHandle,timerChannel,ENABLE);
}
#endif
