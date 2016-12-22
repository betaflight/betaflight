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

#ifdef LED_STRIP

#include "common/color.h"
#include "light_ws2811strip.h"
#include "nvic.h"
#include "dma.h"
#include "io.h"
#include "system.h"
#include "rcc.h"
#include "timer.h"

static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;

static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TimHandle.Instance)
    {
        //HAL_TIM_PWM_Stop_DMA(&TimHandle,WS2811_TIMER_CHANNEL);
        ws2811LedDataTransferInProgress = 0;
    }
}

void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
}

void ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return;
    }

    const timerHardware_t *timerHardware = timerGetByTag(ioTag, TIM_USE_ANY);
    TIM_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;

    if (timerHardware->dmaStream == NULL) {
        return;
    }
    TimHandle.Instance = timer;

    TimHandle.Init.Prescaler = 1;
    TimHandle.Init.Period = 135; // 800kHz
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
    {
        /* Initialization Error */
        return;
    }

    static DMA_HandleTypeDef  hdma_tim;

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();

    /* Set the parameters to be configured */
    hdma_tim.Init.Channel  = timerHardware->dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* Set hdma_tim instance */
    hdma_tim.Instance = timerHardware->dmaStream;

    uint16_t dmaSource = timerDmaSource(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&TimHandle, hdma[dmaSource], hdma_tim);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_LED_STRIP, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaSource);

    /* Initialize TIMx DMA handle */
    if(HAL_DMA_Init(TimHandle.hdma[dmaSource]) != HAL_OK)
    {
        /* Initialization Error */
        return;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK)
    {
        /* Configuration Error */
        return;
    }

    ws2811Initialised = true;
}


void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
    {
        ws2811LedDataTransferInProgress = 0;
        return;
    }

    if (HAL_TIM_PWM_Start_DMA(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK)
    {
        /* Starting PWM generation Error */
        ws2811LedDataTransferInProgress = 0;
        return;
    }

}
#endif
