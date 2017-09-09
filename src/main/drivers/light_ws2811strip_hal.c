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
#include "drivers/nvic.h"
#include "dma.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "rcc.h"
#include "timer.h"

#if !defined(WS2811_PIN)
#define WS2811_PIN                      PA0
#define WS2811_TIMER                    TIM5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_TIMER_CHANNEL            TIM_Channel_1
#define WS2811_TIMER_GPIO_AF            GPIO_AF2_TIM5
#endif

static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;

static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;
static bool timerNChannel = false;

void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    if(timerNChannel) {
        HAL_TIMEx_PWMN_Stop_DMA(&TimHandle,timerChannel);
    } else {
        HAL_TIM_PWM_Stop_DMA(&TimHandle,timerChannel);
    }
    ws2811LedDataTransferInProgress = 0;
}

void ws2811LedStripHardwareInit(void)
{
    const timerHardware_t *timerHardware = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);
    TIM_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;

    if (timerHardware == NULL) {
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

    ws2811IO = IOGetByTag(IO_TAG(WS2811_PIN));
    IOInit(ws2811IO, OWNER_LED_STRIP, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();

    /* Set the parameters to be configured */
    hdma_tim.Init.Channel = WS2811_DMA_CHANNEL;
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
    hdma_tim.Instance = WS2811_DMA_STREAM;

    uint16_t dmaSource = timerDmaSource(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&TimHandle, hdma[dmaSource], hdma_tim);

    dmaInit(WS2811_DMA_HANDLER_IDENTIFER, OWNER_LED_STRIP, 0);
    dmaSetHandler(WS2811_DMA_HANDLER_IDENTIFER, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaSource);

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(TimHandle.hdma[dmaSource]) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        timerNChannel = true;
    }
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW: TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;

    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return;
    }

    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised) {
        ws2811LedDataTransferInProgress = 0;
        return;
    }

    if (timerNChannel) {
        if (HAL_TIMEx_PWMN_Start_DMA(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
            /* Starting PWM generation Error */
            ws2811LedDataTransferInProgress = 0;
            return;
        }
    } else {
        if (HAL_TIM_PWM_Start_DMA(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
            /* Starting PWM generation Error */
            ws2811LedDataTransferInProgress = 0;
            return;
        }
    }
}
#endif
