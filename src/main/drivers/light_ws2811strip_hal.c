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
#include "gpio.h"

#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "nvic.h"

static TIM_HandleTypeDef ledTimHandle;
static DMA_HandleTypeDef ledDmaHandle;

void ws2811LedStripHardwareInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = LED_STRIP_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Alternate = LED_STRIP_AF;
    HAL_GPIO_Init(LED_STRIP_GPIO, &GPIO_InitStructure);


    ledTimHandle.Instance = LED_STRIP_TIMER;
    // Stop timer
    HAL_TIM_Base_Stop(&ledTimHandle);
    
    ledTimHandle.Init.Prescaler = (uint16_t) (SystemCoreClock / 2 / 84000000) - 1;
    ledTimHandle.Init.Period = 104; // 800kHz
    ledTimHandle.Init.ClockDivision = 0;
    ledTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    
    HAL_TIM_PWM_Init(&ledTimHandle);

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&ledTimHandle, &TIM_OCInitStructure, LED_STRIP_TIMER_CHANNEL);
    HAL_TIM_Base_Start(&ledTimHandle);

    /* Set the parameters to be configured */
    ledDmaHandle.Instance = LED_STRIP_DMA_STREAM;
    ledDmaHandle.Init.Channel = LED_STRIP_DMA_CHANNEL;
    ledDmaHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    ledDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    ledDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    ledDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
    ledDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
    ledDmaHandle.Init.Mode = DMA_NORMAL;
    ledDmaHandle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    ledDmaHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    ledDmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    ledDmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    ledDmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
     __HAL_LINKDMA(&ledTimHandle, hdma[LED_STRIP_TIMER_DMA_RQ], ledDmaHandle);
     
     /* Initialize TIMx DMA handle */
     HAL_DMA_Init(ledTimHandle.hdma[LED_STRIP_TIMER_DMA_RQ]);
     
     HAL_NVIC_SetPriority(LED_STRIP_DMA_STREAM_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_WS2811_DMA), NVIC_PRIORITY_SUB(NVIC_PRIO_WS2811_DMA));
     HAL_NVIC_EnableIRQ(LED_STRIP_DMA_STREAM_IRQn);

//    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM5->CCR1);
//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
//    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;


    setStripColor(&hsv_white);
    ws2811UpdateStrip();
}

void LED_STRIP_DMA_IRQHandler(void)
{
    HAL_DMA_IRQHandler(ledTimHandle.hdma[LED_STRIP_TIMER_DMA_RQ]);
    ws2811LedDataTransferInProgress = 0;
}

void ws2811LedStripDMAEnable(void)
{
    __HAL_TIM_SET_COUNTER(&ledTimHandle, 0);
    HAL_TIM_PWM_Start_DMA(&ledTimHandle, LED_STRIP_TIMER_CHANNEL, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE);
}

