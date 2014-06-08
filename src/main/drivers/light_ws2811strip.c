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


/*
 * "Note that the timing on the WS2812/WS2812B LEDs has changed as of batches from WorldSemi
 * manufactured made in October 2013, and timing tolerance for approx 10-30% of parts is very small.
 * Recommendation from WorldSemi is now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
 *
 * Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
 */
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#define LED_STRIP_LENGTH 10

static uint8_t LED_BYTE_Buffer[24 * LED_STRIP_LENGTH + 42];
static volatile uint8_t ws2811LedDataTransferInProgress = 0;


void ws2811LedStripInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* GPIOA Configuration: TIM3 Channel 1 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* Compute the prescaler value */
    prescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* configure DMA */
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel6 Config */
    DMA_DeInit(DMA1_Channel6);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR1;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)LED_BYTE_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 42;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    /* TIM3 CC1 DMA Request enable */
    TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);

    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel6_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC6)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel6, DISABLE);            // disable DMA channel 6
        DMA_ClearFlag(DMA1_FLAG_TC6);               // clear DMA1 Channel 6 transfer complete flag
    }
}


/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 *
 * This will result in the RGB triplet passed by argument 1 being sent to
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 *
 * this method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
void ws2812SetStripColors(const uint8_t (*color)[3], uint16_t len)
{
    uint8_t j;
    uint8_t led;
    uint16_t memaddr;
    uint16_t buffersize;

    while(ws2811LedDataTransferInProgress);   // wait until previous transfer completes

    buffersize = (len*24)+42;   // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
    memaddr = 0;                // reset buffer memory index
    led = 0;                    // reset led index

    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    while (len)
    {
        for (j = 0; j < 8; j++)                 // GREEN data
        {
            if ( (color[led][1]<<j) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
            {
                LED_BYTE_Buffer[memaddr] = 17;  // compare value for logical 1
            }
            else
            {
                LED_BYTE_Buffer[memaddr] = 9;   // compare value for logical 0
            }
            memaddr++;
        }

        for (j = 0; j < 8; j++)                 // RED data
        {
            if ( (color[led][0]<<j) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
            {
                LED_BYTE_Buffer[memaddr] = 17;  // compare value for logical 1
            }
            else
            {
                LED_BYTE_Buffer[memaddr] = 9;   // compare value for logical 0
            }
            memaddr++;
        }

        for (j = 0; j < 8; j++)                 // BLUE data
        {
            if ( (color[led][2]<<j) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
            {
                LED_BYTE_Buffer[memaddr] = 17;  // compare value for logical 1
            }
            else
            {
                LED_BYTE_Buffer[memaddr] = 9;   // compare value for logical 0
            }
            memaddr++;
        }

        led++;
        len--;
    }

    // add needed delay at end of byte cycle, pulsewidth = 0
    while(memaddr < buffersize)
    {
        LED_BYTE_Buffer[memaddr] = 0;
        memaddr++;
    }

    ws2811LedDataTransferInProgress = 1;

    DMA_SetCurrDataCounter(DMA1_Channel6, buffersize);  // load number of bytes to be transferred
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);                      // enable Timer 3
    DMA_Cmd(DMA1_Channel6, ENABLE);             // enable DMA channel 6
}


