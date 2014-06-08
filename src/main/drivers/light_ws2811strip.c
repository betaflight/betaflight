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

#include "drivers/light_ws2811strip.h"

#define LED_STRIP_LENGTH 10
#define BITS_PER_LED 24
#define DELAY_BUFFER_LENGTH 42 // for 50us delay

#define DMA_BUFFER_SIZE (BITS_PER_LED * LED_STRIP_LENGTH + DELAY_BUFFER_LENGTH)   // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)

static uint8_t ledStripDMABuffer[DMA_BUFFER_SIZE];
static volatile uint8_t ws2811LedDataTransferInProgress = 0;

static rgbColor24bpp_t ledColorBuffer[LED_STRIP_LENGTH];

const rgbColor24bpp_t black = { {0, 0, 0} };
const rgbColor24bpp_t orange = { {255, 128, 0} };
const rgbColor24bpp_t white = { {255, 255, 255} };

void setLedColor(uint16_t index, const rgbColor24bpp_t *color)
{
    ledColorBuffer[index].rgb = color->rgb;
}

void setStripColor(const rgbColor24bpp_t *color)
{
    uint16_t index;
    for (index = 0; index < LED_STRIP_LENGTH; index++) {
        setLedColor(index, color);
    }
}

void setStripColors(const rgbColor24bpp_t *colors)
{
    uint16_t index;
    for (index = 0; index < LED_STRIP_LENGTH; index++) {
        setLedColor(index, colors++);
    }
}

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
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
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

    setStripColor(&white);
    ws2811UpdateStrip();
}

void DMA1_Channel6_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC6)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel6, DISABLE);            // disable DMA channel 6
        DMA_ClearFlag(DMA1_FLAG_TC6);               // clear DMA1 Channel 6 transfer complete flag
    }
}

bool isWS2811LedStripReady(void)
{
    return !ws2811LedDataTransferInProgress;
}

static uint16_t dmaBufferOffset;
static int16_t ledIndex;

void updateLEDDMABuffer(uint8_t componentValue)
{
    uint8_t bitIndex;
    
    for (bitIndex = 0; bitIndex < 8; bitIndex++)
    {
        if ((componentValue << bitIndex) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
        {
            ledStripDMABuffer[dmaBufferOffset] = 17;  // compare value for logical 1
        }
        else
        {
            ledStripDMABuffer[dmaBufferOffset] = 9;   // compare value for logical 0
        }
        dmaBufferOffset++;
    }
}

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
void ws2811UpdateStrip(void)
{
    static uint32_t waitCounter = 0;
    // wait until previous transfer completes
    while(ws2811LedDataTransferInProgress) {
        waitCounter++;
    }

    dmaBufferOffset = 0;                // reset buffer memory index
    ledIndex = 0;                    // reset led index

    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    while (ledIndex < LED_STRIP_LENGTH)
    {
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.g);
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.r);
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.b);

        ledIndex++;
    }

    // add needed delay at end of byte cycle, pulsewidth = 0
    while(dmaBufferOffset < DMA_BUFFER_SIZE)
    {
        ledStripDMABuffer[dmaBufferOffset] = 0;
        dmaBufferOffset++;
    }

    ws2811LedDataTransferInProgress = 1;

    DMA_SetCurrDataCounter(DMA1_Channel6, DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
    DMA_Cmd(DMA1_Channel6, ENABLE);
}


