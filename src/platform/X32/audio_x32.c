/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/audio.h"


void audioSetupIO(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO, ENABLE);
    /* Enable GTIMA1,DAC1,DMA1 and DMAMUX1 clocks */
    RCC_EnableAPB2PeriphClk1(RCC_APB2_PERIPHEN_M7_GTIMA1, ENABLE);
    RCC_EnableAPB1PeriphClk2(RCC_APB1_PERIPHEN_M7_DAC12, ENABLE);
    /* Config DAC12 prescaler factor,DAC_CLK = 1M*/
    DAC_ConfigClkPrescaler(DAC12,150);

    DAC_SetHighFrequencyMode(DAC12,DAC_HIGH_FREQ_MODE_BELOW_160M);

    /* Configure PA4 DAC1_OUT as analog output -----------------------*/
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin          = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral( GPIOA, &GPIO_InitStructure );
}


void audioGenerateWhiteNoise(void)
{

    DAC_InitType DAC_InitStructure;
    uint16_t  PrescalerValue = 0;
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    RCC_ClocksTypeDef sysclk_value;

    RCC_GetClocksFreqValue(&sysclk_value);
    PrescalerValue = (uint32_t)(sysclk_value.APB2ClkFreq*2 / 16000000) - 1;
    /* Time base configuration */
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period        = 16-1;         
    TIM_TimeBaseStructure.Prescaler     = PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv        = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CounterMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(GTIMA1, &TIM_TimeBaseStructure);

    /* Prescaler configuration */
    TIM_ConfigPrescaler(GTIMA1, PrescalerValue, TIM_PSC_RELOAD_MODE_IMMEDIATE);
    TIM_SelectOutputTrig(GTIMA1,TIM_TRGO_SRC_UPDATE);


    DAC_StructInit(&DAC_InitStructure);
    /* DAC1 Configuration */
    DAC_InitStructure.DAC_Trigger                       = DAC_Trigger_GTIMA1_TRGO;
    DAC_InitStructure.DAC_WaveGeneration                = DAC_WaveGeneration_Noise;
    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude  = DAC_LFSRUnmask_Bits10_0;
    DAC_InitStructure.DAC_DMADoubleDataMode             = DISABLE;
    DAC_InitStructure.DAC_OutputBuffer                  = ENABLE;
    DAC_InitStructure.DAC_ConnectOnChipPeripheral       = DISABLE;
    DAC_InitStructure.DAC_ConnectExternalPin            = ENABLE;
    DAC_InitStructure.DAC_TriggerEnable                 = ENABLE;
    DAC_Init(DAC1, &DAC_InitStructure);

    DAC_SetData(DAC1, DAC_ALIGN_L_12BIT,0xcd00);
    /* Enable DAC1  */
    DAC_Enable(DAC1, ENABLE);
}

#define TONE_SPREAD 8

void audioPlayTone(uint8_t tone)
{
    uint32_t Period = 64 + (MAX(TONE_MIN,MIN(tone, TONE_MAX)) * TONE_SPREAD);
    TIM_SetAutoReload(GTIMA1, Period);
    TIM_Enable(GTIMA1, ENABLE);
}

void audioSilence(void)
{
    DAC_Enable(DAC1, DISABLE);
    TIM_Enable(GTIMA1, DISABLE);
}
