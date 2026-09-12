/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "platform/timer.h"

#ifdef USE_HAL_DRIVER

void timerChannelEnable(const timerHardware_t *timHw)
{
    TIM_CCxChannelCmd((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CCx_ENABLE);
}

void timerChannelDisable(const timerHardware_t *timHw)
{
    TIM_CCxChannelCmd((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CCx_DISABLE);
}

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)

void pwmICConfig(timerResource_t *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    TIM_HandleTypeDef *Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = (polarity == TIMER_POLARITY_RISING) ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = filter;

    HAL_TIM_IC_ConfigChannel(Handle, &TIM_ICInitStructure, channel);
    HAL_TIM_IC_Start_IT(Handle, channel);
}

#endif

#elif defined(USE_GDBSP_DRIVER)

extern void gd32_timer_input_capture_config(void* timer, uint16_t channel, uint8_t state);

void timerChannelEnable(const timerHardware_t *timHw)
{
    gd32_timer_input_capture_config((TIM_TypeDef *)timHw->tim, timHw->channel, ENABLE);
}

void timerChannelDisable(const timerHardware_t *timHw)
{
    gd32_timer_input_capture_config((TIM_TypeDef *)timHw->tim, timHw->channel, DISABLE);
}

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)

void pwmICConfig(void *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    timer_ic_parameter_struct timer_icinitpara;

    timer_channel_input_struct_para_init(&timer_icinitpara);
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icpolarity =  (polarity == TIMER_POLARITY_RISING) ? TIMER_IC_POLARITY_RISING : TIMER_IC_POLARITY_FALLING;
    timer_icinitpara.icfilter = filter;

    timer_input_capture_config((uint32_t)tim, channel, &timer_icinitpara);
}

#endif

#else

void timerChannelEnable(const timerHardware_t *timHw)
{
    TIM_CCxCmd((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CCx_Enable);
}

void timerChannelDisable(const timerHardware_t *timHw)
{
    TIM_CCxCmd((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CCx_Disable);
}

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)

void pwmICConfig(timerResource_t *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = (polarity == TIMER_POLARITY_RISING) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = filter;

    TIM_ICInit((TIM_TypeDef *)tim, &TIM_ICInitStructure);
}

#endif

#endif
