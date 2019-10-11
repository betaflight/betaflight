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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
/*
    { TIM2,   IO_TAG(PB3),  TIM_Channel_2, TIM_USE_PPM,   0, GPIO_AF_1,  NULL, 0 }, // PPM IN
    { TIM3,   IO_TAG(PB0),  TIM_Channel_3, TIM_USE_PWM,   0, GPIO_AF_2,  NULL, 0 }, // SS1 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,   IO_TAG(PB1),  TIM_Channel_4, TIM_USE_PWM,   0, GPIO_AF_2,  NULL, 0 }, // SS1 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM4,   IO_TAG(PB7),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_2,  DMA1_Channel4, DMA1_CH4_HANDLER }, // PWM4 - S1
    { TIM8,   IO_TAG(PB6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_5,  DMA2_Channel3, DMA2_CH3_HANDLER }, // PWM5 - S2
    { TIM17,  IO_TAG(PB5),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_10, DMA1_Channel7, DMA1_CH7_HANDLER }, // PWM6 - S3
    { TIM16,  IO_TAG(PB4),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1,  DMA1_Channel3, DMA1_CH3_HANDLER }, // PWM7 - S4

    { TIM1,   IO_TAG(PA8),  TIM_Channel_1, TIM_USE_LED,   1, GPIO_AF_6,  DMA1_Channel2, DMA1_CH2_HANDLER }, // GPIO TIMER - LED_STRIP
*/
    DEF_TIM(TIM2,  CH2, PB3, TIM_USE_PPM,   0), // PPM IN
    DEF_TIM(TIM3,  CH3, PB0, TIM_USE_PWM,   0), // SS1 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_PWM,   0), // SS1 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    DEF_TIM(TIM4,  CH2, PB7, TIM_USE_MOTOR, 0), // PWM4 - S1
    DEF_TIM(TIM8,  CH1, PB6, TIM_USE_MOTOR, 0), // PWM5 - S2
    DEF_TIM(TIM17, CH1, PB5, TIM_USE_MOTOR, 0), // PWM6 - S3
    DEF_TIM(TIM16, CH1, PB4, TIM_USE_MOTOR, 0), // PWM7 - S4

    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_LED,   0), // GPIO TIMER - LED_STRIP

};
