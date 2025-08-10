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

#include "platform.h"

#ifdef USE_CAMERA_CONTROL

#include <math.h>

#include "drivers/camera_control_impl.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#include "build/atomic.h"
#endif

#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
void cameraControlHardwarePwmInit(timerChannel_t *channel, const timerHardware_t *timerHardware, uint8_t inverted)
{
    pwmOutConfig(channel, timerHardware, timerClock(timerHardware->tim), CAMERA_CONTROL_PWM_RESOLUTION, 0, inverted);
}
#endif

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
void TIM6_DAC_IRQHandler(void)
{
    cameraControlHi();
    TIM6->SR = 0;
}

void TIM7_IRQHandler(void)
{
    cameraControlLo();
    TIM7->SR = 0;
}

void cameraControlSoftwarePwmInit(void)
{
    NVIC_InitTypeDef nvicTIM6 = {
        TIM6_DAC_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
    };
    NVIC_Init(&nvicTIM6);

    NVIC_InitTypeDef nvicTIM7 = {
        TIM7_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
    };
    NVIC_Init(&nvicTIM7);

    RCC->APB1ENR |= RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7;
    TIM6->PSC = 0;
    TIM7->PSC = 0;
}

void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    TIM6->CNT = hiTime;
    TIM6->ARR = period;

    TIM7->CNT = 0;
    TIM7->ARR = period;

    // Start two timers as simultaneously as possible
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        TIM6->CR1 = TIM_CR1_CEN;
        TIM7->CR1 = TIM_CR1_CEN;
    }

    // Enable interrupt generation
    TIM6->DIER = TIM_IT_Update;
    TIM7->DIER = TIM_IT_Update;
}

void cameraControlSoftwarePwmDisable(void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM6->DIER = 0;
    TIM7->DIER = 0;
}
#endif

#endif // USE_CAMERA_CONTROL
