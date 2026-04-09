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
/*
 * porting for ch32h41x by Temperslee
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
    pwmOutConfig(channel, timerHardware, timerClock(TIM6), CAMERA_CONTROL_PWM_RESOLUTION, 0, inverted);
}
#endif

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
__FAST_INTERRUPT
void TIM6_IRQHandler(void)
{
    cameraControlHi();
    TIM6->INTFR = 0;
}
__FAST_INTERRUPT
void TIM7_IRQHandler(void)
{
    cameraControlLo();
    TIM7->INTFR = 0;
}

void cameraControlSoftwarePwmInit(void)
{
    // NVIC_InitTypeDef nvicTIM6 = {
    //     TIM6_DAC_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
    // };
    // NVIC_Init(&nvicTIM6);

    // NVIC_InitTypeDef nvicTIM7 = {
    //     TIM7_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
    // };
    // NVIC_Init(&nvicTIM7);
    NVIC_SetPriority(TIM6_IRQn,NVIC_PRIO_TIMER);
    NVIC_EnableIRQ(TIM6_IRQn);
    NVIC_SetPriority(TIM7_IRQn,NVIC_PRIO_TIMER);
    NVIC_EnableIRQ(TIM7_IRQn);


    RCC->HB1ENR |= RCC_HB1Periph_TIM6 | RCC_HB1Periph_TIM7;
    TIM6->PSC = 0;
    TIM7->PSC = 0;

}

void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    TIM6->CNT = hiTime;
    TIM6->ATRLR = period;

    TIM7->CNT = 0;
    TIM7->ATRLR = period;

    // Start two timers as simultaneously as possible
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        TIM6->CTLR1 = TIM_CEN;
        TIM7->CTLR1 = TIM_CEN;
    }

    // Enable interrupt generation
    TIM6->DMAINTENR = TIM_IT_Update;
    TIM7->DMAINTENR = TIM_IT_Update;
}

void cameraControlSoftwarePwmDisable(void)
{
    TIM6->CTLR1 &= ~TIM_CEN;
    TIM7->CTLR1 &= ~TIM_CEN;
    TIM6->DIER = 0;
    TIM7->DIER = 0;
}
#endif

#endif // USE_CAMERA_CONTROL
