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
#include "drivers/rcc.h"

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#include "build/atomic.h"
#endif

#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
void cameraControlHardwarePwmInit(timerChannel_t *channel, const timerHardware_t *timerHardware, uint8_t inverted)
{
    pwmOutConfig(channel, timerHardware, timerClock(TMR6), CAMERA_CONTROL_PWM_RESOLUTION, 0, inverted);
}
#endif

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
void TIM6_DAC_IRQHandler(void)
{
    cameraControlHi();
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
}

void TIM7_IRQHandler(void)
{
    cameraControlLo();
    tmr_flag_clear(TMR7, TMR_OVF_FLAG);
}

void cameraControlSoftwarePwmInit(void)
{
    nvic_irq_enable(TMR6_DAC_GLOBAL_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    RCC_ClockCmd(RCC_APB1(TMR6), ENABLE);
    tmr_div_value_set(TMR6, 0);

    nvic_irq_enable(TMR7_GLOBAL_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    RCC_ClockCmd(RCC_APB1(TMR7), ENABLE);
    tmr_div_value_set(TMR7, 0);
}

void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    tmr_counter_value_set(TMR6, hiTime);
    tmr_period_value_set(TMR6, period);

    tmr_counter_value_set(TMR7, 0);
    tmr_period_value_set(TMR7, period);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        tmr_counter_enable(TMR6, TRUE);
        tmr_counter_enable(TMR7, TRUE);
    }

    tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);
    tmr_interrupt_enable(TMR7, TMR_OVF_INT, TRUE);
}

void cameraControlSoftwarePwmDisable(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        tmr_counter_enable(TMR6, FALSE);
        tmr_counter_enable(TMR7, FALSE);
    }

    tmr_interrupt_enable(TMR6, TMR_OVF_INT, FALSE);
    tmr_interrupt_enable(TMR7, TMR_OVF_INT, FALSE);
}
#endif

#endif // USE_CAMERA_CONTROL
