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
#include "platform/rcc.h"

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
void TIMER5_DAC_IRQHandler(void)
{
    cameraControlHi();
    TIMER_INTF(TIMER5) = 0;
}

void TIMER6_IRQHandler(void)
{
    cameraControlLo();
    TIMER_INTF(TIMER6) = 0;
}

void cameraControlSoftwarePwmInit(void)
{
    nvic_irq_enable(TIMER5_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));  
    rcu_periph_clock_enable(RCU_TIMER5);
    TIMER_PSC(TIMER5) = 0;

    nvic_irq_enable(TIMER6_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));  
    rcu_periph_clock_enable(RCU_TIMER6);
    TIMER_PSC(TIMER6) = 0;  
}

void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    timer_counter_value_config(TIMER5, hiTime);
    timer_autoreload_value_config(TIMER5, period);

    timer_counter_value_config(TIMER6, 0);
    timer_autoreload_value_config(TIMER6, period);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) { 
        timer_enable(TIMER5);
        timer_enable(TIMER6);
    }

    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);
}

void cameraControlSoftwarePwmDisable(void)
{
    timer_disable(TIMER5);
    timer_disable(TIMER6);

    TIMER_DMAINTEN(TIMER5) = 0;
    TIMER_DMAINTEN(TIMER6) = 0;
}
#endif

#endif // USE_CAMERA_CONTROL
