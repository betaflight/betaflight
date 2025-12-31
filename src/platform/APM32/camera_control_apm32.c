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

#include "platform/camera_control.h"
#include "drivers/nvic.h"

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
void TMR6_DAC_IRQHandler(void)
{
    cameraControlHi();
    TMR6->STS = 0;
}

void TMR7_IRQHandler(void)
{
    cameraControlLo();
    TMR7->STS = 0;
}

void cameraControlSoftwarePwmInit(void)
{
    DAL_NVIC_SetPriority(TMR6_DAC_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    DAL_NVIC_EnableIRQ(TMR6_DAC_IRQn);

    DAL_NVIC_SetPriority(TMR7_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    DAL_NVIC_EnableIRQ(TMR7_IRQn);

    __DAL_RCM_TMR6_CLK_ENABLE();
    __DAL_RCM_TMR7_CLK_ENABLE();
    DDL_TMR_SetPrescaler(TMR6, 0);
    DDL_TMR_SetPrescaler(TMR7, 0);
}

void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    DDL_TMR_SetCounter(TMR6, hiTime);
    DDL_TMR_SetAutoReload(TMR6, period);

    DDL_TMR_SetCounter(TMR7, 0);
    DDL_TMR_SetAutoReload(TMR7, period);

    // Start two timers as simultaneously as possible
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        DDL_TMR_EnableCounter(TMR6);
        DDL_TMR_EnableCounter(TMR7);
    }

    // Enable interrupt generation
    DDL_TMR_EnableIT_UPDATE(TMR6);
    DDL_TMR_EnableIT_UPDATE(TMR7);
}

void cameraControlSoftwarePwmDisable(void)
{
    DDL_TMR_DisableCounter(TMR6);
    DDL_TMR_DisableCounter(TMR7);

    TMR6->DIEN = 0;
    TMR7->DIEN = 0;
}
#endif

#endif // USE_CAMERA_CONTROL
