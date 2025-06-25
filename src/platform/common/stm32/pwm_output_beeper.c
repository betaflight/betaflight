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

#if defined(USE_BEEPER) && defined(USE_PWM_OUTPUT)

#include "drivers/pwm_output.h"

static pwmOutputPort_t beeperPwm;
static uint16_t freqBeep = 0;

void pwmWriteBeeper(bool on)
{
    if (!beeperPwm.io) {
        return;
    }

    if (on) {
        *beeperPwm.channel.ccr = (PWM_TIMER_1MHZ / freqBeep) / 2;
        beeperPwm.enabled = true;
    } else {
        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
    }
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperPwm.enabled);
}

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    const timerHardware_t *timer = timerAllocate(tag, OWNER_BEEPER, 0);
    IO_t beeperIO = IOGetByTag(tag);

    if (beeperIO && timer) {
        beeperPwm.io = beeperIO;
        IOInit(beeperPwm.io, OWNER_BEEPER, 0);
        IOConfigGPIOAF(beeperPwm.io, IOCFG_AF_PP, timer->alternateFunction);
        freqBeep = frequency;
        pwmOutConfig(&beeperPwm.channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / freqBeep, (PWM_TIMER_1MHZ / freqBeep) / 2, 0);

        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
    }
}

#endif
