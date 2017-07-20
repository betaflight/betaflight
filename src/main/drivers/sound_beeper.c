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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"

#include "sound_beeper.h"


#ifdef BEEPER

static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
static bool beeperState = false;

#endif

void systemBeep(bool onoff)
{
#if !defined(BEEPER)
    UNUSED(onoff);
#elif defined(BEEPER_PWM)
    pwmWriteBeeper(onoff);
    beeperState = onoff;
#else
    IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    beeperState = onoff;
#endif
}

void systemBeepToggle(void)
{
#if defined(BEEPER)
    systemBeep(!beeperState);
#endif
}

void beeperInit(const beeperDevConfig_t *config)
{
#if !defined(BEEPER)
    UNUSED(config);
#else
    beeperIO = IOGetByTag(config->ioTag);
    beeperInverted = config->isInverted;

    if (beeperIO) {
        IOInit(beeperIO, OWNER_BEEPER, RESOURCE_OUTPUT, 0);
#if defined(BEEPER_PWM)
        beeperPwmInit(config->ioTag, BEEPER_PWM_FREQUENCY);
#else
        IOConfigGPIO(beeperIO, config->isOD ? IOCFG_OUT_OD : IOCFG_OUT_PP);
#endif
    }

    systemBeep(false);
#endif
}
