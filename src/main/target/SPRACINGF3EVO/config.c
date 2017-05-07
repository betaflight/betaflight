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

#include <platform.h>

#ifdef TARGET_CONFIG

#include "common/axis.h"

#include "fc/config.h"

#include "flight/mixer.h"
#include "flight/pid.h"


#if defined(SPRACINGF3MQ)
#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz
#endif

void targetConfiguration(void)
{
    // Temporary workaround: Disable SDCard DMA by default since it causes errors on this target
    sdcardConfigMutable()->useDma = false;

#if defined(SPRACINGF3MQ)

    motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;

    pidProfilesMutable(0)->P8[FD_ROLL] = 90;
    pidProfilesMutable(0)->I8[FD_ROLL] = 44;
    pidProfilesMutable(0)->D8[FD_ROLL] = 60;
    pidProfilesMutable(0)->P8[FD_PITCH] = 90;
    pidProfilesMutable(0)->I8[FD_PITCH] = 44;
    pidProfilesMutable(0)->D8[FD_PITCH] = 60;
#endif
}
#endif
