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

#include "pg/motor.h"
#include "fc/rc_controls.h" // for flight3DConfig_t
#include "config/feature.h"
#include "drivers/pwm_output.h"

#if defined(USE_PWM_OUTPUT) && defined(USE_MOTOR)

FAST_DATA_ZERO_INIT pwmOutputPort_t pwmMotors[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT uint8_t pwmMotorCount;

void analogInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    if (featureIsEnabled(FEATURE_3D)) {
        const float outputLimitOffset = (flight3DConfig()->limit3d_high - flight3DConfig()->limit3d_low) * (1 - outputLimit) / 2;
        *disarm = flight3DConfig()->neutral3d;
        *outputLow = flight3DConfig()->limit3d_low + outputLimitOffset;
        *outputHigh = flight3DConfig()->limit3d_high - outputLimitOffset;
        *deadbandMotor3dHigh = flight3DConfig()->deadband3d_high;
        *deadbandMotor3dLow = flight3DConfig()->deadband3d_low;
    } else {
        *disarm = motorConfig->mincommand;
        const float minThrottle = motorConfig->mincommand + motorConfig->motorIdle * 0.1f;
        *outputLow = minThrottle;
        *outputHigh = motorConfig->maxthrottle - ((motorConfig->maxthrottle - minThrottle) * (1 - outputLimit));
    }
}

IO_t pwmGetMotorIO(unsigned index)
{
    if (index >= pwmMotorCount) {
        return IO_NONE;
    }
    return pwmMotors[index].io;
}

bool pwmIsMotorEnabled(unsigned index)
{
    return pwmMotors[index].enabled;
}

bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return pwmMotorCount > 0;
}

#endif
