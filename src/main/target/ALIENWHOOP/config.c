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


/*




         \   |   _ _| __|  \ |\ \      /|  |  _ \  _ \ _ \
        _ \  |     |  _|  .  | \ \ \  / __ | (   |(   |__/
      _/  _\____|___|___|_|\_|  \_/\_/ _| _|\___/\___/_|


              Take me to your leader-board...



*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "common/axis.h"
#include "config/feature.h"
#include "drivers/pwm_esc_detect.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 666           // 666Hz };-)>~ low PWM rate seems to give better power and cooler motors...


void targetConfiguration(void)
{
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfigMutable()->minthrottle = 1080;
    motorConfigMutable()->maxthrottle = 2000;
        pidConfigMutable()->pid_process_denom = 1;
    }

    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
#if defined(ALIENWHOOPF4)
    rxConfigMutable()->sbus_inversion = 0; // TODO: what to do about F4 inversion?
#else
    rxConfigMutable()->sbus_inversion = 1; // invert on F7
#endif

/* Breadboard-specific settings for development purposes only
 */
#if defined(BREADBOARD)
    boardAlignmentMutable()->pitchDegrees = 90; // vertical breakout board
    barometerConfigMutable()->baro_hardware = BARO_DEFAULT; // still testing not on V1 or V2 pcb
#endif

    compassConfigMutable()->mag_hardware =  MAG_DEFAULT;
}
#endif
