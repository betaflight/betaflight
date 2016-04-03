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
#include <string.h>

#include <platform.h>

#include "debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"

#include "motor_and_servo.h"

PG_REGISTER_WITH_RESET(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);

void pgReset_motorAndServoConfig(motorAndServoConfig_t *motorAndServoConfig)
{
    RESET_CONFIG(motorAndServoConfig_t, motorAndServoConfig,
        .minthrottle = 1150,
        .maxthrottle = 1850,
        .mincommand = 1000,
        .servoCenterPulse = 1500,
#ifdef BRUSHED_MOTORS
        .motor_pwm_rate = BRUSHED_MOTORS_PWM_RATE;
#else
        .motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
        .servo_pwm_rate = 50;
    );

}
