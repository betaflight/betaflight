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

#include <stdint.h>

#include "platform.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/pwm_output.h"

#include "io/motors.h"

#ifdef BRUSHED_MOTORS
#define DEFAULT_PWM_PROTOCOL    PWM_TYPE_BRUSHED
#define DEFAULT_PWM_RATE        16000
#define DEFAULT_MIN_THROTTLE    1000
#else
#define DEFAULT_PWM_PROTOCOL    PWM_TYPE_STANDARD
#define DEFAULT_PWM_RATE        400
#define DEFAULT_MIN_THROTTLE    1150
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .minthrottle = DEFAULT_MIN_THROTTLE,
    .motorPwmProtocol = DEFAULT_PWM_PROTOCOL,
    .motorPwmRate = DEFAULT_PWM_RATE,
    .maxthrottle = 1850,
    .mincommand = 1000
);
