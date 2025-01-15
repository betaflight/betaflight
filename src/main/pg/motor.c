/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_MOTOR

#include "drivers/motor_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

#if !defined(DEFAULT_DSHOT_BITBANG)
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_AUTO
#endif

#if !defined(DSHOT_BITBANGED_TIMER_DEFAULT)
#define DSHOT_BITBANGED_TIMER_DEFAULT DSHOT_BITBANGED_TIMER_AUTO
#endif

#if !defined(DEFAULT_DSHOT_BURST)
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
#endif

#if !defined(DEFAULT_DSHOT_TELEMETRY)
#define DEFAULT_DSHOT_TELEMETRY DSHOT_TELEMETRY_OFF
#endif

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 3);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
#ifdef USE_BRUSHED_MOTORS
    motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_BRUSHED;
    motorConfig->dev.useUnsyncedUpdate = true;
#else
    motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
#ifndef USE_DSHOT
    if (motorConfig->dev.motorProtocol == MOTOR_PROTOCOL_PWM50HZ ) {
        motorConfig->dev.useUnsyncedUpdate = true;
    }
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_DISABLED;
#elif defined(DEFAULT_MOTOR_DSHOT_SPEED)
    motorConfig->dev.motorProtocol = DEFAULT_MOTOR_DSHOT_SPEED;
#else
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_DSHOT600;
#endif // USE_DSHOT
#endif // BRUSHED_MOTORS

    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
#ifdef BRUSHED_MOTORS
    motorConfig->motorIdle = 700; // historical default minThrottle for brushed was 1070
#else
    motorConfig->motorIdle = 550;
#endif // BRUSHED_MOTORS
    motorConfig->kv = 1960;

#ifdef USE_TIMER
#ifdef MOTOR1_PIN
    motorConfig->dev.ioTags[0] = IO_TAG(MOTOR1_PIN);
#endif
#ifdef MOTOR2_PIN
    motorConfig->dev.ioTags[1] = IO_TAG(MOTOR2_PIN);
#endif
#ifdef MOTOR3_PIN
    motorConfig->dev.ioTags[2] = IO_TAG(MOTOR3_PIN);
#endif
#ifdef MOTOR4_PIN
    motorConfig->dev.ioTags[3] = IO_TAG(MOTOR4_PIN);
#endif
#ifdef MOTOR5_PIN
    motorConfig->dev.ioTags[4] = IO_TAG(MOTOR5_PIN);
#endif
#ifdef MOTOR6_PIN
    motorConfig->dev.ioTags[5] = IO_TAG(MOTOR6_PIN);
#endif
#ifdef MOTOR7_PIN
    motorConfig->dev.ioTags[6] = IO_TAG(MOTOR7_PIN);
#endif
#ifdef MOTOR8_PIN
    motorConfig->dev.ioTags[7] = IO_TAG(MOTOR8_PIN);
#endif
#endif

    motorConfig->motorPoleCount = 14;   // Most brushless motors that we use are 14 poles

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorConfig->dev.motorOutputReordering[i] = i;
    }

#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = DEFAULT_DSHOT_BURST;
#endif

#ifdef USE_DSHOT_TELEMETRY
    motorConfig->dev.useDshotTelemetry = DEFAULT_DSHOT_TELEMETRY;
#endif

#ifdef USE_DSHOT_BITBANG
    motorConfig->dev.useDshotBitbang = DEFAULT_DSHOT_BITBANG;
    motorConfig->dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_DEFAULT;
#endif
}

#endif // USE_MOTOR
