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

#if !defined(DEFAULT_DSHOT_EDT)
#define DEFAULT_DSHOT_EDT DSHOT_EDT_OFF
#endif

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 3);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
    // Default motor protocol: a target/config may force one via DEFAULT_MOTOR_PROTOCOL
    // (e.g. BRUSHED, or PWM on a DShot-capable build); otherwise pick by capability.
#if defined(DEFAULT_MOTOR_PROTOCOL)
    motorConfig->dev.motorProtocol = DEFAULT_MOTOR_PROTOCOL;
#elif !defined(USE_DSHOT) && defined(USE_PWM_OUTPUT)
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_PWM;
#elif defined(USE_DSHOT) && defined(DEFAULT_MOTOR_DSHOT_SPEED)
    motorConfig->dev.motorProtocol = DEFAULT_MOTOR_DSHOT_SPEED;
#elif defined(USE_DSHOT)
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_DSHOT600;
#else
    motorConfig->dev.motorProtocol = MOTOR_PROTOCOL_DISABLED;
#endif

    // PWM rate and idle defaults follow from whether the chosen protocol is brushed,
    // rather than a separate build macro. Continuous (every-cycle) output only applies
    // to the analog PWM family (standard PWM and brushed); digital protocols (DShot)
    // are updated on demand, so it must not be forced on for a DShot DEFAULT_MOTOR_PROTOCOL.
    if (motorConfig->dev.motorProtocol == MOTOR_PROTOCOL_BRUSHED) {
        motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfig->dev.useContinuousUpdate = true;
        motorConfig->motorIdle = 700; // historical default minThrottle for brushed was 1070
    } else {
        motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
        motorConfig->dev.useContinuousUpdate = (motorConfig->dev.motorProtocol == MOTOR_PROTOCOL_PWM);
        motorConfig->motorIdle = 550;
    }

#if defined(DEFAULT_MOTOR_PWM_RATE)
    // Board override for the motor PWM output frequency (Hz), applied on top of
    // the protocol default above - e.g. an external ESC that reads a duty-cycle
    // PWM throttle at a specific rate.
    motorConfig->dev.motorPwmRate = DEFAULT_MOTOR_PWM_RATE;
#endif

    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->kv = 1960;

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

    motorConfig->motorPoleCount = 14;   // Most brushless motors that we use are 14 poles

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorConfig->dev.motorOutputReordering[i] = i;
    }

#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = DEFAULT_DSHOT_BURST;
#endif

#ifdef USE_DSHOT_TELEMETRY
    motorConfig->dev.useDshotTelemetry = DEFAULT_DSHOT_TELEMETRY;
    motorConfig->dev.useDshotEdt = DEFAULT_DSHOT_EDT;
#endif

#ifdef USE_DSHOT_BITBANG
    motorConfig->dev.useDshotBitbang = DEFAULT_DSHOT_BITBANG;
    motorConfig->dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_DEFAULT;
#endif
}

#endif // USE_MOTOR
