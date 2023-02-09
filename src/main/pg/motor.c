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

#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
#ifdef BRUSHED_MOTORS
    motorConfig->minthrottle = 1000;
    motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
    motorConfig->dev.useUnsyncedPwm = true;
#else
#ifdef USE_BRUSHED_ESC_AUTODETECT
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfig->minthrottle = 1000;
        motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfig->dev.useUnsyncedPwm = true;
    } else
#endif // USE_BRUSHED_ESC_AUTODETECT
    {
        motorConfig->minthrottle = 1070;
        motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_DISABLED;
    }
#endif // BRUSHED_MOTORS

    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->digitalIdleOffsetValue = 550;

#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;
#endif

#ifdef USE_TIMER
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }
#ifdef RESOURCE_MOTOR1_PIN
    motorConfig->dev.ioTags[0] = IO_TAG(RESOURCE_MOTOR1_PIN);
#endif
#ifdef RESOURCE_MOTOR2_PIN
    motorConfig->dev.ioTags[1] = IO_TAG(RESOURCE_MOTOR2_PIN);
#endif
#ifdef RESOURCE_MOTOR3_PIN
    motorConfig->dev.ioTags[2] = IO_TAG(RESOURCE_MOTOR3_PIN);
#endif
#ifdef RESOURCE_MOTOR4_PIN
    motorConfig->dev.ioTags[3] = IO_TAG(RESOURCE_MOTOR4_PIN);
#endif
#ifdef RESOURCE_MOTOR5_PIN
    motorConfig->dev.ioTags[4] = IO_TAG(RESOURCE_MOTOR5_PIN);
#endif
#ifdef RESOURCE_MOTOR6_PIN
    motorConfig->dev.ioTags[5] = IO_TAG(RESOURCE_MOTOR6_PIN);
#endif
#ifdef RESOURCE_MOTOR7_PIN
    motorConfig->dev.ioTags[6] = IO_TAG(RESOURCE_MOTOR7_PIN);
#endif
#ifdef RESOURCE_MOTOR8_PIN
    motorConfig->dev.ioTags[7] = IO_TAG(RESOURCE_MOTOR8_PIN);
#endif
#ifdef RESOURCE_MOTOR9_PIN
    motorConfig->dev.ioTags[8] = IO_TAG(RESOURCE_MOTOR9_PIN);
#endif
#ifdef RESOURCE_MOTOR10_PIN
    motorConfig->dev.ioTags[9] = IO_TAG(RESOURCE_MOTOR10_PIN);
#endif
#ifdef RESOURCE_MOTOR11_PIN
    motorConfig->dev.ioTags[10] = IO_TAG(RESOURCE_MOTOR11_PIN);
#endif
#ifdef RESOURCE_MOTOR12_PIN
    motorConfig->dev.ioTags[11] = IO_TAG(RESOURCE_MOTOR12_PIN);
#endif
#ifdef RESOURCE_MOTOR13_PIN
    motorConfig->dev.ioTags[12] = IO_TAG(RESOURCE_MOTOR13_PIN);
#endif
#ifdef RESOURCE_MOTOR14_PIN
    motorConfig->dev.ioTags[13] = IO_TAG(RESOURCE_MOTOR14_PIN);
#endif
#ifdef RESOURCE_MOTOR15_PIN
    motorConfig->dev.ioTags[14] = IO_TAG(RESOURCE_MOTOR15_PIN);
#endif
#ifdef RESOURCE_MOTOR16_PIN
    motorConfig->dev.ioTags[15] = IO_TAG(RESOURCE_MOTOR16_PIN);
#endif
#endif // USE_TIMER

    motorConfig->motorPoleCount = 14;   // Most brushes motors that we use are 14 poles

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorConfig->dev.motorOutputReordering[i] = i;
    }

#ifdef USE_DSHOT_BITBANG
    motorConfig->dev.useDshotBitbang = DSHOT_BITBANG_DEFAULT;
    motorConfig->dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_DEFAULT;
#endif
}

#endif // USE_MOTOR
