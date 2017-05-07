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

#include <platform.h>

#ifdef TARGET_CONFIG

#include "common/axis.h"

#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"

#include "fc/config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/gyro.h"

#include "hardware_revision.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

// alternative defaults settings for AlienFlight targets
void targetConfiguration(void)
{
    /* depending on revision ... depends on the LEDs to be utilised. */
    if (hardwareRevision == AFF3_REV_2) {
        statusLedConfigMutable()->polarity = 0
#ifdef LED0_A_INVERTED
            | BIT(0)
#endif
#ifdef LED1_A_INVERTED
            | BIT(1)
#endif
#ifdef LED2_A_INVERTED
            | BIT(2)
#endif
            ;

        for (int i = 0; i < LED_NUMBER; i++) {
            statusLedConfigMutable()->ledTags[i] = IO_TAG_NONE;
        }
#ifdef LED0_A
        statusLedConfigMutable()->ledTags[0] = IO_TAG(LED0_A);
#endif
#ifdef LED1_A
        statusLedConfigMutable()->ledTags[1] = IO_TAG(LED1_A);
#endif
#ifdef LED2_A
        statusLedConfigMutable()->ledTags[2] = IO_TAG(LED2_A);
#endif
    } else {
        gyroConfigMutable()->gyro_sync_denom = 2;
        pidConfigMutable()->pid_process_denom = 2;
    }

    rxConfigMutable()->spektrum_sat_bind = 5;
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;

    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        pidConfigMutable()->pid_process_denom = 1;
    }

    pidProfilesMutable(0)->P8[FD_ROLL] = 90;
    pidProfilesMutable(0)->I8[FD_ROLL] = 44;
    pidProfilesMutable(0)->D8[FD_ROLL] = 60;
    pidProfilesMutable(0)->P8[FD_PITCH] = 90;
    pidProfilesMutable(0)->I8[FD_PITCH] = 44;
    pidProfilesMutable(0)->D8[FD_PITCH] = 60;

    *customMotorMixerMutable(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixerMutable(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixerMutable(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixerMutable(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *customMotorMixerMutable(4) = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *customMotorMixerMutable(5) = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *customMotorMixerMutable(6) = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *customMotorMixerMutable(7) = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
}
#endif
