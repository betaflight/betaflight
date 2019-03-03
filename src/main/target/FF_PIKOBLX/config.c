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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG
#include "common/axis.h"

#include "config/feature.h"

#include "drivers/pwm_esc_detect.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "sensors/battery.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

void targetConfiguration(void)
{
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1049;

#if defined(FF_ACROWHOOPSP)
        rxConfigMutable()->serialrx_provider = SERIALRX_SPEKTRUM2048;
        rxConfigMutable()->spektrum_sat_bind = 5;
        rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
#else
        serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)].functionMask = FUNCTION_TELEMETRY_FRSKY_HUB;
        rxConfigMutable()->serialrx_inverted = true;
        featureEnable(FEATURE_TELEMETRY);
#endif
        parseRcChannels("TAER1234", rxConfigMutable());

        for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
            pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

            pidProfile->pid[PID_ROLL].P = 80;
            pidProfile->pid[PID_ROLL].I = 37;
            pidProfile->pid[PID_ROLL].D = 35;
            pidProfile->pid[PID_PITCH].P = 100;
            pidProfile->pid[PID_PITCH].I = 37;
            pidProfile->pid[PID_PITCH].D = 35;
            pidProfile->pid[PID_YAW].P = 180;
            pidProfile->pid[PID_YAW].D = 45;
        }

        for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
            controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

            controlRateConfig->rcRates[FD_ROLL] = 100;
            controlRateConfig->rcRates[FD_PITCH] = 100;
            controlRateConfig->rcRates[FD_YAW] = 100;
            controlRateConfig->rcExpo[FD_ROLL] = 15;
            controlRateConfig->rcExpo[FD_PITCH] = 15;
            controlRateConfig->rcExpo[FD_YAW] = 15;
            controlRateConfig->rates[PID_ROLL] = 80;
            controlRateConfig->rates[PID_PITCH] = 80;
            controlRateConfig->rates[PID_YAW] = 80;
        }
    }
}
#endif
