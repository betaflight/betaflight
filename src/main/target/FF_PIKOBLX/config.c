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
#include <stdbool.h>

#include <platform.h>

#ifdef TARGET_CONFIG
#include "common/axis.h"

#include "config/feature.h"

#include "drivers/pwm_esc_detect.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"

#include "flight/mixer.h"
#include "flight/pid.h"

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
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1049;

#if defined(FF_ACROWHOOPSP)
        rxConfigMutable()->serialrx_provider = SERIALRX_SPEKTRUM2048;
        rxConfigMutable()->spektrum_sat_bind = 5;
        rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
#else
        serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)].functionMask = FUNCTION_TELEMETRY_FRSKY;
        rxConfigMutable()->sbus_inversion = 0;
        featureSet(FEATURE_TELEMETRY);
#endif
        parseRcChannels("TAER1234", rxConfigMutable());
    
        pidProfilesMutable(0)->pid[PID_ROLL].P = 80;
        pidProfilesMutable(0)->pid[PID_ROLL].I = 37;
        pidProfilesMutable(0)->pid[PID_ROLL].D = 35;
        pidProfilesMutable(0)->pid[PID_PITCH].P = 100;
        pidProfilesMutable(0)->pid[PID_PITCH].I = 37;
        pidProfilesMutable(0)->pid[PID_PITCH].D = 35;
        pidProfilesMutable(0)->pid[PID_YAW].P = 180;
        pidProfilesMutable(0)->pid[PID_YAW].D = 45;
    
        controlRateProfilesMutable(0)->rcRate8 = 100;
        controlRateProfilesMutable(0)->rcYawRate8 = 100;
        controlRateProfilesMutable(0)->rcExpo8 = 15;
        controlRateProfilesMutable(0)->rcYawExpo8 = 15;
        controlRateProfilesMutable(0)->rates[PID_ROLL] = 80;
        controlRateProfilesMutable(0)->rates[PID_PITCH] = 80;
        controlRateProfilesMutable(0)->rates[PID_YAW] = 80;
    }
}
#endif
