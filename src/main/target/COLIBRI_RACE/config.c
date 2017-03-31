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
#include <string.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "io/ledstrip.h"
#include "io/serial.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"


void targetApplyDefaultLedStripConfig(ledConfig_t *ledConfigs)
{
    const ledConfig_t defaultLedStripConfig[] = {
        DEFINE_LED( 0,   0,     6,  LD(WEST), LF(COLOR), LO(WARNING),   0 ),
        DEFINE_LED( 0,   1,     6,  LD(WEST), LF(COLOR), LO(WARNING),   0 ),
        DEFINE_LED( 0,   8,     6,  LD(WEST), LF(COLOR), LO(WARNING),   0 ),
        DEFINE_LED( 7,  15,     6,  0,        LF(COLOR), 0,             0 ),
        DEFINE_LED( 8,  15,     6,  0,        LF(COLOR), 0,             0 ),
        DEFINE_LED( 7,  14,     6,  0,        LF(COLOR), 0,             0 ),
        DEFINE_LED( 8,  14,     6,  0,        LF(COLOR), 0,             0 ),
        DEFINE_LED( 15,  8,     6,  LD(EAST), LF(COLOR), LO(WARNING),   0 ),
        DEFINE_LED( 15,  1,     6,  LD(EAST), LF(COLOR), LO(WARNING),   0 ),
        DEFINE_LED( 15,  0,     6,  LD(EAST), LF(COLOR), LO(WARNING),   0 ),
    };
    memcpy(ledConfigs, &defaultLedStripConfig, MIN(LED_MAX_STRIP_LENGTH, sizeof(defaultLedStripConfig)));
}

// alternative defaults settings for COLIBRI RACE targets
void targetConfiguration(void)
{
    motorConfigMutable()->minthrottle = 1025;
    motorConfigMutable()->maxthrottle = 1980;
    motorConfigMutable()->mincommand = 1000;
    servoConfigMutable()->dev.servoCenterPulse = 1500;

    batteryConfigMutable()->vbatmaxcellvoltage = 45;
    batteryConfigMutable()->vbatmincellvoltage = 30;
    batteryConfigMutable()->vbatwarningcellvoltage = 35;

    flight3DConfigMutable()->deadband3d_low = 1406;
    flight3DConfigMutable()->deadband3d_high = 1514;
    flight3DConfigMutable()->neutral3d = 1460;
    flight3DConfigMutable()->deadband3d_throttle = 0;

    failsafeConfigMutable()->failsafe_procedure = 1;
    failsafeConfigMutable()->failsafe_throttle_low_delay = 10;

    gyroConfigMutable()->gyro_sync_denom = 1;
    pidConfigMutable()->pid_process_denom = 3;
    blackboxConfigMutable()->rate_num = 1;
    blackboxConfigMutable()->rate_denom = 1;

    rcControlsConfigMutable()->deadband = 5;
    rcControlsConfigMutable()->yaw_deadband = 5;

    failsafeConfigMutable()->failsafe_delay = 10;

    telemetryConfigMutable()->telemetry_inversion = 1;

    pidProfilesMutable(0)->vbatPidCompensation = 1;

    pidProfilesMutable(0)->P8[ROLL] = 46;     // new PID with preliminary defaults test carefully
    pidProfilesMutable(0)->I8[ROLL] = 48;
    pidProfilesMutable(0)->D8[ROLL] = 23;
    pidProfilesMutable(0)->P8[PITCH] = 89;
    pidProfilesMutable(0)->I8[PITCH] = 59;
    pidProfilesMutable(0)->D8[PITCH] = 25;
    pidProfilesMutable(0)->P8[YAW] = 129;
    pidProfilesMutable(0)->I8[YAW] = 50;
    pidProfilesMutable(0)->D8[YAW] = 20;

    controlRateProfilesMutable(0)->rates[FD_ROLL] = 86;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 86;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 80;

    targetApplyDefaultLedStripConfig(ledStripConfigMutable()->ledConfigs);
}

void targetValidateConfiguration()
{
    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_MSP);
        featureSet(FEATURE_RX_PPM);
    }
}
#endif
