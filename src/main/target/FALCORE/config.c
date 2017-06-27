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
#include "config/feature.h"
#include "fc/config.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/gyro.h"


void targetConfiguration(void)
{
    systemConfigMutable()->asyncMode = ASYNC_MODE_NONE;
    mixerConfigMutable()->mixerMode = MIXER_QUADX;
    
    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_RX_SERIAL);
    featureSet(FEATURE_GPS);
    featureSet(FEATURE_TELEMETRY);
    featureSet(FEATURE_LED_STRIP);
    featureSet(FEATURE_BLACKBOX);

    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;          // VCP
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_GPS;          // UART1
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;    // UART2
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_NONE;         // UART4
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_TELEMETRY_MAVLINK;    // UART5

    
    gyroConfigMutable()->looptime = 1000;
    gyroConfigMutable()->gyroSync = 1;
    gyroConfigMutable()->gyroSyncDenominator = 8;
    gyroConfigMutable()->gyro_lpf = 0;              // 256 Hz
    gyroConfigMutable()->gyro_soft_lpf_hz = 90;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 150;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = 80;
    
    accelerometerConfigMutable()->acc_hardware = ACC_MPU6500;
    accelerometerConfigMutable()->acc_lpf_hz = 15;
    
    compassConfigMutable()->mag_hardware = MAG_HMC5883;
    compassConfigMutable()->mag_align = CW270_DEG_FLIP;
    
    barometerConfigMutable()->baro_hardware = BARO_MS5607;
    barometerConfigMutable()->use_median_filtering = 1;
    
    rxConfigMutable()->mincheck = 1100;
    rxConfigMutable()->maxcheck = 1900;
    rxConfigMutable()->serialrx_provider = SERIALRX_IBUS;
    
    blackboxConfigMutable()->rate_num = 1;
    blackboxConfigMutable()->rate_denom = 4;
    
    motorConfigMutable()->minthrottle = 1015;
    motorConfigMutable()->maxthrottle = 2000;
    motorConfigMutable()->mincommand = 980;
    motorConfigMutable()->motorPwmRate = 2000;
    motorConfigMutable()->motorPwmProtocol = PWM_TYPE_ONESHOT125;
    
    failsafeConfigMutable()->failsafe_delay = 5;
    failsafeConfigMutable()->failsafe_recovery_delay = 5;
    failsafeConfigMutable()->failsafe_off_delay = 200;
    failsafeConfigMutable()->failsafe_throttle = 1200;
    failsafeConfigMutable()->failsafe_throttle_low_delay = 100;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_RTH;
    
    boardAlignmentMutable()->rollDeciDegrees = 0;
    boardAlignmentMutable()->pitchDeciDegrees = 165;
    boardAlignmentMutable()->yawDeciDegrees = 0;
    
    mixerConfigMutable()->yaw_jump_prevention_limit = 200;
    
    imuConfigMutable()->small_angle = 30;
    
    gpsConfigMutable()->provider = GPS_UBLOX;
    gpsConfigMutable()->sbasMode = SBAS_NONE;
    gpsConfigMutable()->dynModel = GPS_DYNMODEL_AIR_1G;
    gpsConfigMutable()->autoConfig = 1;
    gpsConfigMutable()->autoBaud = 1;
    gpsConfigMutable()->gpsMinSats = 7;
    
    rcControlsConfigMutable()->deadband = 10;
    rcControlsConfigMutable()->yaw_deadband = 15;

    navConfigMutable()->general.flags.disarm_on_landing = 1;
    navConfigMutable()->general.flags.use_thr_mid_for_althold = 1;
    navConfigMutable()->general.flags.extra_arming_safety = 1;
    navConfigMutable()->general.flags.rth_allow_landing = 1;
    
    navConfigMutable()->general.max_auto_speed = 500;
    navConfigMutable()->general.max_auto_climb_rate = 200;
    navConfigMutable()->general.max_manual_speed = 500;
    navConfigMutable()->general.max_manual_climb_rate = 200;
    navConfigMutable()->general.rth_altitude = 1000;
    
    navConfigMutable()->general.mc.max_bank_angle = 30;
    navConfigMutable()->general.mc.hover_throttle = 1500;
    navConfigMutable()->general.mc.auto_disarm_delay = 2000;
    
    
    
    
    
}
