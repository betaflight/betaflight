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
#include "drivers/pwm_output.h"
#include "blackbox/blackbox.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/boardalignment.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "navigation/navigation.h"


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
    featureSet(FEATURE_AIRMODE);

    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;          // VCP
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_GPS;          // UART1
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;    // UART2
    serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_NONE;         // UART4
    serialConfigMutable()->portConfigs[4].functionMask = FUNCTION_TELEMETRY_MAVLINK;    // UART5

    
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
    
    navConfigMutable()->mc.max_bank_angle = 30;
    navConfigMutable()->mc.hover_throttle = 1500;
    navConfigMutable()->mc.auto_disarm_delay = 2000;
    
    /*
    aux 0 0 0 1150 2100
    aux 1 2 0 1300 1700
    aux 2 20 0 1150 2100
    aux 3 3 3 1300 1700
    aux 4 9 3 1300 1700
    aux 5 8 3 1700 2100
    aux 6 19 1 1375 2100
    */

    for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationConditionsMutable(index)->modeId = 0;
        modeActivationConditionsMutable(index)->auxChannelIndex = 0;
        modeActivationConditionsMutable(index)->range.startStep = 0;
        modeActivationConditionsMutable(index)->range.endStep = 0;
    }

    configureModeActivationCondition(0, BOXARM,         0, 1150, 2100);
    configureModeActivationCondition(1, BOXANGLE,       0, 1300, 1700);
    configureModeActivationCondition(2, BOXNAVALTHOLD,  3, 1300, 1700);
    configureModeActivationCondition(3, BOXNAVPOSHOLD,  3, 1300, 1700);
    configureModeActivationCondition(4, BOXNAVRTH,      3, 1700, 2100);
    configureModeActivationCondition(5, BOXANGLE,       3, 1700, 2100);
    
    // Rates and PIDs
    setConfigProfile(0);
    pidProfileMutable()->bank_mc.pid[PID_PITCH].P = 65;
    pidProfileMutable()->bank_mc.pid[PID_PITCH].I = 50;
    pidProfileMutable()->bank_mc.pid[PID_PITCH].D = 28;
    pidProfileMutable()->bank_mc.pid[PID_ROLL].P = 45;
    pidProfileMutable()->bank_mc.pid[PID_ROLL].I = 40;
    pidProfileMutable()->bank_mc.pid[PID_ROLL].D = 25;
    pidProfileMutable()->bank_mc.pid[PID_YAW].P = 90;
    pidProfileMutable()->bank_mc.pid[PID_YAW].I = 45;
    pidProfileMutable()->bank_mc.pid[PID_YAW].D = 0;
    pidProfileMutable()->bank_mc.pid[PID_LEVEL].P = 20;
    pidProfileMutable()->bank_mc.pid[PID_LEVEL].I = 10;
    pidProfileMutable()->bank_mc.pid[PID_LEVEL].D = 75;
    
    pidProfileMutable()->max_angle_inclination[FD_ROLL] = 300;
    pidProfileMutable()->max_angle_inclination[FD_PITCH] = 300;
    pidProfileMutable()->dterm_lpf_hz = 70;
    pidProfileMutable()->yaw_lpf_hz = 35;
    pidProfileMutable()->dterm_setpoint_weight = 0;
    pidProfileMutable()->dterm_soft_notch_hz = 0;
    pidProfileMutable()->dterm_soft_notch_cutoff = 1;
    pidProfileMutable()->pidSumLimit = 500;
    pidProfileMutable()->yaw_p_limit = 300;
    pidProfileMutable()->rollPitchItermIgnoreRate = 200;
    pidProfileMutable()->yawItermIgnoreRate = 200;
    pidProfileMutable()->axisAccelerationLimitRollPitch = 0;
    pidProfileMutable()->axisAccelerationLimitYaw = 10000;

    pidProfileMutable()->bank_mc.pid[PID_POS_Z].P = 50;
    pidProfileMutable()->bank_mc.pid[PID_POS_Z].I = 0;
    pidProfileMutable()->bank_mc.pid[PID_POS_Z].D = 0;
    pidProfileMutable()->bank_mc.pid[PID_VEL_Z].P = 100;
    pidProfileMutable()->bank_mc.pid[PID_VEL_Z].I = 50;
    pidProfileMutable()->bank_mc.pid[PID_VEL_Z].D = 10;
    pidProfileMutable()->bank_mc.pid[PID_POS_XY].P = 50;
    pidProfileMutable()->bank_mc.pid[PID_POS_XY].I = 100;
    pidProfileMutable()->bank_mc.pid[PID_POS_XY].D = 10;
    pidProfileMutable()->bank_mc.pid[PID_VEL_XY].P = 150;
    pidProfileMutable()->bank_mc.pid[PID_VEL_XY].I = 20;
    pidProfileMutable()->bank_mc.pid[PID_VEL_XY].D = 70;

    ((controlRateConfig_t*)currentControlRateProfile)->rcExpo8 = 60;
    ((controlRateConfig_t*)currentControlRateProfile)->rcYawExpo8 = 35;
    ((controlRateConfig_t*)currentControlRateProfile)->rates[FD_ROLL] = 54;
    ((controlRateConfig_t*)currentControlRateProfile)->rates[FD_PITCH] = 54;
    ((controlRateConfig_t*)currentControlRateProfile)->rates[FD_YAW] = 36;
    ((controlRateConfig_t*)currentControlRateProfile)->thrMid8 = 50;
    ((controlRateConfig_t*)currentControlRateProfile)->thrExpo8 = 0;
    ((controlRateConfig_t*)currentControlRateProfile)->dynThrPID = 10;
    ((controlRateConfig_t*)currentControlRateProfile)->tpa_breakpoint = 1600;
}
