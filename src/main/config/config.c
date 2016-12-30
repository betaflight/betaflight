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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/rx_spi.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/pitotmeter.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/motors.h"
#include "io/servos.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif
#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400


master_t masterConfig;                 // master config struct with data independent from profiles
profile_t *currentProfile;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;


static void resetAccelerometerTrims(flightDynamicsTrims_t * accZero, flightDynamicsTrims_t * accGain)
{
    accZero->values.pitch = 0;
    accZero->values.roll = 0;
    accZero->values.yaw = 0;

    accGain->values.pitch = 4096;
    accGain->values.roll = 4096;
    accGain->values.yaw = 4096;
}

void resetPidProfile(pidProfile_t *pidProfile)
{
    pidProfile->P8[ROLL] = 40;
    pidProfile->I8[ROLL] = 30;
    pidProfile->D8[ROLL] = 23;
    pidProfile->P8[PITCH] = 40;
    pidProfile->I8[PITCH] = 30;
    pidProfile->D8[PITCH] = 23;
    pidProfile->P8[YAW] = 85;
    pidProfile->I8[YAW] = 45;
    pidProfile->D8[YAW] = 0;        // not used
    pidProfile->P8[PIDALT] = 50;    // NAV_POS_Z_P * 100
    pidProfile->I8[PIDALT] = 0;     // not used
    pidProfile->D8[PIDALT] = 0;     // not used
    pidProfile->P8[PIDPOS] = 65;    // NAV_POS_XY_P * 100
    pidProfile->I8[PIDPOS] = 120;   // posDecelerationTime * 100
    pidProfile->D8[PIDPOS] = 10;    // posResponseExpo * 100
    pidProfile->P8[PIDPOSR] = 180;  // NAV_VEL_XY_P * 100
    pidProfile->I8[PIDPOSR] = 15;   // NAV_VEL_XY_I * 100
    pidProfile->D8[PIDPOSR] = 100;  // NAV_VEL_XY_D * 100
    pidProfile->P8[PIDNAVR] = 10;   // FW_NAV_P * 100
    pidProfile->I8[PIDNAVR] = 5;    // FW_NAV_I * 100
    pidProfile->D8[PIDNAVR] = 8;    // FW_NAV_D * 100
    pidProfile->P8[PIDLEVEL] = 20;  // Self-level strength
    pidProfile->I8[PIDLEVEL] = 15;  // Self-leveing low-pass frequency (0 - disabled)
    pidProfile->D8[PIDLEVEL] = 75;  // 75% horizon strength
    pidProfile->P8[PIDMAG] = 60;
    pidProfile->P8[PIDVEL] = 100;   // NAV_VEL_Z_P * 100
    pidProfile->I8[PIDVEL] = 50;    // NAV_VEL_Z_I * 100
    pidProfile->D8[PIDVEL] = 10;    // NAV_VEL_Z_D * 100

    pidProfile->acc_soft_lpf_hz = 15;
    pidProfile->gyro_soft_lpf_hz = 60;
    pidProfile->dterm_lpf_hz = 40;
    pidProfile->yaw_lpf_hz = 30;

    pidProfile->rollPitchItermIgnoreRate = 200;     // dps
    pidProfile->yawItermIgnoreRate = 50;            // dps

    pidProfile->axisAccelerationLimitYaw = 10000;       // dps/s
    pidProfile->axisAccelerationLimitRollPitch = 0;     // dps/s

    pidProfile->yaw_p_limit = YAW_P_LIMIT_DEFAULT;
    pidProfile->mag_hold_rate_limit = MAG_HOLD_RATE_LIMIT_DEFAULT;

    pidProfile->max_angle_inclination[FD_ROLL] = 300;    // 30 degrees
    pidProfile->max_angle_inclination[FD_PITCH] = 300;    // 30 degrees
#ifdef USE_SERVOS
    pidProfile->fixedWingItermThrowLimit = FW_ITERM_THROW_LIMIT_DEFAULT;
#endif

}

#ifdef NAV
void resetNavConfig(navConfig_t * navConfig)
{
    // Navigation flags
    navConfig->general.flags.use_thr_mid_for_althold = 0;
    navConfig->general.flags.extra_arming_safety = 1;
    navConfig->general.flags.user_control_mode = NAV_GPS_ATTI;
    navConfig->general.flags.rth_alt_control_mode = NAV_RTH_AT_LEAST_ALT;
    navConfig->general.flags.rth_climb_first = 1;                         // Climb first, turn after reaching safe altitude
    navConfig->general.flags.rth_tail_first = 0;
    navConfig->general.flags.disarm_on_landing = 0;

    // Inertial position estimator parameters
#if defined(NAV_AUTO_MAG_DECLINATION)
    navConfig->estimation.automatic_mag_declination = 1;
#endif
    navConfig->estimation.gps_min_sats = 6;
    navConfig->estimation.gps_delay_ms = 200;
    navConfig->estimation.accz_unarmed_cal = 1;
    navConfig->estimation.use_gps_velned = 1;         // "Disabled" is mandatory with gps_dyn_model = Pedestrian

    navConfig->estimation.w_z_baro_p = 0.35f;

    navConfig->estimation.w_z_gps_p = 0.2f;
    navConfig->estimation.w_z_gps_v = 0.5f;

    navConfig->estimation.w_xy_gps_p = 1.0f;
    navConfig->estimation.w_xy_gps_v = 2.0f;

    navConfig->estimation.w_z_res_v = 0.5f;
    navConfig->estimation.w_xy_res_v = 0.5f;

    navConfig->estimation.w_acc_bias = 0.01f;

    navConfig->estimation.max_eph_epv = 1000.0f;
    navConfig->estimation.baro_epv = 100.0f;

    // General navigation parameters
    navConfig->general.pos_failure_timeout = 5;     // 5 sec
    navConfig->general.waypoint_radius = 100;       // 2m diameter
    navConfig->general.max_speed = 300;             // 3 m/s = 10.8 km/h
    navConfig->general.max_climb_rate = 500;        // 5 m/s
    navConfig->general.max_manual_speed = 500;
    navConfig->general.max_manual_climb_rate = 200;
    navConfig->general.land_descent_rate = 200;     // 2 m/s
    navConfig->general.land_slowdown_minalt = 500;  // 5 meters of altitude
    navConfig->general.land_slowdown_maxalt = 2000; // 20 meters of altitude
    navConfig->general.emerg_descent_rate = 500;    // 5 m/s
    navConfig->general.min_rth_distance = 500;      // If closer than 5m - land immediately
    navConfig->general.rth_altitude = 1000;         // 10m

    // MC-specific
    navConfig->mc.max_bank_angle = 30;      // 30 deg
    navConfig->mc.hover_throttle = 1500;
    navConfig->mc.auto_disarm_delay = 2000;

    // Fixed wing
    navConfig->fw.max_bank_angle = 20;      // 30 deg
    navConfig->fw.max_climb_angle = 20;
    navConfig->fw.max_dive_angle = 15;
    navConfig->fw.cruise_throttle = 1400;
    navConfig->fw.max_throttle = 1700;
    navConfig->fw.min_throttle = 1200;
    navConfig->fw.pitch_to_throttle = 10;   // pwm units per degree of pitch (10pwm units ~ 1% throttle)
    navConfig->fw.roll_to_pitch = 75;       // percent of coupling
    navConfig->fw.loiter_radius = 5000;     // 50m

    // Fixed wing launch
    navConfig->fw.launch_velocity_thresh = 300;         // 3 m/s
    navConfig->fw.launch_accel_thresh = 1.9f * 981;     // cm/s/s (1.9*G)
    navConfig->fw.launch_time_thresh = 40;              // 40ms
    navConfig->fw.launch_throttle = 1700;
    navConfig->fw.launch_motor_timer = 500;             // ms
    navConfig->fw.launch_timeout = 5000;                // ms, timeout for launch procedure
    navConfig->fw.launch_climb_angle = 10;              // 10 deg
}

void validateNavConfig(navConfig_t * navConfig)
{
    // Make sure minAlt is not more than maxAlt, maxAlt cannot be set lower than 500.
    navConfig->general.land_slowdown_minalt = MIN(navConfig->general.land_slowdown_minalt, navConfig->general.land_slowdown_maxalt - 100);
}
#endif

void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->use_median_filtering = 1;
}

void resetPitotmeterConfig(pitotmeterConfig_t *pitotmeterConfig)
{
    pitotmeterConfig->use_median_filtering = 1;
    pitotmeterConfig->pitot_noise_lpf = 0.6f;
    pitotmeterConfig->pitot_scale = 1.00f;
}

void resetMotorConfig(motorConfig_t *motorConfig)
{
#ifdef BRUSHED_MOTORS
    motorConfig->minthrottle = 1000;
    motorConfig->motorPwmProtocol = PWM_TYPE_BRUSHED;
    motorConfig->motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
#else
    motorConfig->minthrottle = 1150;
    motorConfig->motorPwmProtocol = PWM_TYPE_STANDARD;
    motorConfig->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
    motorConfig->maxthrottle = 1850;
    motorConfig->mincommand = 1000;

}

#ifdef USE_SERVOS
void resetServoConfig(servoConfig_t *servoConfig)
{
    servoConfig->servoCenterPulse = 1500;
    servoConfig->servoPwmRate = 50;
}
#endif

void resetFlight3DConfig(flight3DConfig_t *flight3DConfig)
{
    flight3DConfig->deadband3d_low = 1406;
    flight3DConfig->deadband3d_high = 1514;
    flight3DConfig->neutral3d = 1460;
    flight3DConfig->deadband3d_throttle = 50;
}

#ifdef TELEMETRY
void resetTelemetryConfig(telemetryConfig_t *telemetryConfig)
{
#if defined(STM32F303xC)
    telemetryConfig->telemetry_inversion = 1;
#else
    telemetryConfig->telemetry_inversion = 0;
#endif
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_METRICS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->frsky_vfas_cell_voltage = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
#ifdef TELEMETRY_SMARTPORT
    telemetryConfig->smartportUartUnidirectional = 0;
#endif
}
#endif

void resetBatteryConfig(batteryConfig_t *batteryConfig)
{
    batteryConfig->vbatscale = VBAT_SCALE_DEFAULT;
    batteryConfig->vbatresdivval = VBAT_RESDIVVAL_DEFAULT;
    batteryConfig->vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT;
    batteryConfig->vbatmaxcellvoltage = 43;
    batteryConfig->vbatmincellvoltage = 33;
    batteryConfig->vbatwarningcellvoltage = 35;
    batteryConfig->currentMeterOffset = 0;
    batteryConfig->currentMeterScale = 400; // for Allegro ACS758LCB-100U (40mV/A)
    batteryConfig->batteryCapacity = 0;
    batteryConfig->currentMeterType = CURRENT_SENSOR_ADC;
}

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void resetSerialConfig(serialConfig_t *serialConfig)
{
    uint8_t index;
    memset(serialConfig, 0, sizeof(serialConfig_t));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialConfig->portConfigs[index].identifier = serialPortIdentifiers[index];
        serialConfig->portConfigs[index].msp_baudrateIndex = BAUD_115200;
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_38400;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#ifdef CC3D
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}

static void resetControlRateConfig(controlRateConfig_t *controlRateConfig) {
    controlRateConfig->rcExpo8 = 70;
    controlRateConfig->thrMid8 = 50;
    controlRateConfig->thrExpo8 = 0;
    controlRateConfig->dynThrPID = 0;
    controlRateConfig->rcYawExpo8 = 20;
    controlRateConfig->tpa_breakpoint = 1500;

    for (uint8_t axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        if (axis == FD_YAW) {
            controlRateConfig->rates[axis] = CONTROL_RATE_CONFIG_YAW_RATE_DEFAULT;
        } else {
            controlRateConfig->rates[axis] = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT;
        }
    }

}

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig) {
    rcControlsConfig->deadband = 5;
    rcControlsConfig->yaw_deadband = 5;
    rcControlsConfig->pos_hold_deadband = 20;
    rcControlsConfig->alt_hold_deadband = 50;
}

static void resetMixerConfig(mixerConfig_t *mixerConfig)
{
    mixerConfig->yaw_motor_direction = 1;
    mixerConfig->yaw_jump_prevention_limit = 200;
}

#ifdef USE_SERVOS
static void resetServoMixerConfig(servoMixerConfig_t *servoMixerConfig)
{
    servoMixerConfig->tri_unarmed_servo = 1;
    servoMixerConfig->servo_lowpass_freq = 400;
    servoMixerConfig->servo_lowpass_enable = 0;
}
#endif

#ifdef ASYNC_GYRO_PROCESSING
uint32_t getPidUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_NONE) {
        return getGyroUpdateRate();
    } else {
        return gyroConfig()->looptime;
    }
}

uint32_t getGyroUpdateRate(void) {
    return gyro.targetLooptime;
}

uint16_t getAccUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / masterConfig.accTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
}

uint16_t getAttitudeUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / masterConfig.attitudeTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
}

uint8_t getAsyncMode(void) {
    return masterConfig.asyncMode;
}
#endif

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

void setProfile(uint8_t profileIndex)
{
    currentProfile = &masterConfig.profile[profileIndex];
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex) {
    return &masterConfig.controlRateProfiles[profileIndex];
}

void setControlRateProfile(uint8_t profileIndex)
{
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = &masterConfig.controlRateProfiles[profileIndex];
}

uint16_t getCurrentMinthrottle(void)
{
    return motorConfig()->minthrottle;
}

// Default settings
static void resetConf(void)
{
    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));
    setProfile(0);
    setControlRateProfile(0);

    masterConfig.version = EEPROM_CONF_VERSION;
    mixerConfig()->mixerMode = MIXER_QUADX;
    featureClearAll();
    featureSet(DEFAULT_RX_FEATURE | FEATURE_FAILSAFE);
#ifdef DEFAULT_FEATURES
    featureSet(DEFAULT_FEATURES);
#endif

#ifdef OSD
    featureSet(FEATURE_OSD);
    osdResetConfig(&masterConfig.osdProfile);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

    // global settings
    masterConfig.current_profile_index = 0;     // default profile
    imuConfig()->dcm_kp_acc = 2500;             // 0.25 * 10000
    imuConfig()->dcm_ki_acc = 50;               // 0.005 * 10000
    imuConfig()->dcm_kp_mag = 10000;            // 1.00 * 10000
    imuConfig()->dcm_ki_mag = 0;                // 0.00 * 10000
    imuConfig()->small_angle = 25;
    gyroConfig()->gyro_lpf = 3;                  // INV_FILTER_42HZ, In case of ST gyro, will default to 32Hz instead

    resetAccelerometerTrims(&accelerometerConfig()->accZero, &accelerometerConfig()->accGain);

    gyroConfig()->gyro_align = ALIGN_DEFAULT;
    accelerometerConfig()->acc_align = ALIGN_DEFAULT;
    compassConfig()->mag_align = ALIGN_DEFAULT;

    boardAlignment()->rollDeciDegrees = 0;
    boardAlignment()->pitchDeciDegrees = 0;
    boardAlignment()->yawDeciDegrees = 0;

    gyroConfig()->gyroMovementCalibrationThreshold = 32;

    accelerometerConfig()->acc_hardware = ACC_AUTODETECT;     // default/autodetect

#ifdef MAG
    compassConfig()->mag_hardware = MAG_AUTODETECT;
#else
    compassConfig()->mag_hardware = MAG_NONE;
#endif

#ifdef BARO
    barometerConfig()->baro_hardware = BARO_AUTODETECT;
#else
    barometerConfig()->baro_hardware = BARO_NONE;
#endif

#ifdef PITOT
    pitotmeterConfig()->pitot_hardware = PITOT_AUTODETECT;
#else
    pitotmeterConfig()->pitot_hardware = PITOT_NONE;
#endif

    resetBatteryConfig(&masterConfig.batteryConfig);

#ifdef TELEMETRY
    resetTelemetryConfig(&masterConfig.telemetryConfig);
#endif

#ifdef SERIALRX_PROVIDER
    rxConfig()->serialrx_provider = SERIALRX_PROVIDER;
#else
    rxConfig()->serialrx_provider = 0;
#endif
    rxConfig()->rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL;
    rxConfig()->spektrum_sat_bind = 0;
    rxConfig()->midrc = 1500;
    rxConfig()->mincheck = 1100;
    rxConfig()->maxcheck = 1900;
    rxConfig()->rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
    rxConfig()->rx_max_usec = 2115;         // any of first 4 channels above this value will trigger rx loss detection

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig()->failsafe_channel_configurations[i];
        if (i < NON_AUX_CHANNEL_COUNT) {
            channelFailsafeConfiguration->mode = (i == THROTTLE) ? RX_FAILSAFE_MODE_HOLD : RX_FAILSAFE_MODE_AUTO;
        }
        else {
            channelFailsafeConfiguration->mode = RX_FAILSAFE_MODE_HOLD;
        }

        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->midrc);
    }

    rxConfig()->rssi_channel = 0;
    rxConfig()->rssi_scale = RSSI_SCALE_DEFAULT;
    rxConfig()->rssi_ppm_invert = 0;
    rxConfig()->rcSmoothing = 1;

    resetAllRxChannelRangeConfigurations(rxConfig()->channelRanges);

    pwmRxConfig()->inputFilteringMode = INPUT_FILTERING_DISABLED;

    armingConfig()->disarm_kill_switch = 1;
    armingConfig()->auto_disarm_delay = 5;

    resetMixerConfig(&masterConfig.mixerConfig);
#ifdef USE_SERVOS
    resetServoMixerConfig(&masterConfig.servoMixerConfig);
    resetServoConfig(&masterConfig.servoConfig);
#endif

    resetMotorConfig(&masterConfig.motorConfig);
    resetFlight3DConfig(&masterConfig.flight3DConfig);

#ifdef GPS
    // gps/nav stuff
    gpsConfig()->provider = GPS_UBLOX;
    gpsConfig()->sbasMode = SBAS_NONE;
    gpsConfig()->autoConfig = GPS_AUTOCONFIG_ON;
    gpsConfig()->autoBaud = GPS_AUTOBAUD_ON;
    gpsConfig()->dynModel = GPS_DYNMODEL_AIR_1G;
#endif

#ifdef NAV
    resetNavConfig(&masterConfig.navConfig);
#endif

    resetSerialConfig(&masterConfig.serialConfig);

    gyroConfig()->looptime = 2000;
    masterConfig.i2c_overclock = 0;
    gyroConfig()->gyroSync = 0;
    gyroConfig()->gyroSyncDenominator = 2;

#ifdef ASYNC_GYRO_PROCESSING
    masterConfig.accTaskFrequency = ACC_TASK_FREQUENCY_DEFAULT;
    masterConfig.attitudeTaskFrequency = ATTITUDE_TASK_FREQUENCY_DEFAULT;
    masterConfig.asyncMode = ASYNC_MODE_NONE;
#endif

    resetPidProfile(&currentProfile->pidProfile);

    resetControlRateConfig(&masterConfig.controlRateProfiles[0]);

    // for (int i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;

    compassConfig()->mag_declination = 0;

    currentProfile->modeActivationOperator = MODE_OPERATOR_OR; // default is to OR multiple-channel mode activation conditions

    resetBarometerConfig(&masterConfig.barometerConfig);
    resetPitotmeterConfig(&masterConfig.pitotmeterConfig);

    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", &masterConfig.rxConfig);
#else
    parseRcChannels("AETR1234", &masterConfig.rxConfig);
#endif

    resetRcControlsConfig(&currentProfile->rcControlsConfig);

    currentProfile->throttle_tilt_compensation_strength = 0;      // 0-100, 0 - disabled

    // Failsafe Variables
    failsafeConfig()->failsafe_delay = 5;               // 0.5 sec
    failsafeConfig()->failsafe_recovery_delay = 5;      // 0.5 seconds (plus 200ms explicit delay)
    failsafeConfig()->failsafe_off_delay = 200;         // 20sec
    failsafeConfig()->failsafe_throttle = 1000;         // default throttle off.
    failsafeConfig()->failsafe_kill_switch = 0;         // default failsafe switch action is identical to rc link loss
    failsafeConfig()->failsafe_throttle_low_delay = 100;// default throttle low delay for "just disarm" on failsafe condition
    failsafeConfig()->failsafe_procedure = 0;           // default full failsafe procedure is 0: auto-landing, 1: drop, 2 : RTH

#ifdef USE_SERVOS
    // servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        currentProfile->servoConf[i].min = DEFAULT_SERVO_MIN;
        currentProfile->servoConf[i].max = DEFAULT_SERVO_MAX;
        currentProfile->servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        currentProfile->servoConf[i].rate = 100;
        currentProfile->servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        currentProfile->servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        currentProfile->servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    // gimbal
    currentProfile->gimbalConfig.mode = GIMBAL_MODE_NORMAL;

    currentProfile->flaperon_throw_offset = FLAPERON_THROW_DEFAULT;
    currentProfile->flaperon_throw_inverted = 0;

#endif

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        masterConfig.customMotorMixer[i].throttle = 0.0f;
    }

#ifdef LED_STRIP
    applyDefaultColors(ledStripConfig()->colors);
    applyDefaultLedStripConfig(ledStripConfig()->ledConfigs);
    applyDefaultModeColors(ledStripConfig()->modeColors);
    applyDefaultSpecialColors(&(ledStripConfig()->specialColors));
    ledStripConfig()->ledstrip_visual_beeper = 0;
#endif

#ifdef BLACKBOX
#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
    featureSet(FEATURE_BLACKBOX);
    blackboxConfig()->device = BLACKBOX_DEVICE_FLASH;
#else
    blackboxConfig()->device = BLACKBOX_DEVICE_SERIAL;
#endif
    blackboxConfig()->rate_num = 1;
    blackboxConfig()->rate_denom = 1;
#endif

    // alternative defaults settings for ALIENFLIGHTF1 and ALIENFLIGHTF3 targets
#ifdef ALIENFLIGHTF1
#ifdef ALIENFLIGHTF3
    serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    batteryConfig()->vbatscale = 20;
#else
    serialConfig()->portConfigs[1].functionMask = FUNCTION_RX_SERIAL;
#endif
    rxConfig()->spektrum_sat_bind = 5;
    motorConfig()->minthrottle = 1000;
    motorConfig()->maxthrottle = 2000;
    motorConfig()->motorPwmRate = 32000;
    masterConfig.looptime = 2000;
    currentProfile->pidProfile.P8[ROLL] = 36;
    currentProfile->pidProfile.P8[PITCH] = 36;
    failsafeConfig()->failsafe_delay = 2;
    failsafeConfig()->failsafe_off_delay = 0;
    currentControlRateProfile->rates[FD_PITCH] = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT;
    currentControlRateProfile->rates[FD_ROLL] = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT;
    currentControlRateProfile->rates[FD_YAW] = CONTROL_RATE_CONFIG_YAW_RATE_DEFAULT;
    parseRcChannels("TAER1234", &masterConfig.rxConfig);

    //  { 1.0f, -0.414178f,  1.0f, -1.0f },          // REAR_R
    masterConfig.customMotorMixer[0].throttle = 1.0f;
    masterConfig.customMotorMixer[0].roll = -0.414178f;
    masterConfig.customMotorMixer[0].pitch = 1.0f;
    masterConfig.customMotorMixer[0].yaw = -1.0f;

    //  { 1.0f, -0.414178f, -1.0f,  1.0f },          // FRONT_R
    masterConfig.customMotorMixer[1].throttle = 1.0f;
    masterConfig.customMotorMixer[1].roll = -0.414178f;
    masterConfig.customMotorMixer[1].pitch = -1.0f;
    masterConfig.customMotorMixer[1].yaw = 1.0f;

    //  { 1.0f,  0.414178f,  1.0f,  1.0f },          // REAR_L
    masterConfig.customMotorMixer[2].throttle = 1.0f;
    masterConfig.customMotorMixer[2].roll = 0.414178f;
    masterConfig.customMotorMixer[2].pitch = 1.0f;
    masterConfig.customMotorMixer[2].yaw = 1.0f;

    //  { 1.0f,  0.414178f, -1.0f, -1.0f },          // FRONT_L
    masterConfig.customMotorMixer[3].throttle = 1.0f;
    masterConfig.customMotorMixer[3].roll = 0.414178f;
    masterConfig.customMotorMixer[3].pitch = -1.0f;
    masterConfig.customMotorMixer[3].yaw = -1.0f;

    //  { 1.0f, -1.0f, -0.414178f, -1.0f },          // MIDFRONT_R
    masterConfig.customMotorMixer[4].throttle = 1.0f;
    masterConfig.customMotorMixer[4].roll = -1.0f;
    masterConfig.customMotorMixer[4].pitch = -0.414178f;
    masterConfig.customMotorMixer[4].yaw = -1.0f;

    //  { 1.0f,  1.0f, -0.414178f,  1.0f },          // MIDFRONT_L
    masterConfig.customMotorMixer[5].throttle = 1.0f;
    masterConfig.customMotorMixer[5].roll = 1.0f;
    masterConfig.customMotorMixer[5].pitch = -0.414178f;
    masterConfig.customMotorMixer[5].yaw = 1.0f;

    //  { 1.0f, -1.0f,  0.414178f,  1.0f },          // MIDREAR_R
    masterConfig.customMotorMixer[6].throttle = 1.0f;
    masterConfig.customMotorMixer[6].roll = -1.0f;
    masterConfig.customMotorMixer[6].pitch = 0.414178f;
    masterConfig.customMotorMixer[6].yaw = 1.0f;

    //  { 1.0f,  1.0f,  0.414178f, -1.0f },          // MIDREAR_L
    masterConfig.customMotorMixer[7].throttle = 1.0f;
    masterConfig.customMotorMixer[7].roll = 1.0f;
    masterConfig.customMotorMixer[7].pitch = 0.414178f;
    masterConfig.customMotorMixer[7].yaw = -1.0f;
#endif

#if defined(TARGET_CONFIG)
    targetConfiguration(&masterConfig);
#endif

    // copy first profile into remaining profile
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.profile[i], currentProfile, sizeof(profile_t));
    }

    // copy first control rate config into remaining profile
    for (int i = 1; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.controlRateProfiles[i], currentControlRateProfile, sizeof(controlRateConfig_t));
    }

    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        masterConfig.profile[i].defaultRateProfileIndex = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }
}

void activateControlRateConfig(void)
{
    generateThrottleCurve(currentControlRateProfile, &masterConfig.motorConfig);
}

void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        currentProfile->modeActivationConditions,
        &masterConfig.motorConfig,
        &currentProfile->pidProfile
    );

    gyroConfig()->gyro_soft_lpf_hz = currentProfile->pidProfile.gyro_soft_lpf_hz;

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);

    setAccelerationCalibrationValues(&accelerometerConfig()->accZero, &accelerometerConfig()->accGain);
    setAccelerationFilter(currentProfile->pidProfile.acc_soft_lpf_hz);

    mixerUseConfigs(&masterConfig.flight3DConfig, &masterConfig.motorConfig, &masterConfig.mixerConfig, &masterConfig.rxConfig);
#ifdef USE_SERVOS
    servosUseConfigs(&masterConfig.servoMixerConfig, currentProfile->servoConf, &currentProfile->gimbalConfig, &masterConfig.rxConfig);
#endif

    imuConfigure(&masterConfig.imuConfig, &currentProfile->pidProfile);

    pidInit();

#ifdef NAV
    navigationUseConfig(&masterConfig.navConfig);
    navigationUsePIDs(&currentProfile->pidProfile);
    navigationUseRcControlsConfig(&currentProfile->rcControlsConfig);
    navigationUseRxConfig(&masterConfig.rxConfig);
    navigationUseFlight3DConfig(&masterConfig.flight3DConfig);
    navigationUsemotorConfig(&masterConfig.motorConfig);
#endif

#ifdef BARO
    useBarometerConfig(&masterConfig.barometerConfig);
#endif
#ifdef PITOT
    usePitotmeterConfig(&masterConfig.pitotmeterConfig);
#endif
}

void validateAndFixConfig(void)
{
    // Disable unused features
    featureClear(FEATURE_UNUSED_1 | FEATURE_UNUSED_2);

#ifdef DISABLE_RX_PWM_FEATURE
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }
#endif

    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP) || featureConfigured(FEATURE_RX_SPI))) {
        featureSet(DEFAULT_RX_FEATURE);
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_SPI)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#if defined(CC3D)
        // There is a timer clash between PWM RX pins and motor output pins - this forces us to have same timer tick rate for these timers
        // which is only possible when using brushless motors w/o oneshot (timer tick rate is PWM_TIMER_MHZ)
        // On CC3D OneShot is incompatible with PWM RX
        motorConfig()->motorPwmProtocol = PWM_TYPE_STANDARD;
        motorConfig()->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        // led strip needs the same ports
        featureClear(FEATURE_LED_STRIP);
#endif

        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }

#ifdef USE_SOFTSPI
    if (featureConfigured(FEATURE_SOFTSPI)) {
        featureClear(FEATURE_RX_PPM | FEATURE_RX_PARALLEL_PWM | FEATURE_SOFTSERIAL | FEATURE_VBAT);
#if defined(STM32F10X)
        featureClear(FEATURE_LED_STRIP);
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif
    }
#endif

#ifdef ASYNC_GYRO_PROCESSING
    /*
     * When async processing mode is enabled, gyroSync has to be forced to "ON"
     */
    if (getAsyncMode() != ASYNC_MODE_NONE) {
        gyroConfig()->gyroSync = 1;
    }
#endif

#ifdef STM32F10X
    // avoid overloading the CPU on F1 targets when using gyro sync and GPS.

    if (featureConfigured(FEATURE_GPS)) {
        // avoid overloading the CPU when looptime < 2000 and GPS

        uint8_t denominatorLimit = 2;

        if (gyroConfig()->gyro_lpf == 0) {
            denominatorLimit = 16;
        }

        if (gyroConfig()->gyroSyncDenominator < denominatorLimit) {
            gyroConfig()->gyroSyncDenominator = denominatorLimit;
        }

        if (gyroConfig()->looptime < 2000) {
            gyroConfig()->looptime = 2000;
        }

    }
#else

#endif

#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (WS2811_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (WS2811_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(NAZE) && defined(SONAR)
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(SONAR)
    if (feature(FEATURE_SONAR) && feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(CC3D) && defined(USE_DASHBOARD) && defined(USE_UART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DASHBOARD)) {
        featureClear(FEATURE_DASHBOARD);
    }
#endif

#if defined(CC3D)
#if defined(CC3D_PPM1)
#if defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif
#else
#if defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((featureConfigured(FEATURE_SONAR) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
       featureClear(FEATURE_SONAR);
       featureClear(FEATURE_SOFTSERIAL);
       featureClear(FEATURE_RSSI_ADC);
    }
#endif
#endif // CC3D_PPM1
#endif // CC3D

#ifndef USE_PMW_SERVO_DRIVER
    featureClear(FEATURE_PWM_SERVO_DRIVER);
#endif

    useRxConfig(&masterConfig.rxConfig);

    serialConfig_t *serialConfig = &masterConfig.serialConfig;

    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }

    /*
     * If provided predefined mixer setup is disabled, fallback to default one
     */
    if (!isMixerEnabled(mixerConfig()->mixerMode)) {
        mixerConfig()->mixerMode = DEFAULT_MIXER;
    }

#if defined(NAV)
    // Ensure sane values of navConfig settings
    validateNavConfig(&masterConfig.navConfig);
#endif

    /* Limitations of different protocols */
#ifdef BRUSHED_MOTORS
    motorConfig()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
#else
    switch (motorConfig()->motorPwmProtocol) {
    case PWM_TYPE_STANDARD: // Limited to 490 Hz
        motorConfig()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 490);
        break;

    case PWM_TYPE_ONESHOT125:   // Limited to 3900 Hz
        motorConfig()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 3900);
        break;

    case PWM_TYPE_ONESHOT42:    // 2-8 kHz
        motorConfig()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 8000);
        break;

    case PWM_TYPE_MULTISHOT:    // 2-16 kHz
        motorConfig()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 16000);
        break;
    case PWM_TYPE_BRUSHED:      // 500Hz - 32kHz
        motorConfig()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
        break;
    }
#endif
}

void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch)
{
    updateBoardAlignment(&masterConfig.boardAlignment, roll, pitch);

    saveConfigAndNotify();
}


void readEEPROMAndNotify(void)
{
    // re-read written data
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }

    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROMAndNotify();
}

void changeProfile(uint8_t profileIndex)
{
    masterConfig.current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex > MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = MAX_CONTROL_RATE_PROFILE_COUNT - 1;
    }
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void beeperOffSet(uint32_t mask)
{
    masterConfig.beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
    masterConfig.beeper_off_flags = (1 << beeperCount) -1;
}

void beeperOffClear(uint32_t mask)
{
    masterConfig.beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
    masterConfig.beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
    return masterConfig.beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
    masterConfig.beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
    return masterConfig.preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
    masterConfig.preferred_beeper_off_flags = mask;
}
