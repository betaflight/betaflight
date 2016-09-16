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
#include "debug.h"

#include "build_config.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/pwm_output.h"
#include "drivers/max7456.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/vtx.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#ifdef STM32F4
#define BRUSHLESS_MOTORS_PWM_RATE 2000
#else
#define BRUSHLESS_MOTORS_PWM_RATE 400
#endif

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);
void targetConfiguration(master_t *config);

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif


#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #if defined(STM32F40_41xxx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

    #if defined (STM32F411xE)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#if defined(STM32F40_41xxx)
#define FLASH_PAGE_COUNT 4 // just to make calculations work
#elif defined (STM32F411xE)
#define FLASH_PAGE_COUNT 4 // just to make calculations work
#else
#define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 128
#define FLASH_TO_RESERVE_FOR_CONFIG 0x800
#else
#define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

// use the last flash pages for storage
#ifdef CUSTOM_FLASH_MEMORY_ADDRESS
size_t custom_flash_memory_address = 0;
#define CONFIG_START_FLASH_ADDRESS (custom_flash_memory_address)
#else
// use the last flash pages for storage
#ifndef CONFIG_START_FLASH_ADDRESS
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))
#endif
#endif

master_t masterConfig;                 // master config struct with data independent from profiles
profile_t *currentProfile;
static uint32_t activeFeaturesLatch = 0;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

static const uint8_t EEPROM_CONF_VERSION = 146;

static void resetAccelerometerTrims(flightDynamicsTrims_t *accelerometerTrims)
{
    accelerometerTrims->values.pitch = 0;
    accelerometerTrims->values.roll = 0;
    accelerometerTrims->values.yaw = 0;
}

static void resetControlRateConfig(controlRateConfig_t *controlRateConfig)
{
    controlRateConfig->rcRate8 = 100;
    controlRateConfig->rcYawRate8 = 100;
    controlRateConfig->rcExpo8 = 0;
    controlRateConfig->thrMid8 = 50;
    controlRateConfig->thrExpo8 = 0;
    controlRateConfig->dynThrPID = 10;
    controlRateConfig->rcYawExpo8 = 0;
    controlRateConfig->tpa_breakpoint = 1650;

    for (uint8_t axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        controlRateConfig->rates[axis] = 70;
    }
}

static void resetPidProfile(pidProfile_t *pidProfile)
{
#if defined(SKIP_PID_FLOAT)
    pidProfile->pidController = PID_CONTROLLER_LEGACY;
#else
    pidProfile->pidController = PID_CONTROLLER_BETAFLIGHT;
#endif

    pidProfile->P8[ROLL] = 45;
    pidProfile->I8[ROLL] = 40;
    pidProfile->D8[ROLL] = 20;
    pidProfile->P8[PITCH] = 60;
    pidProfile->I8[PITCH] = 65;
    pidProfile->D8[PITCH] = 22;
    pidProfile->P8[YAW] = 70;
    pidProfile->I8[YAW] = 45;
    pidProfile->D8[YAW] = 20;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15;   // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0;    // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34;  // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 14;  // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 53;  // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 25;  // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 33;  // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 83;  // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 50;
    pidProfile->I8[PIDLEVEL] = 50;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 55;
    pidProfile->I8[PIDVEL] = 55;
    pidProfile->D8[PIDVEL] = 75;

    pidProfile->yaw_p_limit = YAW_P_LIMIT_MAX;
    pidProfile->yaw_lpf_hz = 0;
    pidProfile->rollPitchItermIgnoreRate = 130;
    pidProfile->yawItermIgnoreRate = 32;
    pidProfile->dterm_filter_type = FILTER_BIQUAD;
    pidProfile->dterm_lpf_hz = 100;    // filtering ON by default
    pidProfile->dterm_notch_hz = 260;
    pidProfile->dterm_notch_cutoff = 160;
    pidProfile->deltaMethod = DELTA_FROM_MEASUREMENT;
    pidProfile->vbatPidCompensation = 0;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;

    // Betaflight PID controller parameters
    pidProfile->ptermSRateWeight = 85;
    pidProfile->dtermSetpointWeight = 150;
    pidProfile->yawRateAccelLimit = 220;
    pidProfile->rateAccelLimit = 0;
    pidProfile->itermThrottleGain = 0;

#ifdef GTUNE
    pidProfile->gtune_lolimP[ROLL] = 10;          // [0..200] Lower limit of ROLL P during G tune.
    pidProfile->gtune_lolimP[PITCH] = 10;         // [0..200] Lower limit of PITCH P during G tune.
    pidProfile->gtune_lolimP[YAW] = 10;           // [0..200] Lower limit of YAW P during G tune.
    pidProfile->gtune_hilimP[ROLL] = 100;         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[PITCH] = 100;        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[YAW] = 100;          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_pwr = 0;                    // [0..10] Strength of adjustment
    pidProfile->gtune_settle_time = 450;          // [200..1000] Settle time in ms
    pidProfile->gtune_average_cycles = 16;        // [8..128] Number of looptime cycles used for gyro average calculation
#endif
}

void resetProfile(profile_t *profile)
{
    resetPidProfile(&profile->pidProfile);

    for (int rI = 0; rI<MAX_RATEPROFILES; rI++) {
        resetControlRateConfig(&profile->controlRateProfile[rI]);
    }

    profile->activeRateProfile = 0;
}

#ifdef GPS
void resetGpsProfile(gpsProfile_t *gpsProfile)
{
    gpsProfile->gps_wp_radius = 200;
    gpsProfile->gps_lpf = 20;
    gpsProfile->nav_slew_rate = 30;
    gpsProfile->nav_controls_heading = 1;
    gpsProfile->nav_speed_min = 100;
    gpsProfile->nav_speed_max = 300;
    gpsProfile->ap_mode = 40;
}
#endif

#ifdef BARO
void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 0.6f;
    barometerConfig->baro_cf_vel = 0.985f;
    barometerConfig->baro_cf_alt = 0.965f;
}
#endif

void resetSensorAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    sensorAlignmentConfig->gyro_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->acc_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->mag_align = ALIGN_DEFAULT;
}

void resetEscAndServoConfig(escAndServoConfig_t *escAndServoConfig)
{
#ifdef BRUSHED_MOTORS
    escAndServoConfig->minthrottle = 1000;
#else
    escAndServoConfig->minthrottle = 1070;
#endif
    escAndServoConfig->maxthrottle = 2000;
    escAndServoConfig->mincommand = 1000;
    escAndServoConfig->servoCenterPulse = 1500;
    escAndServoConfig->maxEscThrottleJumpMs = 0;
}

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
    telemetryConfig->telemetry_inversion = 1;
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_METRICS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->frsky_vfas_cell_voltage = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
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
    batteryConfig->vbathysteresis = 1;
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
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_57600;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#if defined(USE_VCP)
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig)
{
    rcControlsConfig->deadband = 0;
    rcControlsConfig->yaw_deadband = 0;
    rcControlsConfig->alt_hold_deadband = 40;
    rcControlsConfig->alt_hold_fast_change = 1;
}

void resetMixerConfig(mixerConfig_t *mixerConfig)
{
    mixerConfig->yaw_motor_direction = 1;
#ifdef USE_SERVOS
    mixerConfig->tri_unarmed_servo = 1;
    mixerConfig->servo_lowpass_freq = 400;
    mixerConfig->servo_lowpass_enable = 0;
#endif
}

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

static void setProfile(uint8_t profileIndex)
{
    currentProfile = &masterConfig.profile[profileIndex];
    currentControlRateProfileIndex = currentProfile->activeRateProfile;
    currentControlRateProfile = &currentProfile->controlRateProfile[currentControlRateProfileIndex];
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

static void setControlRateProfile(uint8_t profileIndex)
{
    currentControlRateProfileIndex = profileIndex;
    masterConfig.profile[getCurrentProfile()].activeRateProfile = profileIndex;
    currentControlRateProfile = &masterConfig.profile[getCurrentProfile()].controlRateProfile[profileIndex];
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex)
{
    return &masterConfig.profile[profileIndex].controlRateProfile[masterConfig.profile[profileIndex].activeRateProfile];
}

uint16_t getCurrentMinthrottle(void)
{
    return masterConfig.escAndServoConfig.minthrottle;
}

static void intFeatureClearAll(master_t *config);
static void intFeatureSet(uint32_t mask, master_t *config);
static void intFeatureClear(uint32_t mask, master_t *config);

// Default settings
void createDefaultConfig(master_t *config)
{
    // Clear all configuration
    memset(config, 0, sizeof(master_t));

    intFeatureClearAll(config);
    intFeatureSet(DEFAULT_RX_FEATURE | FEATURE_FAILSAFE , config);
#ifdef DEFAULT_FEATURES
    intFeatureSet(DEFAULT_FEATURES, config);
#endif

#ifdef OSD
    intFeatureSet(FEATURE_OSD, config);
    resetOsdConfig(&config->osdProfile);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    intFeatureSet(FEATURE_VBAT, config);
#endif

    config->version = EEPROM_CONF_VERSION;
    config->mixerMode = MIXER_QUADX;

    // global settings
    config->current_profile_index = 0;     // default profile
    config->dcm_kp = 2500;                // 1.0 * 10000
    config->dcm_ki = 0;                    // 0.003 * 10000
    config->gyro_lpf = 0;                 // 256HZ default
#ifdef STM32F10X
    config->gyro_sync_denom = 8;
    config->pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)
    config->gyro_sync_denom = 1;
    config->pid_process_denom = 4;
#else
    config->gyro_sync_denom = 4;
    config->pid_process_denom = 2;
#endif
    config->gyro_soft_type = FILTER_PT1;
    config->gyro_soft_lpf_hz = 90;
    config->gyro_soft_notch_hz = 0;
    config->gyro_soft_notch_cutoff = 130;

    config->debug_mode = DEBUG_NONE;

    resetAccelerometerTrims(&config->accZero);

    resetSensorAlignment(&config->sensorAlignmentConfig);

    config->boardAlignment.rollDegrees = 0;
    config->boardAlignment.pitchDegrees = 0;
    config->boardAlignment.yawDegrees = 0;
    config->acc_hardware = ACC_DEFAULT;     // default/autodetect
    config->max_angle_inclination = 700;    // 70 degrees
    config->yaw_control_direction = 1;
    config->gyroConfig.gyroMovementCalibrationThreshold = 32;

    // xxx_hardware: 0:default/autodetect, 1: disable
    config->mag_hardware = 1;

    config->baro_hardware = 1;

    resetBatteryConfig(&config->batteryConfig);

#ifdef TELEMETRY
    resetTelemetryConfig(&config->telemetryConfig);
#endif

#ifdef SERIALRX_PROVIDER
    config->rxConfig.serialrx_provider = SERIALRX_PROVIDER;
#else
    config->rxConfig.serialrx_provider = 0;
#endif
    config->rxConfig.sbus_inversion = 1;
    config->rxConfig.spektrum_sat_bind = 0;
    config->rxConfig.spektrum_sat_bind_autoreset = 1;
    config->rxConfig.midrc = 1500;
    config->rxConfig.mincheck = 1100;
    config->rxConfig.maxcheck = 1900;
    config->rxConfig.rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
    config->rxConfig.rx_max_usec = 2115;         // any of first 4 channels above this value will trigger rx loss detection

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &config->rxConfig.failsafe_channel_configurations[i];
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.midrc);
    }

    config->rxConfig.rssi_channel = 0;
    config->rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
    config->rxConfig.rssi_ppm_invert = 0;
    config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;
    config->rxConfig.rcInterpolationInterval = 19;
    config->rxConfig.fpvCamAngleDegrees = 0;
    config->rxConfig.max_aux_channel = MAX_AUX_CHANNELS;
    config->rxConfig.airModeActivateThreshold = 1350;

    resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);

    config->inputFilteringMode = INPUT_FILTERING_DISABLED;

    config->gyro_cal_on_first_arm = 0;  // TODO - Cleanup retarded arm support
    config->disarm_kill_switch = 1;
    config->auto_disarm_delay = 5;
    config->small_angle = 25;

    resetMixerConfig(&config->mixerConfig);

    config->airplaneConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetEscAndServoConfig(&config->escAndServoConfig);
    resetFlight3DConfig(&config->flight3DConfig);

#ifdef BRUSHED_MOTORS
    config->motor_pwm_rate = BRUSHED_MOTORS_PWM_RATE;
    config->motor_pwm_protocol = PWM_TYPE_BRUSHED;
    config->use_unsyncedPwm = true;
#else
    config->motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
    config->motor_pwm_protocol = PWM_TYPE_ONESHOT125;
#endif

    config->servo_pwm_rate = 50;

#ifdef CC3D
    config->use_buzzer_p6 = 0;
#endif

#ifdef GPS
    // gps/nav stuff
    config->gpsConfig.provider = GPS_NMEA;
    config->gpsConfig.sbasMode = SBAS_AUTO;
    config->gpsConfig.autoConfig = GPS_AUTOCONFIG_ON;
    config->gpsConfig.autoBaud = GPS_AUTOBAUD_OFF;
#endif

    resetSerialConfig(&config->serialConfig);

    resetProfile(&config->profile[0]);

    resetRollAndPitchTrims(&config->accelerometerTrims);

    config->mag_declination = 0;
    config->acc_lpf_hz = 10.0f;
    config->accDeadband.xy = 40;
    config->accDeadband.z = 40;
    config->acc_unarmedcal = 1;

#ifdef BARO
    resetBarometerConfig(&config->barometerConfig);
#endif

    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", &config->rxConfig);
#else
    parseRcChannels("AETR1234", &config->rxConfig);
#endif

    resetRcControlsConfig(&config->rcControlsConfig);

    config->throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    config->throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    config->failsafeConfig.failsafe_delay = 10;                            // 1sec
    config->failsafeConfig.failsafe_off_delay = 10;                        // 1sec
    config->failsafeConfig.failsafe_throttle = 1000;                       // default throttle off.
    config->failsafeConfig.failsafe_kill_switch = 0;                       // default failsafe switch action is identical to rc link loss
    config->failsafeConfig.failsafe_throttle_low_delay = 100;              // default throttle low delay for "just disarm" on failsafe condition
    config->failsafeConfig.failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;// default full failsafe procedure is 0: auto-landing

#ifdef USE_SERVOS
    // servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        config->servoConf[i].min = DEFAULT_SERVO_MIN;
        config->servoConf[i].max = DEFAULT_SERVO_MAX;
        config->servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        config->servoConf[i].rate = 100;
        config->servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        config->servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        config->servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    // gimbal
    config->gimbalConfig.mode = GIMBAL_MODE_NORMAL;
#endif

#ifdef GPS
    resetGpsProfile(&config->gpsProfile);
#endif

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        config->customMotorMixer[i].throttle = 0.0f;
    }

#ifdef LED_STRIP
    applyDefaultColors(config->colors);
    applyDefaultLedStripConfig(config->ledConfigs);
    applyDefaultModeColors(config->modeColors);
    applyDefaultSpecialColors(&(config->specialColors));
    config->ledstrip_visual_beeper = 0;
#endif

#ifdef VTX
    config->vtx_band = 4;    //Fatshark/Airwaves
    config->vtx_channel = 1; //CH1
    config->vtx_mode = 0;    //CH+BAND mode
    config->vtx_mhz = 5740;  //F0
#endif

#ifdef TRANSPONDER
    static const uint8_t defaultTransponderData[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC }; // Note, this is NOT a valid transponder code, it's just for testing production hardware

    memcpy(config->transponderData, &defaultTransponderData, sizeof(defaultTransponderData));
#endif

#ifdef BLACKBOX
#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
    intFeatureSet(FEATURE_BLACKBOX, config);
    config->blackbox_device = BLACKBOX_DEVICE_FLASH;
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
    intFeatureSet(FEATURE_BLACKBOX, config);
    config->blackbox_device = BLACKBOX_DEVICE_SDCARD;
#else
    config->blackbox_device = BLACKBOX_DEVICE_SERIAL;
#endif

    config->blackbox_rate_num = 1;
    config->blackbox_rate_denom = 1;
    config->blackbox_on_motor_test = 0; // default off
#endif // BLACKBOX

#ifdef SERIALRX_UART
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        config->serialConfig.portConfigs[SERIALRX_UART].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

#if defined(TARGET_CONFIG)
    targetConfiguration(config);
#endif

   
    // copy first profile into remaining profile
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        memcpy(&config->profile[i], &config->profile[0], sizeof(profile_t));
    }
}

static void resetConf(void)
{
    createDefaultConfig(&masterConfig);

    setProfile(0);

#ifdef LED_STRIP
    reevaluateLedConfig();
#endif
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;
    return checksum;
}

static bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

void activateControlRateConfig(void)
{
    generateThrottleCurve(currentControlRateProfile, &masterConfig.escAndServoConfig);
}

void activateConfig(void)
{
    static imuRuntimeConfig_t imuRuntimeConfig;

    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        masterConfig.modeActivationConditions,
        &masterConfig.escAndServoConfig,
        &currentProfile->pidProfile
    );

    gyroUseConfig(&masterConfig.gyroConfig, masterConfig.gyro_soft_lpf_hz, masterConfig.gyro_soft_notch_hz, masterConfig.gyro_soft_notch_cutoff, masterConfig.gyro_soft_type);

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif
    pidSetController(currentProfile->pidProfile.pidController);

#ifdef GPS
    gpsUseProfile(&masterConfig.gpsProfile);
    gpsUsePIDs(&currentProfile->pidProfile);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);
    setAccelerationTrims(&masterConfig.accZero);
    setAccelerationFilter(masterConfig.acc_lpf_hz);

    mixerUseConfigs(
#ifdef USE_SERVOS
        masterConfig.servoConf,
        &masterConfig.gimbalConfig,
#endif
        &masterConfig.flight3DConfig,
        &masterConfig.escAndServoConfig,
        &masterConfig.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig
    );

    imuRuntimeConfig.dcm_kp = masterConfig.dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = masterConfig.dcm_ki / 10000.0f;
    imuRuntimeConfig.acc_unarmedcal = masterConfig.acc_unarmedcal;
    imuRuntimeConfig.small_angle = masterConfig.small_angle;

    imuConfigure(
        &imuRuntimeConfig,
        &currentProfile->pidProfile,
        &masterConfig.accDeadband,
        masterConfig.throttle_correction_angle
    );

    configureAltitudeHold(
        &currentProfile->pidProfile,
        &masterConfig.barometerConfig,
        &masterConfig.rcControlsConfig,
        &masterConfig.escAndServoConfig
    );

#ifdef BARO
    useBarometerConfig(&masterConfig.barometerConfig);
#endif
}

void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(DEFAULT_RX_FEATURE);
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL);
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        // led strip needs the same ports
        featureClear(FEATURE_LED_STRIP);
#endif

        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }


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
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(SONAR)
    if (feature(FEATURE_SONAR) && feature(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(CC3D) && defined(DISPLAY) && defined(USE_UART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DISPLAY)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

/*#if defined(LED_STRIP) && defined(TRANSPONDER) // TODO - Add transponder feature
    if ((WS2811_DMA_TC_FLAG == TRANSPONDER_DMA_TC_FLAG) && featureConfigured(FEATURE_TRANSPONDER) && featureConfigured(FEATURE_LED_STRIP)) {
        featureClear(FEATURE_LED_STRIP);
    }
#endif
*/

#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((featureConfigured(FEATURE_SONAR) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
        featureClear(FEATURE_SONAR);
        featureClear(FEATURE_SOFTSERIAL);
        featureClear(FEATURE_RSSI_ADC);
    }
#endif

#if defined(COLIBRI_RACE)
    masterConfig.serialConfig.portConfigs[0].functionMask = FUNCTION_MSP;
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_MSP);
        featureSet(FEATURE_RX_PPM);
    }
#endif

    useRxConfig(&masterConfig.rxConfig);

    serialConfig_t *serialConfig = &masterConfig.serialConfig;

    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }
}

void initEEPROM(void)
{
}

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);

    suspendRxSignal();

    // Read flash
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));

    if (masterConfig.current_profile_index > MAX_PROFILE_COUNT - 1) // sanity check
        masterConfig.current_profile_index = 0;

    setProfile(masterConfig.current_profile_index);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void readEEPROMAndNotify(void)
{
    // re-read written data
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void writeEEPROM(void)
{
    // Generate compile time error if the config does not fit in the reserved area of flash.
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    suspendRxSignal();

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
#if defined(STM32F4)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F303)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F40_41xxx)
                status = FLASH_EraseSector(FLASH_Sector_8, VoltageRange_3); //0x08080000 to 0x080A0000
#elif defined (STM32F411xE)
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); //0x08060000 to 0x08080000
#else
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
#endif
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset,
                    *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
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
    if (profileIndex > MAX_RATEPROFILES) {
        profileIndex = MAX_RATEPROFILES - 1;
    }
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void latchActiveFeatures()
{
    activeFeaturesLatch = masterConfig.enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return masterConfig.enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    intFeatureSet(mask, &masterConfig);
}

static void intFeatureSet(uint32_t mask, master_t *config)
{
    config->enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    intFeatureClear(mask, &masterConfig);
}

static void intFeatureClear(uint32_t mask, master_t *config)
{
    config->enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    intFeatureClearAll(&masterConfig);
}

static void intFeatureClearAll(master_t *config)
{
    config->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return masterConfig.enabledFeatures;
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
