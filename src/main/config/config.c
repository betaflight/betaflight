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
#include <stddef.h>

#include <platform.h>

#include "build_config.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "io/rate_profile.h"
#include "io/rc_controls.h"
#include "io/rc_adjustments.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/transponder_ir.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "blackbox/blackbox_io.h"
#include "blackbox/blackbox.h"

#include "telemetry/telemetry.h"
#include "telemetry/hott.h"

#include "flight/mixer.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/profile.h"
#include "config/config_reset.h"
#include "config/config_system.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig) {
    RESET_CONFIG_2(rcControlsConfig_t, rcControlsConfig,
        .deadband = 0,
        .yaw_deadband = 0,
        .alt_hold_deadband = 40,
        .alt_hold_fast_change = 1,
        .yaw_control_direction = 1,
        .deadband3d_throttle = 50,
    );
}

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
        .values.roll = 0,
        .values.pitch = 0,
    );
}

uint16_t getCurrentMinthrottle(void)
{
    return motorAndServoConfig()->minthrottle;
}

// Default settings
STATIC_UNIT_TESTED void resetConf(void)
{
    int i;

    pgResetAll(MAX_PROFILE_COUNT);

    setProfile(0);
    pgActivateProfile(0);

    setControlRateProfile(0);

    featureClearAll();
#if defined(CJMCU) || defined(SPARKY) || defined(COLIBRI_RACE) || defined(MOTOLAB) || defined(SPRACINGF3MINI) || defined(LUX_RACE)
    featureSet(FEATURE_RX_PPM);
#endif

//#if defined(SPRACINGF3MINI)
//    featureSet(FEATURE_DISPLAY);
//#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

    featureSet(FEATURE_FAILSAFE);

    // imu settings
    RESET_CONFIG(imuConfig_t, imuConfig(),
        .dcm_kp = 2500,                // 1.0 * 10000
        .looptime = 2000,
        .gyroSync = 1,
        .gyroSyncDenominator = 1,
        .small_angle = 25,
        .max_angle_inclination = 500,    // 50 degrees
    );

    RESET_CONFIG(gyroConfig_t, gyroConfig(),
        .gyro_lpf = 1,                 // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead
        .soft_gyro_lpf_hz = 60,        // Software based lpf filter for gyro

        .gyroMovementCalibrationThreshold = 32,
    );

#ifdef TELEMETRY
    hottTelemetryConfig()->hottAlarmSoundInterval = 5;
#endif

    RESET_CONFIG_2(rxConfig_t, rxConfig(),
        .sbus_inversion = 1,
        .midrc = 1500,
        .mincheck = 1100,
        .maxcheck = 1900,
        .rx_min_usec = 885,          // any of first 4 channels below this value will trigger rx loss detection
        .rx_max_usec = 2115,         // any of first 4 channels above this value will trigger rx loss detection
        .rssi_scale = RSSI_SCALE_DEFAULT,
    );
    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig()->failsafe_channel_configurations[i];
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(rxConfig()->midrc);
    }

    resetAllRxChannelRangeConfigurations(rxConfig()->channelRanges);

    parseRcChannels("AETR1234", rxConfig());

    RESET_CONFIG(armingConfig_t, armingConfig(),
         .disarm_kill_switch = 1,
         .auto_disarm_delay = 5,
         .max_arm_angle = 25,
    );
    airplaneConfig()->fixedwing_althold_dir = 1;

#ifdef GPS
    // gps/nav stuff
    gpsConfig()->autoConfig = GPS_AUTOCONFIG_ON;
#endif

    systemConfig()->i2c_highspeed = 1;

    resetControlRateConfig(controlRateProfiles(0));

    compassConfig()->mag_declination = 0;

    RESET_CONFIG_2(accelerometerConfig_t, accelerometerConfig(),
        .acc_cut_hz = 15,
        .accz_lpf_cutoff = 5.0f,
        .accDeadband.z = 40,
        .accDeadband.xy = 40,
        .acc_unarmedcal = 1,
    );
    resetRollAndPitchTrims(&accelerometerConfig()->accelerometerTrims);

    // Radio

    resetRcControlsConfig(rcControlsConfig());

    throttleCorrectionConfig()->throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    throttleCorrectionConfig()->throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    RESET_CONFIG(failsafeConfig_t, failsafeConfig(),
        .failsafe_delay = 10,              // 1sec
        .failsafe_off_delay = 200,         // 20sec
        .failsafe_throttle = 1000,         // default throttle off.
        .failsafe_throttle_low_delay = 100, // default throttle low delay for "just disarm" on failsafe condition
    );

#ifdef USE_SERVOS
    // gimbal
    gimbalConfig()->mode = GIMBAL_MODE_NORMAL;
#endif

#ifdef LED_STRIP
    applyDefaultColors();
    applyDefaultLedStripConfig();
#endif

#ifdef TRANSPONDER
    static const uint8_t defaultTransponderData[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC }; // Note, this is NOT a valid transponder code, it's just for testing production hardware

    memcpy(&transponderConfig()->data, &defaultTransponderData, sizeof(defaultTransponderData));
#endif

#ifdef BLACKBOX

#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
    featureSet(FEATURE_BLACKBOX);
    blackboxConfig()->device = BLACKBOX_DEVICE_FLASH;
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
    featureSet(FEATURE_BLACKBOX);
    blackboxConfig()->device = BLACKBOX_DEVICE_SDCARD;
#else
    blackboxConfig()->device = BLACKBOX_DEVICE_SERIAL;
#endif

    blackboxConfig()->rate_num = 1;
    blackboxConfig()->rate_denom = 1;
#endif

    // alternative defaults settings for COLIBRI RACE targets
#if defined(COLIBRI_RACE)
    imuConfig()->looptime = 1000;

    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;

    parseRcChannels("TAER1234", rxConfig());

    featureSet(FEATURE_ONESHOT125);
    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_LED_STRIP);
    featureSet(FEATURE_FAILSAFE);
#endif

    // alternative defaults settings for ALIENWIIF1 and ALIENWIIF3 targets
#ifdef ALIENWII32
    featureSet(FEATURE_RX_SERIAL);
    featureSet(FEATURE_MOTOR_STOP);
# ifdef ALIENWIIF3
    serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    batteryConfig()->vbatscale = 20;
# else
    serialConfig()->portConfigs[1].functionMask = FUNCTION_RX_SERIAL;
# endif
    rxConfig()->serialrx_provider = 1;
    rxConfig()->spektrum_sat_bind = 5;
    motorAndServoConfig()->minthrottle = 1000;
    motorAndServoConfig()->maxthrottle = 2000;
    motorAndServoConfig()->motor_pwm_rate = 32000;
    imuConfig()->looptime = 2000;
    pidProfile()->pidController = 3;
    pidProfile()->P8[PIDROLL] = 36;
    pidProfile()->P8[PIDPITCH] = 36;
    failsafeConfig()->failsafe_delay = 2;
    failsafeConfig()->failsafe_off_delay = 0;
    currentControlRateProfile->rcRate8 = 130;
    currentControlRateProfile->rates[ROLL] = 20;
    currentControlRateProfile->rates[PITCH] = 20;
    currentControlRateProfile->rates[YAW] = 100;
    parseRcChannels("TAER1234", rxConfig());

    //  { 1.0f, -0.414178f,  1.0f, -1.0f },          // REAR_R
    customMotorMixer(0)->throttle = 1.0f;
    customMotorMixer(0)->roll = -0.414178f;
    customMotorMixer(0)->pitch = 1.0f;
    customMotorMixer(0)->yaw = -1.0f;

    //  { 1.0f, -0.414178f, -1.0f,  1.0f },          // FRONT_R
    customMotorMixer(1)->throttle = 1.0f;
    customMotorMixer(1)->roll = -0.414178f;
    customMotorMixer(1)->pitch = -1.0f;
    customMotorMixer(1)->yaw = 1.0f;

    //  { 1.0f,  0.414178f,  1.0f,  1.0f },          // REAR_L
    customMotorMixer(2)->throttle = 1.0f;
    customMotorMixer(2)->roll = 0.414178f;
    customMotorMixer(2)->pitch = 1.0f;
    customMotorMixer(2)->yaw = 1.0f;

    //  { 1.0f,  0.414178f, -1.0f, -1.0f },          // FRONT_L
    customMotorMixer(3)->throttle = 1.0f;
    customMotorMixer(3)->roll = 0.414178f;
    customMotorMixer(3)->pitch = -1.0f;
    customMotorMixer(3)->yaw = -1.0f;

    //  { 1.0f, -1.0f, -0.414178f, -1.0f },          // MIDFRONT_R
    customMotorMixer(4)->throttle = 1.0f;
    customMotorMixer(4)->roll = -1.0f;
    customMotorMixer(4)->pitch = -0.414178f;
    customMotorMixer(4)->yaw = -1.0f;

    //  { 1.0f,  1.0f, -0.414178f,  1.0f },          // MIDFRONT_L
    customMotorMixer(5)->throttle = 1.0f;
    customMotorMixer(5)->roll = 1.0f;
    customMotorMixer(5)->pitch = -0.414178f;
    customMotorMixer(5)->yaw = 1.0f;

    //  { 1.0f, -1.0f,  0.414178f,  1.0f },          // MIDREAR_R
    customMotorMixer(6)->throttle = 1.0f;
    customMotorMixer(6)->roll = -1.0f;
    customMotorMixer(6)->pitch = 0.414178f;
    customMotorMixer(6)->yaw = 1.0f;

    //  { 1.0f,  1.0f,  0.414178f, -1.0f },          // MIDREAR_L
    customMotorMixer(7)->throttle = 1.0f;
    customMotorMixer(7)->roll = 1.0f;
    customMotorMixer(7)->pitch = 0.414178f;
    customMotorMixer(7)->yaw = -1.0f;
#endif

    // copy first profile into remaining profile
    PG_FOREACH_PROFILE(reg) {
        for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
            memcpy(reg->address + i * pgSize(reg), reg->address, pgSize(reg));
        }
    }

    // FIXME implement differently

    // copy first control rate config into remaining profile
    for (i = 1; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        memcpy(controlRateProfiles(i), controlRateProfiles(0), sizeof(controlRateConfig_t));
    }

    // TODO
    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        configureRateProfileSelection(i, i % MAX_CONTROL_RATE_PROFILE_COUNT);
    }
}

void activateConfig(void)
{
    static imuRuntimeConfig_t imuRuntimeConfig;

    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        modeActivationProfile()->modeActivationConditions
    );

    pidSetController(pidProfile()->pidController);

#ifdef GPS
    gpsUsePIDs(pidProfile());
#endif

    useFailsafeConfig();
    setAccelerationTrims(&sensorTrims()->accZero);

    mixerUseConfigs(
#ifdef USE_SERVOS
        servoProfile()->servoConf
#endif
    );

    recalculateMagneticDeclination();


    imuRuntimeConfig.dcm_kp = imuConfig()->dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = imuConfig()->dcm_ki / 10000.0f;
    imuRuntimeConfig.acc_cut_hz = accelerometerConfig()->acc_cut_hz;
    imuRuntimeConfig.acc_unarmedcal = accelerometerConfig()->acc_unarmedcal;
    imuRuntimeConfig.small_angle = imuConfig()->small_angle;

    imuConfigure(
        &imuRuntimeConfig,
        &accelerometerConfig()->accDeadband,
        accelerometerConfig()->accz_lpf_cutoff,
        throttleCorrectionConfig()->throttle_correction_angle
    );
}

void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(DEFAULT_RX_FEATURE);
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_SERIAL | FEATURE_RX_MSP);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM);
    }

#ifdef STM32F10X
    // avoid overloading the CPU on F1 targets when using gyro sync and GPS.
    if (imuConfig()->gyroSync && imuConfig()->gyroSyncDenominator < 2 && featureConfigured(FEATURE_GPS)) {
        imuConfig()->gyroSyncDenominator = 2;
    }
#endif

#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (LED_STRIP_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (LED_STRIP_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(CC3D) && defined(DISPLAY) && defined(USE_UART3)
    if (doesConfigurationUsePort(SERIAL_PORT_UART3) && featureConfigured(FEATURE_DISPLAY)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

#ifdef STM32F303xC
    // hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
#ifdef TELEMETRY
    telemetryConfig()->telemetry_inversion = 1;
#endif
#endif

    /*
     * The retarded_arm setting is incompatible with pid_at_min_throttle because full roll causes the craft to roll over on the ground.
     * The pid_at_min_throttle implementation ignores yaw on the ground, but doesn't currently ignore roll when retarded_arm is enabled.
     */
    if (armingConfig()->retarded_arm && mixerConfig()->pid_at_min_throttle) {
        mixerConfig()->pid_at_min_throttle = 0;
    }

#if defined(LED_STRIP) && defined(TRANSPONDER) && !defined(UNIT_TEST)
    if ((WS2811_DMA_TC_FLAG == TRANSPONDER_DMA_TC_FLAG) && featureConfigured(FEATURE_TRANSPONDER) && featureConfigured(FEATURE_LED_STRIP)) {
        featureClear(FEATURE_LED_STRIP);
    }
#endif


#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif

#if defined(COLIBRI_RACE)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
    if(featureConfigured(FEATURE_RX_SERIAL)) {
        serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

    if (!isSerialConfigValid(serialConfig())) {
        pgReset_serialConfig(serialConfig());
    }
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check
    // Read flash
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    pgActivateProfile(getCurrentProfile());

    if (rateProfileSelection()->defaultRateProfileIndex > MAX_CONTROL_RATE_PROFILE_COUNT - 1) // sanity check
        rateProfileSelection()->defaultRateProfileIndex = 0;

    setControlRateProfile(rateProfileSelection()->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void writeEEPROM(void)
{
    suspendRxSignal();

    writeConfigToEEPROM();

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
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void changeProfile(uint8_t profileIndex)
{
    setProfile(profileIndex);
    writeEEPROM();
    readEEPROM();
}

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
    StopPwmAllMotors();
    delay(50);
    // Apply additional delay when OneShot125 feature changed from on to off state
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
        delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
    }
}
