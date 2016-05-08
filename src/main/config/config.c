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
#include "sensors/compass.h"
#include "sensors/acceleration.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/ledstrip.h"

#include "blackbox/blackbox_io.h"
#include "blackbox/blackbox.h"

#include "telemetry/telemetry.h"

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

// FIXME remove the includes below when target specific configuration is moved out of this file
#include "sensors/battery.h"


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

// Default settings
STATIC_UNIT_TESTED void resetConf(void)
{
    pgResetAll(MAX_PROFILE_COUNT);

    setProfile(0);
    pgActivateProfile(0);

    setControlRateProfile(0);

    featureClearAll();

    featureSet(DEFAULT_RX_FEATURE);

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

    featureSet(FEATURE_FAILSAFE);

    parseRcChannels("AETR1234", rxConfig());

    featureSet(FEATURE_BLACKBOX);

#if defined(COLIBRI_RACE)
    // alternative defaults settings for COLIBRI RACE targets
    imuConfig()->looptime = 1000;
    featureSet(FEATURE_ONESHOT125);
    featureSet(FEATURE_LED_STRIP);
#endif

#ifdef SPRACINGF3EVO
    featureSet(FEATURE_TRANSPONDER);
    featureSet(FEATURE_RSSI_ADC);
    featureSet(FEATURE_CURRENT_METER);
    featureSet(FEATURE_TELEMETRY);
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
    rxConfig()->serialrx_provider = SERIALRX_SPEKTRUM2048;
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
    *customMotorMixer(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixer(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixer(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixer(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *customMotorMixer(4) = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *customMotorMixer(5) = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *customMotorMixer(6) = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *customMotorMixer(7) = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
#endif

    // copy first profile into remaining profile
    PG_FOREACH_PROFILE(reg) {
        for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
            memcpy(reg->address + i * pgSize(reg), reg->address, pgSize(reg));
        }
    }
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        configureRateProfileSelection(i, i % MAX_CONTROL_RATE_PROFILE_COUNT);
    }
}

void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(modeActivationProfile()->modeActivationConditions);

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

    static imuRuntimeConfig_t imuRuntimeConfig;
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

#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((featureConfigured(FEATURE_SONAR) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
    	featureClear(FEATURE_SONAR);
    	featureClear(FEATURE_SOFTSERIAL);
    	featureClear(FEATURE_RSSI_ADC);
    }
#endif

#if defined(COLIBRI_RACE)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        serialConfig()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

    if (!isSerialConfigValid(serialConfig())) {
        PG_RESET_CURRENT(serialConfig);
    }

#if defined(USE_VCP)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP;
#endif
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    pgActivateProfile(getCurrentProfile());

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
