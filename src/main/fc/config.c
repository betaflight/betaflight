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
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "cms/cms.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/io.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/max7456.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/vcd.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/vtx.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

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
#ifdef STM32F4
#define BRUSHLESS_MOTORS_PWM_RATE 2000
#else
#define BRUSHLESS_MOTORS_PWM_RATE 400
#endif

master_t masterConfig;                 // master config struct with data independent from profiles
profile_t *currentProfile;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

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
    pidProfile->P8[ROLL] = 40;
    pidProfile->I8[ROLL] = 40;
    pidProfile->D8[ROLL] = 20;
    pidProfile->P8[PITCH] = 55;
    pidProfile->I8[PITCH] = 50;
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
    pidProfile->pidSumLimit = PIDSUM_LIMIT;
    pidProfile->yaw_lpf_hz = 0;
    pidProfile->rollPitchItermIgnoreRate = 200;
    pidProfile->yawItermIgnoreRate = 55;
    pidProfile->dterm_filter_type = FILTER_BIQUAD;
    pidProfile->dterm_lpf_hz = 100;    // filtering ON by default
    pidProfile->dterm_notch_hz = 260;
    pidProfile->dterm_notch_cutoff = 160;
    pidProfile->vbatPidCompensation = 0;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;
    pidProfile->levelAngleLimit = 55;
    pidProfile->levelSensitivity = 55;
    pidProfile->setpointRelaxRatio = 30;
    pidProfile->dtermSetpointWeight = 200;
    pidProfile->yawRateAccelLimit = 10.0f;
    pidProfile->rateAccelLimit = 0.0f;
    pidProfile->itermThrottleThreshold = 300;
    pidProfile->itermAcceleratorGain = 4.0f;
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

#ifdef LED_STRIP
void resetLedStripConfig(ledStripConfig_t *ledStripConfig)
{
    applyDefaultColors(ledStripConfig->colors);
    applyDefaultLedStripConfig(ledStripConfig->ledConfigs);
    applyDefaultModeColors(ledStripConfig->modeColors);
    applyDefaultSpecialColors(&(ledStripConfig->specialColors));
    ledStripConfig->ledstrip_visual_beeper = 0;
    ledStripConfig->ledstrip_aux_channel = THROTTLE;

    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_LED) {
            ledStripConfig->ioTag = timerHardware[i].tag;
            return;
        }
    }
    ledStripConfig->ioTag = IO_TAG_NONE;
}
#endif

#ifdef USE_SERVOS
void resetServoConfig(servoConfig_t *servoConfig)
{
    servoConfig->servoCenterPulse = 1500;
    servoConfig->servoPwmRate = 50;

    int servoIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && servoIndex < MAX_SUPPORTED_SERVOS; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_SERVO) {
            servoConfig->ioTags[servoIndex] = timerHardware[i].tag;
            servoIndex++;
        }
    }
}
#endif

void resetMotorConfig(motorConfig_t *motorConfig)
{
#ifdef BRUSHED_MOTORS
    motorConfig->minthrottle = 1000;
    motorConfig->motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfig->motorPwmProtocol = PWM_TYPE_BRUSHED;
    motorConfig->useUnsyncedPwm = true;
#else
#ifdef BRUSHED_ESC_AUTODETECT
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfig->minthrottle = 1000;
        motorConfig->motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfig->motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfig->useUnsyncedPwm = true;
    } else
#endif
    {
        motorConfig->minthrottle = 1070;
        motorConfig->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
        motorConfig->motorPwmProtocol = PWM_TYPE_ONESHOT125;
    }
#endif
    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->digitalIdleOffsetPercent = 3.0f;

    int motorIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_MOTORS; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
            motorConfig->ioTags[motorIndex] = timerHardware[i].tag;
            motorIndex++;
        }
    }
}

#ifdef SONAR
void resetSonarConfig(sonarConfig_t *sonarConfig)
{
#if defined(SONAR_TRIGGER_PIN) && defined(SONAR_ECHO_PIN)
    sonarConfig->triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
    sonarConfig->echoTag = IO_TAG(SONAR_ECHO_PIN);
#else
#error Sonar not defined for target
#endif
}
#endif

#ifdef USE_SDCARD
void resetsdcardConfig(sdcardConfig_t *sdcardConfig)
{
#if defined(SDCARD_DMA_CHANNEL_TX)
    sdcardConfig->useDma = true;
#else
    sdcardConfig->useDma = false;
#endif
}
#endif

#ifdef USE_ADC
void resetAdcConfig(adcConfig_t *adcConfig)
{
#ifdef VBAT_ADC_PIN
    adcConfig->vbat.enabled = true;
    adcConfig->vbat.ioTag = IO_TAG(VBAT_ADC_PIN);
#endif

#ifdef EXTERNAL1_ADC_PIN
    adcConfig->external1.enabled = true;
    adcConfig->external1.ioTag = IO_TAG(EXTERNAL1_ADC_PIN);
#endif

#ifdef CURRENT_METER_ADC_PIN
    adcConfig->currentMeter.enabled = true;
    adcConfig->currentMeter.ioTag = IO_TAG(CURRENT_METER_ADC_PIN);
#endif

#ifdef RSSI_ADC_PIN
    adcConfig->rssi.enabled = true;
    adcConfig->rssi.ioTag = IO_TAG(RSSI_ADC_PIN);
#endif

}
#endif


#ifdef BEEPER
void resetBeeperConfig(beeperConfig_t *beeperConfig)
{
#ifdef BEEPER_INVERTED
    beeperConfig->isOpenDrain = false;
    beeperConfig->isInverted = true;
#else
    beeperConfig->isOpenDrain = true;
    beeperConfig->isInverted = false;
#endif
    beeperConfig->ioTag = IO_TAG(BEEPER);
}
#endif

#if defined(USE_PWM) || defined(USE_PPM)
void resetPpmConfig(ppmConfig_t *ppmConfig)
{
#ifdef PPM_PIN
    ppmConfig->ioTag = IO_TAG(PPM_PIN);
#else
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PPM) {
            ppmConfig->ioTag = timerHardware[i].tag;
            return;
        }
    }

    ppmConfig->ioTag = IO_TAG_NONE;
#endif
}

void resetPwmConfig(pwmConfig_t *pwmConfig)
{
    int inputIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && inputIndex < PWM_INPUT_PORT_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PWM) {
            pwmConfig->ioTags[inputIndex] = timerHardware[i].tag;
            inputIndex++;
        }
    }
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
    telemetryConfig->telemetry_inversion = 1;
    telemetryConfig->sportHalfDuplex = 1;
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_METRICS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->frsky_vfas_cell_voltage = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
    telemetryConfig->pidValuesAsTelemetry = 0;
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
    batteryConfig->batteryMeterType = BATTERY_SENSOR_ADC;
    batteryConfig->currentMeterOffset = 0;
    batteryConfig->currentMeterScale = 400; // for Allegro ACS758LCB-100U (40mV/A)
    batteryConfig->batteryCapacity = 0;
    batteryConfig->currentMeterType = CURRENT_SENSOR_ADC;
    batteryConfig->batterynotpresentlevel = 55; // VBAT below 5.5 V will be igonored
    batteryConfig->useVBatAlerts = true;
    batteryConfig->useConsumptionAlerts = false;
    batteryConfig->consumptionWarningPercentage = 10;
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
    memset(serialConfig, 0, sizeof(serialConfig_t));
    serialConfig->serial_update_rate_hz =  100;
    serialConfig->reboot_character = 'R';

    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialConfig->portConfigs[index].identifier = serialPortIdentifiers[index];
        serialConfig->portConfigs[index].msp_baudrateIndex = BAUD_115200;
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_57600;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;
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
#ifdef TARGET_DEFAULT_MIXER
    mixerConfig->mixerMode = TARGET_DEFAULT_MIXER;
#else
    mixerConfig->mixerMode = MIXER_QUADX;
#endif
    mixerConfig->yaw_motor_direction = 1;
}

#ifdef USE_SERVOS
void resetServoMixerConfig(servoMixerConfig_t *servoMixerConfig)
{
    servoMixerConfig->tri_unarmed_servo = 1;
    servoMixerConfig->servo_lowpass_freq = 400;
    servoMixerConfig->servo_lowpass_enable = 0;
}
#endif

#ifdef USE_MAX7456
void resetMax7456Config(vcdProfile_t *pVcdProfile)
{
    pVcdProfile->video_system = VIDEO_SYSTEM_AUTO;
    pVcdProfile->h_offset = 0;
    pVcdProfile->v_offset = 0;
}
#endif

void resetDisplayPortProfile(displayPortProfile_t *pDisplayPortProfile)
{
    pDisplayPortProfile->colAdjust = 0;
    pDisplayPortProfile->rowAdjust = 0;
}

void resetStatusLedConfig(statusLedConfig_t *statusLedConfig)
{
    for (int i = 0; i < LED_NUMBER; i++) {
        statusLedConfig->ledTags[i] = IO_TAG_NONE;
    }

#ifdef LED0
    statusLedConfig->ledTags[0] = IO_TAG(LED0);
#endif
#ifdef LED1
    statusLedConfig->ledTags[1] = IO_TAG(LED1);
#endif
#ifdef LED2
    statusLedConfig->ledTags[2] = IO_TAG(LED2);
#endif

    statusLedConfig->polarity = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;    
}

#ifdef USE_FLASHFS
void resetFlashConfig(flashConfig_t *flashConfig)
{
#ifdef M25P16_CS_PIN
    flashConfig->csTag = IO_TAG(M25P16_CS_PIN);
#else
    flashConfig->csTag = IO_TAG_NONE;
#endif
}
#endif

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

void setProfile(uint8_t profileIndex)
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
    return motorConfig()->minthrottle;
}


void createDefaultConfig(master_t *config)
{
    // Clear all configuration
    memset(config, 0, sizeof(master_t));

    uint32_t *featuresPtr = &config->enabledFeatures;

    intFeatureClearAll(featuresPtr);
    intFeatureSet(DEFAULT_RX_FEATURE | FEATURE_FAILSAFE , featuresPtr);
#ifdef DEFAULT_FEATURES
    intFeatureSet(DEFAULT_FEATURES, featuresPtr);
#endif

#ifdef USE_MSP_DISPLAYPORT
    resetDisplayPortProfile(&config->displayPortProfileMsp);
#endif
#ifdef USE_MAX7456
    resetDisplayPortProfile(&config->displayPortProfileMax7456);
#endif

#ifdef USE_MAX7456
    resetMax7456Config(&config->vcdProfile);
#endif

#ifdef OSD
    intFeatureSet(FEATURE_OSD, featuresPtr);
    osdResetConfig(&config->osdProfile);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    intFeatureSet(FEATURE_VBAT, featuresPtr);
#endif

    config->version = EEPROM_CONF_VERSION;

    // global settings
    config->current_profile_index = 0;    // default profile
    config->imuConfig.dcm_kp = 2500;                // 1.0 * 10000
    config->imuConfig.dcm_ki = 0;                   // 0.003 * 10000
    config->gyroConfig.gyro_lpf = GYRO_LPF_256HZ;    // 256HZ default
#ifdef STM32F10X
    config->gyroConfig.gyro_sync_denom = 8;
    config->pidConfig.pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
    config->gyroConfig.gyro_sync_denom = 1;
    config->pidConfig.pid_process_denom = 4;
#else
    config->gyroConfig.gyro_sync_denom = 4;
    config->pidConfig.pid_process_denom = 2;
#endif
    config->gyroConfig.gyro_soft_lpf_type = FILTER_PT1;
    config->gyroConfig.gyro_soft_lpf_hz = 90;
    config->gyroConfig.gyro_soft_notch_hz_1 = 400;
    config->gyroConfig.gyro_soft_notch_cutoff_1 = 300;
    config->gyroConfig.gyro_soft_notch_hz_2 = 200;
    config->gyroConfig.gyro_soft_notch_cutoff_2 = 100;

    config->debug_mode = DEBUG_MODE;
    config->task_statistics = true;

    resetAccelerometerTrims(&config->accelerometerConfig.accZero);

    config->gyroConfig.gyro_align = ALIGN_DEFAULT;
    config->accelerometerConfig.acc_align = ALIGN_DEFAULT;
    config->compassConfig.mag_align = ALIGN_DEFAULT;

    config->boardAlignment.rollDegrees = 0;
    config->boardAlignment.pitchDegrees = 0;
    config->boardAlignment.yawDegrees = 0;
    config->accelerometerConfig.acc_hardware = ACC_DEFAULT;     // default/autodetect
    config->rcControlsConfig.yaw_control_direction = 1;
    config->gyroConfig.gyroMovementCalibrationThreshold = 48;

    // xxx_hardware: 0:default/autodetect, 1: disable
    config->compassConfig.mag_hardware = 1;

    config->barometerConfig.baro_hardware = 1;

    resetBatteryConfig(&config->batteryConfig);

#if defined(USE_PWM) || defined(USE_PPM)
    resetPpmConfig(&config->ppmConfig);
    resetPwmConfig(&config->pwmConfig);
#endif

#ifdef TELEMETRY
    resetTelemetryConfig(&config->telemetryConfig);
#endif

#ifdef USE_ADC
    resetAdcConfig(&config->adcConfig);
#endif

#ifdef BEEPER
    resetBeeperConfig(&config->beeperConfig);
#endif

#ifdef SONAR
    resetSonarConfig(&config->sonarConfig);
#endif

#ifdef USE_SDCARD
    intFeatureSet(FEATURE_SDCARD, featuresPtr);
    resetsdcardConfig(&config->sdcardConfig);
#endif

#ifdef SERIALRX_PROVIDER
    config->rxConfig.serialrx_provider = SERIALRX_PROVIDER;
#else
    config->rxConfig.serialrx_provider = 0;
#endif
    config->rxConfig.rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL;
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
    config->rxConfig.rcInterpolationChannels = 0;
    config->rxConfig.rcInterpolationInterval = 19;
    config->rxConfig.fpvCamAngleDegrees = 0;
    config->rxConfig.max_aux_channel = MAX_AUX_CHANNELS;
    config->rxConfig.airModeActivateThreshold = 1350;

    resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);

#ifdef USE_PWM
    config->pwmConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
#endif

    config->armingConfig.gyro_cal_on_first_arm = 0;  // TODO - Cleanup retarded arm support
    config->armingConfig.disarm_kill_switch = 1;
    config->armingConfig.auto_disarm_delay = 5;
    config->imuConfig.small_angle = 25;

    config->airplaneConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetMixerConfig(&config->mixerConfig);
    resetMotorConfig(&config->motorConfig);
#ifdef USE_SERVOS
    resetServoMixerConfig(&config->servoMixerConfig);
    resetServoConfig(&config->servoConfig);
#endif
    resetFlight3DConfig(&config->flight3DConfig);

#ifdef LED_STRIP
    resetLedStripConfig(&config->ledStripConfig);
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

    resetRollAndPitchTrims(&config->accelerometerConfig.accelerometerTrims);

    config->compassConfig.mag_declination = 0;
    config->accelerometerConfig.acc_lpf_hz = 10.0f;

    config->imuConfig.accDeadband.xy = 40;
    config->imuConfig.accDeadband.z = 40;
    config->imuConfig.acc_unarmedcal = 1;

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

    config->throttleCorrectionConfig.throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    config->throttleCorrectionConfig.throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

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
        config->servoProfile.servoConf[i].min = DEFAULT_SERVO_MIN;
        config->servoProfile.servoConf[i].max = DEFAULT_SERVO_MAX;
        config->servoProfile.servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        config->servoProfile.servoConf[i].rate = 100;
        config->servoProfile.servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        config->servoProfile.servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        config->servoProfile.servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
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
    intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
    config->blackboxConfig.device = BLACKBOX_DEVICE_FLASH;
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
    intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
    config->blackboxConfig.device = BLACKBOX_DEVICE_SDCARD;
#else
    config->blackboxConfig.device = BLACKBOX_DEVICE_SERIAL;
#endif

    config->blackboxConfig.rate_num = 1;
    config->blackboxConfig.rate_denom = 1;
    config->blackboxConfig.on_motor_test = 0; // default off
#endif // BLACKBOX

#ifdef SERIALRX_UART
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        int serialIndex = findSerialPortIndexByIdentifier(SERIALRX_UART);
        if (serialIndex >= 0) {
            config->serialConfig.portConfigs[serialIndex].functionMask = FUNCTION_RX_SERIAL;
        }
    }
#endif

#ifdef USE_FLASHFS
    resetFlashConfig(&config->flashConfig);
#endif

    resetStatusLedConfig(&config->statusLedConfig);

    /* merely to force a reset if the person inadvertently flashes the wrong target */
    strncpy(config->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER));
    
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

void activateControlRateConfig(void)
{
    generateThrottleCurve(currentControlRateProfile, &masterConfig.motorConfig);
}

void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        modeActivationProfile()->modeActivationConditions,
        &masterConfig.motorConfig,
        &currentProfile->pidProfile
    );

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif

#ifdef GPS
    gpsUseProfile(&masterConfig.gpsProfile);
    gpsUsePIDs(&currentProfile->pidProfile);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);
    setAccelerationTrims(&accelerometerConfig()->accZero);
    setAccelerationFilter(accelerometerConfig()->acc_lpf_hz);

    mixerUseConfigs(
        &masterConfig.flight3DConfig,
        &masterConfig.motorConfig,
        &masterConfig.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig
    );

#ifdef USE_SERVOS
    servoUseConfigs(&masterConfig.servoMixerConfig, masterConfig.servoProfile.servoConf, &masterConfig.gimbalConfig);
#endif


    imuConfigure(
        &masterConfig.imuConfig,
        &currentProfile->pidProfile,
        throttleCorrectionConfig()->throttle_correction_angle
    );

    configureAltitudeHold(
        &currentProfile->pidProfile,
        &masterConfig.barometerConfig,
        &masterConfig.rcControlsConfig,
        &masterConfig.motorConfig
    );

#ifdef BARO
    useBarometerConfig(&masterConfig.barometerConfig);
#endif
}

void validateAndFixConfig(void)
{
    if((motorConfig()->motorPwmProtocol == PWM_TYPE_BRUSHED) && (motorConfig()->mincommand < 1000)){
        motorConfig()->mincommand = 1000;
    }

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

#if defined(CC3D) && defined(DISPLAY) && defined(USE_UART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DASHBOARD)) {
        featureClear(FEATURE_DASHBOARD);
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

    validateAndFixGyroConfig();

#if defined(TARGET_VALIDATECONFIG)
    targetValidateConfiguration(&masterConfig);
#endif
}

void validateAndFixGyroConfig(void)
{
    // Prevent invalid notch cutoff
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfig()->gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfig()->gyro_soft_notch_hz_2 = 0;
    }

    float samplingTime = 0.000125f;

    if (gyroConfig()->gyro_lpf != GYRO_LPF_256HZ && gyroConfig()->gyro_lpf != GYRO_LPF_NONE) {
        pidConfig()->pid_process_denom = 1; // When gyro set to 1khz always set pid speed 1:1 to sampling speed
        gyroConfig()->gyro_sync_denom = 1;
        gyroConfig()->gyro_use_32khz = false;
        samplingTime = 0.001f;
    }

    if (gyroConfig()->gyro_use_32khz) {
        samplingTime = 0.00003125;
        // F1 and F3 can't handle high sample speed.
#if defined(STM32F1)
        gyroConfig()->gyro_sync_denom = constrain(gyroConfig()->gyro_sync_denom, 16, 16);
#elif defined(STM32F3)
        gyroConfig()->gyro_sync_denom = constrain(gyroConfig()->gyro_sync_denom, 4, 16);
#endif
    }

#if !defined(GYRO_USES_SPI) || !defined(USE_MPU_DATA_READY_SIGNAL)
    gyroConfig()->gyro_isr_update = false;
#endif

    // check for looptime restrictions based on motor protocol. Motor times have safety margin
    const float pidLooptime = samplingTime * gyroConfig()->gyro_sync_denom * pidConfig()->pid_process_denom;
    float motorUpdateRestriction;
    switch(motorConfig()->motorPwmProtocol) {
        case (PWM_TYPE_STANDARD):
            motorUpdateRestriction = 0.002f;
            break;
        case (PWM_TYPE_ONESHOT125):
            motorUpdateRestriction = 0.0005f;
            break;
        case (PWM_TYPE_ONESHOT42):
            motorUpdateRestriction = 0.0001f;
            break;
#ifdef USE_DSHOT
        case (PWM_TYPE_DSHOT150):
            motorUpdateRestriction = 0.000250f;
            break;
        case (PWM_TYPE_DSHOT300):
            motorUpdateRestriction = 0.0001f;
            break;
#endif
        default:
            motorUpdateRestriction = 0.00003125f;
    }

    if(pidLooptime < motorUpdateRestriction)
        pidConfig()->pid_process_denom = motorUpdateRestriction / (samplingTime * gyroConfig()->gyro_sync_denom);

    // Prevent overriding the max rate of motors
    if(motorConfig()->useUnsyncedPwm && (motorConfig()->motorPwmProtocol <= PWM_TYPE_BRUSHED)) {
        uint32_t maxEscRate = lrintf(1.0f / motorUpdateRestriction);

        if(motorConfig()->motorPwmRate > maxEscRate)
            motorConfig()->motorPwmRate = maxEscRate;
    }
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
    if (profileIndex >= MAX_PROFILE_COUNT) {
        profileIndex = MAX_PROFILE_COUNT - 1;
    }
    masterConfig.current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_RATEPROFILES) {
        profileIndex = MAX_RATEPROFILES - 1;
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
