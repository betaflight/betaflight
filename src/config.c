#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "flight_common.h"

#include "drivers/accgyro_common.h"
#include "drivers/system_common.h"

#include "sensors_common.h"
#include "sensors_gyro.h"

#include "statusindicator.h"
#include "sensors_acceleration.h"
#include "sensors_barometer.h"
#include "telemetry_common.h"
#include "gps_common.h"

#include "drivers/serial_common.h"
#include "flight_mixer.h"
#include "boardalignment.h"
#include "battery.h"
#include "gimbal.h"
#include "escservo.h"
#include "rc_controls.h"
#include "rc_curves.h"
#include "rx_common.h"
#include "gps_common.h"
#include "serial_common.h"
#include "failsafe.h"

#include "runtime_config.h"
#include "config.h"
#include "config_profile.h"
#include "config_master.h"

void setPIDController(int type); // FIXME PID code needs to be in flight_pid.c/h
void mixerUseConfigs(servoParam_t *servoConfToUse, flight3DConfig_t *flight3DConfigToUse, escAndServoConfig_t *escAndServoConfigToUse, mixerConfig_t *mixerConfigToUse, airplaneConfig_t *airplaneConfigToUse, rxConfig_t *rxConfig, gimbalConfig_t *gimbalConfigToUse);


#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 2))       // use the last 2 KB for storage

master_t masterConfig;  // master config struct with data independent from profiles
profile_t currentProfile;   // profile config struct

static const uint8_t EEPROM_CONF_VERSION = 64;
static void resetConf(void);

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
    const master_t *temp = (const master_t *)FLASH_WRITE_ADDR;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *)temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

void activateConfig(void)
{
    generatePitchCurve(&currentProfile.controlRateConfig);
    generateThrottleCurve(&currentProfile.controlRateConfig, &masterConfig.escAndServoConfig);

    useGyroConfig(&masterConfig.gyroConfig);
    useTelemetryConfig(&masterConfig.telemetryConfig);
    setPIDController(currentProfile.pidController);
    gpsUseProfile(&currentProfile.gpsProfile);
    gpsUsePIDs(&currentProfile.pidProfile);
    useFailsafeConfig(&currentProfile.failsafeConfig);
    setAccelerationTrims(&masterConfig.accZero);
    mixerUseConfigs(
        currentProfile.servoConf,
        &masterConfig.flight3DConfig,
        &masterConfig.escAndServoConfig,
        &currentProfile.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig,
        &currentProfile.gimbalConfig
    );

#ifdef BARO
    useBarometerConfig(&currentProfile.barometerConfig);
#endif
}

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(10);

    // Read flash
    memcpy(&masterConfig, (char *)FLASH_WRITE_ADDR, sizeof(master_t));
    // Copy current profile
    if (masterConfig.current_profile_index > 2) // sanity check
        masterConfig.current_profile_index = 0;
    memcpy(&currentProfile, &masterConfig.profile[masterConfig.current_profile_index], sizeof(profile_t)); 

    activateConfig();
}

void readEEPROMAndNotify(void)
{
    // re-read written data
    readEEPROM();
    blinkLedAndSoundBeeper(15, 20, 1);
}

void copyCurrentProfileToProfileSlot(uint8_t profileSlotIndex)
{
    // copy current in-memory profile to stored configuration
    memcpy(&masterConfig.profile[profileSlotIndex], &currentProfile, sizeof(profile_t));
}

void writeEEPROM(void)
{
    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *)&masterConfig, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
#ifdef STM32F3DISCOVERY
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#endif
#ifdef STM32F10X_MD
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        status = FLASH_ErasePage(FLASH_WRITE_ADDR);
        for (wordOffset = 0; wordOffset < sizeof(master_t) && status == FLASH_COMPLETE; wordOffset += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + wordOffset, *(uint32_t *) ((char *)&masterConfig + wordOffset));
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
        failureMode(10);
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

static void resetAccelerometerTrims(int16_flightDynamicsTrims_t *accelerometerTrims)
{
    accelerometerTrims->trims.pitch = 0;
    accelerometerTrims->trims.roll = 0;
    accelerometerTrims->trims.yaw = 0;
}

static void resetPidProfile(pidProfile_t *pidProfile)
{
    pidProfile->P8[ROLL] = 40;
    pidProfile->I8[ROLL] = 30;
    pidProfile->D8[ROLL] = 23;
    pidProfile->P8[PITCH] = 40;
    pidProfile->I8[PITCH] = 30;
    pidProfile->D8[PITCH] = 23;
    pidProfile->P8[YAW] = 85;
    pidProfile->I8[YAW] = 45;
    pidProfile->D8[YAW] = 0;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 11; // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0; // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 20; // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 8; // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 45; // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 14; // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 20; // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 80; // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 90;
    pidProfile->I8[PIDLEVEL] = 10;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 120;
    pidProfile->I8[PIDVEL] = 45;
    pidProfile->D8[PIDVEL] = 1;
}

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

void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 0.6f;
    barometerConfig->baro_cf_vel = 0.985f;
    barometerConfig->baro_cf_alt = 0.965f;
}

void resetSensorAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    sensorAlignmentConfig->gyro_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->acc_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->mag_align = ALIGN_DEFAULT;
}

void resetEscAndServoConfig(escAndServoConfig_t *escAndServoConfig)
{
    escAndServoConfig->minthrottle = 1150;
    escAndServoConfig->maxthrottle = 1850;
    escAndServoConfig->mincommand = 1000;
}

void resetFlight3DConfig(flight3DConfig_t *flight3DConfig)
{
    flight3DConfig->deadband3d_low = 1406;
    flight3DConfig->deadband3d_high = 1514;
    flight3DConfig->neutral3d = 1460;
    flight3DConfig->deadband3d_throttle = 50;
}

void resetTelemetryConfig(telemetryConfig_t *telemetryConfig)
{
    telemetryConfig->telemetry_provider = TELEMETRY_PROVIDER_FRSKY;
    telemetryConfig->telemetry_port = TELEMETRY_PORT_UART;
    telemetryConfig->telemetry_switch = 0;

}
// Default settings
static void resetConf(void)
{
    int i;
    int8_t servoRates[8] = { 30, 30, 100, 100, 100, 100, 100, 100 };

    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));
    memset(&currentProfile, 0, sizeof(profile_t));

    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
    featureSet(FEATURE_VBAT);

    // global settings
    masterConfig.current_profile_index = 0;       // default profile
    masterConfig.gyro_cmpf_factor = 600;    // default MWC
    masterConfig.gyro_cmpfm_factor = 250;   // default MWC
    masterConfig.gyro_lpf = 42;             // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead

    resetAccelerometerTrims(&masterConfig.accZero);

    resetSensorAlignment(&masterConfig.sensorAlignmentConfig);

    masterConfig.boardAlignment.rollDegrees = 0;
    masterConfig.boardAlignment.pitchDegrees = 0;
    masterConfig.boardAlignment.yawDegrees = 0;
    masterConfig.acc_hardware = ACC_DEFAULT;     // default/autodetect
    masterConfig.max_angle_inclination = 500;    // 50 degrees
    masterConfig.yaw_control_direction = 1;
    masterConfig.gyroConfig.gyroMovementCalibrationThreshold = 32;
    masterConfig.batteryConfig.vbatscale = 110;
    masterConfig.batteryConfig.vbatmaxcellvoltage = 43;
    masterConfig.batteryConfig.vbatmincellvoltage = 33;
    masterConfig.power_adc_channel = 0;

    resetTelemetryConfig(&masterConfig.telemetryConfig);

    masterConfig.rxConfig.serialrx_type = 0;
    masterConfig.rxConfig.midrc = 1500;
    masterConfig.rxConfig.mincheck = 1100;
    masterConfig.rxConfig.maxcheck = 1900;
    masterConfig.retarded_arm = 0;       // disable arm/disarm on roll left/right
    masterConfig.airplaneConfig.flaps_speed = 0;
    masterConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetEscAndServoConfig(&masterConfig.escAndServoConfig);
    resetFlight3DConfig(&masterConfig.flight3DConfig);
    masterConfig.motor_pwm_rate = 400;
    masterConfig.servo_pwm_rate = 50;
    // gps/nav stuff
    masterConfig.gps_type = GPS_NMEA;
    masterConfig.gps_baudrate = GPS_BAUD_115200;

    // serial (USART1) baudrate
    masterConfig.serialConfig.port1_baudrate = 115200;
    masterConfig.serialConfig.softserial_baudrate = 9600;
    masterConfig.serialConfig.softserial_1_inverted = 0;
    masterConfig.serialConfig.softserial_2_inverted = 0;
    masterConfig.serialConfig.reboot_character = 'R';

    masterConfig.looptime = 3500;
    masterConfig.emf_avoidance = 0;
    masterConfig.rssi_aux_channel = 0;

    currentProfile.pidController = 0;
    resetPidProfile(&currentProfile.pidProfile);

    currentProfile.controlRateConfig.rcRate8 = 90;
    currentProfile.controlRateConfig.rcExpo8 = 65;
    currentProfile.controlRateConfig.rollPitchRate = 0;
    currentProfile.controlRateConfig.yawRate = 0;
    currentProfile.dynThrPID = 0;
    currentProfile.tpa_breakpoint = 1500;
    currentProfile.controlRateConfig.thrMid8 = 50;
    currentProfile.controlRateConfig.thrExpo8 = 0;

    // for (i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;

    resetRollAndPitchTrims(&currentProfile.accelerometerTrims);

    currentProfile.mag_declination = 0;
    currentProfile.acc_lpf_factor = 4;
    currentProfile.accz_deadband = 40;
    currentProfile.accxy_deadband = 40;

    resetBarometerConfig(&currentProfile.barometerConfig);

    currentProfile.acc_unarmedcal = 1;

    // Radio
    parseRcChannels("AETR1234", &masterConfig.rxConfig);
    currentProfile.deadband = 0;
    currentProfile.yaw_deadband = 0;
    currentProfile.alt_hold_throttle_neutral = 40;
    currentProfile.alt_hold_fast_change = 1;
    currentProfile.throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    currentProfile.throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    currentProfile.failsafeConfig.failsafe_delay = 10;                // 1sec
    currentProfile.failsafeConfig.failsafe_off_delay = 200;           // 20sec
    currentProfile.failsafeConfig.failsafe_throttle = 1200;           // decent default which should always be below hover throttle for people.
    currentProfile.failsafeConfig.failsafe_detect_threshold = 985;    // any of first 4 channels below this value will trigger failsafe

    // servos
    for (i = 0; i < 8; i++) {
        currentProfile.servoConf[i].min = DEFAULT_SERVO_MIN;
        currentProfile.servoConf[i].max = DEFAULT_SERVO_MAX;
        currentProfile.servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        currentProfile.servoConf[i].rate = servoRates[i];
        currentProfile.servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    currentProfile.mixerConfig.yaw_direction = 1;
    currentProfile.mixerConfig.tri_unarmed_servo = 1;

    // gimbal
    currentProfile.gimbalConfig.gimbal_flags = GIMBAL_NORMAL;

    resetGpsProfile(&currentProfile.gpsProfile);

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        masterConfig.customMixer[i].throttle = 0.0f;

    // copy default config into all 3 profiles
    for (i = 0; i < 3; i++)
        memcpy(&masterConfig.profile[i], &currentProfile, sizeof(profile_t));
}

void saveAndReloadCurrentProfileToCurrentProfileSlot(void)
{
    copyCurrentProfileToProfileSlot(masterConfig.current_profile_index);
    writeEEPROM();
    readEEPROMAndNotify();
}

bool feature(uint32_t mask)
{
    return masterConfig.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    masterConfig.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    masterConfig.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    masterConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return masterConfig.enabledFeatures;
}

bool canSoftwareSerialBeUsed(void)
{
    // FIXME this is not ideal because it means you can't disable parallel PWM input even when using spektrum/sbus etc.
    // really we want to say 'return !feature(FEATURE_PARALLEL_PWM);'
    return feature(FEATURE_SOFTSERIAL) && feature(FEATURE_PPM); // Software serial can only be used in PPM mode because parallel PWM uses the same hardware pins/timers
}

