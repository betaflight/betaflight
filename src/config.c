#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "flight_common.h"

#include "drivers/system_common.h"

#include "statusindicator.h"
#include "sensors_common.h"
#include "sensors_acceleration.h"
#include "telemetry_common.h"
#include "gps_common.h"

#include "drivers/serial_common.h"
#include "flight_mixer.h"
#include "sensors_common.h"
#include "boardalignment.h"
#include "battery.h"
#include "gimbal.h"
#include "rx_common.h"
#include "gps_common.h"
#include "serial_common.h"
#include "failsafe.h"

#include "runtime_config.h"
#include "config.h"
#include "config_storage.h"

void setPIDController(int type); // FIXME PID code needs to be in flight_pid.c/h

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 2))       // use the last 2 KB for storage

master_t mcfg;  // master config struct with data independent from profiles
config_t cfg;   // profile config struct

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

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(10);

    // Read flash
    memcpy(&mcfg, (char *)FLASH_WRITE_ADDR, sizeof(master_t));
    // Copy current profile
    if (mcfg.current_profile > 2) // sanity check
        mcfg.current_profile = 0;
    memcpy(&cfg, &mcfg.profile[mcfg.current_profile], sizeof(config_t));

    generatePitchCurve(&cfg.controlRateConfig);
    generateThrottleCurve(&cfg.controlRateConfig, mcfg.minthrottle, mcfg.maxthrottle);

    setPIDController(cfg.pidController);
    gpsSetPIDs();
    useFailsafeConfig(&cfg.failsafeConfig);
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
    memcpy(&mcfg.profile[profileSlotIndex], &cfg, sizeof(config_t));
}

void writeEEPROM(void)
{
    FLASH_Status status;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    // prepare checksum/version constants
    mcfg.version = EEPROM_CONF_VERSION;
    mcfg.size = sizeof(master_t);
    mcfg.magic_be = 0xBE;
    mcfg.magic_ef = 0xEF;
    mcfg.chk = 0; // erase checksum before recalculating
    mcfg.chk = calculateChecksum((const uint8_t *)&mcfg, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

        status = FLASH_ErasePage(FLASH_WRITE_ADDR);
        for (wordOffset = 0; wordOffset < sizeof(master_t) && status == FLASH_COMPLETE; wordOffset += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + wordOffset, *(uint32_t *) ((char *)&mcfg + wordOffset));
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

// Default settings
static void resetConf(void)
{
    int i;
    int8_t servoRates[8] = { 30, 30, 100, 100, 100, 100, 100, 100 };

    // Clear all configuration
    memset(&mcfg, 0, sizeof(master_t));
    memset(&cfg, 0, sizeof(config_t));

    mcfg.version = EEPROM_CONF_VERSION;
    mcfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
    featureSet(FEATURE_VBAT);

    // global settings
    mcfg.current_profile = 0;       // default profile
    mcfg.gyro_cmpf_factor = 600;    // default MWC
    mcfg.gyro_cmpfm_factor = 250;   // default MWC
    mcfg.gyro_lpf = 42;             // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead
    mcfg.accZero[0] = 0;
    mcfg.accZero[1] = 0;
    mcfg.accZero[2] = 0;
    mcfg.gyro_align = ALIGN_DEFAULT;
    mcfg.acc_align = ALIGN_DEFAULT;
    mcfg.mag_align = ALIGN_DEFAULT;
    mcfg.boardAlignment.rollDegrees = 0;
    mcfg.boardAlignment.pitchDegrees = 0;
    mcfg.boardAlignment.yawDegrees = 0;
    mcfg.acc_hardware = ACC_DEFAULT;     // default/autodetect
    mcfg.max_angle_inclination = 500;    // 50 degrees
    mcfg.yaw_control_direction = 1;
    mcfg.moron_threshold = 32;
    mcfg.batteryConfig.vbatscale = 110;
    mcfg.batteryConfig.vbatmaxcellvoltage = 43;
    mcfg.batteryConfig.vbatmincellvoltage = 33;
    mcfg.power_adc_channel = 0;
    mcfg.telemetry_provider = TELEMETRY_PROVIDER_FRSKY;
    mcfg.telemetry_port = TELEMETRY_PORT_UART;
    mcfg.telemetry_switch = 0;
    mcfg.rxConfig.serialrx_type = 0;
    mcfg.rxConfig.midrc = 1500;
    mcfg.rxConfig.mincheck = 1100;
    mcfg.rxConfig.maxcheck = 1900;
    mcfg.retarded_arm = 0;       // disable arm/disarm on roll left/right
    mcfg.flaps_speed = 0;
    mcfg.fixedwing_althold_dir = 1;
    // Motor/ESC/Servo
    mcfg.minthrottle = 1150;
    mcfg.maxthrottle = 1850;
    mcfg.mincommand = 1000;
    mcfg.deadband3d_low = 1406;
    mcfg.deadband3d_high = 1514;
    mcfg.neutral3d = 1460;
    mcfg.deadband3d_throttle = 50;
    mcfg.motor_pwm_rate = 400;
    mcfg.servo_pwm_rate = 50;
    // gps/nav stuff
    mcfg.gps_type = GPS_NMEA;
    mcfg.gps_baudrate = GPS_BAUD_115200;

    // serial (USART1) baudrate
    mcfg.serialConfig.port1_baudrate = 115200;
    mcfg.serialConfig.softserial_baudrate = 9600;
    mcfg.serialConfig.softserial_1_inverted = 0;
    mcfg.serialConfig.softserial_2_inverted = 0;
    mcfg.serialConfig.reboot_character = 'R';

    mcfg.looptime = 3500;
    mcfg.rssi_aux_channel = 0;

    cfg.pidController = 0;
    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 45;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 50;
    cfg.I8[PIDALT] = 0;
    cfg.D8[PIDALT] = 0;
    cfg.P8[PIDPOS] = 11; // POSHOLD_P * 100;
    cfg.I8[PIDPOS] = 0; // POSHOLD_I * 100;
    cfg.D8[PIDPOS] = 0;
    cfg.P8[PIDPOSR] = 20; // POSHOLD_RATE_P * 10;
    cfg.I8[PIDPOSR] = 8; // POSHOLD_RATE_I * 100;
    cfg.D8[PIDPOSR] = 45; // POSHOLD_RATE_D * 1000;
    cfg.P8[PIDNAVR] = 14; // NAV_P * 10;
    cfg.I8[PIDNAVR] = 20; // NAV_I * 100;
    cfg.D8[PIDNAVR] = 80; // NAV_D * 1000;
    cfg.P8[PIDLEVEL] = 90;
    cfg.I8[PIDLEVEL] = 10;
    cfg.D8[PIDLEVEL] = 100;
    cfg.P8[PIDMAG] = 40;
    cfg.P8[PIDVEL] = 120;
    cfg.I8[PIDVEL] = 45;
    cfg.D8[PIDVEL] = 1;
    cfg.controlRateConfig.rcRate8 = 90;
    cfg.controlRateConfig.rcExpo8 = 65;
    cfg.controlRateConfig.rollPitchRate = 0;
    cfg.controlRateConfig.yawRate = 0;
    cfg.dynThrPID = 0;
    cfg.tpaBreakPoint = 1500;
    cfg.controlRateConfig.thrMid8 = 50;
    cfg.controlRateConfig.thrExpo8 = 0;
    // for (i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;
    cfg.angleTrim[0] = 0;
    cfg.angleTrim[1] = 0;
    cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    cfg.acc_lpf_factor = 4;
    cfg.accz_deadband = 40;
    cfg.accxy_deadband = 40;
    cfg.baro_tab_size = 21;
    cfg.baro_noise_lpf = 0.6f;
    cfg.baro_cf_vel = 0.985f;
    cfg.baro_cf_alt = 0.965f;
    cfg.acc_unarmedcal = 1;

    // Radio
    parseRcChannels("AETR1234", &mcfg.rxConfig);
    cfg.deadband = 0;
    cfg.yawdeadband = 0;
    cfg.alt_hold_throttle_neutral = 40;
    cfg.alt_hold_fast_change = 1;
    cfg.throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    cfg.throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    cfg.failsafeConfig.failsafe_delay = 10;                // 1sec
    cfg.failsafeConfig.failsafe_off_delay = 200;           // 20sec
    cfg.failsafeConfig.failsafe_throttle = 1200;           // decent default which should always be below hover throttle for people.
    cfg.failsafeConfig.failsafe_detect_threshold = 985;    // any of first 4 channels below this value will trigger failsafe

    // servos
    for (i = 0; i < 8; i++) {
        cfg.servoConf[i].min = DEFAULT_SERVO_MIN;
        cfg.servoConf[i].max = DEFAULT_SERVO_MAX;
        cfg.servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        cfg.servoConf[i].rate = servoRates[i];
        cfg.servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    cfg.yaw_direction = 1;
    cfg.tri_unarmed_servo = 1;

    // gimbal
    cfg.gimbal_flags = GIMBAL_NORMAL;

    // gps/nav stuff
    cfg.gps_wp_radius = 200;
    cfg.gps_lpf = 20;
    cfg.nav_slew_rate = 30;
    cfg.nav_controls_heading = 1;
    cfg.nav_speed_min = 100;
    cfg.nav_speed_max = 300;
    cfg.ap_mode = 40;

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        mcfg.customMixer[i].throttle = 0.0f;

    // copy default config into all 3 profiles
    for (i = 0; i < 3; i++)
        memcpy(&mcfg.profile[i], &cfg, sizeof(config_t));
}


bool feature(uint32_t mask)
{
    return mcfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    mcfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    mcfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    mcfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return mcfg.enabledFeatures;
}

