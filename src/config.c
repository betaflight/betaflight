#include "board.h"
#include "mw.h"
#include <string.h>

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage

master_t mcfg;  // master config struct with data independent from profiles
config_t cfg;   // profile config struct
const char rcChannelLetters[] = "AERT1234";

static uint8_t EEPROM_CONF_VERSION = 47;
static uint32_t enabledSensors = 0;
static void resetConf(void);

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s)
            mcfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

static uint8_t validEEPROM(void)
{
    const master_t *temp = (const master_t *)FLASH_WRITE_ADDR;
    const uint8_t *p;
    uint8_t chk = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return 0;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return 0;

    // verify integrity of temporary copy
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(master_t)); p++)
        chk ^= *p;

    // checksum failed
    if (chk != 0)
        return 0;

    // looks good, let's roll!
    return 1;
}

void readEEPROM(void)
{
    uint8_t i;

    // Sanity check
    if (!validEEPROM())
        failureMode(10);

    // Read flash
    memcpy(&mcfg, (char *)FLASH_WRITE_ADDR, sizeof(master_t));
    // Copy current profile
    if (mcfg.current_profile > 2) // sanity check
        mcfg.current_profile = 0;
    memcpy(&cfg, &mcfg.profile[mcfg.current_profile], sizeof(config_t));

    for (i = 0; i < 6; i++)
        lookupPitchRollRC[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 2500;

    for (i = 0; i < 11; i++) {
        int16_t tmp = 10 * i - cfg.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - cfg.thrMid8;
        if (tmp < 0)
            y = cfg.thrMid8;
        lookupThrottleRC[i] = 10 * cfg.thrMid8 + tmp * (100 - cfg.thrExpo8 + (int32_t) cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        lookupThrottleRC[i] = mcfg.minthrottle + (int32_t) (mcfg.maxthrottle - mcfg.minthrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }

    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max);       //REAR
}

void writeEEPROM(uint8_t b, uint8_t updateProfile)
{
    FLASH_Status status;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;
    int tries = 0;

    // prepare checksum/version constants
    mcfg.version = EEPROM_CONF_VERSION;
    mcfg.size = sizeof(master_t);
    mcfg.magic_be = 0xBE;
    mcfg.magic_ef = 0xEF;
    mcfg.chk = 0;

    // when updateProfile = true, we copy contents of cfg to global configuration. when false, only profile number is updated, and then that profile is loaded on readEEPROM()
    if (updateProfile) {
        // copy current in-memory profile to stored configuration
        memcpy(&mcfg.profile[mcfg.current_profile], &cfg, sizeof(config_t));
    }

    // recalculate checksum before writing
    for (p = (const uint8_t *)&mcfg; p < ((const uint8_t *)&mcfg + sizeof(master_t)); p++)
        chk ^= *p;
    mcfg.chk = chk;

    // write it
retry:
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < sizeof(master_t); i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *)&mcfg + i));
            if (status != FLASH_COMPLETE) {
                FLASH_Lock();
                tries++;
                if (tries < 3)
                    goto retry;
                else
                    break;
            }
        }
    }
    FLASH_Lock();
    
    // Flash write failed - just die now
    if (tries == 3 || !validEEPROM()) {
        failureMode(10);
    }

    // re-read written data
    readEEPROM();
    if (b)
        blinkLED(15, 20, 1);
}

void checkFirstTime(bool reset)
{
    // check the EEPROM integrity before resetting values
    if (!validEEPROM() || reset) {
        resetConf();
        // no need to memcpy profile again, we just did it in resetConf() above
        writeEEPROM(0, false);
    }
}

// Default settings
static void resetConf(void)
{
    int i;
    const int8_t default_align[3][3] = { /* GYRO */ { 0, 0, 0 }, /* ACC */ { 0, 0, 0 }, /* MAG */ { -2, -3, 1 } };

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
    memcpy(&mcfg.align, default_align, sizeof(mcfg.align));
    mcfg.acc_hardware = ACC_DEFAULT;     // default/autodetect
    mcfg.moron_threshold = 32;
    mcfg.gyro_smoothing_factor = 0x00141403;     // default factors of 20, 20, 3 for R/P/Y
    mcfg.vbatscale = 110;
    mcfg.vbatmaxcellvoltage = 43;
    mcfg.vbatmincellvoltage = 33;
    mcfg.power_adc_channel = 0;
    mcfg.spektrum_hires = 0;
    mcfg.midrc = 1500;
    mcfg.mincheck = 1100;
    mcfg.maxcheck = 1900;
    mcfg.retarded_arm = 0;       // disable arm/disarm on roll left/right
    // Motor/ESC/Servo
    mcfg.minthrottle = 1150;
    mcfg.maxthrottle = 1850;
    mcfg.mincommand = 1000;
    mcfg.motor_pwm_rate = 400;
    mcfg.servo_pwm_rate = 50;
    // gps/nav stuff
    mcfg.gps_type = GPS_NMEA;
    mcfg.gps_baudrate = 115200;
    // serial (USART1) baudrate
    mcfg.serial_baudrate = 115200;
    mcfg.looptime = 3500;

    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 45;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 64;
    cfg.I8[PIDALT] = 25;
    cfg.D8[PIDALT] = 24;
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
    cfg.P8[PIDVEL] = 0;
    cfg.I8[PIDVEL] = 0;
    cfg.D8[PIDVEL] = 0;
    cfg.rcRate8 = 90;
    cfg.rcExpo8 = 65;
    cfg.rollPitchRate = 0;
    cfg.yawRate = 0;
    cfg.dynThrPID = 0;
    cfg.thrMid8 = 50;
    cfg.thrExpo8 = 0;
    // for (i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;
    cfg.angleTrim[0] = 0;
    cfg.angleTrim[1] = 0;
    cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    cfg.acc_lpf_factor = 4;
    cfg.accz_deadband = 50;
    cfg.baro_tab_size = 21;
    cfg.baro_noise_lpf = 0.6f;
    cfg.baro_cf = 0.985f;

    // Radio
    parseRcChannels("AETR1234");
    cfg.deadband = 0;
    cfg.yawdeadband = 0;
    cfg.alt_hold_throttle_neutral = 40;
    cfg.alt_hold_fast_change = 1;

    // Failsafe Variables
    cfg.failsafe_delay = 10;                // 1sec
    cfg.failsafe_off_delay = 200;           // 20sec
    cfg.failsafe_throttle = 1200;           // decent default which should always be below hover throttle for people.
    cfg.failsafe_detect_threshold = 985;    // any of first 4 channels below this value will trigger failsafe

    // servos
    cfg.yaw_direction = 1;
    cfg.tri_yaw_middle = 1500;
    cfg.tri_yaw_min = 1020;
    cfg.tri_yaw_max = 2000;

    // flying wing
    cfg.wing_left_min = 1020;
    cfg.wing_left_mid = 1500;
    cfg.wing_left_max = 2000;
    cfg.wing_right_min = 1020;
    cfg.wing_right_mid = 1500;
    cfg.wing_right_max = 2000;
    cfg.pitch_direction_l = 1;
    cfg.pitch_direction_r = -1;
    cfg.roll_direction_l = 1;
    cfg.roll_direction_r = 1;

    // gimbal
    cfg.gimbal_pitch_gain = 10;
    cfg.gimbal_roll_gain = 10;
    cfg.gimbal_flags = GIMBAL_NORMAL;
    cfg.gimbal_pitch_min = 1020;
    cfg.gimbal_pitch_max = 2000;
    cfg.gimbal_pitch_mid = 1500;
    cfg.gimbal_roll_min = 1020;
    cfg.gimbal_roll_max = 2000;
    cfg.gimbal_roll_mid = 1500;

    // gps/nav stuff
    cfg.gps_wp_radius = 200;
    cfg.gps_lpf = 20;
    cfg.nav_slew_rate = 30;
    cfg.nav_controls_heading = 1;
    cfg.nav_speed_min = 100;
    cfg.nav_speed_max = 300;
    cfg.ap_mode = 40;

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_MOTORS; i++)
        mcfg.customMixer[i].throttle = 0.0f;

    // copy default config into all 3 profiles
    for (i = 0; i < 3; i++)
        memcpy(&mcfg.profile[i], &cfg, sizeof(config_t));
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
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
