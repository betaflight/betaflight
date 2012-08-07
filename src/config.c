#include "board.h"
#include "mw.h"
#include <string.h>

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage

config_t cfg;
const char rcChannelLetters[] = "AERT1234";

static uint32_t enabledSensors = 0;
uint8_t checkNewConf = 25;

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s)
            cfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

void readEEPROM(void)
{
    uint8_t i;

    // Read flash
    memcpy(&cfg, (char *) FLASH_WRITE_ADDR, sizeof(config_t));

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
        lookupThrottleRC[i] = cfg.minthrottle + (int32_t) (cfg.maxthrottle - cfg.minthrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }

    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max);       //REAR
}

void writeParams(uint8_t b)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < sizeof(config_t); i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
            if (status != FLASH_COMPLETE)
                break;          // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
    if (b)
        blinkLED(15, 20, 1);
}

void checkFirstTime(bool reset)
{
    uint8_t test_val, i;

    test_val = *(uint8_t *) FLASH_WRITE_ADDR;

    if (!reset && test_val == checkNewConf)
        return;

    // Default settings
    cfg.version = checkNewConf;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
    featureSet(FEATURE_VBAT);

    cfg.looptime = 0;
    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 45;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 16;
    cfg.I8[PIDALT] = 15;
    cfg.D8[PIDALT] = 7;
    cfg.P8[PIDPOS] = 11; // POSHOLD_P * 100;
    cfg.I8[PIDPOS] = 0; // POSHOLD_I * 100;
    cfg.D8[PIDPOS] = 0;
    cfg.P8[PIDPOSR] = 20; // POSHOLD_RATE_P * 10;
    cfg.I8[PIDPOSR] = 8; // POSHOLD_RATE_I * 100;
    cfg.D8[PIDPOSR] = 45; // POSHOLD_RATE_D * 1000;
    cfg.P8[PIDNAVR] = 14; // NAV_P * 10;
    cfg.I8[PIDNAVR] = 20; // NAV_I * 100;
    cfg.D8[PIDNAVR] = 80; // NAV_D * 1000;
    cfg.P8[PIDLEVEL] = 70;
    cfg.I8[PIDLEVEL] = 10;
    cfg.D8[PIDLEVEL] = 20;
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
    for (i = 0; i < CHECKBOXITEMS; i++)
        cfg.activate[i] = 0;
    cfg.angleTrim[0] = 0;
    cfg.angleTrim[1] = 0;
    cfg.accZero[0] = 0;
    cfg.accZero[1] = 0;
    cfg.accZero[2] = 0;
    cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    cfg.acc_hardware = ACC_DEFAULT;     // default/autodetect
    cfg.acc_lpf_factor = 4;
    cfg.gyro_cmpf_factor = 400; // default MWC
    cfg.gyro_lpf = 42;
    cfg.mpu6050_scale = 1; // fuck invensense
    cfg.gyro_smoothing_factor = 0x00141403;     // default factors of 20, 20, 3 for R/P/Y
    cfg.vbatscale = 110;
    cfg.vbatmaxcellvoltage = 43;
    cfg.vbatmincellvoltage = 33;

    // Radio
    parseRcChannels("AETR1234");
    cfg.deadband = 0;
    cfg.yawdeadband = 0;
    cfg.alt_hold_throttle_neutral = 20;
    cfg.spektrum_hires = 0;
    cfg.midrc = 1500;
    cfg.mincheck = 1100;
    cfg.maxcheck = 1900;
    cfg.retarded_arm = 0;       // disable arm/disarm on roll left/right

    // Failsafe Variables
    cfg.failsafe_delay = 10;    // 1sec
    cfg.failsafe_off_delay = 200;       // 20sec
    cfg.failsafe_throttle = 1200;       // decent default which should always be below hover throttle for people.

    // Motor/ESC/Servo
    cfg.minthrottle = 1150;
    cfg.maxthrottle = 1850;
    cfg.mincommand = 1000;
    cfg.motor_pwm_rate = 400;
    cfg.servo_pwm_rate = 50;

    // servos
    cfg.yaw_direction = 1;
    cfg.tri_yaw_middle = 1500;
    cfg.tri_yaw_min = 1020;
    cfg.tri_yaw_max = 2000;

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
    cfg.gps_baudrate = 9600;
    cfg.gps_wp_radius = 200;
    cfg.gps_lpf = 20;
    cfg.nav_slew_rate = 30;
    cfg.nav_controls_heading = 1;
    cfg.nav_speed_min = 100;
    cfg.nav_speed_max = 300;

    // serial(uart1) baudrate
    cfg.serial_baudrate = 115200;

    writeParams(0);
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
    return cfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return cfg.enabledFeatures;
}
