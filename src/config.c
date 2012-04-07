#include "board.h"
#include "mw.h"
#include <string.h>

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 64
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))    // use the last KB for storage

config_t cfg;
const char rcChannelLetters[] = "AERT1234";

static uint32_t enabledSensors = 0;
static uint8_t checkNewConf = 12;

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
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));

    for (i = 0; i < 7; i++)
        lookupRX[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 1250;

    cfg.wing_left_mid = constrain(cfg.wing_left_mid, WING_LEFT_MIN, WING_LEFT_MAX);     //LEFT 
    cfg.wing_right_mid = constrain(cfg.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
}

void writeParams(void)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    
    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < sizeof(config_t); i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)&cfg + i));
            if (status != FLASH_COMPLETE)
                break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
    blinkLED(15, 20, 1);
}

void checkFirstTime(bool reset)
{
    uint8_t test_val, i;

    test_val = *(uint8_t *)FLASH_WRITE_ADDR;

    if (!reset && test_val == checkNewConf)
        return;

    // Default settings
    cfg.version = checkNewConf;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
    featureSet(FEATURE_VBAT); // | FEATURE_PPM); // sadly, this is for hackers only

    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 0;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 16;
    cfg.I8[PIDALT] = 15;
    cfg.D8[PIDALT] = 7;
    cfg.P8[PIDGPS] = 50;
    cfg.I8[PIDGPS] = 0;
    cfg.D8[PIDGPS] = 15;
    cfg.P8[PIDVEL] = 0;
    cfg.I8[PIDVEL] = 0;
    cfg.D8[PIDVEL] = 0;
    cfg.P8[PIDLEVEL] = 90;
    cfg.I8[PIDLEVEL] = 45;
    cfg.D8[PIDLEVEL] = 100;
    cfg.P8[PIDMAG] = 40;
    cfg.rcRate8 = 45;               // = 0.9 in GUI
    cfg.rcExpo8 = 65;
    cfg.rollPitchRate = 0;
    cfg.yawRate = 0;
    cfg.dynThrPID = 0;
    for (i = 0; i < CHECKBOXITEMS; i++) {
        cfg.activate1[i] = 0;
        cfg.activate2[i] = 0;
    }
    cfg.accTrim[0] = 0;
    cfg.accTrim[1] = 0;
    cfg.accZero[0] = 0;
    cfg.accZero[1] = 0;
    cfg.accZero[2] = 0;
    cfg.acc_lpf_factor = 4;
    cfg.gyro_lpf = 42;
    cfg.gyro_smoothing_factor = 0x00141403; // default factors of 20, 20, 3 for R/P/Y
    cfg.vbatscale = 110;
    cfg.vbatmaxcellvoltage = 43;
    cfg.vbatmincellvoltage = 33;

    // Radio
    parseRcChannels("AETR1234");
    cfg.deadband = 0;
    cfg.yawdeadband = 0;
    cfg.spektrum_hires = 0;
    cfg.midrc = 1500;
    cfg.mincheck = 1100;
    cfg.maxcheck = 1900;

    // Motor/ESC/Servo
    cfg.minthrottle = 1150;
    cfg.maxthrottle = 1850;
    cfg.mincommand = 1000;
    cfg.motor_pwm_rate = 400;
    cfg.servo_pwm_rate = 50;

    // servos
    cfg.yaw_direction = 1;
    cfg.wing_left_mid = 1500;
    cfg.wing_right_mid = 1500;
    cfg.tri_yaw_middle = 1500;
    cfg.tri_yaw_min = 1020;
    cfg.tri_yaw_max = 2000;

    // gimbal
    cfg.gimbal_pitch_gain = 10;
    cfg.gimbal_roll_gain = 10;
    cfg.gimbal_pitch_min = 1020;
    cfg.gimbal_pitch_max = 2000;
    cfg.gimbal_pitch_mid = 1500;
    cfg.gimbal_roll_min = 1020;
    cfg.gimbal_roll_max = 2000;
    cfg.gimbal_roll_mid = 1500;

    // gps baud-rate
    cfg.gps_baudrate = 9600;    

    // serial(uart1) baudrate
    cfg.serial_baudrate = 115200;

    writeParams();
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
