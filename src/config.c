#include "board.h"
#include "mw.h"
#include <string.h>

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * 63)    // use the last KB for storage

config_t cfg;

static uint32_t enabledSensors = 0;
static uint8_t checkNewConf = 4;

void readEEPROM(void)
{
    uint8_t i;

    // Read flash
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));

#if defined(POWERMETER)
    pAlarm = (uint32_t) cfg.powerTrigger1 *(uint32_t) PLEVELSCALE *(uint32_t) PLEVELDIV;    // need to cast before multiplying
#endif

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
    cfg.gyro_smoothing_factor = 0x00141403; // default factors of 20, 20, 3 for R/P/Y
    cfg.powerTrigger1 = 0;

    // Radio/ESC
    cfg.deadband = 0;
    cfg.midrc = 1500;
    cfg.minthrottle = 1150;
    cfg.maxthrottle = 1850;
    cfg.mincommand = 1000;

    // servos
    cfg.yaw_direction = 1;
    cfg.wing_left_mid = 1500;
    cfg.wing_right_mid = 1500;
    cfg.tri_yaw_middle = 1500;
    cfg.tri_yaw_min = 1020;
    cfg.tri_yaw_max = 2000;

    // gimbal
    cfg.tilt_pitch_prop = 10;
    cfg.tilt_roll_prop = 10;

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
