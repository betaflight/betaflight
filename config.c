#include "board.h"
#include "mw.h"
#include <string.h>

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * 63)    // use the last KB for storage

static uint8_t checkNewConf = 151;

typedef struct eep_entry_t {
    void *var;
    uint8_t size;
} eep_entry_t;

// ************************************************************************************************************
// EEPROM Layout definition
// ************************************************************************************************************
volatile eep_entry_t eep_entry[] = {
    {&checkNewConf, sizeof(checkNewConf)}
    , {&P8, sizeof(P8)}
    , {&I8, sizeof(I8)}
    , {&D8, sizeof(D8)}
    , {&rcRate8, sizeof(rcRate8)}
    , {&rcExpo8, sizeof(rcExpo8)}
    , {&rollPitchRate, sizeof(rollPitchRate)}
    , {&yawRate, sizeof(yawRate)}
    , {&dynThrPID, sizeof(dynThrPID)}
    , {&accZero, sizeof(accZero)}
    , {&magZero, sizeof(magZero)}
    , {&accTrim, sizeof(accTrim)}
    , {&activate1, sizeof(activate1)}
    , {&activate2, sizeof(activate2)}
    , {&powerTrigger1, sizeof(powerTrigger1)}
#ifdef FLYING_WING
    , {&wing_left_mid, sizeof(wing_left_mid)}
    , {&wing_right_mid, sizeof(wing_right_mid)}
#endif
#ifdef TRI
    , {&tri_yaw_middle, sizeof(tri_yaw_middle)}
#endif

};

#define EEBLOCK_SIZE sizeof(eep_entry) / sizeof(eep_entry_t)

void readEEPROM(void)
{
    uint8_t i, _address = eep_entry[0].size;

    // Read flash
    for (i = 1; i < EEBLOCK_SIZE; i++) {
        memcpy(eep_entry[i].var, (char *)FLASH_WRITE_ADDR + _address, eep_entry[i].size);
        _address += eep_entry[i].size;
    }

#if defined(POWERMETER)
    pAlarm = (uint32_t) powerTrigger1 *(uint32_t) PLEVELSCALE *(uint32_t) PLEVELDIV;    // need to cast before multiplying
#endif
    for (i = 0; i < 7; i++)
        lookupRX[i] = (2500 + rcExpo8 * (i * i - 25)) * i * (int32_t) rcRate8 / 1250;
#ifdef FLYING_WING
    wing_left_mid = constrain(wing_left_mid, WING_LEFT_MIN, WING_LEFT_MAX);     //LEFT 
    wing_right_mid = constrain(wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
#endif
#ifdef TRI
    tri_yaw_middle = constrain(tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
#endif
}

void writeParams(void)
{
    FLASH_Status FLASHStatus;
    uint32_t address;
    uint8_t conf[256];
    uint8_t *p = conf;
    uint8_t i;
    
    // TODO this is garbage. do it properly later using FLASH_ProgramHalfWord without caching shit.
    for (i = 0; i < EEBLOCK_SIZE; i++) {
        memcpy(p, eep_entry[i].var, eep_entry[i].size);
        p += eep_entry[i].size;
    }
    
    p = conf;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if ((FLASHStatus = FLASH_ErasePage(FLASH_WRITE_ADDR)) == FLASH_COMPLETE) {
    	address = 0;
    	while (FLASHStatus == FLASH_COMPLETE && address < sizeof(eep_entry)) {
    	    if ((FLASHStatus = FLASH_ProgramWord(FLASH_WRITE_ADDR + address, *(uint32_t *)((char *)p + address))) != FLASH_COMPLETE)
    		    break;
    	    address += 4;
    	}
    }

    FLASH_Lock();

    readEEPROM();
    blinkLED(15, 20, 1);
}

void checkFirstTime(void)
{
    uint8_t test_val, i;
    
    test_val = *(uint8_t *)FLASH_WRITE_ADDR;

    if (test_val == checkNewConf)
        return;

    // Default settings
    P8[ROLL] = 40;
    I8[ROLL] = 30;
    D8[ROLL] = 23;
    P8[PITCH] = 40;
    I8[PITCH] = 30;
    D8[PITCH] = 23;
    P8[YAW] = 85;
    I8[YAW] = 0;
    D8[YAW] = 0;
    P8[PIDALT] = 16;
    I8[PIDALT] = 15;
    D8[PIDALT] = 7;
    P8[PIDGPS] = 10;
    I8[PIDGPS] = 0;
    D8[PIDGPS] = 0;
    P8[PIDVEL] = 0;
    I8[PIDVEL] = 0;
    D8[PIDVEL] = 0;
    P8[PIDLEVEL] = 90;
    I8[PIDLEVEL] = 45;
    D8[PIDLEVEL] = 100;
    P8[PIDMAG] = 40;
    rcRate8 = 45;               // = 0.9 in GUI
    rcExpo8 = 65;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID = 0;
    for (i = 0; i < CHECKBOXITEMS; i++) {
        activate1[i] = 0;
        activate2[i] = 0;
    }
    accTrim[0] = 0;
    accTrim[1] = 0;
    powerTrigger1 = 0;
#ifdef FLYING_WING
    wing_left_mid = WING_LEFT_MID;
    wing_right_mid = WING_RIGHT_MID;
#endif
#ifdef TRI
    tri_yaw_middle = TRI_YAW_MIDDLE;
#endif
    writeParams();
}
