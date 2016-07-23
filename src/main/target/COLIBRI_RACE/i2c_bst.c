/*                                             By Larry Ho Ka Wai @ 23/06/2015*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build_config.h"
#include "debug.h"

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "telemetry/telemetry.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "mw.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "version.h"
#ifdef NAZE
#include "hardware_revision.h"
#endif

#include "bus_bst.h"
#include "i2c_bst.h"

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);

#define BST_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1 // increment when major changes are made
#define API_VERSION_MINOR                   12 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH                  2

#define MULTIWII_IDENTIFIER "MWII";
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define BASEFLIGHT_IDENTIFIER "BAFL";

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
static const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;
#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

#define BST_API_VERSION                 1    //out message
#define BST_FC_VARIANT                  2    //out message
#define BST_FC_VERSION                  3    //out message
#define BST_BOARD_INFO                  4    //out message
#define BST_BUILD_INFO                  5    //out message

//
// MSP commands for Cleanflight original features
//
#define BST_MODE_RANGES                 34    //out message         Returns all mode ranges
#define BST_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define BST_FEATURE                     36
#define BST_SET_FEATURE                 37

#define BST_BOARD_ALIGNMENT             38
#define BST_SET_BOARD_ALIGNMENT         39

#define BST_CURRENT_METER_CONFIG        40
#define BST_SET_CURRENT_METER_CONFIG    41

#define BST_MIXER                       42
#define BST_SET_MIXER                   43

#define BST_RX_CONFIG                   44
#define BST_SET_RX_CONFIG               45

#define BST_LED_COLORS                  46
#define BST_SET_LED_COLORS              47

#define BST_LED_STRIP_CONFIG            48
#define BST_SET_LED_STRIP_CONFIG        49

#define BST_RSSI_CONFIG                 50
#define BST_SET_RSSI_CONFIG             51

#define BST_ADJUSTMENT_RANGES           52
#define BST_SET_ADJUSTMENT_RANGE        53

// private - only to be used by the configurator, the commands are likely to change
#define BST_CF_SERIAL_CONFIG            54
#define BST_SET_CF_SERIAL_CONFIG        55

#define BST_VOLTAGE_METER_CONFIG        56
#define BST_SET_VOLTAGE_METER_CONFIG    57

#define BST_SONAR_ALTITUDE              58 //out message get sonar altitude [cm]

#define BST_PID_CONTROLLER              59
#define BST_SET_PID_CONTROLLER          60

#define BST_ARMING_CONFIG               61 //out message         Returns auto_disarm_delay and disarm_kill_switch parameters
#define BST_SET_ARMING_CONFIG           62 //in message          Sets auto_disarm_delay and disarm_kill_switch parameters

#define BST_DATAFLASH_SUMMARY           80 //out message - get description of dataflash chip
#define BST_DATAFLASH_READ              81 //out message - get content of dataflash chip
#define BST_DATAFLASH_ERASE             82 //in message - erase dataflash chip

#define BST_LOOP_TIME                   83 //out message         Returns FC cycle time i.e looptime parameter
#define BST_SET_LOOP_TIME               84 //in message          Sets FC cycle time i.e looptime parameter

#define BST_FAILSAFE_CONFIG             85 //out message         Returns FC Fail-Safe settings
#define BST_SET_FAILSAFE_CONFIG         86 //in message          Sets FC Fail-Safe settings

#define BST_RXFAIL_CONFIG               87 //out message         Returns RXFAIL settings
#define BST_SET_RXFAIL_CONFIG           88 //in message          Sets RXFAIL settings

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define BST_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define BST_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from BST_RX_MAP

// FIXME - Provided for backwards compatibility with configurator code until configurator is updated.
// DEPRECATED - DO NOT USE "BST_BF_CONFIG" and BST_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
#define BST_BF_CONFIG                   66 //out message baseflight-specific settings that aren't covered elsewhere
#define BST_SET_BF_CONFIG               67 //in message baseflight-specific settings save

#define BST_REBOOT                      68 //in message reboot settings

// DEPRECATED - Use BST_BUILD_INFO instead
#define BST_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

#define BST_DISARM                    70 //in message to disarm
#define BST_ENABLE_ARM                71 //in message to enable arm

#define BST_DEADBAND                72 //out message
#define BST_SET_DEADBAND                73 //in message

#define BST_FC_FILTERS                74 //out message
#define BST_SET_FC_FILTERS            75 //in message

//
// Multwii original MSP commands
//

// DEPRECATED - See BST_API_VERSION and BST_MIXER
#define BST_IDENT                100    //out message         mixerMode + multiwii version + protocol version + capability variable


#define BST_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define BST_RAW_IMU              102    //out message         9 DOF
#define BST_SERVO                103    //out message         servos
#define BST_MOTOR                104    //out message         motor
#define BST_RC                   105    //out message         rc channels and more
#define BST_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define BST_COMP_GPS             107    //out message         distance home, direction home
#define BST_ATTITUDE             108    //out message         2 angles 1 heading
#define BST_ALTITUDE             109    //out message         altitude, variometer
#define BST_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define BST_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define BST_PID                  112    //out message         P I D coeff (9 are used currently)
#define BST_BOX                  113    //out message         BOX setup (number is dependant of your setup)
#define BST_MISC                 114    //out message         powermeter trig
#define BST_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define BST_BOXNAMES             116    //out message         the aux switch names
#define BST_PIDNAMES             117    //out message         the PID names
#define BST_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define BST_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define BST_SERVO_CONFIGURATIONS 120    //out message         All servo configurations.
#define BST_NAV_STATUS           121    //out message         Returns navigation status
#define BST_NAV_CONFIG           122    //out message         Returns navigation parameters

#define BST_SET_RAW_RC           200    //in message          8 rc chan
#define BST_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define BST_SET_PID              202    //in message          P I D coeff (9 are used currently)
#define BST_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define BST_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
#define BST_ACC_CALIBRATION      205    //in message          no param
#define BST_MAG_CALIBRATION      206    //in message          no param
#define BST_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define BST_RESET_CONF           208    //in message          no param
#define BST_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define BST_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define BST_SET_HEAD             211    //in message          define a new heading hold direction
#define BST_SET_SERVO_CONFIGURATION 212    //in message          Servo settings
#define BST_SET_MOTOR            214    //in message          PropBalance function
#define BST_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom

// #define BST_BIND                 240    //in message          no param

#define BST_EEPROM_WRITE         250    //in message          no param

#define BST_DEBUGMSG             253    //out message         debug string buffer
#define BST_DEBUG                254    //out message         debug1,debug2,debug3,debug4

// Additional commands that are not compatible with MultiWii
#define BST_UID                  160    //out message         Unique device ID
#define BST_ACC_TRIM             240    //out message         get acc angle trim values
#define BST_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define BST_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)
#define BST_SERVO_MIX_RULES      241    //out message         Returns servo mixer configuration
#define BST_SET_SERVO_MIX_RULE   242    //in message          Sets servo mixer configuration

extern volatile uint8_t CRC8;
extern volatile bool coreProReady;
extern uint16_t cycleTime; // FIXME dependency on mw.c
extern uint16_t rssi; // FIXME dependency on mw.c

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;
// from mixer.c
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

// cause reboot after BST processing complete
static bool isRebootScheduled = false;

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

#define BOARD_IDENTIFIER_LENGTH                4

typedef struct box_e {
    const uint8_t boxId;                        // see boxId_e
    const char *boxName;                        // GUI-readable box name
    const uint8_t permanentId;                //
} box_t;

// FIXME remove ;'s
static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    //{ BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXGTUNE, "GTUNE;", 21 },
    { BOXSONAR, "SONAR;", 22 },
    { BOXSERVO1, "SERVO1;", 23 },
    { BOXSERVO2, "SERVO2;", 24 },
    { BOXSERVO3, "SERVO3;", 25 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

extern uint8_t readData[DATA_BUFFER_SIZE];
extern uint8_t writeData[DATA_BUFFER_SIZE];

/*************************************************************************************************/
uint8_t writeBufferPointer = 1;
static void bstWrite8(uint8_t data) {
    writeData[writeBufferPointer++] = data;
    writeData[0] = writeBufferPointer;
}

static void bstWrite16(uint16_t data)
{
    bstWrite8((uint8_t)(data >> 0));
    bstWrite8((uint8_t)(data >> 8));
}

static void bstWrite32(uint32_t data)
{
    bstWrite16((uint16_t)(data >> 0));
    bstWrite16((uint16_t)(data >> 16));
}

uint8_t readBufferPointer = 4;
static uint8_t bstCurrentAddress(void)
{
    return readData[0];
}

static uint8_t bstRead8(void)
{
    return readData[readBufferPointer++] & 0xff;
}

static uint16_t bstRead16(void)
{
    uint16_t t = bstRead8();
    t += (uint16_t)bstRead8() << 8;
    return t;
}

static uint32_t bstRead32(void)
{
    uint32_t t = bstRead16();
    t += (uint32_t)bstRead16() << 16;
    return t;
}

static uint8_t bstReadDataSize(void)
{
    return readData[1]-5;
}

static uint8_t bstReadCRC(void)
{
    return readData[readData[1]+1];
}

static void s_struct(uint8_t *cb, uint8_t siz)
{
    while (siz--)
        bstWrite8(*cb++);
}

static void bstWriteNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        bstWrite8(*c);
}

static const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permenantId)
{
    uint8_t boxIndex;
    const box_t *candidate;
    for (boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

static void bstWriteBoxNamesReply(void)
{
    int i, activeBoxId, j, flag = 1, count = 0, len;
    const box_t *box;

reset:
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < activeBoxIdCount; i++) {
        activeBoxId = activeBoxIds[i];

        box = findBoxByActiveBoxId(activeBoxId);
        if (!box) {
            continue;
        }

        len = strlen(box->boxName);
        if (flag) {
            count += len;
        } else {
            for (j = 0; j < len; j++)
                bstWrite8(box->boxName[j]);
        }
    }

    if (flag) {
        flag = 0;
        goto reset;
    }
}

static void bstWriteDataflashSummaryReply(void)
{
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    bstWrite8(flashfsIsReady() ? 1 : 0);
    bstWrite32(geometry->sectors);
    bstWrite32(geometry->totalSize);
    bstWrite32(flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    bstWrite8(0);
    bstWrite32(0);
    bstWrite32(0);
    bstWrite32(0);
#endif
}

#ifdef USE_FLASHFS
static void bstWriteDataflashReadReply(uint32_t address, uint8_t size)
{
    uint8_t buffer[128];
    int bytesRead;

    if (size > sizeof(buffer)) {
        size = sizeof(buffer);
    }

    bstWrite32(address);

    // bytesRead will be lower than that requested if we reach end of volume
    bytesRead = flashfsReadAbs(address, buffer, size);

    for (int i = 0; i < bytesRead; i++) {
        bstWrite8(buffer[i]);
    }
}
#endif

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

/*************************************************************************************************/
#define BST_USB_COMMANDS                                        0x0A
#define BST_GENERAL_HEARTBEAT                                    0x0B
#define BST_USB_DEVICE_INFO_REQUEST                            0x04    //Handshake
#define BST_USB_DEVICE_INFO_FRAME                                0x05    //Handshake
#define BST_READ_COMMANDS                                    0x26
#define BST_WRITE_COMMANDS                                    0x25
#define BST_PASSED                                            0x01
#define BST_FAILED                                            0x00

static bool bstSlaveProcessFeedbackCommand(uint8_t bstRequest)
{
    uint32_t i, tmp, junk;

#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0;
#endif

    switch(bstRequest) {
        case BST_API_VERSION:
            bstWrite8(BST_PROTOCOL_VERSION);

            bstWrite8(API_VERSION_MAJOR);
            bstWrite8(API_VERSION_MINOR);
            break;
        case BST_FC_VARIANT:
            for (i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
                bstWrite8(flightControllerIdentifier[i]);
            }
            break;
        case BST_FC_VERSION:
            bstWrite8(FC_VERSION_MAJOR);
            bstWrite8(FC_VERSION_MINOR);
            bstWrite8(FC_VERSION_PATCH_LEVEL);
            break;
        case BST_BOARD_INFO:
            for (i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
                bstWrite8(boardIdentifier[i]);
            }
            break;
        case BST_BUILD_INFO:
            for (i = 0; i < BUILD_DATE_LENGTH; i++) {
                bstWrite8(buildDate[i]);
            }
            for (i = 0; i < BUILD_TIME_LENGTH; i++) {
                bstWrite8(buildTime[i]);
            }

            for (i = 0; i < GIT_SHORT_REVISION_LENGTH; i++) {
                bstWrite8(shortGitRevision[i]);
            }
            break;
            // DEPRECATED - Use MSP_API_VERSION
        case BST_IDENT:
            bstWrite8(MW_VERSION);
            bstWrite8(masterConfig.mixerMode);
            bstWrite8(BST_PROTOCOL_VERSION);
            bstWrite32(CAP_DYNBALANCE); // "capability"
            break;

        case BST_STATUS:
            bstWrite16(cycleTime);
#ifdef USE_I2C
            bstWrite16(i2cGetErrorCounter());
#else
            bstWrite16(0);
#endif
            bstWrite16(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
            // BST the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
            // Requires new Multiwii protocol version to fix
            // It would be preferable to setting the enabled bits based on BOXINDEX.
            junk = 0;
            tmp = IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << BOXANGLE |
                    IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << BOXHORIZON |
                    IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << BOXBARO |
                    IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << BOXMAG |
                    IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)) << BOXHEADFREE |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)) << BOXHEADADJ |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)) << BOXCAMSTAB |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMTRIG)) << BOXCAMTRIG |
                    IS_ENABLED(FLIGHT_MODE(GPS_HOME_MODE)) << BOXGPSHOME |
                    IS_ENABLED(FLIGHT_MODE(GPS_HOLD_MODE)) << BOXGPSHOLD |
                    IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)) << BOXPASSTHRU |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)) << BOXBEEPERON |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDMAX)) << BOXLEDMAX |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)) << BOXLEDLOW |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)) << BOXLLIGHTS |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCALIB)) << BOXCALIB |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGOV)) << BOXGOV |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)) << BOXOSD |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)) << BOXTELEMETRY |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGTUNE)) << BOXGTUNE |
                    IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << BOXSONAR |
                    IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
                    IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE;
            for (i = 0; i < activeBoxIdCount; i++) {
                int flag = (tmp & (1 << activeBoxIds[i]));
                if (flag)
                    junk |= 1 << i;
            }
            bstWrite32(junk);
            bstWrite8(masterConfig.current_profile_index);
            break;
        case BST_RAW_IMU:
            {
                // Hack scale due to choice of units for sensor data in multiwii
                uint8_t scale = (acc.acc_1G > 1024) ? 8 : 1;

                for (i = 0; i < 3; i++)
                    bstWrite16(accSmooth[i] / scale);
                for (i = 0; i < 3; i++)
                    bstWrite16(gyroADC[i]);
                for (i = 0; i < 3; i++)
                    bstWrite16(magADC[i]);
            }
            break;
#ifdef USE_SERVOS
        case BST_SERVO:
            s_struct((uint8_t *)&servo, MAX_SUPPORTED_SERVOS * 2);
            break;
        case BST_SERVO_CONFIGURATIONS:
            for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
                bstWrite16(masterConfig.servoConf[i].min);
                bstWrite16(masterConfig.servoConf[i].max);
                bstWrite16(masterConfig.servoConf[i].middle);
                bstWrite8(masterConfig.servoConf[i].rate);
                bstWrite8(masterConfig.servoConf[i].angleAtMin);
                bstWrite8(masterConfig.servoConf[i].angleAtMax);
                bstWrite8(masterConfig.servoConf[i].forwardFromChannel);
                bstWrite32(masterConfig.servoConf[i].reversedSources);
            }
            break;
        case BST_SERVO_MIX_RULES:
            for (i = 0; i < MAX_SERVO_RULES; i++) {
                bstWrite8(masterConfig.customServoMixer[i].targetChannel);
                bstWrite8(masterConfig.customServoMixer[i].inputSource);
                bstWrite8(masterConfig.customServoMixer[i].rate);
                bstWrite8(masterConfig.customServoMixer[i].speed);
                bstWrite8(masterConfig.customServoMixer[i].min);
                bstWrite8(masterConfig.customServoMixer[i].max);
                bstWrite8(masterConfig.customServoMixer[i].box);
            }
            break;
#endif
        case BST_MOTOR:
            s_struct((uint8_t *)motor, 16);
            break;
        case BST_RC:
            for (i = 0; i < rxRuntimeConfig.channelCount; i++)
                bstWrite16(rcData[i]);
            break;
        case BST_ATTITUDE:
            for (i = 0; i < 2; i++)
                bstWrite16(attitude.raw[i]);
            //bstWrite16(heading);
            break;
        case BST_ALTITUDE:
#if defined(BARO) || defined(SONAR)
            bstWrite32(altitudeHoldGetEstimatedAltitude());
#else
            bstWrite32(0);
#endif
            bstWrite16(vario);
            break;
        case BST_SONAR_ALTITUDE:
#if defined(SONAR)
            bstWrite32(sonarGetLatestAltitude());
#else
            bstWrite32(0);
#endif
            break;
        case BST_ANALOG:
            bstWrite8((uint8_t)constrain(vbat, 0, 255));
            bstWrite16((uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            bstWrite16(rssi);
            if(masterConfig.batteryConfig.multiwiiCurrentMeterOutput) {
                bstWrite16((uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
            } else
                bstWrite16((int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
            break;
        case BST_ARMING_CONFIG:
            bstWrite8(masterConfig.auto_disarm_delay);
            bstWrite8(masterConfig.disarm_kill_switch);
            break;
        case BST_LOOP_TIME:
            //bstWrite16(masterConfig.looptime);
            bstWrite16(cycleTime);
            break;
        case BST_RC_TUNING:
            bstWrite8(currentControlRateProfile->rcRate8);
            bstWrite8(currentControlRateProfile->rcExpo8);
            for (i = 0 ; i < 3; i++) {
                bstWrite8(currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
            }
            bstWrite8(currentControlRateProfile->dynThrPID);
            bstWrite8(currentControlRateProfile->thrMid8);
            bstWrite8(currentControlRateProfile->thrExpo8);
            bstWrite16(currentControlRateProfile->tpa_breakpoint);
            bstWrite8(currentControlRateProfile->rcYawExpo8);
            break;
        case BST_PID:
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                bstWrite8(currentProfile->pidProfile.P8[i]);
                bstWrite8(currentProfile->pidProfile.I8[i]);
                bstWrite8(currentProfile->pidProfile.D8[i]);
            }
            break;
        case BST_PIDNAMES:
            bstWriteNames(pidnames);
            break;
        case BST_PID_CONTROLLER:
            bstWrite8(currentProfile->pidProfile.pidController);
            break;
        case BST_MODE_RANGES:
            for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
                modeActivationCondition_t *mac = &masterConfig.modeActivationConditions[i];
                const box_t *box = &boxes[mac->modeId];
                bstWrite8(box->permanentId);
                bstWrite8(mac->auxChannelIndex);
                bstWrite8(mac->range.startStep);
                bstWrite8(mac->range.endStep);
            }
            break;
        case BST_ADJUSTMENT_RANGES:
            for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
                adjustmentRange_t *adjRange = &masterConfig.adjustmentRanges[i];
                bstWrite8(adjRange->adjustmentIndex);
                bstWrite8(adjRange->auxChannelIndex);
                bstWrite8(adjRange->range.startStep);
                bstWrite8(adjRange->range.endStep);
                bstWrite8(adjRange->adjustmentFunction);
                bstWrite8(adjRange->auxSwitchChannelIndex);
            }
            break;
        case BST_BOXNAMES:
            bstWriteBoxNamesReply();
            break;
        case BST_BOXIDS:
            for (i = 0; i < activeBoxIdCount; i++) {
                const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
                if (!box) {
                    continue;
                }
                bstWrite8(box->permanentId);
            }
            break;
        case BST_MISC:
            bstWrite16(masterConfig.rxConfig.midrc);

            bstWrite16(masterConfig.escAndServoConfig.minthrottle);
            bstWrite16(masterConfig.escAndServoConfig.maxthrottle);
            bstWrite16(masterConfig.escAndServoConfig.mincommand);

            bstWrite16(masterConfig.failsafeConfig.failsafe_throttle);

#ifdef GPS
            bstWrite8(masterConfig.gpsConfig.provider); // gps_type
            bstWrite8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            bstWrite8(masterConfig.gpsConfig.sbasMode); // gps_ubx_sbas
#else
            bstWrite8(0); // gps_type
            bstWrite8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            bstWrite8(0); // gps_ubx_sbas
#endif
            bstWrite8(masterConfig.batteryConfig.multiwiiCurrentMeterOutput);
            bstWrite8(masterConfig.rxConfig.rssi_channel);
            bstWrite8(0);

            bstWrite16(masterConfig.mag_declination / 10);

            bstWrite8(masterConfig.batteryConfig.vbatscale);
            bstWrite8(masterConfig.batteryConfig.vbatmincellvoltage);
            bstWrite8(masterConfig.batteryConfig.vbatmaxcellvoltage);
            bstWrite8(masterConfig.batteryConfig.vbatwarningcellvoltage);
            break;
        case BST_MOTOR_PINS:
             // FIXME This is hardcoded and should not be.
            for (i = 0; i < 8; i++)
                bstWrite8(i + 1);
            break;
#ifdef GPS
        case BST_RAW_GPS:
            bstWrite8(STATE(GPS_FIX));
            bstWrite8(GPS_numSat);
            bstWrite32(GPS_coord[LAT]);
            bstWrite32(GPS_coord[LON]);
            bstWrite16(GPS_altitude);
            bstWrite16(GPS_speed);
            bstWrite16(GPS_ground_course);
            break;
        case BST_COMP_GPS:
            bstWrite16(GPS_distanceToHome);
            bstWrite16(GPS_directionToHome);
            bstWrite8(GPS_update & 1);
            break;
        case BST_WP:
            wp_no = bstRead8();    // get the wp number
            if (wp_no == 0) {
                lat = GPS_home[LAT];
                lon = GPS_home[LON];
            } else if (wp_no == 16) {
                lat = GPS_hold[LAT];
                lon = GPS_hold[LON];
            }
            bstWrite8(wp_no);
            bstWrite32(lat);
            bstWrite32(lon);
            bstWrite32(AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
            bstWrite16(0);                 // heading  will come here (deg)
            bstWrite16(0);                 // time to stay (ms) will come here
            bstWrite8(0);                  // nav flag will come here
            break;
        case BST_GPSSVINFO:
            bstWrite8(GPS_numCh);
            for (i = 0; i < GPS_numCh; i++){
                bstWrite8(GPS_svinfo_chn[i]);
                bstWrite8(GPS_svinfo_svid[i]);
                bstWrite8(GPS_svinfo_quality[i]);
                bstWrite8(GPS_svinfo_cno[i]);
            }
            break;
#endif
        case BST_DEBUG:
            // output some useful QA statistics
            // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

            for (i = 0; i < DEBUG16_VALUE_COUNT; i++)
                bstWrite16(debug[i]);      // 4 variables are here for general monitoring purpose
            break;

        // Additional commands that are not compatible with MultiWii
        case BST_ACC_TRIM:
            bstWrite16(masterConfig.accelerometerTrims.values.pitch);
            bstWrite16(masterConfig.accelerometerTrims.values.roll);
            break;

        case BST_UID:
            bstWrite32(U_ID_0);
            bstWrite32(U_ID_1);
            bstWrite32(U_ID_2);
            break;

        case BST_FEATURE:
            bstWrite32(featureMask());
            break;

        case BST_BOARD_ALIGNMENT:
            bstWrite16(masterConfig.boardAlignment.rollDegrees);
            bstWrite16(masterConfig.boardAlignment.pitchDegrees);
            bstWrite16(masterConfig.boardAlignment.yawDegrees);
            break;

        case BST_VOLTAGE_METER_CONFIG:
            bstWrite8(masterConfig.batteryConfig.vbatscale);
            bstWrite8(masterConfig.batteryConfig.vbatmincellvoltage);
            bstWrite8(masterConfig.batteryConfig.vbatmaxcellvoltage);
            bstWrite8(masterConfig.batteryConfig.vbatwarningcellvoltage);
            break;

        case BST_CURRENT_METER_CONFIG:
            bstWrite16(masterConfig.batteryConfig.currentMeterScale);
            bstWrite16(masterConfig.batteryConfig.currentMeterOffset);
            bstWrite8(masterConfig.batteryConfig.currentMeterType);
            bstWrite16(masterConfig.batteryConfig.batteryCapacity);
            break;

        case BST_MIXER:
            bstWrite8(masterConfig.mixerMode);
            break;

        case BST_RX_CONFIG:
            bstWrite8(masterConfig.rxConfig.serialrx_provider);
            bstWrite16(masterConfig.rxConfig.maxcheck);
            bstWrite16(masterConfig.rxConfig.midrc);
            bstWrite16(masterConfig.rxConfig.mincheck);
            bstWrite8(masterConfig.rxConfig.spektrum_sat_bind);
            bstWrite16(masterConfig.rxConfig.rx_min_usec);
            bstWrite16(masterConfig.rxConfig.rx_max_usec);
            break;

        case BST_FAILSAFE_CONFIG:
            bstWrite8(masterConfig.failsafeConfig.failsafe_delay);
            bstWrite8(masterConfig.failsafeConfig.failsafe_off_delay);
            bstWrite16(masterConfig.failsafeConfig.failsafe_throttle);
            break;

        case BST_RXFAIL_CONFIG:
            for (i = NON_AUX_CHANNEL_COUNT; i < rxRuntimeConfig.channelCount; i++) {
                bstWrite8(masterConfig.rxConfig.failsafe_channel_configurations[i].mode);
                bstWrite16(RXFAIL_STEP_TO_CHANNEL_VALUE(masterConfig.rxConfig.failsafe_channel_configurations[i].step));
            }
            break;

        case BST_RSSI_CONFIG:
            bstWrite8(masterConfig.rxConfig.rssi_channel);
            break;

        case BST_RX_MAP:
            for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++)
                bstWrite8(masterConfig.rxConfig.rcmap[i]);
            break;

        case BST_BF_CONFIG:
            bstWrite8(masterConfig.mixerMode);

            bstWrite32(featureMask());

            bstWrite8(masterConfig.rxConfig.serialrx_provider);

            bstWrite16(masterConfig.boardAlignment.rollDegrees);
            bstWrite16(masterConfig.boardAlignment.pitchDegrees);
            bstWrite16(masterConfig.boardAlignment.yawDegrees);

            bstWrite16(masterConfig.batteryConfig.currentMeterScale);
            bstWrite16(masterConfig.batteryConfig.currentMeterOffset);
            break;

        case BST_CF_SERIAL_CONFIG:
            for (i = 0; i < SERIAL_PORT_COUNT; i++) {
                if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                    continue;
                };
                bstWrite8(masterConfig.serialConfig.portConfigs[i].identifier);
                bstWrite16(masterConfig.serialConfig.portConfigs[i].functionMask);
                bstWrite8(masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex);
                bstWrite8(masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex);
                bstWrite8(masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex);
                bstWrite8(masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex);
            }
            break;

#ifdef LED_STRIP
        case BST_LED_COLORS:
            for (i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
                hsvColor_t *color = &masterConfig.colors[i];
                bstWrite16(color->h);
                bstWrite8(color->s);
                bstWrite8(color->v);
            }
            break;

        case BST_LED_STRIP_CONFIG:
            for (i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
                ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
                bstWrite32(*ledConfig);
            }
            break;
#endif

        case BST_DATAFLASH_SUMMARY:
            bstWriteDataflashSummaryReply();
            break;

#ifdef USE_FLASHFS
        case BST_DATAFLASH_READ:
            {
                uint32_t readAddress = bstRead32();

                bstWriteDataflashReadReply(readAddress, 128);
            }
            break;
#endif

        case BST_BF_BUILD_INFO:
            for (i = 0; i < 11; i++)
                bstWrite8(buildDate[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            bstWrite32(0); // future exp
            bstWrite32(0); // future exp
            break;
        case BST_DEADBAND:
            bstWrite8(masterConfig.rcControlsConfig.alt_hold_deadband);
            bstWrite8(masterConfig.rcControlsConfig.alt_hold_fast_change);
            bstWrite8(masterConfig.rcControlsConfig.deadband);
            bstWrite8(masterConfig.rcControlsConfig.yaw_deadband);
            break;
        case BST_FC_FILTERS:
            bstWrite16(constrain(masterConfig.gyro_lpf, 0, 1)); // Extra safety to prevent OSD setting corrupt values
            break;
        default:
            // we do not know how to handle the (valid) message, indicate error BST
            return false;
    }
    //bstSlaveWrite(writeData);
    return true;
}

static bool bstSlaveProcessWriteCommand(uint8_t bstWriteCommand)
{
    uint32_t i;
    uint16_t tmp;
    uint8_t rate;

#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif

    bool ret = BST_PASSED;
    switch(bstWriteCommand) {
        case BST_SELECT_SETTING:
            if (!ARMING_FLAG(ARMED)) {
                masterConfig.current_profile_index = bstRead8();
                if (masterConfig.current_profile_index > 2) {
                    masterConfig.current_profile_index = 0;
                }
                writeEEPROM();
                readEEPROM();
            }
            break;
        case BST_SET_HEAD:
            magHold = bstRead16();
            break;
        case BST_SET_RAW_RC:
            {
                uint8_t channelCount = bstReadDataSize() / sizeof(uint16_t);
                if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                    ret = BST_FAILED;
                } else {
                    uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];

                    for (i = 0; i < channelCount; i++) {
                        frame[i] = bstRead16();
                    }

                    rxMspFrameReceive(frame, channelCount);
                }
            }
        case BST_SET_ACC_TRIM:
            masterConfig.accelerometerTrims.values.pitch = bstRead16();
            masterConfig.accelerometerTrims.values.roll  = bstRead16();
            break;
        case BST_SET_ARMING_CONFIG:
            masterConfig.auto_disarm_delay = bstRead8();
            masterConfig.disarm_kill_switch = bstRead8();
            break;
        case BST_SET_LOOP_TIME:
            //masterConfig.looptime = bstRead16();
            cycleTime = bstRead16();
            break;
        case BST_SET_PID_CONTROLLER:
            currentProfile->pidProfile.pidController = bstRead8();
            pidSetController(currentProfile->pidProfile.pidController);
            break;
        case BST_SET_PID:
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                currentProfile->pidProfile.P8[i] = bstRead8();
                currentProfile->pidProfile.I8[i] = bstRead8();
                currentProfile->pidProfile.D8[i] = bstRead8();
            }
            break;
        case BST_SET_MODE_RANGE:
            i = bstRead8();
            if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                modeActivationCondition_t *mac = &masterConfig.modeActivationConditions[i];
                i = bstRead8();
                const box_t *box = findBoxByPermenantId(i);
                if (box) {
                    mac->modeId = box->boxId;
                    mac->auxChannelIndex = bstRead8();
                    mac->range.startStep = bstRead8();
                    mac->range.endStep = bstRead8();

                    useRcControlsConfig(masterConfig.modeActivationConditions, &masterConfig.escAndServoConfig, &currentProfile->pidProfile);
                } else {
                    ret = BST_FAILED;
                }
            } else {
                ret = BST_FAILED;
            }
            break;
        case BST_SET_ADJUSTMENT_RANGE:
            i = bstRead8();
            if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
                adjustmentRange_t *adjRange = &masterConfig.adjustmentRanges[i];
                i = bstRead8();
                if (i < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    adjRange->adjustmentIndex = i;
                    adjRange->auxChannelIndex = bstRead8();
                    adjRange->range.startStep = bstRead8();
                    adjRange->range.endStep = bstRead8();
                    adjRange->adjustmentFunction = bstRead8();
                    adjRange->auxSwitchChannelIndex = bstRead8();
                } else {
                    ret = BST_FAILED;
                }
            } else {
                ret = BST_FAILED;
            }
            break;
        case BST_SET_RC_TUNING:
            if (bstReadDataSize() >= 10) {
                currentControlRateProfile->rcRate8 = bstRead8();
                currentControlRateProfile->rcExpo8 = bstRead8();
                for (i = 0; i < 3; i++) {
                    rate = bstRead8();
                    currentControlRateProfile->rates[i] = MIN(rate, i == FD_YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
                }
                rate = bstRead8();
                currentControlRateProfile->dynThrPID = MIN(rate, CONTROL_RATE_CONFIG_TPA_MAX);
                currentControlRateProfile->thrMid8 = bstRead8();
                currentControlRateProfile->thrExpo8 = bstRead8();
                currentControlRateProfile->tpa_breakpoint = bstRead16();
                if (bstReadDataSize() >= 11) {
                    currentControlRateProfile->rcYawExpo8 = bstRead8();
                }
            } else {
                ret = BST_FAILED;
            }
            break;
        case BST_SET_MISC:
            tmp = bstRead16();
            if (tmp < 1600 && tmp > 1400)
                masterConfig.rxConfig.midrc = tmp;

            masterConfig.escAndServoConfig.minthrottle = bstRead16();
            masterConfig.escAndServoConfig.maxthrottle = bstRead16();
            masterConfig.escAndServoConfig.mincommand = bstRead16();

            masterConfig.failsafeConfig.failsafe_throttle = bstRead16();

    #ifdef GPS
            masterConfig.gpsConfig.provider = bstRead8(); // gps_type
            bstRead8(); // gps_baudrate
            masterConfig.gpsConfig.sbasMode = bstRead8(); // gps_ubx_sbas
    #else
            bstRead8(); // gps_type
            bstRead8(); // gps_baudrate
            bstRead8(); // gps_ubx_sbas
    #endif
            masterConfig.batteryConfig.multiwiiCurrentMeterOutput = bstRead8();
            masterConfig.rxConfig.rssi_channel = bstRead8();
            bstRead8();

            masterConfig.mag_declination = bstRead16() * 10;

            masterConfig.batteryConfig.vbatscale = bstRead8();           // actual vbatscale as intended
            masterConfig.batteryConfig.vbatmincellvoltage = bstRead8();  // vbatlevel_warn1 in MWC2.3 GUI
            masterConfig.batteryConfig.vbatmaxcellvoltage = bstRead8();  // vbatlevel_warn2 in MWC2.3 GUI
            masterConfig.batteryConfig.vbatwarningcellvoltage = bstRead8();  // vbatlevel when buzzer starts to alert
            break;
        case BST_SET_MOTOR:
            for (i = 0; i < 8; i++) // FIXME should this use MAX_MOTORS or MAX_SUPPORTED_MOTORS instead of 8
                motor_disarmed[i] = bstRead16();
            break;
        case BST_SET_SERVO_CONFIGURATION:
#ifdef USE_SERVOS
           if (bstReadDataSize() != 1 + sizeof(servoParam_t)) {
               ret = BST_FAILED;
               break;
           }
           i = bstRead8();
           if (i >= MAX_SUPPORTED_SERVOS) {
               ret = BST_FAILED;
           } else {
               masterConfig.servoConf[i].min = bstRead16();
               masterConfig.servoConf[i].max = bstRead16();
               masterConfig.servoConf[i].middle = bstRead16();
               masterConfig.servoConf[i].rate = bstRead8();
               masterConfig.servoConf[i].angleAtMin = bstRead8();
               masterConfig.servoConf[i].angleAtMax = bstRead8();
               masterConfig.servoConf[i].forwardFromChannel = bstRead8();
               masterConfig.servoConf[i].reversedSources = bstRead32();
           }
#endif
           break;
        case BST_SET_SERVO_MIX_RULE:
#ifdef USE_SERVOS
           i = bstRead8();
           if (i >= MAX_SERVO_RULES) {
               ret = BST_FAILED;
           } else {
               masterConfig.customServoMixer[i].targetChannel = bstRead8();
               masterConfig.customServoMixer[i].inputSource = bstRead8();
               masterConfig.customServoMixer[i].rate = bstRead8();
               masterConfig.customServoMixer[i].speed = bstRead8();
               masterConfig.customServoMixer[i].min = bstRead8();
               masterConfig.customServoMixer[i].max = bstRead8();
               masterConfig.customServoMixer[i].box = bstRead8();
               loadCustomServoMixer();
           }
#endif
           break;
        case BST_RESET_CONF:
           if (!ARMING_FLAG(ARMED)) {
               resetEEPROM();
               readEEPROM();
           }
           break;
        case BST_ACC_CALIBRATION:
           if (!ARMING_FLAG(ARMED))
               accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
           break;
        case BST_MAG_CALIBRATION:
           if (!ARMING_FLAG(ARMED))
               ENABLE_STATE(CALIBRATE_MAG);
           break;
        case BST_EEPROM_WRITE:
            if (ARMING_FLAG(ARMED)) {
                ret = BST_FAILED;
                bstWrite8(ret);
                //bstSlaveWrite(writeData);
                return ret;
            }
            writeEEPROM();
            readEEPROM();
            break;
#ifdef USE_FLASHFS
        case BST_DATAFLASH_ERASE:
            flashfsEraseCompletely();
            break;
#endif
#ifdef GPS
        case BST_SET_RAW_GPS:
            if (bstRead8()) {
                ENABLE_STATE(GPS_FIX);
            } else {
                DISABLE_STATE(GPS_FIX);
            }
            GPS_numSat = bstRead8();
            GPS_coord[LAT] = bstRead32();
            GPS_coord[LON] = bstRead32();
            GPS_altitude = bstRead16();
            GPS_speed = bstRead16();
            GPS_update |= 2;        // New data signalisation to GPS functions // FIXME Magic Numbers
            break;
        case BST_SET_WP:
            wp_no = bstRead8();    //get the wp number
            lat = bstRead32();
            lon = bstRead32();
            alt = bstRead32();     // to set altitude (cm)
            bstRead16();           // future: to set heading (deg)
            bstRead16();           // future: to set time to stay (ms)
            bstRead8();            // future: to set nav flag
            if (wp_no == 0) {
                GPS_home[LAT] = lat;
                GPS_home[LON] = lon;
                DISABLE_FLIGHT_MODE(GPS_HOME_MODE);        // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
                ENABLE_STATE(GPS_FIX_HOME);
                if (alt != 0)
                    AltHold = alt;          // temporary implementation to test feature with apps
            } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
                GPS_hold[LAT] = lat;
                GPS_hold[LON] = lon;
                if (alt != 0)
                    AltHold = alt;          // temporary implementation to test feature with apps
                nav_mode = NAV_MODE_WP;
                GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            }
            break;
#endif
        case BST_SET_FEATURE:
            featureClearAll();
            featureSet(bstRead32()); // features bitmap
            break;
        case BST_SET_BOARD_ALIGNMENT:
            masterConfig.boardAlignment.rollDegrees = bstRead16();
            masterConfig.boardAlignment.pitchDegrees = bstRead16();
            masterConfig.boardAlignment.yawDegrees = bstRead16();
            break;
        case BST_SET_VOLTAGE_METER_CONFIG:
            masterConfig.batteryConfig.vbatscale = bstRead8();           // actual vbatscale as intended
            masterConfig.batteryConfig.vbatmincellvoltage = bstRead8();  // vbatlevel_warn1 in MWC2.3 GUI
            masterConfig.batteryConfig.vbatmaxcellvoltage = bstRead8();  // vbatlevel_warn2 in MWC2.3 GUI
            masterConfig.batteryConfig.vbatwarningcellvoltage = bstRead8();  // vbatlevel when buzzer starts to alert
            break;
        case BST_SET_CURRENT_METER_CONFIG:
            masterConfig.batteryConfig.currentMeterScale = bstRead16();
            masterConfig.batteryConfig.currentMeterOffset = bstRead16();
            masterConfig.batteryConfig.currentMeterType = bstRead8();
            masterConfig.batteryConfig.batteryCapacity = bstRead16();
            break;

#ifndef USE_QUAD_MIXER_ONLY
        case BST_SET_MIXER:
            masterConfig.mixerMode = bstRead8();
            break;
#endif

        case BST_SET_RX_CONFIG:
           masterConfig.rxConfig.serialrx_provider = bstRead8();
           masterConfig.rxConfig.maxcheck = bstRead16();
           masterConfig.rxConfig.midrc = bstRead16();
           masterConfig.rxConfig.mincheck = bstRead16();
           masterConfig.rxConfig.spektrum_sat_bind = bstRead8();
           if (bstReadDataSize() > 8) {
               masterConfig.rxConfig.rx_min_usec = bstRead16();
               masterConfig.rxConfig.rx_max_usec = bstRead16();
           }
           break;
        case BST_SET_FAILSAFE_CONFIG:
           masterConfig.failsafeConfig.failsafe_delay = bstRead8();
           masterConfig.failsafeConfig.failsafe_off_delay = bstRead8();
           masterConfig.failsafeConfig.failsafe_throttle = bstRead16();
           break;
        case BST_SET_RXFAIL_CONFIG:
           {
               uint8_t channelCount = bstReadDataSize() / 3;
               if (channelCount > MAX_AUX_CHANNEL_COUNT) {
                   ret = BST_FAILED;
               } else {
                   for (i = NON_AUX_CHANNEL_COUNT; i < channelCount; i++) {
                       masterConfig.rxConfig.failsafe_channel_configurations[i].mode = bstRead8();
                       masterConfig.rxConfig.failsafe_channel_configurations[i].step = CHANNEL_VALUE_TO_RXFAIL_STEP(bstRead16());
                   }
               }
           }
           break;
        case BST_SET_RSSI_CONFIG:
           masterConfig.rxConfig.rssi_channel = bstRead8();
           break;
        case BST_SET_RX_MAP:
            for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
                masterConfig.rxConfig.rcmap[i] = bstRead8();
            }
            break;
        case BST_SET_BF_CONFIG:

#ifdef USE_QUAD_MIXER_ONLY
           bstRead8(); // mixerMode ignored
#else
           masterConfig.mixerMode = bstRead8(); // mixerMode
#endif

           featureClearAll();
           featureSet(bstRead32()); // features bitmap

           masterConfig.rxConfig.serialrx_provider = bstRead8(); // serialrx_type

           masterConfig.boardAlignment.rollDegrees = bstRead16(); // board_align_roll
           masterConfig.boardAlignment.pitchDegrees = bstRead16(); // board_align_pitch
           masterConfig.boardAlignment.yawDegrees = bstRead16(); // board_align_yaw

           masterConfig.batteryConfig.currentMeterScale = bstRead16();
           masterConfig.batteryConfig.currentMeterOffset = bstRead16();
           break;
        case BST_SET_CF_SERIAL_CONFIG:
           {
               uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);
               if (bstReadDataSize() % portConfigSize != 0) {
                   ret = BST_FAILED;
                   break;
               }

               uint8_t remainingPortsInPacket = bstReadDataSize() / portConfigSize;

               while (remainingPortsInPacket--) {
                   uint8_t identifier = bstRead8();

                   serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                   if (!portConfig) {
                       ret = BST_FAILED;
                       break;
                   }

                   portConfig->identifier = identifier;
                   portConfig->functionMask = bstRead16();
                   portConfig->msp_baudrateIndex = bstRead8();
                   portConfig->gps_baudrateIndex = bstRead8();
                   portConfig->telemetry_baudrateIndex = bstRead8();
                   portConfig->blackbox_baudrateIndex = bstRead8();
               }
           }
           break;
#ifdef LED_STRIP
        case BST_SET_LED_COLORS:
           //for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
           {
               i = bstRead8();
               hsvColor_t *color = &masterConfig.colors[i];
               color->h = bstRead16();
               color->s = bstRead8();
               color->v = bstRead8();
           }
           break;
        case BST_SET_LED_STRIP_CONFIG:
           {
               i = bstRead8();
               if (i >= LED_MAX_STRIP_LENGTH || bstReadDataSize() != (1 + 4)) {
                   ret = BST_FAILED;
                   break;
               }
               ledConfig_t *ledConfig = &masterConfig.ledConfigs[i];
               *ledConfig = bstRead32();
               reevaluateLedConfig();
           }
           break;
#endif
        case BST_REBOOT:
            isRebootScheduled = true;
            break;
        case BST_DISARM:
            if (ARMING_FLAG(ARMED))
                    mwDisarm();
            ENABLE_ARMING_FLAG(PREVENT_ARMING);
            break;
        case BST_ENABLE_ARM:
                DISABLE_ARMING_FLAG(PREVENT_ARMING);
            break;
        case BST_SET_DEADBAND:
            masterConfig.rcControlsConfig.alt_hold_deadband = bstRead8();
            masterConfig.rcControlsConfig.alt_hold_fast_change = bstRead8();
            masterConfig.rcControlsConfig.deadband = bstRead8();
            masterConfig.rcControlsConfig.yaw_deadband = bstRead8();
            break;
        case BST_SET_FC_FILTERS:
            masterConfig.gyro_lpf = bstRead16();
            break;

        default:
            // we do not know how to handle the (valid) message, indicate error BST
            ret = BST_FAILED;
    }
    bstWrite8(ret);
    //bstSlaveWrite(writeData);
    if(ret == BST_FAILED)
        return false;
    return true;
}

static bool bstSlaveUSBCommandFeedback(/*uint8_t bstFeedback*/)
{
    bstWrite8(BST_USB_DEVICE_INFO_FRAME);                        //Sub CPU Device Info FRAME
    bstWrite8(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
    bstWrite8(0x00);
    bstWrite8(0x00);
    bstWrite8(0x00);
    bstWrite8(FC_VERSION_MAJOR);                                //Firmware ID
    bstWrite8(FC_VERSION_MINOR);                                //Firmware ID
    bstWrite8(0x00);
    bstWrite8(0x00);
    //bstSlaveWrite(writeData);
    return true;
}

/*************************************************************************************************/
#define BST_RESET_TIME            1.2*1000*1000 //micro-seconds
uint32_t resetBstTimer = 0;
bool needResetCheck = true;
extern bool cleanflight_data_ready;

void bstProcessInCommand(void)
{
    readBufferPointer = 2;
    if(bstCurrentAddress() == CLEANFLIGHT_FC) {
        if(bstReadCRC() == CRC8 && bstRead8()==BST_USB_COMMANDS) {
            uint8_t i;
            writeBufferPointer = 1;
            cleanflight_data_ready = false;
            for(i = 0; i < DATA_BUFFER_SIZE; i++) {
                writeData[i] = 0;
            }
            switch (bstRead8()) {
                case BST_USB_DEVICE_INFO_REQUEST:
                    bstRead8();
                    if(bstSlaveUSBCommandFeedback(/*bstRead8()*/))
                        coreProReady = true;
                    break;
                case BST_READ_COMMANDS:
                    bstWrite8(BST_READ_COMMANDS);
                    bstSlaveProcessFeedbackCommand(bstRead8());
                    break;
                case BST_WRITE_COMMANDS:
                    bstWrite8(BST_WRITE_COMMANDS);
                    bstSlaveProcessWriteCommand(bstRead8());
                    break;
                default:
                    // we do not know how to handle the (valid) message, indicate error BST
                    break;
            }
            cleanflight_data_ready = true;
        }
    } else if(bstCurrentAddress() == 0x00) {
        if(bstReadCRC() == CRC8 && bstRead8()==BST_GENERAL_HEARTBEAT) {
            resetBstTimer = micros();
            needResetCheck = true;
        }
    }
}

void resetBstChecker(void)
{
    if(needResetCheck) {
        uint32_t currentTimer = micros();
        if(currentTimer >= (resetBstTimer + BST_RESET_TIME))
        {
            bstTimeoutUserCallback();
            needResetCheck = false;
        }
    }
}

/*************************************************************************************************/
#define UPDATE_AT_02HZ ((1000 * 1000) / 2)
static uint32_t next02hzUpdateAt_1 = 0;

#define UPDATE_AT_20HZ ((1000 * 1000) / 20)
static uint32_t next20hzUpdateAt_1 = 0;

static uint8_t sendCounter = 0;

void taskBstMasterProcess(void)
{
    if(coreProReady) {
        uint32_t now = micros();
        if(now >= next02hzUpdateAt_1 && !bstWriteBusy()) {
            writeFCModeToBST();
            next02hzUpdateAt_1 = now + UPDATE_AT_02HZ;
        }
        if(now >= next20hzUpdateAt_1 && !bstWriteBusy()) {
            if(sendCounter == 0)
                writeRCChannelToBST();
            else if(sendCounter == 1)
                writeRollPitchYawToBST();
            sendCounter++;
            if(sendCounter > 1)
                sendCounter = 0;
            next20hzUpdateAt_1 = now + UPDATE_AT_20HZ;
        }

        if(sensors(SENSOR_GPS) && !bstWriteBusy())
            writeGpsPositionPrameToBST();
    }
    bstMasterWriteLoop();
    if (isRebootScheduled) {
        stopMotors();
        systemReset();
    }
    resetBstChecker();
}

/*************************************************************************************************/
static uint8_t masterWriteBufferPointer;
static uint8_t masterWriteData[DATA_BUFFER_SIZE];

static void bstMasterStartBuffer(uint8_t address)
{
    masterWriteData[0] = address;
    masterWriteBufferPointer = 2;
}

static void bstMasterWrite8(uint8_t data)
{
    masterWriteData[masterWriteBufferPointer++] = data;
    masterWriteData[1] = masterWriteBufferPointer;
}

static void bstMasterWrite16(uint16_t data)
{
    bstMasterWrite8((uint8_t)(data >> 8));
    bstMasterWrite8((uint8_t)(data >> 0));
}

static void bstMasterWrite32(uint32_t data)
{
    bstMasterWrite16((uint8_t)(data >> 16));
    bstMasterWrite16((uint8_t)(data >> 0));
}

/*************************************************************************************************/
#define PUBLIC_ADDRESS            0x00

static int32_t lat = 0;
static int32_t lon = 0;
static uint16_t alt = 0;
static uint8_t numOfSat = 0;

bool writeGpsPositionPrameToBST(void)
{
    if((lat != GPS_coord[LAT]) || (lon != GPS_coord[LON]) || (alt != GPS_altitude) || (numOfSat != GPS_numSat)) {
        lat = GPS_coord[LAT];
        lon = GPS_coord[LON];
        alt = GPS_altitude;
        numOfSat = GPS_numSat;
        uint16_t speed = (GPS_speed * 9 / 25);
        uint16_t gpsHeading = 0;
        uint16_t altitude = 0;
        gpsHeading = GPS_ground_course * 10;
        altitude = alt * 10 + 1000;

        bstMasterStartBuffer(PUBLIC_ADDRESS);
        bstMasterWrite8(GPS_POSITION_FRAME_ID);
        bstMasterWrite32(lat);
        bstMasterWrite32(lon);
        bstMasterWrite16(speed);
        bstMasterWrite16(gpsHeading);
        bstMasterWrite16(altitude);
        bstMasterWrite8(numOfSat);
        bstMasterWrite8(0x00);

        return bstMasterWrite(masterWriteData);
    } else
        return false;
}

bool writeRollPitchYawToBST(void)
{
    int16_t X = -attitude.values.pitch * (M_PIf / 1800.0f) * 10000;
    int16_t Y = attitude.values.roll * (M_PIf / 1800.0f) * 10000;
    int16_t Z = 0;//radiusHeading * 10000;

    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(FC_ATTITUDE_FRAME_ID);
    bstMasterWrite16(X);
    bstMasterWrite16(Y);
    bstMasterWrite16(Z);

    return bstMasterWrite(masterWriteData);
}

bool writeRCChannelToBST(void)
{
    uint8_t i = 0;
    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(RC_CHANNEL_FRAME_ID);
    for(i = 0; i < (USABLE_TIMER_CHANNEL_COUNT-1); i++) {
        bstMasterWrite16(rcData[i]);
    }

    return bstMasterWrite(masterWriteData);
}

bool writeFCModeToBST(void)
{
#ifdef CLEANFLIGHT_FULL_STATUS_SET
    uint32_t tmp = 0;
     tmp = IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << BOXANGLE |
            IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << BOXHORIZON |
            IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << BOXBARO |
            IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << BOXMAG |
            IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)) << BOXHEADFREE |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)) << BOXHEADADJ |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)) << BOXCAMSTAB |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMTRIG)) << BOXCAMTRIG |
            IS_ENABLED(FLIGHT_MODE(GPS_HOME_MODE)) << BOXGPSHOME |
            IS_ENABLED(FLIGHT_MODE(GPS_HOLD_MODE)) << BOXGPSHOLD |
            IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)) << BOXPASSTHRU |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)) << BOXBEEPERON |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDMAX)) << BOXLEDMAX |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)) << BOXLEDLOW |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)) << BOXLLIGHTS |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCALIB)) << BOXCALIB |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGOV)) << BOXGOV |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)) << BOXOSD |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)) << BOXTELEMETRY |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGTUNE)) << BOXGTUNE |
            IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << BOXSONAR |
            IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
            IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
            IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE;
    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(CLEANFLIGHT_MODE_FRAME_ID);
    bstMasterWrite32(tmp);
    bstMasterWrite16(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
#else
    uint8_t tmp = 0;
    tmp = IS_ENABLED(ARMING_FLAG(ARMED)) |
           IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << 1 |
           IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << 2 |
           IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << 3 |
           IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << 4 |
           IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << 5 |
           IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << 6 |
           IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << 7;

    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(CLEANFLIGHT_MODE_FRAME_ID);
    bstMasterWrite8(tmp);
    bstMasterWrite8(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
#endif

    return bstMasterWrite(masterWriteData);
}
/*************************************************************************************************/
