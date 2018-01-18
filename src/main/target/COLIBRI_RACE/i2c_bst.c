/*                                             By Larry Ho Ka Wai @ 23/06/2015*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/bus_i2c.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/motors.h"
#include "io/servos.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "bus_bst.h"
#include "i2c_bst.h"

#define GPS_POSITION_FRAME_ID               0x02
#define GPS_TIME_FRAME_ID                   0x03
#define FC_ATTITUDE_FRAME_ID                0x1E
#define RC_CHANNEL_FRAME_ID                 0x15
#define CROSSFIRE_RSSI_FRAME_ID             0x14
#define CLEANFLIGHT_MODE_FRAME_ID           0x20

#define BST_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1 // increment when major changes are made
#define API_VERSION_MINOR                   13 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH                  2

//
// MSP commands for Cleanflight original features
//

#define BST_API_VERSION                 1    //out message
#define BST_FC_VARIANT                  2    //out message
#define BST_FC_VERSION                  3    //out message
#define BST_BOARD_INFO                  4    //out message
#define BST_BUILD_INFO                  5    //out message

#define BST_MODE_RANGES                 34  //out message         Returns all mode ranges
#define BST_SET_MODE_RANGE              35  //in message          Sets a single mode range
#define BST_FEATURE                     36
#define BST_SET_FEATURE                 37
#define BST_RX_CONFIG                   44
#define BST_SET_RX_CONFIG               45
#define BST_LED_COLORS                  46
#define BST_SET_LED_COLORS              47
#define BST_LED_STRIP_CONFIG            48
#define BST_SET_LED_STRIP_CONFIG        49
#define BST_LOOP_TIME                   83  //out message         Returns FC cycle time i.e looptime parameter
#define BST_SET_LOOP_TIME               84  //in message          Sets FC cycle time i.e looptime parameter
#define BST_RX_MAP                      64  //out message         Get channel map (also returns number of channels total)
#define BST_SET_RX_MAP                  65  //in message          Set rx map, numchannels to set comes from BST_RX_MAP
#define BST_REBOOT                      68  //in message          Reboot
#define BST_DISARM                      70  //in message          Disarm
#define BST_ENABLE_ARM                  71  //in message          Enable arm
#define BST_DEADBAND                    72  //out message
#define BST_SET_DEADBAND                73  //in message
#define BST_FC_FILTERS                  74  //out message
#define BST_SET_FC_FILTERS              75  //in message
#define BST_STATUS                      101 //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define BST_RC_TUNING                   111 //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define BST_PID                         112 //out message         P I D coeff (9 are used currently)
#define BST_MISC                        114 //out message         powermeter trig
#define BST_SET_PID                     202 //in message          P I D coeff (9 are used currently)
#define BST_ACC_CALIBRATION             205 //in message          no param
#define BST_MAG_CALIBRATION             206 //in message          no param
#define BST_SET_MISC                    207 //in message          powermeter trig + 8 free for future use
#define BST_SELECT_SETTING              210 //in message          Select Setting Number (0-2)
#define BST_EEPROM_WRITE                250 //in message          no param

extern volatile uint8_t CRC8;
extern volatile bool coreProReady;

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;
// from mixer.c
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];

// cause reboot after BST processing complete
static bool isRebootScheduled = false;

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
    { BOXOSD, "OSD DISABLE SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXGTUNE, "GTUNE;", 21 },
    { BOXRANGEFINDER, "RANGEFINDER;", 22 },
    { BOXSERVO1, "SERVO1;", 23 },
    { BOXSERVO2, "SERVO2;", 24 },
    { BOXSERVO3, "SERVO3;", 25 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

extern uint8_t readData[BST_BUFFER_SIZE];
extern uint8_t writeData[BST_BUFFER_SIZE];

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

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

/*************************************************************************************************/
#define BST_USB_COMMANDS                                        0x0A
#define BST_GENERAL_HEARTBEAT                                   0x0B
#define BST_USB_DEVICE_INFO_REQUEST                             0x04    //Handshake
#define BST_USB_DEVICE_INFO_FRAME                               0x05    //Handshake
#define BST_READ_COMMANDS                                       0x26
#define BST_WRITE_COMMANDS                                      0x25
#define BST_PASSED                                              0x01
#define BST_FAILED                                              0x00

static bool bstSlaveProcessFeedbackCommand(uint8_t bstRequest)
{
    uint32_t i, tmp, junk;

    switch (bstRequest) {
        case BST_API_VERSION:
            bstWrite8(BST_PROTOCOL_VERSION);

            bstWrite8(API_VERSION_MAJOR);
            bstWrite8(API_VERSION_MINOR);
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
        case BST_STATUS:
            bstWrite16(getTaskDeltaTime(TASK_GYROPID));
#ifdef USE_I2C
            bstWrite16(i2cGetErrorCounter());
#else
            bstWrite16(0);
#endif
            bstWrite16(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4);
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
                    IS_ENABLED(FLIGHT_MODE(RANGEFINDER_MODE)) << BOXRANGEFINDER |
                    IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
                    IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
                    IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE;
            for (i = 0; i < activeBoxIdCount; i++) {
                int flag = (tmp & (1 << activeBoxIds[i]));
                if (flag)
                    junk |= 1 << i;
            }
            bstWrite32(junk);
            bstWrite8(getCurrentPidProfileIndex());
            break;
        case BST_LOOP_TIME:
            bstWrite16(getTaskDeltaTime(TASK_GYROPID));
            break;
        case BST_RC_TUNING:
            bstWrite8(currentControlRateProfile->rcRates[FD_ROLL]);
            bstWrite8(currentControlRateProfile->rcExpo[FD_ROLL]);
            for (i = 0 ; i < 3; i++) {
                bstWrite8(currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
            }
            bstWrite8(currentControlRateProfile->dynThrPID);
            bstWrite8(currentControlRateProfile->thrMid8);
            bstWrite8(currentControlRateProfile->thrExpo8);
            bstWrite16(currentControlRateProfile->tpa_breakpoint);
            bstWrite8(currentControlRateProfile->rcExpo[FD_YAW]);
            bstWrite8(currentControlRateProfile->rcRates[FD_YAW]);
            break;
        case BST_PID:
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                bstWrite8(currentPidProfile->pid[i].P);
                bstWrite8(currentPidProfile->pid[i].I);
                bstWrite8(currentPidProfile->pid[i].D);
            }
            pidInitConfig(currentPidProfile);
            break;
        case BST_MODE_RANGES:
            for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
                const modeActivationCondition_t *mac = modeActivationConditions(i);
                const box_t *box = &boxes[mac->modeId];
                bstWrite8(box->permanentId);
                bstWrite8(mac->auxChannelIndex);
                bstWrite8(mac->range.startStep);
                bstWrite8(mac->range.endStep);
            }
            break;
        case BST_MISC:
            bstWrite16(rxConfig()->midrc);

            bstWrite16(motorConfig()->minthrottle);
            bstWrite16(motorConfig()->maxthrottle);
            bstWrite16(motorConfig()->mincommand);

            bstWrite16(failsafeConfig()->failsafe_throttle);

#ifdef USE_GPS
            bstWrite8(gpsConfig()->provider); // gps_type
            bstWrite8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            bstWrite8(gpsConfig()->sbasMode); // gps_ubx_sbas
#else
            bstWrite8(0); // gps_type
            bstWrite8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            bstWrite8(0); // gps_ubx_sbas
#endif
            bstWrite8(0); // legacy - was multiwiiCurrentMeterOutput);
            bstWrite8(rxConfig()->rssi_channel);
            bstWrite8(0);

            bstWrite16(compassConfig()->mag_declination / 10);

            bstWrite8(voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale);
            bstWrite8(batteryConfig()->vbatmincellvoltage);
            bstWrite8(batteryConfig()->vbatmaxcellvoltage);
            bstWrite8(batteryConfig()->vbatwarningcellvoltage);
            break;

        case BST_FEATURE:
            bstWrite32(featureMask());
            break;

        case BST_RX_CONFIG:
            bstWrite8(rxConfig()->serialrx_provider);
            bstWrite16(rxConfig()->maxcheck);
            bstWrite16(rxConfig()->midrc);
            bstWrite16(rxConfig()->mincheck);
            bstWrite8(rxConfig()->spektrum_sat_bind);
            bstWrite16(rxConfig()->rx_min_usec);
            bstWrite16(rxConfig()->rx_max_usec);
            break;

        case BST_RX_MAP:
            for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++)
                bstWrite8(rxConfig()->rcmap[i]);
            break;


#ifdef USE_LED_STRIP
        case BST_LED_COLORS:
            for (i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
                hsvColor_t *color = &ledStripConfigMutable()->colors[i];
                bstWrite16(color->h);
                bstWrite8(color->s);
                bstWrite8(color->v);
            }
            break;

        case BST_LED_STRIP_CONFIG:
            for (i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
                const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[i];
                bstWrite32(*ledConfig);
            }
            break;
#endif
        case BST_DEADBAND:
            bstWrite8(rcControlsConfig()->alt_hold_deadband);
            bstWrite8(rcControlsConfig()->alt_hold_fast_change);
            bstWrite8(rcControlsConfig()->deadband);
            bstWrite8(rcControlsConfig()->yaw_deadband);
            break;
        case BST_FC_FILTERS:
            bstWrite16(constrain(gyroConfig()->gyro_lpf, 0, 1)); // Extra safety to prevent OSD setting corrupt values
            break;
        default:
            // we do not know how to handle the (valid) message, indicate error BST
            return false;
    }
    return true;
}

static bool bstSlaveProcessWriteCommand(uint8_t bstWriteCommand)
{
    uint32_t i;
    uint16_t tmp;

    bool ret = BST_PASSED;
    switch (bstWriteCommand) {
        case BST_SELECT_SETTING:
            if (!ARMING_FLAG(ARMED)) {
                changePidProfile(bstRead8());
            }
            break;
        case BST_SET_LOOP_TIME:
            bstRead16();
            break;
        case BST_SET_PID:
            for (i = 0; i < PID_ITEM_COUNT; i++) {
                currentPidProfile->pid[i].P = bstRead8();
                currentPidProfile->pid[i].I = bstRead8();
                currentPidProfile->pid[i].D = bstRead8();
            }
            break;
        case BST_SET_MODE_RANGE:
            i = bstRead8();
            if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
                i = bstRead8();
                const box_t *box = findBoxByPermenantId(i);
                if (box) {
                    mac->modeId = box->boxId;
                    mac->auxChannelIndex = bstRead8();
                    mac->range.startStep = bstRead8();
                    mac->range.endStep = bstRead8();

                    useRcControlsConfig(currentPidProfile);
                } else {
                    ret = BST_FAILED;
                }
            } else {
                ret = BST_FAILED;
            }
            break;
        case BST_SET_MISC:
            tmp = bstRead16();
            if (tmp < 1600 && tmp > 1400)
                rxConfigMutable()->midrc = tmp;

            motorConfigMutable()->minthrottle = bstRead16();
            motorConfigMutable()->maxthrottle = bstRead16();
            motorConfigMutable()->mincommand = bstRead16();

            failsafeConfigMutable()->failsafe_throttle = bstRead16();

#ifdef USE_GPS
            gpsConfigMutable()->provider = bstRead8(); // gps_type
            bstRead8(); // gps_baudrate
            gpsConfigMutable()->sbasMode = bstRead8(); // gps_ubx_sbas
#else
            bstRead8(); // gps_type
            bstRead8(); // gps_baudrate
            bstRead8(); // gps_ubx_sbas
#endif
            bstRead8(); // legacy - was multiwiiCurrentMeterOutput
            rxConfigMutable()->rssi_channel = bstRead8();
            bstRead8();

            compassConfigMutable()->mag_declination = bstRead16() * 10;

            voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = bstRead8();  // actual vbatscale as intended
            batteryConfigMutable()->vbatmincellvoltage = bstRead8();  // vbatlevel_warn1 in MWC2.3 GUI
            batteryConfigMutable()->vbatmaxcellvoltage = bstRead8();  // vbatlevel_warn2 in MWC2.3 GUI
            batteryConfigMutable()->vbatwarningcellvoltage = bstRead8();  // vbatlevel when buzzer starts to alert
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
                return ret;
            }
            writeEEPROM();
            readEEPROM();
            break;
        case BST_SET_FEATURE:
            featureClearAll();
            featureSet(bstRead32()); // features bitmap
#ifdef SERIALRX_UART
            if (featureConfigured(FEATURE_RX_SERIAL)) {
                serialConfigMutable()->portConfigs[SERIALRX_UART].functionMask = FUNCTION_RX_SERIAL;
            } else {
                serialConfigMutable()->portConfigs[SERIALRX_UART].functionMask = FUNCTION_NONE;
            }
#endif
            break;
        case BST_SET_RX_CONFIG:
           rxConfigMutable()->serialrx_provider = bstRead8();
           rxConfigMutable()->maxcheck = bstRead16();
           rxConfigMutable()->midrc = bstRead16();
           rxConfigMutable()->mincheck = bstRead16();
           rxConfigMutable()->spektrum_sat_bind = bstRead8();
           if (bstReadDataSize() > 8) {
               rxConfigMutable()->rx_min_usec = bstRead16();
               rxConfigMutable()->rx_max_usec = bstRead16();
           }
           break;
        case BST_SET_RX_MAP:
            for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
                rxConfigMutable()->rcmap[i] = bstRead8();
            }
            break;

#ifdef USE_LED_STRIP
        case BST_SET_LED_COLORS:
           //for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
           {
               i = bstRead8();
               hsvColor_t *color = &ledStripConfigMutable()->colors[i];
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
               ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[i];
               *ledConfig = bstRead32();
               reevaluateLedConfig();
           }
           break;
#endif
        case BST_REBOOT:
            isRebootScheduled = true;
            break;
        case BST_DISARM:
            if (ARMING_FLAG(ARMED)) {
                    disarm();
            }
            setArmingDisabled(ARMING_DISABLED_BST);
            break;
        case BST_ENABLE_ARM:
            unsetArmingDisabled(ARMING_DISABLED_BST);
            break;
        case BST_SET_DEADBAND:
            rcControlsConfigMutable()->alt_hold_deadband = bstRead8();
            rcControlsConfigMutable()->alt_hold_fast_change = bstRead8();
            rcControlsConfigMutable()->deadband = bstRead8();
            rcControlsConfigMutable()->yaw_deadband = bstRead8();
            break;
        case BST_SET_FC_FILTERS:
            gyroConfigMutable()->gyro_lpf = bstRead16();
            break;

        default:
            // we do not know how to handle the (valid) message, indicate error BST
            ret = BST_FAILED;
    }
    bstWrite8(ret);

    if (ret == BST_FAILED)
        return false;

    return true;
}

static bool bstSlaveUSBCommandFeedback(/*uint8_t bstFeedback*/)
{
    bstWrite8(BST_USB_DEVICE_INFO_FRAME);                        //Sub CPU Device Info FRAME
    bstWrite8(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4);
    bstWrite8(0x00);
    bstWrite8(0x00);
    bstWrite8(0x00);
    bstWrite8(FC_VERSION_MAJOR);                                //Firmware ID
    bstWrite8(FC_VERSION_MINOR);                                //Firmware ID
    bstWrite8(0x00);
    bstWrite8(0x00);
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
    if (bstCurrentAddress() == I2C_ADDR_CLEANFLIGHT_FC) {
        if (bstReadCRC() == CRC8 && bstRead8()==BST_USB_COMMANDS) {
            uint8_t i;
            writeBufferPointer = 1;
            cleanflight_data_ready = false;
            for (i = 0; i < BST_BUFFER_SIZE; i++) {
                writeData[i] = 0;
            }
            switch (bstRead8()) {
                case BST_USB_DEVICE_INFO_REQUEST:
                    bstRead8();
                    if (bstSlaveUSBCommandFeedback(/*bstRead8()*/))
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
    } else if (bstCurrentAddress() == 0x00) {
        if (bstReadCRC() == CRC8 && bstRead8()==BST_GENERAL_HEARTBEAT) {
            resetBstTimer = micros();
            needResetCheck = true;
        }
    }
}

static void resetBstChecker(timeUs_t currentTimeUs)
{
    if (needResetCheck) {
        if (currentTimeUs >= (resetBstTimer + BST_RESET_TIME))
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

void taskBstMasterProcess(timeUs_t currentTimeUs)
{
    if (coreProReady) {
        if (currentTimeUs >= next02hzUpdateAt_1 && !bstWriteBusy()) {
            writeFCModeToBST();
            next02hzUpdateAt_1 = currentTimeUs + UPDATE_AT_02HZ;
        }
        if (currentTimeUs >= next20hzUpdateAt_1 && !bstWriteBusy()) {
            if (sendCounter == 0)
                writeRCChannelToBST();
            else if (sendCounter == 1)
                writeRollPitchYawToBST();
            sendCounter++;
            if (sendCounter > 1)
                sendCounter = 0;
            next20hzUpdateAt_1 = currentTimeUs + UPDATE_AT_20HZ;
        }
#ifdef USE_GPS
        if (sensors(SENSOR_GPS) && !bstWriteBusy())
            writeGpsPositionPrameToBST();
#endif

    }
    bstMasterWriteLoop();
    if (isRebootScheduled) {
        stopMotors();
        systemReset();
    }
    resetBstChecker(currentTimeUs);
}

/*************************************************************************************************/
static uint8_t masterWriteBufferPointer;
static uint8_t masterWriteData[BST_BUFFER_SIZE];

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

/*************************************************************************************************/
#define PUBLIC_ADDRESS            0x00

#ifdef USE_GPS
static void bstMasterWrite32(uint32_t data)
{
    bstMasterWrite16((uint8_t)(data >> 16));
    bstMasterWrite16((uint8_t)(data >> 0));
}

static int32_t lat = 0;
static int32_t lon = 0;
static uint16_t alt = 0;
static uint8_t numOfSat = 0;
#endif

#ifdef USE_GPS
bool writeGpsPositionPrameToBST(void)
{
    if ((lat != gpsSol.llh.lat) || (lon != gpsSol.llh.lon) || (alt != gpsSol.llh.alt) || (numOfSat != gpsSol.numSat)) {
        lat = gpsSol.llh.lat;
        lon = gpsSol.llh.lon;
        alt = gpsSol.llh.alt;
        numOfSat = gpsSol.numSat;
        uint16_t speed = (gpsSol.groundSpeed * 9 / 25);
        uint16_t gpsHeading = 0;
        uint16_t altitude = 0;
        gpsHeading = gpsSol.groundCourse * 10;
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
#endif

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
    for (i = 0; i < (USABLE_TIMER_CHANNEL_COUNT-1); i++) {
        bstMasterWrite16(rcData[i]);
    }

    return bstMasterWrite(masterWriteData);
}

bool writeFCModeToBST(void)
{
    uint8_t tmp = 0;
    tmp = IS_ENABLED(ARMING_FLAG(ARMED)) |
           IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << 1 |
           IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << 2 |
           IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << 3 |
           IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << 4 |
           IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << 5 |
           IS_ENABLED(FLIGHT_MODE(RANGEFINDER_MODE)) << 6 |
           IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << 7;

    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(CLEANFLIGHT_MODE_FRAME_ID);
    bstMasterWrite8(tmp);
    bstMasterWrite8(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4);

    return bstMasterWrite(masterWriteData);
}
/*************************************************************************************************/
