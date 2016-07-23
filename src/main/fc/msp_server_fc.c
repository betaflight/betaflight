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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build/build_config.h"
#include "build/debug.h"
#include <platform.h>

#include "common/axis.h"
#include "common/utils.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"
#include "config/profile.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/gpio.h"
#include "drivers/pwm_mapping.h"
#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/sdcard.h"
#include "drivers/buf_writer.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "fc/rate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"
#include "fc/fc_tasks.h"
#include "fc/runtime_config.h"
#include "fc/config.h"

#include "scheduler/scheduler.h"

#include "io/motor_and_servo.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/transponder_ir.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/serial_4way.h"

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
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "fc/cleanflight_fc.h"

#include "build/version.h"
#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "fc/msp_server_fc.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "io/serial_4way.h"
#endif

extern uint16_t cycleTime; // FIXME dependency on mw.c
extern uint16_t rssi; // FIXME dependency on mw.c
extern void resetPidProfile(pidProfile_t *pidProfile);

static const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

typedef struct box_e {
    const char *boxName;            // GUI-readable box name
    const uint8_t boxId;            // see boxId_e (it is equal to table index, may be optimized)
    const uint8_t permanentId;      // ID for MSP protocol
} box_t;

static const box_t boxes[CHECKBOX_ITEM_COUNT] = {
    { "ARM",       BOXARM,        0 },
    { "ANGLE",     BOXANGLE,      1 },
    { "HORIZON",   BOXHORIZON,    2 },
    { "BARO",      BOXBARO,       3 },
  //{ "VARIO",     BOXVARIO,      4 },
    { "MAG",       BOXMAG,        5 },
    { "HEADFREE",  BOXHEADFREE,   6 },
    { "HEADADJ",   BOXHEADADJ,    7 },
    { "CAMSTAB",   BOXCAMSTAB,    8 },
    { "CAMTRIG",   BOXCAMTRIG,    9 },
    { "GPS HOME",  BOXGPSHOME,   10 },
    { "GPS HOLD",  BOXGPSHOLD,   11 },
    { "PASSTHRU",  BOXPASSTHRU,  12 },
    { "BEEPER",    BOXBEEPERON,  13 },
    { "LEDMAX",    BOXLEDMAX,    14 },
    { "LEDLOW",    BOXLEDLOW,    15 },
    { "LLIGHTS",   BOXLLIGHTS,   16 },
    { "CALIB",     BOXCALIB,     17 },
    { "GOVERNOR",  BOXGOV,       18 },
    { "OSD SW",    BOXOSD,       19 },
    { "TELEMETRY", BOXTELEMETRY, 20 },
    { "GTUNE",     BOXGTUNE,     21 },
    { "SONAR",     BOXSONAR,     22 },
    { "SERVO1",    BOXSERVO1,    23 },
    { "SERVO2",    BOXSERVO2,    24 },
    { "SERVO3",    BOXSERVO3,    25 },
    { "BLACKBOX",  BOXBLACKBOX,  26 },
    { "FAILSAFE",  BOXFAILSAFE,  27 },
    { "AIR MODE",  BOXAIRMODE,   28 },
};

// mask of enabled IDs, calculated on start based on enabled features. boxId_e is used as bit index.
static uint32_t activeBoxIds;

// from mixer.c
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

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

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4,
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTTED   = 1,
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_BIT_READY        = 1,
    MSP_FLASHFS_BIT_SUPPORTED    = 2,
} mspFlashfsFlags_e;

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
void msp4WayIfFn(mspPort_t *msp)
{
    waitForSerialPortToFinishTransmitting(msp->port);
    // esc4wayInit() was called in msp command
    // modal switch to esc4way, will return only after 4way exit command
    // port parameters are shared with esc4way, no need to close/reopen it
    esc4wayProcess(msp->port);
    // continue processing
}
#endif

void mspRebootFn(mspPort_t *msp)
{
    waitForSerialPortToFinishTransmitting(msp->port);  // TODO - postpone reboot, allow all modules to react
    stopMotors();
    handleOneshotFeatureChangeOnRestart();
    systemReset();

    // control should never return here.
    while(1) ;
}

static const box_t *findBoxByBoxId(uint8_t boxId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->boxId == boxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permanentId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->permanentId == permanentId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    for (int i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        if(!(activeBoxIds & (1 << i)))
            continue;                          // box is not enabled
        const box_t *box = findBoxByBoxId(i);
        sbufWriteString(dst, box->boxName);
        sbufWriteU8(dst, ';');                 // TODO - sbufWriteChar?
    }
}

static void serializeBoxIdsReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    for (int i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        if(!(activeBoxIds & (1 << i)))
            continue;
        const box_t *box = findBoxByBoxId(i);
        sbufWriteU8(dst, box->permanentId);
    }
}

static void initActiveBoxIds(void)
{
    uint32_t ena = 0;

    ena |= 1 << BOXARM;

    if (sensors(SENSOR_ACC)) {
        ena |= 1 << BOXANGLE;
        ena |= 1 << BOXHORIZON;
    }

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        ena |= 1 << BOXBARO;
    }
#endif

    ena |= 1 << BOXAIRMODE;

    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        ena |= 1 << BOXMAG;
        ena |= 1 << BOXHEADFREE;
        ena |= 1 << BOXHEADADJ;
    }

    if (feature(FEATURE_SERVO_TILT))
        ena |= 1 << BOXCAMSTAB;

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        ena |= 1 << BOXGPSHOME;
        ena |= 1 << BOXGPSHOLD;
    }
#endif

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING
        || mixerConfig()->mixerMode == MIXER_AIRPLANE
        || mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE)
        ena |= 1 << BOXPASSTHRU;

    ena |= 1 << BOXBEEPERON;

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        ena |= 1 << BOXLEDLOW;
    }
#endif

    if (feature(FEATURE_INFLIGHT_ACC_CAL))
        ena |= 1 << BOXCALIB;

    ena |= 1 << BOXOSD;

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY) && telemetryConfig()->telemetry_switch)
        ena |= 1 << BOXTELEMETRY;
#endif

    if (feature(FEATURE_SONAR)){
        ena |= 1 << BOXSONAR;
    }

#ifdef USE_SERVOS
    if (mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE) {
        ena |= 1 << BOXSERVO1;
        ena |= 1 << BOXSERVO2;
        ena |= 1 << BOXSERVO3;
    }
#endif

#ifdef BLACKBOX
    if (feature(FEATURE_BLACKBOX)){
        ena |= 1 << BOXBLACKBOX;
    }
#endif

    if (feature(FEATURE_FAILSAFE)){
        ena |= 1 << BOXFAILSAFE;
    }

#ifdef GTUNE
    ena |= 1 << BOXGTUNE;
#endif

    // check that all enabled IDs are in boxes array (check is skipped when using findBoxBy<id>() functions
    for(boxId_e boxId = 0;  boxId < CHECKBOX_ITEM_COUNT; boxId++)
        if((ena & (1 << boxId))
           && findBoxByBoxId(boxId) == NULL)
            ena &= ~ (1 << boxId);                // this should not happen, but handle it gracefully
    activeBoxIds = ena;
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

static uint32_t packFlightModeFlags(void)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.

    uint32_t boxEnabledMask = 0;      // enabled BOXes, bits indexed by boxId_e

    // enable BOXes dependent on FLIGHT_MODE, use mapping table
    static const int8_t flightMode_boxId_map[] = FLIGHT_MODE_BOXID_MAP_INITIALIZER;
    flightModeFlags_e flightModeCopyMask = ~(GTUNE_MODE);  // BOXGTUNE is based on rcMode, not flight mode
    for(unsigned i = 0; i < ARRAYLEN(flightMode_boxId_map); i++) {
        if(flightMode_boxId_map[i] == -1)
            continue;                 // boxId_e does not exist for this FLIGHT_MODE
        if((flightModeCopyMask & (1 << i)) == 0)
            continue;                 // this flightmode is not copied
        if(FLIGHT_MODE(1 << i))
            boxEnabledMask |= 1 << flightMode_boxId_map[i];
    }

    // enable BOXes dependent on rcMode bits, indexes are the same.
    // only subset of BOXes depend on rcMode, use mask to mark them
#define BM(x) (1 << (x))
    const uint32_t rcModeCopyMask = BM(BOXHEADADJ) | BM(BOXCAMSTAB) | BM(BOXCAMTRIG) | BM(BOXBEEPERON)
        | BM(BOXLEDMAX) | BM(BOXLEDLOW) | BM(BOXLLIGHTS) | BM(BOXCALIB) | BM(BOXGOV) | BM(BOXOSD)
        | BM(BOXTELEMETRY) | BM(BOXGTUNE) | BM(BOXBLACKBOX)  | BM(BOXAIRMODE) ;
    for(unsigned i = 0; i < sizeof(rcModeCopyMask) * 8; i++) {
        if((rcModeCopyMask & BM(i)) == 0)
            continue;
        if(rcModeIsActive(i))
            boxEnabledMask |= 1 << i;
    }
#undef BM
    // copy ARM state
    if(ARMING_FLAG(ARMED))
        boxEnabledMask |= 1 << BOXARM;

    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    uint32_t mspBoxEnabledMask = 0;
    unsigned mspBoxIdx = 0;           // index of active boxId (matches sent permanentId and boxNames)
    for (boxId_e boxId = 0; boxId < CHECKBOX_ITEM_COUNT; boxId++) {
        if((activeBoxIds & (1 << boxId)) == 0)
            continue;                 // this box is not active
        if (boxEnabledMask & (1 << boxId))
            mspBoxEnabledMask |= 1 << mspBoxIdx;      // box is enabled
        mspBoxIdx++;                  // next output bit ID
    }
    return mspBoxEnabledMask;
}

static void serializeSDCardSummaryReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
#ifdef USE_SDCARD
    uint8_t flags = MSP_SDCARD_FLAG_SUPPORTTED;
    uint8_t state;

    sbufWriteU8(dst, flags);

    // Merge the card and filesystem states together
    if (!sdcard_isInserted()) {
        state = MSP_SDCARD_STATE_NOT_PRESENT;
    } else if (!sdcard_isFunctional()) {
        state = MSP_SDCARD_STATE_FATAL;
    } else {
        switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                if (sdcard_isInitialized()) {
                    state = MSP_SDCARD_STATE_FS_INIT;
                } else {
                    state = MSP_SDCARD_STATE_CARD_INIT;
                }
                break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
        }
    }

    sbufWriteU8(dst, state);
    sbufWriteU8(dst, afatfs_getLastError());
    // Write free space and total space in kilobytes
    sbufWriteU32(dst, afatfs_getContiguousFreeSpace() / 1024);
    sbufWriteU32(dst, sdcard_getMetadata()->numBlocks / 2); // Block size is half a kilobyte
#else
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

static void serializeDataflashSummaryReply(mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    uint8_t flags = (flashfsIsReady() ? MSP_FLASHFS_BIT_READY : 0) | MSP_FLASHFS_BIT_SUPPORTED;

    sbufWriteU8(dst, flags);
    sbufWriteU32(dst, geometry->sectors);
    sbufWriteU32(dst, geometry->totalSize);
    sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    sbufWriteU8(dst, 0); // FlashFS is neither ready nor supported
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(mspPacket_t *reply, uint32_t address, int size)
{
    sbuf_t *dst = &reply->buf;
    sbufWriteU32(dst, address);
    size = MIN(size, sbufBytesRemaining(dst));    // limit reply to available buffer space
    // bytesRead will be lower than that requested if we reach end of volume
    int bytesRead = flashfsReadAbs(address, sbufPtr(dst), size);
    sbufAdvance(dst, bytesRead);
}
#endif

// return positive for ACK, negative on error, zero for no reply
int mspServerCommandHandler(mspPacket_t *cmd, mspPacket_t *reply)
{
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;

    int len = sbufBytesRemaining(src);

    switch (cmd->cmd) {
        case MSP_API_VERSION:
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);

            sbufWriteU8(dst, API_VERSION_MAJOR);
            sbufWriteU8(dst, API_VERSION_MINOR);
            break;

        case MSP_FC_VARIANT:
            sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
            break;

        case MSP_FC_VERSION:
            sbufWriteU8(dst, FC_VERSION_MAJOR);
            sbufWriteU8(dst, FC_VERSION_MINOR);
            sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
            break;

        case MSP_BOARD_INFO:
            sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
            sbufWriteU16(dst, hardwareRevision);
#else
            sbufWriteU16(dst, 0); // No hardware revision available.
#endif
            break;

        case MSP_BUILD_INFO:
            sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
            sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
            sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
            break;

            // DEPRECATED - Use MSP_API_VERSION
        case MSP_IDENT:
            sbufWriteU8(dst, MW_VERSION);
            sbufWriteU8(dst, mixerConfig()->mixerMode);
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU32(dst, CAP_DYNBALANCE); // "capability"
            break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
            sbufWriteU16(dst, cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
            sbufWriteU32(dst, packFlightModeFlags());
            sbufWriteU8(dst, getCurrentProfile());
            if(cmd->cmd == MSP_STATUS_EX) {
                sbufWriteU16(dst, averageSystemLoadPercent);
            }
            break;

        case MSP_RAW_IMU: {
            // Hack scale due to choice of units for sensor data in multiwii
            unsigned scale_shift = (acc.acc_1G > 1024) ? 3 : 0;

            for (unsigned i = 0; i < 3; i++)
                sbufWriteU16(dst, accSmooth[i] >> scale_shift);
            for (unsigned i = 0; i < 3; i++)
                sbufWriteU16(dst, gyroADC[i]);
            for (unsigned i = 0; i < 3; i++)
                sbufWriteU16(dst, magADC[i]);
            break;
        }

#ifdef USE_SERVOS
        case MSP_SERVO:
            sbufWriteData(dst, &servo, MAX_SUPPORTED_SERVOS * 2);
            break;

        case MSP_SERVO_CONFIGURATIONS:
            for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
                sbufWriteU16(dst, servoProfile()->servoConf[i].min);
                sbufWriteU16(dst, servoProfile()->servoConf[i].max);
                sbufWriteU16(dst, servoProfile()->servoConf[i].middle);
                sbufWriteU8(dst, servoProfile()->servoConf[i].rate);
                sbufWriteU8(dst, servoProfile()->servoConf[i].angleAtMin);
                sbufWriteU8(dst, servoProfile()->servoConf[i].angleAtMax);
                sbufWriteU8(dst, servoProfile()->servoConf[i].forwardFromChannel);
                sbufWriteU32(dst, servoProfile()->servoConf[i].reversedSources);
            }
            break;

        case MSP_SERVO_MIX_RULES:
            for (unsigned i = 0; i < MAX_SERVO_RULES; i++) {
                sbufWriteU8(dst, customServoMixer(i)->targetChannel);
                sbufWriteU8(dst, customServoMixer(i)->inputSource);
                sbufWriteU8(dst, customServoMixer(i)->rate);
                sbufWriteU8(dst, customServoMixer(i)->speed);
                sbufWriteU8(dst, customServoMixer(i)->min);
                sbufWriteU8(dst, customServoMixer(i)->max);
                sbufWriteU8(dst, customServoMixer(i)->box);
            }
            break;
#endif

        case MSP_MOTOR:
            for (unsigned i = 0; i < 8; i++) {
                sbufWriteU16(dst, i < MAX_SUPPORTED_MOTORS ? motor[i] : 0);
            }
            break;

        case MSP_RC:
            for (int i = 0; i < rxRuntimeConfig.channelCount; i++)
                sbufWriteU16(dst, rcData[i]);
            break;

        case MSP_ATTITUDE:
            sbufWriteU16(dst, attitude.values.roll);
            sbufWriteU16(dst, attitude.values.pitch);
            sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
            break;

        case MSP_ALTITUDE:
#if defined(BARO) || defined(SONAR)
            sbufWriteU32(dst, altitudeHoldGetEstimatedAltitude());
            sbufWriteU16(dst, vario);
#else
            sbufWriteU32(dst, 0);
            sbufWriteU16(dst, 0);
#endif
            break;

        case MSP_SONAR_ALTITUDE:
#if defined(SONAR)
            sbufWriteU32(dst, sonarGetLatestAltitude());
#else
            sbufWriteU32(dst, 0);
#endif
            break;

        case MSP_ANALOG:
            sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
            sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            sbufWriteU16(dst, rssi);
            if(batteryConfig()->multiwiiCurrentMeterOutput) {
                sbufWriteU16(dst, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
            } else
                sbufWriteU16(dst, (int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
            break;

        case MSP_ARMING_CONFIG:
            sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
            sbufWriteU8(dst, armingConfig()->disarm_kill_switch);
            break;

        case MSP_LOOP_TIME:
            sbufWriteU16(dst, imuConfig()->looptime);
            break;

        case MSP_RC_TUNING:
            sbufWriteU8(dst, currentControlRateProfile->rcRate8);
            sbufWriteU8(dst, currentControlRateProfile->rcExpo8);
            for (unsigned i = 0 ; i < 3; i++) {
                sbufWriteU8(dst, currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
            }
            sbufWriteU8(dst, currentControlRateProfile->dynThrPID);
            sbufWriteU8(dst, currentControlRateProfile->thrMid8);
            sbufWriteU8(dst, currentControlRateProfile->thrExpo8);
            sbufWriteU16(dst, currentControlRateProfile->tpa_breakpoint);
            sbufWriteU8(dst, currentControlRateProfile->rcYawExpo8);
            break;

        case MSP_PID:
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                sbufWriteU8(dst, pidProfile()->P8[i]);
                sbufWriteU8(dst, pidProfile()->I8[i]);
                sbufWriteU8(dst, pidProfile()->D8[i]);
            }
            break;

        case MSP_PIDNAMES:
            sbufWriteString(dst, pidnames);
            break;

        case MSP_PID_CONTROLLER:
            sbufWriteU8(dst, pidProfile()->pidController);
            break;

        case MSP_MODE_RANGES:
            for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
                modeActivationCondition_t *mac = &modeActivationProfile()->modeActivationConditions[i];
                const box_t *box = findBoxByBoxId(mac->modeId);
                sbufWriteU8(dst, box->permanentId);
                sbufWriteU8(dst, mac->auxChannelIndex);
                sbufWriteU8(dst, mac->range.startStep);
                sbufWriteU8(dst, mac->range.endStep);
            }
            break;

        case MSP_ADJUSTMENT_RANGES:
            for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
                adjustmentRange_t *adjRange = &adjustmentProfile()->adjustmentRanges[i];
                sbufWriteU8(dst, adjRange->adjustmentIndex);
                sbufWriteU8(dst, adjRange->auxChannelIndex);
                sbufWriteU8(dst, adjRange->range.startStep);
                sbufWriteU8(dst, adjRange->range.endStep);
                sbufWriteU8(dst, adjRange->adjustmentFunction);
                sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
            }
            break;

        case MSP_BOXNAMES:
            serializeBoxNamesReply(reply);
            break;

        case MSP_BOXIDS:
            serializeBoxIdsReply(reply);
            break;

        case MSP_MISC:
            sbufWriteU16(dst, rxConfig()->midrc);

            sbufWriteU16(dst, motorAndServoConfig()->minthrottle);
            sbufWriteU16(dst, motorAndServoConfig()->maxthrottle);
            sbufWriteU16(dst, motorAndServoConfig()->mincommand);

            sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);

#ifdef GPS
            sbufWriteU8(dst, gpsConfig()->provider); // gps_type
            sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            sbufWriteU8(dst, gpsConfig()->sbasMode); // gps_ubx_sbas
#else
            sbufWriteU8(dst, 0); // gps_type
            sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            sbufWriteU8(dst, 0); // gps_ubx_sbas
#endif
            sbufWriteU8(dst, batteryConfig()->multiwiiCurrentMeterOutput);
            sbufWriteU8(dst, rxConfig()->rssi_channel);
            sbufWriteU8(dst, 0);

            sbufWriteU16(dst, compassConfig()->mag_declination);

            sbufWriteU8(dst, batteryConfig()->vbatscale);
            sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
            sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
            sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
            break;

        case MSP_MOTOR_PINS:
            // FIXME This is hardcoded and should not be.
            for (int i = 0; i < 8; i++)
                sbufWriteU8(dst, i + 1);
            break;

#ifdef GPS
        case MSP_RAW_GPS:
            sbufWriteU8(dst, STATE(GPS_FIX));
            sbufWriteU8(dst, GPS_numSat);
            sbufWriteU32(dst, GPS_coord[LAT]);
            sbufWriteU32(dst, GPS_coord[LON]);
            sbufWriteU16(dst, GPS_altitude);
            sbufWriteU16(dst, GPS_speed);
            sbufWriteU16(dst, GPS_ground_course);
            break;

        case MSP_COMP_GPS:
            sbufWriteU16(dst, GPS_distanceToHome);
            sbufWriteU16(dst, GPS_directionToHome);
            sbufWriteU8(dst, GPS_update & 1);
            break;

        case MSP_WP: {
            uint8_t wp_no = sbufReadU8(src);    // get the wp number
            int32_t lat = 0, lon = 0;
            if (wp_no == 0) {
                lat = GPS_home[LAT];
                lon = GPS_home[LON];
            } else if (wp_no == 16) {
                lat = GPS_hold[LAT];
                lon = GPS_hold[LON];
            }
            sbufWriteU8(dst, wp_no);
            sbufWriteU32(dst, lat);
            sbufWriteU32(dst, lon);
            sbufWriteU32(dst, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
            sbufWriteU16(dst, 0);                 // heading  will come here (deg)
            sbufWriteU16(dst, 0);                 // time to stay (ms) will come here
            sbufWriteU8(dst, 0);                  // nav flag will come here
            break;
        }

        case MSP_GPSSVINFO:
            sbufWriteU8(dst, GPS_numCh);
            for (int i = 0; i < GPS_numCh; i++){
                sbufWriteU8(dst, GPS_svinfo_chn[i]);
                sbufWriteU8(dst, GPS_svinfo_svid[i]);
                sbufWriteU8(dst, GPS_svinfo_quality[i]);
                sbufWriteU8(dst, GPS_svinfo_cno[i]);
            }
            break;
#endif

        case MSP_DEBUG:
            // output some useful QA statistics
            // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

            for (int i = 0; i < DEBUG16_VALUE_COUNT; i++)
                sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
            break;

            // Additional commands that are not compatible with MultiWii
        case MSP_ACC_TRIM:
            sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.pitch);
            sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.roll);
            break;

        case MSP_UID:
            sbufWriteU32(dst, U_ID_0);
            sbufWriteU32(dst, U_ID_1);
            sbufWriteU32(dst, U_ID_2);
            break;

        case MSP_FEATURE:
            sbufWriteU32(dst, featureMask());
            break;

        case MSP_BOARD_ALIGNMENT:
        	sbufWriteU16(dst, boardAlignment()->rollDegrees);
        	sbufWriteU16(dst, boardAlignment()->pitchDegrees);
        	sbufWriteU16(dst, boardAlignment()->yawDegrees);
            break;

        case MSP_VOLTAGE_METER_CONFIG:
            sbufWriteU8(dst, batteryConfig()->vbatscale);
            sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
            sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
            sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
            break;

        case MSP_CURRENT_METER_CONFIG:
            sbufWriteU16(dst, batteryConfig()->currentMeterScale);
            sbufWriteU16(dst, batteryConfig()->currentMeterOffset);
            sbufWriteU8(dst, batteryConfig()->currentMeterType);
            sbufWriteU16(dst, batteryConfig()->batteryCapacity);
            break;

        case MSP_MIXER:
            sbufWriteU8(dst, mixerConfig()->mixerMode);
            break;

        case MSP_RX_CONFIG:
            sbufWriteU8(dst, rxConfig()->serialrx_provider);
            sbufWriteU16(dst, rxConfig()->maxcheck);
            sbufWriteU16(dst, rxConfig()->midrc);
            sbufWriteU16(dst, rxConfig()->mincheck);
            sbufWriteU8(dst, rxConfig()->spektrum_sat_bind);
            sbufWriteU16(dst, rxConfig()->rx_min_usec);
            sbufWriteU16(dst, rxConfig()->rx_max_usec);
            break;

        case MSP_FAILSAFE_CONFIG:
            sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
            sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
            sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
            sbufWriteU8(dst, failsafeConfig()->failsafe_kill_switch);
            sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
            sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
            break;

        case MSP_RXFAIL_CONFIG:
            for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
                sbufWriteU8(dst, failsafeChannelConfigs(i)->mode);
                sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(failsafeChannelConfigs(i)->step));
            }
            break;

        case MSP_RSSI_CONFIG:
            sbufWriteU8(dst, rxConfig()->rssi_channel);
            break;

        case MSP_RX_MAP:
            for (int i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++)
                sbufWriteU8(dst, rxConfig()->rcmap[i]);
            break;

        case MSP_BF_CONFIG:
            sbufWriteU8(dst, mixerConfig()->mixerMode);

            sbufWriteU32(dst, featureMask());

            sbufWriteU8(dst, rxConfig()->serialrx_provider);

            sbufWriteU16(dst, boardAlignment()->rollDegrees);
            sbufWriteU16(dst, boardAlignment()->pitchDegrees);
            sbufWriteU16(dst, boardAlignment()->yawDegrees);

            sbufWriteU16(dst, batteryConfig()->currentMeterScale);
            sbufWriteU16(dst, batteryConfig()->currentMeterOffset);
            break;

        case MSP_CF_SERIAL_CONFIG:
            for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
                if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                    continue;
                };
                sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
                sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
                for (int baudRateIndex = 0; baudRateIndex < FUNCTION_BAUD_RATE_COUNT; baudRateIndex++) {
                	sbufWriteU8(dst, serialConfig()->portConfigs[i].baudRates[baudRateIndex]);
                }
            }
            break;

#ifdef LED_STRIP
        case MSP_LED_COLORS:
            for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
                hsvColor_t *color = colors(i);
                sbufWriteU16(dst, color->h);
                sbufWriteU8(dst, color->s);
                sbufWriteU8(dst, color->v);
            }
            break;

        case MSP_LED_STRIP_CONFIG:
            for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
                ledConfig_t *ledConfig = ledConfigs(i);
                sbufWriteU32(dst, *ledConfig);
            }
            break;

        case MSP_LED_STRIP_MODECOLOR:
            for (int i = 0; i < LED_MODE_COUNT; i++) {
                for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                    sbufWriteU8(dst, i);
                    sbufWriteU8(dst, j);
                    sbufWriteU8(dst, modeColors(i)->color[j]);
                }
            }
            for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
                sbufWriteU8(dst, LED_MODE_COUNT);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, specialColors_System.color[j]);
            }
            break;
#endif

        case MSP_DATAFLASH_SUMMARY:
            serializeDataflashSummaryReply(reply);
            break;

#ifdef USE_FLASHFS
        case MSP_DATAFLASH_READ: {
            uint32_t readAddress = sbufReadU32(src);

            serializeDataflashReadReply(reply, readAddress, 128);
            break;
        }
#endif

        case MSP_BLACKBOX_CONFIG:

#ifdef BLACKBOX
            sbufWriteU8(dst, 1); //Blackbox supported
            sbufWriteU8(dst, blackboxConfig()->device);
            sbufWriteU8(dst, blackboxConfig()->rate_num);
            sbufWriteU8(dst, blackboxConfig()->rate_denom);
#else
            sbufWriteU8(dst, 0); // Blackbox not supported
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
#endif
            break;

        case MSP_SDCARD_SUMMARY:
            serializeSDCardSummaryReply(reply);
            break;

        case MSP_TRANSPONDER_CONFIG:
#ifdef TRANSPONDER
            sbufWriteU8(dst, 1); //Transponder supported
            sbufWriteData(dst, transponderConfig()->data, sizeof(transponderConfig()->data));
#else
            sbufWriteU8(dst, 0); // Transponder not supported
#endif
            break;

        case MSP_BF_BUILD_INFO:
            sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            sbufWriteU32(dst, 0); // future exp
            sbufWriteU32(dst, 0); // future exp
            break;

        case MSP_3D:
            sbufWriteU16(dst, motor3DConfig()->deadband3d_low);
            sbufWriteU16(dst, motor3DConfig()->deadband3d_high);
            sbufWriteU16(dst, motor3DConfig()->neutral3d);
            break;

        case MSP_RC_DEADBAND:
            sbufWriteU8(dst, rcControlsConfig()->deadband);
            sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
            sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
            sbufWriteU16(dst, rcControlsConfig()->deadband3d_throttle);
            break;

        case MSP_SENSOR_ALIGNMENT:
            sbufWriteU8(dst, sensorAlignmentConfig()->gyro_align);
            sbufWriteU8(dst, sensorAlignmentConfig()->acc_align);
            sbufWriteU8(dst, sensorAlignmentConfig()->mag_align);
            break;

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
        case MSP_SET_4WAY_IF:
            // initialize 4way ESC interface, return number of ESCs available
            sbufWriteU8(dst, esc4wayInit());
            mspPostProcessFn = msp4WayIfFn;
            break;
#endif

        case MSP_SELECT_SETTING:
            if (!ARMING_FLAG(ARMED)) {
                int profile = sbufReadU8(src);
                changeProfile(profile);
            }
            break;

        case MSP_SET_HEAD:
            magHold = sbufReadU16(src);
            break;

        case MSP_SET_RAW_RC: {
            uint8_t channelCount = len / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT)
                return -1;
            uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];

            for (unsigned i = 0; i < channelCount; i++) {
                frame[i] = sbufReadU16(src);
            }

            rxMspFrameReceive(frame, channelCount);
            break;
        }

        case MSP_SET_ACC_TRIM:
            accelerometerConfig()->accelerometerTrims.values.pitch = sbufReadU16(src);
            accelerometerConfig()->accelerometerTrims.values.roll  = sbufReadU16(src);
            break;

        case MSP_SET_ARMING_CONFIG:
            armingConfig()->auto_disarm_delay = sbufReadU8(src);
            armingConfig()->disarm_kill_switch = sbufReadU8(src);
            break;

        case MSP_SET_LOOP_TIME:
            imuConfig()->looptime = sbufReadU16(src);
            break;

        case MSP_SET_PID_CONTROLLER:
            pidProfile()->pidController = sbufReadU8(src);
            pidSetController(pidProfile()->pidController);
            break;

        case MSP_SET_PID:
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                    pidProfile()->P8[i] = sbufReadU8(src);
                    pidProfile()->I8[i] = sbufReadU8(src);
                    pidProfile()->D8[i] = sbufReadU8(src);
                }
            break;

        case MSP_SET_MODE_RANGE: {
            int i = sbufReadU8(src);
            if (i >= MAX_MODE_ACTIVATION_CONDITION_COUNT)
                return -1;
            modeActivationCondition_t *mac = &modeActivationProfile()->modeActivationConditions[i];
            int permId = sbufReadU8(src);
            const box_t *box = findBoxByPermenantId(permId);
            if (box == NULL)
                return -1;
            mac->modeId = box->boxId;
            mac->auxChannelIndex = sbufReadU8(src);
            mac->range.startStep = sbufReadU8(src);
            mac->range.endStep = sbufReadU8(src);

            useRcControlsConfig(modeActivationProfile()->modeActivationConditions);
            break;
        }

        case MSP_SET_ADJUSTMENT_RANGE: {
            int aRange = sbufReadU8(src);
            if (aRange >= MAX_ADJUSTMENT_RANGE_COUNT)
                return -1;
            adjustmentRange_t *adjRange = &adjustmentProfile()->adjustmentRanges[aRange];
            int aIndex = sbufReadU8(src);
            if (aIndex > MAX_SIMULTANEOUS_ADJUSTMENT_COUNT)
                return -1;
            adjRange->adjustmentIndex = aIndex;
            adjRange->auxChannelIndex = sbufReadU8(src);
            adjRange->range.startStep = sbufReadU8(src);
            adjRange->range.endStep = sbufReadU8(src);
            adjRange->adjustmentFunction = sbufReadU8(src);
            adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            break;
        }

        case MSP_SET_RC_TUNING:
            if (len < 10)
                return -1;
            currentControlRateProfile->rcRate8 = sbufReadU8(src);
            currentControlRateProfile->rcExpo8 = sbufReadU8(src);
            for (int i = 0; i < 3; i++) {
                unsigned rate = sbufReadU8(src);
                currentControlRateProfile->rates[i] = MIN(rate, i == YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            }
            unsigned rate = sbufReadU8(src);
            currentControlRateProfile->dynThrPID = MIN(rate, CONTROL_RATE_CONFIG_TPA_MAX);
            currentControlRateProfile->thrMid8 = sbufReadU8(src);
            currentControlRateProfile->thrExpo8 = sbufReadU8(src);
            currentControlRateProfile->tpa_breakpoint = sbufReadU16(src);
            if (len < 11)
                break;
            currentControlRateProfile->rcYawExpo8 = sbufReadU8(src);
            break;

        case MSP_SET_MISC: {
            unsigned midrc = sbufReadU16(src);
            if (midrc > 1400 && midrc < 1600)
                rxConfig()->midrc = midrc;

            motorAndServoConfig()->minthrottle = sbufReadU16(src);
            motorAndServoConfig()->maxthrottle = sbufReadU16(src);
            motorAndServoConfig()->mincommand = sbufReadU16(src);

            failsafeConfig()->failsafe_throttle = sbufReadU16(src);

#ifdef GPS
            gpsConfig()->provider = sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            gpsConfig()->sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
            sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            sbufReadU8(src); // gps_ubx_sbas
#endif
            batteryConfig()->multiwiiCurrentMeterOutput = sbufReadU8(src);
            rxConfig()->rssi_channel = sbufReadU8(src);
            sbufReadU8(src);

            compassConfig()->mag_declination = sbufReadU16(src);

            batteryConfig()->vbatscale = sbufReadU8(src);           // actual vbatscale as intended
            batteryConfig()->vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
            batteryConfig()->vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
            batteryConfig()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            break;
        }

        case MSP_SET_MOTOR:
            for (int i = 0; i < 8; i++) {
                const int16_t disarmed = sbufReadU16(src);
                if (i < MAX_SUPPORTED_MOTORS) {
                    motor_disarmed[i] = disarmed;
                }
            }
            break;

        case MSP_SET_SERVO_CONFIGURATION: {
#ifdef USE_SERVOS
            if (len != 1 + sizeof(servoParam_t))
                return -1;
            unsigned i = sbufReadU8(src);
            if (i >= MAX_SUPPORTED_SERVOS)
                return -1;

            servoProfile()->servoConf[i].min = sbufReadU16(src);
            servoProfile()->servoConf[i].max = sbufReadU16(src);
            servoProfile()->servoConf[i].middle = sbufReadU16(src);
            servoProfile()->servoConf[i].rate = sbufReadU8(src);
            servoProfile()->servoConf[i].angleAtMin = sbufReadU8(src);
            servoProfile()->servoConf[i].angleAtMax = sbufReadU8(src);
            servoProfile()->servoConf[i].forwardFromChannel = sbufReadU8(src);
            servoProfile()->servoConf[i].reversedSources = sbufReadU32(src);
#endif
            break;
        }

        case MSP_SET_SERVO_MIX_RULE: {
#ifdef USE_SERVOS
            int i = sbufReadU8(src);
            if (i >= MAX_SERVO_RULES)
                return -1;

            customServoMixer(i)->targetChannel = sbufReadU8(src);
            customServoMixer(i)->inputSource = sbufReadU8(src);
            customServoMixer(i)->rate = sbufReadU8(src);
            customServoMixer(i)->speed = sbufReadU8(src);
            customServoMixer(i)->min = sbufReadU8(src);
            customServoMixer(i)->max = sbufReadU8(src);
            customServoMixer(i)->box = sbufReadU8(src);
            loadCustomServoMixer();
#endif
            break;
        }

        case MSP_SET_3D:
            motor3DConfig()->deadband3d_low = sbufReadU16(src);
            motor3DConfig()->deadband3d_high = sbufReadU16(src);
            motor3DConfig()->neutral3d = sbufReadU16(src);
            break;

        case MSP_SET_RC_DEADBAND:
            rcControlsConfig()->deadband = sbufReadU8(src);
            rcControlsConfig()->yaw_deadband = sbufReadU8(src);
            rcControlsConfig()->alt_hold_deadband = sbufReadU8(src);
            rcControlsConfig()->deadband3d_throttle = sbufReadU16(src);
            break;

        case MSP_SET_RESET_CURR_PID:
            PG_RESET_CURRENT(pidProfile);
            break;

        case MSP_SET_SENSOR_ALIGNMENT:
            sensorAlignmentConfig()->gyro_align = sbufReadU8(src);
            sensorAlignmentConfig()->acc_align = sbufReadU8(src);
            sensorAlignmentConfig()->mag_align = sbufReadU8(src);
            break;

        case MSP_RESET_CONF:
            if (!ARMING_FLAG(ARMED)) {
                resetEEPROM();
                readEEPROM();
            }
            break;

        case MSP_ACC_CALIBRATION:
            if (!ARMING_FLAG(ARMED))
                accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
            break;

        case MSP_MAG_CALIBRATION:
            if (!ARMING_FLAG(ARMED))
                ENABLE_STATE(CALIBRATE_MAG);
            break;

        case MSP_EEPROM_WRITE:
            if (ARMING_FLAG(ARMED))
                return -1;
            writeEEPROM();
            readEEPROM();
            break;

#ifdef BLACKBOX
        case MSP_SET_BLACKBOX_CONFIG:
            // Don't allow config to be updated while Blackbox is logging
            if (!blackboxMayEditConfig())
                return -1;
            blackboxConfig()->device = sbufReadU8(src);
            blackboxConfig()->rate_num = sbufReadU8(src);
            blackboxConfig()->rate_denom = sbufReadU8(src);
            break;
#endif

#ifdef TRANSPONDER
        case MSP_SET_TRANSPONDER_CONFIG:
            if (len != sizeof(transponderConfig()->data))
                return -1;
            sbufReadData(src, transponderConfig()->data, sizeof(transponderConfig()->data));
            transponderUpdateData(transponderConfig()->data);
            break;
#endif

#ifdef USE_FLASHFS
        case MSP_DATAFLASH_ERASE:
            flashfsEraseCompletely();
            break;
#endif

#ifdef GPS
        case MSP_SET_RAW_GPS:
            if (sbufReadU8(src)) {
                ENABLE_STATE(GPS_FIX);
            } else {
                DISABLE_STATE(GPS_FIX);
            }
            GPS_numSat = sbufReadU8(src);
            GPS_coord[LAT] = sbufReadU32(src);
            GPS_coord[LON] = sbufReadU32(src);
            GPS_altitude = sbufReadU16(src);
            GPS_speed = sbufReadU16(src);
            GPS_update |= 2;        // New data signalisation to GPS functions // FIXME Magic Numbers
            break;

        case MSP_SET_WP: {
            uint8_t wp_no = sbufReadU8(src);             // get the wp number
            int32_t lat = sbufReadU32(src);
            int32_t lon = sbufReadU32(src);
            int32_t alt = sbufReadU32(src);              // to set altitude (cm)
            sbufReadU16(src);                            // future: to set heading (deg)
            sbufReadU16(src);                            // future: to set time to stay (ms)
            sbufReadU8(src);                             // future: to set nav flag
            if (wp_no == 0) {
                GPS_home[LAT] = lat;
                GPS_home[LON] = lon;
                DISABLE_FLIGHT_MODE(GPS_HOME_MODE);     // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
                ENABLE_STATE(GPS_FIX_HOME);
                if (alt != 0)
                    AltHold = alt;                      // temporary implementation to test feature with apps
            } else if (wp_no == 16) {                   // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
                GPS_hold[LAT] = lat;
                GPS_hold[LON] = lon;
                if (alt != 0)
                    AltHold = alt;                      // temporary implementation to test feature with apps
                nav_mode = NAV_MODE_WP;
                GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            }
            break;
        }
#endif

        case MSP_SET_FEATURE:
            featureClearAll();
            featureSet(sbufReadU32(src)); // features bitmap
            break;

        case MSP_SET_BOARD_ALIGNMENT:
            boardAlignment()->rollDegrees = sbufReadU16(src);
            boardAlignment()->pitchDegrees = sbufReadU16(src);
            boardAlignment()->yawDegrees = sbufReadU16(src);
            break;

        case MSP_SET_VOLTAGE_METER_CONFIG:
            batteryConfig()->vbatscale = sbufReadU8(src);               // actual vbatscale as intended
            batteryConfig()->vbatmincellvoltage = sbufReadU8(src);      // vbatlevel_warn1 in MWC2.3 GUI
            batteryConfig()->vbatmaxcellvoltage = sbufReadU8(src);      // vbatlevel_warn2 in MWC2.3 GUI
            batteryConfig()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            break;

        case MSP_SET_CURRENT_METER_CONFIG:
            batteryConfig()->currentMeterScale = sbufReadU16(src);
            batteryConfig()->currentMeterOffset = sbufReadU16(src);
            batteryConfig()->currentMeterType = sbufReadU8(src);
            batteryConfig()->batteryCapacity = sbufReadU16(src);
            break;

#ifndef USE_QUAD_MIXER_ONLY
        case MSP_SET_MIXER:
            mixerConfig()->mixerMode = sbufReadU8(src);
            break;
#endif

        case MSP_SET_RX_CONFIG:
            rxConfig()->serialrx_provider = sbufReadU8(src);
            rxConfig()->maxcheck = sbufReadU16(src);
            rxConfig()->midrc = sbufReadU16(src);
            rxConfig()->mincheck = sbufReadU16(src);
            rxConfig()->spektrum_sat_bind = sbufReadU8(src);
            if (sbufBytesRemaining(src) < 2)
                break;
            rxConfig()->rx_min_usec = sbufReadU16(src);
            rxConfig()->rx_max_usec = sbufReadU16(src);
            break;

        case MSP_SET_FAILSAFE_CONFIG:
            failsafeConfig()->failsafe_delay = sbufReadU8(src);
            failsafeConfig()->failsafe_off_delay = sbufReadU8(src);
            failsafeConfig()->failsafe_throttle = sbufReadU16(src);
            failsafeConfig()->failsafe_kill_switch = sbufReadU8(src);
            failsafeConfig()->failsafe_throttle_low_delay = sbufReadU16(src);
            failsafeConfig()->failsafe_procedure = sbufReadU8(src);
            break;

        case MSP_SET_RXFAIL_CONFIG: {
            int channel =  sbufReadU8(src);
            if (channel >= MAX_SUPPORTED_RC_CHANNEL_COUNT)
                return -1;
            failsafeChannelConfigs(channel)->mode = sbufReadU8(src);
            failsafeChannelConfigs(channel)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
            break;
        }

        case MSP_SET_RSSI_CONFIG:
            rxConfig()->rssi_channel = sbufReadU8(src);
            break;

        case MSP_SET_RX_MAP:
            for (int i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
                rxConfig()->rcmap[i] = sbufReadU8(src);
            }
            break;

        case MSP_SET_BF_CONFIG:
#ifdef USE_QUAD_MIXER_ONLY
            sbufReadU8(src);                                   // mixerMode ignored
#else
            mixerConfig()->mixerMode = sbufReadU8(src);        // mixerMode
#endif

            featureClearAll();
            featureSet(sbufReadU32(src));                      // features bitmap

            rxConfig()->serialrx_provider = sbufReadU8(src);   // serialrx_type

            boardAlignment()->rollDegrees = sbufReadU16(src);  // board_align_roll
            boardAlignment()->pitchDegrees = sbufReadU16(src); // board_align_pitch
            boardAlignment()->yawDegrees = sbufReadU16(src);   // board_align_yaw

            batteryConfig()->currentMeterScale = sbufReadU16(src);
            batteryConfig()->currentMeterOffset = sbufReadU16(src);
            break;

        case MSP_SET_CF_SERIAL_CONFIG: {
            int portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (len % portConfigSize != 0)
                return -1;

            while (sbufBytesRemaining(src) >= portConfigSize) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig)
                    return -1;

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                for (int baudRateIndex = 0; baudRateIndex < FUNCTION_BAUD_RATE_COUNT; baudRateIndex++) {
                	portConfig->baudRates[baudRateIndex] = sbufReadU8(src);
                }
            }
            break;
        }

#ifdef LED_STRIP
        case MSP_SET_LED_COLORS:

            for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT && sbufBytesRemaining(src) >= 4; i++) {
                hsvColor_t *color = colors(i);

                int h = sbufReadU16(src);
                int s = sbufReadU8(src);
                int v = sbufReadU8(src);

                if (h > HSV_HUE_MAX || s > HSV_SATURATION_MAX || v > HSV_VALUE_MAX) {
                    memset(color, 0, sizeof(*color));
                    return -1;
                }

                color->h = h;
                color->s = s;
                color->v = v;
            }
            break;

        case MSP_SET_LED_STRIP_CONFIG: {
            int i = sbufReadU8(src);
            if (len != (1 + 4) || i >= LED_MAX_STRIP_LENGTH)
                return -1;

            ledConfig_t *ledConfig = ledConfigs(i);
            *ledConfig = sbufReadU32(src);

            reevaluateLedConfig();
        }
        break;

        case MSP_SET_LED_STRIP_MODECOLOR:
            while (sbufBytesRemaining(src) >= 3) {
                ledModeIndex_e modeIdx = sbufReadU8(src);
                int funIdx = sbufReadU8(src);
                int color = sbufReadU8(src);

                if (!setModeColor(modeIdx, funIdx, color))
                    return -1;
            }
            break;
#endif

        case MSP_REBOOT:
            mspPostProcessFn = mspRebootFn;
            break;

        default:
            // we do not know how to handle the message
            return -1;
    }
    return 1;     // message was handled successfully
}

void mspInit(void)
{
    initActiveBoxIds();
}
