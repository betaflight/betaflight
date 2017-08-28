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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/bitarray.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/compass/compass.h"
#include "drivers/max7456.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/hil.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/serial_4way.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

extern timeDelta_t cycleTime; // FIXME dependency on mw.c
extern uint16_t rssi; // FIXME dependency on mw.c

static const char * const flightControllerIdentifier = INAV_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

typedef struct box_e {
    const uint8_t boxId;            // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} box_t;

// FIXME remove ;'s
static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXNAVALTHOLD, "NAV ALTHOLD;", 3 },   // old BARO
    { BOXHEADINGHOLD, "HEADING HOLD;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXNAVRTH, "NAV RTH;", 10 },         // old GPS HOME
    { BOXNAVPOSHOLD, "NAV POSHOLD;", 11 },     // old GPS HOLD
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXAUTOTUNE, "AUTO TUNE;", 21 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { BOXNAVWP, "NAV WP;", 28 },
    { BOXAIRMODE, "AIR MODE;", 29 },
    { BOXHOMERESET, "HOME RESET;", 30 },
    { BOXGCSNAV, "GCS NAV;", 31 },
    //{ BOXHEADINGLOCK, "HEADING LOCK;", 32 },
    { BOXSURFACE, "SURFACE;", 33 },
    { BOXFLAPERON, "FLAPERON;", 34 },
    { BOXTURNASSIST, "TURN ASSIST;", 35 },
    { BOXNAVLAUNCH, "NAV LAUNCH;", 36 },
    { BOXAUTOTRIM, "SERVO AUTOTRIM;", 37 },
    { BOXKILLSWITCH, "KILLSWITCH;", 38 },
    { BOXCAMERA1, "CAMERA CONTROL 1;", 39 },
    { BOXCAMERA2, "CAMERA CONTROL 2;", 40 },
    { BOXCAMERA3, "CAMERA CONTROL 3;", 41 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;
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
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTTED   = 1
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_BIT_READY        = 1,
    MSP_FLASHFS_BIT_SUPPORTED    = 2
} mspFlashfsFlags_e;

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define ESC_4WAY 0xff

static uint8_t escMode;
static uint8_t escPortIndex;

static void mspFc4waySerialCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);

    if (dataSize == 0) {
        // Legacy format
        escMode = ESC_4WAY;
    } else {
        escMode = sbufReadU8(src);
        escPortIndex = sbufReadU8(src);
    }

    switch (escMode) {
    case ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

    default:
        sbufWriteU8(dst, 0);
    }
}

#endif

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    stopMotors();
    stopPwmAllMotors();

    // extra delay before reboot to give ESCs chance to reset
    delay(1000);
    systemReset();

    // control should never return here.
    while (true) ;
}

static const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permenantId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(sbuf_t *dst)
{
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (int i = 0; i < activeBoxIdCount; i++) {
        const int activeBoxId = activeBoxIds[i];
        const box_t *box = findBoxByActiveBoxId(activeBoxId);
        if (box) {
            const int len = strlen(box->boxName);
            sbufWriteData(dst, box->boxName, len);
        }
    }
}

static void initActiveBoxIds(void)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    activeBoxIdCount = 0;
    activeBoxIds[activeBoxIdCount++] = BOXARM;

    if (sensors(SENSOR_ACC)) {
        activeBoxIds[activeBoxIdCount++] = BOXANGLE;
        activeBoxIds[activeBoxIdCount++] = BOXHORIZON;

#ifdef USE_FLM_TURN_ASSIST
        activeBoxIds[activeBoxIdCount++] = BOXTURNASSIST;
#endif
    }

    if (!feature(FEATURE_AIRMODE)) {
        activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;
    }

    activeBoxIds[activeBoxIdCount++] = BOXHEADINGHOLD;

    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
        activeBoxIds[activeBoxIdCount++] = BOXHEADADJ;
    }

    if (feature(FEATURE_SERVO_TILT))
        activeBoxIds[activeBoxIdCount++] = BOXCAMSTAB;

#ifdef GPS
    if (sensors(SENSOR_BARO) || (STATE(FIXED_WING) && feature(FEATURE_GPS))) {
        activeBoxIds[activeBoxIdCount++] = BOXNAVALTHOLD;
        activeBoxIds[activeBoxIdCount++] = BOXSURFACE;
    }
    if ((feature(FEATURE_GPS) && sensors(SENSOR_MAG) && sensors(SENSOR_ACC)) || (STATE(FIXED_WING) && sensors(SENSOR_ACC) && feature(FEATURE_GPS))) {
        activeBoxIds[activeBoxIdCount++] = BOXNAVPOSHOLD;
        activeBoxIds[activeBoxIdCount++] = BOXNAVRTH;
        activeBoxIds[activeBoxIdCount++] = BOXNAVWP;
        activeBoxIds[activeBoxIdCount++] = BOXHOMERESET;
        activeBoxIds[activeBoxIdCount++] = BOXGCSNAV;
    }
#endif

    if (STATE(FIXED_WING)) {
        activeBoxIds[activeBoxIdCount++] = BOXPASSTHRU;
        activeBoxIds[activeBoxIdCount++] = BOXNAVLAUNCH;
        activeBoxIds[activeBoxIdCount++] = BOXAUTOTRIM;
#if defined(AUTOTUNE_FIXED_WING)
        activeBoxIds[activeBoxIdCount++] = BOXAUTOTUNE;
#endif
    }

#ifdef USE_SERVOS
    /*
     * FLAPERON mode active only in case of airplane and custom airplane. Activating on
     * flying wing can cause bad thing
     */
    if (STATE(FLAPERON_AVAILABLE)) {
        activeBoxIds[activeBoxIdCount++] = BOXFLAPERON;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        activeBoxIds[activeBoxIdCount++] = BOXLEDLOW;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXOSD;

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY) && telemetryConfig()->telemetry_switch)
        activeBoxIds[activeBoxIdCount++] = BOXTELEMETRY;
#endif

#ifdef BLACKBOX
    if (feature(FEATURE_BLACKBOX)){
        activeBoxIds[activeBoxIdCount++] = BOXBLACKBOX;
    }
#endif

    activeBoxIds[activeBoxIdCount++] = BOXKILLSWITCH;
    activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;

#ifdef USE_RCSPLIT
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA1;
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA2;
    activeBoxIds[activeBoxIdCount++] = BOXCAMERA3;
#endif
}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)
#define CHECK_ACTIVE_BOX(condition, index)    do { if (IS_ENABLED(condition)) { activeBoxes[index] = 1; } } while(0)

static void packBoxModeFlags(boxBitmask_t * mspBoxModeFlags)
{
    uint8_t activeBoxes[CHECKBOX_ITEM_COUNT];
    memset(activeBoxes, 0, sizeof(activeBoxes));

    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)),           BOXANGLE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)),         BOXHORIZON);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADING_MODE)),         BOXHEADINGHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)),        BOXHEADFREE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)),     BOXHEADADJ);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)),     BOXCAMSTAB);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)),        BOXPASSTHRU);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)),    BOXBEEPERON);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)),      BOXLEDLOW);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)),     BOXLLIGHTS);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)),         BOXOSD);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)),   BOXTELEMETRY);
    CHECK_ACTIVE_BOX(IS_ENABLED(ARMING_FLAG(ARMED)),                BOXARM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)),    BOXBLACKBOX);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)),        BOXFAILSAFE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_ALTHOLD_MODE)),     BOXNAVALTHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_POSHOLD_MODE)),     BOXNAVPOSHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_RTH_MODE)),         BOXNAVRTH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_WP_MODE)),          BOXNAVWP);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)),     BOXAIRMODE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGCSNAV)),      BOXGCSNAV);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXSURFACE)),     BOXSURFACE);
#ifdef USE_FLM_FLAPERON
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FLAPERON)),             BOXFLAPERON);
#endif
#ifdef USE_FLM_TURN_ASSIST
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(TURN_ASSISTANT)),       BOXTURNASSIST);
#endif
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_LAUNCH_MODE)),      BOXNAVLAUNCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(AUTO_TUNE)),            BOXAUTOTUNE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAUTOTRIM)),    BOXAUTOTRIM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXKILLSWITCH)),  BOXKILLSWITCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHOMERESET)),   BOXHOMERESET);

    memset(mspBoxModeFlags, 0, sizeof(boxBitmask_t));
    for (uint32_t i = 0; i < activeBoxIdCount; i++) {
        if (activeBoxes[activeBoxIds[i]]) {
            bitArraySet(mspBoxModeFlags, i);
        }
    }
}

static uint16_t packSensorStatus(void)
{
    // Sensor bits
    uint16_t sensorStatus =
            IS_ENABLED(sensors(SENSOR_ACC))     << 0 |
            IS_ENABLED(sensors(SENSOR_BARO))    << 1 |
            IS_ENABLED(sensors(SENSOR_MAG))     << 2 |
            IS_ENABLED(sensors(SENSOR_GPS))     << 3 |
            IS_ENABLED(sensors(SENSOR_RANGEFINDER))   << 4 |
            //IS_ENABLED(sensors(SENSOR_OPFLOW))  << 5 |
            IS_ENABLED(sensors(SENSOR_PITOT))   << 6;

    // Hardware failure indication bit
    if (!isHardwareHealthy()) {
        sensorStatus |= 1 << 15;        // Bit 15 of sensor bit field indicates hardware failure
    }

    return sensorStatus;
}

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
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

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashfsGetGeometry();
    sbufWriteU8(dst, flashfsIsReady() ? 1 : 0);
    sbufWriteU32(dst, geometry->sectors);
    sbufWriteU32(dst, geometry->totalSize);
    sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, uint16_t size)
{
    // Check how much bytes we can read
    const int bytesRemainingInBuf = sbufBytesRemaining(dst);
    uint16_t readLen = (size > bytesRemainingInBuf) ? bytesRemainingInBuf : size;

    // size will be lower than that requested if we reach end of volume
    const uint32_t flashfsSize = flashfsGetSize();
    if (readLen > flashfsSize - address) {
        // truncate the request
        readLen = flashfsSize - address;
    }

    // Write address
    sbufWriteU32(dst, address);

    // Read into streambuf directly
    const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);
    sbufAdvance(dst, bytesRead);
}
#endif

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspFcProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    switch (cmdMSP) {
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
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
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
        sbufWriteU32(dst, CAP_PLATFORM_32BIT | CAP_DYNBALANCE | CAP_FLAPS | CAP_NAVCAP | CAP_EXTAUX); // "capability"
        break;

#ifdef HIL
    case MSP_HIL_STATE:
        sbufWriteU16(dst, hilToSIM.pidCommand[ROLL]);
        sbufWriteU16(dst, hilToSIM.pidCommand[PITCH]);
        sbufWriteU16(dst, hilToSIM.pidCommand[YAW]);
        sbufWriteU16(dst, hilToSIM.pidCommand[THROTTLE]);
        break;
#endif

    case MSP_SENSOR_STATUS:
        sbufWriteU8(dst, isHardwareHealthy() ? 1 : 0);
        sbufWriteU8(dst, getHwGyroStatus());
        sbufWriteU8(dst, getHwAccelerometerStatus());
        sbufWriteU8(dst, getHwCompassStatus());
        sbufWriteU8(dst, getHwBarometerStatus());
        sbufWriteU8(dst, getHwGPSStatus());
        sbufWriteU8(dst, getHwRangefinderStatus());
        sbufWriteU8(dst, getHwPitotmeterStatus());
        sbufWriteU8(dst, HW_SENSOR_NONE);                   // Optical flow
        break;

    case MSP_ACTIVEBOXES:
        {
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);
            sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
        }
        break;

    case MSP_STATUS_EX:
    case MSP_STATUS:
        {
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);

            sbufWriteU16(dst, (uint16_t)cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, packSensorStatus());
            sbufWriteData(dst, &mspBoxModeFlags, 4);
            sbufWriteU8(dst, getConfigProfile());

            if (cmdMSP == MSP_STATUS_EX) {
                sbufWriteU16(dst, averageSystemLoadPercent);
                sbufWriteU16(dst, armingFlags);
                sbufWriteU8(dst, accGetCalibrationAxisFlags());
            }
        }
        break;

    case MSP_RAW_IMU:
        {
            // Hack scale due to choice of units for sensor data in multiwii
            const uint8_t scale = (acc.dev.acc_1G > 1024) ? 8 : 1;
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, acc.accADC[i] / scale);
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, mag.magADC[i]);
            }
        }
        break;

#ifdef USE_SERVOS
    case MSP_SERVO:
        sbufWriteData(dst, &servo, MAX_SUPPORTED_SERVOS * 2);
        break;
    case MSP_SERVO_CONFIGURATIONS:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, servoParams(i)->min);
            sbufWriteU16(dst, servoParams(i)->max);
            sbufWriteU16(dst, servoParams(i)->middle);
            sbufWriteU8(dst, servoParams(i)->rate);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, servoParams(i)->forwardFromChannel);
            sbufWriteU32(dst, servoParams(i)->reversedSources);
        }
        break;
    case MSP_SERVO_MIX_RULES:
        for (int i = 0; i < MAX_SERVO_RULES; i++) {
            sbufWriteU8(dst, customServoMixers(i)->targetChannel);
            sbufWriteU8(dst, customServoMixers(i)->inputSource);
            sbufWriteU8(dst, customServoMixers(i)->rate);
            sbufWriteU8(dst, customServoMixers(i)->speed);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 100);
            sbufWriteU8(dst, 0);
        }
        break;
#endif

    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++) {
            sbufWriteU16(dst, i < MAX_SUPPORTED_MOTORS ? motor[i] : 0);
        }
        break;

    case MSP_RC:
        for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
            sbufWriteU16(dst, rcData[i]);
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
#if defined(NAV)
        sbufWriteU32(dst, lrintf(getEstimatedActualPosition(Z)));
        sbufWriteU16(dst, lrintf(getEstimatedActualVelocity(Z)));
#else
        sbufWriteU32(dst, 0);
        sbufWriteU16(dst, 0);
#endif
#if defined(BARO)
        sbufWriteU32(dst, baroGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_SONAR_ALTITUDE:
#ifdef USE_RANGEFINDER
        sbufWriteU32(dst, rangefinderGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, rssi);
        if (batteryConfig()->multiwiiCurrentMeterOutput) {
            sbufWriteU16(dst, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
        } else
            sbufWriteU16(dst, (int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
        sbufWriteU8(dst, armingConfig()->disarm_kill_switch);
        break;

    case MSP_LOOP_TIME:
        sbufWriteU16(dst, gyroConfig()->looptime);
        break;

    case MSP_RC_TUNING:
        sbufWriteU8(dst, 100); //rcRate8 kept for compatibity reasons, this setting is no longer used
        sbufWriteU8(dst, currentControlRateProfile->rcExpo8);
        for (int i = 0 ; i < 3; i++) {
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
            sbufWriteU8(dst, pidBank()->pid[i].P);
            sbufWriteU8(dst, pidBank()->pid[i].I);
            sbufWriteU8(dst, pidBank()->pid[i].D);
        }
        break;

    case MSP_PIDNAMES:
        for (const char *c = pidnames; *c; c++) {
            sbufWriteU8(dst, *c);
        }
        break;

    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, 2);      // FIXME: Report as LuxFloat
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByActiveBoxId(mac->modeId);
            sbufWriteU8(dst, box ? box->permanentId : 0);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, adjRange->adjustmentIndex);
            sbufWriteU8(dst, adjRange->auxChannelIndex);
            sbufWriteU8(dst, adjRange->range.startStep);
            sbufWriteU8(dst, adjRange->range.endStep);
            sbufWriteU8(dst, adjRange->adjustmentFunction);
            sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
        }
        break;

    case MSP_BOXNAMES:
        serializeBoxNamesReply(dst);
        break;

    case MSP_BOXIDS:
        for (int i = 0; i < activeBoxIdCount; i++) {
            const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
            if (!box) {
                continue;
            }
            sbufWriteU8(dst, box->permanentId);
        }
        break;

    case MSP_MISC:
        sbufWriteU16(dst, rxConfig()->midrc);

        sbufWriteU16(dst, motorConfig()->minthrottle);
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

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

        sbufWriteU16(dst, compassConfig()->mag_declination / 10);

        sbufWriteU8(dst, batteryConfig()->vbatscale);
        sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
        break;

    case MSP_MOTOR_PINS:
        // FIXME This is hardcoded and should not be.
        for (int i = 0; i < 8; i++) {
            sbufWriteU8(dst, i + 1);
        }
        break;

#ifdef GPS
    case MSP_RAW_GPS:
        sbufWriteU8(dst, gpsSol.fixType);
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, gpsSol.llh.alt/100); // meters
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        sbufWriteU16(dst, gpsSol.hdop);
        break;

    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, gpsSol.flags.gpsHeartbeat ? 1 : 0);
        break;
#ifdef NAV
    case MSP_NAV_STATUS:
        sbufWriteU8(dst, NAV_Status.mode);
        sbufWriteU8(dst, NAV_Status.state);
        sbufWriteU8(dst, NAV_Status.activeWpAction);
        sbufWriteU8(dst, NAV_Status.activeWpNumber);
        sbufWriteU8(dst, NAV_Status.error);
        //sbufWriteU16(dst,  (int16_t)(target_bearing/100));
        sbufWriteU16(dst, getHeadingHoldTarget());
        break;
#endif

    case MSP_GPSSVINFO:
        /* Compatibility stub - return zero SVs */
        sbufWriteU8(dst, 1);

        // HDOP
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, gpsSol.hdop / 100);
        sbufWriteU8(dst, gpsSol.hdop / 100);
        break;

    case MSP_GPSSTATISTICS:
        sbufWriteU16(dst, gpsStats.lastMessageDt);
        sbufWriteU32(dst, gpsStats.errors);
        sbufWriteU32(dst, gpsStats.timeouts);
        sbufWriteU32(dst, gpsStats.packetCount);
        sbufWriteU16(dst, gpsSol.hdop);
        sbufWriteU16(dst, gpsSol.eph);
        sbufWriteU16(dst, gpsSol.epv);
        break;
#endif
    case MSP_DEBUG:
        // output some useful QA statistics
        // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

        for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
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
        sbufWriteU16(dst, boardAlignment()->rollDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDeciDegrees);
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
        sbufWriteU8(dst, 0); // for compatibility with betaflight
        sbufWriteU8(dst, 0); // for compatibility with betaflight
        sbufWriteU16(dst, 0); // for compatibility with betaflight
        sbufWriteU8(dst, rxConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxConfig()->rx_spi_id);
        sbufWriteU8(dst, rxConfig()->rx_spi_rf_channel_count);
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, 0);    // was failsafe_kill_switch
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        sbufWriteU8(dst, failsafeConfig()->failsafe_recovery_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_roll_angle);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_pitch_angle);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_yaw_rate);
        sbufWriteU16(dst, failsafeConfig()->failsafe_stick_motion_threshold);
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, MAX_MAPPABLE_RX_INPUTS);
        break;

    case MSP_BF_CONFIG:
        sbufWriteU8(dst, mixerConfig()->mixerMode);

        sbufWriteU32(dst, featureMask());

        sbufWriteU8(dst, rxConfig()->serialrx_provider);

        sbufWriteU16(dst, boardAlignment()->rollDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDeciDegrees);

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
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].peripheral_baudrateIndex);
        }
        break;

#ifdef LED_STRIP
    case MSP_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            const hsvColor_t *color = &ledStripConfig()->colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;

    case MSP_LED_STRIP_CONFIG:
        for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
            const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[i];
            sbufWriteU32(dst, *ledConfig);
        }
        break;

    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, ledStripConfig()->modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, ledStripConfig()->specialColors.color[j]);
        }
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

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
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_OSD_CONFIG:
#ifdef OSD
        sbufWriteU8(dst, 1); // OSD supported
        // send video system (AUTO/PAL/NTSC)
#ifdef USE_MAX7456
        sbufWriteU8(dst, osdConfig()->video_system);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, osdConfig()->units);
        sbufWriteU8(dst, osdConfig()->rssi_alarm);
        sbufWriteU16(dst, osdConfig()->cap_alarm);
        sbufWriteU16(dst, osdConfig()->time_alarm);
        sbufWriteU16(dst, osdConfig()->alt_alarm);
        for (int i = 0; i < OSD_ITEM_COUNT; i++) {
            sbufWriteU16(dst, osdConfig()->item_pos[i]);
        }
#else
        sbufWriteU8(dst, 0); // OSD not supported
#endif
        break;

    case MSP_BF_BUILD_INFO:
        sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        sbufWriteU32(dst, 0); // future exp
        sbufWriteU32(dst, 0); // future exp
        break;

    case MSP_3D:
        sbufWriteU16(dst, flight3DConfig()->deadband3d_low);
        sbufWriteU16(dst, flight3DConfig()->deadband3d_high);
        sbufWriteU16(dst, flight3DConfig()->neutral3d);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, rcControlsConfig()->deadband);
        sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
        sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
        sbufWriteU16(dst, rcControlsConfig()->deadband3d_throttle);
        break;

    case MSP_SENSOR_ALIGNMENT:
        sbufWriteU8(dst, gyroConfig()->gyro_align);
        sbufWriteU8(dst, accelerometerConfig()->acc_align);
        sbufWriteU8(dst, compassConfig()->mag_align);
        break;

    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, gyroConfig()->gyroSyncDenominator);
        sbufWriteU8(dst, 1);    // BF: masterConfig.pid_process_denom
        sbufWriteU8(dst, 1);    // BF: motorConfig()->useUnsyncedPwm
        sbufWriteU8(dst, motorConfig()->motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->motorPwmRate);
#ifdef USE_SERVOS
        sbufWriteU16(dst, servoConfig()->servoPwmRate);
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU8(dst, gyroConfig()->gyroSync);
        break;

    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, gyroConfig()->gyro_soft_lpf_hz);
        sbufWriteU16(dst, pidProfile()->dterm_lpf_hz);
        sbufWriteU16(dst, pidProfile()->yaw_lpf_hz);
#ifdef USE_GYRO_NOTCH_1
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1); //masterConfig.gyro_soft_notch_hz_1
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1); //BF: masterConfig.gyro_soft_notch_cutoff_1
#else
        sbufWriteU16(dst, 0); //masterConfig.gyro_soft_notch_hz_1
        sbufWriteU16(dst, 1); //BF: masterConfig.gyro_soft_notch_cutoff_1
#endif

#ifdef USE_DTERM_NOTCH
        sbufWriteU16(dst, pidProfile()->dterm_soft_notch_hz); //BF: pidProfile()->dterm_notch_hz
        sbufWriteU16(dst, pidProfile()->dterm_soft_notch_cutoff); //pidProfile()->dterm_notch_cutoff
#else
        sbufWriteU16(dst, 0); //BF: pidProfile()->dterm_notch_hz
        sbufWriteU16(dst, 1); //pidProfile()->dterm_notch_cutoff
#endif

#ifdef USE_GYRO_NOTCH_2
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2); //BF: masterConfig.gyro_soft_notch_hz_2
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2); //BF: masterConfig.gyro_soft_notch_cutoff_2
#else
        sbufWriteU16(dst, 0); //BF: masterConfig.gyro_soft_notch_hz_2
        sbufWriteU16(dst, 1); //BF: masterConfig.gyro_soft_notch_cutoff_2
#endif
        break;

    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, pidProfile()->rollPitchItermIgnoreRate);
        sbufWriteU16(dst, pidProfile()->yawItermIgnoreRate);
        sbufWriteU16(dst, pidProfile()->yaw_p_limit);
        sbufWriteU8(dst, 0); //BF: pidProfile()->deltaMethod
        sbufWriteU8(dst, 0); //BF: pidProfile()->vbatPidCompensation
        sbufWriteU8(dst, 0); //BF: pidProfile()->setpointRelaxRatio
        sbufWriteU8(dst, constrain(pidProfile()->dterm_setpoint_weight * 100, 0, 255));
        sbufWriteU16(dst, pidProfile()->pidSumLimit);
        sbufWriteU8(dst, 0); //BF: pidProfile()->itermThrottleGain

        /*
         * To keep compatibility on MSP frame length level with Betaflight, axis axisAccelerationLimitYaw
         * limit will be sent and received in [dps / 10]
         */
        sbufWriteU16(dst, constrain(pidProfile()->axisAccelerationLimitRollPitch / 10, 0, 65535));
        sbufWriteU16(dst, constrain(pidProfile()->axisAccelerationLimitYaw / 10, 0, 65535));
        break;

    case MSP_INAV_PID:
    #ifdef ASYNC_GYRO_PROCESSING
        sbufWriteU8(dst, systemConfig()->asyncMode);
        sbufWriteU16(dst, systemConfig()->accTaskFrequency);
        sbufWriteU16(dst, systemConfig()->attitudeTaskFrequency);
    #else
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    #endif
        sbufWriteU8(dst, pidProfile()->heading_hold_rate_limit);
        sbufWriteU8(dst, HEADING_HOLD_ERROR_LPF_FREQ);
        sbufWriteU16(dst, mixerConfig()->yaw_jump_prevention_limit);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf);
        sbufWriteU8(dst, pidProfile()->acc_soft_lpf_hz);
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        break;

    case MSP_SENSOR_CONFIG:
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
#ifdef BARO
        sbufWriteU8(dst, barometerConfig()->baro_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef MAG
        sbufWriteU8(dst, compassConfig()->mag_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef PITOT
        sbufWriteU8(dst, pitotmeterConfig()->pitot_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_RANGEFINDER
        sbufWriteU8(dst, rangefinderConfig()->rangefinder_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, 0);    // optical flow hardware
        break;

#ifdef NAV
    case MSP_NAV_POSHOLD:
        sbufWriteU8(dst, navConfig()->general.flags.user_control_mode);
        sbufWriteU16(dst, navConfig()->general.max_auto_speed);
        sbufWriteU16(dst, navConfig()->general.max_auto_climb_rate);
        sbufWriteU16(dst, navConfig()->general.max_manual_speed);
        sbufWriteU16(dst, navConfig()->general.max_manual_climb_rate);
        sbufWriteU8(dst, navConfig()->mc.max_bank_angle);
        sbufWriteU8(dst, navConfig()->general.flags.use_thr_mid_for_althold);
        sbufWriteU16(dst, navConfig()->mc.hover_throttle);
        break;

    case MSP_RTH_AND_LAND_CONFIG:
        sbufWriteU16(dst, navConfig()->general.min_rth_distance);
        sbufWriteU8(dst, navConfig()->general.flags.rth_climb_first);
        sbufWriteU8(dst, navConfig()->general.flags.rth_climb_ignore_emerg);
        sbufWriteU8(dst, navConfig()->general.flags.rth_tail_first);
        sbufWriteU8(dst, navConfig()->general.flags.rth_allow_landing);
        sbufWriteU8(dst, navConfig()->general.flags.rth_alt_control_mode);
        sbufWriteU16(dst, navConfig()->general.rth_abort_threshold);
        sbufWriteU16(dst, navConfig()->general.rth_altitude);
        sbufWriteU16(dst, navConfig()->general.land_descent_rate);
        sbufWriteU16(dst, navConfig()->general.land_slowdown_minalt);
        sbufWriteU16(dst, navConfig()->general.land_slowdown_maxalt);
        sbufWriteU16(dst, navConfig()->general.emerg_descent_rate);
        break;

    case MSP_FW_CONFIG:
        sbufWriteU16(dst, navConfig()->fw.cruise_throttle);
        sbufWriteU16(dst, navConfig()->fw.min_throttle);
        sbufWriteU16(dst, navConfig()->fw.max_throttle);
        sbufWriteU8(dst, navConfig()->fw.max_bank_angle);
        sbufWriteU8(dst, navConfig()->fw.max_climb_angle);
        sbufWriteU8(dst, navConfig()->fw.max_dive_angle);
        sbufWriteU8(dst, navConfig()->fw.pitch_to_throttle);
        sbufWriteU16(dst, navConfig()->fw.loiter_radius);
        break;
#endif

    case MSP_CALIBRATION_DATA:
    #ifdef ACC
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[X]);
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[Y]);
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[Z]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[X]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[Y]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[Z]);
    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    #endif

    #ifdef MAG
        sbufWriteU16(dst, compassConfig()->magZero.raw[X]);
        sbufWriteU16(dst, compassConfig()->magZero.raw[Y]);
        sbufWriteU16(dst, compassConfig()->magZero.raw[Z]);
    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    #endif
        break;

    case MSP_POSITION_ESTIMATION_CONFIG:
    #ifdef NAV

        sbufWriteU16(dst, positionEstimationConfig()->w_z_baro_p * 100); //     inav_w_z_baro_p float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_z_gps_p * 100);  // 2   inav_w_z_gps_p  float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_z_gps_v * 100);  // 2   inav_w_z_gps_v  float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_xy_gps_p * 100); // 2   inav_w_xy_gps_p float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_xy_gps_v * 100); // 2   inav_w_xy_gps_v float as value * 100
        sbufWriteU8(dst, gpsConfigMutable()->gpsMinSats);                // 1
        sbufWriteU8(dst, positionEstimationConfig()->use_gps_velned);    // 1   inav_use_gps_velned ON/OFF

    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
    #endif
        break;

    case MSP_REBOOT:
        if (!ARMING_FLAG(ARMED)) {
            if (mspPostProcessFn) {
                *mspPostProcessFn = mspRebootFn;
            }
        }
        break;

    case MSP_WP_GETINFO:
#ifdef NAV
        sbufWriteU8(dst, 0);                        // Reserved for waypoint capabilities
        sbufWriteU8(dst, NAV_MAX_WAYPOINTS);        // Maximum number of waypoints supported
        sbufWriteU8(dst, isWaypointListValid());    // Is current mission valid
        sbufWriteU8(dst, getWaypointCount());       // Number of waypoints in current mission
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    default:
        return false;
    }
    return true;
}

#ifdef NAV
static void mspFcWaypointOutCommand(sbuf_t *dst, sbuf_t *src)
{
    const uint8_t msp_wp_no = sbufReadU8(src);    // get the wp number
    navWaypoint_t msp_wp;
    getWaypoint(msp_wp_no, &msp_wp);
    sbufWriteU8(dst, msp_wp_no);   // wp_no
    sbufWriteU8(dst, msp_wp.action);  // action (WAYPOINT)
    sbufWriteU32(dst, msp_wp.lat);    // lat
    sbufWriteU32(dst, msp_wp.lon);    // lon
    sbufWriteU32(dst, msp_wp.alt);    // altitude (cm)
    sbufWriteU16(dst, msp_wp.p1);     // P1
    sbufWriteU16(dst, msp_wp.p2);     // P2
    sbufWriteU16(dst, msp_wp.p3);     // P3
    sbufWriteU8(dst, msp_wp.flag);    // flags
}
#endif

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    uint16_t readLength;

    const uint32_t readAddress = sbufReadU32(src);

    // Request payload:
    //  uint32_t    - address to read from
    //  uint16_t    - size of block to read (optional)
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
    }
    else {
        readLength = 128;
    }

    serializeDataflashReadReply(dst, readAddress, readLength);
}
#endif

static mspResult_e mspFcProcessInCommand(uint8_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint16_t tmp;
    uint8_t rate;

    const unsigned int dataSize = sbufBytesRemaining(src);

    switch (cmdMSP) {
#ifdef HIL
    case MSP_SET_HIL_STATE:
        hilToFC.rollAngle = sbufReadU16(src);
        hilToFC.pitchAngle = sbufReadU16(src);
        hilToFC.yawAngle = sbufReadU16(src);
        hilToFC.baroAlt = sbufReadU32(src);
        hilToFC.bodyAccel[0] = sbufReadU16(src);
        hilToFC.bodyAccel[1] = sbufReadU16(src);
        hilToFC.bodyAccel[2] = sbufReadU16(src);
        hilActive = true;
        break;
#endif
    case MSP_SELECT_SETTING:
        if (!ARMING_FLAG(ARMED)) {
            const uint8_t profileIndex = sbufReadU8(src);
            setConfigProfileAndWriteEEPROM(profileIndex);
        }
        break;

    case MSP_SET_HEAD:
        updateHeadingHoldTarget(sbufReadU16(src));
        break;

    case MSP_SET_RAW_RC:
#ifdef USE_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;

    case MSP_SET_ARMING_CONFIG:
        armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);
        armingConfigMutable()->disarm_kill_switch = sbufReadU8(src);
        break;

    case MSP_SET_LOOP_TIME:
        gyroConfigMutable()->looptime = sbufReadU16(src);
        break;

    case MSP_SET_PID_CONTROLLER:
        // FIXME: Do nothing
        break;

    case MSP_SET_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            pidBankMutable()->pid[i].P = sbufReadU8(src);
            pidBankMutable()->pid[i].I = sbufReadU8(src);
            pidBankMutable()->pid[i].D = sbufReadU8(src);
        }
        schedulePidGainsUpdate();
#if defined(NAV)
        navigationUsePIDs();
#endif
        break;

    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermenantId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);

                updateUsedModeActivationConditionFlags();
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = adjustmentRangesMutable(i);
            i = sbufReadU8(src);
            if (i < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                adjRange->adjustmentIndex = i;
                adjRange->auxChannelIndex = sbufReadU8(src);
                adjRange->range.startStep = sbufReadU8(src);
                adjRange->range.endStep = sbufReadU8(src);
                adjRange->adjustmentFunction = sbufReadU8(src);
                adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        if (dataSize >= 10) {
            sbufReadU8(src); //Read rcRate8, kept for protocol compatibility reasons
            // need to cast away const to set controlRateProfile
            ((controlRateConfig_t*)currentControlRateProfile)->rcExpo8 = sbufReadU8(src);
            for (int i = 0; i < 3; i++) {
                rate = sbufReadU8(src);
                if (i == FD_YAW) {
                    ((controlRateConfig_t*)currentControlRateProfile)->rates[i] = constrain(rate, CONTROL_RATE_CONFIG_YAW_RATE_MIN, CONTROL_RATE_CONFIG_YAW_RATE_MAX);
                }
                else {
                    ((controlRateConfig_t*)currentControlRateProfile)->rates[i] = constrain(rate, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
                }
            }
            rate = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->dynThrPID = MIN(rate, CONTROL_RATE_CONFIG_TPA_MAX);
            ((controlRateConfig_t*)currentControlRateProfile)->thrMid8 = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->thrExpo8 = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->tpa_breakpoint = sbufReadU16(src);
            if (dataSize >= 11) {
                ((controlRateConfig_t*)currentControlRateProfile)->rcYawExpo8 = sbufReadU8(src);
            }

            schedulePidGainsUpdate();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_MISC:
        tmp = sbufReadU16(src);
        if (tmp < 1600 && tmp > 1400)
            rxConfigMutable()->midrc = tmp;

        motorConfigMutable()->minthrottle = sbufReadU16(src);
        motorConfigMutable()->maxthrottle = sbufReadU16(src);
        motorConfigMutable()->mincommand = sbufReadU16(src);

        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);

#ifdef GPS
        gpsConfigMutable()->provider = sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        gpsConfigMutable()->sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
        sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        sbufReadU8(src); // gps_ubx_sbas
#endif
        batteryConfigMutable()->multiwiiCurrentMeterOutput = sbufReadU8(src);
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        sbufReadU8(src);

#ifdef MAG
        compassConfigMutable()->mag_declination = sbufReadU16(src) * 10;
#else
        sbufReadU16(src);
#endif

        batteryConfigMutable()->vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_MOTOR:
        for (int i = 0; i < 8; i++) {
            const int16_t disarmed = sbufReadU16(src);
            if (i < MAX_SUPPORTED_MOTORS) {
                motor_disarmed[i] = disarmed;
            }
        }
        break;

    case MSP_SET_SERVO_CONFIGURATION:
#ifdef USE_SERVOS
        if (dataSize != (1 + 14)) {
            return MSP_RESULT_ERROR;
        }
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        } else {
            servoParamsMutable(i)->min = sbufReadU16(src);
            servoParamsMutable(i)->max = sbufReadU16(src);
            servoParamsMutable(i)->middle = sbufReadU16(src);
            servoParamsMutable(i)->rate = sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            servoParamsMutable(i)->forwardFromChannel = sbufReadU8(src);
            servoParamsMutable(i)->reversedSources = sbufReadU32(src);
        }
#endif
        break;

    case MSP_SET_SERVO_MIX_RULE:
#ifdef USE_SERVOS
        i = sbufReadU8(src);
        if (i >= MAX_SERVO_RULES) {
            return MSP_RESULT_ERROR;
        } else {
            customServoMixersMutable(i)->targetChannel = sbufReadU8(src);
            customServoMixersMutable(i)->inputSource = sbufReadU8(src);
            customServoMixersMutable(i)->rate = sbufReadU8(src);
            customServoMixersMutable(i)->speed = sbufReadU8(src);
            sbufReadU16(src); //Read 2bytes for min/max and ignore it
            sbufReadU8(src); //Read 1 byte for `box` and ignore it
            loadCustomServoMixer();
        }
#endif
        break;

    case MSP_SET_3D:
        flight3DConfigMutable()->deadband3d_low = sbufReadU16(src);
        flight3DConfigMutable()->deadband3d_high = sbufReadU16(src);
        flight3DConfigMutable()->neutral3d = sbufReadU16(src);
        break;

    case MSP_SET_RC_DEADBAND:
        rcControlsConfigMutable()->deadband = sbufReadU8(src);
        rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->alt_hold_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->deadband3d_throttle = sbufReadU16(src);
        break;

    case MSP_SET_RESET_CURR_PID:
        PG_RESET_CURRENT(pidProfile);
        break;

    case MSP_SET_SENSOR_ALIGNMENT:
        gyroConfigMutable()->gyro_align = sbufReadU8(src);
        accelerometerConfigMutable()->acc_align = sbufReadU8(src);
#ifdef MAG
        compassConfigMutable()->mag_align = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        break;

    case MSP_SET_ADVANCED_CONFIG:
        gyroConfigMutable()->gyroSyncDenominator = sbufReadU8(src);
        sbufReadU8(src);    // BF: masterConfig.pid_process_denom
        sbufReadU8(src);    // BF: motorConfig()->useUnsyncedPwm
        motorConfigMutable()->motorPwmProtocol = sbufReadU8(src);
        motorConfigMutable()->motorPwmRate = sbufReadU16(src);
#ifdef USE_SERVOS
        servoConfigMutable()->servoPwmRate = sbufReadU16(src);
#else
        sbufReadU16(src);
#endif
        gyroConfigMutable()->gyroSync = sbufReadU8(src);
        break;

    case MSP_SET_FILTER_CONFIG :
        gyroConfigMutable()->gyro_soft_lpf_hz = sbufReadU8(src);
        pidProfileMutable()->dterm_lpf_hz = constrain(sbufReadU16(src), 0, 255);
        pidProfileMutable()->yaw_lpf_hz = constrain(sbufReadU16(src), 0, 255);
#ifdef USE_GYRO_NOTCH_1
        gyroConfigMutable()->gyro_soft_notch_hz_1 = constrain(sbufReadU16(src), 0, 500);
        gyroConfigMutable()->gyro_soft_notch_cutoff_1 = constrain(sbufReadU16(src), 1, 500);
#endif
#ifdef USE_DTERM_NOTCH
        pidProfileMutable()->dterm_soft_notch_hz = constrain(sbufReadU16(src), 0, 500);
        pidProfileMutable()->dterm_soft_notch_cutoff = constrain(sbufReadU16(src), 1, 500);
        pidInitFilters();
#endif
#ifdef USE_GYRO_NOTCH_2
        gyroConfigMutable()->gyro_soft_notch_hz_2 = constrain(sbufReadU16(src), 0, 500);
        gyroConfigMutable()->gyro_soft_notch_cutoff_2 = constrain(sbufReadU16(src), 1, 500);
#endif
        break;

    case MSP_SET_PID_ADVANCED:
        pidProfileMutable()->rollPitchItermIgnoreRate = sbufReadU16(src);
        pidProfileMutable()->yawItermIgnoreRate = sbufReadU16(src);
        pidProfileMutable()->yaw_p_limit = sbufReadU16(src);

        sbufReadU8(src); //BF: pidProfileMutable()->deltaMethod
        sbufReadU8(src); //BF: pidProfileMutable()->vbatPidCompensation
        sbufReadU8(src); //BF: pidProfileMutable()->setpointRelaxRatio
        pidProfileMutable()->dterm_setpoint_weight = constrainf(sbufReadU8(src) / 100.0f, 0.0f, 2.0f);
        pidProfileMutable()->pidSumLimit = sbufReadU16(src);
        sbufReadU8(src); //BF: pidProfileMutable()->itermThrottleGain

        /*
         * To keep compatibility on MSP frame length level with Betaflight, axis axisAccelerationLimitYaw
         * limit will be sent and received in [dps / 10]
         */
        pidProfileMutable()->axisAccelerationLimitRollPitch = sbufReadU16(src) * 10;
        pidProfileMutable()->axisAccelerationLimitYaw = sbufReadU16(src) * 10;
        break;

    case MSP_SET_INAV_PID:
        #ifdef ASYNC_GYRO_PROCESSING
            systemConfigMutable()->asyncMode = sbufReadU8(src);
            systemConfigMutable()->accTaskFrequency = sbufReadU16(src);
            systemConfigMutable()->attitudeTaskFrequency = sbufReadU16(src);
        #else
            sbufReadU8(src);
            sbufReadU16(src);
            sbufReadU16(src);
        #endif
            pidProfileMutable()->heading_hold_rate_limit = sbufReadU8(src);
            sbufReadU8(src); //HEADING_HOLD_ERROR_LPF_FREQ
            mixerConfigMutable()->yaw_jump_prevention_limit = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf = sbufReadU8(src);
            pidProfileMutable()->acc_soft_lpf_hz = sbufReadU8(src);
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
        break;

    case MSP_SET_SENSOR_CONFIG:
        accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
#ifdef BARO
        barometerConfigMutable()->baro_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#ifdef MAG
        compassConfigMutable()->mag_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#ifdef PITOT
        pitotmeterConfigMutable()->pitot_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#ifdef USE_RANGEFINDER
        rangefinderConfigMutable()->rangefinder_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);        // rangefinder hardware
#endif
        sbufReadU8(src);        // optical flow hardware
        break;

#ifdef NAV
    case MSP_SET_NAV_POSHOLD:
        navConfigMutable()->general.flags.user_control_mode = sbufReadU8(src);
        navConfigMutable()->general.max_auto_speed = sbufReadU16(src);
        navConfigMutable()->general.max_auto_climb_rate = sbufReadU16(src);
        navConfigMutable()->general.max_manual_speed = sbufReadU16(src);
        navConfigMutable()->general.max_manual_climb_rate = sbufReadU16(src);
        navConfigMutable()->mc.max_bank_angle = sbufReadU8(src);
        navConfigMutable()->general.flags.use_thr_mid_for_althold = sbufReadU8(src);
        navConfigMutable()->mc.hover_throttle = sbufReadU16(src);
        break;

    case MSP_SET_RTH_AND_LAND_CONFIG:
        navConfigMutable()->general.min_rth_distance = sbufReadU16(src);
        navConfigMutable()->general.flags.rth_climb_first = sbufReadU8(src);
        navConfigMutable()->general.flags.rth_climb_ignore_emerg = sbufReadU8(src);
        navConfigMutable()->general.flags.rth_tail_first = sbufReadU8(src);
        navConfigMutable()->general.flags.rth_allow_landing = sbufReadU8(src);
        navConfigMutable()->general.flags.rth_alt_control_mode = sbufReadU8(src);
        navConfigMutable()->general.rth_abort_threshold = sbufReadU16(src);
        navConfigMutable()->general.rth_altitude = sbufReadU16(src);
        navConfigMutable()->general.land_descent_rate = sbufReadU16(src);
        navConfigMutable()->general.land_slowdown_minalt = sbufReadU16(src);
        navConfigMutable()->general.land_slowdown_maxalt = sbufReadU16(src);
        navConfigMutable()->general.emerg_descent_rate = sbufReadU16(src);
        break;

    case MSP_SET_FW_CONFIG:
        navConfigMutable()->fw.cruise_throttle = sbufReadU16(src);
        navConfigMutable()->fw.min_throttle = sbufReadU16(src);
        navConfigMutable()->fw.max_throttle = sbufReadU16(src);
        navConfigMutable()->fw.max_bank_angle = sbufReadU8(src);
        navConfigMutable()->fw.max_climb_angle = sbufReadU8(src);
        navConfigMutable()->fw.max_dive_angle = sbufReadU8(src);
        navConfigMutable()->fw.pitch_to_throttle = sbufReadU8(src);
        navConfigMutable()->fw.loiter_radius = sbufReadU16(src);
        break;

#endif

    case MSP_SET_CALIBRATION_DATA:
    #ifdef ACC
        accelerometerConfigMutable()->accZero.raw[X] = sbufReadU16(src);
        accelerometerConfigMutable()->accZero.raw[Y] = sbufReadU16(src);
        accelerometerConfigMutable()->accZero.raw[Z] = sbufReadU16(src);
        accelerometerConfigMutable()->accGain.raw[X] = sbufReadU16(src);
        accelerometerConfigMutable()->accGain.raw[Y] = sbufReadU16(src);
        accelerometerConfigMutable()->accGain.raw[Z] = sbufReadU16(src);
    #else
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src);
    #endif

    #ifdef MAG
        compassConfigMutable()->magZero.raw[X] = sbufReadU16(src);
        compassConfigMutable()->magZero.raw[Y] = sbufReadU16(src);
        compassConfigMutable()->magZero.raw[Z] = sbufReadU16(src);
    #else
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src);
    #endif
        break;

    case MSP_SET_POSITION_ESTIMATION_CONFIG:
    #ifdef NAV
        positionEstimationConfigMutable()->w_z_baro_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
        positionEstimationConfigMutable()->w_z_gps_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
        positionEstimationConfigMutable()->w_z_gps_v = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
        positionEstimationConfigMutable()->w_xy_gps_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
        positionEstimationConfigMutable()->w_xy_gps_v = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
        gpsConfigMutable()->gpsMinSats = constrain(sbufReadU8(src), 5, 10);
        positionEstimationConfigMutable()->use_gps_velned = constrain(sbufReadU8(src), 0, 1);
    #endif
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
        if (ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
        }
        writeEEPROM();
        readEEPROM();
        break;

#ifdef BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            blackboxConfigMutable()->device = sbufReadU8(src);
            blackboxConfigMutable()->rate_num = sbufReadU8(src);
            blackboxConfigMutable()->rate_denom = sbufReadU8(src);
        }
        break;
#endif

#ifdef OSD
    case MSP_SET_OSD_CONFIG:
        {
            const uint8_t addr = sbufReadU8(src);
            // set all the other settings
            if ((int8_t)addr == -1) {
#ifdef USE_MAX7456
                osdConfigMutable()->video_system = sbufReadU8(src);
#else
                sbufReadU8(src); // Skip video system
#endif
                osdConfigMutable()->units = sbufReadU8(src);
                osdConfigMutable()->rssi_alarm = sbufReadU8(src);
                osdConfigMutable()->cap_alarm = sbufReadU16(src);
                osdConfigMutable()->time_alarm = sbufReadU16(src);
                osdConfigMutable()->alt_alarm = sbufReadU16(src);
            } else {
                // set a position setting
                const uint16_t pos  = sbufReadU16(src);
                if (addr < OSD_ITEM_COUNT) {
                    osdConfigMutable()->item_pos[addr] = pos;
                }
            }
        }
        break;
    case MSP_OSD_CHAR_WRITE:
#ifdef USE_MAX7456
        {
            uint8_t font_data[64];
            const uint8_t addr = sbufReadU8(src);
            for (int i = 0; i < 54; i++) {
                font_data[i] = sbufReadU8(src);
            }
            // !!TODO - replace this with a device independent implementation
            max7456WriteNvm(addr, font_data);
        }
#else
        // just discard the data
        sbufReadU8(src);
        for (int i = 0; i < 54; i++) {
            sbufReadU8(src);
        }
#endif
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
        gpsSol.flags.validVelNE = 0;
        gpsSol.flags.validVelD = 0;
        gpsSol.flags.validEPE = 0;
        gpsSol.numSat = sbufReadU8(src);
        gpsSol.llh.lat = sbufReadU32(src);
        gpsSol.llh.lon = sbufReadU32(src);
        gpsSol.llh.alt = sbufReadU16(src);
        gpsSol.groundSpeed = sbufReadU16(src);
        gpsSol.velNED[X] = 0;
        gpsSol.velNED[Y] = 0;
        gpsSol.velNED[Z] = 0;
        gpsSol.eph = 100;
        gpsSol.epv = 100;
        // Feed data to navigation
        sensorsSet(SENSOR_GPS);
        onNewGPSData();
        break;
#endif
#ifdef NAV
    case MSP_SET_WP:
        {
            const uint8_t msp_wp_no = sbufReadU8(src);     // get the waypoint number
            navWaypoint_t msp_wp;
            msp_wp.action = sbufReadU8(src);    // action
            msp_wp.lat = sbufReadU32(src);      // lat
            msp_wp.lon = sbufReadU32(src);      // lon
            msp_wp.alt = sbufReadU32(src);      // to set altitude (cm)
            msp_wp.p1 = sbufReadU16(src);       // P1
            msp_wp.p2 = sbufReadU16(src);       // P2
            msp_wp.p3 = sbufReadU16(src);       // P3
            msp_wp.flag = sbufReadU8(src);      // future: to set nav flag
            setWaypoint(msp_wp_no, &msp_wp);
        }
        break;
#endif
    case MSP_SET_FEATURE:
        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap
        break;

    case MSP_SET_BOARD_ALIGNMENT:
        boardAlignmentMutable()->rollDeciDegrees = sbufReadU16(src);
        boardAlignmentMutable()->pitchDeciDegrees = sbufReadU16(src);
        boardAlignmentMutable()->yawDeciDegrees = sbufReadU16(src);
        break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
        batteryConfigMutable()->vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_CURRENT_METER_CONFIG:
        batteryConfigMutable()->currentMeterScale = sbufReadU16(src);
        batteryConfigMutable()->currentMeterOffset = sbufReadU16(src);
        batteryConfigMutable()->currentMeterType = sbufReadU8(src);
        batteryConfigMutable()->batteryCapacity = sbufReadU16(src);
        break;

#ifndef USE_QUAD_MIXER_ONLY
    case MSP_SET_MIXER:
        mixerConfigMutable()->mixerMode = sbufReadU8(src);
        mixerUpdateStateFlags();    // Required for correct preset functionality
        break;
#endif

    case MSP_SET_RX_CONFIG:
        rxConfigMutable()->serialrx_provider = sbufReadU8(src);
        rxConfigMutable()->maxcheck = sbufReadU16(src);
        rxConfigMutable()->midrc = sbufReadU16(src);
        rxConfigMutable()->mincheck = sbufReadU16(src);
        rxConfigMutable()->spektrum_sat_bind = sbufReadU8(src);
        if (dataSize > 8) {
            rxConfigMutable()->rx_min_usec = sbufReadU16(src);
            rxConfigMutable()->rx_max_usec = sbufReadU16(src);
        }
        if (dataSize > 12) {
            // for compatibility with betaflight
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU16(src);
        }
        if (dataSize > 16) {
            rxConfigMutable()->rx_spi_protocol = sbufReadU8(src);
        }
        if (dataSize > 17) {
            rxConfigMutable()->rx_spi_id = sbufReadU32(src);
        }
        if (dataSize > 21) {
            rxConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
        }
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_off_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
        sbufReadU8(src);    // was failsafe_kill_switch
        failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
        if (dataSize > 8) {
            failsafeConfigMutable()->failsafe_recovery_delay = sbufReadU8(src);
        }
        if (dataSize > 9) {
            failsafeConfigMutable()->failsafe_fw_roll_angle = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_fw_pitch_angle = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_fw_yaw_rate = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_stick_motion_threshold = sbufReadU16(src);
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (int i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
            rxConfigMutable()->rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_BF_CONFIG:
#ifdef USE_QUAD_MIXER_ONLY
        sbufReadU8(src); // mixerMode ignored
#else
        mixerConfigMutable()->mixerMode = sbufReadU8(src); // mixerMode
        mixerUpdateStateFlags();    // Required for correct preset functionality
#endif

        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap

        rxConfigMutable()->serialrx_provider = sbufReadU8(src); // serialrx_type

        boardAlignmentMutable()->rollDeciDegrees = sbufReadU16(src); // board_align_roll
        boardAlignmentMutable()->pitchDeciDegrees = sbufReadU16(src); // board_align_pitch
        boardAlignmentMutable()->yawDeciDegrees = sbufReadU16(src); // board_align_yaw

        batteryConfigMutable()->currentMeterScale = sbufReadU16(src);
        batteryConfigMutable()->currentMeterOffset = sbufReadU16(src);
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->peripheral_baudrateIndex = sbufReadU8(src);
            }
        }
        break;

#ifdef LED_STRIP
    case MSP_SET_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &ledStripConfigMutable()->colors[i];
            color->h = sbufReadU16(src);
            color->s = sbufReadU8(src);
            color->v = sbufReadU8(src);
        }
        break;

    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = sbufReadU8(src);
            if (i >= LED_MAX_STRIP_LENGTH || dataSize != (1 + 4)) {
                return MSP_RESULT_ERROR;
            }
            ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[i];
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
        }
        break;

    case MSP_SET_LED_STRIP_MODECOLOR:
        {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color))
                return MSP_RESULT_ERROR;
        }
        break;
#endif

#ifdef NAV_NON_VOLATILE_WAYPOINT_STORAGE
    case MSP_WP_MISSION_LOAD:
        sbufReadU8(src);    // Mission ID (reserved)
        if (!loadNonVolatileWaypointList()) {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_WP_MISSION_SAVE:
        sbufReadU8(src);    // Mission ID (reserved)
        if (!saveNonVolatileWaypointList()) {
            return MSP_RESULT_ERROR;
        }
        break;
#endif

    default:
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspFcProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    } else if (cmdMSP == MSP_SET_4WAY_IF) {
        mspFc4waySerialCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
#endif
#ifdef NAV
    } else if (cmdMSP == MSP_WP) {
        mspFcWaypointOutCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
#ifdef USE_FLASHFS
    } else if (cmdMSP == MSP_DATAFLASH_READ) {
        mspFcDataFlashReadCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspFcProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}

/*
 * Return a pointer to the process command function
 */
void mspFcInit(void)
{
    initActiveBoxIds();
}
