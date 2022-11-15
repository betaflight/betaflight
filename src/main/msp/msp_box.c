/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/bitarray.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#include "pg/piniobox.h"

#include "msp_box.h"


// permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
static const box_t boxes[CHECKBOX_ITEM_COUNT] = {
    { .boxId = BOXARM, .boxName = "ARM", .permanentId = 0 },
    { .boxId = BOXANGLE, .boxName = "ANGLE", .permanentId = 1 },
    { .boxId = BOXHORIZON, .boxName = "HORIZON", .permanentId = 2 },
//    { .boxId = BOXBARO, .boxName = "BARO", .permanentId = 3 },
    { .boxId = BOXANTIGRAVITY, .boxName = "ANTI GRAVITY", .permanentId = 4 },
    { .boxId = BOXMAG, .boxName = "MAG", .permanentId = 5 },
    { .boxId = BOXHEADFREE, .boxName = "HEADFREE", .permanentId = 6 },
    { .boxId = BOXHEADADJ, .boxName = "HEADADJ", .permanentId = 7 },
    { .boxId = BOXCAMSTAB, .boxName = "CAMSTAB", .permanentId = 8 },
//    { .boxId = BOXCAMTRIG, .boxName = "CAMTRIG", .permanentId = 9 },
//    { .boxId = BOXGPSHOME, .boxName = "GPS HOME", .permanentId = 10 },
//    { .boxId = BOXGPSHOLD, .boxName = "GPS HOLD", .permanentId = 11 },
    { .boxId = BOXPASSTHRU, .boxName = "PASSTHRU", .permanentId = 12 },
    { .boxId = BOXBEEPERON, .boxName = "BEEPER", .permanentId = 13 },
//    { .boxId = BOXLEDMAX, .boxName = "LEDMAX", .permanentId = 14 }, (removed)
    { .boxId = BOXLEDLOW, .boxName = "LEDLOW", .permanentId = 15 },
//    { .boxId = BOXLLIGHTS, .boxName = "LLIGHTS", .permanentId = 16 }, (removed)
    { .boxId = BOXCALIB, .boxName = "CALIB", .permanentId = 17 },
//    { .boxId = BOXGOV, .boxName = "GOVERNOR", .permanentId = 18 }, (removed)
    { .boxId = BOXOSD, .boxName = "OSD DISABLE", .permanentId = 19 },
    { .boxId = BOXTELEMETRY, .boxName = "TELEMETRY", .permanentId = 20 },
//    { .boxId = BOXGTUNE, .boxName = "GTUNE", .permanentId = 21 }, (removed)
//    { .boxId = BOXRANGEFINDER, .boxName = "RANGEFINDER", .permanentId = 22 }, (removed)
    { .boxId = BOXSERVO1, .boxName = "SERVO1", .permanentId = 23 },
    { .boxId = BOXSERVO2, .boxName = "SERVO2", .permanentId = 24 },
    { .boxId = BOXSERVO3, .boxName = "SERVO3", .permanentId = 25 },
    { .boxId = BOXBLACKBOX, .boxName = "BLACKBOX", .permanentId = 26 },
    { .boxId = BOXFAILSAFE, .boxName = "FAILSAFE", .permanentId = 27 },
    { .boxId = BOXAIRMODE, .boxName = "AIR MODE", .permanentId = 28 },
    { .boxId = BOX3D, .boxName = "3D DISABLE / SWITCH", .permanentId = 29},
    { .boxId = BOXFPVANGLEMIX, .boxName = "FPV ANGLE MIX", .permanentId = 30},
    { .boxId = BOXBLACKBOXERASE, .boxName = "BLACKBOX ERASE (>30s)", .permanentId = 31 },
    { .boxId = BOXCAMERA1, .boxName = "CAMERA CONTROL 1", .permanentId = 32},
    { .boxId = BOXCAMERA2, .boxName = "CAMERA CONTROL 2", .permanentId = 33},
    { .boxId = BOXCAMERA3, .boxName = "CAMERA CONTROL 3", .permanentId = 34 },
    { .boxId = BOXFLIPOVERAFTERCRASH, .boxName = "FLIP OVER AFTER CRASH", .permanentId = 35 },
    { .boxId = BOXPREARM, .boxName = "PREARM", .permanentId = 36 },
    { .boxId = BOXBEEPGPSCOUNT, .boxName = "GPS BEEP SATELLITE COUNT", .permanentId = 37 },
//    { .boxId = BOX3DONASWITCH, .boxName = "3D ON A SWITCH", .permanentId = 38 }, (removed)
    { .boxId = BOXVTXPITMODE, .boxName = "VTX PIT MODE", .permanentId = 39 },
    { .boxId = BOXUSER1, .boxName = "USER1", .permanentId = 40 },
    { .boxId = BOXUSER2, .boxName = "USER2", .permanentId = 41 },
    { .boxId = BOXUSER3, .boxName = "USER3", .permanentId = 42 },
    { .boxId = BOXUSER4, .boxName = "USER4", .permanentId = 43 },
    { .boxId = BOXPIDAUDIO, .boxName = "PID AUDIO", .permanentId = 44 },
    { .boxId = BOXPARALYZE, .boxName = "PARALYZE", .permanentId = 45 },
    { .boxId = BOXGPSRESCUE, .boxName = "GPS RESCUE", .permanentId = 46 },
    { .boxId = BOXACROTRAINER, .boxName = "ACRO TRAINER", .permanentId = 47 },
    { .boxId = BOXVTXCONTROLDISABLE, .boxName = "VTX CONTROL DISABLE", .permanentId = 48},
    { .boxId = BOXLAUNCHCONTROL, .boxName = "LAUNCH CONTROL", .permanentId = 49 },
    { .boxId = BOXMSPOVERRIDE, .boxName = "MSP OVERRIDE", .permanentId = 50},
    { .boxId = BOXSTICKCOMMANDDISABLE, .boxName = "STICK COMMANDS DISABLE", .permanentId = 51},
    { .boxId = BOXBEEPERMUTE, .boxName = "BEEPER MUTE", .permanentId = 52},
    { .boxId = BOXREADY, .boxName = "READY", .permanentId = 53}
};

// mask of enabled IDs, calculated on startup based on enabled features. boxId_e is used as bit index

static boxBitmask_t activeBoxIds;

const box_t *findBoxByBoxId(boxId_e boxId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->boxId == boxId)
            return candidate;
    }
    return NULL;
}

const box_t *findBoxByPermanentId(uint8_t permanentId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->permanentId == permanentId)
            return candidate;
    }
    return NULL;
}

static bool activeBoxIdGet(boxId_e boxId)
{
    if (boxId > sizeof(activeBoxIds) * 8) {
        return false;
    }

    return bitArrayGet(&activeBoxIds, boxId);
}

void serializeBoxNameFn(sbuf_t *dst, const box_t *box)
{
#if defined(USE_CUSTOM_BOX_NAMES)
    if (box->boxId == BOXUSER1 && strlen(modeActivationConfig()->box_user_1_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_1_name);
    } else if (box->boxId == BOXUSER2 && strlen(modeActivationConfig()->box_user_2_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_2_name);
    } else if (box->boxId == BOXUSER3 && strlen(modeActivationConfig()->box_user_3_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_3_name);
    } else if (box->boxId == BOXUSER4 && strlen(modeActivationConfig()->box_user_4_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_4_name);
    } else
#endif
    {
        sbufWriteString(dst, box->boxName);
    }
    sbufWriteU8(dst, ';');
}

void serializeBoxPermanentIdFn(sbuf_t *dst, const box_t *box)
{
    sbufWriteU8(dst, box->permanentId);
}

// serialize 'page' of boxNames.
// Each page contains at most 32 boxes
void serializeBoxReply(sbuf_t *dst, int page, serializeBoxFn *serializeBox)
{
    unsigned boxIdx = 0;
    unsigned pageStart = page * 32;
    unsigned pageEnd = pageStart + 32;
    for (boxId_e id = 0; id < CHECKBOX_ITEM_COUNT; id++) {
        if (activeBoxIdGet(id)) {
            if (boxIdx >= pageStart && boxIdx < pageEnd) {
                (*serializeBox)(dst, findBoxByBoxId(id));
            }
            boxIdx++;                 // count active boxes
        }
    }
}

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and set corresponding activeBoxIds bits
    boxBitmask_t ena;  // temporary variable to collect result
    memset(&ena, 0, sizeof(ena));

    // macro to enable boxId (BoxidMaskEnable). Reference to ena is hidden, local use only
#define BME(boxId) do { bitArraySet(&ena, boxId); } while (0)
    BME(BOXARM);
    BME(BOXPREARM);
    if (!featureIsEnabled(FEATURE_AIRMODE)) {
        BME(BOXAIRMODE);
    }

    bool acceleratorGainsEnabled = false;
    for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
        if (pidProfiles(i)->anti_gravity_gain != ITERM_ACCELERATOR_GAIN_OFF) {
            acceleratorGainsEnabled = true;
        }
    }
    if (acceleratorGainsEnabled && !featureIsEnabled(FEATURE_ANTI_GRAVITY)) {
        BME(BOXANTIGRAVITY);
    }

    if (sensors(SENSOR_ACC)) {
        BME(BOXANGLE);
        BME(BOXHORIZON);
        BME(BOXHEADFREE);
        BME(BOXHEADADJ);
        BME(BOXFPVANGLEMIX);
        if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
            BME(BOXCALIB);
        }
#if defined(USE_ACRO_TRAINER) && defined(USE_ACC)
        BME(BOXACROTRAINER);
#endif // USE_ACRO_TRAINER
    }

#ifdef USE_MAG
    if (sensors(SENSOR_MAG)) {
        BME(BOXMAG);
    }
#endif

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
#ifdef USE_GPS_RESCUE
        if (!featureIsEnabled(FEATURE_3D) && !isFixedWing()) {
            BME(BOXGPSRESCUE);
        }
#endif
        BME(BOXBEEPGPSCOUNT);
    }
#endif

    BME(BOXFAILSAFE);

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE || mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE) {
        BME(BOXPASSTHRU);
    }

    BME(BOXBEEPERON);
    BME(BOXBEEPERMUTE);

#ifdef USE_LED_STRIP
    if (featureIsEnabled(FEATURE_LED_STRIP)) {
        BME(BOXLEDLOW);
    }
#endif

#ifdef USE_BLACKBOX
    BME(BOXBLACKBOX);
#ifdef USE_FLASHFS
    BME(BOXBLACKBOXERASE);
#endif
#endif

    if (featureIsEnabled(FEATURE_3D)) {
        BME(BOX3D);
    }

#ifdef USE_DSHOT
    bool configuredMotorProtocolDshot;
    checkMotorProtocolEnabled(&motorConfig()->dev, &configuredMotorProtocolDshot);
    if (configuredMotorProtocolDshot) {
        BME(BOXFLIPOVERAFTERCRASH);
    }
#endif

    if (featureIsEnabled(FEATURE_SERVO_TILT)) {
        BME(BOXCAMSTAB);
    }

    BME(BOXOSD);

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        BME(BOXTELEMETRY);
    }
#endif

#ifdef USE_SERVOS
    if (mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE) {
        BME(BOXSERVO1);
        BME(BOXSERVO2);
        BME(BOXSERVO3);
    }
#endif

#ifdef USE_RCDEVICE
    BME(BOXCAMERA1);
    BME(BOXCAMERA2);
    BME(BOXCAMERA3);
#endif

#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP) || defined(USE_VTX_MSP)
    BME(BOXVTXPITMODE);
    BME(BOXVTXCONTROLDISABLE);
#endif

    BME(BOXPARALYZE);

#ifdef USE_PINIOBOX
    // Turn BOXUSERx only if pinioBox facility monitors them, as the facility is the only BOXUSERx observer.
    // Note that pinioBoxConfig can be set to monitor any box.
    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxConfig()->permanentId[i] != PERMANENT_ID_NONE) {
            const box_t *box = findBoxByPermanentId(pinioBoxConfig()->permanentId[i]);
            if (box) {
                switch(box->boxId) {
                case BOXUSER1:
                case BOXUSER2:
                case BOXUSER3:
                case BOXUSER4:
                    BME(box->boxId);
                    break;
                default:
                    break;
                }
            }
        }
    }
#endif

#if defined(USE_PID_AUDIO)
    BME(BOXPIDAUDIO);
#endif

#ifdef USE_LAUNCH_CONTROL
    BME(BOXLAUNCHCONTROL);
#endif

#if defined(USE_RX_MSP_OVERRIDE)
    if (rxConfig()->msp_override_channels_mask) {
        BME(BOXMSPOVERRIDE);
    }
#endif

    BME(BOXSTICKCOMMANDDISABLE);
    BME(BOXREADY);

#undef BME
    // check that all enabled IDs are in boxes array (check may be skipped when using findBoxById() functions)
    for (boxId_e boxId = 0;  boxId < CHECKBOX_ITEM_COUNT; boxId++)
        if (bitArrayGet(&ena, boxId)
            && findBoxByBoxId(boxId) == NULL)
            bitArrayClr(&ena, boxId);                 // this should not happen, but handle it gracefully

    activeBoxIds = ena;                               // set global variable
}

// return state of given boxId box, handling ARM and FLIGHT_MODE
bool getBoxIdState(boxId_e boxid)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
    STATIC_ASSERT(ARRAYLEN(boxIdToFlightModeMap) == BOXID_FLIGHTMODE_LAST + 1, FLIGHT_MODE_BOXID_MAP_INITIALIZER_does_not_match_boxId_e);

    if (boxid == BOXARM) {
        return ARMING_FLAG(ARMED);
    } else if (boxid <= BOXID_FLIGHTMODE_LAST) {
        return FLIGHT_MODE(1 << boxIdToFlightModeMap[boxid]);
    } else {
        return IS_RC_MODE_ACTIVE(boxid);
    }
}

// pack used flightModeFlags into supplied array
// returns number of bits used
int packFlightModeFlags(boxBitmask_t *mspFlightModeFlags)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    memset(mspFlightModeFlags, 0, sizeof(boxBitmask_t));
    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    unsigned mspBoxIdx = 0;           // index of active boxId (matches sent permanentId and boxNames)
    for (boxId_e boxId = 0; boxId < CHECKBOX_ITEM_COUNT; boxId++) {
        if (activeBoxIdGet(boxId)) {
            if (getBoxIdState(boxId))
                bitArraySet(mspFlightModeFlags, mspBoxIdx);       // box is enabled
            mspBoxIdx++;                                          // box is active, count it
        }
    }
    // return count of used bits
    return mspBoxIdx;
}
