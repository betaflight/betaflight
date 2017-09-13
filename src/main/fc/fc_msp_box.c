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

#include "platform.h"

#include "common/bitarray.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/fc_msp_box.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"

#include "sensors/sensors.h"

#include "telemetry/telemetry.h"


#ifndef USE_OSD_SLAVE
// permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
static const box_t boxes[CHECKBOX_ITEM_COUNT] = {
    { BOXARM, "ARM", 0 },
    { BOXANGLE, "ANGLE", 1 },
    { BOXHORIZON, "HORIZON", 2 },
    { BOXBARO, "BARO", 3 },
    { BOXANTIGRAVITY, "ANTI GRAVITY", 4 },
    { BOXMAG, "MAG", 5 },
    { BOXHEADFREE, "HEADFREE", 6 },
    { BOXHEADADJ, "HEADADJ", 7 },
    { BOXCAMSTAB, "CAMSTAB", 8 },
    { BOXCAMTRIG, "CAMTRIG", 9 },
    { BOXGPSHOME, "GPS HOME", 10 },
    { BOXGPSHOLD, "GPS HOLD", 11 },
    { BOXPASSTHRU, "PASSTHRU", 12 },
    { BOXBEEPERON, "BEEPER", 13 },
    { BOXLEDMAX, "LEDMAX", 14 },
    { BOXLEDLOW, "LEDLOW", 15 },
    { BOXLLIGHTS, "LLIGHTS", 16 },
    { BOXCALIB, "CALIB", 17 },
    { BOXGOV, "GOVERNOR", 18 },
    { BOXOSD, "OSD DISABLE SW", 19 },
    { BOXTELEMETRY, "TELEMETRY", 20 },
    { BOXGTUNE, "GTUNE", 21 },
    { BOXSONAR, "SONAR", 22 },
    { BOXSERVO1, "SERVO1", 23 },
    { BOXSERVO2, "SERVO2", 24 },
    { BOXSERVO3, "SERVO3", 25 },
    { BOXBLACKBOX, "BLACKBOX", 26 },
    { BOXFAILSAFE, "FAILSAFE", 27 },
    { BOXAIRMODE, "AIR MODE", 28 },
    { BOX3DDISABLE, "DISABLE 3D", 29},
    { BOXFPVANGLEMIX, "FPV ANGLE MIX", 30},
    { BOXBLACKBOXERASE, "BLACKBOX ERASE (>30s)", 31 },
    { BOXCAMERA1, "CAMERA CONTROL 1", 32},
    { BOXCAMERA2, "CAMERA CONTROL 2", 33},
    { BOXCAMERA3, "CAMERA CONTROL 3", 34 },
    { BOXFLIPOVERAFTERCRASH, "FLIP OVER AFTER CRASH (DSHOT ONLY)", 35 },
    { BOXPREARM, "PREARM", 36 },
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
    if (boxId > sizeof(activeBoxIds) * 8)
        return false;
    return bitArrayGet(&activeBoxIds, boxId);
}

void serializeBoxNameFn(sbuf_t *dst, const box_t *box)
{
    sbufWriteString(dst, box->boxName);
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
    if (!feature(FEATURE_AIRMODE)) {
        BME(BOXAIRMODE);
    }

    if (!feature(FEATURE_ANTI_GRAVITY)) {
        BME(BOXANTIGRAVITY);
    }

    if (sensors(SENSOR_ACC)) {
        BME(BOXANGLE);
        BME(BOXHORIZON);
        BME(BOXHEADFREE);
        BME(BOXHEADADJ);
    }

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        BME(BOXBARO);
    }
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        BME(BOXMAG);
    }
#endif

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        BME(BOXGPSHOME);
        BME(BOXGPSHOLD);
    }
#endif

#ifdef SONAR
    if (feature(FEATURE_SONAR)) {
        BME(BOXSONAR);
    }
#endif

    BME(BOXFAILSAFE);

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE || mixerConfig()->mixerMode == MIXER_CUSTOM_AIRPLANE) {
        BME(BOXPASSTHRU);
    }

    BME(BOXBEEPERON);

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        BME(BOXLEDLOW);
    }
#endif

#ifdef BLACKBOX
    BME(BOXBLACKBOX);
#ifdef USE_FLASHFS
    BME(BOXBLACKBOXERASE);
#endif
#endif

    BME(BOXFPVANGLEMIX);

    if (feature(FEATURE_3D)) {
        BME(BOX3DDISABLE);
    }

    if (isMotorProtocolDshot()) {
        BME(BOXFLIPOVERAFTERCRASH);
    }

    if (feature(FEATURE_SERVO_TILT)) {
        BME(BOXCAMSTAB);
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        BME(BOXCALIB);
    }

    BME(BOXOSD);

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY) && telemetryConfig()->telemetry_switch) {
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

#ifdef USE_RCSPLIT
    BME(BOXCAMERA1);
    BME(BOXCAMERA2);
    BME(BOXCAMERA3);
#endif

#undef BME
    // check that all enabled IDs are in boxes array (check may be skipped when using findBoxById() functions)
    for (boxId_e boxId = 0;  boxId < CHECKBOX_ITEM_COUNT; boxId++)
        if (bitArrayGet(&ena, boxId)
            && findBoxByBoxId(boxId) == NULL)
            bitArrayClr(&ena, boxId);                 // this should not happen, but handle it gracefully

    activeBoxIds = ena;                               // set global variable
}

// pack used flightModeFlags into supplied array
// returns number of bits used
int packFlightModeFlags(boxBitmask_t *mspFlightModeFlags)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    memset(mspFlightModeFlags, 0, sizeof(boxBitmask_t));

    // enabled BOXes, bits indexed by boxId_e
    boxBitmask_t boxEnabledMask;
    memset(&boxEnabledMask, 0, sizeof(boxEnabledMask));

    // enable BOXes dependent on FLIGHT_MODE, use mapping table (from runtime_config.h)
    // flightMode_boxId_map[HORIZON_MODE] == BOXHORIZON
    static const int8_t flightMode_boxId_map[] = FLIGHT_MODE_BOXID_MAP_INITIALIZER;
    flightModeFlags_e flightModeCopyMask = ~0;  // only modes with bit set will be copied
    for (unsigned i = 0; i < ARRAYLEN(flightMode_boxId_map); i++) {
        if (flightMode_boxId_map[i] != -1        // boxId_e does exist for this FLIGHT_MODE
           && (flightModeCopyMask & (1 << i))   // this flightmode is copy is enabled
           && FLIGHT_MODE(1 << i)) {            // this flightmode is active
            bitArraySet(&boxEnabledMask, flightMode_boxId_map[i]);
        }
    }

    // enable BOXes dependent on rcMode bits, indexes are the same.
    // only subset of BOXes depend on rcMode, use mask to select them
#define BM(x) (1ULL << (x))
    // limited to 64 BOXes now to keep code simple
    const uint64_t rcModeCopyMask = BM(BOXHEADADJ) | BM(BOXCAMSTAB) | BM(BOXCAMTRIG) | BM(BOXBEEPERON)
        | BM(BOXLEDMAX) | BM(BOXLEDLOW) | BM(BOXLLIGHTS) | BM(BOXCALIB) | BM(BOXGOV) | BM(BOXOSD)
        | BM(BOXTELEMETRY) | BM(BOXGTUNE) | BM(BOXBLACKBOX) | BM(BOXBLACKBOXERASE) | BM(BOXAIRMODE)
        | BM(BOXANTIGRAVITY) | BM(BOXFPVANGLEMIX) | BM(BOXFLIPOVERAFTERCRASH) | BM(BOX3DDISABLE);
    STATIC_ASSERT(sizeof(rcModeCopyMask) * 8 >= CHECKBOX_ITEM_COUNT, copy_mask_too_small_for_boxes);
    for (unsigned i = 0; i < CHECKBOX_ITEM_COUNT; i++) {
        if ((rcModeCopyMask & BM(i))    // mode copy is enabled
           && IS_RC_MODE_ACTIVE(i)) {    // mode is active
            bitArraySet(&boxEnabledMask, i);
        }
    }
#undef BM
    // copy ARM state
    if (ARMING_FLAG(ARMED))
        bitArraySet(&boxEnabledMask, BOXARM);

    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    unsigned mspBoxIdx = 0;           // index of active boxId (matches sent permanentId and boxNames)
    for (boxId_e boxId = 0; boxId < CHECKBOX_ITEM_COUNT; boxId++) {
        if (activeBoxIdGet(boxId)) {
            if (bitArrayGet(&boxEnabledMask,  boxId))
                bitArraySet(mspFlightModeFlags,  mspBoxIdx);      // box is enabled
            mspBoxIdx++;                                          // box is active, count it
        }
    }
    // return count of used bits
    return mspBoxIdx;
}
#endif // USE_OSD_SLAVE
