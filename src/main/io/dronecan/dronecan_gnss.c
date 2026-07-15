/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if ENABLE_DRONECAN && defined(USE_GPS)

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_gnss.h"
#include "io/dronecan/dronecan_msg.h"

#define CM_PER_METRE                100

static float dronecanDecodeFloat16(const CanardRxTransfer *t, uint32_t bitOffset)
{
    uint16_t raw = 0;
    canardDecodeScalar(t, bitOffset, 16, false, &raw);
    return canardConvertFloat16ToNativeFloat(raw);
}

// Scale 1e8 degrees down to the betaflight 1e7 convention. Divide before
// the int32 cast so we don't overflow on high-latitude values.
static int32_t scaleLonLat_1e8to1e7(int64_t deg_1e8)
{
    return (int32_t)(deg_1e8 / 10);
}

// Seqlock-style publication so a reader on a different task can never observe
// a partially-written solution (the shared struct is ~40 bytes). Writer bumps
// the sequence before and after the copy: odd value = write in progress, even
// value = stable. Reader loads sequence, copies, reloads; retries if either
// read saw an odd count or the endpoints differ. Betaflight's scheduler is
// cooperative between tasks today, so the retry loop is defensive against a
// future preemptive scheduler rather than fixing a current race.
static gpsSolutionData_t latest;
static volatile uint32_t latestSeq = 0;
static volatile bool received = false;
static volatile timeUs_t lastUpdateUs = 0;

static void handleFix2(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    uint64_t gnssTimeUsec = 0;
    uint8_t timeStandard = 0;
    int64_t lon_1e8 = 0;
    int64_t lat_1e8 = 0;
    int32_t heightMslMm = 0;
    float velN = 0.0f;
    float velE = 0.0f;
    float velD = 0.0f;
    uint8_t satsUsed = 0;
    uint8_t status = 0;

    // Walk the payload as a running bit offset: the covariance/pdop tail is
    // variable-length, so it can't be reached with absolute offsets. Note
    // ned_velocity is float32[3] (96 bits) — a narrower read would misalign
    // every field that follows it.
    uint32_t bit = 0;
    bit += 56;                                                       // timestamp
    canardDecodeScalar(t, bit, 56, false, &gnssTimeUsec); bit += 56; // gnss_timestamp
    canardDecodeScalar(t, bit,  3, false, &timeStandard); bit += 3;
    bit += 13;                                                       // void13
    bit += 8;                                                        // num_leap_seconds
    canardDecodeScalar(t, bit, 37, true,  &lon_1e8);      bit += 37;
    canardDecodeScalar(t, bit, 37, true,  &lat_1e8);      bit += 37;
    bit += 27;                                                       // height_ellipsoid_mm
    canardDecodeScalar(t, bit, 27, true,  &heightMslMm);  bit += 27;
    canardDecodeScalar(t, bit, 32, false, &velN);         bit += 32;
    canardDecodeScalar(t, bit, 32, false, &velE);         bit += 32;
    canardDecodeScalar(t, bit, 32, false, &velD);         bit += 32;
    canardDecodeScalar(t, bit,  6, false, &satsUsed);     bit += 6;
    canardDecodeScalar(t, bit,  2, false, &status);       bit += 2;
    bit += 4;                                                        // mode
    bit += 6;                                                        // sub_mode

    uint32_t hAccMm = 0;
    uint32_t vAccMm = 0;
    uint32_t sAccMmS = 0;
    uint16_t pdop100 = 0;

    const uint32_t payloadBits = (uint32_t)t->payload_len * 8U;
    if (bit + 6U <= payloadBits) {
        uint8_t covLength = 0;
        canardDecodeScalar(t, bit, 6, false, &covLength); bit += 6;

        // Covariance is a 6x6 position/velocity matrix. Modules send either
        // the full row-major matrix (36) or just its diagonal (6); any other
        // length carries no usable per-axis variance.
        float diag[6];
        bool haveDiag = false;
        if (covLength == 6) {
            for (unsigned i = 0; i < 6; i++) {
                diag[i] = dronecanDecodeFloat16(t, bit + i * 16U);
            }
            haveDiag = true;
        } else if (covLength == 36) {
            for (unsigned i = 0; i < 6; i++) {
                diag[i] = dronecanDecodeFloat16(t, bit + (i * 6U + i) * 16U);
            }
            haveDiag = true;
        }

        if (haveDiag) {
            const float hVar = 0.5f * (diag[0] + diag[1]);
            const float vVar = diag[2];
            const float sVar = (diag[3] + diag[4] + diag[5]) / 3.0f;
            if (hVar > 0.0f) {
                hAccMm = (uint32_t)(sqrtf(hVar) * 1000.0f);
            }
            if (vVar > 0.0f) {
                vAccMm = (uint32_t)(sqrtf(vVar) * 1000.0f);
            }
            if (sVar > 0.0f) {
                sAccMmS = (uint32_t)(sqrtf(sVar) * 1000.0f);
            }
        }

        bit += (uint32_t)covLength * 16U;
        if (bit + 16U <= payloadBits) {
            const float pdop = dronecanDecodeFloat16(t, bit);
            if (!isnan(pdop) && pdop > 0.0f) {
                pdop100 = (uint16_t)constrainf(pdop * 100.0f, 0, UINT16_MAX);
            }
        }
    }

    gpsDateTime_t dateTime;
    memset(&dateTime, 0, sizeof(dateTime));
    // GPS/TAI stamps need leap-second bookkeeping to become a wall clock, so
    // only UTC is converted; other standards leave the calendar untouched.
    if (timeStandard == UAVCAN_GNSS_TIME_STANDARD_UTC && gnssTimeUsec != 0) {
        gpsUnixSecondsToDateTime(&dateTime,
                                 (int64_t)(gnssTimeUsec / 1000000ULL),
                                 (uint16_t)((gnssTimeUsec / 1000ULL) % 1000ULL));
        dateTime.valid = true;
    }

    const float speed2d = sqrtf(velN * velN + velE * velE);
    const float speed3d = sqrtf(velN * velN + velE * velE + velD * velD);
    float courseDeg = (speed2d > 0.01f) ? (atan2f(velE, velN) * (180.0f / M_PIf)) : 0.0f;
    if (courseDeg < 0.0f) {
        courseDeg += 360.0f;
    }

    // Enter write: mark the sequence odd before touching the struct, and a
    // release-style compiler fence so the reader can't see the incremented
    // sequence before the writes below.
    latestSeq++;
    __asm volatile ("" ::: "memory");

    // hdop/vdop arrive on the Auxiliary message; preserve them across the
    // reset so a Fix2 update doesn't wipe the last DOP figures.
    const uint16_t hdop = latest.dop.hdop;
    const uint16_t vdop = latest.dop.vdop;

    memset(&latest, 0, sizeof(latest));
    latest.llh.lat   = scaleLonLat_1e8to1e7(lat_1e8);
    latest.llh.lon   = scaleLonLat_1e8to1e7(lon_1e8);
    latest.llh.altCm = heightMslMm / 10; // mm -> cm
    // Fix2 reports NED and gpsVelned_t is NED, so forward N/E/D unchanged
    // (velD is Down, positive downward).
    latest.velned.velN = (int16_t)constrainf(velN * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.velned.velE = (int16_t)constrainf(velE * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.velned.velD = (int16_t)constrainf(velD * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.groundSpeed = (uint16_t)constrainf(speed2d * CM_PER_METRE, 0, UINT16_MAX);
    latest.speed3d     = (uint16_t)constrainf(speed3d * CM_PER_METRE, 0, UINT16_MAX);
    latest.groundCourse = (uint16_t)constrainf(courseDeg * 10.0f, 0, UINT16_MAX);
    latest.numSat      = satsUsed;
    latest.acc.hAcc    = hAccMm;
    latest.acc.vAcc    = vAccMm;
    latest.acc.sAcc    = sAccMmS;
    latest.dop.pdop    = pdop100;
    latest.dop.hdop    = hdop;
    latest.dop.vdop    = vdop;
    latest.dateTime    = dateTime;

    // Signal a loss of fix explicitly so the downstream state can react
    // rather than stay armed on the last position.
    if (status < UAVCAN_GNSS_FIX2_STATUS_3D_FIX) {
        latest.numSat = 0;
    }

    lastUpdateUs = micros();
    received = true;

    // Leave write: fence before the final bump so everything above is visible.
    __asm volatile ("" ::: "memory");
    latestSeq++;
}

static void handleAuxiliary(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    // Field order is gdop, pdop, hdop, vdop, ...; only hdop/vdop are taken —
    // pdop stays sourced from Fix2 so both providers scale it identically.
    // Unknown DOP values are transmitted as NaN.
    const float hdop = dronecanDecodeFloat16(t, 32);
    const float vdop = dronecanDecodeFloat16(t, 48);

    latestSeq++;
    __asm volatile ("" ::: "memory");

    if (!isnan(hdop) && hdop > 0.0f) {
        latest.dop.hdop = (uint16_t)constrainf(hdop * 100.0f, 0, UINT16_MAX);
    }
    if (!isnan(vdop) && vdop > 0.0f) {
        latest.dop.vdop = (uint16_t)constrainf(vdop * 100.0f, 0, UINT16_MAX);
    }

    __asm volatile ("" ::: "memory");
    latestSeq++;
}

void dronecanGnssInit(void)
{
    // Clear the cache before registering so a frame that lands between
    // dronecanRegisterSubscriber() and the reset can't be silently clobbered.
    memset(&latest, 0, sizeof(latest));
    received = false;
    lastUpdateUs = 0;

    const dronecanSubscriber_t fix2Sub = {
        .signature    = UAVCAN_GNSS_FIX2_SIGNATURE,
        .dataTypeId   = UAVCAN_GNSS_FIX2_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleFix2,
    };
    (void)dronecanRegisterSubscriber(&fix2Sub);

    const dronecanSubscriber_t auxSub = {
        .signature    = UAVCAN_GNSS_AUXILIARY_SIGNATURE,
        .dataTypeId   = UAVCAN_GNSS_AUXILIARY_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleAuxiliary,
    };
    (void)dronecanRegisterSubscriber(&auxSub);
}

bool dronecanGnssGetLatest(gpsSolutionData_t *out)
{
    if (!received || out == NULL) {
        return false;
    }

    // Seqlock-style read: spin until the sequence is even (writer quiescent)
    // so we never copy a mid-update struct, then confirm the count was
    // stable across the copy. Splitting the wait into its own inner loop
    // avoids the UB of comparing an uninitialised s2 on the first iteration
    // if the writer happens to be mid-update when we first sample the
    // sequence. Task-context access converges in one or two iterations.
    uint32_t s1;
    uint32_t s2;
    do {
        do {
            s1 = latestSeq;
        } while (s1 & 1U);
        __asm volatile ("" ::: "memory");
        *out = latest;
        __asm volatile ("" ::: "memory");
        s2 = latestSeq;
    } while (s1 != s2);

    return true;
}

timeUs_t dronecanGnssLastUpdateUs(void)
{
    // timeUs_t is 64-bit on targets with USE_64BIT_TIME (e.g. SITL), so a
    // plain read can tear against a concurrent writer. Use the same seqlock
    // pattern as dronecanGnssGetLatest() to stay consistent.
    uint32_t s1;
    uint32_t s2;
    timeUs_t t;
    do {
        do {
            s1 = latestSeq;
        } while (s1 & 1U);
        __asm volatile ("" ::: "memory");
        t = lastUpdateUs;
        __asm volatile ("" ::: "memory");
        s2 = latestSeq;
    } while (s1 != s2);
    return t;
}

#endif // ENABLE_DRONECAN && USE_GPS
