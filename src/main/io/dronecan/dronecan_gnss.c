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

#if ENABLE_DRONECAN

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

// Bit offsets into the Fix2 payload (from the DSDL definition). The header
// fields up to sub_mode are fixed — everything past that (covariance, pdop,
// ecef) is variable-length with TAO rules, so we stop at sub_mode until we
// need those fields. bit_length values that cross byte boundaries are why
// we lean on canardDecodeScalar() rather than a packed struct cast.
#define FIX2_OFFSET_LONGITUDE_1E8   136U   // 37 bits signed
#define FIX2_OFFSET_LATITUDE_1E8    173U   // 37 bits signed
#define FIX2_OFFSET_HEIGHT_MSL_MM   237U   // 27 bits signed
#define FIX2_OFFSET_VEL_N_F16       264U   // 16 bits float16
#define FIX2_OFFSET_VEL_E_F16       280U   // 16 bits float16
#define FIX2_OFFSET_VEL_D_F16       296U   // 16 bits float16
#define FIX2_OFFSET_SATS_USED       312U   //  6 bits unsigned
#define FIX2_OFFSET_STATUS          318U   //  2 bits unsigned

#define CM_PER_METRE                100

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

    int64_t lon_1e8 = 0;
    int64_t lat_1e8 = 0;
    int32_t height_msl_mm = 0;
    uint16_t velN_f16 = 0;
    uint16_t velE_f16 = 0;
    uint16_t velD_f16 = 0;
    uint8_t satsUsed = 0;
    uint8_t status = 0;

    canardDecodeScalar(t, FIX2_OFFSET_LONGITUDE_1E8,  37, true,  &lon_1e8);
    canardDecodeScalar(t, FIX2_OFFSET_LATITUDE_1E8,   37, true,  &lat_1e8);
    canardDecodeScalar(t, FIX2_OFFSET_HEIGHT_MSL_MM,  27, true,  &height_msl_mm);
    canardDecodeScalar(t, FIX2_OFFSET_VEL_N_F16,      16, false, &velN_f16);
    canardDecodeScalar(t, FIX2_OFFSET_VEL_E_F16,      16, false, &velE_f16);
    canardDecodeScalar(t, FIX2_OFFSET_VEL_D_F16,      16, false, &velD_f16);
    canardDecodeScalar(t, FIX2_OFFSET_SATS_USED,       6, false, &satsUsed);
    canardDecodeScalar(t, FIX2_OFFSET_STATUS,          2, false, &status);

    const float velN = canardConvertFloat16ToNativeFloat(velN_f16);
    const float velE = canardConvertFloat16ToNativeFloat(velE_f16);
    const float velD = canardConvertFloat16ToNativeFloat(velD_f16);

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

    memset(&latest, 0, sizeof(latest));
    latest.llh.lat   = scaleLonLat_1e8to1e7(lat_1e8);
    latest.llh.lon   = scaleLonLat_1e8to1e7(lon_1e8);
    latest.llh.altCm = height_msl_mm / 10; // mm -> cm
    // NED -> ENU on the ned_velocity reporting shortcut: N/E components map
    // one-to-one for the horizontal speed + course we care about.
    latest.velned.velN = (int16_t)constrainf(velN * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.velned.velE = (int16_t)constrainf(velE * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.velned.velD = (int16_t)constrainf(velD * CM_PER_METRE, INT16_MIN, INT16_MAX);
    latest.groundSpeed = (uint16_t)constrainf(speed2d * CM_PER_METRE, 0, UINT16_MAX);
    latest.speed3d     = (uint16_t)constrainf(speed3d * CM_PER_METRE, 0, UINT16_MAX);
    latest.groundCourse = (uint16_t)constrainf(courseDeg * 10.0f, 0, UINT16_MAX);
    latest.numSat      = satsUsed;

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

void dronecanGnssInit(void)
{
    const dronecanSubscriber_t sub = {
        .signature    = UAVCAN_GNSS_FIX2_SIGNATURE,
        .dataTypeId   = UAVCAN_GNSS_FIX2_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleFix2,
    };
    (void)dronecanRegisterSubscriber(&sub);

    memset(&latest, 0, sizeof(latest));
    received = false;
    lastUpdateUs = 0;
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

#endif // ENABLE_DRONECAN
