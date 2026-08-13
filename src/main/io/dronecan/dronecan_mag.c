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

#if ENABLE_DRONECAN && defined(USE_MAG)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_mag.h"
#include "io/dronecan/dronecan_msg.h"

// Bit offsets into the MagneticFieldStrength2 payload. sensor_id then the
// fixed-length float16[3] field vector; the trailing float16[<=9] covariance
// is a TAO tail we don't decode. bit_length values that cross byte boundaries
// are why we lean on canardDecodeScalar() rather than a packed struct cast.
// The legacy MagneticFieldStrength carries the same float16[3] vector with
// no sensor_id, so its fields sit 8 bits earlier.
#define MAG2_OFFSET_SENSOR_ID   0U    //  8 bits unsigned
#define MAG2_OFFSET_FIELD_X     8U    // 16 bits float16
#define MAG2_OFFSET_FIELD_Y     24U   // 16 bits float16
#define MAG2_OFFSET_FIELD_Z     40U   // 16 bits float16
#define MAG_OFFSET_FIELD_X      0U    // 16 bits float16

#define GAUSS_TO_MILLIGAUSS     1000.0f

// Seqlock-style publication so a reader on the compass task can never observe a
// partially-written vector. Same discipline as dronecan_gnss.c: writer bumps
// the sequence odd before the copy and even after, reader retries on an odd or
// mismatched count.
static int16_t latest[XYZ_AXIS_COUNT];
static volatile uint32_t latestSeq = 0;
static volatile bool received = false;
static volatile timeUs_t lastUpdateUs = 0;

// First-seen sensor_id is latched so a multi-mag node's other sensors are
// ignored. Only touched from the handler (single task context), so it needs no
// seqlock.
static bool sensorIdLatched = false;
static uint8_t latchedSensorId = 0;

static void publishField(const CanardRxTransfer *t, uint32_t fieldBitOffset)
{
    uint16_t fieldX_f16 = 0;
    uint16_t fieldY_f16 = 0;
    uint16_t fieldZ_f16 = 0;

    canardDecodeScalar(t, fieldBitOffset,       16, false, &fieldX_f16);
    canardDecodeScalar(t, fieldBitOffset + 16U, 16, false, &fieldY_f16);
    canardDecodeScalar(t, fieldBitOffset + 32U, 16, false, &fieldZ_f16);

    const float fieldX = canardConvertFloat16ToNativeFloat(fieldX_f16);
    const float fieldY = canardConvertFloat16ToNativeFloat(fieldY_f16);
    const float fieldZ = canardConvertFloat16ToNativeFloat(fieldZ_f16);

    latestSeq++;
    __asm volatile ("" ::: "memory");

    latest[X] = (int16_t)constrainf(fieldX * GAUSS_TO_MILLIGAUSS, INT16_MIN, INT16_MAX);
    latest[Y] = (int16_t)constrainf(fieldY * GAUSS_TO_MILLIGAUSS, INT16_MIN, INT16_MAX);
    latest[Z] = (int16_t)constrainf(fieldZ * GAUSS_TO_MILLIGAUSS, INT16_MIN, INT16_MAX);

    lastUpdateUs = micros();
    received = true;

    __asm volatile ("" ::: "memory");
    latestSeq++;
}

static void handleMag2(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    uint8_t sensorId = 0;
    canardDecodeScalar(t, MAG2_OFFSET_SENSOR_ID, 8, false, &sensorId);

    if (!sensorIdLatched) {
        latchedSensorId = sensorId;
        sensorIdLatched = true;
    } else if (sensorId != latchedSensorId) {
        return;
    }

    publishField(t, MAG2_OFFSET_FIELD_X);
}

static void handleMagLegacy(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    publishField(t, MAG_OFFSET_FIELD_X);
}

void dronecanMagInit(void)
{
    memset(latest, 0, sizeof(latest));
    received = false;
    lastUpdateUs = 0;
    sensorIdLatched = false;
    latchedSensorId = 0;

    const dronecanSubscriber_t mag2Sub = {
        .signature    = UAVCAN_MAG2_SIGNATURE,
        .dataTypeId   = UAVCAN_MAG2_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleMag2,
    };
    (void)dronecanRegisterSubscriber(&mag2Sub);

    const dronecanSubscriber_t magLegacySub = {
        .signature    = UAVCAN_MAG_SIGNATURE,
        .dataTypeId   = UAVCAN_MAG_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleMagLegacy,
    };
    (void)dronecanRegisterSubscriber(&magLegacySub);
}

bool dronecanMagGetLatest(int16_t mag[XYZ_AXIS_COUNT])
{
    if (!received || mag == NULL) {
        return false;
    }

    uint32_t s1;
    uint32_t s2;
    do {
        do {
            s1 = latestSeq;
        } while (s1 & 1U);
        __asm volatile ("" ::: "memory");
        mag[X] = latest[X];
        mag[Y] = latest[Y];
        mag[Z] = latest[Z];
        __asm volatile ("" ::: "memory");
        s2 = latestSeq;
    } while (s1 != s2);

    return true;
}

timeUs_t dronecanMagLastUpdateUs(void)
{
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

#endif // ENABLE_DRONECAN && USE_MAG
