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

#if ENABLE_DRONECAN && defined(USE_PITOT)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_airspeed.h"
#include "io/dronecan/dronecan_msg.h"

// Seqlock publication (same discipline as dronecan_mag.c): the handler runs on
// the dronecan task, readers on the pitot task. Writer bumps the sequence odd
// before the copy and even after; readers retry on an odd or mismatched count.
static float latestDiffPa;
static float latestStaticPa;
static float latestTempK;
static volatile uint32_t latestSeq = 0;
static volatile bool received = false;
static volatile timeUs_t lastUpdateUs = 0;

static float decodeFloat32(const CanardRxTransfer *t, uint32_t bitOffset)
{
    uint32_t raw = 0;
    canardDecodeScalar(t, bitOffset, 32, false, &raw);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void handleRawAirData(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    const float diffPa = decodeFloat32(t, RAWAIRDATA_OFFSET_DIFF_P);
    const float staticPa = decodeFloat32(t, RAWAIRDATA_OFFSET_STATIC_P);

    uint16_t tempF16 = 0;
    canardDecodeScalar(t, RAWAIRDATA_OFFSET_DIFF_TEMP, 16, false, &tempF16);
    float tempK = canardConvertFloat16ToNativeFloat(tempF16);
    if (tempK < 233.0f || tempK > 333.0f) {
        tempK = 0.0f;   // node did not supply a usable air temperature; 0 = unknown
    }

    latestSeq++;
    __asm volatile ("" ::: "memory");

    latestDiffPa = diffPa;
    latestStaticPa = staticPa;
    latestTempK = tempK;
    lastUpdateUs = micros();
    received = true;

    __asm volatile ("" ::: "memory");
    latestSeq++;
}

void dronecanAirspeedInit(void)
{
    latestDiffPa = 0.0f;
    latestStaticPa = 0.0f;
    latestTempK = 0.0f;
    received = false;
    lastUpdateUs = 0;

    const dronecanSubscriber_t sub = {
        .signature    = UAVCAN_RAWAIRDATA_SIGNATURE,
        .dataTypeId   = UAVCAN_RAWAIRDATA_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleRawAirData,
    };
    (void)dronecanRegisterSubscriber(&sub);
}

bool dronecanAirspeedGetLatest(float *diffPressurePa, float *staticPressurePa, float *temperatureK)
{
    if (!received) {
        return false;
    }

    uint32_t s1;
    uint32_t s2;
    float diffPa;
    float staticPa;
    float tempK;
    do {
        do {
            s1 = latestSeq;
        } while (s1 & 1U);
        __asm volatile ("" ::: "memory");
        diffPa = latestDiffPa;
        staticPa = latestStaticPa;
        tempK = latestTempK;
        __asm volatile ("" ::: "memory");
        s2 = latestSeq;
    } while (s1 != s2);

    if (diffPressurePa) {
        *diffPressurePa = diffPa;
    }
    if (staticPressurePa) {
        *staticPressurePa = staticPa;
    }
    if (temperatureK) {
        *temperatureK = tempK;
    }
    return true;
}

timeUs_t dronecanAirspeedLastUpdateUs(void)
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

#endif // ENABLE_DRONECAN && USE_PITOT
