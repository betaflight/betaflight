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

// Core temperature for the i.MXRT 10xx family from the TEMPMON block, for the
// temperature functions sensors/adcinternal.c calls. The OCOTP trim word holds
// a hot temperature with its count and the room temperature count; the count
// is linear between them. Fields and conversion follow the SDK's fsl_tempmon.c.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC_INTERNAL

#include "common/utils.h"

#include "drivers/adc.h"

#include "platform/adc_impl.h"

#define TEMPMON_CAL_HOT_TEMP_SHIFT      0
#define TEMPMON_CAL_HOT_TEMP_MASK       0xffu
#define TEMPMON_CAL_HOT_COUNT_SHIFT     8
#define TEMPMON_CAL_HOT_COUNT_MASK      0xfffu
#define TEMPMON_CAL_ROOM_COUNT_SHIFT    20
#define TEMPMON_CAL_ROOM_COUNT_MASK     0xfffu

// The temperature of the room count, the same for every part.
#define TEMPMON_ROOM_TEMP               25.0f

// The repeat period in 32 kHz ticks, about a millisecond.
#define TEMPMON_MEASURE_PERIOD          32u

// Report busy until the first measurement lands; the sensor reads zero before.
static bool sensorReady;

void adcInternalStartConversion(void)
{
    // Safe to repeat while the block runs.
    TEMPMON->TEMPSENSE1 = TEMPMON_TEMPSENSE1_MEASURE_FREQ(TEMPMON_MEASURE_PERIOD);
    TEMPMON->TEMPSENSE0_CLR = TEMPMON_TEMPSENSE0_POWER_DOWN_MASK;
    TEMPMON->TEMPSENSE0_SET = TEMPMON_TEMPSENSE0_MEASURE_TEMP_MASK;
}

bool adcInternalIsBusy(void)
{
    if (!sensorReady) {
        adcInternalStartConversion();
        sensorReady = (TEMPMON->TEMPSENSE0 & TEMPMON_TEMPSENSE0_FINISHED_MASK) != 0;
    }

    return !sensorReady;
}

uint16_t adcInternalTemperatureRaw(void)
{
    return (TEMPMON->TEMPSENSE0 & TEMPMON_TEMPSENSE0_TEMP_CNT_MASK) >> TEMPMON_TEMPSENSE0_TEMP_CNT_SHIFT;
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    UNUSED(vrefValue);

    const uint32_t cal = OCOTP->ANA1;

    const float hotTemp = (cal >> TEMPMON_CAL_HOT_TEMP_SHIFT) & TEMPMON_CAL_HOT_TEMP_MASK;
    const int32_t hotCount = (cal >> TEMPMON_CAL_HOT_COUNT_SHIFT) & TEMPMON_CAL_HOT_COUNT_MASK;
    const int32_t roomCount = (cal >> TEMPMON_CAL_ROOM_COUNT_SHIFT) & TEMPMON_CAL_ROOM_COUNT_MASK;

    // An untrimmed part has a blank fuse word.
    const int32_t countSpan = roomCount - hotCount;
    if (countSpan <= 0) {
        return 0;
    }

    const float degrees = hotTemp
                        - ((float)tempAdcValue - hotCount) * ((hotTemp - TEMPMON_ROOM_TEMP) / countSpan);

    return lrintf(degrees);
}

#endif // USE_ADC_INTERNAL
