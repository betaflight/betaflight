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

// Core temperature for the i.MXRT 11xx from TMPSNS, converted as in the SDK's
// fsl_tempsensor.c. TMPSNS sits behind the VDDLPSR analog interface, so the addresses
// below are offsets, and a transfer is done when the toggle write retires.

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC_INTERNAL

#include "common/utils.h"

#include "drivers/adc.h"

#include "platform/adc_impl.h"

#define TMPSNS_AI_CTRL1                 offsetof(TMPSNS_Type, CTRL1)
#define TMPSNS_AI_STATUS0               offsetof(TMPSNS_Type, STATUS0)

// The repeat period, in the block's own units.
#define TMPSNS_MEASURE_PERIOD           32u

// Coefficients of the quadratic count to temperature curve; the 25 degree count
// comes from the trim word.
#define TMPSNS_CAL_A                    133.6f
#define TMPSNS_CAL_B                    (-5.39f)
#define TMPSNS_CAL_C                    0.002f

// Report busy until the first measurement lands; the sensor reads zero before.
static bool sensorReady;

static uint32_t tmpsnsAiRead(uint32_t addr)
{
    ANADIG_MISC->VDDLPSR_AI_CTRL = ANADIG_MISC_VDDLPSR_AI_CTRL_VDDLPSR_AIRWB_MASK
                                 | ANADIG_MISC_VDDLPSR_AI_CTRL_VDDLPSR_AI_ADDR(addr);

    ANADIG_TEMPSENSOR->TEMPSENSOR ^= ANADIG_TEMPSENSOR_TEMPSENSOR_TEMPSNS_AI_TOGGLE_MASK;

    return ANADIG_MISC->VDDLPSR_AI_RDATA_TMPSNS;
}

static void tmpsnsAiWrite(uint32_t addr, uint32_t wdata)
{
    ANADIG_MISC->VDDLPSR_AI_CTRL = ANADIG_MISC_VDDLPSR_AI_CTRL_VDDLPSR_AI_ADDR(addr);
    ANADIG_MISC->VDDLPSR_AI_WDATA = wdata;

    ANADIG_TEMPSENSOR->TEMPSENSOR ^= ANADIG_TEMPSENSOR_TEMPSENSOR_TEMPSNS_AI_TOGGLE_MASK;
}

void adcInternalStartConversion(void)
{
    uint32_t ctrl1 = tmpsnsAiRead(TMPSNS_AI_CTRL1);

    ctrl1 &= ~(TMPSNS_CTRL1_PWD_MASK | TMPSNS_CTRL1_PWD_FULL_MASK | TMPSNS_CTRL1_FREQ_MASK);

    // FINISH_IE is needed for the finished flag; the NVIC interrupt stays off.
    ctrl1 |= TMPSNS_CTRL1_FREQ(TMPSNS_MEASURE_PERIOD)
           | TMPSNS_CTRL1_FINISH_IE_MASK
           | TMPSNS_CTRL1_START_MASK;

    tmpsnsAiWrite(TMPSNS_AI_CTRL1, ctrl1);
}

bool adcInternalIsBusy(void)
{
    if (!sensorReady) {
        adcInternalStartConversion();
        sensorReady = (tmpsnsAiRead(TMPSNS_AI_STATUS0) & TMPSNS_STATUS0_FINISH_MASK) != 0;
    }

    return !sensorReady;
}

uint16_t adcInternalTemperatureRaw(void)
{
    return (tmpsnsAiRead(TMPSNS_AI_STATUS0) & TMPSNS_STATUS0_TEMP_VAL_MASK) >> TMPSNS_STATUS0_TEMP_VAL_SHIFT;
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    UNUSED(vrefValue);

    // The count at 25 degrees, from the trim.
    const uint32_t cal25 = (ANADIG_TEMPSENSOR->TEMPSNS_OTP_TRIM_VALUE
                            & ANADIG_TEMPSENSOR_TEMPSNS_OTP_TRIM_VALUE_TEMPSNS_TEMP_VAL_MASK)
                           >> ANADIG_TEMPSENSOR_TEMPSNS_OTP_TRIM_VALUE_TEMPSNS_TEMP_VAL_SHIFT;

    // An untrimmed part has no 25 degree count.
    if (cal25 == 0) {
        return 0;
    }

    // Solve the quadratic; a negative discriminant means no valid reading.
    const float discriminant = TMPSNS_CAL_B * TMPSNS_CAL_B
                             - 4.0f * TMPSNS_CAL_C * (TMPSNS_CAL_A + (float)cal25 - (float)tempAdcValue);
    if (discriminant < 0.0f) {
        return 0;
    }

    return lrintf((-TMPSNS_CAL_B - sqrtf(discriminant)) / (2.0f * TMPSNS_CAL_C));
}

#endif // USE_ADC_INTERNAL
