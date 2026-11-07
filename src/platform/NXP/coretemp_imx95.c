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

// Core temperature for the i.MX95 from the System Manager's temp_ana sensor, read
// in hundredths of a degree without waiting for the System Manager; a failed enable or read reports zero.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC_INTERNAL

#include "common/utils.h"

#include "drivers/adc.h"

#include "platform/adc_impl.h"

#include "scmi_imx95.h"

// temp_ana in the System Manager configuration.
#define CORETEMP_SENSOR_ID      0

static bool sensorAsked;
static bool sensorEnabled;
static int16_t coreTemperature;
static uint16_t lastCentiCelsius;

bool adcInternalIsBusy(void)
{
    return scmiSensorReadBusy();
}

// Posts the read and returns; the next task run collects the reply.
void adcInternalStartConversion(void)
{
    if (!sensorAsked) {
        sensorAsked = true;
        sensorEnabled = scmiSensorEnable(CORETEMP_SENSOR_ID) == SCMI_SUCCESS;
    }

    if (sensorEnabled) {
        scmiSensorReadStart(CORETEMP_SENSOR_ID);
    }
}

uint16_t adcInternalTemperatureRaw(void)
{
    int64_t centiCelsius;
    const int32_t status = sensorEnabled ? scmiSensorReadResult(&centiCelsius) : SCMI_DENIED;

    // Nothing to collect keeps the last reading.
    if (status == SCMI_NOTHING_PENDING) {
        return lastCentiCelsius;
    }

    // Readings outside absolute zero to 1000 degrees are not a temperature.
    if (status != SCMI_SUCCESS || centiCelsius < -27315 || centiCelsius > 100000) {
        coreTemperature = 0;
        lastCentiCelsius = 0;
        return 0;
    }

    coreTemperature = lrintf((float)centiCelsius / 100);
    lastCentiCelsius = (uint16_t)centiCelsius;
    return lastCentiCelsius;
}

// The latest reading, not the average, so a failed read is never mixed in.
int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    UNUSED(tempAdcValue);
    UNUSED(vrefValue);

    return coreTemperature;
}

uint16_t adcInternalCompensateVref(uint16_t intVRefAdcValue)
{
    UNUSED(intVRefAdcValue);

    return ADC_VOLTAGE_REFERENCE_MV;
}

#endif // USE_ADC_INTERNAL
