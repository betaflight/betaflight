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

#include <platform.h>

#include "build/build_config.h"

#include "pitotmeter.h"
#include "pitotmeter_adc.h"
#include "adc.h"

#if defined(USE_PITOT_ADC)

/*
 * NXP MPXV7002DP differential pressure sensor
 *
 */
#define PITOT_ADC_VOLTAGE_SCALER        (2.0f / 1.0f)       // MPXV7002DP is 5V device, assumed resistive divider 1K:1K
#define PITOT_ADC_VOLTAGE_ZERO          (2.5f)              // Pressure offset is 2.5V
#define PITOT_ADC_VOLTAGE_TO_PRESSURE   (1000.0f)           // 1V/kPa = 1000 Pa/V

static void adcPitotStart(void)
{
}

static void adcPitotRead(void)
{
}

static void adcPitotCalculate(float *pressure, float *temperature)
{
    uint16_t adcRaw = adcGetChannel(ADC_AIRSPEED);
    float voltage = (float)adcRaw * (3.3f / 4095.0f);  // 12 bit ADC with 3.3V VREF

    if (pressure)
        *pressure = (voltage * PITOT_ADC_VOLTAGE_SCALER - PITOT_ADC_VOLTAGE_ZERO) * PITOT_ADC_VOLTAGE_TO_PRESSURE;
    if (temperature)
        *temperature = 288.15f;     // Temperature at standard sea level (288.15 K)
}

bool adcPitotDetect(pitotDev_t *pitot)
{
    pitot->delay = 10000;
    pitot->start = adcPitotStart;
    pitot->get = adcPitotRead;
    pitot->calculate = adcPitotCalculate;
    return adcIsFunctionAssigned(ADC_AIRSPEED);
}
#endif
