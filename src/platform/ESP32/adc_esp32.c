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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/adc_ll.h"
#pragma GCC diagnostic pop

#include "common/utils.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

void adcInit(const adcConfig_t *config)
{
    UNUSED(config);

    adc_ll_enable_bus_clock(true);

    // Configure ADC1 for 12-bit resolution
    adc_oneshot_ll_set_output_bits(ADC_UNIT_1, ADC_BITWIDTH_12);

    // Set attenuation for all channels to 11dB (0-3.3V range)
    for (int ch = 0; ch < 10; ch++) {
        adc_oneshot_ll_set_atten(ADC_UNIT_1, ch, ADC_ATTEN_DB_12);
    }

    adc_oneshot_ll_enable(ADC_UNIT_1);
}

uint16_t adcInternalReadVrefint(void)
{
    // ESP32-S3 uses eFuse-calibrated Vref, nominal 1100mV
    return 1100;
}

uint16_t adcInternalReadTempsensor(void)
{
    // ESP32-S3 has a built-in temperature sensor
    // For now, return a nominal room temperature value
    // TODO: implement using temperature_sensor_ll.h
    return 25;
}

#endif // USE_ADC
