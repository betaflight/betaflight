/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/adc.h"
#include "drivers/io.h"

#include "pg/adc.h"

PG_REGISTER_WITH_RESET_FN(adcConfig_t, adcConfig, PG_ADC_CONFIG, 0);

void pgResetFn_adcConfig(adcConfig_t *adcConfig)
{
#ifdef ADC_VBAT_PIN
    adcConfig->vbat.enabled = true;
    adcConfig->vbat.ioTag = IO_TAG(ADC_VBAT_PIN);
#endif

#ifdef ADC_EXTERNAL1_PIN
    adcConfig->external1.enabled = true;
    adcConfig->external1.ioTag = IO_TAG(ADC_EXTERNAL1_PIN);
#endif

#ifdef ADC_CURR_PIN
    adcConfig->current.enabled = true;
    adcConfig->current.ioTag = IO_TAG(ADC_CURR_PIN);
#endif

#ifdef ADC_RSSI_PIN
    adcConfig->rssi.enabled = true;
    adcConfig->rssi.ioTag = IO_TAG(ADC_RSSI_PIN);
#endif

#if PLATFORM_TRAIT_ADC_DEVICE
    platform_pgResetFn_adcConfig(adcConfig);
#endif

    adcConfig->vrefIntCalibration = 0;
    adcConfig->tempSensorCalibration1 = 0;
    adcConfig->tempSensorCalibration2 = 0;
}
#endif // USE_ADC
