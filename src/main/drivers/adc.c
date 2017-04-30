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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"

#include "drivers/io.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "common/utils.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

#ifndef VBAT_ADC_INSTANCE
#define VBAT_ADC_INSTANCE           ADC_INSTANCE
#endif
#ifndef RSSI_ADC_INSTANCE
#define RSSI_ADC_INSTANCE           ADC_INSTANCE
#endif
#ifndef CURRENT_METER_ADC_INSTANCE
#define CURRENT_METER_ADC_INSTANCE  ADC_INSTANCE
#endif
#ifndef EXTERNAL1_ADC_INSTANCE
#define EXTERNAL1_ADC_INSTANCE      ADC_INSTANCE
#endif
#ifndef AIRSPEED_ADC_INSTANCE
#define AIRSPEED_ADC_INSTANCE       ADC_INSTANCE
#endif

#ifdef USE_ADC

adc_config_t adcConfig[ADC_CHANNEL_COUNT];
volatile uint16_t adcValues[ADCDEV_COUNT][ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
    }
    return 0;
}

uint16_t adcGetChannel(uint8_t channel)
{
    if (adcConfig[channel].adcDevice != ADCINVALID && adcConfig[channel].enabled) {
        return adcValues[adcConfig[channel].adcDevice][adcConfig[channel].dmaIndex];
    } else {
        return 0;
    }
}

void adcInit(drv_adc_config_t *init)
{
    memset(&adcConfig, 0, sizeof(adcConfig));

#ifdef VBAT_ADC_PIN
    if (init->enableVBat) {
        adcConfig[ADC_BATTERY].adcDevice = adcDeviceByInstance(VBAT_ADC_INSTANCE);
        if (adcConfig[ADC_BATTERY].adcDevice != ADCINVALID) {
            adcConfig[ADC_BATTERY].tag = IO_TAG(VBAT_ADC_PIN);
        }
    }
#endif

#ifdef RSSI_ADC_PIN
    if (init->enableRSSI) {
        adcConfig[ADC_RSSI].adcDevice = adcDeviceByInstance(RSSI_ADC_INSTANCE);
        if (adcConfig[ADC_RSSI].adcDevice != ADCINVALID) {
            adcConfig[ADC_RSSI].tag = IO_TAG(RSSI_ADC_PIN);
        }
    }
#endif

#ifdef CURRENT_METER_ADC_PIN
    if (init->enableCurrentMeter) {
        adcConfig[ADC_CURRENT].adcDevice = adcDeviceByInstance(CURRENT_METER_ADC_INSTANCE);
        if (adcConfig[ADC_CURRENT].adcDevice != ADCINVALID) {
            adcConfig[ADC_CURRENT].tag = IO_TAG(CURRENT_METER_ADC_PIN);
        }
    }
#endif

#ifdef EXTERNAL1_ADC_PIN
    if (init->enableExternal1) {
        adcConfig[ADC_EXTERNAL1].adcDevice = adcDeviceByInstance(EXTERNAL1_ADC_INSTANCE);
        if (adcConfig[ADC_EXTERNAL1].adcDevice != ADCINVALID) {
            adcConfig[ADC_EXTERNAL1].tag = IO_TAG(EXTERNAL1_ADC_PIN);
        }
    }
#endif

#ifdef AIRSPEED_ADC_PIN
    if (init->enableAirSpeed) {
        adcConfig[ADC_AIRSPEED].adcDevice = adcDeviceByInstance(AIRSPEED_ADC_INSTANCE);
        if (adcConfig[ADC_AIRSPEED].adcDevice != ADCINVALID) {
            adcConfig[ADC_AIRSPEED].tag = IO_TAG(AIRSPEED_ADC_PIN);
        }
    }
#endif

    adcHardwareInit(init);
}

#else

uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

#endif
