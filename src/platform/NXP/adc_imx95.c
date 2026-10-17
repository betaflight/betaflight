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

/*
 * SAR ADC. The inputs are dedicated balls with no pad mux, so the board picks them with ADC_<source>_CHANNEL
 * and adcConfig ioTags are unused. adcGetValue() collects a software started chain over every input and
 * starts the next. The 12 bit result spans 0 to the 1.8 V reference.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC

#include "common/utils.h"

#include "drivers/adc.h"

#include "pg/adc.h"

#include "platform/adc_impl.h"
#include "platform/nxp_clock.h"

#define ADC_INPUT_NONE          0xffu

#ifndef ADC_VBAT_CHANNEL
#define ADC_VBAT_CHANNEL        ADC_INPUT_NONE
#endif
#ifndef ADC_CURR_CHANNEL
#define ADC_CURR_CHANNEL        ADC_INPUT_NONE
#endif
#ifndef ADC_EXTERNAL1_CHANNEL
#define ADC_EXTERNAL1_CHANNEL   ADC_INPUT_NONE
#endif
#ifndef ADC_RSSI_CHANNEL
#define ADC_RSSI_CHANNEL        ADC_INPUT_NONE
#endif

#define ADC_INPUT_VALID(input)  ((input) < ADC_CDR0_COUNT || (input) == ADC_INPUT_NONE)

STATIC_ASSERT(ADC_INPUT_VALID(ADC_VBAT_CHANNEL), adc_vbat_channel_is_not_a_sar_adc_input);
STATIC_ASSERT(ADC_INPUT_VALID(ADC_CURR_CHANNEL), adc_curr_channel_is_not_a_sar_adc_input);
STATIC_ASSERT(ADC_INPUT_VALID(ADC_EXTERNAL1_CHANNEL), adc_external1_channel_is_not_a_sar_adc_input);
STATIC_ASSERT(ADC_INPUT_VALID(ADC_RSSI_CHANNEL), adc_rssi_channel_is_not_a_sar_adc_input);

static const uint8_t adcInputs[ADC_EXTERNAL_COUNT] = {
    [ADC_BATTERY] = ADC_VBAT_CHANNEL,
    [ADC_CURRENT] = ADC_CURR_CHANNEL,
    [ADC_EXTERNAL1] = ADC_EXTERNAL1_CHANNEL,
    [ADC_RSSI] = ADC_RSSI_CHANNEL,
};

static uint16_t adcValues[ADC_EXTERNAL_COUNT];

// The inputs of the chain, as NCMR0 bits; zero when the ADC is not running.
static uint32_t adcChannelMask;

// Power up and calibration take tens of microseconds; this bounds the wait.
#define ADC_WAIT_LOOPS          100000

// MSR[ADCSTATUS] values.
#define ADC_STATUS_IDLE         0
#define ADC_STATUS_POWER_DOWN   1

// Wait for the converter to reach a state with no calibration or chain busy.
static bool adcWaitStatus(uint32_t status)
{
    for (unsigned i = 0; i < ADC_WAIT_LOOPS; i++) {
        const uint32_t msr = ADC->MSR;
        if (!(msr & (ADC_MSR_CALBUSY_MASK | ADC_MSR_NSTART_MASK))
            && (msr & ADC_MSR_ADCSTATUS_MASK) == status) {
            return true;
        }
    }

    return false;
}

static void adcStartChain(void)
{
    ADC->ISR = ADC_ISR_ECH_MASK | ADC_ISR_EOC_MASK;
    ADC->CEOCFR0 = adcChannelMask;
    ADC->NCMR0 = adcChannelMask;
    ADC->MCR |= ADC_MCR_NSTART_MASK;
}

void adcInit(const adcConfig_t *config)
{
    UNUSED(config);

    adcChannelMask = 0;

    for (unsigned source = 0; source < ARRAYLEN(adcInputs); source++) {
        if (adcInputs[source] < ADC_CDR0_COUNT) {
            adcChannelMask |= 1u << adcInputs[source];
        }
    }

    if (!adcChannelMask) {
        return;
    }

    if (!nxpClockGateEnable(NXP_CLK_ADC1)) {
        adcChannelMask = 0;
        return;
    }

    // MODE, ADCLKSE and OWREN apply only while powered down: one shot chains, AD_CLK at half the bus clock,
    // right aligned data, newest result kept.
    uint32_t mcr = ADC->MCR;
    mcr &= ~(ADC_MCR_MODE_MASK | ADC_MCR_ADCLKSE_MASK | ADC_MCR_WLSIDE_MASK | ADC_MCR_ABORTCHAIN_MASK
             | ADC_MCR_ABORT_MASK | ADC_MCR_ACKO_MASK | ADC_MCR_NSTART_MASK);
    mcr |= ADC_MCR_OWREN_MASK | ADC_MCR_PWDN_MASK;
    ADC->MCR = mcr;

    ADC->IMR = 0;
    ADC->CIMR0 = 0;

    if (!adcWaitStatus(ADC_STATUS_POWER_DOWN)) {
        adcChannelMask = 0;
        return;
    }

    ADC->MCR = mcr & ~ADC_MCR_PWDN_MASK;

    if (!adcWaitStatus(ADC_STATUS_IDLE)) {
        adcChannelMask = 0;
        return;
    }

    ADC->MCR |= ADC_MCR_CALSTART_MASK;

    if (!adcWaitStatus(ADC_STATUS_IDLE)) {
        adcChannelMask = 0;
        return;
    }

    adcStartChain();
}

// Collects a finished chain and starts the next, or restarts a chain that never started.
static void adcUpdate(void)
{
    if (!adcChannelMask) {
        return;
    }

    // MSR first: a chain that ends between the two reads then shows ECH.
    const bool running = ADC->MSR & ADC_MSR_NSTART_MASK;

    if (ADC->ISR & ADC_ISR_ECH_MASK) {
        for (unsigned source = 0; source < ARRAYLEN(adcInputs); source++) {
            if (adcInputs[source] >= ADC_CDR0_COUNT) {
                continue;
            }

            const uint32_t data = ADC->PCDR[adcInputs[source]];
            if (data & ADC_PCDR_VALID_MASK) {
                adcValues[source] = data & ADC_PCDR_CDATA_MASK;
            }
        }
        adcStartChain();
    } else if (!running) {
        adcStartChain();
    }
}

uint16_t adcGetValue(adcSource_e source)
{
#ifdef USE_ADC_INTERNAL
    // The core temperature comes from the System Manager, not an analog input.
    if (source == ADC_TEMPSENSOR) {
        return adcInternalTemperatureRaw();
    }
#endif

    if ((unsigned)source >= ARRAYLEN(adcValues)) {
        return 0;
    }

    adcUpdate();

    return adcValues[source];
}

#endif // USE_ADC
