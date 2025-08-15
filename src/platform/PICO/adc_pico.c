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
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"

#include "pg/adc.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "common/utils.h"   // popcount, llog2
#include "common/maths.h"   // MAX/MIN macros

#if defined(RP2350)
#define PICO_ADC_FIFO_SIZE 8
#else
#error "ADC FIFO Size not defined"
#endif

#if defined(RP2350A)
#define PICO_ADC_CHANNEL_COUNT          5
#define PICO_ADC_INTERNAL_TEMP_CHANNEL  4
#elif defined(RP2350B)
#define PICO_ADC_CHANNEL_COUNT          9
#define PICO_ADC_INTERNAL_TEMP_CHANNEL  8
#else
#error "Internal ADC channels not defined"
#endif

typedef struct adcOperatingConfig_s {
    ioTag_t tag;
    uint8_t channel;  // hardware channel number for this input.
    uint8_t dmaIndex; // location in ADC values for this input.
    bool enabled;
} adcOperatingConfig_t;

static adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
static volatile uint16_t adcValues[PICO_ADC_CHANNEL_COUNT];

static int adcChannelByPin(const int pin)
{
#ifdef RP2350A
    if (pin >= 26 && pin <= 29) {
        return pin - 26;
    }
#elif defined(RP2350B)
    if (pin >= 40 && pin <= 47) {
        return pin - 40;
    }
#else
    UNUSED(pin);
#endif
    return -1;
}

void adcInit(const adcConfig_t *config)
{
    adc_init();

    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));

    // Set channels (ioTags) for enabled ADC inputs
    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
    }

    uint32_t mask = 0u;

    // loop over all possible channels and build the adcOperatingConfig to represent
    // the set of enabled channels
    uint8_t dmaIndex = 0;
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        adcOperatingConfig[i].enabled = false;
        do {
#ifdef USE_ADC_INTERNAL
            if (i >= ADC_SOURCE_INTERNAL_FIRST_ID) {
                if (i == ADC_TEMPSENSOR) {
                    adcOperatingConfig[i].channel = PICO_ADC_INTERNAL_TEMP_CHANNEL;
                    adcOperatingConfig[i].enabled = true;
                    adc_set_temp_sensor_enabled(true);
                }
                break;
            }
#endif
            if (!adcOperatingConfig[i].tag) {
                break;
            }

            const int pin = DEFIO_TAG_PIN(adcOperatingConfig[i].tag);
            const int channel = adcChannelByPin(pin);
            if (channel >= 0) {
                adcOperatingConfig[i].channel = channel;
                adc_gpio_init(pin);
                adcOperatingConfig[i].enabled = true;
            }
        } while (false);

        if (!adcOperatingConfig[i].enabled) {
            continue;
        }

        mask |= (1u << adcOperatingConfig[i].channel);
        adcOperatingConfig[i].dmaIndex = dmaIndex;
        dmaIndex++;
    }

    const unsigned channelCount = popcount(mask);
    if (channelCount == 0) {
        /* don't enable the interrupt */
        return;
    }

    const int firstEnabled = llog2(mask & -mask);
    adc_select_input(firstEnabled);

    adc_set_round_robin(mask);
    adc_fifo_setup(
        true,  // Write each completed conversion to the sample FIFO
        true,  // Enable DMA data request (DREQ)
        1,     // Fire DREQ when FIFO has at least 1 sample
        false, // We won't see the ERR bit because of 12-bit reads; disable
        false  // Don't shift each sample to 8 bits when pushing to FIFO
    );

    /*
        Sampling requires 96 cycles per sample
        clkdiv = (48,000,000 / (sample_rate * 96)) - 1

        Sample rate required at 10hz for X channels = 10 x X
    */
    const float clkdiv = (48e6f / (channelCount * 10.f * 96.f)) - 1;
    adc_set_clkdiv(clkdiv);

    const dmaIdentifier_e dma_id = dmaGetFreeIdentifier();
    if (dma_id == DMA_NONE) {
        return; // No free DMA channel available
    }

    if (!dmaAllocate(dma_id, OWNER_ADC, 0)) {
        return;
    }

    // --- DMA Setup ---
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Configure DMA to read from the ADC FIFO and write to our buffer.
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false); // Read from the same address (ADC FIFO)
    channel_config_set_write_increment(&cfg, true); // Write to sequential addresses in our buffer
    channel_config_set_dreq(&cfg, DREQ_ADC);
    // Set ring buffer behavior: after writing the last element, wrap back to the start.
    // The size is 2^X = Z half-words
    // note ADC handles the reset, the ring is never triggered
    const unsigned sizeBits = 32 - __builtin_clz(ARRAYLEN(adcValues) - 1);
    channel_config_set_ring(&cfg, true, sizeBits);

    dma_channel_configure(
        dma_chan,
        &cfg,
        adcValues,         // Destination pointer
        &adc_hw->fifo,     // Source pointer
        -1,                // RP2350 has endless mode
        true               // Start immediately
    );

    /* start the ADC in free running mode */
    adc_run(true);
}

uint16_t adcGetValue(adcSource_e source)
{
    if ((unsigned)source >= ARRAYLEN(adcOperatingConfig) || !adcOperatingConfig[source].enabled) {
        return 0;
    }
    const uint8_t index = adcOperatingConfig[source].dmaIndex;
    return adcValues[index];
}

#ifdef USE_ADC_INTERNAL

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    //NOOP
}

#endif // USE_ADC_INTERNAL
#endif // USE_ADC
