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
#include "drivers/dma.h"
#include "platform/dma.h"

#include "pg/adc.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "common/utils.h"   // popcount, llog2
#include "common/maths.h"   // MAX/MIN macros

#define PICO_ADC_MAX_CHANNELS           4
#define PICO_ADC_FIFO_SIZE              8

#if defined(RP2350A)
#define PICO_ADC_CHANNEL_COUNT          5
#define PICO_ADC_INTERNAL_TEMP_CHANNEL  4
#elif defined(RP2350B)
#define PICO_ADC_CHANNEL_COUNT          9
#define PICO_ADC_INTERNAL_TEMP_CHANNEL  8
#else
#error "ADC not properly defined, perhaps incorrect PICO target?"
#endif

#if defined(USE_ADC_INTERNAL)
#warning "PICO: Internal temp/Vref are used for ADC padding for DMA only - not intended for actual use"
#endif

#if ADC_SOURCE_COUNT > PICO_ADC_MAX_CHANNELS
#warning "PICO currently only supports maximum of 4 ADC channels"
#endif

typedef struct adcOperatingConfig_s {
    ioTag_t tag;
    uint8_t channel;  // hardware channel number for this input.
    uint8_t dmaIndex;    // location in ADC values for this input.
    bool enabled;
} adcOperatingConfig_t;

static adcOperatingConfig_t adcOperatingConfig[PICO_ADC_MAX_CHANNELS];
// NOTE the ring buffer is not using the last two positions, and wrapping to
// a position up to two sizes before set write address. Hence the padding.
static volatile uint16_t adcValues[PICO_ADC_MAX_CHANNELS] __attribute__((aligned(PICO_ADC_MAX_CHANNELS * sizeof(uint16_t))));

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

static inline bool is_adc_busy(void)
{
    // The READY bit is 0 while a conversion is in progress.
    return !(adc_hw->cs & ADC_CS_READY_BITS);
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
    for (unsigned i = 0; i < ARRAYLEN(adcOperatingConfig); i++) {
        if (!adcOperatingConfig[i].tag) {
            continue;
        }

        const int pin = DEFIO_TAG_PIN(adcOperatingConfig[i].tag);
        const int channel = adcChannelByPin(pin);

        if (channel < 0) {
            continue;
        }

        adcOperatingConfig[i].channel = channel;
        adcOperatingConfig[i].enabled = true;
        // dmaIndex will be assigned later based on channel rank within 'mask' (which may differ from enum order)

        adc_gpio_init(pin);
        mask |= (1u << channel);
    }

    if (popcount(mask) == 0) {
        /* don't enable the adc/dma - exit immediately */
        return;
    }

    if (popcount(mask) == 3) {
        // padding required due to DMA ring buffer supporting 2^X
        // enable the internal temp sensor to add one more reading.
        adc_set_temp_sensor_enabled(true);
        mask |= (1u << PICO_ADC_INTERNAL_TEMP_CHANNEL);
    }

    // empty the ADC fifo for safety
    adc_fifo_drain();

    // set the round robin for ADC
    adc_set_round_robin(mask);

    // select the first channel in the mask (it is ordered)
    adc_select_input(llog2(mask & -mask));

    // Map each enabled source to its DMA slot by channel rank (ascending channel order)
    // Mask must be populated entirely to find the correct order.
    for (unsigned i = 0; i < ARRAYLEN(adcOperatingConfig); i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        const uint8_t channel = adcOperatingConfig[i].channel;
        const unsigned rank = popcount(mask & ((1u << channel) - 1u)); // number of lower channels set
        adcOperatingConfig[i].dmaIndex = (uint8_t)(rank);
    }

    // Write each completed conversion to the sample FIFO
    // Enable DMA data request (DREQ)
    // Fire DREQ when FIFO has at least 1 sample
    // Ignore ERR bit, save masking the result (as it is 12 bit).
    // Don't shift each sample to 8 bits when pushing to FIFO
    adc_fifo_setup(true, true, 1, false, false);

    /*
        Conversion requires 96 cycles per sample, at ADC clock of 48MHz.
        clkdiv divides ADC clock to generate next auto trigger event

        clkdiv = (48,000,000 / (sample_rate)) - 1

        Sample rate required at 10hz for X channels = 10 x X

        65535.f + 255.f / 256.f for as slow as possible
        0 for as fast as possible
    */
    adc_set_clkdiv(65535);

    const dmaIdentifier_e dma_id = dmaGetFreeIdentifier();
    if (dma_id == DMA_NONE || !dmaAllocate(dma_id, OWNER_ADC, 0)) {
        return; // No free DMA channel available
    }
    const uint8_t dmaChannel = DMA_IDENTIFIER_TO_CHANNEL(dma_id);

    // --- DMA Setup ---
    dma_channel_config cfg = dma_channel_get_default_config(dmaChannel);

    // Configure DMA to read from the ADC FIFO and write to our buffer.
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false); // Read from the same address (ADC FIFO)
    channel_config_set_write_increment(&cfg, true); // Write to sequential addresses into adcValues
    channel_config_set_dreq(&cfg, DREQ_ADC);

    /*
        Setup the ring  buffer, 2^X so need 1, 2 and 4 positions

        channelCount results in:
        1 - 1 channel, no padding required
        2 - 2 channels, no padding required
        3 - 4 channels needed (due to ring buffer) so padding required if only 3 channels active
        4 - 4 channels available - no padding required.

        size_bits is the number of bytes = (1 << sizebits)
        1 channel  => size bits = 1 (2 byte ring required i.e. 1 << 1)
        2 channels => size bits = 2 (4 byte ring required i.e. 1 << 2)
        4 channels => size bits = 3 (8 byte ring required i.e. 1 << 3)

        We will never have 3 channels as 6 bytes is unavailable in the ring buffer,
        hence the padding with the internal temp sensor to prevent misalignment.
    */

    // NOTE fixing at size_bits = 3 (8 bytes) - for consistency in timing
    // i.e. ignoring "repeated" readings when less than 4 channels enabled
    // and it gives equivalent sample rate - regardless of enabled channels.
    channel_config_set_ring(&cfg, true, llog2(PICO_ADC_MAX_CHANNELS * sizeof(uint16_t)));

    // Set the DMA into free running mode and start
    // -1 for transfer count is endless mode
    dma_channel_configure(dmaChannel, &cfg, adcValues, &adc_hw->fifo, -1, true);

    /* ADC start, in round robin - continuous */
    adc_run(true);
}

uint16_t adcGetValue(adcSource_e source)
{
    if ((unsigned)source >= ARRAYLEN(adcOperatingConfig) || !adcOperatingConfig[source].enabled) {
        return 0;
    }

    const uint8_t dmaIndex = adcOperatingConfig[source].dmaIndex;
    return adcValues[dmaIndex];
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
