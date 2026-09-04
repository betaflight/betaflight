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

// ADC driver for the i.MXRT 10xx 12 bit SAR ADC. Writing HC0 starts a software triggered
// conversion, HS[COCO0] flags the result and reading R0 clears it. Conversions are polled
// one at a time, cycling over the enabled inputs.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/adc.h"

#include "platform/adc_impl.h"
#include "platform/nxp_clock.h"

#if !NXP_HAS_CLASSIC_ADC
#error "adc_imxrt10xx.c is for the classic ADC; an LPADC family needs its own file"
#endif

// No analog input on this pad for that instance.
#define ADC_CHANNEL_ABSENT      0xffu

// IPG / 2 / 8 is 9 MHz. Long sample with ADSTS 2 is 21 ADC clocks for a source behind a divider.
// REFSEL 0 is VREFH/VREFL. Four sample averaging at run time, 32 during calibration.
#define ADC_CFG_BASE            (ADC_CFG_ADICLK(1) | ADC_CFG_ADIV(3) | ADC_CFG_MODE(2) \
                                 | ADC_CFG_ADLSMP_MASK | ADC_CFG_ADSTS(2) | ADC_CFG_REFSEL(0))
#define ADC_CFG_RUN             (ADC_CFG_BASE | ADC_CFG_AVGS(0))
#define ADC_CFG_CALIBRATE       (ADC_CFG_BASE | ADC_CFG_AVGS(3))

// Calibration takes well under a millisecond; the bound catches a closed clock gate.
#define ADC_CALIBRATION_TIMEOUT_US  10000

// A conversion takes about 21 us; a result later than this is lost and the round robin moves on.
#define ADC_CONVERSION_TIMEOUT_US   500

typedef struct adcHardware_s {
    ADC_Type *reg;
    nxpClock_e clock;
} adcHardware_t;

static const adcHardware_t adcHardware[] = {
    { ADC1, NXP_CLK_ADC1 },
    { ADC2, NXP_CLK_ADC2 },
};

// ADC channel per GPIO1 (port A) pin and instance; GPIO_AD_B1_11 wraps to channel 0.
typedef struct adcPad_s {
    uint8_t pin;                // port A pin, which is the GPIO1 pin number
    uint8_t channel[2];         // ADCH value per instance, or ADC_CHANNEL_ABSENT
} adcPad_t;

static const adcPad_t adcPads[] = {
    { 12, { 1,  ADC_CHANNEL_ABSENT } },     // GPIO_AD_B0_12
    { 13, { 2,  ADC_CHANNEL_ABSENT } },     // GPIO_AD_B0_13
    { 14, { 3,  ADC_CHANNEL_ABSENT } },     // GPIO_AD_B0_14
    { 15, { 4,  ADC_CHANNEL_ABSENT } },     // GPIO_AD_B0_15
    { 16, { 5,  5  } },                     // GPIO_AD_B1_00
    { 17, { 6,  6  } },                     // GPIO_AD_B1_01
    { 18, { 7,  7  } },                     // GPIO_AD_B1_02
    { 19, { 8,  8  } },                     // GPIO_AD_B1_03
    { 20, { 9,  9  } },                     // GPIO_AD_B1_04
    { 21, { 10, 10 } },                     // GPIO_AD_B1_05
    { 22, { 11, 11 } },                     // GPIO_AD_B1_06
    { 23, { 12, 12 } },                     // GPIO_AD_B1_07
    { 24, { 13, 13 } },                     // GPIO_AD_B1_08
    { 25, { 14, 14 } },                     // GPIO_AD_B1_09
    { 26, { 15, 15 } },                     // GPIO_AD_B1_10
    { 27, { 0,  0  } },                     // GPIO_AD_B1_11
    { 28, { ADC_CHANNEL_ABSENT, 1 } },      // GPIO_AD_B1_12
    { 29, { ADC_CHANNEL_ABSENT, 2 } },      // GPIO_AD_B1_13
    { 30, { ADC_CHANNEL_ABSENT, 3 } },      // GPIO_AD_B1_14
    { 31, { ADC_CHANNEL_ABSENT, 4 } },      // GPIO_AD_B1_15
};

typedef struct adcInput_s {
    ADC_Type *reg;              // NULL when this source has no input
    uint8_t channel;
    uint16_t value;
} adcInput_t;

static adcInput_t adcInputs[ADC_EXTERNAL_COUNT];

// The enabled sources, in the order the round robin visits them.
static uint8_t adcSequence[ADC_EXTERNAL_COUNT];
static uint8_t adcSequenceLength;
static uint8_t adcSequencePosition;

// Find the instance and channel a pad reaches, preferring ADC1 when both can
// read the pad.
static bool adcFindChannel(ioTag_t tag, unsigned *instance, uint8_t *channel)
{
    if (!tag || DEFIO_TAG_GPIOID(tag) != 0) {
        return false;
    }

    const unsigned pin = DEFIO_TAG_PIN(tag);

    for (unsigned i = 0; i < ARRAYLEN(adcPads); i++) {
        if (adcPads[i].pin != pin) {
            continue;
        }
        for (unsigned dev = 0; dev < ARRAYLEN(adcHardware); dev++) {
            if (adcPads[i].channel[dev] != ADC_CHANNEL_ABSENT) {
                *instance = dev;
                *channel = adcPads[i].channel[dev];
                return true;
            }
        }
        return false;
    }

    return false;
}

// Runs after CFG is written and before any conversion, with the software trigger selected.
static bool adcCalibrate(ADC_Type *reg)
{
    reg->CFG = ADC_CFG_CALIBRATE;
    reg->GC = ADC_GC_AVGE_MASK;

    // CALF is write one to clear, so a failure left over from an earlier pass
    // does not get read back as this one failing.
    reg->GS = ADC_GS_CALF_MASK;

    reg->GC |= ADC_GC_CAL_MASK;

    const timeUs_t startUs = micros();
    while (reg->GC & ADC_GC_CAL_MASK) {
        if (reg->GS & ADC_GS_CALF_MASK) {
            break;
        }
        if (cmpTimeUs(micros(), startUs) > ADC_CALIBRATION_TIMEOUT_US) {
            return false;
        }
    }

    // CALF set, or no completed conversion, means the trim is not valid.
    if ((reg->GS & ADC_GS_CALF_MASK) || !(reg->HS & ADC_HS_COCO0_MASK)) {
        return false;
    }

    // Reading the result register clears the conversion complete flag.
    (void)reg->R[0];

    reg->CFG = ADC_CFG_RUN;
    reg->GC = ADC_GC_AVGE_MASK;

    return true;
}

// When the conversion in flight was started, so a lost one can be spotted.
static timeUs_t adcConversionStartUs;

static void adcStartConversion(void)
{
    const adcInput_t *input = &adcInputs[adcSequence[adcSequencePosition]];

    adcConversionStartUs = micros();

    // Writing the channel to the software trigger slot starts the conversion.
    // AIEN stays clear: nothing here runs from an interrupt.
    input->reg->HC[0] = ADC_HC_ADCH(input->channel);
}

// Move the round robin on if the conversion in flight has finished, and start
// the next one. Called from adcGetValue.
static void adcUpdate(void)
{
    if (!adcSequenceLength) {
        return;
    }

    adcInput_t *input = &adcInputs[adcSequence[adcSequencePosition]];

    if (!(input->reg->HS & ADC_HS_COCO0_MASK)) {
        // Nothing came back in time, so trigger the same channel again rather
        // than leave the round robin parked on it.
        if (cmpTimeUs(micros(), adcConversionStartUs) > ADC_CONVERSION_TIMEOUT_US) {
            adcStartConversion();
        }
        return;
    }

    input->value = input->reg->R[0] & ADC_R_CDATA_MASK;

    adcSequencePosition = (adcSequencePosition + 1) % adcSequenceLength;
    adcStartConversion();
}

void adcInit(const adcConfig_t *config)
{
    memset(adcInputs, 0, sizeof(adcInputs));
    adcSequenceLength = 0;
    adcSequencePosition = 0;

    ioTag_t tags[ADC_EXTERNAL_COUNT] = { 0 };

    if (config->vbat.enabled) {
        tags[ADC_BATTERY] = config->vbat.ioTag;
    }
    if (config->current.enabled) {
        tags[ADC_CURRENT] = config->current.ioTag;
    }
    if (config->external1.enabled) {
        tags[ADC_EXTERNAL1] = config->external1.ioTag;
    }
    if (config->rssi.enabled) {
        tags[ADC_RSSI] = config->rssi.ioTag;
    }

    // An instance is set up once however many inputs it feeds.
    bool wanted[ARRAYLEN(adcHardware)] = { false };
    bool ready[ARRAYLEN(adcHardware)] = { false };

    unsigned instance[ADC_EXTERNAL_COUNT] = { 0 };
    uint8_t channel[ADC_EXTERNAL_COUNT] = { 0 };

    for (unsigned source = 0; source < ARRAYLEN(tags); source++) {
        if (adcFindChannel(tags[source], &instance[source], &channel[source])) {
            wanted[instance[source]] = true;
        } else {
            tags[source] = IO_TAG_NONE;
        }
    }

    for (unsigned dev = 0; dev < ARRAYLEN(adcHardware); dev++) {
        if (!wanted[dev]) {
            continue;
        }

        // Calibration needs the clock before any register below takes effect.
        nxpClockGateEnable(adcHardware[dev].clock);

        ready[dev] = adcCalibrate(adcHardware[dev].reg);
    }

    for (unsigned source = 0; source < ARRAYLEN(tags); source++) {
        if (!tags[source] || !ready[instance[source]]) {
            continue;
        }

        // IOCFG_ANALOG turns the output driver and pull off.
        IO_t io = IOGetByTag(tags[source]);
        IOInit(io, OWNER_ADC_BATT + source, 0);
        IOConfigGPIO(io, IOCFG_ANALOG);

        adcInputs[source].reg = adcHardware[instance[source]].reg;
        adcInputs[source].channel = channel[source];

        adcSequence[adcSequenceLength++] = source;
    }

    if (adcSequenceLength) {
        adcStartConversion();
    }
}

uint16_t adcGetValue(adcSource_e source)
{
#ifdef USE_ADC_INTERNAL
    // The core temperature sensor is a block of its own, not an analog input.
    if (source == ADC_TEMPSENSOR) {
        return adcInternalTemperatureRaw();
    }
#endif

    if ((unsigned)source >= ARRAYLEN(adcInputs)) {
        return 0;
    }

    adcUpdate();

    return adcInputs[source].value;
}

#ifdef USE_ADC_INTERNAL

// Nominal VREFH; there is no band gap input to measure the reference against.
#define ADC_FULL_SCALE_MV       3300

uint16_t adcInternalCompensateVref(uint16_t intVRefAdcValue)
{
    UNUSED(intVRefAdcValue);

    return ADC_FULL_SCALE_MV;
}

#endif // USE_ADC_INTERNAL

#endif // USE_ADC
