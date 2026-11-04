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

// ADC driver for the two LPADC instances of the i.MXRT 11xx family. A software
// trigger runs CMD1 and RESFIFO returns the result with its valid bit set; an
// empty FIFO reads with the valid bit clear. Conversions are polled one at a
// time, cycling over the enabled inputs.

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

#if NXP_HAS_CLASSIC_ADC
#error "adc_imxrt11xx.c is for the LPADC; a classic ADC family needs its own file"
#endif

// No analog input on this pad for that instance.
#define ADC_INPUT_ABSENT        0xffu

// Commands are numbered from one in TCMD and CMDSRC but from zero in CMD[].
#define ADC_COMMAND             1u
#define ADC_COMMAND_INDEX       (ADC_COMMAND - 1u)

// PWREN and PUDLY at the SDK defaults, PWRSEL 3 (highest bias) for the 88 MHz
// clock, and REFSEL 0, the 1.825 V reference the board dividers assume.
#define ADC_CFG_SETUP           (ADC_CFG_PWREN_MASK | ADC_CFG_PUDLY(0x80) \
                                 | ADC_CFG_PWRSEL(3) | ADC_CFG_REFSEL(0))

// STS 6 samples for 67.5 ADC clocks (about 770 ns at 88 MHz) for resistive
// dividers; AVGS 2 averages four conversions.
#define ADC_CMDH_SETUP          (ADC_CMDH_STS(6) | ADC_CMDH_AVGS(2))

// The input scale must match IMXRT_ADC_FULL_SCALE_MV, both from adc_impl.h.
#define ADC_CMDL_SETUP          IMXRT_ADC_CMDL_SCALE

// A single ended 12 bit result sits in bits 14 down to 3.
#define ADC_RESULT_SHIFT        3

// A conversion takes about 4 us; a result later than this was lost.
#define ADC_CONVERSION_TIMEOUT_US   500

typedef struct adcHardware_s {
    ADC_Type *reg;
    nxpClock_e clock;
} adcHardware_t;

static const adcHardware_t adcHardware[] = {
    { LPADC1, NXP_CLK_ADC1 },
    { LPADC2, NXP_CLK_ADC2 },
};

// Input per port C pin and instance, as twice the channel plus one for side B.
typedef struct adcPad_s {
    uint8_t pin;                // port C pin, which is the GPIO3 pin number
    uint8_t input[2];           // channel and side per instance, or ADC_INPUT_ABSENT
} adcPad_t;

static const adcPad_t adcPads[] = {
    {  5, { 0,  ADC_INPUT_ABSENT } },       // GPIO_AD_06
    {  6, { 1,  ADC_INPUT_ABSENT } },       // GPIO_AD_07
    {  7, { 2,  ADC_INPUT_ABSENT } },       // GPIO_AD_08
    {  8, { 3,  ADC_INPUT_ABSENT } },       // GPIO_AD_09
    {  9, { 4,  ADC_INPUT_ABSENT } },       // GPIO_AD_10
    { 10, { 5,  ADC_INPUT_ABSENT } },       // GPIO_AD_11
    { 11, { 6,  6  } },                     // GPIO_AD_12
    { 12, { 7,  7  } },                     // GPIO_AD_13
    { 13, { 8,  8  } },                     // GPIO_AD_14
    { 14, { 9,  9  } },                     // GPIO_AD_15
    { 15, { 10, 10 } },                     // GPIO_AD_16
    { 16, { 11, 11 } },                     // GPIO_AD_17
    { 17, { ADC_INPUT_ABSENT, 0  } },       // GPIO_AD_18
    { 18, { ADC_INPUT_ABSENT, 1  } },       // GPIO_AD_19
    { 19, { ADC_INPUT_ABSENT, 2  } },       // GPIO_AD_20
    { 20, { ADC_INPUT_ABSENT, 3  } },       // GPIO_AD_21
    { 21, { ADC_INPUT_ABSENT, 4  } },       // GPIO_AD_22
    { 22, { ADC_INPUT_ABSENT, 5  } },       // GPIO_AD_23
    { 23, { ADC_INPUT_ABSENT, 12 } },       // GPIO_AD_24
    { 24, { ADC_INPUT_ABSENT, 13 } },       // GPIO_AD_25
};

typedef struct adcInput_s {
    ADC_Type *reg;              // NULL when this source has no input
    uint8_t input;
    uint16_t value;
} adcInput_t;

static adcInput_t adcInputs[ADC_EXTERNAL_COUNT];

// The enabled sources, in the order the round robin visits them.
static uint8_t adcSequence[ADC_EXTERNAL_COUNT];
static uint8_t adcSequenceLength;
static uint8_t adcSequencePosition;

// The channel number with the side it is on, single ended, at the board's
// channel scale.
static uint32_t adcCommandLow(uint8_t input)
{
    return ADC_CMDL_SETUP | ADC_CMDL_ADCH(input / 2) | ADC_CMDL_ABSEL(input % 2);
}

// Find the instance and input a pad reaches, preferring LPADC1 when both can
// read the pad.
static bool adcFindInput(ioTag_t tag, unsigned *instance, uint8_t *input)
{
    if (!tag || DEFIO_TAG_GPIOID(tag) != 2) {
        return false;
    }

    const unsigned pin = DEFIO_TAG_PIN(tag);

    for (unsigned i = 0; i < ARRAYLEN(adcPads); i++) {
        if (adcPads[i].pin != pin) {
            continue;
        }
        for (unsigned dev = 0; dev < ARRAYLEN(adcHardware); dev++) {
            if (adcPads[i].input[dev] != ADC_INPUT_ABSENT) {
                *instance = dev;
                *input = adcPads[i].input[dev];
                return true;
            }
        }
        return false;
    }

    return false;
}

// Bring one instance up; the RT117x LPADC has no calibration hardware.
static void adcSetup(ADC_Type *reg)
{
    // Software reset; the bit does not clear itself.
    reg->CTRL = ADC_CTRL_RST_MASK;
    reg->CTRL = 0;

    reg->CFG = ADC_CFG_SETUP;

    reg->IE = 0;
    reg->DE = 0;

    reg->CMD[ADC_COMMAND_INDEX].CMDH = ADC_CMDH_SETUP;

    // Trigger 0 runs that command, from software only.
    reg->TCTRL[0] = ADC_TCTRL_TCMD(ADC_COMMAND);

    reg->CTRL = ADC_CTRL_ADCEN_MASK;
}

// When the conversion in flight was started, so a lost one can be spotted.
static timeUs_t adcConversionStartUs;

static void adcStartConversion(void)
{
    const adcInput_t *input = &adcInputs[adcSequence[adcSequencePosition]];

    adcConversionStartUs = micros();

    input->reg->CMD[ADC_COMMAND_INDEX].CMDL = adcCommandLow(input->input);

    // Drop any late result of an earlier, abandoned conversion.
    input->reg->CTRL = ADC_CTRL_ADCEN_MASK | ADC_CTRL_RSTFIFO_MASK;

    input->reg->SWTRIG = ADC_SWTRIG_SWT0_MASK;
}

// Move the round robin on if the conversion in flight has finished, and start
// the next one. Called from adcGetValue.
static void adcUpdate(void)
{
    if (!adcSequenceLength) {
        return;
    }

    adcInput_t *input = &adcInputs[adcSequence[adcSequencePosition]];

    const uint32_t result = input->reg->RESFIFO;

    if (!(result & ADC_RESFIFO_VALID_MASK)) {
        // Nothing came back in time, so trigger the same input again.
        if (cmpTimeUs(micros(), adcConversionStartUs) > ADC_CONVERSION_TIMEOUT_US) {
            adcStartConversion();
        }
        return;
    }

    input->value = (result & ADC_RESFIFO_D_MASK) >> ADC_RESULT_SHIFT;

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

    unsigned instance[ADC_EXTERNAL_COUNT] = { 0 };
    uint8_t input[ADC_EXTERNAL_COUNT] = { 0 };

    for (unsigned source = 0; source < ARRAYLEN(tags); source++) {
        if (adcFindInput(tags[source], &instance[source], &input[source])) {
            wanted[instance[source]] = true;
        } else {
            tags[source] = IO_TAG_NONE;
        }
    }

    for (unsigned dev = 0; dev < ARRAYLEN(adcHardware); dev++) {
        if (!wanted[dev]) {
            continue;
        }

        nxpClockGateEnable(adcHardware[dev].clock);

        adcSetup(adcHardware[dev].reg);
    }

    for (unsigned source = 0; source < ARRAYLEN(tags); source++) {
        if (!tags[source]) {
            continue;
        }

        // IOCFG_ANALOG: GPIO function, output off, no pull or keeper.
        IO_t io = IOGetByTag(tags[source]);
        IOInit(io, OWNER_ADC_BATT + source, 0);
        IOConfigGPIO(io, IOCFG_ANALOG);

        adcInputs[source].reg = adcHardware[instance[source]].reg;
        adcInputs[source].input = input[source];

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

// The nominal full scale at the pin; there is no band gap input to measure the
// reference against.
uint16_t adcInternalCompensateVref(uint16_t intVRefAdcValue)
{
    UNUSED(intVRefAdcValue);

    return IMXRT_ADC_FULL_SCALE_MV;
}

#endif // USE_ADC_INTERNAL

#endif // USE_ADC
