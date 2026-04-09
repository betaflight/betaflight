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

#include "platform.h"

#ifdef USE_ADC

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/adc_ll.h"
#pragma GCC diagnostic pop

#include "soc/adc_channel.h"

#include "common/utils.h"
#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "pg/adc.h"

// ESP32-S3 ADC1 channels 0-9 map to GPIO 1-10
#define ESP32_ADC1_CHANNEL_COUNT  10
#define ESP32_ADC1_GPIO_BASE      1   // GPIO1 = ADC1_CH0

// Polling timeout for oneshot conversion (iterations)
#define ADC_CONV_TIMEOUT  10000

typedef struct adcOperatingConfig_s {
    ioTag_t tag;
    uint8_t channel;      // ADC1 hardware channel (0-9)
    bool enabled;
} adcOperatingConfig_t;

static adcOperatingConfig_t adcOperatingConfig[ADC_EXTERNAL_COUNT];
static volatile uint16_t adcValues[ADC_EXTERNAL_COUNT];

// Convert a GPIO pin number to an ADC1 channel, or -1 if not an ADC pin
static int adcChannelByPin(int pin)
{
    int channel = pin - ESP32_ADC1_GPIO_BASE;
    if (channel >= 0 && channel < ESP32_ADC1_CHANNEL_COUNT) {
        return channel;
    }
    return -1;
}

// Perform a single oneshot ADC1 read on the given channel
static uint16_t adcReadChannel(uint8_t channel)
{
    adc_oneshot_ll_set_channel(ADC_UNIT_1, channel);
    adc_oneshot_ll_start(ADC_UNIT_1);

    int timeout = ADC_CONV_TIMEOUT;
    while (!adc_oneshot_ll_raw_check_valid(ADC_UNIT_1) && --timeout > 0) {
    }

    if (timeout <= 0) {
        return 0;
    }

    return (uint16_t)adc_oneshot_ll_get_raw_result(ADC_UNIT_1);
}

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    for (unsigned i = 0; i < ARRAYLEN(adcValues); i++) {
        adcValues[i] = 0;
    }

    // Enable ADC bus clock and reset
    {
        int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
        adc_ll_enable_bus_clock(true);
        adc_ll_reset_register();
    }

    // Configure ADC1 for 12-bit resolution
    adc_oneshot_ll_set_output_bits(ADC_UNIT_1, ADC_BITWIDTH_12);

    // Map enabled channels from config
    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }
    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
    }
    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
    }
    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
    }

    // Resolve GPIO tags to hardware channels and configure attenuation
    for (unsigned i = 0; i < ARRAYLEN(adcOperatingConfig); i++) {
        if (!adcOperatingConfig[i].tag) {
            continue;
        }

        int pin = DEFIO_TAG_PIN(adcOperatingConfig[i].tag);
        int channel = adcChannelByPin(pin);

        if (channel < 0) {
            continue;
        }

        IO_t io = IOGetByTag(adcOperatingConfig[i].tag);
        IOInit(io, OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);

        adcOperatingConfig[i].channel = channel;
        adcOperatingConfig[i].enabled = true;

        // Set attenuation to 12dB for 0-3.3V input range
        adc_oneshot_ll_set_atten(ADC_UNIT_1, channel, ADC_ATTEN_DB_12);
    }

    adc_oneshot_ll_enable(ADC_UNIT_1);
}

uint16_t adcGetValue(adcSource_e source)
{
    if ((unsigned)source >= ARRAYLEN(adcOperatingConfig) || !adcOperatingConfig[source].enabled) {
        return 0;
    }

    // Perform a fresh oneshot read (ADC runs at task rate ~10Hz, no DMA needed)
    adcValues[source] = adcReadChannel(adcOperatingConfig[source].channel);

    return adcValues[source];
}

uint16_t adcInternalReadVrefint(void)
{
    // ESP32-S3 uses eFuse-calibrated Vref, nominal 1100mV
    return 1100;
}

uint16_t adcInternalReadTempsensor(void)
{
    // ESP32-S3 has a built-in temperature sensor but it uses a separate
    // peripheral (temperature_sensor_ll.h), not the SAR ADC.
    // Return nominal room temperature for now.
    return 25;
}

#endif // USE_ADC
