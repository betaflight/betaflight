#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "system_common.h"

#include "accgyro_common.h"

#include "adc_common.h"

adc_config_t adcConfig[ADC_CHANNEL_COUNT];
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];
uint8_t adcChannelCount = 0;

extern int16_t debug[4];

uint16_t adcGetChannel(uint8_t channel)
{
#if 0
    switch (adcChannelCount) {
        case 3:
            debug[2] = adcValues[adcConfig[2].dmaIndex];
            /* no break */
        case 2:
            debug[1] = adcValues[adcConfig[1].dmaIndex];
            /* no break */
        case 1:
            debug[0] = adcValues[adcConfig[0].dmaIndex];
            /* no break */
        default:
            break;
    }
#endif
    return adcValues[adcConfig[channel].dmaIndex];
}

