#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "rx_common.h"
#include "config.h"

#include "drivers/pwm_common.h"

#include "failsafe.h"
#include "rx_common.h"
#include "rx_pwm.h"

static uint16_t pwmReadRawRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    data = pwmRead(rxConfig->rcmap[chan]);
    if (data < 750 || data > 2250)
        data = rxConfig->midrc;

    return data;
}

void pwmRxInit(rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback)
{
    // configure PWM/CPPM read function and max number of channels. serial rx below will override both of these, if enabled
    *callback = pwmReadRawRC;
    rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT;
}

