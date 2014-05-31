#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "drivers/pwm_rx.h"

#include "config/config.h"

#include "rx/rx.h"
#include "rx/pwm.h"

static uint16_t pwmReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    return pwmRead(chan);
}

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    // configure PWM/CPPM read function and max number of channels. serial rx below will override both of these, if enabled
    *callback = pwmReadRawRC;

    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;
    }
    if (feature(FEATURE_RX_PPM)) {
        rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT;
    }
}

