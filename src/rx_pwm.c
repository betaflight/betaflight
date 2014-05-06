#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "drivers/pwm_rx.h"

#include "config.h"

#include "failsafe.h"

#include "rx_common.h"
#include "rx_pwm.h"

static uint16_t pwmReadRawRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    return pwmRead(rxConfig->rcmap[chan]);
}

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback)
{
    // configure PWM/CPPM read function and max number of channels. serial rx below will override both of these, if enabled
    *callback = pwmReadRawRC;
    rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT;
}

