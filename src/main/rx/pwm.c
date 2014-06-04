/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

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

