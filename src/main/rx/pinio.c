/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_PINIO

#include "common/utils.h"

#include "drivers/io.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "io/piniobox.h"
#include "drivers/pinio.h"
#include "drivers/time.h"
#include "rx/pinio.h"

#define FRAME_US (2000)
#define REFRESH_HZ (4 * 1000000 / FRAME_US)
#define DEBOUNCE_MS (24)
#define DEBOUNCE_COUNT (24 * 1000 / FRAME_US)

static timeUs_t nextFrameUs = 0;

static uint8_t debounce;
static bool toggleState = false;
static bool togglePrev = true;

static float rxPinioReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);

    /* schedule next frame at the nearest multiple of FRAME_US in the future */
    const timeUs_t nowUs = microsISR();
    nextFrameUs = ((nowUs / FRAME_US) + 1) * FRAME_US;

    uint16_t rawrc;

    int value, pin;

    switch(chan) {
    case 0:
        return PWM_RANGE_MIDDLE;
        break;

    case 1:
        return PWM_RANGE_MIDDLE;
        break;

    case 2:
        return PWM_RANGE_MIDDLE;
        break;

    case 3:
        return PWM_RANGE_MIN; /* throttle */
        break;

    case 4:
        pin = 0;
        break;

    case 5:
        pin = 1;
        break;

    case 6:
        pin = 2;
        break;

    case 7:
        pin = 3;
        break;

    default: 
        return 0;
    };

    /* The first pinio is a toggle switch, the other three are momentary */

    if(pin == 0) {
        if(debounce) {
            debounce -= 1;
        } else {
            value = pinioGet(pin);
            if(!value && togglePrev) {
                debounce = DEBOUNCE_COUNT;
                toggleState = !toggleState;
            };
            togglePrev = value;
        }

        rawrc = toggleState ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    } else {
        value = pinioGet(pin);
        rawrc = value ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    };

    return rawrc;
}

static uint8_t rxPinioFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t deltaUs = cmpTimeUs(nowUs, nextFrameUs);

    if (deltaUs >= 0) {
        return RX_FRAME_COMPLETE;
    } else {
        return RX_FRAME_PENDING;
    };
}

void rxPinioInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxConfig);

    rxRuntimeState->channelCount = 8;
    rxRuntimeState->rxRefreshRate = REFRESH_HZ;

    rxRuntimeState->rcReadRawFn = rxPinioReadRawRC;
    rxRuntimeState->rcFrameStatusFn = rxPinioFrameStatus;
}
#endif