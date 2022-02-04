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

#include <stdint.h>

#include "platform.h"

#ifdef USE_PINIOBOX

#include "build/debug.h"

#include "common/time.h"
#include "common/utils.h"

#include "msp/msp_box.h"

#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "scheduler/scheduler.h"

#include "piniobox.h"

#define ON_DURATION_US 100000 // on duration (OD) is configured in 100 ms increments

typedef enum { ON_NONE, ON_ACTIVE, ON_DONE } onState_e;

typedef struct pinioBoxRuntimeConfig_s {
    uint8_t boxId;
    uint8_t onDuration; // on duration x 100 ms
    onState_e onState; // state of the "on duration" logic
    timeUs_t onStartTimeUs; // time when its was activated
} pinioBoxRuntimeConfig_t;

static pinioBoxRuntimeConfig_t pinioBoxRuntimeConfig[PINIO_COUNT];

void pinioBoxInit(const pinioBoxConfig_t *pinioBoxConfig)
{
    // Convert permanentId to boxId and extract pulse configuration
    for (int i = 0; i < PINIO_COUNT; i++) {
        const box_t *box = findBoxByPermanentId(pinioBoxConfig->permanentId[i]);
        pinioBoxRuntimeConfig[i].boxId = box ? box->boxId : BOXID_NONE;
        pinioBoxRuntimeConfig[i].onDuration = pinioBoxConfig->onDuration[i];
    }
}

void pinioBoxUpdate(timeUs_t currentTimeUs)
{
    for (int i = 0; i < PINIO_COUNT; i++) {
        uint8_t boxId = pinioBoxRuntimeConfig[i].boxId;
        if (boxId == BOXID_NONE) continue; // nothing to do for this one -- not configured
        bool desiredOn = getBoxIdState(boxId); // desired to turn "on"
        if (pinioBoxRuntimeConfig[i].onDuration) {
            // adjust desiredOn value according to on duration timing
            switch (pinioBoxRuntimeConfig[i].onState) {
                case ON_NONE:
                    // activate if desired
                    if (desiredOn) {
                        pinioBoxRuntimeConfig[i].onState = ON_ACTIVE;
                        pinioBoxRuntimeConfig[i].onStartTimeUs = currentTimeUs;
                    }
                    break;
                case ON_ACTIVE:
                    // turn off only when the duration is over
                    desiredOn = cmpTimeUs(currentTimeUs, pinioBoxRuntimeConfig[i].onStartTimeUs) <
                                (timeDelta_t) pinioBoxRuntimeConfig[i].onDuration * ON_DURATION_US;
                    if (!desiredOn) {
                        pinioBoxRuntimeConfig[i].onState = ON_DONE;
                    }
                    break;
                case ON_DONE:
                    if (desiredOn) {
                        desiredOn = false; // on duration is over!
                    } else {
                        // reset on duration logic when it is no longer desired
                        pinioBoxRuntimeConfig[i].onState = ON_NONE;
                    }
            }
        }
        pinioSet(i, desiredOn);
    }
}

void pinioBoxTaskControl(void)
{
    bool enableTask = false;
    for (int i = 0; i < PINIO_COUNT; i++) {
        uint8_t boxId = pinioBoxRuntimeConfig[i].boxId;
        if (boxId != BOXID_NONE && isModeActivationConditionPresent(boxId)) {
            enableTask = true;
            break;
        }
    }
    setTaskEnabled(TASK_PINIOBOX, enableTask);
}
#endif
