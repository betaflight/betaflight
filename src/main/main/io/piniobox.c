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

#include "common/utils.h"
#include "common/time.h"

#include "msp/msp_box.h"

#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "piniobox.h"

typedef struct pinioBoxRuntimeConfig_s {
    uint8_t boxId[PINIO_COUNT];
} pinioBoxRuntimeConfig_t;

static pinioBoxRuntimeConfig_t pinioBoxRuntimeConfig;

void pinioBoxInit(const pinioBoxConfig_t *pinioBoxConfig)
{
    // Convert permanentId to boxId_e

    for (int i = 0; i < PINIO_COUNT; i++) {
        const box_t *box = findBoxByPermanentId(pinioBoxConfig->permanentId[i]);

        pinioBoxRuntimeConfig.boxId[i] = box ? box->boxId : BOXID_NONE;
    }
}

void pinioBoxUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxRuntimeConfig.boxId[i] != BOXID_NONE) {
            pinioSet(i, getBoxIdState(pinioBoxRuntimeConfig.boxId[i]));
        }
    }
}

#endif
