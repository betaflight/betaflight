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
#include <string.h>

#include "platform.h"

#include "rc_modes.h"

#include "common/bitarray.h"
#include "common/maths.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#include "rx/rx.h"

boxBitmask_t rcModeActivationMask; // one bit per mode defined in boxId_e

static bool paralyzeModeEverDisabled = false;

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions,
                  PG_MODE_ACTIVATION_PROFILE, 1);

bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
    return bitArrayGet(&rcModeActivationMask, boxId);
}

void rcModeUpdate(boxBitmask_t *newState)
{
    rcModeActivationMask = *newState;
}

bool isAirmodeActive(void) {
    return (IS_RC_MODE_ACTIVE(BOXAIRMODE) || feature(FEATURE_AIRMODE));
}

bool isAntiGravityModeActive(void) {
    return (IS_RC_MODE_ACTIVE(BOXANTIGRAVITY) || feature(FEATURE_ANTI_GRAVITY));
}

bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range) {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    const uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

void updateMasksForMac(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask) {
    boxId_e mode = mac->modeId;

    bool bAnd = (mac->modeLogic == MODELOGIC_AND) || bitArrayGet(andMask, mode);
    bool bAct = isRangeActive(mac->auxChannelIndex, &mac->range);
    if (bAnd)
        bitArraySet(andMask, mode);
    if (bAnd != bAct)
        bitArraySet(newMask, mode);
}

void updateActivatedModes(void)
{
    boxBitmask_t newMask, andMask;
    memset(&andMask, 0, sizeof(andMask));
    memset(&newMask, 0, sizeof(newMask));

    // determine which conditions set/clear the mode
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        boxId_e mode = mac->modeId;

        // Skip linked macs for now to fully determine target states
        boxId_e linkedTo = mac->linkedTo;
        if (linkedTo) {
            continue;
        }

        // Ensure sticky modes are sticky
        if (mode == BOXPARALYZE) {
            if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
                bitArrayClr(&andMask, mode);
                bitArraySet(&newMask, mode);
            } else {
                if (paralyzeModeEverDisabled) {
                    updateMasksForMac(mac, &andMask, &newMask);
                } else {
                    paralyzeModeEverDisabled = !isRangeActive(mac->auxChannelIndex, &mac->range);
                }
            }
        } else if (mode < CHECKBOX_ITEM_COUNT) {
            updateMasksForMac(mac, &andMask, &newMask);
        }
    }

    bitArrayXor(&newMask, sizeof(&newMask), &newMask, &andMask);

    // Update linked modes
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        boxId_e linkedTo = mac->linkedTo;
        if (!linkedTo) {
            continue;
        }

        boxId_e mode = mac->modeId;
        bitArrayCopy(&newMask, linkedTo, mode);
    }

    rcModeUpdate(&newMask);
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && (IS_RANGE_USABLE(&mac->range) || mac->linkedTo)) {
            return true;
        }
    }

    return false;
}

void removeModeActivationCondition(const boxId_e modeId)
{
    unsigned offset = 0;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(i);

        if (mac->modeId == modeId && !offset) {
            offset++;
        }

        if (offset) {
            while (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT && modeActivationConditions(i + offset)->modeId == modeId) {
                offset++;
            }

            if (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                memcpy(mac, modeActivationConditions(i + offset), sizeof(modeActivationCondition_t));
            } else {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        }
    }
}
