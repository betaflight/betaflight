/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "rc_modes.h"

#include "common/bitarray.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#include "rx/rx.h"

boxBitmask_t rcModeActivationMask; // one bit per mode defined in boxId_e

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions,
                  PG_MODE_ACTIVATION_PROFILE, 0);


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


void updateActivatedModes(void)
{
    boxBitmask_t newMask;
    memset(&newMask, 0, sizeof(newMask));

    for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(index);

        if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
            boxId_e mode = modeActivationCondition->modeId;
            if (mode < CHECKBOX_ITEM_COUNT)
                bitArraySet(&newMask, mode);
        }
    }
    rcModeUpdate(&newMask);
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    uint8_t index;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(index);

        if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            return true;
        }
    }

    return false;
}
