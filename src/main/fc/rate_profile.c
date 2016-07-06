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
#include <stddef.h>

#include <platform.h>

#include "build/build_config.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"
#include "config/profile.h"

//#include "fc/config.h"
#include "fc/rc_curves.h"
#include "fc/rate_profile.h"

PG_REGISTER_PROFILE(rateProfileSelection_t, rateProfileSelection, PG_RATE_PROFILE_SELECTION, 0);
PG_REGISTER_ARR_WITH_RESET_FN(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 0);

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

void pgResetFn_controlRateProfiles(controlRateConfig_t *instance)
{
    for (int i = 0; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &instance[i],
            .rcRate8 = 90,
            .rcExpo8 = 65,
            .thrMid8 = 50,
            .tpa_breakpoint = 1500,
        );
    }
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex)
{
    return controlRateProfiles(profileIndex);
}

void setControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = 0;
    }
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = controlRateProfiles(profileIndex);
}

void activateControlRateConfig(void)
{
    generatePitchRollCurve();
    generateYawCurve();
    generateThrottleCurve();
}

void changeControlRateProfile(uint8_t profileIndex)
{
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void configureRateProfileSelection(uint8_t profileIndex, uint8_t rateProfileIndex)
{
    rateProfileSelection_Storage[profileIndex].defaultRateProfileIndex = rateProfileIndex % MAX_CONTROL_RATE_PROFILE_COUNT;
}
