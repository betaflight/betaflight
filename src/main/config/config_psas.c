/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "config/config.h"

#include "scheduler/scheduler.h"
#include "io/beeper.h"

#include "flight/psas.h"

#ifdef USE_PSAS
psasProfile_t *currentPsasProfile;

uint8_t getCurrentPsasProfileIndex(void)
{
    return systemConfig()->psasProfileIndex;
}

void loadPsasProfile(void)
{
    currentPsasProfile = psasProfilesMutable(systemConfig()->psasProfileIndex);
}

void changePsasProfile(uint8_t psasProfileIndex)
{
    // The config switch will cause a big enough delay in the current task to upset the scheduler
    schedulerIgnoreTaskExecTime();

    if (psasProfileIndex < PSAS_PROFILE_COUNT) {
        systemConfigMutable()->psasProfileIndex = psasProfileIndex;
        loadPsasProfile();

        psasInit(currentPsasProfile);
        beeperConfirmationBeeps(psasProfileIndex + 1);
    }
}
#endif