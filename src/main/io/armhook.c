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
#include <platform.h>

#ifdef USE_ARM_HOOK

#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "armhook.h"

// all available hooks
#ifdef SPEEDYBEEF4
#include "target/SPEEDYBEEF4/switch_control.h"
#endif


static armHookInjection_t armHooks[] = {
#ifdef SPEEDYBEEF4
    {false, &sbfcSwitchArmHook}, // SpeedyBee FC Switch Hook
#endif

    {false, NULL}
};

void armHookInit(void)
{
    for (unsigned i = 0; i < ARRAYLEN(armHooks); i++) {
        armHookInjection_t *iterator = &armHooks[i];
        if (iterator->vTable != NULL) {
            iterator->isEnabled = iterator->vTable->initialize();
        }
    }

    return ;
}

bool armHookIsEnabled(void)
{
    for (unsigned i = 0; i < ARRAYLEN(armHooks); i++) {
        armHookInjection_t *iterator = &armHooks[i];
        if (iterator->isEnabled) {
            return true;
        }
    }

    return false;
}

void armHookProcess(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    bool isArming = ARMING_FLAG(ARMED);
    for (unsigned i = 0; i < ARRAYLEN(armHooks); i++) {
        armHookInjection_t *iterator = &armHooks[i];
        if (iterator->isEnabled && iterator->vTable != NULL) {
            iterator->vTable->process(isArming);
        }
    }
}

#endif
