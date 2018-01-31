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

#ifdef SPEEDYBEEF4

#include "fc/runtime_config.h"

#include "drivers/io_types.h"
#include "drivers/io.h"

#include "switch_control.h"


#ifndef SBFC_SWITCH_PIN
#define SBFC_SWITCH_PIN NONE
#endif

#ifndef SBFC_CONNECTION_STATE_PIN
#define SBFC_CONNECTION_STATE_PIN NONE
#endif

static IO_t switchPin = NULL;
static IO_t connectionStatePin = NULL;
static bool isEnabled = false;

bool sbfcSwitchControlInitialize(void)
{
    isEnabled = false;
    switchPin = IOGetByTag(IO_TAG(SBFC_SWITCH_PIN));
    if (switchPin == NULL) {
        return false;
    }

    connectionStatePin = IOGetByTag(IO_TAG(SBFC_CONNECTION_STATE_PIN));
    if (connectionStatePin == NULL) {
        return false;
    }

    IOInit(switchPin, OWNER_SBFC_SWITCH_PIN, 0);
    IOConfigGPIO(switchPin, IOCFG_OUT_PP);

    IOInit(connectionStatePin, OWNER_SBFC_CONNECTION_STATE_PIN, 0);
    IOConfigGPIO(connectionStatePin, IOCFG_IN_FLOATING);

    isEnabled = true;
    return true;
}

void sbfcSwitchControlUpdateState(bool isArming)
{
    bool isHi = isArming;
    if (isArming && !IORead(connectionStatePin)) { // connectionStatePin is low, so connection state is connected
        return ;
    }

    IOWrite(switchPin, isHi);

    return ;
}

bool sbfcSwitchControlIsEnabled(void)
{
    return isEnabled;
}

#endif
