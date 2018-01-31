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

#ifdef USE_ARM_STATE

#include "fc/runtime_config.h"

#include "drivers/io_types.h"
#include "drivers/io.h"

#ifndef ARM_STATE_PIN
#define ARM_STATE_PIN NONE
#endif

static IO_t armStatePin = NULL;

bool armStateInit(void)
{
    armStatePin = IOGetByTag(IO_TAG(ARM_STATE_PIN));
    if (armStatePin == NULL) {
        return false;
    }

    IOInit(armStatePin, OWNER_ARM_STATE, 0);
    IOConfigGPIO(armStatePin, IOCFG_OUT_PP);

    IOLo(armStatePin); // default state is Lo

    return true;
}

void armStateUpdate(bool isArming)
{
    IOWrite(armStatePin, !isArming);
}

#endif
