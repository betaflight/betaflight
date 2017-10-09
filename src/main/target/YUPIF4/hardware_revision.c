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
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "hardware_revision.h"

uint8_t hardwareRevision = UNKNOWN;

void detectHardwareRevision(void)
{
    IO_t pin1 = IOGetByTag(IO_TAG(PC13));
    IOInit(pin1, OWNER_SYSTEM, 1);
    IOConfigGPIO(pin1, IOCFG_IPU);

    IO_t pin2 = IOGetByTag(IO_TAG(PC14));
    IOInit(pin2, OWNER_SYSTEM, 1);
    IOConfigGPIO(pin2, IOCFG_IPU);

    IO_t pin3 = IOGetByTag(IO_TAG(PC15));
    IOInit(pin3, OWNER_SYSTEM, 1);
    IOConfigGPIO(pin3, IOCFG_IPU);

    // Check hardware revision
    delayMicroseconds(10);  // allow configuration to settle

    /*
        Hardware pins : Pin1 = PC13 / Pin2 = PC14 / Pin3 = PC15
        no Hardware pins tied to ground => Race V1
        if Pin 1 is the only one tied to ground => Mini
        if Pin 2 is the only one tied to ground => Race V2
        if Pin 2 and Pin 1 are tied to ground => Race V3
        if Pin 3 is the only one tied to ground => Navigation
        Other combinations available for potential evolutions
    */
    if (!IORead(pin1) && IORead(pin2)) {
        hardwareRevision = YUPIF4_MINI;
    } else if (IORead(pin1) && !IORead(pin2)) {
        hardwareRevision = YUPIF4_RACE2;
    } else if (!IORead(pin1) && !IORead(pin2)) {
        hardwareRevision = YUPIF4_RACE3;
    } else if (!IORead(pin3)) {
        hardwareRevision = YUPIF4_NAV;
    } else {
        hardwareRevision = YUPIF4_RACE1;
    }
}

void updateHardwareRevision(void) {
}
