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

#include "drivers/time.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/flash_m25p16.h"
#include "hardware_revision.h"

#if 0
static const char * const hardwareRevisionNames[] = {
    "Unknown",
    "BlueJay rev1",
    "BlueJay rev2",
    "BlueJay rev3",
    "BlueJay rev3a"
};
#endif

uint8_t hardwareRevision = UNKNOWN;

void detectHardwareRevision(void)
{
    IO_t pin1 = IOGetByTag(IO_TAG(PB12));
    IOInit(pin1, OWNER_SYSTEM, RESOURCE_INPUT, 1);
    IOConfigGPIO(pin1, IOCFG_IPU);

    IO_t pin2 = IOGetByTag(IO_TAG(PB13));
    IOInit(pin2, OWNER_SYSTEM, RESOURCE_INPUT, 2);
    IOConfigGPIO(pin2, IOCFG_IPU);

    // Check hardware revision
    delayMicroseconds(10);  // allow configuration to settle

    /*
        if both PB12 and 13 are tied to GND then it is Rev3A (mini)
        if only PB12 is tied to GND then it is a Rev3 (full size)
    */
    if (!IORead(pin1)) {
        if (!IORead(pin2)) {
            hardwareRevision = BJF4_REV3A;
        }
        hardwareRevision = BJF4_REV3;
    }

    if (hardwareRevision == UNKNOWN) {
        hardwareRevision = BJF4_REV2;
        return;
    }

    /*
        enable the UART1 inversion PC9

        TODO: once param groups are in place, inverter outputs
        can be moved to be simple IO outputs, and merely set them
        HI or LO in configuration.
    */
    IO_t uart1invert = IOGetByTag(IO_TAG(PC9));
    IOInit(uart1invert, OWNER_INVERTER, RESOURCE_OUTPUT, 2);
    IOConfigGPIO(uart1invert, IOCFG_AF_PP);
    IOLo(uart1invert);
}

void updateHardwareRevision(void)
{
    if (hardwareRevision != BJF4_REV2) {
        return;
    }

    /*
        if flash exists on PB3 then Rev1
    */
    if (m25p16_init(IO_TAG(PB3))) {
        hardwareRevision = BJF4_REV1;
    } else {
        IOInit(IOGetByTag(IO_TAG(PB3)), OWNER_FREE, RESOURCE_NONE, 0);
    }
}


