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
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "hardware_revision.h"

uint8_t hardwareRevision = UNKNOWN;

void detectHardwareRevision(void)
{
    IO_t pin1 = IOGetByTag(IO_TAG(PB12));
    IOInit(pin1, OWNER_SYSTEM, 1);
    IOConfigGPIO(pin1, IOCFG_IPU);

    // Check hardware revision
    delayMicroseconds(10);  // allow configuration to settle

    /*
        if both PB12 and 13 are tied to GND then it is Rev3A (mini)
        if only PB12 is tied to GND then it is a Rev3 (full size)
    */
    if (!IORead(pin1)) {
        hardwareRevision = BJF4_REV3;

        IO_t pin2 = IOGetByTag(IO_TAG(PB13));
        IOInit(pin2, OWNER_SYSTEM, 2);
        IOConfigGPIO(pin2, IOCFG_IPU);

        if (!IORead(pin2)) {
            hardwareRevision = BJF4_REV4;
        }
    } else {
        IO_t pin2 = IOGetByTag(IO_TAG(PB13));
        IOInit(pin2, OWNER_SYSTEM, 2);
        IOConfigGPIO(pin2, IOCFG_OUT_PP);

        IOWrite(pin2, false);

        if (!IORead(pin1)) {
            hardwareRevision = BJF4_MINI_REV3A;
        }
    }

    if (hardwareRevision == UNKNOWN) {
        hardwareRevision = BJF4_REV2;
        return;
    }
}

void updateHardwareRevision(void)
{
    if (hardwareRevision != BJF4_REV2) {
        return;
    }

    /*
        if flash exists on PB3 then Rev1
    */
    flashConfig_t flashConfig = { .csTag = IO_TAG(PB3) };
    if (flashInit(&flashConfig)) {
        hardwareRevision = BJF4_REV1;
    } else {
        IOInit(IOGetByTag(IO_TAG(PB3)), OWNER_FREE, 0);
    }
}
