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

#include "drivers/io.h"
#include "drivers/time.h"

#include "hardware_revision.h"

uint8_t hardwareRevision = FORTINIF4_UNKNOWN;

static IO_t HWDetectPinA = IO_NONE;
static IO_t HWDetectPinB = IO_NONE;

void detectHardwareRevision(void)
{
    HWDetectPinA = IOGetByTag(IO_TAG(HW_PIN_A));
    IOInit(HWDetectPinA, OWNER_SYSTEM, 0);
    IOConfigGPIO(HWDetectPinA, IOCFG_IPU);
    HWDetectPinB = IOGetByTag(IO_TAG(HW_PIN_B));
    IOInit(HWDetectPinB, OWNER_SYSTEM, 0);
    IOConfigGPIO(HWDetectPinB, IOCFG_IPU);

    delayMicroseconds(10);  // allow configuration to settle

    // Check hardware revision
    if (IORead(HWDetectPinB)) {
        if (IORead(HWDetectPinA)) {
            hardwareRevision = FORTINIF4_REV_1;
        } else {
            hardwareRevision = FORTINIF4_REV_2;
        }
    } else {
        if (IORead(HWDetectPinA)) {
            hardwareRevision = FORTINIF4_REV_4;
        } else {
            hardwareRevision = FORTINIF4_REV_3;
        }
    }
}

void updateHardwareRevision(void)
{
}
