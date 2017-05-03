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

#include "platform.h"
#include "drivers/io.h"
#include "drivers/time.h"

static IO_t usbDetectPin = IO_NONE;

void usbCableDetectDeinit(void)
{
#ifdef USB_DETECT_PIN
    IOInit(usbDetectPin, OWNER_FREE, RESOURCE_NONE);
    IOConfigGPIO(usbDetectPin, IOCFG_IN_FLOATING);
    usbDetectPin = IO_NONE;
#endif
}

void usbCableDetectInit(void)
{
#ifdef USB_DETECT_PIN
    usbDetectPin = IOGetByTag(IO_TAG(USB_DETECT_PIN));

    IOInit(usbDetectPin, OWNER_USB, RESOURCE_INPUT);
    IOConfigGPIO(usbDetectPin, IOCFG_OUT_PP);
#endif
}

bool usbCableIsInserted(void)
{
    bool result = false;

#ifdef USB_DETECT_PIN
    result = IORead(usbDetectPin) != 0;
#endif

    return result;
}
