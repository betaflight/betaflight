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

#ifdef USB_IO

#include "drivers/io.h"
#include "drivers/time.h"
#include "usb_io.h"
#include "sdcard.h"



#ifdef USB_DETECT_PIN
static IO_t usbDetectPin = IO_NONE;
#endif

void usbCableDetectDeinit(void)
{
#ifdef USB_DETECT_PIN
    IOInit(usbDetectPin, OWNER_FREE, RESOURCE_NONE, 0);
    IOConfigGPIO(usbDetectPin, IOCFG_IN_FLOATING);
    usbDetectPin = IO_NONE;
#endif
}

void usbCableDetectInit(void)
{
#ifdef USB_DETECT_PIN
    usbDetectPin = IOGetByTag(IO_TAG(USB_DETECT_PIN));

    IOInit(usbDetectPin, OWNER_USB, RESOURCE_INPUT, 0);
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

void usbGenerateDisconnectPulse(void)
{
    /* Pull down PA12 to create USB disconnect pulse */
    IO_t usbPin = IOGetByTag(IO_TAG(PA12));
    IOConfigGPIO(usbPin, IOCFG_OUT_OD);

    IOHi(usbPin);

    delay(200);

    IOLo(usbPin);
}

#endif
