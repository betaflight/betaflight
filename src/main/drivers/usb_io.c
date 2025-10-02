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

#include "platform.h"

#ifdef USE_VCP

#include "drivers/io.h"
#include "drivers/time.h"
#include "usb_io.h"

#include "drivers/serial_usb_vcp.h"
#ifdef USE_USB_DETECT
static IO_t usbDetectPin;
#endif

void usbCableDetectDeinit(void)
{
#ifdef USE_USB_DETECT
    IOInit(usbDetectPin, OWNER_FREE, 0);
    IOConfigGPIO(usbDetectPin, IOCFG_IN_FLOATING);
    usbDetectPin = IO_NONE;
#endif
}

void usbCableDetectInit(void)
{
#ifdef USE_USB_DETECT
    usbDetectPin = IOGetByTag(IO_TAG(USB_DETECT_PIN));

    IOInit(usbDetectPin, OWNER_USB_DETECT, 0);
    IOConfigGPIO(usbDetectPin, IOCFG_IPD);
#endif
}

bool usbCableIsInserted(void)
{
    bool result = false;

#ifdef USE_USB_DETECT
    if (usbDetectPin) {
        result = IORead(usbDetectPin) != 0;
    }
#endif

#if defined(USE_VCP)
    result = result || usbVcpIsConnected() != 0;
#endif

    return result;
}

void usbGenerateDisconnectPulse(void)
{
#ifdef USB_DP_PIN
    /* Pull down USB_DP_PIN to create USB disconnect pulse */
    IO_t usbPin = IOGetByTag(IO_TAG(USB_DP_PIN));
    if (!usbPin) {
        return;
    }

    IOConfigGPIO(usbPin, IOCFG_OUT_OD);
    IOLo(usbPin);
    delay(200);
    IOHi(usbPin);
#endif
}
#endif
