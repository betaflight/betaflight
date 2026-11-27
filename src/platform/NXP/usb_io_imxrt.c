/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// USB cable detection; with USE_USB_DETECT the PHY's VBUS comparator sees the cable.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_VCP

#include "drivers/serial_usb_vcp.h"
#include "drivers/usb_io.h"

void usbCableDetectInit(void)
{
    // usbVcpInit() has already opened the USB clock gate and released the PHY.
}

bool usbCableIsInserted(void)
{
#if !defined(USE_USB_DETECT)
    return usbVcpIsConnected();
#elif defined(IMXRT11XX)
    return USBPHY1->USB1_VBUS_DET_STAT & USBPHY_USB1_VBUS_DET_STAT_VBUS_VALID_3V_MASK;
#else
    return USB_ANALOG->INSTANCE[0].VBUS_DETECT_STAT & USB_ANALOG_VBUS_DETECT_STAT_VBUS_VALID_MASK;
#endif
}

bool usbCableIsActive(void)
{
    // With USE_USB_DETECT this already requires VBUS.
    return usbVcpIsActive();
}

#endif
