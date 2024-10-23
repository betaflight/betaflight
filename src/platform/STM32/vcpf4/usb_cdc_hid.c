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

#include "platform.h"

#ifdef USE_USB_CDC_HID

#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"

void sendReport(uint8_t *report, uint8_t len)
{
    USBD_HID_SendReport(&USB_OTG_dev, report, len);
}

#endif
