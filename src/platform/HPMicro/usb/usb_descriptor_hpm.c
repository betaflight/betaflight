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

/*
 * USB descriptor support: device serial number derived from the OTP UUID.
 */

#include "platform.h"

#include "hpm_otp_drv.h"

#include "usb_descriptor_hpm.h"

_Static_assert(HPM_USB_SERIAL_NUMBER_LENGTH == OTP_SOC_UUID_LEN * 2, "USB serial length must match the OTP UUID");

const char *hpmUsbGetSerialNumber(void)
{
    static char serialNumber[HPM_USB_SERIAL_NUMBER_LENGTH + 1];
    static const char hex[] = "0123456789ABCDEF";

    if (!serialNumber[0]) {
        for (unsigned wordIndex = 0; wordIndex < OTP_SOC_UUID_LEN / sizeof(uint32_t); wordIndex++) {
            const uint32_t word = otp_read_from_shadow(OTP_SOC_UUID_IDX + wordIndex);
            for (unsigned nibble = 0; nibble < 8; nibble++) {
                serialNumber[wordIndex * 8 + nibble] = hex[(word >> (28 - nibble * 4)) & 0x0f];
            }
        }
    }

    return serialNumber;
}
