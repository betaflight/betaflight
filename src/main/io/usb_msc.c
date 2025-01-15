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

#include "blackbox/blackbox.h"

#include "drivers/sdcard.h"

#include "io/flashfs.h"
#include "io/usb_msc.h"

#if defined(USE_USB_MSC)

bool mscCheckFilesystemReady(void)
{
    return false
#if defined(USE_SDCARD)
        || (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD && sdcard_isFunctional())
#endif
#if defined(USE_FLASHFS)
        || (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH && flashfsGetSize() > 0)
#endif
        ;
}
#endif
