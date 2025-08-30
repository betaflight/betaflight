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

/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#if defined(USE_USB_MSC)

#include "build/build_config.h"

#include "common/utils.h"

#include "blackbox/blackbox.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sdmmc_sdio.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

#include "msc/usbd_storage.h"

#include "pg/sdcard.h"
#include "pg/usb.h"

#include "usbd_core.h"
#include "usbd_cdc_vcp.h"
#include "drivers/usb_io.h"

extern USBD_INFO_T gUsbDevice;

static void USB_DevUserHandler(USBD_INFO_T* usbInfo, uint8_t userStatus);

extern USBD_MSC_MEMORY_T USBD_MEMORY_INTERFACE;
extern USBD_DESC_T USBD_DESC_MSC;
uint8_t mscStart(void)
{
    //Start USB
    usbGenerateDisconnectPulse();

    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        switch (sdcardConfig()->mode) {
#ifdef USE_SDCARD_SDIO
        case SDCARD_MODE_SDIO:
            USBD_MSC_RegisterMemory(&gUsbDevice, &USBD_MEMORY_INTERFACE);
            break;
#endif
#ifdef USE_SDCARD_SPI
        case SDCARD_MODE_SPI:
            USBD_MSC_RegisterMemory(&gUsbDevice, &USBD_MEMORY_INTERFACE);
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        USBD_MSC_RegisterMemory(&gUsbDevice, &USBD_MEMORY_INTERFACE);
        break;
#endif
    default:
        return 1;
    }

    USBD_Init(&gUsbDevice, USBD_SPEED_FS, &USBD_DESC_MSC, &USBD_MSC_CLASS, USB_DevUserHandler);

    // NVIC configuration for SYSTick
    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(0, 0));
    NVIC_EnableIRQ(SysTick_IRQn);

    return 0;
}


void mscTask(void)
{
    // Nothing to do here
}

static void USB_DevUserHandler(USBD_INFO_T* usbInfo, uint8_t userStatus)
{
    UNUSED(usbInfo);
    UNUSED(userStatus);
}
#endif
