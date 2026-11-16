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
#include "drivers/serial_usb_vcp.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

#include "msc/usbd_storage.h"

#include "pg/sdcard.h"
#include "pg/usb.h"


#include "drivers/usb_io.h"
#include "usbd_msc.h"
#include "usbd_msc_desc.h"

static USBD_MSC_StorageType *pdev;

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *blockCount, uint32_t *blockSize)
{
    UNUSED(busid);

    usbd_msc_block_size_t size = 512;
    if (pdev->GetCapacity(lun, blockCount, &size) != 0) {
        *blockCount = 0;
    }
    *blockSize = size;
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);

    return pdev->Read(lun, buffer, sector, length / 512U);
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);

    return pdev->Write(lun, buffer, sector, length / 512U);
}

uint8_t mscStart(void)
{
    //Start USB
    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);

    /** Register interface callbacks */
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        switch (sdcardConfig()->mode) {
#ifdef USE_SDCARD_SDIO
        case SDCARD_MODE_SDIO:
			pdev = &USBD_MSC_MICRO_SDIO_fops;
            break;
#endif
#ifdef USE_SDCARD_SPI
        case SDCARD_MODE_SPI:
			pdev = &USBD_MSC_MICRO_SD_SPI_fops;
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
					pdev = &USBD_MSC_EMFAT_fops;
        break;
#endif

    default:
        return 1;
    }

    if (pdev->Init(0) != 0) {
        return 1;
    }

    mscUsbInit(0, USB_BASE);

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
#endif
