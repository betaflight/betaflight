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

#include "usbd_conf.h"
#include "drivers/usb_io.h"
#include "usbd_msc_desc.h"
#include "usbd_msc_mem.h"
#include "drv_usb_hw.h"

extern usb_core_driver USB_OTG_dev;
usbd_mem_cb *usbd_mem_fops;

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
            USBD_STORAGE_fops = &USBD_MSC_MICRO_SDIO_fops;
            break;
#endif
#ifdef USE_SDCARD_SPI
        case SDCARD_MODE_SPI:
            USBD_STORAGE_fops = &USBD_MSC_MICRO_SD_SPI_fops;
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        USBD_STORAGE_fops = &USBD_MSC_EMFAT_fops;
        break;
#endif
    default:
        return 1;
    }

    static usbd_mem_cb usbd_internal_storage_fops;
	{
        usbd_internal_storage_fops.mem_init = USBD_STORAGE_fops->Init;
        usbd_internal_storage_fops.mem_ready = USBD_STORAGE_fops->IsReady;
        usbd_internal_storage_fops.mem_protected = USBD_STORAGE_fops->IsWriteProtected;
        usbd_internal_storage_fops.mem_read = USBD_STORAGE_fops->Read;
        usbd_internal_storage_fops.mem_write = USBD_STORAGE_fops->Write;
        usbd_internal_storage_fops.mem_maxlun = USBD_STORAGE_fops->GetMaxLun;
        usbd_internal_storage_fops.mem_inquiry_data[0] = (uint8_t *)(USBD_STORAGE_fops->pInquiry);
        usbd_internal_storage_fops.mem_getcapacity = USBD_STORAGE_fops->GetCapacity;

        usbd_internal_storage_fops.mem_block_size[0] = 0;
        usbd_internal_storage_fops.mem_block_len[0] = 0;
    }
    
    usbd_mem_fops = &usbd_internal_storage_fops;
    usb_gpio_config();
    usb_rcu_config();

    usbd_init(&USB_OTG_dev, USB_CORE_ENUM_FS, &bf_msc_desc, &bf_msc_class);
    usb_intr_config();

    // NVIC configuration for SYSTick
    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(0, 0));
    NVIC_EnableIRQ(SysTick_IRQn);

    return 0;
}

#endif
