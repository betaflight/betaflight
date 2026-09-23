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
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

// MSC mode entry point: selects the storage backend from config,
// initialises the USB MSC device, and runs the polling task.

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

#if defined(USE_USB_MSC)

#include "blackbox/blackbox.h"
#include "drivers/usb_msc.h"
#include "pg/sdcard.h"
#include "usb_config.h"
#include "msc_hpm.h"
#include "watchdog_hpmicro.h"

extern void usbd_msc_polling(uint8_t busid);

void mscTask(void)
{
    /* CONFIG_USBDEV_MSC_POLLING keeps potentially blocking storage callbacks
     * in the mscWaitForButton loop instead of the USB ISR. IRQ preemption is
     * enabled independently by DISABLE_IRQ_PREEMPTIVE=0. */
    usbd_msc_polling(0);
}

uint8_t mscStart(void)
{
    /* MSC mode runs its own polling loop instead of the scheduler, so the
     * scheduler-progress watchdog must not remain armed. */
    systemWatchdogDisable();

    // Start USB
    extern void hpm_usb_init(USB_Type *ptr);
    hpm_usb_init((USB_Type *) CONFIG_HPM_USBD_BASE);

    hpmMscStorage_e storage;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        switch (sdcardConfig()->mode) {
#ifdef USE_SDCARD_SDIO
        case SDCARD_MODE_SDIO:
            storage = HPM_MSC_STORAGE_SDIO;
            break;
#endif
#ifdef USE_SDCARD_SPI
        case SDCARD_MODE_SPI:
            storage = HPM_MSC_STORAGE_SD_SPI;
            break;
#endif
        default:
            return 1;
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        storage = HPM_MSC_STORAGE_EMFAT;
        break;
#endif

    default:
        return 1;
    }
    /* USB0's PLIC priority is otherwise never configured on the MSC boot path
     * (usbVcpInit() runs after initMsc() and only in normal mode). The selected
     * SDXC IRQ is enabled by hpmSdioConfigureHost() in msc_device_init(). */
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 1);
    /* usbd_initialize() resets the HPM USB PHY and controller before it
     * connects, providing the required startup re-enumeration interval. */
    if (msc_device_init(0, CONFIG_HPM_USBD_BASE, storage) != 0) {
        return 1;
    }

    return 0;
}

#endif
