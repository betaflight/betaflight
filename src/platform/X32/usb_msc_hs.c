/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * X32M7 USB Mass Storage Class (MSC) device initialization.
 *
 * It provides the MSC boot mode: when Betaflight enters MSC mode
 * (via boot button or CLI command), this file initializes the USB
 * device as a pure MSC device, exposing SD card or onboard flash
 * as a USB mass storage drive to the host PC.
 *
 * MSC boot flow (from main.c):
 *   systemInit() -> mscCheckBootAndReset() -> mscStart() -> mscWaitForButton()
 *
 * mscStart() is called instead of usbVcpInit() — they are mutually
 * exclusive. In MSC mode, no VCP serial port is available.
 *
 * Architecture:
 *
 *   mscStart()  (this file)
 *       |
 *       v
 *   USBD_Init(&USB_dev, ..., &USBD_MSC_cb, ...)
 *       |
 *       v
 *   usbd_msc_core.c  (X32 MSC class driver)
 *       |
 *       v
 *   USBD_STORAGE_fops  (storage backend: SDIO, SPI-SD, or EMFAT)
 *       |
 *       v
 *   Physical storage (SD card or onboard flash)
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
#include "drivers/time.h"
#include "drivers/usb_io.h"
#include "drivers/usb_msc.h"

#include "io/usb_msc.h"

#include "msc/usbd_storage.h"

#include "pg/sdcard.h"
#include "pg/usb.h"

/* X32 USB library headers */
#include "usbd_cdc_vcp.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc_desc.h"
#include "usbd_user.h"
#include "usbd_msc_core.h"

/*
 * USB core ID selection.
 *
 * X32M7 has two USB HS controllers (USB1_HS, USB2_HS).
 * Default to USB1 unless overridden by the target definition.
 */
#ifdef USE_USBHS1
#ifndef USB_CORE_ID
#define USB_CORE_ID USBHS1_CORE_ID
#endif
#endif /* USE_USBHS1 */

#ifdef USE_USBHS2
#ifndef USB_CORE_ID
#define USB_CORE_ID USBHS2_CORE_ID
#endif
#endif /* USE_USBHS2 */

/* USB pin definitions, can be overridden by target */
#ifdef USE_USBHS1
#ifndef USB_DM_PIN
#define USB_DM_PIN PA11
#endif

#ifndef USB_DP_PIN
#define USB_DP_PIN PA12
#endif
#endif /* USE_USBHS1 */

#ifdef USE_USBHS2
#ifndef USB_DM_PIN
#define USB_DM_PIN PB14
#endif

#ifndef USB_DP_PIN
#define USB_DP_PIN PB15
#endif
#endif /* USE_USBHS2 */

/**
 * @brief  Start USB MSC device mode.
 *
 * Called from main.c when MSC boot is requested (via RTC backup register
 * flag set by systemResetToMsc(), or via boot button held during power-on).
 *
 * Performs:
 *   1. Generate USB disconnect pulse to force host re-enumeration.
 *   2. Register USB DM/DP pins with IO subsystem.
 *   3. Select the MSC storage backend based on blackbox configuration:
 *      - SDCARD via SDIO: USBD_MSC_MICRO_SDIO_fops
 *      - SDCARD via SPI:  USBD_MSC_MICRO_SD_SPI_fops
 *      - Onboard flash:   USBD_MSC_EMFAT_fops (virtual FAT32 via EMFAT)
 *   4. Enable X32 USB HS clock and power.
 *   5. Initialize the X32 USB device library in MSC-only mode:
 *      - USB_dev:       USB device handle (defined in usbd_cdc_vcp.c)
 *      - USBD_MSC_desc: MSC device descriptors (different PID from VCP)
 *      - USBD_MSC_cb:   MSC class callbacks (from usbd_msc_core.c)
 *      - USER_cb:       Device user callbacks (from usbd_user.c)
 *   6. Reconfigure SysTick priority to highest (0,0) — in MSC mode
 *      there's no flight controller running, SysTick drives delay()
 *      and must not be preempted by USB interrupts.
 *
 * @return 0 on success, 1 if no valid storage backend is available.
 */
uint8_t mscStart(void)
{
    /* Enable X32 USB HS clock and power */
    RCC_ConfigUSBRefClk(RCC_USBREFCLK_HSE_DIV1);
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_PWR, ENABLE);

#ifdef USE_USBHS1
    RCC_EnableAHB2PeriphClk1(RCC_AHB2_PERIPHEN_M7_USB1, ENABLE);
    PWR_MoudlePowerEnable(HSC1_USB1_PWRCTRL, ENABLE);
#endif

#ifdef USE_USBHS2
    RCC_EnableAHB1PeriphClk1(RCC_AHB1_PERIPHEN_M7_USB2, ENABLE);
    PWR_MoudlePowerEnable(HSC2_USB2_PWRCTRL, ENABLE);
#endif

    usbGenerateDisconnectPulse();

    IOInit(IOGetByTag(IO_TAG(USB_DM_PIN)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(USB_DP_PIN)), OWNER_USB, 0);

    /*
     * Select storage backend based on blackbox device configuration.
     * This determines which SCSI read/write callbacks the MSC class
     * driver will use to access the physical storage medium.
     */
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

    /* Initialize USB device in MSC-only mode */
    memset(&USB_dev, 0, sizeof(USB_dev));
    USBD_Init(&USB_dev, USB_CORE_ID, &USBD_MSC_desc, &USBD_MSC_cb, &USER_cb);

    /*
     * Reconfigure SysTick to highest priority.
     *
     * In MSC mode, the main loop is just mscWaitForButton() — an infinite
     * loop calling delay(). SysTick must have the highest priority so
     * delay() works correctly even when USB interrupts are active.
     * Without this, USB interrupt handlers could starve SysTick and
     * cause delay() to hang.
     */
    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(0, 0));
    NVIC_EnableIRQ(SysTick_IRQn);

    return 0;
}

/**
 * @brief  MSC periodic task (no-op).
 *
 * Called from the MSC main loop (mscWaitForButton). On X32, all MSC
 * operations are interrupt-driven by the USB HS controller — the
 * MSC class driver handles SCSI commands entirely in interrupt context
 * via USBD_MSC_cb callbacks. No polling is needed.
 *
 * This function exists only to satisfy the mscTask() prototype
 * declared in usb_msc.h.
 */
void mscTask(void)
{
}

#endif /* USE_USB_MSC */
