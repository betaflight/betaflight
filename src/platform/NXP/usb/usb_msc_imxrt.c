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

// Mass storage boot mode: the SD card handed to the host as a disk, with
// TinyUSB's blocking mass storage class over the src/main/msc/ backend.

// TinyUSB comes first: platform.h poisons sprintf and snprintf.
#include "tusb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/version.h"

#include "common/maths.h"
#include "common/utils.h"

#include "msc/usbd_storage.h"

#include "usb_cdc.h"
#include "usb_descriptors.h"

#ifdef USE_USB_MSC

#include "blackbox/blackbox.h"

#include "drivers/sdmmc_sdio.h"
#include "drivers/usb_msc.h"

#include "pg/sdcard.h"

#endif

// The SD driver only moves 512 byte blocks, one per endpoint buffer.
#define MSC_BLOCK_SIZE      512

STATIC_ASSERT(CFG_TUD_MSC_EP_BUFSIZE == MSC_BLOCK_SIZE,
              "the mass storage endpoint buffer has to be one SD card block");

static void mscActivity(void)
{
#ifdef USE_USB_MSC
    mscSetActive();
#endif
}

// TinyUSB mass storage callbacks; without storage they report an empty drive.

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    return USBD_STORAGE_fops && USBD_STORAGE_fops->IsReady(lun) == 0;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *blockCount, uint16_t *blockSize)
{
    uint32_t count = 0;
    usbd_msc_block_size_t size = MSC_BLOCK_SIZE;

    if (!USBD_STORAGE_fops || USBD_STORAGE_fops->GetCapacity(lun, &count, &size) != 0) {
        count = 0;
        size = MSC_BLOCK_SIZE;
    }

    *blockCount = count;
    *blockSize = (uint16_t)size;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    return USBD_STORAGE_fops && USBD_STORAGE_fops->IsWriteProtected(lun) == 0;
}

// Requests that are not whole blocks on a block boundary are stalled.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    if (!USBD_STORAGE_fops || offset != 0 || (bufsize % MSC_BLOCK_SIZE) != 0) {
        return -1;
    }

    if (USBD_STORAGE_fops->Read(lun, (uint8_t *)buffer, lba, (uint16_t)(bufsize / MSC_BLOCK_SIZE)) != 0) {
        return -1;
    }

    mscActivity();

    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!USBD_STORAGE_fops || offset != 0 || (bufsize % MSC_BLOCK_SIZE) != 0) {
        return -1;
    }

    if (USBD_STORAGE_fops->Write(lun, buffer, lba, (uint16_t)(bufsize / MSC_BLOCK_SIZE)) != 0) {
        return -1;
    }

    mscActivity();

    return (int32_t)bufsize;
}

// SCSI INQUIRY fields are fixed width and space padded.
static void inquiryField(uint8_t *field, size_t width, const char *text)
{
    const size_t length = MIN(strlen(text), width);

    memcpy(field, text, length);
    memset(field + length, ' ', width - length);
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendorId[8], uint8_t productId[16], uint8_t productRev[4])
{
    UNUSED(lun);

    inquiryField(vendorId, 8, "BETAFLT");
    inquiryField(productId, 16, targetName);
    inquiryField(productRev, 4, "0001");
}

// Stall any SCSI command TinyUSB does not answer itself.
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsiCommand[16], void *buffer, uint16_t bufsize)
{
    UNUSED(lun);
    UNUSED(scsiCommand);
    UNUSED(buffer);
    UNUSED(bufsize);

    return -1;
}

#ifdef USE_USB_MSC

// The drivers/usb_msc.h interface

#ifdef USE_SDCARD_SDIO
bool mscSdioInitDma(void)
{
    return SD_InitialiseHardware(NULL);
}
#endif

uint8_t mscStart(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (sdcardConfig()->mode != SDCARD_MODE_SDIO) {
            return 1;
        }
        USBD_STORAGE_fops = &USBD_MSC_MICRO_SDIO_fops;
        break;
#endif
    default:
        return 1;
    }

    // A missing or unreadable card fails here, before the host sees anything.
    if (USBD_STORAGE_fops->Init(0) != 0) {
        return 1;
    }

    // Descriptors must switch before the stack starts.
    usbDescriptorsSetMscMode(true);
    cdcUsbInit();

    return 0;
}

void mscTask(void)
{
    cdcUsbTask();
}

#endif // USE_USB_MSC
