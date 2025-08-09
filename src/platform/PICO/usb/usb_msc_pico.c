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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_USB_MSC

#include <string.h>

#include "build/build_config.h"

#include "common/utils.h"

#include "blackbox/blackbox.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

// Storage backends (SD SPI / EMFAT) use the legacy USBD storage fops interface.
// On PICO we include the lightweight header with the callback struct.
#include "usbd_msc_mem.h"

// Backends provided by common MSC storage layers
#ifdef USE_SDCARD_SPI
// Provide an internal SPI SD backend for PICO builds so we don't rely on
// non-PICO storage sources being compiled in.
#include "drivers/sdcard.h"
#include "pg/sdcard.h"
static int8_t pico_sdspi_Init(uint8_t lun)
{
    UNUSED(lun);
    sdcard_init(sdcardConfig());
    return 0;
}
static int8_t pico_sdspi_GetCapacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    UNUSED(lun);
    *block_num = sdcard_getMetadata()->numBlocks;
    *block_size = 512;
    return 0;
}
static int8_t pico_sdspi_IsReady(uint8_t lun)
{
    UNUSED(lun);
    return sdcard_poll() ? 0 : -1;
}
static int8_t pico_sdspi_IsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return 0;
}
static int8_t pico_sdspi_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_count)
{
    UNUSED(lun);
    for (uint16_t i = 0; i < blk_count; i++) {
        while (sdcard_readBlock(blk_addr + i, buf + (512u * i), NULL, 0) == 0) {}
        while (sdcard_poll() == 0) {}
    }
    mscSetActive();
    return 0;
}
static int8_t pico_sdspi_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_count)
{
    UNUSED(lun);
    for (uint16_t i = 0; i < blk_count; i++) {
        sdcardOperationStatus_e st = sdcard_writeBlock(blk_addr + i, buf + (512u * i), NULL, 0);
        if (st == SDCARD_OPERATION_IN_PROGRESS) {
            while (sdcard_poll() == 0) {}
        } else if (st == SDCARD_OPERATION_SUCCESS) {
            // ok
        } else {
            return -1;
        }
    }
    mscSetActive();
    return 0;
}
static int8_t pico_sdspi_GetMaxLun(void)
{
    return 0;
}
static uint8_t pico_sdspi_Inquirydata[] = {
    0x00, 0x80, 0x02, 0x02,
    (USBD_STD_INQUIRY_LENGTH - 5),
    0x00, 0x00, 0x00,
    'B','E','T','A','F','L','T',' ',
    'S','D',' ','S','t','o','r','a','g','e',' ',' ',' ',' ',' ',
    '0','.','0','1'
};
USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops = {
    pico_sdspi_Init,
    pico_sdspi_GetCapacity,
    pico_sdspi_IsReady,
    pico_sdspi_IsWriteProtected,
    pico_sdspi_Read,
    pico_sdspi_Write,
    pico_sdspi_GetMaxLun,
    (int8_t *)pico_sdspi_Inquirydata,
};
#endif
#ifdef USE_FLASHFS
extern USBD_STORAGE_cb_TypeDef USBD_MSC_EMFAT_fops;
#endif

#include "tusb_config.h"

// TinyUSB device API
#define _STDIO_H_
#include "tusb.h"

// CDC is initialized in usb_cdc.c which also initializes TinyUSB core.
// Here we only ensure TinyUSB is up and register MSC callbacks via TinyUSB.

// TinyUSB MSC callbacks - forward to existing USBD_STORAGE_fops for SD/FLASH

bool pico_msc_active = false;

// Selected storage backend (registered in mscStart)
static USBD_STORAGE_cb_TypeDef *msc_storage = NULL;

#define MSC_SD_INIT_TIMEOUT 2000

static int8_t storage_get_capacity(uint8_t lun, uint32_t *block_count, uint32_t *block_size)
{
    if (!msc_storage) {
        return -1;
    }
    return msc_storage->GetCapacity(lun, block_count, block_size);
}

// TinyUSB: Called when received SCSI READ10/WRITE10 command.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    if (!msc_storage) {
        return -1;
    }
    // offset is always 0 for 512-byte block alignment, but handle generically
    uint8_t *buf = (uint8_t *)buffer;
    if (offset == 0 && (bufsize % 512u) == 0) {
        uint16_t block_len = (uint16_t)(bufsize / 512u);
        if (msc_storage->Read(lun, buf, lba, block_len) != 0) {
            return -1;
        }
        mscSetActive();
        return (int32_t)bufsize;
    }
    // Fallback: read one block then memcpy slice
    uint8_t temp[512];
    if (msc_storage->Read(lun, temp, lba, 1) != 0) {
        return -1;
    }
    const uint32_t n = MIN(bufsize, 512u - offset);
    memcpy(buf, temp + offset, n);
    mscSetActive();
    return (int32_t)n;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    if (!msc_storage) {
        return false;
    }
    return msc_storage->IsWriteProtected(lun) == 0;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!msc_storage) {
        return -1;
    }
    if (msc_storage->IsWriteProtected(lun) != 0) {
        return -1;
    }
    if (offset == 0 && (bufsize % 512u) == 0) {
        uint16_t block_len = (uint16_t)(bufsize / 512u);
        if (msc_storage->Write(lun, (uint8_t *)buffer, lba, block_len) != 0) {
            return -1;
        }
        mscSetActive();
        return (int32_t)bufsize;
    }
    // Fallback: read-modify-write block
    uint8_t temp[512];
    if (msc_storage->Read(lun, temp, lba, 1) != 0) {
        return -1;
    }
    const uint32_t n = MIN(bufsize, 512u - offset);
    memcpy(temp + offset, buffer, n);
    if (msc_storage->Write(lun, temp, lba, 1) != 0) {
        return -1;
    }
    mscSetActive();
    return (int32_t)n;
}

// Invoked to determine disk size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    uint32_t count = 0;
    uint32_t size32 = 512;
    if (storage_get_capacity(lun, &count, &size32) == 0) {
        *block_count = count;
        *block_size = (uint16_t)size32;
    } else {
        *block_count = 0;
        *block_size = 512;
    }
}

// Invoked to check if device is ready for read/write
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    if (!msc_storage) {
        return false;
    }
    return msc_storage->IsReady(lun) == 0;
}

// SCSI INQUIRY
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    UNUSED(lun);

    if (msc_storage && msc_storage->pInquiry) {
        // Map from standard 36-byte inquiry to TinyUSB split fields
        memcpy(vendor_id, &msc_storage->pInquiry[8], 8);
        memcpy(product_id, &msc_storage->pInquiry[16], 16);
        memcpy(product_rev, &msc_storage->pInquiry[32], 4);
        return;
    }
    const char vid[8] = { 'B','e','t','a','F','l','t',' ' };
    const char pid[16] = { 'M','S','C',' ','S','t','o','r','a','g','e',' ',' ',' ',' ',' ' };
    const char rev[4] = { '0','.','0','1' };
    memcpy(vendor_id, vid, 8);
    memcpy(product_id, pid, 16);
    memcpy(product_rev, rev, 4);
}

// SCSI REQUEST SENSE, leave default if not needed
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    UNUSED(lun); UNUSED(power_condition); UNUSED(start); UNUSED(load_eject);
    return true;
}

// Handle other SCSI commands not handled by simple callbacks above
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void* buffer, uint16_t bufsize)
{
    UNUSED(lun);
    UNUSED(scsi_cmd);
    UNUSED(buffer);
    UNUSED(bufsize);
    // Return -1 to indicate unsupported command
    return -1;
}

uint8_t mscStart(void)
{
    msc_storage = NULL;

    // Decide backend based on configuration
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        #ifdef USE_SDCARD_SPI
        msc_storage = &USBD_MSC_MICRO_SD_SPI_fops;
        #endif
        if (!msc_storage) {
            return 1;
        }
        if (msc_storage->Init) {
            msc_storage->Init(0);
        }
        {
            const uint32_t start = millis();
            while (msc_storage->IsReady && msc_storage->IsReady(0) != 0) {
                if (cmpTimeMs(millis(), start) > MSC_SD_INIT_TIMEOUT) {
                    return 1; // error
                }
            }
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        msc_storage = &USBD_MSC_EMFAT_fops;
        if (msc_storage->Init) {
            msc_storage->Init(0);
        }
        break;
#endif
    default:
        return 1;
    }

    // TinyUSB core should already be initialized by CDC init; ensure it is
    if (!tud_inited()) {
        tusb_init();
    }

    pico_msc_active = true;
    return 0;
}

void mscTask(void)
{
    tud_task();
}
#endif // USE_USB_MSC


