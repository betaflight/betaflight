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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_USB_MSC

#include "blackbox/blackbox.h"
#include "build/build_config.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"
#include "msc/usbd_storage.h"
#include "pg/pg.h"

#include "usb_ch32h41x.h"
#include "usb_ch32h41x_usbhs_reg.h"
#include "usbd_core.h"
#include "usbd_msc.h"

#define MSC_IN_EP           0x81
#define MSC_OUT_EP          0x01
#define USBD_MSC_VID        0x1A86
#define USBD_MSC_PID        0xE205
#define USBD_MSC_MAX_POWER  100
#define MSC_USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

#define MSC_SD_INIT_TIMEOUT_MS 2000
#define MSC_SD_IO_TIMEOUT_MS   1000

#ifdef USE_SDCARD_SPI
#include "drivers/sdcard.h"
#include "pg/sdcard.h"

typedef enum {
    CH32_SD_OP_PENDING = 0,
    CH32_SD_OP_OK,
    CH32_SD_OP_ERROR,
} ch32SdOpStatus_e;

static volatile ch32SdOpStatus_e ch32SdOpStatus;

static void ch32SdSpiOpCallback(sdcardBlockOperation_e operation, uint32_t blockIndex, uint8_t *buffer, uint32_t callbackData)
{
    UNUSED(operation);
    if (blockIndex == callbackData && buffer != NULL) {
        ch32SdOpStatus = CH32_SD_OP_OK;
    } else {
        ch32SdOpStatus = CH32_SD_OP_ERROR;
    }
}

static bool ch32SdSpiWaitReady(const timeMs_t start)
{
    while (sdcard_poll() == 0) {
        if (!sdcard_isFunctional() || !sdcard_isInitialized() ||
            cmpTimeMs(millis(), start) > MSC_SD_IO_TIMEOUT_MS) {
            return false;
        }
    }
    return sdcard_isFunctional() && sdcard_isInitialized();
}

static int8_t ch32SdSpiInit(uint8_t lun)
{
    UNUSED(lun);
    sdcard_init(sdcardConfig());
    return 0;
}

static int8_t ch32SdSpiGetCapacity(uint8_t lun, uint32_t *block_num, usbd_msc_block_size_t *block_size)
{
    UNUSED(lun);
    const sdcardMetadata_t *meta = sdcard_getMetadata();
    if (!meta || meta->numBlocks == 0) {
        return -1;
    }
    *block_num = meta->numBlocks;
    *block_size = 512;
    return 0;
}

static int8_t ch32SdSpiIsReady(uint8_t lun)
{
    UNUSED(lun);
    return (sdcard_poll() && sdcard_isFunctional() && sdcard_isInitialized()) ? 0 : -1;
}

static int8_t ch32SdSpiIsWriteProtected(uint8_t lun)
{
    UNUSED(lun);
    return 0;
}

static int8_t ch32SdSpiRead(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    UNUSED(lun);
    for (uint16_t i = 0; i < blk_len; i++) {
        const uint32_t lba = blk_addr + i;
        const timeMs_t start = millis();
        ch32SdOpStatus = CH32_SD_OP_PENDING;
        while (sdcard_readBlock(lba, buf + (512u * i), ch32SdSpiOpCallback, lba) == 0) {
            if (!sdcard_isFunctional() || !sdcard_isInitialized() ||
                cmpTimeMs(millis(), start) > MSC_SD_IO_TIMEOUT_MS) {
                return -1;
            }
            (void)sdcard_poll();
        }
        while (ch32SdOpStatus == CH32_SD_OP_PENDING) {
            (void)sdcard_poll();
            if (ch32SdOpStatus == CH32_SD_OP_PENDING &&
                (!sdcard_isFunctional() || !sdcard_isInitialized() ||
                 cmpTimeMs(millis(), start) > MSC_SD_IO_TIMEOUT_MS)) {
                return -1;
            }
        }
        if (ch32SdOpStatus != CH32_SD_OP_OK || !ch32SdSpiWaitReady(start)) {
            return -1;
        }
    }
    return 0;
}

static int8_t ch32SdSpiWrite(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    UNUSED(lun);
    for (uint16_t i = 0; i < blk_len; i++) {
        const uint32_t lba = blk_addr + i;
        const timeMs_t start = millis();
        ch32SdOpStatus = CH32_SD_OP_PENDING;
        sdcardOperationStatus_e st;
        while ((st = sdcard_writeBlock(lba, buf + (512u * i), ch32SdSpiOpCallback, lba)) == SDCARD_OPERATION_BUSY) {
            if (!sdcard_isFunctional() || !sdcard_isInitialized() ||
                cmpTimeMs(millis(), start) > MSC_SD_IO_TIMEOUT_MS) {
                return -1;
            }
            (void)sdcard_poll();
        }
        if (st == SDCARD_OPERATION_IN_PROGRESS) {
            while (ch32SdOpStatus == CH32_SD_OP_PENDING) {
                (void)sdcard_poll();
                if (ch32SdOpStatus == CH32_SD_OP_PENDING &&
                    (!sdcard_isFunctional() || !sdcard_isInitialized() ||
                     cmpTimeMs(millis(), start) > MSC_SD_IO_TIMEOUT_MS)) {
                    return -1;
                }
            }
            if (ch32SdOpStatus != CH32_SD_OP_OK || !ch32SdSpiWaitReady(start)) {
                return -1;
            }
        } else if (st != SDCARD_OPERATION_SUCCESS) {
            return -1;
        }
    }
    return 0;
}

static int8_t ch32SdSpiGetMaxLun(void)
{
    return 0;
}

static const uint8_t ch32SdSpiInquiryData[USBD_STD_INQUIRY_LENGTH] = {
    0x00, 0x80, 0x02, 0x02,
    (USBD_STD_INQUIRY_LENGTH - 5),
    0x00, 0x00, 0x00,
    'B', 'E', 'T', 'A', 'F', 'L', 'T', ' ',
    'S', 'D', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e', ' ', ' ', ' ', ' ', ' ', ' ',
    '0', '.', '0', '1'
};

USBD_MSC_StorageType USBD_MSC_MICRO_SD_SPI_fops = {
    .Init = ch32SdSpiInit,
    .GetCapacity = ch32SdSpiGetCapacity,
    .IsReady = ch32SdSpiIsReady,
    .IsWriteProtected = ch32SdSpiIsWriteProtected,
    .Read = ch32SdSpiRead,
    .Write = ch32SdSpiWrite,
    .GetMaxLun = ch32SdSpiGetMaxLun,
    .pInquiry = (int8_t *)ch32SdSpiInquiryData,
};
#endif

static const uint8_t mscDeviceDescriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_MSC_VID, USBD_MSC_PID, 0x0100, 0x01)
};

static const uint8_t mscConfigDescriptorHs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(MSC_USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MSC_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_HS, 0x04)
};

static const uint8_t mscConfigDescriptorFs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(MSC_USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MSC_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_FS, 0x04)
};

static const uint8_t mscOtherSpeedDescriptorHs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(MSC_USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MSC_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_FS, 0x04)
};

static const uint8_t mscOtherSpeedDescriptorFs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(MSC_USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MSC_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_HS, 0x04)
};

static const uint8_t mscDeviceQualifierDescriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01)
};

static char mscSerialString[25];

static const char *const mscStringDescriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "Betaflight",
    USBD_PRODUCT_STRING,
    NULL,
    "Betaflight MSC",
};

static const uint8_t *mscDeviceDescriptorCb(uint8_t speed)
{
    UNUSED(speed);
    return mscDeviceDescriptor;
}

static const uint8_t *mscConfigDescriptorCb(uint8_t speed)
{
    return (speed == USB_SPEED_HIGH) ? mscConfigDescriptorHs : mscConfigDescriptorFs;
}

static const uint8_t *mscDeviceQualifierDescriptorCb(uint8_t speed)
{
    UNUSED(speed);
    return mscDeviceQualifierDescriptor;
}

static const uint8_t *mscOtherSpeedDescriptorCb(uint8_t speed)
{
    return (speed == USB_SPEED_HIGH) ? mscOtherSpeedDescriptorHs : mscOtherSpeedDescriptorFs;
}

static const char *mscStringDescriptorCb(uint8_t speed, uint8_t index)
{
    UNUSED(speed);
    if (index >= ARRAYLEN(mscStringDescriptors)) {
        return NULL;
    }
    if (index == USB_STRING_SERIAL_INDEX) {
        return ch32UsbGetUidSerialString(mscSerialString);
    }
    return mscStringDescriptors[index];
}

static const struct usb_descriptor mscDescriptor = {
    .device_descriptor_callback = mscDeviceDescriptorCb,
    .config_descriptor_callback = mscConfigDescriptorCb,
    .device_quality_descriptor_callback = mscDeviceQualifierDescriptorCb,
    .other_speed_descriptor_callback = mscOtherSpeedDescriptorCb,
    .string_descriptor_callback = mscStringDescriptorCb,
};

static struct usbd_interface mscIntf;

static void usbdMscEventHandler(uint8_t busid, uint8_t event)
{
    UNUSED(busid);
    UNUSED(event);
}

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    UNUSED(busid);
    uint32_t num = 0;
    usbd_msc_block_size_t size = 512;

    if (USBD_STORAGE_fops && USBD_STORAGE_fops->GetCapacity &&
        USBD_STORAGE_fops->GetCapacity(lun, &num, &size) == 0 && size != 0) {
        *block_num = num;
        *block_size = size;
    } else {
        *block_num = 0;
        *block_size = 512;
    }
}

bool usbd_msc_get_inquiry(uint8_t busid, uint8_t lun, uint8_t *inquiry_buf)
{
    UNUSED(busid);
    UNUSED(lun);
    if (!USBD_STORAGE_fops || !USBD_STORAGE_fops->pInquiry || !inquiry_buf) {
        return false;
    }
    memcpy(inquiry_buf, USBD_STORAGE_fops->pInquiry, USBD_STD_INQUIRY_LENGTH);
    return true;
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);
    if (!USBD_STORAGE_fops || !USBD_STORAGE_fops->Read || (length % 512u) != 0) {
        return -1;
    }
    const uint16_t blkLen = (uint16_t)(length / 512u);
    if (USBD_STORAGE_fops->Read(lun, buffer, sector, blkLen) != 0) {
        return -1;
    }
    mscSetActive();
    return 0;
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    UNUSED(busid);
    if (!USBD_STORAGE_fops || !USBD_STORAGE_fops->Write || (length % 512u) != 0) {
        return -1;
    }
    if (USBD_STORAGE_fops->IsWriteProtected && USBD_STORAGE_fops->IsWriteProtected(lun) != 0) {
        return -1;
    }
    const uint16_t blkLen = (uint16_t)(length / 512u);
    if (USBD_STORAGE_fops->Write(lun, buffer, sector, blkLen) != 0) {
        return -1;
    }
    mscSetActive();
    return 0;
}

uint8_t mscStart(void)
{
    USBD_STORAGE_fops = NULL;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#ifdef USE_SDCARD_SPI
        USBD_STORAGE_fops = &USBD_MSC_MICRO_SD_SPI_fops;
#endif
        if (!USBD_STORAGE_fops) {
            return 1;
        }
        if (USBD_STORAGE_fops->Init && USBD_STORAGE_fops->Init(0) != 0) {
            return 1;
        }
        {
            const timeMs_t start = millis();
            while (USBD_STORAGE_fops->IsReady && USBD_STORAGE_fops->IsReady(0) != 0) {
                if (cmpTimeMs(millis(), start) > MSC_SD_INIT_TIMEOUT_MS) {
                    return 1;
                }
            }
        }
        break;
#endif
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        USBD_STORAGE_fops = &USBD_MSC_EMFAT_fops;
        if (USBD_STORAGE_fops->Init && USBD_STORAGE_fops->Init(0) != 0) {
            return 1;
        }
        break;
#endif
    default:
        return 1;
    }

    if (!ch32UsbHsPhyInit()) {
        return 1;
    }

    usb_rxsof_handler = NULL;
#ifdef CONFIG_USBDEV_ADVANCE_DESC
    usbd_desc_register(0, &mscDescriptor);
#endif
    usbd_add_interface(0, usbd_msc_init_intf(0, &mscIntf, MSC_OUT_EP, MSC_IN_EP));
    const bool readOnly = USBD_STORAGE_fops->IsWriteProtected && (USBD_STORAGE_fops->IsWriteProtected(0) != 0);
    usbd_msc_set_readonly(0, readOnly);
    usbd_initialize(0, 0, usbdMscEventHandler);

    NVIC_SetPriority(USBHS_IRQn, NVIC_PRIO_USB);
    NVIC_EnableIRQ(USBHS_IRQn);

    return 0;
}

void mscTask(void)
{
    usbd_msc_polling(0);
}

#endif /* USE_USB_MSC */
