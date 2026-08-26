/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

// USB MSC (mass storage) device backed by an SD card (SDIO or SPI) or
// emulated FAT flash: USB descriptors and block read/write callbacks.

#include "platform.h"

#include "build/version.h"

#include "usbd_core.h"
#include "usbd_msc.h"
#include "hpm_sdmmc_sd.h"
#include "sdio_hpmicro.h"

#include "drivers/usb_msc.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "msc_hpm.h"
#include "usb_descriptor_hpm.h"
#include "nvic_hpmicro.h"

#ifdef USE_SDCARD_SPI
#include "drivers/sdcard.h"
#endif

#ifdef USE_FLASHFS
#include "msc/emfat.h"
#include "msc/usbd_storage_emfat.h"
#endif

#define MSC_IN_EP  0x81
#define MSC_OUT_EP 0x02

#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

/* Match the SD SPI driver's worst-case operation timeouts. */
#define MSC_SD_SPI_READ_TIMEOUT_MILLIS  100U
#define MSC_SD_SPI_WRITE_TIMEOUT_MILLIS 250U
#define MSC_SD_SPI_INIT_TIMEOUT_MILLIS  200U

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_MSC_PID, 0x0200, 0x01),
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *const string_descriptors[] = {
    (const char[]) { 0x09, 0x04 },      /* Langid */
    FC_FIRMWARE_NAME,           /* Manufacturer */
    USBD_PRODUCT_STRING " MSC", /* Product */
    NULL,                       /* Serial Number: generated from OTP UUID */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void) speed;

    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void) speed;

    return device_quality_descriptor;
}

static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return other_speed_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return other_speed_config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void) speed;

    if (index == USB_STRING_SERIAL_INDEX) {
        return hpmUsbGetSerialNumber();
    }
    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

static const struct usb_descriptor msc_device_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static struct usbd_interface intf0;
extern sdmmc_host_t g_sdmmc_host;
extern sd_card_t g_sd;
static hpmMscStorage_e mscStorage;

#ifdef USE_SDCARD_SPI
static int mscSdSpiRead(uint32_t sector, uint8_t *buffer, uint32_t blockCount)
{
    for (uint32_t i = 0; i < blockCount; i++) {
        const timeMs_t start = millis();
        while (!sdcard_readBlock(sector + i, buffer + 512U * i, NULL, 0)) {
            if (!sdcard_isFunctional() || (timeMs_t) (millis() - start) >= MSC_SD_SPI_READ_TIMEOUT_MILLIS) {
                return -1;
            }
            sdcard_poll();
        }
        while (!sdcard_poll()) {
            if (!sdcard_isFunctional() || (timeMs_t) (millis() - start) >= MSC_SD_SPI_READ_TIMEOUT_MILLIS) {
                return -1;
            }
        }
    }

    mscSetActive();
    return 0;
}

static int mscSdSpiWrite(uint32_t sector, uint8_t *buffer, uint32_t blockCount)
{
    for (uint32_t i = 0; i < blockCount; i++) {
        const timeMs_t start = millis();
        sdcardOperationStatus_e status;
        do {
            status = sdcard_writeBlock(sector + i, buffer + 512U * i, NULL, 0);
            if (status == SDCARD_OPERATION_BUSY) {
                if (!sdcard_isFunctional() || (timeMs_t) (millis() - start) >= MSC_SD_SPI_WRITE_TIMEOUT_MILLIS) {
                    return -1;
                }
                sdcard_poll();
            }
        } while (status == SDCARD_OPERATION_BUSY);

        if (status == SDCARD_OPERATION_IN_PROGRESS) {
            while (!sdcard_poll()) {
                if (!sdcard_isFunctional() || (timeMs_t) (millis() - start) >= MSC_SD_SPI_WRITE_TIMEOUT_MILLIS) {
                    return -1;
                }
            }
        } else if (status != SDCARD_OPERATION_SUCCESS) {
            return -1;
        }
    }

    mscSetActive();
    return 0;
}
#endif

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void) busid;

    switch (event) {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    (void) busid;
    (void) lun;

    *block_num = 0;
    *block_size = 0;

    switch (mscStorage) {
#ifdef USE_SDCARD_SDIO
    case HPM_MSC_STORAGE_SDIO:
        if (g_sd.host->card_inserted) {
            *block_num = g_sd.block_count;
            *block_size = g_sd.block_size;
        }
        break;
#endif
#ifdef USE_SDCARD_SPI
    case HPM_MSC_STORAGE_SD_SPI:
        if (sdcard_isFunctional()) {
            *block_num = sdcard_getMetadata()->numBlocks;
            *block_size = 512;
        }
        break;
#endif
#ifdef USE_FLASHFS
    case HPM_MSC_STORAGE_EMFAT:
        *block_num = emfat.disk_sectors;
        *block_size = 512;
        break;
#endif
    default:
        break;
    }
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    (void) busid;
    (void) lun;

    switch (mscStorage) {
#ifdef USE_SDCARD_SDIO
    case HPM_MSC_STORAGE_SDIO: {
        if (!g_sd.host->card_inserted) {
            return -1;
        }
        uint32_t sysBufAddr = core_local_mem_to_sys_address(0, (uint32_t) buffer);
        hpm_stat_t status = sd_read_blocks(&g_sd, (uint8_t *) sysBufAddr, sector, length / g_sd.block_size);
        return (status != status_success) ? -1 : 0;
    }
#endif
#ifdef USE_SDCARD_SPI
    case HPM_MSC_STORAGE_SD_SPI:
        return mscSdSpiRead(sector, buffer, length / 512U);
#endif
#ifdef USE_FLASHFS
    case HPM_MSC_STORAGE_EMFAT:
        emfat_read(&emfat, buffer, sector, length / 512U);
        mscSetActive();
        return 0;
#endif
    default:
        return -1;
    }
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    (void) busid;
    (void) lun;

    switch (mscStorage) {
#ifdef USE_SDCARD_SDIO
    case HPM_MSC_STORAGE_SDIO: {
        if (!g_sd.host->card_inserted) {
            return -1;
        }
        uint32_t sysBufAddr = core_local_mem_to_sys_address(0, (uint32_t) buffer);
        hpm_stat_t status = sd_write_blocks(&g_sd, (uint8_t *) sysBufAddr, sector, length / g_sd.block_size);
        return (status != status_success) ? -1 : 0;
    }
#endif
#ifdef USE_SDCARD_SPI
    case HPM_MSC_STORAGE_SD_SPI:
        return mscSdSpiWrite(sector, buffer, length / 512U);
#endif
#ifdef USE_FLASHFS
    case HPM_MSC_STORAGE_EMFAT:
        return -1;
#endif
    default:
        return -1;
    }
}

int msc_device_init(uint8_t busid, uint32_t reg_base, hpmMscStorage_e storage)
{
    mscStorage = storage;

    switch (storage) {
#ifdef USE_SDCARD_SDIO
    case HPM_MSC_STORAGE_SDIO: {
        hpm_stat_t status = hpmSdioConfigureHost(&g_sdmmc_host, hpmPlicPriorityFromNvic(NVIC_PRIO_SDIO_DMA));
        if (status != status_success) {
            return -1;
        }

        status = sd_init(&g_sd);
        if (status != status_success) {
            return -1;
        }
        break;
    }
#endif
#ifdef USE_SDCARD_SPI
    case HPM_MSC_STORAGE_SD_SPI:
        {
        const timeMs_t start = millis();
        while (!sdcard_isInitialized()) {
            if (!sdcard_isFunctional() || (timeMs_t) (millis() - start) >= MSC_SD_SPI_INIT_TIMEOUT_MILLIS) {
                return -1;
            }
            sdcard_poll();
        }
        }
        break;
#endif
#ifdef USE_FLASHFS
    case HPM_MSC_STORAGE_EMFAT:
        if (!emfat.disk_sectors) {
            return -1;
        }
        break;
#endif
    default:
        return -1;
    }

    usbd_desc_register(busid, &msc_device_descriptor);
    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));
    usbd_msc_set_readonly(busid, storage == HPM_MSC_STORAGE_EMFAT);

    usbd_initialize(busid, reg_base, usbd_event_handler);

    return 0;
}
