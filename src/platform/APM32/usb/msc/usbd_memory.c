/**
 * @file        usbd_memory.c
 *
 * @brief       USB device memory management program body
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes ***************************************************************/
#include "usbd_memory.h"

/* Private includes *******************************************************/
#include "usbd_storage.h"
#include "platform.h"

/* Private macro **********************************************************/
#define MEMORY_LUN_NUM              1
#define MEMORY_BLOCK_NUM            80
#define MEMORY_BLOCK_SIZE           512

/* Private variables ******************************************************/

/* USB Mass storage Standard Inquiry Data */
const uint8_t memoryInquiryData[] =
{
    /* lun 0 */
    0x00,
    0x80,
    0x02,
    0x02,
    (USBD_LEN_STD_INQUIRY - 5),
    0x00,
    0x00,
    0x00,
    /* Manufacturer : 8 bytes */
    'G', 'e', 'e', 'h', 'y', ' ', ' ', ' ',
    /* Product : 16 Bytes */
    'S', 't', 'o', 'r', 'a', 'g', 'e', ' ',
    'D', 'i', 's', 'k', ' ', ' ', ' ', ' ',
    /* Version : 4 Bytes */
    '1', '.', '0', '0',
};

/* Private typedef ********************************************************/

/* USB FS MSC memory management handler */
USBD_MSC_MEMORY_T USBD_MEMORY_INTERFACE =
{
    "MSC Memory",
    (uint8_t*)memoryInquiryData,
    USBD_MSC_MemoryReadMaxLun,
    USBD_MSC_MemoryInit,
    USBD_MSC_MemoryReadCapacity,
    USBD_MSC_MemoryCheckReady,
    USBD_MSC_MemoryCheckWPR,
    USBD_MSC_MemoryReadData,
    USBD_MSC_MemoryWriteData,
};

/* Private function prototypes ********************************************/

/* External variables *****************************************************/

/* External functions *****************************************************/

/**
 * @brief   USB device MSC memory unit init handler
 *
 * @param   lun: lun number
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryInit(uint8_t lun)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.Init(lun);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.Init(lun);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.Init(lun);
#else
    UNUSED(lun);
#endif

    return usbStatus;
}

/**
 * @brief   USB device MSC memory unit read capacity handler
 *
 * @param   lun: lun number
 *
 * @param   blockNum: block number
 *
 * @param   blockSize: block size
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryReadCapacity(uint8_t lun, uint32_t* blockNum, \
                                      uint16_t* blockSize)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.GetCapacity(lun, blockNum, blockSize);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.GetCapacity(lun, blockNum, blockSize);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.GetCapacity(lun, blockNum, blockSize);
#else
    UNUSED(lun);
    UNUSED(blockNum);
    UNUSED(blockSize);
#endif

    return usbStatus;
}

/**
 * @brief   USB device MSC memory unit check read status handler
 *
 * @param   lun: lun number
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryCheckReady(uint8_t lun)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.IsReady(lun);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.IsReady(lun);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.IsReady(lun);
#else
    UNUSED(lun);
#endif

    return usbStatus;
}

/**
 * @brief   USB device MSC memory unit check write protected status handler
 *
 * @param   lun: lun number
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryCheckWPR(uint8_t lun)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.IsWriteProtected(lun);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.IsWriteProtected(lun);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.IsWriteProtected(lun);
#else
    UNUSED(lun);
#endif

    return usbStatus;
}

/**
 * @brief   USB device MSC memory read max LUN handler
 *
 * @param   None
 *
 * @retval  Max LUN number
 */
uint8_t USBD_MSC_MemoryReadMaxLun(void)
{
#ifdef USE_FLASHFS
    return USBD_MSC_EMFAT_fops.GetMaxLun();
#elif defined(USE_SDCARD_SDIO)
    return USBD_MSC_MICRO_SDIO_fops.GetMaxLun();
#elif defined(USE_SDCARD_SPI)
    return USBD_MSC_MICRO_SD_SPI_fops.GetMaxLun();
#else
    return 0;
#endif
}

/**
 * @brief   USB device MSC memory unit read data handler
 *
 * @param   lun: lun number
 *
 * @param   buffer: data buffer
 *
 * @param   blockAddr: block address
 *
 * @param   blockLength: block number
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryReadData(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, \
                                  uint16_t blockLength)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.Read(lun, buffer, blockAddr, blockLength);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.Read(lun, buffer, blockAddr, blockLength);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.Read(lun, buffer, blockAddr, blockLength);
#else
    UNUSED(lun);
    UNUSED(buffer);
    UNUSED(blockAddr);
    UNUSED(blockLength);
#endif

    return usbStatus;
}

/**
 * @brief   USB device MSC memory unit write data handler
 *
 * @param   lun: lun number
 *
 * @param   buffer: data buffer
 *
 * @param   blockAddr: block address
 *
 * @param   blockLength: block number
 *
 * @retval  USB device operation status
 */
USBD_STA_T USBD_MSC_MemoryWriteData(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, \
                                   uint16_t blockLength)
{
    USBD_STA_T usbStatus = USBD_OK;

#ifdef USE_FLASHFS
    USBD_MSC_EMFAT_fops.Write(lun, buffer, blockAddr, blockLength);
#elif defined(USE_SDCARD_SDIO)
    USBD_MSC_MICRO_SDIO_fops.Write(lun, buffer, blockAddr, blockLength);
#elif defined(USE_SDCARD_SPI)
    USBD_MSC_MICRO_SD_SPI_fops.Write(lun, buffer, blockAddr, blockLength);
#else
    UNUSED(lun);
    UNUSED(buffer);
    UNUSED(blockAddr);
    UNUSED(blockLength);
#endif

    return usbStatus;
}
