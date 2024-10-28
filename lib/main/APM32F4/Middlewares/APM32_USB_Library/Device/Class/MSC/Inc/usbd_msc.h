/*!
 * @file        usbd_msc.h
 *
 * @brief       usb device msc class handler header file
 *
 * @version     V1.0.0
 *
 * @date        2023-01-16
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

/* Define to prevent recursive inclusion */
#ifndef _USBD_MSC_H_
#define _USBD_MSC_H_

/* Includes */
#include "usbd_core.h"
#include "usbd_msc_scsi.h"
#include "usbd_msc_bot.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_MSC_Class
  @{
  */

/** @defgroup USBD_MSC_Macros Macros
  @{
*/

#define USBD_MSC_OUT_EP_ADDR            0x01
#define USBD_MSC_IN_EP_ADDR             0x81

#define USBD_MSC_FS_MP_SIZE             0x40
#define USBD_MSC_HS_MP_SIZE             0x200

#define USBD_CLASS_GET_MAX_LUN          0xFE
#define USBD_CLASS_BOT_RESET            0xFF


/**@} end of group USBD_MSC_Macros*/

/** @defgroup USBD_MSC_Structures Structures
  @{
  */

/**
 * @brief   USB device storage handler
 */
typedef struct
{
    const char*  memoryName;
    uint8_t*     inquiryData;
    uint8_t     (*MemoryReadMaxLun)(void);
    USBD_STA_T (*MemoryInit)(uint8_t lun);
    USBD_STA_T (*MemoryReadCapacity)(uint8_t lun, uint32_t* blockNum, uint16_t* blockSize);
    USBD_STA_T (*MemoryCheckReady)(uint8_t lun);
    USBD_STA_T (*MemoryCheckWPR)(uint8_t lun);
    USBD_STA_T (*MemoryReadData)(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, uint16_t blockLength);
    USBD_STA_T (*MemoryWriteData)(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, uint16_t blockLength);
} USBD_MSC_MEMORY_T;

typedef struct
{
  int8_t (* Init)(uint8_t lun);
  int8_t (* GetCapacity)(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
  int8_t (* IsReady)(uint8_t lun);
  int8_t (* IsWriteProtected)(uint8_t lun);
  int8_t (* Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;

} USBD_StorageTypeDef;

/**
 * @brief    MSC information management
 */
typedef struct
{
    uint8_t             epInAddr;
    uint8_t             epOutAddr;

    uint8_t             maxLun;
    uint8_t             itf;

    USBD_MSC_BOT_T*      mscBot;
    USBD_BOT_INFO_T     usbDevBOT;
    USBD_SCSI_INFO_T    usbDevSCSI;
} USBD_MSC_INFO_T;

extern USBD_CLASS_T USBD_MSC_CLASS;

/**@} end of group USBD_MSC_Structures*/

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

USBD_STA_T USBD_MSC_RegisterMemory(USBD_INFO_T* usbInfo, USBD_MSC_MEMORY_T* memory);

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */

#endif
