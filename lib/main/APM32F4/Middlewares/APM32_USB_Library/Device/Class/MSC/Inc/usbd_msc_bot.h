/*!
 * @file        usbd_msc_bot.h
 *
 * @brief       usb device msc bot handler header file
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
#ifndef _USBD_MSC_BOT_H_
#define _USBD_MSC_BOT_H_

/* Includes */
#include "usbd_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_MSC_Class
  @{
  */

/** @defgroup USBD_MSC_Macros Macros
  @{
*/

/* CBW parameter */
#define USBD_MSC_BOT_CBW_SIGNATURE               (uint32_t)(0x43425355)
#define USBD_MSC_BOT_CBW_TAG                     (uint32_t)(0x20304050)
#define USBD_MSC_BOT_CBW_LEN                     31
#define USBD_BOT_CBW_CB_LEN                      16
#define USBD_LEN_CBW                             10

/* CSW parameter */
#define USBD_MSC_BOT_CSW_SIGNATURE               (uint32_t)(0x53425355)
#define USBD_MSC_BOT_CSW_LEN                     13
#define USBD_LEN_CSW_MAX                         63

#ifndef USBD_SUP_MSC_MEDIA_PACKET
#define USBD_SUP_MSC_MEDIA_PACKET                512U
#endif /* USBD_SUP_MSC_MEDIA_PACKET */

/**@} end of group USBD_MSC_Macros*/

/** @defgroup USBD_MSC_Enumerates Enumerates
  @{
  */

/**
 * @brief    SCSI transmission state of BOT
 */
typedef enum
{
    USBD_BOT_IDLE,
    USBD_BOT_DATAOUT,
    USBD_BOT_DATAIN,
    USBD_BOT_DATAIN_LAST,
    USBD_BOT_DATA_SEND,
    USBD_BOT_NO_DATA,
} USBD_BOT_STATE_T;

/**
 * @brief    SCSI transmission status of BOT
 */
typedef enum
{
    USBD_BOT_NORMAL,
    USBD_BOT_RECOVERY,
    USBD_BOT_ERR,
} USBD_BOT_STATUS_T;

/**
 * @brief    CSW status of BOT
 */
typedef enum
{
    USBD_BOT_CSW_OK,
    USBD_BOT_CSW_FAIL,
    USBD_BOT_CSW_ERROR,
} USBD_BOT_CSW_STA_T;

/**@} end of group USBD_MSC_Enumerates*/

/** @defgroup USBD_MSC_Structures Structures
  @{
  */

/**
 * @brief   USB device SCSI handler
 */
typedef struct
{
    USBD_STA_T(*MemoryInit)(uint8_t lun);
    USBD_STA_T(*MemoryReadCapacity)(uint8_t lun, uint32_t* blockNum, uint16_t* blockSize);
    USBD_STA_T(*MemoryCheckReady)(uint8_t lun);
    USBD_STA_T(*MemoryCheckWPR)(uint8_t lun);
    USBD_STA_T(*MemoryReadData)(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, uint16_t blockLength);
    USBD_STA_T(*MemoryWriteData)(uint8_t lun, uint8_t* buffer, uint32_t blockAddr, uint16_t blockLength);
} USBD_MSC_SCSI_T;

/**
 * @brief    CBW flag
 */
typedef union
{
    uint8_t CBW_Flag;

    struct
    {
        uint8_t reserved : 7;
        uint8_t dir      : 1;
    } CBW_FLAG_B;

} USBD_BOT_CBW_FLAG_T;

/**
 * @brief    Command Block Wrapper
 */
typedef union
{
    struct
    {
        uint32_t            dSignature;
        uint32_t            dTag;
        uint32_t            dDataXferLen;
        USBD_BOT_CBW_FLAG_T bmFlags;
        uint8_t             bLUN;
        uint8_t             bCBLen;
        uint8_t             CB[16];
    } DATA_FIELD;

    uint8_t buffer[31];
} USBD_BOT_CBW_T;

/**
 * @brief    Command Status Wrapper
 */
typedef union
{
    struct
    {
        uint32_t dSignature;
        uint32_t dTag;
        uint32_t dDataResidue;
        uint8_t  bStatus;
    } DATA_FIELD;

    uint8_t buffer[13];
} USBD_BOT_CSW_T;

/**
 * @brief   USB device BOT handler
 */
typedef struct
{
    USBD_STA_T(*Init)(USBD_INFO_T* usbInfo);
    USBD_STA_T(*DeInit)(USBD_INFO_T* usbInfo);
} USBD_MSC_BOT_T;

/**
 * @brief    BOT transmission parameter
 */
typedef struct
{
    USBD_BOT_CBW_T      CBW;
    USBD_BOT_CSW_T      CSW;
    USBD_BOT_CSW_STA_T  cswStatus;
    uint8_t cbwStatus;
} USBD_BOT_CMDPACK_T;

/**
 * @brief    MSC BOT information
 */
typedef struct
{
    uint8_t             state;
    uint8_t             status;
    USBD_BOT_CMDPACK_T  cmdPack;
    uint32_t            dataLen;
    uint8_t             data[USBD_SUP_MSC_MEDIA_PACKET];
} USBD_BOT_INFO_T;

/**@} end of group USBD_MSC_Structures*/

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

USBD_STA_T USBD_MSC_BOT_Init(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_MSC_BOT_DeInit(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_MSC_BOT_Reset(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_MSC_BOT_CBW_Decode(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_MSC_BOT_ClearFeature(USBD_INFO_T* usbInfo, uint8_t epNum);
USBD_STA_T USBD_MSC_BOT_SendCSW(USBD_INFO_T* usbInfo, USBD_BOT_CSW_STA_T status);

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */

#endif
