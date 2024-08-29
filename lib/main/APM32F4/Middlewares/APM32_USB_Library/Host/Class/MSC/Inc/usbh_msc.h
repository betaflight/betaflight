/*!
 * @file        usbh_msc.h
 *
 * @brief       usb host msc class handler header file
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
#ifndef _USBH_MSC_H_
#define _USBH_MSC_H_

/* Includes */
#include "usbh_core.h"
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_MSC_Class
  @{
  */

/** @defgroup USBH_MSC_Macros Macros
  @{
*/

#define USBH_CLASS_GET_MAX_LUN              0xFE
#define USBH_CLASS_BOT_RESET                0xFF
#define USBH_SUPPORTED_LUN_MAX              0x02
#define USBH_MSC_SCSI_CLASS_CODE            0x06

/**@} end of group USBH_MSC_Macros*/

/** @defgroup USBH_MSC_Enumerates Enumerates
  @{
  */

/**
 * @brief    MSC state table
 */
typedef enum
{
    USBH_MSC_INIT = 0,
    USBH_MSC_IDLE = 1,
    USBH_MSC_INQUIRY,
    USBH_MSC_TEST_UNIT_READY,
    USBH_MSC_REQUEST_SENSE,
    USBH_MSC_READ_CAPACITY,
    USBH_MSC_UNRECOVERED_STATE,

    USBH_MSC_RW_READ,
    USBH_MSC_RW_WRITE,
    USBH_MSC_RW_REQIEST_SENSE,
} USBH_MSC_STATE_T;

/**
 * @brief    MSC error state type
 */
typedef enum
{
    USBH_MSC_OK,
    USBH_MSC_BUSY,
    USBH_MSC_ERR,
} USBH_MSC_ERR_STATE_T;

/**
 * @brief    MSC class request state table
 */
typedef enum
{
    USBH_MSC_REQ_GET_MAX_LUN,
    USBH_MSC_REQ_BOT_RESET,
    USBH_MSC_REQ_CTRL_ERROR
} USBH_MSC_REQ_STATE_T;

/**
 * @brief    MSC protocol code
 */
typedef enum
{
    USBH_MSC_PROTOCOL_CBI_00 = 0x00,
    USBH_MSC_PROTOCOL_CBI_01 = 0x01,
    USBH_MSC_PROTOCOL_BBB    = 0x50,
    USBH_MSC_PROTOCOL_UAS    = 0x62
} USBH_MSC_PROTOCOL_CODE_T;

/**@} end of group USBH_MSC_Enumerates*/

/** @defgroup USBH_MSC_Structures Structures
  @{
  */

/**
 * @brief    MSC Storage info
 */
typedef struct
{
    USBH_SCSI_READ_CAPACITY_REQ_T   capacity;
    USBH_SCSI_INQUIRY_REQ_T         inquiryReq;
    USBH_SCSI_SENSE_REQ_T           sense;
    USBH_MSC_STATE_T                state;
    uint8_t                         changeState;
    uint8_t                         preReadyState;
    USBH_MSC_ERR_STATE_T            errState;
} USBH_MSC_STORAGE_INFO_T;

/* Host MSC class state handler function */
typedef USBH_STA_T(*USBH_MscStateHandler_T)(USBH_INFO_T* usbInfo, uint8_t lun);

/**
 * @brief    MSC information management
 */
typedef struct
{
    USBH_MSC_STATE_T        state;
    USBH_MSC_ERR_STATE_T    errState;
    USBH_MSC_REQ_STATE_T    preClassReqState;
    USBH_MSC_REQ_STATE_T    classReqState;
    uint8_t                 maxLun;
    uint8_t                 curLun;
    uint8_t                 opLun;
    uint32_t                timer;
    USBH_MSC_STORAGE_INFO_T storage[USBH_SUPPORTED_LUN_MAX];
    USBH_BOT_INFO_T         usbHostBOT;
} USBH_MSC_INFO_T;

extern USBH_CLASS_T USBH_MSC_CLASS;

/**@} end of group USBH_MSC_Structures*/

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

USBH_STA_T USBH_MSC_ReadDevInfo(USBH_INFO_T* usbInfo, uint8_t lun, USBH_MSC_STORAGE_INFO_T* device);
uint8_t USBH_MSC_DevStatus(USBH_INFO_T* usbInfo, uint8_t lun);
uint8_t USBH_MSC_ReadDevWP(USBH_INFO_T* usbInfo, uint8_t lun);
USBH_STA_T USBH_MSC_DevRead(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                            uint8_t* buffer, uint16_t cnt);
USBH_STA_T USBH_MSC_DevWrite(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                             uint8_t* buffer, uint16_t cnt);

/**@} end of group USBH_MSC_Functions */
/**@} end of group USBH_MSC_Class */
/**@} end of group APM32_USB_Library */

#endif
