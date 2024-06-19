/*!
 * @file        usbh_cdc.h
 *
 * @brief       USB CDC core function head file
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
#ifndef _USBH_CDC_H_
#define _USBH_CDC_H_

/* Includes */
#include "usbh_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_CDC_Class
  @{
  */

/** @defgroup USBH_CDC_Macros Macros
  @{
*/

#define USBH_CDC_ACM_CODE           0x02
#define USBH_CDC_AT_COMMAND_CODE    0x01
#define USBH_CDC_LINE_CODING_NUM    0x07

/**@} end of group USBH_CDC_Macros*/

/** @defgroup USBH_CDC_Enumerates Enumerates
  @{
  */

/**
 * @brief    USB CDC state table
 */
typedef enum
{
    USBH_CDC_INIT = 0,
    USBH_CDC_IDLE = 1,
    USBH_CDC_SET_LINE_CODING_STATE,
    USBH_CDC_GET_LINE_CODING_STATE,
    USBH_CDC_SET_CONTROL_LINE_STATE,
    USBH_CDC_TRANSFER_DATA_STATE,
    USBH_CDC_ERROR_STATE,
} USBH_CDC_STATE_T;

/**
 * @brief    USB CDC data state table
 */
typedef enum
{
    USBH_CDC_DATA_IDLE = 0,
    USBH_CDC_DATA_SEND,
    USBH_CDC_DATA_SEND_WAIT,
    USBH_CDC_DATA_RECEIVE,
    USBH_CDC_DATA_RECEIVE_WAIT,
    USBH_CDC_DATA_ERROR,
} USBH_CDC_DATA_STATE_T;

/**
 * @brief   USB CDC device class requests type
 */
typedef enum
{
    USBH_CDC_REQ_SEND_ENCAPSULATED_COMMAND = 0,
    USBH_CDC_REQ_SET_LINE_CODING = 0x20,
    USBH_CDC_REQ_GET_LINE_CODING,
    USBH_CDC_REQ_SET_CONTROL_LINE_STATE,
} USBH_CDC_REQ_TYPE_T;

/**@} end of group USBH_CDC_Enumerates*/

/** @defgroup USBH_CDC_Structures Structures
  @{
  */

/* Host CDC class state handler function */
typedef USBH_STA_T(*USBH_CDCStateHandler_T)(USBH_INFO_T* usbInfo);
typedef USBH_STA_T(*USBH_CDCDataHandler_T)(USBH_INFO_T* usbInfo);

/**
 * @brief    CDC line coding structure
 */
typedef union
{
    uint8_t data[USBH_CDC_LINE_CODING_NUM];

    struct
    {
        uint32_t        dwDTERate;
        uint8_t         bCharFormat;
        uint8_t         bParityType;
        uint8_t         bDataBits;
    } DATA_B;
} USBH_CDC_LINE_CODING_T;

/**
 * @brief    CDC control line state structure
 */
typedef union
{
    uint8_t bitmap;

    struct
    {
        uint8_t     DTR         : 1;
        uint8_t     RTS         : 1;
        uint8_t     RESERVED    : 6;
    } DATA_B;

} USBH_CDC_CONTROL_LINE_STATE_T;

/**
 * @brief    CDC communication data
 */
typedef struct
{
    uint8_t     notifyChNum;
    uint8_t     notifyEpAddr;
    uint16_t    notifyEpsize;
    uint8_t     notifyBuffer[8];
} USBH_CDC_COMMUNICATION_T;

/**
 * @brief    CDC transfer data
 */
typedef struct
{
    uint32_t    txdLength;
    uint32_t    rxdLength;
    uint8_t*     txBuffer;
    uint8_t*     rxBuffer;
    uint8_t     inChNum;
    uint8_t     outChNum;
    uint8_t     inEpAddr;
    uint8_t     outEpAddr;
    uint16_t    inEpsize;
    uint16_t    outEpsize;
    uint8_t     dataBuffer[8];
} USBH_CDC_XFER_T;

/**
 * @brief    CDC information management
 */
typedef struct
{
    USBH_CDC_STATE_T                state;
    USBH_CDC_DATA_STATE_T           dataXferState;
    USBH_CDC_XFER_T                 dataXfer;
    USBH_CDC_LINE_CODING_T          userLineCoding;
    USBH_CDC_LINE_CODING_T*          lineCoding;
    USBH_CDC_CONTROL_LINE_STATE_T*   controlLine;
    USBH_CDC_COMMUNICATION_T        comm;
    uint32_t                        timer;
} USBH_CDC_INFO_T;

extern USBH_CLASS_T USBH_CDC_CLASS;

/**@} end of group USBH_CDC_Structures*/

/** @defgroup USBH_CDC_Functions Functions
  @{
  */

USBH_CDC_DATA_STATE_T USBH_CDC_ReadDataStatus(USBH_INFO_T* usbInfo);
uint16_t USBH_CDC_ReadRevDataSize(USBH_INFO_T* usbInfo);
USBH_STA_T USBH_CDC_SendData(USBH_INFO_T* usbInfo, uint8_t* buffer, uint32_t length);
USBH_STA_T USBH_CDC_ReceiveData(USBH_INFO_T* usbInfo, uint8_t* buffer, uint32_t length);

void USBH_CDC_LineCodingIsChangeCallback(USBH_INFO_T* usbInfo);
void USBH_CDC_XferEndCallback(USBH_INFO_T* usbInfo);
void USBH_CDC_RevEndCallback(USBH_INFO_T* usbInfo);

/**@} end of group USBH_CDC_Functions */
/**@} end of group USBH_CDC_Class */
/**@} end of group APM32_USB_Library */

#endif
