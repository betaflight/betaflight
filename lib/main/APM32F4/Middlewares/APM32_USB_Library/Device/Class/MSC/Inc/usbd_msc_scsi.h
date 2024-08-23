/*!
 * @file        usbd_msc_scsi.h
 *
 * @brief       usb device msc scsi handler header file
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
#ifndef _USBD_MSC_SCSI_H_
#define _USBD_MSC_SCSI_H_

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

/* Length define of command */
#define USBD_LEN_STD_INQUIRY                            36
#define USBD_LEN_STD_MODE_SENSE6                        23
#define USBD_LEN_STD_MODE_SENSE10                       27
#define USBD_LEN_STD_REQ_SENSE                          18
#define USBD_SCSI_SENSE_LIST_NUMBER                     4
#define USBD_LEN_INQUIRY_PAGE00                         6
#define USBD_LEN_INQUIRY_PAGE80                         8


#define STANDARD_INQUIRY_DATA_LEN                   USBD_LEN_STD_INQUIRY

/* SCSI Commands */
#define USBD_SCSI_CMD_FORMAT_UNIT                       ((uint8_t)0x04)
#define USBD_SCSI_CMD_INQUIRY                           ((uint8_t)0x12)
#define USBD_SCSI_CMD_SEND_DIAGNOSTIC                   ((uint8_t)0x1D)
#define USBD_SCSI_CMD_ALLOW_MEDIUM_REMOVAL              ((uint8_t)0x1E)

#define USBD_SCSI_CMD_MODE_SELECT_6                     ((uint8_t)0x15)
#define USBD_SCSI_CMD_MODE_SELECT_10                    ((uint8_t)0x55)
#define USBD_SCSI_CMD_MODE_SENSE_6                      ((uint8_t)0x1A)
#define USBD_SCSI_CMD_MODE_SENSE_10                     ((uint8_t)0x5A)

#define USBD_SCSI_CMD_READ_FORMAT_CAPACITIES            ((uint8_t)0x23)
#define USBD_SCSI_CMD_READ_CAPACITY                     ((uint8_t)0x25)
#define USBD_SCSI_CMD_READ_CAPACITY_16                  ((uint8_t)0x9E)

#define USBD_SCSI_CMD_REQUEST_SENSE                     ((uint8_t)0x03)
#define USBD_SCSI_CMD_START_STOP_UNIT                   ((uint8_t)0x1B)
#define USBD_SCSI_CMD_TEST_UNIT_READY                   ((uint8_t)0x00)

#define USBD_SCSI_CMD_WRITE6                            ((uint8_t)0x0A)
#define USBD_SCSI_CMD_WRITE10                           ((uint8_t)0x2A)
#define USBD_SCSI_CMD_WRITE12                           ((uint8_t)0xAA)
#define USBD_SCSI_CMD_WRITE16                           ((uint8_t)0x8A)

#define USBD_SCSI_CMD_VERIFY_10                         ((uint8_t)0x2F)
#define USBD_SCSI_CMD_VERIFY_12                         ((uint8_t)0xAF)
#define USBD_SCSI_CMD_VERIFY_16                         ((uint8_t)0x8F)

#define USBD_SCSI_CMD_READ_6                            ((uint8_t)0x08)
#define USBD_SCSI_CMD_READ_10                           ((uint8_t)0x28)
#define USBD_SCSI_CMD_READ_12                           ((uint8_t)0xA8)
#define USBD_SCSI_CMD_READ_16                           ((uint8_t)0x88)

/**@} end of group USBD_MSC_Macros*/

/** @defgroup USBD_MSC_Enumerates Enumerates
  @{
  */

/**
 * @brief    MSC SCSI medium status
 */
typedef enum
{
    USBD_SCSI_MEDIUM_UNLOCK,
    USBD_SCSI_MEDIUM_LOCK,
    USBD_SCSI_MEDIUM_EJECT,
} USBD_MEDIUM_STA_T;

/**
 * @brief    SCSI sense key type
 */
typedef enum
{
    USBD_SCSI_SENSE_KEY_NO_SENSE             = 0x00,
    USBD_SCSI_SENSE_KEY_RECOVERED_ERROR,
    USBD_SCSI_SENSE_KEY_NOT_READY,
    USBD_SCSI_SENSE_KEY_MEDIUM_ERROR,
    USBD_SCSI_SENSE_KEY_HARDWARE_ERROR,
    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST,
    USBD_SCSI_SENSE_KEY_UNIT_ATTENTION,
    USBD_SCSI_SENSE_KEY_DATA_PROTECT,
    USBD_SCSI_SENSE_KEY_BLANK_CHECK,
    USBD_SCSI_SENSE_KEY_VENDOR_SPECIFIC,
    USBD_SCSI_SENSE_KEY_COPY_ABORTED,
    USBD_SCSI_SENSE_KEY_ABORTED_COMMAND,
    USBD_SCSI_SENSE_KEY_VOLUME_OVERFLOW      = 0x0D,
    USBD_SCSI_SENSE_KEY_MISCOMPARE           = 0x0E,
} USBD_SCSI_SENSE_KEY_T;

/**
 * @brief    SCSI sense ASC type
 */
typedef enum
{
    USBD_SCSI_ASC_WRITE_FAULT                       = 0x03,
    USBD_SCSI_ASC_UNRECOVERED_READ_ERROR            = 0x11,
    USBD_SCSI_ASC_PARAMETER_LIST_LENGTH_ERROR       = 0x1A,
    USBD_SCSI_ASC_INVALID_CDB                       = 0x20,
    USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE              = 0x21,
    USBD_SCSI_ASC_INVALID_FIELED_IN_COMMAND         = 0x24,
    USBD_SCSI_ASC_INVALID_FIELD_IN_PARAMETER_LIST   = 0x26,
    USBD_SCSI_ASC_WRITE_PROTECTED                   = 0x27,
    USBD_SCSI_ASC_MEDIUM_HAVE_CHANGED               = 0x28,
    USBD_SCSI_ASC_MEDIUM_NOT_PRESENT                = 0x3A,
} USBD_SCSI_SENSE_ASC_T;

/**@} end of group USBD_MSC_Enumerates*/

/** @defgroup USBD_MSC_Structures Structures
  @{
  */

/**
 * @brief    MSC SCSI sense data type
 */
typedef struct
{
    uint8_t Key;
    uint8_t ASC;
    uint8_t ASCQ;
} USBD_SCSI_SENSE_T;

/**
 * @brief    MSC SCSI information
 */
typedef struct
{
    uint8_t             senseHead;
    uint8_t             senseEnd;
    uint8_t             mediumState;

    uint16_t            blockSize;
    uint32_t            blockNum;

    uint32_t            blockAddr;
    uint32_t            blockLen;
    USBD_SCSI_SENSE_T   sense[USBD_SCSI_SENSE_LIST_NUMBER];
} USBD_SCSI_INFO_T;

/**@} end of group USBD_MSC_Structures*/

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

USBD_STA_T USBD_SCSI_Handle(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command);
USBD_STA_T USBD_SCSI_CodeSense(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq);

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */

#endif
