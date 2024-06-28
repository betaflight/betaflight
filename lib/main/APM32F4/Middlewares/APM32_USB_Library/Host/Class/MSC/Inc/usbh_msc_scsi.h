/*!
 * @file        usbh_msc_scsi.h
 *
 * @brief       USB host MSC scsi
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
#ifndef __USBH_MSC_SCSI_H_
#define __USBH_MSC_SCSI_H_

/* Includes */
#include "usbh_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_MSC_Class
  @{
  */

/** @defgroup USBH_MSC_Macros Macros
  @{
*/

/* Length define of command */
#define LEN_XFER_TEST_UNIT_READY                        0
#define LEN_XFER_READ_CAPACITY                          8
#define LEN_XFER_REQUEST_SENSE                          14
#define LEN_XFER_INQUIRY                                36

/* SCSI Commands */
#define USBH_SCSI_CMD_FORMAT_UNIT                       ((uint8_t)0x04)
#define USBH_SCSI_CMD_INQUIRY                           ((uint8_t)0x12)
#define USBH_SCSI_CMD_MODE_SELECT_6                     ((uint8_t)0x15)
#define USBH_SCSI_CMD_MODE_SELECT_10                    ((uint8_t)0x55)
#define USBH_SCSI_CMD_MODE_SENSE_6                      ((uint8_t)0x1A)
#define USBH_SCSI_CMD_MODE_SENSE_10                     ((uint8_t)0x5A)
#define USBH_SCSI_CMD_ALLOW_MEDIUM_REMOVAL              ((uint8_t)0x1E)
#define USBH_SCSI_CMD_READ_6                            ((uint8_t)0x08)
#define USBH_SCSI_CMD_READ_10                           ((uint8_t)0x28)
#define USBH_SCSI_CMD_READ_12                           ((uint8_t)0xA8)
#define USBH_SCSI_CMD_READ_16                           ((uint8_t)0x88)

#define USBH_SCSI_CMD_READ_CAPACITY                     ((uint8_t)0x25)
#define USBH_SCSI_CMD_READ_CAPACITY_16                  ((uint8_t)0x9E)

#define USBH_SCSI_CMD_REQUEST_SENSE                     ((uint8_t)0x03)
#define USBH_SCSI_CMD_START_STOP_UNIT                   ((uint8_t)0x1B)
#define USBH_SCSI_CMD_TEST_UNIT_READY                   ((uint8_t)0x00)
#define USBH_SCSI_CMD_WRITE6                            ((uint8_t)0x0A)
#define USBH_SCSI_CMD_WRITE10                           ((uint8_t)0x2A)
#define USBH_SCSI_CMD_WRITE12                           ((uint8_t)0xAA)
#define USBH_SCSI_CMD_WRITE16                           ((uint8_t)0x8A)

#define USBH_SCSI_CMD_VERIFY_10                         ((uint8_t)0x2F)
#define USBH_SCSI_CMD_VERIFY_12                         ((uint8_t)0xAF)
#define USBH_SCSI_CMD_VERIFY_16                         ((uint8_t)0x8F)

#define USBH_SCSI_CMD_SEND_DIAGNOSTIC                   ((uint8_t)0x1D)
#define USBH_SCSI_CMD_READ_FORMAT_CAPACITIES            ((uint8_t)0x23)

/**@} end of group USBH_MSC_Macros*/

/** @defgroup USBH_MSC_Enumerates Enumerates
  @{
  */

/**
 * @brief    SCSI sense ASC type
 */
typedef enum
{
    USBH_SCSI_ASC_NO_ADDITIONAL_SENSE_INFORMATION       = 0x00,
    USBH_SCSI_ASCQ_FORMAT_COMMAND_FAILED                = 0x01,
    USBH_SCSI_ASCQ_INITIALIZING_COMMAND_REQUIRED        = 0x02,
    USBH_SCSI_ASC_LOGICAL_UNIT_NOT_READY                = 0x04,
    USBH_SCSI_ASCQ_OPERATION_IN_PROGRESS                = 0x07,
    USBH_SCSI_ASC_INVALID_COMMAND_OPERATION_CODE        = 0x20,
    USBH_SCSI_ASC_INVALID_FIELD_IN_CDB                  = 0x24,
    USBH_SCSI_ASC_WRITE_PROTECTED                       = 0x27,
    USBH_SCSI_ASC_NOT_READY_TO_READY_CHANGE             = 0x28,
    USBH_SCSI_ASC_FORMAT_ERROR                          = 0x31,
    USBH_SCSI_ASC_MEDIUM_NOT_PRESENT                    = 0x3A,
} USBH_SCSI_SENSE_ASC_T;

/**
 * @brief    SCSI sense key type
 */
typedef enum
{
    USBH_SCSI_SENSE_KEY_NO_SENSE             = 0x00,
    USBH_SCSI_SENSE_KEY_RECOVERED_ERROR,
    USBH_SCSI_SENSE_KEY_NOT_READY,
    USBH_SCSI_SENSE_KEY_MEDIUM_ERROR,
    USBH_SCSI_SENSE_KEY_HARDWARE_ERROR,
    USBH_SCSI_SENSE_KEY_ILLEGAL_REQUEST,
    USBH_SCSI_SENSE_KEY_UNIT_ATTENTION,
    USBH_SCSI_SENSE_KEY_DATA_PROTECT,
    USBH_SCSI_SENSE_KEY_BLANK_CHECK,
    USBH_SCSI_SENSE_KEY_VENDOR_SPECIFIC,
    USBH_SCSI_SENSE_KEY_COPY_ABORTED,
    USBH_SCSI_SENSE_KEY_ABORTED_COMMAND,
    USBH_SCSI_SENSE_KEY_VOLUME_OVERFLOW      = 0x0D,
    USBH_SCSI_SENSE_KEY_MISCOMPARE           = 0x0E,
} USBH_SCSI_SENSE_KEY_T;

/**@} end of group USBH_MSC_Enumerates*/

/** @defgroup USBH_MSC_Structures Structures
  @{
  */

/**
 * @brief    SCSI inquiry response data type
 */
typedef struct
{
    uint8_t peripheral;
    uint8_t devType;
    uint8_t media;
    uint8_t vendorID[9];
    uint8_t productID[17];
    uint8_t revID[5];
} USBH_SCSI_INQUIRY_REQ_T;

/**
 * @brief    SCSI read capacity response data type
 */
typedef struct
{
    uint32_t blockNum;
    uint32_t blockSize;
} USBH_SCSI_READ_CAPACITY_REQ_T;

/**
 * @brief    SCSI sense response data type
 */
typedef struct
{
    uint8_t key;
    uint8_t asc;
    uint8_t ascq;
} USBH_SCSI_SENSE_REQ_T;

/**@} end of group USBH_MSC_Structures*/

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

USBH_STA_T USBH_MSC_SCSI_Inquiry(USBH_INFO_T* usbInfo, uint8_t lun, \
                                 USBH_SCSI_INQUIRY_REQ_T* inquiry);
USBH_STA_T USBH_MSC_SCSI_TestUnitReady(USBH_INFO_T* usbInfo, uint8_t lun);

USBH_STA_T USBH_MSC_SCSI_ReadCapacity(USBH_INFO_T* usbInfo, uint8_t lun, \
                                      USBH_SCSI_READ_CAPACITY_REQ_T* capacity);

USBH_STA_T USBH_MSC_SCSI_RequestSense(USBH_INFO_T* usbInfo, uint8_t lun, \
                                      USBH_SCSI_SENSE_REQ_T* sense);

USBH_STA_T USBH_MSC_SCSI_Read(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                              uint8_t* buffer, uint16_t cnt);

USBH_STA_T USBH_MSC_SCSI_Write(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                               uint8_t* buffer, uint16_t cnt);

/**@} end of group USBH_MSC_Functions */
/**@} end of group USBH_MSC_Class */
/**@} end of group APM32_USB_Library */

#endif
