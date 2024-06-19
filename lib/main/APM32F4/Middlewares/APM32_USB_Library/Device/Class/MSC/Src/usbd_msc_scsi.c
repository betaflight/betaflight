/*!
 * @file        usbd_msc_scsi.c
 *
 * @brief       usb device msc scsi handler
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

/* Includes */
#include "usbd_msc_scsi.h"
#include "usbd_msc.h"
#include "usbd_msc_bot.h"
#include "usbd_dataXfer.h"
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_MSC_Class
  @{
  */

/** @defgroup USBD_MSC_Variables Variables
  @{
  */

/* USB mass storage page 00 inquiry data */
uint8_t page00InquiryData[USBD_LEN_INQUIRY_PAGE00] =
{
    0x00,
    0x00,
    0x00,
    (USBD_LEN_INQUIRY_PAGE00 - 4U),
    0x00,
    0x80
};

/* USB mass storage page 80 inquiry data */
uint8_t page80InquiryData[USBD_LEN_INQUIRY_PAGE80] =
{
    0x00,
    0x80,
    0x00,
    USBD_LEN_INQUIRY_PAGE80,
    0x20,
    0x20,
    0x20,
    0x20
};

/* USB mass storage sense 6 data */
uint8_t modeSense6data[USBD_LEN_STD_MODE_SENSE6] =
{
    0x22,
    0x00,
    0x00,
    0x00,
    0x08,
    0x12,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};

/* USB Mass storage sense 10  Data */
uint8_t modeSense10data[USBD_LEN_STD_MODE_SENSE10] =
{
    0x00,
    0x26,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x08,
    0x12,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};

/**@} end of group USBD_MSC_Variables*/

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

/*!
 * @brief     Put the sense code to array
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     key: sense Key
 *
 * @param     asc: Additional Sense Code
 *
 * @param     ascq: Additional Sense Code Qualifier
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_CodeSense(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(lun);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseEnd].Key = key;
    usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseEnd].ASC = asc;
    usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseEnd].ASCQ = ascq;
    usbDevMSC->usbDevSCSI.senseEnd++;

    if (usbDevMSC->usbDevSCSI.senseEnd == USBD_SCSI_SENSE_LIST_NUMBER)
    {
        usbDevMSC->usbDevSCSI.senseEnd = 0;
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI configure BOT data
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length: data length
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ConfigBotData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint16_t length)
{
    USBD_STA_T  usbStatus = USBD_BUSY;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint16_t i;

    if (usbDevMSC == NULL)
    {
        return USBD_OK;
    }

    usbDevMSC->usbDevBOT.dataLen = length;

    for (i = 0; i < length; i++)
    {
        usbDevMSC->usbDevBOT.data[i] = buffer[i];
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read capacity 16 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ReadCapacity16(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryReadCapacity(lun, \
                &usbDevMSC->usbDevSCSI.blockNum, &usbDevMSC->usbDevSCSI.blockSize);

    if ((reqStatus != USBD_OK) || (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT))
    {
        USBD_SCSI_CodeSense(usbInfo, lun, \
                            USBD_SCSI_SENSE_KEY_NOT_READY, \
                            USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                            0);

        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.dataLen = (uint32_t)((command[10] << 24) | \
                                   (command[11] << 16) | \
                                   (command[12] << 8) | \
                                   (command[13]));

    memset(usbDevMSC->usbDevBOT.data, 0, usbDevMSC->usbDevBOT.dataLen);

    usbDevMSC->usbDevBOT.data[4] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 24);
    usbDevMSC->usbDevBOT.data[5] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 16);
    usbDevMSC->usbDevBOT.data[6] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 8);
    usbDevMSC->usbDevBOT.data[7] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1));

    usbDevMSC->usbDevBOT.data[8] = (uint8_t)((usbDevMSC->usbDevSCSI.blockSize) >> 24);
    usbDevMSC->usbDevBOT.data[9] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize >> 16);
    usbDevMSC->usbDevBOT.data[10] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize >> 8);
    usbDevMSC->usbDevBOT.data[11] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize);

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read capacity command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ReadCapacity(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(command);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryReadCapacity(lun, \
                &usbDevMSC->usbDevSCSI.blockNum, &usbDevMSC->usbDevSCSI.blockSize);

    if ((reqStatus != USBD_OK) || (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT))
    {
        USBD_SCSI_CodeSense(usbInfo, lun, \
                            USBD_SCSI_SENSE_KEY_NOT_READY, \
                            USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                            0);

        return USBD_FAIL;
    }

    memset(usbDevMSC->usbDevBOT.data, 0, 8);

    usbDevMSC->usbDevBOT.data[0] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 24);
    usbDevMSC->usbDevBOT.data[1] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 16);
    usbDevMSC->usbDevBOT.data[2] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1) >> 8);
    usbDevMSC->usbDevBOT.data[3] = (uint8_t)((usbDevMSC->usbDevSCSI.blockNum - 1));

    usbDevMSC->usbDevBOT.data[4] = (uint8_t)((usbDevMSC->usbDevSCSI.blockSize) >> 24);
    usbDevMSC->usbDevBOT.data[5] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize >> 16);
    usbDevMSC->usbDevBOT.data[6] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize >> 8);
    usbDevMSC->usbDevBOT.data[7] = (uint8_t)(usbDevMSC->usbDevSCSI.blockSize);

    usbDevMSC->usbDevBOT.dataLen = 8;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read format capacity command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ReadFormatCapacity(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint16_t blockSize;
    uint32_t blockNum;

    UNUSED(command);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryReadCapacity(lun, &blockNum, &blockSize);

    if ((reqStatus != USBD_OK) || (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT))
    {
        USBD_SCSI_CodeSense(usbInfo, lun, \
                            USBD_SCSI_SENSE_KEY_NOT_READY, \
                            USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                            0);

        return USBD_FAIL;
    }

    memset(usbDevMSC->usbDevBOT.data, 0, 12);

    blockNum -= 1;

    usbDevMSC->usbDevBOT.data[3] = 0x08;
    usbDevMSC->usbDevBOT.data[4] = (uint8_t)(blockNum >> 24);
    usbDevMSC->usbDevBOT.data[5] = (uint8_t)(blockNum >> 16);
    usbDevMSC->usbDevBOT.data[6] = (uint8_t)(blockNum >> 8);
    usbDevMSC->usbDevBOT.data[7] = (uint8_t)(blockNum);

    usbDevMSC->usbDevBOT.data[8] = 0x02;
    usbDevMSC->usbDevBOT.data[9] = (uint8_t)(blockSize >> 16);
    usbDevMSC->usbDevBOT.data[10] = (uint8_t)(blockSize >> 8);
    usbDevMSC->usbDevBOT.data[11] = (uint8_t)(blockSize);

    usbDevMSC->usbDevBOT.dataLen = 12;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI inquiry command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Inquiry(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint8_t epvd = command[1] & 0x01;
    uint8_t addLen = command[4];
    uint8_t* buffer;
    uint16_t bufferLen;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen == 0)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_INVALID_CDB, 0);
        return USBD_FAIL;
    }

    /* EPVD is set */
    if (epvd != 0)
    {
        if (command[2] == 0)
        {
            USBD_SCSI_ConfigBotData(usbInfo, page00InquiryData, USBD_LEN_INQUIRY_PAGE00);
        }
        else if (command[2] == 0x80)
        {
            USBD_SCSI_ConfigBotData(usbInfo, page80InquiryData, USBD_LEN_INQUIRY_PAGE80);
        }
        else
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_FIELED_IN_COMMAND, 0);
            return USBD_FAIL;
        }
    }
    else
    {
        buffer = (uint8_t*) & ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->inquiryData[lun * USBD_LEN_STD_INQUIRY];
        bufferLen = buffer[4] + 5;

        if (addLen <= bufferLen)
        {
            bufferLen = addLen;
        }

        USBD_SCSI_ConfigBotData(usbInfo, buffer, bufferLen);
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI allow medium removable command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_AllowMediumRemoval(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(lun);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (command[4])
    {
        usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_LOCK;
    }
    else
    {
        usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_UNLOCK;
    }

    usbDevMSC->usbDevBOT.dataLen = 0;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI mode sense 6 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ModeSense6(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint16_t length;

    UNUSED(lun);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    length = USBD_LEN_STD_MODE_SENSE6;

    if (length >= command[4])
    {
        length = command[4];
    }

    USBD_SCSI_ConfigBotData(usbInfo, modeSense6data, length);

    return usbStatus;
}

/*!
 * @brief     USB device SCSI mode sense 10 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_ModeSense10(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint16_t length;

    UNUSED(lun);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    length = USBD_LEN_STD_MODE_SENSE10;

    if (length >= command[8])
    {
        length = command[8];
    }

    USBD_SCSI_ConfigBotData(usbInfo, modeSense10data, length);

    return usbStatus;
}

/*!
 * @brief     USB device SCSI request sense command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_RequestSense(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen == 0)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_INVALID_CDB, \
                            0);

        return USBD_FAIL;
    }

    memset(usbDevMSC->usbDevBOT.data, 0, USBD_LEN_STD_REQ_SENSE);

    usbDevMSC->usbDevBOT.data[0] = 0x70;
    usbDevMSC->usbDevBOT.data[7] = USBD_LEN_STD_REQ_SENSE - 6;

    UNUSED(lun);

    if ((usbDevMSC->usbDevSCSI.senseHead != usbDevMSC->usbDevSCSI.senseEnd))
    {
        usbDevMSC->usbDevBOT.data[2] = usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseHead].Key;
        usbDevMSC->usbDevBOT.data[12] = usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseHead].ASC;
        usbDevMSC->usbDevBOT.data[13] = usbDevMSC->usbDevSCSI.sense[usbDevMSC->usbDevSCSI.senseHead].ASCQ;

        if (++usbDevMSC->usbDevSCSI.senseHead == USBD_SCSI_SENSE_LIST_NUMBER)
        {
            usbDevMSC->usbDevSCSI.senseHead = 0;
        }
    }

    usbDevMSC->usbDevBOT.dataLen = (command[4] <= USBD_LEN_STD_REQ_SENSE) ? \
                                   command[4] : USBD_LEN_STD_REQ_SENSE;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI start or stop unit command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_StartStopUnit(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(lun);

    uint8_t temp = command[4] & 0x03;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if ((usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_LOCK) && \
            (temp == 0x02))
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_INVALID_FIELED_IN_COMMAND, \
                            0);

        return USBD_FAIL;
    }

    switch (temp)
    {
        /* START=1 */
        case 0x01:
            usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_UNLOCK;
            break;

        /* START=0 and LOEJ Load Eject=1 */
        case 0x02:
            usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_EJECT;
            break;

        /* START=1 and LOEJ Load Eject=1 */
        case 0x03:
            usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_UNLOCK;
            break;

        default:
            break;
    }

    usbDevMSC->usbDevBOT.dataLen = 0;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI test unit ready command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_TestUnitReady(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(command);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != 0)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_INVALID_CDB, \
                            0);

        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_NOT_READY, \
                            USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                            0);

        usbDevMSC->usbDevBOT.state = USBD_BOT_NO_DATA;

        return USBD_FAIL;
    }

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckReady(lun);

    if (reqStatus != USBD_OK)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_NOT_READY, \
                            USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                            0);

        usbDevMSC->usbDevBOT.state = USBD_BOT_NO_DATA;

        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.dataLen = 0;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI write data handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_TxData(USBD_INFO_T* usbInfo, uint8_t lun)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint32_t length;
    uint32_t blockSize = usbDevMSC->usbDevSCSI.blockSize;
    uint32_t blockLen = usbDevMSC->usbDevSCSI.blockLen;
    uint32_t blockAddr = usbDevMSC->usbDevSCSI.blockAddr;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    length = (blockLen * blockSize) < USBD_SUP_MSC_MEDIA_PACKET ? \
             (blockLen * blockSize) : USBD_SUP_MSC_MEDIA_PACKET;

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryWriteData(lun, \
                usbDevMSC->usbDevBOT.data, \
                blockAddr, \
                (length / blockSize));

    if (reqStatus != USBD_OK)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_HARDWARE_ERROR, \
                            USBD_SCSI_ASC_WRITE_FAULT, \
                            0);

        return USBD_FAIL;
    }

    usbDevMSC->usbDevSCSI.blockAddr += (length / blockSize);
    usbDevMSC->usbDevSCSI.blockLen -= (length / blockSize);

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dDataResidue -= length;

    if (usbDevMSC->usbDevSCSI.blockLen)
    {
        length = (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize) < USBD_SUP_MSC_MEDIA_PACKET ? \
                 (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize) : USBD_SUP_MSC_MEDIA_PACKET;

        USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                                usbDevMSC->usbDevBOT.data, \
                                length);
    }
    else
    {
        USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_OK);
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read data handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_RxData(USBD_INFO_T* usbInfo, uint8_t lun)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint32_t length;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    length = (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize) < USBD_SUP_MSC_MEDIA_PACKET ? \
             (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize) : USBD_SUP_MSC_MEDIA_PACKET;

    reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryReadData(lun, \
                usbDevMSC->usbDevBOT.data, \
                usbDevMSC->usbDevSCSI.blockAddr, \
                (length / usbDevMSC->usbDevSCSI.blockSize));

    if (reqStatus != USBD_OK)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_HARDWARE_ERROR, \
                            USBD_SCSI_ASC_UNRECOVERED_READ_ERROR, \
                            0);

        return USBD_FAIL;
    }

    USBD_EP_TransferCallback(usbInfo, usbDevMSC->epInAddr, \
                             usbDevMSC->usbDevBOT.data, \
                             length);

    usbDevMSC->usbDevSCSI.blockAddr += (length / usbDevMSC->usbDevSCSI.blockSize);
    usbDevMSC->usbDevSCSI.blockLen -= (length / usbDevMSC->usbDevSCSI.blockSize);

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dDataResidue -= length;

    if (usbDevMSC->usbDevSCSI.blockLen == 0)
    {
        usbDevMSC->usbDevBOT.state = USBD_BOT_DATAIN_LAST;
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI write 10 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Write10(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint32_t length;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    switch (usbDevMSC->usbDevBOT.state)
    {
        case USBD_BOT_IDLE:
            if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen == 0)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }

            if ((usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_Flag & 0x80) == 0x80)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }

            reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckReady(lun);

            if (reqStatus != USBD_OK)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_NOT_READY, \
                                    USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                    0);

                return USBD_FAIL;
            }

            reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckWPR(lun);

            if (reqStatus != USBD_OK)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_NOT_READY, \
                                    USBD_SCSI_ASC_WRITE_PROTECTED, \
                                    0);

                return USBD_FAIL;
            }

            /* Store write block information */
            usbDevMSC->usbDevSCSI.blockAddr = (uint32_t)((command[2] << 24) | \
                                              (command[3] << 16) | \
                                              (command[4] << 8) | \
                                              (command[5]));

            usbDevMSC->usbDevSCSI.blockLen = (uint32_t)((command[7] << 8) | \
                                             (command[8]));

            /* check if LBA address is in the right range */
            if ((usbDevMSC->usbDevSCSI.blockAddr + usbDevMSC->usbDevSCSI.blockLen) > \
                    usbDevMSC->usbDevSCSI.blockNum)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE, \
                                    0);

                return USBD_FAIL;
            }

            length = usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize;

            if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != length)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }


            length =  length < USBD_SUP_MSC_MEDIA_PACKET ? \
                      length : USBD_SUP_MSC_MEDIA_PACKET;

            usbDevMSC->usbDevBOT.state = USBD_BOT_DATAOUT;

            USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                                    usbDevMSC->usbDevBOT.data, \
                                    length);

            break;

        default:
            USBD_SCSI_TxData(usbInfo, lun);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI write 12 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Write12(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint32_t length;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    switch (usbDevMSC->usbDevBOT.state)
    {
        case USBD_BOT_IDLE:
            if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen == 0)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }

            if ((usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_Flag & 0x80) == 0x80)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }

            reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckReady(lun);

            if (reqStatus != USBD_OK)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_NOT_READY, \
                                    USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                    0);

                usbDevMSC->usbDevBOT.state = USBD_BOT_NO_DATA;

                return USBD_FAIL;
            }

            reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckWPR(lun);

            if (reqStatus != USBD_OK)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_NOT_READY, \
                                    USBD_SCSI_ASC_WRITE_PROTECTED, \
                                    0);

                usbDevMSC->usbDevBOT.state = USBD_BOT_NO_DATA;

                return USBD_FAIL;
            }

            /* Store write block information */
            usbDevMSC->usbDevSCSI.blockAddr = (uint32_t)((command[2] << 24) | \
                                              (command[3] << 16) | \
                                              (command[4] << 8) | \
                                              (command[5]));

            usbDevMSC->usbDevSCSI.blockLen = (uint32_t)((command[6] << 24) | \
                                             (command[7] << 16) | \
                                             (command[8] << 8) | \
                                             (command[9]));

            /* check if LBA address is in the right range */
            if ((usbDevMSC->usbDevSCSI.blockAddr + usbDevMSC->usbDevSCSI.blockLen) > \
                    usbDevMSC->usbDevSCSI.blockNum)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE, \
                                    0);

                return USBD_FAIL;
            }

            length = usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize;

            if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != length)
            {
                USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                    USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                    USBD_SCSI_ASC_INVALID_CDB, \
                                    0);

                return USBD_FAIL;
            }

            length =  length < USBD_SUP_MSC_MEDIA_PACKET ? \
                      length : USBD_SUP_MSC_MEDIA_PACKET;

            usbDevMSC->usbDevBOT.state = USBD_BOT_DATAOUT;

            USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                                    usbDevMSC->usbDevBOT.data, \
                                    length);

            break;

        default:
            USBD_SCSI_TxData(usbInfo, lun);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device SCSI verify 10 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Verify10(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(lun);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if ((command[1] & 0x02) == 0x02)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_INVALID_FIELED_IN_COMMAND, \
                            0);

        return USBD_FAIL;
    }

    /* check if LBA address is in the right range */
    if ((usbDevMSC->usbDevSCSI.blockAddr + usbDevMSC->usbDevSCSI.blockLen) > \
            usbDevMSC->usbDevSCSI.blockNum)
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                            USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE, \
                            0);

        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.dataLen = 0;

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read 10 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Read10(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevBOT.state == USBD_BOT_IDLE)
    {
        if ((usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_Flag & 0x80) != 0x80)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_CDB, \
                                0);

            return USBD_FAIL;
        }

        if (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_NOT_READY, \
                                USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                0);

            return USBD_FAIL;
        }

        reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckReady(lun);

        if (reqStatus != USBD_OK)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_NOT_READY, \
                                USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                0);

            return USBD_FAIL;
        }

        /* Store write block information */
        usbDevMSC->usbDevSCSI.blockAddr = (uint32_t)((command[2] << 24) | \
                                          (command[3] << 16) | \
                                          (command[4] << 8) | \
                                          (command[5]));

        usbDevMSC->usbDevSCSI.blockLen = (uint32_t)((command[7] << 8) | \
                                         (command[8]));

        /* check if LBA address is in the right range */
        if ((usbDevMSC->usbDevSCSI.blockAddr + usbDevMSC->usbDevSCSI.blockLen) > \
                usbDevMSC->usbDevSCSI.blockNum)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE, \
                                0);

            return USBD_FAIL;
        }

        if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != \
                (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize))
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_CDB, \
                                0);

            return USBD_FAIL;
        }

        usbDevMSC->usbDevBOT.state = USBD_BOT_DATAIN;
    }

    usbDevMSC->usbDevBOT.dataLen = USBD_SUP_MSC_MEDIA_PACKET;

    usbStatus = USBD_SCSI_RxData(usbInfo, lun);

    return usbStatus;
}

/*!
 * @brief     USB device SCSI read 12 command handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Read12(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if (usbDevMSC->usbDevBOT.state == USBD_BOT_IDLE)
    {
        if ((usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_Flag & 0x80) != 0x80)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_CDB, \
                                0);

            return USBD_FAIL;
        }

        if (usbDevMSC->usbDevSCSI.mediumState == USBD_SCSI_MEDIUM_EJECT)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_NOT_READY, \
                                USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                0);

            return USBD_FAIL;
        }

        reqStatus = ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryCheckReady(lun);

        if (reqStatus != USBD_OK)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_NOT_READY, \
                                USBD_SCSI_ASC_MEDIUM_NOT_PRESENT, \
                                0);

            return USBD_FAIL;
        }

        /* Store write block information */
        usbDevMSC->usbDevSCSI.blockAddr = (uint32_t)((command[2] << 24) | \
                                          (command[3] << 16) | \
                                          (command[4] << 8) | \
                                          (command[5]));

        usbDevMSC->usbDevSCSI.blockLen = (uint32_t)((command[6] << 24) | \
                                         (command[7] << 16) | \
                                         (command[8] << 8) | \
                                         (command[9]));

        /* check if LBA address is in the right range */
        if ((usbDevMSC->usbDevSCSI.blockAddr + usbDevMSC->usbDevSCSI.blockLen) > \
                usbDevMSC->usbDevSCSI.blockNum)
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_ADDRESS_OUT_OF_RANGE, \
                                0);

            return USBD_FAIL;
        }

        if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != \
                (usbDevMSC->usbDevSCSI.blockLen * usbDevMSC->usbDevSCSI.blockSize))
        {
            USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_CDB, \
                                0);

            return USBD_FAIL;
        }

        usbDevMSC->usbDevBOT.state = USBD_BOT_DATAIN;
    }

    usbDevMSC->usbDevBOT.dataLen = USBD_SUP_MSC_MEDIA_PACKET;

    usbStatus = USBD_SCSI_RxData(usbInfo, lun);

    return usbStatus;
}

/*!
 * @brief     USB device SCSI handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : LUN
 *
 * @param     command: SCSI command
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_SCSI_Handle(USBD_INFO_T* usbInfo, uint8_t lun, uint8_t* command)
{
    USBD_STA_T  usbStatus = USBD_FAIL;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return usbStatus;
    }

    switch (command[0])
    {
        case USBD_SCSI_CMD_INQUIRY:
            usbStatus = USBD_SCSI_Inquiry(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_ALLOW_MEDIUM_REMOVAL:
            usbStatus = USBD_SCSI_AllowMediumRemoval(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_MODE_SENSE_6:
            usbStatus = USBD_SCSI_ModeSense6(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_MODE_SENSE_10:
            usbStatus = USBD_SCSI_ModeSense10(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_READ_FORMAT_CAPACITIES:
            usbStatus = USBD_SCSI_ReadFormatCapacity(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_READ_CAPACITY:
            usbStatus = USBD_SCSI_ReadCapacity(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_READ_CAPACITY_16:
            usbStatus = USBD_SCSI_ReadCapacity16(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_REQUEST_SENSE:
            usbStatus = USBD_SCSI_RequestSense(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_START_STOP_UNIT:
            usbStatus = USBD_SCSI_StartStopUnit(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_TEST_UNIT_READY:
            usbStatus = USBD_SCSI_TestUnitReady(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_WRITE10:
            usbStatus = USBD_SCSI_Write10(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_WRITE12:
            usbStatus = USBD_SCSI_Write12(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_VERIFY_10:
            usbStatus = USBD_SCSI_Verify10(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_READ_10:
            usbStatus = USBD_SCSI_Read10(usbInfo, lun, command);
            break;

        case USBD_SCSI_CMD_READ_12:
            usbStatus = USBD_SCSI_Read12(usbInfo, lun, command);
            break;

        default:
            USBD_SCSI_CodeSense(usbInfo, lun, USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, \
                                USBD_SCSI_ASC_INVALID_CDB, 0);
            usbDevMSC->usbDevBOT.status = USBD_BOT_ERR;
            usbStatus = USBD_FAIL;
            break;
    }

    return usbStatus;
}

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */
