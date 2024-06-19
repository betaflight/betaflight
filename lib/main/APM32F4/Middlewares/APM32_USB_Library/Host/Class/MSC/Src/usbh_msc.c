/*!
 * @file        usbh_msc.c
 *
 * @brief       usb host msc class handler
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
#include "usbh_msc.h"
#include "usbh_msc_bot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_MSC_Class
  @{
  */

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

static USBH_STA_T USBH_MSC_ClassInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_MSC_ClassDeInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_MSC_ClassReqHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_MSC_CoreHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_MSC_SOFHandler(USBH_INFO_T* usbInfo);

static USBH_STA_T USBH_MSC_IdleHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_InitHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_InquiryHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_TestUnitReadyHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_RequestSenseHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_ReadCapacityHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_ErrorUnrecoveredHandler(USBH_INFO_T* usbInfo, uint8_t lun);

static USBH_STA_T USBH_MSC_ReadHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_WriteHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_RWRequestSenseHandler(USBH_INFO_T* usbInfo, uint8_t lun);

/**@} end of group USBH_MSC_Functions */

/** @defgroup USBH_MSC_Structures Structures
  @{
  */

/* MSC class handler */
USBH_CLASS_T USBH_MSC_CLASS =
{
    "Class MSC",
    USBH_CLASS_MSC,
    NULL,
    USBH_MSC_ClassInitHandler,
    USBH_MSC_ClassDeInitHandler,
    USBH_MSC_ClassReqHandler,
    USBH_MSC_CoreHandler,
    USBH_MSC_SOFHandler,
};

/* USB host MSC state handler function */
USBH_MscStateHandler_T USBH_MSC_Handler[] =
{
    USBH_MSC_InitHandler,
    USBH_MSC_IdleHandler,
    USBH_MSC_InquiryHandler,
    USBH_MSC_TestUnitReadyHandler,
    USBH_MSC_RequestSenseHandler,
    USBH_MSC_ReadCapacityHandler,
    USBH_MSC_ErrorUnrecoveredHandler,
    USBH_MSC_ReadHandler,
    USBH_MSC_WriteHandler,
    USBH_MSC_RWRequestSenseHandler,
};

/**@} end of group USBH_MSC_Structures*/

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

/*!
 * @brief       USB host MSC logic unit init handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_InitHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_InitHandler");

    USBH_USR_Debug("Configure LUN:%d", usbHostMSC->curLun);

    usbHostMSC->timer = usbInfo->timer;

    usbHostMSC->storage[usbHostMSC->curLun].state = USBH_MSC_INQUIRY;

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit read handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ReadHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    USBH_USR_Debug("USBH_MSC_ReadHandler");

    reqStatus = USBH_MSC_SCSI_Read(usbInfo, lun, 0, NULL, 0);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbStatus = USBH_OK;
            break;

        case USBH_FAIL:
            usbHostMSC->storage[lun].state = USBH_MSC_RW_REQIEST_SENSE;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_UNRECOVERED_STATE;
            usbStatus = USBH_FAIL;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit write handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_WriteHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    USBH_USR_Debug("USBH_MSC_WriteHandler");

    reqStatus = USBH_MSC_SCSI_Write(usbInfo, lun, 0, NULL, 0);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbStatus = USBH_OK;
            break;

        case USBH_FAIL:
            usbHostMSC->storage[lun].state = USBH_MSC_RW_REQIEST_SENSE;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_UNRECOVERED_STATE;
            usbStatus = USBH_FAIL;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit read and write request sense handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_RWRequestSenseHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    USBH_USR_Debug("USBH_MSC_RWRequestSenseHandler");

    reqStatus = USBH_MSC_SCSI_RequestSense(usbInfo, lun, \
                                           &usbHostMSC->storage[lun].sense);

    switch (reqStatus)
    {
        case USBH_OK:
            USBH_USR_LOG("Sense KEY  : %x", usbHostMSC->storage[lun].sense.key);
            USBH_USR_LOG("Sense ASC  : %x", usbHostMSC->storage[lun].sense.asc);
            USBH_USR_LOG("Sense ASCQ : %x", usbHostMSC->storage[lun].sense.ascq);

            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            usbStatus = USBH_FAIL;
            break;

        case USBH_FAIL:
            USBH_USR_LOG("MSC device is not ready");
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_UNRECOVERED_STATE;
            usbStatus = USBH_FAIL;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit idle handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_IdleHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit INQUIRY handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_InquiryHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    USBH_USR_Debug("USBH_MSC_InquiryHandler");

    reqStatus = USBH_MSC_SCSI_Inquiry(usbInfo, lun, \
                                      &usbHostMSC->storage[lun].inquiryReq);

    switch (reqStatus)
    {
        case USBH_OK:
            USBH_USR_LOG("Inquiry Revision :%s", usbHostMSC->storage[lun].inquiryReq.revID);
            USBH_USR_LOG("Inquiry Product  :%s", usbHostMSC->storage[lun].inquiryReq.productID);
            USBH_USR_LOG("Inquiry Vendor   :%s", usbHostMSC->storage[lun].inquiryReq.vendorID);

            usbHostMSC->storage[lun].state = USBH_MSC_TEST_UNIT_READY;
            break;

        case USBH_FAIL:
            usbHostMSC->storage[lun].state = USBH_MSC_REQUEST_SENSE;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit test unit ready handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_TestUnitReadyHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    USBH_USR_Debug("USBH_MSC_TestUnitReadyHandler");

    reqStatus = USBH_MSC_SCSI_TestUnitReady(usbInfo, lun);

    switch (reqStatus)
    {
        case USBH_OK:
            if (usbHostMSC->storage[lun].preReadyState == USBH_OK)
            {
                usbHostMSC->storage[lun].changeState = DISABLE;
            }
            else
            {
                usbHostMSC->storage[lun].changeState = ENABLE;
                USBH_USR_LOG("MSC device is ready");
            }

            usbHostMSC->storage[lun].state = USBH_MSC_READ_CAPACITY;
            usbHostMSC->storage[lun].preReadyState = USBH_OK;
            usbHostMSC->storage[lun].errState = USBH_MSC_OK;
            break;

        case USBH_FAIL:
            if (usbHostMSC->storage[lun].preReadyState == USBH_FAIL)
            {
                usbHostMSC->storage[lun].changeState = DISABLE;
            }
            else
            {
                usbHostMSC->storage[lun].changeState = ENABLE;
                USBH_USR_LOG("MSC device is not ready");
            }

            /* Media not ready, try to check again during 10s */
            usbHostMSC->storage[lun].state = USBH_MSC_REQUEST_SENSE;
            usbHostMSC->storage[lun].preReadyState = USBH_FAIL;
            usbHostMSC->storage[lun].errState = USBH_MSC_BUSY;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit read capacity handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ReadCapacityHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;

    uint32_t blockNum;
    uint32_t blockSize;

    USBH_USR_Debug("USBH_MSC_ReadCapacityHandler");
    reqStatus = USBH_MSC_SCSI_ReadCapacity(usbInfo, lun, \
                                           &usbHostMSC->storage[lun].capacity);

    switch (reqStatus)
    {
        case USBH_OK:
            if (usbHostMSC->storage[lun].changeState == ENABLE)
            {
                blockNum = usbHostMSC->storage[lun].capacity.blockNum;
                blockSize = usbHostMSC->storage[lun].capacity.blockSize;

                USBH_USR_LOG("MSC device capacity     : %lu bytes", (uint32_t)(blockNum * blockSize));

                USBH_USR_LOG("MSC device block number : %lu", blockNum);

                USBH_USR_LOG("MSC device block size   : %lu", blockSize);
            }

            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_OK;
            usbHostMSC->curLun++;
            break;

        case USBH_FAIL:
            usbHostMSC->storage[lun].state = USBH_MSC_REQUEST_SENSE;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit request sense handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_RequestSenseHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t reqStatus;
    uint8_t senseKey;

    USBH_USR_Debug("USBH_MSC_RequestSenseHandler");

    reqStatus = USBH_MSC_SCSI_RequestSense(usbInfo, lun, \
                                           &usbHostMSC->storage[lun].sense);

    switch (reqStatus)
    {
        case USBH_OK:
            senseKey = usbHostMSC->storage[lun].sense.key;

            if ((senseKey == USBH_SCSI_SENSE_KEY_UNIT_ATTENTION) || (senseKey == USBH_SCSI_SENSE_KEY_NOT_READY))
            {
                if ((usbInfo->timer - usbHostMSC->timer) < 0x2FFF)
                {
                    /* timer is less than 10s and retry again */
                    usbHostMSC->storage[lun].state = USBH_MSC_TEST_UNIT_READY;
                    break;
                }

                USBH_USR_Debug("Sense KEY  : %x", usbHostMSC->storage[lun].sense.key);
                USBH_USR_Debug("Sense ASC  : %x", usbHostMSC->storage[lun].sense.asc);
                USBH_USR_Debug("Sense ASCQ : %x", usbHostMSC->storage[lun].sense.ascq);

                usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
                usbHostMSC->curLun++;
            }
            else
            {
                USBH_USR_LOG("Device sense key error. Please unplug the Device.");
                usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
                usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            }
            break;

        case USBH_FAIL:
            USBH_USR_LOG("MSC device is not ready");
            usbHostMSC->storage[lun].state = USBH_MSC_UNRECOVERED_STATE;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->storage[lun].state = USBH_MSC_IDLE;
            usbHostMSC->storage[lun].errState = USBH_MSC_ERR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC logic unit unrecovered error handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ErrorUnrecoveredHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;

    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_ErrorUnrecoveredHandler");

    usbHostMSC->curLun++;

    return usbStatus;
}

/*!
 * @brief       USB host MSC configuration handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ClassInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_MSC_INFO_T* usbHostMSC;
    uint8_t itfNum, epNum;
    uint8_t classInterface;
    uint8_t subClass;
    uint8_t protocolInterface;
    uint8_t epAddr;
    uint8_t epDir;

    USBH_USR_Debug("USBH_MSC_ClassInitHandler");

    /* Link class data */
    usbInfo->activeClass->classData = (USBH_MSC_INFO_T*)malloc(sizeof(USBH_MSC_INFO_T));
    usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    memset(usbHostMSC, 0, sizeof(USBH_MSC_INFO_T));

    USBH_USR_Debug("USBH_MSC_INFO_T size %d\r\n", sizeof(USBH_MSC_INFO_T));

    if (usbHostMSC == NULL)
    {
        USBH_USR_LOG("usbHostMSC is NULL");
        return USBH_FAIL;
    }

    itfNum = USBH_ReadConfigurationItfNum(usbInfo);

    while (itfNum--)
    {
        classInterface = USBH_ReadInterfaceClass(usbInfo, itfNum);
        protocolInterface = USBH_ReadInterfaceProtocol(usbInfo, itfNum);
        subClass = USBH_ReadInterfaceSubClass(usbInfo, itfNum);

        if (subClass != USBH_MSC_SCSI_CLASS_CODE)
        {
            usbStatus = USBH_ERR_NOT_SUP;
            continue;
        }

        if ((classInterface != USBH_CLASS_MSC) || (protocolInterface != USBH_MSC_PROTOCOL_BBB))
        {
            usbStatus = USBH_ERR_NOT_SUP;
            continue;
        }

        epNum = USBH_ReadInterfaceEpNum(usbInfo, itfNum);

        while (epNum--)
        {
            /* Get endpoint and size */
            epAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
            epDir = epAddr & 0x80;

            if (epDir)
            {
                usbHostMSC->usbHostBOT.bulkInEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostMSC->usbHostBOT.bulkInEpSize = USBH_ReadEndpointMPS(usbInfo, itfNum, epNum);
            }
            else
            {
                usbHostMSC->usbHostBOT.bulkOutEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostMSC->usbHostBOT.bulkOutEpSize = USBH_ReadEndpointMPS(usbInfo, itfNum, epNum);
            }
        }
    }

    /* Init BOT */
    USBH_MSC_BOT_Init(usbInfo);

    usbHostMSC->state = USBH_MSC_INIT;
    usbHostMSC->errState = USBH_MSC_OK;
    usbHostMSC->classReqState = USBH_MSC_REQ_GET_MAX_LUN;
    usbHostMSC->preClassReqState = usbHostMSC->classReqState;

    /* Out channels */
    usbHostMSC->usbHostBOT.outChNum = USBH_CH_AllocChannel(usbInfo, usbHostMSC->usbHostBOT.bulkOutEpAddr);

    /* In channels */
    usbHostMSC->usbHostBOT.inChNum = USBH_CH_AllocChannel(usbInfo, usbHostMSC->usbHostBOT.bulkInEpAddr);

    /* Open the new OUT channels */
    if ((usbHostMSC->usbHostBOT.bulkOutEpAddr == 0) || (usbHostMSC->usbHostBOT.bulkOutEpSize == 0))
    {
        return USBH_ERR_NOT_SUP;
    }
    else
    {
        USBH_OpenChannelCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum,
                                 usbHostMSC->usbHostBOT.bulkOutEpAddr,
                                 usbInfo->devInfo.address,
                                 usbInfo->devInfo.speed,
                                 EP_TYPE_BULK,
                                 usbHostMSC->usbHostBOT.bulkOutEpSize);
    }

    /* Open the new IN channels */
    if ((usbHostMSC->usbHostBOT.bulkInEpAddr == 0) || (usbHostMSC->usbHostBOT.bulkInEpSize == 0))
    {
        return USBH_ERR_NOT_SUP;
    }
    else
    {
        USBH_OpenChannelCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum,
                                 usbHostMSC->usbHostBOT.bulkInEpAddr,
                                 usbInfo->devInfo.address,
                                 usbInfo->devInfo.speed,
                                 EP_TYPE_BULK,
                                 usbHostMSC->usbHostBOT.bulkInEpSize);
    }

    USBH_ConfigDataPidCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum, 0);
    USBH_ConfigDataPidCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum, 0);

    return usbStatus;
}

/*!
 * @brief       USB host MSC class reset handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ClassDeInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if (usbHostMSC->usbHostBOT.inChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostMSC->usbHostBOT.inChNum);
        usbHostMSC->usbHostBOT.inChNum  = 0;
    }

    if (usbHostMSC->usbHostBOT.outChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostMSC->usbHostBOT.outChNum);
        usbHostMSC->usbHostBOT.outChNum  = 0;
    }

    if (usbInfo->activeClass->classData != NULL)
    {
        free(usbInfo->activeClass->classData);
        usbInfo->activeClass->classData = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC class reguest handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_ClassReqHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t i;

    USBH_USR_Debug("USBH_MSC_ClassReqHandler");

    switch (usbHostMSC->classReqState)
    {
        case USBH_MSC_REQ_GET_MAX_LUN:
            reqStatus = USBH_MSC_BOT_REQ_GetMaxLunHandler(usbInfo, &usbHostMSC->maxLun);

            switch (reqStatus)
            {
                case USBH_OK:
                    if (usbHostMSC->maxLun > USBH_SUPPORTED_LUN_MAX)
                    {
                        usbHostMSC->maxLun = USBH_SUPPORTED_LUN_MAX;
                    }
                    else
                    {
                        usbHostMSC->maxLun += 1;
                    }

                    for (i = 0; i < usbHostMSC->maxLun; i++)
                    {
                        usbHostMSC->storage[i].changeState = DISABLE;
                        usbHostMSC->storage[i].preReadyState = USBH_FAIL;
                    }

                    usbStatus = USBH_OK;
                    break;

                case USBH_ERR_NOT_SUP:
                    usbHostMSC->maxLun = 0;
                    usbStatus = USBH_OK;
                    break;

                default:
                    break;
            }
            break;


        case USBH_MSC_REQ_CTRL_ERROR:
            reqStatus = USBH_ClearFeature(usbInfo, 0);

            switch (reqStatus)
            {
                case USBH_OK:
                    usbHostMSC->classReqState = usbHostMSC->preClassReqState;
                    break;

                default:
                    break;
            }
            break;

        default :
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SOF handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_SOFHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;

    return usbStatus;
}

/*!
 * @brief       USB host MSC handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_MSC_CoreHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if (usbHostMSC->state == USBH_MSC_INIT)
    {
        if (usbHostMSC->curLun >= usbHostMSC->maxLun)
        {
            usbHostMSC->curLun = 0;
            usbHostMSC->state = USBH_MSC_IDLE;

            USBH_USR_Debug("%s is launched", usbInfo->hostClass[usbInfo->classNum]->className);

            /* Notify User */
            usbInfo->userCallback(usbInfo, USBH_USER_CLASS_LAUNCHED);
        }
        else
        {
            usbHostMSC->storage[usbHostMSC->curLun].errState = USBH_MSC_BUSY;

            /* MSC process */
            USBH_MSC_Handler[usbHostMSC->storage[usbHostMSC->curLun].state](usbInfo, usbHostMSC->curLun);
        }
    }
    else if (usbHostMSC->state == USBH_MSC_IDLE)
    {
        usbStatus = USBH_MSC_Handler[usbHostMSC->state](usbInfo, usbHostMSC->curLun);
    }
    else
    {

    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC read unit information
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @param       device: device unit information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_ReadDevInfo(USBH_INFO_T* usbInfo, uint8_t lun, USBH_MSC_STORAGE_INFO_T* device)
{
    USBH_STA_T  usbStatus = USBH_FAIL;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if (usbInfo->hostState == USBH_HOST_CLASS)
    {
        memcpy(device, &usbHostMSC->storage[lun], sizeof(USBH_MSC_STORAGE_INFO_T));

        usbStatus = USBH_OK;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC read device ready status
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      Unit ready status
 *              @arg 0: not ready
 *              @arg 1: ready
 */
uint8_t USBH_MSC_DevStatus(USBH_INFO_T* usbInfo, uint8_t lun)
{
    uint8_t unitStatus = 0;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if (usbInfo->hostState == USBH_HOST_CLASS)
    {
        if (usbHostMSC->storage[lun].errState == USBH_MSC_OK)
        {
            unitStatus = 1;
        }
    }

    return unitStatus;
}

/*!
 * @brief       USB host MSC read device WP status
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @retval      WP status
 *              @arg DISABLE: not protect
 *              @arg 1: protect
 */
uint8_t USBH_MSC_ReadDevWP(USBH_INFO_T* usbInfo, uint8_t lun)
{
    uint8_t wpStatus = DISABLE;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if (usbInfo->hostState == USBH_HOST_CLASS)
    {
        if (usbHostMSC->storage[lun].sense.asc == USBH_SCSI_ASC_WRITE_PROTECTED)
        {
            wpStatus = ENABLE;
        }
    }

    return wpStatus;
}

/*!
 * @brief       USB host MSC read unit information
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @param       address: sector address
 *
 * @param       buffer: buffer point to data
 *
 * @param       cnt: count number of data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_DevRead(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                            uint8_t* buffer, uint16_t cnt)
{
    USBH_STA_T usbStatus = USBH_FAIL;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if ((usbInfo->devInfo.connectedStatus == DISABLE) || \
            (usbInfo->hostState != USBH_HOST_CLASS) || \
            (usbHostMSC->storage[lun].state != USBH_MSC_IDLE))
    {
        return usbStatus;
    }

    usbHostMSC->state = USBH_MSC_RW_READ;
    usbHostMSC->storage[lun].state = USBH_MSC_RW_READ;
    usbHostMSC->opLun = lun;

    USBH_MSC_SCSI_Read(usbInfo, lun, address, buffer, cnt);

    usbHostMSC->timer = usbInfo->timer;

    while (USBH_MSC_Handler[usbHostMSC->storage[lun].state](usbInfo, lun) == USBH_BUSY)
    {
        USBH_USR_Debug("read usbInfo->timer:%d", usbInfo->timer);
        USBH_USR_Debug("read usbHostMSC->timer:%d", usbHostMSC->timer);

        if (((usbInfo->timer - usbHostMSC->timer) > (0x2FFF * cnt)) || \
                (usbInfo->devInfo.connectedStatus == DISABLE))
        {
            usbHostMSC->state = USBH_MSC_IDLE;
            return USBH_FAIL;
        }
    }

    usbHostMSC->state = USBH_MSC_IDLE;
    usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_MSC_DevRead end");

    return usbStatus;
}

/*!
 * @brief       USB host MSC write
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @param       address: sector address
 *
 * @param       buffer: buffer point to data
 *
 * @param       cnt: count number of data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_DevWrite(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                             uint8_t* buffer, uint16_t cnt)
{
    USBH_STA_T usbStatus = USBH_FAIL;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    if ((usbInfo->devInfo.connectedStatus == DISABLE) || \
            (usbInfo->hostState != USBH_HOST_CLASS) || \
            (usbHostMSC->storage[lun].state != USBH_MSC_IDLE))
    {
        return usbStatus;
    }

    usbHostMSC->state = USBH_MSC_RW_WRITE;
    usbHostMSC->storage[lun].state = USBH_MSC_RW_WRITE;
    usbHostMSC->opLun = lun;

    USBH_MSC_SCSI_Write(usbInfo, lun, address, buffer, cnt);

    usbHostMSC->timer = usbInfo->timer;

    while (USBH_MSC_Handler[usbHostMSC->storage[lun].state](usbInfo, lun) == USBH_BUSY)
    {
        USBH_USR_Debug("write usbInfo->timer:%d", usbInfo->timer);
        USBH_USR_Debug("write usbHostMSC->timer:%d", usbHostMSC->timer);

        if (((usbInfo->timer - usbHostMSC->timer) > (0x2FFF * cnt)) || \
                (usbInfo->devInfo.connectedStatus == DISABLE))
        {
            usbHostMSC->state = USBH_MSC_IDLE;
            return USBH_FAIL;
        }
    }

    usbHostMSC->state = USBH_MSC_IDLE;
    usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_MSC_DevWrite end");

    return usbStatus;
}

/**@} end of group USBH_MSC_Functions */
/**@} end of group USBH_MSC_Class */
/**@} end of group APM32_USB_Library */
