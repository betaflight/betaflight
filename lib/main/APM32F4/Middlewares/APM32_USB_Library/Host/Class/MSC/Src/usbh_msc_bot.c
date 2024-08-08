/*!
 * @file        usbh_msc_bot.c
 *
 * @brief       USB host MSC bot
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
#include "usbh_msc_bot.h"
#include "usbh_msc.h"
#include "usbh_dataXfer.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_MSC_Class
  @{
  */

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

static USBH_STA_T USBH_MSC_BOT_SendCBWHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_SendCBWWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_DataInHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_DataInWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_DataOutHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_DataOutWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_RevCSWHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_RevCSWWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_ErrorInHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_ErrorOutHandler(USBH_INFO_T* usbInfo, uint8_t lun);
static USBH_STA_T USBH_MSC_BOT_ErrorUnrecoveredHandler(USBH_INFO_T* usbInfo, uint8_t lun);

/**@} end of group USBH_MSC_Functions */

/** @defgroup USBH_MSC_Structures Structures
  @{
  */

/* USB host BOT state handler function */
USBH_BotStateHandler_T USBH_MSC_BOT_Handler[] =
{
    USBH_MSC_BOT_SendCBWHandler,
    USBH_MSC_BOT_SendCBWWaitHandler,
    USBH_MSC_BOT_DataInHandler,
    USBH_MSC_BOT_DataInWaitHandler,
    USBH_MSC_BOT_DataOutHandler,
    USBH_MSC_BOT_DataOutWaitHandler,
    USBH_MSC_BOT_RevCSWHandler,
    USBH_MSC_BOT_RevCSWWaitHandler,
    USBH_MSC_BOT_ErrorInHandler,
    USBH_MSC_BOT_ErrorOutHandler,
    USBH_MSC_BOT_ErrorUnrecoveredHandler,
};

/**@} end of group USBH_MSC_Structures*/

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

/*!
 * @brief     USB host MSC BOT send CBW handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    CSW decode status
 */
static USBH_BOT_CSW_STA_T USBH_MSC_BOT_DecodeCSW(USBH_INFO_T* usbInfo)
{
    USBH_BOT_CSW_STA_T cswStatus = USBH_BOT_CSW_FAIL;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint32_t lastXferSize;

    lastXferSize = USBH_ReadLastXferSizeCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum);

    /* CSW length is Correct */
    if (lastXferSize == USBH_MSC_BOT_CSW_LEN)
    {
        /* dSignature is equal to 0x53425355 */
        if (usbHostMSC->usbHostBOT.cmdPack.CSW.DATA_FIELD.dSignature == USBH_MSC_BOT_CSW_SIGNATURE)
        {
            /* dTag matches the dTag from the corresponding CBW */
            if (usbHostMSC->usbHostBOT.cmdPack.CSW.DATA_FIELD.dTag == usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dTag)
            {
                switch (usbHostMSC->usbHostBOT.cmdPack.CSW.DATA_FIELD.bStatus)
                {
                    case USBH_BOT_CSW_OK:
                        cswStatus = USBH_BOT_CSW_OK;
                        break;

                    case USBH_BOT_CSW_FAIL:
                        cswStatus = USBH_BOT_CSW_FAIL;
                        break;

                    case USBH_BOT_CSW_ERR:
                        cswStatus = USBH_BOT_CSW_ERR;
                        break;
                }
            }
        }
        else
        {
            cswStatus = USBH_BOT_CSW_ERR;
        }
    }
    else
    {
        cswStatus = USBH_BOT_CSW_ERR;
    }

    return cswStatus;
}

/*!
 * @brief     USB host MSC BOT send CBW handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_SendCBWHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bLUN = lun;

    USBH_USR_Debug("USBH_MSC_BOT_SendCBWHandler");

    USBH_BulkSendDataReq(usbInfo, usbHostMSC->usbHostBOT.outChNum, usbHostMSC->usbHostBOT.cmdPack.CBW.buffer, \
                         USBH_MSC_BOT_CBW_LEN, ENABLE);

    usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW_WAIT;

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT send CBW wait handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_SendCBWWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_SendCBWWaitHandler");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen == 0)
            {
                usbHostMSC->usbHostBOT.state = USBH_BOT_RECEIVE_CSW;
            }
            else
            {
                if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir == USBH_REQ_DIR_IN)
                {
                    usbHostMSC->usbHostBOT.state = USBH_BOT_DATAIN;
                }
                else
                {
                    usbHostMSC->usbHostBOT.state = USBH_BOT_DATAOUT;
                }
            }
            break;

        case USB_URB_NOREADY:
            /* Resend CBW */
            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            break;

        case USB_URB_STALL:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_OUT;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT data IN handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_DataInHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_DataInHandler");
    USBH_BulkReceiveDataReq(usbInfo, usbHostMSC->usbHostBOT.inChNum, usbHostMSC->usbHostBOT.buffer, \
                            usbHostMSC->usbHostBOT.bulkInEpSize);

    usbHostMSC->usbHostBOT.state = USBH_BOT_DATAIN_WAIT;

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT data IN wait handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_DataInWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_DataInWaitHandler");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen <= usbHostMSC->usbHostBOT.bulkInEpSize)
            {
                usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = 0;
            }
            else
            {
                usbHostMSC->usbHostBOT.buffer += usbHostMSC->usbHostBOT.bulkInEpSize;
                usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen -= usbHostMSC->usbHostBOT.bulkInEpSize;
            }

            if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen > 0)
            {
                /* Receive the next bulk packet */
                USBH_BulkReceiveDataReq(usbInfo, usbHostMSC->usbHostBOT.inChNum, usbHostMSC->usbHostBOT.buffer, \
                                        usbHostMSC->usbHostBOT.bulkInEpSize);
            }
            else
            {
                usbHostMSC->usbHostBOT.state = USBH_BOT_RECEIVE_CSW;
            }
            break;

        case USB_URB_STALL:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_IN;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT data OUT handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_DataOutHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_DataOutHandler");

    USBH_BulkSendDataReq(usbInfo, usbHostMSC->usbHostBOT.outChNum, usbHostMSC->usbHostBOT.buffer, \
                         usbHostMSC->usbHostBOT.bulkOutEpSize, ENABLE);

    usbHostMSC->usbHostBOT.state = USBH_BOT_DATAOUT_WAIT;

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT data OUT wait handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_DataOutWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_DataOutWaitHandler");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen <= usbHostMSC->usbHostBOT.bulkOutEpSize)
            {
                usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = 0;
            }
            else
            {
                usbHostMSC->usbHostBOT.buffer += usbHostMSC->usbHostBOT.bulkOutEpSize;
                usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen -= usbHostMSC->usbHostBOT.bulkOutEpSize;
            }

            if (usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen > 0)
            {
                /* Send the next bulk packet */
                USBH_BulkSendDataReq(usbInfo, usbHostMSC->usbHostBOT.outChNum, usbHostMSC->usbHostBOT.buffer, \
                                     usbHostMSC->usbHostBOT.bulkOutEpSize, ENABLE);
            }
            else
            {
                usbHostMSC->usbHostBOT.state = USBH_BOT_RECEIVE_CSW;
            }
            break;

        case USB_URB_NOREADY:
            usbHostMSC->usbHostBOT.state = USBH_BOT_DATAOUT;
            break;

        case USB_URB_STALL:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_OUT;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT receive CSW handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_RevCSWHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_RevCSWHandler");

    USBH_BulkReceiveDataReq(usbInfo, usbHostMSC->usbHostBOT.inChNum, usbHostMSC->usbHostBOT.cmdPack.CSW.buffer, \
                            USBH_MSC_BOT_CSW_LEN);

    usbHostMSC->usbHostBOT.state = USBH_BOT_RECEIVE_CSW_WAIT;

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT reveive CSW wait handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_RevCSWWaitHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_BOT_CSW_STA_T cswStatus = USBH_BOT_CSW_FAIL;

    USBH_USR_Debug("USBH_MSC_BOT_RevCSWWaitHandler");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_START;
            cswStatus = USBH_MSC_BOT_DecodeCSW(usbInfo);

            if (cswStatus == USBH_BOT_CSW_OK)
            {
                usbStatus = USBH_OK;
            }
            else
            {
                usbStatus = USBH_FAIL;
            }
            break;

        case USB_URB_STALL:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_IN;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT IN error handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_ErrorInHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_ErrorInHandler");

    reqStatus = USBH_ClearFeature(usbInfo, usbHostMSC->usbHostBOT.bulkInEpAddr);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostMSC->usbHostBOT.state = USBH_BOT_RECEIVE_CSW;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_UNRECOVERED;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT OUT error handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_ErrorOutHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t toggle;

    USBH_USR_Debug("USBH_MSC_BOT_ErrorOutHandler");

    reqStatus = USBH_ClearFeature(usbInfo, usbHostMSC->usbHostBOT.bulkOutEpAddr);

    switch (reqStatus)
    {
        case USBH_OK:
            toggle = USBH_ReadToggleCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum);

            USBH_ConfigToggleCallback(usbInfo, usbHostMSC->usbHostBOT.outChNum, 1 - toggle);
            USBH_ConfigToggleCallback(usbInfo, usbHostMSC->usbHostBOT.inChNum, 0);

            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_IN;
            break;

        case USBH_ERR_UNRECOVERED:
            usbHostMSC->usbHostBOT.state = USBH_BOT_ERROR_UNRECOVERED;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host MSC BOT unrecovered error handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lun : logical unit numer
 *
 * @retval    USB host operation status
 */
static USBH_STA_T USBH_MSC_BOT_ErrorUnrecoveredHandler(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_MSC_BOT_ErrorUnrecoveredHandler");

    reqStatus = USBH_MSC_BOT_REQ_Reset(usbInfo);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     Init BOT of USB host MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB host operation status
 */
USBH_STA_T USBH_MSC_BOT_Init(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dSignature = USBH_MSC_BOT_CBW_SIGNATURE;
    usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dTag = USBH_MSC_BOT_CBW_TAG;

    usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_START;
    usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;

    return usbStatus;
}

/*!
 * @brief     Reset request of MSC BOT
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB host operation status
 */
USBH_STA_T USBH_MSC_BOT_REQ_Reset(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    /* Config Request */
    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir = USBH_REQ_DIR_OUT;
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = USBH_RECIPIENT_INTERFACE;
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type = USBH_REQ_TYPE_CLASS;

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_CLASS_BOT_RESET;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 0x00;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0x00;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0x00;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0x00;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = 0x00;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = 0x00;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, NULL, 0);

    return usbStatus;
}

/*!
 * @brief     Get the max lun of MSC BOT
 *
 * @param     usbInfo : usb handler information
 *
 * @param     maxLun: max of logic unit number
 *
 * @retval    USB host operation status
 */
USBH_STA_T USBH_MSC_BOT_REQ_GetMaxLunHandler(USBH_INFO_T* usbInfo, uint8_t* maxLun)
{
    USBH_STA_T  usbStatus = USBH_OK;

    /* Config Request */
    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir = USBH_REQ_DIR_IN;
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = USBH_RECIPIENT_INTERFACE;
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type = USBH_REQ_TYPE_CLASS;

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_CLASS_GET_MAX_LUN;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 0x00;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0x00;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0x00;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0x00;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = 0x01;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = 0x00;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, maxLun, 1);

    return usbStatus;
}

/**@} end of group USBH_MSC_Functions */
/**@} end of group USBH_MSC_Class */
/**@} end of group APM32_USB_Library */
