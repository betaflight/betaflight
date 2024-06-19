/*!
 * @file        usbh_cdc.h
 *
 * @brief       USB CDC core function
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
#include "usbh_cdc.h"
#include "usbh_stdReq.h"
#include "usbh_dataXfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_CDC_Class
  @{
  */

/** @defgroup USBH_CDC_Functions Functions
  @{
  */

static USBH_STA_T USBH_CDC_ClassInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_ClassDeInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_ClassReqHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_SOFHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_CoreHandler(USBH_INFO_T* usbInfo);

static USBH_STA_T USBH_CDC_InitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_IdleHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_SetLineCodingHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_GetLineCodingHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_SetControlLineHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_TransferDataHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_ErrorHandler(USBH_INFO_T* usbInfo);

static USBH_STA_T USBH_CDC_DataIdleHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_DataSendHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_DataSendWaitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_DataRevHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_DataRevWaitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CDC_DataErrorHandler(USBH_INFO_T* usbInfo);

/**@} end of group USBH_CDC_Functions */

/** @defgroup USBH_CDC_Structures Structures
  @{
  */

/* CDC class handler */
USBH_CLASS_T USBH_CDC_CLASS =
{
    "Class CDC",
    USBH_CLASS_CDCC,
    NULL,
    USBH_CDC_ClassInitHandler,
    USBH_CDC_ClassDeInitHandler,
    USBH_CDC_ClassReqHandler,
    USBH_CDC_CoreHandler,
    USBH_CDC_SOFHandler,
};

/* USB host CDC state handler function */
USBH_CDCStateHandler_T USBH_CDC_Handler[] =
{
    USBH_CDC_InitHandler,
    USBH_CDC_IdleHandler,
    USBH_CDC_SetLineCodingHandler,
    USBH_CDC_GetLineCodingHandler,
    USBH_CDC_SetControlLineHandler,
    USBH_CDC_TransferDataHandler,
    USBH_CDC_ErrorHandler,
};

/* USB host CDC data state handler function */
USBH_CDCDataHandler_T USBH_CDC_DATA_Handler[] =
{
    USBH_CDC_DataIdleHandler,
    USBH_CDC_DataSendHandler,
    USBH_CDC_DataSendWaitHandler,
    USBH_CDC_DataRevHandler,
    USBH_CDC_DataRevWaitHandler,
    USBH_CDC_DataErrorHandler,
};

/**@} end of group USBH_CDC_Structures*/

/** @defgroup USBH_CDC_Functions Functions
  @{
  */

/*!
 * @brief     USB host get CDC line coding request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     lineCoding : line coding structure
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_REQ_GetLineCoding(USBH_INFO_T* usbInfo,  uint8_t reqType, \
        USBH_CDC_LINE_CODING_T* lineCoding)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_CDC_REQ_GET_LINE_CODING;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = USBH_CDC_LINE_CODING_NUM & 0xFF;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = USBH_CDC_LINE_CODING_NUM >> 8;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, lineCoding->data, USBH_CDC_LINE_CODING_NUM);

    return usbStatus;
}

/*!
 * @brief     USB host set CDC line coding request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     lineCoding : line coding structure
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_REQ_SetLineCoding(USBH_INFO_T* usbInfo,  uint8_t reqType, \
        USBH_CDC_LINE_CODING_T* lineCoding)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_CDC_REQ_SET_LINE_CODING;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = USBH_CDC_LINE_CODING_NUM & 0xFF;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = USBH_CDC_LINE_CODING_NUM >> 8;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, lineCoding->data, USBH_CDC_LINE_CODING_NUM);

    return usbStatus;
}

/*!
 * @brief     USB host set CDC line coding request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     controlLine : control line status
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_REQ_SetControlLineState(USBH_INFO_T* usbInfo,  uint8_t reqType, \
        USBH_CDC_CONTROL_LINE_STATE_T* controlLine)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_CDC_REQ_SET_CONTROL_LINE_STATE;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = controlLine->bitmap;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = 0;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, NULL, 0);

    return usbStatus;
}

/*!
 * @brief     USB host get CDC line coding
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lineCoding : line coding structure
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_GetLineCoding(USBH_INFO_T* usbInfo, USBH_CDC_LINE_CODING_T* lineCoding)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_CDC_REQ_GetLineCoding(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                                           (USBH_REQ_TYPE_CLASS << 5) | \
                                           (USBH_RECIPIENT_INTERFACE)),
                                           lineCoding);

    return usbStatus;
}

/*!
 * @brief     USB host set CDC line coding
 *
 * @param     usbInfo : usb handler information
 *
 * @param     lineCoding : line coding structure
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_SetLineCoding(USBH_INFO_T* usbInfo, USBH_CDC_LINE_CODING_T* lineCoding)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_CDC_REQ_SetLineCoding(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                           (USBH_REQ_TYPE_CLASS << 5) | \
                                           (USBH_RECIPIENT_INTERFACE)),
                                           lineCoding);

    return usbStatus;
}

/*!
 * @brief     USB host set CDC control line state
 *
 * @param     usbInfo : usb handler information
 *
 * @param     controlLine : control line status
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CDC_SetControlLineState(USBH_INFO_T* usbInfo, USBH_CDC_CONTROL_LINE_STATE_T* controlLine)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_CDC_REQ_SetControlLineState(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                (USBH_REQ_TYPE_CLASS << 5) | \
                (USBH_RECIPIENT_INTERFACE)),
                controlLine);

    return usbStatus;
}

/*!
 * @brief       USB host CDC read data status
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host CDC data status
 */
USBH_CDC_DATA_STATE_T USBH_CDC_ReadDataStatus(USBH_INFO_T* usbInfo)
{
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    return usbHostCDC->dataXferState;
}

/*!
 * @brief       USB host CDC configure control line state
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_CDC_ConfigControlLineState(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    if (usbHostCDC->state == USBH_CDC_IDLE)
    {

        usbHostCDC->controlLine->DATA_B.DTR = 0;
        usbHostCDC->controlLine->DATA_B.RTS = 0;

        usbHostCDC->state = USBH_CDC_SET_CONTROL_LINE_STATE;

        usbStatus = USBH_OK;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC read receive data size
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host CDC receive data size
 */
uint16_t USBH_CDC_ReadRevDataSize(USBH_INFO_T* usbInfo)
{
    uint32_t revDataSize = 0;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    if (usbInfo->hostState == USBH_HOST_CLASS)
    {
        revDataSize = USBH_ReadLastXferSizeCallback(usbInfo, usbHostCDC->dataXfer.inChNum);
    }

    return revDataSize;
}

/*!
 * @brief       USB host CDC send data
 *
 * @param       usbInfo: usb host information
 *
 * @param       buffer: buffer point to send data
 *
 * @param       length: length of send data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_CDC_SendData(USBH_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    if ((usbHostCDC->state == USBH_CDC_TRANSFER_DATA_STATE) || \
            usbHostCDC->state == USBH_CDC_IDLE)
    {
        usbHostCDC->dataXfer.txdLength = length;
        usbHostCDC->dataXfer.txBuffer = buffer;

        usbHostCDC->state = USBH_CDC_TRANSFER_DATA_STATE;
        usbHostCDC->dataXferState = USBH_CDC_DATA_SEND;

        usbStatus = USBH_OK;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC receive data
 *
 * @param       usbInfo: usb host information
 *
 * @param       buffer: buffer point to receive data
 *
 * @param       length: length of receive data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_CDC_ReceiveData(USBH_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    if ((usbHostCDC->state == USBH_CDC_TRANSFER_DATA_STATE) || \
            usbHostCDC->state == USBH_CDC_IDLE)
    {
        usbHostCDC->dataXfer.rxdLength = length;
        usbHostCDC->dataXfer.rxBuffer = buffer;

        usbHostCDC->state = USBH_CDC_TRANSFER_DATA_STATE;
        usbHostCDC->dataXferState = USBH_CDC_DATA_RECEIVE;

        usbStatus = USBH_OK;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC init handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_InitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    /* Notify User */
    usbInfo->userCallback(usbInfo, USBH_USER_CLASS_LAUNCHED);

    usbHostCDC->state = USBH_CDC_IDLE;

    return usbStatus;
}

/*!
 * @brief       USB host CDC idle handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_IdleHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host CDC data idle handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataIdleHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host CDC send data handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataSendHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    if (usbHostCDC->dataXfer.txdLength > usbHostCDC->dataXfer.outEpsize)
    {
        USBH_BulkSendDataReq(usbInfo, usbHostCDC->dataXfer.outChNum, \
                             usbHostCDC->dataXfer.txBuffer, \
                             usbHostCDC->dataXfer.outEpsize, ENABLE);
    }
    else
    {
        USBH_BulkSendDataReq(usbInfo, usbHostCDC->dataXfer.outChNum, \
                             usbHostCDC->dataXfer.txBuffer, \
                             usbHostCDC->dataXfer.txdLength, ENABLE);
    }

    usbHostCDC->dataXferState = USBH_CDC_DATA_SEND_WAIT;

    return usbStatus;
}

/*!
 * @brief       USB host CDC send data wait handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataSendWaitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostCDC->dataXfer.outChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            if (usbHostCDC->dataXfer.txdLength > usbHostCDC->dataXfer.outEpsize)
            {
                usbHostCDC->dataXfer.txdLength -= usbHostCDC->dataXfer.outEpsize;
                usbHostCDC->dataXfer.txBuffer += usbHostCDC->dataXfer.outEpsize;
            }
            else
            {
                usbHostCDC->dataXfer.txdLength = 0;
            }

            if (usbHostCDC->dataXfer.txdLength > 0)
            {
                usbHostCDC->dataXferState = USBH_CDC_DATA_SEND;
            }
            else
            {
                usbHostCDC->dataXferState = USBH_CDC_DATA_IDLE;

                /* Notify User */
                USBH_CDC_XferEndCallback(usbInfo);
            }
            break;

        case USB_URB_NOREADY:
            usbHostCDC->dataXferState = USBH_CDC_DATA_SEND;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC receive data handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataRevHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    USBH_BulkReceiveDataReq(usbInfo, usbHostCDC->dataXfer.inChNum, \
                            usbHostCDC->dataXfer.rxBuffer, \
                            usbHostCDC->dataXfer.inEpsize);

    usbHostCDC->dataXferState = USBH_CDC_DATA_RECEIVE_WAIT;

    return usbStatus;
}

/*!
 * @brief       USB host CDC receive data wait handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataRevWaitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;
    uint32_t length;

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostCDC->dataXfer.inChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            length = USBH_ReadLastXferSizeCallback(usbInfo, usbHostCDC->dataXfer.inChNum);

            if ((length > usbHostCDC->dataXfer.inEpsize) && \
                    (usbHostCDC->dataXfer.rxdLength - length > 0))
            {
                usbHostCDC->dataXfer.rxBuffer += length;
                usbHostCDC->dataXfer.rxdLength -= length;

                usbHostCDC->dataXferState = USBH_CDC_DATA_RECEIVE;
            }
            else
            {
                usbHostCDC->dataXferState = USBH_CDC_DATA_IDLE;

                /* Notify User */
                USBH_CDC_RevEndCallback(usbInfo);
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC data error handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_DataErrorHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host CDC set line coding handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_SetLineCodingHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t reqStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    reqStatus = USBH_CDC_SetLineCoding(usbInfo, usbHostCDC->lineCoding);
    switch (reqStatus)
    {
        case USBH_OK:
            usbHostCDC->state = USBH_CDC_GET_LINE_CODING_STATE;
            break;

        case USBH_BUSY:
            break;

        default:
            usbHostCDC->state = USBH_CDC_ERROR_STATE;
            break;
    }


    return usbStatus;
}

/*!
 * @brief       USB host CDC get line coding handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_GetLineCodingHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t reqStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;
    uint8_t i;
    uint8_t lineCodingStatus = USBH_OK;

    reqStatus = USBH_CDC_GetLineCoding(usbInfo, &usbHostCDC->userLineCoding);
    switch (reqStatus)
    {
        case USBH_OK:
            usbHostCDC->state = USBH_CDC_IDLE;

            for (i = 0; i < USBH_CDC_LINE_CODING_NUM; i++)
            {
                if (usbHostCDC->userLineCoding.data[i] != usbHostCDC->lineCoding->data[i])
                {
                    lineCodingStatus = USBH_FAIL;
                }
            }

            if (lineCodingStatus == USBH_OK)
            {
                USBH_CDC_LineCodingIsChangeCallback(usbInfo);
            }

            break;

        case USBH_BUSY:
            break;

        default:
            usbHostCDC->state = USBH_CDC_ERROR_STATE;
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC set control line state handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_SetControlLineHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t reqStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    reqStatus = USBH_CDC_SetControlLineState(usbInfo, usbHostCDC->controlLine);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostCDC->state = USBH_CDC_IDLE;
            break;

        default:
            usbHostCDC->state = USBH_CDC_ERROR_STATE;
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC transfer data state handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_TransferDataHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    usbStatus = USBH_CDC_DATA_Handler[usbHostCDC->dataXferState](usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB host CDC error state handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_ErrorHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t reqStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    reqStatus = USBH_ClearFeature(usbInfo, 0);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostCDC->state = USBH_CDC_IDLE;
            break;

        default:
            break;
    }


    return usbStatus;
}

/*!
 * @brief       USB host CDC configuration handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_ClassInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_CDC_INFO_T* usbHostCDC;

    uint8_t itfNum;
    uint8_t subClass;
    uint8_t classInterface;
    uint8_t protocolInterface;

    uint8_t epNum;
    uint8_t epAddr;
    uint8_t epDir;

    USBH_USR_Debug("USBH_CDC_ClassInitHandler");

    /* Link class data */
    usbInfo->activeClass->classData = (USBH_CDC_INFO_T*)malloc(sizeof(USBH_CDC_INFO_T));
    usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;
    memset(usbHostCDC, 0, sizeof(USBH_CDC_INFO_T));

    /* Configure communication endpoint */
    itfNum = USBH_ReadConfigurationItfNum(usbInfo);

    while (itfNum--)
    {
        classInterface = USBH_ReadInterfaceClass(usbInfo, itfNum);
        if (classInterface != USBH_CLASS_CDCC)
        {
            usbStatus = USBH_ERR_NOT_SUP;
            continue;
        }

        subClass = USBH_ReadInterfaceSubClass(usbInfo, itfNum);
        if (subClass != USBH_CDC_ACM_CODE)
        {
            usbStatus =  USBH_FAIL;
            continue;
        }

        protocolInterface = USBH_ReadInterfaceProtocol(usbInfo, itfNum);
        if (protocolInterface != USBH_CDC_AT_COMMAND_CODE)
        {
            usbStatus =  USBH_FAIL;
            continue;
        }

        epNum = USBH_ReadInterfaceEpNum(usbInfo, itfNum);

        if (epNum > ENDPOINT_DESC_MAX_NUM)
        {
            epNum = ENDPOINT_DESC_MAX_NUM;
        }

        while (epNum--)
        {
            /* Get endpoint and size */
            epAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
            epDir = epAddr & 0x80;

            if (epDir)
            {
                usbHostCDC->comm.notifyEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostCDC->comm.notifyEpsize = USBH_ReadEndpointMPS(usbInfo, itfNum, epNum);
            }
        }
    }

    /* Notify channels */
    usbHostCDC->comm.notifyChNum = USBH_CH_AllocChannel(usbInfo, usbHostCDC->comm.notifyEpAddr);

    /* Open the new Notify channels */
    USBH_OpenChannelCallback(usbInfo, usbHostCDC->comm.notifyChNum,
                             usbHostCDC->comm.notifyEpAddr,
                             usbInfo->devInfo.address,
                             usbInfo->devInfo.speed,
                             EP_TYPE_INTERRUPT,
                             usbHostCDC->comm.notifyEpsize);

    USBH_ConfigDataPidCallback(usbInfo, usbHostCDC->comm.notifyChNum, 0);

    /* Configure data endpoint */
    itfNum = USBH_ReadConfigurationItfNum(usbInfo);

    while (itfNum--)
    {
        classInterface = USBH_ReadInterfaceClass(usbInfo, itfNum);
        if (classInterface != USBH_CLASS_CDCD)
        {
            usbStatus = USBH_ERR_NOT_SUP;
            continue;
        }

        epNum = USBH_ReadInterfaceEpNum(usbInfo, itfNum);

        if (epNum > ENDPOINT_DESC_MAX_NUM)
        {
            epNum = ENDPOINT_DESC_MAX_NUM;
        }

        while (epNum--)
        {
            /* Get endpoint and size */
            epAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
            epDir = epAddr & 0x80;

            if (epDir)
            {
                usbHostCDC->dataXfer.inEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostCDC->dataXfer.inEpsize = USBH_ReadEndpointMPS(usbInfo, itfNum, epNum);
            }
            else
            {
                usbHostCDC->dataXfer.outEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostCDC->dataXfer.outEpsize = USBH_ReadEndpointMPS(usbInfo, itfNum, epNum);
            }
        }
    }

    /* Out channels */
    usbHostCDC->dataXfer.outChNum = USBH_CH_AllocChannel(usbInfo, usbHostCDC->dataXfer.outEpAddr);

    /* In channels */
    usbHostCDC->dataXfer.inChNum = USBH_CH_AllocChannel(usbInfo, usbHostCDC->dataXfer.inEpAddr);

    /* Open the new OUT channels */
    USBH_OpenChannelCallback(usbInfo, usbHostCDC->dataXfer.outChNum,
                             usbHostCDC->dataXfer.outEpAddr,
                             usbInfo->devInfo.address,
                             usbInfo->devInfo.speed,
                             EP_TYPE_BULK,
                             usbHostCDC->dataXfer.outEpsize);


    /* Open the new IN channels */
    USBH_OpenChannelCallback(usbInfo, usbHostCDC->dataXfer.inChNum,
                             usbHostCDC->dataXfer.inEpAddr,
                             usbInfo->devInfo.address,
                             usbInfo->devInfo.speed,
                             EP_TYPE_BULK,
                             usbHostCDC->dataXfer.inEpsize);

    USBH_ConfigDataPidCallback(usbInfo, usbHostCDC->dataXfer.outChNum, 0);

    USBH_ConfigDataPidCallback(usbInfo, usbHostCDC->dataXfer.inChNum, 0);

    usbStatus = USBH_OK;

    usbHostCDC->state = USBH_CDC_INIT;
    usbHostCDC->dataXferState = USBH_CDC_DATA_IDLE;

    return usbStatus;
}

/*!
 * @brief       USB host CDC class reset handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_ClassDeInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_CDC_ClassDeInitHandler");

    if (usbHostCDC->comm.notifyChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostCDC->comm.notifyChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostCDC->comm.notifyChNum);
        usbHostCDC->comm.notifyChNum  = 0;
    }

    if (usbHostCDC->dataXfer.inChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostCDC->dataXfer.inChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostCDC->dataXfer.inChNum);
        usbHostCDC->dataXfer.inChNum  = 0;
    }

    if (usbHostCDC->dataXfer.outChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostCDC->dataXfer.outChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostCDC->dataXfer.outChNum);
        usbHostCDC->dataXfer.outChNum  = 0;
    }

    if (usbInfo->activeClass->classData != NULL)
    {
        free(usbInfo->activeClass->classData);
        usbInfo->activeClass->classData = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC class reguest handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_ClassReqHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_BUSY;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_CDC_ClassReqHandler");

    reqStatus = USBH_CDC_GetLineCoding(usbInfo, &usbHostCDC->userLineCoding);

    switch (reqStatus)
    {
        case USBH_OK:
            usbStatus = USBH_OK;
            break;

        case USBH_ERR_NOT_SUP:
            usbStatus = USBH_ERR_NOT_SUP;
            USBH_USR_LOG("CTRL error: get device line coding configuration");
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host CDC SOF handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_SOFHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host CDC handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_CDC_CoreHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_CDC_INFO_T* usbHostCDC = (USBH_CDC_INFO_T*)usbInfo->activeClass->classData;

    usbStatus = USBH_CDC_Handler[usbHostCDC->state](usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB host CDC send data finish callback
 *
 * @param       usbInfo: usb host information
 *
 * @retval      None
 */
__weak void USBH_CDC_XferEndCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief       USB host CDC receive data finish callback
 *
 * @param       usbInfo: usb host information
 *
 * @retval      None
 */
__weak void USBH_CDC_RevEndCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief       USB host CDC line coding status is change callback
 *
 * @param       usbInfo: usb host information
 *
 * @retval      None
 */
__weak void USBH_CDC_LineCodingIsChangeCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/**@} end of group USBH_CDC_Functions */
/**@} end of group USBH_CDC_Class */
/**@} end of group APM32_USB_Library */
