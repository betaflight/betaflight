/*!
 * @file        usbh_hid.h
 *
 * @brief       USB HID core function
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
#include "usbh_hid.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keyboard.h"
#include "usbh_stdReq.h"
#include "usbh_dataXfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_HID_Class
  @{
  */

/** @defgroup USBH_HID_Functions Functions
  @{
  */

static USBH_STA_T USBH_HID_ClassInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_ClassDeInitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_ClassReqHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_SOFHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_CoreHandler(USBH_INFO_T* usbInfo);

static USBH_STA_T USBH_HID_InitHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_IdleHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_SyncHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_InDataHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_PollingHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_OutDataHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_BusyHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_HID_ErrorHandler(USBH_INFO_T* usbInfo);

/**@} end of group USBH_HID_Functions */

/** @defgroup USBH_HID_Structures Structures
  @{
  */

/* HID class handler */
USBH_CLASS_T USBH_HID_CLASS =
{
    "Class HID",
    USBH_CLASS_HID,
    NULL,
    USBH_HID_ClassInitHandler,
    USBH_HID_ClassDeInitHandler,
    USBH_HID_ClassReqHandler,
    USBH_HID_CoreHandler,
    USBH_HID_SOFHandler,
};

/* USB host HID state handler function */
USBH_HIDStateHandler_T USBH_HID_Handler[] =
{
    USBH_HID_InitHandler,
    USBH_HID_IdleHandler,
    USBH_HID_SyncHandler,
    USBH_HID_InDataHandler,
    USBH_HID_PollingHandler,
    USBH_HID_OutDataHandler,
    USBH_HID_BusyHandler,
    USBH_HID_ErrorHandler,
};

/**@} end of group USBH_HID_Structures*/

/** @defgroup USBH_HID_Functions Functions
  @{
  */

/*!
 * @brief     Config HID get report descriptor request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type
 *
 * @param     reportType : type of report
 *
 * @param     reportID : ID of report
 *
 * @param     buffer : report buffer
 *
 * @param     length : report length
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_REQ_GetRepDescriptor(USBH_INFO_T* usbInfo, uint8_t reqType, \
        uint8_t reportType, uint8_t reportID, \
        uint8_t* buffer, uint8_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_HID_GET_REPORT;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = reportID;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = reportType;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0;
            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wLength[0] = length & 0xFF;
            usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] = length >> 8;
            break;

        default:
            break;
    }

    usbStatus = USBH_REQ_CtrlXferHandler(usbInfo, buffer, length);

    return usbStatus;
}

/*!
 * @brief     USB host HID set idle request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     reportID : traget report ID
 *
 * @param     duration : duration for idle request
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_REQ_SetIdle(USBH_INFO_T* usbInfo, uint8_t reqType, \
                                       uint8_t reportID, uint8_t duration)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_HID_SET_IDLE;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = reportID;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = duration;

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
 * @brief     USB host HID set protocol request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     protocol : boot / report protocol
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_REQ_SetProtocol(USBH_INFO_T* usbInfo, uint8_t reqType, \
        uint8_t protocol)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_HID_SET_PROTOCOL;

            if (protocol)
            {
                usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 0;
            }
            else
            {
                usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = 1;
            }

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
 * @brief     USB host get HID description
 *
 * @param     usbInfo : usb handler information
 *
 * @param     desLength : length of description
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_GetHIDDesc(USBH_INFO_T* usbInfo, uint16_t desLength)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_GetDescriptor(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                                       (USBH_REQ_TYPE_STANDARD << 5) | \
                                       (USBH_RECIPIENT_INTERFACE)), \
                                       USBH_DESC_HID_REPORT,
                                       usbInfo->devInfo.data,
                                       desLength);
    return usbStatus;
}

/*!
 * @brief     USB host get HID report description
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reportType : type of report
 *
 * @param     reportID : ID of report
 *
 * @param     buffer : report buffer
 *
 * @param     length : report length
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_GetReportDesc(USBH_INFO_T* usbInfo, uint8_t reportType, \
        uint8_t reportID, uint8_t* buffer, uint8_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_HID_REQ_GetRepDescriptor(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                (USBH_REQ_TYPE_CLASS << 5) | \
                (USBH_RECIPIENT_INTERFACE)), \
                reportType,
                reportID,
                buffer,
                length);

    return usbStatus;
}

/*!
 * @brief     USB host set HID idle
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reportID : traget report ID
 *
 * @param     duration : duration for idle request
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_SetIdle(USBH_INFO_T* usbInfo, uint8_t reportID, uint8_t duration)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_HID_REQ_SetIdle(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                     (USBH_REQ_TYPE_CLASS << 5) | \
                                     (USBH_RECIPIENT_INTERFACE)),
                                     reportID, \
                                     duration);

    return usbStatus;
}

/*!
 * @brief     USB host set HID protocol
 *
 * @param     usbInfo : usb handler information
 *
 * @param     protocol : boot / report protocol
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_HID_SetProtocol(USBH_INFO_T* usbInfo, uint8_t protocol)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_HID_REQ_SetProtocol(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                         (USBH_REQ_TYPE_CLASS << 5) | \
                                         (USBH_RECIPIENT_INTERFACE)),
                                         protocol);

    return usbStatus;
}

/*!
 * @brief       USB host HID init handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_InitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    usbHostHID->callback->InitHandler(usbInfo);

    /* Notify User */
    usbInfo->userCallback(usbInfo, USBH_USER_CLASS_LAUNCHED);

    usbHostHID->state = USBH_HID_IDLE;

    return usbStatus;
}

/*!
 * @brief       USB host HID idle handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_IdleHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t reqStatus = USBH_BUSY;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_IdleHandler");

    reqStatus = USBH_HID_GetReportDesc(usbInfo,
                                       HID_INPUT_REPORT,
                                       0,
                                       usbHostHID->buffer,
                                       usbHostHID->epSize);

    switch (reqStatus)
    {
        case USBH_OK:
            usbHostHID->state = USBH_HID_SYNC;
            break;

        case USBH_BUSY:
            usbHostHID->state = USBH_HID_IDLE;
            usbStatus = USBH_OK;
            break;

        case USBH_ERR_NOT_SUP:
            usbHostHID->state = USBH_HID_SYNC;
            usbStatus = USBH_OK;
            break;

        default:
            usbHostHID->state = USBH_HID_ERR;
            usbStatus = USBH_FAIL;
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID SYNC handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_SyncHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_SyncHandler");

    /* Sync with SOF */
    if (usbInfo->timer % 2)
    {
        usbHostHID->state = USBH_HID_IN_DATA;
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID get data handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_InDataHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_InDataHandler");

    USBH_IntReceiveDataReq(usbInfo, usbHostHID->inChNum, usbHostHID->buffer, \
                           usbHostHID->epSize);

    usbHostHID->state = USBH_HID_POLL;
    usbHostHID->timer = usbInfo->timer;
    usbHostHID->dataFlag = DISABLE;

    return usbStatus;
}

/*!
 * @brief       USB host HID polling handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_PollingHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t usbUrbStatus;
    uint8_t reqStatus = USBH_BUSY;
    uint32_t xferSize;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_PollingHandler");

    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbHostHID->inChNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            xferSize = USBH_ReadLastXferSizeCallback(usbInfo, usbHostHID->inChNum);

            if ((xferSize != 0) && (usbHostHID->dataFlag == DISABLE))
            {
                usbHostHID->dataFlag = ENABLE;
                //USBH_HID_WriteFifo(&usbHostHID->fifo, usbHostHID->buffer, usbHostHID->epSize);

                /* HID event callback */
                USBH_HID_PollCallback(usbInfo);
            }
            break;

        case USB_URB_STALL:
            reqStatus = USBH_ClearFeature(usbInfo, usbHostHID->epAddr);

            switch (reqStatus)
            {
                case USBH_OK:
                    usbHostHID->state = USBH_HID_IN_DATA;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID send data handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_OutDataHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    return usbStatus;
}

/*!
 * @brief       USB host HID busy handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_BusyHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_HID_BusyHandler");

    return usbStatus;
}

/*!
 * @brief       USB host HID error handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_ErrorHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_HID_ErrorHandler");

    return usbStatus;
}

/*!
 * @brief     Parse HID descriptor
 *
 * @param     hidDesc : HID descriptor
 *
 * @param     buffer : source data of configuration descriptor
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_HidDescParse(USBH_HID_DESC_T* hidDesc, uint8_t* buffer)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint16_t totalLenTemp = 0;
    uint8_t subLen = 0;
    uint16_t parseIndex = 0;

    totalLenTemp = *(uint8_t*)(buffer + 2) | (*(uint8_t*)(buffer + 3) << 8);
    totalLenTemp = ((totalLenTemp) < (CFG_DESC_MAX_LEN) ? (totalLenTemp) : (CFG_DESC_MAX_LEN));

    if (totalLenTemp > STD_CFG_DESC_SIZE)
    {
        parseIndex = STD_CFG_DESC_SIZE;

        while (totalLenTemp > parseIndex)
        {
            subLen = buffer[parseIndex];

            switch (buffer[parseIndex + 1])
            {
                case USBH_DESC_HID:
                    hidDesc->bLength                = buffer[parseIndex + 0];
                    hidDesc->bDescriptorType        = buffer[parseIndex + 1];
                    hidDesc->bcdHID[0]              = buffer[parseIndex + 2];
                    hidDesc->bcdHID[1]              = buffer[parseIndex + 3];
                    hidDesc->bCountryCode           = buffer[parseIndex + 4];
                    hidDesc->bNumDescriptors        = buffer[parseIndex + 5];
                    hidDesc->bReportDescriptorType  = buffer[parseIndex + 6];
                    hidDesc->wDescriptorLength[0]   = buffer[parseIndex + 7];
                    hidDesc->wDescriptorLength[1]   = buffer[parseIndex + 8];
                    break;

                default:
                    break;
            }

            parseIndex += subLen;

            /* To avoid some useless data left */
            if ((totalLenTemp - parseIndex) < STD_EP_DESC_SIZE)
            {
                break;
            }
        }
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID configuration handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_ClassInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_HID_INFO_T* usbHostHID;
    uint8_t itfNum;
    uint8_t subClass;
    uint8_t classInterface;
    uint8_t protocolInterface;
    uint16_t mps;
    uint8_t epAddr;
    uint8_t pollInterval;
    uint8_t epNum;
    uint8_t epDir;

    USBH_USR_Debug("USBH_HID_ClassInitHandler");

    /* Link class data */
    usbInfo->activeClass->classData = (USBH_HID_INFO_T*)malloc(sizeof(USBH_HID_INFO_T));
    usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;
    memset(usbHostHID, 0, sizeof(USBH_HID_INFO_T));

    itfNum = USBH_ReadConfigurationItfNum(usbInfo);

    while (itfNum--)
    {
        subClass = USBH_ReadInterfaceSubClass(usbInfo, itfNum);

        if (subClass != USBH_HID_BOOT_CODE)
        {
            USBH_USR_Debug("Interface is not valid");
            usbStatus =  USBH_FAIL;
            continue;
        }

        classInterface = USBH_ReadInterfaceClass(usbInfo, itfNum);
        if (classInterface != USBH_CLASS_HID)
        {
            USBH_USR_Debug("Class type is not support");
            usbStatus = USBH_ERR_NOT_SUP;
            continue;
        }

        protocolInterface = USBH_ReadInterfaceProtocol(usbInfo, itfNum);
        /* Decode class protocol */
        switch (protocolInterface)
        {
            case USBH_HID_KEYBOARD_BOOT_CODE:
                USBH_USR_LOG("Register keyboard class init");
                usbHostHID->callback = &USBH_HID_KEYBOARD_Handler;
                usbStatus = USBH_OK;
                break;

            case USBH_HID_MOUSE_BOOT_CODE:
                USBH_USR_LOG("Register mouse class init");
                usbHostHID->callback = &USBH_HID_MOUSE_Handler;
                usbStatus = USBH_OK;
                break;

            default:
                USBH_USR_LOG("Protocol is not support");
                usbStatus =  USBH_FAIL;
                break;
        }

        epAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, 0);
        mps = USBH_ReadEndpointMPS(usbInfo, itfNum, 0);
        pollInterval = USBH_ReadEndpointInterval(usbInfo, itfNum, 0);

        usbHostHID->epAddr           = epAddr;
        usbHostHID->epSize           = mps;
        usbHostHID->pollInterval     = pollInterval;
        usbHostHID->state            = USBH_HID_INIT;
        usbHostHID->classReqState    = USBH_HID_REQ_INIT;

        if (usbHostHID->pollInterval < USBH_HID_POLL_MIN_NUM)
        {
            usbHostHID->pollInterval = USBH_HID_POLL_MIN_NUM;
        }

        epNum = USBH_ReadInterfaceEpNum(usbInfo, itfNum);

        if (epNum > ENDPOINT_DESC_MAX_NUM)
        {
            epNum = ENDPOINT_DESC_MAX_NUM;
        }

        while (epNum--)
        {
            /* Get endpoint and size */
            epDir = epAddr & 0x80;

            if (epDir)
            {
                usbHostHID->intInEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostHID->inChNum = USBH_CH_AllocChannel(usbInfo, usbHostHID->intInEpAddr);

                USBH_OpenChannelCallback(usbInfo, usbHostHID->inChNum,
                                         usbHostHID->intInEpAddr,
                                         usbInfo->devInfo.address,
                                         usbInfo->devInfo.speed,
                                         EP_TYPE_INTERRUPT,
                                         usbHostHID->epSize);

                USBH_ConfigDataPidCallback(usbInfo, usbHostHID->inChNum, 0);
            }
            else
            {
                usbHostHID->intOutEpAddr = USBH_ReadEndpointAddress(usbInfo, itfNum, epNum);
                usbHostHID->outChNum = USBH_CH_AllocChannel(usbInfo, usbHostHID->intOutEpAddr);

                USBH_OpenChannelCallback(usbInfo, usbHostHID->outChNum,
                                         usbHostHID->intOutEpAddr,
                                         usbInfo->devInfo.address,
                                         usbInfo->devInfo.speed,
                                         EP_TYPE_INTERRUPT,
                                         usbHostHID->epSize);

                USBH_ConfigDataPidCallback(usbInfo, usbHostHID->outChNum, 0);
            }
        }
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID class reset handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_ClassDeInitHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_ClassDeInitHandler");

    if (usbHostHID->inChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostHID->inChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostHID->inChNum);
        usbHostHID->inChNum  = 0;
    }

    if (usbHostHID->outChNum != 0)
    {
        USBH_CloseChannelCallback(usbInfo, usbHostHID->outChNum);
        USBH_CH_FreeChannel(usbInfo, usbHostHID->outChNum);
        usbHostHID->outChNum  = 0;
    }

    if (usbInfo->activeClass->classData != NULL)
    {
        free(usbInfo->activeClass->classData);
        usbInfo->activeClass->classData = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID class reguest handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_ClassReqHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    uint8_t reqStatus = USBH_BUSY;
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    uint16_t descLength;

    USBH_USR_Debug("USBH_HID_ClassReqHandler");

    switch (usbHostHID->classReqState)
    {
        case USBH_HID_REQ_IDLE:
            break;

        case USBH_HID_REQ_INIT:
        case USBH_HID_REQ_GET_HID_DESC:
            USBH_HidDescParse(&usbHostHID->desc, usbInfo->devInfo.desc.cfgDescBuf);
            usbHostHID->classReqState = USBH_HID_REQ_GET_REP_DESC;
            break;

        case USBH_HID_REQ_GET_REP_DESC:
            descLength = usbHostHID->desc.wDescriptorLength[0] | usbHostHID->desc.wDescriptorLength[1] << 8;
            reqStatus = USBH_HID_GetHIDDesc(usbInfo, descLength);
            switch (reqStatus)
            {
                case USBH_OK:
                    usbHostHID->classReqState = USBH_HID_REQ_SET_IDLE;
                    break;

                case USBH_ERR_NOT_SUP:
                    USBH_USR_LOG("Class Req Error: Get report descriptor failed");
                    usbStatus = USBH_FAIL;
                    break;

                default:
                    break;
            }
            break;

        case USBH_HID_REQ_SET_IDLE:
            reqStatus = USBH_HID_SetIdle(usbInfo, 0, 0);
            switch (reqStatus)
            {
                case USBH_OK:
                    usbHostHID->classReqState = USBH_HID_REQ_SET_PROTOCOL;
                    break;

                case USBH_ERR_NOT_SUP:
                    usbHostHID->classReqState = USBH_HID_REQ_SET_PROTOCOL;
                    break;

                default:
                    break;
            }
            break;

        case USBH_HID_REQ_SET_PROTOCOL:
            reqStatus = USBH_HID_SetProtocol(usbInfo, 0);
            switch (reqStatus)
            {
                case USBH_OK:
                    usbHostHID->classReqState = USBH_HID_REQ_IDLE;
                    usbStatus = USBH_OK;
                    break;

                case USBH_ERR_NOT_SUP:
                    USBH_USR_LOG("Class Req Error: Set protocol failed");
                    usbStatus = USBH_FAIL;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID SOF handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_SOFHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;
    uint32_t interval;

    USBH_USR_Debug("USBH_HID_SOFHandler");

    if (usbHostHID->state == USBH_HID_POLL)
    {
        interval = usbInfo->timer - usbHostHID->timer;

        if (interval >= usbHostHID->pollInterval)
        {
            usbHostHID->state = USBH_HID_IN_DATA;
        }
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
static USBH_STA_T USBH_HID_CoreHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    USBH_USR_Debug("USBH_HID_CoreHandler");

    usbStatus = USBH_HID_Handler[usbHostHID->state](usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB host HID handler
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
__weak void USBH_HID_PollCallback(USBH_INFO_T* usbInfo)
{
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;

    /* callback interface */
    usbHostHID->callback->DecodeHandler(usbInfo);
}

/**@} end of group USBH_HID_Functions */
/**@} end of group USBH_HID_Class */
/**@} end of group APM32_USB_Library */
