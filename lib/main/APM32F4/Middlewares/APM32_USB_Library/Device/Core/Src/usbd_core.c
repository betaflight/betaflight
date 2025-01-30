/*!
 * @file        usbd_core.c
 *
 * @brief       USB device core function
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
#include "usbd_core.h"
#include "usbd_dataXfer.h"
#include "usbd_stdReq.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Functions Functions
  @{
  */

/*!
 * @brief     USB device core init
 *
 * @param     usbInfo : usb handler information
 *
 * @param     usbDevSpeed
 *
 * @param     usbDevDesc
 *
 * @param     usbDevClass
 *
 * @param     userCallback
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Init(USBD_INFO_T* usbInfo, USBD_SPEED_T usbDevSpeed, \
                     USBD_DESC_T* usbDevDesc, \
                     USBD_CLASS_T* usbDevClass, \
                     void (*userCallbackFunc)(struct _USBD_INFO_T*, uint8_t))
{
    USBD_STA_T usbStatus = USBD_OK;

    /* Register descriptor function */
    if (usbDevDesc != NULL)
    {
        usbInfo->devDesc = usbDevDesc;
    }

    /* Register class function */
    if (usbDevClass == NULL)
    {
        usbStatus = USBD_FAIL;
        return usbStatus;
    }
    else
    {
        usbInfo->devClass[usbInfo->classNum++] = usbDevClass;
    }

    /* Register user application */
    usbInfo->userCallback = userCallbackFunc;

    usbInfo->devState = USBD_DEV_DEFAULT;
    /* Set USB device speed */
    usbInfo->devSpeed = usbDevSpeed;

    /* Init USB hardware */
    USBD_HardwareInit(usbInfo);

    return usbStatus;
}

/*!
 * @brief     USB device core de-init
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_DeInit(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;
    
    usbInfo->devState = USBD_DEV_DEFAULT;
    usbInfo->classNum = 0;

    if(usbInfo->devClass[0] != NULL)
    {
        usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, usbInfo->devCfg);
    }
    
    if(usbInfo->dataPoint != NULL)
    {
        USBD_StopCallback(usbInfo);
    
        USBD_StopDeviceCallback(usbInfo);
    }
    
    usbInfo->userCallback = NULL;
    usbInfo->devDesc = NULL;
    
    /* Reset USB hardware */
    USBD_HardwareReset(usbInfo);
    
    return usbStatus;
}

/*!
 * @brief     USB device set speed
 *
 * @param     usbInfo : usb handler information
 *
 * @param     speed : device speed
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_SetSpeed(USBD_INFO_T* usbInfo, USBD_DEVICE_SPEED_T speed)
{
    USBD_STA_T usbStatus = USBD_OK;

    if (speed == USBD_DEVICE_SPEED_FS)
    {
        usbInfo->devSpeed = USBD_SPEED_FS;
    }
    else
    {
        usbInfo->devSpeed = USBD_SPEED_HS;
    }

    return usbStatus;
}

/*!
 * @brief     USB device test mode handle
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_TestModeHandler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(usbInfo);

    return usbStatus;
}

/*!
 * @brief     USB device SETUP stage
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_SetupStage(USBD_INFO_T* usbInfo, uint8_t* setup)
{
    USBD_STA_T usbStatus = USBD_OK;
    
    USBD_EP_INFO_T* ep;

    uint8_t recipient;
    uint8_t reqType;
    uint8_t request;
    uint8_t classIndex;
    uint8_t epAddr;
    uint16_t reqWvalue;
    uint16_t reqWLength;
    
    USBH_SetupReqParse(setup, &usbInfo->reqSetup);

    recipient = usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.recipient;
    reqType = usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.type;
    request = usbInfo->reqSetup.DATA_FIELD.bRequest;
    reqWvalue = usbInfo->reqSetup.DATA_FIELD.wValue[0] | \
                usbInfo->reqSetup.DATA_FIELD.wValue[1] << 8;
    reqWLength = usbInfo->reqSetup.DATA_FIELD.wLength[0] | \
                 usbInfo->reqSetup.DATA_FIELD.wLength[1] << 8;

    usbInfo->devEp0State = USBD_DEV_EP0_SETUP;

    usbInfo->devEp0DataLen = usbInfo->reqSetup.DATA_FIELD.wLength[0] | \
                             usbInfo->reqSetup.DATA_FIELD.wLength[1] << 8;

    switch (recipient)
    {
        case USBD_RECIPIENT_DEVICE:
            switch (reqType)
            {
                case USBD_REQ_TYPE_STANDARD:
                    if (request >= USBD_STD_CNT)
                    {
                        USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                    }
                    else
                    {
                        /* Standard device request */
                        if (USBD_StdDevReqHandler[request] != NULL)
                        {
                            USBD_StdDevReqHandler[request](usbInfo, &usbInfo->reqSetup);
                        }
                    }
                    break;

                case USBD_REQ_TYPE_CLASS:
                case USBD_REQ_TYPE_VENDOR:
                    usbInfo->devClass[usbInfo->classID]->ClassSetup(usbInfo, &usbInfo->reqSetup);
                    break;

                default:
                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                    break;
            }
            break;

        case USBD_RECIPIENT_INTERFACE:
            switch (reqType)
            {
                case USBD_REQ_TYPE_STANDARD:
                case USBD_REQ_TYPE_CLASS:
                case USBD_REQ_TYPE_VENDOR:
                    switch (usbInfo->devState)
                    {
                        case USBD_DEV_DEFAULT:
                        case USBD_DEV_ADDRESS:
                        case USBD_DEV_CONFIGURE:
                            if(request == USBD_VEN_REQ_MS_CODE)
                            {
                                usbInfo->devClass[usbInfo->classID]->ClassSetup(usbInfo, &usbInfo->reqSetup);
                            }
                            else
                            {
                                if (usbInfo->reqSetup.DATA_FIELD.wIndex[0] <= USBD_SUP_INTERFACE_MAX_NUM)
                                {
                                    /* Add multi class support */
                                    classIndex = 0;
                                    if ((classIndex != 0xFF) && (classIndex < USBD_SUP_CLASS_MAX_NUM))
                                    {
                                        usbInfo->classID = classIndex;
                                        
                                        if (usbInfo->devClass[classIndex]->ClassSetup != NULL)
                                        {
                                            usbStatus = usbInfo->devClass[classIndex]->ClassSetup(usbInfo, &usbInfo->reqSetup);
                                        }
                                        else
                                        {
                                            usbStatus = USBD_FAIL;
                                        }
                                    }
                                    else
                                    {
                                        usbStatus = USBD_FAIL;
                                    }

                                    if ((usbStatus == USBD_OK) && (reqWLength == 0))
                                    {
                                        USBD_CtrlSendStatus(usbInfo);
                                    }
                                }
                                else
                                {
                                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                }
                            }
                            break;

                        default:
                            USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                            break;
                    }
                    break;

                default:
                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                    break;
            }
            break;

        case USBD_RECIPIENT_ENDPOINT:
            epAddr = usbInfo->reqSetup.DATA_FIELD.wIndex[0];

            switch (reqType)
            {
                case USBD_REQ_TYPE_STANDARD:
                    switch (request)
                    {
                        case USBD_STD_GET_STATUS:
                            switch (usbInfo->devState)
                            {
                                case USBD_DEV_ADDRESS:
                                    if ((epAddr != 0x00) && (epAddr != 0x80))
                                    {
                                        USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                        break;
                                    }

                                    if ((epAddr & 0x80) == 0x80)
                                    {
                                        ep = &usbInfo->devEpIn[epAddr & 0x7F];

                                    }
                                    else
                                    {
                                        ep = &usbInfo->devEpOut[epAddr & 0x7F];
                                    }

                                    ep->status = 0x0000;

                                    USBD_CtrlSendData(usbInfo, \
                                                      (uint8_t*)&ep->status, \
                                                      2);
                                    break;

                                case USBD_DEV_CONFIGURE:
                                    if ((epAddr & 0x80) == 0x80)
                                    {
                                        if (usbInfo->devEpIn[epAddr & 0x0F].useStatus == DISABLE)
                                        {
                                            USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        if (usbInfo->devEpOut[epAddr & 0x0F].useStatus == DISABLE)
                                        {
                                            USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                            break;
                                        }
                                    }

                                    if ((epAddr & 0x80) == 0x80)
                                    {
                                        ep = &usbInfo->devEpIn[epAddr & 0x7F];

                                    }
                                    else
                                    {
                                        ep = &usbInfo->devEpOut[epAddr & 0x7F];
                                    }

                                    if ((epAddr == 0x00) || epAddr == 0x80)
                                    {
                                        ep->status = 0x0000;
                                    }
                                    else if (USBD_EP_ReadStallStatusCallback(usbInfo, epAddr))
                                    {
                                        ep->status = 0x0001;
                                    }
                                    else
                                    {
                                        ep->status = 0x0000;
                                    }

                                    USBD_CtrlSendData(usbInfo, \
                                                      (uint8_t*)&ep->status, \
                                                      2);
                                    break;

                                default:
                                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                    break;
                            }
                            break;

                        case USBD_STD_CLEAR_FEATURE:
                            switch (usbInfo->devState)
                            {
                                case USBD_DEV_ADDRESS:
                                    if ((epAddr != 0x00) && (epAddr != 0x80))
                                    {
                                        USBD_EP_StallCallback(usbInfo, epAddr);
                                        USBD_EP_StallCallback(usbInfo, 0x80);
                                    }
                                    else
                                    {
                                        USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                    }
                                    break;

                                case USBD_DEV_CONFIGURE:
                                    if (reqWvalue == USBD_FEATURE_SELECTOR_ENDPOINT_HALT)
                                    {
                                        if ((epAddr & 0x7F) != 0x00)
                                        {
                                            USBD_EP_ClearStallCallback(usbInfo, epAddr);
                                        }

                                        USBD_CtrlSendStatus(usbInfo);

                                        /* Add multi class support */
                                        classIndex = 0;
                                        if ((classIndex != 0xFF) && (classIndex < USBD_SUP_CLASS_MAX_NUM))
                                        {
                                            usbInfo->classID = classIndex;

                                            if (usbInfo->devClass[classIndex]->ClassSetup != NULL)
                                            {
                                                usbStatus = usbInfo->devClass[classIndex]->ClassSetup(usbInfo, &usbInfo->reqSetup);
                                            }
                                        }
                                    }
                                    break;

                                default:
                                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                    break;
                            }
                            break;

                        case USBD_STD_SET_FEATURE:
                            switch (usbInfo->devState)
                            {
                                case USBD_DEV_ADDRESS:
                                    if ((epAddr != 0x00) && (epAddr != 0x80))
                                    {
                                        USBD_EP_StallCallback(usbInfo, epAddr);
                                        USBD_EP_StallCallback(usbInfo, 0x80);
                                    }
                                    else
                                    {
                                        USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                    }
                                    break;

                                case USBD_DEV_CONFIGURE:
                                    if (reqWvalue == USBD_FEATURE_SELECTOR_ENDPOINT_HALT)
                                    {
                                        if ((epAddr != 0x00) && (epAddr != 0x80) && \
                                                (reqWLength == 0x00))
                                        {
                                            USBD_EP_StallCallback(usbInfo, epAddr);
                                        }
                                    }

                                    USBD_CtrlSendStatus(usbInfo);
                                    break;

                                default:
                                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                                    break;
                            }
                            break;

                        default:
                            USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                            break;
                    }
                    break;

                case USBD_REQ_TYPE_CLASS:
                case USBD_REQ_TYPE_VENDOR:
                    /* Add multi class support */
                    classIndex = 0;
                    if ((classIndex != 0xFF) && (classIndex < USBD_SUP_CLASS_MAX_NUM))
                    {
                        usbInfo->classID = classIndex;

                        if (usbInfo->devClass[classIndex]->ClassSetup != NULL)
                        {
                            usbStatus = usbInfo->devClass[classIndex]->ClassSetup(usbInfo, &usbInfo->reqSetup);
                        }
                    }
                    break;

                default:
                    USBD_REQ_CtrlError(usbInfo, &usbInfo->reqSetup);
                    break;
            }
            break;

        default:
            usbStatus = USBD_EP_StallCallback(usbInfo, usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.dir);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device data OUT stage
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : endpoint number
 *
 * @param     buffer : data buffer
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_DataOutStage(USBD_INFO_T* usbInfo, uint8_t epNum, uint8_t* buffer)
{
    USBD_STA_T usbStatus = USBD_OK;
    uint32_t ctrlLenTemp;
    uint8_t classIndex;

    if (epNum != 0)
    {
        /* Add multi class support */
        classIndex = 0;

        if ((classIndex != 0xFF) && (classIndex < USBD_SUP_CLASS_MAX_NUM))
        {
            if (usbInfo->devState == USBD_DEV_CONFIGURE)
            {
                if (usbInfo->devClass[classIndex]->ClassDataOut != NULL)
                {
                    usbInfo->classNum = classIndex;
                    usbStatus = usbInfo->devClass[classIndex]->ClassDataOut(usbInfo, epNum);

                    if (usbStatus != USBD_OK)
                    {
                        return usbStatus;
                    }
                }
            }
        }
    }
    /* EP0 */
    else
    {
        switch (usbInfo->devEp0State)
        {
            case USBD_DEV_EP0_DATA_OUT:
                if (usbInfo->devEpOut[USBD_EP_0].remainLen > usbInfo->devEpOut[USBD_EP_0].mp)
                {
                    usbInfo->devEpOut[USBD_EP_0].remainLen -= usbInfo->devEpOut[USBD_EP_0].mp;

                    ctrlLenTemp = usbInfo->devEpOut[USBD_EP_0].remainLen < usbInfo->devEpOut[USBD_EP_0].mp ? \
                                  usbInfo->devEpOut[USBD_EP_0].remainLen : usbInfo->devEpOut[USBD_EP_0].mp;

                    USBD_CtrlReceiveData(usbInfo, buffer, ctrlLenTemp);
                }
                else
                {
                    switch (usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.recipient)
                    {
                        case USBD_RECIPIENT_DEVICE:
                            classIndex = 0;
                            break;

                        case USBD_RECIPIENT_INTERFACE:
                            /* Add multi class support */
                            classIndex = 0;
                            break;

                        case USBD_RECIPIENT_ENDPOINT:
                            /* Add multi class support */
                            classIndex = 0;
                            break;

                        default:
                            classIndex = 0;
                            break;
                    }

                    if (classIndex < USBD_SUP_CLASS_MAX_NUM)
                    {
                        if (usbInfo->devState == USBD_DEV_CONFIGURE)
                        {
                            if (usbInfo->devClass[classIndex]->ClassRxEP0 != NULL)
                            {
                                usbInfo->classNum = classIndex;
                                usbInfo->devClass[classIndex]->ClassRxEP0(usbInfo);
                            }
                        }
                    }

                    USBD_CtrlSendStatus(usbInfo);
                }
                break;

            default:
                
                break;
        }
    }

    return usbStatus;
}

/*!
 * @brief     USB device data IN stage
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : endpoint number
 *
 * @param     buffer : data buffer
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_DataInStage(USBD_INFO_T* usbInfo, uint8_t epNum, uint8_t* buffer)
{
    USBD_STA_T usbStatus = USBD_OK;
    uint8_t classIndex;
    USBD_EP_INFO_T* ep;

    if (epNum)
    {
        /* Add multi class support */
        classIndex = 0;
        if ((classIndex != 0xFF) && (classIndex < USBD_SUP_CLASS_MAX_NUM))
        {
            if (usbInfo->devState == USBD_DEV_CONFIGURE)
            {
                if (usbInfo->devClass[classIndex]->ClassDataIn != NULL)
                {
                    usbInfo->classID = classIndex;

                    usbStatus = usbInfo->devClass[classIndex]->ClassDataIn(usbInfo, epNum);

                    if (usbStatus != USBD_OK)
                    {
                        return usbStatus;
                    }
                }
            }
        }
    }
    else
    {
        ep = &usbInfo->devEpIn[USBD_EP_0];
        
        if (usbInfo->devEp0State == USBD_DEV_EP0_DATA_IN)
        {
            if (ep->remainLen > ep->mp)
            {
                ep->remainLen -= ep->mp;

                USBD_CtrlSendNextData(usbInfo, buffer, ep->remainLen);

                USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, NULL, 0);
            }
            else
            {
                /* last packet is MPS multiple data, so need to send ZLP packet */
                if ((ep->mp == ep->remainLen) && \
                        (ep->length >= ep->mp) && \
                        (ep->length < usbInfo->devEp0DataLen))
                {
                    USBD_CtrlSendNextData(usbInfo, NULL, 0);

                    usbInfo->devEp0DataLen = 0;

                    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, NULL, 0);
                }
                else
                {
                    classIndex = 0;

                    if (usbInfo->devState == USBD_DEV_CONFIGURE)
                    {
                        if (usbInfo->devClass[USBD_EP_0]->ClassTxEP0 != NULL)
                        {
                            usbInfo->classID = classIndex;
                            usbInfo->devClass[USBD_EP_0]->ClassTxEP0(usbInfo);
                        }
                    }
                    USBD_EP_StallCallback(usbInfo, 0x80);
                    USBD_CtrlReceiveStatus(usbInfo);
                }
            }
        }
        else
        {

        }

        if (usbInfo->devTestModeStatus == ENABLE)
        {
            USBD_TestModeHandler(usbInfo);
            usbInfo->devTestModeStatus = DISABLE;
        }
    }

    return usbStatus;
}

/*!
 * @brief     USB device resume
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Resume(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->userCallback(usbInfo, USBD_USER_RESUME);

    if (usbInfo->devState == USBD_DEV_SUSPEND)
    {
        usbInfo->devState = usbInfo->preDevState;
    }

    return usbStatus;
}

/*!
 * @brief     USB device suspend
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Suspend(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->userCallback(usbInfo, USBD_USER_SUSPEND);

    usbInfo->preDevState = usbInfo->devState;
    usbInfo->devState = USBD_DEV_SUSPEND;

    return usbStatus;
}

/*!
 * @brief     USB device reset
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Reset(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devCfg                 = 0;
    usbInfo->devTestModeStatus      = 0;
    usbInfo->devRemoteWakeUpStatus  = 0;
    usbInfo->devState               = USBD_DEV_DEFAULT;
    usbInfo->devEp0State            = USBD_DEV_EP0_IDLE;

    if ((usbInfo->devClass[0] != NULL) && \
            (usbInfo->devClass[0]->ClassDeInitHandler != NULL))
    {
        usbStatus = usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, \
                    usbInfo->devCfg);
        if (usbStatus != USBD_OK)
        {
            usbStatus = USBD_FAIL;
        }
    }

    /* Open EP0 OUT */
    USBD_EP_OpenCallback(usbInfo, 0x00, EP_TYPE_CONTROL, USBD_EP0_PACKET_MAX_SIZE);
    usbInfo->devEpOut[0x00 & 0x0F].useStatus = ENABLE;
    usbInfo->devEpOut[0].mp = USBD_EP0_PACKET_MAX_SIZE;
    
    /* Open EP0 IN */
    USBD_EP_OpenCallback(usbInfo, 0x80, EP_TYPE_CONTROL, USBD_EP0_PACKET_MAX_SIZE);
    usbInfo->devEpIn[0x80 & 0x0F].useStatus = ENABLE;
    usbInfo->devEpIn[0].mp = USBD_EP0_PACKET_MAX_SIZE;
    
    usbInfo->userCallback(usbInfo, USBD_USER_RESET);

    return usbStatus;
}

/*!
 * @brief     USB device SOF handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_HandleSOF(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    if (usbInfo->devState == USBD_DEV_CONFIGURE)
    {
        if ((usbInfo->devClass[0] != NULL) && \
                usbInfo->devClass[0]->ClassSofHandler != NULL)
        {
            usbInfo->devClass[0]->ClassSofHandler(usbInfo);
        }
    }

    return usbStatus;
}

/*!
 * @brief     USB device ISO IN in complete handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : endpoint number
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_IsoInInComplete(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T usbStatus = USBD_OK;

    if (usbInfo->devClass[usbInfo->classID] == NULL)
    {
        return USBD_FAIL;
    }

    if (usbInfo->devState == USBD_DEV_CONFIGURE)
    {
        if (usbInfo->devClass[usbInfo->classID]->ClassIsoInIncomplete != NULL)
        {
            usbInfo->devClass[usbInfo->classID]->ClassIsoInIncomplete(usbInfo, epNum);
        }
    }

    return usbStatus;
}

/*!
 * @brief     USB device ISO OUT in complete handler
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : endpoint number
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_IsoOutInComplete(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T usbStatus = USBD_OK;

    if (usbInfo->devClass[usbInfo->classID] == NULL)
    {
        return USBD_FAIL;
    }

    if (usbInfo->devState == USBD_DEV_CONFIGURE)
    {
        if (usbInfo->devClass[usbInfo->classID]->ClassIsoOutIncomplete != NULL)
        {
            usbInfo->devClass[usbInfo->classID]->ClassIsoOutIncomplete(usbInfo, epNum);
        }
    }

    return usbStatus;
}

/*!
 * @brief     USB device connect handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Connect(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->userCallback(usbInfo, USBD_USER_CONNECT);

    return usbStatus;
}

/*!
 * @brief     USB device disconnect handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_Disconnect(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;

    usbInfo->devState = USBD_DEV_DEFAULT;

    if (usbInfo->devClass[0] != NULL)
    {
        reqStatus = usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, usbInfo->devCfg);

        switch (reqStatus)
        {
            case USBD_OK:
                break;

            default:
                usbStatus = USBD_FAIL;
                break;
        }
    }

    usbInfo->userCallback(usbInfo, USBD_USER_DISCONNECT);

    return usbStatus;
}

/*!
 * @brief     USB device open EP callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @param     epType: endpoint type
 *
 * @param     epMps: endpoint maxinum of packet size
 *
 * @retval    None
 */
__weak void USBD_EP_OpenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                                 uint8_t epType, uint16_t epMps)
{
    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);
    UNUSED(epType);
    UNUSED(epMps);
}

/*!
 * @brief     USB device close EP callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @retval    None
 */
__weak void USBD_EP_CloseCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);
}

/*!
 * @brief     USB device set EP to stall status callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @retval    None
 */
__weak USBD_STA_T USBD_EP_StallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);

    return usbStatus;
}

/*!
 * @brief     USB device read EP last receive data size callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @retval    size of last receive data
 */
__weak uint32_t USBD_EP_ReadRxDataLenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);

    return 0;
}

/*!
 * @brief     USB device clear EP stall status callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @retval    None
 */
__weak USBD_STA_T USBD_EP_ClearStallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);

    return usbStatus;
}

/*!
 * @brief     USB device read EP stall status callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr: endpoint address
 *
 * @retval    Stall status
 */
__weak uint8_t USBD_EP_ReadStallStatusCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    /* callback interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);

    return 0;
}

/*!
 * @brief     USB device EP receive handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr : endpoint address
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
__weak USBD_STA_T USBD_EP_ReceiveCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
        uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* Callback Interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);
    UNUSED(buffer);
    UNUSED(length);

    return usbStatus;
}

/*!
 * @brief     USB device EP transfer handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr : endpoint address
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
__weak USBD_STA_T USBD_EP_TransferCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
        uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* Callback Interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);
    UNUSED(buffer);
    UNUSED(length);

    return usbStatus;
}

/*!
 * @brief     USB device flush EP handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epAddr : endpoint address
 *
 * @retval    usb device status
 */
__weak USBD_STA_T USBD_EP_FlushCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* Callback Interface */
    UNUSED(usbInfo);
    UNUSED(epAddr);

    return usbStatus;
}

/*!
 * @brief     USB device set device address handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @param     address : address
 *
 * @retval    usb device status
 */
__weak USBD_STA_T USBD_SetDevAddressCallback(USBD_INFO_T* usbInfo, uint8_t address)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* Callback Interface */
    UNUSED(usbInfo);
    UNUSED(address);

    return usbStatus;
}

/*!
 * @brief     USB device start handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBD_StartCallback(USBD_INFO_T* usbInfo)
{
    /* Callback Interface */
    UNUSED(usbInfo);
}

/*!
 * @brief     USB device stop handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBD_StopCallback(USBD_INFO_T* usbInfo)
{
    /* Callback Interface */
    UNUSED(usbInfo);
}

/*!
 * @brief     USB device stop device mode handler callback
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBD_StopDeviceCallback(USBD_INFO_T* usbInfo)
{
    /* Callback Interface */
    UNUSED(usbInfo);
}

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */
