/*!
 * @file        usbd_stdReq.c
 *
 * @brief       USB standard request process
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
#include "usbd_stdReq.h"
#include "usbd_dataXfer.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Functions Functions
  @{
  */

static USBD_STA_T USBD_REQ_GetStatus(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_ClearFeature(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SetFeature(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SetAddress(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_GetDesc(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SetDesc(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_GetCfg(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SetCfg(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_GetItf(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SetItf(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_REQ_SyncFrame(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);

/**@} end of group USBD_Core_Functions */

/** @defgroup USBD_Core_Structures Structures
  @{
  */

/* Standard device request function match with USBD_STD_REQ_TYPE_T */
USBD_StdDevReqCallback_T USBD_StdDevReqHandler[] =
{
    USBD_REQ_GetStatus,
    USBD_REQ_ClearFeature,
    NULL,
    USBD_REQ_SetFeature,
    NULL,
    USBD_REQ_SetAddress,
    USBD_REQ_GetDesc,
    USBD_REQ_SetDesc,
    USBD_REQ_GetCfg,
    USBD_REQ_SetCfg,
    USBD_REQ_GetItf,
    USBD_REQ_SetItf,
    USBD_REQ_SyncFrame,
};

/**@} end of group USBD_Core_Structures*/

/** @defgroup USBD_Core_Functions Functions
  @{
  */

/*!
 * @brief     USB device get status request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_GetStatus(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;

    switch (usbInfo->devState)
    {
        case USBD_DEV_DEFAULT:
        case USBD_DEV_ADDRESS:
        case USBD_DEV_CONFIGURE:
            if (wLength != 0x02)
            {
                USBD_REQ_CtrlError(usbInfo, req);
                break;
            }
#if USBD_SUP_SELF_PWR
            usbInfo->devCfgStatus = USBD_CFG_SELF_POWER;
#else
            usbInfo->devCfgStatus = USBD_CFG_NONE;
#endif

            if (usbInfo->devRemoteWakeUpStatus == ENABLE)
            {
                usbInfo->devCfgStatus |= USBD_CFG_REMOTE_WAKEUP;
            }

            USBD_CtrlSendData(usbInfo, (uint8_t*)&usbInfo->devCfgStatus, 2);

            break;

        default:
            USBD_REQ_CtrlError(usbInfo, req);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device clear feature request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_ClearFeature(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    uint16_t wValue = req->DATA_FIELD.wValue[0] | req->DATA_FIELD.wValue[1] << 8;

    switch (usbInfo->devState)
    {
        case USBD_DEV_DEFAULT:
        case USBD_DEV_ADDRESS:
        case USBD_DEV_CONFIGURE:
            if (wValue == USBD_FEATURE_REMOTE_WAKEUP)
            {
                usbInfo->devRemoteWakeUpStatus = DISABLE;
                USBD_CtrlSendStatus(usbInfo);
            }
            break;

        default:
            USBD_REQ_CtrlError(usbInfo, req);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device set feature request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SetFeature(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    uint16_t wValue = req->DATA_FIELD.wValue[0] | req->DATA_FIELD.wValue[1] << 8;

    switch (wValue)
    {
        case USBD_FEATURE_REMOTE_WAKEUP:
            usbInfo->devRemoteWakeUpStatus = ENABLE;
            USBD_CtrlSendStatus(usbInfo);
            break;

        case USBD_FEATURE_TEST_MODE:
            usbInfo->devTestModeStatus = req->DATA_FIELD.wIndex[1];
            USBD_CtrlSendStatus(usbInfo);
            break;

        default:
            USBD_REQ_CtrlError(usbInfo, req);
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device set address request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SetAddress(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    uint16_t wIndex = req->DATA_FIELD.wIndex[0] | req->DATA_FIELD.wIndex[1] << 8;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;
    uint16_t wValue = req->DATA_FIELD.wValue[0] | req->DATA_FIELD.wValue[1] << 8;
    uint8_t devAddr = wValue & 0x7F;

    if ((wIndex == 0) && (wLength == 0) && (wValue < 0x80))
    {
        switch (usbInfo->devState)
        {
            case USBD_DEV_CONFIGURE:
                USBD_REQ_CtrlError(usbInfo, req);
                break;

            default:
                usbInfo->devAddr = devAddr;
                USBD_SetDevAddressCallback(usbInfo, devAddr);

                USBD_CtrlSendStatus(usbInfo);

                if (devAddr)
                {
                    usbInfo->devState = USBD_DEV_ADDRESS;
                }
                else
                {
                    usbInfo->devState = USBD_DEV_DEFAULT;
                }
                break;
        }
    }
    else
    {
        USBD_REQ_CtrlError(usbInfo, req);
    }

    return usbStatus;
}

/*!
 * @brief     USB device get descriptor request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_GetDesc(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;
    USBD_DESC_INFO_T descInfo;
    uint8_t reqError = 0;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;

    switch (req->DATA_FIELD.wValue[1])
    {
#if USBD_SUP_LPM
        case USBD_DESC_BOS:
            if (usbInfo->devDesc->bosDescHandler != NULL)
            {
                descInfo = usbInfo->devDesc->bosDescHandler(usbInfo->devSpeed);
            }
            else
            {
                USBD_REQ_CtrlError(usbInfo, req);
                reqError++;
            }
            break;
#endif
        
        case USBD_DESC_DEVICE:
            descInfo = usbInfo->devDesc->deviceDescHandler(usbInfo->devSpeed);
            break;

        case USBD_DESC_CONFIGURATION:
            descInfo = usbInfo->devDesc->configDescHandler(usbInfo->devSpeed);
            break;

        case USBD_DESC_STRING:
            switch (req->DATA_FIELD.wValue[0])
            {
                case USBD_DESC_STR_LANGID:
                    if (usbInfo->devDesc->langIdStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->langIdStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                case USBD_DESC_STR_MFC:
                    if (usbInfo->devDesc->manufacturerStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->manufacturerStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                case USBD_DESC_STR_PRODUCT:
                    if (usbInfo->devDesc->productStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->productStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                case USBD_DESC_STR_SERIAL:
                    if (usbInfo->devDesc->serialStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->serialStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                case USBD_DESC_STR_CONFIG:
                    if (usbInfo->devDesc->configStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->configStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                case USBD_DESC_STR_INTERFACE:
                    if (usbInfo->devDesc->interfaceStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->interfaceStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;
                    
                case USBD_DESC_STR_WINUSB_OS:
                    if (usbInfo->devDesc->winUsbOsStrDescHandler != NULL)
                    {
                        descInfo = usbInfo->devDesc->winUsbOsStrDescHandler(usbInfo->devSpeed);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        reqError++;
                    }
                    break;

                default:
                    USBD_REQ_CtrlError(usbInfo, req);
                    reqError++;
                    break;
            }
            break;

        case USBD_DESC_DEVICE_QUALIFIER:
            if (usbInfo->devSpeed == USBD_SPEED_FS)
            {
                USBD_REQ_CtrlError(usbInfo, req);
                reqError++;
            }
            else
            {
                descInfo = usbInfo->devDesc->devQualifierDescHandler(usbInfo->devSpeed);
            }
            break;

        case USBD_DESC_OTHER_SPEED:
            if (usbInfo->devSpeed == USBD_SPEED_FS)
            {
                USBD_REQ_CtrlError(usbInfo, req);
                reqError++;
            }
            else
            {
                descInfo = usbInfo->devDesc->otherSpeedConfigDescHandler(usbInfo->devSpeed);
            }
            break;

        default:
            USBD_REQ_CtrlError(usbInfo, req);
            reqError++;
            break;
    }

    if (reqError)
    {
        usbStatus = USBD_FAIL;
        return usbStatus;
    }

    if (wLength)
    {
        if (descInfo.size)
        {
            if (descInfo.size > wLength)
            {
                descInfo.size = wLength;
            }

            USBD_CtrlSendData(usbInfo, descInfo.desc, descInfo.size);
        }
        else
        {
            USBD_REQ_CtrlError(usbInfo, req);
        }
    }
    else
    {
        USBD_CtrlSendStatus(usbInfo);
    }

    return usbStatus;
}

/*!
 * @brief     USB device set descriptor request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     req : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SetDesc(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(usbInfo);
    UNUSED(req);

    return usbStatus;
}

/*!
 * @brief     USB device get configuration request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_GetCfg(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;

    if (wLength == 1)
    {
        switch (usbInfo->devState)
        {
            case USBD_DEV_DEFAULT:
            case USBD_DEV_ADDRESS:
                usbInfo->devCfgDefault = 0;
                USBD_CtrlSendData(usbInfo, (uint8_t*)&usbInfo->devCfgDefault, 1);
                break;

            case USBD_DEV_CONFIGURE:
                USBD_CtrlSendData(usbInfo, (uint8_t*)&usbInfo->devCfg, 1);
                break;

            default:
                USBD_REQ_CtrlError(usbInfo, req);
                break;
        }
    }
    else
    {
        USBD_REQ_CtrlError(usbInfo, req);
    }

    return usbStatus;
}

/*!
 * @brief     USB device set configuration request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SetCfg(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;
    static uint8_t cfgIndex;

    cfgIndex = req->DATA_FIELD.wValue[0];

    if (cfgIndex > USBD_SUP_CONFIGURATION_MAX_NUM)
    {
        USBD_REQ_CtrlError(usbInfo, req);

        return USBD_FAIL;
    }

    switch (usbInfo->devState)
    {
        case USBD_DEV_ADDRESS:
            if (cfgIndex == 0)
            {
                USBD_CtrlSendStatus(usbInfo);
            }
            else
            {
                usbInfo->devCfg = cfgIndex;

                /* Set class configuration */
                if (usbInfo->devClass[0] != NULL)
                {
                    usbStatus = usbInfo->devClass[0]->ClassInitHandler(usbInfo, cfgIndex);
                }

                if (usbStatus == USBD_OK)
                {
                    USBD_CtrlSendStatus(usbInfo);
                    usbInfo->devState = USBD_DEV_CONFIGURE;
                }
                else
                {
                    USBD_REQ_CtrlError(usbInfo, req);
                    usbInfo->devState = USBD_DEV_ADDRESS;
                }
            }
            break;

        case USBD_DEV_CONFIGURE:
            if (cfgIndex ==  0)
            {
                usbInfo->devState = USBD_DEV_ADDRESS;
                usbInfo->devCfg = cfgIndex;

                /* Clear class configuration */
                usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, cfgIndex);

                USBD_CtrlSendStatus(usbInfo);
            }
            else if (cfgIndex != usbInfo->devCfg)
            {
                /* Clear old class configuration */
                usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, usbInfo->devCfg);

                usbInfo->devCfg = cfgIndex;

                /* Set class configuration */
                if (usbInfo->devClass[0] != NULL)
                {
                    usbStatus = usbInfo->devClass[0]->ClassInitHandler(usbInfo, cfgIndex);
                }

                if (usbStatus == USBD_OK)
                {
                    USBD_CtrlSendStatus(usbInfo);
                }
                else
                {
                    USBD_REQ_CtrlError(usbInfo, req);
                    /* Clear old class configuration */
                    usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, usbInfo->devCfg);
                    usbInfo->devState = USBD_DEV_ADDRESS;
                }
            }
            else
            {
                USBD_CtrlSendStatus(usbInfo);
            }
            break;

        default:
            USBD_REQ_CtrlError(usbInfo, req);

            /* Clear class configuration */
            if (usbInfo->devClass[0]->ClassDeInitHandler(usbInfo, cfgIndex) != USBD_OK)
            {
                usbStatus = USBD_FAIL;
            }

            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB device get interface request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_GetItf(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(usbInfo);
    UNUSED(req);

    return usbStatus;
}

/*!
 * @brief     USB device set interface request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SetItf(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(usbInfo);
    UNUSED(req);

    return usbStatus;
}

/*!
 * @brief     USB device sync frame request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
static USBD_STA_T USBD_REQ_SyncFrame(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(usbInfo);
    UNUSED(req);

    return usbStatus;
}

/*!
 * @brief     USB device control request error
 *
 * @param     usbInfo : usb handler information
 *
 * @param     setup : setup data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_REQ_CtrlError(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(req);

    USBD_EP_StallCallback(usbInfo, 0x80);
    USBD_EP_StallCallback(usbInfo, 0x00);

    return usbStatus;
}

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */
