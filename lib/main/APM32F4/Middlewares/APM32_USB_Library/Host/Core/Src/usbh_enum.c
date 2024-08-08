/*!
 * @file        usbh_enum.c
 *
 * @brief       USB host enum hander function
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
#include "usbh_enum.h"
#include "usbh_stdReq.h"
#include "usbh_core.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

static USBH_STA_T USBH_ENUM_IdleHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetDevDescHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_SetAddressHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetConfigurationDescHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetFullConfigurationDescHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetMFCStringHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetProductStringHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ENUM_GetSerialNumStringHandler(USBH_INFO_T* usbInfo);

/**@} end of group USBH_Core_Functions */

/** @defgroup USBH_Core_Structures Structures
  @{
  */

/* USB host enum state handler function */
USBH_EnumHandler_T USBH_EnumHandler[] =
{
    USBH_ENUM_IdleHandler,
    USBH_ENUM_GetDevDescHandler,
    USBH_ENUM_SetAddressHandler,
    USBH_ENUM_GetConfigurationDescHandler,
    USBH_ENUM_GetFullConfigurationDescHandler,
    USBH_ENUM_GetMFCStringHandler,
    USBH_ENUM_GetProductStringHandler,
    USBH_ENUM_GetSerialNumStringHandler,
};

/**@} end of group USBH_Core_Structures*/

/** @defgroup USBH_Core_Functions Functions
  @{
  */

/*!
 * @brief     Handle enum idle stage and get 8 bytes device descriptor
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_IdleHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    usbStatus = USBH_GetDevDesc(usbInfo, 0x08);

    switch (usbStatus)
    {
        case USBH_OK:
            usbInfo->ctrl.channelSize = usbInfo->devInfo.desc.device.bMaxPacketSize;

            /* Open and config in channel */
            USBH_OpenChannelCallback(usbInfo, usbInfo->ctrl.channelInNum,
                                     0x80, usbInfo->devInfo.address,
                                     usbInfo->devInfo.speed, EP_TYPE_CONTROL,
                                     usbInfo->ctrl.channelSize);

            /* Open and config out channel */
            USBH_OpenChannelCallback(usbInfo, usbInfo->ctrl.channelOutNum,
                                     0x00, usbInfo->devInfo.address,
                                     usbInfo->devInfo.speed, EP_TYPE_CONTROL,
                                     usbInfo->ctrl.channelSize);

            usbInfo->hostEnumState = USBH_ENUM_GET_DEV_DESC;
            break;

        case USBH_ERR_NOT_SUP:
            USBH_USR_LOG("CTRL error: get device descriptor");
            usbInfo->devInfo.enumCnt++;

            if (usbInfo->devInfo.enumCnt <= 3)
            {
                /* Free control channel */
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

                /* Reset USB host state machine */
                usbInfo->hostState = USBH_HOST_IDLE;
            }
            else
            {
                USBH_USR_LOG("CTRL error: device no response, please unplug the device");
                usbInfo->hostState = USBH_HOST_ABORT;
            }
            break;

        default:
            break;
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get full bytes device descriptor stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetDevDescHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_GetDevDescHandler");

    usbStatus = USBH_GetDevDesc(usbInfo, STD_DEV_DESC_SIZE);

    switch (usbStatus)
    {
        case USBH_OK:
            USBH_USR_LOG("PID: 0x%04X", usbInfo->devInfo.desc.device.idProduct[0] | \
                         usbInfo->devInfo.desc.device.idProduct[1] << 8);

            USBH_USR_LOG("VID: 0x%04X", usbInfo->devInfo.desc.device.idVendor[0] | \
                         usbInfo->devInfo.desc.device.idVendor[1] << 8);
            USBH_USR_LOG("Endpoint 0 max packet size if %d", usbInfo->devInfo.desc.device.bMaxPacketSize);


            usbInfo->hostEnumState = USBH_ENUM_SET_ADDR;
            break;

        case USBH_ERR_NOT_SUP:
            USBH_USR_LOG("CTRL error: get full device descriptor");
            usbInfo->devInfo.enumCnt++;

            if (usbInfo->devInfo.enumCnt <= 3)
            {
                /* Free control channel */
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

                /* Reset USB host state machine */
                usbInfo->hostState = USBH_HOST_IDLE;
                usbInfo->hostEnumState = USBH_ENUM_IDLE;
            }
            else
            {
                USBH_USR_LOG("CTRL error: device no response, please unplug the device");
                usbInfo->hostState = USBH_HOST_ABORT;
            }
            break;

        default:
            break;
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum set address stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_SetAddressHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_SetAddressHandler");

    usbStatus = USBH_SetAddr(usbInfo, USBH_DEVICE_CONFIGURED_ADDRESS);

    switch (usbStatus)
    {
        case USBH_OK:
            /* Update device address */
            usbInfo->devInfo.address = USBH_DEVICE_CONFIGURED_ADDRESS;
            USBH_USR_LOG("USB device address: %d", usbInfo->devInfo.address);

            USBH_Delay(5);

            /* Modify channels to new device address */
            /* Open and config out channel */
            USBH_OpenChannelCallback(usbInfo, usbInfo->ctrl.channelOutNum,
                                     0x00, usbInfo->devInfo.address,
                                     usbInfo->devInfo.speed, EP_TYPE_CONTROL,
                                     usbInfo->ctrl.channelSize);

            /* Open and config in channel */
            USBH_OpenChannelCallback(usbInfo, usbInfo->ctrl.channelInNum,
                                     0x80, usbInfo->devInfo.address,
                                     usbInfo->devInfo.speed, EP_TYPE_CONTROL,
                                     usbInfo->ctrl.channelSize);

            usbInfo->hostEnumState = USBH_ENUM_GET_CFG_DESC;
            break;

        case USBH_ERR_NOT_SUP:
            USBH_USR_LOG("CTRL error: set device address");
            USBH_USR_LOG("CTRL error: device no response, please unplug the device");
            usbInfo->hostState = USBH_HOST_ABORT;
            usbInfo->hostEnumState = USBH_ENUM_IDLE;
            break;

        default:
            break;
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get configuration descriptor stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetConfigurationDescHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_GetConfigurationDescHandler");

    usbStatus = USBH_GetCfgDesc(usbInfo, STD_CFG_DESC_SIZE);

    switch (usbStatus)
    {
        case USBH_OK:
            usbInfo->hostEnumState = USBH_ENUM_GET_FULL_CFG_DESC;
            break;

        case USBH_ERR_NOT_SUP:
            USBH_USR_LOG("CTRL error: get configuration descriptor");

            usbInfo->devInfo.enumCnt++;

            if (usbInfo->devInfo.enumCnt <= 3)
            {
                /* Free control channel */
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

                /* Reset USB host state machine */
                usbInfo->hostState = USBH_HOST_IDLE;
                usbInfo->hostEnumState = USBH_ENUM_IDLE;
            }
            else
            {
                USBH_USR_LOG("CTRL error: device no response, please unplug the device");
                usbInfo->hostState = USBH_HOST_ABORT;
            }
            break;

        default:
            break;
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get full configuration descriptor stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetFullConfigurationDescHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;
    uint16_t cfgDescTotalTemp;

    USBH_USR_Debug("USBH_ENUM_GetFullConfigurationDescHandler");

    cfgDescTotalTemp = usbInfo->devInfo.desc.configuration.wTotalLength[0] | \
                       usbInfo->devInfo.desc.configuration.wTotalLength[1] << 8;

    usbStatus = USBH_GetCfgDesc(usbInfo, cfgDescTotalTemp);

    switch (usbStatus)
    {
        case USBH_OK:
            usbInfo->hostEnumState = USBH_ENUM_GET_MFC_STRING_DESC;
            break;

        case USBH_ERR_NOT_SUP:
            USBH_USR_LOG("CTRL error: get full configuration descriptor");

            usbInfo->devInfo.enumCnt++;

            if (usbInfo->devInfo.enumCnt <= 3)
            {
                /* Free control channel */
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
                USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

                /* Reset USB host state machine */
                usbInfo->hostState = USBH_HOST_IDLE;
                usbInfo->hostEnumState = USBH_ENUM_IDLE;
            }
            else
            {
                USBH_USR_LOG("CTRL error: device no response, please unplug the device");
                usbInfo->hostState = USBH_HOST_ABORT;
            }
            break;

        default:
            break;
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get manufacturer string stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetMFCStringHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_GetMFCStringHandler");

    if (usbInfo->devInfo.desc.device.iManufacturer == 0)
    {
        USBH_USR_LOG("Manufacturer is N/A");

        usbInfo->hostEnumState = USBH_ENUM_GET_PRODUCT_STRING_DESC;
    }
    else
    {
        usbStatus = USBH_GetStringDesc(usbInfo, usbInfo->devInfo.desc.device.iManufacturer, \
                                       usbInfo->devInfo.desc.stringBuf, 0xFF);

        switch (usbStatus)
        {
            case USBH_OK:
                USBH_USR_LOG("Manufacturer: %s", (char*)usbInfo->devInfo.desc.stringBuf);
                usbInfo->hostEnumState = USBH_ENUM_GET_PRODUCT_STRING_DESC;
                break;

            case USBH_ERR_NOT_SUP:
                USBH_USR_LOG("Manufacturer is N/A");
                usbInfo->hostEnumState = USBH_ENUM_GET_PRODUCT_STRING_DESC;
                break;

            default:
                break;
        }
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get product string stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetProductStringHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_GetProductStringHandler");

    if (usbInfo->devInfo.desc.device.iProduct == 0)
    {
        USBH_USR_LOG("Product is N/A");

        usbInfo->hostEnumState = USBH_ENUM_GET_SERIALNUM_STRING_DESC;
    }
    else
    {
        usbStatus = USBH_GetStringDesc(usbInfo, usbInfo->devInfo.desc.device.iProduct, \
                                       usbInfo->devInfo.desc.stringBuf, 0xFF);

        switch (usbStatus)
        {
            case USBH_OK:
                USBH_USR_LOG("Product: %s", (char*)usbInfo->devInfo.desc.stringBuf);
                usbInfo->hostEnumState = USBH_ENUM_GET_SERIALNUM_STRING_DESC;
                break;

            case USBH_ERR_NOT_SUP:
                USBH_USR_LOG("Product is N/A");
                usbInfo->hostEnumState = USBH_ENUM_GET_SERIALNUM_STRING_DESC;
                break;

            default:
                break;
        }
    }

    return usbEnumStatus;
}

/*!
 * @brief     Handle enum get serial number string stage
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ENUM_GetSerialNumStringHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbEnumStatus = USBH_BUSY;
    uint8_t usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_ENUM_GetSerialNumStringHandler");

    if (usbInfo->devInfo.desc.device.iSerialNumber == 0)
    {
        USBH_USR_LOG("SerialNumber is N/A");
        usbEnumStatus = USBH_OK;
    }
    else
    {
        usbStatus = USBH_GetStringDesc(usbInfo, usbInfo->devInfo.desc.device.iSerialNumber, \
                                       usbInfo->devInfo.desc.stringBuf, 0xFF);

        switch (usbStatus)
        {
            case USBH_OK:
                USBH_USR_LOG("SerialNumber: %s", (char*)usbInfo->devInfo.desc.stringBuf);
                usbEnumStatus = USBH_OK;
                break;

            case USBH_ERR_NOT_SUP:
                USBH_USR_LOG("SerialNumber is N/A");
                usbEnumStatus = USBH_OK;
                break;

            default:
                break;
        }
    }

    return usbEnumStatus;
}

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */
