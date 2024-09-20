/*!
 * @file        usbh_stdReq.c
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
#include "usbh_stdReq.h"
#include "usbh_dataXfer.h"
#include "usbh_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

/*!
 * @brief     Parse string descriptor
 *
 * @param     stringDesc : string descriptor
 *
 * @param     buffer : source recevice data
 *
 * @param     length : device descriptor length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_StringDescParse(uint8_t* stringDesc, uint8_t* buffer, uint16_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    uint8_t descType;
    uint16_t stringLen;
    uint16_t i;

    descType = *(uint8_t*)(buffer + 1);

    if (descType == USBH_DESC_STRING)
    {
        stringLen = *(uint8_t*)(buffer + 0) - 2;
        if (stringLen > length)
        {
            stringLen = length;
        }

        buffer += 2;

        /* only copy UNICODE bsting */
        for (i = 0; i < stringLen; i += 2)
        {
            *stringDesc = buffer[i];
            stringDesc++;
        }
    }

    return usbStatus;
}

/*!
 * @brief       Copy data from source buffer to destination buffer.
 *
 * @param       desBuffer: point to destination buffer
 *
 * @param       srcBuf   : point to source buffer
 *
 * @param       len: copy length
 *
 * @retval      None
 */
static void USBH_CopyBuffer(uint8_t* desBuffer, uint8_t* srcBuf, uint32_t len)
{
    while (len--)
    {
        desBuffer[len] = srcBuf[len];
    }
}

/*!
 * @brief     Parse configuration descriptor
 *
 * @param     cfgDesc : configuration descriptor
 *
 * @param     interface : interface of intDesc and epDesc
 *
 * @param     buffer : source recevice data
 *
 * @param     length : device descriptor length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_CfgDescParse(USBH_CFG_DESC_T* cfgDesc, USBH_INTERFACE_T* interface, uint8_t* buffer, uint16_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint16_t totalLenTemp = 0;
    uint16_t parseIndex = 0;
    uint8_t itfIndex = 0;
    uint8_t epIndex = 0;
    uint8_t subLen = 0;

    cfgDesc->bLength                = *(uint8_t*)(buffer + 0);
    cfgDesc->bDescriptorType        = *(uint8_t*)(buffer + 1);

    totalLenTemp = *(uint8_t*)(buffer + 2) | (*(uint8_t*)(buffer + 3) << 8);
    totalLenTemp = ((totalLenTemp) < (CFG_DESC_MAX_LEN) ? (totalLenTemp) : (CFG_DESC_MAX_LEN));

    cfgDesc->wTotalLength[0]        = totalLenTemp & 0xFF;
    cfgDesc->wTotalLength[1]        = (totalLenTemp >> 8) & 0xFF;
    cfgDesc->bNumInterfaces         = *(uint8_t*)(buffer + 4);
    cfgDesc->bConfigurationValue    = *(uint8_t*)(buffer + 5);
    cfgDesc->iConfiguration         = *(uint8_t*)(buffer + 6);
    cfgDesc->bmAttributes           = *(uint8_t*)(buffer + 7);
    cfgDesc->bMaxPower              = *(uint8_t*)(buffer + 8);

    if (cfgDesc->bLength != STD_CFG_DESC_SIZE)
    {
        cfgDesc->bLength = STD_CFG_DESC_SIZE;
    }

    /* USB configuration descriptor lenght > STD_CFG_DESC_SIZE */
    if (length > STD_CFG_DESC_SIZE)
    {
        parseIndex = STD_CFG_DESC_SIZE;

        while (totalLenTemp > parseIndex)
        {
            /* Get descriptor length at first byte */
            subLen = buffer[parseIndex];

            /* Check the descriptor Type at second byte */
            switch (buffer[parseIndex + 1])
            {
                case USBH_DESC_INTERFACE:
                    if (itfIndex < INTERFACE_DESC_MAX_NUM)
                    {
                        /* Move data from parse Buffer to Interface descriptor */
                        USBH_CopyBuffer((uint8_t*)&interface[itfIndex].interfaceDesc,
                                        &buffer[parseIndex],
                                        STD_INTERFACE_DESC_SIZE);

                        itfIndex++;
                        epIndex = 0;
                    }
                    break;

                case USBH_DESC_ENDPOINT:
                    if ((itfIndex > 0) && (epIndex < ENDPOINT_DESC_MAX_NUM))
                    {
                        /* Move data from parse Buffer to Endpoint descriptor */
                        USBH_CopyBuffer((uint8_t*)&interface[itfIndex - 1].endpointDesc[epIndex],
                                        &buffer[parseIndex],
                                        STD_INTERFACE_DESC_SIZE);

                        epIndex++;
                    }
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
 * @brief     Parse device descriptor
 *
 * @param     devDesc : device descriptor
 *
 * @param     buffer : source recevice data
 *
 * @param     length : device descriptor length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_DevDescParse(USBH_DEV_DESC_T* devDesc, uint8_t* buffer, uint16_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    devDesc->bLength            = *(uint8_t*)(buffer + 0);
    devDesc->bDescriptorType    = *(uint8_t*)(buffer + 1);
    devDesc->bcdUSB[0]          = *(uint8_t*)(buffer + 2);
    devDesc->bcdUSB[1]          = *(uint8_t*)(buffer + 3);
    devDesc->bDeviceClass       = *(uint8_t*)(buffer + 4);
    devDesc->bDeviceSubClass    = *(uint8_t*)(buffer + 5);
    devDesc->bDeviceProtocol    = *(uint8_t*)(buffer + 6);
    devDesc->bMaxPacketSize     = *(uint8_t*)(buffer + 7);

    switch (devDesc->bMaxPacketSize)
    {
        case 8:
        case 16:
        case 32:
        case 64:
            devDesc->bMaxPacketSize = devDesc->bMaxPacketSize;
            break;

        default:
            devDesc->bMaxPacketSize = 64;
            break;
    }

    if (length > 8)
    {
        devDesc->idVendor[0]        = *(uint8_t*)(buffer + 8);
        devDesc->idVendor[1]        = *(uint8_t*)(buffer + 9);
        devDesc->idProduct[0]       = *(uint8_t*)(buffer + 10);
        devDesc->idProduct[1]       = *(uint8_t*)(buffer + 11);
        devDesc->bcdDevice[0]       = *(uint8_t*)(buffer + 12);
        devDesc->bcdDevice[1]       = *(uint8_t*)(buffer + 13);
        devDesc->iManufacturer      = *(uint8_t*)(buffer + 14);
        devDesc->iProduct           = *(uint8_t*)(buffer + 15);
        devDesc->iSerialNumber      = *(uint8_t*)(buffer + 16);
        devDesc->bNumConfigurations = *(uint8_t*)(buffer + 17);
    }

    return usbStatus;
}

/*!
 * @brief     Handle control setup transfer.
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : control transfer buffer
 *
 * @param     length : length of response
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_CtrlXferHandler(USBH_INFO_T* usbInfo, uint8_t* buffer, uint16_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            USBH_USR_Debug("USBH_XFER_START");
            usbInfo->ctrl.buffer = buffer;
            usbInfo->ctrl.length = length;
            usbInfo->ctrl.state  = USBH_CTRL_SETUP;
            usbInfo->xferState   = USBH_XFER_WAITING;
            usbStatus            = USBH_BUSY;
            break;

        case USBH_XFER_WAITING:
            USBH_USR_Debug("USBH_XFER_WAITING");
            usbStatus = USBH_CtrlStateHandler[usbInfo->ctrl.state](usbInfo);

            if ((usbStatus == USBH_OK) || (usbStatus == USBH_ERR_NOT_SUP))
            {
                usbInfo->xferState = USBH_XFER_START;
                usbInfo->ctrl.state = USBH_CTRL_IDLE;
            }
            else if (usbStatus == USBH_FAIL)
            {
                usbInfo->xferState = USBH_XFER_START;
            }
            else
            {

            }
            break;
    }

    return usbStatus;
}

/*!
 * @brief     USB host set device feature request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     feature : feature value
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_SetFeature(USBH_INFO_T* usbInfo, uint8_t reqType, \
                               uint8_t feature)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_STD_SET_FEATURE;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = feature;
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
 * @brief     USB host clear device feature request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     feature : feature value
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_ClearFeature(USBH_INFO_T* usbInfo, uint8_t reqType, \
                                 uint8_t epNum)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_STD_CLEAR_FEATURE;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = USBH_FEATURE_SELECTOR_ENDPOINT_HALT;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = 0;

            usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = epNum;
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
 * @brief     USB host set device configuration request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     configuration : configuration value
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_SetConfiguration(USBH_INFO_T* usbInfo, uint8_t reqType, \
                                     uint16_t configuration)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_STD_SET_CONFIGURATION;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = configuration;
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
 * @brief     USB host set device address request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type.
 *
 * @param     addr : device address
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_SetAddr(USBH_INFO_T* usbInfo, uint8_t reqType, \
                            uint8_t addr)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_STD_SET_ADDRESS;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = addr;
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
 * @brief     Config standard get descriptor request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     reqType : Select request type
 *
 * @param     desType : Specifies descriptor type
 *                  This value can be one of the following values:
 *                  @arg USBH_DESC_DEVICE
 *                  @arg USBH_DESC_CONFIGURATION
 *                  @arg USBH_DESC_STRING
 *                  @arg USBH_DESC_INTERFACE
 *                  @arg USBH_DESC_ENDPOINT
 *                  @arg USBH_DESC_DEVICE_QUALIFIER
 *                  @arg USBH_DESC_OTHER_SPEED
 *                  @arg USBH_INTERFACE_POWER
 *
 * @param     buffer : buffer to store descriptor
 *
 * @param     length : Specifies len of request
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_REQ_GetDescriptor(USBH_INFO_T* usbInfo, uint8_t reqType, \
                                  uint16_t desType, uint8_t* buffer, uint16_t length)
{
    USBH_STA_T  usbStatus = USBH_OK;

    switch (usbInfo->xferState)
    {
        case USBH_XFER_START:
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir       = ((reqType & 0x80) >> 7);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.type      = ((reqType & 0x60) >> 5);
            usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.recipient = (reqType & 0x1F);

            usbInfo->ctrl.reqData.DATA_FIELD.bRequest = USBH_STD_GET_DESCRIPTOR;

            usbInfo->ctrl.reqData.DATA_FIELD.wValue[0] = (desType >> 8) & 0xFF;
            usbInfo->ctrl.reqData.DATA_FIELD.wValue[1] = desType & 0xFF;

            if ((desType & 0xFF) == USBH_DESC_STRING)
            {
                usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = USBH_LANG_ID & 0xFF;
                usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = USBH_LANG_ID >> 8;
            }
            else
            {
                usbInfo->ctrl.reqData.DATA_FIELD.wIndex[0] = 0;
                usbInfo->ctrl.reqData.DATA_FIELD.wIndex[1] = 0;
            }

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
 * @brief     USB host set feature
 *
 * @param     usbInfo : usb handler information
 *
 * @param     feature : value of feature
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_SetFeature(USBH_INFO_T* usbInfo, uint8_t feature)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_SetFeature(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                    (USBH_REQ_TYPE_STANDARD << 5) | \
                                    (USBH_RECIPIENT_DEVICE)),
                                    feature);

    return usbStatus;
}

/*!
 * @brief     USB host clear feature
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : number of endpoint
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_ClearFeature(USBH_INFO_T* usbInfo, uint8_t epNum)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_ClearFeature(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                      (USBH_REQ_TYPE_STANDARD << 5) | \
                                      (USBH_RECIPIENT_ENDPOINT)),
                                      epNum);

    return usbStatus;
}

/*!
 * @brief     USB host set configuration
 *
 * @param     usbInfo : usb handler information
 *
 * @param     configuration : value of configuration
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_SetConfiguration(USBH_INFO_T* usbInfo, uint16_t configuration)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_SetConfiguration(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                          (USBH_REQ_TYPE_STANDARD << 5) | \
                                          (USBH_RECIPIENT_DEVICE)),
                                          configuration);

    return usbStatus;
}

/*!
 * @brief     USB host get configuration description
 *
 * @param     usbInfo : usb handler information
 *
 * @param     desLength : length of description
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_GetCfgDesc(USBH_INFO_T* usbInfo, uint16_t desLength)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_GetDescriptor(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                                       (USBH_REQ_TYPE_STANDARD << 5) | \
                                       (USBH_RECIPIENT_DEVICE)), \
                                       USBH_DESC_CONFIGURATION,
                                       usbInfo->devInfo.desc.cfgDescBuf,
                                       desLength);
    if (usbStatus == USBH_OK)
    {
        /* Store received data */
        USBH_CfgDescParse(&usbInfo->devInfo.desc.configuration, \
                          usbInfo->devInfo.desc.interface,
                          usbInfo->devInfo.desc.cfgDescBuf, desLength);
    }

    return usbStatus;
}

/*!
 * @brief     USB host get string description
 *
 * @param     usbInfo : usb handler information
 *
 * @param     stringIndex : string index
 *
 * @param     buffer : buffer to store descriptor
 *
 * @param     desLength : length of description
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_GetStringDesc(USBH_INFO_T* usbInfo, uint8_t stringIndex, uint8_t* buffer, uint16_t desLength)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_GetDescriptor(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                                       (USBH_REQ_TYPE_STANDARD << 5) | \
                                       (USBH_RECIPIENT_DEVICE)), \
                                       (USBH_DESC_STRING | stringIndex << 8),
                                       usbInfo->devInfo.desc.stringBuf,
                                       desLength);

    if (usbStatus == USBH_OK)
    {
        /* Store received data */
        USBH_StringDescParse(usbInfo->devInfo.desc.stringBuf, buffer, desLength);
    }

    return usbStatus;
}

/*!
 * @brief     USB host set device address
 *
 * @param     usbInfo : usb handler information
 *
 * @param     address : device address
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_SetAddr(USBH_INFO_T* usbInfo, uint8_t address)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_SetAddr(usbInfo, ((USBH_REQ_DIR_OUT << 7) | \
                                           (USBH_REQ_TYPE_STANDARD << 5) | \
                                           (USBH_RECIPIENT_DEVICE)),
                                 address);

    return usbStatus;
}

/*!
 * @brief     USB host get device description
 *
 * @param     usbInfo : usb handler information
 *
 * @param     desLength : length of description
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_GetDevDesc(USBH_INFO_T* usbInfo, uint8_t desLength)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbStatus = USBH_REQ_GetDescriptor(usbInfo, ((USBH_REQ_DIR_IN << 7) | \
                                       (USBH_REQ_TYPE_STANDARD << 5) | \
                                       (USBH_RECIPIENT_DEVICE)), \
                                       USBH_DESC_DEVICE,
                                       usbInfo->devInfo.data,
                                       desLength);

    if (usbStatus == USBH_OK)
    {
        /* Store received data */
        USBH_DevDescParse(&usbInfo->devInfo.desc.device, usbInfo->devInfo.data, desLength);
    }

    return usbStatus;
}

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */
