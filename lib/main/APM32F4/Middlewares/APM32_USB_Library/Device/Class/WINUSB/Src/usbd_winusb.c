/*!
 * @file        usbd_winusb.c
 *
 * @brief       usb device winUSB class handler
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
#include "usbd_winusb.h"
#include "usbd_stdReq.h"
#include "usbd_dataXfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_WINUSB_Class
  @{
  */

/** @defgroup USBD_WINUSB_Functions Functions
  @{
  */

static USBD_STA_T USBD_WINUSB_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_WINUSB_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_WINUSB_SOFHandler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_WINUSB_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_WINUSB_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum);
static USBD_STA_T USBD_WINUSB_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum);

/**@} end of group USBD_WINUSB_Functions */

/** @defgroup USBD_WINUSB_Structures Structures
  @{
  */

/* WINUSB class handler */
USBD_CLASS_T USBD_WINUSB_CLASS =
{
    /* Class handler */
    "Class WINUSB",
    NULL,
    USBD_WINUSB_ClassInitHandler,
    USBD_WINUSB_ClassDeInitHandler,
    USBD_WINUSB_SOFHandler,

    /* Control endpoint */
    USBD_WINUSB_SetupHandler,
    NULL,
    NULL,
    /* Specific endpoint */
    USBD_WINUSB_DataInHandler,
    USBD_WINUSB_DataOutHandler,
    NULL,
    NULL,
};

/**@} end of group USBD_WINUSB_Structures*/

/** @defgroup USBD_WINUSB_Variables Variables
  @{
  */

/**
 * @brief   WinUSB OS feature descriptor
 */
uint8_t USBD_WinUsbOsFeatureDesc[USBD_WINUSB_OS_FEATURE_DESC_SIZE] =
{
    /* dwLength */
    0x28, 0x00, 0x00, 0x00,
    /* bcdVersion */
    0x00, 0x01,
    /* wIndex extended compat ID descritor */
    0x04, 0x00,
    /* bCount */
    0x01,
    /* Reserved */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* WCID Function */
    /* bFirstInterfaceNumber */
    0x00,
    /* bReserved */
    0x00,
    /* CID */
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    /* Sub CID */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Reserved */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* L"DeviceInterfaceGUID" : wIndex = 0x0005 */
/* L"{12345678-1234-1234-1234-123456789ABC}" */
uint8_t USBD_WinUsbOsPropertyDesc[USBD_WINUSB_OS_PROPERTY_DESC_SIZE] = 
{
    /* dwTotalSize = Header + All sections */
    0x8E, 0x00, 0x00, 0x00,
    /* bcdVersion */
    0x00, 0x01,
    /* wIndex */
    0x05, 0x00,
    /* bCount */
    0x01, 0x00,

    /* dwSize - this section */
    0x84, 0x00, 0x00, 0x00,

    /* dwPropertyDataType */
    0x01, 0x00, 0x00, 0x00,

    /* wPropertyNameLength */
    0x28, 0x00,

    /* WCHAR L"DeviceInterfaceGUID" */
    'D', 0x00, 'e', 0x00,
    'v', 0x00, 'i', 0x00,
    'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00,
    't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00,
    'e', 0x00, 'G', 0x00,
    'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00,

    /* dwPropertyDataLength : 78 Bytes = 0x0000004E */
    0x4E, 0x00, 0x00, 0x00,

    /* WCHAR : L"{12345678-1234-1234-1234-123456789ABC}" */
    '{', 0x00, '1', 0x00,
    '2', 0x00, '3', 0x00,
    '4', 0x00, '5', 0x00,
    '6', 0x00, '7', 0x00,
    '8', 0x00, '-', 0x00,
    '1', 0x00, '2', 0x00,
    '3', 0x00, '4', 0x00,
    '-', 0x00, '1', 0x00,
    '2', 0x00, '3', 0x00,
    '4', 0x00, '-', 0x00,
    '1', 0x00, '2', 0x00,
    '3', 0x00, '4', 0x00,
    '-', 0x00, '1', 0x00,
    '2', 0x00, '3', 0x00,
    '4', 0x00, '5', 0x00,
    '6', 0x00, '7', 0x00,
    '8', 0x00, '9', 0x00,
    'A', 0x00, 'B', 0x00,
    'C', 0x00, '}', 0x00,
    0x00, 0x00
};

/**@} end of group USBD_WINUSB_Variables*/

/** @defgroup USBD_WINUSB_Functions Functions
  @{
  */

/*!
 * @brief     USB device WINUSB feature descriptor
 *
 * @param     usbSpeed : usb speed
 *
 * @retval    usb descriptor information
 */
static USBD_DESC_INFO_T USBD_WinUsbFeatureDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    descInfo.desc = USBD_WinUsbOsFeatureDesc;
    descInfo.size = sizeof(USBD_WinUsbOsFeatureDesc);

    return descInfo;
}

/*!
 * @brief     USB device WINUSB property descriptor
 *
 * @param     usbSpeed : usb speed
 *
 * @retval    usb descriptor information
 */
static USBD_DESC_INFO_T USBD_WinUsbPropertyDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    descInfo.desc = USBD_WinUsbOsPropertyDesc;
    descInfo.size = sizeof(USBD_WinUsbOsPropertyDesc);

    return descInfo;
}

/*!
 * @brief       USB device WINUSB configuration handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_WINUSB_INFO_T* usbDevWINUSB;

    /* Link class data */
    usbInfo->devClass[usbInfo->classID]->classData = (USBD_WINUSB_INFO_T*)malloc(sizeof(USBD_WINUSB_INFO_T));
    usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    memset(usbDevWINUSB, 0, sizeof(USBD_WINUSB_INFO_T));

    USBD_USR_Debug("USBD_WINUSB_INFO_T size %d\r\n", sizeof(USBD_WINUSB_INFO_T));

    if (usbDevWINUSB == NULL)
    {
        USBD_USR_LOG("usbDevWINUSB is NULL");
        return USBD_FAIL;
    }

    usbDevWINUSB->epInAddr = USBD_WINUSB_DATA_IN_EP_ADDR;
    usbDevWINUSB->epOutAddr = USBD_WINUSB_DATA_OUT_EP_ADDR;
    
    /* Open Data endpoint */
    switch (usbInfo->devSpeed)
    {
        case USBD_SPEED_FS:
            USBD_EP_OpenCallback(usbInfo, usbDevWINUSB->epOutAddr, EP_TYPE_BULK, USBD_WINUSB_FS_MP_SIZE);
            usbInfo->devEpOut[usbDevWINUSB->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevWINUSB->epInAddr, EP_TYPE_BULK, USBD_WINUSB_FS_MP_SIZE);
            usbInfo->devEpIn[usbDevWINUSB->epInAddr & 0x0F].useStatus = ENABLE;
            break;

        default:
            USBD_EP_OpenCallback(usbInfo, usbDevWINUSB->epOutAddr, EP_TYPE_BULK, USBD_WINUSB_HS_MP_SIZE);
            usbInfo->devEpOut[usbDevWINUSB->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevWINUSB->epInAddr, EP_TYPE_BULK, USBD_WINUSB_HS_MP_SIZE);
            usbInfo->devEpIn[usbDevWINUSB->epInAddr & 0x0F].useStatus = ENABLE;
            break;
    }
    
    /* Interface Init */
    usbDevWINUSB->winusbTx.buffer = NULL;
    usbDevWINUSB->winusbRx.buffer = NULL;
    
    usbDevWINUSB->winusbTx.state = USBD_WINUSB_XFER_IDLE;
    usbDevWINUSB->winusbRx.state = USBD_WINUSB_XFER_IDLE;
    
    ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfInit();
    
    if(usbDevWINUSB->winusbRx.buffer == NULL)
    {
        USBD_USR_LOG("winusbRx buffer is NULL");
        return USBD_FAIL;
    }
    
    switch (usbInfo->devSpeed)
    {
        case USBD_SPEED_FS:
            USBD_EP_ReceiveCallback(usbInfo, usbDevWINUSB->epOutAddr, \
                                    usbDevWINUSB->winusbRx.buffer, \
                                    USBD_WINUSB_FS_MP_SIZE);
            break;

        default:
            USBD_EP_ReceiveCallback(usbInfo, usbDevWINUSB->epOutAddr, \
                                    usbDevWINUSB->winusbRx.buffer, \
                                    USBD_WINUSB_HS_MP_SIZE);
            break;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB reset handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    /* Close WINUSB EP */
    USBD_EP_CloseCallback(usbInfo, usbDevWINUSB->epOutAddr);
    usbInfo->devEpOut[usbDevWINUSB->epOutAddr & 0x0F].useStatus = DISABLE;

    USBD_EP_CloseCallback(usbInfo, usbDevWINUSB->epInAddr);
    usbInfo->devEpIn[usbDevWINUSB->epInAddr & 0x0F].useStatus = DISABLE;
    
    if (usbInfo->devClass[usbInfo->classID]->classData != NULL)
    {
        if(((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit != NULL)
        {
            ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit();
        }
        
        free(usbInfo->devClass[usbInfo->classID]->classData);
        usbInfo->devClass[usbInfo->classID]->classData = 0;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB SOF handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_SOFHandler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_BUSY;

    return usbStatus;
}

/*!
 * @brief     USB WINUSB device receive CTRL status
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_WINUSB_CtrlReceiveData(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devEp0State = USBD_DEV_EP0_DATA_OUT;
    usbInfo->devEpOut[USBD_EP_0].length = length;
    usbInfo->devEpOut[USBD_EP_0].remainLen = length;

    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/*!
 * @brief       USB device WINUSB SETUP handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       req: setup request
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint8_t request;
    uint8_t reqType;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;
    uint16_t wIndex = req->DATA_FIELD.wIndex[0] | req->DATA_FIELD.wIndex[1] << 8;
    uint16_t status = 0x0000;
    uint16_t length;
    
    USBD_DESC_INFO_T descInfo;
    
    request = req->DATA_FIELD.bRequest;
    reqType = usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.type;
    
    if(request != USBD_VEN_REQ_MS_CODE)
    {
        if (usbDevWINUSB == NULL)
        {
            return USBD_FAIL;
        }
    }
    
    switch (reqType)
    {
        case USBD_REQ_TYPE_STANDARD:
            switch (request)
            {
                case USBD_STD_GET_STATUS:
                    if (usbInfo->devState == USBD_DEV_CONFIGURE)
                    {
                        USBD_CtrlSendData(usbInfo, (uint8_t*)&status, 2);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
                    break;

                case USBD_STD_GET_INTERFACE:
                    if (usbInfo->devState == USBD_DEV_CONFIGURE)
                    {
                        usbDevWINUSB->itf = 0;
                        USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevWINUSB->itf, 1);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
                    break;

                case USBD_STD_SET_INTERFACE:
                    if (usbInfo->devState == USBD_DEV_CONFIGURE)
                    {
                        
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
                    break;

                case USBD_STD_CLEAR_FEATURE:
                    break;

                default:
                    USBD_REQ_CtrlError(usbInfo, req);
                    usbStatus = USBD_FAIL;
                    break;
            }
            break;

        case USBD_REQ_TYPE_CLASS:
            if(wLength)
            {
                if((usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE & 0x80) == 0x80)
                {
                    ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfCtrl(request, \
                                                                                                   (uint8_t *)usbDevWINUSB->data,
                                                                                                   wLength);
                    
                    length = USBD_WINUSB_DATA_MP_SIZE < wLength ? USBD_WINUSB_DATA_MP_SIZE : wLength;
                    USBD_CtrlSendData(usbInfo, (uint8_t *)usbDevWINUSB->data, length);
                }
            }
            else
            {
                ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfCtrl(request, \
                                                                                              (uint8_t *)req, \
                                                                                               0);
            }
            break;

        case USBD_REQ_TYPE_VENDOR:
            switch (request)
            {
                case USBD_VEN_REQ_MS_CODE:
                    switch(wIndex)
                    {
                        case USBD_WINUSB_DESC_FEATURE:
                            descInfo = USBD_WinUsbFeatureDescHandler(usbInfo->devSpeed);

                            descInfo.size = descInfo.size < wLength ? descInfo.size : wLength;
                            break;
                        
                        case USBD_WINUSB_DESC_PROPERTY:
                            descInfo = USBD_WinUsbPropertyDescHandler(usbInfo->devSpeed);

                            descInfo.size = descInfo.size < wLength ? descInfo.size : wLength;
                            break;
                        
                        default:
                            USBD_REQ_CtrlError(usbInfo, req);
                            usbStatus = USBD_FAIL;
                            break;
                    }
                    
                    if (descInfo.desc != NULL)
                    {
                        USBD_CtrlSendData(usbInfo, descInfo.desc, descInfo.size);
                    }
                    break;
                
                default:
                    USBD_REQ_CtrlError(usbInfo, req);
                    usbStatus = USBD_FAIL;
                    break;
            }
            break;

        default:
            usbStatus = USBD_FAIL;
            USBD_REQ_CtrlError(usbInfo, req);
            break;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB IN data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

#if defined(USE_DAL_DRIVER)
    PCD_HandleTypeDef* usbdh = (PCD_HandleTypeDef *)usbInfo->dataPoint;
#else
    USBD_HANDLE_T* usbdh = (USBD_HANDLE_T *)usbInfo->dataPoint;
#endif /* USE_DAL_DRIVER */
    if (usbdh == NULL)
    {
        return USBD_FAIL;
    }
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }

#if defined(USE_DAL_DRIVER)
    if((usbInfo->devEpIn[epNum & 0x0F].length > 0) && \
       (usbInfo->devEpIn[epNum & 0x0F].length % usbdh->IN_ep[epNum & 0x0F].maxpacket) == 0)
#else
    if((usbInfo->devEpIn[epNum & 0x0F].length > 0) && \
       (usbInfo->devEpIn[epNum & 0x0F].length % usbdh->epIN[epNum & 0x0F].mps) == 0)
#endif /* USE_DAL_DRIVER */
    {
        usbInfo->devEpIn[epNum & 0x0F].length = 0;
        
        USBD_EP_TransferCallback(usbInfo, epNum, NULL, 0);
    }
    else
    {
        usbDevWINUSB->winusbTx.state = USBD_WINUSB_XFER_IDLE;
        
        if(((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSendEnd != NULL)
        {
            ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSendEnd(epNum, \
                                                                                              usbDevWINUSB->winusbTx.buffer, \
                                                                                              &usbDevWINUSB->winusbTx.length);
        }
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB OUT data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_WINUSB_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevWINUSB->winusbRx.length = USBD_EP_ReadRxDataLenCallback(usbInfo, epNum);
    
    ((USBD_WINUSB_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfReceive(usbDevWINUSB->winusbRx.buffer, \
                                                                                      &usbDevWINUSB->winusbRx.length);
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB configure TX buffer handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       buffer: tx buffer
 *
 * @param       length: tx buffer length
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_WINUSB_ConfigTxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevWINUSB->winusbTx.buffer = buffer;
    usbDevWINUSB->winusbTx.length = length;
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB configure RX buffer handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       buffer: tx buffer
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_WINUSB_ConfigRxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevWINUSB->winusbRx.buffer = buffer;
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB register interface handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       itf: interface handler
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_WINUSB_RegisterItf(USBD_INFO_T* usbInfo, USBD_WINUSB_INTERFACE_T* itf)
{
    USBD_STA_T usbStatus = USBD_FAIL;

    if (itf != NULL)
    {
        usbInfo->devClassUserData[usbInfo->classID] = itf;
        usbStatus = USBD_OK;
    }

    return usbStatus;
}

/*!
 * @brief       USB device WINUSB transmit packet handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_WINUSB_TxPacket(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_BUSY;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }
    
    if(usbDevWINUSB->winusbTx.state == USBD_WINUSB_XFER_IDLE)
    {
        usbDevWINUSB->winusbTx.state = USBD_WINUSB_XFER_BUSY;
        
        usbInfo->devEpIn[usbDevWINUSB->epInAddr & 0x0F].length = usbDevWINUSB->winusbTx.length;
        
        USBD_EP_TransferCallback(usbInfo, usbDevWINUSB->epInAddr, usbDevWINUSB->winusbTx.buffer, usbDevWINUSB->winusbTx.length);
        
        usbStatus = USBD_OK;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device WINUSB receive packet handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_WINUSB_RxPacket(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_BUSY;
    USBD_WINUSB_INFO_T* usbDevWINUSB = (USBD_WINUSB_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevWINUSB == NULL)
    {
        return USBD_FAIL;
    }
    
    if(usbInfo->devSpeed == USBD_SPEED_HS)
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevWINUSB->epOutAddr, \
                                usbDevWINUSB->winusbRx.buffer, \
                                USBD_WINUSB_HS_MP_SIZE);
    }
    else
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevWINUSB->epOutAddr, \
                                usbDevWINUSB->winusbRx.buffer, \
                                USBD_WINUSB_FS_MP_SIZE);
    }
    
    return usbStatus;
}

/*!
 * @brief     USB device WINUSB read interval
 *
 * @param     usbInfo: usb device information
 *
 * @retval    usb interval
 */
uint8_t USBD_WINUSB_ReadInterval(USBD_INFO_T* usbInfo)
{
    uint8_t interval;

    if (usbInfo->devSpeed == USBD_SPEED_FS)
    {
        interval = USBD_WINUSB_FS_INTERVAL;
    }
    else
    {
        interval = USBD_WINUSB_HS_INTERVAL;
    }

    return interval;
}

/**@} end of group USBD_WINUSB_Functions */
/**@} end of group USBD_WINUSB_Class */
/**@} end of group APM32_USB_Library */
