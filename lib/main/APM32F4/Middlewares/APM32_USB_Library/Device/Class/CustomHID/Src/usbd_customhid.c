/*!
 * @file        usbd_customhid.c
 *
 * @brief       usb device custom hid class handler
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
#include "usbd_customhid.h"
#include "usbd_stdReq.h"
#include "usbd_dataXfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_CUSTOM_HID_Class
  @{
  */

/** @defgroup USBD_CUSTOM_HID_Functions Functions
  @{
  */

static USBD_STA_T USBD_CUSTOM_HID_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_CUSTOM_HID_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_CUSTOM_HID_SOFHandler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_CUSTOM_HID_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_CUSTOM_HID_RxEP0Handler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_CUSTOM_HID_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum);
static USBD_STA_T USBD_CUSTOM_HID_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum);

static USBD_DESC_INFO_T USBD_CUSTOM_HID_ReportDescHandler(USBD_INFO_T* usbInfo);
static USBD_DESC_INFO_T USBD_CUSTOM_HID_DescHandler(uint8_t usbSpeed);

/**@} end of group USBD_CUSTOM_HID_Functions */

/** @defgroup USBD_CUSTOM_HID_Structures Structures
  @{
  */

/* CUSTOM HID class handler */
USBD_CLASS_T USBD_CUSTOM_HID_CLASS =
{
    /* Class handler */
    "Class custom HID",
    NULL,
    USBD_CUSTOM_HID_ClassInitHandler,
    USBD_CUSTOM_HID_ClassDeInitHandler,
    USBD_CUSTOM_HID_SOFHandler,

    /* Control endpoint */
    USBD_CUSTOM_HID_SetupHandler,
    NULL,
    USBD_CUSTOM_HID_RxEP0Handler,
    /* Specific endpoint */
    USBD_CUSTOM_HID_DataInHandler,
    USBD_CUSTOM_HID_DataOutHandler,
    NULL,
    NULL,
};

/**@} end of group USBD_CUSTOM_HID_Structures*/

/** @defgroup USBD_CUSTOM_HID_Variables Variables
  @{
  */

/**
 * @brief   HID descriptor
 */
uint8_t USBD_HIDDesc[USBD_CUSTOM_HID_DESC_SIZE] =
{
    /* bLength */
    0x09,
    /* bDescriptorType: HID */
    USBD_DESC_HID,
    /* bcdHID */
    0x11, 0x01,
    /* bCountryCode */
    0x00,
    /* bNumDescriptors */
    0x01,
    /* bDescriptorType */
    USBD_DESC_HID_REPORT,
    /* wItemLength */
    USBD_CUSTOM_HID_REPORT_DESC_SIZE & 0xFF, USBD_CUSTOM_HID_REPORT_DESC_SIZE >> 8,
};

/**@} end of group USBD_CUSTOM_HID_Variables*/

/** @defgroup USBD_CUSTOM_HID_Functions Functions
  @{
  */

/*!
 * @brief       USB device CUSTOM HID configuration handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_CUSTOM_HID_INFO_T* usbDevHID;

    /* Link class data */
    usbInfo->devClass[usbInfo->classID]->classData = (USBD_CUSTOM_HID_INFO_T*)malloc(sizeof(USBD_CUSTOM_HID_INFO_T));
    usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    memset(usbDevHID, 0, sizeof(USBD_CUSTOM_HID_INFO_T));

    USBD_USR_Debug("USBD_CUSTOM_HID_INFO_T size %d\r\n", sizeof(USBD_CUSTOM_HID_INFO_T));

    if (usbDevHID == NULL)
    {
        USBD_USR_LOG("usbDevHID is NULL");
        return USBD_FAIL;
    }

    usbDevHID->epInAddr = USBD_CUSTOM_HID_IN_EP_ADDR;
    usbDevHID->epOutAddr = USBD_CUSTOM_HID_OUT_EP_ADDR;

    if (usbInfo->devSpeed == USBD_SPEED_FS)
    {
        usbInfo->devEpIn[usbDevHID->epInAddr & 0x0F].interval = USBD_CUSTOM_HID_FS_INTERVAL;
        usbInfo->devEpOut[usbDevHID->epOutAddr & 0x0F].interval = USBD_CUSTOM_HID_FS_INTERVAL;
    }
    else
    {
        usbInfo->devEpIn[usbDevHID->epInAddr & 0x0F].interval = USBD_CUSTOM_HID_HS_INTERVAL;
        usbInfo->devEpOut[usbDevHID->epOutAddr & 0x0F].interval = USBD_CUSTOM_HID_HS_INTERVAL;
    }

    /* Open endpoint */
    USBD_EP_OpenCallback(usbInfo, usbDevHID->epInAddr, EP_TYPE_INTERRUPT, USBD_CUSTOM_HID_IN_EP_SIZE);
    usbInfo->devEpIn[usbDevHID->epInAddr & 0x0F].useStatus = ENABLE;

    USBD_EP_OpenCallback(usbInfo, usbDevHID->epOutAddr, EP_TYPE_INTERRUPT, USBD_CUSTOM_HID_OUT_EP_SIZE);
    usbInfo->devEpOut[usbDevHID->epOutAddr & 0x0F].useStatus = ENABLE;
    
    ((USBD_CUSTOM_HID_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfInit();
    
    USBD_EP_ReceiveCallback(usbInfo, usbDevHID->epOutAddr, \
                            usbDevHID->report, \
                            USBD_CUSTOM_HID_OUT_EP_SIZE);

    usbDevHID->state = USBD_CUSTOM_HID_IDLE;

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID reset handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    /* Close HID EP */
    USBD_EP_CloseCallback(usbInfo, usbDevHID->epInAddr);
    usbInfo->devEpIn[usbDevHID->epInAddr & 0x0F].interval = 0;
    usbInfo->devEpIn[usbDevHID->epInAddr & 0x0F].useStatus = DISABLE;
    
    USBD_EP_CloseCallback(usbInfo, usbDevHID->epOutAddr);
    usbInfo->devEpOut[usbDevHID->epOutAddr & 0x0F].useStatus = DISABLE;

    if (usbInfo->devClass[usbInfo->classID]->classData != NULL)
    {
        if(((USBD_CUSTOM_HID_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit != NULL)
        {
            ((USBD_CUSTOM_HID_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit();
        }
        free(usbInfo->devClass[usbInfo->classID]->classData);
        usbInfo->devClass[usbInfo->classID]->classData = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID SOF handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_SOFHandler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_BUSY;

    return usbStatus;
}

/*!
 * @brief     USB CUSTOM HID device receive CTRL status
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CUSTOM_HID_CtrlReceiveData(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devEp0State = USBD_DEV_EP0_DATA_OUT;
    usbInfo->devEpOut[USBD_EP_0].length = length;
    usbInfo->devEpOut[USBD_EP_0].remainLen = length;

    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID SETUP handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       req: setup request
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    USBD_DESC_INFO_T descInfo;
    uint8_t request;
    uint8_t reqType;
    uint16_t wValue = req->DATA_FIELD.wValue[0] | req->DATA_FIELD.wValue[1] << 8;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;
    uint16_t status = 0x0000;

    if (usbDevHID == NULL)
    {
        USBD_USR_LOG("usbDevHID is NULL");
        return USBD_FAIL;
    }

    request = req->DATA_FIELD.bRequest;
    reqType = usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE_B.type;

    switch (reqType)
    {
        case USBD_REQ_TYPE_STANDARD:
            switch (request)
            {
                /* HID descriptor */
                case USBD_STD_GET_DESCRIPTOR:
                    switch (req->DATA_FIELD.wValue[1])
                    {
                        case USBD_DESC_HID_REPORT:
                            descInfo = USBD_CUSTOM_HID_ReportDescHandler(usbInfo);

                            descInfo.size = descInfo.size < wLength ? descInfo.size : wLength;
                            break;

                        case USBD_DESC_HID:
                            descInfo = USBD_CUSTOM_HID_DescHandler(usbInfo->devSpeed);

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
                        USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevHID->altSettingStatus, 1);
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
                        usbDevHID->altSettingStatus = wValue;
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
            switch (request)
            {
                case USBD_CLASS_SET_IDLE:
                    usbDevHID->idleStatus = req->DATA_FIELD.wValue[1];
                    break;

                case USBD_CLASS_GET_IDLE:
                    USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevHID->idleStatus, 1);
                    break;

                case USBD_CLASS_SET_PROTOCOL:
                    usbDevHID->protocol = req->DATA_FIELD.wValue[0];
                    break;

                case USBD_CLASS_GET_PROTOCOL:
                    USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevHID->protocol, 1);
                    break;
                case USBD_CLASS_SET_REPORT:
                    if (wLength < USBD_CUSTOM_HID_OUT_EP_SIZE)
                    {
                        USBD_CUSTOM_HID_CtrlReceiveData(usbInfo, usbDevHID->report, wLength);
                        usbDevHID->reportSize = wLength;
                    }
                    else
                    {
                        USBD_CUSTOM_HID_CtrlReceiveData(usbInfo, usbDevHID->report, USBD_CUSTOM_HID_OUT_EP_SIZE);
                        usbDevHID->reportSize = USBD_CUSTOM_HID_OUT_EP_SIZE;
                    }

                    usbDevHID->getReport = 1;
                    break;
                default:
                    USBD_REQ_CtrlError(usbInfo, req);
                    usbStatus = USBD_FAIL;
                    break;
            }
            break;

        case USBD_REQ_TYPE_VENDOR:
            USBD_REQ_CtrlError(usbInfo, req);
            usbStatus = USBD_FAIL;
            break;

        default:
            usbStatus = USBD_FAIL;
            USBD_REQ_CtrlError(usbInfo, req);
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID EP0 receive handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_RxEP0Handler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevHID == NULL)
    {
        return USBD_FAIL;
    }

    if((usbInfo->devClassUserData[usbInfo->classID] != NULL) && (usbDevHID->getReport == 1))
    {
        ((USBD_CUSTOM_HID_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfReceive(usbDevHID->report, \
                                                                                          &usbDevHID->reportSize);
        usbDevHID->getReport = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID IN data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevHID == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevHID->state = USBD_CUSTOM_HID_IDLE;

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID OUT data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CUSTOM_HID_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevHID == NULL)
    {
        return USBD_FAIL;
    }

    usbDevHID->reportSize =  USBD_EP_ReadRxDataLenCallback(usbInfo, epNum);

    ((USBD_CUSTOM_HID_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfReceive(usbDevHID->report, \
                                                                                      &usbDevHID->reportSize);


    return usbStatus;
}

/*!
 * @brief     USB device CUSTOM HID report descriptor
 *
 * @param     usbSpeed : usb speed
 *
 * @retval    usb descriptor information
 */
static USBD_DESC_INFO_T USBD_CUSTOM_HID_ReportDescHandler(USBD_INFO_T* usbInfo)
{
    USBD_DESC_INFO_T descInfo;

    descInfo.desc = (uint8_t*)(((USBD_CUSTOM_HID_INTERFACE_T*)usbInfo->devClassUserData[usbInfo->classID])->report);
    descInfo.size = USBD_CUSTOM_HID_REPORT_DESC_SIZE;

    return descInfo;
}

/*!
 * @brief     USB device CUSTOM HID descriptor
 *
 * @param     usbSpeed : usb speed
 *
 * @retval    usb descriptor information
 */
static USBD_DESC_INFO_T USBD_CUSTOM_HID_DescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    descInfo.desc = USBD_HIDDesc;
    descInfo.size = sizeof(USBD_HIDDesc);

    return descInfo;
}

/*!
 * @brief     USB device CUSTOM HID send report descriptor
 *
 * @param     usbInfo: usb device information
 *
 * @param     report: report buffer
 *
 * @param     length: report data length
 *
 * @retval    usb descriptor information
 */
USBD_STA_T USBD_CUSTOM_HID_TxReport(USBD_INFO_T* usbInfo, uint8_t* report, uint16_t length)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevHID == NULL)
    {
        return USBD_FAIL;
    }

    switch (usbInfo->devState)
    {
        case USBD_DEV_CONFIGURE:
            if (usbDevHID->state == USBD_CUSTOM_HID_IDLE)
            {
                usbDevHID->state = USBD_CUSTOM_HID_BUSY;
                USBD_EP_TransferCallback(usbInfo, usbDevHID->epInAddr, report, length);
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB device CUSTOM HID receive packet handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CUSTOM_HID_RxPacket(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_BUSY;
    USBD_CUSTOM_HID_INFO_T* usbDevHID = (USBD_CUSTOM_HID_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevHID == NULL)
    {
        return USBD_FAIL;
    }
    
    if(usbInfo->devSpeed == USBD_SPEED_HS)
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevHID->epOutAddr, \
                                usbDevHID->report, \
                                USBD_CUSTOM_HID_OUT_EP_SIZE);
    }
    else
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevHID->epOutAddr, \
                                usbDevHID->report, \
                                USBD_CUSTOM_HID_OUT_EP_SIZE);
    }
    
    return usbStatus;
}

/*!
 * @brief     USB device CUSTOM HID read interval
 *
 * @param     usbInfo: usb device information
 *
 * @retval    usb interval
 */
uint8_t USBD_CUSTOM_HID_ReadInterval(USBD_INFO_T* usbInfo)
{
    uint8_t interval;

    if (usbInfo->devSpeed == USBD_SPEED_FS)
    {
        interval = USBD_CUSTOM_HID_FS_INTERVAL;
    }
    else
    {
        interval = ((1 << (USBD_CUSTOM_HID_FS_INTERVAL - 1)) / 8);
    }

    return interval;
}

/*!
 * @brief       USB device CUSTOM HID register interface handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       itf: interface handler
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CUSTOM_HID_RegisterItf(USBD_INFO_T* usbInfo, USBD_CUSTOM_HID_INTERFACE_T* itf)
{
    USBD_STA_T usbStatus = USBD_FAIL;

    if (itf != NULL)
    {
        usbInfo->devClassUserData[usbInfo->classID] = itf;
        usbStatus = USBD_OK;
    }

    return usbStatus;
}

/**@} end of group USBD_CUSTOM_HID_Functions */
/**@} end of group USBD_CUSTOM_HID_Class */
/**@} end of group APM32_USB_Library */
