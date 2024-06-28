/*!
 * @file        usbd_cdc.c
 *
 * @brief       usb device cdc class handler
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
#include "usbd_cdc.h"
#include "usbd_stdReq.h"
#include "usbd_dataXfer.h"
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_CDC_Class
  @{
  */

/** @defgroup USBD_CDC_Functions Functions
  @{
  */

static USBD_STA_T USBD_CDC_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_CDC_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_CDC_SOFHandler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_CDC_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_CDC_RxEP0Handler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_CDC_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum);
static USBD_STA_T USBD_CDC_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum);

/**@} end of group USBD_CDC_Functions */

/** @defgroup USBD_CDC_Structures Structures
  @{
  */

/* CDC class handler */
USBD_CLASS_T USBD_CDC_CLASS =
{
    /* Class handler */
    "Class CDC",
    NULL,
    USBD_CDC_ClassInitHandler,
    USBD_CDC_ClassDeInitHandler,
    USBD_CDC_SOFHandler,

    /* Control endpoint */
    USBD_CDC_SetupHandler,
    NULL,
    USBD_CDC_RxEP0Handler,
    /* Specific endpoint */
    USBD_CDC_DataInHandler,
    USBD_CDC_DataOutHandler,
    NULL,
    NULL,
};

/**@} end of group USBD_CDC_Structures*/

/** @defgroup USBD_CDC_Functions Functions
  @{
  */

/*!
 * @brief       USB device CDC configuration handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_CDC_INFO_T* usbDevCDC;

    UNUSED(cfgIndex);

    /* Link class data */
    usbInfo->devClass[usbInfo->classID]->classData = (USBD_CDC_INFO_T*)malloc(sizeof(USBD_CDC_INFO_T));
    usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    memset(usbDevCDC, 0, sizeof(USBD_CDC_INFO_T));

    USBD_USR_Debug("USBD_CDC_INFO_T size %d\r\n", sizeof(USBD_CDC_INFO_T));

    if (usbDevCDC == NULL)
    {
        USBD_USR_LOG("usbDevCDC is NULL");
        return USBD_FAIL;
    }
    
    usbDevCDC->epCmdAddr = USBD_CDC_CMD_EP_ADDR;
    usbDevCDC->epInAddr = USBD_CDC_DATA_IN_EP_ADDR;
    usbDevCDC->epOutAddr = USBD_CDC_DATA_OUT_EP_ADDR;
    
    /* Open Command endpoint */
    USBD_EP_OpenCallback(usbInfo, usbDevCDC->epCmdAddr, EP_TYPE_INTERRUPT, USBD_CDC_CMD_MP_SIZE);
    usbInfo->devEpIn[usbDevCDC->epCmdAddr & 0x0F].useStatus = ENABLE;
    
    /* Open Data endpoint */
    switch (usbInfo->devSpeed)
    {
        case USBD_SPEED_FS:
            USBD_EP_OpenCallback(usbInfo, usbDevCDC->epOutAddr, EP_TYPE_BULK, USBD_CDC_FS_MP_SIZE);
            usbInfo->devEpOut[usbDevCDC->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevCDC->epInAddr, EP_TYPE_BULK, USBD_CDC_FS_MP_SIZE);
            usbInfo->devEpIn[usbDevCDC->epInAddr & 0x0F].useStatus = ENABLE;
        
            usbInfo->devEpIn[usbDevCDC->epCmdAddr & 0x0F].interval = USBD_CDC_FS_INTERVAL;
            break;

        default:
            USBD_EP_OpenCallback(usbInfo, usbDevCDC->epOutAddr, EP_TYPE_BULK, USBD_CDC_HS_MP_SIZE);
            usbInfo->devEpOut[usbDevCDC->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevCDC->epInAddr, EP_TYPE_BULK, USBD_CDC_HS_MP_SIZE);
            usbInfo->devEpIn[usbDevCDC->epInAddr & 0x0F].useStatus = ENABLE;
        
            usbInfo->devEpIn[usbDevCDC->epCmdAddr & 0x0F].interval = USBD_CDC_HS_INTERVAL;
            break;
    }
    
    /* Interface Init */
    usbDevCDC->cdcTx.buffer = NULL;
    usbDevCDC->cdcRx.buffer = NULL;
    
    usbDevCDC->cdcTx.state = USBD_CDC_XFER_IDLE;
    usbDevCDC->cdcRx.state = USBD_CDC_XFER_IDLE;
    
    ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfInit();
    
    if(usbDevCDC->cdcRx.buffer == NULL)
    {
        USBD_USR_LOG("cdcRx buffer is NULL");
        return USBD_FAIL;
    }
    
    switch (usbInfo->devSpeed)
    {
        case USBD_SPEED_FS:
            USBD_EP_ReceiveCallback(usbInfo, usbDevCDC->epOutAddr, \
                                    usbDevCDC->cdcRx.buffer, \
                                    USBD_CDC_FS_MP_SIZE);
            break;

        default:
            USBD_EP_ReceiveCallback(usbInfo, usbDevCDC->epOutAddr, \
                                    usbDevCDC->cdcRx.buffer, \
                                    USBD_CDC_HS_MP_SIZE);
            break;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC reset handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    UNUSED(cfgIndex);

    /* Close CDC EP */
    USBD_EP_CloseCallback(usbInfo, usbDevCDC->epOutAddr);
    usbInfo->devEpOut[usbDevCDC->epOutAddr & 0x0F].useStatus = DISABLE;

    USBD_EP_CloseCallback(usbInfo, usbDevCDC->epInAddr);
    usbInfo->devEpIn[usbDevCDC->epInAddr & 0x0F].useStatus = DISABLE;
    
    USBD_EP_CloseCallback(usbInfo, usbDevCDC->epCmdAddr);
    usbInfo->devEpIn[usbDevCDC->epCmdAddr & 0x0F].useStatus = DISABLE;
    usbInfo->devEpIn[usbDevCDC->epCmdAddr & 0x0F].interval = 0;
    
    if (usbInfo->devClass[usbInfo->classID]->classData != NULL)
    {
        if(((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit != NULL)
        {
            ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfDeInit();
        }
        
        free(usbInfo->devClass[usbInfo->classID]->classData);
        usbInfo->devClass[usbInfo->classID]->classData = 0;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC SOF handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_SOFHandler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_BUSY;

    if(((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSOF != NULL)
    {
        ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSOF();
    }

    return usbStatus;
}

/*!
 * @brief     USB CDC device receive CTRL status
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CDC_CtrlReceiveData(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devEp0State = USBD_DEV_EP0_DATA_OUT;
    usbInfo->devEpOut[USBD_EP_0].length = length;
    usbInfo->devEpOut[USBD_EP_0].remainLen = length;

    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/*!
 * @brief       USB device CDC SETUP handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       req: setup request
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint8_t request;
    uint8_t reqType;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;
    uint16_t status = 0x0000;
    uint16_t length;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    request = req->DATA_FIELD.bRequest;
    reqType = req->DATA_FIELD.bmRequest.REQ_TYPE_B.type;
    
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
                        usbDevCDC->itf = 0;
                        USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevCDC->itf, 1);
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
                if((usbInfo->reqSetup.DATA_FIELD.bmRequest.REQ_TYPE & 0x80) != 0)
                {
                    ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfCtrl(request, \
                                                                                                   (uint8_t *)usbDevCDC->data,
                                                                                                   wLength);
                    
                    length = USBD_CDC_DATA_MP_SIZE < wLength ? USBD_CDC_DATA_MP_SIZE : wLength;
                    USBD_CtrlSendData(usbInfo, (uint8_t *)usbDevCDC->data, length);
                }
                else
                {
                    usbDevCDC->cdcCmd.opcode = request;
                    usbDevCDC->cdcCmd.length = wLength < USBD_EP0_PACKET_MAX_SIZE ? \
                                               wLength : USBD_EP0_PACKET_MAX_SIZE;
                    
                    USBD_CDC_CtrlReceiveData(usbInfo, (uint8_t *)usbDevCDC->data, usbDevCDC->cdcCmd.length);
                }
            }
            else
            {
                ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfCtrl(request, \
                                                                                              (uint8_t *)req, \
                                                                                               0);
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
 * @brief       USB device CDC EP0 receive handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_RxEP0Handler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    if((usbInfo->devClassUserData[usbInfo->classID] != NULL) && (usbDevCDC->cdcCmd.opcode != 0xFF))
    {
        ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfCtrl(usbDevCDC->cdcCmd.opcode, \
                                                                                      (uint8_t *)usbDevCDC->data, \
                                                                                      (uint16_t)usbDevCDC->cdcCmd.length);
        
        usbDevCDC->cdcCmd.opcode = 0xFF;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC IN data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

#if defined(USE_DAL_DRIVER)
    PCD_HandleTypeDef* usbdh = (PCD_HandleTypeDef *)usbInfo->dataPoint;
#else
    USBD_HANDLE_T* usbdh = (USBD_HANDLE_T *)usbInfo->dataPoint;
#endif /* USE_DAL_DRIVER */
    if (usbdh == NULL)
    {
        return USBD_FAIL;
    }
    
    if (usbDevCDC == NULL)
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
        usbDevCDC->cdcTx.state = USBD_CDC_XFER_IDLE;
        
        if(((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSendEnd != NULL)
        {
            ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfSendEnd(epNum, \
                                                                                              usbDevCDC->cdcTx.buffer, \
                                                                                              &usbDevCDC->cdcTx.length);
        }
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC OUT data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevCDC->cdcRx.length = USBD_EP_ReadRxDataLenCallback(usbInfo, epNum);
    
    ((USBD_CDC_INTERFACE_T *)usbInfo->devClassUserData[usbInfo->classID])->ItfReceive(usbDevCDC->cdcRx.buffer, \
                                                                                      &usbDevCDC->cdcRx.length);
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC configure TX buffer handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       buffer: tx buffer
 *
 * @param       length: tx buffer length
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CDC_ConfigTxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevCDC->cdcTx.buffer = buffer;
    usbDevCDC->cdcTx.length = length;
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC configure RX buffer handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       buffer: tx buffer
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CDC_ConfigRxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    usbDevCDC->cdcRx.buffer = buffer;
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC register interface handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       itf: interface handler
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CDC_RegisterItf(USBD_INFO_T* usbInfo, USBD_CDC_INTERFACE_T* itf)
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
 * @brief       USB device CDC transmit packet handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CDC_TxPacket(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_BUSY;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    if(usbDevCDC->cdcTx.state == USBD_CDC_XFER_IDLE)
    {
        usbDevCDC->cdcTx.state = USBD_CDC_XFER_BUSY;
        
        usbInfo->devEpIn[usbDevCDC->epInAddr & 0x0F].length = usbDevCDC->cdcTx.length;
        
        USBD_EP_TransferCallback(usbInfo, usbDevCDC->epInAddr, usbDevCDC->cdcTx.buffer, usbDevCDC->cdcTx.length);
        
        usbStatus = USBD_OK;
    }
    
    return usbStatus;
}

/*!
 * @brief       USB device CDC receive packet handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_CDC_RxPacket(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_BUSY;
    USBD_CDC_INFO_T* usbDevCDC = (USBD_CDC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    
    if (usbDevCDC == NULL)
    {
        return USBD_FAIL;
    }
    
    if(usbInfo->devSpeed == USBD_SPEED_HS)
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevCDC->epOutAddr, \
                                usbDevCDC->cdcRx.buffer, \
                                USBD_CDC_HS_MP_SIZE);
    }
    else
    {
        USBD_EP_ReceiveCallback(usbInfo, usbDevCDC->epOutAddr, \
                                usbDevCDC->cdcRx.buffer, \
                                USBD_CDC_FS_MP_SIZE);
    }
    
    return usbStatus;
}

/*!
 * @brief     USB device CDC read interval
 *
 * @param     usbInfo: usb device information
 *
 * @retval    usb interval
 */
uint8_t USBD_CDC_ReadInterval(USBD_INFO_T* usbInfo)
{
    uint8_t interval;

    if (usbInfo->devSpeed == USBD_SPEED_FS)
    {
        interval = USBD_CDC_FS_INTERVAL;
    }
    else
    {
        interval = USBD_CDC_HS_INTERVAL;
    }

    return interval;
}

/**@} end of group USBD_CDC_Functions */
/**@} end of group USBD_CDC_Class */
/**@} end of group APM32_USB_Library */
