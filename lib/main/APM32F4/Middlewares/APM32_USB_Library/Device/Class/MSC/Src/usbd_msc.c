/*!
 * @file        usbd_msc.c
 *
 * @brief       usb device msc class handler
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
#include "usbd_msc.h"
#include "usbd_msc_bot.h"
#include "usbd_stdReq.h"
#include "usbd_dataXfer.h"
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_MSC_Class
  @{
  */

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

static USBD_STA_T USBD_MSC_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_MSC_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex);
static USBD_STA_T USBD_MSC_SOFHandler(USBD_INFO_T* usbInfo);
static USBD_STA_T USBD_MSC_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
static USBD_STA_T USBD_MSC_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum);
static USBD_STA_T USBD_MSC_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum);

/**@} end of group USBD_MSC_Functions */

/** @defgroup USBD_MSC_Structures Structures
  @{
  */

/* MSC class handler */
USBD_CLASS_T USBD_MSC_CLASS =
{
    /* Class handler */
    "Class MSC",
    NULL,
    USBD_MSC_ClassInitHandler,
    USBD_MSC_ClassDeInitHandler,
    USBD_MSC_SOFHandler,

    /* Control endpoint */
    USBD_MSC_SetupHandler,
    NULL,
    NULL,
    /* Specific endpoint */
    USBD_MSC_DataInHandler,
    USBD_MSC_DataOutHandler,
    NULL,
    NULL,
};

/**@} end of group USBD_MSC_Structures*/

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

/*!
 * @brief       USB device MSC configuration handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_ClassInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC;

    UNUSED(cfgIndex);

    /* Link class data */
    usbInfo->devClass[usbInfo->classID]->classData = (USBD_MSC_INFO_T*)malloc(sizeof(USBD_MSC_INFO_T));
    usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    memset(usbDevMSC, 0, sizeof(USBD_MSC_INFO_T));

    USBD_USR_Debug("USBD_MSC_INFO_T size %d\r\n", sizeof(USBD_MSC_INFO_T));

    if (usbDevMSC == NULL)
    {
        USBD_USR_LOG("usbDevMSC is NULL");
        return USBD_FAIL;
    }

    usbDevMSC->epInAddr = USBD_MSC_IN_EP_ADDR;
    usbDevMSC->epOutAddr = USBD_MSC_OUT_EP_ADDR;

    /* Open endpoint */
    switch (usbInfo->devSpeed)
    {
        case USBD_SPEED_FS:
            USBD_EP_OpenCallback(usbInfo, usbDevMSC->epOutAddr, EP_TYPE_BULK, USBD_MSC_FS_MP_SIZE);
            usbInfo->devEpOut[usbDevMSC->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevMSC->epInAddr, EP_TYPE_BULK, USBD_MSC_FS_MP_SIZE);
            usbInfo->devEpIn[usbDevMSC->epInAddr & 0x0F].useStatus = ENABLE;
            break;

        default:
            USBD_EP_OpenCallback(usbInfo, usbDevMSC->epOutAddr, EP_TYPE_BULK, USBD_MSC_HS_MP_SIZE);
            usbInfo->devEpOut[usbDevMSC->epOutAddr & 0x0F].useStatus = ENABLE;

            USBD_EP_OpenCallback(usbInfo, usbDevMSC->epInAddr, EP_TYPE_BULK, USBD_MSC_HS_MP_SIZE);
            usbInfo->devEpIn[usbDevMSC->epInAddr & 0x0F].useStatus = ENABLE;
            break;
    }

    USBD_MSC_BOT_Init(usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB device MSC reset handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       cfgIndex: configuration index
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_ClassDeInitHandler(USBD_INFO_T* usbInfo, uint8_t cfgIndex)
{
    USBD_STA_T usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
	
    UNUSED(cfgIndex);

    /* Close MSC EP */
    USBD_EP_CloseCallback(usbInfo, usbDevMSC->epOutAddr);
    usbInfo->devEpOut[usbDevMSC->epOutAddr & 0x0F].useStatus = DISABLE;

    USBD_EP_CloseCallback(usbInfo, usbDevMSC->epInAddr);
    usbInfo->devEpIn[usbDevMSC->epInAddr & 0x0F].useStatus = DISABLE;

    if (usbInfo->devClassUserData[usbInfo->classID] != NULL)
    {
        USBD_MSC_BOT_DeInit(usbInfo);
    }

    if (usbInfo->devClass[usbInfo->classID]->classData != NULL)
    {
        free(usbInfo->devClass[usbInfo->classID]->classData);
        usbInfo->devClass[usbInfo->classID]->classData = 0;
    }

    return usbStatus;
}

/*!
 * @brief       USB device MSC SOF handler
 *
 * @param       usbInfo: usb device information
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_SOFHandler(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_BUSY;

    UNUSED(usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB device MSC SETUP handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       req: setup request
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_SetupHandler(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T  usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint8_t request;
    uint8_t reqType;
    uint16_t wIndex = req->DATA_FIELD.wIndex[0] | req->DATA_FIELD.wIndex[1] << 8;
    uint16_t wValue = req->DATA_FIELD.wValue[0] | req->DATA_FIELD.wValue[1] << 8;
    uint16_t wLength = req->DATA_FIELD.wLength[0] | req->DATA_FIELD.wLength[1] << 8;
    uint16_t status = 0x0000;

    if (usbDevMSC == NULL)
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
                        USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevMSC->itf, 1);
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
                        usbDevMSC->itf = wValue;
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
                    break;

                case USBD_STD_CLEAR_FEATURE:
                    if (usbInfo->devState == USBD_DEV_CONFIGURE)
                    {
                        if (wValue == USBD_FEATURE_SELECTOR_ENDPOINT_HALT)
                        {
                            USBD_EP_FlushCallback(usbInfo, wIndex);

                            /* BOT error */
                            USBD_MSC_BOT_ClearFeature(usbInfo, wIndex);
                        }
                    }
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
                case USBD_CLASS_GET_MAX_LUN:
                    if ((req->DATA_FIELD.bmRequest.REQ_TYPE_B.dir == EP_DIR_IN) && \
                            (wValue == 0) && (wLength == 1))
                    {
                        usbDevMSC->maxLun = \
                                            ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryReadMaxLun();

                        USBD_CtrlSendData(usbInfo, (uint8_t*)&usbDevMSC->maxLun, 1);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
                    break;

                case USBD_CLASS_BOT_RESET:
                    if ((req->DATA_FIELD.bmRequest.REQ_TYPE_B.dir == EP_DIR_OUT) && \
                            (wValue == 0) && (wLength == 0))
                    {
                        USBD_MSC_BOT_Reset(usbInfo);
                    }
                    else
                    {
                        USBD_REQ_CtrlError(usbInfo, req);
                        usbStatus = USBD_FAIL;
                    }
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
 * @brief       USB device MSC IN data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_DataInHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
	
    UNUSED(epNum);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    switch (usbDevMSC->usbDevBOT.state)
    {
        case USBD_BOT_DATAIN:
            reqStatus = USBD_SCSI_Handle(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                         &usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.CB[0]);

            if (reqStatus == USBD_FAIL)
            {
                USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_FAIL);
            }
            break;

        case USBD_BOT_DATAIN_LAST:
        case USBD_BOT_DATA_SEND:
            USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_OK);
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB device MSC OUT data handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       epNum: endpoint number
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_MSC_DataOutHandler(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
	
    UNUSED(epNum);

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    /* Handler BOT state */
    switch (usbDevMSC->usbDevBOT.state)
    {
        case USBD_BOT_IDLE:
            USBD_MSC_BOT_CBW_Decode(usbInfo);
            break;

        case USBD_BOT_DATAOUT:
            reqStatus = USBD_SCSI_Handle(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                         &usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.CB[0]);
            if (reqStatus == USBD_FAIL)
            {
                USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_FAIL);
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB device MSC register memory handler
 *
 * @param       usbInfo: usb device information
 *
 * @param       memory: memory handler
 *
 * @retval      USB device operation status
 */
USBD_STA_T USBD_MSC_RegisterMemory(USBD_INFO_T* usbInfo, USBD_MSC_MEMORY_T* memory)
{
    USBD_STA_T usbStatus = USBD_FAIL;

    if (memory != NULL)
    {
        usbInfo->devClassUserData[usbInfo->classID] = memory;
        usbStatus = USBD_OK;
    }

    return usbStatus;
}

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */
