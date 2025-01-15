/*!
 * @file        usbd_msc_bot.c
 *
 * @brief       usb device msc bot handler
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
#include "usbd_msc_bot.h"
#include "usbd_msc.h"
#include "usbd_dataXfer.h"

#pragma GCC diagnostic ignored "-Wmissing-prototypes"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_MSC_Class
  @{
  */

/** @defgroup USBD_MSC_Functions Functions
  @{
  */

/*!
 * @brief     Init BOT of USB device MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_Init(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevSCSI.senseHead = 0;
    usbDevMSC->usbDevSCSI.senseEnd = 0;
    usbDevMSC->usbDevSCSI.mediumState = USBD_SCSI_MEDIUM_UNLOCK;

    /* Init USB device memory managment */
    ((USBD_MSC_MEMORY_T*)usbInfo->devClassUserData[usbInfo->classID])->MemoryInit(0);

    usbDevMSC->usbDevBOT.state = USBD_BOT_IDLE;
    usbDevMSC->usbDevBOT.status = USBD_BOT_NORMAL;

    USBD_EP_FlushCallback(usbInfo, usbDevMSC->epInAddr);
    USBD_EP_FlushCallback(usbInfo, usbDevMSC->epOutAddr);

    USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                            (uint8_t*)&usbDevMSC->usbDevBOT.cmdPack.CBW,
                            USBD_MSC_BOT_CBW_LEN);
    return usbStatus;
}

/*!
 * @brief     De-init BOT of USB device MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_DeInit(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.state = USBD_BOT_IDLE;

    return usbStatus;
}

/*!
 * @brief     Reset BOT of USB device MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_Reset(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.state = USBD_BOT_IDLE;
    usbDevMSC->usbDevBOT.status = USBD_BOT_RECOVERY;

    USBD_EP_ClearStallCallback(usbInfo, usbDevMSC->epInAddr);
    USBD_EP_ClearStallCallback(usbInfo, usbDevMSC->epOutAddr);

    USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                            (uint8_t*)&usbDevMSC->usbDevBOT.cmdPack.CBW, \
                            USBD_MSC_BOT_CBW_LEN);

    return usbStatus;
}

/*!
 * @brief     Abort BOT of USB device MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_Abort(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    if ((usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_Flag == 0) && \
            (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen != 0) && \
            (usbDevMSC->usbDevBOT.status == USBD_BOT_NORMAL))
    {
        USBD_EP_StallCallback(usbInfo, usbDevMSC->epOutAddr);
    }

    USBD_EP_StallCallback(usbInfo, usbDevMSC->epInAddr);
    
    if (usbDevMSC->usbDevBOT.status == USBD_BOT_ERR)
    {
         USBD_EP_StallCallback(usbInfo, usbDevMSC->epInAddr);
         USBD_EP_StallCallback(usbInfo, usbDevMSC->epOutAddr);
    }

    return usbStatus;
}

/*!
 * @brief     Send CSW packet
 *
 * @param     usbInfo : usb handler information
 *
 * @param     status : CSW status
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_SendCSW(USBD_INFO_T* usbInfo, USBD_BOT_CSW_STA_T status)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dSignature = USBD_MSC_BOT_CSW_SIGNATURE;
    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.bStatus = status;

    usbDevMSC->usbDevBOT.state = USBD_BOT_IDLE;

    USBD_EP_TransferCallback(usbInfo, usbDevMSC->epInAddr, \
                             (uint8_t*)&usbDevMSC->usbDevBOT.cmdPack.CSW, \
                             USBD_MSC_BOT_CSW_LEN);

    USBD_EP_ReceiveCallback(usbInfo, usbDevMSC->epOutAddr, \
                            (uint8_t*)&usbDevMSC->usbDevBOT.cmdPack.CBW, \
                            USBD_MSC_BOT_CBW_LEN);

    return usbStatus;
}

/*!
 * @brief     Send BOT data
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : data length
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_SendData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBD_STA_T  usbStatus = USBD_OK;

    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    uint32_t lengthTemp;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    lengthTemp = length;

    if (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen < length)
    {
        lengthTemp = usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen;
    }

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dDataResidue -= length;
    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.bStatus = USBD_BOT_CSW_OK;
    usbDevMSC->usbDevBOT.state = USBD_BOT_DATA_SEND;

    USBD_EP_TransferCallback(usbInfo, usbDevMSC->epInAddr, buffer, lengthTemp);

    return usbStatus;
}

/*!
 * @brief     Decode CBW packet
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_CBW_Decode(USBD_INFO_T* usbInfo)
{
    USBD_STA_T  usbStatus = USBD_OK;
    uint8_t reqStatus = USBD_BUSY;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;
    uint32_t lastRevDataLen;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dTag = \
            usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dTag;

    usbDevMSC->usbDevBOT.cmdPack.CSW.DATA_FIELD.dDataResidue = \
            usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen;

    lastRevDataLen = USBD_EP_ReadRxDataLenCallback(usbInfo, usbDevMSC->epOutAddr);

    if ((lastRevDataLen != USBD_MSC_BOT_CBW_LEN) || \
            (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.dSignature != USBD_MSC_BOT_CBW_SIGNATURE) || \
            (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN > 1) || \
            (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bCBLen < 1) || \
            (usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bCBLen > 16))
    {
        USBD_SCSI_CodeSense(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                            USBD_SCSI_SENSE_KEY_ILLEGAL_REQUEST, USBD_SCSI_ASC_INVALID_CDB, 0);

        usbDevMSC->usbDevBOT.status = USBD_BOT_ERR;
        USBD_MSC_BOT_Abort(usbInfo);
    }
    else
    {
        reqStatus = USBD_SCSI_Handle(usbInfo, usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.bLUN, \
                                     &usbDevMSC->usbDevBOT.cmdPack.CBW.DATA_FIELD.CB[0]);
        if (reqStatus == USBD_FAIL)
        {
            if (usbDevMSC->usbDevBOT.state == USBD_BOT_NO_DATA)
            {
                USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_FAIL);
            }
            else
            {
                USBD_MSC_BOT_Abort(usbInfo);
            }
        }
        else if ((usbDevMSC->usbDevBOT.state != USBD_BOT_DATAIN) && \
                 (usbDevMSC->usbDevBOT.state != USBD_BOT_DATAOUT) && \
                 (usbDevMSC->usbDevBOT.state != USBD_BOT_DATAIN_LAST))
        {
            if (usbDevMSC->usbDevBOT.dataLen == 0)
            {
                USBD_MSC_BOT_SendCSW(usbInfo, USBD_BOT_CSW_OK);
            }
            else if (usbDevMSC->usbDevBOT.dataLen > 0)
            {
                USBD_MSC_BOT_SendData(usbInfo, usbDevMSC->usbDevBOT.data, usbDevMSC->usbDevBOT.dataLen);
            }
            else
            {
                USBD_MSC_BOT_Abort(usbInfo);
            }
        }
        else
        {
            return USBD_OK;
        }
    }

    return usbStatus;
}

/*!
 * @brief     Clear feature BOT of USB device MSC
 *
 * @param     usbInfo : usb handler information
 *
 * @param     epNum : endpoint number
 *
 * @retval    USB device operation status
 */
USBD_STA_T USBD_MSC_BOT_ClearFeature(USBD_INFO_T* usbInfo, uint8_t epNum)
{
    USBD_STA_T  usbStatus = USBD_OK;
    USBD_MSC_INFO_T* usbDevMSC = (USBD_MSC_INFO_T*)usbInfo->devClass[usbInfo->classID]->classData;

    if (usbDevMSC == NULL)
    {
        return USBD_FAIL;
    }

    switch (usbDevMSC->usbDevBOT.status)
    {
        /* CBW signature error */
        case USBD_BOT_ERR:
            USBD_EP_StallCallback(usbInfo, usbDevMSC->epInAddr);
            USBD_EP_StallCallback(usbInfo, usbDevMSC->epOutAddr);
            break;

        case USBD_BOT_NORMAL:
            if ((epNum & 0x80) == 0x80)
            {
                usbDevMSC->usbDevBOT.cmdPack.cswStatus = USBD_BOT_CSW_FAIL;
                USBD_MSC_BOT_SendCSW(usbInfo, usbDevMSC->usbDevBOT.cmdPack.cswStatus);
            }
            break;

        default:
            break;
    }

    return usbStatus;
}

/**@} end of group USBD_MSC_Functions */
/**@} end of group USBD_MSC_Class */
/**@} end of group APM32_USB_Library */
