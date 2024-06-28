/*!
 * @file        usbh_msc_scsi.c
 *
 * @brief       USB host MSC scsi
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
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"
#include "usbh_msc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_MSC_Class
  @{
  */

/** @defgroup USBH_MSC_Functions Functions
  @{
  */

/*!
 * @brief       USB host MSC SCSI INQUIRY handler
 *
 * @param       usbInfo
 *
 * @param       lun: logical unit number
 *
 * @param       inquiry: inquiry response
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_Inquiry(USBH_INFO_T* usbInfo, uint8_t lun, \
                                 USBH_SCSI_INQUIRY_REQ_T* inquiry)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = LEN_XFER_INQUIRY;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_IN;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_INQUIRY;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[1] = (lun << 5);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[2] = 0;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[3] = 0;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[4] = 36;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[5] = 0;

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;

            /* Init buffer point */
            usbHostMSC->usbHostBOT.buffer = (uint8_t*)(void*)usbHostMSC->usbHostBOT.data;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);

            switch (usbStatus)
            {
                case USBH_OK:
                    memset(inquiry, 0, sizeof(USBH_SCSI_INQUIRY_REQ_T));

                    inquiry->devType = usbHostMSC->usbHostBOT.buffer[0] & 0x1F;
                    inquiry->peripheral = usbHostMSC->usbHostBOT.buffer[0] >> 5;

                    if (((uint32_t) usbHostMSC->usbHostBOT.buffer[1] & 0x80) == 0x80)
                    {
                        inquiry->media = 1;
                    }
                    else
                    {
                        inquiry->media = 0;
                    }

                    memcpy(inquiry->revID, &usbHostMSC->usbHostBOT.buffer[32], 4);
                    memcpy(inquiry->productID, &usbHostMSC->usbHostBOT.buffer[16], 16);
                    memcpy(inquiry->vendorID, &usbHostMSC->usbHostBOT.buffer[8], 8);
                    break;

                default:
                    break;
            }
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SCSI TEST UNIT READY handler
 *
 * @param       usbInfo
 *
 * @param       lun: logical unit number
 *
 * @param       inquiry: inquiry response
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_TestUnitReady(USBH_INFO_T* usbInfo, uint8_t lun)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:
            usbStatus = USBH_BUSY;

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = LEN_XFER_TEST_UNIT_READY;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_OUT;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_TEST_UNIT_READY;

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SCSI READ CAPACITY handler
 *
 * @param       usbInfo
 *
 * @param       lun: logical unit number
 *
 * @param       capacity: capacity response
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_ReadCapacity(USBH_INFO_T* usbInfo, uint8_t lun, \
                                      USBH_SCSI_READ_CAPACITY_REQ_T* capacity)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = LEN_XFER_READ_CAPACITY;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_IN;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_READ_CAPACITY;

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;

            /* Init buffer point */
            usbHostMSC->usbHostBOT.buffer = (uint8_t*)(void*)usbHostMSC->usbHostBOT.data;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);

            switch (usbStatus)
            {
                case USBH_OK:
                    capacity->blockNum = usbHostMSC->usbHostBOT.buffer[3] | (usbHostMSC->usbHostBOT.buffer[2] << 8) | \
                                         (usbHostMSC->usbHostBOT.buffer[1] << 16) | (usbHostMSC->usbHostBOT.buffer[0] << 24);

                    capacity->blockSize = usbHostMSC->usbHostBOT.buffer[7] | (usbHostMSC->usbHostBOT.buffer[6] << 8) | \
                                          (usbHostMSC->usbHostBOT.buffer[5] << 16) | (usbHostMSC->usbHostBOT.buffer[4] << 24);
                    break;

                default:
                    break;
            }
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SCSI request sense handler
 *
 * @param       usbInfo
 *
 * @param       lun: logical unit number
 *
 * @param       sense: sense response
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_RequestSense(USBH_INFO_T* usbInfo, uint8_t lun, \
                                      USBH_SCSI_SENSE_REQ_T* sense)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = LEN_XFER_REQUEST_SENSE;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_IN;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_REQUEST_SENSE;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[1] = (lun << 5);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[2] = 0;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[3] = 0;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[4] = LEN_XFER_REQUEST_SENSE;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[5] = 0;

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;

            /* Init buffer point */
            usbHostMSC->usbHostBOT.buffer = (uint8_t*)(void*)usbHostMSC->usbHostBOT.data;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);

            switch (usbStatus)
            {
                case USBH_OK:
                    sense->key = usbHostMSC->usbHostBOT.buffer[2] & 0x0F;
                    sense->asc = usbHostMSC->usbHostBOT.buffer[12];
                    sense->ascq = usbHostMSC->usbHostBOT.buffer[13];
                    break;

                default:
                    break;
            }
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SCSI read10 handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @param       address: sector address
 *
 * @param       buffer: buffer point to data
 *
 * @param       cnt: count number of data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_Read(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                              uint8_t* buffer, uint16_t cnt)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = cnt * usbHostMSC->storage[0].capacity.blockSize;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_IN;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_READ_10;

            /* logical block address*/
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[2] = (((uint8_t*)&address)[3]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[3] = (((uint8_t*)&address)[2]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[4] = (((uint8_t*)&address)[1]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[5] = (((uint8_t*)&address)[0]);

            /* Transfer length */
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[7] = (((uint8_t*)&cnt)[1]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[8] = (((uint8_t*)&cnt)[0]);

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;

            /* Init buffer point */
            usbHostMSC->usbHostBOT.buffer = buffer;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);
            break;
    }

    return usbStatus;
}

/*!
 * @brief       USB host MSC SCSI write10 handler
 *
 * @param       usbInfo: usb host information
 *
 * @param       lun: logical unit number
 *
 * @param       address: sector address
 *
 * @param       buffer: buffer point to data
 *
 * @param       cnt: count number of data
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_MSC_SCSI_Write(USBH_INFO_T* usbInfo, uint8_t lun, uint32_t address, \
                               uint8_t* buffer, uint16_t cnt)
{
    USBH_STA_T  usbStatus = USBH_BUSY;
    USBH_MSC_INFO_T* usbHostMSC = (USBH_MSC_INFO_T*)usbInfo->activeClass->classData;

    switch (usbHostMSC->usbHostBOT.xferState)
    {
        case USBH_BOT_XFER_START:

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.dDataXferLen = cnt * usbHostMSC->storage[0].capacity.blockSize;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bmFlags.CBW_FLAG_B.dir = USBH_REQ_DIR_OUT;
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.bCBLen = USBH_LEN_CBW;

            memset(usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB, 0, USBH_BOT_CBW_CB_LEN);

            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[0] = USBH_SCSI_CMD_WRITE10;

            /* logical block address*/
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[2] = (((uint8_t*)&address)[3]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[3] = (((uint8_t*)&address)[2]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[4] = (((uint8_t*)&address)[1]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[5] = (((uint8_t*)&address)[0]);

            /* Transfer length */
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[7] = (((uint8_t*)&cnt)[1]);
            usbHostMSC->usbHostBOT.cmdPack.CBW.DATA_FIELD.CB[8] = (((uint8_t*)&cnt)[0]);

            usbHostMSC->usbHostBOT.state = USBH_BOT_SEND_CBW;
            usbHostMSC->usbHostBOT.xferState = USBH_BOT_XFER_WAITING;

            /* Init buffer point */
            usbHostMSC->usbHostBOT.buffer = buffer;
            break;

        case USBH_BOT_XFER_WAITING:
            usbStatus = USBH_MSC_BOT_Handler[usbHostMSC->usbHostBOT.state](usbInfo, lun);
            break;
    }

    return usbStatus;
}

/**@} end of group USBH_MSC_Functions */
/**@} end of group USBH_MSC_Class */
/**@} end of group APM32_USB_Library */
