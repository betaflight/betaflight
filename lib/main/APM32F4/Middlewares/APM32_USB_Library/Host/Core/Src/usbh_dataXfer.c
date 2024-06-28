/*!
 * @file        usbh_dataXfer.c
 *
 * @brief       USB host input and output hander function
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
#include "usbh_dataXfer.h"
#include "usbh_core.h"
#include "usbh_stdReq.h"
#include "usbh_channel.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

static USBH_STA_T USBH_CtrlXferIdle(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferSetup(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferSetupWait(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferInData(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferInDataWait(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferOutData(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferOutDataWait(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferInSta(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferInStaWait(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferOutSta(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferOutStaWait(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferError(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferStall(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_CtrlXferOk(USBH_INFO_T* usbInfo);

/**@} end of group USBH_Core_Functions */

/** @defgroup USBH_Core_Structures Structures
  @{
  */

/* USB control transfer state handler function */
USBH_CtrlStateHandler_T USBH_CtrlStateHandler[] =
{
    USBH_CtrlXferIdle,
    USBH_CtrlXferSetup,
    USBH_CtrlXferSetupWait,
    USBH_CtrlXferInData,
    USBH_CtrlXferInDataWait,
    USBH_CtrlXferOutData,
    USBH_CtrlXferOutDataWait,
    USBH_CtrlXferInSta,
    USBH_CtrlXferInStaWait,
    USBH_CtrlXferOutSta,
    USBH_CtrlXferOutStaWait,
    USBH_CtrlXferError,
    USBH_CtrlXferStall,
    USBH_CtrlXferOk,
};

/**@} end of group USBH_Core_Structures*/

/** @defgroup USBH_Core_Functions Functions
  @{
  */

/*!
 * @brief     Control transfer send data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @param     pingStatus : ping status
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                       uint8_t* buffer, uint16_t length, \
                                       uint8_t pingStatus)
{
    USBH_STA_T usbStatus = USBH_OK;

    if ((usbInfo->devInfo.speed != USBH_DEVICE_SPEED_HS) && (pingStatus == ENABLE))
    {
        pingStatus = DISABLE;
    }

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_OUT, \
                           EP_TYPE_CONTROL, \
                           USBH_PID_DATA,   \
                           (uint8_t*)buffer, \
                           length, \
                           pingStatus);

    return usbStatus;
}

/*!
 * @brief     Control transfer receive data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
        uint8_t* buffer, uint16_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_IN, \
                           EP_TYPE_CONTROL, \
                           USBH_PID_DATA,   \
                           (uint8_t*)buffer, \
                           length, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Start control setup transfer
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be sent
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlSetupReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                    USBH_REQ_DATA_T* buffer)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_OUT, \
                           EP_TYPE_CONTROL, \
                           USBH_PID_SETUP,   \
                           (uint8_t*)buffer, \
                           USBH_SETUP_PACKET_SIZE, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Handle control idle state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferIdle(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_OK;

    usbInfo->ctrl.state = USBH_CTRL_IDLE;

    USBH_USR_Debug("USBH_CtrlXferIdle");

    return usbStatus;
}

/*!
 * @brief     Handle control setup transfer
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferSetup(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferSetup");

    /* Sent setup packet */
    USBH_CtrlSetupReq(usbInfo, usbInfo->ctrl.channelOutNum, &usbInfo->ctrl.reqData);

    usbInfo->ctrl.state = USBH_CTRL_SETUP_WAIT;

    return usbStatus;
}

/*!
 * @brief     Handle control setup wait URB status transfer
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferSetupWait(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;
    uint16_t reqLenghTemp;
    uint8_t reqDirTemp;
    //static uint32_t timeout = 0;

    USBH_USR_Debug("USBH_CtrlXferSetupWait");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbInfo->ctrl.channelOutNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            //timeout = 0;
            reqDirTemp   = usbInfo->ctrl.reqData.DATA_FIELD.bmRequestType.REQ_TYPE_B.dir;
            reqLenghTemp = usbInfo->ctrl.reqData.DATA_FIELD.wLength[1] << 8 | \
                           usbInfo->ctrl.reqData.DATA_FIELD.wLength[0];

            /* Goto DATA state */
            if (reqLenghTemp != 0)
            {
                if (reqDirTemp == USB_REQ_DIR_OUT)
                {
                    usbInfo->ctrl.state = USBH_CTRL_DATA_OUT;
                }
                else
                {
                    usbInfo->ctrl.state = USBH_CTRL_DATA_IN;
                }
            }
            else
            {
                if (reqDirTemp == USB_REQ_DIR_OUT)
                {
                    usbInfo->ctrl.state = USBH_CTRL_STA_IN;
                }
                else
                {
                    usbInfo->ctrl.state = USBH_CTRL_STA_OUT;
                }
            }
            break;

        case USB_URB_ERROR:
        case USB_URB_NOREADY:
            //timeout = 0;
            usbInfo->ctrl.state = USBH_CTRL_ERROR;
            break;

        default:
            //timeout++;
            break;
    }

    /*if(timeout >= 500)
    {
        timeout = 0;

        usbInfo->ctrl.state = USBH_CTRL_ERROR;
    }*/

    return usbStatus;
}

/*!
 * @brief     Handle control IN state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferInData(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferInData");

    USBH_CtrlReceiveDataReq(usbInfo, usbInfo->ctrl.channelInNum, \
                            usbInfo->ctrl.buffer, usbInfo->ctrl.length);

    usbInfo->ctrl.state = USBH_CTRL_DATA_IN_WAIT;

    return usbStatus;
}

/*!
 * @brief     Handle control IN wait URB status state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferInDataWait(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;

    USBH_USR_Debug("USBH_CtrlXferInDataWait");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbInfo->ctrl.channelInNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            usbInfo->ctrl.state = USBH_CTRL_STA_OUT;
            break;

        case USB_URB_STALL:
            usbStatus = USBH_ERR_NOT_SUP;
            break;

        case USB_URB_ERROR:
            usbInfo->ctrl.state = USBH_CTRL_ERROR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     Handle control OUT state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferOutData(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferOutData");

    USBH_CtrlSendDataReq(usbInfo, usbInfo->ctrl.channelOutNum, \
                         usbInfo->ctrl.buffer, usbInfo->ctrl.length, \
                         ENABLE);

    usbInfo->ctrl.state = USBH_CTRL_DATA_OUT_WAIT;

    return usbStatus;
}


/*!
 * @brief     Handle control OUT wait URB status state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferOutDataWait(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;

    USBH_USR_Debug("USBH_CtrlXferOutDataWait");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbInfo->ctrl.channelOutNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            usbInfo->ctrl.state = USBH_CTRL_STA_IN;
            break;

        case USB_URB_STALL:
            usbInfo->ctrl.state = USBH_CTRL_STALL;
            usbStatus = USBH_ERR_NOT_SUP;
            break;

        case USB_URB_NOREADY:
            usbInfo->ctrl.state = USBH_CTRL_DATA_OUT;
            break;

        case USB_URB_ERROR:
            usbInfo->ctrl.state = USBH_CTRL_ERROR;
            usbStatus = USBH_FAIL;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     Handle control STATUS IN state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferInSta(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferInSta");

    /* Send 0 byte out packet */
    USBH_CtrlReceiveDataReq(usbInfo, usbInfo->ctrl.channelInNum, \
                            NULL, 0);

    usbInfo->ctrl.state = USBH_CTRL_STA_IN_WAIT;

    return usbStatus;
}

/*!
 * @brief     Handle control STATUS IN wait state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferInStaWait(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;

    USBH_USR_Debug("USBH_CtrlXferInStaWait");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbInfo->ctrl.channelInNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            usbInfo->ctrl.state = USBH_CTRL_OK;
            usbStatus = USBH_OK;
            break;

        case USB_URB_STALL:
            usbStatus = USBH_ERR_NOT_SUP;
            break;

        case USB_URB_ERROR:
            usbInfo->ctrl.state = USBH_CTRL_ERROR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     Handle control STATUS OUT state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferOutSta(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferOutSta");

    /* Send 0 byte out packet */
    USBH_CtrlSendDataReq(usbInfo, usbInfo->ctrl.channelOutNum, \
                         NULL, 0, ENABLE);

    usbInfo->ctrl.state = USBH_CTRL_STA_OUT_WAIT;

    return usbStatus;
}

/*!
 * @brief     Handle control STATUS OUT wait state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferOutStaWait(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;
    uint8_t usbUrbStatus = USB_URB_IDLE;

    USBH_USR_Debug("USBH_CtrlXferOutStaWait");

    /* Read USB URB status */
    usbUrbStatus = USBH_ReadUrbStatusCallback(usbInfo, usbInfo->ctrl.channelOutNum);

    switch (usbUrbStatus)
    {
        case USB_URB_OK:
            usbInfo->ctrl.state = USBH_CTRL_OK;
            usbStatus = USBH_OK;
            break;

        case USB_URB_NOREADY:
            usbInfo->ctrl.state = USBH_CTRL_STA_OUT;
            break;

        case USB_URB_ERROR:
            usbInfo->ctrl.state = USBH_CTRL_ERROR;
            break;

        default:
            break;
    }

    return usbStatus;
}

/*!
 * @brief     Handle control ERROR state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferError(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_BUSY;

    USBH_USR_Debug("USBH_CtrlXferError");

    usbInfo->ctrl.errCnt++;

    if (usbInfo->ctrl.errCnt > 2)
    {
        /* Notify user */
        usbInfo->userCallback(usbInfo, USBH_USER_ERROR);
        usbInfo->ctrl.errCnt = 0;
        USBH_USR_LOG("Control error: Device No Response");

        USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
        USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

        usbInfo->hostState = USBH_HOST_IDLE;
        usbStatus = USBH_FAIL;
//        usbStatus = USBH_ERR_NOT_SUP;
    }
    else
    {
        /* Retry again */
        usbInfo->ctrl.state = USBH_CTRL_SETUP;
        usbInfo->xferState = USBH_XFER_START;
    }

    return usbStatus;
}

/*!
 * @brief     Handle control STALL state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferStall(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_ERR_NOT_SUP;

    USBH_USR_Debug("USBH_CtrlXferStall");

    return usbStatus;
}

/*!
 * @brief     Handle control OK state
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_CtrlXferOk(USBH_INFO_T* usbInfo)
{
    USBH_STA_T usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_CtrlXferOk");

    return usbStatus;
}

/*!
 * @brief     Bulk transfer send data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @param     pingStatus : ping status
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_BulkSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                uint8_t* buffer, uint16_t length, \
                                uint8_t pingStatus)
{
    USBH_STA_T usbStatus = USBH_OK;

    if ((usbInfo->devInfo.speed != USBH_DEVICE_SPEED_HS) && (pingStatus == ENABLE))
    {
        pingStatus = DISABLE;
    }

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_OUT, \
                           EP_TYPE_BULK, \
                           USBH_PID_DATA,   \
                           (uint8_t*)buffer, \
                           length, \
                           pingStatus);

    return usbStatus;
}

/*!
 * @brief     Bulk transfer receive data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_BulkReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                   uint8_t* buffer, uint16_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_IN, \
                           EP_TYPE_BULK, \
                           USBH_PID_DATA,   \
                           buffer, \
                           length, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Interrupt transfer receive data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_IntReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                  uint8_t* buffer, uint8_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_IN, \
                           EP_TYPE_INTERRUPT, \
                           USBH_PID_DATA,   \
                           buffer, \
                           length, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Interrupt transfer send data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_IntSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                               uint8_t* buffer, uint8_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_OUT, \
                           EP_TYPE_INTERRUPT, \
                           USBH_PID_DATA,   \
                           buffer, \
                           length, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Isochronous transfer receive data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_IsoReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                  uint8_t* buffer, uint8_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_IN, \
                           EP_TYPE_ISO, \
                           USBH_PID_DATA,   \
                           buffer, \
                           length, \
                           0);

    return usbStatus;
}

/*!
 * @brief     Isochronous transfer send data request
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @param     buffer : buffer to be receive data
 *
 * @param     length : transfer length
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_IsoSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                               uint8_t* buffer, uint8_t length)
{
    USBH_STA_T usbStatus = USBH_OK;

    /* Callback USB URB submit */
    USBH_UrbSubmitCallback(usbInfo, channelNum, \
                           EP_DIR_OUT, \
                           EP_TYPE_ISO, \
                           USBH_PID_DATA,   \
                           buffer, \
                           length, \
                           0);

    return usbStatus;
}

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */
