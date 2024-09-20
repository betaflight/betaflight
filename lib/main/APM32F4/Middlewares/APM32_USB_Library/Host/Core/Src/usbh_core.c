/*!
 * @file        usbh_core.c
 *
 * @brief       USB host core function
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
#include "usbh_core.h"
#include "usbh_enum.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

static USBH_STA_T USBH_IdleHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_DevAttachedHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_DevDisconnectHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_EnumerationHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ClassReqHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_UserInputHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_SetConfigHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_SetFeatureHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ClassActiveHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_ClassHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_SuspendHandler(USBH_INFO_T* usbInfo);
static USBH_STA_T USBH_AbortHandler(USBH_INFO_T* usbInfo);
static void USBH_ClearDevData(USBH_INFO_T* usbInfo);

/**@} end of group USBH_Core_Functions */

/** @defgroup USBH_Core_Structures Structures
  @{
  */

/* USB host state handler function */
USBH_CoreHandler_T USBH_CoreHandler[] =
{
    USBH_IdleHandler,                   /* !< Array number match with USBH_HOST_IDLE enumerate */
    USBH_DevAttachedHandler,
    USBH_DevDisconnectHandler,          /* !< Array number match with USBH_HOST_DEV_DISCONNECTED enumerate */
    USBH_EnumerationHandler,
    USBH_ClassReqHandler,
    USBH_UserInputHandler,
    USBH_SetConfigHandler,
    USBH_SetFeatureHandler,
    USBH_ClassActiveHandler,
    USBH_ClassHandler,
    USBH_SuspendHandler,
    USBH_AbortHandler,
};

/**@} end of group USBH_Core_Structures*/

/** @defgroup USBH_Core_Functions Functions
  @{
  */

/*!
 * @brief     Host idle state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_IdleHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_IdleHandler");

    if (usbInfo->devInfo.connectedStatus == ENABLE)
    {
        USBH_USR_Debug("USB device insert");

        /* Wait for 200 ms */
        USBH_Delay(200);

        /* Reset usb host port */
        USBH_ResetCallback(usbInfo);

        usbInfo->devInfo.address = USBH_DEVICE_DEFAULT_ADDRESS;
        /* Next USB host state */
        usbInfo->hostState = USBH_HOST_DEVICE_ATTACHED;

        usbInfo->timeout = 0;
    }

    return usbStatus;
}

/*!
 * @brief     Host Attached state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_DevAttachedHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_DevAttachedHandler");

    if (usbInfo->devInfo.portEnable == ENABLE)
    {
        USBH_USR_LOG("USB Device Reset Completed");
        usbInfo->devInfo.rstCnt = 0;

        /* Notify user device is connected */
        usbInfo->userCallback(usbInfo, USBH_USER_CONNECTION);

        USBH_Delay(120);

        /* Get speed */
        usbInfo->devInfo.speed = USBH_ReadSpeedCallback(usbInfo);

        /* Notify user device speed is detected */
        usbInfo->userCallback(usbInfo, USBH_USER_DETECTED_SPEED);
        
        usbInfo->ctrl.channelOutNum = USBH_CH_AllocChannel(usbInfo, 0x00);
        usbInfo->ctrl.channelInNum = USBH_CH_AllocChannel(usbInfo, 0x80);

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

        /* Next USB host state */
        usbInfo->hostState = USBH_HOST_ENUMERATION;
    }
    else
    {
        if (usbInfo->timeout > USBH_DEVICE_RESET_TIMEOUT)
        {
            usbInfo->devInfo.rstCnt++;
            if (usbInfo->devInfo.rstCnt > 5)
            {
                USBH_USR_LOG("USB Reset Failed. Please unplug the device and reset system");
                /* Next USB host state */
                usbInfo->hostState = USBH_HOST_ABORT;
            }
            else
            {
                /* Next USB host state */
                usbInfo->hostState = USBH_HOST_IDLE;
            }
        }
        else
        {
            usbInfo->timeout++;
            USBH_Delay(10);
        }
    }

    return usbStatus;
}

/*!
 * @brief     Host device disconnect state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_DevDisconnectHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_DevDisconnectHandler");

    usbInfo->devInfo.disconnectedStatus = DISABLE;

    /* De-init data and state machine */
    USBH_ClearDevData(usbInfo);
    USBH_CH_Clear(usbInfo);

    usbInfo->hostState                   = USBH_HOST_IDLE;
    usbInfo->hostEnumState               = USBH_ENUM_IDLE;
    usbInfo->xferState                   = USBH_XFER_START;
    /* De-init control state */
    usbInfo->ctrl.state                  = USBH_CTRL_SETUP;
    usbInfo->ctrl.channelSize            = USBH_EP0_PACKET_SIZE;
    usbInfo->ctrl.errCnt                 = 0;
    usbInfo->timer                       = 0;

    /* Clear device all connection status */
    usbInfo->devInfo.address             = USBH_DEVICE_DEFAULT_ADDRESS;
    usbInfo->devInfo.speed               = USBH_DEVICE_SPEED_FS;
    usbInfo->devInfo.rstCnt              = 0;
    usbInfo->devInfo.enumCnt             = 0;

    /* Re-init usb host for new enumeration */
    if (usbInfo->activeClass != NULL)
    {
        usbInfo->activeClass->classDeInitHandler(usbInfo);
        usbInfo->activeClass = NULL;
    }

    /* Notify user */
    usbInfo->userCallback(usbInfo, USBH_USER_DISCONNECTION);

    USBH_USR_Debug("USB device disconnected");

    if (usbInfo->devInfo.reEnumStatus == ENABLE)
    {
        usbInfo->devInfo.reEnumStatus = DISABLE;

        USBH_StartCallback(usbInfo);

        /* Add re-enable VBUS interface */
    }
    else
    {
        USBH_StartCallback(usbInfo);
    }

    return usbStatus;
}

/*!
 * @brief     Host enum state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_EnumerationHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t usbEnumStatus;

    usbEnumStatus = USBH_EnumHandler[usbInfo->hostEnumState](usbInfo);

    if (usbEnumStatus == USBH_OK)
    {
        USBH_USR_Debug("USB device enumeration OK");

        usbInfo->userCallback(usbInfo, USBH_USER_ENUMERATION);

        if (usbInfo->devInfo.desc.device.bNumConfigurations != 1)
        {
            usbInfo->hostState = USBH_HOST_USER_INPUT;
        }
        else
        {
            USBH_USR_LOG("USB device has only one configuration");
            usbInfo->hostState = USBH_HOST_SET_CONFIGURATION;
        }
    }

    return usbStatus;
}

/*!
 * @brief     Through user input to go to class
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_UserInputHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_UserInputHandler");

    usbInfo->hostState = USBH_HOST_SET_CONFIGURATION;

    return usbStatus;
}

/*!
 * @brief     Host set configuration handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_SetConfigHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t setCfgStatus;

    USBH_USR_Debug("USBH_SetConfigHandler");

    setCfgStatus = USBH_SetConfiguration(usbInfo, \
                                         (uint16_t)usbInfo->devInfo.desc.configuration.bConfigurationValue);

    if (setCfgStatus == USBH_OK)
    {
        usbInfo->hostState = USBH_HOST_SET_FEATURE;
        USBH_USR_LOG("Set to default configuration");
    }

    return usbStatus;
}

/*!
 * @brief     Host set feature handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_SetFeatureHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t setFeatureStatus;
    uint8_t remoteWakeupStatus;

    USBH_USR_Debug("USBH_SetFeatureHandler");

    remoteWakeupStatus = usbInfo->devInfo.desc.configuration.bmAttributes & (1 << 5);

    if (remoteWakeupStatus)
    {
        setFeatureStatus = USBH_SetFeature(usbInfo, USBH_FEATURE_REMOTE_WAKEUP);

        switch (setFeatureStatus)
        {
            case USBH_OK:
                USBH_USR_LOG("USB device set to remote wakeup");
                usbInfo->hostState = USBH_HOST_CLASS_ACTIVE;
                break;

            case USBH_ERR_NOT_SUP:
                USBH_USR_LOG("USB device is not support remote wakeup");
                usbInfo->hostState = USBH_HOST_CLASS_ACTIVE;
                break;

            default:
                break;
        }
    }
    else
    {
        usbInfo->hostState = USBH_HOST_CLASS_ACTIVE;
    }

    return usbStatus;
}

/*!
 * @brief     Host class active handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ClassActiveHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t classStatus;
    uint8_t classInterface;
    uint8_t classCode;
    uint32_t i;

    USBH_USR_Debug("USBH_ClassActiveHandler");

    /* multi class support */
    if (usbInfo->classNum != 0)
    {
        classInterface = USBH_ReadInterfaceClass(usbInfo, 0);

        usbInfo->activeClass = NULL;

        for (i = 0; i < USBH_SUP_CLASS_MAX_NUM; i++)
        {
            classCode = usbInfo->hostClass[i]->classCode;

            if (classCode == classInterface)
            {
                usbInfo->activeClass = usbInfo->hostClass[i];
                break;
            }
        }

        if (usbInfo->activeClass != NULL)
        {
            classStatus = usbInfo->activeClass->classInitHandler(usbInfo);

            switch (classStatus)
            {
                case USBH_OK:
                    /* Next state */
                    usbInfo->hostState = USBH_HOST_CLASS_REQ;
                    break;

                default:
                    usbInfo->hostState = USBH_HOST_ABORT;
                    USBH_USR_LOG("Device not support to %s", usbInfo->activeClass->className);
                    usbInfo->userCallback(usbInfo,USBH_USER_NOT_SUPPORT);
                    break;
            }
        }
        else
        {
            usbInfo->hostState = USBH_HOST_ABORT;
            USBH_USR_LOG("USB host class is not register for this device");
            usbInfo->userCallback(usbInfo,USBH_USER_NOT_SUPPORT);
        }
    }
    else
    {
        USBH_USR_LOG("USB host class is not register for this device");
    }

    return usbStatus;
}

/*!
 * @brief     Host class request state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ClassReqHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t classStatus;

    if (usbInfo->activeClass != NULL)
    {
        classStatus = usbInfo->activeClass->classReqHandler(usbInfo);

        switch (classStatus)
        {
            case USBH_OK:
                usbInfo->hostState = USBH_HOST_CLASS;
                break;

            case USBH_FAIL:
                usbInfo->hostState = USBH_HOST_ABORT;
                USBH_USR_LOG("Device is not response. Please reseat device and reset system");
                break;

            default:
                break;
        }

    }
    else
    {
        usbInfo->hostState = USBH_HOST_ABORT;
        USBH_USR_LOG("USB host class driver is invalid");
    }

    return usbStatus;
}

/*!
 * @brief     Host class handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_ClassHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    if (usbInfo->activeClass != NULL)
    {
        usbInfo->activeClass->classCoreHandler(usbInfo);
    }

    return usbStatus;
}

/*!
 * @brief     Host suspend state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_SuspendHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_SuspendHandler");

    return usbStatus;
}

/*!
 * @brief     Host abort state handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
static USBH_STA_T USBH_AbortHandler(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_Debug("USBH_AbortHandler");

    return usbStatus;
}

/*!
 * @brief     Clear all of the device data buffer.
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
static void USBH_ClearDevData(USBH_INFO_T* usbInfo)
{
    uint32_t i = 0;

    for (i = 0; i < USBH_DATA_BUF_MAX_NUM; i++)
    {
        usbInfo->devInfo.data[i] = 0;
    }
}

/*!
 * @brief     USB host connection event
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
USBH_STA_T USBH_Connect(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbInfo->devInfo.connectedStatus    = ENABLE;
    usbInfo->devInfo.reEnumStatus       = DISABLE;
    usbInfo->devInfo.disconnectedStatus = DISABLE;

    return usbStatus;
}

/*!
 * @brief     USB host disconnection event
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
USBH_STA_T USBH_Disconnect(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbInfo->devInfo.connectedStatus    = DISABLE;
    usbInfo->devInfo.portEnable         = DISABLE;
    usbInfo->devInfo.disconnectedStatus = ENABLE;

    /* USB OTG stop host callback */
    USBH_StopHostCallback(usbInfo);

    /* Free control channel */
    USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelInNum);
    USBH_CH_FreeChannel(usbInfo, usbInfo->ctrl.channelOutNum);

    return usbStatus;
}

/*!
 * @brief     USB host port enable event
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
USBH_STA_T USBH_PortEnable(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbInfo->devInfo.portEnable = ENABLE;

    return usbStatus;
}

/*!
 * @brief     USB host port disable event
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
USBH_STA_T USBH_PortDisable(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    usbInfo->devInfo.portEnable = DISABLE;

    return usbStatus;
}

/*!
 * @brief     USB host SOF handler
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
static void USBH_HandleSOF(USBH_INFO_T* usbInfo)
{
    if ((usbInfo->hostState == USBH_HOST_CLASS) && (usbInfo->activeClass != NULL))
    {
        usbInfo->activeClass->classSofHandler(usbInfo);
    }
}

/*!
 * @brief     USB host configure timer
 *
 * @param     usbInfo : usb handler information
 *
 * @param     tick : time tick
 *
 * @retval    None
 */
void USBH_ConfigTimer(USBH_INFO_T* usbInfo, uint32_t tick)
{
    usbInfo->timer = tick;
}

/*!
 * @brief     USB host increase timer
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
void USBH_IncTimer(USBH_INFO_T* usbInfo)
{
    usbInfo->timer++;
    USBH_HandleSOF(usbInfo);
}

/*!
 * @brief     USB host core init
 *
 * @param     usbInfo : usb handler information
 *
 * @param     usbHostSpeed
 *
 * @param     usbHostClass
 *
 * @param     userCallback
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_Init(USBH_INFO_T* usbInfo, USBH_SPEED_T usbHostSpeed, USBH_CLASS_T* usbHostClass,
                     void (*userCallbackFunc)(struct _USBH_INFO_T*, uint8_t))
{
    USBH_STA_T  usbStatus = USBH_OK;

    /* Set USB host speed */
    usbInfo->hostSpeed = usbHostSpeed;

    /* De-init data and state machine */
    USBH_ClearDevData(usbInfo);
    USBH_CH_Clear(usbInfo);

    usbInfo->hostState                      = USBH_HOST_IDLE;
    usbInfo->hostEnumState                  = USBH_ENUM_IDLE;
    usbInfo->xferState                      = USBH_XFER_START;
    /* De-init control state */
    usbInfo->ctrl.state                     = USBH_CTRL_SETUP;
    usbInfo->ctrl.channelSize               = USBH_EP0_PACKET_SIZE;
    usbInfo->ctrl.errCnt                    = 0;

    /* Clear device all connection status */
    usbInfo->devInfo.address                = USBH_DEVICE_DEFAULT_ADDRESS;
    usbInfo->devInfo.speed                  = USBH_DEVICE_SPEED_FS;
    usbInfo->devInfo.portEnable             = DISABLE;
    usbInfo->devInfo.disconnectedStatus     = DISABLE;
    usbInfo->devInfo.connectedStatus        = DISABLE;
    usbInfo->devInfo.reEnumStatus           = DISABLE;
    usbInfo->devInfo.rstCnt                 = 0;
    usbInfo->devInfo.enumCnt                = 0;
    usbInfo->timer                          = 0;

    /* Register user application */
    usbInfo->userCallback = userCallbackFunc;

    /* Register class function */
    if (usbInfo->classNum < USBH_SUP_CLASS_MAX_NUM)
    {
        usbInfo->hostClass[usbInfo->classNum++] = usbHostClass;
    }

    /* Init USB hardware */
    USBH_HardwareInit(usbInfo);

    return usbStatus;
}

/*!
 * @brief     USB device core de-init
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_DeInit(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    
    /* De-init data and state machine */
    USBH_ClearDevData(usbInfo);
    USBH_CH_Clear(usbInfo);

    usbInfo->hostState                      = USBH_HOST_IDLE;
    usbInfo->hostEnumState                  = USBH_ENUM_IDLE;
    usbInfo->xferState                      = USBH_XFER_START;
    /* De-init control state */
    usbInfo->ctrl.state                     = USBH_CTRL_SETUP;
    usbInfo->ctrl.channelSize               = USBH_EP0_PACKET_SIZE;
    usbInfo->ctrl.errCnt                    = 0;

    /* Clear device all connection status */
    usbInfo->devInfo.address                = USBH_DEVICE_DEFAULT_ADDRESS;
    usbInfo->devInfo.speed                  = USBH_DEVICE_SPEED_FS;
    usbInfo->devInfo.portEnable             = DISABLE;
    usbInfo->devInfo.disconnectedStatus     = DISABLE;
    usbInfo->devInfo.connectedStatus        = DISABLE;
    usbInfo->devInfo.reEnumStatus           = DISABLE;
    usbInfo->devInfo.rstCnt                 = 0;
    usbInfo->devInfo.enumCnt                = 0;
    usbInfo->timer                          = 0;

    for(uint8_t i = 0; i < USBH_SUP_CLASS_MAX_NUM; i++)
    {
        if(usbInfo->hostClass[0] != NULL && usbInfo->activeClass != NULL && \
            usbInfo->activeClass->classData != NULL )
        {
            usbInfo->hostClass[0]->classDeInitHandler(usbInfo);
        }
    }

    if(usbInfo->dataPoint != NULL)
    {
        USBH_StopHostCallback(usbInfo);
    }
    
    USBH_HardwareReset(usbInfo);
    
    return usbStatus;
}

/*!
 * @brief     Host register class
 *
 * @param     usbInfo : usb handler information
 * 
 * @param     usbHostClass : host class
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_RegisterClass(USBH_INFO_T* usbInfo, USBH_CLASS_T* usbHostClass)
{
    USBH_STA_T  usbStatus = USBH_OK;
    
    if (usbHostClass != NULL)
    {
        /* Register class function */
        if (usbInfo->classNum < USBH_SUP_CLASS_MAX_NUM)
        {
            usbInfo->hostClass[usbInfo->classNum++] = usbHostClass;

            usbStatus = USBH_OK;
        }
        else
        {
            usbStatus = USBH_FAIL;
        }
    }
    else
    {
        usbStatus = USBH_FAIL;
    }
    
    return usbStatus;
}

/*!
 * @brief     Host state machine polling Process
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb host status
 */
USBH_STA_T USBH_PollingProcess(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    /* Device disconnect */
    if (usbInfo->devInfo.disconnectedStatus == ENABLE)
    {
        usbInfo->hostState = USBH_HOST_DEVICE_DISCONNECTED;
    }

    usbStatus = USBH_CoreHandler[usbInfo->hostState](usbInfo);

    return usbStatus;
}

/*!
 * @brief     USB stop host event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBH_StopHostCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief     USB host start event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBH_StartCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief     USB host stop event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBH_StopCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief     USB host reset event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    None
 */
__weak void USBH_ResetCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
}

/*!
 * @brief     USB host read speed event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    speed
 */
__weak uint8_t USBH_ReadSpeedCallback(USBH_INFO_T* usbInfo)
{
    /* callback interface */
    return 0;
}

/*!
 * @brief     USB host config the channel to transfer event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel numer
 *
 * @param     endPointNum : end point number
 *
 * @param     devAddr : USB device address
 *
 * @param     devSpeed : USB device speed
 *
 * @param     epType : end point type
 *
 * @param     packetMaxSize : max size of packet
 *
 * @retval    None
 */
__weak void USBH_OpenChannelCallback(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                     uint8_t endPointNum, uint8_t devAddr, \
                                     uint8_t devSpeed, uint8_t epType, uint16_t packetMaxSize)
{
    /* callback interface */
}

/*!
 * @brief     USB submit URB event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     chNum : channel number
 *
 * @param     dir : channel direction
 *
 * @param     epType : endpoint type
 *
 * @param     tokenType : tokenType
 *
 * @param     buffer : URB data
 *
 * @param     length : length of URB data
 *
 * @param     pingStatus : ping status
 *
 * @retval    None
 */
__weak void USBH_UrbSubmitCallback(USBH_INFO_T* usbInfo, uint8_t chNum, uint8_t dir, \
                                   uint8_t epType, uint8_t tokenType, uint8_t* buffer, \
                                   uint16_t length, uint8_t pingStatus)
{
    /* callback interface */
}

/*!
 * @brief     USB host read URB status event callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum : channel number
 *
 * @retval    URB status
 */
__weak uint8_t USBH_ReadUrbStatusCallback(USBH_INFO_T* usbInfo, uint8_t channelNum)
{
    /* callback interface */
    return 0;
}

/*!
 * @brief     USB host configure data PID callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum: channel number
 *
 * @param     dataPid: data PID
 *
 * @retval    None
 */
__weak void USBH_ConfigDataPidCallback(USBH_INFO_T* usbInfo, uint8_t channelNum, uint8_t dataPid)
{
    /* callback interface */
}

/*!
 * @brief     USB host close channel callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum: channel number
 *
 * @retval    None
 */
__weak void USBH_CloseChannelCallback(USBH_INFO_T* usbInfo, uint8_t channelNum)
{
    /* callback interface */
}

/*!
 * @brief     USB host read size of last xfer callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum: channel number
 *
 * @retval    xfer size
 */
__weak uint32_t USBH_ReadLastXferSizeCallback(USBH_INFO_T* usbInfo, uint8_t channelNum)
{
    /* callback interface */
    return 0;
}

/*!
 * @brief     USB host configure current toggle of channel callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum: channel number
 *
 * @param     toggle: toggle
 *
 * @retval    xfer size
 */
__weak void USBH_ConfigToggleCallback(USBH_INFO_T* usbInfo, uint8_t channelNum, uint8_t toggle)
{
    /* callback interface */
}

/*!
 * @brief     USB host read current toggle of channel callback function
 *
 * @param     usbInfo : usb handler information
 *
 * @param     channelNum: channel number
 *
 * @retval    xfer size
 */
__weak uint8_t USBH_ReadToggleCallback(USBH_INFO_T* usbInfo, uint8_t channelNum)
{
    /* callback interface */
    return 0;
}

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */
