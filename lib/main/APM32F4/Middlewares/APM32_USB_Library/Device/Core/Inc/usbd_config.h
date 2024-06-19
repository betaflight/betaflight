/*!
 * @file        usbd_config.h
 *
 * @brief       usb device config header file
 *
 * @version     V1.0.1
 *
 * @date        2023-03-27
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

/* Define to prevent recursive inclusion */
#ifndef _USBD_CONFIG_H_
#define _USBD_CONFIG_H_

/* Includes */
#include "usbd_board.h"
//#include <stdio.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Macros Macros
  @{
*/

/*!< [31:16] APM32 USB Device Library main version V1.1.3*/
#define __APM32_USB_DEVICE_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __APM32_USB_DEVICE_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __APM32_USB_DEVICE_VERSION_SUB2   (0x03) /*!< [15:8]  sub2 version */
#define __APM32_USB_DEVICE_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __APM32_USB_DEVICE_VERSION        ((__APM32_USB_DEVICE_VERSION_MAIN << 24)\
                                          |(__APM32_USB_DEVICE_VERSION_SUB1 << 16)\
                                          |(__APM32_USB_DEVICE_VERSION_SUB2 << 8 )\
                                          |(__APM32_USB_DEVICE_VERSION_RC))

#define USBD_DEVICE_DEFAULT_ADDRESS         0
#define USBD_EP0_PACKET_MAX_SIZE            64

/**@} end of group USBD_Core_Macros*/

/** @defgroup USBD_Core_Enumerates Enumerates
  @{
  */

/**
 * @brief   USB device operation status
 */
typedef enum
{
    USBD_OK,
    USBD_BUSY,
    USBD_FAIL,
} USBD_STA_T;

/**
 * @brief   USB device speed type
 */
typedef enum
{
    USBD_SPEED_FS,
    USBD_SPEED_HS,
} USBD_SPEED_T;

/**
 * @brief   USB device speed
 */
typedef enum
{
    USBD_DEVICE_SPEED_HS,
    USBD_DEVICE_SPEED_FS,
    USBD_DEVICE_SPEED_LS
} USBD_DEVICE_SPEED_T;

/**
 * @brief   USB device state
 */
typedef enum
{
    USBD_DEV_IDLE,
    USBD_DEV_DEFAULT,
    USBD_DEV_ADDRESS,
    USBD_DEV_CONFIGURE,
    USBD_DEV_SUSPEND,
} USBD_DEV_STA_T;

/**
 * @brief   USB device EP0 state
 */
typedef enum
{
    USBD_DEV_EP0_IDLE,
    USBD_DEV_EP0_SETUP,
    USBD_DEV_EP0_DATA_IN,
    USBD_DEV_EP0_DATA_OUT,
    USBD_DEV_EP0_STATUS_IN,
    USBD_DEV_EP0_STATUS_OUT,
    USBD_DEV_EP0_STALL,
} USBD_DEV_EP0_STA_T;

/**
 * @brief   USB device request type
 */
typedef enum
{
    USBD_REQ_TYPE_STANDARD = 0,
    USBD_REQ_TYPE_CLASS,
    USBD_REQ_TYPE_VENDOR,
    USBD_REQ_TYPE_RESERVED
} USBD_DEV_REQ_TYPE_T;

/**
 * @brief   USB device feature request type
 */
typedef enum
{
    USBD_FEATURE_SELECTOR_ENDPOINT_HALT,
    USBD_FEATURE_REMOTE_WAKEUP,
    USBD_FEATURE_TEST_MODE,
} USBD_REQ_FEATURE_T;

/**
 * @brief   USB device status configuration type
 */
typedef enum
{
    USBD_CFG_NONE,
    USBD_CFG_SELF_POWER,
    USBD_CFG_REMOTE_WAKEUP,
} USBD_REQ_CFG_T;

/**
 * @brief   USB standard device standard requests type
 */
typedef enum
{
    USBD_STD_GET_STATUS          = 0,
    USBD_STD_CLEAR_FEATURE       = 1,
    USBD_STD_RESERVED1           = 2,
    USBD_STD_SET_FEATURE         = 3,
    USBD_STD_RESERVED2           = 4,
    USBD_STD_SET_ADDRESS         = 5,
    USBD_STD_GET_DESCRIPTOR      = 6,
    USBD_STD_SET_DESCRIPTOR      = 7,
    USBD_STD_GET_CONFIGURATION   = 8,
    USBD_STD_SET_CONFIGURATION   = 9,
    USBD_STD_GET_INTERFACE       = 10,
    USBD_STD_SET_INTERFACE       = 11,
    USBD_STD_SYNCH_FRAME         = 12,
    USBD_STD_CNT,                       /* <! Request counter */
} USBD_STD_REQ_TYPE_T;

/**
 * @brief   USB device vendor requests type
 */
typedef enum
{
    USBD_VEN_REQ_MS_CODE        = 0xA0,
} USBD_VENDOR_REQ_TYPE_T;

/**
 * @brief   USB device winusb descriptor type
 */
typedef enum
{
    USBD_WINUSB_DESC_FEATURE    = 0x04,
    USBD_WINUSB_DESC_PROPERTY   = 0x05,
} USBD_WINUSB_DESC_TYPE_T;

/**
 * @brief   USB descriptor types
 */
typedef enum
{
    USBD_DESC_DEVICE             = 0x01,
    USBD_DESC_CONFIGURATION      = 0x02,
    USBD_DESC_STRING             = 0x03,
    USBD_DESC_INTERFACE          = 0x04,
    USBD_DESC_ENDPOINT           = 0x05,
    USBD_DESC_DEVICE_QUALIFIER   = 0x06,
    USBD_DESC_OTHER_SPEED        = 0x07,
    USBD_DESC_INTERFACE_POWER    = 0x08,
    USBD_DESC_IAD                = 0x0B,
    USBD_DESC_BOS                = 0x0F,
    USBD_DESC_HID                = 0x21,
    USBD_DESC_HID_REPORT         = 0x22,
    USBD_DESC_HID_PHY            = 0x23,
} USBD_DESC_TYPE_T;

/**
 * @brief   USB string descriptor types
 */
typedef enum
{
    USBD_DESC_STR_LANGID,
    USBD_DESC_STR_MFC,
    USBD_DESC_STR_PRODUCT,
    USBD_DESC_STR_SERIAL,
    USBD_DESC_STR_CONFIG,
    USBD_DESC_STR_INTERFACE,
    USBD_DESC_STR_WINUSB_OS     = 0xEE,
} USBD_DESC_STR_T;

/**
 * @brief   USB device request recipient
 */
typedef enum
{
    USBD_RECIPIENT_DEVICE = 0,
    USBD_RECIPIENT_INTERFACE,
    USBD_RECIPIENT_ENDPOINT,
    USBD_RECIPIENT_OTHER
} USBD_DEV_RECIPIENT_T;

/**@} end of group USBD_Core_Enumerates*/

/** @defgroup USBD_Core_Structures Structures
  @{
  */

/**
 * @brief   USB device endpoint information
 */
typedef struct
{
    uint32_t length;
    uint32_t remainLen;
    uint32_t status;
    uint32_t mp;
    uint16_t useStatus;
    uint16_t interval;
} USBD_EP_INFO_T;

/**
 * @brief   USB device descriptor information
 */
typedef struct
{
    uint8_t* desc;
    uint8_t size;
} USBD_DESC_INFO_T;

struct _USBD_INFO_T;

/* Descriptor callback function type define */
typedef USBD_DESC_INFO_T(*USBD_DescCallback_T)(uint8_t usbSpeed);

/**
 * @brief   USB device descriptor structure
 */
typedef struct
{
    const char*             descName;
    USBD_DescCallback_T     deviceDescHandler;
    USBD_DescCallback_T     configDescHandler;
    USBD_DescCallback_T     configStrDescHandler;
    USBD_DescCallback_T     interfaceStrDescHandler;
    USBD_DescCallback_T     langIdStrDescHandler;
    USBD_DescCallback_T     manufacturerStrDescHandler;
    USBD_DescCallback_T     productStrDescHandler;
    USBD_DescCallback_T     serialStrDescHandler;
#if USBD_SUP_LPM
    USBD_DescCallback_T     bosDescHandler;
#endif
    USBD_DescCallback_T     winUsbOsStrDescHandler;
    USBD_DescCallback_T     otherSpeedConfigDescHandler;
    USBD_DescCallback_T     devQualifierDescHandler;
} USBD_DESC_T;

/**
 * @brief   USB request type
 */
typedef union
{
    uint8_t REQ_TYPE;

    struct
    {
        uint8_t recipient       : 5;
        uint8_t type            : 2;
        uint8_t dir             : 1;
    } REQ_TYPE_B;
} USBD_REQ_TYPE_T;

/**
 * @brief   USB device SETUP request
 */
typedef struct
{
    union
    {
        uint8_t REQ_DATA[8];

        struct
        {
            USBD_REQ_TYPE_T     bmRequest;
            uint8_t             bRequest;
            uint8_t             wValue[2];
            uint8_t             wIndex[2];
            uint8_t             wLength[2];
        } DATA_FIELD;
    };
} USBD_REQ_SETUP_T;

/* Standard device request callback function type define */
typedef USBD_STA_T(*USBD_StdDevReqCallback_T)(struct _USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);

/**
 * @brief   USB device class handler
 */
typedef struct
{
    /* Class handler */
    const char*          className;
    void*                classData;
    USBD_STA_T(*ClassInitHandler)(struct _USBD_INFO_T* usbInfo, uint8_t cfgIndex);
    USBD_STA_T(*ClassDeInitHandler)(struct _USBD_INFO_T* usbInfo, uint8_t cfgIndex);
    USBD_STA_T(*ClassSofHandler)(struct _USBD_INFO_T* usbInfo);

    /* Control endpoint */
    USBD_STA_T(*ClassSetup)(struct _USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);
    USBD_STA_T(*ClassTxEP0)(struct _USBD_INFO_T* usbInfo);
    USBD_STA_T(*ClassRxEP0)(struct _USBD_INFO_T* usbInfo);
    /* Specific endpoint */
    USBD_STA_T(*ClassDataIn)(struct _USBD_INFO_T* usbInfo, uint8_t epNum);
    USBD_STA_T(*ClassDataOut)(struct _USBD_INFO_T* usbInfo, uint8_t epNum);
    USBD_STA_T(*ClassIsoOutIncomplete)(struct _USBD_INFO_T* usbInfo, uint8_t epNum);
    USBD_STA_T(*ClassIsoInIncomplete)(struct _USBD_INFO_T* usbInfo, uint8_t epNum);
} USBD_CLASS_T;

/**
 * @brief   USB device information
 */
typedef struct _USBD_INFO_T
{
    __IO uint8_t            devState;
    __IO uint8_t            preDevState;
    __IO uint8_t            devEp0State;
    uint32_t                devEp0DataLen;

    uint8_t                 devSpeed;
    uint8_t                 devAddr;

    USBD_DESC_T*            devDesc;
    USBD_CLASS_T*           devClass[USBD_SUP_CLASS_MAX_NUM];

    void*                   devClassUserData[USBD_SUP_CLASS_MAX_NUM];
    uint32_t                classID;
    uint32_t                classNum;

    void*                   cfgDesc;
    USBD_REQ_SETUP_T        reqSetup;

    uint32_t                devCfg;
    uint32_t                devCfgStatus;
    uint32_t                devCfgDefault;
    uint8_t                 devTestModeStatus;
    uint32_t                devRemoteWakeUpStatus;

    USBD_EP_INFO_T          devEpIn[16];
    USBD_EP_INFO_T          devEpOut[16];
    void (*userCallback)(struct _USBD_INFO_T* usbInfo, uint8_t userStatus);
    void*                   dataPoint;
} USBD_INFO_T;

/**@} end of group USBD_Core_Structures*/
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */

#endif
