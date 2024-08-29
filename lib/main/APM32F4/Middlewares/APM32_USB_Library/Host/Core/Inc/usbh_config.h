/*!
 * @file        usbh_config.h
 *
 * @brief       usb host config header file
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
#ifndef _USBH_CONFIG_H_
#define _USBH_CONFIG_H_

/* Includes */
#include "usbh_board.h"
#include <stdio.h>

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Macros Macros
  @{
*/

/*!< [31:16] APM32 USB Host Library main version V1.1.3*/
#define __APM32_USB_HOST_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __APM32_USB_HOST_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __APM32_USB_HOST_VERSION_SUB2   (0x03) /*!< [15:8]  sub2 version */
#define __APM32_USB_HOST_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __APM32_USB_HOST_VERSION        ((__APM32_USB_HOST_VERSION_MAIN << 24)\
                                        |(__APM32_USB_HOST_VERSION_SUB1 << 16)\
                                        |(__APM32_USB_HOST_VERSION_SUB2 << 8 )\
                                        |(__APM32_USB_HOST_VERSION_RC))

#define USBH_EP0_PACKET_SIZE                0x40
#define USBH_LANG_ID                        0x0409

#define USB_REQ_DIR_OUT                     0x00
#define USB_REQ_DIR_IN                      0x80

#define USBH_DEVICE_RESET_TIMEOUT           0x64
#define USBH_DEVICE_DEFAULT_ADDRESS         0
#define USBH_DEVICE_CONFIGURED_ADDRESS      1

#define STD_DEV_DESC_SIZE                   18
#define STD_CFG_DESC_SIZE                   9
#define STD_INTERFACE_DESC_SIZE             9
#define STD_EP_DESC_SIZE                    7
#define STD_HID_DESC_SIZE                   9

#define CFG_DESC_MAX_LEN                    256
#define STRING_DESC_MAX_LEN                 0x200

#ifndef USBH_Delay
#define USBH_Delay                          DAL_Delay
#endif

/**@} end of group USBH_Core_Macros*/

/** @defgroup USBH_Core_Enumerates Enumerates
  @{
  */

/**
 * @brief   USB host speed type
 */
typedef enum
{
    USBH_SPEED_FS,
    USBH_SPEED_HS,
} USBH_SPEED_T;

/**
 * @brief   USB device speed
 */
typedef enum
{
    USBH_DEVICE_SPEED_HS,
    USBH_DEVICE_SPEED_FS,
    USBH_DEVICE_SPEED_LS
} USBH_DEVICE_SPEED_T;

/**
 * @brief   USB host operation status
 */
typedef enum
{
    USBH_OK,
    USBH_BUSY,
    USBH_FAIL,
    USBH_ERR_NOT_SUP,
    USBH_ERR_UNRECOVERED,
    USBH_ERR_SPEED_UNKOWN,
} USBH_STA_T;

/**
 * @brief   USB host state machine
 */
typedef enum
{
    USBH_HOST_IDLE,
    USBH_HOST_DEVICE_ATTACHED,
    USBH_HOST_DEVICE_DISCONNECTED,
    USBH_HOST_ENUMERATION,
    USBH_HOST_CLASS_REQ,
    USBH_HOST_USER_INPUT,
    USBH_HOST_SET_CONFIGURATION,
    USBH_HOST_SET_FEATURE,
    USBH_HOST_CLASS_ACTIVE,
    USBH_HOST_CLASS,
    USBH_HOST_SUSPEND,
    USBH_HOST_ABORT,
} USBH_HOST_STA_T;

/**
 * @brief   USB host enumeration state
 */
typedef enum
{
    USBH_ENUM_IDLE,                      //!< enum idle
    USBH_ENUM_GET_DEV_DESC,              //!< Get device descriptor
    USBH_ENUM_SET_ADDR,                  //!< Set address
    USBH_ENUM_GET_CFG_DESC,              //!< Get Configuration descriptor
    USBH_ENUM_GET_FULL_CFG_DESC,         //!< Get Full Configuration descriptor
    USBH_ENUM_GET_MFC_STRING_DESC,       //!< Get Manufacturer string
    USBH_ENUM_GET_PRODUCT_STRING_DESC,   //!< Get Product string
    USBH_ENUM_GET_SERIALNUM_STRING_DESC, //!< Get serial number string
} USBH_ENUM_STA_T;

/**
 * @brief   USB transfer state
 */
typedef enum
{
    USBH_XFER_START,
    USBH_XFER_WAITING,
    USBH_XFER_OK,
} USBH_XFER_STA_T;

/**
 * @brief   USB Host control transfer state
 */
typedef enum
{
    USBH_CTRL_IDLE,
    USBH_CTRL_SETUP,
    USBH_CTRL_SETUP_WAIT,
    USBH_CTRL_DATA_IN,
    USBH_CTRL_DATA_IN_WAIT,
    USBH_CTRL_DATA_OUT,
    USBH_CTRL_DATA_OUT_WAIT,
    USBH_CTRL_STA_IN,
    USBH_CTRL_STA_IN_WAIT,
    USBH_CTRL_STA_OUT,
    USBH_CTRL_STA_OUT_WAIT,
    USBH_CTRL_ERROR,
    USBH_CTRL_STALL,
    USBH_CTRL_OK
} USBH_CTRL_STATE_T;

/**
 * @brief   USB device feature request type
 */
typedef enum
{
    USBH_FEATURE_SELECTOR_ENDPOINT_HALT,
    USBH_FEATURE_REMOTE_WAKEUP,
    USBH_FEATURE_TEST_MODE,
} USBH_REQ_FEATURE_T;

/**
 * @brief   USB device request direction
 */
typedef enum
{
    USBH_REQ_DIR_OUT,
    USBH_REQ_DIR_IN
} USBH_DEV_REQ_DIR_T;

/**
 * @brief   USB device request type
 */
typedef enum
{
    USBH_REQ_TYPE_STANDARD = 0,
    USBH_REQ_TYPE_CLASS,
    USBH_REQ_TYPE_VENDOR,
    USBH_REQ_TYPE_RESERVED
} USBH_DEV_REQ_TYPE_T;

/**
 * @brief   USB device request recipient
 */
typedef enum
{
    USBH_RECIPIENT_DEVICE = 0,
    USBH_RECIPIENT_INTERFACE,
    USBH_RECIPIENT_ENDPOINT,
    USBH_RECIPIENT_OTHER
} USBH_DEV_RECIPIENT_T;

/**
 * @brief   USB standard device standard requests type
 */
typedef enum
{
    USBH_STD_GET_STATUS          = 0,
    USBH_STD_CLEAR_FEATURE       = 1,
    USBH_STD_SET_FEATURE         = 3,
    USBH_STD_SET_ADDRESS         = 5,
    USBH_STD_GET_DESCRIPTOR      = 6,
    USBH_STD_SET_DESCRIPTOR      = 7,
    USBH_STD_GET_CONFIGURATION   = 8,
    USBH_STD_SET_CONFIGURATION   = 9,
    USBH_STD_GET_INTERFACE       = 10,
    USBH_STD_SET_INTERFACE       = 11,
    USBH_STD_SYNCH_FRAME         = 12
} USBH_STD_REQ_TYPE_T;

/**
 * @brief   USB descriptor types
 */
typedef enum
{
    USBH_DESC_DEVICE             = 1,
    USBH_DESC_CONFIGURATION      = 2,
    USBH_DESC_STRING             = 3,
    USBH_DESC_INTERFACE          = 4,
    USBH_DESC_ENDPOINT           = 5,
    USBH_DESC_DEVICE_QUALIFIER   = 6,
    USBH_DESC_OTHER_SPEED        = 7,
    USBH_DESC_INTERFACE_POWER    = 8,
    USBH_DESC_HID                = 0x21,
    USBH_DESC_HID_REPORT         = 0x22,
    USBH_DESC_HID_PHY            = 0x23,
} USBH_DESC_TYPE_T;

/**
 * @brief   USB Class type
 */
typedef enum
{
    USBH_CLASS_AUDIO     = 0x01,  //!< Audio
    USBH_CLASS_CDCC      = 0x02,  //!< Communications and CDC Control
    USBH_CLASS_HID       = 0x03,  //!< HID (Human Interface Device)
    USBH_CLASS_PRINTER   = 0x07,  //!< Printer
    USBH_CLASS_MSC       = 0x08,  //!< Mass Storage
    USBH_CLASS_HUB       = 0x09,  //!< Hub
    USBH_CLASS_CDCD      = 0x0A,  //!< CDC-Data
    USBH_CLASS_SMARTCARD = 0x0B,  //!< Smart Card
    USBH_CLASS_VIDEO     = 0x0E,  //!< Video
    USBH_CLASS_AVD       = 0x10   //!< Audio/Video Devices
} USBH_CLASS_TYPE_T;

/**@} end of group USBH_Core_Enumerates*/

/** @defgroup USBH_Core_Structures Structures
  @{
  */

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
} USBH_REQ_TYPE_T;

/**
 * @brief   USB Host request data
 */
typedef struct
{
    union
    {
        uint8_t REQ_DATA[8];

        struct
        {
            USBH_REQ_TYPE_T bmRequestType;
            uint8_t         bRequest;
            uint8_t         wValue[2];
            uint8_t         wIndex[2];
            uint8_t         wLength[2];
        } DATA_FIELD;
    };

} USBH_REQ_DATA_T;

/**
 * @brief   USB device descriptor
 */
typedef struct
{
    uint8_t bLength;            //!< Descriptor length
    uint8_t bDescriptorType;    //!< Descriptor Type
    uint8_t bcdUSB[2];          //!< BCD of the supported USB specification
    uint8_t bDeviceClass;       //!< USB device class
    uint8_t bDeviceSubClass;    //!< USB device subclass
    uint8_t bDeviceProtocol;    //!< USB device protocol
    uint8_t bMaxPacketSize;     //!< Max Packet Size
    uint8_t idVendor[2];        //!< Vendor ID
    uint8_t idProduct[2];       //!< Product ID
    uint8_t bcdDevice[2];       //!< Device Release Number
    uint8_t iManufacturer;      //!< Index of Manufacturer String Descriptor
    uint8_t iProduct;           //!< Index of Product String Descriptor
    uint8_t iSerialNumber;      //!< Index of Serial Number String Descriptor
    uint8_t bNumConfigurations; //!< Number of Possible Configurations
} USBH_DEV_DESC_T;

/**
 * @brief   USB Endpoint descriptor
 */
typedef struct
{
    uint8_t bLength;           //!< Descriptor length
    uint8_t bDescriptorType;   //!< Descriptor Type
    uint8_t bEndpointAddress;  //!< Indicates what endpoint this descriptor is describing
    uint8_t bmAttributes;      //!< Specifies the transfer type
    uint8_t wMaxPacketSize[2]; //!< Maximum Packet Size this endpoint is capable of sending or receiving
    uint8_t bInterval;         //!< Polling interval in milliseconds for the endpoint
} USBH_EP_DESC_T;

/**
 * @brief   USB Interface descriptor
 */
typedef struct
{
    uint8_t bLength;             //!< Descriptor length
    uint8_t bDescriptorType;     //!< Descriptor Type
    uint8_t bInterfaceNumber;    //!< Interface Number
    uint8_t bAlternateSetting;   //!< Value used to select alternative setting
    uint8_t bNumEndpoints;       //!< Number of Endpoints used for this interface
    uint8_t bInterfaceClass;     //!< Class Code
    uint8_t bInterfaceSubClass;  //!< Sub class Code
    uint8_t bInterfaceProtocol;  //!< Protocol Code
    uint8_t iInterface;          //!< Index of String Descriptor of this interface
} USBH_ITF_DESC_T;

/**
 * @brief   USB Configuration descriptor
 */
typedef struct
{
    uint8_t bLength;             //!< Descriptor length
    uint8_t bDescriptorType;     //!< Descriptor Type
    uint8_t wTotalLength[2];     //!< Total Length of Configuration Descriptor gather
    uint8_t bNumInterfaces;      //!< Total number of interfaces in the configuration
    uint8_t bConfigurationValue; //!< Configuration index of the current configuration
    uint8_t iConfiguration;      //!< Index of a string descriptor describing the configuration
    uint8_t bmAttributes;        //!< Configuration attributes
    uint8_t bMaxPower;           //!< Maximum power consumption
} USBH_CFG_DESC_T;

/**
 * @brief   USB Interface and Endpoint descriptor
 */
typedef struct
{
    USBH_ITF_DESC_T interfaceDesc;
    USBH_EP_DESC_T  endpointDesc[ENDPOINT_DESC_MAX_NUM];
} USBH_INTERFACE_T;

/**
 * @brief   USB Host Descriptor structure
 */
typedef struct
{
    USBH_DEV_DESC_T         device;
    USBH_CFG_DESC_T         configuration;
    USBH_INTERFACE_T        interface[INTERFACE_DESC_MAX_NUM];
    uint8_t                 cfgDescBuf[CFG_DESC_MAX_LEN];
    uint8_t                 stringBuf[STRING_DESC_MAX_LEN];
} USBH_DESC_T;

/**
 * @brief   USB Host device information
 */
typedef struct
{
    uint8_t         data[USBH_DATA_BUF_MAX_NUM];
    uint8_t         address;
    uint8_t         speed;
    __IO uint8_t    connectedStatus;
    __IO uint8_t    disconnectedStatus;
    __IO uint8_t    reEnumStatus;
    uint8_t         portEnable;
    uint8_t         rstCnt;
    uint8_t         enumCnt;
    USBH_DESC_T     desc;
} USBH_DEV_INFO_T;

/**
 * @brief   USB Control thansfer info
 */
typedef struct
{
    uint8_t             state;
    uint8_t             errCnt;
    uint8_t             channelInNum;
    uint8_t             channelOutNum;
    uint8_t             channelSize;
    uint8_t*             buffer;
    uint16_t            length;
    USBH_REQ_DATA_T     reqData;
} USBH_CTRL_T;

struct _USBH_INFO_T;

/* Class callback function type define */
typedef USBH_STA_T(*USBH_ClassCallback_T)(struct _USBH_INFO_T* usbInfo);

/**
 * @brief   USB host class handler
 */
typedef struct
{
    const char*              className;
    uint8_t                 classCode;
    void*                    classData;
    USBH_ClassCallback_T    classInitHandler;
    USBH_ClassCallback_T    classDeInitHandler;
    USBH_ClassCallback_T    classReqHandler;
    USBH_ClassCallback_T    classCoreHandler;
    USBH_ClassCallback_T    classSofHandler;
} USBH_CLASS_T;

/* Host state handler function */
typedef USBH_STA_T(*USBH_CoreHandler_T)(struct _USBH_INFO_T* usbInfo);

/* Host enum state handler function */
typedef USBH_STA_T(*USBH_EnumHandler_T)(struct _USBH_INFO_T* usbInfo);

/* Host control transfer state handler function */
typedef USBH_STA_T(*USBH_CtrlStateHandler_T)(struct _USBH_INFO_T* usbInfo);

/**
 * @brief   USB host information
 */
typedef struct _USBH_INFO_T
{
    USBH_HOST_STA_T         hostState;      /*!< USB Host State Machine*/
    USBH_ENUM_STA_T         hostEnumState;
    USBH_SPEED_T            hostSpeed;
    USBH_DEV_INFO_T         devInfo;
    uint8_t                 xferState;
    USBH_CTRL_T             ctrl;                                   /*!< Control Thansfer info management*/
    USBH_CLASS_T*            hostClass[USBH_SUP_CLASS_MAX_NUM];     /*!< USB host class */
    USBH_CLASS_T*            activeClass;
    uint32_t                classNum;
    __IO uint32_t           timer;
    uint32_t                timeout;
    uint32_t                xferChannel[USBH_CHANNEL_MAX_NUM]; /*!< Thansfer data management (The sixteenth bit is indicate free or not status)*/
    void (*userCallback)(struct _USBH_INFO_T* usbInfo, uint8_t userStatus);
    void*                    dataPoint;
} USBH_INFO_T;


/**@} end of group USBH_Core_Structures*/
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */

#endif
