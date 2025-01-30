/**
 * @file        usbd_descriptor.c
 *
 * @brief       USB device descriptor configuration
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

/* Includes ***************************************************************/
#include "usbd_msc_descriptor.h"

/* Private includes *******************************************************/
#include "usbd_msc.h"
#include "platform.h"

#include <string.h>

/* Private macro **********************************************************/
#define USBD_GEEHY_VID                  0x314B
#define USBD_FS_PID                     0x5720
#define USBD_LANGID_STR                 0x0409
#define USBD_MANUFACTURER_STR           "Geehy"
#define USBD_PRODUCT_HS_STR             "Betaflight FC Mass Storage (HS Mode)"
#define USBD_PRODUCT_FS_STR             "Betaflight FC Mass Storage (FS Mode)"
#define USBD_CONFIGURATION_HS_STR       "MSC Config"
#define USBD_CONFIGURATION_FS_STR       "MSC Config"
#define USBD_INTERFACE_HS_STR           "MSC Interface"
#define USBD_INTERFACE_FS_STR           "MSC Interface"

/* Private function prototypes ********************************************/
static USBD_DESC_INFO_T USBD_MSC_DeviceDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_ConfigDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_ConfigStrDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_InterfaceStrDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_LangIdStrDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_ManufacturerStrDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_ProductStrDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_MSC_SerialStrDescHandler(uint8_t usbSpeed);
#if USBD_SUP_LPM
static USBD_DESC_INFO_T USBD_MSC_BosDescHandler(uint8_t usbSpeed);
#endif
static USBD_DESC_INFO_T USBD_OtherSpeedConfigDescHandler(uint8_t usbSpeed);
static USBD_DESC_INFO_T USBD_DevQualifierDescHandler(uint8_t usbSpeed);

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void Get_SerialNum(void);

/* Private typedef ********************************************************/

/* USB device descripotr handler */
USBD_DESC_T USBD_DESC_MSC =
{
    "MSC Descriptor",
    USBD_MSC_DeviceDescHandler,
    USBD_MSC_ConfigDescHandler,
    USBD_MSC_ConfigStrDescHandler,
    USBD_MSC_InterfaceStrDescHandler,
    USBD_MSC_LangIdStrDescHandler,
    USBD_MSC_ManufacturerStrDescHandler,
    USBD_MSC_ProductStrDescHandler,
    USBD_MSC_SerialStrDescHandler,
#if USBD_SUP_LPM
    USBD_MSC_BosDescHandler,
#endif
    NULL,
    USBD_OtherSpeedConfigDescHandler,
    USBD_DevQualifierDescHandler,
};

/* Private variables ******************************************************/

/**
 * @brief   Device descriptor
 */
static uint8_t USBD_DeviceDesc[USBD_DEVICE_DESCRIPTOR_SIZE] =
{
    /* bLength */
    0x12,
    /* bDescriptorType */
    USBD_DESC_DEVICE,
    /* bcdUSB */
#if USBD_SUP_LPM
    0x01,            /*<! For resume test of USBCV3.0. Only support LPM USB device */
#else
    0x00,
#endif
    0x02,
    /* bDeviceClass */
    0x00,
    /* bDeviceSubClass */
    0x00,
    /* bDeviceProtocol */
    0x00,
    /* bMaxPacketSize */
    USBD_EP0_PACKET_MAX_SIZE,
    /* idVendor */
    USBD_GEEHY_VID & 0xFF, USBD_GEEHY_VID >> 8,
    /* idProduct */
    USBD_FS_PID & 0xFF, USBD_FS_PID >> 8,
    /* bcdDevice = 2.00 */
    0x00, 0x02,
    /* Index of string descriptor describing manufacturer */
    USBD_DESC_STR_MFC,
    /* Index of string descriptor describing product */
    USBD_DESC_STR_PRODUCT,
    /* Index of string descriptor describing the device serial number */
    USBD_DESC_STR_SERIAL,
    /* bNumConfigurations */
    USBD_SUP_CONFIGURATION_MAX_NUM,
};

/**
 * @brief   Configuration descriptor
 */
static uint8_t USBD_ConfigDesc[USBD_CONFIG_DESCRIPTOR_SIZE] =
{
    /* bLength */
    0x09,
    /* bDescriptorType */
    USBD_DESC_CONFIGURATION,
    /* wTotalLength */
    USBD_CONFIG_DESCRIPTOR_SIZE & 0xFF,
    USBD_CONFIG_DESCRIPTOR_SIZE >> 8,

    /* bNumInterfaces */
    0x01,
    /* bConfigurationValue */
    0x01,
    /* iConfiguration */
    0x01,
    /* bmAttributes */
#if USBD_SUP_SELF_PWR
    0xC0,
#else
    0x80,
#endif
    /* MaxPower */
    0x32,

    /* Mass Storage interface */
    /* bLength */
    0x09,
    /* bDescriptorType */
    USBD_DESC_INTERFACE,
    /* bInterfaceNumber */
    0x00,
    /* bAlternateSetting */
    0x00,
    /* bNumEndpoints */
    0x02,
    /* bInterfaceClass */
    USBD_MSC_ITF_CLASS_ID,
    /* bInterfaceSubClass */
    USBD_MSC_ITF_SUB_CLASS,
    /* bInterfaceProtocol */
    USBD_MSC_ITF_PROTOCOL,
    /* iInterface */
    0x05,

    /* Mass Storage Endpoints */
    /* bLength */
    0x07,
    /* bDescriptorType: Endpoint */
    USBD_DESC_ENDPOINT,
    /* bEndpointAddress */
    USBD_MSC_IN_EP_ADDR,
    /* bmAttributes */
    0x02,
    /* wMaxPacketSize: */
    USBD_MSC_FS_MP_SIZE & 0xFF,
    USBD_MSC_FS_MP_SIZE >> 8,
    /* bInterval: */
    0x00,

    /* bLength */
    0x07,
    /* bDescriptorType: Endpoint */
    USBD_DESC_ENDPOINT,
    /* bEndpointAddress */
    USBD_MSC_OUT_EP_ADDR,
    /* bmAttributes */
    0x02,
    /* wMaxPacketSize: */
    USBD_MSC_FS_MP_SIZE & 0xFF,
    USBD_MSC_FS_MP_SIZE >> 8,
    /* bInterval: */
    0x00,
};

/**
 * @brief   Other speed configuration descriptor
 */
static uint8_t USBD_OtherSpeedCfgDesc[USBD_CONFIG_DESCRIPTOR_SIZE] =
{
    /* bLength */
    0x09,
    /* bDescriptorType */
    USBD_DESC_OTHER_SPEED,
    /* wTotalLength */
    USBD_CONFIG_DESCRIPTOR_SIZE & 0xFF,
    USBD_CONFIG_DESCRIPTOR_SIZE >> 8,

    /* bNumInterfaces */
    0x01,
    /* bConfigurationValue */
    0x01,
    /* iConfiguration */
    0x01,
    /* bmAttributes */
#if USBD_SUP_SELF_PWR
    0xC0,
#else
    0x80,
#endif
    /* MaxPower */
    0x32,

    /* Mass Storage interface */
    /* bLength */
    0x09,
    /* bDescriptorType */
    USBD_DESC_INTERFACE,
    /* bInterfaceNumber */
    0x00,
    /* bAlternateSetting */
    0x00,
    /* bNumEndpoints */
    0x02,
    /* bInterfaceClass */
    USBD_MSC_ITF_CLASS_ID,
    /* bInterfaceSubClass */
    USBD_MSC_ITF_SUB_CLASS,
    /* bInterfaceProtocol */
    USBD_MSC_ITF_PROTOCOL,
    /* iInterface */
    0x05,

    /* Mass Storage Endpoints */
    /* bLength */
    0x07,
    /* bDescriptorType: Endpoint */
    USBD_DESC_ENDPOINT,
    /* bEndpointAddress */
    USBD_MSC_IN_EP_ADDR,
    /* bmAttributes */
    0x02,
    /* wMaxPacketSize: */
    USBD_MSC_FS_MP_SIZE & 0xFF,
    USBD_MSC_FS_MP_SIZE >> 8,
    /* bInterval: */
    0x00,

    /* bLength */
    0x07,
    /* bDescriptorType: Endpoint */
    USBD_DESC_ENDPOINT,
    /* bEndpointAddress */
    USBD_MSC_OUT_EP_ADDR,
    /* bmAttributes */
    0x02,
    /* wMaxPacketSize: */
    USBD_MSC_FS_MP_SIZE & 0xFF,
    USBD_MSC_FS_MP_SIZE >> 8,
    /* bInterval: */
    0x00,
};

#if USBD_SUP_LPM
/**
 * @brief   BOS descriptor
 */
static uint8_t USBD_BosDesc[USBD_BOS_DESCRIPTOR_SIZE] =
{
    /* bLength */
    0x05,
    /* bDescriptorType */
    USBD_DESC_BOS,
    /* wtotalLength */
    0x0C, 0x00,
    /* bNumDeviceCaps */
    0x01,

    /* Device Capability */
    /* bLength */
    0x07,
    /* bDescriptorType */
    USBD_DEVICE_CAPABILITY_TYPE,
    /* bDevCapabilityType */
    USBD_20_EXTENSION_TYPE,
    /* bmAttributes */
    0x02, 0x00, 0x00, 0x00,
};
#endif

/**
 * @brief   Serial string descriptor
 */
static uint8_t USBD_SerialStrDesc[USBD_SERIAL_STRING_SIZE] =
{
    /* bLength */
    USBD_SERIAL_STRING_SIZE,
    /* bDescriptorType */
    USBD_DESC_STRING,
};

/**
 * @brief   Language ID string descriptor
 */
static uint8_t USBD_LandIDStrDesc[USBD_LANGID_STRING_SIZE] =
{
    /* bLength */
    USBD_LANGID_STRING_SIZE,
    /* bDescriptorType */
    USBD_DESC_STRING,
    USBD_LANGID_STR & 0xFF, USBD_LANGID_STR >> 8
};

/**
 * @brief   Device qualifier descriptor
 */
static uint8_t USBD_DevQualifierDesc[USBD_DEVICE_QUALIFIER_DESCRIPTOR_SIZE] =
{
    /* bLength */
    USBD_DEVICE_QUALIFIER_DESCRIPTOR_SIZE,
    /* bDescriptorType */
    USBD_DESC_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    USBD_MSC_FS_MP_SIZE, /* In FS device*/
    0x01,
    0x00,
};

/* Private functions ******************************************************/

/**
 * @brief   USB device convert ascii string descriptor to unicode format
 *
 * @param   desc : descriptor string
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_DESC_Ascii2Unicode(uint8_t* desc)
{
    USBD_DESC_INFO_T descInfo;
    uint8_t* buffer;
    uint8_t str[USBD_SUP_STR_DESC_MAX_NUM];

    uint8_t* unicode = str;
    uint16_t length = 0;
    __IO uint8_t index = 0;

    if (desc == NULL)
    {
        descInfo.desc = NULL;
        descInfo.size = 0;
    }
    else
    {
        buffer = desc;
        length = (strlen((char*)buffer) * 2) + 2;
        /* Get unicode descriptor */
        unicode[index] = length;

        index++;
        unicode[index] = USBD_DESC_STRING;
        index++;

        while (*buffer != '\0')
        {
            unicode[index] = *buffer;
            buffer++;
            index++;

            unicode[index] = 0x00;
            index++;
        }
    }

    descInfo.desc = unicode;
    descInfo.size = length;

    return descInfo;
}

/**
 * @brief   USB device FS device descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_DeviceDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo.desc = USBD_DeviceDesc;
    descInfo.size = sizeof(USBD_DeviceDesc);

    return descInfo;
}

/**
 * @brief   USB device FS configuration descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_ConfigDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo.desc = USBD_ConfigDesc;
    descInfo.size = sizeof(USBD_ConfigDesc);

    return descInfo;
}

#if USBD_SUP_LPM
/**
 * @brief   USB device FS BOS descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_BosDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo.desc = USBD_BosDesc;
    descInfo.size = sizeof(USBD_BosDesc);

    return descInfo;
}
#endif

/**
 * @brief   USB device FS configuration string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_ConfigStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    if (usbSpeed == USBD_SPEED_HS)
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_CONFIGURATION_HS_STR);
    }
    else
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_CONFIGURATION_FS_STR);
    }

    return descInfo;
}

/**
 * @brief   USB device FS interface string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_InterfaceStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    if (usbSpeed == USBD_SPEED_HS)
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_INTERFACE_HS_STR);
    }
    else
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_INTERFACE_FS_STR);
    }

    return descInfo;
}

/**
 * @brief   USB device FS LANG ID string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_LangIdStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo.desc = USBD_LandIDStrDesc;
    descInfo.size = sizeof(USBD_LandIDStrDesc);

    return descInfo;
}

/**
 * @brief   USB device FS manufacturer string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_ManufacturerStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_MANUFACTURER_STR);

    return descInfo;
}

/**
 * @brief   USB device FS product string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_ProductStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    if (usbSpeed == USBD_SPEED_HS)
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_PRODUCT_HS_STR);
    }
    else
    {
        descInfo = USBD_DESC_Ascii2Unicode((uint8_t*)USBD_PRODUCT_FS_STR);
    }

    return descInfo;
}

/**
 * @brief   USB device FS serial string descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_MSC_SerialStrDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    /* Update the serial number string descriptor with the data from the unique ID*/
    Get_SerialNum();

    descInfo.desc = USBD_SerialStrDesc;
    descInfo.size = sizeof(USBD_SerialStrDesc);

    return descInfo;
}

/**
 * @brief   USB device other speed configuration descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_OtherSpeedConfigDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    /* Use FS configuration */
    descInfo.desc = USBD_OtherSpeedCfgDesc;
    descInfo.size = sizeof(USBD_OtherSpeedCfgDesc);

    return descInfo;
}

/**
 * @brief   USB device device qualifier descriptor
 *
 * @param   usbSpeed : usb speed
 *
 * @retval  usb descriptor information
 */
static USBD_DESC_INFO_T USBD_DevQualifierDescHandler(uint8_t usbSpeed)
{
    USBD_DESC_INFO_T descInfo;

    UNUSED(usbSpeed);

    descInfo.desc = USBD_DevQualifierDesc;
    descInfo.size = sizeof(USBD_DevQualifierDesc);

    return descInfo;
}

/**
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = U_ID_0;
  deviceserial1 = U_ID_1;
  deviceserial2 = U_ID_2;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode (deviceserial0, &USBD_SerialStrDesc[2] ,8);
    IntToUnicode (deviceserial1, &USBD_SerialStrDesc[18] ,4);
  }
}

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for ( idx = 0; idx < len; idx ++)
  {
    if ( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}
