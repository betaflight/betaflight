/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "usbd_desc.h"
#include "usbd_msc_desc.h"

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#define USBD_VID                          0xFFFE
#define USBD_PID                          0x5780

#define USBD_LANGID_STRING                0x409
#define USBD_MANUFACTURER_STRING          "XCORELABS"


#define USBD_MSC_PRODUCT_HS_STRING        "Mass Storage in HS Mode"
#define USBD_MSC_PRODUCT_FS_STRING        "Mass Storage in FS Mode"
#define USBD_MSC_CONFIGURATION_HS_STRING  "MSC Config"
#define USBD_MSC_INTERFACE_HS_STRING      "MSC Interface"
#define USBD_MSC_CONFIGURATION_FS_STRING  "MSC Config"
#define USBD_MSC_INTERFACE_FS_STRING      "MSC Interface"


USBD_DEVICE_DESC USBD_MSC_desc = 
{
    USBD_MSC_USER_DeviceDescriptor,
    USBD_MSC_USER_LangIDStrDescriptor,
    USBD_MSC_USER_ManufacturerStrDescriptor,
    USBD_MSC_USER_ProductStrDescriptor,
    USBD_MSC_USER_SerialStrDescriptor,
    USBD_MSC_USER_ConfigStrDescriptor,
    USBD_MSC_USER_InterfaceStrDescriptor,
};


#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_MSC_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END = 
{
    0x12,                         /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,   /* bDescriptorType */
    0x00,                         /* bcdUSB */
    0x02,
    0x00,                         /* bDeviceClass */
    0x00,                         /* bDeviceSubClass */
    0x00,                         /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,             /* bMaxPacketSize */
    LOBYTE(USBD_VID),             /* idVendor */
    HIBYTE(USBD_VID),             /* idVendor */
    LOBYTE(USBD_PID),             /* idVendor */
    HIBYTE(USBD_PID),             /* idVendor */
    0x00,                         /* bcdDevice rel. 2.00 */
    0x02,
    USBD_IDX_MFC_STR,             /* Index of manufacturer string */
    USBD_IDX_PRODUCT_STR,         /* Index of product string */
    USBD_IDX_SERIAL_STR,          /* Index of serial number string */
    USBD_CFG_MAX_NUM              /* bNumConfigurations */
};                                /* USB_DeviceDescriptor */


/*
 * USBD_DeviceQualifierDesc, USBD_LangIDDesc, USBD_StringSerial, USBD_StrDesc,
 * IntToUnicode() and Get_SerialNum() are defined in usbd_desc.c and are
 * shared across all USB classes (VCP, MSC, ...). Redefining them here would
 * cause "multiple definition" link errors. The externs come from usbd_desc.h.
 */
extern void Get_SerialNum(void);


/**
*\*\name   USBD_MSC_USER_DeviceDescriptor.
*\*\fun    return the device descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_DeviceDescriptor(uint8_t speed, uint16_t * length)
{
    *length = sizeof(USBD_MSC_DeviceDesc);
    return (uint8_t *) USBD_MSC_DeviceDesc;
}

/**
*\*\name   USBD_MSC_USER_LangIDStrDescriptor.
*\*\fun    return the LangID string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_LangIDStrDescriptor(uint8_t speed, uint16_t * length)
{
    *length = sizeof(USBD_LangIDDesc);
    return (uint8_t *) USBD_LangIDDesc;
}

/**
*\*\name   USBD_MSC_USER_ProductStrDescriptor.
*\*\fun    return the product string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_ProductStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == USB_SPEED_HIGH)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_PRODUCT_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_PRODUCT_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}

/**
*\*\name   USBD_MSC_USER_ManufacturerStrDescriptor.
*\*\fun    return the manufacturer string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_ManufacturerStrDescriptor(uint8_t speed, uint16_t * length)
{
    USBD_GetString((uint8_t *) (uint8_t *) USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/**
*\*\name   USBD_MSC_USER_SerialStrDescriptor.
*\*\fun    return the serial number string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_SerialStrDescriptor(uint8_t speed, uint16_t * length)
{
    *length = USB_SIZ_STRING_SERIAL;

    /* Update the serial number string descriptor with the data from the unique * ID */
    Get_SerialNum();

    return (uint8_t *) USBD_StringSerial;
}

/**
*\*\name   USBD_MSC_USER_ConfigStrDescriptor.
*\*\fun    return the configuration string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_ConfigStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == USB_SPEED_HIGH)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}

/**
*\*\name   USBD_MSC_USER_InterfaceStrDescriptor.
*\*\fun    return the interface string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_MSC_USER_InterfaceStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == 0)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_INTERFACE_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_MSC_INTERFACE_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}


#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif



