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

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#define USBD_VID                          0xFFFE
#define USBD_PID                          0x5740

#define USBD_LANGID_STRING                0x409
#define USBD_MANUFACTURER_STRING          "XCORELABS"


#define USBD_VCP_PRODUCT_HS_STRING        "Virtual ComPort in HS Mode"
#define USBD_VCP_PRODUCT_FS_STRING        "Virtual ComPort in FS Mode"
#define USBD_VCP_CONFIGURATION_HS_STRING  "VCP Config"
#define USBD_VCP_INTERFACE_HS_STRING      "VCP Interface"
#define USBD_VCP_CONFIGURATION_FS_STRING  "VCP Config"
#define USBD_VCP_INTERFACE_FS_STRING      "VCP Interface"

USBD_DEVICE_DESC USBD_VCP_desc = 
{
    USBD_VCP_USER_DeviceDescriptor,
    USBD_VCP_USER_LangIDStrDescriptor,
    USBD_VCP_USER_ManufacturerStrDescriptor,
    USBD_VCP_USER_ProductStrDescriptor,
    USBD_VCP_USER_SerialStrDescriptor,
    USBD_VCP_USER_ConfigStrDescriptor,
    USBD_VCP_USER_InterfaceStrDescriptor,
};


#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_INTERNAL_DMA_ENABLED */
/* USB VCP Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_VCP_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END = 
{
    0x12,                         /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,   /* bDescriptorType */
    0x00,                         /* bcdUSB */
    0x02,
    0x02,                         /* bDeviceClass */
    0x02,                         /* bDeviceSubClass */
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


#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]
  __ALIGN_END = 
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END = 
{
    USB_SIZ_STRING_LANGID,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING),
};

uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] = 
{
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ] __ALIGN_END;


/**
*\*\name   IntToUnicode.
*\*\fun    Convert Hex 32Bits value into char.
*\*\param  value: value to convert
*\*\param  pbuf: pointer to the buffer 
*\*\param  len: buffer length
*\*\return none.
*/
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
    uint8_t idx = 0;

    for (idx = 0; idx < len; idx++)
    {
        if (((value >> 28)) < 0xA)
        {
            pbuf[2 * idx] = (value >> 28) + '0';
        }
        else
        {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }
        value = value << 4;
        pbuf[2 * idx + 1] = 0;
    }
}


/**
*\*\name   Get_SerialNum.
*\*\fun    Create the serial number string descriptor .
*\*\param  none
*\*\param  none
*\*\return pointer to descriptor buffer
*/
void Get_SerialNum(void)
{
    volatile uint32_t deviceserial0, deviceserial1, deviceserial2;

    deviceserial0 = 1;
    deviceserial1 = 2;
    deviceserial2 = 3;

    deviceserial0 += deviceserial2;

    if (deviceserial0 != 0)
    {
        IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
        IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
    }
	
	/* Add a volatile opration, prevent code optimization */
    volatile uint8_t dummy = USBD_StringSerial[0];
	/* Avoid unused variable warnings */
    (void)dummy; 
}



/**
*\*\name   USBD_VCP_USER_DeviceDescriptor.
*\*\fun    return the device descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_DeviceDescriptor(uint8_t speed, uint16_t * length)
{
    *length = sizeof(USBD_VCP_DeviceDesc);
    return (uint8_t *) USBD_VCP_DeviceDesc;
}

/**
*\*\name   USBD_VCP_USER_LangIDStrDescriptor.
*\*\fun    return the LangID string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_LangIDStrDescriptor(uint8_t speed, uint16_t * length)
{
    *length = sizeof(USBD_LangIDDesc);
    return (uint8_t *) USBD_LangIDDesc;
}

/**
*\*\name   USBD_VCP_USER_ProductStrDescriptor.
*\*\fun    return the product string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_ProductStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == 0)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_PRODUCT_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_PRODUCT_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}

/**
*\*\name   USBD_VCP_USER_ManufacturerStrDescriptor.
*\*\fun    return the manufacturer string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_ManufacturerStrDescriptor(uint8_t speed, uint16_t * length)
{
    USBD_GetString((uint8_t *) (uint8_t *) USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/**
*\*\name   USBD_VCP_USER_SerialStrDescriptor.
*\*\fun    return the serial number string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_SerialStrDescriptor(uint8_t speed, uint16_t * length)
{
    *length = USB_SIZ_STRING_SERIAL;

    /* Update the serial number string descriptor with the data from the unique * ID */
    Get_SerialNum();

    return (uint8_t *) USBD_StringSerial;
}

/**
*\*\name   USBD_VCP_USER_ConfigStrDescriptor.
*\*\fun    return the configuration string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_ConfigStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == USB_SPEED_HIGH)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}

/**
*\*\name   USBD_VCP_USER_InterfaceStrDescriptor.
*\*\fun    return the interface string descriptor.
*\*\param  speed : current device speed
*\*\param  length : pointer to data length variable
*\*\return pointer to descriptor buffer
*/
uint8_t *USBD_VCP_USER_InterfaceStrDescriptor(uint8_t speed, uint16_t * length)
{
    if (speed == 0)
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_INTERFACE_HS_STRING, USBD_StrDesc, length);
    }
    else
    {
        USBD_GetString((uint8_t *) (uint8_t *) USBD_VCP_INTERFACE_FS_STRING, USBD_StrDesc, length);
    }
    return USBD_StrDesc;
}


#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif



