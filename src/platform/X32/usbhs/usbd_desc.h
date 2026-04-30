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


#ifndef __USB_DESC_H
#define __USB_DESC_H

#include "usbd_req.h"

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_SIZ_DEVICE_DESC                     18
#define USB_SIZ_STRING_LANGID                   4


#define  USB_SIZ_STRING_SERIAL                  0x1A

/*
 * Common descriptor buffers — defined in usbd_desc.c and shared by any USB
 * class (VCP, MSC, ...). They must NOT be redefined in class-specific files,
 * or the linker reports "multiple definition" errors.
 */
extern  uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];
extern  uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID];
extern  uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL];
extern  uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];


extern  USBD_DEVICE_DESC USBD_VCP_desc;

uint8_t *USBD_VCP_USER_DeviceDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_LangIDStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_ProductStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_ManufacturerStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_SerialStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_ConfigStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_VCP_USER_InterfaceStrDescriptor(uint8_t speed, uint16_t * length);


#endif /* __USBD_DESC_H */