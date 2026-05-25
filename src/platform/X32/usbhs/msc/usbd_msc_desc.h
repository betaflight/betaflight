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


#ifndef __USB_MSC_DESC_H
#define __USB_MSC_DESC_H

#include "usbd_req.h"


extern  USBD_DEVICE_DESC USBD_MSC_desc; 

uint8_t *USBD_MSC_USER_DeviceDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_LangIDStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_ProductStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_ManufacturerStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_SerialStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_ConfigStrDescriptor(uint8_t speed, uint16_t * length);
uint8_t *USBD_MSC_USER_InterfaceStrDescriptor(uint8_t speed, uint16_t * length);


#endif /* __USB_MSC_DESC_H */