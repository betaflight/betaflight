/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __USBD_CDC_VCP_H__
#define __USBD_CDC_VCP_H__

#include <stdint.h>

#include "usbd_cdc_core.h"
#include "usbd_core.h"
#include "usbd_conf.h"

extern USB_CORE_MODULE USB_dev;
extern CDC_IF_Prop_TypeDef VCP_fops;

uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength);
uint32_t CDC_Send_FreeBytes(void);
uint32_t CDC_Receive_DATA(uint8_t *recvBuf, uint32_t len);
uint32_t CDC_Receive_BytesAvailable(void);

uint8_t usbIsConfigured(void);
uint8_t usbIsConnected(void);
uint32_t CDC_BaudRate(void);
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context);
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context);

#endif /* __USBD_CDC_VCP_H__ */
