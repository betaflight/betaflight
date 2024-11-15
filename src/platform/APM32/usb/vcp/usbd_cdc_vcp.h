/**
 * @file        usbd_cdc_vcp.h
 *
 * @brief       usb device CDC class VCP handler header file
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
#ifndef _USBD_CDC_VCP_H_
#define _USBD_CDC_VCP_H_

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ***************************************************************/
#include "usbd_cdc.h"

/* Exported macro *********************************************************/

/* Periodically, the state of the buffer "cdcTxBuffer" is checked.
   The period depends on CDC_POLLING_INTERVAL */
#define CDC_POLLING_INTERVAL             5 /* in ms. The max is 65 and the min is 1 */

/* Exported typedef *******************************************************/
/* The following structures groups all needed parameters to be configured for the
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct __attribute__ ((packed))
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} LINE_CODING;

/* Exported function prototypes *******************************************/
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength);
uint32_t CDC_Send_FreeBytes(void);
uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len);
uint32_t CDC_Receive_BytesAvailable(void);

uint8_t usbIsConfigured(void);
uint8_t usbIsConnected(void);
uint32_t CDC_BaudRate(void);
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context);
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context);

#ifdef __cplusplus
  extern "C" {
#endif

#endif
