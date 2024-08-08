/**
 * @file        usbd_descriptor.h
 *
 * @brief       usb device descriptor
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
#ifndef _USBD_DESCRIPTOR_H_
#define _USBD_DESCRIPTOR_H_

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ***************************************************************/
#include "usbd_core.h"

/* Exported macro *********************************************************/
#define USBD_DEVICE_DESCRIPTOR_SIZE             18
#define USBD_CONFIG_DESCRIPTOR_SIZE             67
#define USBD_SERIAL_STRING_SIZE                 0x1A
#define USBD_LANGID_STRING_SIZE                 4
#define USBD_DEVICE_QUALIFIER_DESCRIPTOR_SIZE   10
#define USBD_BOS_DESCRIPTOR_SIZE                12

#define USBD_DEVICE_CAPABILITY_TYPE             0x10
#define USBD_20_EXTENSION_TYPE                  0x02

#define USBD_CDC_CTRL_ITF_CLASS_ID              0x02
#define USBD_CDC_DATA_ITF_CLASS_ID              0x0A
#define USBD_CDC_SUB_CLASS_ACM                  0x02
#define USBD_CDC_ITF_PORTOCOL_AT                0x01

/* Exported typedef *******************************************************/

/* Exported function prototypes *******************************************/

#ifdef __cplusplus
}
#endif

#endif
