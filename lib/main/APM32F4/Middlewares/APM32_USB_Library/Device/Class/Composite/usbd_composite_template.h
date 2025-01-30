/*!
 * @file        usbd_composite_template.h
 *
 * @brief       usb device composite class handler header file
 *
 * @version     V1.0.0
 *
 * @date        2023-11-13
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
#ifndef _USBD_COMPOSITE_H_
#define _USBD_COMPOSITE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "usbd_core.h"

/** @addtogroup Examples
  @{
  */

/** @addtogroup OTGD_Composite
  @{
  */

/** @defgroup OTGD_Composite_Macros Macros
  @{
*/

/**@} end of group OTGD_Composite_Macros*/

/** @defgroup OTGD_Composite_Structures Structures
  @{
  */
extern USBD_CLASS_T USBD_COMPOSITE_CLASS;
/**@} end of group OTGD_Composite_Structures*/

/** @defgroup OTGD_Composite_Functions Functions
  @{
  */
USBD_STA_T USBD_Composite_Init(USBD_INFO_T* usbInfo, void* itf1, void* itf2);
USBD_STA_T USBD_Composite_Deinit(USBD_INFO_T* usbInfo);
/**@} end of group OTGD_Composite_Functions */
/**@} end of group OTGD_Composite */
/**@} end of group Examples */

#ifdef __cplusplus
}
#endif

#endif  /* _USBD_COMPOSITE_H_ */
