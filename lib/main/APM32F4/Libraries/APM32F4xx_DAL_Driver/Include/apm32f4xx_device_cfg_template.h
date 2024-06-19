/**
 * @file        apm32f4xx_device_cfg.h
 *
 * @brief       This file provides all configuration support for device
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
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
#ifndef APM32F4XX_DEVICE_CFG_H
#define APM32F4XX_DEVICE_CFG_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes */
#include "apm32f4xx_dal.h"

/** @addtogroup Config
  @{
  */

/** @addtogroup Device_Config
  @{
  */


/** @defgroup Device_Config_Functions
  @{
*/

void DAL_DeviceConfig(void);
void DAL_SysClkConfig(void);
void DAL_RCM_PeripheralClkConfig(void);
void DAL_GPIO_Config(void);
void DAL_DMA_Config(void);
void DAL_NVIC_Config(void);

/**@} end of group Device_Config_Functions */
/**@} end of group Device_Config */
/**@} end of group Config */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4XX_DEVICE_CFG_H */
