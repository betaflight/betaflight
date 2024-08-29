/**
 * @file        apm32f4xx_device_cfg.c
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

/* Includes */
#include "apm32f4xx_device_cfg.h"

/** @addtogroup Config
  @{
  */

/** @addtogroup Device_Config
  @{
  */

/** @defgroup Device_Config_Functions
  @{
*/

/**
 * @brief   Device configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_DeviceConfig(void)
{
    /* Configure DAL library */
    DAL_Init();

    /* Configure system clock */
    DAL_SysClkConfig();

    /* Configure peripheral clock */
    DAL_RCM_PeripheralClkConfig();

    /* Configure GPIO */
    DAL_GPIO_Config();

    /* Configure DMA */
    DAL_DMA_Config();

    /* Configure peripheral */

    /* Configure NVIC */
    DAL_NVIC_Config();
}

/**
 * @brief   Device reset
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_DeviceReset(void)
{
    /* Reset DAL library */
    DAL_DeInit();

    /* Reset Peripheral */

    /* Reset service */
}

/**
 * @brief   System clock configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_SysClkConfig(void)
{

}

/**
 * @brief   Peripheral Clock configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_RCM_PeripheralClkConfig(void)
{
    /* Configure the Peripheral Clock */
}

/**
 * @brief   GPIO configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_GPIO_Config(void)
{
    /* Configure GPIO */
}

/**
 * @brief   DMA configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_DMA_Config(void)
{
    /* Configure DMA */
}

/**
 * @brief   NVIC configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_NVIC_Config(void)
{
    /* Configure NVIC */
}

/**
 * @brief     Error handler
 *
 * @param     None
 *
 * @retval    None
 */
void DAL_ErrorHandler(void)
{
    /* When the function is needed, this function 
       could be implemented in the user file
    */
    while(1)
    {
    }
}

/**
 * @brief   Assert failed handler
 *
 * @param   file :Pointer to the source file name
 *
 * @param   line :Error line source number
 *
 * @retval  None
 */
void AssertFailedHandler(uint8_t *file, uint32_t line)
{ 
    /* When the function is needed, this function 
       could be implemented in the user file
    */
    UNUSED(file);
    UNUSED(line);
    while(1)
    {
    }
}

/**@} end of group Device_Config_Functions */
/**@} end of group Device_Config */
/**@} end of group Config */
