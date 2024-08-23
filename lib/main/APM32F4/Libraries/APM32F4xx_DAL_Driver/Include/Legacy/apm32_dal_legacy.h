/**
  *
  * @file    apm32_dal_legacy.h
  * @brief   This file contains aliases definition for the DAL constants
  *          macros and functions maintained for legacy purpose.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2021 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32_DAL_LEGACY
#define APM32_DAL_LEGACY

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup DAL_CORTEX_Aliased_Defines DAL CORTEX Aliased Defines maintained for legacy purpose
  * @{
  */
#define __DAL_CORTEX_SYSTICKCLK_CONFIG DAL_SYSTICK_CLKSourceConfig

/**
  * @}
  */

/** @defgroup DAL_GPIO_Aliased_Macros DAL GPIO Aliased Macros maintained for legacy purpose
  * @{
  */

#if defined(APM32F4)
#define  GPIO_SPEED_LOW                           GPIO_SPEED_FREQ_LOW
#define  GPIO_SPEED_MEDIUM                        GPIO_SPEED_FREQ_MEDIUM
#define  GPIO_SPEED_FAST                          GPIO_SPEED_FREQ_HIGH
#define  GPIO_SPEED_HIGH                          GPIO_SPEED_FREQ_VERY_HIGH
#endif /* APM32F4 */

/**
  * @}
  */

/** @defgroup DAL_SD_Aliased_Macros DAL SD/MMC Aliased Macros maintained for legacy purpose
  * @{
  */
#define SDMMC_STATIC_FLAGS         SDIO_STATIC_FLAGS

 /**
  * @}
  */

/** @defgroup DAL_PPP_Aliased_Macros DAL PPP Aliased Macros maintained for legacy purpose
  * @{
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32_DAL_LEGACY */
