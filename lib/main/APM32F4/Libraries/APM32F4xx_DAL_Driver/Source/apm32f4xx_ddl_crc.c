/**
  *
  * @file    apm32f4xx_ddl_crc.c
  * @brief   CRC DDL module driver.
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
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_crc.h"
#include "apm32f4xx_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (CRC)

/** @addtogroup CRC_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRC_DDL_Exported_Functions
  * @{
  */

/** @addtogroup CRC_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize CRC registers (Registers restored to their default values).
  * @param  CRCx CRC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: CRC registers are de-initialized
  *          - ERROR: CRC registers are not de-initialized
  */
ErrorStatus DDL_CRC_DeInit(CRC_TypeDef *CRCx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_CRC_ALL_INSTANCE(CRCx));

  if (CRCx == CRC)
  {
    /* Force CRC reset */
    DDL_AHB1_GRP1_ForceReset(DDL_AHB1_GRP1_PERIPH_CRC);

    /* Release CRC reset */
    DDL_AHB1_GRP1_ReleaseReset(DDL_AHB1_GRP1_PERIPH_CRC);
  }
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (CRC) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
