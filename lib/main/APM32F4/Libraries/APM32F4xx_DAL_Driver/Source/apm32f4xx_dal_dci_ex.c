/**
  *
  * @file    apm32f4xx_dal_dci_ex.c
  * @brief   DCI Extension DAL module driver
  *          This file provides firmware functions to manage the following
  *          functionalities of DCI extension peripheral:
  *           + Extension features functions
  *
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  @verbatim
  ==============================================================================
               ##### DCI peripheral extension features  #####
  ==============================================================================

  [..] Comparing to other previous devices, the DCI interface for APM32F446xx
       devices contains the following additional features :

       (+) Support of Black and White cameras

                     ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to manage the Black and White feature

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
/** @defgroup DCIEx DCIEx
  * @brief DCI Extended DAL module driver
  * @{
  */

#ifdef DAL_DCI_MODULE_ENABLED

#if defined(APM32F407xx) || defined(APM32F417xx)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup DCIEx_Exported_Functions DCI Extended Exported Functions
  * @{
  */

/**
  * @}
  */

/** @addtogroup DCI_Exported_Functions_Group1 Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Initializes the DCI according to the specified
  *         parameters in the DCI_InitTypeDef and create the associated handle.
  * @param  hdcmi pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_Init(DCI_HandleTypeDef *hdcmi)
{
  /* Check the DCI peripheral state */
  if(hdcmi == NULL)
  {
     return DAL_ERROR;
  }

  /* Check function parameters */
  ASSERT_PARAM(IS_DCI_ALL_INSTANCE(hdcmi->Instance));
  ASSERT_PARAM(IS_DCI_PCKPOLARITY(hdcmi->Init.PCKPolarity));
  ASSERT_PARAM(IS_DCI_VSPOLARITY(hdcmi->Init.VSPolarity));
  ASSERT_PARAM(IS_DCI_HSPOLARITY(hdcmi->Init.HSPolarity));
  ASSERT_PARAM(IS_DCI_SYNCHRO(hdcmi->Init.SynchroMode));
  ASSERT_PARAM(IS_DCI_CAPTURE_RATE(hdcmi->Init.CaptureRate));
  ASSERT_PARAM(IS_DCI_EXTENDED_DATA(hdcmi->Init.ExtendedDataMode));
  ASSERT_PARAM(IS_DCI_MODE_JPEG(hdcmi->Init.JPEGMode));

  if(hdcmi->State == DAL_DCI_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hdcmi->Lock = DAL_UNLOCKED;
    /* Init the low level hardware */
    /* Init the DCI Callback settings */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    hdcmi->FrameEventCallback = DAL_DCI_FrameEventCallback; /* Legacy weak FrameEventCallback  */
    hdcmi->VsyncEventCallback = DAL_DCI_VsyncEventCallback; /* Legacy weak VsyncEventCallback  */
    hdcmi->LineEventCallback  = DAL_DCI_LineEventCallback;  /* Legacy weak LineEventCallback   */
    hdcmi->ErrorCallback      = DAL_DCI_ErrorCallback;      /* Legacy weak ErrorCallback       */

    if(hdcmi->MspInitCallback == NULL)
    {
      /* Legacy weak MspInit Callback        */
      hdcmi->MspInitCallback = DAL_DCI_MspInit;
    }
    /* Initialize the low level hardware (MSP) */
    hdcmi->MspInitCallback(hdcmi);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_DCI_MspInit(hdcmi);
#endif /* (USE_DAL_DCI_REGISTER_CALLBACKS) */
    DAL_DCI_MspInit(hdcmi);
  }

  /* Change the DCI state */
  hdcmi->State = DAL_DCI_STATE_BUSY;
  /* Configures the HS, VS, DE and PC polarity */
  hdcmi->Instance->CTRL &= ~(DCI_CTRL_PXCLKPOL | DCI_CTRL_HSYNCPOL  | DCI_CTRL_VSYNCPOL  | DCI_CTRL_EXDMOD_0 |\
                           DCI_CTRL_EXDMOD_1  | DCI_CTRL_FCRCFG_0 | DCI_CTRL_FCRCFG_1 | DCI_CTRL_JPGFM  |\
                           DCI_CTRL_ESYNCSEL
                           );
  hdcmi->Instance->CTRL |=  (uint32_t)(hdcmi->Init.SynchroMode | hdcmi->Init.CaptureRate |\
                                     hdcmi->Init.VSPolarity  | hdcmi->Init.HSPolarity  |\
                                     hdcmi->Init.PCKPolarity | hdcmi->Init.ExtendedDataMode |\
                                     hdcmi->Init.JPEGMode
                                     );
  if(hdcmi->Init.SynchroMode == DCI_SYNCHRO_EMBEDDED)
  {
    hdcmi->Instance->ESYNCC = (((uint32_t)hdcmi->Init.SyncroCode.FrameStartCode)    |
                             ((uint32_t)hdcmi->Init.SyncroCode.LineStartCode << DCI_POSITION_ESCR_LSC)|
                             ((uint32_t)hdcmi->Init.SyncroCode.LineEndCode << DCI_POSITION_ESCR_LEC) |
                             ((uint32_t)hdcmi->Init.SyncroCode.FrameEndCode << DCI_POSITION_ESCR_FEC));

  }

  /* Enable the Line, Vsync, Error and Overrun interrupts */
  __DAL_DCI_ENABLE_IT(hdcmi, DCI_IT_LINE | DCI_IT_VSYNC | DCI_IT_ERR | DCI_IT_OVR);

  /* Update error code */
  hdcmi->ErrorCode = DAL_DCI_ERROR_NONE;

  /* Initialize the DCI state*/
  hdcmi->State  = DAL_DCI_STATE_READY;

  return DAL_OK;
}

/**
  * @}
  */
#endif /* APM32F407xx || APM32F417xx */
#endif /* DAL_DCI_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
