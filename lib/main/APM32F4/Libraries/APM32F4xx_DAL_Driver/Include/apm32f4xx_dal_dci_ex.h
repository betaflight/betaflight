/**
  *
  * @file    apm32f4xx_dal_dci_ex.h
  * @brief   Header file of DCI Extension DAL module.
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
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_DCI_EX_H
#define APM32F4xx_DAL_DCI_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(APM32F407xx) || defined(APM32F417xx)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"


/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup DCIEx
  * @brief DCI DAL module driver
  * @{
  */  

/* Exported types ------------------------------------------------------------*/
/** @defgroup DCIEx_Exported_Types DCI Extended Exported Types
  * @{
  */
/** 
  * @brief   DCIEx Embedded Synchronisation CODE Init structure definition
  */ 
typedef struct
{
  uint8_t FrameStartCode; /*!< Specifies the code of the frame start delimiter. */
  uint8_t LineStartCode;  /*!< Specifies the code of the line start delimiter.  */
  uint8_t LineEndCode;    /*!< Specifies the code of the line end delimiter.    */
  uint8_t FrameEndCode;   /*!< Specifies the code of the frame end delimiter.   */
}DCI_CodesInitTypeDef;

/** 
  * @brief   DCI Init structure definition
  */  
typedef struct
{
  uint32_t  SynchroMode;                /*!< Specifies the Synchronization Mode: Hardware or Embedded.
                                             This parameter can be a value of @ref DCI_Synchronization_Mode   */

  uint32_t  PCKPolarity;                /*!< Specifies the Pixel clock polarity: Falling or Rising.
                                             This parameter can be a value of @ref DCI_PIXCK_Polarity         */

  uint32_t  VSPolarity;                 /*!< Specifies the Vertical synchronization polarity: High or Low.
                                             This parameter can be a value of @ref DCI_VSYNC_Polarity         */

  uint32_t  HSPolarity;                 /*!< Specifies the Horizontal synchronization polarity: High or Low.
                                             This parameter can be a value of @ref DCI_HSYNC_Polarity         */

  uint32_t  CaptureRate;                /*!< Specifies the frequency of frame capture: All, 1/2 or 1/4.
                                             This parameter can be a value of @ref DCI_Capture_Rate           */

  uint32_t  ExtendedDataMode;           /*!< Specifies the data width: 8-bit, 10-bit, 12-bit or 14-bit.
                                             This parameter can be a value of @ref DCI_Extended_Data_Mode     */

  DCI_CodesInitTypeDef SyncroCode;     /*!< Specifies the code of the frame start delimiter.                  */

  uint32_t JPEGMode;                    /*!< Enable or Disable the JPEG mode
                                             This parameter can be a value of @ref DCI_MODE_JPEG              */
}DCI_InitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define DCI_POSITION_ESCR_LSC     (uint32_t)DCI_ESYNCC_LSDC_Pos     /*!< Required left shift to set line start delimiter */
#define DCI_POSITION_ESCR_LEC     (uint32_t)DCI_ESYNCC_LEDC_Pos     /*!< Required left shift to set line end delimiter   */
#define DCI_POSITION_ESCR_FEC     (uint32_t)DCI_ESYNCC_FEDC_Pos     /*!< Required left shift to set frame end delimiter  */

/* Private macro -------------------------------------------------------------*/

/** @defgroup DCIEx_Private_Macros DCI Extended Private Macros
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
#endif /* APM32F407xx || APM32F417xx */


/**
  * @}
  */

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_DCI_H */
