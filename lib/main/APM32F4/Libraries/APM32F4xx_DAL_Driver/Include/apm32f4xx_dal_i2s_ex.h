/**
  *
  * @file    apm32f4xx_dal_i2s_ex.h
  * @brief   Header file of I2S DAL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_I2S_EX_H
#define APM32F4xx_DAL_I2S_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
#if defined(SPI_I2S_FULLDUPLEX_SUPPORT)
/** @addtogroup I2SEx I2SEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/** @defgroup I2SEx_Exported_Macros I2S Extended Exported Macros
  * @{
  */

#define I2SxEXT(__INSTANCE__) ((__INSTANCE__) == (SPI2)? (SPI_TypeDef *)(I2S2ext_BASE): (SPI_TypeDef *)(I2S3ext_BASE))

/** @brief  Enable or disable the specified I2SExt peripheral.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2SEXT_ENABLE(__HANDLE__) (I2SxEXT((__HANDLE__)->Instance)->I2SCFG |= SPI_I2SCFG_I2SEN)
#define __DAL_I2SEXT_DISABLE(__HANDLE__) (I2SxEXT((__HANDLE__)->Instance)->I2SCFG &= ~SPI_I2SCFG_I2SEN)

/** @brief  Enable or disable the specified I2SExt interrupts.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *        This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __DAL_I2SEXT_ENABLE_IT(__HANDLE__, __INTERRUPT__) (I2SxEXT((__HANDLE__)->Instance)->CTRL2 |= (__INTERRUPT__))
#define __DAL_I2SEXT_DISABLE_IT(__HANDLE__, __INTERRUPT__) (I2SxEXT((__HANDLE__)->Instance)->CTRL2 &= ~(__INTERRUPT__))

/** @brief  Checks if the specified I2SExt interrupt source is enabled or disabled.
  * @param  __HANDLE__ specifies the I2S Handle.
  *         This parameter can be I2S where x: 1, 2, or 3 to select the I2S peripheral.
  * @param  __INTERRUPT__ specifies the I2S interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_I2SEXT_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((I2SxEXT((__HANDLE__)->Instance)->CTRL2\
                                                                 & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks whether the specified I2SExt flag is set or not.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg I2S_FLAG_RXNE: Receive buffer not empty flag
  *            @arg I2S_FLAG_TXE: Transmit buffer empty flag
  *            @arg I2S_FLAG_UDR: Underrun flag
  *            @arg I2S_FLAG_OVR: Overrun flag
  *            @arg I2S_FLAG_FRE: Frame error flag
  *            @arg I2S_FLAG_CHSIDE: Channel Side flag
  *            @arg I2S_FLAG_BSY: Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_I2SEXT_GET_FLAG(__HANDLE__, __FLAG__) (((I2SxEXT((__HANDLE__)->Instance)->STS) & (__FLAG__)) == (__FLAG__))

/** @brief Clears the I2SExt OVR pending flag.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2SEXT_CLEAR_OVRFLAG(__HANDLE__) do{                                                 \
                                                   __IO uint32_t tmpreg_ovr = 0x00U;                \
                                                   tmpreg_ovr = I2SxEXT((__HANDLE__)->Instance)->DATA;\
                                                   tmpreg_ovr = I2SxEXT((__HANDLE__)->Instance)->STS;\
                                                   UNUSED(tmpreg_ovr);                              \
                                                  }while(0U)
/** @brief Clears the I2SExt UDR pending flag.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2SEXT_CLEAR_UDRFLAG(__HANDLE__) do{                                                 \
                                                   __IO uint32_t tmpreg_udr = 0x00U;                \
                                                   tmpreg_udr = I2SxEXT((__HANDLE__)->Instance)->STS;\
                                                   UNUSED(tmpreg_udr);                              \
                                                  }while(0U)
/** @brief Flush the I2S and I2SExt DR Registers.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2SEXT_FLUSH_RX_DR(__HANDLE__) do{                                                    \
                                                   __IO uint32_t tmpreg_dr = 0x00U;                 \
                                                   tmpreg_dr = I2SxEXT((__HANDLE__)->Instance)->DATA; \
                                                   tmpreg_dr = ((__HANDLE__)->Instance->DATA);        \
                                                   UNUSED(tmpreg_dr);                               \
                                                  }while(0U)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2SEx_Exported_Functions I2S Extended Exported Functions
  * @{
  */

/** @addtogroup I2SEx_Exported_Functions_Group1 I2S Extended IO operation functions
  * @{
  */

/* Extended features functions *************************************************/
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                               uint16_t Size);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                                uint16_t Size);
/* I2S IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA) */
void DAL_I2SEx_FullDuplex_IRQHandler(I2S_HandleTypeDef *hi2s);
void DAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void DAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @}
  */

#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* APM32F4xx_DAL_I2S_EX_H */

