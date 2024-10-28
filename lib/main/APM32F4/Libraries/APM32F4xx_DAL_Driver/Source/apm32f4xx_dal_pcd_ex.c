/**
  *
  * @file    apm32f4xx_dal_pcd_ex.c
  * @brief   PCD Extended DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Extended features functions
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

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup PCDEx PCDEx
  * @brief PCD Extended DAL module driver
  * @{
  */

#ifdef DAL_PCD_MODULE_ENABLED

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup PCDEx_Exported_Functions PCDEx Exported Functions
  * @{
  */

/** @defgroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @brief    PCDEx control functions
  *
@verbatim
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Update FIFO configuration

@endverbatim
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/**
  * @brief  Set Tx FIFO
  * @param  hpcd PCD handle
  * @param  fifo The number of Tx fifo
  * @param  size Fifo size
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size)
{
  uint8_t i;
  uint32_t Tx_Offset;

  /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows:
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

  Tx_Offset = hpcd->Instance->GRXFIFO;

  if (fifo == 0U)
  {
    hpcd->Instance->GTXFCFG = ((uint32_t)size << 16) | Tx_Offset;
  }
  else
  {
    Tx_Offset += (hpcd->Instance->GTXFCFG) >> 16;
    for (i = 0U; i < (fifo - 1U); i++)
    {
      Tx_Offset += (hpcd->Instance->DTXFIFO[i] >> 16);
    }

    /* Multiply Tx_Size by 2 to get higher performance */
    hpcd->Instance->DTXFIFO[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
  }

  return DAL_OK;
}

/**
  * @brief  Set Rx FIFO
  * @param  hpcd PCD handle
  * @param  size Size of Rx fifo
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size)
{
  hpcd->Instance->GRXFIFO = size;

  return DAL_OK;
}

#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/**
  * @brief  Send LPM message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval DAL status
  */
__weak void DAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCDEx_LPM_Callback could be implemented in the user file
   */
}

/**
  * @brief  Send BatteryCharging message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval DAL status
  */
__weak void DAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCDEx_BCD_Callback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* DAL_PCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
