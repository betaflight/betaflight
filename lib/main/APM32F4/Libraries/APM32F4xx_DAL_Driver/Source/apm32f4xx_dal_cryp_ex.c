/**
  *
  * @file    apm32f4xx_dal_cryp_ex.c
  * @brief   Extended CRYP DAL module driver
  *          This file provides firmware functions to manage the following
  *          functionalities of CRYP extension peripheral:
  *           + Extended AES processing functions
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
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The CRYP extension DAL driver can be used as follows:
    (#)After AES-GCM or AES-CCM  Encryption/Decryption user can start following API
       to get the  authentication messages :
      (##) DAL_CRYPEx_AESGCM_GenerateAuthTAG
      (##) DAL_CRYPEx_AESCCM_GenerateAuthTAG

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
#if defined (AES)  || defined (CRYP)
#if defined (CRYP_CTRL_ALGOMSEL_AES_GCM)|| defined (AES)
/** @defgroup CRYPEx CRYPEx
  * @brief CRYP Extension DAL module driver.
  * @{
  */


#ifdef DAL_CRYP_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup CRYPEx_Private_Defines
  * @{
  */
#if defined(AES)
#define CRYP_PHASE_INIT                              0x00000000U             /*!< GCM/GMAC (or CCM) init phase */
#define CRYP_PHASE_HEADER                            AES_CR_GCMPH_0          /*!< GCM/GMAC or CCM header phase */
#define CRYP_PHASE_PAYLOAD                           AES_CR_GCMPH_1          /*!< GCM(/CCM) payload phase   */
#define CRYP_PHASE_FINAL                             AES_CR_GCMPH            /*!< GCM/GMAC or CCM  final phase  */

#define CRYP_OPERATINGMODE_ENCRYPT                   0x00000000U             /*!< Encryption mode   */
#define CRYP_OPERATINGMODE_KEYDERIVATION             AES_CR_MODE_0           /*!< Key derivation mode  only used when performing ECB and CBC decryptions  */
#define CRYP_OPERATINGMODE_DECRYPT                   AES_CR_MODE_1           /*!< Decryption       */
#define CRYP_OPERATINGMODE_KEYDERIVATION_DECRYPT     AES_CR_MODE             /*!< Key derivation and decryption only used when performing ECB and CBC decryptions  */

#else /* CRYP */

#define CRYP_PHASE_INIT                 0x00000000U
#define CRYP_PHASE_HEADER               CRYP_CTRL_GCM_CCMPH_0
#define CRYP_PHASE_PAYLOAD              CRYP_CTRL_GCM_CCMPH_1
#define CRYP_PHASE_FINAL                CRYP_CTRL_GCM_CCMPH

#define CRYP_OPERATINGMODE_ENCRYPT      0x00000000U
#define CRYP_OPERATINGMODE_DECRYPT      CRYP_CTRL_ALGODIRSEL
#endif /* End AES or CRYP */

#define  CRYPEx_PHASE_PROCESS       0x02U     /*!< CRYP peripheral is in processing phase */
#define  CRYPEx_PHASE_FINAL         0x03U     /*!< CRYP peripheral is in final phase this is relevant only with CCM and GCM modes */

/*  CTR0 information to use in CCM algorithm */
#define CRYP_CCM_CTR0_0            0x07FFFFFFU
#define CRYP_CCM_CTR0_3            0xFFFFFF00U


/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/



/* Exported functions---------------------------------------------------------*/
/** @addtogroup CRYPEx_Exported_Functions
  * @{
  */

/** @defgroup CRYPEx_Exported_Functions_Group1 Extended AES processing functions
  *  @brief   Extended processing functions.
  *
@verbatim
  ==============================================================================
              ##### Extended AES processing functions #####
  ==============================================================================
    [..]  This section provides functions allowing to generate the authentication
          TAG in Polling mode
      (#)DAL_CRYPEx_AESGCM_GenerateAuthTAG
      (#)DAL_CRYPEx_AESCCM_GenerateAuthTAG
         they should be used after Encrypt/Decrypt operation.

@endverbatim
  * @{
  */


/**
  * @brief  generate the GCM authentication TAG.
  * @param  hcryp: pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  AuthTag: Pointer to the authentication buffer
  * @param  Timeout: Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CRYPEx_AESGCM_GenerateAuthTAG(CRYP_HandleTypeDef *hcryp, uint32_t *AuthTag, uint32_t Timeout)
{
  uint32_t tickstart;
  /* Assume first Init.HeaderSize is in words */
  uint64_t headerlength = (uint64_t)(hcryp->Init.HeaderSize) * 32U; /* Header length in bits */
  uint64_t inputlength = (uint64_t)hcryp->SizesSum * 8U; /* Input length in bits */
  uint32_t tagaddr = (uint32_t)AuthTag;

  /* Correct headerlength if Init.HeaderSize is actually in bytes */
  if (hcryp->Init.HeaderWidthUnit == CRYP_HEADERWIDTHUNIT_BYTE)
  {
    headerlength /= 4U;
  }

  if (hcryp->State == DAL_CRYP_STATE_READY)
  {
    /* Process locked */
    __DAL_LOCK(hcryp);

    /* Change the CRYP peripheral state */
    hcryp->State = DAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if (hcryp->Phase == CRYPEx_PHASE_PROCESS)
    {
      /* Change the CRYP phase */
      hcryp->Phase = CRYPEx_PHASE_FINAL;
    }
    else /* Initialization phase has not been performed*/
    {
      /* Disable the Peripheral */
      __DAL_CRYP_DISABLE(hcryp);

      /* Sequence error code field */
      hcryp->ErrorCode |= DAL_CRYP_ERROR_AUTH_TAG_SEQUENCE;

      /* Change the CRYP peripheral state */
      hcryp->State = DAL_CRYP_STATE_READY;

      /* Process unlocked */
      __DAL_UNLOCK(hcryp);
      return DAL_ERROR;
    }

#if defined(CRYP)

    /* Disable CRYP to start the final phase */
    __DAL_CRYP_DISABLE(hcryp);

    /* Select final phase */
    MODIFY_REG(hcryp->Instance->CTRL, CRYP_CTRL_GCM_CCMPH, CRYP_PHASE_FINAL);

    /*ALGODIR bit must be set to '0'.*/
    hcryp->Instance->CTRL &=  ~CRYP_CTRL_ALGODIRSEL;

    /* Enable the CRYP peripheral */
    __DAL_CRYP_ENABLE(hcryp);

    /* Write the number of bits in header (64 bits) followed by the number of bits
    in the payload */
    if (hcryp->Init.DataType == CRYP_DATATYPE_1B)
    {
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __RBIT((uint32_t)(headerlength));
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __RBIT((uint32_t)(inputlength));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_8B)
    {
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __REV((uint32_t)(headerlength));
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __REV((uint32_t)(inputlength));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_16B)
    {
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __ROR((uint32_t)headerlength, 16U);
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = __ROR((uint32_t)inputlength, 16U);
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_32B)
    {
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = (uint32_t)(headerlength);
      hcryp->Instance->DATAIN = 0U;
      hcryp->Instance->DATAIN = (uint32_t)(inputlength);
    }
    else
    {
      /* Nothing to do */
    }

    /* Wait for OFNE flag to be raised */
    tickstart = DAL_GetTick();
    while (DAL_IS_BIT_CLR(hcryp->Instance->STS, CRYP_FLAG_OFNE))
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Disable the CRYP Peripheral Clock */
          __DAL_CRYP_DISABLE(hcryp);

          /* Change state */
          hcryp->ErrorCode |= DAL_CRYP_ERROR_TIMEOUT;
          hcryp->State = DAL_CRYP_STATE_READY;

          /* Process unlocked */
          __DAL_UNLOCK(hcryp);
          return DAL_ERROR;
        }
      }
    }

    /* Read the authentication TAG in the output FIFO */
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;

#else /* AES*/

    /* Select final phase */
    MODIFY_REG(hcryp->Instance->CTRL, AES_CR_GCMPH, CRYP_PHASE_FINAL);

    /* Write the number of bits in header (64 bits) followed by the number of bits
    in the payload */
    if (hcryp->Init.DataType == CRYP_DATATYPE_1B)
    {
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __RBIT((uint32_t)(headerlength));
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __RBIT((uint32_t)(inputlength));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_8B)
    {
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __REV((uint32_t)(headerlength));
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __REV((uint32_t)(inputlength));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_16B)
    {
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __ROR((uint32_t)headerlength, 16U);
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = __ROR((uint32_t)inputlength, 16U);
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_32B)
    {
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = (uint32_t)(headerlength);
      hcryp->Instance->DATAINR = 0U;
      hcryp->Instance->DATAINR = (uint32_t)(inputlength);
    }
    else
    {
      /* Nothing to do */
    }
    /* Wait for CCF flag to be raised */
    tickstart = DAL_GetTick();
    while (DAL_IS_BIT_CLR(hcryp->Instance->STS, AES_SR_CCF))
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Disable the CRYP peripheral clock */
          __DAL_CRYP_DISABLE(hcryp);

          /* Change state */
          hcryp->ErrorCode |= DAL_CRYP_ERROR_TIMEOUT;
          hcryp->State = DAL_CRYP_STATE_READY;

          /* Process unlocked */
          __DAL_UNLOCK(hcryp);
          return DAL_ERROR;
        }
      }
    }

    /* Read the authentication TAG in the output FIFO */
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;

    /* Clear CCF flag */
    __DAL_CRYP_CLEAR_FLAG(hcryp, CRYP_CCF_CLEAR);

#endif /* End AES or CRYP */

    /* Disable the peripheral */
    __DAL_CRYP_DISABLE(hcryp);

    /* Change the CRYP peripheral state */
    hcryp->State = DAL_CRYP_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hcryp);
  }
  else
  {
    /* Busy error code field */
    hcryp->ErrorCode |= DAL_CRYP_ERROR_BUSY;
    return DAL_ERROR;
  }
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  AES CCM Authentication TAG generation.
  * @param  hcryp: pointer to a CRYP_HandleTypeDef structure that contains
  *         the configuration information for CRYP module
  * @param  AuthTag: Pointer to the authentication buffer
  * @param  Timeout: Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CRYPEx_AESCCM_GenerateAuthTAG(CRYP_HandleTypeDef *hcryp, uint32_t *AuthTag, uint32_t Timeout)
{
  uint32_t tagaddr = (uint32_t)AuthTag;
  uint32_t ctr0 [4] = {0};
  uint32_t ctr0addr = (uint32_t)ctr0;
  uint32_t tickstart;

  if (hcryp->State == DAL_CRYP_STATE_READY)
  {
    /* Process locked */
    __DAL_LOCK(hcryp);

    /* Change the CRYP peripheral state */
    hcryp->State = DAL_CRYP_STATE_BUSY;

    /* Check if initialization phase has already been performed */
    if (hcryp->Phase == CRYPEx_PHASE_PROCESS)
    {
      /* Change the CRYP phase */
      hcryp->Phase = CRYPEx_PHASE_FINAL;
    }
    else /* Initialization phase has not been performed*/
    {
      /* Disable the peripheral */
      __DAL_CRYP_DISABLE(hcryp);

      /* Sequence error code field */
      hcryp->ErrorCode |= DAL_CRYP_ERROR_AUTH_TAG_SEQUENCE;

      /* Change the CRYP peripheral state */
      hcryp->State = DAL_CRYP_STATE_READY;

      /* Process unlocked */
      __DAL_UNLOCK(hcryp);
      return DAL_ERROR;
    }

#if defined(CRYP)

    /* Disable CRYP to start the final phase */
    __DAL_CRYP_DISABLE(hcryp);

    /* Select final phase & ALGODIR bit must be set to '0'. */
    MODIFY_REG(hcryp->Instance->CTRL, CRYP_CTRL_GCM_CCMPH | CRYP_CTRL_ALGODIRSEL, CRYP_PHASE_FINAL | CRYP_OPERATINGMODE_ENCRYPT);

    /* Enable the CRYP peripheral */
    __DAL_CRYP_ENABLE(hcryp);

    /* Write the counter block in the IN FIFO, CTR0 information from B0
    data has to be swapped according to the DATATYPE*/
    ctr0[0] = (hcryp->Init.B0[0]) & CRYP_CCM_CTR0_0;
    ctr0[1] = hcryp->Init.B0[1];
    ctr0[2] = hcryp->Init.B0[2];
    ctr0[3] = hcryp->Init.B0[3] &  CRYP_CCM_CTR0_3;

    if (hcryp->Init.DataType == CRYP_DATATYPE_8B)
    {
      hcryp->Instance->DATAIN = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __REV(*(uint32_t *)(ctr0addr));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_16B)
    {
      hcryp->Instance->DATAIN = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __ROR(*(uint32_t *)(ctr0addr), 16U);
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_1B)
    {
      hcryp->Instance->DATAIN = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = __RBIT(*(uint32_t *)(ctr0addr));
    }
    else
    {
      hcryp->Instance->DATAIN = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAIN = *(uint32_t *)(ctr0addr);
    }
    /* Wait for OFNE flag to be raised */
    tickstart = DAL_GetTick();
    while (DAL_IS_BIT_CLR(hcryp->Instance->STS, CRYP_FLAG_OFNE))
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Disable the CRYP peripheral Clock */
          __DAL_CRYP_DISABLE(hcryp);

          /* Change state */
          hcryp->ErrorCode |= DAL_CRYP_ERROR_TIMEOUT;
          hcryp->State = DAL_CRYP_STATE_READY;

          /* Process unlocked */
          __DAL_UNLOCK(hcryp);
          return DAL_ERROR;
        }
      }
    }

    /* Read the Auth TAG in the IN FIFO */
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUT;

#else /* AES */

    /* Select final phase */
    MODIFY_REG(hcryp->Instance->CTRL, AES_CR_GCMPH, CRYP_PHASE_FINAL);

    /* Write the counter block in the IN FIFO, CTR0 information from B0
    data has to be swapped according to the DATATYPE*/
    if (hcryp->Init.DataType == CRYP_DATATYPE_8B)
    {
      ctr0[0] = (__REV(hcryp->Init.B0[0]) & CRYP_CCM_CTR0_0);
      ctr0[1] = __REV(hcryp->Init.B0[1]);
      ctr0[2] = __REV(hcryp->Init.B0[2]);
      ctr0[3] = (__REV(hcryp->Init.B0[3])& CRYP_CCM_CTR0_3);

      hcryp->Instance->DATAINR = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __REV(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __REV(*(uint32_t *)(ctr0addr));
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_16B)
    {
      ctr0[0] = (__ROR((hcryp->Init.B0[0]), 16U)& CRYP_CCM_CTR0_0);
      ctr0[1] =   __ROR((hcryp->Init.B0[1]), 16U);
      ctr0[2] =   __ROR((hcryp->Init.B0[2]), 16U);
      ctr0[3] = (__ROR((hcryp->Init.B0[3]), 16U)& CRYP_CCM_CTR0_3);

      hcryp->Instance->DATAINR = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __ROR(*(uint32_t *)(ctr0addr), 16U);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __ROR(*(uint32_t *)(ctr0addr), 16U);
    }
    else if (hcryp->Init.DataType == CRYP_DATATYPE_1B)
    {
      ctr0[0] = (__RBIT(hcryp->Init.B0[0])& CRYP_CCM_CTR0_0);
      ctr0[1] = __RBIT(hcryp->Init.B0[1]);
      ctr0[2] = __RBIT(hcryp->Init.B0[2]);
      ctr0[3] = (__RBIT(hcryp->Init.B0[3])& CRYP_CCM_CTR0_3);

      hcryp->Instance->DATAINR = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __RBIT(*(uint32_t *)(ctr0addr));
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = __RBIT(*(uint32_t *)(ctr0addr));
    }
    else
    {
      ctr0[0] = (hcryp->Init.B0[0]) & CRYP_CCM_CTR0_0;
      ctr0[1] = hcryp->Init.B0[1];
      ctr0[2] = hcryp->Init.B0[2];
      ctr0[3] = hcryp->Init.B0[3] &  CRYP_CCM_CTR0_3;

      hcryp->Instance->DATAINR = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = *(uint32_t *)(ctr0addr);
      ctr0addr += 4U;
      hcryp->Instance->DATAINR = *(uint32_t *)(ctr0addr);
    }

    /* Wait for CCF flag to be raised */
    tickstart = DAL_GetTick();
    while (DAL_IS_BIT_CLR(hcryp->Instance->STS, AES_SR_CCF))
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Disable the CRYP peripheral Clock */
          __DAL_CRYP_DISABLE(hcryp);

          /* Change state */
          hcryp->ErrorCode |= DAL_CRYP_ERROR_TIMEOUT;
          hcryp->State = DAL_CRYP_STATE_READY;

          /* Process unlocked */
          __DAL_UNLOCK(hcryp);
          return DAL_ERROR;
        }
      }
    }

    /* Read the authentication TAG in the output FIFO */
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;
    tagaddr += 4U;
    *(uint32_t *)(tagaddr) = hcryp->Instance->DATAOUTR;

    /* Clear CCF Flag */
    __DAL_CRYP_CLEAR_FLAG(hcryp, CRYP_CCF_CLEAR);

#endif /* End of AES || CRYP */

    /* Change the CRYP peripheral state */
    hcryp->State = DAL_CRYP_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hcryp);

    /* Disable CRYP  */
    __DAL_CRYP_DISABLE(hcryp);
  }
  else
  {
    /* Busy error code field */
    hcryp->ErrorCode = DAL_CRYP_ERROR_BUSY;
    return DAL_ERROR;
  }
  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

#if defined (AES)
/** @defgroup CRYPEx_Exported_Functions_Group2 Key Derivation functions
  *  @brief   AutoKeyDerivation functions
  *
@verbatim
  ==============================================================================
              ##### Key Derivation functions #####
  ==============================================================================
    [..]  This section provides functions allowing to Enable or Disable the
          the AutoKeyDerivation parameter in CRYP_HandleTypeDef structure
          These function are allowed only in TinyAES IP.

@endverbatim
  * @{
  */

/**
  * @brief  AES enable key derivation functions
  * @param  hcryp: pointer to a CRYP_HandleTypeDef structure.
  * @retval None
  */
void  DAL_CRYPEx_EnableAutoKeyDerivation(CRYP_HandleTypeDef *hcryp)
{
  if (hcryp->State == DAL_CRYP_STATE_READY)
  {
    hcryp->AutoKeyDerivation = ENABLE;
  }
  else
  {
    /* Busy error code field */
    hcryp->ErrorCode = DAL_CRYP_ERROR_BUSY;
  }
}
/**
  * @brief  AES disable key derivation functions
  * @param  hcryp: pointer to a CRYP_HandleTypeDef structure.
  * @retval None
  */
void  DAL_CRYPEx_DisableAutoKeyDerivation(CRYP_HandleTypeDef *hcryp)
{
  if (hcryp->State == DAL_CRYP_STATE_READY)
  {
    hcryp->AutoKeyDerivation = DISABLE;
  }
  else
  {
    /* Busy error code field */
    hcryp->ErrorCode = DAL_CRYP_ERROR_BUSY;
  }
}

/**
  * @}
  */
#endif /* AES or GCM CCM defined*/
#endif /* AES */
#endif /* DAL_CRYP_MODULE_ENABLED */

/**
  * @}
  */
#endif /* TinyAES or CRYP*/
/**
  * @}
  */

/**
  * @}
  */

