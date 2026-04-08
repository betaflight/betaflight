/**
  *
  * @file    apm32f4xx_dal_flash_ex.c
  * @brief   Extended FLASH DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the FLASH extension peripheral:
  *           + Extended programming operations functions
  *
  @verbatim
  ==============================================================================
                   ##### Flash Extension features #####
  ==============================================================================

  [..] Comparing to other previous devices, the FLASH interface for APM32F411xx
       devices contains the following additional features

       (+) Capacity up to 2 Mbyte with dual bank architecture supporting read-while-write
           capability (RWW)
       (+) Dual bank memory organization
       (+) PCROP protection for all banks

                      ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to configure and program the FLASH memory
       of all APM32F411xx devices. It includes
      (#) FLASH Memory Erase functions:
           (++) Lock and Unlock the FLASH interface using DAL_FLASH_Unlock() and
                DAL_FLASH_Lock() functions
           (++) Erase function: Erase sector, erase all sectors
           (++) There are two modes of erase :
             (+++) Polling Mode using DAL_FLASHEx_Erase()
             (+++) Interrupt Mode using DAL_FLASHEx_Erase_IT()

      (#) Option Bytes Programming functions: Use DAL_FLASHEx_OBProgram() to :
           (++) Set/Reset the write protection
           (++) Set the Read protection Level
           (++) Set the BOR level
           (++) Program the user Option Bytes
      (#) Advanced Option Bytes Programming functions: Use DAL_FLASHEx_AdvOBProgram() to :
       (++) Extended space (bank 2) erase function
       (++) Full FLASH space (2 Mo) erase (bank 1 and bank 2)
       (++) Dual Boot activation
       (++) Write protection configuration for bank 2
       (++) PCROP protection configuration and control for both banks

  @endverbatim
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
  * The original code has been modified by Geehy Semiconductor.
  * Copyright (c) 2017 STMicroelectronics. Copyright (C) 2023-2025 Geehy Semiconductor.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup FLASHEx FLASHEx
  * @brief FLASH DAL Extension module driver
  * @{
  */

#ifdef DAL_FLASH_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup FLASHEx_Private_Constants
  * @{
  */
#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_POSITION_OB_UOB_BIT        FLASH_OBCS_UOB_Pos
#define FLASH_POSITION_OB_USERDATA0_BIT  FLASH_OBCS_DATA0_Pos
#define FLASH_POSITION_OB_USERDATA1_BIT  FLASH_OBCS_DATA1_Pos
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASHEx_Private_Variables
  * @{
  */
extern FLASH_ProcessTypeDef pFlash;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup FLASHEx_Private_Functions
  * @{
  */
/* Option bytes control */
static DAL_StatusTypeDef  FLASH_OB_RDP_LevelConfig(uint8_t Level);
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
static void               FLASH_MassErase(uint8_t VoltageRange, uint32_t Banks);
static DAL_StatusTypeDef  FLASH_OB_UserConfig(uint8_t Iwdt, uint8_t Stop, uint8_t Stdby);
static DAL_StatusTypeDef  FLASH_OB_BOR_LevelConfig(uint8_t Level);
static DAL_StatusTypeDef  FLASH_OB_EnableWRP(uint32_t WRPSector, uint32_t Banks);
static DAL_StatusTypeDef  FLASH_OB_DisableWRP(uint32_t WRPSector, uint32_t Banks);
static uint8_t            FLASH_OB_GetBOR(void);
#else
static void               FLASH_MassErase(uint32_t Banks);
static DAL_StatusTypeDef  FLASH_OB_UserConfig(uint8_t UserConfig);
static DAL_StatusTypeDef  FLASH_OB_ProgramData(uint32_t Address, uint8_t Data);
static DAL_StatusTypeDef  FLASH_OB_EnableWRP(uint32_t WriteProtectPage, uint32_t Banks);
static DAL_StatusTypeDef  FLASH_OB_DisableWRP(uint32_t WriteProtectPage, uint32_t Banks);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
static uint8_t            FLASH_OB_GetUser(void);
static uint32_t           FLASH_OB_GetWRP(void);
static uint8_t            FLASH_OB_GetRDP(void);

#if defined(APM32F411xx)
static DAL_StatusTypeDef  FLASH_OB_EnablePCROP(uint32_t Sector);
static DAL_StatusTypeDef  FLASH_OB_DisablePCROP(uint32_t Sector);
#endif /* APM32F411xx */

extern DAL_StatusTypeDef  FLASH_WaitForLastOperation(uint32_t Timeout);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup FLASHEx_Exported_Functions FLASHEx Exported Functions
  * @{
  */

/** @defgroup FLASHEx_Exported_Functions_Group1 Extended IO operation functions
 *  @brief   Extended IO operation functions
 *
@verbatim
 ===============================================================================
                ##### Extended programming operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the Extension FLASH
    programming operations.

@endverbatim
  * @{
  */
/**
  * @brief  Perform a mass erase or erase the specified FLASH memory sectors
  * @param[in]  pEraseInit pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  *
  * @param[out]  SectorError pointer to variable  that
  *         contains the configuration information on faulty sector in case of error
  *         (0xFFFFFFFFU means that all the sectors have been correctly erased)
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
{
  DAL_StatusTypeDef status = DAL_ERROR;
  uint32_t index = 0U;

  /* Process Locked */
  __DAL_LOCK(&pFlash);

  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_TYPEERASE(pEraseInit->TypeErase));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    /*Initialization of SectorError variable*/
    *SectorError = 0xFFFFFFFFU;

    if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE)
    {
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
      /*Mass erase to be done*/
      FLASH_MassErase((uint8_t) pEraseInit->VoltageRange, pEraseInit->Banks);
#else
      /*Mass erase to be done*/
      FLASH_MassErase(pEraseInit->Banks);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
      /* if the erase operation is completed, disable the MER Bit */
      FLASH->CTRL &= (~FLASH_MER_BIT);
#else
      /* if the erase operation is completed, disable the MER Bit */
      FLASH->CTRL2 &= (~FLASH_CTRL2_MASSERA);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
    }
    else
    {
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
      /* Check the parameters */
      ASSERT_PARAM(IS_FLASH_NBSECTORS(pEraseInit->NbSectors + pEraseInit->Sector));

      /* Erase by sector by sector to be done*/
      for (index = pEraseInit->Sector; index < (pEraseInit->NbSectors + pEraseInit->Sector); index++)
      {
        FLASH_Erase_Sector(index, (uint8_t) pEraseInit->VoltageRange);

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

        /* If the erase operation is completed, disable the SER and SNB Bits */
        CLEAR_BIT(FLASH->CTRL, (FLASH_CTRL_SERS | FLASH_CTRL_SNUM));

        if (status != DAL_OK)
        {
          /* In case of error, stop erase procedure and return the faulty sector*/
          *SectorError = index;
          break;
        }
      }
#else
    /* Page Erase is requested */
    /* Check the parameters */
    ASSERT_PARAM(IS_FLASH_PROGRAM_ADDRESS(pEraseInit->PageAddress));
    ASSERT_PARAM(IS_FLASH_NB_PAGES(pEraseInit->PageAddress, pEraseInit->NbPages));

      /* Erase page by page to be done*/
      for(index = pEraseInit->PageAddress;
          index < ((pEraseInit->NbPages * FLASH_PAGE_SIZE) + pEraseInit->PageAddress);
          index += FLASH_PAGE_SIZE)
      {
        FLASH_PageErase(index);

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

        /* If the erase operation is completed, disable the PER Bit */
        CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_PAGEERA);

        if (status != DAL_OK)
        {
          /* In case of error, stop erase procedure and return the faulty address */
          *SectorError = index;
          break;
        }
      }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
    }
    /* Flush the caches to be sure of the data consistency */
    FLASH_FlushCaches();
  }

  /* Process Unlocked */
  __DAL_UNLOCK(&pFlash);

  return status;
}

/**
  * @brief  Perform a mass erase or erase the specified FLASH memory sectors  with interrupt enabled
  * @param  pEraseInit pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process Locked */
  __DAL_LOCK(&pFlash);

  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_TYPEERASE(pEraseInit->TypeErase));

  /* Enable End of FLASH Operation interrupt */
  __DAL_FLASH_ENABLE_IT(FLASH_IT_EOP);

  /* Enable Error source interrupt */
  __DAL_FLASH_ENABLE_IT(FLASH_IT_ERR);

#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Clear pending flags (if any) */
  __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#else
  /* Clear pending flags (if any) */
  __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | FLASH_FLAG_WRPERR | \
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif /* APM32F403xx || APM32F402xx */

  if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE)
  {
    /*Mass erase to be done*/
    pFlash.ProcedureOnGoing = FLASH_PROC_MASSERASE;
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    pFlash.Bank = pEraseInit->Banks;
    FLASH_MassErase((uint8_t) pEraseInit->VoltageRange, pEraseInit->Banks);
#else
    FLASH_MassErase(pEraseInit->Banks);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
  }
  else
  {
    /* Erase by sector to be done*/

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    /* Check the parameters */
    ASSERT_PARAM(IS_FLASH_NBSECTORS(pEraseInit->NbSectors + pEraseInit->Sector));

    pFlash.ProcedureOnGoing = FLASH_PROC_SECTERASE;
    pFlash.NbSectorsToErase = pEraseInit->NbSectors;
    pFlash.Sector = pEraseInit->Sector;
    pFlash.VoltageForErase = (uint8_t)pEraseInit->VoltageRange;

    /*Erase 1st sector and wait for IT*/
    FLASH_Erase_Sector(pEraseInit->Sector, pEraseInit->VoltageRange);
#else
    /* Check the parameters */
    ASSERT_PARAM(IS_FLASH_PROGRAM_ADDRESS(pEraseInit->PageAddress));
    ASSERT_PARAM(IS_FLASH_NB_PAGES(pEraseInit->PageAddress, pEraseInit->NbPages));

    pFlash.ProcedureOnGoing = FLASH_PROC_PAGEERASE;
    pFlash.DataRemaining = pEraseInit->NbPages;
    pFlash.Address = pEraseInit->PageAddress;

    /*Erase 1st page and wait for IT*/
    FLASH_PageErase(pEraseInit->PageAddress);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
  }

  return status;
}

#if defined (APM32F403xx) || defined (APM32F402xx)
/**
  * @brief  Erases the FLASH option bytes.
  * @note   This functions erases all option bytes except the Read protection (RDP).
  *         The function @ref DAL_FLASH_Unlock() should be called before to unlock the FLASH interface
  *         The function @ref DAL_FLASH_OB_Unlock() should be called before to unlock the options bytes
  *         The function @ref DAL_FLASH_OB_Launch() should be called after to force the reload of the options bytes
  *         (system reset will occur)
  * @retval DAL status
  */

DAL_StatusTypeDef DAL_FLASHEx_OBErase(void)
{
  uint8_t rdptmp = OB_RDP_LEVEL_0;
  DAL_StatusTypeDef status = DAL_ERROR;

  /* Get the actual read protection Option Byte value */
  rdptmp = FLASH_OB_GetRDP();

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* If the previous operation is completed, proceed to erase the option bytes */
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBE);
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_STA);

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the erase operation is completed, disable the OPTER Bit */
    CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBE);

    if(status == DAL_OK)
    {
      /* Restore the last read protection Option Byte value */
      status = FLASH_OB_RDP_LevelConfig(rdptmp);
    }
  }

  /* Return the erase status */
  return status;
}

/**
  * @brief  Get the Option byte user data
  * @param  DATAAdress Address of the option byte DATA
  *          This parameter can be one of the following values:
  *            @arg @ref OB_DATA_ADDRESS_DATA0
  *            @arg @ref OB_DATA_ADDRESS_DATA1
  * @retval Value programmed in USER data
  */
uint32_t DAL_FLASHEx_OBGetUserData(uint32_t DATAAdress)
{
  uint32_t value = 0;

  if (DATAAdress == OB_DATA_ADDRESS_DATA0)
  {
    /* Get value programmed in OB USER Data0 */
    value = READ_BIT(FLASH->OBCS, FLASH_OBCS_DATA0) >> FLASH_POSITION_OB_USERDATA0_BIT;
  }
  else
  {
    /* Get value programmed in OB USER Data1 */
    value = READ_BIT(FLASH->OBCS, FLASH_OBCS_DATA1) >> FLASH_POSITION_OB_USERDATA1_BIT;
  }

  return value;
}

#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief   Program option bytes
  * @param   pOBInit pointer to an FLASH_OBInitStruct structure that
  *          contains the configuration information for the programming.
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit)
{
  DAL_StatusTypeDef status = DAL_ERROR;

  /* Process Locked */
  __DAL_LOCK(&pFlash);

  /* Check the parameters */
  ASSERT_PARAM(IS_OPTIONBYTE(pOBInit->OptionType));

  /* Write protection configuration */
  if ((pOBInit->OptionType & OPTIONBYTE_WRP) == OPTIONBYTE_WRP)
  {
    ASSERT_PARAM(IS_WRPSTATE(pOBInit->WRPState));
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    if (pOBInit->WRPState == OB_WRPSTATE_ENABLE)
    {
      /*Enable of Write protection on the selected Sector*/
      status = FLASH_OB_EnableWRP(pOBInit->WRPSector, pOBInit->Banks);
    }
    else
    {
      /*Disable of Write protection on the selected Sector*/
      status = FLASH_OB_DisableWRP(pOBInit->WRPSector, pOBInit->Banks);
    }
  }
#else
    if (pOBInit->WRPState == OB_WRPSTATE_ENABLE)
    {
      /*Enable of Write protection on the selected Sector*/
      status = FLASH_OB_EnableWRP(pOBInit->WRPPage, pOBInit->Banks);
    }
    else
    {
      /*Disable of Write protection on the selected Sector*/
      status = FLASH_OB_DisableWRP(pOBInit->WRPPage, pOBInit->Banks);
    }
  }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  /*Read protection configuration*/
  if ((pOBInit->OptionType & OPTIONBYTE_RDP) == OPTIONBYTE_RDP)
  {
    status = FLASH_OB_RDP_LevelConfig(pOBInit->RDPLevel);
  }

  /*USER  configuration*/
  if ((pOBInit->OptionType & OPTIONBYTE_USER) == OPTIONBYTE_USER)
  {
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    status = FLASH_OB_UserConfig(pOBInit->USERConfig & OB_IWDT_SW,
                                 pOBInit->USERConfig & OB_STOP_NO_RST,
                                 pOBInit->USERConfig & OB_STDBY_NO_RST);
#else
    status = FLASH_OB_UserConfig(pOBInit->USERConfig);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
  }

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  /*BOR Level  configuration*/
  if ((pOBInit->OptionType & OPTIONBYTE_BOR) == OPTIONBYTE_BOR)
  {
    status = FLASH_OB_BOR_LevelConfig(pOBInit->BORLevel);
  }
#else
  /* DATA configuration*/
  if((pOBInit->OptionType & OPTIONBYTE_DATA) == OPTIONBYTE_DATA)
  {
    status = FLASH_OB_ProgramData(pOBInit->DATAAddress, pOBInit->DATAData);
  }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  /* Process Unlocked */
  __DAL_UNLOCK(&pFlash);

  return status;
}

/**
  * @brief   Get the Option byte configuration
  * @param  pOBInit pointer to an FLASH_OBInitStruct structure that
  *         contains the configuration information for the programming.
  *
  * @retval None
  */
void DAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit)
{
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  pOBInit->OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;

  /*Get WRP*/
  pOBInit->WRPSector = (uint32_t)FLASH_OB_GetWRP();
#else
  pOBInit->OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER;

  /*Get WRP*/
  pOBInit->WRPPage = (uint32_t)FLASH_OB_GetWRP();
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  /*Get RDP Level*/
  pOBInit->RDPLevel = (uint32_t)FLASH_OB_GetRDP();

  /*Get USER*/
  pOBInit->USERConfig = (uint8_t)FLASH_OB_GetUser();

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  /*Get BOR Level*/
  pOBInit->BORLevel = (uint32_t)FLASH_OB_GetBOR();
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
}

#if defined(APM32F411xx)
/**
  * @brief   Program option bytes
  * @param  pAdvOBInit pointer to an FLASH_AdvOBProgramInitTypeDef structure that
  *         contains the configuration information for the programming.
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_AdvOBProgram(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit)
{
  DAL_StatusTypeDef status = DAL_ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_OBEX(pAdvOBInit->OptionType));

  /*Program PCROP option byte*/
  if (((pAdvOBInit->OptionType) & OPTIONBYTE_PCROP) == OPTIONBYTE_PCROP)
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_PCROPSTATE(pAdvOBInit->PCROPState));
    if ((pAdvOBInit->PCROPState) == OB_PCROP_STATE_ENABLE)
    {
      /*Enable of Write protection on the selected Sector*/
#if defined(APM32F411xx)
      status = FLASH_OB_EnablePCROP(pAdvOBInit->Sectors);
#endif /* APM32F411xx */
    }
    else
    {
      /*Disable of Write protection on the selected Sector*/
#if defined(APM32F411xx)
      status = FLASH_OB_DisablePCROP(pAdvOBInit->Sectors);
#endif /* APM32F411xx */
    }
  }

  return status;
}

/**
  * @brief   Get the OBEX byte configuration
  * @param  pAdvOBInit pointer to an FLASH_AdvOBProgramInitTypeDef structure that
  *         contains the configuration information for the programming.
  *
  * @retval None
  */
void DAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit)
{
#if defined(APM32F411xx)
  /*Get Sector*/
  pAdvOBInit->Sectors = (*(__IO uint16_t *)(OPTCTRL_BYTE2_ADDRESS));
#endif /* APM32F411xx */
}

/**
  * @brief  Select the Protection Mode
  *
  * @note   After PCROP activated Option Byte modification NOT POSSIBLE! excepted
  *         Global Read Out Protection modification (from level1 to level0)
  * @note   Once SPRMOD bit is active unprotection of a protected sector is not possible
  * @note   Read a protected sector will set RDERR Flag and write a protected sector will set WRPERR Flag
  * @note   This function can be used only for APM32F411xx devices.
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_OB_SelectPCROP(void)
{
  uint8_t optiontmp = 0xFF;

  /* Mask SPRMOD bit */
  optiontmp = (uint8_t)((*(__IO uint8_t *)OPTCTRL_BYTE3_ADDRESS) & (uint8_t)0x7F);

  /* Update Option Byte */
  *(__IO uint8_t *)OPTCTRL_BYTE3_ADDRESS = (uint8_t)(OB_PCROP_SELECTED | optiontmp);

  return DAL_OK;
}

/**
  * @brief  Deselect the Protection Mode
  *
  * @note   After PCROP activated Option Byte modification NOT POSSIBLE! excepted
  *         Global Read Out Protection modification (from level1 to level0)
  * @note   Once SPRMOD bit is active unprotection of a protected sector is not possible
  * @note   Read a protected sector will set RDERR Flag and write a protected sector will set WRPERR Flag
  * @note   This function can be used only for APM32F411xx devices.
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASHEx_OB_DeSelectPCROP(void)
{
  uint8_t optiontmp = 0xFF;

  /* Mask SPRMOD bit */
  optiontmp = (uint8_t)((*(__IO uint8_t *)OPTCTRL_BYTE3_ADDRESS) & (uint8_t)0x7F);

  /* Update Option Byte */
  *(__IO uint8_t *)OPTCTRL_BYTE3_ADDRESS = (uint8_t)(OB_PCROP_DESELECTED | optiontmp);

  return DAL_OK;
}
#endif /* APM32F411xx */

/**
  * @}
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx) || defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
/**
  * @brief  Mass erase of FLASH memory
  * @param  VoltageRange The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg FLASH_VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg FLASH_VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg FLASH_VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg FLASH_VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @param  Banks Banks to be erased
  *          This parameter can be one of the following values:
  *            @arg FLASH_BANK_1: Bank1 to be erased
  *
  * @retval None
  */
static void FLASH_MassErase(uint8_t VoltageRange, uint32_t Banks)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_VOLTAGERANGE(VoltageRange));
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  UNUSED(Banks);

  /* If the previous operation is completed, proceed to erase all sectors */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= FLASH_CTRL_MERS;
  FLASH->CTRL |= FLASH_CTRL_START | ((uint32_t)VoltageRange << 8U);
}

/**
  * @brief  Erase the specified FLASH memory sector
  * @param  Sector FLASH sector to erase
  *         The value of this parameter depend on device used within the same series
  * @param  VoltageRange The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg FLASH_VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg FLASH_VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg FLASH_VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg FLASH_VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval None
  */
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_SECTOR(Sector));
  ASSERT_PARAM(IS_VOLTAGERANGE(VoltageRange));

  if (VoltageRange == FLASH_VOLTAGE_RANGE_1)
  {
    tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if (VoltageRange == FLASH_VOLTAGE_RANGE_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if (VoltageRange == FLASH_VOLTAGE_RANGE_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }

  /* If the previous operation is completed, proceed to erase the sector */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= tmp_psize;
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_SNUM);
  FLASH->CTRL |= FLASH_CTRL_SERS | (Sector << FLASH_CTRL_SNUM_Pos);
  FLASH->CTRL |= FLASH_CTRL_START;
}

/**
  * @brief  Enable the write protection of the desired bank 1 sectors
  *
  * @note   When the memory read protection level is selected (RDP level = 1),
  *         it is not possible to program or erase the flash sector i if CortexM4
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1
  * @note   Active value of nWRPi bits is inverted when PCROP mode is active (SPRMOD =1).
  *
  * @param  WRPSector specifies the sector(s) to be write protected.
  *         The value of this parameter depend on device used within the same series
  *
  * @param  Banks Enable write protection on all the sectors for the specific bank
  *          This parameter can be one of the following values:
  *            @arg FLASH_BANK_1: WRP on all sectors of bank1
  *
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_EnableWRP(uint32_t WRPSector, uint32_t Banks)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_WRP_SECTOR(WRPSector));
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  UNUSED(Banks);

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    *(__IO uint16_t *)OPTCTRL_BYTE2_ADDRESS &= (~WRPSector);
  }

  return status;
}

/**
  * @brief  Disable the write protection of the desired bank 1 sectors
  *
  * @note   When the memory read protection level is selected (RDP level = 1),
  *         it is not possible to program or erase the flash sector i if CortexM4
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1
  * @note   Active value of nWRPi bits is inverted when PCROP mode is active (SPRMOD =1).
  *
  * @param  WRPSector specifies the sector(s) to be write protected.
  *         The value of this parameter depend on device used within the same series
  *
  * @param  Banks Enable write protection on all the sectors for the specific bank
  *          This parameter can be one of the following values:
  *            @arg FLASH_BANK_1: WRP on all sectors of bank1
  *
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_DisableWRP(uint32_t WRPSector, uint32_t Banks)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_WRP_SECTOR(WRPSector));
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  UNUSED(Banks);

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    *(__IO uint16_t *)OPTCTRL_BYTE2_ADDRESS |= (uint16_t)WRPSector;
  }

  return status;
}

/**
  * @brief  Program the FLASH User Option Byte: IWDT_SW / RST_STOP / RST_STDBY.
  * @param  Iwdt Selects the IWDT mode
  *          This parameter can be one of the following values:
  *            @arg OB_IWDT_SW: Software IWDT selected
  *            @arg OB_IWDT_HW: Hardware IWDT selected
  * @param  Stop Reset event when entering STOP mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STOP_NO_RST: No reset generated when entering in STOP
  *            @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  Stdby Reset event when entering Standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STDBY_NO_RST: No reset generated when entering in STANDBY
  *            @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_UserConfig(uint8_t Iwdt, uint8_t Stop, uint8_t Stdby)
{
  uint8_t optiontmp = 0xFF;
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_IWDT_SOURCE(Iwdt));
  ASSERT_PARAM(IS_OB_STOP_SOURCE(Stop));
  ASSERT_PARAM(IS_OB_STDBY_SOURCE(Stdby));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    /* Mask OPTLOCK, OPTSTRT, BOR_LEV and BFB2 bits */
    optiontmp = (uint8_t)((*(__IO uint8_t *)OPTCTRL_BYTE0_ADDRESS) & (uint8_t)0x1F);

    /* Update User Option Byte */
    *(__IO uint8_t *)OPTCTRL_BYTE0_ADDRESS = Iwdt | (uint8_t)(Stdby | (uint8_t)(Stop | ((uint8_t)optiontmp)));
  }

  return status;
}

/**
  * @brief  Set the BOR Level.
  * @param  Level specifies the Option Bytes BOR Reset Level.
  *          This parameter can be one of the following values:
  *            @arg OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *            @arg OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *            @arg OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *            @arg OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.1 V
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_BOR_LevelConfig(uint8_t Level)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_OB_BOR_LEVEL(Level));

  /* Set the BOR Level */
  *(__IO uint8_t *)OPTCTRL_BYTE0_ADDRESS &= (~FLASH_OPTCTRL_BORLVL);
  *(__IO uint8_t *)OPTCTRL_BYTE0_ADDRESS |= Level;

  return DAL_OK;

}

/**
  * @brief  Set the read protection level.
  * @param  Level specifies the read protection level.
  *          This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  *
  * @note WARNING: When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  *
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_RDP_LevelConfig(uint8_t Level)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_RDP_LEVEL(Level));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    *(__IO uint8_t *)OPTCTRL_BYTE1_ADDRESS = Level;
  }

  return status;
}

#endif /* APM32F40xxx || APM32F41xxx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

#if defined(APM32F411xx)
/**
  * @brief  Enable the read/write protection (PCROP) of the desired sectors.
  * @param  Sector specifies the sector(s) to be read/write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_PCROP: A value between OB_PCROP_Sector0 and OB_PCROP_Sector5
  *            @arg OB_PCROP_Sector_All
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_EnablePCROP(uint32_t Sector)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_PCROP(Sector));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    *(__IO uint16_t *)OPTCTRL_BYTE2_ADDRESS |= (uint16_t)Sector;
  }

  return status;
}

/**
  * @brief  Disable the read/write protection (PCROP) of the desired sectors.
  * @param  Sector specifies the sector(s) to be read/write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_PCROP: A value between OB_PCROP_Sector0 and OB_PCROP_Sector5
  *            @arg OB_PCROP_Sector_All
  * @retval DAL Status
  */
static DAL_StatusTypeDef FLASH_OB_DisablePCROP(uint32_t Sector)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_PCROP(Sector));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == DAL_OK)
  {
    *(__IO uint16_t *)OPTCTRL_BYTE2_ADDRESS &= (~Sector);
  }

  return status;

}
#endif /* APM32F411xx */

#if defined (APM32F403xx) || defined (APM32F402xx)
/**
  * @brief  Full erase of FLASH memory Bank
  * @param  Banks Banks to be erased
  *          This parameter can be one of the following values:
  *            @arg @ref FLASH_BANK_1 Bank1 to be erased
  *
  * @retval None
  */
static void FLASH_MassErase(uint32_t Banks)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  /* Clean the error context */
  pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

  /* Only bank1 will be erased*/
  SET_BIT(FLASH->CTRL2, FLASH_CTRL2_MASSERA);
  SET_BIT(FLASH->CTRL2, FLASH_CTRL2_STA);
}

/**
  * @brief  Erase the specified FLASH memory page
  * @param  PageAddress FLASH page to erase
  *         The value of this parameter depend on device used within the same series
  *
  * @retval None
  */
void FLASH_PageErase(uint32_t PageAddress)
{
  /* Clean the error context */
  pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

  /* Proceed to erase the page */
  SET_BIT(FLASH->CTRL2, FLASH_CTRL2_PAGEERA);
  WRITE_REG(FLASH->ADDR, PageAddress);
  SET_BIT(FLASH->CTRL2, FLASH_CTRL2_STA);
}

/**
  * @brief  Enable the write protection of the desired pages
  * @note   An option byte erase is done automatically in this function.
  * @note   When the memory read protection level is selected (RDP level = 1),
  *         it is not possible to program or erase the flash page i if
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1
  *
  * @param  WriteProtectPage specifies the page(s) to be write protected.
  *         The value of this parameter depend on device used within the same series
  * @retval HAL status
  */
static DAL_StatusTypeDef FLASH_OB_EnableWRP(uint32_t WriteProtectPage, uint32_t Banks)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint16_t WRP0_Data = 0xFFFF;
  uint16_t WRP1_Data = 0xFFFF;
  uint16_t WRP2_Data = 0xFFFF;
  uint16_t WRP3_Data = 0xFFFF;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_WRP(WriteProtectPage));
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  /* Get current write protected pages and the new pages to be protected ******/
  WriteProtectPage = (uint32_t)(~((~FLASH_OB_GetWRP()) | WriteProtectPage));

  WRP0_Data = (uint16_t)(WriteProtectPage & OB_WRP_PAGES0TO15MASK);
  WRP1_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES16TO31MASK) >> 8U);
  WRP2_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES32TO47MASK) >> 16U);
  WRP3_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES48TO127MASK) >> 24U);

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* To be able to write again option byte, need to perform a option byte erase */
    status = DAL_FLASHEx_OBErase();
    if (status == DAL_OK)
    {
      /* Enable write protection */
      SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);

      if(WRP0_Data != 0xFFU)
      {
        OB->WRP0 &= WRP0_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP1_Data != 0xFFU))
      {
        OB->WRP1 &= WRP1_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP2_Data != 0xFFU))
      {
        OB->WRP2 &= WRP2_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP3_Data != 0xFFU))
      {
        OB->WRP3 &= WRP3_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      /* if the program operation is completed, disable the OPTPG Bit */
      CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
    }
  }

  return status;
}

/**
  * @brief  Disable the write protection of the desired pages
  * @note   An option byte erase is done automatically in this function.
  * @note   When the memory read protection level is selected (RDP level = 1),
  *         it is not possible to program or erase the flash page i if
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1
  *
  * @param  WriteProtectPage specifies the page(s) to be write unprotected.
  *         The value of this parameter depend on device used within the same series
  * @retval DAL status
  */
static DAL_StatusTypeDef FLASH_OB_DisableWRP(uint32_t WriteProtectPage, uint32_t Banks)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint16_t WRP0_Data = 0xFFFF;
  uint16_t WRP1_Data = 0xFFFF;
  uint16_t WRP2_Data = 0xFFFF;
  uint16_t WRP3_Data = 0xFFFF;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_WRP(WriteProtectPage));
  ASSERT_PARAM(IS_FLASH_BANK(Banks));

  /* Get current write protected pages and the new pages to be unprotected ******/
  WriteProtectPage = (FLASH_OB_GetWRP() | WriteProtectPage);

  WRP0_Data = (uint16_t)(WriteProtectPage & OB_WRP_PAGES0TO15MASK);
  WRP1_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES16TO31MASK) >> 8U);
  WRP2_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES32TO47MASK) >> 16U);
  WRP3_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES48TO127MASK) >> 24U);

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* To be able to write again option byte, need to perform a option byte erase */
    status = DAL_FLASHEx_OBErase();
    if (status == DAL_OK)
    {
      /* Enable write protection */
      SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);

      if(WRP0_Data != 0xFFU)
      {
        OB->WRP0 |= WRP0_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP1_Data != 0xFFU))
      {
        OB->WRP1 |= WRP1_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP2_Data != 0xFFU))
      {
        OB->WRP2 |= WRP2_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      if((status == DAL_OK) && (WRP3_Data != 0xFFU))
      {
        OB->WRP3 |= WRP3_Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
      }

      /* if the program operation is completed, disable the OPTPG Bit */
      CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
    }
  }

  return status;
}

/**
  * @brief  Set the read protection level.
  * @param  ReadProtectLevel specifies the read protection level.
  *         This parameter can be one of the following values:
  *            @arg @ref OB_RDP_LEVEL_0 No protection
  *            @arg @ref OB_RDP_LEVEL_1 Read protection of the memory
  * @retval DAL status
  */
static DAL_StatusTypeDef FLASH_OB_RDP_LevelConfig(uint8_t ReadProtectLevel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_RDP_LEVEL(ReadProtectLevel));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* If the previous operation is completed, proceed to erase the option bytes */
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBE);
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_STA);

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the erase operation is completed, disable the OBE Bit */
    CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBE);

    if(status == DAL_OK)
    {
      /* Enable the Option Bytes Programming operation */
      SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);

      WRITE_REG(OB->READPROT, ReadProtectLevel);

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

      /* if the program operation is completed, disable the OBP Bit */
      CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
    }
  }

  return status;
}

/**
  * @brief  Program the FLASH User Option Byte.
  * @note   Programming of the OB should be performed only after an erase (otherwise PGERR occurs)
  * @param  UserConfig The FLASH User Option Bytes values FLASH_OBR_IWDG_SW(Bit0),
  *         FLASH_OBR_nRST_STOP(Bit1),FLASH_OBR_nRST_STDBY(Bit2),FLASHEx_Option_Bytes_nROM_SEL(Bit4).
  * @retval DAL status
  */
static DAL_StatusTypeDef FLASH_OB_UserConfig(uint8_t UserConfig)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_IWDT_SOURCE((UserConfig&OB_IWDT_SW)));
  ASSERT_PARAM(IS_OB_STOP_SOURCE((UserConfig&OB_STOP_NO_RST)));
  ASSERT_PARAM(IS_OB_STDBY_SOURCE((UserConfig&OB_STDBY_NO_RST)));
  ASSERT_PARAM(IS_OB_STDBY_SOURCE((UserConfig&OB_SEL_INFO)));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* Enable the Option Bytes Programming operation */
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);

    OB->UOB = (UserConfig | 0xE8U);

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* if the program operation is completed, disable the OPTPG Bit */
    CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
  }

  return status;
}

/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   The function @ref DAL_FLASH_Unlock() should be called before to unlock the FLASH interface
  *         The function @ref DAL_FLASH_OB_Unlock() should be called before to unlock the options bytes
  *         The function @ref DAL_FLASH_OB_Launch() should be called after to force the reload of the options bytes
  *         (system reset will occur)
  *         Programming of the OB should be performed only after an erase (otherwise PGERR occurs)
  * @param  Address specifies the address to be programmed.
  *         This parameter can be 0x1FFFF804 or 0x1FFFF806.
  * @param  Data specifies the data to be programmed.
  * @retval DAL status
  */
static DAL_StatusTypeDef FLASH_OB_ProgramData(uint32_t Address, uint8_t Data)
{
  DAL_StatusTypeDef status = DAL_ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_OB_DATA_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
    /* Clean the error context */
    pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

    /* Enables the Option Bytes Programming operation */
    SET_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
    *(__IO uint16_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the program operation is completed, disable the OPTPG Bit */
    CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBP);
  }
  /* Return the Option Byte Data Program Status */
  return status;
}

#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Return the FLASH User Option Byte value.
  @if APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx
  * @retval uint8_t FLASH User Option Bytes values: IWDT_SW(Bit5), RST_STOP(Bit6)
  *         and RST_STDBY(Bit7).
  @endif
  @if APM32F402/403xx
  * @retval uint8_t FLASH User Option Bytes values: IWDT_SW(Bit0), RST_STOP(Bit1),
  *         RST_STDBY(Bit2) and ROM_SEL(Bit4).
  @endif
  */
static uint8_t FLASH_OB_GetUser(void)
{
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  /* Return the User Option Byte */
  return ((uint8_t)(FLASH->OPTCTRL & 0xE0));
#else
  /* Return the User Option Byte */
  return (uint8_t)((READ_REG(FLASH->OBCS) & FLASH_OBCS_UOB) >> FLASH_POSITION_OB_UOB_BIT);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
}

/**
  * @brief  Return the FLASH Write Protection Option Bytes value.
  @if APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx
  * @retval uint16_t FLASH Write Protection Option Bytes value
  @endif
  @if APM32F402/403xx
  * @retval uint32_t FLASH Write Protection Option Bytes value
  @endif
  */
static uint32_t FLASH_OB_GetWRP(void)
{
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  /* Return the FLASH write protection Register value */
  return (*(__IO uint16_t *)(OPTCTRL_BYTE2_ADDRESS));
#else
  /* Return the FLASH write protection Register value */
  return (uint32_t)(READ_REG(FLASH->WRTPROT));
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
}

/**
  * @brief  Returns the FLASH Read Protection level.
  * @retval FLASH ReadOut Protection Status:
  *         This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection (*)
  */
static uint8_t FLASH_OB_GetRDP(void)
{
  uint8_t readstatus = OB_RDP_LEVEL_0;

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  if (*(__IO uint8_t *)(OPTCTRL_BYTE1_ADDRESS) == (uint8_t)OB_RDP_LEVEL_2)
  {
    readstatus = OB_RDP_LEVEL_2;
  }
  else if (*(__IO uint8_t *)(OPTCTRL_BYTE1_ADDRESS) == (uint8_t)OB_RDP_LEVEL_0)
  {
    readstatus = OB_RDP_LEVEL_0;
  }
  else
  {
    readstatus = OB_RDP_LEVEL_1;
  }
#else
  if (READ_BIT(FLASH->OBCS, FLASH_OBCS_READPROT) == FLASH_OBCS_READPROT)
  {
    readstatus = OB_RDP_LEVEL_1;
  }
  else
  {
    readstatus = OB_RDP_LEVEL_0;
  }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  return readstatus;
}

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
/**
  * @brief  Returns the FLASH BOR level.
  * @retval uint8_t The FLASH BOR level:
  *           - OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *           - OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *           - OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *           - OB_BOR_OFF   : Supply voltage ranges from 1.62 to 2.1 V
  */
static uint8_t FLASH_OB_GetBOR(void)
{
  /* Return the FLASH BOR level */
  return (uint8_t)(*(__IO uint8_t *)(OPTCTRL_BYTE0_ADDRESS) & (uint8_t)0x0C);
}
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @brief  Flush the instruction and data caches
  * @retval None
  */
void FLASH_FlushCaches(void)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Flush instruction cache  */
  if (READ_BIT(FLASH->CTRL1, FLASH_CTRL1_ICACHEEN) != RESET)
#else
  /* Flush instruction cache  */
  if (READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHEEN) != RESET)
#endif /* APM32F403xx || APM32F402xx */
  {
    /* Disable instruction cache  */
    __DAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    /* Reset instruction cache */
    __DAL_FLASH_INSTRUCTION_CACHE_RESET();
    /* Enable instruction cache */
    __DAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  }

#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Flush instruction cache  */
  if (READ_BIT(FLASH->CTRL1, FLASH_CTRL1_DCACHEEN) != RESET)
#else
  /* Flush data cache */
  if (READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHEEN) != RESET)
#endif /* APM32F403xx || APM32F402xx */
  {
    /* Disable data cache  */
    __DAL_FLASH_DATA_CACHE_DISABLE();
    /* Reset data cache */
    __DAL_FLASH_DATA_CACHE_RESET();
    /* Enable data cache */
    __DAL_FLASH_DATA_CACHE_ENABLE();
  }
}

/**
  * @}
  */

#endif /* DAL_FLASH_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

