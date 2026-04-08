/**
  *
  * @file    apm32f4xx_dal_flash.c
  * @brief   FLASH DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the internal FLASH memory:
  *           + Program operations functions
  *           + Memory Control functions
  *           + Peripheral Errors functions
  *
  @verbatim
  ==============================================================================
                        ##### FLASH peripheral features #####
  ==============================================================================

  [..] The Flash memory interface manages CPU AHB I-Code and D-Code accesses
       to the Flash memory. It implements the erase and program Flash memory operations
       and the read and write protection mechanisms.

  [..] The Flash memory interface accelerates code execution with a system of instruction
       prefetch and cache lines.

  [..] The FLASH main features are:
      (+) Flash memory read operations
      (+) Flash memory program/erase operations
      (+) Read / write protections
      (+) Prefetch on I-Code
      (+) 64 cache lines of 128 bits on I-Code
      (+) 8 cache lines of 128 bits on D-Code


                     ##### How to use this driver #####
  ==============================================================================
    [..]
      This driver provides functions and macros to configure and program the FLASH
      memory of all APM32F4xx devices.

      (#) FLASH Memory IO Programming functions:
           (++) Lock and Unlock the FLASH interface using DAL_FLASH_Unlock() and
                DAL_FLASH_Lock() functions
           (++) Program functions: byte, half word, word and double word
           (++) There Two modes of programming :
            (+++) Polling mode using DAL_FLASH_Program() function
            (+++) Interrupt mode using DAL_FLASH_Program_IT() function

      (#) Interrupts and flags management functions :
           (++) Handle FLASH interrupts by calling DAL_FLASH_IRQHandler()
           (++) Wait for last FLASH operation according to its status
           (++) Get error flag status by calling DAL_SetErrorCode()

    [..]
      In addition to these functions, this driver includes a set of macros allowing
      to handle the following operations:
       (+) Set the latency
       (+) Enable/Disable the prefetch buffer
       (+) Enable/Disable the Instruction cache and the Data cache
       (+) Reset the Instruction cache and the Data cache
       (+) Enable/Disable the FLASH interrupts
       (+) Monitor the FLASH flags status

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

/** @defgroup FLASH FLASH
  * @brief FLASH DAL module driver
  * @{
  */

#ifdef DAL_FLASH_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants
  * @{
  */
#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Variables
  * @{
  */
/* Variable used for Erase sectors under interruption */
FLASH_ProcessTypeDef pFlash;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup FLASH_Private_Functions
  * @{
  */
/* Program operations */
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
static void   FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);
static void   FLASH_Program_Word(uint32_t Address, uint32_t Data);
static void   FLASH_Program_Byte(uint32_t Address, uint8_t Data);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
static void   FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static void   FLASH_SetErrorCode(void);

DAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
  * @{
  */

/** @defgroup FLASH_Exported_Functions_Group1 Programming operation functions
 *  @brief   Programming operation functions
 *
@verbatim
 ===============================================================================
                  ##### Programming operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the FLASH
    program operations.

@endverbatim
  * @{
  */

/**
  * @brief  Program byte, halfword, word or double word at a specified address
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
  *
  * @retval DAL_StatusTypeDef DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  DAL_StatusTypeDef status = DAL_ERROR;

  /* Process Locked */
  __DAL_LOCK(&pFlash);

  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_TYPEPROGRAM(TypeProgram));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if(status == DAL_OK)
  {
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
    {
      /*Program byte (8-bit) at a specified address.*/
      FLASH_Program_Byte(Address, (uint8_t) Data);
    }
    else if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
    {
      /*Program halfword (16-bit) at a specified address.*/
      FLASH_Program_HalfWord(Address, (uint16_t) Data);
    }
    else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
    {
      /*Program word (32-bit) at a specified address.*/
      FLASH_Program_Word(Address, (uint32_t) Data);
    }
    else
    {
      /*Program double word (64-bit) at a specified address.*/
      FLASH_Program_DoubleWord(Address, Data);
    }

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the program operation is completed, disable the PG Bit */
    FLASH->CTRL &= (~FLASH_CTRL_PG);
#else
    uint8_t index = 0;
    uint8_t nbiterations = 0;

    ASSERT_PARAM(IS_FLASH_PROGRAM_ADDRESS(Address));

    if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
    {
      /* Program halfword (16-bit) at a specified address. */
      nbiterations = 1U;
    }
    else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
    {
      /* Program word (32-bit = 2*16-bit) at a specified address. */
      nbiterations = 2U;
    }
    else
    {
      /* Program double word (64-bit = 4*16-bit) at a specified address. */
      nbiterations = 4U;
    }

    for (index = 0U; index < nbiterations; index++)
    {
      FLASH_Program_HalfWord((Address + (2U*index)), (uint16_t)(Data >> (16U*index)));

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

      /* If the program operation is completed, disable the PG Bit */
      CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_PG);
    }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
  }

  /* Process Unlocked */
  __DAL_UNLOCK(&pFlash);

  return status;
}

/**
  * @brief   Program byte, halfword, word or double word at a specified address  with interrupt enabled.
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
  *
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process Locked */
  __DAL_LOCK(&pFlash);

  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_TYPEPROGRAM(TypeProgram));

  /* Enable End of FLASH Operation interrupt */
  __DAL_FLASH_ENABLE_IT(FLASH_IT_EOP);

  /* Enable Error source interrupt */
  __DAL_FLASH_ENABLE_IT(FLASH_IT_ERR);

  pFlash.Address = Address;

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAM;

  if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
  {
    /*Program byte (8-bit) at a specified address.*/
      FLASH_Program_Byte(Address, (uint8_t) Data);
  }
  else if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
  {
    /*Program halfword (16-bit) at a specified address.*/
    FLASH_Program_HalfWord(Address, (uint16_t) Data);
  }
  else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
  {
    /*Program word (32-bit) at a specified address.*/
    FLASH_Program_Word(Address, (uint32_t) Data);
  }
  else
  {
    /*Program double word (64-bit) at a specified address.*/
    FLASH_Program_DoubleWord(Address, Data);
  }
#else
  pFlash.Data = Data;

  if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMHALFWORD;
    /* Program halfword (16-bit) at a specified address. */
    pFlash.DataRemaining = 1U;
  }
  else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMWORD;
    /* Program word (32-bit : 2*16-bit) at a specified address. */
    pFlash.DataRemaining = 2U;
  }
  else
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMDOUBLEWORD;
    /* Program double word (64-bit : 4*16-bit) at a specified address. */
    pFlash.DataRemaining = 4U;
  }

  /* Program halfword (16-bit) at a specified address. */
  FLASH_Program_HalfWord(Address, (uint16_t)Data);
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  return status;
}

/**
  * @brief This function handles FLASH interrupt request.
  * @retval None
  */
void DAL_FLASH_IRQHandler(void)
{
  uint32_t addresstmp = 0U;

  /* Check FLASH operation error flags */
#if defined(FLASH_STS_RPROERR)
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
    FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#elif defined(FLASH_FLAG_PGAERR) || defined(FLASH_FLAG_PGPERR) || defined(FLASH_FLAG_PGSERR)
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
    FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
#else
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR)) != RESET)
#endif /* FLASH_STS_RPROERR */
  {
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    if(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
    {
      /*return the faulty sector*/
      addresstmp = pFlash.Sector;
      pFlash.Sector = 0xFFFFFFFFU;
    }
    else if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
    {
      /*return the faulty bank*/
      addresstmp = pFlash.Bank;
    }
    else
    {
      /*return the faulty address*/
      addresstmp = pFlash.Address;
    }
#else
    /* Return the faulty address */
    addresstmp = pFlash.Address;
    /* Reset address */
    pFlash.Address = 0xFFFFFFFFU;
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

    /*Save the Error code*/
    FLASH_SetErrorCode();

    /* FLASH error interrupt user callback */
    DAL_FLASH_OperationErrorCallback(addresstmp);

    /*Stop the procedure ongoing*/
    pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
  }

  /* Check FLASH End of Operation flag  */
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_EOP) != RESET)
  {
    /* Clear FLASH End of Operation pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
    if(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
    {
      /*Nb of sector to erased can be decreased*/
      pFlash.NbSectorsToErase--;

      /* Check if there are still sectors to erase*/
      if(pFlash.NbSectorsToErase != 0U)
      {
        addresstmp = pFlash.Sector;
        /*Indicate user which sector has been erased*/
        DAL_FLASH_EndOfOperationCallback(addresstmp);

        /*Increment sector number*/
        pFlash.Sector++;
        addresstmp = pFlash.Sector;
        FLASH_Erase_Sector(addresstmp, pFlash.VoltageForErase);
      }
      else
      {
        /*No more sectors to Erase, user callback can be called.*/
        /*Reset Sector and stop Erase sectors procedure*/
        pFlash.Sector = addresstmp = 0xFFFFFFFFU;
        pFlash.ProcedureOnGoing = FLASH_PROC_NONE;

        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches() ;

        /* FLASH EOP interrupt user callback */
        DAL_FLASH_EndOfOperationCallback(addresstmp);
      }
    }
#else
    if(pFlash.ProcedureOnGoing == FLASH_PROC_PAGEERASE)
    {
      /* Nb of pages to erased can be decreased */
      pFlash.DataRemaining--;

      /* Check if there are still pages to erase */
      if(pFlash.DataRemaining != 0U)
      {
        addresstmp = pFlash.Address;

        /*Indicate user which sector has been erased */
        DAL_FLASH_EndOfOperationCallback(addresstmp);

        /*Increment sector number*/
        addresstmp = pFlash.Address + FLASH_PAGE_SIZE;
        pFlash.Address = addresstmp;

        /* If the erase operation is completed, disable the PAGEERA Bit */
        CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_PAGEERA);
        FLASH_PageErase(addresstmp);
      }
      else
      {
        /* No more pages to Erase, user callback can be called. */
        /* Reset Sector and stop Erase pages procedure */
        pFlash.Address = addresstmp = 0xFFFFFFFFU;
        pFlash.ProcedureOnGoing = FLASH_PROC_NONE;

        /* FLASH EOP interrupt user callback */
        DAL_FLASH_EndOfOperationCallback(addresstmp);
      }
    }
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
    else
    {
#if defined (APM32F403xx) || defined (APM32F402xx)
      if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
      {
        /* FLASH EOP interrupt user callback */
        DAL_FLASH_EndOfOperationCallback(0U);

        pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
      }
      else
      {
        /* Nb of 16-bit data to program can be decreased */
        pFlash.DataRemaining--;

        /* Check if there are still 16-bit data to program */
        if(pFlash.DataRemaining != 0U)
        {
          /* Increment address to 16-bit */
          pFlash.Address += 2U;
          addresstmp = pFlash.Address;

          /* Shift to have next 16-bit data */
          pFlash.Data = (pFlash.Data >> 16U);

          /* Operation is completed, disable the PG Bit */
          CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_PG);

          /*Program halfword (16-bit) at a specified address.*/
          FLASH_Program_HalfWord(addresstmp, (uint16_t)pFlash.Data);
        }
        else
        {
          /* Program ended. Return the selected address */
          /* FLASH EOP interrupt user callback */
          if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMHALFWORD)
          {
            DAL_FLASH_EndOfOperationCallback(pFlash.Address);
          }
          else if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMWORD)
          {
            DAL_FLASH_EndOfOperationCallback(pFlash.Address - 2U);
          }
          else
          {
            DAL_FLASH_EndOfOperationCallback(pFlash.Address - 6U);
          }

          /* Reset Address and stop Program procedure */
          pFlash.Address = 0xFFFFFFFFU;
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
        }
      }
#else
      if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
      {
        /* MassErase ended. Return the selected bank */
        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches() ;

        /* FLASH EOP interrupt user callback */
        DAL_FLASH_EndOfOperationCallback(pFlash.Bank);
      }
      else
      {
        /*Program ended. Return the selected address*/
        /* FLASH EOP interrupt user callback */
        DAL_FLASH_EndOfOperationCallback(pFlash.Address);
      }
      pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
#endif /* APM32F403xx || APM32F402xx */
    }
  }

  if(pFlash.ProcedureOnGoing == FLASH_PROC_NONE)
  {
#if defined(APM32F403xx) || defined(APM32F402xx)
    /* Operation is completed, disable the PG, PAGEERA and MASSERA Bits */
    CLEAR_BIT(FLASH->CTRL2, (FLASH_CTRL2_PG | FLASH_CTRL2_PAGEERA | FLASH_CTRL2_MASSERA));
#else
    /* Operation is completed, disable the PG, SERS, SNUM and MER Bits */
    CLEAR_BIT(FLASH->CTRL, (FLASH_CTRL_PG | FLASH_CTRL_SERS | FLASH_CTRL_SNUM | FLASH_MER_BIT));
#endif /* APM32F403 || APM32F402xx */

    /* Disable End of FLASH Operation interrupt */
    __DAL_FLASH_DISABLE_IT(FLASH_IT_EOP);

    /* Disable Error source interrupt */
    __DAL_FLASH_DISABLE_IT(FLASH_IT_ERR);

    /* Process Unlocked */
    __DAL_UNLOCK(&pFlash);
  }
}

/**
  * @brief  FLASH end of operation interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                  Mass Erase: Bank number which has been requested to erase
  *                  Sectors Erase: Sector which has been erased
  *                    (if 0xFFFFFFFFU, it means that all the selected sectors have been erased)
  *                  Program: Address which was selected for data program
  * @retval None
  */
__weak void DAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_FLASH_EndOfOperationCallback could be implemented in the user file
   */
}

/**
  * @brief  FLASH operation error interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                 Mass Erase: Bank number which has been requested to erase
  *                 Sectors Erase: Sector number which returned an error
  *                 Program: Address which was selected for data program
  * @retval None
  */
__weak void DAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_FLASH_OperationErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group2 Peripheral Control functions
 *  @brief   management functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the FLASH
    memory operations.

@endverbatim
  * @{
  */

/**
  * @brief  Unlock the FLASH control register access
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_Unlock(void)
{
  DAL_StatusTypeDef status = DAL_OK;

#if defined(APM32F403xx) || defined(APM32F402xx)
  if(READ_BIT(FLASH->CTRL2, FLASH_CTRL2_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEY, FLASH_KEY1);
    WRITE_REG(FLASH->KEY, FLASH_KEY2);

    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CTRL2, FLASH_CTRL2_LOCK) != RESET)
    {
      status = DAL_ERROR;
    }
  }
#else
  if(READ_BIT(FLASH->CTRL, FLASH_CTRL_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEY, FLASH_KEY1);
    WRITE_REG(FLASH->KEY, FLASH_KEY2);

    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CTRL, FLASH_CTRL_LOCK) != RESET)
    {
      status = DAL_ERROR;
    }
  }
#endif /* APM32F403xx || APM32F402xx */

  return status;
}

/**
  * @brief  Locks the FLASH control register access
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_Lock(void)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CTRL2 |= FLASH_CTRL2_LOCK;
#else
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CTRL |= FLASH_CTRL_LOCK;
#endif /* APM32F403xx || APM32F402xx */

  return DAL_OK;
}

/**
  * @brief  Unlock the FLASH Option Control Registers access.
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_OB_Unlock(void)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  if((FLASH->CTRL2 & FLASH_CTRL2_OBWEN) != RESET)
#else
  if((FLASH->OPTCTRL & FLASH_OPTCTRL_OPTLOCK) != RESET)
#endif /* APM32F403xx || APM32F402xx */
  {
    /* Authorizes the Option Byte register programming */
    FLASH->OPTKEY = FLASH_OPT_KEY1;
    FLASH->OPTKEY = FLASH_OPT_KEY2;
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Lock the FLASH Option Control Registers access.
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_OB_Lock(void)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Clear the OPTWRE Bit to lock the FLASH Option Byte Registers access */
  CLEAR_BIT(FLASH->CTRL2, FLASH_CTRL2_OBWEN);
#else
  /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
  FLASH->OPTCTRL |= FLASH_OPTCTRL_OPTLOCK;
#endif /* APM32F403xx || APM32F402xx */

  return DAL_OK;
}

#if defined(APM32F403xx) || defined(APM32F402xx)
/**
  * @brief  Launch the option byte loading.
  * @retval None
  */
void DAL_FLASH_OB_Launch(void)
{
  /* Initiates a system reset request to launch the option byte loading */
  DAL_NVIC_SystemReset();
}
#else
/**
  * @brief  Launch the option byte loading.
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_FLASH_OB_Launch(void)
{
  /* Set the OPTSTRT bit in OPTCR register */
  *(__IO uint8_t *)OPTCTRL_BYTE0_ADDRESS |= FLASH_OPTCTRL_OPTSTART;

  /* Wait for last operation to be completed */
  return(FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE));
}
#endif /* APM32F403xx || APM32F402xx */

/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief   Peripheral Errors functions
 *
@verbatim
 ===============================================================================
                ##### Peripheral Errors functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time Errors of the FLASH peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Get the specific FLASH error flag.
  * @retval FLASH_ErrorCode: The returned value can be a combination of:
  *            @arg DAL_FLASH_ERROR_RD: FLASH Read Protection error flag (PCROP)
  *            @arg DAL_FLASH_ERROR_PGS: FLASH Programming Sequence error flag
  *            @arg DAL_FLASH_ERROR_PGP: FLASH Programming Parallelism error flag
  *            @arg DAL_FLASH_ERROR_PGA: FLASH Programming Alignment error flag
  *            @arg DAL_FLASH_ERROR_WRP: FLASH Write protected error flag
  *            @arg DAL_FLASH_ERROR_OPERATION: FLASH operation Error flag
  */
uint32_t DAL_FLASH_GetError(void)
{
   return pFlash.ErrorCode;
}

/**
  * @}
  */

/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout maximum flash operationtimeout
  * @retval DAL Status
  */
DAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Clear Error Code */
  pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  /* Get tick */
  tickstart = DAL_GetTick();

  while(__DAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
  {
    if(Timeout != DAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((DAL_GetTick() - tickstart ) > Timeout))
      {
        return DAL_TIMEOUT;
      }
    }
  }

  /* Check FLASH End of Operation flag  */
  if (__DAL_FLASH_GET_FLAG(FLASH_FLAG_EOP) != RESET)
  {
    /* Clear FLASH End of Operation pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }
#if defined(FLASH_STS_RPROERR)
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#elif defined(FLASH_FLAG_PGAERR) || defined(FLASH_FLAG_PGPERR) || defined(FLASH_FLAG_PGSERR)
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
#else
  if(__DAL_FLASH_GET_FLAG((FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR )) ||
     __DAL_FLASH_GET_FLAG((FLASH_FLAG_OPTVERR)))
#endif /* FLASH_STS_RPROERR */
  {
    /*Save the error code*/
    FLASH_SetErrorCode();
    return DAL_ERROR;
  }

  /* If there is no error flag set */
  return DAL_OK;
}

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
/**
  * @brief  Program a double word (64-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V and Vpp in the range 7V to 9V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_ADDRESS(Address));

  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= FLASH_PSIZE_DOUBLE_WORD;
  FLASH->CTRL |= FLASH_CTRL_PG;

  /* Program first word */
  *(__IO uint32_t*)Address = (uint32_t)Data;

  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();

  /* Program second word */
  *(__IO uint32_t*)(Address+4) = (uint32_t)(Data >> 32);
}

/**
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Word(uint32_t Address, uint32_t Data)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_ADDRESS(Address));

  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= FLASH_PSIZE_WORD;
  FLASH->CTRL |= FLASH_CTRL_PG;

  *(__IO uint32_t*)Address = Data;
}
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.1V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
  /* Check the parameters */
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  ASSERT_PARAM(IS_FLASH_ADDRESS(Address));
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

#if defined(APM32F403xx) || defined(APM32F402xx)
  /* Clean the error context */
  pFlash.ErrorCode = DAL_FLASH_ERROR_NONE;

  /* Proceed to program the new data */
  SET_BIT(FLASH->CTRL2, FLASH_CTRL2_PG);
#else
  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= FLASH_PSIZE_HALF_WORD;
  FLASH->CTRL |= FLASH_CTRL_PG;
#endif /* APM32F403xx || APM32F402xx */

  *(__IO uint16_t*)Address = Data;
}

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
/**
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Byte(uint32_t Address, uint8_t Data)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_FLASH_ADDRESS(Address));

  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CTRL, FLASH_CTRL_PGSIZE);
  FLASH->CTRL |= FLASH_PSIZE_BYTE;
  FLASH->CTRL |= FLASH_CTRL_PG;

  *(__IO uint8_t*)Address = Data;
}
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @brief  Set the specific FLASH error flag.
  * @retval None
  */
static void FLASH_SetErrorCode(void)
{
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET)
  {
   pFlash.ErrorCode |= DAL_FLASH_ERROR_WRP;

   /* Clear FLASH write protection error pending bit */
   __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  }

#if defined(FLASH_FLAG_PGAERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) != RESET)
  {
   pFlash.ErrorCode |= DAL_FLASH_ERROR_PGA;

   /* Clear FLASH Programming alignment error pending bit */
   __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  }
#endif /* FLASH_FLAG_PGAERR */

#if defined(FLASH_FLAG_PGPERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR) != RESET)
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_PGP;

    /* Clear FLASH Programming parallelism error pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  }
#endif /* FLASH_FLAG_PGPERR */

#if defined(FLASH_FLAG_PGSERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR) != RESET)
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_PGS;

    /* Clear FLASH Programming sequence error pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
  }
#endif /* FLASH_FLAG_PGSERR */

#if defined(FLASH_STS_RPROERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_RDERR) != RESET)
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_RD;

    /* Clear FLASH Proprietary readout protection error pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_RDERR);
  }
#endif /* FLASH_STS_RPROERR */

#if defined(FLASH_FLAG_OPERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR) != RESET)
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_OPERATION;

    /* Clear FLASH Operation error pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
  }
#endif /* FLASH_FLAG_OPERR */

#if defined(FLASH_FLAG_PGERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR) != RESET)
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_PROG;

    /* Clear FLASH Operation error pending bit */
    __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR);
  }
#endif /* FLASH_FLAG_PGERR */

#if defined(FLASH_FLAG_OPTVERR)
  if(__DAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR))
  {
    pFlash.ErrorCode |= DAL_FLASH_ERROR_OPTV;
  __DAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  }
#endif /* FLASH_FLAG_OPTVERR */
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

