/**
  *
  * @file    apm32f4xx_ddl_smc.c
  * @brief   SMC Low Layer DDL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the Static Memory Controller (SMC) peripheral memories:
  *           + Initialization/de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
                        ##### SMC peripheral features #####
  ==============================================================================
  [..] The Static memory controller (SMC) includes following memory controllers:
       (+) The NOR/PSRAM memory controller
       (+) The NAND/PC Card memory controller

  [..] The SMC functional block makes the interface with synchronous and asynchronous static
       memories and 16-bit PC memory cards. Its main purposes are:
       (+) to translate AHB transactions into the appropriate external device protocol
       (+) to meet the access time requirements of the external memory devices

  [..] All external memories share the addresses, data and control signals with the controller.
       Each external device is accessed by means of a unique Chip Select. The SMC performs
       only one access at a time to an external device.
       The main features of the SMC controller are the following:
        (+) Interface with static-memory mapped devices including:
           (++) Static random access memory (SRAM)
           (++) Read-only memory (ROM)
           (++) NOR Flash memory/OneNAND Flash memory
           (++) PSRAM (4 memory banks)
           (++) 16-bit PC Card compatible devices
           (++) Two banks of NAND Flash memory with ECC hardware to check up to 8 Kbytes of
                data
        (+) Independent Chip Select control for each memory bank
        (+) Independent configuration for each memory bank

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
#if defined(DAL_NOR_MODULE_ENABLED) || defined(DAL_SRAM_MODULE_ENABLED) || defined(DAL_NAND_MODULE_ENABLED) || defined(DAL_PCCARD_MODULE_ENABLED)

/** @defgroup SMC_DDL SMC Low Layer
  * @brief SMC driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup SMC_DDL_Private_Constants SMC Low Layer Private Constants
  * @{
  */

/* ----------------------- SMC registers bit mask --------------------------- */

#if defined(SMC_Bank1)
/* --- BCR Register ---*/
/* BCR register clear mask */

/* --- BTR Register ---*/
/* BTR register clear mask */
#define BTR_CLEAR_MASK    ((uint32_t)(SMC_CSTIM1_ADDRSETCFG | SMC_CSTIM1_ADDRHLDCFG  |\
                                      SMC_CSTIM1_DATASETCFG | SMC_CSTIM1_BUSTURNCFG |\
                                      SMC_CSTIM1_CLKDIVCFG | SMC_CSTIM1_DATALATCFG  |\
                                      SMC_CSTIM1_ASYNCACCCFG))

/* --- BWTR Register ---*/
/* BWTR register clear mask */
#define BWTR_CLEAR_MASK   ((uint32_t)(SMC_WRTTIM1_ADDRSETCFG | SMC_WRTTIM1_ADDRHLDCFG  |\
                                      SMC_WRTTIM1_DATASETCFG | SMC_WRTTIM1_BUSTURNCFG |\
                                      SMC_WRTTIM1_ASYNCACCCFG))
#endif /* SMC_Bank1 */
#if defined(SMC_Bank2_3)

#if defined (SMC_PCR_PWAITEN)
/* --- PCR Register ---*/
/* PCR register clear mask */
#define PCR_CLEAR_MASK    ((uint32_t)(SMC_PCR_PWAITEN | SMC_PCR_PBKEN  | \
                                      SMC_PCR_PTYP    | SMC_PCR_PWID   | \
                                      SMC_PCR_ECCEN   | SMC_PCR_TCLR   | \
                                      SMC_PCR_TAR     | SMC_PCR_ECCPS))
/* --- PMEM Register ---*/
/* PMEM register clear mask */
#define PMEM_CLEAR_MASK   ((uint32_t)(SMC_PMEM_MEMSET2  | SMC_PMEM_MEMWAIT2 |\
                                      SMC_PMEM_MEMHOLD2 | SMC_PMEM_MEMHIZ2))

/* --- PATT Register ---*/
/* PATT register clear mask */
#define PATT_CLEAR_MASK   ((uint32_t)(SMC_PATT_ATTSET2  | SMC_PATT_ATTWAIT2 |\
                                      SMC_PATT_ATTHOLD2 | SMC_PATT_ATTHIZ2))
#else
/* --- PCR Register ---*/
/* PCR register clear mask */
#define PCR_CLEAR_MASK    ((uint32_t)(SMC_CTRL2_WAITFEN | SMC_CTRL2_MBKEN  | \
                                      SMC_CTRL2_MTYPECFG    | SMC_CTRL2_DBWIDCFG   | \
                                      SMC_CTRL2_ECCEN   | SMC_CTRL2_C2RDCFG   | \
                                      SMC_CTRL2_A2RDCFG     | SMC_CTRL2_ECCPSCFG))
/* --- PMEM Register ---*/
/* PMEM register clear mask */
#define PMEM_CLEAR_MASK   ((uint32_t)(SMC_CMSTIM2_SET2  | SMC_CMSTIM2_WAIT2 |\
                                      SMC_CMSTIM2_HLD2 | SMC_CMSTIM2_HIZ2))

/* --- PATT Register ---*/
/* PATT register clear mask */
#define PATT_CLEAR_MASK   ((uint32_t)(SMC_AMSTIM2_SET2  | SMC_AMSTIM2_WAIT2 |\
                                      SMC_AMSTIM2_HLD2 | SMC_AMSTIM2_HIZ2))

#endif /* SMC_PCR_PWAITEN */
#endif /* SMC_Bank2_3 */
#if defined(SMC_Bank4)
/* --- PCR Register ---*/
/* PCR register clear mask */
#define PCR4_CLEAR_MASK   ((uint32_t)(SMC_CTRL4_WAITFEN | SMC_CTRL4_MBKEN  | \
                                      SMC_CTRL4_MTYPECFG    | SMC_CTRL4_DBWIDCFG   | \
                                      SMC_CTRL4_ECCEN   | SMC_CTRL4_C2RDCFG   | \
                                      SMC_CTRL4_A2RDCFG     | SMC_CTRL4_ECCPSCFG))
/* --- PMEM Register ---*/
/* PMEM register clear mask */
#define PMEM4_CLEAR_MASK  ((uint32_t)(SMC_CMSTIM4_SET4  | SMC_CMSTIM4_WAIT4 |\
                                      SMC_CMSTIM4_HLD4 | SMC_CMSTIM4_HIZ4))

/* --- PATT Register ---*/
/* PATT register clear mask */
#define PATT4_CLEAR_MASK  ((uint32_t)(SMC_AMSTIM4_SET4  | SMC_AMSTIM4_WAIT4 |\
                                      SMC_AMSTIM4_HLD4 | SMC_AMSTIM4_HIZ4))

/* --- PIO4 Register ---*/
/* PIO4 register clear mask */
#define PIO4_CLEAR_MASK   ((uint32_t)(SMC_IOSTIM4_SET4  | SMC_IOSTIM4_WAIT4 | \
                                      SMC_IOSTIM4_HLD4 | SMC_IOSTIM4_HIZ4))

#endif /* SMC_Bank4 */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup SMC_DDL_Exported_Functions SMC Low Layer Exported Functions
  * @{
  */

#if defined(SMC_Bank1)

/** @defgroup SMC_DDL_Exported_Functions_NORSRAM SMC Low Layer NOR SRAM Exported Functions
  * @brief  NORSRAM Controller functions
  *
  @verbatim
  ==============================================================================
                   ##### How to use NORSRAM device driver #####
  ==============================================================================

  [..]
    This driver contains a set of APIs to interface with the SMC NORSRAM banks in order
    to run the NORSRAM external devices.

    (+) SMC NORSRAM bank reset using the function SMC_NORSRAM_DeInit()
    (+) SMC NORSRAM bank control configuration using the function SMC_NORSRAM_Init()
    (+) SMC NORSRAM bank timing configuration using the function SMC_NORSRAM_Timing_Init()
    (+) SMC NORSRAM bank extended timing configuration using the function
        SMC_NORSRAM_Extended_Timing_Init()
    (+) SMC NORSRAM bank enable/disable write operation using the functions
        SMC_NORSRAM_WriteOperation_Enable()/SMC_NORSRAM_WriteOperation_Disable()

@endverbatim
  * @{
  */

/** @defgroup SMC_DDL_NORSRAM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
  *
  @verbatim
  ==============================================================================
              ##### Initialization and de_initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the SMC NORSRAM interface
    (+) De-initialize the SMC NORSRAM interface
    (+) Configure the SMC clock and associated GPIOs

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SMC_NORSRAM device according to the specified
  *         control parameters in the SMC_NORSRAM_InitTypeDef
  * @param  Device Pointer to NORSRAM device instance
  * @param  Init Pointer to NORSRAM Initialization structure
  * @retval DAL status
  */
DAL_StatusTypeDef  SMC_NORSRAM_Init(SMC_NORSRAM_TypeDef *Device,
                                    SMC_NORSRAM_InitTypeDef *Init)
{
  uint32_t flashaccess;
  uint32_t btcr_reg;
  uint32_t mask;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NORSRAM_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Init->NSBank));
  ASSERT_PARAM(IS_SMC_MUX(Init->DataAddressMux));
  ASSERT_PARAM(IS_SMC_MEMORY(Init->MemoryType));
  ASSERT_PARAM(IS_SMC_NORSRAM_MEMORY_WIDTH(Init->MemoryDataWidth));
  ASSERT_PARAM(IS_SMC_BURSTMODE(Init->BurstAccessMode));
  ASSERT_PARAM(IS_SMC_WAIT_POLARITY(Init->WaitSignalPolarity));
#if defined(SMC_CSCTRL1_WRAPBEN)
  ASSERT_PARAM(IS_SMC_WRAP_MODE(Init->WrapMode));
#endif /* SMC_CSCTRL1_WRAPBEN */
  ASSERT_PARAM(IS_SMC_WAIT_SIGNAL_ACTIVE(Init->WaitSignalActive));
  ASSERT_PARAM(IS_SMC_WRITE_OPERATION(Init->WriteOperation));
  ASSERT_PARAM(IS_SMC_WAITE_SIGNAL(Init->WaitSignal));
  ASSERT_PARAM(IS_SMC_EXTENDED_MODE(Init->ExtendedMode));
  ASSERT_PARAM(IS_SMC_ASYNWAIT(Init->AsynchronousWait));
  ASSERT_PARAM(IS_SMC_WRITE_BURST(Init->WriteBurst));
#if defined(SMC_CSCTRL1_CCLKEN)
  ASSERT_PARAM(IS_SMC_CONTINOUS_CLOCK(Init->ContinuousClock));
#endif
#if defined(SMC_CSCTRL1_WFDIS)
  ASSERT_PARAM(IS_SMC_WRITE_FIFO(Init->WriteFifo));
#endif /* SMC_CSCTRL1_WFDIS */
  ASSERT_PARAM(IS_SMC_PAGESIZE(Init->PageSize));

  /* Disable NORSRAM Device */
  __SMC_NORSRAM_DISABLE(Device, Init->NSBank);

  /* Set NORSRAM device control parameters */
  if (Init->MemoryType == SMC_MEMORY_TYPE_NOR)
  {
    flashaccess = SMC_NORSRAM_FLASH_ACCESS_ENABLE;
  }
  else
  {
    flashaccess = SMC_NORSRAM_FLASH_ACCESS_DISABLE;
  }

  btcr_reg = (flashaccess                   | \
              Init->DataAddressMux          | \
              Init->MemoryType              | \
              Init->MemoryDataWidth         | \
              Init->BurstAccessMode         | \
              Init->WaitSignalPolarity      | \
              Init->WaitSignalActive        | \
              Init->WriteOperation          | \
              Init->WaitSignal              | \
              Init->ExtendedMode            | \
              Init->AsynchronousWait        | \
              Init->WriteBurst);

#if defined(SMC_CSCTRL1_WRAPBEN)
  btcr_reg |= Init->WrapMode;
#endif /* SMC_CSCTRL1_WRAPBEN */
#if defined(SMC_CSCTRL1_CCLKEN)
  btcr_reg |= Init->ContinuousClock;
#endif /* SMC_CSCTRL1_CCLKEN */
#if defined(SMC_CSCTRL1_WFDIS)
  btcr_reg |= Init->WriteFifo;
#endif /* SMC_CSCTRL1_WFDIS */
  btcr_reg |= Init->PageSize;

  mask = (SMC_CSCTRL1_MBKEN                |
          SMC_CSCTRL1_ADMUXEN                |
          SMC_CSCTRL1_MTYPECFG                 |
          SMC_CSCTRL1_MDBWIDCFG                 |
          SMC_CSCTRL1_NORFMACCEN               |
          SMC_CSCTRL1_BURSTEN              |
          SMC_CSCTRL1_WSPOLCFG              |
          SMC_CSCTRL1_WTIMCFG              |
          SMC_CSCTRL1_WREN                 |
          SMC_CSCTRL1_WAITEN               |
          SMC_CSCTRL1_EXTMODEEN               |
          SMC_CSCTRL1_WSASYNCEN            |
          SMC_CSCTRL1_WRBURSTEN);

#if defined(SMC_CSCTRL1_WRAPBEN)
  mask |= SMC_CSCTRL1_WRAPBEN;
#endif /* SMC_CSCTRL1_WRAPBEN */
#if defined(SMC_CSCTRL1_CCLKEN)
  mask |= SMC_CSCTRL1_CCLKEN;
#endif
#if defined(SMC_CSCTRL1_WFDIS)
  mask |= SMC_CSCTRL1_WFDIS;
#endif /* SMC_CSCTRL1_WFDIS */
  mask |= SMC_CSCTRL1_CRAMPSIZECFG;

  MODIFY_REG(Device->CSTR[Init->NSBank], mask, btcr_reg);

#if defined(SMC_CSCTRL1_CCLKEN)
  /* Configure synchronous mode when Continuous clock is enabled for bank2..4 */
  if ((Init->ContinuousClock == SMC_CONTINUOUS_CLOCK_SYNC_ASYNC) && (Init->NSBank != SMC_NORSRAM_BANK1))
  {
    MODIFY_REG(Device->CSTR[SMC_NORSRAM_BANK1], SMC_CSCTRL1_CCLKEN, Init->ContinuousClock);
  }
#endif
#if defined(SMC_CSCTRL1_WFDIS)

  if (Init->NSBank != SMC_NORSRAM_BANK1)
  {
    /* Configure Write FIFO mode when Write Fifo is enabled for bank2..4 */
    SET_BIT(Device->CSTR[SMC_NORSRAM_BANK1], (uint32_t)(Init->WriteFifo));
  }
#endif /* SMC_CSCTRL1_WFDIS */

  return DAL_OK;
}

/**
  * @brief  DeInitialize the SMC_NORSRAM peripheral
  * @param  Device Pointer to NORSRAM device instance
  * @param  ExDevice Pointer to NORSRAM extended mode device instance
  * @param  Bank NORSRAM bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NORSRAM_DeInit(SMC_NORSRAM_TypeDef *Device,
                                     SMC_NORSRAM_EXTENDED_TypeDef *ExDevice, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NORSRAM_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NORSRAM_EXTENDED_DEVICE(ExDevice));
  ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Bank));

  /* Disable the SMC_NORSRAM device */
  __SMC_NORSRAM_DISABLE(Device, Bank);

  /* De-initialize the SMC_NORSRAM device */
  /* SMC_NORSRAM_BANK1 */
  if (Bank == SMC_NORSRAM_BANK1)
  {
    Device->CSTR[Bank] = 0x000030DBU;
  }
  /* SMC_NORSRAM_BANK2, SMC_NORSRAM_BANK3 or SMC_NORSRAM_BANK4 */
  else
  {
    Device->CSTR[Bank] = 0x000030D2U;
  }

  Device->CSTR[Bank + 1U] = 0x0FFFFFFFU;
  ExDevice->WRTTIM[Bank]   = 0x0FFFFFFFU;

  return DAL_OK;
}

/**
  * @brief  Initialize the SMC_NORSRAM Timing according to the specified
  *         parameters in the SMC_NORSRAM_TimingTypeDef
  * @param  Device Pointer to NORSRAM device instance
  * @param  Timing Pointer to NORSRAM Timing structure
  * @param  Bank NORSRAM bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NORSRAM_Timing_Init(SMC_NORSRAM_TypeDef *Device,
                                          SMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank)
{
#if defined(SMC_CSCTRL1_CCLKEN)
  uint32_t tmpr;
#endif

  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NORSRAM_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_ADDRESS_SETUP_TIME(Timing->AddressSetupTime));
  ASSERT_PARAM(IS_SMC_ADDRESS_HOLD_TIME(Timing->AddressHoldTime));
  ASSERT_PARAM(IS_SMC_DATASETUP_TIME(Timing->DataSetupTime));
  ASSERT_PARAM(IS_SMC_TURNAROUND_TIME(Timing->BusTurnAroundDuration));
  ASSERT_PARAM(IS_SMC_CLK_DIV(Timing->CLKDivision));
  ASSERT_PARAM(IS_SMC_DATA_LATENCY(Timing->DataLatency));
  ASSERT_PARAM(IS_SMC_ACCESS_MODE(Timing->AccessMode));
  ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Bank));

  /* Set SMC_NORSRAM device timing parameters */
  MODIFY_REG(Device->CSTR[Bank + 1U], BTR_CLEAR_MASK, (Timing->AddressSetupTime                                  |
                                                       ((Timing->AddressHoldTime)        << SMC_CSTIM1_ADDRHLDCFG_Pos)  |
                                                       ((Timing->DataSetupTime)          << SMC_CSTIM1_DATASETCFG_Pos)  |
                                                       ((Timing->BusTurnAroundDuration)  << SMC_CSTIM1_BUSTURNCFG_Pos) |
                                                       (((Timing->CLKDivision) - 1U)     << SMC_CSTIM1_CLKDIVCFG_Pos)  |
                                                       (((Timing->DataLatency) - 2U)     << SMC_CSTIM1_DATALATCFG_Pos)  |
                                                       (Timing->AccessMode)));

#if defined(SMC_CSCTRL1_CCLKEN)
  /* Configure Clock division value (in NORSRAM bank 1) when continuous clock is enabled */
  if (DAL_IS_BIT_SET(Device->CSTR[SMC_NORSRAM_BANK1], SMC_CSCTRL1_CCLKEN))
  {
    tmpr = (uint32_t)(Device->CSTR[SMC_NORSRAM_BANK1 + 1U] & ~((0x0FU) << SMC_CSTIM1_CLKDIVCFG_Pos));
    tmpr |= (uint32_t)(((Timing->CLKDivision) - 1U) << SMC_CSTIM1_CLKDIVCFG_Pos);
    MODIFY_REG(Device->CSTR[SMC_NORSRAM_BANK1 + 1U], SMC_CSTIM1_CLKDIVCFG, tmpr);
  }

#endif
  return DAL_OK;
}

/**
  * @brief  Initialize the SMC_NORSRAM Extended mode Timing according to the specified
  *         parameters in the SMC_NORSRAM_TimingTypeDef
  * @param  Device Pointer to NORSRAM device instance
  * @param  Timing Pointer to NORSRAM Timing structure
  * @param  Bank NORSRAM bank number
  * @param  ExtendedMode SMC Extended Mode
  *          This parameter can be one of the following values:
  *            @arg SMC_EXTENDED_MODE_DISABLE
  *            @arg SMC_EXTENDED_MODE_ENABLE
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NORSRAM_Extended_Timing_Init(SMC_NORSRAM_EXTENDED_TypeDef *Device,
                                                   SMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank,
                                                   uint32_t ExtendedMode)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_EXTENDED_MODE(ExtendedMode));

  /* Set NORSRAM device timing register for write configuration, if extended mode is used */
  if (ExtendedMode == SMC_EXTENDED_MODE_ENABLE)
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_SMC_NORSRAM_EXTENDED_DEVICE(Device));
    ASSERT_PARAM(IS_SMC_ADDRESS_SETUP_TIME(Timing->AddressSetupTime));
    ASSERT_PARAM(IS_SMC_ADDRESS_HOLD_TIME(Timing->AddressHoldTime));
    ASSERT_PARAM(IS_SMC_DATASETUP_TIME(Timing->DataSetupTime));
    ASSERT_PARAM(IS_SMC_TURNAROUND_TIME(Timing->BusTurnAroundDuration));
    ASSERT_PARAM(IS_SMC_ACCESS_MODE(Timing->AccessMode));
    ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Bank));

    /* Set NORSRAM device timing register for write configuration, if extended mode is used */
    MODIFY_REG(Device->WRTTIM[Bank], BWTR_CLEAR_MASK, (Timing->AddressSetupTime                                    |
                                                     ((Timing->AddressHoldTime)        << SMC_WRTTIM1_ADDRHLDCFG_Pos)  |
                                                     ((Timing->DataSetupTime)          << SMC_WRTTIM1_DATASETCFG_Pos)  |
                                                     Timing->AccessMode                                          |
                                                     ((Timing->BusTurnAroundDuration)  << SMC_WRTTIM1_BUSTURNCFG_Pos)));
  }
  else
  {
    Device->WRTTIM[Bank] = 0x0FFFFFFFU;
  }

  return DAL_OK;
}
/**
  * @}
  */

/** @addtogroup SMC_DDL_NORSRAM_Private_Functions_Group2
  *  @brief   management functions
  *
@verbatim
  ==============================================================================
                      ##### SMC_NORSRAM Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the SMC NORSRAM interface.

@endverbatim
  * @{
  */

/**
  * @brief  Enables dynamically SMC_NORSRAM write operation.
  * @param  Device Pointer to NORSRAM device instance
  * @param  Bank NORSRAM bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NORSRAM_WriteOperation_Enable(SMC_NORSRAM_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NORSRAM_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Bank));

  /* Enable write operation */
  SET_BIT(Device->CSTR[Bank], SMC_WRITE_OPERATION_ENABLE);

  return DAL_OK;
}

/**
  * @brief  Disables dynamically SMC_NORSRAM write operation.
  * @param  Device Pointer to NORSRAM device instance
  * @param  Bank NORSRAM bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NORSRAM_WriteOperation_Disable(SMC_NORSRAM_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NORSRAM_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NORSRAM_BANK(Bank));

  /* Disable write operation */
  CLEAR_BIT(Device->CSTR[Bank], SMC_WRITE_OPERATION_ENABLE);

  return DAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* SMC_Bank1 */

#if defined(SMC_Bank2_3)

/** @defgroup SMC_DDL_Exported_Functions_NAND SMC Low Layer NAND Exported Functions
  * @brief    NAND Controller functions
  *
  @verbatim
  ==============================================================================
                    ##### How to use NAND device driver #####
  ==============================================================================
  [..]
    This driver contains a set of APIs to interface with the SMC NAND banks in order
    to run the NAND external devices.

    (+) SMC NAND bank reset using the function SMC_NAND_DeInit()
    (+) SMC NAND bank control configuration using the function SMC_NAND_Init()
    (+) SMC NAND bank common space timing configuration using the function
        SMC_NAND_CommonSpace_Timing_Init()
    (+) SMC NAND bank attribute space timing configuration using the function
        SMC_NAND_AttributeSpace_Timing_Init()
    (+) SMC NAND bank enable/disable ECC correction feature using the functions
        SMC_NAND_ECC_Enable()/SMC_NAND_ECC_Disable()
    (+) SMC NAND bank get ECC correction code using the function SMC_NAND_GetECC()

@endverbatim
  * @{
  */

/** @defgroup SMC_DDL_NAND_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and de_initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the SMC NAND interface
    (+) De-initialize the SMC NAND interface
    (+) Configure the SMC clock and associated GPIOs

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SMC_NAND device according to the specified
  *         control parameters in the SMC_NAND_HandleTypeDef
  * @param  Device Pointer to NAND device instance
  * @param  Init Pointer to NAND Initialization structure
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_Init(SMC_NAND_TypeDef *Device, SMC_NAND_InitTypeDef *Init)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Init->NandBank));
  ASSERT_PARAM(IS_SMC_WAIT_FEATURE(Init->Waitfeature));
  ASSERT_PARAM(IS_SMC_NAND_MEMORY_WIDTH(Init->MemoryDataWidth));
  ASSERT_PARAM(IS_SMC_ECC_STATE(Init->EccComputation));
  ASSERT_PARAM(IS_SMC_ECCPAGE_SIZE(Init->ECCPageSize));
  ASSERT_PARAM(IS_SMC_TCLR_TIME(Init->TCLRSetupTime));
  ASSERT_PARAM(IS_SMC_TAR_TIME(Init->TARSetupTime));

  /* Set NAND device control parameters */
  if (Init->NandBank == SMC_NAND_BANK2)
  {
    /* NAND bank 2 registers configuration */
    MODIFY_REG(Device->CTRL2, PCR_CLEAR_MASK, (Init->Waitfeature                                      |
                                              SMC_PCR_MEMORY_TYPE_NAND                               |
                                              Init->MemoryDataWidth                                  |
                                              Init->EccComputation                                   |
                                              Init->ECCPageSize                                      |
                                              ((Init->TCLRSetupTime) << SMC_CTRL2_C2RDCFG_Pos) |
                                              ((Init->TARSetupTime)  << SMC_CTRL2_A2RDCFG_Pos)));
  }
  else
  {
    /* NAND bank 3 registers configuration */
    MODIFY_REG(Device->CTRL3, PCR_CLEAR_MASK, (Init->Waitfeature                                      |
                                              SMC_PCR_MEMORY_TYPE_NAND                               |
                                              Init->MemoryDataWidth                                  |
                                              Init->EccComputation                                   |
                                              Init->ECCPageSize                                      |
                                              ((Init->TCLRSetupTime) << SMC_CTRL2_C2RDCFG_Pos)  |
                                              ((Init->TARSetupTime)  << SMC_CTRL2_A2RDCFG_Pos)));
  }

  return DAL_OK;
}

/**
  * @brief  Initializes the SMC_NAND Common space Timing according to the specified
  *         parameters in the SMC_NAND_PCC_TimingTypeDef
  * @param  Device Pointer to NAND device instance
  * @param  Timing Pointer to NAND timing structure
  * @param  Bank NAND bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_CommonSpace_Timing_Init(SMC_NAND_TypeDef *Device,
                                                   SMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_SETUP_TIME(Timing->SetupTime));
  ASSERT_PARAM(IS_SMC_WAIT_TIME(Timing->WaitSetupTime));
  ASSERT_PARAM(IS_SMC_HOLD_TIME(Timing->HoldSetupTime));
  ASSERT_PARAM(IS_SMC_HIZ_TIME(Timing->HiZSetupTime));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Set SMC_NAND device timing parameters */
  if (Bank == SMC_NAND_BANK2)
  {
    /* NAND bank 2 registers configuration */
    MODIFY_REG(Device->CMSTIM2, PMEM_CLEAR_MASK, (Timing->SetupTime                                             |
                                                ((Timing->WaitSetupTime) << SMC_CMSTIM2_WAIT2_Pos) |
                                                ((Timing->HoldSetupTime) << SMC_CMSTIM2_HLD2_Pos) |
                                                ((Timing->HiZSetupTime)  << SMC_CMSTIM2_HIZ2_Pos)));
  }
  else
  {
    /* NAND bank 3 registers configuration */
    MODIFY_REG(Device->CMSTIM3, PMEM_CLEAR_MASK, (Timing->SetupTime                                             |
                                                ((Timing->WaitSetupTime) << SMC_CMSTIM3_WAIT3_Pos) |
                                                ((Timing->HoldSetupTime) << SMC_CMSTIM3_HLD3_Pos) |
                                                ((Timing->HiZSetupTime)  << SMC_CMSTIM3_HIZ3_Pos)));
  }

  return DAL_OK;
}

/**
  * @brief  Initializes the SMC_NAND Attribute space Timing according to the specified
  *         parameters in the SMC_NAND_PCC_TimingTypeDef
  * @param  Device Pointer to NAND device instance
  * @param  Timing Pointer to NAND timing structure
  * @param  Bank NAND bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_AttributeSpace_Timing_Init(SMC_NAND_TypeDef *Device,
                                                      SMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_SETUP_TIME(Timing->SetupTime));
  ASSERT_PARAM(IS_SMC_WAIT_TIME(Timing->WaitSetupTime));
  ASSERT_PARAM(IS_SMC_HOLD_TIME(Timing->HoldSetupTime));
  ASSERT_PARAM(IS_SMC_HIZ_TIME(Timing->HiZSetupTime));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Set SMC_NAND device timing parameters */
  if (Bank == SMC_NAND_BANK2)
  {
    /* NAND bank 2 registers configuration */
    MODIFY_REG(Device->AMSTIM2, PATT_CLEAR_MASK, (Timing->SetupTime                                             |
                                                ((Timing->WaitSetupTime) << SMC_AMSTIM2_WAIT2_Pos) |
                                                ((Timing->HoldSetupTime) << SMC_AMSTIM2_HLD2_Pos) |
                                                ((Timing->HiZSetupTime)  << SMC_AMSTIM2_HIZ2_Pos)));
  }
  else
  {
    /* NAND bank 3 registers configuration */
    MODIFY_REG(Device->AMSTIM3, PATT_CLEAR_MASK, (Timing->SetupTime                                             |
                                                ((Timing->WaitSetupTime) << SMC_AMSTIM3_WAIT3_Pos) |
                                                ((Timing->HoldSetupTime) << SMC_AMSTIM3_HLD3_Pos) |
                                                ((Timing->HiZSetupTime)  << SMC_AMSTIM3_HIZ3_Pos)));
  }

  return DAL_OK;
}

/**
  * @brief  DeInitializes the SMC_NAND device
  * @param  Device Pointer to NAND device instance
  * @param  Bank NAND bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_DeInit(SMC_NAND_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Disable the NAND Bank */
  __SMC_NAND_DISABLE(Device, Bank);

  /* De-initialize the NAND Bank */
  if (Bank == SMC_NAND_BANK2)
  {
    /* Set the SMC_NAND_BANK2 registers to their reset values */
    WRITE_REG(Device->CTRL2,  0x00000018U);
    WRITE_REG(Device->STSINT2,   0x00000040U);
    WRITE_REG(Device->CMSTIM2, 0xFCFCFCFCU);
    WRITE_REG(Device->AMSTIM2, 0xFCFCFCFCU);
  }
  /* SMC_Bank3_NAND */
  else
  {
    /* Set the SMC_NAND_BANK3 registers to their reset values */
    WRITE_REG(Device->CTRL3,  0x00000018U);
    WRITE_REG(Device->STSINT3,   0x00000040U);
    WRITE_REG(Device->CMSTIM3, 0xFCFCFCFCU);
    WRITE_REG(Device->AMSTIM3, 0xFCFCFCFCU);
  }

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup DAL_SMC_NAND_Group2 Peripheral Control functions
  *  @brief   management functions
  *
@verbatim
  ==============================================================================
                       ##### SMC_NAND Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the SMC NAND interface.

@endverbatim
  * @{
  */


/**
  * @brief  Enables dynamically SMC_NAND ECC feature.
  * @param  Device Pointer to NAND device instance
  * @param  Bank NAND bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_ECC_Enable(SMC_NAND_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Enable ECC feature */
  if (Bank == SMC_NAND_BANK2)
  {
    SET_BIT(Device->CTRL2, SMC_CTRL2_ECCEN);
  }
  else
  {
    SET_BIT(Device->CTRL3, SMC_CTRL3_ECCEN);
  }

  return DAL_OK;
}


/**
  * @brief  Disables dynamically SMC_NAND ECC feature.
  * @param  Device Pointer to NAND device instance
  * @param  Bank NAND bank number
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_ECC_Disable(SMC_NAND_TypeDef *Device, uint32_t Bank)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Disable ECC feature */
  if (Bank == SMC_NAND_BANK2)
  {
    CLEAR_BIT(Device->CTRL2, SMC_CTRL2_ECCEN);
  }
  else
  {
    CLEAR_BIT(Device->CTRL3, SMC_CTRL3_ECCEN);
  }

  return DAL_OK;
}

/**
  * @brief  Disables dynamically SMC_NAND ECC feature.
  * @param  Device Pointer to NAND device instance
  * @param  ECCval Pointer to ECC value
  * @param  Bank NAND bank number
  * @param  Timeout Timeout wait value
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_NAND_GetECC(SMC_NAND_TypeDef *Device, uint32_t *ECCval, uint32_t Bank,
                                  uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_NAND_DEVICE(Device));
  ASSERT_PARAM(IS_SMC_NAND_BANK(Bank));

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait until FIFO is empty */
  while (__SMC_NAND_GET_FLAG(Device, Bank, SMC_FLAG_FEMPT) == RESET)
  {
    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
      {
        return DAL_TIMEOUT;
      }
    }
  }

  if (Bank == SMC_NAND_BANK2)
  {
    /* Get the ECCRS2 register value */
    *ECCval = (uint32_t)Device->ECCRS2;
  }
  else
  {
    /* Get the ECCRS3 register value */
    *ECCval = (uint32_t)Device->ECCRS3;
  }

  return DAL_OK;
}

/**
  * @}
  */
#endif /* SMC_Bank2_3 */

#if defined(SMC_Bank4)

/** @addtogroup SMC_DDL_PCCARD
  * @brief    PCCARD Controller functions
  *
  @verbatim
  ==============================================================================
                    ##### How to use PCCARD device driver #####
  ==============================================================================
  [..]
    This driver contains a set of APIs to interface with the SMC PCCARD bank in order
    to run the PCCARD/compact flash external devices.

    (+) SMC PCCARD bank reset using the function SMC_PCCARD_DeInit()
    (+) SMC PCCARD bank control configuration using the function SMC_PCCARD_Init()
    (+) SMC PCCARD bank common space timing configuration using the function
        SMC_PCCARD_CommonSpace_Timing_Init()
    (+) SMC PCCARD bank attribute space timing configuration using the function
        SMC_PCCARD_AttributeSpace_Timing_Init()
    (+) SMC PCCARD bank IO space timing configuration using the function
        SMC_PCCARD_IOSpace_Timing_Init()
@endverbatim
  * @{
  */

/** @addtogroup SMC_DDL_PCCARD_Private_Functions_Group1
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and de_initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the SMC PCCARD interface
    (+) De-initialize the SMC PCCARD interface
    (+) Configure the SMC clock and associated GPIOs

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SMC_PCCARD device according to the specified
  *         control parameters in the SMC_PCCARD_HandleTypeDef
  * @param  Device Pointer to PCCARD device instance
  * @param  Init Pointer to PCCARD Initialization structure
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_PCCARD_Init(SMC_PCCARD_TypeDef *Device, SMC_PCCARD_InitTypeDef *Init)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_PCCARD_DEVICE(Device));
#if defined(SMC_Bank2_3)
  ASSERT_PARAM(IS_SMC_WAIT_FEATURE(Init->Waitfeature));
  ASSERT_PARAM(IS_SMC_TCLR_TIME(Init->TCLRSetupTime));
  ASSERT_PARAM(IS_SMC_TAR_TIME(Init->TARSetupTime));
#endif /* SMC_Bank2_3 */

  /* Set SMC_PCCARD device control parameters */
  MODIFY_REG(Device->CTRL4,
             (SMC_CTRL4_MTYPECFG                                          |
              SMC_CTRL4_WAITFEN                                       |
              SMC_CTRL4_DBWIDCFG                                          |
              SMC_CTRL4_C2RDCFG                                          |
              SMC_CTRL4_A2RDCFG),
             (SMC_PCR_MEMORY_TYPE_PCCARD                             |
              Init->Waitfeature                                      |
              SMC_NAND_PCC_MEM_BUS_WIDTH_16                          |
              (Init->TCLRSetupTime << SMC_CTRL4_C2RDCFG_Pos)   |
              (Init->TARSetupTime  << SMC_CTRL4_A2RDCFG_Pos)));

  return DAL_OK;
}

/**
  * @brief  Initializes the SMC_PCCARD Common space Timing according to the specified
  *         parameters in the SMC_NAND_PCC_TimingTypeDef
  * @param  Device Pointer to PCCARD device instance
  * @param  Timing Pointer to PCCARD timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_PCCARD_CommonSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                              SMC_NAND_PCC_TimingTypeDef *Timing)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_PCCARD_DEVICE(Device));
#if defined(SMC_Bank2_3)
  ASSERT_PARAM(IS_SMC_SETUP_TIME(Timing->SetupTime));
  ASSERT_PARAM(IS_SMC_WAIT_TIME(Timing->WaitSetupTime));
  ASSERT_PARAM(IS_SMC_HOLD_TIME(Timing->HoldSetupTime));
  ASSERT_PARAM(IS_SMC_HIZ_TIME(Timing->HiZSetupTime));
#endif /* SMC_Bank2_3 */

  /* Set PCCARD timing parameters */
  MODIFY_REG(Device->CMSTIM4, PMEM4_CLEAR_MASK,
             (Timing->SetupTime                                              |
              ((Timing->WaitSetupTime) << SMC_CMSTIM4_WAIT4_Pos)  |
              ((Timing->HoldSetupTime) << SMC_CMSTIM4_HLD4_Pos)  |
              ((Timing->HiZSetupTime)  << SMC_CMSTIM4_HIZ4_Pos)));

  return DAL_OK;
}

/**
  * @brief  Initializes the SMC_PCCARD Attribute space Timing according to the specified
  *         parameters in the SMC_NAND_PCC_TimingTypeDef
  * @param  Device Pointer to PCCARD device instance
  * @param  Timing Pointer to PCCARD timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_PCCARD_AttributeSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                                 SMC_NAND_PCC_TimingTypeDef *Timing)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_PCCARD_DEVICE(Device));
#if defined(SMC_Bank2_3)
  ASSERT_PARAM(IS_SMC_SETUP_TIME(Timing->SetupTime));
  ASSERT_PARAM(IS_SMC_WAIT_TIME(Timing->WaitSetupTime));
  ASSERT_PARAM(IS_SMC_HOLD_TIME(Timing->HoldSetupTime));
  ASSERT_PARAM(IS_SMC_HIZ_TIME(Timing->HiZSetupTime));
#endif /* SMC_Bank2_3 */

  /* Set PCCARD timing parameters */
  MODIFY_REG(Device->AMSTIM4, PATT4_CLEAR_MASK,
             (Timing->SetupTime                                              |
              ((Timing->WaitSetupTime) << SMC_AMSTIM4_WAIT4_Pos)  |
              ((Timing->HoldSetupTime) << SMC_AMSTIM4_HLD4_Pos)  |
              ((Timing->HiZSetupTime)  << SMC_AMSTIM4_HIZ4_Pos)));

  return DAL_OK;
}

/**
  * @brief  Initializes the SMC_PCCARD IO space Timing according to the specified
  *         parameters in the SMC_NAND_PCC_TimingTypeDef
  * @param  Device Pointer to PCCARD device instance
  * @param  Timing Pointer to PCCARD timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_PCCARD_IOSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                          SMC_NAND_PCC_TimingTypeDef *Timing)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_PCCARD_DEVICE(Device));
#if defined(SMC_Bank2_3)
  ASSERT_PARAM(IS_SMC_SETUP_TIME(Timing->SetupTime));
  ASSERT_PARAM(IS_SMC_WAIT_TIME(Timing->WaitSetupTime));
  ASSERT_PARAM(IS_SMC_HOLD_TIME(Timing->HoldSetupTime));
  ASSERT_PARAM(IS_SMC_HIZ_TIME(Timing->HiZSetupTime));
#endif /* SMC_Bank2_3 */

  /* Set SMC_PCCARD device timing parameters */
  MODIFY_REG(Device->IOSTIM4, PIO4_CLEAR_MASK,
             (Timing->SetupTime                                           |
              (Timing->WaitSetupTime   << SMC_IOSTIM4_WAIT4_Pos) |
              (Timing->HoldSetupTime   << SMC_IOSTIM4_HLD4_Pos) |
              (Timing->HiZSetupTime    << SMC_IOSTIM4_HIZ4_Pos)));

  return DAL_OK;
}

/**
  * @brief  DeInitializes the SMC_PCCARD device
  * @param  Device Pointer to PCCARD device instance
  * @retval DAL status
  */
DAL_StatusTypeDef SMC_PCCARD_DeInit(SMC_PCCARD_TypeDef *Device)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMC_PCCARD_DEVICE(Device));

  /* Disable the SMC_PCCARD device */
  __SMC_PCCARD_DISABLE(Device);

  /* De-initialize the SMC_PCCARD device */
  Device->CTRL4    = 0x00000018U;
  Device->STSINT4     = 0x00000040U;
  Device->CMSTIM4   = 0xFCFCFCFCU;
  Device->AMSTIM4   = 0xFCFCFCFCU;
  Device->IOSTIM4    = 0xFCFCFCFCU;

  return DAL_OK;
}

/**
  * @}
  */
#endif /* SMC_Bank4 */


/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_NOR_MODULE_ENABLED */
/**
  * @}
  */
/**
  * @}
  */
