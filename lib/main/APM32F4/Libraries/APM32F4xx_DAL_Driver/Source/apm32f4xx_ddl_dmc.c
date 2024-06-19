/**
  *
  * @file    apm32f4xx_ddl_dmc.c
  * @brief   DMC Low Layer DDL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the Dynamic Memory Controller (DMC) peripheral memories:
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
                        ##### DMC peripheral features #####
  ==============================================================================
  [..] The Dynamic memory controller (DMC) includes following memory controllers:
       (+) The SDRAM memory controller

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
#if defined(DAL_SDRAM_MODULE_ENABLED)

/** @defgroup DMC_DDL DMC Low Layer
  * @brief DMC driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup DMC_DDL_Private_Constants DMC Low Layer Private Constants
  * @{
  */

/* ----------------------- DMC registers bit mask --------------------------- */

#if defined(DMC)

/* --- CFG Register ---*/
/* CFG register clear mask */
#define CFG_CLEAR_MASK      ((uint32_t)(DMC_CFG_BAWCFG | DMC_CFG_RAWCFG | \
                                      DMC_CFG_CAWCFG | DMC_CFG_DWCFG))

/* CTRL1 register clear mask */
#define CTRL1_CLEAR_MASK    ((uint32_t)(DMC_CTRL1_SRMEN | DMC_CTRL1_PDMEN | \
                                      DMC_CTRL1_FRBSREN | DMC_CTRL1_FRASREN | \
                                      DMC_CTRL1_RDNUMMCFG | DMC_CTRL1_BANKNUMCFG))

/* CTRL2 register clear mask */
#define CTRL2_CLEAR_MASK    ((uint32_t)(DMC_CTRL2_CPHACFG | DMC_CTRL2_RDDEN | \
                                      DMC_CTRL2_RDDCFG | DMC_CTRL2_WPEN | \
                                      DMC_CTRL2_BUFFEN | DMC_CTRL2_WRPBSEL))

/* TIM0 register clear mask */
#define TIM0_CLEAR_MASK    ((uint32_t)(DMC_TIM0_CASLSEL0 | DMC_TIM0_RASMINTSEL | \
                                      DMC_TIM0_DTIMSEL | DMC_TIM0_PCPSEL | \
                                      DMC_TIM0_WRTIMSEL | DMC_TIM0_ARPSEL | \
                                      DMC_TIM0_XSR0 | DMC_TIM0_ATACP | \
                                      DMC_TIM0_ECASLSEL1 | DMC_TIM0_EXSR1))

/* TIM1 register clear mask */
#define TIM1_CLEAR_MASK    ((uint32_t)(DMC_TIM1_STBTIM | DMC_TIM1_ARNUMCFG))
#endif /* DMC */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup DMC_DDL_Exported_Functions DMC Low Layer Exported Functions
  * @{
  */

#if defined(DMC)

/** @defgroup DMC_DDL_SDRAM
  * @brief    SDRAM Controller functions
  *
  @verbatim
  ==============================================================================
                     ##### How to use SDRAM device driver #####
  ==============================================================================
  [..]
    This driver contains a set of APIs to interface with the DMC SDRAM banks in order
    to run the SDRAM external devices.

    (+) DMC SDRAM bank reset using the function DMC_SDRAM_DeInit()
    (+) DMC SDRAM bank control configuration using the function DMC_SDRAM_Init()
    (+) DMC SDRAM bank timing configuration using the function DMC_SDRAM_Timing_Init()
    (+) DMC SDRAM bank enable/disable write operation using the functions
        DMC_SDRAM_WriteOperation_Enable()/DMC_SDRAM_WriteOperation_Disable()
    (+) DMC SDRAM bank send command using the function DMC_SDRAM_SendCommand()

@endverbatim
  * @{
  */

/** @addtogroup DMC_DDL_SDRAM_Private_Functions_Group1
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and de_initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the DMC SDRAM interface
    (+) De-initialize the DMC SDRAM interface
    (+) Configure the DMC clock and associated GPIOs

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the DMC_SDRAM device according to the specified
  *         control parameters in the DMC_SDRAM_InitTypeDef
  * @param  Device Pointer to SDRAM device instance
  * @param  Init Pointer to SDRAM Initialization structure
  * @retval DAL status
  */
DAL_StatusTypeDef DMC_SDRAM_Init(DMC_SDRAM_TypeDef *Device, DMC_SDRAM_InitTypeDef *Init)
{
    uint32_t tickstart;

    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));
    ASSERT_PARAM(IS_DMC_BANK_WIDTH(Init->BankWidth));
    ASSERT_PARAM(IS_DMC_COLUMNBITS_NUMBER(Init->ColumnBitsNumber));
    ASSERT_PARAM(IS_DMC_ROWBITS_NUMBER(Init->RowBitsNumber));
    ASSERT_PARAM(IS_DMC_MEMORY_WIDTH(Init->MemoryDataWidth));
    ASSERT_PARAM(IS_DMC_CLK_PHASE(Init->ClockPhase));
    ASSERT_PARAM(IS_DMC_RD_DELAY(Init->RDDelay));
    ASSERT_PARAM(IS_DMC_RD_DELAY_CLK(Init->RDDelayClk));
    ASSERT_PARAM(IS_DMC_WRITE_PIPE(Init->WritePipe));
    ASSERT_PARAM(IS_DMC_ACCELERATE_MODE(Init->AccelerateMode));
    ASSERT_PARAM(IS_DMC_WRAP_BURST(Init->WRAPBurstType));
    ASSERT_PARAM(IS_DMC_POWER_DOWN_MODE(Init->PowerDownMode));
    ASSERT_PARAM(IS_DMC_SELF_REFRESH_MODE(Init->SelfRefreshMode));
    ASSERT_PARAM(IS_DMC_REFRESH_TYPE(Init->RefreshTypeEnterSelfRefresh));
    ASSERT_PARAM(IS_DMC_REFRESH_TYPE(Init->RefreshTypeExitSelfRefresh));
    ASSERT_PARAM(IS_DMC_REG_INSERT_NUMBER(Init->RegisterInsertNumber));
    ASSERT_PARAM(IS_DMC_OPEN_BANK_NUMBER(Init->OpenBankNumber));

    /* Switch to DMC controller */
    __DMC_SDRAM_ENABLE(Device);

    /* Get Start Tick */
    tickstart = DAL_GetTick();

    /* Wait till DMC controller is ready */
    while((Device->CTRL1 & DMC_CTRL1_INIT) != RESET)
    {
        if((DAL_GetTick() - tickstart ) > DMC_TIMEOUT_VALUE)
        {
            return DAL_TIMEOUT;
        }
    }

    /* Set SDRAM CFG parameters */
    MODIFY_REG(Device->CFG, CFG_CLEAR_MASK, \
               (Init->BankWidth | \
                Init->ColumnBitsNumber | \
                Init->MemoryDataWidth | \
                Init->RowBitsNumber));

    /* Set SDRAM CTRL1 parameters */
    MODIFY_REG(Device->CTRL1, CTRL1_CLEAR_MASK, \
               (Init->SelfRefreshMode | \
                Init->PowerDownMode | \
                ((Init->RefreshTypeEnterSelfRefresh) << DMC_CTRL1_FRBSREN_Pos) | \
                ((Init->RefreshTypeExitSelfRefresh) << DMC_CTRL1_FRASREN_Pos) | \
                ((Init->RegisterInsertNumber) << DMC_CTRL1_RDNUMMCFG_Pos) | \
                (((Init->OpenBankNumber) - 1U) << DMC_CTRL1_BANKNUMCFG_Pos)));

    /* Set SDRAM CTRL2 parameters */
    MODIFY_REG(Device->CTRL2, CTRL2_CLEAR_MASK, \
               (Init->ClockPhase | \
                Init->RDDelay | \
                (Init->RDDelayClk << DMC_CTRL2_RDDCFG_Pos) | \
                Init->WritePipe | \
                Init->AccelerateMode | \
                Init->WRAPBurstType));

    /* Update mode setup */
    __DMC_SDRAM_UPDATE_MODE_SETUP(Device);

    return DAL_OK;
}


/**
  * @brief  Initializes the DMC_SDRAM device timing according to the specified
  *         parameters in the DMC_SDRAM_TimingTypeDef
  * @param  Device Pointer to SDRAM device instance
  * @param  Timing Pointer to SDRAM Timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef DMC_SDRAM_Timing_Init(DMC_SDRAM_TypeDef *Device,
                                        DMC_SDRAM_TimingTypeDef *Timing)
{
    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));
    ASSERT_PARAM(IS_DMC_CAS_LATENCY(Timing->CASLatency));
    ASSERT_PARAM(IS_DMC_RAS_TIME(Timing->RASTime));
    ASSERT_PARAM(IS_DMC_RAS_TO_CAS_DELAY(Timing->RASToCASDelay));
    ASSERT_PARAM(IS_DMC_PRECHARGE_MODE(Timing->PrechargeMode));
    ASSERT_PARAM(IS_DMC_PRECHARGE_PERIOD(Timing->PrechargePeriod));
    ASSERT_PARAM(IS_DMC_AUTO_REFRESH_TIME(Timing->AutoRefreshTime));
    ASSERT_PARAM(IS_DMC_WRITE_RECOVERY_TIME(Timing->WriteRecoveryTime));
    ASSERT_PARAM(IS_DMC_XSR_TIME(Timing->XSRTime));
    ASSERT_PARAM(IS_DMC_ACTIVE_COMMAND_TIME(Timing->ActiveCommandPeriod));
    ASSERT_PARAM(IS_DMC_REFRESH_PERIOD(Timing->RefreshPeriod));
    ASSERT_PARAM(IS_DMC_STABLE_TIME(Timing->StableTime));

    /* Set SDRAM device CTRL1 parameters */
    MODIFY_REG(Device->CTRL1, DMC_CTRL1_PCACFG, Timing->PrechargeMode);

    /* Set SDRAM device timing 0 parameters */
    MODIFY_REG(Device->TIM0, TIM0_CLEAR_MASK, \
               (((Timing->CASLatency & 0x03U) << DMC_TIM0_CASLSEL0_Pos) | \
                (((Timing->CASLatency >> 0x02U) & 0x01U) << DMC_TIM0_ECASLSEL1_Pos) | \
                (((Timing->RASTime) - 1U) << DMC_TIM0_RASMINTSEL_Pos) | \
                (((Timing->RASToCASDelay) - 1U) << DMC_TIM0_DTIMSEL_Pos) | \
                (((Timing->PrechargePeriod) - 1U) << DMC_TIM0_PCPSEL_Pos) | \
                (((Timing->AutoRefreshTime) - 1U) << DMC_TIM0_ARPSEL_Pos) | \
                (((Timing->WriteRecoveryTime) - 1U) << DMC_TIM0_WRTIMSEL_Pos) | \
                (((Timing->ActiveCommandPeriod) - 1U) << DMC_TIM0_ATACP_Pos) | \
                (((Timing->XSRTime) & 0x0FU) << DMC_TIM0_XSR0_Pos) | \
                ((((Timing->XSRTime) >> 0x04U) & 0x1FU) << DMC_TIM0_EXSR1_Pos)));

    /* Set SDRAM device timing 1 parameters */
    MODIFY_REG(Device->TIM1, TIM1_CLEAR_MASK, \
               (((Timing->StableTime) << DMC_TIM1_STBTIM_Pos) | \
                (((Timing->AutoRefreshNumber) - 1U) << DMC_TIM1_ARNUMCFG_Pos)));

    /* Set SDRAM device refresh cycle */
    MODIFY_REG(Device->REF, DMC_REF_RCYCCFG, Timing->RefreshPeriod);

    return DAL_OK;
}

/**
  * @brief  DeInitializes the DMC_SDRAM peripheral
  * @param  Device Pointer to SDRAM device instance
  * @retval DAL status
  */
DAL_StatusTypeDef DMC_SDRAM_DeInit(DMC_SDRAM_TypeDef *Device)
{
    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));

    /* De-initialize the SDRAM device */
    Device->CFG     = 0x00141388U;
    Device->TIM0    = 0x019A5252U;
    Device->TIM1    = 0x00074E20U;
    Device->CTRL1   = 0x00003048U;
    Device->CTRL2   = 0x0000002EU;
    Device->REF     = 0x000000C3U;

    return DAL_OK;
}

/**
  * @}
  */

/** @addtogroup DMC_DDL_SDRAM_Private_Functions_Group2
  *  @brief   management functions
  *
@verbatim
  ==============================================================================
                      ##### DMC_SDRAM Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the FMC SDRAM interface.

@endverbatim
  * @{
  */

/**
  * @brief  Program the SDRAM Memory Refresh period.
  * @param  Device Pointer to SDRAM device instance
  * @param  RefreshPeriod The SDRAM refresh period value.
  * @retval DAL state
  */
DAL_StatusTypeDef DMC_SDRAM_ProgramRefreshPeriod(DMC_SDRAM_TypeDef *Device, uint32_t RefreshPeriod)
{
    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));
    ASSERT_PARAM(IS_DMC_REFRESH_PERIOD(RefreshPeriod));

    /* Set SDRAM device refresh cycle */
    MODIFY_REG(Device->REF, DMC_REF_RCYCCFG, RefreshPeriod);

    return DAL_OK;
}

/**
  * @brief  Set the Number of consecutive SDRAM Memory open bank.
  * @param  Device Pointer to SDRAM device instance
  * @param  OpenBankNumber Specifies the open bank number.
  * @retval None
  */
DAL_StatusTypeDef DMC_SDRAM_SetOpenBankNumber(DMC_SDRAM_TypeDef *Device, uint32_t OpenBankNumber)
{
    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));
    ASSERT_PARAM(IS_DMC_OPEN_BANK_NUMBER(OpenBankNumber));

    /* Set SDRAM CTRL1 parameters */
    MODIFY_REG(Device->CTRL1, DMC_CTRL1_BANKNUMCFG, (((OpenBankNumber) - 1U) << DMC_CTRL1_BANKNUMCFG_Pos));

    return DAL_OK;
}

/**
  * @brief  Returns the indicated DMC SDRAM mode status.
  * @param  Device Pointer to SDRAM device instance
  * @retval The DMC SDRAM bank mode status, could be on of the following values:
  *         DMC_SDRAM_NORMAL_MODE, DMC_SDRAM_SELF_REFRESH_MODE or
  *         DMC_SDRAM_POWER_DOWN_MODE.
  */
uint32_t DMC_SDRAM_GetModeStatus(DMC_SDRAM_TypeDef *Device)
{
    uint32_t status = DMC_SDRAM_NORMAL_MODE;
    
    /* Check the parameters */
    ASSERT_PARAM(IS_DMC_SDRAM_DEVICE(Device));

    if ((Device->CTRL1 & DMC_CTRL1_PDMEN) != RESET)
    {
        return DMC_SDRAM_POWER_DOWN_MODE;
    }
    else if ((Device->CTRL1 & DMC_CTRL1_SRMEN) && (Device->CTRL1 & DMC_CTRL1_FRBSREN) && \
             (Device->CTRL1 & DMC_CTRL1_SRMFLG))
    {
        return DMC_SDRAM_SELF_REFRESH_MODE;
    }

    return status;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DMC */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_SDRAM_MODULE_ENABLED */
/**
  * @}
  */
/**
  * @}
  */
