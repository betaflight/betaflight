/**
  *
  * @file    apm32f4xx_dal_qspi_ex.c
  * @brief   Extended QSPI DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the QSPI peripheral:
  *           + QSPI XIP functions
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
  * The original code has been modified by Geehy Semiconductor.
  * Copyright (c) 2017 STMicroelectronics. Copyright (C) 2025 Geehy Semiconductor.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    *** XIP functional mode ***
    =========================
    [..]
      (#) DAL_QSPIEx_MemoryMapped() to configure the QSPI XIP memory mapped mode.
  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

#if defined(QSPI)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup QSPIEx QSPIEx
 * @brief QSPI Extended DAL module driver
  * @{
  */

#ifdef DAL_QSPI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static DAL_StatusTypeDef DAL_QSPIEx_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag, FlagStatus State, uint32_t TickStart, uint32_t Timeout);

/* Exported functions --------------------------------------------------------*/
/** @defgroup QSPIEx_Exported_Functions QSPIEx Exported Functions
  * @{
  */

/** @defgroup QSPIEx_Exported_Functions_Group1 QSPI XIP functions
 * @brief    QSPI XIP functions
 *
 * @verbatim
 * ===============================================================================
 *             ##### QSPI XIP functions #####
 * ===============================================================================
 *   [..] This section provides functions allowing to configure XIP feature
 * @endverbatim
  * @{
  */

#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)

/**
 * @brief  Configure the QSPI XIP memory mapped mode.
 * @param  hqspi: QSPI handle
 * @param  xipConfig: structure the contains the XIP configuration information.
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPIEx_MemoryMapped(QSPI_HandleTypeDef *hqspi, QSPI_XIPTypeDef *xipConfig)
{
    DAL_StatusTypeDef status;
    uint32_t tickstart = DAL_GetTick();

    /* Check the parameters */
    ASSERT_PARAM(IS_QSPI_XIP_INSTRUCTION(xipConfig->Instruction));
    ASSERT_PARAM(IS_QSPI_XIP_WRAP_CODE(xipConfig->WrapCode));
    ASSERT_PARAM(IS_QSPI_XIP_ADDRESS_SIZE(xipConfig->AddressSize));
    ASSERT_PARAM(IS_QSPI_XIP_INSTRUCTION_MODE(xipConfig->InstructionMode));
    ASSERT_PARAM(IS_QSPI_XIP_INSTRUCTION_SIZE(xipConfig->InstructionSize));
    ASSERT_PARAM(IS_QSPI_XIP_FRAME_FORMAT(xipConfig->FrameFormat));
    ASSERT_PARAM(IS_QSPI_XIP_DUMMY_CYCLES(xipConfig->DummyCycles));
    ASSERT_PARAM(IS_QSPI_XIP_ENDIANNES(xipConfig->Endianness));

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_BUSY;

        status = DAL_QSPIEx_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);

        if (status == DAL_OK)
        {
            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

          /* Configure endianness for external memory access */
          __DAL_QSPI_XIP_SET_MEM_ACCESS_FORMAT(hqspi, xipConfig->Endianness);

          /* Enable QSPI XIP memory access */
          __DAL_QSPI_XIP_MEM_ACCESS_ENABLE(hqspi);

          /* Flash support instruction state */
          if (xipConfig->InstructionSize != QSPI_XIP_INSTRUCTION_SIZE_NONE)
          {
              WRITE_REG(hqspi->Instance->XIP_INCR_INST, xipConfig->Instruction);
              WRITE_REG(hqspi->Instance->XIP_WRAP_INST, xipConfig->WrapCode);

              /* Enable XIP instruction state */
              __DAL_QSPI_XIP_INST_ENABLE(hqspi);
          }
          else
          {
              /* Disable XIP instruction state */
              __DAL_QSPI_XIP_INST_DISABLE(hqspi);
          }

          /* Configure XIP parameters */
          MODIFY_REG(hqspi->Instance->XIP_CTRL,
                    (QSPI_XIP_CTRL_ADDR_L | QSPI_XIP_CTRL_TRANS_TYPE | \
                    QSPI_XIP_CTRL_INST_L | QSPI_XIP_CTRL_FRF | \
                    QSPI_XIP_CTRL_WAIT_CYCLES | QSPI_XIP_CTRL_CONT_XFER_EN | \
                    QSPI_XIP_CTRL_XIP_PREFETCH_EN), \
                    (xipConfig->AddressSize | xipConfig->InstructionMode | \
                    xipConfig->InstructionSize | xipConfig->FrameFormat | \
                    (xipConfig->DummyCycles << QSPI_XIP_CTRL_WAIT_CYCLES_Pos) | \
                    (xipConfig->ContinuousMode << QSPI_XIP_CTRL_CONT_XFER_EN_Pos) | \
                    (xipConfig->PrefetchMode << QSPI_XIP_CTRL_XIP_PREFETCH_EN_Pos)));

            /* Enable XIP Slave Select signal */
            __DAL_QSPI_XIP_ENABLE_SS(hqspi);

            /* Enable the QSPI peripheral */
            __DAL_QSPI_ENABLE(hqspi);

            /* Enable XIP */
            __DAL_QSPI_XIP_ENABLE(hqspi);
        }
        else
        {
            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    /* Return function status */
    return status;
}

/**
 * @brief  Wait for a flag state until timeout
 * @param  hqspi: QSPI handle
 * @param  Flag: Flag checked
 * @param  State: Value of the flag expected
 * @param  TickStart: Tick start value
 * @param  Timeout: Timeout duration
 * @retval DAL status
 */
static DAL_StatusTypeDef DAL_QSPIEx_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag, FlagStatus State, uint32_t TickStart, uint32_t Timeout)
{
    /* Wait until flag is in expected state */
    while ((__DAL_QSPI_GET_FLAG(hqspi, Flag)) != State)
    {
        /* Check for the Timeout */
        if (Timeout != DAL_MAX_DELAY)
        {
            if (((DAL_GetTick() - TickStart) > Timeout) || (Timeout == 0U))
            {
                hqspi->State = DAL_QSPI_STATE_ERROR;
                hqspi->ErrorCode |= DAL_QSPI_ERROR_TIMEOUT;

                return DAL_ERROR;
            }
        }
    }

    return DAL_OK;
}

#endif /* APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @}
  */


#endif /* DAL_QSPI_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

#endif /* QSPI */
