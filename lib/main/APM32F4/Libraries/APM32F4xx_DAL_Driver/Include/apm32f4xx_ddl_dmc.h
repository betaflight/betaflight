/**
  *
  * @file    apm32f4xx_ddl_dmc.h
  * @brief   Header file of DMC DDL module.
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
#ifndef APM32F4xx_DDL_DMC_H
#define APM32F4xx_DDL_DMC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup DMC_DDL
  * @{
  */

/** @addtogroup DMC_DDL_Private_Macros
  * @{
  */

#if defined(DMC)

#define DMC_TIMEOUT_VALUE                    100U

#define IS_DMC_SDRAM_DEVICE(__INSTANCE__)    ((__INSTANCE__) == DMC_SDRAM_DEVICE)

#define IS_DMC_BANK_WIDTH(__WIDTH__)         (((__WIDTH__) == DMC_SDRAM_BANK_WIDTH_BITS_NUM_1)  || \
                                              ((__WIDTH__) == DMC_SDRAM_BANK_WIDTH_BITS_NUM_2))

#define IS_DMC_COLUMNBITS_NUMBER(__COLUMN__) (((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_8)  || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_9)  || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_10) || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_11) || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_12) || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_13) || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_14) || \
                                              ((__COLUMN__) == DMC_SDRAM_COLUMN_BITS_NUM_15))

#define IS_DMC_ROWBITS_NUMBER(__ROW__)       (((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_11) || \
                                              ((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_12) || \
                                              ((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_13) || \
                                              ((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_14) || \
                                              ((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_15) || \
                                              ((__ROW__) == DMC_SDRAM_ROW_BITS_NUM_16))
#define IS_DMC_MEMORY_WIDTH(__WIDTH__)        (((__WIDTH__) == DMC_SDRAM_MEM_BUS_WIDTH_16))

#define IS_DMC_CLK_PHASE(__PHASE__)          (((__PHASE__) == DMC_SDRAM_CLK_PHASE_NORMAL)  || \
                                              ((__PHASE__) == DMC_SDRAM_CLK_PHASE_REVERSE))

#define IS_DMC_RD_DELAY(__STATUS__)          (((__STATUS__) == DMC_SDRAM_RD_DELAY_ENABLE)  || \
                                              ((__STATUS__) == DMC_SDRAM_RD_DELAY_DISABLE))

#define IS_DMC_RD_DELAY_CLK(__VALUE__)         ((__VALUE__) <= 7U)

#define IS_DMC_WRITE_PIPE(__STATUS__)         (((__STATUS__) == DMC_SDRAM_WRITE_PIPE_ENABLE)  || \
                                              ((__STATUS__) == DMC_SDRAM_WRITE_PIPE_DISABLE))

#define IS_DMC_ACCELERATE_MODE(__STATUS__)    (((__STATUS__) == DMC_SDRAM_ACCELERATE_MODE_ENABLE)  || \
                                              ((__STATUS__) == DMC_SDRAM_ACCELERATE_MODE_DISABLE))

#define IS_DMC_WRAP_BURST(__INC__)           (((__INC__) == DMC_SDRAM_WRAP_BURST_INC4)  || \
                                              ((__INC__) == DMC_SDRAM_WRAP_BURST_INC8))

#define IS_DMC_CAS_LATENCY(__LATENCY__)      (((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_1) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_2) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_3) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_4) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_5) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_6) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_7) || \
                                              ((__LATENCY__) == DMC_SDRAM_CAS_LATENCY_8))

#define IS_DMC_RAS_TIME(__TIME__)            (((__TIME__) > 0U) && ((__TIME__) <= 16U))
#define IS_DMC_RAS_TO_CAS_DELAY(__DELAY__)    (((__DELAY__) > 0U) && ((__DELAY__) <= 8U))
#define IS_DMC_PRECHARGE_PERIOD(__PERIOD__)   (((__PERIOD__) > 0U) && ((__PERIOD__) <= 8U))
#define IS_DMC_AUTO_REFRESH_TIME(__TIME__)    (((__TIME__) > 0U) && ((__TIME__) <= 16U))
#define IS_DMC_WRITE_RECOVERY_TIME(__TIME__)  (((__TIME__) > 0U) && ((__TIME__) <= 4U))
#define IS_DMC_XSR_TIME(__TIME__)             ((__TIME__) <= 0x1FFU)
#define IS_DMC_ACTIVE_COMMAND_TIME(__TIME__)  (((__TIME__) > 0U) && ((__TIME__) <= 16U))
#define IS_DMC_REFRESH_PERIOD(__PERIOD__)     ((__PERIOD__) <= 0xFFFFU)
#define IS_DMC_STABLE_TIME(__TIME__)          ((__TIME__) <= 0xFFFFU)

#define IS_DMC_POWER_DOWN_MODE(__MODE__)     (((__MODE__) == DMC_SDRAM_POWER_DOWN_ENABLE)  || \
                                              ((__MODE__) == DMC_SDRAM_POWER_DOWN_DISABLE))

#define IS_DMC_SELF_REFRESH_MODE(__MODE__)   (((__MODE__) == DMC_SDRAM_SELF_REFRESH_ENABLE)  || \
                                              ((__MODE__) == DMC_SDRAM_SELF_REFRESH_DISABLE))

#define IS_DMC_REFRESH_TYPE(__TYPE__)        (((__TYPE__) == DMC_SDRAM_REFRESH_TYPE_ROW_ONE)  || \
                                              ((__TYPE__) == DMC_SDRAM_REFRESH_TYPE_ROW_ALL))

#define IS_DMC_PRECHARGE_MODE(__MODE__)      (((__MODE__) == DMC_SDRAM_PRECHARGE_MODE_IM)  || \
                                              ((__MODE__) == DMC_SDRAM_PRECHARGE_MODE_DELAY))

#define IS_DMC_REG_INSERT_NUMBER(__NUMBER__)  ((__NUMBER__) <= 0x7U)

#define IS_DMC_OPEN_BANK_NUMBER(__NUMBER__)     (((__NUMBER__) > 0U) && ((__NUMBER__) <= 16U))

#endif /* DMC */

/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup DMC_DDL_Exported_typedef DMC Low Layer Exported Types
  * @{
  */

#if defined(DMC)
#define DMC_SDRAM_TypeDef              DMC_TypeDef
#endif /* DMC */

#if defined(DMC)
#define DMC_SDRAM_DEVICE               DMC
#endif /* DMC */

#if defined(DMC)
/**
  * @brief  DMC SDRAM Configuration Structure definition
  */
typedef struct
{
  uint32_t BankWidth;                   /*!< Defines the number of bits of bank width.
                                             This parameter can be a value of @ref DMC_SDRAM_Bank_Width_Bits_number. */

  uint32_t ColumnBitsNumber;            /*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref DMC_SDRAM_Column_Bits_number. */

  uint32_t RowBitsNumber;               /*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref DMC_SDRAM_Row_Bits_number.    */

  uint32_t MemoryDataWidth;             /*!< Defines the memory device width.
                                             This parameter can be a value of @ref DMC_SDRAM_Memory_Bus_Width.   */

  uint32_t ClockPhase;                  /*!< Defines the clock phase.
                                             This parameter can be a value of @ref DMC_SDRAM_Clock_Phase.   */

  uint32_t RDDelay;                     /*!< Defines the RD delay enable status.
                                             This parameter can be a value of @ref DMC_SDRAM_RD_Delay_Enable.   */

  uint32_t RDDelayClk;                  /*!< Defines the RD delay clock.
                                             This parameter can be a value between Min_Data = 0 and Max_Data = 7  */

  uint32_t WritePipe;                   /*!< Defines the Write pipe enable status.
                                             This parameter can be a value of @ref DMC_SDRAM_Write_Pipe_Enable.   */

  uint32_t AccelerateMode;              /*!< Defines the DMC accelerate mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Accelerate_Mode.   */

  uint32_t WRAPBurstType;               /*!< Defines the DMC WRAP burst type.
                                             This parameter can be a value of @ref DMC_SDRAM_WRAP_Burst_Type.   */

  uint32_t SelfRefreshMode;             /*!< Defines the DMC Self refresh mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Self_Refresh_Mode.   */

  uint32_t PowerDownMode;               /*!< Defines the DMC Power down mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Power_Down_Mode.   */

  uint32_t RefreshTypeEnterSelfRefresh; /*!< Defines the DMC refresh type before entering self-refresh mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Refresh_Type.   */

  uint32_t RefreshTypeExitSelfRefresh;  /*!< Defines the DMC refresh type after exit self-refresh mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Refresh_Type.   */

  uint32_t RegisterInsertNumber;        /*!< Defines the number of registers inserted in read data path.
                                             This parameter can be a value between Min_Data = 0 and Max_Data = 7  */

  uint32_t OpenBankNumber;              /*!< Defines the number of open banks.
                                             This parameter can be a value between Min_Data = 1 and Max_Data = 16  */
} DMC_SDRAM_InitTypeDef;

/**
  * @brief DMC SDRAM Timing parameters structure definition
  */
typedef struct
{
  uint32_t CASLatency;                  /*!< Defines the SDRAM CAS latency in number of memory clock cycles.
                                             This parameter can be a value of @ref DMC_SDRAM_CAS_Latency.        */

  uint32_t RASTime;                     /*!< Defines the minimum RAS time in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t RASToCASDelay;               /*!< Defines the delay between RAS and CAS in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 8  */

  uint32_t PrechargeMode;               /*!< Defines the SDRAM Precharge mode.
                                             This parameter can be a value of @ref DMC_SDRAM_Precharge_Mode.        */

  uint32_t PrechargePeriod;             /*!< Defines the period for precharge.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 8  */

  uint32_t AutoRefreshTime;              /*!< Defines the Auto Refresh period in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t AutoRefreshNumber;            /*!< Defines the Auto Refresh number.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t WriteRecoveryTime;            /*!< Defines the Write recovery Time in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 4  */

  uint32_t XSRTime;                      /*!< Defines the delay between a Active Command and time of exit self refresh
                                              in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 0x000 and Max_Data = 0x1FF  */

  uint32_t ActiveCommandPeriod;          /*!< Defines the period for active command.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t RefreshPeriod;                /*!< Defines the period between two continuous refresh in number of memory clock
                                              cycles.
                                              This parameter can be a value between Min_Data = 0x0000 and Max_Data = 0xFFFF  */

  uint32_t StableTime;                   /*!< Defines the stable time in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 0x0000 and Max_Data = 0xFFFF  */
} DMC_SDRAM_TimingTypeDef;

#endif /* DMC */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @addtogroup DMC_DDL_Exported_Constants DMC Low Layer Exported Constants
  * @{
  */

#if defined(DMC)
/** @defgroup DMC_DDL_SDRAM_Controller DMC SDRAM Controller
  * @{
  */

/** @defgroup DMC_SDRAM_Bank_Width_Bits_number DMC SDRAM Bank Width Bits number
  * @{
  */
#define DMC_SDRAM_BANK_WIDTH_BITS_NUM_1         (0x00UL << DMC_CFG_BAWCFG_Pos)
#define DMC_SDRAM_BANK_WIDTH_BITS_NUM_2         (0x01UL << DMC_CFG_BAWCFG_Pos)


/** @defgroup DMC_SDRAM_Row_Bits_number DMC SDRAM Row Bits number
  * @{
  */
#define DMC_SDRAM_ROW_BITS_NUM_11               (0x0AUL << DMC_CFG_RAWCFG_Pos)
#define DMC_SDRAM_ROW_BITS_NUM_12               (0x0BUL << DMC_CFG_RAWCFG_Pos)
#define DMC_SDRAM_ROW_BITS_NUM_13               (0x0CUL << DMC_CFG_RAWCFG_Pos)
#define DMC_SDRAM_ROW_BITS_NUM_14               (0x0DUL << DMC_CFG_RAWCFG_Pos)
#define DMC_SDRAM_ROW_BITS_NUM_15               (0x0EUL << DMC_CFG_RAWCFG_Pos)
#define DMC_SDRAM_ROW_BITS_NUM_16               (0x0FUL << DMC_CFG_RAWCFG_Pos)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Column_Bits_number DMC SDRAM Column Bits number
  * @{
  */
#define DMC_SDRAM_COLUMN_BITS_NUM_8             (0x07UL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_9             (0x08UL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_10            (0x09UL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_11            (0x0AUL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_12            (0x0BUL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_13            (0x0CUL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_14            (0x0DUL << DMC_CFG_CAWCFG_Pos)
#define DMC_SDRAM_COLUMN_BITS_NUM_15            (0x0EUL << DMC_CFG_CAWCFG_Pos)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Memory_Bus_Width DMC SDRAM Memory Bus Width
  * @{
  */
#define DMC_SDRAM_MEM_BUS_WIDTH_16              (0x00UL << DMC_CFG_DWCFG_Pos)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Clock_Phase DMC SDRAM Clock Phase
  * @{
  */
#define DMC_SDRAM_CLK_PHASE_NORMAL              (0x00000000U)
#define DMC_SDRAM_CLK_PHASE_REVERSE             (0x00000001U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_RD_Delay_Enable DMC SDRAM RD Delay Enable
  * @{
  */
#define DMC_SDRAM_RD_DELAY_ENABLE              (0x00000000U)
#define DMC_SDRAM_RD_DELAY_DISABLE             (0x00000002U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Write_Pipe_Enable DMC SDRAM Write Pipe Enable
  * @{
  */
#define DMC_SDRAM_WRITE_PIPE_ENABLE             (0x00000020U)
#define DMC_SDRAM_WRITE_PIPE_DISABLE            (0x00000000U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Accelerate_Mode DMC SDRAM Accelerate Mode
  * @{
  */
#define DMC_SDRAM_ACCELERATE_MODE_ENABLE        (0x00000040U)
#define DMC_SDRAM_ACCELERATE_MODE_DISABLE       (0x00000000U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_WRAP_Burst_Type DMC SDRAM WRAP Burst Type
  * @{
  */
#define DMC_SDRAM_WRAP_BURST_INC4               (0x00000000U)
#define DMC_SDRAM_WRAP_BURST_INC8               (0x00000080U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_CAS_Latency DMC SDRAM CAS Latency
  * @{
  */
#define DMC_SDRAM_CAS_LATENCY_1                 (0x00000000U)
#define DMC_SDRAM_CAS_LATENCY_2                 (0x00000001U)
#define DMC_SDRAM_CAS_LATENCY_3                 (0x00000002U)
#define DMC_SDRAM_CAS_LATENCY_4                 (0x00000003U)
#define DMC_SDRAM_CAS_LATENCY_5                 (0x00000004U)
#define DMC_SDRAM_CAS_LATENCY_6                 (0x00000005U)
#define DMC_SDRAM_CAS_LATENCY_7                 (0x00000006U)
#define DMC_SDRAM_CAS_LATENCY_8                 (0x00000007U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Precharge_Mode DMC SDRAM Precharge Mode
  * @{
  */
#define DMC_SDRAM_PRECHARGE_MODE_IM             (0x00000000U)
#define DMC_SDRAM_PRECHARGE_MODE_DELAY          (0x00000008U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Self_Refresh_Mode DMC SDRAM Self Refresh Mode
  * @{
  */
#define DMC_SDRAM_SELF_REFRESH_ENABLE           (0x00000002U)
#define DMC_SDRAM_SELF_REFRESH_DISABLE          (0x00000000U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Power_Down_Mode DMC SDRAM Power Down Mode
  * @{
  */
#define DMC_SDRAM_POWER_DOWN_ENABLE             (0x00000004U)
#define DMC_SDRAM_POWER_DOWN_DISABLE            (0x00000000U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Refresh_Type DMC SDRAM Refresh Type
  * @{
  */
#define DMC_SDRAM_REFRESH_TYPE_ROW_ONE          (0x00000000U)
#define DMC_SDRAM_REFRESH_TYPE_ROW_ALL          (0x00000001U)
/**
  * @}
  */

/** @defgroup DMC_SDRAM_Mode_Status DMC SDRAM Mode Status
  * @{
  */
#define DMC_SDRAM_NORMAL_MODE                   (0x00000000U)
#define DMC_SDRAM_SELF_REFRESH_MODE             (0x00000001U)
#define DMC_SDRAM_POWER_DOWN_MODE               (0x00000002U)
/**
  * @}
  */

/**
  * @}
  */

#endif /* DMC */

/** @defgroup DMC_DDL_Interrupt_definition DMC Low Layer Interrupt definition
  * @{
  */

/** @defgroup DMC_Flag_definition DMC Flag definition
  * @{
  */
#define DMC_SDRAM_FLAG_SELF_REFRESH             ((uint32_t)DMC_CTRL1_SRMFLG)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup DMC_DDL_Private_Macros DMC_DDL  Private Macros
  * @{
  */

#if defined(DMC)

/** @defgroup DMC_DDL_NOR_Macros DMC SDRAM Macros
  * @brief macros to handle SDRAM device switch operations
  * @{
  */

/**
  * @brief  Switch to the DMC Controller.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_ENABLE(__INSTANCE__)            ((__INSTANCE__)->SW |= DMC_SW_MCSW)

/**
  * @brief  Switch to the SMC Controller.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_DISABLE(__INSTANCE__)           ((__INSTANCE__)->SW &= ~DMC_SW_MCSW)

/**
  * @}
  */

/** @defgroup DMC_DDL_NOR_Macros DMC SDRAM Macros
  * @brief macros to handle SDRAM device control operations
  * @{
  */

/**
  * @brief  Update the SDRAM Mode Setup.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_UPDATE_MODE_SETUP(__INSTANCE__)             ((__INSTANCE__)->CTRL1 |= DMC_CTRL1_MODESET)

/**
  * @brief  Enable SDRAM self-refresh mode.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_SELF_REFRESH_MODE_ENABLE(__INSTANCE__)      ((__INSTANCE__)->CTRL1 |= DMC_CTRL1_SRMEN)

/**
  * @brief  Disable SDRAM self-refresh mode.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_SELF_REFRESH_MODE_DISABLE(__INSTANCE__)     ((__INSTANCE__)->CTRL1 &= ~DMC_CTRL1_SRMEN)

/**
  * @brief  Enable SDRAM power down mode.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_POWER_DOWN_MODE_ENABLE(__INSTANCE__)        ((__INSTANCE__)->CTRL1 |= DMC_CTRL1_PDMEN)

/**
  * @brief  Disable SDRAM power down mode.
  * @param  __INSTANCE__ DMC_SDRAM Instance
  * @retval None
  */
#define __DMC_SDRAM_POWER_DOWN_MODE_DISABLE(__INSTANCE__)       ((__INSTANCE__)->CTRL1 &= ~DMC_CTRL1_PDMEN)

/**
  * @}
  */

/** @defgroup DMC_DDL_SDRAM_Interrupt DMC SDRAM Interrupt
  * @brief macros to handle SDRAM interrupts
  * @{
  */

/**
  * @brief  Get flag status of the SDRAM device.
  * @param  __INSTANCE__ DMC_SDRAM instance
  * @param  __FLAG__     DMC_SDRAM flag
  *         This parameter can be any combination of the following values:
  *            @arg DMC_SDRAM_FLAG_SELF_REFRESH: Self-refresh mode flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define __FMC_SDRAM_GET_FLAG(__INSTANCE__, __FLAG__)  (((__INSTANCE__)->CTRL1 & (__FLAG__)) == (__FLAG__))


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

/* Private functions ---------------------------------------------------------*/
/** @defgroup DMC_DDL_Private_Functions DMC DDL Private Functions
  *  @{
  */

#if defined(DMC)
/** @defgroup DMC_DDL_SDRAM SDRAM
  *  @{
  */
/** @defgroup DMC_DDL_SDRAM_Private_Functions_Group1 SDRAM Initialization/de-initialization functions
  *  @{
  */
DAL_StatusTypeDef  DMC_SDRAM_Init(DMC_SDRAM_TypeDef *Device, DMC_SDRAM_InitTypeDef *Init);
DAL_StatusTypeDef  DMC_SDRAM_Timing_Init(DMC_SDRAM_TypeDef *Device,
                                         DMC_SDRAM_TimingTypeDef *Timing);
DAL_StatusTypeDef  DMC_SDRAM_DeInit(DMC_SDRAM_TypeDef *Device);
/**
  * @}
  */

/** @defgroup DMC_DDL_SDRAM_Private_Functions_Group2 SDRAM Control functions
  *  @{
  */
DAL_StatusTypeDef  DMC_SDRAM_ProgramRefreshPeriod(DMC_SDRAM_TypeDef *Device, uint32_t RefreshPeriod);
DAL_StatusTypeDef DMC_SDRAM_SetOpenBankNumber(DMC_SDRAM_TypeDef *Device, uint32_t OpenBankNumber);
uint32_t DMC_SDRAM_GetModeStatus(DMC_SDRAM_TypeDef *Device);
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_DMC_H */
