/**
  *
  * @file    apm32f4xx_ddl_smc.h
  * @brief   Header file of SMC DDL module.
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
#ifndef APM32F4xx_DDL_SMC_H
#define APM32F4xx_DDL_SMC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup SMC_DDL
  * @{
  */

/** @addtogroup SMC_DDL_Private_Macros
  * @{
  */
#if defined(SMC_Bank1)

#define IS_SMC_NORSRAM_BANK(__BANK__) (((__BANK__) == SMC_NORSRAM_BANK1) || \
                                       ((__BANK__) == SMC_NORSRAM_BANK2) || \
                                       ((__BANK__) == SMC_NORSRAM_BANK3) || \
                                       ((__BANK__) == SMC_NORSRAM_BANK4))
#define IS_SMC_MUX(__MUX__) (((__MUX__) == SMC_DATA_ADDRESS_MUX_DISABLE) || \
                             ((__MUX__) == SMC_DATA_ADDRESS_MUX_ENABLE))
#define IS_SMC_MEMORY(__MEMORY__) (((__MEMORY__) == SMC_MEMORY_TYPE_SRAM) || \
                                   ((__MEMORY__) == SMC_MEMORY_TYPE_PSRAM)|| \
                                   ((__MEMORY__) == SMC_MEMORY_TYPE_NOR))
#define IS_SMC_NORSRAM_MEMORY_WIDTH(__WIDTH__) (((__WIDTH__) == SMC_NORSRAM_MEM_BUS_WIDTH_8)  || \
                                                ((__WIDTH__) == SMC_NORSRAM_MEM_BUS_WIDTH_16) || \
                                                ((__WIDTH__) == SMC_NORSRAM_MEM_BUS_WIDTH_32))
#define IS_SMC_PAGESIZE(__SIZE__) (((__SIZE__) == SMC_PAGE_SIZE_NONE) || \
                                   ((__SIZE__) == SMC_PAGE_SIZE_128) || \
                                   ((__SIZE__) == SMC_PAGE_SIZE_256) || \
                                   ((__SIZE__) == SMC_PAGE_SIZE_512) || \
                                   ((__SIZE__) == SMC_PAGE_SIZE_1024))
#if defined(SMC_CSCTRL1_WFDIS)
#define IS_SMC_WRITE_FIFO(__FIFO__) (((__FIFO__) == SMC_WRITE_FIFO_DISABLE) || \
                                     ((__FIFO__) == SMC_WRITE_FIFO_ENABLE))
#endif /* SMC_CSCTRL1_WFDIS */
#define IS_SMC_ACCESS_MODE(__MODE__) (((__MODE__) == SMC_ACCESS_MODE_A) || \
                                      ((__MODE__) == SMC_ACCESS_MODE_B) || \
                                      ((__MODE__) == SMC_ACCESS_MODE_C) || \
                                      ((__MODE__) == SMC_ACCESS_MODE_D))
#define IS_SMC_BURSTMODE(__STATE__) (((__STATE__) == SMC_BURST_ACCESS_MODE_DISABLE) || \
                                     ((__STATE__) == SMC_BURST_ACCESS_MODE_ENABLE))
#define IS_SMC_WAIT_POLARITY(__POLARITY__) (((__POLARITY__) == SMC_WAIT_SIGNAL_POLARITY_LOW) || \
                                            ((__POLARITY__) == SMC_WAIT_SIGNAL_POLARITY_HIGH))
#define IS_SMC_WRAP_MODE(__MODE__) (((__MODE__) == SMC_WRAP_MODE_DISABLE) || \
                                             ((__MODE__) == SMC_WRAP_MODE_ENABLE))
#define IS_SMC_WAIT_SIGNAL_ACTIVE(__ACTIVE__) (((__ACTIVE__) == SMC_WAIT_TIMING_BEFORE_WS) || \
                                               ((__ACTIVE__) == SMC_WAIT_TIMING_DURING_WS))
#define IS_SMC_WRITE_OPERATION(__OPERATION__) (((__OPERATION__) == SMC_WRITE_OPERATION_DISABLE) || \
                                               ((__OPERATION__) == SMC_WRITE_OPERATION_ENABLE))
#define IS_SMC_WAITE_SIGNAL(__SIGNAL__) (((__SIGNAL__) == SMC_WAIT_SIGNAL_DISABLE) || \
                                         ((__SIGNAL__) == SMC_WAIT_SIGNAL_ENABLE))
#define IS_SMC_EXTENDED_MODE(__MODE__) (((__MODE__) == SMC_EXTENDED_MODE_DISABLE) || \
                                        ((__MODE__) == SMC_EXTENDED_MODE_ENABLE))
#define IS_SMC_ASYNWAIT(__STATE__) (((__STATE__) == SMC_ASYNCHRONOUS_WAIT_DISABLE) || \
                                    ((__STATE__) == SMC_ASYNCHRONOUS_WAIT_ENABLE))
#define IS_SMC_DATA_LATENCY(__LATENCY__) (((__LATENCY__) > 1U) && ((__LATENCY__) <= 17U))
#define IS_SMC_WRITE_BURST(__BURST__) (((__BURST__) == SMC_WRITE_BURST_DISABLE) || \
                                       ((__BURST__) == SMC_WRITE_BURST_ENABLE))
#define IS_SMC_CONTINOUS_CLOCK(__CCLOCK__) (((__CCLOCK__) == SMC_CONTINUOUS_CLOCK_SYNC_ONLY) || \
                                            ((__CCLOCK__) == SMC_CONTINUOUS_CLOCK_SYNC_ASYNC))
#define IS_SMC_ADDRESS_SETUP_TIME(__TIME__) ((__TIME__) <= 15U)
#define IS_SMC_ADDRESS_HOLD_TIME(__TIME__) (((__TIME__) > 0U) && ((__TIME__) <= 15U))
#define IS_SMC_DATASETUP_TIME(__TIME__) (((__TIME__) > 0U) && ((__TIME__) <= 255U))
#define IS_SMC_DATAHOLD_DURATION(__DATAHOLD__) ((__DATAHOLD__) <= 3U)
#define IS_SMC_TURNAROUND_TIME(__TIME__) ((__TIME__) <= 15U)
#define IS_SMC_CLK_DIV(__DIV__) (((__DIV__) > 1U) && ((__DIV__) <= 16U))
#define IS_SMC_NORSRAM_DEVICE(__INSTANCE__) ((__INSTANCE__) == SMC_NORSRAM_DEVICE)
#define IS_SMC_NORSRAM_EXTENDED_DEVICE(__INSTANCE__) ((__INSTANCE__) == SMC_NORSRAM_EXTENDED_DEVICE)

#endif /* SMC_Bank1 */
#if  defined(SMC_Bank2_3)

#define IS_SMC_NAND_BANK(__BANK__) (((__BANK__) == SMC_NAND_BANK2) || \
                                            ((__BANK__) == SMC_NAND_BANK3))
#define IS_SMC_WAIT_FEATURE(__FEATURE__) (((__FEATURE__) == SMC_NAND_PCC_WAIT_FEATURE_DISABLE) || \
                                                   ((__FEATURE__) == SMC_NAND_PCC_WAIT_FEATURE_ENABLE))
#define IS_SMC_NAND_MEMORY_WIDTH(__WIDTH__) (((__WIDTH__) == SMC_NAND_PCC_MEM_BUS_WIDTH_8) || \
                                                      ((__WIDTH__) == SMC_NAND_PCC_MEM_BUS_WIDTH_16))
#define IS_SMC_ECC_STATE(__STATE__) (((__STATE__) == SMC_NAND_ECC_DISABLE) || \
                                     ((__STATE__) == SMC_NAND_ECC_ENABLE))

#define IS_SMC_ECCPAGE_SIZE(__SIZE__) (((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_256BYTE)  || \
                                       ((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_512BYTE)  || \
                                       ((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_1024BYTE) || \
                                       ((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_2048BYTE) || \
                                       ((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_4096BYTE) || \
                                       ((__SIZE__) == SMC_NAND_ECC_PAGE_SIZE_8192BYTE))
#define IS_SMC_TCLR_TIME(__TIME__) ((__TIME__) <= 255U)
#define IS_SMC_TAR_TIME(__TIME__) ((__TIME__) <= 255U)
#define IS_SMC_SETUP_TIME(__TIME__) ((__TIME__) <= 254U)
#define IS_SMC_WAIT_TIME(__TIME__) ((__TIME__) <= 254U)
#define IS_SMC_HOLD_TIME(__TIME__) ((__TIME__) <= 254U)
#define IS_SMC_HIZ_TIME(__TIME__) ((__TIME__) <= 254U)
#define IS_SMC_NAND_DEVICE(__INSTANCE__) ((__INSTANCE__) == SMC_NAND_DEVICE)

#endif /* SMC_Bank2_3 */
#if defined(SMC_Bank4)
#define IS_SMC_PCCARD_DEVICE(__INSTANCE__) ((__INSTANCE__) == SMC_PCCARD_DEVICE)

#endif /* SMC_Bank4 */

/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup SMC_DDL_Exported_typedef SMC Low Layer Exported Types
  * @{
  */

#if defined(SMC_Bank1)
#define SMC_NORSRAM_TypeDef            SMC_Bank1_TypeDef
#define SMC_NORSRAM_EXTENDED_TypeDef   SMC_Bank1E_TypeDef
#endif /* SMC_Bank1 */
#if defined(SMC_Bank2_3)
#define SMC_NAND_TypeDef               SMC_Bank2_3_TypeDef
#endif /* SMC_Bank2_3 */
#if defined(SMC_Bank4)
#define SMC_PCCARD_TypeDef             SMC_Bank4_TypeDef
#endif /* SMC_Bank4 */

#if defined(SMC_Bank1)
#define SMC_NORSRAM_DEVICE             SMC_Bank1
#define SMC_NORSRAM_EXTENDED_DEVICE    SMC_Bank1E
#endif /* SMC_Bank1 */
#if defined(SMC_Bank2_3)
#define SMC_NAND_DEVICE                SMC_Bank2_3
#endif /* SMC_Bank2_3 */
#if defined(SMC_Bank4)
#define SMC_PCCARD_DEVICE              SMC_Bank4
#endif /* SMC_Bank4 */

#if defined(SMC_Bank1)
/**
  * @brief  SMC NORSRAM Configuration Structure definition
  */
typedef struct
{
  uint32_t NSBank;                       /*!< Specifies the NORSRAM memory device that will be used.
                                              This parameter can be a value of @ref SMC_NORSRAM_Bank                  */

  uint32_t DataAddressMux;               /*!< Specifies whether the address and data values are
                                              multiplexed on the data bus or not.
                                              This parameter can be a value of @ref SMC_Data_Address_Bus_Multiplexing */

  uint32_t MemoryType;                   /*!< Specifies the type of external memory attached to
                                              the corresponding memory device.
                                              This parameter can be a value of @ref SMC_Memory_Type                   */

  uint32_t MemoryDataWidth;              /*!< Specifies the external memory device width.
                                              This parameter can be a value of @ref SMC_NORSRAM_Data_Width            */

  uint32_t BurstAccessMode;              /*!< Enables or disables the burst access mode for Flash memory,
                                              valid only with synchronous burst Flash memories.
                                              This parameter can be a value of @ref SMC_Burst_Access_Mode             */

  uint32_t WaitSignalPolarity;           /*!< Specifies the wait signal polarity, valid only when accessing
                                              the Flash memory in burst mode.
                                              This parameter can be a value of @ref SMC_Wait_Signal_Polarity          */

  uint32_t WrapMode;                     /*!< Enables or disables the Wrapped burst access mode for Flash
                                              memory, valid only when accessing Flash memories in burst mode.
                                              This parameter can be a value of @ref SMC_Wrap_Mode
                                              This mode is available only for the APM32F405/407/417xx devices             */

  uint32_t WaitSignalActive;             /*!< Specifies if the wait signal is asserted by the memory one
                                              clock cycle before the wait state or during the wait state,
                                              valid only when accessing memories in burst mode.
                                              This parameter can be a value of @ref SMC_Wait_Timing                   */

  uint32_t WriteOperation;               /*!< Enables or disables the write operation in the selected device by the SMC.
                                              This parameter can be a value of @ref SMC_Write_Operation               */

  uint32_t WaitSignal;                   /*!< Enables or disables the wait state insertion via wait
                                              signal, valid for Flash memory access in burst mode.
                                              This parameter can be a value of @ref SMC_Wait_Signal                   */

  uint32_t ExtendedMode;                 /*!< Enables or disables the extended mode.
                                              This parameter can be a value of @ref SMC_Extended_Mode                 */

  uint32_t AsynchronousWait;             /*!< Enables or disables wait signal during asynchronous transfers,
                                              valid only with asynchronous Flash memories.
                                              This parameter can be a value of @ref SMC_AsynchronousWait              */

  uint32_t WriteBurst;                   /*!< Enables or disables the write burst operation.
                                              This parameter can be a value of @ref SMC_Write_Burst                   */

  uint32_t ContinuousClock;              /*!< Enables or disables the SMC clock output to external memory devices.
                                              This parameter is only enabled through the SMC_CSCTRL1 register,
                                              and don't care through SMC_CSCTRL2..4 registers.
                                              This parameter can be a value of @ref SMC_Continous_Clock
                                              This mode is available only for the APM32F412Vx/Zx/Rx devices           */

  uint32_t WriteFifo;                    /*!< Enables or disables the write FIFO used by the SMC controller.
                                              This parameter is only enabled through the SMC_CSCTRL1 register,
                                              and don't care through SMC_CSCTRL2..4 registers.
                                              This parameter can be a value of @ref SMC_Write_FIFO
                                              This mode is available only for the APM32F412Vx/Vx devices              */

  uint32_t PageSize;                     /*!< Specifies the memory page size.
                                              This parameter can be a value of @ref SMC_Page_Size                     */
} SMC_NORSRAM_InitTypeDef;

/**
  * @brief  SMC NORSRAM Timing parameters structure definition
  */
typedef struct
{
  uint32_t AddressSetupTime;             /*!< Defines the number of HCLK cycles to configure
                                              the duration of the address setup time.
                                              This parameter can be a value between Min_Data = 0 and Max_Data = 15.
                                              @note This parameter is not used with synchronous NOR Flash memories.   */

  uint32_t AddressHoldTime;              /*!< Defines the number of HCLK cycles to configure
                                              the duration of the address hold time.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 15.
                                              @note This parameter is not used with synchronous NOR Flash memories.   */

  uint32_t DataSetupTime;                /*!< Defines the number of HCLK cycles to configure
                                              the duration of the data setup time.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 255.
                                              @note This parameter is used for SRAMs, ROMs and asynchronous multiplexed
                                              NOR Flash memories.                                                     */

  uint32_t BusTurnAroundDuration;        /*!< Defines the number of HCLK cycles to configure
                                              the duration of the bus turnaround.
                                              This parameter can be a value between Min_Data = 0 and Max_Data = 15.
                                              @note This parameter is only used for multiplexed NOR Flash memories.   */

  uint32_t CLKDivision;                  /*!< Defines the period of CLK clock output signal, expressed in number of
                                              HCLK cycles. This parameter can be a value between Min_Data = 2 and
                                              Max_Data = 16.
                                              @note This parameter is not used for asynchronous NOR Flash, SRAM or ROM
                                              accesses.                                                               */

  uint32_t DataLatency;                  /*!< Defines the number of memory clock cycles to issue
                                              to the memory before getting the first data.
                                              The parameter value depends on the memory type as shown below:
                                              - It must be set to 0 in case of a CRAM
                                              - It is don't care in asynchronous NOR, SRAM or ROM accesses
                                              - It may assume a value between Min_Data = 2 and Max_Data = 17
                                                in NOR Flash memories with synchronous burst mode enable              */

  uint32_t AccessMode;                   /*!< Specifies the asynchronous access mode.
                                              This parameter can be a value of @ref SMC_Access_Mode                   */
} SMC_NORSRAM_TimingTypeDef;
#endif /* SMC_Bank1 */

#if defined(SMC_Bank2_3)
/**
  * @brief  SMC NAND Configuration Structure definition
  */
typedef struct
{
  uint32_t NandBank;               /*!< Specifies the NAND memory device that will be used.
                                        This parameter can be a value of @ref SMC_NAND_Bank                  */

  uint32_t Waitfeature;            /*!< Enables or disables the Wait feature for the NAND Memory device.
                                        This parameter can be any value of @ref SMC_Wait_feature             */

  uint32_t MemoryDataWidth;        /*!< Specifies the external memory device width.
                                        This parameter can be any value of @ref SMC_NAND_Data_Width          */

  uint32_t EccComputation;         /*!< Enables or disables the ECC computation.
                                        This parameter can be any value of @ref SMC_ECC                      */

  uint32_t ECCPageSize;            /*!< Defines the page size for the extended ECC.
                                        This parameter can be any value of @ref SMC_ECC_Page_Size            */

  uint32_t TCLRSetupTime;          /*!< Defines the number of HCLK cycles to configure the
                                        delay between CLE low and RE low.
                                        This parameter can be a value between Min_Data = 0 and Max_Data = 255  */

  uint32_t TARSetupTime;           /*!< Defines the number of HCLK cycles to configure the
                                        delay between ALE low and RE low.
                                        This parameter can be a number between Min_Data = 0 and Max_Data = 255 */
} SMC_NAND_InitTypeDef;
#endif

#if defined(SMC_Bank2_3) || defined(SMC_Bank4)
/**
  * @brief  SMC NAND Timing parameters structure definition
  */
typedef struct
{
  uint32_t SetupTime;            /*!< Defines the number of HCLK cycles to setup address before
                                      the command assertion for NAND-Flash read or write access
                                      to common/Attribute or I/O memory space (depending on
                                      the memory space timing to be configured).
                                      This parameter can be a value between Min_Data = 0 and Max_Data = 254    */

  uint32_t WaitSetupTime;        /*!< Defines the minimum number of HCLK cycles to assert the
                                      command for NAND-Flash read or write access to
                                      common/Attribute or I/O memory space (depending on the
                                      memory space timing to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254   */

  uint32_t HoldSetupTime;        /*!< Defines the number of HCLK clock cycles to hold address
                                      (and data for write access) after the command de-assertion
                                      for NAND-Flash read or write access to common/Attribute
                                      or I/O memory space (depending on the memory space timing
                                      to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254   */

  uint32_t HiZSetupTime;         /*!< Defines the number of HCLK clock cycles during which the
                                      data bus is kept in HiZ after the start of a NAND-Flash
                                      write access to common/Attribute or I/O memory space (depending
                                      on the memory space timing to be configured).
                                      This parameter can be a number between Min_Data = 0 and Max_Data = 254   */
} SMC_NAND_PCC_TimingTypeDef;
#endif /* SMC_Bank2_3 */

#if defined(SMC_Bank4)
/**
  * @brief SMC PCCARD Configuration Structure definition
  */
typedef struct
{
  uint32_t Waitfeature;            /*!< Enables or disables the Wait feature for the PCCARD Memory device.
                                        This parameter can be any value of @ref SMC_Wait_feature      */

  uint32_t TCLRSetupTime;          /*!< Defines the number of HCLK cycles to configure the
                                        delay between CLE low and RE low.
                                        This parameter can be a value between Min_Data = 0 and Max_Data = 255  */

  uint32_t TARSetupTime;           /*!< Defines the number of HCLK cycles to configure the
                                        delay between ALE low and RE low.
                                        This parameter can be a number between Min_Data = 0 and Max_Data = 255 */
}SMC_PCCARD_InitTypeDef;
#endif /* SMC_Bank4 */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @addtogroup SMC_DDL_Exported_Constants SMC Low Layer Exported Constants
  * @{
  */
#if defined(SMC_Bank1)

/** @defgroup SMC_DDL_NOR_SRAM_Controller SMC NOR/SRAM Controller
  * @{
  */

/** @defgroup SMC_NORSRAM_Bank SMC NOR/SRAM Bank
  * @{
  */
#define SMC_NORSRAM_BANK1                       (0x00000000U)
#define SMC_NORSRAM_BANK2                       (0x00000002U)
#define SMC_NORSRAM_BANK3                       (0x00000004U)
#define SMC_NORSRAM_BANK4                       (0x00000006U)
/**
  * @}
  */

/** @defgroup SMC_Data_Address_Bus_Multiplexing SMC Data Address Bus Multiplexing
  * @{
  */
#define SMC_DATA_ADDRESS_MUX_DISABLE            (0x00000000U)
#define SMC_DATA_ADDRESS_MUX_ENABLE             (0x00000002U)
/**
  * @}
  */

/** @defgroup SMC_Memory_Type SMC Memory Type
  * @{
  */
#define SMC_MEMORY_TYPE_SRAM                    (0x00000000U)
#define SMC_MEMORY_TYPE_PSRAM                   (0x00000004U)
#define SMC_MEMORY_TYPE_NOR                     (0x00000008U)
/**
  * @}
  */

/** @defgroup SMC_NORSRAM_Data_Width SMC NORSRAM Data Width
  * @{
  */
#define SMC_NORSRAM_MEM_BUS_WIDTH_8             (0x00000000U)
#define SMC_NORSRAM_MEM_BUS_WIDTH_16            (0x00000010U)
#define SMC_NORSRAM_MEM_BUS_WIDTH_32            (0x00000020U)
/**
  * @}
  */

/** @defgroup SMC_NORSRAM_Flash_Access SMC NOR/SRAM Flash Access
  * @{
  */
#define SMC_NORSRAM_FLASH_ACCESS_ENABLE         (0x00000040U)
#define SMC_NORSRAM_FLASH_ACCESS_DISABLE        (0x00000000U)
/**
  * @}
  */

/** @defgroup SMC_Burst_Access_Mode SMC Burst Access Mode
  * @{
  */
#define SMC_BURST_ACCESS_MODE_DISABLE           (0x00000000U)
#define SMC_BURST_ACCESS_MODE_ENABLE            (0x00000100U)
/**
  * @}
  */

/** @defgroup SMC_Wait_Signal_Polarity SMC Wait Signal Polarity
  * @{
  */
#define SMC_WAIT_SIGNAL_POLARITY_LOW            (0x00000000U)
#define SMC_WAIT_SIGNAL_POLARITY_HIGH           (0x00000200U)
/**
  * @}
  */

/** @defgroup SMC_Wrap_Mode SMC Wrap Mode
  * @note  These values are available only for the APM32F405/407/417xx devices.
  * @{
  */
#define SMC_WRAP_MODE_DISABLE                   (0x00000000U)
#define SMC_WRAP_MODE_ENABLE                    (0x00000400U)
/**
  * @}
  */

/** @defgroup SMC_Wait_Timing SMC Wait Timing
  * @{
  */
#define SMC_WAIT_TIMING_BEFORE_WS               (0x00000000U)
#define SMC_WAIT_TIMING_DURING_WS               (0x00000800U)
/**
  * @}
  */

/** @defgroup SMC_Write_Operation SMC Write Operation
  * @{
  */
#define SMC_WRITE_OPERATION_DISABLE             (0x00000000U)
#define SMC_WRITE_OPERATION_ENABLE              (0x00001000U)
/**
  * @}
  */

/** @defgroup SMC_Wait_Signal SMC Wait Signal
  * @{
  */
#define SMC_WAIT_SIGNAL_DISABLE                 (0x00000000U)
#define SMC_WAIT_SIGNAL_ENABLE                  (0x00002000U)
/**
  * @}
  */

/** @defgroup SMC_Extended_Mode SMC Extended Mode
  * @{
  */
#define SMC_EXTENDED_MODE_DISABLE               (0x00000000U)
#define SMC_EXTENDED_MODE_ENABLE                (0x00004000U)
/**
  * @}
  */

/** @defgroup SMC_AsynchronousWait SMC Asynchronous Wait
  * @{
  */
#define SMC_ASYNCHRONOUS_WAIT_DISABLE           (0x00000000U)
#define SMC_ASYNCHRONOUS_WAIT_ENABLE            (0x00008000U)
/**
  * @}
  */

/** @defgroup SMC_Page_Size SMC Page Size
  * @{
  */
#define SMC_PAGE_SIZE_NONE                      (0x00000000U)
#define SMC_PAGE_SIZE_128                       SMC_CSCTRL1_CRAMPSIZECFG_0
#define SMC_PAGE_SIZE_256                       SMC_CSCTRL1_CRAMPSIZECFG_1
#define SMC_PAGE_SIZE_512                       (SMC_CSCTRL1_CRAMPSIZECFG_0\
                                                 | SMC_CSCTRL1_CRAMPSIZECFG_1)
#define SMC_PAGE_SIZE_1024                      SMC_CSCTRL1_CRAMPSIZECFG_2
/**
  * @}
  */

/** @defgroup SMC_Write_Burst SMC Write Burst
  * @{
  */
#define SMC_WRITE_BURST_DISABLE                 (0x00000000U)
#define SMC_WRITE_BURST_ENABLE                  (0x00080000U)
/**
  * @}
  */

/** @defgroup SMC_Continous_Clock SMC Continuous Clock
  * @note  These values are available only for the APM32F412Vx/Zx/Rx devices.
  * @{
  */
#define SMC_CONTINUOUS_CLOCK_SYNC_ONLY          (0x00000000U)
#define SMC_CONTINUOUS_CLOCK_SYNC_ASYNC         (0x00100000U)
/**
  * @}
  */

#if defined(SMC_CSCTRL1_WFDIS)
/** @defgroup SMC_Write_FIFO SMC Write FIFO
  * @note  These values are available only for the APM32F412Vx/Zx/Rx devices.
  * @{
  */
#define SMC_WRITE_FIFO_DISABLE                  SMC_CSCTRL1_WFDIS
#define SMC_WRITE_FIFO_ENABLE                   (0x00000000U)
#endif /* SMC_CSCTRL1_WFDIS */
/**
  * @}
  */

/** @defgroup SMC_Access_Mode SMC Access Mode
  * @{
  */
#define SMC_ACCESS_MODE_A                       (0x00000000U)
#define SMC_ACCESS_MODE_B                       (0x10000000U)
#define SMC_ACCESS_MODE_C                       (0x20000000U)
#define SMC_ACCESS_MODE_D                       (0x30000000U)
/**
  * @}
  */

/**
  * @}
  */
#endif /* SMC_Bank1 */

#if defined(SMC_Bank2_3) || defined(SMC_Bank4)

/** @defgroup SMC_DDL_NAND_Controller SMC NAND Controller
  * @{
  */
/** @defgroup SMC_NAND_Bank SMC NAND Bank
  * @{
  */
#if defined(SMC_Bank2_3)
#define SMC_NAND_BANK2                          (0x00000010U)
#endif
#define SMC_NAND_BANK3                          (0x00000100U)
/**
  * @}
  */

/** @defgroup SMC_Wait_feature SMC Wait feature
  * @{
  */
#define SMC_NAND_PCC_WAIT_FEATURE_DISABLE       (0x00000000U)
#define SMC_NAND_PCC_WAIT_FEATURE_ENABLE        (0x00000002U)
/**
  * @}
  */

/** @defgroup SMC_PCR_Memory_Type SMC PCR Memory Type
  * @{
  */
#if defined(SMC_Bank4)
#define SMC_PCR_MEMORY_TYPE_PCCARD              (0x00000000U)
#endif /* SMC_Bank4 */
#define SMC_PCR_MEMORY_TYPE_NAND                (0x00000008U)
/**
  * @}
  */

/** @defgroup SMC_NAND_Data_Width SMC NAND Data Width
  * @{
  */
#define SMC_NAND_PCC_MEM_BUS_WIDTH_8            (0x00000000U)
#define SMC_NAND_PCC_MEM_BUS_WIDTH_16           (0x00000010U)
/**
  * @}
  */

/** @defgroup SMC_ECC SMC ECC
  * @{
  */
#define SMC_NAND_ECC_DISABLE                    (0x00000000U)
#define SMC_NAND_ECC_ENABLE                     (0x00000040U)
/**
  * @}
  */

/** @defgroup SMC_ECC_Page_Size SMC ECC Page Size
  * @{
  */
#define SMC_NAND_ECC_PAGE_SIZE_256BYTE          (0x00000000U)
#define SMC_NAND_ECC_PAGE_SIZE_512BYTE          (0x00020000U)
#define SMC_NAND_ECC_PAGE_SIZE_1024BYTE         (0x00040000U)
#define SMC_NAND_ECC_PAGE_SIZE_2048BYTE         (0x00060000U)
#define SMC_NAND_ECC_PAGE_SIZE_4096BYTE         (0x00080000U)
#define SMC_NAND_ECC_PAGE_SIZE_8192BYTE         (0x000A0000U)
/**
  * @}
  */

/**
  * @}
  */
#endif /* SMC_Bank2_3 || SMC_Bank4 */


/** @defgroup SMC_DDL_Interrupt_definition SMC Low Layer Interrupt definition
  * @{
  */
#if defined(SMC_Bank2_3) || defined(SMC_Bank4)
#define SMC_IT_RISING_EDGE                      (0x00000008U)
#define SMC_IT_LEVEL                            (0x00000010U)
#define SMC_IT_FALLING_EDGE                     (0x00000020U)
#endif /* SMC_Bank2_3 || SMC_Bank4 */
/**
  * @}
  */

/** @defgroup SMC_DDL_Flag_definition SMC Low Layer Flag definition
  * @{
  */
#if defined(SMC_Bank2_3) || defined(SMC_Bank4)
#define SMC_FLAG_RISING_EDGE                    (0x00000001U)
#define SMC_FLAG_LEVEL                          (0x00000002U)
#define SMC_FLAG_FALLING_EDGE                   (0x00000004U)
#define SMC_FLAG_FEMPT                          (0x00000040U)
#endif /* SMC_Bank2_3 || SMC_Bank4 */
/**
  * @}
  */

/** @defgroup SMC_DDL_Alias_definition  SMC Alias definition
  * @{
  */
#define FMC_WRITE_OPERATION_DISABLE          SMC_WRITE_OPERATION_DISABLE
#define FMC_WRITE_OPERATION_ENABLE           SMC_WRITE_OPERATION_ENABLE

#define FMC_NORSRAM_MEM_BUS_WIDTH_8           SMC_NORSRAM_MEM_BUS_WIDTH_8
#define FMC_NORSRAM_MEM_BUS_WIDTH_16          SMC_NORSRAM_MEM_BUS_WIDTH_16
#define FMC_NORSRAM_MEM_BUS_WIDTH_32          SMC_NORSRAM_MEM_BUS_WIDTH_32

#define FMC_NORSRAM_TypeDef                   SMC_NORSRAM_TypeDef
#define FMC_NORSRAM_EXTENDED_TypeDef          SMC_NORSRAM_EXTENDED_TypeDef
#define FMC_NORSRAM_InitTypeDef               SMC_NORSRAM_InitTypeDef
#define FMC_NORSRAM_TimingTypeDef             SMC_NORSRAM_TimingTypeDef

#define FMC_NORSRAM_Init                      SMC_NORSRAM_Init
#define FMC_NORSRAM_Timing_Init               SMC_NORSRAM_Timing_Init
#define FMC_NORSRAM_Extended_Timing_Init      SMC_NORSRAM_Extended_Timing_Init
#define FMC_NORSRAM_DeInit                    SMC_NORSRAM_DeInit
#define FMC_NORSRAM_WriteOperation_Enable     SMC_NORSRAM_WriteOperation_Enable
#define FMC_NORSRAM_WriteOperation_Disable    SMC_NORSRAM_WriteOperation_Disable

#define __FMC_NORSRAM_ENABLE                  __SMC_NORSRAM_ENABLE
#define __FMC_NORSRAM_DISABLE                 __SMC_NORSRAM_DISABLE

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx)
#define FMC_NAND_InitTypeDef                  SMC_NAND_InitTypeDef
#define FMC_PCCARD_InitTypeDef                SMC_PCCARD_InitTypeDef
#define FMC_NAND_PCC_TimingTypeDef            SMC_NAND_PCC_TimingTypeDef

#define FMC_NAND_Init                         SMC_NAND_Init
#define FMC_NAND_CommonSpace_Timing_Init      SMC_NAND_CommonSpace_Timing_Init
#define FMC_NAND_AttributeSpace_Timing_Init   SMC_NAND_AttributeSpace_Timing_Init
#define FMC_NAND_DeInit                       SMC_NAND_DeInit
#define FMC_NAND_ECC_Enable                   SMC_NAND_ECC_Enable
#define FMC_NAND_ECC_Disable                  SMC_NAND_ECC_Disable
#define FMC_NAND_GetECC                       SMC_NAND_GetECC
#define FMC_PCCARD_Init                       SMC_PCCARD_Init
#define FMC_PCCARD_CommonSpace_Timing_Init    SMC_PCCARD_CommonSpace_Timing_Init
#define FMC_PCCARD_AttributeSpace_Timing_Init SMC_PCCARD_AttributeSpace_Timing_Init
#define FMC_PCCARD_IOSpace_Timing_Init        SMC_PCCARD_IOSpace_Timing_Init
#define FMC_PCCARD_DeInit                     SMC_PCCARD_DeInit

#define __FMC_NAND_ENABLE                     __SMC_NAND_ENABLE
#define __FMC_NAND_DISABLE                    __SMC_NAND_DISABLE
#define __FMC_PCCARD_ENABLE                   __SMC_PCCARD_ENABLE
#define __FMC_PCCARD_DISABLE                  __SMC_PCCARD_DISABLE
#define __FMC_NAND_ENABLE_IT                  __SMC_NAND_ENABLE_IT
#define __FMC_NAND_DISABLE_IT                 __SMC_NAND_DISABLE_IT
#define __FMC_NAND_GET_FLAG                   __SMC_NAND_GET_FLAG
#define __FMC_NAND_CLEAR_FLAG                 __SMC_NAND_CLEAR_FLAG
#define __FMC_PCCARD_ENABLE_IT                __SMC_PCCARD_ENABLE_IT
#define __FMC_PCCARD_DISABLE_IT               __SMC_PCCARD_DISABLE_IT
#define __FMC_PCCARD_GET_FLAG                 __SMC_PCCARD_GET_FLAG
#define __FMC_PCCARD_CLEAR_FLAG               __SMC_PCCARD_CLEAR_FLAG
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#define FMC_NORSRAM_TypeDef                   SMC_NORSRAM_TypeDef
#define FMC_NORSRAM_EXTENDED_TypeDef          SMC_NORSRAM_EXTENDED_TypeDef
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx)
#define FMC_NAND_TypeDef                      SMC_NAND_TypeDef
#define FMC_PCCARD_TypeDef                    SMC_PCCARD_TypeDef
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#define FMC_NORSRAM_DEVICE                    SMC_NORSRAM_DEVICE
#define FMC_NORSRAM_EXTENDED_DEVICE           SMC_NORSRAM_EXTENDED_DEVICE
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx)
#define FMC_NAND_DEVICE                       SMC_NAND_DEVICE
#define FMC_PCCARD_DEVICE                     SMC_PCCARD_DEVICE

#define FMC_NAND_BANK2                        SMC_NAND_BANK2
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#define FMC_NORSRAM_BANK1                     SMC_NORSRAM_BANK1
#define FMC_NORSRAM_BANK2                     SMC_NORSRAM_BANK2
#define FMC_NORSRAM_BANK3                     SMC_NORSRAM_BANK3

#define FMC_IT_RISING_EDGE                    SMC_IT_RISING_EDGE
#define FMC_IT_LEVEL                          SMC_IT_LEVEL
#define FMC_IT_FALLING_EDGE                   SMC_IT_FALLING_EDGE
#define FMC_IT_REFRESH_ERROR                  SMC_IT_REFRESH_ERROR

#define FMC_FLAG_RISING_EDGE                  SMC_FLAG_RISING_EDGE
#define FMC_FLAG_LEVEL                        SMC_FLAG_LEVEL
#define FMC_FLAG_FALLING_EDGE                 SMC_FLAG_FALLING_EDGE
#define FMC_FLAG_FEMPT                        SMC_FLAG_FEMPT
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
/** @defgroup SMC_DDL_Private_Macros SMC_DDL  Private Macros
  * @{
  */
#if defined(SMC_Bank1)
/** @defgroup SMC_DDL_NOR_Macros SMC NOR/SRAM Macros
  * @brief macros to handle NOR device enable/disable and read/write operations
  * @{
  */

/**
  * @brief  Enable the NORSRAM device access.
  * @param  __INSTANCE__ SMC_NORSRAM Instance
  * @param  __BANK__ SMC_NORSRAM Bank
  * @retval None
  */
#define __SMC_NORSRAM_ENABLE(__INSTANCE__, __BANK__)  ((__INSTANCE__)->CSTR[(__BANK__)]\
                                                       |= SMC_CSCTRL1_MBKEN)

/**
  * @brief  Disable the NORSRAM device access.
  * @param  __INSTANCE__ SMC_NORSRAM Instance
  * @param  __BANK__ SMC_NORSRAM Bank
  * @retval None
  */
#define __SMC_NORSRAM_DISABLE(__INSTANCE__, __BANK__) ((__INSTANCE__)->CSTR[(__BANK__)]\
                                                       &= ~SMC_CSCTRL1_MBKEN)

/**
  * @}
  */
#endif /* SMC_Bank1 */

#if defined(SMC_Bank2_3)
/** @defgroup SMC_DDL_NAND_Macros SMC NAND Macros
  *  @brief macros to handle NAND device enable/disable
  *  @{
  */

/**
  * @brief  Enable the NAND device access.
  * @param  __INSTANCE__ SMC_NAND Instance
  * @param  __BANK__     SMC_NAND Bank
  * @retval None
  */
#define __SMC_NAND_ENABLE(__INSTANCE__, __BANK__)  (((__BANK__) == SMC_NAND_BANK2)? ((__INSTANCE__)->CTRL2 |= SMC_CTRL2_MBKEN): \
                                                             ((__INSTANCE__)->CTRL3 |= SMC_CTRL3_MBKEN))

/**
  * @brief  Disable the NAND device access.
  * @param  __INSTANCE__ SMC_NAND Instance
  * @param  __BANK__     SMC_NAND Bank
  * @retval None
  */
#define __SMC_NAND_DISABLE(__INSTANCE__, __BANK__) (((__BANK__) == SMC_NAND_BANK2)? CLEAR_BIT((__INSTANCE__)->CTRL2, SMC_CTRL2_MBKEN): \
                                                             CLEAR_BIT((__INSTANCE__)->CTRL3, SMC_CTRL3_MBKEN))

/**
  * @}
  */
#endif /* SMC_Bank2_3 */

#if defined(SMC_Bank4)
/** @defgroup SMC_DDL_PCCARD_Macros FMC PCCARD Macros
  *  @brief macros to handle PCCARD read/write operations
  *  @{
  */
/**
  * @brief  Enable the PCCARD device access.
  * @param  __INSTANCE__ SMC_PCCARD Instance
  * @retval None
  */
#define __SMC_PCCARD_ENABLE(__INSTANCE__)  ((__INSTANCE__)->CTRL4 |= SMC_CTRL4_MBKEN)

/**
  * @brief  Disable the PCCARD device access.
  * @param  __INSTANCE__ SMC_PCCARD Instance
  * @retval None
  */
#define __SMC_PCCARD_DISABLE(__INSTANCE__) ((__INSTANCE__)->CTRL4 &= ~SMC_CTRL4_MBKEN)
/**
  * @}
  */

#endif
#if defined(SMC_Bank2_3)
/** @defgroup SMC_DDL_NAND_Interrupt SMC NAND Interrupt
  * @brief macros to handle NAND interrupts
  * @{
  */

/**
  * @brief  Enable the NAND device interrupt.
  * @param  __INSTANCE__  SMC_NAND instance
  * @param  __BANK__     SMC_NAND Bank
  * @param  __INTERRUPT__ SMC_NAND interrupt
  *         This parameter can be any combination of the following values:
  *            @arg SMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg SMC_IT_LEVEL: Interrupt level.
  *            @arg SMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  */
#define __SMC_NAND_ENABLE_IT(__INSTANCE__, __BANK__, __INTERRUPT__)  (((__BANK__) == SMC_NAND_BANK2)? ((__INSTANCE__)->STSINT2 |= (__INTERRUPT__)): \
                                                                               ((__INSTANCE__)->STSINT3 |= (__INTERRUPT__)))

/**
  * @brief  Disable the NAND device interrupt.
  * @param  __INSTANCE__  SMC_NAND Instance
  * @param  __BANK__     SMC_NAND Bank
  * @param  __INTERRUPT__ SMC_NAND interrupt
  *         This parameter can be any combination of the following values:
  *            @arg SMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg SMC_IT_LEVEL: Interrupt level.
  *            @arg SMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  */
#define __SMC_NAND_DISABLE_IT(__INSTANCE__, __BANK__, __INTERRUPT__)  (((__BANK__) == SMC_NAND_BANK2)? ((__INSTANCE__)->STSINT2 &= ~(__INTERRUPT__)): \
                                                                                ((__INSTANCE__)->STSINT3 &= ~(__INTERRUPT__)))

/**
  * @brief  Get flag status of the NAND device.
  * @param  __INSTANCE__ SMC_NAND Instance
  * @param  __BANK__     SMC_NAND Bank
  * @param  __FLAG__     SMC_NAND flag
  *         This parameter can be any combination of the following values:
  *            @arg SMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg SMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg SMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg SMC_FLAG_FEMPT: FIFO empty flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define __SMC_NAND_GET_FLAG(__INSTANCE__, __BANK__, __FLAG__)  (((__BANK__) == SMC_NAND_BANK2)? (((__INSTANCE__)->STSINT2 &(__FLAG__)) == (__FLAG__)): \
                                                                         (((__INSTANCE__)->STSINT3 &(__FLAG__)) == (__FLAG__)))

/**
  * @brief  Clear flag status of the NAND device.
  * @param  __INSTANCE__ SMC_NAND Instance
  * @param  __BANK__     SMC_NAND Bank
  * @param  __FLAG__     SMC_NAND flag
  *         This parameter can be any combination of the following values:
  *            @arg SMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg SMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg SMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg SMC_FLAG_FEMPT: FIFO empty flag.
  * @retval None
  */
#define __SMC_NAND_CLEAR_FLAG(__INSTANCE__, __BANK__, __FLAG__)  (((__BANK__) == SMC_NAND_BANK2)? ((__INSTANCE__)->STSINT2 &= ~(__FLAG__)): \
                                                                           ((__INSTANCE__)->STSINT3 &= ~(__FLAG__)))

/**
  * @}
  */
#endif /* SMC_Bank2_3 */

#if defined(SMC_Bank4)
/** @defgroup SMC_DDL_PCCARD_Interrupt SMC PCCARD Interrupt
  * @brief macros to handle PCCARD interrupts
  * @{
  */

/**
  * @brief  Enable the PCCARD device interrupt.
  * @param  __INSTANCE__ SMC_PCCARD instance
  * @param  __INTERRUPT__ SMC_PCCARD interrupt
  *         This parameter can be any combination of the following values:
  *            @arg SMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg SMC_IT_LEVEL: Interrupt level.
  *            @arg SMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  */
#define __SMC_PCCARD_ENABLE_IT(__INSTANCE__, __INTERRUPT__)  ((__INSTANCE__)->STSINT4 |= (__INTERRUPT__))

/**
  * @brief  Disable the PCCARD device interrupt.
  * @param  __INSTANCE__ SMC_PCCARD instance
  * @param  __INTERRUPT__ SMC_PCCARD interrupt
  *         This parameter can be any combination of the following values:
  *            @arg SMC_IT_RISING_EDGE: Interrupt rising edge.
  *            @arg SMC_IT_LEVEL: Interrupt level.
  *            @arg SMC_IT_FALLING_EDGE: Interrupt falling edge.
  * @retval None
  */
#define __SMC_PCCARD_DISABLE_IT(__INSTANCE__, __INTERRUPT__)  ((__INSTANCE__)->STSINT4 &= ~(__INTERRUPT__))

/**
  * @brief  Get flag status of the PCCARD device.
  * @param  __INSTANCE__ SMC_PCCARD instance
  * @param  __FLAG__ SMC_PCCARD flag
  *         This parameter can be any combination of the following values:
  *            @arg  SMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg  SMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg  SMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg  SMC_FLAG_FEMPT: FIFO empty flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define __SMC_PCCARD_GET_FLAG(__INSTANCE__, __FLAG__)  (((__INSTANCE__)->STSINT4 &(__FLAG__)) == (__FLAG__))

/**
  * @brief  Clear flag status of the PCCARD device.
  * @param  __INSTANCE__ SMC_PCCARD instance
  * @param  __FLAG__ SMC_PCCARD flag
  *         This parameter can be any combination of the following values:
  *            @arg  SMC_FLAG_RISING_EDGE: Interrupt rising edge flag.
  *            @arg  SMC_FLAG_LEVEL: Interrupt level edge flag.
  *            @arg  SMC_FLAG_FALLING_EDGE: Interrupt falling edge flag.
  *            @arg  SMC_FLAG_FEMPT: FIFO empty flag.
  * @retval None
  */
#define __SMC_PCCARD_CLEAR_FLAG(__INSTANCE__, __FLAG__)  ((__INSTANCE__)->STSINT4 &= ~(__FLAG__))

/**
  * @}
  */
#endif

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup SMC_DDL_Private_Functions SMC DDL Private Functions
  *  @{
  */

#if defined(SMC_Bank1)
/** @defgroup SMC_DDL_NORSRAM  NOR SRAM
  *  @{
  */
/** @defgroup SMC_DDL_NORSRAM_Private_Functions_Group1 NOR SRAM Initialization/de-initialization functions
  *  @{
  */
DAL_StatusTypeDef  SMC_NORSRAM_Init(SMC_NORSRAM_TypeDef *Device,
                                    SMC_NORSRAM_InitTypeDef *Init);
DAL_StatusTypeDef  SMC_NORSRAM_Timing_Init(SMC_NORSRAM_TypeDef *Device,
                                           SMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank);
DAL_StatusTypeDef  SMC_NORSRAM_Extended_Timing_Init(SMC_NORSRAM_EXTENDED_TypeDef *Device,
                                                    SMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank,
                                                    uint32_t ExtendedMode);
DAL_StatusTypeDef  SMC_NORSRAM_DeInit(SMC_NORSRAM_TypeDef *Device,
                                      SMC_NORSRAM_EXTENDED_TypeDef *ExDevice, uint32_t Bank);
/**
  * @}
  */

/** @defgroup SMC_DDL_NORSRAM_Private_Functions_Group2 NOR SRAM Control functions
  *  @{
  */
DAL_StatusTypeDef  SMC_NORSRAM_WriteOperation_Enable(SMC_NORSRAM_TypeDef *Device, uint32_t Bank);
DAL_StatusTypeDef  SMC_NORSRAM_WriteOperation_Disable(SMC_NORSRAM_TypeDef *Device, uint32_t Bank);
/**
  * @}
  */
/**
  * @}
  */
#endif /* SMC_Bank1 */

#if defined(SMC_Bank2_3)
/** @defgroup SMC_DDL_NAND NAND
  *  @{
  */
/** @defgroup SMC_DDL_NAND_Private_Functions_Group1 NAND Initialization/de-initialization functions
  *  @{
  */
DAL_StatusTypeDef  SMC_NAND_Init(SMC_NAND_TypeDef *Device, SMC_NAND_InitTypeDef *Init);
DAL_StatusTypeDef  SMC_NAND_CommonSpace_Timing_Init(SMC_NAND_TypeDef *Device,
                                                    SMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
DAL_StatusTypeDef  SMC_NAND_AttributeSpace_Timing_Init(SMC_NAND_TypeDef *Device,
                                                       SMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
DAL_StatusTypeDef  SMC_NAND_DeInit(SMC_NAND_TypeDef *Device, uint32_t Bank);
/**
  * @}
  */

/** @defgroup SMC_DDL_NAND_Private_Functions_Group2 NAND Control functions
  *  @{
  */
DAL_StatusTypeDef  SMC_NAND_ECC_Enable(SMC_NAND_TypeDef *Device, uint32_t Bank);
DAL_StatusTypeDef  SMC_NAND_ECC_Disable(SMC_NAND_TypeDef *Device, uint32_t Bank);
DAL_StatusTypeDef  SMC_NAND_GetECC(SMC_NAND_TypeDef *Device, uint32_t *ECCval, uint32_t Bank,
                                   uint32_t Timeout);
/**
  * @}
  */
/**
  * @}
  */
#endif /* SMC_Bank2_3 */

#if defined(SMC_Bank4)
/** @defgroup SMC_DDL_PCCARD PCCARD
  *  @{
  */
/** @defgroup SMC_DDL_PCCARD_Private_Functions_Group1 PCCARD Initialization/de-initialization functions
  *  @{
  */
DAL_StatusTypeDef  SMC_PCCARD_Init(SMC_PCCARD_TypeDef *Device, SMC_PCCARD_InitTypeDef *Init);
DAL_StatusTypeDef  SMC_PCCARD_CommonSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                               SMC_NAND_PCC_TimingTypeDef *Timing);
DAL_StatusTypeDef  SMC_PCCARD_AttributeSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                                  SMC_NAND_PCC_TimingTypeDef *Timing);
DAL_StatusTypeDef  SMC_PCCARD_IOSpace_Timing_Init(SMC_PCCARD_TypeDef *Device,
                                                           SMC_NAND_PCC_TimingTypeDef *Timing);
DAL_StatusTypeDef  SMC_PCCARD_DeInit(SMC_PCCARD_TypeDef *Device);
/**
  * @}
  */
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_SMC_H */
