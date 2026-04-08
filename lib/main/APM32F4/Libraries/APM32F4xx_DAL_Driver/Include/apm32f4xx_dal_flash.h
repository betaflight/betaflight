/**
  *
  * @file    apm32f4xx_dal_flash.h
  * @brief   Header file of FLASH DAL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_FLASH_H
#define APM32F4xx_DAL_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Types FLASH Exported Types
  * @{
  */

/**
  * @brief  FLASH Procedure structure definition
  */
typedef enum
{
  FLASH_PROC_NONE = 0U,
#if defined (APM32F403xx) || defined (APM32F402xx)
  FLASH_PROC_PAGEERASE,
#else
  FLASH_PROC_SECTERASE,
#endif /* APM32F403xx || APM32F402xx */
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM,
  FLASH_PROC_PROGRAMHALFWORD,
  FLASH_PROC_PROGRAMWORD,
  FLASH_PROC_PROGRAMDOUBLEWORD
} FLASH_ProcedureTypeDef;

/**
  * @brief  FLASH handle Structure definition
  */
typedef struct
{
  __IO FLASH_ProcedureTypeDef ProcedureOnGoing;   /* Internal variable to indicate which procedure is ongoing or not in IT context*/

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  __IO uint32_t               NbSectorsToErase;   /* Internal variable to save the remaining sectors to erase in IT context*/

  __IO uint8_t                VoltageForErase;    /* Internal variable to provide voltage range selected by user in IT context*/

  __IO uint32_t               Sector;             /* Internal variable to define the current sector which is erasing*/

  __IO uint32_t               Bank;               /* Internal variable to save current bank selected during mass erase*/

  __IO uint32_t               Address;            /* Internal variable to save address selected for program*/
#else
  __IO uint32_t               DataRemaining;      /* Internal variable to save the remaining pages to erase or half-word to program in IT context */

  __IO uint32_t               Address;            /* Internal variable to save address selected for program or erase */

  __IO uint64_t               Data;               /* Internal variable to save data to be programmed */
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  DAL_LockTypeDef             Lock;               /* FLASH locking object                */

  __IO uint32_t               ErrorCode;          /* FLASH error code                    */

}FLASH_ProcessTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
  */
/** @defgroup FLASH_Error_Code FLASH Error Code
  * @brief    FLASH Error Code
  * @{
  */
#define DAL_FLASH_ERROR_NONE         0x00000000U    /*!< No error                      */
#define DAL_FLASH_ERROR_RD           0x00000001U    /*!< Read Protection error         */
#define DAL_FLASH_ERROR_PGS          0x00000002U    /*!< Programming Sequence error    */
#define DAL_FLASH_ERROR_PGP          0x00000004U    /*!< Programming Parallelism error */
#define DAL_FLASH_ERROR_PGA          0x00000008U    /*!< Programming Alignment error   */
#define DAL_FLASH_ERROR_WRP          0x00000010U    /*!< Write protection error        */
#define DAL_FLASH_ERROR_OPERATION    0x00000020U    /*!< Operation Error               */
#define DAL_FLASH_ERROR_PROG         0x00000040U    /*!< Programming error             */
#define DAL_FLASH_ERROR_OPTV         0x00000080U    /*!< Option validity error         */
/**
  * @}
  */

/** @defgroup FLASH_Type_Program FLASH Type Program
  * @{
  */
#define FLASH_TYPEPROGRAM_BYTE        0x00000000U  /*!< Program byte (8-bit) at a specified address           */
#define FLASH_TYPEPROGRAM_HALFWORD    0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_TYPEPROGRAM_WORD        0x00000002U  /*!< Program a word (32-bit) at a specified address        */
#define FLASH_TYPEPROGRAM_DOUBLEWORD  0x00000003U  /*!< Program a double word (64-bit) at a specified address */
/**
  * @}
  */

/** @defgroup FLASH_Flag_definition FLASH Flag definition
  * @brief Flag definition
  * @{
  */
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
#define FLASH_FLAG_EOP                 FLASH_STS_OPRCMP             /*!< FLASH End of Operation flag               */
#define FLASH_FLAG_OPERR               FLASH_STS_OPRERR             /*!< FLASH Operation Error flag                */
#define FLASH_FLAG_WRPERR              FLASH_STS_WPROTERR           /*!< FLASH Write protected error flag          */
#define FLASH_FLAG_PGAERR              FLASH_STS_PGALGERR           /*!< FLASH Programming Alignment error flag    */
#define FLASH_FLAG_PGPERR              FLASH_STS_PGPRLERR           /*!< FLASH Programming Parallelism error flag  */
#define FLASH_FLAG_PGSERR              FLASH_STS_PGSEQERR           /*!< FLASH Programming Sequence error flag     */
#if defined(FLASH_STS_RPROERR)
#define FLASH_FLAG_RDERR               FLASH_STS_RPROERR            /*!< Read Protection error flag (PCROP)        */
#endif /* FLASH_STS_RPROERR */
#define FLASH_FLAG_BSY                 FLASH_STS_BUSY               /*!< FLASH Busy flag                           */
#else
#define FLASH_FLAG_EOP                 FLASH_STS_OCF                /*!< FLASH End of Operation flag               */
#define FLASH_FLAG_PGERR               FLASH_STS_PEF                /*!< FLASH Programming Error flag                */
#define FLASH_FLAG_WRPERR              FLASH_STS_WPEF               /*!< FLASH Write protected error flag          */
#define FLASH_FLAG_BSY                 FLASH_STS_BUSYF              /*!< FLASH Busy flag                           */
#define FLASH_FLAG_OPTVERR             ((OBR_REG_INDEX << 8U | FLASH_OBCS_OBE)) /*!< Option Byte Error        */
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
/**
  * @}
  */

/** @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
  * @brief FLASH Interrupt definition
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_IT_EOP                   FLASH_CTRL2_OCIE         /*!< End of FLASH Operation Interrupt source */
#define FLASH_IT_ERR                   FLASH_CTRL2_ERRIE        /*!< Error Interrupt source                  */
#else
#define FLASH_IT_EOP                   FLASH_CTRL_OPCINTEN      /*!< End of FLASH Operation Interrupt source */
#define FLASH_IT_ERR                   0x02000000U              /*!< Error Interrupt source                  */
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @defgroup FLASH_Program_Parallelism FLASH Program Parallelism
  * @{
  */
#define FLASH_PSIZE_BYTE           0x00000000U
#define FLASH_PSIZE_HALF_WORD      0x00000100U
#define FLASH_PSIZE_WORD           0x00000200U
#define FLASH_PSIZE_DOUBLE_WORD    0x00000300U
#define CR_PSIZE_MASK              0xFFFFFCFFU
/**
  * @}
  */

/** @defgroup FLASH_Keys FLASH Keys
  * @{
  */
#define RDP_KEY                  ((uint16_t)0x00A5)
#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_OPT_KEY1           FLASH_KEY1
#define FLASH_OPT_KEY2           FLASH_KEY2
#else
#define FLASH_OPT_KEY1           0x08192A3BU
#define FLASH_OPT_KEY2           0x4C5D6E7FU
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */
/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__ FLASH Latency
  *         The value of this parameter depend on device used within the same series
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_SET_LATENCY(__LATENCY__) (FLASH->CTRL1 = (FLASH->CTRL1&(~(FLASH_CTRL1_WS02|FLASH_CTRL1_WS34))) | __LATENCY__)
#else
#define __DAL_FLASH_SET_LATENCY(__LATENCY__) (*(__IO uint8_t *)ACCTRL_BYTE0_ADDRESS = (uint8_t)(__LATENCY__))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency
  *          The value of this parameter depend on device used within the same series
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_GET_LATENCY()     (READ_BIT((FLASH->CTRL1), (FLASH_CTRL1_WS02 | FLASH_CTRL1_WS34)))
#else
#define __DAL_FLASH_GET_LATENCY()     (READ_BIT((FLASH->ACCTRL), FLASH_ACCTRL_WAITP))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Enable the FLASH prefetch buffer.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_PREFETCH_BUFFER_ENABLE()  (FLASH->CTRL1 |= FLASH_CTRL1_PBEN)
#else
#define __DAL_FLASH_PREFETCH_BUFFER_ENABLE()  (FLASH->ACCTRL |= FLASH_ACCTRL_PREFEN)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Disable the FLASH prefetch buffer.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_PREFETCH_BUFFER_DISABLE()   (FLASH->CTRL1 &= (~FLASH_CTRL1_PBEN))
#else
#define __DAL_FLASH_PREFETCH_BUFFER_DISABLE()   (FLASH->ACCTRL &= (~FLASH_ACCTRL_PREFEN))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_INSTRUCTION_CACHE_ENABLE()  (FLASH->CTRL1 |= FLASH_CTRL1_ICACHEEN)
#else
#define __DAL_FLASH_INSTRUCTION_CACHE_ENABLE()  (FLASH->ACCTRL |= FLASH_ACCTRL_ICACHEEN)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Disable the FLASH instruction cache.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_INSTRUCTION_CACHE_DISABLE()   (FLASH->CTRL1 &= (~FLASH_CTRL1_ICACHEEN))
#else
#define __DAL_FLASH_INSTRUCTION_CACHE_DISABLE()   (FLASH->ACCTRL &= (~FLASH_ACCTRL_ICACHEEN))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Enable the FLASH data cache.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_DATA_CACHE_ENABLE()  (FLASH->CTRL1 |= FLASH_CTRL1_DCACHEEN)
#else
#define __DAL_FLASH_DATA_CACHE_ENABLE()  (FLASH->ACCTRL |= FLASH_ACCTRL_DCACHEEN)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Disable the FLASH data cache.
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_DATA_CACHE_DISABLE()   (FLASH->CTRL1 &= (~FLASH_CTRL1_DCACHEEN))
#else
#define __DAL_FLASH_DATA_CACHE_DISABLE()   (FLASH->ACCTRL &= (~FLASH_ACCTRL_DCACHEEN))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Resets the FLASH instruction Cache.
  * @note   This function must be used only when the Instruction Cache is disabled.
  * @retval None
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_INSTRUCTION_CACHE_RESET() do {FLASH->CTRL1 |= FLASH_CTRL1_ICACHERST;  \
                                                  FLASH->CTRL1 &= ~FLASH_CTRL1_ICACHERST; \
                                                 }while(0U)
#else
#define __DAL_FLASH_INSTRUCTION_CACHE_RESET() do {FLASH->ACCTRL |= FLASH_ACCTRL_ICACHERST;  \
                                                  FLASH->ACCTRL &= ~FLASH_ACCTRL_ICACHERST; \
                                                 }while(0U)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Resets the FLASH data Cache.
  * @note   This function must be used only when the data Cache is disabled.
  * @retval None
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_DATA_CACHE_RESET() do {FLASH->CTRL1 |= FLASH_CTRL1_DCACHERST;  \
                                           FLASH->CTRL1 &= ~FLASH_CTRL1_DCACHERST; \
                                          }while(0U)
#else
#define __DAL_FLASH_DATA_CACHE_RESET() do {FLASH->ACCTRL |= FLASH_ACCTRL_DCACHERST;  \
                                           FLASH->ACCTRL &= ~FLASH_ACCTRL_DCACHERST; \
                                          }while(0U)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Enable the specified FLASH interrupt.
  * @param  __INTERRUPT__  FLASH interrupt
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_ENABLE_IT(__INTERRUPT__)  (FLASH->CTRL2 |= (__INTERRUPT__))
#else
#define __DAL_FLASH_ENABLE_IT(__INTERRUPT__)  (FLASH->CTRL |= (__INTERRUPT__))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Disable the specified FLASH interrupt.
  * @param  __INTERRUPT__  FLASH interrupt
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_DISABLE_IT(__INTERRUPT__)  (FLASH->CTRL2 &= ~(uint32_t)(__INTERRUPT__))
#else
#define __DAL_FLASH_DISABLE_IT(__INTERRUPT__)  (FLASH->CTRL &= ~(uint32_t)(__INTERRUPT__))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Get the specified FLASH flag status.
  @if APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx
  * @param  __FLAG__ specifies the FLASH flags to check.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_PGSERR: FLASH Programming Sequence error flag
  *            @arg FLASH_FLAG_RDERR : FLASH Read Protection error flag (PCROP) (*)
  *            @arg FLASH_FLAG_BSY   : FLASH Busy flag
  *           (*) FLASH_FLAG_RDERR is not available for APM32F405xx/407xx/415xx/417xx devices
  @endif
  @if APM32F402/403xx
  * @param  __FLAG__ specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg @ref FLASH_FLAG_EOP    FLASH End of Operation flag
  *            @arg @ref FLASH_FLAG_WRPERR FLASH Write protected error flag
  *            @arg @ref FLASH_FLAG_PGERR  FLASH Programming error flag
  *            @arg @ref FLASH_FLAG_BSY    FLASH Busy flag
  *            @arg @ref FLASH_FLAG_OPTVERR  Loaded OB and its complement do not match
  @endif
  * @retval The new state of __FLAG__ (SET or RESET).
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_GET_FLAG(__FLAG__)  (((__FLAG__) == FLASH_FLAG_OPTVERR) ? \
                                            (FLASH->OBCS & FLASH_OBCS_OBE) : \
                                            (FLASH->STS & (__FLAG__)))
#else
#define __DAL_FLASH_GET_FLAG(__FLAG__)   ((FLASH->STS & (__FLAG__)))
#endif /* APM32F403xx || APM32F402xx */

/**
  * @brief  Clear the specified FLASH flags.
  @if APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx
  * @param  __FLAG__ specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_PGSERR: FLASH Programming Sequence error flag
  *            @arg FLASH_FLAG_RDERR : FLASH Read Protection error flag (PCROP) (*)
  *           (*) FLASH_FLAG_RDERR is not available for APM32F405xx/407xx/415xx/417xx devices
  @endif
  @if APM32F402/403xx
  * @param  __FLAG__ specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg @ref FLASH_FLAG_EOP    FLASH End of Operation flag
  *            @arg @ref FLASH_FLAG_WRPERR FLASH Write protected error flag
  *            @arg @ref FLASH_FLAG_PGERR  FLASH Programming error flag
  *            @arg @ref FLASH_FLAG_OPTVERR  Loaded OB and its complement do not match
  @endif
  * @retval none
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define __DAL_FLASH_CLEAR_FLAG(__FLAG__)   do { \
                          /* Clear FLASH_FLAG_OPTVERR flag */ \
                          if ((__FLAG__) == FLASH_FLAG_OPTVERR) \
                          { \
                            CLEAR_BIT(FLASH->OBCS, FLASH_OBCS_OBE); \
                          } \
                          else { \
                            /* Clear Flag in Bank1 */ \
                            FLASH->STS  = (__FLAG__); \
                          } \
                    } while(0U)
#else
#define __DAL_FLASH_CLEAR_FLAG(__FLAG__)   (FLASH->STS = (__FLAG__))
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/* Include FLASH DAL Extension module */
#include "apm32f4xx_dal_flash_ex.h"
#include "apm32f4xx_dal_flash_ramfunc.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_Exported_Functions
  * @{
  */
/** @addtogroup FLASH_Exported_Functions_Group1
  * @{
  */
/* Program operation functions  ***********************************************/
DAL_StatusTypeDef DAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
DAL_StatusTypeDef DAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
/* FLASH IRQ handler method */
void DAL_FLASH_IRQHandler(void);
/* Callbacks in non blocking modes */
void DAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void DAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);
/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  **********************************************/
DAL_StatusTypeDef DAL_FLASH_Unlock(void);
DAL_StatusTypeDef DAL_FLASH_Lock(void);
DAL_StatusTypeDef DAL_FLASH_OB_Unlock(void);
DAL_StatusTypeDef DAL_FLASH_OB_Lock(void);
/* Option bytes control */
#if defined(APM32F403xx) || defined(APM32F402xx)
void DAL_FLASH_OB_Launch(void);
#else
DAL_StatusTypeDef DAL_FLASH_OB_Launch(void);
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  ************************************************/
uint32_t DAL_FLASH_GetError(void);
DAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup FLASH_Private_Variables FLASH Private Variables
  * @{
  */

/**
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/** @defgroup FLASH_Private_Constants FLASH Private Constants
  * @{
  */

/**
  * @brief   ACCTRL register byte 0 (Bits[7:0]) base address
  */
#define ACCTRL_BYTE0_ADDRESS          0x40023C00U
/**
  * @brief   OPTCTRL register byte 0 (Bits[7:0]) base address
  */
#define OPTCTRL_BYTE0_ADDRESS         0x40023C14U
/**
  * @brief   OPTCTRL register byte 1 (Bits[15:8]) base address
  */
#define OPTCTRL_BYTE1_ADDRESS         0x40023C15U
/**
  * @brief   OPTCTRL register byte 2 (Bits[23:16]) base address
  */
#define OPTCTRL_BYTE2_ADDRESS         0x40023C16U
/**
  * @brief   OPTCTRL register byte 3 (Bits[31:24]) base address
  */
#define OPTCTRL_BYTE3_ADDRESS         0x40023C17U

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup FLASH_Private_Macros FLASH Private Macros
  * @{
  */

/** @defgroup FLASH_IS_FLASH_Definitions FLASH Private macros to check input parameters
  * @{
  */

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
#define IS_FLASH_TYPEPROGRAM(VALUE)(((VALUE) == FLASH_TYPEPROGRAM_BYTE)     || \
                                    ((VALUE) == FLASH_TYPEPROGRAM_HALFWORD) || \
                                    ((VALUE) == FLASH_TYPEPROGRAM_WORD)     || \
                                    ((VALUE) == FLASH_TYPEPROGRAM_DOUBLEWORD))
#else
#define IS_FLASH_TYPEPROGRAM(VALUE)(((VALUE) == FLASH_TYPEPROGRAM_HALFWORD) || \
                                    ((VALUE) == FLASH_TYPEPROGRAM_WORD)     || \
                                    ((VALUE) == FLASH_TYPEPROGRAM_DOUBLEWORD))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup FLASH_Private_Functions FLASH Private Functions
  * @{
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

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_FLASH_H */

