/**
  **********************************************************************************************************************
  * @file    stm32h5xx_hal_def.h
  * @author  MCD Application Team
  * @brief   This file contains HAL common defines, enumeration, macros and
  *          structures definitions.
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef __STM32H5xx_HAL_DEF
#define __STM32H5xx_HAL_DEF

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
#include <arm_cmse.h>
#endif /* __ARM_FEATURE_CMSE */

#include "stm32h5xx.h"
#include "Legacy/stm32_hal_legacy.h"  /* Aliases file for old names compatibility */
#include <stddef.h>
#include <math.h>

/* Exported types ----------------------------------------------------------------------------------------------------*/

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

/* Exported macros ---------------------------------------------------------------------------------------------------*/

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define ARMCC_MIN_VERSION  6010050

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)             \
  do{                                                    \
    (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
    (__DMA_HANDLE__).Parent = (__HANDLE__);              \
  } while(0)

#if !defined(UNUSED)
#define UNUSED(x) ((void)(x))
#endif /* UNUSED */

/** @brief Reset the Handle's State field.
  * @param __HANDLE__: specifies the Peripheral Handle.
  * @note  This macro can be used for the following purpose:
  *          - When the Handle is declared as local variable; before passing it as parameter
  *            to HAL_PPP_Init() for the first time, it is mandatory to use this macro
  *            to set to 0 the Handle's "State" field.
  *            Otherwise, "State" field may have any random value and the first time the function
  *            HAL_PPP_Init() is called, the low level hardware initialization will be missed
  *            (i.e. HAL_PPP_MspInit() will not be executed).
  *          - When there is a need to reconfigure the low level hardware: instead of calling
  *            HAL_PPP_DeInit() then HAL_PPP_Init(), user can make a call to this macro then HAL_PPP_Init().
  *            In this later function, when the Handle's "State" field is set to 0, it will execute the function
  *            HAL_PPP_MspInit() which will reconfigure the low level hardware.
  * @retval None
  */
#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0)

#if (USE_RTOS == 1)
/* Reserved for future use */
#error " USE_RTOS should be 0 in the current HAL release "
#else
#define __HAL_LOCK(__HANDLE__)             \
  do{                                      \
    if((__HANDLE__)->Lock == HAL_LOCKED)   \
    {                                      \
      return HAL_BUSY;                     \
    }                                      \
    else                                   \
    {                                      \
      (__HANDLE__)->Lock = HAL_LOCKED;     \
    }                                      \
  }while (0)

#define __HAL_UNLOCK(__HANDLE__)           \
  do{                                      \
    (__HANDLE__)->Lock = HAL_UNLOCKED;     \
  }while (0)
#endif /* USE_RTOS */

#if  defined ( __GNUC__ )
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)
#ifndef __weak
#define __weak  __WEAK
#endif /* __weak */
#ifndef __packed
#define __packed  __PACKED
#endif /* __packed */
#endif

/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4"
   must be used instead */
#if defined   (__GNUC__)        /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)
#ifndef __ALIGN_END
#define __ALIGN_END    __ALIGNED(4)
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/* Macro to get variable aligned on 32-bytes,needed for cache maintenance purpose */
#if defined   (__GNUC__)        /* GNU Compiler */
#define ALIGN_32BYTES(buf)  buf __attribute__ ((aligned (32)))
#elif defined (__ICCARM__)    /* IAR Compiler */
#define ALIGN_32BYTES(buf) _Pragma("data_alignment=32") buf
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)
#define ALIGN_32BYTES(buf) __ALIGNED(32) buf
#elif defined   (__CC_ARM)      /* ARM Compiler */
#define ALIGN_32BYTES(buf) __align(32) buf
#endif /* __GNUC__ */

/**
  * @brief  __RAM_FUNC definition
  */
#if defined ( __CC_ARM   ) || ((__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION))

/* ARM Compiler

   RAM functions are defined using the toolchain options.
   Functions that are executed in RAM should reside in a separate source module.
   Using the 'Options for File' dialog you can simply change the 'Code / Const'
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog.
*/
#define __RAM_FUNC HAL_StatusTypeDef

#elif defined ( __ICCARM__ )
/* ICCARM Compiler

   RAM functions are defined using a specific toolchain keyword "__ramfunc".
*/
#define __RAM_FUNC __ramfunc HAL_StatusTypeDef

#elif defined   (  __GNUC__  )
/* GNU Compiler

  RAM functions are defined using a specific toolchain attribute
   "__attribute__((section(".RamFunc")))".
*/
#define __RAM_FUNC HAL_StatusTypeDef  __attribute__((section(".RamFunc")))

#endif /* defined ( __CC_ARM   ) || ((__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)) */

/**
  * @brief  __NOINLINE definition
  */
#if defined ( __CC_ARM   ) || ((__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)) || defined   (  __GNUC__  )
/* ARM & GNUCompiler

*/
#define __NOINLINE __attribute__ ( (noinline) )

#elif defined ( __ICCARM__ )
/* ICCARM Compiler

*/
#define __NOINLINE _Pragma("optimize = no_inline")

#endif /* ( __CC_ARM   ) || ((__ARMCC_VERSION) && (__ARMCC_VERSION >= ARMCC_MIN_VERSION)) || defined   (  __GNUC__  ) */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ___STM32H5xx_HAL_DEF */

