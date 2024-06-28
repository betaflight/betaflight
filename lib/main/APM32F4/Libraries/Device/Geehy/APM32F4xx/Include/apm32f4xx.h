/**
 *
 * @file        apm32f4xx.h
 *
 * @brief       CMSIS APM32F4xx Device Peripheral Access Layer Header File.
 *
 * @version     V1.1.2
 *
 * @date        2023-12-01
 *
 * @attention
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The original code has been modified by Geehy Semiconductor.
 *
 * Copyright (c) 2017 STMicroelectronics.
 * Copyright (C) 2023 Geehy Semiconductor.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup apm32f4xx
  * @{
  */
    
#ifndef __APM32F4xx_H
#define __APM32F4xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/** @addtogroup Library_configuration_section
  * @{
  */
  
/**
  * @brief APM32 Family
  */
#if !defined  (APM32F4)
#define APM32F4
#endif /* APM32F4 */

/* Uncomment the line below according to the target APM32 device used in your
   application 
  */
#if !defined (APM32F405xx) && !defined (APM32F407xx) && !defined (APM32F417xx) && \
    !defined (APM32F411xx) && !defined (APM32F465xx)
    /* #define APM32F405xx */   /*!< APM32F405RG, APM32F405VG and APM32F405ZG Devices */
    /* #define APM32F407xx */   /*!< APM32F407VG, APM32F407VE, APM32F407ZG, APM32F407ZE, APM32F407IG and APM32F407IE Devices */
    /* #define APM32F417xx */   /*!< APM32F417VG, APM32F417VE, APM32F417ZG, APM32F417ZE, APM32F417IG and APM32F417IE Devices */
    /* #define APM32F411xx */   /*!< APM32F411CC, APM32F411CE, APM32F411RC, APM32F411RE, APM32F411VC and APM32F411VE Devices */
    /* #define APM32F465xx */   /*!< APM32F465CE, APM32F465RE and APM32F465VE Devices */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_DAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will 
   be based on direct access to peripherals registers 
   */
  /*#define USE_DAL_DRIVER */
#endif /* USE_DAL_DRIVER */

/**
  * @brief CMSIS version number V1.1.2
  */
#define __APM32F4xx_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __APM32F4xx_CMSIS_VERSION_SUB1   (0x01U) /*!< [23:16] sub1 version */
#define __APM32F4xx_CMSIS_VERSION_SUB2   (0x02U) /*!< [15:8]  sub2 version */
#define __APM32F4xx_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __APM32F4xx_CMSIS_VERSION        ((__APM32F4xx_CMSIS_VERSION_MAIN << 24)\
                                         |(__APM32F4xx_CMSIS_VERSION_SUB1 << 16)\
                                         |(__APM32F4xx_CMSIS_VERSION_SUB2 << 8 )\
                                         |(__APM32F4xx_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(APM32F405xx)
  #include "apm32f405xx.h"
#elif defined(APM32F407xx)
  #include "apm32f407xx.h"
#elif defined(APM32F417xx)
  #include "apm32f417xx.h"
#elif defined(APM32F411xx)
  #include "apm32f411xx.h"
#elif defined(APM32F465xx)
  #include "apm32f465xx.h"
#else
 #error "Please select first the target APM32F4xx device used in your application (in apm32f4xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */ 
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macro
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 

/* Use of CMSIS compiler intrinsics for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)                          \
  do {                                                                     \
    uint32_t val;                                                          \
    do {                                                                   \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);               \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define ATOMIC_SETH_BIT(REG, BIT)                            \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);       \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear one or several bits */
#define ATOMIC_CLEARH_BIT(REG, BIT)                          \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK)                         \
  do {                                                                     \
    uint16_t val;                                                          \
    do {                                                                   \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);               \
  } while(0)

/**
  * @}
  */

#if defined (USE_DAL_DRIVER)
 #include "apm32f4xx_dal.h"
#endif /* USE_DAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APM32F4xx_H */
/**
  * @}
  */

/**
  * @}
  */
