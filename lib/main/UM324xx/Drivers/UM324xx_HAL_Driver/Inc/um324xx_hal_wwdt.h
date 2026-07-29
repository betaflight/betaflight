/**
  ******************************************************************************
  * @file    um324xx_hal_wwdt.h
  * @author  MCU Team
  * @version V1.00
  * @date    2023-4-21
  * @brief   Header file of WWDT HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 - 2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_WWDT_H__
#define __UM324XX_HAL_WWDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup WWDT
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup WWDT_Exported_Types WWDT Exported Types
  * @{
  */

/**
  * @brief  WWDT Init structure definition
  */
typedef struct
{
  uint32_t Counter;                                                            /*!< Specifies the WWDT free-running upcounter  value.
                                                                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F */
  uint32_t IE ;                                                                /*!< Specifies if WWDT Warning Interrupt is enable or not.
                                                                                      This parameter can be a value of @ref WWDT_IE_Mode */
} WWDT_InitTypeDef;

/**
  * @brief  WWDT handle Structure definition
  */
#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
typedef struct __WWDT_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */
{
  WWDT_TypeDef      *Instance;                                                 /*!< Register base address */
  WWDT_InitTypeDef  Init;                                                      /*!< WWDT required parameters */

#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
  void (* EwiCallback)(struct __WWDT_HandleTypeDef *hwwdt);                     /*!< WWDT Early WakeUp Interrupt callback */
  void (* MspInitCallback)(struct __WWDT_HandleTypeDef *hwwdt);                /*!< WWDT Msp Init callback */
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */
} WWDT_HandleTypeDef;

#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL WWDT common Callback ID enumeration definition
  */
typedef enum
{
  HAL_WWDT_INT_CB_ID           = 0x00U,                                         /*!< WWDT IE callback ID */
  HAL_WWDT_MSPINIT_CB_ID       = 0x01U,                                         /*!< WWDT MspInit callback ID */
} HAL_WWDT_CallbackIDTypeDef;

/**
  * @brief  HAL WWDT Callback pointer definition
  */
typedef void (*pWWDT_CallbackTypeDef)(WWDT_HandleTypeDef *hppp);  /*!< pointer to a WWDT common callback functions */

#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup WWDT_Exported_Constants WWDT Exported Constants
  * @{
  */

/** @defgroup WWDT_Interrupt_definition WWDT Interrupt definition
  * @{
  */
#define WWDT_IT_IE                     WWDT_IE_IE  /*!< Warning interrupt */
/**
  * @}
  */

/** @defgroup WWDT_Flag_definition WWDT Flag definition
  * @brief WWDT Flag definition
  * @{
  */
#define WWDT_FLAG_IF                   WWDT_IF_IF  /*!< Warning interrupt flag */
/**
  * @}
  */

/** @defgroup WWDT_Counter WWDT Counter
  * @{
  */
#define WWDT_COUNTER_1                 0x00000000u                                       /*!< WWDT counter = 1     */
#define WWDT_COUNTER_4                 (WWDT_CFG_CFG_0)                                  /*!< WWDT counter = 4     */
#define WWDT_COUNTER_16                (WWDT_CFG_CFG_1)                                  /*!< WWDT counter = 16    */
#define WWDT_COUNTER_64                (WWDT_CFG_CFG_0 | WWDT_CFG_CFG_1)                 /*!< WWDT counter = 64    */
#define WWDT_COUNTER_128               (WWDT_CFG_CFG_2)                                  /*!< WWDT counter = 128   */
#define WWDT_COUNTER_256               (WWDT_CFG_CFG_2 | WWDT_CFG_CFG_0)                 /*!< WWDT counter = 256   */
#define WWDT_COUNTER_512               (WWDT_CFG_CFG_2 | WWDT_CFG_CFG_1)                 /*!< WWDT counter = 512   */
#define WWDT_COUNTER_1024              (WWDT_CFG_CFG_2 | WWDT_CFG_CFG_1 | WWDT_CFG_CFG_0)/*!< WWDT counter = 1024  */
#define WWDT_COUNTER_2048              (WWDT_CFG_CFG_3)                                  /*!< WWDT counter = 2048  */
#define WWDT_COUNTER_4096              (WWDT_CFG_CFG_3 | WWDT_CFG_CFG_0)                 /*!< WWDT counter = 4096  */
#define WWDT_COUNTER_8192              (WWDT_CFG_CFG_3 | WWDT_CFG_CFG_1)                 /*!< WWDT counter = 8192  */
#define WWDT_COUNTER_16384             (WWDT_CFG_CFG_3 | WWDT_CFG_CFG_1 | WWDT_CFG_CFG_0)/*!< WWDT counter = 16384 */
#define WWDT_COUNTER_32768             (WWDT_CFG_CFG_3 | WWDT_CFG_CFG_2 )                /*!< WWDT counter = 32768 */
#define WWDT_COUNTER_65536             (WWDT_CFG_CFG_3 | WWDT_CFG_CFG_2 | WWDT_CFG_CFG_0)/*!< WWDT counter = 65536 */
/**
  * @}
  */

/** @defgroup WWDT_IE_Mode WWDT Interrupt Mode
  * @{
  */
#define WWDT_IE_DISABLE                0x00000000u                         /*!< IE Disable */
#define WWDT_IE_ENABLE                 WWDT_IE_IE                          /*!< IE Enable */
/**
  * @}
  */

/** @defgroup WWDT_CTRL_Mode WWDT CTRL Mode
  * @{
  */
#define WWDT_ENABLE                    0x5Au                                   /*!< WWDT Enable        */
#define WWDT_COUNTER_CLEAR             0xACu                                   /*!< WWDT Counter Clear */
#define WWDT_SOFT_RESET                0x00u                                   /*!< WWDT Soft Reset    */      
/**
  * @}
  */
  
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/** @defgroup WWDT_Private_Macros WWDT Private Macros
  * @{
  */
#define IS_WWDT_COUNTER(__COUNTER__)             (((__COUNTER__) == WWDT_COUNTER_1)      || \
                                                  ((__COUNTER__) == WWDT_COUNTER_4)      || \
                                                  ((__COUNTER__) == WWDT_COUNTER_16)     || \
                                                  ((__COUNTER__) == WWDT_COUNTER_64)     || \
                                                  ((__COUNTER__) == WWDT_COUNTER_128)    || \
                                                  ((__COUNTER__) == WWDT_COUNTER_256)    || \
                                                  ((__COUNTER__) == WWDT_COUNTER_512)    || \
                                                  ((__COUNTER__) == WWDT_COUNTER_1024)   || \
                                                  ((__COUNTER__) == WWDT_COUNTER_2048)   || \
                                                  ((__COUNTER__) == WWDT_COUNTER_4096)   || \
                                                  ((__COUNTER__) == WWDT_COUNTER_8192)   || \
                                                  ((__COUNTER__) == WWDT_COUNTER_16384)  || \
                                                  ((__COUNTER__) == WWDT_COUNTER_32768)  || \
                                                  ((__COUNTER__) == WWDT_COUNTER_65536)) 

#define IS_WWDT_IE_MODE(__MODE__)                (((__MODE__) == WWDT_IE_ENABLE)         || \
                                                  ((__MODE__) == WWDT_IE_DISABLE))

#define IS_WWDT_CTRL_MODE(__MODE__)              (((__MODE__) == WWDT_ENABLE)            || \
                                                  ((__MODE__) == WWDT_COUNTER_CLEAR))    || \
                                                  ((__MODE__) == WWDT_SOFT_RESET))
/**
  * @}
  */


/* Exported macros ------------------------------------------------------------*/

/** @defgroup WWDT_Exported_Macros WWDT Exported Macros
  * @{
  */

/**
  * @brief  Enable the WWDT peripheral.
  * @param  __HANDLE__  WWDT handle
  * @retval None
  */
#define __HAL_WWDT_ENABLE(__HANDLE__)                         WRITE_REG((__HANDLE__)->Instance->CTRL, WWDT_ENABLE)

/**
  * @brief  Enable the WWDT interrupt.
  * @param  __HANDLE__     WWDT handle
  * @param  __INTERRUPT__  specifies the interrupt to enable.
  *         This parameter can be one of the following values:
  *            @arg WWDT_IT_IE: Interrupt
  * @note   Once enabled this interrupt cannot be disabled except by a system reset.
  * @retval None
  */
#define __HAL_WWDT_ENABLE_IT(__HANDLE__, __INTERRUPT__)       SET_BIT((__HANDLE__)->Instance->IE, __INTERRUPT__)

/**
  * @brief  Check whether the selected WWDT interrupt has occurred or not.
  * @param  __HANDLE__  WWDT handle
  * @param  __INTERRUPT__  specifies the it to check.
  *        This parameter can be one of the following values:
  *            @arg WWDT_FLAG_IF: Interrupt IT
  * @retval The new state of WWDT_FLAG (SET or RESET).
  */
#define __HAL_WWDT_GET_IT(__HANDLE__, __INTERRUPT__)        __HAL_WWDT_GET_FLAG((__HANDLE__), (__INTERRUPT__))

/** @brief  Clear the WWDT interrupt pending bits.
  *         bits to clear the selected interrupt pending bits.
  * @param  __HANDLE__  WWDT handle
  * @param  __INTERRUPT__  specifies the interrupt pending bit to clear.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_IF: Interrupt flag
  */
#define __HAL_WWDT_CLEAR_IT(__HANDLE__, __INTERRUPT__)      __HAL_WWDT_CLEAR_FLAG((__HANDLE__), (__INTERRUPT__))

/**
  * @brief  Check whether the specified WWDT flag is set or not.
  * @param  __HANDLE__  WWDT handle
  * @param  __FLAG__  specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_IF: Early Warning interrupt flag
  * @retval The new state of WWDT_FLAG (SET or RESET).
  */
#define __HAL_WWDT_GET_FLAG(__HANDLE__, __FLAG__)           (((__HANDLE__)->Instance->IF & (__FLAG__)) == (__FLAG__))

/**
  * @brief  Clear the WWDT's pending flags.
  * @param  __HANDLE__  WWDT handle
  * @param  __FLAG__  specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_IF: Early Warning interrupt flag
  * @retval None
  */
#define __HAL_WWDT_CLEAR_FLAG(__HANDLE__, __FLAG__)         ((__HANDLE__)->Instance->IF = (__FLAG__))

/** @brief  Check whether the specified WWDT interrupt source is enabled or not.
  * @param  __HANDLE__  WWDT Handle.
  * @param  __INTERRUPT__  specifies the WWDT interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg WWDT_IT_IE: Early Warning Interrupt
  * @retval state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_WWDT_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IE\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup WWDT_Exported_Functions
  * @{
  */

/** @addtogroup WWDT_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef     HAL_WWDT_Init(WWDT_HandleTypeDef *hwwdt);
void                  HAL_WWDT_MspInit(WWDT_HandleTypeDef *hwwdt);
/* Callbacks Register/UnRegister functions  *************************************/
#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef     HAL_WWDT_RegisterCallback(WWDT_HandleTypeDef *hwwdt, HAL_WWDT_CallbackIDTypeDef CallbackID,
                                                pWWDT_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_WWDT_UnRegisterCallback(WWDT_HandleTypeDef *hwwdt, HAL_WWDT_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup WWDT_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
HAL_StatusTypeDef     HAL_WWDT_Refresh(WWDT_HandleTypeDef *hwwdt);
HAL_StatusTypeDef     HAL_WWDT_SoftReset(WWDT_HandleTypeDef *hwwdt);
void                  HAL_WWDT_IRQHandler(WWDT_HandleTypeDef *hwwdt);
void                  HAL_WWDT_EarlyWarningCallback(WWDT_HandleTypeDef *hwwdt);
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

#ifdef __cplusplus
}
#endif

#endif /* __UM324XF_HAL_WWDT_H__ */
