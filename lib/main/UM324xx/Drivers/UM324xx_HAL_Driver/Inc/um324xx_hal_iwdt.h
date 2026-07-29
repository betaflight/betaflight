/**
  ******************************************************************************
  * @file    um324xx_hal_iwdt.h
  * @author  MCU Team
  * @version V1.00
  * @date    2023-4-21
  * @brief   Header file of IWDT HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 - 2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_IWDT_H__
#define __UM324XX_HAL_IWDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup IWDT
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup IWDT_Exported_Types IWDT Exported Types
  * @{
  */

/**
  * @brief  IWDT Init structure definition
  */
typedef struct
{
  uint32_t Load;                                                       /*!< Specifies the IWDT free-running upcounter  value.
                                                                            This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F */                                                                                    
  uint32_t Inten;                                                      /*!< Specifies if IWDT Warning Interrupt is enable or not.
                                                                            This parameter can be a value of @ref IWDT_IE_Mode */
  uint32_t Rsten;                                                      /*!< Specifies if IWDT Warning reset is enable or not.
                                                                            This parameter can be a value of @ref IWDT_RST_Mode */
  uint32_t RstTimes;                                                      /*!< Specifies if IWDT Warning reset counter numbers.
                                                                            This parameter can be a value of @ref IWDT_RST_TIMES_Mode */
  uint32_t Clkdiv;                                                     /*!< Specifies if IWDT CLK devison.
                                                                            This parameter can be a value of ((0 ~ 0xFFFF)+1) prescale */
  uint32_t Stallen;                                                    /*!< Specifies if IWDT stall state is enable or not.
                                                                            This parameter can be a value of @ref IWDT_STALL_Mode */
} IWDT_InitTypeDef;

/**
  * @brief  IWDT handle Structure definition
  */
#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
typedef struct __IWDT_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */
{
  IWDT_TypeDef      *Instance;                                                 /*!< Register base address */
  IWDT_InitTypeDef  Init;                                                      /*!< IWDT required parameters */

#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
  void (* IntCallback)(struct __IWDT_HandleTypeDef *hiwdt);                    /*!< IWDT Interrupt callback */
  void (* MspInitCallback)(struct __IWDT_HandleTypeDef *hiwdt);                /*!< IWDT Msp Init callback */
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */
} IWDT_HandleTypeDef;

#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL IWDT common Callback ID enumeration definition
  */
typedef enum
{
  HAL_IWDT_INT_CB_ID           = 0x00U,                                         /*!< IWDT IE callback ID */
  HAL_IWDT_MSPINIT_CB_ID       = 0x01U,                                         /*!< IWDT MspInit callback ID */
} HAL_IWDT_CallbackIDTypeDef;

/**
  * @brief  HAL IWDT Callback pointer definition
  */
typedef void (*pIWDT_CallbackTypeDef)(IWDT_HandleTypeDef *hppp);  /*!< pointer to a IWDT common callback functions */

#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup IWDT_Exported_Constants IWDT Exported Constants
  * @{
  */

/** @defgroup IWDT_Interrupt_definition IWDT Interrupt definition
  * @{
  */
#define IWDT_IT_IE                     IWDT_CTRL_INTEN  /*!< Overfolw interrupt */
/**
  * @}
  */

/** @defgroup IWDT_Reset_Mode_definition IWDT Reset Mode definition
  * @{
  */
#define IWDT_RST_TIMES_IE              IWDT_CTRL_RST_MODE  /*!<  Reset times  */
/**
  * @}
  */

/** @defgroup IWDT_Reset_definition IWDT Reset definition
  * @{
  */
#define IWDT_RST_IE                    IWDT_CTRL_RSTEN  /*!< Overfolw  Reset   */
/**
  * @}
  */

/** @defgroup IWDT_Stall_definition IWDT Stall definition
  * @{
  */
#define IWDT_STALL_IE                  IWDT_STALL_STALL  /*!< Counter stop work at Halt Status */
/**
  * @}
  */

/** @defgroup IWDT_Flag_definition IWDT Flag definition
  * @brief IWDT Flag definition
  * @{
  */
#if defined(UM32x42x) || defined(UM32x41x)
#define IWDT_FLAG_IF                   IWDT_MINTS_INTMS  /*!< Overfolw interrupt flag */
#endif
#if defined(UM324xF)
#define IWDT_FLAG_IF                   IWDT_INTMS_INTMS  /*!< Overfolw interrupt flag */
#endif

#define IWDT_FLAG_WRC                  IWDT_CTRL_WRC  /*!< Write CTRL or LOAD registers update flag */
/**
  * @}
  */

/** @defgroup IWDT_IE_Mode IWDT Interrupt Mode
  * @{
  */
#define IWDT_IE_DISABLE                0x00000000u                              /*!< IE Disable */
#define IWDT_IE_ENABLE                 IWDT_CTRL_INTEN                          /*!< IE Enable  */
/**
  * @}
  */

/** @defgroup IWDT_RST_TIMES_Mode IWDT Reset Times Mode
  * @{
  */
#define IWDT_RST_2TIMES                0x00000000u                              /*!< Overflow Reset 2 times  */
#define IWDT_RST_1TIMES                IWDT_CTRL_RST_MODE                       /*!< Overflow Reset 1 times */
/**
  * @}
  */

/** @defgroup IWDT_RST_Mode IWDT Reset Mode
  * @{
  */
#define IWDT_RST_DISABLE                0x00000000u                              /*!< Overflow Reset Disable */
#define IWDT_RST_ENABLE                 IWDT_CTRL_RSTEN                          /*!< Overflow Reset Enable  */
/**
  * @}
  */


/** @defgroup IWDT_STALL_Mode IWDT Stall Mode
  * @{
  */
#define IWDT_STALL_DISABLE              0x00000000u                              /*!< Counter continue at halt status  */
#define IWDT_STALL_ENABLE               IWDT_STALL_STALL                          /*!< Counter stop at halt status  */
/**
  * @}
  */
  
/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/

/** @defgroup IWDT_Exported_Macros IWDT Exported Macros
  * @{
  */

/**
  * @brief  IWDT start to work.
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define __HAL_IWDT_START(__HANDLE__, __LOAD__)                         WRITE_REG((__HANDLE__)->Instance->LOAD, (__LOAD__))

/**
  * @brief  Reload the IWDT_CNT register.
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define __HAL_IWDT_RELOAD(__HANDLE__)                      ((__HANDLE__)->Instance->CLR = (IWDT_CLR_CARRY))

/**
  * @brief  Enable the IWDT reset.
  * @param  __HANDLE__     IWDT handle
  * @param  __RESET__      specifies the reset to enable.
  *         This parameter can be one of the following values:
  *            @arg IWDT_RST_IE: Reset
  * @note   Once enabled this reset cannot be disabled except by a system reset.
  * @retval None
  */
#define __HAL_IWDT_ENABLE_RST(__HANDLE__, __RESET__)       SET_BIT((__HANDLE__)->Instance->CTRL, __RESET__)

/**
  * @brief  Enable the IWDT stall.
  * @param  __HANDLE__     IWDT handle
  * @param  __STALL__  specifies the stall to enable.
  *         This parameter can be one of the following values:
  *            @arg IWDT_STALl_IE: Stall 
  * @note   
  * @retval None
  */
#define __HAL_IWDT_ENABLE_STALL(__HANDLE__, __STALL__)       SET_BIT((__HANDLE__)->Instance->STALL, __STALL__)

/**
  * @brief  Enable the IWDT interrupt.
  * @param  __HANDLE__     IWDT handle
  * @param  __INTERRUPT__  specifies the interrupt to enable.
  *         This parameter can be one of the following values:
  *            @arg IWDT_IT_IE: Interrupt 
  * @note   Once enabled this interrupt cannot be disabled except by a system reset.
  * @retval None
  */
#define __HAL_IWDT_ENABLE_IT(__HANDLE__, __INTERRUPT__)        SET_BIT((__HANDLE__)->Instance->CTRL, (__INTERRUPT__))

/**
  * @brief  Check whether the specified IWDT flag is set or not.
  * @param  __HANDLE__  IWDT handle
  * @param  __FLAG__  specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg IWDT_FLAG_IF:  Interrupt flag
  * @retval The new state of IWDT_FLAG (SET or RESET).
  */
#if defined(UM32x42x) || defined(UM32x41x) 
#define __HAL_IWDT_GET_FLAG_IT(__HANDLE__, __FLAG_IT__)            (((__HANDLE__)->Instance->MINTS & (__FLAG_IT__)) == (__FLAG_IT__))
#endif
#if defined(UM324xF)
#define __HAL_IWDT_GET_FLAG_IT(__HANDLE__, __FLAG_IT__)            (((__HANDLE__)->Instance->INTMS & (__FLAG_IT__)) == (__FLAG_IT__))
#endif
/**
  * @brief  Clear the IWDT's pending flags.
  * @param  __HANDLE__  IWDT handle
  * @param  __FLAG__  specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg IWDT_FLAG_IF:  Interrupt flag
  * @retval None
  */
#define __HAL_IWDT_CLEAR_FLAG_IT(__HANDLE__)                    ((__HANDLE__)->Instance->CLR = (IWDT_CLR_CARRY))

/** @brief  Check whether the specified IWDT interrupt source is enabled or not.
  * @param  __HANDLE__  IWDT Handle.
  * @param  __INTERRUPT__  specifies the IWDT interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg IWDT_IT_IE: IWDT Interrupt
  * @retval state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_IWDT_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (((__HANDLE__)->Instance->INTRAW\
                                                                    & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Check whether the specified IWDT flag is set or not.
  * @param  __HANDLE__      IWDT handle
  * @param  __FLAG_WRC__    specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg IWDT_FLAG_WRC: write Ctrl or Load registers update flag
  * @retval The new state of IWDT_FLAG_WRC (SET or RESET).
  */
#define __HAL_IWDT_GET_FLAG_WRC(__HANDLE__, __FLAG_WRC__)            (((__HANDLE__)->Instance->MINTS & (__FLAG_WRC__)) == (__FLAG_WRC__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup IWDT_Exported_Functions
  * @{
  */

/** @addtogroup IWDT_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef     HAL_IWDT_Init(IWDT_HandleTypeDef *hiwdt);
HAL_StatusTypeDef     HAL_IWDT_Start(IWDT_HandleTypeDef *hiwdt);
void                  HAL_IWDT_MspInit(IWDT_HandleTypeDef *hiwdt);

/* Callbacks Register/UnRegister functions  *************************************/
#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef     HAL_IWDT_RegisterCallback(IWDT_HandleTypeDef *hiwdt, HAL_IWDT_CallbackIDTypeDef CallbackID,
                                                pIWDT_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_IWDT_UnRegisterCallback(IWDT_HandleTypeDef *hiwdt, HAL_IWDT_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup IWDT_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
HAL_StatusTypeDef     HAL_IWDT_Refresh(IWDT_HandleTypeDef *hiwdt);
void                  HAL_IWDT_IRQHandler(IWDT_HandleTypeDef *hiwdt);
void                  HAL_IWDT_IntCallback(IWDT_HandleTypeDef *hiwdt);
/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/


/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* UM324xF_HAL_IWDT_H */
