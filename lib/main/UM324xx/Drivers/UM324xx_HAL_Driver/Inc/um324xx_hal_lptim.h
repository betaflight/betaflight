 /**
  ******************************************************************************
  * @file     um324xx_hal_lptim.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-06-26  
  * @brief   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_LPTIM_H__
#define __UM324XX_HAL_LPTIM_H__



#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup LPTIM
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup LPTIM_Exported_typedefs LPTIM Exported Typedefs
  * @{
  */ 
  
/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_LPTIM_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  HAL_LPTIM_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_LPTIM_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
  HAL_LPTIM_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
  HAL_LPTIM_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} HAL_LPTIM_StateTypeDef;

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup LPTIM_Exported_constants LPTIM Exported Constants
  * @{
  */ 

/** @defgroup LPTIM_CLOCK_DIVSEL LPTIM CLOCK DIVSEL
  * @{
  */
#define LPTIM_CLOCK_DIV_1                	(0x00000000U)
#define LPTIM_CLOCK_DIV_2                	((uint32_t)LPTIM_CFG_DIVSEL_0)  
#define LPTIM_CLOCK_DIV_4                	((uint32_t)LPTIM_CFG_DIVSEL_1)
#define LPTIM_CLOCK_DIV_8                	((uint32_t)(LPTIM_CFG_DIVSEL_0 | LPTIM_CFG_DIVSEL_1))
#define LPTIM_CLOCK_DIV_16               	((uint32_t)LPTIM_CFG_DIVSEL_2)
#define LPTIM_CLOCK_DIV_32               	((uint32_t)(LPTIM_CFG_DIVSEL_0 | LPTIM_CFG_DIVSEL_2))
#define LPTIM_CLOCK_DIV_64               	((uint32_t)(LPTIM_CFG_DIVSEL_1 | LPTIM_CFG_DIVSEL_2))
#define LPTIM_CLOCK_DIV_128              	((uint32_t)(LPTIM_CFG_DIVSEL_0 | LPTIM_CFG_DIVSEL_1 | LPTIM_CFG_DIVSEL_2))
/**
  * @}
  */

/** @defgroup LPTIM_CLOCK_SOURCE_SEL LPTIM CLOCK SOURCE SEL
  * @{
  */
#define LPTIM_CLOCK_SOURCE_LSCLK           	(0x00000000U)
#define LPTIM_CLOCK_SOURCE_RCLP          	((uint32_t)LPTIM_CFG_CLKSEL_0)  
#define LPTIM_CLOCK_SOURCE_PCLK0           	((uint32_t)LPTIM_CFG_CLKSEL_1)
#define LPTIM_CLOCK_SOURCE_LPTIN          	((uint32_t)(LPTIM_CFG_CLKSEL_0 | LPTIM_CFG_CLKSEL_1))
/**
  * @}
  */

/** @defgroup LPTIM_Interrupt_definition LPTIM interrupt Definition
  * @{
  */
#define LPTIM_IT_TRIG                      	LPTIM_IE_TRIGIE                         	/*!< Trigger interrupt      */
#define LPTIM_IT_OVER                      	LPTIM_IE_OVIE                       		/*!< Over interrupt 		*/
#define LPTIM_IT_COMP                      	LPTIM_IE_COMPIE                       		/*!< Compare interrupt 		*/
/**
  * @}
  */

/** @defgroup LPTIM_Flag_definition LPTIM Flag Definition
  * @{
  */
#define LPTIM_FLAG_TRIG                    	LPTIM_IF_TRIGIF                        	/*!< Trigger interrupt flag */
#define LPTIM_FLAG_OVER                    	LPTIM_IF_OVIF                       	/*!< Over interrupt flag    */
#define LPTIM_FLAG_COMP                    	LPTIM_IF_COMPIF                       	/*!< Compare interrupt flag	*/
/**
  * @}
  */

/** @defgroup LPTIM_PWM_modes LPTIM PWM modes
  * @{
  */
#define LPTIM_PWM_PERIOD_SQUARE             (0x00000000U)    	
#define LPTIM_PWM_PWM                    	LPTIM_CFG_PWM
/**
  * @}
  */
/** @defgroup LPTIM_PWM_Polarity  LPTIM PWM Polarity
  * @{
  */
#define LPTIM_PWM_POLARITY_HIGH             (0x00000000U)    	
#define LPTIM_PWM_POLARITY_LOW              LPTIM_CFG_POLARITY
/**
  * @}
  */

/** @defgroup LPTIM_Trigger_Edge  LPTIM Trigger Edge
  * @{
  */
#define LPTIM_TRIGGER_EDGE_RISE             (0x00000000U)    	
#define LPTIM_Trigger_Edge_FALL             LPTIM_CFG_TRIGCFG_0
#define LPTIM_Trigger_Edge_RISE_FALL        LPTIM_CFG_TRIGCFG_1
/**
  * @}
  */

/** @defgroup LPTIM_Count_Edge  LPTIM Count Edge
  * @{
  */
#define LPTIM_COUNT_EDGE_RISE             	(0x00000000U)    	
#define LPTIM_COUNT_EDGE_FALL              	LPTIM_CFG_EDGESEL
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LPTIM_Exported_macro LPTIM Exported Macro
  * @{
  */ 

/**
  * @brief  LPTIM base Configuration Structure definition
  */
typedef struct
{
  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

  uint32_t ClockSource;       /*!< Specifies the clock source.
                                   This parameter can be a value of @ref LPTIM_CLOCK_SOURCE_SEL */
	
  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref LPTIM_CLOCK_DIVSEL */

  uint32_t ContinuousMode;    /*!< Specifies the Continuous mode.
                                   This parameter can be a value of ENABLE or DISABLE */
} LPTIM_Base_InitTypeDef;

/**
  * @brief  LPTIM Output PWM Configuration Structure definition
  */
typedef struct
{
  uint32_t PWMMode;       /*!< Specifies the LPTIM PWM mode.
                               This parameter can be a value of @ref LPTIM_PWM_modes */

  uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Compare Register.
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t PWMPolarity;   /*!< Specifies the PWM polarity.
                               This parameter can be a value of @ref LPTIM_PWM_Polarity */

} LPTIM_PWM_InitTypeDef;

/**
  * @brief  LPTIM Trigger Configuration Structure definition
  */
typedef struct
{
  uint32_t TriggerEdge;   /*!< Specifies the LPTIM Trigger Edge.
                               This parameter can be a value of @ref LPTIM_Trigger_Edge */
	
  uint32_t TimeoutMode;   /*!< Specifies the LPTIM Timeout Mode.
                               This parameter can be a value of ENABLE or DISABLE */
} LPTIM_Trigger_InitTypeDef;

/**
  * @brief  LPTIM External Clock Configuration Structure definition
  */
typedef struct
{
  uint32_t CountEdge;     /*!< Specifies the LPTIM Count Edge.
                               This parameter can be a value of @ref LPTIM_Count_Edge */

} LPTIM_ExtClock_InitTypeDef;

/**
  * @brief  LPTIM Time Base Handle Structure definition
  */
typedef struct __LPTIM_HandleTypeDef
{
  LPTIM_TypeDef                         *Instance;         	/*!< Register base address                             */
  LPTIM_Base_InitTypeDef               	Init;              	/*!< LPTIM Time Base required parameters               */
  HAL_LockTypeDef                    	Lock;              	/*!< Locking object                                    */
  __IO HAL_LPTIM_StateTypeDef          	State;             	/*!< LPTIM operation state                             */

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  void (* Base_MspInitCallback)(struct __LPTIM_HandleTypeDef *lptim);              	/*!< LPTIM Base Msp Init Callback                              	*/
  void (* Base_MspDeInitCallback)(struct __LPTIM_HandleTypeDef *lptim);            	/*!< LPTIM Base Msp DeInit Callback                            	*/
  void (* Trigger_MspInitCallback)(struct __LPTIM_HandleTypeDef *lptim);           	/*!< LPTIM Trigger Msp Init Callback                        	*/
  void (* Trigger_MspDeInitCallback)(struct __LPTIM_HandleTypeDef *lptim);        	/*!< LPTIM Trigger Msp DeInit Callback                      	*/
  void (* PWM_MspInitCallback)(struct __LPTIM_HandleTypeDef *lptim);               	/*!< LPTIM PWM Msp Init Callback                               	*/
  void (* PWM_MspDeInitCallback)(struct __LPTIM_HandleTypeDef *lptim);             	/*!< LPTIM PWM Msp DeInit Callback                             	*/
  void (* TriggerCallback)(struct __LPTIM_HandleTypeDef *lptim);                 	/*!< LPTIM Trigger Callback                                		*/
  void (* OverCallback)(struct __LPTIM_HandleTypeDef *lptim); 					   	/*!< LPTIM Over Callback           							  	*/
  void (* CompCallback)(struct __LPTIM_HandleTypeDef *lptim); 					   	/*!< LPTIM Compare Callback           						  	*/
  void (* ErrorCallback)(struct __LPTIM_HandleTypeDef *lptim);                     	/*!< LPTIM Error Callback                                      	*/
#endif /* USE_HAL_LPLPTIM_REGISTER_CALLBACKS */
} LPTIM_HandleTypeDef;

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL LPTIM Callback ID enumeration definition
  */
typedef enum
{
  HAL_LPTIM_BASE_MSPINIT_CB_ID              = 0x00U   /*!< LPTIM Base MspInit Callback ID                              	*/
  , HAL_LPTIM_BASE_MSPDEINIT_CB_ID          = 0x01U   /*!< LPTIM Base MspDeInit Callback ID                            	*/
  , HAL_LPTIM_TRIGGER_MSPINIT_CB_ID         = 0x02U   /*!< LPTIM TRIGGER MspInit Callback ID                           	*/
  , HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID       = 0x03U   /*!< LPTIM TRIGGER MspDeInit Callback ID                         	*/
  , HAL_LPTIM_PWM_MSPINIT_CB_ID             = 0x04U   /*!< LPTIM PWM MspInit Callback ID                               	*/
  , HAL_LPTIM_PWM_MSPDEINIT_CB_ID           = 0x05U   /*!< LPTIM PWM MspDeInit Callback ID                             	*/
  , HAL_LPTIM_TRIGGER_CB_ID                 = 0x06U   /*!< LPTIM Trigger Callback ID                                   	*/
  , HAL_LPTIM_OVER_CB_ID              	    = 0x07U   /*!< LPTIM OVER Callback ID           		                   	*/
  , HAL_LPTIM_COMP_CB_ID         			= 0x08U   /*!< LPTIM Compare Callback ID        					        */
  , HAL_LPTIM_ERROR_CB_ID                   = 0x09U   /*!< LPTIM Error Callback ID                                      */
} HAL_LPTIM_CallbackIDTypeDef;

/**
  * @brief  HAL LPTIM Callback pointer definition
  */
typedef  void (*pLPTIM_CallbackTypeDef)(LPTIM_HandleTypeDef *hlptim);  /*!< pointer to the LPTIM callback function */

#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

/** @brief  Enable the specified LPTIM interrupt.
  * @param  __HANDLE__ specifies the LPTIM Handle.
  * @param  __INTERRUPT__ specifies the LPTIM interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg LPTIM_IT_TRIG:  Trigger interrupt
  *            @arg LPTIM_IT_OVER:  Over interrupt
  *            @arg LPTIM_IT_COMP:  Compare interrupt
  * @retval None
  */
#define __HAL_LPTIM_ENABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->IE |= (__INTERRUPT__))

/** @brief  Check whether the specified LPTIM interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the LPTIM Handle.
  * @param  __INTERRUPT__ specifies the LPTIM interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg LPTIM_IT_TRIG:  Trigger interrupt
  *            @arg LPTIM_IT_OVER:  Over interrupt
  *            @arg LPTIM_IT_COMP:  Compare interrupt
  * @retval None
  */
#define __HAL_LPTIM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)    (((__HANDLE__)->Instance->IE & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Check whether the specified LPTIM interrupt flag is set or not.
  * @param  __HANDLE__ specifies the LPTIM Handle.
  * @param  __FLAG__ specifies the LPTIM interrupt flag to check.
  *          This parameter can be one of the following values:
  *            @arg LPTIM_FLAG_TRIG:  Trigger interrupt flag
  *            @arg LPTIM_FLAG_OVER:  Over interrupt flag
  *            @arg LPTIM_FLAG_COMP:  Compare interrupt flag
  * @retval None
  */
#define __HAL_LPTIM_GET_FLAG(__HANDLE__, __FLAG__)    (((__HANDLE__)->Instance->IF & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified LPTIM interrupt flag.
  * @param  __HANDLE__ specifies the LPTIM Handle.
  * @param  __FLAG__ specifies the LPTIM interrupt flag to clear.
  *          This parameter can be one of the following values:
  *            @arg LPTIM_FLAG_TRIG:  Trigger interrupt flag
  *            @arg LPTIM_FLAG_OVER:  Over interrupt flag
  *            @arg LPTIM_FLAG_COMP:  Compare interrupt flag
  * @retval None
  */
#define __HAL_LPTIM_CLEAR_FLAG(__HANDLE__, __FLAG__)    ((__HANDLE__)->Instance->IF = (__FLAG__))

/**
  * @brief  Enable the LPTIM peripheral.
  * @param  __HANDLE__ LPTIM handle
  * @retval None
  */
#define __HAL_LPTIM_ENABLE(__HANDLE__)                  ((__HANDLE__)->Instance->CTRL |=(LPTIM_CTRL_LPTEN))

/**
  * @brief  Disable the LPTIM peripheral.
  * @param  __HANDLE__ LPTIM handle
  * @retval None
  */
#define __HAL_LPTIM_DISABLE(__HANDLE__)                  ((__HANDLE__)->Instance->CTRL &= ~(LPTIM_CTRL_LPTEN))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup LPTIM_Exported_Functions
  * @{
  */ 
/** @addtogroup LPTIM_Exported_Functions_Group1 Lptime Base functions
  *  @brief   Lptime Base functions
  * @{
  */
/* Lptime Base functions ********************************************************/
HAL_StatusTypeDef HAL_LPTIM_Base_Init(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Base_DeInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_Base_MspInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_Base_MspDeInit(LPTIM_HandleTypeDef *hlptim);

/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_LPTIM_Base_Start(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Base_Stop(LPTIM_HandleTypeDef *hlptim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_LPTIM_Base_Start_IT(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Base_Stop_IT(LPTIM_HandleTypeDef *hlptim);
/**
  * @}
  */
/** @addtogroup LPTIM_Exported_Functions_Group2 Lptime PWM functions
  *  @brief   Lptime PWM functions
  * @{
  */
/* Lptimer PWM functions ********************************************************/
HAL_StatusTypeDef HAL_LPTIM_PWM_Init(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_PWM_DeInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_PWM_MspInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_PWM_MspDeInit(LPTIM_HandleTypeDef *hlptim);
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_LPTIM_PWM_Start(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop(LPTIM_HandleTypeDef *hlptim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_LPTIM_PWM_Start_IT(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop_IT(LPTIM_HandleTypeDef *hlptim);
/**
  * @}
  */
/** @addtogroup LPTIM_Exported_Functions_Group3 Lptime Trigger functions
  *  @brief   Lptime Trigger functions
  * @{
  */
/* Lptime ExtTrigger functions ********************************************************/
HAL_StatusTypeDef HAL_LPTIM_Trigger_Init(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Trigger_DeInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_Trigger_MspInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_Trigger_MspDeInit(LPTIM_HandleTypeDef *hlptim);

/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Start(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Trigger_Stop(LPTIM_HandleTypeDef *hlptim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Start_IT(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Trigger_Stop_IT(LPTIM_HandleTypeDef *hlptim);
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup LPTIM_Exported_Functions_Group4 Lptime Callbacks functions
  *  @brief   Lptime Callbacks functions
  * @{
  */
void HAL_LPTIM_OverCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_CompCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_TriggerCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_ErrorCallback(LPTIM_HandleTypeDef *hlptim);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_LPTIM_RegisterCallback(LPTIM_HandleTypeDef *hlptim, HAL_LPTIM_CallbackIDTypeDef CallbackID,
                                           pLPTIM_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_LPTIM_UnRegisterCallback(LPTIM_HandleTypeDef *hlptim, HAL_LPTIM_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group5 Lptime IRQ handler management
  *  @brief   Lptime IRQ handler management
  * @{
  */
/* Interrupt Handler functions  ***********************************************/
void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *hlptim);
/**
  * @}
  */

/** @defgroup LPTIM_Exported_Functions_Group6 LPTIM Peripheral Control functions
  *  @brief   Peripheral Control functions
  * @{
  */
/* Control functions  *********************************************************/
HAL_StatusTypeDef HAL_LPTIM_PWM_ConfigChannel(LPTIM_HandleTypeDef *hlptim, LPTIM_PWM_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_LPTIM_Trigger_ConfigChannel(LPTIM_HandleTypeDef *hlptim, LPTIM_Trigger_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_LPTIM_ext_ClockConfig(LPTIM_HandleTypeDef *hlptim, LPTIM_ExtClock_InitTypeDef *sConfig);
/**
  * @}
  */

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup LPTIM_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup LPTIM_Private_Functions LPTIM Private Functions
  * @{
  */  
void LPTIM_Base_SetConfig(LPTIM_HandleTypeDef *hlptim);

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
void LPTIM_ResetCallback(LPTIM_HandleTypeDef *hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
