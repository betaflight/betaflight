/**
  ******************************************************************************
  * @file    um324xx_hal_cordic.h
  * @author  MCU Team
  * @version V1.00 
  * @date    2023-04-11
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

#ifndef __UM324XX_HAL_CORDIC_H__
#define __UM324XX_HAL_CORDIC_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup CORDIC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Types CORDIC Exported Types
  * @{
  */

/**
  * @brief  CORDIC HAL State Structure definition
  */
typedef enum
{
  HAL_CORDIC_STATE_RESET     = 0x00U,  /*!< CORDIC not yet initialized or disabled */
  HAL_CORDIC_STATE_READY     = 0x01U,  /*!< CORDIC initialized and ready for use   */
  HAL_CORDIC_STATE_BUSY      = 0x02U,  /*!< CORDIC internal process is ongoing     */
  HAL_CORDIC_STATE_ERROR     = 0x03U   /*!< CORDIC error state                     */
} HAL_CORDIC_StateTypeDef;


/**
  * @brief CRC Init Structure definition
  */
typedef struct
{
  uint32_t   Function;     /*!< Function
                                This parameter can be a value of @ref CORDIC_Function */

  uint32_t   Scale;        /*!< Scaling factor
                                This parameter can be a value of @ref CORDIC_Scale */
  uint32_t   Iteration;     /*!<The more iterations/2 in the operation, the more accurate and 
                                time-consuming it is. The available range is [4,12] */
    
  uint32_t   WIDTH_OUT;     /*!< Width of output data
                                This parameter can be a value of @ref CORDIC_Out_Size */
  uint32_t   WIDTH_IN;      /*!< Width of input data
                                This parameter can be a value of @ref CORDIC_In_Size */

  uint32_t   ADDR_IN;       /*!< Output address mode:0: Read data from two output data registers separately
                                    1: Two data can be sequentially read from output data register 1 */
  uint32_t   ADDR_OUT;      /*!<Enter address mode:0: Two parameters need to be written to two data input registers respectively
                                    1: Two parameters can be sequentially written to input data register 1 */
    
  uint32_t   MERGE_IN;          /*!< Number of 32-bit read expected after one calculation
                                This parameter can be a value of @ref CORDIC_Nb_Read */
  uint32_t   MERGE_OUT;        /*!< Number of 32-bit write expected for one calculation
                                This parameter can be a value of @ref CORDIC_Nb_Write */
                                                                                             
} CORDIC_InitTypeDef;

/**
  * @brief  CORDIC Handle Structure definition
  */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
typedef struct __CORDIC_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
{
  CORDIC_TypeDef                *Instance;   /*!< Register base address */

  CORDIC_InitTypeDef            Init;       /*!< Cordic communication parameters      */
    
  int32_t                       *pInBuff;    /*!< Pointer to CORDIC input data buffer */

  int32_t                       *pOutBuff;   /*!< Pointer to CORDIC output data buffer */

  uint32_t                      NbCalcToOrder; /*!< Remaining number of calculation to order */

  uint32_t                      NbCalcToGet; /*!< Remaining number of calculation result to get */

  uint32_t                      DMADirection; /*!< Direction of CORDIC DMA transfers */

  DMA_HandleTypeDef              *hdmaIn;     /*!< CORDIC peripheral input data DMA handle parameters */
    
  DMA_HandleTypeDef              *hdmaOut;    /*!< CORDIC peripheral output data DMA handle parameters */

  HAL_LockTypeDef               Lock;        /*!< CORDIC locking object */

  __IO HAL_CORDIC_StateTypeDef  State;       /*!< CORDIC state */

  __IO uint32_t                 ErrorCode;   /*!< CORDIC peripheral error code
                                                  This parameter can be a value of @ref CORDIC_Error_Code */

#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1

  void (* MspInitCallback)(struct __CORDIC_HandleTypeDef *hcordic);        /*!< CORDIC Msp Init callback */
  void (* MspDeInitCallback)(struct __CORDIC_HandleTypeDef *hcordic);      /*!< CORDIC Msp DeInit callback */
  void (* CalculateCpltCallback)(struct __CORDIC_HandleTypeDef *hcordic);  /*!< CORDIC Calculate Cplt callback */  
  void (* ErrorCallback)(struct __CORDIC_HandleTypeDef *hcordic);          /*!< CORDIC Error callback */  


#endif /* (USE_HAL_CORDIC_REGISTER_CALLBACKS) */

} CORDIC_HandleTypeDef;



#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
/**
  * @brief  HAL CORDIC Callback ID enumeration definition
  */
typedef enum
{
  HAL_CORDIC_ERROR_CB_ID             = 0x00U,    /*!< CORDIC error callback ID */
  HAL_CORDIC_CALCULATE_CPLT_CB_ID    = 0x01U,    /*!< CORDIC calculate complete callback ID */

  HAL_CORDIC_MSPINIT_CB_ID           = 0x02U,    /*!< CORDIC MspInit callback ID */
  HAL_CORDIC_MSPDEINIT_CB_ID         = 0x03U,    /*!< CORDIC MspDeInit callback ID */

} HAL_CORDIC_CallbackIDTypeDef;

/**
  * @brief  HAL CORDIC Callback pointer definition
  */
typedef  void (*pCORDIC_CallbackTypeDef)(CORDIC_HandleTypeDef *hcordic);  /*!< pointer to a CORDIC callback function */

#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */

/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Constants CORDIC Exported Constants
  * @{
  */

/** @defgroup CORDIC_Error_Code CORDIC Error code
  * @{
  */
#define HAL_CORDIC_ERROR_NONE              ((uint32_t)0x00000000U)   /*!< No error                */
#define HAL_CORDIC_ERROR_PARAM             ((uint32_t)0x00000001U)   /*!< Wrong parameter error   */
#define HAL_CORDIC_ERROR_NOT_READY         ((uint32_t)0x00000002U)   /*!< Peripheral not ready    */
#define HAL_CORDIC_ERROR_TIMEOUT           ((uint32_t)0x00000004U)   /*!< Timeout error           */
#define HAL_CORDIC_ERROR_DMA               ((uint32_t)0x00000008U)   /*!< DMA error               */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
#define HAL_CORDIC_ERROR_INVALID_CALLBACK  ((uint32_t)0x00000010U)   /*!< Invalid Callback error  */
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup CORDIC_Function CORDIC Function
  * @{
  */
#define CORDIC_FUNCTION_MODE0 (0x00000000U)                                                          
#define CORDIC_FUNCTION_MODE1 ((uint32_t)(CORDIC_CTRL_MODE_0))                                        
#define CORDIC_FUNCTION_MODE2 ((uint32_t)(CORDIC_CTRL_MODE_1))                                        
#define CORDIC_FUNCTION_MODE3 ((uint32_t)(CORDIC_CTRL_MODE_1 | CORDIC_CTRL_MODE_0))                    
#define CORDIC_FUNCTION_MODE4 ((uint32_t)(CORDIC_CTRL_MODE_2))                                        
#define CORDIC_FUNCTION_MODE5 ((uint32_t)(CORDIC_CTRL_MODE_2 | CORDIC_CTRL_MODE_0))                    
#define CORDIC_FUNCTION_MODE6 ((uint32_t)(CORDIC_CTRL_MODE_2 | CORDIC_CTRL_MODE_1))                    
#define CORDIC_FUNCTION_MODE7 ((uint32_t)(CORDIC_CTRL_MODE_2 | CORDIC_CTRL_MODE_1 | CORDIC_CTRL_MODE_0))

/*!< Square Root */
/**
  * @}
  */

/** @defgroup CORDIC_Precision_In_Cycles_Number CORDIC Precision in Cycles Number
  * @{
  */
/* Note: 1 cycle corresponds to 4 algorithm iterations */

#define CORDIC_PRECISION_4CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_2))
#define CORDIC_PRECISION_5CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_2 | CORDIC_CTRL_ITERATION_0))
#define CORDIC_PRECISION_6CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_2 | CORDIC_CTRL_ITERATION_1))
#define CORDIC_PRECISION_7CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_2\
                                              | CORDIC_CTRL_ITERATION_1 | CORDIC_CTRL_ITERATION_0))
#define CORDIC_PRECISION_8CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_3))
#define CORDIC_PRECISION_9CYCLES    ((uint32_t)(CORDIC_CTRL_ITERATION_3 | CORDIC_CTRL_ITERATION_0))
#define CORDIC_PRECISION_10CYCLES   ((uint32_t)(CORDIC_CTRL_ITERATION_3 | CORDIC_CTRL_ITERATION_1))
#define CORDIC_PRECISION_11CYCLES   ((uint32_t)(CORDIC_CTRL_ITERATION_3\
                                                | CORDIC_CTRL_ITERATION_1 | CORDIC_CTRL_ITERATION_0))
#define CORDIC_PRECISION_12CYCLES   ((uint32_t)(CORDIC_CTRL_ITERATION_3 | CORDIC_CTRL_ITERATION_2))

/**
  * @}
  */

/** @defgroup CORDIC_Scale CORDIC Scaling factor
  * @{
  */
/* Scale factor value 'n' implies that the input data have been multiplied
   by a factor 2exp(-n), and/or the output data need to be multiplied by 2exp(n). */
#define CORDIC_SCALE_0              (0x00000000U)
#define CORDIC_SCALE_1              ((uint32_t)(CORDIC_CTRL_SCALE_0))
#define CORDIC_SCALE_2              ((uint32_t)(CORDIC_CTRL_SCALE_1))
#define CORDIC_SCALE_3              ((uint32_t)(CORDIC_CTRL_SCALE_1 | CORDIC_CTRL_SCALE_0))

/**
  * @}
  */

/** @defgroup CORDIC_DMA_Direction CORDIC DMA direction
  * @{
  */
#define CORDIC_DMA_DIR_NONE        ((uint32_t)0x00000000U)   /*!< DMA direction : none */
#define CORDIC_DMA_DIR_IN          ((uint32_t)0x00000001U)   /*!< DMA direction : Input of CORDIC */
#define CORDIC_DMA_DIR_OUT         ((uint32_t)0x00000002U)   /*!< DMA direction : Output of CORDIC */
#define CORDIC_DMA_DIR_IN_OUT      ((uint32_t)0x00000003U)   /*!< DMA direction : Input and Output of CORDIC */
/**
  * @}
  */

/** @defgroup CORDIC_Interrupts_Enable CORDIC Interrupts Enable bit
  * @{
  */
#define CORDIC_IT_IEN              CORDIC_CSR_IEN            /*!< Result ready interrupt enable */
/**
  * @}
  */

/** @defgroup CORDIC_DMAR DMA Read Request Enable bit
  * @{
  */
#define CORDIC_DMA_OUT             CORDIC_CTRL_DMA_OUT         /*!< DMA Read requests enable */
/**
  * @}
  */

/** @defgroup CORDIC_DMAW DMA Write Request Enable bit
  * @{
  */
#define CORDIC_DMA_IN              CORDIC_CTRL_DMA_IN         /*!< DMA Write channel enable */
/**
  * @}
  */

/** @defgroup CORDIC_Nb_Write CORDIC Number of 32-bit write required for one calculation
  * @{
  */
#define CORDIC_MERGE_OUT_1           CORDIC_CTRL_MERGE_OUT           
#define CORDIC_MERGE_OUT_2           (0x00000000U)          
/**
  * @}
  */

/** @defgroup CORDIC_Nb_Read CORDIC Number of 32-bit read required after one calculation
  * @{
  */
#define CORDIC_MERGE_IN_1            CORDIC_CTRL_MERGE_IN           
#define CORDIC_MERGE_IN_2             (0x00000000U)           
/**
  * @}
  */
  
/** @defgroup CORDIC_Nb_Write CORDIC Number of 32-bit write required for one calculation
  * @{
  */
#define CORDIC_ADDR_OUT_1           CORDIC_CTRL_ADDR_OUT           
#define CORDIC_ADDR_OUT_2           (0x00000000U)          
/**
  * @}
  */

/** @defgroup CORDIC_Nb_Read CORDIC Number of 32-bit read required after one calculation
  * @{
  */
#define CORDIC_ADDR_IN_1            CORDIC_CTRL_ADDR_IN           
#define CORDIC_ADDR_IN_2             (0x00000000U)           
/**
  * @}
  */

/** @defgroup CORDIC_In_Size CORDIC input data size
  * @{
  */
#define CORDIC_INSIZE_32BITS       (0x00000000U)             /*!< 32 bits input data size (Q1.31 format) */
#define CORDIC_INSIZE_16BITS       CORDIC_CTRL_WIDTH_IN        /*!< 16 bits input data size (Q1.15 format) */
/**
  * @}
  */

/** @defgroup CORDIC_Out_Size CORDIC Results Size
  * @{
  */
#define CORDIC_OUTSIZE_32BITS      (0x00000000U)             /*!< 32 bits output data size (Q1.31 format) */
#define CORDIC_OUTSIZE_16BITS      CORDIC_CTRL_WIDTH_OUT        /*!< 16 bits output data size (Q1.15 format) */
/**
  * @}
  */

/** @defgroup CORDIC_Flags  CORDIC status flags
  * @{
  */
#define CORDIC_FLAG_RRDY           CORDIC_CTRL_DATA_READY           /*!< Result Ready Flag */
/**
  * @}
  */

/** @defgroup CORDIC_DMA_Direction CORDIC DMA direction
  * @{
  */
#define CORDIC_DMA_OUT_ENABLE           CORDIC_CTRL_DMA_OUT    
#define CORDIC_DMA_OUT_DISABLE          0x00000000U            
#define CORDIC_DMA_IN_ENABLE            CORDIC_CTRL_DMA_IN     
#define CORDIC_DMA_IN_DISABLE           0x00000000U            
/**
  * @}
  */

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Macros CORDIC Exported Macros
  * @{
  */

/** @brief  Reset CORDIC handle state.
  * @param  __HANDLE__ CORDIC handle
  * @retval None
  */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
#define __HAL_CORDIC_RESET_HANDLE_STATE(__HANDLE__) do{                                                \
                                                        (__HANDLE__)->State = HAL_CORDIC_STATE_RESET;   \
                                                        (__HANDLE__)->MspInitCallback = NULL;           \
                                                        (__HANDLE__)->MspDeInitCallback = NULL;         \
                                                      } while(0)
#else
#define __HAL_CORDIC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CORDIC_STATE_RESET)
#endif /*USE_HAL_CORDIC_REGISTER_CALLBACKS */

/**
  * @brief  Enable the CORDIC interrupt when result is ready
  * @param  __HANDLE__ CORDIC handle.
  * @param  __INTERRUPT__ CORDIC Interrupt.
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_IT_IEN Enable Interrupt
  * @retval None
  */
#define __HAL_CORDIC_ENABLE_IT(__HANDLE__, __INTERRUPT__)                     \
  (((__HANDLE__)->Instance->CSR) |= (__INTERRUPT__))

/**
  * @brief  Disable the CORDIC interrupt
  * @param  __HANDLE__ CORDIC handle.
  * @param  __INTERRUPT__ CORDIC Interrupt.
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_IT_IEN Enable Interrupt
  * @retval None
  */
#define __HAL_CORDIC_DISABLE_IT(__HANDLE__, __INTERRUPT__)                    \
  (((__HANDLE__)->Instance->CSR) &= ~(__INTERRUPT__))

/** @brief  Check whether the specified CORDIC interrupt occurred or not.
            Dummy macro as no interrupt status flag.
  * @param  __HANDLE__ CORDIC handle.
  * @param  __INTERRUPT__ CORDIC interrupt to check
  * @retval SET (interrupt occurred) or RESET (interrupt did not occurred)
  */
#define __HAL_CORDIC_GET_IT(__HANDLE__, __INTERRUPT__)     /* Dummy macro */

/** @brief  Clear specified CORDIC interrupt status. Dummy macro as no
            interrupt status flag.
  * @param  __HANDLE__ CORDIC handle.
  * @param  __INTERRUPT__ CORDIC interrupt to clear
  * @retval None
  */
#define __HAL_CORDIC_CLEAR_IT(__HANDLE__, __INTERRUPT__)   /* Dummy macro */

/** @brief  Check whether the specified CORDIC status flag is set or not.
  * @param  __HANDLE__ CORDIC handle.
  * @param  __FLAG__ CORDIC flag to check
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_FLAG_RRDY Result Ready Flag
  * @retval SET (flag is set) or RESET (flag is reset)
  */
#define __HAL_CORDIC_GET_FLAG(__HANDLE__, __FLAG__)                           \
  ((((__HANDLE__)->Instance->CSR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear specified CORDIC status flag. Dummy macro as no
            flag can be cleared.
  * @param  __HANDLE__ CORDIC handle.
  * @param  __FLAG__ CORDIC flag to clear
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_FLAG_RRDY Result Ready Flag
  * @retval None
  */
#define __HAL_CORDIC_CLEAR_FLAG(__HANDLE__, __FLAG__)     /* Dummy macro */

/** @brief  Check whether the specified CORDIC interrupt is enabled or not.
  * @param  __HANDLE__ CORDIC handle.
  * @param  __INTERRUPT__ CORDIC interrupt to check
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_IT_IEN Enable Interrupt
  * @retval FlagStatus
  */
#define __HAL_CORDIC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)                 \
  (((__HANDLE__)->Instance->CSR) & (__INTERRUPT__))

/**
  * @}
  */



/** @addtogroup CORDIC_Exported_Functions
  * @{
  */
/* Exported functions ------------------------------------------------------- */

/** @addtogroup CORDIC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions ******************************/
HAL_StatusTypeDef HAL_CORDIC_Init(CORDIC_HandleTypeDef *hcordic);
HAL_StatusTypeDef HAL_CORDIC_DeInit(CORDIC_HandleTypeDef *hcordic);
void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef *hcordic);
void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef *hcordic);

#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
/* Callbacks Register/UnRegister functions  ***********************************/
HAL_StatusTypeDef HAL_CORDIC_RegisterCallback(CORDIC_HandleTypeDef *hcordic, HAL_CORDIC_CallbackIDTypeDef CallbackID,
                                              pCORDIC_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_CORDIC_UnRegisterCallback(CORDIC_HandleTypeDef *hcordic, HAL_CORDIC_CallbackIDTypeDef CallbackID);
/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group2
  * @{
  */
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
/* Peripheral Control functions ***********************************************/
//HAL_StatusTypeDef HAL_CORDIC_Configure(CORDIC_HandleTypeDef *hcordic, CORDIC_ConfigTypeDef *sConfig);
HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuff, int32_t *pOutBuff,
                                       uint32_t Timeout);
HAL_StatusTypeDef HAL_CORDIC_Calculate_DMA(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuff, int32_t *pOutBuff,
                                            uint32_t DMADirection);
/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group3
  * @{
  */
/* Callback functions *********************************************************/
void HAL_CORDIC_ErrorCallback(CORDIC_HandleTypeDef *hcordic);
void HAL_CORDIC_CalculateCpltCallback(CORDIC_HandleTypeDef *hcordic);
/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group4
  * @{
  */
/* IRQ handler management *****************************************************/
void HAL_CORDIC_IRQHandler(CORDIC_HandleTypeDef *hcordic);
/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group5
  * @{
  */
/* Peripheral State functions *************************************************/
HAL_CORDIC_StateTypeDef HAL_CORDIC_GetState(CORDIC_HandleTypeDef *hcordic);
uint32_t HAL_CORDIC_GetError(CORDIC_HandleTypeDef *hcordic);
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

#endif /* CORDIC */

#ifdef __cplusplus
}


#endif /* UM32x42x_HAL_CORDIC_H */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

