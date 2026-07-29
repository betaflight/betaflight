 /**
  ******************************************************************************
  * @file     um324xx_hal_acmp.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-17  
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
#ifndef __UM324XX_HAL_ACMP_H__
#define __UM324XX_HAL_ACMP_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup ACMP
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup ACMP_Exported_typedefs ACMP Exported Typedefs
  * @{
  */ 
  
typedef enum
{
    HAL_ACMP_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
    HAL_ACMP_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
    HAL_ACMP_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
} HAL_ACMP_StateTypeDef;
  
  
/**
 * @brief  COMP Init structure definition
 */

typedef struct
{
    uint32_t  CrvinCtrl;            /*!< Specifies the negative input resistor voltage-dividing input for the selected acmp.
                                        This parameter can be a value of @ref ACMP_crvinctrl_define */
    uint32_t  CrvCtrl;              /*!< Specifies the Negative input resistance voltage dividing ratio for the selected acmp.
                                        This parameter can be a value of @ref ACMP_crvctrl_define */
    uint32_t  CnegSel;              /*!< Specifies the Negative input channel for the selected acmp.
                                        This parameter can be a value of @ref ACMP_cnegsel_define */
    uint32_t  CposSel;              /*!< Specifies the Positive input channel for the selected acmp.
                                        This parameter can be a value of @ref ACMP_cpossel_define */
    uint32_t  ChySel;               /*!< Specifies the Hysteresis voltage for the selected acmp.
                                        This parameter can be a value of @ref ACMP_chysel_define */
    uint32_t  Clpm;                 /*!< Specifies the Low power consumption for the selected acmp.
                                        This parameter can be a value of @ref ACMP_clpm_define */
    uint32_t  Interrupt_Mode;       /*!< Specifies the interrupt mode for the selected acmp.    
                                       This parameter can be a value of @ref ACMP_interrupt_mode_define */

}ACMP_InitTypeDef;

/**
  * @brief  UART handle Structure definition
  */
typedef struct __ACMP_HandleTypeDef
{
    
    uint32_t                    Acmpx;            /*!< Specifies the selected acmp.
                                                        This parameter can be a value of @ref ACMP_acmpx_define */
    
    ACMP_InitTypeDef             Init;             /*!< ACMP communication parameters      */
   
    HAL_LockTypeDef              Lock;             /*!< ACMP locking object                       */

    __IO HAL_ACMP_StateTypeDef   gState;           /*!< ACMP state information related to global Handle management.
                                                       This parameter can be a value of @ref HAL_ACMP_StateTypeDef */
    
}ACMP_HandleTypeDef;
/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/

/** @defgroup ACMP_Exported_constants ACMP Exported Constants
  * @{
  */ 

/** @defgroup ACMP_acmpx_define ACMP acmpx define
  * @brif ACMP custom handle
  * @{
  */  
#define ACMP0        		                     ((uint32_t)0x00000000U)           
#define ACMP1         			                 ((uint32_t)0x00000001U)           
#define ACMP2       				             ((uint32_t)0x00000002U)           
/**
  * @}
  */

/** @defgroup ACMP_interrupt_mode_define ACMP interrupt mode define
   * @{
   */  
#define ACMP_ARBITRARY_EDGE       		         ((uint32_t)0x00010000U)           // arbit edge
#define ACMP_TRAILING_EDGE       			     ((uint32_t)0x00020000U)           // trailing edge
#define ACMP_RISING_EDGE      				     ((uint32_t)0x00030000U)           // rising edge
/**
  * @}
  */

/** @defgroup ACMP_cnegsel_define ACMP cnegsel define
   * @{
   */  
#define ACMP_CNEGSEL_CIN         		         ((uint32_t)0x00000000U)           // External pin input
#define ACMP_CNEGSEL_VDDA        			     ((uint32_t)0x00000040U)           // Internal vdda input
#define ACMP_CNEGSEL_VREF         				 ((uint32_t)0x00000040U)           // Vref input
#define ACMP_CNEGSEL_VBG                         ((uint32_t)0x00000080U)           // 0.6V internal signal input
#define ACMP_CNEGSEL_DACOUT                      ((uint32_t)0x000000C0U)           // Dac input
/**
  * @}
  */

/** @defgroup ACMP_cpossel_define ACMP cpossel define
   * @{
   */  
#define ACMP_CPOSSEL_CP0      		             ((uint32_t)0x00000000U)           
#define ACMP_CPOSSEL_CP1      			         ((uint32_t)0x00000010U)           
#define ACMP_CPOSSEL_CP2       				     ((uint32_t)0x00000020U)           
#define ACMP_CPOSSEL_CP3                         ((uint32_t)0x00000030U)           
/**
  * @}
  */

/** @defgroup ACMP_crvctrl_define ACMP crvctrl define
   * @{
   */  
#define ACMP_CRVCTRL_0      			        ((uint32_t)0x00000000U)          
#define ACMP_CRVCTRL_1         		            ((uint32_t)0x00000100U) 
#define ACMP_CRVCTRL_2      			        ((uint32_t)0x00000200U)          
#define ACMP_CRVCTRL_3         		            ((uint32_t)0x00000300U)   
#define ACMP_CRVCTRL_4      			        ((uint32_t)0x00000400U)          
#define ACMP_CRVCTRL_5         		            ((uint32_t)0x00000500U)   
#define ACMP_CRVCTRL_6      			        ((uint32_t)0x00000600U)          
#define ACMP_CRVCTRL_7         		            ((uint32_t)0x00000700U)   
#define ACMP_CRVCTRL_8      			        ((uint32_t)0x00000800U)          
#define ACMP_CRVCTRL_9         		            ((uint32_t)0x00000900U)   
#define ACMP_CRVCTRL_10      			        ((uint32_t)0x00000A00U)          
#define ACMP_CRVCTRL_11         		        ((uint32_t)0x00000B00U)   
#define ACMP_CRVCTRL_12      			        ((uint32_t)0x00000C00U)          
#define ACMP_CRVCTRL_13         		        ((uint32_t)0x00000D00U)   
#define ACMP_CRVCTRL_14      			        ((uint32_t)0x00000E00U)          
#define ACMP_CRVCTRL_15         		        ((uint32_t)0x00000F00U) 
/**
  * @}
  */

/** @defgroup ACMP_crvinctrl_define ACMP crvinctrl define
   * @{
   */  
#define ACMP_CRVINCTRL_VDDA       			    ((uint32_t)0x00000000U)          
#define ACMP_CRVINCTRL_VREF         		    ((uint32_t)0x00001000U)          
/**
  * @}
  */

/** @defgroup ACMP_chysel_define ACMP chysel define
   * @{
   */  
#define ACMP_CHYSEL_0      			            ((uint32_t)0x00000000U)          
#define ACMP_CHYSEL_10         		            ((uint32_t)0x00000004U)      
#define ACMP_CHYSEL_20      			        ((uint32_t)0x00000008U)          
#define ACMP_CHYSEL_30         		            ((uint32_t)0x0000000CU)  
/**
  * @}
  */

/** @defgroup ACMP_clpm_define ACMP clpm define
   * @{
   */  
#define ACMP_CLPM_CLOSE      			        ((uint32_t)0x00000000U)          
#define ACMP_CLPM_OPEN         		            ((uint32_t)0x00000002U)       
/**
  * @}
  */

/** @defgroup ACMP_en_define ACMP en define
   * @{
   */  
#define ACMP_EN_CLOSE      			            ((uint32_t)0x00000000U)          
#define ACMP_EN_OPEN         		            ((uint32_t)0x00000001U)       
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ACMP_Exported_macro ACMP Exported Macro
  * @{
  */ 
  
/** @brief  Enable the specified ACMP peripheral.
  * @param  __HANDLE__ specifies the ACMP custom Handle.
  * @retval None
  */
#define __HAL_ACMP_ENABLE(__HANDLE__)                  do{ \
                                                          HAL_ACMP_Unlock(); \
                                                          if((__HANDLE__) == ACMP0) \
                                                          { \
                                                               SET_BIT((ACMP)->CFG0, ACMP_CFG0_EN); \
                                                          }  \
                                                          if((__HANDLE__) == ACMP1) \
                                                          { \
                                                               SET_BIT((ACMP)->CFG1, ACMP_CFG1_EN); \
                                                          } \
                                                          if((__HANDLE__) == ACMP2) \
                                                          { \
                                                               SET_BIT((ACMP)->CFG2, ACMP_CFG2_EN); \
                                                          } \
                                                          HAL_ACMP_Lock(); \
                                                       }while(0U)

/** @brief  Disable the specified ACMP peripheral.
  * @param  __HANDLE__ specifies the ACMP  custom Handle.
  * @retval None
  */
#define __HAL_ACMP_DISABLE(__HANDLE__)                 do{ \
                                                          HAL_ACMP_Unlock(); \
                                                          if((__HANDLE__) == ACMP0) \
                                                          { \
                                                               CLEAR_BIT((ACMP)->CFG0, ACMP_CFG0_EN); \
                                                          }  \
                                                          if((__HANDLE__) == ACMP1) \
                                                          { \
                                                               CLEAR_BIT((ACMP)->CFG1, ACMP_CFG1_EN); \
                                                          } \
                                                          if((__HANDLE__) == ACMP2) \
                                                          { \
                                                               CLEAR_BIT((ACMP)->CFG2, ACMP_CFG2_EN); \
                                                          } \
                                                          HAL_ACMP_Lock(); \
                                                       }while(0U) 


/** @brief  Clears the ACMP pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the ACMP custom Handle.
  * @retval None
  */
#define __HAL_ACMP_CLEAR_IT_FLAG(__HANDLE__)           ((__HANDLE__ == ACMP0)? \
                                                       (ACMP->CFG0 |= (ACMP_CFG0_INTS) ):((__HANDLE__ ==ACMP1)? \
                                                       (ACMP->CFG1 |= ACMP_CFG1_INTS):(ACMP->CFG2 |= ACMP_CFG2_INTS)))
                                                      
                                            
/** @brief  Checks whether the specified ACMP interrupt flag is set or not.
  * @param  __HANDLE__ specifies the ACMP custom Handle.
  * @retval The new state of __HANDLE__ (SET or RESET).
  */
#define __HAL_ACMP_GET_IT_FLAG(__HANDLE__)            ((__HANDLE__ == ACMP0)? \
                                                      (ACMP->CFG0 & ACMP_CFG1_INTS):((__HANDLE__ ==ACMP1)? \
                                                      (ACMP->CFG1 & ACMP_CFG1_INTS):(ACMP->CFG2 & ACMP_CFG2_INTS)))    
                                                     
/**
  * @}
  */

                                                        
                                                        
                                                        
/* Exported functions --------------------------------------------------------*/
/** @addtogroup ACMP_Exported_Functions
  * @{
  */ 
/* Initialization and de-initialization functions******************************/
HAL_StatusTypeDef HAL_ACMP_Init(ACMP_HandleTypeDef *hacmp);
HAL_StatusTypeDef HAL_ACMP_DeInit(ACMP_HandleTypeDef *hacmp);        
void HAL_ACMP_MspInit(ACMP_HandleTypeDef *hacmp);
void HAL_ACMP_MspDeInit(ACMP_HandleTypeDef *hacmp);
uint8_t HAL_ACMP_GetOutput_Status(ACMP_HandleTypeDef *hacmp);
                                                       
/** @addtogroup ACMP_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  **********************************************/
HAL_StatusTypeDef HAL_ACMP_Unlock(void);
HAL_StatusTypeDef HAL_ACMP_Lock(void);
/* ACMP IRQ handler method */
void HAL_ACMP_IRQHandler(ACMP_HandleTypeDef *hacmp);
/* Callbacks in non blocking modes */
void HAL_ACMP_Callback(ACMP_HandleTypeDef *hacmp);

/* Private macros ------------------------------------------------------------*/
/** @defgroup ACMP_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup ACMP_Private_Functions ACMP Private Functions
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

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
