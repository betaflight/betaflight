 /**
  ******************************************************************************
  * @file     um324xx_hal_opa.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-20  
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
#ifndef __UM324XX_HAL_OPA_H__
#define __UM324XX_HAL_OPA_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup OPA OPA
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup OPA_Exported_typedefs OPA Exported Typedefs
  * @{
  */ 

/**
  * @brief  OPA Init structure definition
  */ 
typedef struct
{
    uint32_t  Compen;                /*!< Specifies the Comparator function for the selected opa.
                                            This parameter can be a value of @ref OPA_compen_define */
    uint32_t  GainSel;               /*!< Specifies the Gain multiple for the selected opa.
                                            This parameter can be a value of @ref OPA_gainsel_define */
    uint32_t  Selp;                  /*!< Specifies the Positive input channel for the selected opa.
                                            This parameter can be a value of @ref OPA_selp_define */
    uint32_t  Seln;                  /*!< Specifies the Negative input channel for the selected opa.
                                            This parameter can be a value of @ref OPA_seln_define */
    uint32_t  Otpen;                 /*!< Specifies the OPA output to IO pin for the selected opa.
                                            This parameter can be a value of @ref OPA_otpen_define */
    uint32_t  Capen;                 /*!< Specifies the PGA internal feedback capacitor for the selected opa.
                                            This parameter can be a value of @ref OPA_capen_define */
    uint32_t  Fbresen;               /*!< Specifies the PGA internal feedback resistor for the selected opa.
                                            This parameter can be a value of @ref OPA_fbresen_define */
    uint32_t  Interrupt_Mode;       /*!< Specifies the interrupt mode for the selected opa.    
                                            This parameter can be a value of @ref OPA_interrupt_mode_define */     
    
}OPA_InitTypeDef;

/** 
  * @brief  HAL Mode structures definition  
  */
typedef enum 
{
  HAL_OPA_MODE_UNITBUFF = 0x00U,
  HAL_OPA_MODE_PGA  = 0x01U,
  HAL_OPA_MODE_CMP  = 0x02U,    
  HAL_OPA_MODE_OPA  = 0x03U,     
} HAL_ModeTypeDef;

/**
  * @brief  OPA Handle Structure definition
  */
typedef struct
{
    OPA_InitTypeDef             Init;       /*!< OPA required parameters */
    
    HAL_LockTypeDef             Lock;       /*!< OPA locking object  */
    
    HAL_ModeTypeDef             Opa_Mode;   /*!< OPA mode object  */
    
    uint32_t                    Opax;       /*!< Specifies the selected opa.
                                            This parameter can be a value of @ref OPA_opax_define */    
   
}OPA_HandleTypeDef;
  
/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup OPA_Exported_constants OPA Exported Constants
  * @{
  */ 
/** @defgroup OPA_opax_define OPA opax define
  * @brif OPA custom handle
  * @{
  */  
#define OPA0        		                     ((uint32_t)0x00000000U)           
#define OPA1         			                 ((uint32_t)0x00000001U)           
#define OPA2         			                 ((uint32_t)0x00000002U)       
/**
  * @}
  */

/** @defgroup OPA_interrupt_mode_define OPA interrupt mode define
   * @{
   */  
#define OPA_INTERRUPT_CLOSE       		         ((uint32_t)0x00000000U)           // opa interrupt close  
#define OPA_ARBITRARY_EDGE       		         ((uint32_t)0x00010000U)           // arbit edge
#define OPA_TRAILING_EDGE       			     ((uint32_t)0x00020000U)           // trailing edge
#define OPA_RISING_EDGE      				     ((uint32_t)0x00030000U)           // rising edge

/**
  * @}
  */

/** @defgroup OPA_compen_define OPA compen define
   * @{
   */  
#define OPA_COMPEN_CLOSE       		             ((uint32_t)0x00000000U)           
#define OPA_COMPEN_OPEN      			         ((uint32_t)0x00002000U)           

/**
  * @}
  */

/** @defgroup OPA_gainsel_define OPA gainsel define
   * @{
   */  
#define OPA_GAINSEL_1X       		             ((uint32_t)0x00000000U)           
#define OPA_GAINSEL_2X      			         ((uint32_t)0x00000400U)           
#define OPA_GAINSEL_4X       		             ((uint32_t)0x00000800U)           
#define OPA_GAINSEL_8X     			             ((uint32_t)0x00000C00U)           
#define OPA_GAINSEL_16X       		             ((uint32_t)0x00001000U)           
#define OPA_GAINSEL_32X      			         ((uint32_t)0x00001400U)           
#define OPA_GAINSEL_64X       		             ((uint32_t)0x00001800U)           

/**
  * @}
  */

/** @defgroup OPA_selp_define OPA selp define
   * @{
   */  
#define OPA_SELP_VIN       		             ((uint32_t)0x00000000U)           
#define OPA_SELP_CORE      			         ((uint32_t)0x00000180U)           
#define OPA_SELP_ADCMUX       		         ((uint32_t)0x00000200U)           

/**
  * @}
  */

/** @defgroup OPA_seln_define OPA seln define
   * @{
   */  
#define OPA_SELN_VIN       		             ((uint32_t)0x00000000U)           
#define OPA_SELN_GROUND      			     ((uint32_t)0x00000010U)           
#define OPA_SELN_RESERVE      			     ((uint32_t)0x00000070U)           

/**
  * @}
  */

/** @defgroup OPA_otpen_define OPA otpen define
   * @{
   */  
#define OPA_OTPEN_CLOSE       		         ((uint32_t)0x00000000U)           
#define OPA_OTPEN_CONNECT_PIN      			 ((uint32_t)0x00000008U)           

/**
  * @}
  */

/** @defgroup OPA_capen_define OPA capen define
   * @{
   */  
#define OPA_CAPEN_CLOSE       		        ((uint32_t)0x00000000U)           
#define OPA_CAPEN_OPEN      			    ((uint32_t)0x00000004U)           

/**
  * @}
  */

/** @defgroup OPA_fbresen_define OPA fbresen define
   * @{
   */  
#define OPA_FBRESEN_CLOSE       		    ((uint32_t)0x00000000U)           
#define OPA_FBRESEN_OPEN      			    ((uint32_t)0x00000002U)           

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup OPA_Exported_macro OPA Exported Macro
  * @{
  */ 

/** @brief  Enable the specified OPA peripheral.
  * @param  __HANDLE__ specifies the OPA custom Handle.
  * @retval None
  */
#define __HAL_OPA_ENABLE(__HANDLE__)                  do{ \
                                                          HAL_OPA_Unlock(); \
                                                          if((__HANDLE__) == OPA0) \
                                                          { \
                                                               SET_BIT((OPA)->CFG0, OPA_CFG0_EN); \
                                                          }  \
                                                          if((__HANDLE__) == OPA1) \
                                                          { \
                                                               SET_BIT((OPA)->CFG1, OPA_CFG1_EN); \
                                                          } \
                                                          HAL_OPA_Lock(); \
                                                       }while(0U)


/** @brief  Disable the specified OPA peripheral.
  * @param  __HANDLE__ specifies the OPA custom Handle.
  * @retval None
  */
#define __HAL_OPA_DISABLE(__HANDLE__)                 do{ \
                                                          HAL_OPA_Unlock(); \
                                                          if((__HANDLE__) == OPA0) \
                                                          { \
                                                               CLEAR_BIT((OPA)->CFG0, OPA_CFG0_EN); \
                                                          }  \
                                                          if((__HANDLE__) == OPA1) \
                                                          { \
                                                               CLEAR_BIT((OPA)->CFG1, OPA_CFG1_EN); \
                                                          } \
                                                          HAL_OPA_Lock(); \
                                                       }while(0U) 


/** @brief  Clears the OPA pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the OPA custom Handle.
  * @retval None
  */
#define __HAL_OPA_CLEAR_IT_FLAG(__HANDLE__)           ((__HANDLE__ == OPA0)? \
                                                       (OPA->CFG0 |= (OPA_CFG0_INT) ):((__HANDLE__ ==OPA1)? \
                                                       (OPA->CFG1 |= OPA_CFG1_INT):(0U)))
                                                      
                                            
/** @brief  Checks whether the specified OPA interrupt flag is set or not.
  * @param  __HANDLE__ specifies the OPA custom Handle.
  * @retval The new state of __HANDLE__ (SET or RESET).
  */
#define __HAL_OPA_GET_IT_FLAG(__HANDLE__)            ((__HANDLE__ == OPA0)? \
                                                      (OPA->CFG0 & OPA_CFG0_INT):((__HANDLE__ ==OPA1)? \
                                                      (OPA->CFG1 & OPA_CFG1_INT):(0U)))    
                                                     
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup OPA_Exported_Functions    OPA Exported Functions
  * @{
  */ 
/* Initialization and de-initialization functions******************************/
HAL_StatusTypeDef HAL_OPA_Init(OPA_HandleTypeDef *hopa);
HAL_StatusTypeDef HAL_OPA_DeInit(OPA_HandleTypeDef *hopa);        
void HAL_OPA_MspInit(OPA_HandleTypeDef *hopa);
void HAL_OPA_MspDeInit(OPA_HandleTypeDef *hopa);
_Bool HAL_OPA_GetOutput_Status(OPA_HandleTypeDef *hopa);
   
/** @addtogroup OPA_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  **********************************************/
HAL_StatusTypeDef HAL_OPA_Unlock(void);
HAL_StatusTypeDef HAL_OPA_Lock(void);
/* OPA IRQ handler method */
void HAL_OPA_IRQHandler(OPA_HandleTypeDef *hopa);
/* Callbacks in non blocking modes */
void HAL_OPA_Callback(OPA_HandleTypeDef *hopa);

/**
  * @}
  */
  
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup OPA_Private_Macros OPA Private Macros
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup OPA_Private_Functions OPA Private Functions
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
