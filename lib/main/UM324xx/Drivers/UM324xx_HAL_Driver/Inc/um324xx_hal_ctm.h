 /**
  ******************************************************************************
  * @file     um324xx_hal_ctm.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-11
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
#ifndef __UM324XX_HAL_CTM_H__
#define __UM324XX_HAL_CTM_H__


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

typedef struct
{
    uint32_t Digital_Filter;        /*!< Specifies the digital filter function selection.
                                    This parameter can be any value of @ref DFS_define */     
    
    uint32_t Ref_ClkDiv;            /*!< Specifies the measurement reference clock division selection.
                                    This parameter can be any value of @ref REF_CLKDIV_define */ 
    
    uint32_t Ref_ClkSel;            /*!< Specifies measurement reference clock selection.
                                    This parameter can be any value of @ref REF_CLKSEL_define */ 
    
    uint32_t Ref_SignalSourceSel;  /*!< Specifies the reference signal source selection.
                                    This parameter can be any value of @ref REF_SignalSource_define */ 
    
    uint32_t REF_SignalEdgeSel;    /*!< Specifies the reference signal edge selection.
                                    This parameter can be any value of @ref REF_SignalEdge_define */ 
   
    uint32_t Object_ClkDiv;         /*!< Specifies the measurement object clock div selection.
                                    This parameter can be any value of @ref ObjectClock_Div_define */  
    
    uint32_t Object_ClkSel;        /*!< Specifies the measurement object clock selection.
                                    This parameter can be any value of @ref ObjectClock_define */ 
    
    uint16_t UpLimit_Value;         /*!< Set the upper limit value for counting.
                                    The range of this parameter is 0x0~0xFFFF */
                                    
    uint16_t LowLimit_Value;        /*!< Set the low limit value for counting.
                                    The range of this parameter is 0x0~0xFFFF */

    uint16_t Irq;                   /*!< Specifies the irq selection.
                                    This parameter can be any value of @ref CTM_IRQ_define */ 

} CTM_InitTypeDef;


typedef struct __CTM_HandleTypeDef
{
    CTM_TypeDef                 *Instance;         /*!< CTM registers base address        */

    CTM_InitTypeDef              Init;             /*!< CTM communication parameters      */


#if (USE_HAL_CTM_REGISTER_CALLBACKS == 1)

    void (* MspInitCallback)(struct __CTM_HandleTypeDef *hctm);                 /*!< CTM Msp Init callback                  */
    void (* MspDeInitCallback)(struct __CTM_HandleTypeDef *hctm);               /*!< CTM Msp DeInit callback                */ 
    
    void (* MeasureCpltCallback)(struct __CTM_HandleTypeDef *hctm);             /*!< CTM Measurement completed callback     */

    void (* HAL_CTM_OverFlowCallback)(struct __CTM_HandleTypeDef *hctm);        /*!< CTM overflow callback     */
    void (* HAL_CTM_FrqAbnormalCallback)(struct __CTM_HandleTypeDef *hctm);     /*!< CTM abnormal frequency callback     */
    
    
    
#endif  /* USE_HAL_CTM_REGISTER_CALLBACKS */

} CTM_HandleTypeDef;








/** @defgroup DFS_define CTM DFS define
   * @{
   */ 
#define CTM_DFS_DISABLE                 ((uint32_t)0x00000000U)
#define CTM_DFS_OBJECT                  ((uint32_t)0x00004000U)
#define CTM_DFS_OBJECT_DIV4             ((uint32_t)0x00008000U)
#define CTM_DFS_OBJECT_DIV16            ((uint32_t)0x0000C000U)

/**
  * @}
  */


/** @defgroup REF_CLKDIV_define CTM REF CLKDIV define
   * @{
   */ 
#define CTM_RCDS_DIV32                  ((uint32_t)0x00000000U)
#define CTM_RCDS_DIV128                 ((uint32_t)0x00001000U)
#define CTM_RCDS_DIV1024                ((uint32_t)0x00002000U)
#define CTM_RCDS_DIV8192                ((uint32_t)0x00003000U)

/**
  * @}
  */

/** @defgroup REF_CLKSEL_define CTM REF CLKSEL define
   * @{
   */ 
#define CTM_RSCS_RCH                    ((uint32_t)0x00000000U)
#define CTM_RSCS_RCL                    ((uint32_t)0x00000200U)     /*无效*/
#define CTM_RSCS_XTH                    ((uint32_t)0x00000400U)
#define CTM_RSCS_XTL                    ((uint32_t)0x00000600U)
#define CTM_RSCS_PLL                    ((uint32_t)0x00000800U)     /*无效*/
#define CTM_RSCS_HCLK                   ((uint32_t)0x00000A00U)



/**
  * @}
  */


/** @defgroup REF_SignalSource_define CTM REF SignalSource define
   * @{
   */ 
#define CTM_RPS_EXTPININPUT             ((uint32_t)0x00000000U)
#define CTM_RPS_INTINPUT                ((uint32_t)0x00000100U)



/**
  * @}
  */


/** @defgroup REF_SignalEdge_define CTM REF SignalEdge define
   * @{
   */ 
#define CTM_REF_RISINGEDGE              ((uint32_t)0x00000000U)
#define CTM_REF_FALLINGEDGE             ((uint32_t)0x00000040U)
#define CTM_REF_FALLINANDFALLING        ((uint32_t)0x00000080U)


/**
  * @}
  */


/** @defgroup ObjectClock_Div_define CTM ObjectClock Div define
   * @{
   */ 
#define CTM_OBJECTCLK_DIV1               ((uint32_t)0x00000000U)
#define CTM_OBJECTCLK_DIV4               ((uint32_t)0x00000010U)
#define CTM_OBJECTCLK_DIV8               ((uint32_t)0x00000020U)
#define CTM_OBJECTCLK_DIV32              ((uint32_t)0x00000030U)

/**
  * @}
  */



/** @defgroup ObjectClock_define CTM ObjectClock define
   * @{
   */ 
#define CTM_OBJECTCLK_RCH               ((uint32_t)0x00000000U)
#define CTM_OBJECTCLK_RCL               ((uint32_t)0x00000002U)     /*无效*/
#define CTM_OBJECTCLK_XTH               ((uint32_t)0x00000004U)
#define CTM_OBJECTCLK_XTL               ((uint32_t)0x00000006U)
#define CTM_OBJECTCLK_PLL               ((uint32_t)0x00000008U)     /*无效*/
#define CTM_OBJECTCLK_HCLK              ((uint32_t)0x0000000AU)

/**
  * @}
  */


/** @defgroup External_Pin_input_define CTM External pin input define
   * @{
   */ 
#define CTM_REFE_INPUT_DISABLE          ((uint32_t)0x00000000U)
#define CTM_REFE_INPUT_ENABLE           ((uint32_t)0x00000001U)


/**
  * @}
  */


/** @defgroup CTM_IRQ_define CTM ISR define
   * @{
   */ 
#define CTM_IRQ_NONE                    ((uint32_t)0x00000000U)   
#define CTM_IRQ_FERRIE                  ((uint32_t)0x00000001U)
#define CTM_IRQ_MEDNIE                  ((uint32_t)0x00000002U)
#define CTM_IRQ_OVFIE                   ((uint32_t)0x00000004U)

/**
  * @}
  */
  
  
/** @defgroup CTM_ISR_define CTM ISR define
   * @{
   */ 
#define CTM_ISR_NONE                    ((uint32_t)0x00000000U)   
#define CTM_ISR_FERRF                   ((uint32_t)0x00000001U)
#define CTM_ISR_MENDF                   ((uint32_t)0x00000002U)
#define CTM_ISR_OVFF                    ((uint32_t)0x00000004U)
#define CTM_ISR_OVRF                    ((uint32_t)0x00000008U)
#define CTM_ISR_MSK                     ((uint32_t)(CTM_ISR_OVRF|CTM_ISR_OVFF|CTM_ISR_MENDF|CTM_ISR_FERRF))
/**
  * @}
  */


#define __HAL_CTM_FRQ_MEASUREMENT_ENABLE(__HANDLE__)      SET_BIT((__HANDLE__)->Instance->CTRL0 ,1<<0)

#define __HAL_CTM_FRQ_MEASUREMENT_DISABLE(__HANDLE__)     CLEAR_BIT((__HANDLE__)->Instance->CTRL0 ,1<<0)

#define __HAL_CTM_GET_CNT(__HANDLE__)                     READ_REG((__HANDLE__)->Instance->CNTBR)



/** @brief  Checks whether the specified CTM flag is set or not.
  * @param  __HANDLE__ specifies the CTM Handle.
  *         CTM Handle selects the  CTM peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg CTM_ISR_FERRF:  Frequency anomaly flag
  *            @arg CTM_ISR_MENDF:  Frequency measurement completion flag
  *            @arg CTM_ISR_OVFF:   Frequency monitoring counter overflow flag
  *            @arg CTM_ISR_OVRF:   Frequency monitoring count rewrite flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CTM_GET_IRQ_FLAG(__HANDLE__,__FLAG__)       (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))



/** @brief  Clear whether the specified CTM flag is set or not.
  * @param  __HANDLE__ specifies the CTM Handle.
  *         CTM Handle selects the  CTM peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg CTM_ISR_FERRF:  Frequency anomaly flag
  *            @arg CTM_ISR_MENDF:  Frequency measurement completion flag
  *            @arg CTM_ISR_OVFF:   Frequency monitoring counter overflow flag
  *            @arg CTM_ISR_OVRF:   Frequency monitoring count rewrite flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CTM_CLEAR_IRQ_FLAG(__HANDLE__,__FLAG__)        CLEAR_BIT((__HANDLE__)->Instance->STCLR ,(__FLAG__))


/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup CTM_Private_Constants CTM Private Constants
  * @{
  */
HAL_StatusTypeDef HAL_CTM_Init(CTM_HandleTypeDef *hctm);
void HAL_CTM_Start(CTM_HandleTypeDef *hctm);
void HAL_CTM_Stop(CTM_HandleTypeDef *hctm);


void HAL_CTM_MspInit(CTM_HandleTypeDef *hctm);
void HAL_CTM_MspDeInit(CTM_HandleTypeDef *hctm);

void HAL_CTM_IRQHandler(CTM_HandleTypeDef *hctm);

void HAL_CTM_MeasureCpltCallback(CTM_HandleTypeDef *hctm);
void HAL_CTM_OverFlowCallback(CTM_HandleTypeDef *hctm);
void HAL_CTM_FrqAbnormalCallback(CTM_HandleTypeDef *hctm);


/**
  * @}
  */





#ifdef __cplusplus
}
#endif

#endif /* __UM32x41x_HAL_CTM_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
