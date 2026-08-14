/**
  ******************************************************************************
  * @file     um324xx_hal_pmu.c 
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

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Examples
  * @{
  */

/** @defgroup PMU_functions
  * @brief PMU HAL module driver
  * @{
  */

#ifdef HAL_PMU_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_PMU_Private_Functions
  * @{
  */

/**
  * @brief PMU LVD Init.
  * @param hlvd: Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PMU_LVD_Init(PMU_HandleTypeDef *hlvd)
{
    /* Check the UART handle allocation */
    if (hlvd == NULL)
    {
        return HAL_ERROR;
    }
    
#if (USE_HAL_PMU_REGISTER_CALLBACKS == 1)

    if (hlvd->MspInitCallback == NULL)
    {
      hlvd->MspInitCallback = HAL_PMU_LVD_MspInit;
    }

    /* Init the low level hardware */
    hlvd->MspInitCallback(hlvd);
#else
    /* Init INT*/
    HAL_PMU_LVD_MspInit(hlvd);
#endif /* (USE_HAL_PMU_REGISTER_CALLBACKS) */
    
    __HAL_PMU_UNLOCK_REGISTER();  
    
    if(hlvd->Init.En != PMU_LVD_DISABLE)
    {

        MODIFY_REG(PMU->VDCR,(PMU_VDCR_LVD_FILTER_EN_Msk | PMU_VDCR_LVDS_Msk | PMU_VDCR_LVD_RST_EN_Msk),\
        (hlvd->Init.FilterEnable | hlvd->Init.ResetEn | hlvd->Init.VDTS));  
        
        if(hlvd->Init.IrqEn == PMU_LVD_INT_ENABLE)
        {
            SET_BIT(PMU->VDCR ,PMU_LVD_INT_ENABLE); 

            SET_BIT(SYSCFG->MISCCR,SYSCFG_MISCCR_LVD_INT_EN);
        }
        else
        {
            CLEAR_BIT(PMU->VDCR ,PMU_LVD_INT_ENABLE); 
            
            CLEAR_BIT(SYSCFG->MISCCR,SYSCFG_MISCCR_LVD_INT_EN); 
        }
        __HAL_PMU_LVD_ENABLE();
    }
    else
    {
        __HAL_PMU_LVD_DISABLE();
    }
    
    __HAL_PMU_LOCK_REGISTER();
  
    
    return HAL_OK;
    
}

/**
  * @brief Enters Sleep mode.
  * @param SLEEPEntry Specifies if SLEEP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PMU_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
  *            @arg PMU_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
  * @retval None
  */
void HAL_PMU_EnterSLEEPMode(uint8_t SLEEPEntry)
{
    /* Clear SLEEPDEEP bit of Cortex System Control Register */
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select SLEEP mode entry -------------------------------------------------*/
  if(SLEEPEntry == PMU_SLEEPENTRY_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE();
    __WFE();
  }
}

/**
  * @brief Enters Stop mode.
  * @param STOPEntry Specifies if Stop mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PMU_STOPENTRY_WFI: Enter Stop mode with WFI instruction
  *            @arg PMU_STOPENTRY_WFE: Enter Stop mode with WFE instruction
  * @retval None
  */
void HAL_PMU_EnterSTOPMode(uint8_t STOPEntry)
{
    __HAL_PMU_CONFIG_STOPMODE();

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));	

  /* Select Stop mode entry --------------------------------------------------*/
  if(STOPEntry == PMU_STOPENTRY_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE();
    __WFE();
  }
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));  
}

/**
  * @brief Enters Standby0 mode.
  * @retval None
  */
void HAL_PMU_EnterSTANDBY0Mode(void)
{
    __HAL_PMU_CONFIG_STANDBY0();

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));	

    /* Request Wait For Interrupt */
    __WFI();
}

/**
  * @brief Enters Standby1 mode.
  * @retval None
  */
void HAL_PMU_EnterSTANDBY1Mode(void)
{
    __HAL_PMU_CONFIG_STANDBY1();

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
    /* Request Wait For Interrupt */
    __WFI();
}

/**
  * @brief Enters DeepStandby0 mode.
  * @retval None
  */
void HAL_PMU_EnterDEEPSTANDBY0Mode(void)
{
    __HAL_PMU_CONFIG_DEEPSTANDBY0();

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

    /* Request Wait For Interrupt */
    __WFI();
}

/**
  * @brief Enters DeepStandby1 mode.
  * @retval None
  */
void HAL_PMU_EnterDEEPSTANDBY1Mode(void)
{
    __HAL_PMU_CONFIG_DEEPSTANDBY1();

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
    
    /* Request Wait For Interrupt */
    __WFI();
}


/**
  * @brief Configure wake up IO in standby mode.
  * @param WK_IO Select the four IO types to wake up in standby mode
  *          This parameter can be one of the following values:
  *            @arg PMU_STANDBY_WK_PC13: In standby mode,PC13 wakes up
  *            @arg PMU_STANDBY_WK_PC3: In standby mode,PC3 wakes up
  *            @arg PMU_STANDBY_WK_PC2: In standby mode,PC2 wakes up
  *            @arg PMU_STANDBY_WK_PC0: In standby mode,PC0 wakes up
  *            @arg PMU_STANDBY_WK_PA2: In standby mode,PA2 wakes up
  *            @arg PMU_STANDBY_WK_PA0: In standby mode,PA0 wakes up 
  * @param Trigger Select rising or falling edge trigger
  *         This parameter can be one of the following values:
  *            @arg RISINGEDGE: rising edge trigger
  *            @arg FALLINGEDGE: falling edge trigger
  * @Note The specific IO(PC13/PC3/PC2/PC0/PA2/PA0) wake supports four standby modes(standby0/standby1/deepstandby0/deepstandby1)
  * @retval None
  */
void HAL_PMU_Enable_StandbyMode_WakeUpPin(uint32_t WakeUpPinx, uint8_t WakeUpPolarity)
{
    __IO uint32_t pinx_pos;	
    
    /* Unlock PMU protection register */
    __HAL_PMU_UNLOCK_REGISTER();
    
    SET_BIT(PMU->PDWKCR,WakeUpPinx);
    
    
    	/* offset */
	if(WakeUpPinx == PMU_STANDBY_WK_PA0)
		pinx_pos = (WakeUpPinx << 15);
	else
		pinx_pos = (WakeUpPinx << 9);	
    
     
    if(WakeUpPolarity == PMU_WAKEUP_POLARITY_UP)
        {
           /* Set the wake up polarity up */
            CLEAR_BIT(PMU->PDWKCR,(pinx_pos));
        }
    else
         {
            /* Set the wake up polarity down */
            SET_BIT(PMU->PDWKCR,(pinx_pos));            
        }     
    
    /* Lock PMU protection register */
    __HAL_PMU_LOCK_REGISTER();
}

/**
  * @brief Turn off wake-up IO in standby mode.
  * @param WK_IO Select the four IO types to wake up in standby mode
  *          This parameter can be one of the following values:
  *            @arg PMU_STANDBY_WK_PC13: In standby mode,PC13 wakes up
  *            @arg PMU_STANDBY_WK_PC3: In standby mode,PC3 wakes up
  *            @arg PMU_STANDBY_WK_PC2: In standby mode,PC2 wakes up
  *            @arg PMU_STANDBY_WK_PC0: In standby mode,PC0 wakes up
  *            @arg PMU_STANDBY_WK_PA2: In standby mode,PA2 wakes up
  *            @arg PMU_STANDBY_WK_PA0: In standby mode,PA0 wakes up 
  * @retval None 
  */
void HAL_PMU_Disable_StandbyMode_WakeUpPin(uint32_t WakeUpPinx)
{
    /* Unlock PMU protection register */
    __HAL_PMU_UNLOCK_REGISTER();
    
    /* Disable the wake up pin */
    CLEAR_BIT(PMU->PDWKCR,WakeUpPinx);
   
    /* Lock PMU protection register */
    __HAL_PMU_LOCK_REGISTER();
}

/**
  * @brief Configure wake up event in standby mode.
  * @param WK_EVENT Select the four IO types to wake up in standby mode
  *          This parameter can be one of the following values:
  *            @arg PMU_WKE_LPUART1: In standby0 mode,LPUART1 wakes up
  *            @arg PMU_WKE_LPTIM1: In standby0 mode,LPTIM1 wakes up
  *            @arg PMU_WKE_LPTIM0: In standby0 mode,LPTIM0 wakes up
  *            @arg PMU_WKE_LPUART0: In standby0 mode,LPUART0 wakes up
  *            @arg PMU_WKE_IWDT: In standby0 mode,IWDT wakes up
  *            @arg PMU_WKE_RTC_TAMP: In standby0/standby1 mode,RTC TAMP wakes up 
  *            @arg PMU_WKE_RTC_ALARM: In standby mode,RTC ALARM wakes up 
  *            @arg PMU_WKE_RSTN: In lowpower mode,RSTN wakes up 
  * @retval None 
  */
void HAL_PMU_Enable_StandbyMode_WakeUpEvent(uint32_t WakeUpSource)
{
     /* Unlock PMU protection register */
    __HAL_PMU_UNLOCK_REGISTER();
    
    /* Enable the wake up source */
    SET_BIT(PMU->PDWKCR,WakeUpSource);   
    
    /* Lock PMU protection register */
    __HAL_PMU_LOCK_REGISTER();
}

/**
  * @brief Turn off wake-up event in standby mode.
  * @param WK_EVENT Select the four IO types to wake up in standby mode
  *          This parameter can be one of the following values:
  *            @arg PMU_WKE_LPUART1: In standby0 mode,LPUART1 wakes up
  *            @arg PMU_WKE_LPTIM1: In standby0 mode,LPTIM1 wakes up
  *            @arg PMU_WKE_LPTIM0: In standby0 mode,LPTIM0 wakes up
  *            @arg PMU_WKE_LPUART0: In standby0 mode,LPUART0 wakes up
  *            @arg PMU_WKE_IWDT: In standby0 mode,IWDT wakes up
  *            @arg PMU_WKE_RTC_TAMP: In standby0/standby1 mode,RTC TAMP wakes up 
  *            @arg PMU_WKE_RTC_ALARM: In standby mode,RTC ALARM wakes up 
  *            @arg PMU_WKE_RSTN: In lowpower mode,RSTN wakes up 
  * @retval None 
  */
void HAL_PMU_Disable_StandbyMode_WakeUpEvent(uint32_t WakeUpSource)
{
    /* Unlock PMU protection register */
    __HAL_PMU_UNLOCK_REGISTER();
    
    /* Disable the wake up source */
    CLEAR_BIT(PMU->PDWKCR,WakeUpSource);   
    
    /* Lock PMU protection register */
    __HAL_PMU_LOCK_REGISTER();
}

/**
  * @brief Enables the Wake-up EXTIx functionality of EXTI Stop Mode.
  * @param WakeUpEXTIx Specifies the Wake-Up EXTI to enable.
  *         This parameter can be one of the following values:
  *           @arg PMU_WAKEUP_EXTI0
  *           @arg PMU_WAKEUP_EXTI1
  *           @arg PMU_WAKEUP_EXTI2
  *           @arg PMU_WAKEUP_EXTI3
  *           @arg PMU_WAKEUP_EXTI4
  *           @arg PMU_WAKEUP_EXTI5
  *           @arg PMU_WAKEUP_EXTI6
  *           @arg PMU_WAKEUP_EXTI7
  *           @arg PMU_WAKEUP_EXTI8
  *           @arg PMU_WAKEUP_EXTI9
  *           @arg PMU_WAKEUP_EXTI10
  *           @arg PMU_WAKEUP_EXTI11
  *           @arg PMU_WAKEUP_EXTI12
  *           @arg PMU_WAKEUP_EXTI13
  *           @arg PMU_WAKEUP_EXTI14
  *           @arg PMU_WAKEUP_EXTI15
  * @param WakeUpPolarity Specifies the Wake-Up polarity to enable.
  *         This parameter can be one of the following values:
  *           @arg PMU_WAKEUP_POLARITY_UP
  *           @arg PMU_WAKEUP_POLARITY_DOWN
  * @retval None
  */
void HAL_PMU_EXTIStopMode_EnableWakeUpEXTI(uint32_t WakeUpEXTIx, uint8_t WakeUpPolarity)
{
	__IO uint32_t regx;		
	
	/* Check the EXTIx */
	regx = (WakeUpEXTIx >> 2);
	
	/* Enable the wake up EXTIx */
	SET_BIT(SYSCFG->EXTICR[regx], 0x1UL << (WakeUpEXTIx + ((4-regx)<<2)) );
	
	if(WakeUpPolarity == PMU_WAKEUP_POLARITY_UP)
		/* Set the wake up polarity up */
		CLEAR_BIT(SYSCFG->EXTICR[regx], 0x1UL << (WakeUpEXTIx + ((5-regx)<<2)) );
	else
		/* Set the wake up polarity down */
		SET_BIT(SYSCFG->EXTICR[regx], 0x1UL << (WakeUpEXTIx + ((5-regx)<<2)) );			
}

/**
  * @brief Disables the Wake-up EXTIx functionality of EXTI Stop Mode.
  * @param WakeUpEXTIx Specifies the Wake-Up EXTI to disable.
  *         This parameter can be one of the following values:
  *           @arg PMU_WAKEUP_EXTI0
  *           @arg PMU_WAKEUP_EXTI1
  *           @arg PMU_WAKEUP_EXTI2
  *           @arg PMU_WAKEUP_EXTI3
  *           @arg PMU_WAKEUP_EXTI4
  *           @arg PMU_WAKEUP_EXTI5
  *           @arg PMU_WAKEUP_EXTI6
  *           @arg PMU_WAKEUP_EXTI7
  *           @arg PMU_WAKEUP_EXTI8
  *           @arg PMU_WAKEUP_EXTI9
  *           @arg PMU_WAKEUP_EXTI10
  *           @arg PMU_WAKEUP_EXTI11
  *           @arg PMU_WAKEUP_EXTI12
  *           @arg PMU_WAKEUP_EXTI13
  *           @arg PMU_WAKEUP_EXTI14
  *           @arg PMU_WAKEUP_EXTI15
  * @retval None
  */
void HAL_PMU_EXTIStopMode_DisableWakeUpEXTI(uint32_t WakeUpEXTIx)
{
	__IO uint32_t regx;		
	
	/* Check the EXTIx */
	regx = (WakeUpEXTIx >> 2);
	
	/* Disable the wake up EXTIx */
	CLEAR_BIT(SYSCFG->EXTICR[regx], 0x1UL << (WakeUpEXTIx + ((4-regx)<<2)) );
}

/**
  * @brief Enables CORTEX M4 SEVONPEND bit. 
  * @note Sets SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.
  * @retval None
  */
void HAL_PWR_EnableSEVOnPend(void)
{
  /* Set SEVONPEND bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @brief Disables CORTEX M4 SEVONPEND bit. 
  * @note Clears SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.         
  * @retval None
  */
void HAL_PWR_DisableSEVOnPend(void)
{
  /* Clear SEVONPEND bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @brief  Reference interrupt function
  * @param  hlvd Pointer to the LVD_HandleTypeDef structure that contains 
  *				 configuration information for the specified LVD module
  * @retval None
  */
void HAL_LVD_IRQHandler(PMU_HandleTypeDef *hlvd)
{
    if(__HAL_PWR_GET_LVD_INT_FLAG() != 0U)
    {
#if (USE_HAL_PMU_REGISTER_CALLBACKS == 1)
            /* Call registered LVD callback*/
            hlvd->LVD_Callback(hlvd);
#else
            /* Call legacy weak error callback*/
            HAL_LVD_Callback(hlvd);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */     
   }    

    /* Clear LVD interrupt flag*/
    __HAL_PWR_CLEAR_LVD_INT_FLAG();
   
}

/**
  * @brief  PMU LVD MSP Init.
  * @param  hlvd  Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
__weak void HAL_PMU_LVD_MspInit(PMU_HandleTypeDef *hlvd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hlvd);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_PMU_LVD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  PMU LVD MSP DeInit.
  * @param  hlvd  Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
__weak void HAL_PMU_LVD_MspDeInit(PMU_HandleTypeDef *hlvd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hlvd);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_PMU_LVD_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  LVD callbacks.
  * @param  hlvd  Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
__weak void HAL_LVD_Callback(PMU_HandleTypeDef *hlvd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hlvd);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_PMU_LVD_Callback could be implemented in the user file
   */
}




/**
  * @}
  */

#endif /* HAL_PMU_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
  
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
