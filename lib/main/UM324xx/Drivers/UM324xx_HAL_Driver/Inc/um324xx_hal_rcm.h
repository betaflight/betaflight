 #if defined(UM324xF)
/**
  ******************************************************************************
  * @file     um324xx_hal_rcm.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-03-21  
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
#ifndef __UM324XX_HAL_RCM_H__
#define __UM324XX_HAL_RCM_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup RCM
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup RCMEx_Exported_Types RCMEx Exported Typedefs
  * @{
  */ 
/**
  * @brief  RCM PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCM_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCM_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCM_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 16 and Max_Data = 1024 */

  uint32_t PLLP;       /*!< PLLP                                                                      */

}RCM_PLLInitTypeDef;
/**
  * @}
  */ 


/** @defgroup xxx_Exported_typedefs xxx Exported Typedefs
  * @{
  */ 
/**
  * @brief  RCM Internal/External Oscillator (XTH, RCH, XTL and RCL) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCM_Oscillator_Type                   */

  uint32_t XTHState;             /*!< The new state of the XTH.
                                      This parameter can be a value of @ref RCM_XTH_Config                        */

  uint32_t XTLState;             /*!< The new state of the XTL.
                                      This parameter can be a value of @ref RCM_XTL_Config                        */

  uint32_t RCHState;             /*!< The new state of the RCH.
                                      This parameter can be a value of @ref RCM_RCH_Config                        */

  uint32_t RCLState;             /*!< The new state of the RCL.
                                      This parameter can be a value of @ref RCM_RCL_Config                        */

  RCM_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCM_OscInitTypeDef;


/**
  * @brief  RCM System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCM_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCM_System_Clock_Source    */

  uint32_t RCHDivider;            /*!< The clock RCH divider.
                                       This parameter can be a value of @ref RCM_RCH_Clock_Source    */	
	
  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCM_AHB_Clock_Source       */

  uint32_t APB0CLKDivider;        /*!< The APB0 clock (PCLK0) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB0_Clock_Source */
    
  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB1_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB2_Clock_Source */
    
  uint32_t APB3CLKDivider;        /*!< The APB3 clock (PCLK3) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB3_Clock_Source */
									   
  uint32_t USBSDIOCLKDivider;        /*!< The USB/SDIO clock (48M) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCM_USB_SDIO_Clock_Source */									   
}RCM_ClkInitTypeDef;
/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCM_Exported_Constants RCM Exported Constants
  * @{
  */ 
/** @defgroup RCM_Oscillator_Type Oscillator Type
  * @{
  */
#define RCM_OSCILLATORTYPE_NONE            0x00000000U
#define RCM_OSCILLATORTYPE_XTH             0x00000001U
#define RCM_OSCILLATORTYPE_RCH             0x00000002U
#define RCM_OSCILLATORTYPE_XTL             0x00000004U
#define RCM_OSCILLATORTYPE_RCL             0x00000008U
/**
  * @}
  */

/** @defgroup RCM_XTH_Config XTH Config
  * @{
  */
#define RCM_XTH_OFF                      0x00000000U
#define RCM_XTH_ON                       RCM_CR0_XTH_EN
#define RCM_XTH_BYPASS                   ((uint32_t)(RCM_CR0_XTH_BYP | RCM_CR0_XTH_EN))
/**
  * @}
  */
  
/** @defgroup RCM_XTL_Config XTL Config
  * @{
  */
#define RCM_XTL_OFF                    0x00000000U
#define RCM_XTL_ON                     PMU_XTLCR_XTL_EN
/**
  * @}
  */
  
/** @defgroup RCM_RCH_Config RCH Config
  * @{
  */
#define RCM_RCH_OFF                      ((uint8_t)0x00)
#define RCM_RCH_ON                       ((uint8_t)0x01)
/**
  * @}
  */
  
/** @defgroup RCM_RCL_Config RCL Config
  * @{
  */
#define RCM_RCL_OFF                      ((uint8_t)0x01)
#define RCM_RCL_ON                       ((uint8_t)0x00)

#define PLL_TIMEOUT_VALUE          2U  /* 2 ms */
/**
  * @}
  */
  
/** @defgroup RCM_XTH_Configuration XTH Configuration
  * @{
  */

/**
  * @brief  Macro to configure the External High Speed oscillator (XTH).
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not supported by this macro.
  *         User should request a transition to XTH Off first and then XTH On or XTH Bypass.
  * @note   After enabling the XTH (RCM_XTH_ON or RCM_XTH_Bypass), the application
  *         software should wait on HSERDYF flag to be set indicating that XTH clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   XTH state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the XTH state (ex. disable it).
  * @note   The XTH is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the XTH_MEN bit, so if the XTH clock monitor
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__ specifies the new state of the XTH.
  *         This parameter can be one of the following values:
  *            @arg RCM_XTH_OFF: turn OFF the XTH oscillator, then the HSERDYF flag goes low. 
  *            @arg RCM_XTH_ON: turn ON the XTH oscillator.
  *            @arg RCM_XTH_BYPASS: XTH oscillator bypassed with external clock.
  */
#define __HAL_RCM_XTH_CONFIG(__STATE__)                         \
                    do {                                        \
                      if ((__STATE__) == RCM_XTH_ON)            \
                      {                                         \
                        SET_BIT(RCM->CR0, RCM_CR0_XTH_EN);      \
                      }                                         \
                      else if ((__STATE__) == RCM_XTH_BYPASS)   \
                      {                                         \
                        SET_BIT(RCM->CR0, RCM_CR0_XTH_BYP);     \
                        SET_BIT(RCM->CR0, RCM_CR0_XTH_EN);      \
                      }                                         \
                      else                                      \
                      {                                         \
                        CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_EN);    \
                        CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_BYP);   \
                      }                                         \
                    } while(0U)


/** @defgroup RCM_System_Clock_Type System Clock Type
  * @{
  */
#define RCM_CLOCKTYPE_SYSCLK             0x00000001U
#define RCM_CLOCKTYPE_HCLK               0x00000002U
#define RCM_CLOCKTYPE_PCLK0              0x00000004U
#define RCM_CLOCKTYPE_PCLK1              0x00000008U
#define RCM_CLOCKTYPE_PCLK2              0x00000010U
#define RCM_CLOCKTYPE_PCLK3              0x00000020U
#define RCM_CLOCKTYPE_RCH                0x00000040U
#define RCM_CLOCKTYPE_USBSDIO            0x00000080U
     
/** @defgroup RCM_RCH_Clock_Source RCH Clock Type
  * @{
  */
#define RCM_RCH_DIV1                     RCM_CFGR0_RCH_DIV1  
#define RCM_RCH_DIV2                     RCM_CFGR0_RCH_DIV2  
#define RCM_RCH_DIV3                     RCM_CFGR0_RCH_DIV3  
#define RCM_RCH_DIV4                     RCM_CFGR0_RCH_DIV4  


/** @defgroup RCM_System_Clock_Source System Clock Source
  * @{
  */
#define RCM_SYSCLKSOURCE_RCH             RCM_CFGR0_SYS_SW_RCH
#define RCM_SYSCLKSOURCE_XTH             RCM_CFGR0_SYS_SW_XTH
#define RCM_SYSCLKSOURCE_PLL0CLK         RCM_CFGR0_SYS_SW_PLL0
#define RCM_SYSCLKSOURCE_LFC             RCM_CFGR0_SYS_SW_LFC
                    
/** @defgroup RCM_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCM_SYSCLK_DIV1                  RCM_CFGR0_AHB_DIV1  
#define RCM_SYSCLK_DIV2                  RCM_CFGR0_AHB_DIV2  
#define RCM_SYSCLK_DIV4                  RCM_CFGR0_AHB_DIV4  
#define RCM_SYSCLK_DIV8                  RCM_CFGR0_AHB_DIV8  
#define RCM_SYSCLK_DIV16                 RCM_CFGR0_AHB_DIV16 
#define RCM_SYSCLK_DIV32                 RCM_CFGR0_AHB_DIV32 
#define RCM_SYSCLK_DIV64                 RCM_CFGR0_AHB_DIV64 
#define RCM_SYSCLK_DIV128                RCM_CFGR0_AHB_DIV128
#define RCM_SYSCLK_DIV256                RCM_CFGR0_AHB_DIV256


/** @defgroup RCM_APB0_Clock_Source APB0 Clock Source
  * @{
  */
#define RCM_PCLK0_DIV1                    RCM_CFGR0_APB0_DIV1
#define RCM_PCLK0_DIV2                    RCM_CFGR0_APB0_DIV2
#define RCM_PCLK0_DIV4                    RCM_CFGR0_APB0_DIV4
#define RCM_PCLK0_DIV8                    RCM_CFGR0_APB0_DIV8
#define RCM_PCLK0_DIV16                   RCM_CFGR0_APB0_DIV16
#define RCM_PCLK0_DIV32                   RCM_CFGR0_APB0_DIV32 
#define RCM_PCLK0_DIV64                   RCM_CFGR0_APB0_DIV64 
#define RCM_PCLK0_DIV128                  RCM_CFGR0_APB0_DIV128
#define RCM_PCLK0_DIV256                  RCM_CFGR0_APB0_DIV256


/** @defgroup RCM_APB1_Clock_Source APB1 Clock Source
  * @{
  */
#define RCM_PCLK1_DIV1                    RCM_CFGR0_APB1_DIV1
#define RCM_PCLK1_DIV2                    RCM_CFGR0_APB1_DIV2
#define RCM_PCLK1_DIV4                    RCM_CFGR0_APB1_DIV4
#define RCM_PCLK1_DIV8                    RCM_CFGR0_APB1_DIV8
#define RCM_PCLK1_DIV16                   RCM_CFGR0_APB1_DIV16
#define RCM_PCLK1_DIV32                   RCM_CFGR0_APB1_DIV32 
#define RCM_PCLK1_DIV64                   RCM_CFGR0_APB1_DIV64 
#define RCM_PCLK1_DIV128                  RCM_CFGR0_APB1_DIV128
#define RCM_PCLK1_DIV256                  RCM_CFGR0_APB1_DIV256

/** @defgroup RCM_APB2_Clock_Source APB2 Clock Source
  * @{
  */
#define RCM_PCLK2_DIV1                    RCM_CFGR0_APB2_DIV1
#define RCM_PCLK2_DIV2                    RCM_CFGR0_APB2_DIV2
#define RCM_PCLK2_DIV4                    RCM_CFGR0_APB2_DIV4
#define RCM_PCLK2_DIV8                    RCM_CFGR0_APB2_DIV8
#define RCM_PCLK2_DIV16                   RCM_CFGR0_APB2_DIV16
#define RCM_PCLK2_DIV32                   RCM_CFGR0_APB2_DIV32 
#define RCM_PCLK2_DIV64                   RCM_CFGR0_APB2_DIV64 
#define RCM_PCLK2_DIV128                  RCM_CFGR0_APB2_DIV128
#define RCM_PCLK2_DIV256                  RCM_CFGR0_APB2_DIV256

/** @defgroup RCM_APB3_Clock_Source APB3 Clock Source
  * @{
  */
#define RCM_PCLK3_DIV1                    RCM_CFGR0_APB3_DIV1
#define RCM_PCLK3_DIV2                    RCM_CFGR0_APB3_DIV2
#define RCM_PCLK3_DIV4                    RCM_CFGR0_APB3_DIV4
#define RCM_PCLK3_DIV8                    RCM_CFGR0_APB3_DIV8
#define RCM_PCLK3_DIV16                   RCM_CFGR0_APB3_DIV16
#define RCM_PCLK3_DIV32                   RCM_CFGR0_APB3_DIV32 
#define RCM_PCLK3_DIV64                   RCM_CFGR0_APB3_DIV64 
#define RCM_PCLK3_DIV128                  RCM_CFGR0_APB3_DIV128
#define RCM_PCLK3_DIV256                  RCM_CFGR0_APB3_DIV256

/** @defgroup RCM_USB_SDIO_Clock_Source USB/SDIO Clock Source
  * @{
  */
#define RCM_USB_SDIO_DIV1                 RCM_CFGR1_USB_SDIO_DIV1
#define RCM_USB_SDIO_DIV2                 RCM_CFGR1_USB_SDIO_DIV2
#define RCM_USB_SDIO_DIV3                 RCM_CFGR1_USB_SDIO_DIV3
#define RCM_USB_SDIO_DIV4                 RCM_CFGR1_USB_SDIO_DIV4
#define RCM_USB_SDIO_DIV5                 RCM_CFGR1_USB_SDIO_DIV5
#define RCM_USB_SDIO_DIV6                 RCM_CFGR1_USB_SDIO_DIV6
#define RCM_USB_SDIO_DIV7                 RCM_CFGR1_USB_SDIO_DIV7
#define RCM_USB_SDIO_DIV8                 RCM_CFGR1_USB_SDIO_DIV8

/** @defgroup RCM_PLL_Config PLL Config
  * @{
  */
#define RCM_PLL_NONE                      ((uint8_t)0x00)
#define RCM_PLL_OFF                       ((uint8_t)0x01)
#define RCM_PLL_ON                        ((uint8_t)0x02)

/**
  * @}
  */
/** @defgroup RCM_PLL_Clock_Source PLL Clock Source
  * @{
  */
#define RCM_PLLSOURCE_RCH                RCM_CR0_PLLSRC_RCH
#define RCM_PLLSOURCE_XTH                RCM_CR0_PLLSRC_XTH
/**
  * @}
  */
  
/** @defgroup RCM_TIM_PRescaler_Selection RCM TIM PRescaler Selection
  * @{
  */
#define RCM_TIM_CLK_SEL1APB1_SEL2APB2		0x00000000U
#define RCM_TIM_CLK_SEL1SYSPLL_SEL2APB2		0x00100000U
#define RCM_TIM_CLK_SEL1APB1_SEL2SYSPLL		0x00200000U
#define RCM_TIM_CLK_SEL1SYSPLL_SEL2SYSPLL	0x00300000U
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @brief  Unlock protection register
  * @retval none
  */
#define __HAL_RCM_UNLOCK_REGISTER()			(WRITE_REG(RCM->RCMPR, 0xA5A55A5A))

/** @brief  Lock protection register
  * @retval none
  */
#define __HAL_RCM_LOCK_REGISTER()			(WRITE_REG(RCM->RCMPR, 0xFFFFFFFF))

/** @brief  USART7 CLK select PCLK0
  * @retval none
  */
#define __HAL_RCM_USART7_CLK_SEL_PCLK0() do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->CFGR1, RCM_CFGR1_USART7_CLK_SEL);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	

/** @brief  USART7 CLK select RCL_XTL
  * @retval none
  */
#define __HAL_RCM_USART7_CLK_SEL_RCL_XTL() 	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->CFGR1, RCM_CFGR1_USART7_CLK_SEL);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
/** @brief  USART8 CLK select PCLK2
  * @retval none
  */
#define __HAL_RCM_USART8_CLK_SEL_PCLK2() 	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->CFGR1, RCM_CFGR1_USART8_CLK_SEL);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	

/** @brief  USART8 CLK select RCL/XTL
  * @retval none
  */
#define __HAL_RCM_USART8_CLK_SEL_RCL_XTL() 	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->CFGR1, RCM_CFGR1_USART8_CLK_SEL);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)

/** @brief  USART7 CLK Division
  * @note   Only valid when the USART7 clock select MCK/DIV.
  * @retval none
  */
#define __HAL_RCM_USART7_CLK_DIV(_DIV_) 	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											MODIFY_REG(RCM->CFGR1, RCM_CFGR1_USART7_DIV_Msk, _DIV_);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)											
/** @brief  USART8 CLK Division
  * @note   Only valid when the USART8 clock select MCK/DIV.
  * @retval none
  */
#define __HAL_RCM_USART8_CLK_DIV(_DIV_) 	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											MODIFY_REG(RCM->CFGR1, RCM_CFGR1_USART8_DIV_Msk, _DIV_);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)

/** @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *              - RCM_SYSCLKSOURCE_STATUS_RCH: RCH used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_XTH: XTH used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_PLL0: PLL0 used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_LFC: RCL or XTL used as system clock.
  */
#define __HAL_RCM_GET_SYSCLK_SOURCE() ((uint32_t)(RCM->CFGR0 & RCM_CFGR0_SYS_SWS))

/** @brief  Macro to get the oscillator used as PLL clock source.
  * @retval The oscillator used as PLL clock source. The returned value can be one
  *         of the following:
  *              - RCM_PLLSOURCE_RCH: RCH oscillator is used as PLL clock source.
  *              - RCM_PLLSOURCE_XTH: XTH oscillator is used as PLL clock source.
  */
#define __HAL_RCM_GET_PLL_OSCSOURCE() ((uint32_t)(RCM->CR0 & RCM_CR0_PLLSRC))

/** @defgroup PMU_Peripheral_Clock_Enable_Disable_Status PMU Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the PMU peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_CANFD0_IS_CLK_ENABLED()	(READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CANFD0EN) != RESET)
#define __HAL_RCM_CANFD1_IS_CLK_ENABLED()	(READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CANFD1EN) != RESET)

/**
  * @}
  */

/** @defgroup RCM_AHB_Clock_Enable_Disable AHB Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_SRAM0_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM0EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_SRAM1_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM1EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_QSPI_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHBCKENR, RCM_AHBCKENR_QSPIEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_QSPIEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_EMC_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHBCKENR, RCM_AHBCKENR_EMCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_EMCEN);\
											UNUSED(tmpreg); \
											} while(0U)

#define __HAL_RCM_SRAM0_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_SRAM1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_QSPI_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHBCKENR, RCM_AHBCKENR_QSPIEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_EMC_CLK_DISABLE()		do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHBCKENR, RCM_AHBCKENR_EMCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)										  
										  
/**
  * @}
  */
										  
/** @defgroup RCM_AHB_Peripheral_Clock_Enable_Disable_Status AHB Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_SRAM0_IS_CLK_ENABLED()		(READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM0EN) != RESET)   
#define __HAL_RCM_SRAM1_IS_CLK_ENABLED()		(READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_SRAM1EN) != RESET) 
#define __HAL_RCM_QSPI_IS_CLK_ENABLED()			(READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_QSPIEN) != RESET) 
#define __HAL_RCM_EMC_IS_CLK_ENABLED()			(READ_BIT(RCM->AHBCKENR, RCM_AHBCKENR_EMCEN) != RESET) 

/**
  * @}
  */

/** @defgroup RCM_AHB0_Clock_Enable_Disable AHB0 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB0 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_USB_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_USBEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_USBEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_CRC_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_CRCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_CRCEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_DMA1_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA1EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_DMA2_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA2EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_SDIO_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_SDIOEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_SDIOEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_EMAC_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_EMACEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_EMACEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_DCMI_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DCMIEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DCMIEN);\
											UNUSED(tmpreg); \
											} while(0U)
																					
#define __HAL_RCM_USB_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_USBEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_CRC_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_CRCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_DMA1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_DMA2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_SDIO_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_SDIOEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_EMAC_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_EMACEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_DCMI_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DCMIEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
																						
/**
  * @}
  */

/** @defgroup RCM_AHB0_Peripheral_Clock_Enable_Disable_Status AHB0 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB0 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_USB_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_USBEN) != RESET) 
#define __HAL_RCM_CRC_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_CRCEN) != RESET) 
#define __HAL_RCM_DMA1_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA1EN) != RESET) 
#define __HAL_RCM_DMA2_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DMA2EN) != RESET) 
#define __HAL_RCM_SDIO_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_SDIOEN) != RESET) 
#define __HAL_RCM_EMAC_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_EMACEN) != RESET) 
#define __HAL_RCM_DCMI_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB0CKENR, RCM_AHB0CKENR_DCMIEN) != RESET) 
											
/**
  * @}
  */

/** @defgroup RCM_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_GPIOA_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOAEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOAEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_GPIOB_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOBEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOBEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_GPIOC_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOCEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_GPIOD_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIODEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIODEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_GPIOE_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOEEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOEEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_GPIOH_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOHEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOHEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_AESEX_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_AESEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_AESEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_SHA_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_SHAEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_SHAEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_CORDIC_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_CORDICEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_CORDICEN);\
											UNUSED(tmpreg); \
											} while(0U)
											
#define __HAL_RCM_GPIOA_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOAEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_GPIOB_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOBEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_GPIOC_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOCEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_GPIOD_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIODEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_GPIOE_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOEEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_GPIOH_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOHEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_AESEX_CLK_DISABLE()		do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_AESEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_SHA_CLK_DISABLE()		do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_SHAEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_CORDIC_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_CORDICEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)											


/**
  * @}
  */


/** @defgroup RCM_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_GPIOA_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOAEN) != RESET)   
#define __HAL_RCM_GPIOB_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOBEN) != RESET)
#define __HAL_RCM_GPIOC_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOCEN) != RESET) 
#define __HAL_RCM_GPIOD_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIODEN) != RESET) 
#define __HAL_RCM_GPIOE_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOEEN) != RESET) 									
#define __HAL_RCM_GPIOH_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_GPIOHEN) != RESET) 									
#define __HAL_RCM_AES_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_AESEN) != RESET) 									
#define __HAL_RCM_SHA_IS_CLK_ENABLED()		(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_SHAEN) != RESET) 									
#define __HAL_RCM_CORDIC_IS_CLK_ENABLED()	(READ_BIT(RCM->AHB1CKENR, RCM_AHB1CKENR_CORDICEN) != RESET) 

/**
  * @}
  */

/** @defgroup RCM_APB0_Clock_Enable_Disable APB0 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB0 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_WWDT_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB0CKENR, RCM_APB0CKENR_WWDTEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_WWDTEN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_UART2_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB0CKENR, RCM_APB0CKENR_UART2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_UART2EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_USART7_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB0CKENR, RCM_APB0CKENR_USART7EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_USART7EN);\
											UNUSED(tmpreg); \
											} while(0U)											

#define __HAL_RCM_WWDT_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB0CKENR, RCM_APB0CKENR_WWDTEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_UART2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB0CKENR, RCM_APB0CKENR_UART2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_USART7_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB0CKENR, RCM_APB0CKENR_USART7EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												

/**
  * @}
  */

/** @defgroup RCM_APB0_Peripheral_Clock_Enable_Disable_Status APB0 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB0 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_WWDT_IS_CLK_ENABLED()		(READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_WWDTEN) != RESET)   
#define __HAL_RCM_UART2_IS_CLK_ENABLED()	(READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_UART2EN) != RESET)
#define __HAL_RCM_USART7_IS_CLK_ENABLED()	(READ_BIT(RCM->APB0CKENR, RCM_APB0CKENR_USART7EN) != RESET)

/**
  * @}
  */
  
/** @defgroup RCM_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_UART3_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART3EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_UART4_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART4EN);\
											UNUSED(tmpreg); \
											} while(0U)
#define __HAL_RCM_UART5_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART5EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART5EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_UART6_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART6EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART6EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM2_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM2EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM3_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM3EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM4_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM4EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM5_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM5EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM5EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM6_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM6EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM6EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM7_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM7EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM7EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM12_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM12EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM12EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM13_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM13EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM13EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_TIM14_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM14EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM14EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_DAC_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_DACEN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_DACEN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_I2C1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C1EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_I2C2_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C2EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_I2C3_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C3EN);\
											UNUSED(tmpreg); \
											} while(0U)											
#define __HAL_RCM_SPI1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB1CKENR, RCM_APB1CKENR_SPI1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_SPI1EN);\
											UNUSED(tmpreg); \
											} while(0U)	

#define __HAL_RCM_UART3_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_UART4_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_UART5_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART5EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_UART6_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART6EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM3_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM4_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM5_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM5EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM6_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM6EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM7_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM7EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM12_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM12EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM13_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM13EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM14_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM14EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_DAC_CLK_DISABLE()		do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_DACEN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_I2C1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_I2C2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_I2C3_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_SPI1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB1CKENR, RCM_APB1CKENR_SPI1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)																						
											
/**
  * @}
  */

/** @defgroup RCM_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_UART3_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART3EN) != RESET)  
#define __HAL_RCM_UART4_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART4EN) != RESET)
#define __HAL_RCM_UART5_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART5EN) != RESET)
#define __HAL_RCM_UART6_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_UART6EN) != RESET)
#define __HAL_RCM_TIM2_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM2EN) != RESET)
#define __HAL_RCM_TIM3_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM3EN) != RESET)											
#define __HAL_RCM_TIM4_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM4EN) != RESET)											
#define __HAL_RCM_TIM5_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM5EN) != RESET)											
#define __HAL_RCM_TIM6_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM6EN) != RESET)											
#define __HAL_RCM_TIM7_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM7EN) != RESET)											
#define __HAL_RCM_TIM12_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM12EN) != RESET)											
#define __HAL_RCM_TIM13_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM13EN) != RESET)											
#define __HAL_RCM_TIM14_IS_CLK_ENABLED()	(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_TIM14EN) != RESET)
#define __HAL_RCM_DAC_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_DACEN) != RESET)
#define __HAL_RCM_I2C1_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C1EN) != RESET)
#define __HAL_RCM_I2C2_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C2EN) != RESET)
#define __HAL_RCM_I2C3_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_I2C3EN) != RESET)
#define __HAL_RCM_SPI1_IS_CLK_ENABLED()		(READ_BIT(RCM->APB1CKENR, RCM_APB1CKENR_SPI1EN) != RESET)
											
/**
  * @}
  */

/** @defgroup RCM_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_I2S0_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S0EN);\
											UNUSED(tmpreg); \
											} while(0U)	
#define __HAL_RCM_I2S1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S1EN);\
											UNUSED(tmpreg); \
											} while(0U)	
#define __HAL_RCM_ADC1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC1EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_ADC2_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC2EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_SPI2_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI2EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_SPI3_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI3EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_SPI4_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI4EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_TIM1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM1EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_TIM8_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM8EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM8EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_TIM9_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM9EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM9EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_TIM10_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM10EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM10EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_TIM11_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM11EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM11EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_USART8_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_USART8EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_USART8EN);\
											UNUSED(tmpreg); \
											} while(0U)												
#define __HAL_RCM_UART1_CLK_ENABLE()	do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB2CKENR, RCM_APB2CKENR_UART1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_UART1EN);\
											UNUSED(tmpreg); \
											} while(0U)												
										
#define __HAL_RCM_I2S0_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)		
#define __HAL_RCM_I2S1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)	
#define __HAL_RCM_ADC1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_ADC2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_SPI2_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI2EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_SPI3_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI3EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_SPI4_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI4EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM8_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM8EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM9_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM9EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM10_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM10EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_TIM11_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM11EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_USART8_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_USART8EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
#define __HAL_RCM_UART1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB2CKENR, RCM_APB2CKENR_UART1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)												
																					
/**
  * @}
  */

/** @defgroup RCM_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_I2S0_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S0EN) != RESET)  
#define __HAL_RCM_I2S1_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_I2S1EN) != RESET)
#define __HAL_RCM_ADC1_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC1EN) != RESET)
#define __HAL_RCM_ADC2_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_ADC2EN) != RESET)
#define __HAL_RCM_SPI2_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI2EN) != RESET)
#define __HAL_RCM_SPI3_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI3EN) != RESET)
#define __HAL_RCM_SPI4_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_SPI4EN) != RESET)
#define __HAL_RCM_TIM1_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM1EN) != RESET)
#define __HAL_RCM_TIM8_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM8EN) != RESET)
#define __HAL_RCM_TIM9_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM9EN) != RESET)
#define __HAL_RCM_TIM10_IS_CLK_ENABLED()		(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM10EN) != RESET)
#define __HAL_RCM_TIM11_IS_CLK_ENABLED()	(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_TIM11EN) != RESET)
#define __HAL_RCM_USART8_IS_CLK_ENABLED()	(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_USART8EN) != RESET)
#define __HAL_RCM_UART1_IS_CLK_ENABLED()	(READ_BIT(RCM->APB2CKENR, RCM_APB2CKENR_UART1EN) != RESET)

/**
  * @}
  */

/** @defgroup RCM_APB3_Clock_Enable_Disable APB3 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __HAL_RCM_CAN0_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN0EN);\
											UNUSED(tmpreg); \
											} while(0U)	
#define __HAL_RCM_CAN1_CLK_ENABLE()		do { \
											__IO uint32_t tmpreg = 0x00U;\
											__HAL_RCM_UNLOCK_REGISTER();\
											SET_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											/* Delay after an RCM peripheral clock enabling */ \
											tmpreg = READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN1EN);\
											UNUSED(tmpreg); \
											} while(0U)	

#define __HAL_RCM_CAN0_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN0EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)		
#define __HAL_RCM_CAN1_CLK_DISABLE()	do { \
											__HAL_RCM_UNLOCK_REGISTER();\
											CLEAR_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CAN1EN);\
											__HAL_RCM_LOCK_REGISTER();\
											} while(0U)		

/**
  * @}
  */

/** @defgroup RCM_APB3_Peripheral_Clock_Enable_Disable_Status APB3 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */			
#define __HAL_RCM_CANFD0_IS_CLK_ENABLED()	(READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CANFD0EN) != RESET)
#define __HAL_RCM_CANFD1_IS_CLK_ENABLED()	(READ_BIT(RCM->APB3CKENR, RCM_APB3CKENR_CANFD1EN) != RESET)
/**
  * @}
  */
 
/** @defgroup RCM_AHB_Force_Release_Reset AHB Force Release Reset
  * @brief  Force or release AHB peripheral reset.
  * @{
  */   
#define __HAL_RCM_AHB_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHBRSTR = 0x00130300U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_LOCKUPEN_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_LOCKUP_EN); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                                                                         
#define __HAL_RCM_EMC_FORCE_RESET()        do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_EMCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             
#define __HAL_RCM_QSPI_FORCE_RESET()       do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_QSPIRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             
#define __HAL_RCM_SRAM1_FORCE_RESET()      do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_SRAM1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             
#define __HAL_RCM_SRAM0_FORCE_RESET()      do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_SRAM0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             


#define __HAL_RCM_AHB_RELEASE_RESET() do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHBRSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_LOCKUPEN_RELEASE_RESET() do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_LOCKUP_EN); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_EMC_RELEASE_RESET()      do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_EMCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_QSPI_RELEASE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_QSPIRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_SRAM1_RELEASE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_SRAM1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_SRAM0_RELEASE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHBRSTR ,RCM_AHBRSTR_SRAM0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  

/**
  * @}
  */  

                                            
/** @defgroup RCM_AHB0_Force_Release_Reset AHB0 Force Release Reset
  * @brief  Force or release AHB0 peripheral reset.
  * @{
  */ 
#define __HAL_RCM_AHB0_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHB0RSTR = 0x00001571U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	   											
#define __HAL_RCM_DCMI_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DCMIRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_EMAC_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_EMACRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_SDIO_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_SDIORST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_DMA2_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DMA2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             
#define __HAL_RCM_DMA1_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DMA1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                             
#define __HAL_RCM_CRC_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_CRCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_USB_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_USBRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	

#define __HAL_RCM_AHB0_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHB0RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                                              
#define __HAL_RCM_DCMI_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DCMIRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	          
#define __HAL_RCM_EMAC_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_EMACRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_SDIO_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_SDIORST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_DMA2_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DMA2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_DMA1_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_DMA1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_CRC_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_CRCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_USB_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB0RSTR ,RCM_AHB0RSTR_USBRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  

/**
  * @}
  */                                              
                                            
/** @defgroup RCM_AHB1_Force_Release_Reset AHB1 Force Release Reset
  * @brief  Force or release AHB1 peripheral reset.
  * @{
  */ 
#define __HAL_RCM_AHB1_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHB1RSTR = 0x0000B09FU; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)												
#define __HAL_RCM_CORDIC_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_CORDICRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_SHA_FORCE_RESET()		   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_SHARST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_AESEX_FORCE_RESET()       do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_AESRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOH_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOHRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOE_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOERST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	                      
#define __HAL_RCM_GPIOD_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIODRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOC_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOB_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOBRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOA_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOARST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	

#define __HAL_RCM_AHB1_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->AHB1RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_CORDIC_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_CORDICRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	   
#define __HAL_RCM_SHA_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_SHARST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_AESEX_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_AESRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_GPIOH_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOHRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_GPIOE_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOERST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_GPIOD_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIODRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_GPIOC_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOCRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_GPIOB_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOBRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
#define __HAL_RCM_GPIOA_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->AHB1RSTR ,RCM_AHB1RSTR_GPIOARST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	    
  
  
/**
  * @}
  */                                              
                                                    
                                            


/** @defgroup RCM_APB0_Force_Release_Reset APB0 Force Release Reset
  * @brief  Force or release APB0 peripheral reset.
  * @{
  */
#define __HAL_RCM_APB0_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB0RSTR = 0x00001C00U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)												
#define __HAL_RCM_USART7_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_USART7RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_UART2_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_UART2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_WWDT_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_WWDTRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
											
#define __HAL_RCM_APB0_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB0RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 											
#define __HAL_RCM_USART7_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_USART7RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_UART2_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_UART2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_WWDT_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB0RSTR ,RCM_APB0RSTR_WWDTRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
/**
  * @}
  */                                             
                                            
                                            
/** @defgroup RCM_APB1_Force_Release_Reset APB1 Force Release Reset
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */

#define __HAL_RCM_APB1_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB1RSTR = 0x00173FFFU; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 										
#define __HAL_RCM_SPI1_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_SPI1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_I2C3_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_I2C2_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_I2C1_FORCE_RESET()    do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_DAC_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_DACRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	
#define __HAL_RCM_TIM14_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM14RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM13_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM13RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM12_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM12RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM7_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM7RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM6_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM6RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM5_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM5RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM4_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM4RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM3_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_TIM2_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_UART6_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART6RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_UART5_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART5RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_UART4_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART4RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)
#define __HAL_RCM_UART3_FORCE_RESET()     do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            SET_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)

#define __HAL_RCM_APB1_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB1RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)   
#define __HAL_RCM_SPI1_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_SPI1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)      
#define __HAL_RCM_I2C3_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)    
#define __HAL_RCM_I2C2_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)    
#define __HAL_RCM_I2C1_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_I2C1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)    
#define __HAL_RCM_DAC_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_DACRST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM14_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM14RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM13_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM13RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM12_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM12RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM7_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM7RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM6_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM6RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM5_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM5RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM4_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM4RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM3_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_TIM2_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_TIM2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_UART6_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART6RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_UART5_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART5RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_UART4_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART4RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_UART3_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB1RSTR ,RCM_APB1RSTR_UART3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
/**
  * @}
  */  

/** @defgroup RCM_APB2_Force_Release_Reset APB2 Force Release Reset
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
#define __HAL_RCM_APB2_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             RCM->APB2RSTR = 0x0000FFF3U; \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_UART1_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_UART1RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_USART8_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_USART8RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_TIM11_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_TIM11RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_TIM10_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_TIM10RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_TIM9_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_TIM9RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_TIM8_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_TIM8RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_TIM1_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_TIM1RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_SPI4_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_SPI4RST); \
                                             __HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	      
#define __HAL_RCM_SPI3_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR , RCM_APB2RSTR_SPI3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_SPI2_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR,RCM_APB2RSTR_SPI2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	  
#define __HAL_RCM_ADC1_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR,RCM_APB2RSTR_ADC1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_ADC2_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR,RCM_APB2RSTR_ADC2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_I2S1_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR,RCM_APB2RSTR_I2S1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_I2S0_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB2RSTR,RCM_APB2RSTR_I2S0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
                                            
#define __HAL_RCM_APB2_RELEASE_RESET()   do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB2RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)   
#define __HAL_RCM_UART1_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_UART1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_USART8_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_USART8RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_TIM11_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_TIM11RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_TIM10_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_TIM10RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_TIM9_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_TIM9RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_TIM8_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_TIM8RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_TIM1_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_TIM1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_SPI4_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_SPI4RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
 #define __HAL_RCM_SPI3_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_SPI3RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)    
#define __HAL_RCM_SPI2_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_SPI2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)                                               
#define __HAL_RCM_ADC1_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_ADC1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 							
#define __HAL_RCM_ADC2_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_ADC2RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U) 
#define __HAL_RCM_I2S1_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_I2S1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_I2S0_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB2RSTR ,RCM_APB2RSTR_I2S0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
											
/**
  * @}
  */


/** @defgroup RCM_APB3_Force_Release_Reset APB3 Force Release Reset
  * @brief  Force or release APB3 peripheral reset.
  * @{
  */                                           
#define __HAL_RCM_APB3_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             RCM->APB3RSTR = 0x00000003U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 										
#define __HAL_RCM_CAN0_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB3RSTR,RCM_APB3RSTR_CAN0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	 
#define __HAL_RCM_CAN1_FORCE_RESET()     do { \
                                             __HAL_RCM_UNLOCK_REGISTER(); \
                                             SET_BIT(RCM->APB3RSTR,RCM_APB3RSTR_CAN1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)	

#define __HAL_RCM_APB3_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            RCM->APB3RSTR = 0x00000000U; \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_CAN0_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB3RSTR ,RCM_APB3RSTR_CAN0RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  
#define __HAL_RCM_CAN1_RELEASE_RESET()  do { \
                                            __HAL_RCM_UNLOCK_REGISTER(); \
                                            CLEAR_BIT(RCM->APB3RSTR ,RCM_APB3RSTR_CAN1RST); \
											__HAL_RCM_LOCK_REGISTER(); \
											} while(0U)  

/**
  * @}
  */                                              
                                            
                                            
                                            
                                            
/** @defgroup RCM_Flags   RCM FLags
  *           Flag mask in the CR register
  * @{
  */
#define RCM_FLAG_PLL1STB                 ((uint32_t)RCM_CR0_PLL1STB)
#define RCM_FLAG_PLL0STB                 ((uint32_t)RCM_CR0_PLL0STB)
#define RCM_FLAG_XTH_STB                 ((uint32_t)RCM_CR0_XTH_STB)
#define RCM_FLAG_RCH_STB                 ((uint32_t)RCM_CR0_RCH_STB)
									
/** @defgroup RCM_Interrupt Interrupts
  * @{
  */
#define RCM_IT_LSERDYF                   ((uint32_t)RCM_CIFR_LSERDYF)
#define RCM_IT_HSERDYF                   ((uint32_t)RCM_CIFR_HSERDYF)
#define RCM_IT_PLL0RDYF                  ((uint32_t)RCM_CIFR_PLL0RDYF)
#define RCM_IT_PLL1RDYF                  ((uint32_t)RCM_CIFR_PLL1RDYF)
#define RCM_IT_CSSF                      ((uint32_t)RCM_CIFR_CSSF)
										  
/**
  * @}
  */
										  
/**
  * @}
  */
										  

/** @defgroup xxx_Exported_macro xxx Exported Macro
  * @{
  */ 

/** @brief  Check RCM flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg RCM_FLAG_PLL1STB: PLL1 clock ready.
  *            @arg RCM_FLAG_PLL0STB: PLL0 clock ready.
  *            @arg RCM_FLAG_XTH_STB: XTH clock ready.
  *            @arg RCM_FLAG_RCH_STB: RCH clock ready.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_RCM_GET_FLAG(__FLAG__) ((RCM->CR0 & (__FLAG__)) == (__FLAG__))    

/** @brief  Check the RCM's interrupt has occurred or not.
  * @param  __INTERRUPT__ specifies the RCM interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg RCM_IT_LSERDYF : LSE ready interrupt.
  *            @arg RCM_IT_HSERDYF : HSE ready interrupt.
  *            @arg RCM_IT_PLL0RDYF: PLL0 ready interrupt.
  *            @arg RCM_IT_PLL1RDYF: PLL1 ready interrupt.
  *            @arg RCM_IT_CSSF    : CSSF ready interrupt.
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_RCM_GET_IT(__INTERRUPT__) ((RCM->CIR & (__INTERRUPT__)) == (__INTERRUPT__))


/** @brief  Macros to enable or disable the Internal High Speed oscillator (RCH).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  *         It is used (enabled by hardware) as system clock source after startup
  *         from Reset, wake-up from STOP and STANDBY mode.
  * @note   RCH can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the RCH.
  * @note   After enabling the RCH, the application software should wait on RCH_STB
  *         flag to be set indicating that RCH clock is stable and can be used as
  *         system clock source.
  *         This parameter can be: ENABLE or DISABLE.
  * @note   When the RCH is stopped, RCH_STB flag goes low.
  */
#define __HAL_RCM_RCH_ENABLE()      MODIFY_REG(RCM->CR0, RCM_CR0_RCH_EN, (ENABLE << RCM_CR0_RCH_EN_Pos))
#define __HAL_RCM_RCH_DISABLE()     MODIFY_REG(RCM->CR0, RCM_CR0_RCH_EN, (DISABLE << RCM_CR0_RCH_EN_Pos))


/** @brief  Macros to enable or disable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  */
#define __HAL_RCM_PLL_ENABLE()      MODIFY_REG(RCM->CR0, RCM_CR0_PLL0EN, (ENABLE << RCM_CR0_PLL0EN_Pos))
#define __HAL_RCM_PLL_DISABLE()     MODIFY_REG(RCM->CR0, RCM_CR0_PLL0EN, (DISABLE << RCM_CR0_PLL0EN_Pos))

/** @brief  Macros to enable or disable the ext filter.
  */
#define __HAL_RCM_EXT_FILTER_ENABLE()      MODIFY_REG(RCM->EXRSTFER, RCM_EXRSTFER_EXT_FILTER_EN, (ENABLE << RCM_EXRSTFER_EXT_FILTER_EN_Pos))
#define __HAL_RCM_EXT_FILTER_DISABLE()     MODIFY_REG(RCM->EXRSTFER, RCM_EXRSTFER_EXT_FILTER_EN, (DISABLE << RCM_EXRSTFER_EXT_FILTER_EN_Pos))



/**
  * @brief Macro to configure the system clock source.
  * @param __RCM_SYSCLKSOURCE__ specifies the system clock source.
  * This parameter can be one of the following values:
  *              - RCM_SYSCLKSOURCE_RCH: RCH oscillator is used as system clock source.
  *              - RCM_SYSCLKSOURCE_XTH: XTH oscillator is used as system clock source.
  *              - RCM_SYSCLKSOURCE_PLL0CLK: PLL0 output is used as system clock source.
  *              - RCM_SYSCLKSOURCE_LFC: RCL or XTL is used as system clock source.
  */
#define __HAL_RCM_SYSCLK_CONFIG(__RCM_SYSCLKSOURCE__) MODIFY_REG(RCM->CFGR0, RCM_CFGR0_SYS_SW, (__RCM_SYSCLKSOURCE__))

/** @brief  Macro to configure the Timers clocks prescalers   
  * @note	If PLL0 is used as the Timers clock source, the value must meet apbx_clk = HCLK = SYSPLL/2
  * @param  __PRESC__  specifies the Timers clocks prescalers selection
  *         This parameter can be one of the following values:
  *            @arg RCM_TIM_CLK_SEL1APB1_SEL2APB2		: TIM1/2/3/4/11/12/13 Select apb1_clk as the clock source, 
  *														  TIM1/7/8/9/10 Select apb2_clk as the clock source.       
  *            @arg RCM_TIM_CLK_SEL1SYSPLL_SEL2APB2		: TIM1/2/3/4/11/12/13 Select SYSPLL as the clock source.
  *														  TIM1/7/8/9/10 Select apb2_clk as the clock source. 
  *            @arg RCM_TIM_CLK_SEL1APB1_SEL2SYSPLL		: TIM1/2/3/4/11/12/13 Select apb1_clk as the clock source, 
  *														  TIM1/7/8/9/10 Select SYSPLL as the clock source.   
  *            @arg RCM_TIM_CLK_SEL1SYSPLL_SEL2SYSPLL	: TIM1/2/3/4/11/12/13 Select SYSPLL as the clock source,
  *														  TIM1/7/8/9/10 Select SYSPLL as the clock source. 
  */
#define __HAL_RCM_TIMCLKPRESCALER(__PRESC__)	do { \
													__HAL_RCM_UNLOCK_REGISTER();\
													RCM->CFGR1 = ((RCM->CFGR1 & 0xFFCFFFFFU) | (__PRESC__));\
													__HAL_RCM_LOCK_REGISTER();\
													} while(0U)	

													
													
													
													


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCM_Exported_Functions
  * @{
  */ 
/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCM_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  Rwaitcyc FLASH Read wait cycles, this parameter depend on systemclock
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The RCH is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the XTH used directly or indirectly as system clock
  *         (if the Clock Security System XTH_MEN is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @retval None
  */
HAL_StatusTypeDef HAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t Rwaitcyc); 
 
  /**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not
  *         supported by this API. User should request a transition to XTH Off
  *         first and then XTH On or XTH Bypass.
  * @retval HAL status
  */
//__weak HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct);
HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct);

/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is RCH, function returns values based on RCH_VALUE(*)
  * @note     If SYSCLK source is XTH, function returns values based on XTH_VALUE(**)
  * @note     If SYSCLK source is PLL0, function returns values based on XTH_VALUE(**)
  *           or RCH_VALUE(*) multiplied/divided by the PLL0 factors.
  * @note     (*) RCH_VALUE is a constant defined in um324xF_hal_conf.h file (default value
  *               48 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) XTH_VALUE is a constant defined in um324xF_hal_conf.h file (default value
  *                12 MHz), user has to ensure that XTH_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *
  * @note   The result of this function could be not correct when using fractional
  *         value for XTH crystal.
  *
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  *
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  *
  * @retval SYSCLK frequency
  */
uint32_t HAL_RCM_GetSysClockFreq(void);

/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t HAL_RCM_GetHCLKFreq(void);

/**
  * @brief  Returns the PCLK0 frequency
  * @note   Each time PCLK0 changes, this function must be called to update the
  *         right PCLK0 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK0 frequency
  */
uint32_t HAL_RCM_GetPCLK0Freq(void);

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t HAL_RCM_GetPCLK1Freq(void);

/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t HAL_RCM_GetPCLK2Freq(void);

/**
  * @brief  Returns the PCLK3 frequency
  * @note   Each time PCLK3 changes, this function must be called to update the
  *         right PCLK3 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK3 frequency
  */
uint32_t HAL_RCM_GetPCLK3Freq(void);


/* Private macros ------------------------------------------------------------*/
/** @defgroup xxx_Private_Macros 
  * @{
  */


#define XTH_TIMEOUT_VALUE          XTH_STARTUP_TIMEOUT
#define RCH_TIMEOUT_VALUE          2U  /* 2 ms */
#define RCL_TIMEOUT_VALUE          2U  /* 2 ms */
#define CLOCKSWITCH_TIMEOUT_VALUE  5000U /* 5 s */
#define PLLLT_TIME_VALUE  		   (0x10000 | 12000U) /* 500 us */

/* Private constants ---------------------------------------------------------*/ 
/* Private functions ---------------------------------------------------------*/

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
#endif

