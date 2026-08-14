 /**
  ******************************************************************************
  * @file     um324xx_hal_pmu.h
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
#ifndef __UM324XX_HAL_PMU_H__
#define __UM324XX_HAL_PMU_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup PMU
  * @{
  */

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup PMU_Exported_typedefs PMU Exported Typedefs
  * @{
  */


/**
  * @brief LVD Init structure definition
  */
typedef struct
{
    uint32_t En;                /**< The LVD hysteresis filtering function
                                This parameter can be a value of @ref PMU_LVD_EN_define */
    
    uint32_t ResetEn;          /**< The LVD hysteresis filtering function
                                This parameter can be a value of @ref PMU_LVD_RST_EN */
    
    uint32_t FilterEnable;      /**< Filter Enable
                                This parameter can be a value of @ref PMU_FILTER_EN_define */
    
    uint32_t VDTS;             /**< VDD select
                                This parameter can be a value of @ref PMU_LVD_SET */
    
    uint32_t IrqEn;             /**< Interrupt enablement
                                This parameter can be a value of @ref PMU_LVD_INT_EN_define */

} LVD_InitTypeDef;

typedef struct __LVD_HandleTypeDef
{  	
    LVD_InitTypeDef Init;           /* LVD basic required parameters*/
  

#if (USE_HAL_PMU_REGISTER_CALLBACKS == 1)

  void (*LVD_Callback)(struct __LVD_HandleTypeDef *hlvd); 	/* LVD interrupt the callback */

  void (* MspInitCallback)(struct __LVD_HandleTypeDef *hlvd);           /*!< LVD Msp Init callback                */
  void (* MspDeInitCallback)(struct __LVD_HandleTypeDef *hlvd);         /*!< LVD Msp DeInit callback              */
    
#endif  /* USE_HAL_PMU_REGISTER_CALLBACKS */    
    
} PMU_HandleTypeDef;

/**
  * @brief PMU Mode structure definition
  */
typedef enum 
{
    PMU_RUN_MODE        = 0x0U, 
    PMU_STOP_MODE       = 0x1U,
    PMU_STANDBY_MODE    = 0x2U,
    PMU_DEEPSTANDBY_MODE= 0x3U
} PMU_ModeTypeDef;

/**
  * @brief PMU Mode structure definition
  */
typedef enum 
{
    RISINGEDGE        = 0x0U, 
    FALLINGEDGE       = 0x1U,
} WK_IO_TriggerTypeDef;

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PMU_Exported_macro PMU Exported Macro
  * @{
  */ 
/** @brief  Unlock PMU protection register
  * @retval none
  */
#define __HAL_PMU_UNLOCK_REGISTER()         (WRITE_REG(PMU->CPR, 0xABCD))

/** @brief  Lock PMU protection register
  * @retval none
  */
#define __HAL_PMU_LOCK_REGISTER()           (WRITE_REG(PMU->CPR, 0x459E))

/** @defgroup PMU_Function_Clock_Enable_Disable PMU Function Clock Enable Disable
  * @brief  Enable or disable the PMU function clock.
  * @note   After reset, the PMU function clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
  
#if defined(UM32x42x)  
#define __HAL_PMU_LPUART0_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_LPUART0EN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPUART0EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)

#define __HAL_PMU_LPUART1_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_LPUART1EN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPUART1EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#endif
#if defined(UM324xF)

#define __HAL_PMU_LPUART_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_LPUARTEN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPUARTEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#endif                                       

#define __HAL_PMU_IWDT_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U;\
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FCCR, PMU_FCCR_IWDTEN);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        /* Delay after an PMU function clock enabling */ \
                                        tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_IWDTEN);\
                                        UNUSED(tmpreg); \
                                       } while(0U)

#define __HAL_PMU_LPTIM0_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_LPTIM0EN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM0EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
                                                                                        
#define __HAL_PMU_LPTIM1_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_LPTIM1EN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM1EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)

#define __HAL_PMU_RTC_CLK_ENABLE()      do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       SET_BIT(PMU->FCCR, PMU_FCCR_RTCEN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       /* Delay after an PMU function clock enabling */ \
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_RTCEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)


#define __HAL_PMU_LPUART0_CLK_DISABLE()  do { \
                                        __IO uint32_t tmpreg;\
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FCCR, PMU_FCCR_LPUART0EN);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPUART0EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)

#define __HAL_PMU_LPUART1_CLK_DISABLE()  do { \
                                        __IO uint32_t tmpreg;\
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FCCR, PMU_FCCR_LPUART1EN);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPUART1EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)                                        
                                        
#define __HAL_PMU_IWDT_CLK_DISABLE()    do { \
                                        __IO uint32_t tmpreg;\
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FCCR, PMU_FCCR_IWDTEN);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_IWDTEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)

#define __HAL_PMU_LPTIM0_CLK_DISABLE()  do { \
                                        __IO uint32_t tmpreg = 0x00U;\
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FCCR, PMU_FCCR_LPTIM0EN);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM0EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)

#define __HAL_PMU_LPTIM1_CLK_DISABLE()  do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       CLEAR_BIT(PMU->FCCR, PMU_FCCR_LPTIM1EN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM1EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U) 

#define __HAL_PMU_RTC_CLK_DISABLE()     do { \
                                       __IO uint32_t tmpreg = 0x00U;\
                                       __HAL_PMU_UNLOCK_REGISTER();\
                                       CLEAR_BIT(PMU->FCCR, PMU_FCCR_RTCEN);\
                                       __HAL_PMU_LOCK_REGISTER();\
                                       tmpreg = READ_BIT(PMU->FCCR, PMU_FCCR_RTCEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)                                       
                                       

/**
  * @}
  */
                                            
/** @defgroup PMU_Function_Clock_Enable_Disable_Status PMU Function Clock Enable Disable Status
  * @brief  Force or release PMU peripheral reset.
  * @{
  */

#define __HAL_PMU_RTC_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_RTCRST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_RTCRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
                                      
#if defined(UM32x42x)                                       
#define __HAL_PMU_LPUART0_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_LPUART0RST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUART0RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)                                      
#define __HAL_PMU_LPUART1_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_LPUART1RST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUART1RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#endif
#if defined(UM324xF)
#define __HAL_PMU_LPUART_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_LPUARTRST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUARTRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)           

#endif                                      
                                      
#define __HAL_PMU_LPTIM1_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_LPTIM1RST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPTIM1RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_PMU_LPTIM0_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_LPTIM0RST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPTIM0RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_PMU_IWDT_FORCE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        SET_BIT(PMU->FRCR, PMU_FRCR_IWDTRST);\
                                        __HAL_RCM_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_IWDTRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_PMU_RTC_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_RTCRST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_RTCRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#if defined(UM32x42x) 
#define __HAL_PMU_LPUART0_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_LPUART0RST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUART0RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)                                      
#define __HAL_PMU_LPUART1_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_LPUART1RST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUART1RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#endif
#if defined(UM324xF)                                      
#define __HAL_PMU_LPUART_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_LPUARTRST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPUARTRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)                                         
#endif                                     
                                      
#define __HAL_PMU_LPTIM1_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_LPTIM1RST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPTIM1RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_PMU_LPTIM0_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_LPTIM0RST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_LPTIM0RST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_PMU_IWDT_RELEASE_RESET()   do { \
                                        __IO uint32_t tmpreg; \
                                        __HAL_PMU_UNLOCK_REGISTER();\
                                        CLEAR_BIT(PMU->FRCR, PMU_FRCR_IWDTRST);\
                                        __HAL_PMU_LOCK_REGISTER();\
                                        tmpreg = READ_BIT(PMU->FRCR, PMU_FRCR_IWDTRST);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

/**
  * @}
  */
                                            
/** @defgroup PMU_Function_Clock_Enable_Disable_Status PMU Function Clock Enable Disable Status
  * @brief  Get the enable or disable status of the PMU function clock.
  * @note   After reset, the PMU function clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */	
#define __HAL_PMU_LPUART_IS_CLK_ENABLED()		(READ_BIT(PMU->FCCR, PMU_FCCR_LPUARTEN) != RESET)  
#define __HAL_PMU_IWDT_IS_CLK_ENABLED()			(READ_BIT(PMU->FCCR, PMU_FCCR_IWDTEN) != RESET)
#define __HAL_PMU_LPTIM0_IS_CLK_ENABLED()		(READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM0EN) != RESET)
#define __HAL_PMU_LPTIM1_IS_CLK_ENABLED()		(READ_BIT(PMU->FCCR, PMU_FCCR_LPTIM1EN) != RESET)
#define __HAL_PMU_RTC_IS_CLK_ENABLED()			(READ_BIT(PMU->FCCR, PMU_FCCR_RTCEN) != RESET)

/**
  * @}
  */                                            
                                            
/** @defgroup PMU_Exported_constants PMU Exported Constants
  * @{
  */ 

/** @brief  PMU XTL ENABLE
  * @retval none
  */
#define __HAL_PMU_XTL_ENABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->XTLCR,PMU_XTLCR_XTL_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU XTL DISABLE
  * @retval none
  */
#define __HAL_PMU_XTL_DISABLE()             do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->XTLCR,PMU_XTLCR_XTL_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU RCL ENABLE
  * @retval none
  */
#define __HAL_PMU_RCL_ENABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->RCLCR,PMU_RCLCR_RCL_PD);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU RCL DISABLE
  * @retval none
  */
#define __HAL_PMU_RCL_DISABLE()             do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->RCLCR,PMU_RCLCR_RCL_PD);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU RCL DISABLE
  * @retval none
  */
#define __HAL_PMU_LSCLK_SEL(__VALUE__)      do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->XTLCR,PMU_XTLCR_LSCLK_SEL,(__VALUE__));\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU LVD ENABLE
  * @retval none
  */
#define __HAL_PMU_LVD_ENABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->VDCR,PMU_VDCR_LVD_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                            
/** @brief  PMU LVD DISABLE
  * @retval none
  */
#define __HAL_PMU_LVD_DISABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->VDCR,PMU_VDCR_LVD_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)      
/** @brief  PMU BOR ENABLE
  * @retval none
  */
#define __HAL_PMU_BOR_ENABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->VDCR,PMU_VDCR_BOR_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                            
/** @brief  PMU BOR DISABLE
  * @retval none
  */
#define __HAL_PMU_BOR_DISABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->VDCR,PMU_VDCR_BOR_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)  
/** @brief  PMU PDR ENABLE
  * @retval none
  */
#define __HAL_PMU_PDR_ENABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->VDCR,PMU_VDCR_PDR_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                            
/** @brief  PMU PDR DISABLE
  * @retval none
  */
#define __HAL_PMU_PDR_DISABLE()              do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->VDCR,PMU_VDCR_PDR_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U) 
/**
  * @}
  */                                                                                             
                                            
 /** @defgroup PMU_WK_Flag PMU Wake Up Flag
  * @{
  */
#define PMU_FLAG_RESETN_WK              PMU_PDWKCR_RESETN_WK_FLAG
#define PMU_FLAG_PC13_WK                PMU_PDWKCR_PC13_WK_FLAG
#define PMU_FLAG_PC3_WK                 PMU_PDWKCR_PC3_WK_FLAG
#define PMU_FLAG_PC2_WK                 PMU_PDWKCR_PC2_WK_FLAG
#define PMU_FLAG_PC0_WK                 PMU_PDWKCR_PC0_WK_FLAG
#define PMU_FLAG_PA2_WK                 PMU_PDWKCR_PA2_WK_FLAG
#define PMU_FLAG_PA0_WK                 PMU_PDWKCR_PA0_WK_FLAG

/**
  * @}
  */
                                            
/** @defgroup PMU_WK_IO PMU Wake Up IO
  * @{
  */
#define PMU_STANDBY_WK_PC13                 PMU_PDWKCR_PC13_WKE
#define PMU_STANDBY_WK_PC3                  PMU_PDWKCR_PC3_WKE
#define PMU_STANDBY_WK_PC2                  PMU_PDWKCR_PC2_WKE
#define PMU_STANDBY_WK_PC0                  PMU_PDWKCR_PC0_WKE
#define PMU_STANDBY_WK_PA2                  PMU_PDWKCR_PA2_WKE
#define PMU_STANDBY_WK_PA0                  PMU_PDWKCR_PA0_WKE

/**
  * @}
  */                                            

/** @defgroup PMU_WAKEUP_POLARITY PMU WAKEUP POLARITY
  * @{
  */
#define PMU_WAKEUP_POLARITY_UP			0x01U
#define PMU_WAKEUP_POLARITY_DOWN		0x00U
/**
  * @}
  */

/** @defgroup PMU_WK_EVENT PMU Wake Up Event
  * @{
  */
#define PMU_WKE_LPUART1                 PMU_PDWKCR_LPUART1_WEK
#define PMU_WKE_LPTIM1                  PMU_PDWKCR_LPTIM1_WKE
#define PMU_WKE_LPTIM0                  PMU_PDWKCR_LPTIM0_WKE
#define PMU_WKE_LPUART0                 PMU_PDWKCR_LPUART0_WKE
#define PMU_WKE_IWDT                    PMU_PDWKCR_IWDT_WKE
#define PMU_WKE_RTC_TAMP                PMU_PDWKCR_RTC_TAMP_WKE
#define PMU_WKE_RTC_ALARM               PMU_PDWKCR_RTC_ALARM_WKE
#define PMU_WKE_RSTN                    PMU_PDWKCR_RSTN_WKE
/**
  * @}
  */
  
/** @defgroup PMU_WAKEUP_POLARITY PMU WAKEUP POLARITY
  * @{
  */

#define PMU_WAKEUP_EXTI0				SYSCFG_EXTIWR_EXTI0_CLR_Pos
#define PMU_WAKEUP_EXTI1				SYSCFG_EXTIWR_EXTI1_CLR_Pos
#define PMU_WAKEUP_EXTI2				SYSCFG_EXTIWR_EXTI2_CLR_Pos
#define PMU_WAKEUP_EXTI3				SYSCFG_EXTIWR_EXTI3_CLR_Pos
#define PMU_WAKEUP_EXTI4				SYSCFG_EXTIWR_EXTI4_CLR_Pos
#define PMU_WAKEUP_EXTI5				SYSCFG_EXTIWR_EXTI5_CLR_Pos
#define PMU_WAKEUP_EXTI6				SYSCFG_EXTIWR_EXTI6_CLR_Pos
#define PMU_WAKEUP_EXTI7				SYSCFG_EXTIWR_EXTI7_CLR_Pos
#define PMU_WAKEUP_EXTI8				SYSCFG_EXTIWR_EXTI8_CLR_Pos
#define PMU_WAKEUP_EXTI9				SYSCFG_EXTIWR_EXTI9_CLR_Pos
#define PMU_WAKEUP_EXTI10				SYSCFG_EXTIWR_EXTI10_CLR_Pos
#define PMU_WAKEUP_EXTI11				SYSCFG_EXTIWR_EXTI11_CLR_Pos
#define PMU_WAKEUP_EXTI12				SYSCFG_EXTIWR_EXTI12_CLR_Pos
#define PMU_WAKEUP_EXTI13				SYSCFG_EXTIWR_EXTI13_CLR_Pos
#define PMU_WAKEUP_EXTI14				SYSCFG_EXTIWR_EXTI14_CLR_Pos
#define PMU_WAKEUP_EXTI15				SYSCFG_EXTIWR_EXTI15_CLR_Pos
/**
  * @}
  */

#define PMU_STANDBY_IO_KEEP_ANALOG_HIGH_RES     0x00000000U
#define PMU_STANDBY_IO_KEEP_STATE               PMU_MR_IO_KEEP

#define PMU_RCL_ENABLE                          0x00000000U
#define PMU_RCL_DISABLE                         PMU_RCLCR_RCL_PD

#define PMU_LSCLK_SEL_RCL                        0x00000000U
#define PMU_LSCLK_SEL_XTL                        PMU_XTLCR_LSCLK_SEL

/** @defgroup PMU_SLEEP_mode_entry PMU SLEEP mode entry
  * @{
  */
#define PMU_SLEEPENTRY_WFI              ((uint8_t)0x01)
#define PMU_SLEEPENTRY_WFE              ((uint8_t)0x02)
/**
  * @}
  */

/** @defgroup PMU_STOP_mode_entry PMU STOP mode entry
  * @{
  */
#define PMU_STOPENTRY_WFI               ((uint8_t)0x01)
#define PMU_STOPENTRY_WFE               ((uint8_t)0x02)
/**
  * @}
  */

/** @defgroup PMU_RST_Flag PMU Rst Flag
  * @{
  */
#define PMU_FLAG_LVD_RST                PMU_SASR_LVD_FLAG
#define PMU_FLAG_BOR_RST                PMU_SASR_BOR_FLAG
#define PMU_FLAG_XTL_RSTN               PMU_SASR_XTL_RSTN_FLAG
#define PMU_FLAG_FCORE_PDN_RST          PMU_SASR_CORE_PDN_FLAG
#define PMU_FLAG_PDR_RST                PMU_SASR_PDR_FLAG

/**
  * @}
  */
  
/** @defgroup PMU_LVD_RST_EN PMU LVD RST EN
  * @{
  */
#define PMU_LVD_RST_ENABLE              PMU_VDCR_LVD_RST_EN
#define PMU_LVD_RST_DISABLE             0x00000000U  

/**
  * @}
  */

/** @defgroup PMU_LVD_INT_EN_define PMU LVD INT EN define
  * @{
  */
#define PMU_LVD_INT_ENABLE              PMU_VDCR_LVD_INT_EN
#define PMU_LVD_INT_DISABLE             0x00000000U  

/**
  * @}
  */  
  
/** @defgroup PMU_FILTER_EN_define PMU FILTER EN define
  * @{
  */
#define PMU_FILTER_ENABLE               PMU_VDCR_LVD_FILTER_EN
#define PMU_FILTER_DISABLE              0x00000000U    

/**
  * @}
  */  
  
/** @defgroup PMU_LVD_EN_define PMU LVD EN define
  * @{
  */
#define PMU_LVD_ENABLE               PMU_VDCR_LVD_EN
#define PMU_LVD_DISABLE              0x00000000U    

/**
  * @}
  */    
   
/** @defgroup LVD_RST_PMU_EN_SET LVD RST PMU SET
  * @{
  */
#define LVD_RST_PMU_ENABLE               PMU_SASR_LVD_PMU_EN
#define LVD_RST_PMU_DISABLE              0x00000000U    

/**
  * @}
  */    

/** @defgroup BOR_RST_PMU_EN_SET BOR RST PMU SET
  * @{
  */
#define BOR_RST_PMU_ENABLE               PMU_SASR_BOR_PMU_EN
#define BOR_RST_PMU_DISABLE              0x00000000U    

/**
  * @}
  */    

/** @defgroup LOW_VDD_SWITCH LOW VDD SWITCH
  * @{
  */
#define PMU_LOW_VDD_ENABLE               PMU_SASR_EFC_LOW_VDD_EN
#define PMU_LOW_VDD_DISABLE              0x00000000U    

/**
  * @}
  */

/** @defgroup PMU_LVD_SET PMU LVD SET
  * @{
  */
#define PMU_LVD_SET_1V59                0x00000000U
#define PMU_LVD_SET_1V70                PMU_VDCR_LVDS_0
#define PMU_LVD_SET_1V80                PMU_VDCR_LVDS_1
#define PMU_LVD_SET_1V90                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_1
#define PMU_LVD_SET_2V00                PMU_VDCR_LVDS_2
#define PMU_LVD_SET_2V10                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_2
#define PMU_LVD_SET_2V21                PMU_VDCR_LVDS_1|PMU_VDCR_LVDS_2
#define PMU_LVD_SET_2V31                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_1|PMU_VDCR_LVDS_2
#define PMU_LVD_SET_2V41                PMU_VDCR_LVDS_3
#define PMU_LVD_SET_2V51                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_2V61                PMU_VDCR_LVDS_1|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_2V72                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_1|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_2V82                PMU_VDCR_LVDS_2|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_2V92                PMU_VDCR_LVDS_0|PMU_VDCR_LVDS_2|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_3V02                PMU_VDCR_LVDS_1|PMU_VDCR_LVDS_2|PMU_VDCR_LVDS_3
#define PMU_LVD_SET_3V12                PMU_VDCR_LVDS

/**
  * @}
  */
  
/** @defgroup PMU_BOR_SET PMU BOR SET
  * @{
  */
#define PMU_BOR_SET_1V59                0x00000000U
#define PMU_BOR_SET_1V70                PMU_VDCR_BORS_0
#define PMU_BOR_SET_1V80                PMU_VDCR_BORS_1
#define PMU_BOR_SET_1V90                PMU_VDCR_BORS_0|PMU_VDCR_BORS_1
#define PMU_BOR_SET_2V00                PMU_VDCR_BORS_2
#define PMU_BOR_SET_2V10                PMU_VDCR_BORS_0|PMU_VDCR_BORS_2
#define PMU_BOR_SET_2V21                PMU_VDCR_BORS_1|PMU_VDCR_BORS_2
#define PMU_BOR_SET_2V31                PMU_VDCR_BORS_0|PMU_VDCR_BORS_1|PMU_VDCR_BORS_2
#define PMU_BOR_SET_2V41                PMU_VDCR_BORS_3
#define PMU_BOR_SET_2V51                PMU_VDCR_BORS_0|PMU_VDCR_BORS_3
#define PMU_BOR_SET_2V61                PMU_VDCR_BORS_1|PMU_VDCR_BORS_3
#define PMU_BOR_SET_2V72                PMU_VDCR_BORS_0|PMU_VDCR_BORS_1|PMU_VDCR_BORS_3
#define PMU_BOR_SET_2V82                PMU_VDCR_BORS_2|PMU_VDCR_BORS_3
#define PMU_BOR_SET_2V92                PMU_VDCR_BORS_0|PMU_VDCR_BORS_2|PMU_VDCR_BORS_3
#define PMU_BOR_SET_3V02                PMU_VDCR_BORS_1|PMU_VDCR_BORS_2|PMU_VDCR_BORS_3
#define PMU_BOR_SET_3V12                PMU_VDCR_BORS

/**
  * @}
  */

/** @defgroup PMU_PDR_SET_TRIG_POINT PMU PDR SET TRIG POINT
  * @{
  */
#define PMU_PDR_SET_RISING_TRIG_1V70    0x00000000U
#define PMU_PDR_SET_RISING_TRIG_1V80    PMU_VDCR_PDRS_0
#define PMU_PDR_SET_RISING_TRIG_1V90    PMU_VDCR_PDRS_1
#define PMU_PDR_SET_RISING_TRIG_2V00    PMU_VDCR_PDRS

#define PMU_PDR_SET_FALLING_TRIG_1V58   0x00000000U
#define PMU_PDR_SET_FALLING_TRIG_1V68   PMU_VDCR_PDRS_0
#define PMU_PDR_SET_FALLING_TRIG_1V78   PMU_VDCR_PDRS_1
#define PMU_PDR_SET_FALLING_TRIG_1V87   PMU_VDCR_PDRS

/**
  * @}
  */
  
/** @defgroup PMU_BAT_SEL PMU BAT SEL
  * @{
  */
#define PMU_NO_EXT_BAT_POWER            0x00000000U
#define PMU_EXT_BAT_POWER               PMU_SASR_BAT_SEL

/**
  * @}
  */
    
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @brief  PMU LVD SET
  * @param  __VALUE__ Set LVD trigger voltage range.
  *         This parameter can be a value of @ref PMU_LVD_SET
  * @retval none
  */
#define __HAL_PMU_LVD_SET(__VALUE__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->VDCR,PMU_VDCR_LVDS_Msk,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU LVD FILTER ENABLE
  * @retval none
  */
#define __HAL_PMU_LVD_FILTER_ENABLE()         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->VDCR,PMU_VDCR_LVD_FILTER_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
/** @brief  PMU LVD FILTER DISABLE
  * @retval none
  */
#define __HAL_PMU_LVD_FILTER_DISABLE()        do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            CLEAR_BIT(PMU->VDCR,PMU_VDCR_LVD_FILTER_EN);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)
   
/** @brief  PMU BOR SET
  * @param  __VALUE__ Set BOR trigger voltage range.
  *         This parameter can be a value of @ref PMU_BOR_SET
  * @retval none
  */
#define __HAL_PMU_BOR_SET(__VALUE__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->VDCR,PMU_VDCR_BORS_Msk,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU PDR SET
  * @param  __VALUE__ Set PDR trigger voltage range.
  *         This parameter can be a value of @ref PMU_PDR_SET_TRIG_POINT
  * @retval none
  */
#define __HAL_PMU_PDR_SET(__VALUE__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->VDCR,PMU_VDCR_PDRS_Msk,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)     

                                            
/** @brief  LVD RST PMU SET
  * @param  __SWITCH__ Reset the PMU state machine.
  *         This parameter can be a value of @ref LVD_RST_PMU_EN_SET
  * @retval none
  */
#define __HAL_LVD_RST_PMU_SET(__SWITCH__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->SASR,PMU_SASR_LVD_PMU_EN_Msk,__SWITCH__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                                
/** @brief  BOR RST PMU SET
  * @param  __SWITCH__ Reset the PMU state machine.
  *         This parameter can be a value of @ref BOR_RST_PMU_EN_SET
  * @retval none
  */
#define __HAL_BOR_RST_PMU_SET(__SWITCH__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->SASR,PMU_SASR_BOR_PMU_EN_Msk,__SWITCH__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                                   
                                            
/** @brief  Low voltage warning effective control bit
  * @param  __SWITCH__ Low voltage warning switch.
  *         This parameter can be a value of @ref LOW_VDD_SWITCH
  * @retval none
  */
#define __HAL_PMU_LOW_VDD(__SWITCH__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->SASR,PMU_SASR_EFC_LOW_VDD_EN_Msk,__SWITCH__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)   
                                                                                                                                   
/** @brief  PMU BAT SEL
  * @param  __VALUE__ Set Select BAT.
  *         This parameter can be a value of @ref PMU_BAT_SEL
  * @retval None
  */
#define __HAL_PMU_BAT_SEL(__VALUE__)         do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->SASR,PMU_SASR_BAT_SEL_Msk,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU TAMPER PIN SEL
  * @param  __VALUE__ Set Select TAMPER PIN.
  *         This parameter can be a value of @ref PMU_TAMPERPIN_SEL
  * @retval None
  */
#define __HAL_PMU_TAMPERPIN_SEL(__VALUE__)  do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->SASR,PMU_SASR_TAMPERPIN_SEL_Msk,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU WKE SEL
  * @param  __VALUE__ Set Select WKE.
  *         This parameter can be a value of @ref PMU_PDWKCR
  * @retval None
  */
#define __HAL_PMU_WKE_SEL(__VALUE__)        do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->PDWKCR,0xFFFF,__VALUE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  Set PMU mode.
  * @param  __MODE__ specifies the mode to check.
  *           This parameter can be one of the following values:
  *            @arg PMU_RUN_MODE: The system is in normal operating mode.  
  *            @arg PMU_STOP_MODE: The system is in stop mode.    
  *            @arg PMU_STANDBY_MODE: The system is in Standby mode.
  *            @arg PMU_DEEPSTANDBY_MODE: The system is in Deep standby mode.  
  * @retval The new state of __MODE__ (TRUE or FALSE).
  */
#define __HAL_SET_PMU_MODE(__MODE__)        do { \
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->MR,0x3,__MODE__);\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)                                           
                                                                                        
/** @brief  Check PMU mode is set or not.
  * @param  __MODE__ specifies the mode to check.
  *           This parameter can be one of the following values:
  *            @arg PMU_RUN_MODE: The system is in normal operating mode.  
  *            @arg PMU_STOP_MODE: The system is in stop mode.    
  *            @arg PMU_STANDBY_MODE: The system is in Standby mode.
  *            @arg PMU_DEEPSTANDBY_MODE: The system is in Deep standby mode.  
  * @retval The new state of __MODE__ (TRUE or FALSE).
  */
#define __HAL_GET_PMU_MODE(__MODE__)        ((PMU->MR & (__MODE__)) == (__MODE__))
                                                

/** @brief  Check PMU flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *           This parameter can be one of the following values:
  *            @arg PMU_FLAG_LVD_RST: LVD resets the status flag bit
  *            @arg PMU_FLAG_BOR_RST: BOR resets the status flag bit
  *            @arg PMU_FLAG_XTL_RSTN: XTL clock exception status flag bit
  *            @arg PMU_FLAG_FCORE_PDN_RST: CORE power failure reset status flag bit                                         
  *            @arg PMU_FLAG_PDR_RST: PDR reset status flag bit
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_PWR_GET_RST_FLAG(__FLAG__)    ((PMU->SASR & (__FLAG__)) == (__FLAG__))


/** @brief  LVD interrupt flag bit.
  * @param  None
  * @retval The new state (TRUE or FALSE).
  */
#define __HAL_PWR_GET_LVD_INT_FLAG()    ((EFC->INTSTATUS & (EFC_INTSTATUS_VDDLS_Msk)) == (EFC_INTSTATUS_VDDLS_Msk))


/** @brief  Clear LVD interrupt flag.
  * @param  None
  * @retval None
  */
#define __HAL_PWR_CLEAR_LVD_INT_FLAG()    SET_BIT(EFC->INTSTATUS ,EFC_INTSTATUS_VDDLS_Msk)


/** @brief  Clear the PMU all rst flags.
  * @retval None
  */
#define __HAL_PWR_CLEAR_ALL_RST_FLAG()      do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            SET_BIT(PMU->SASR,PMU_SASR_RST_FLAG_CLR);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  Check PMU flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *           This parameter can be one of the following values:
  *            @arg PMU_FLAG_RESETN_WK: External RESETN reset wake status flag
  *            @arg PMU_FLAG_PC13_WK: PC13 Wake up status bit
  *            @arg PMU_FLAG_PC3_WK: PC3 Wake up status bit
  *            @arg PMU_FLAG_PC2_WK: PC2 Wake up status bit                                      
  *            @arg PMU_FLAG_PC0_WK: PC0 Wake up status bit
  *            @arg PMU_FLAG_PA2_WK: PA2 Wake up status bit                                      
  *            @arg PMU_FLAG_PA0_WK: PA0 Wake up status bit
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_PWR_GET_WK_FLAG(__FLAG__) ((PMU->SASR & (__FLAG__)) == (__FLAG__))



/** @brief  Clear the PMU IO(PA0/PA2/PC0/PC2/PC3/PC13) WK flags.
  * @retval none
  */
#define __HAL_PWR_CLEAR_ALL_IO_WK_FLAG()      do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            MODIFY_REG(PMU->MR, PMU_PDWKCR_PA0_WKE|PMU_PDWKCR_PA2_WKE|PMU_PDWKCR_PC0_WKE|\
                                            PMU_PDWKCR_PC2_WKE|PMU_PDWKCR_PC3_WKE|PMU_PDWKCR_PC13_WKE, 0x00U);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU configure stop mode
  * @retval none
  */
#define __HAL_PMU_CONFIG_STOPMODE()     do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            /* Keep the same system clock after Stop mode wake up */\
                                            SET_BIT(PMU->MR, PMU_MR_STOP_CLK_SEL);\
                                            /* Config PMU Mode:  Stop Mode*/\
                                            MODIFY_REG(PMU->MR, PMU_MR_PMU_MODE, PMU_STOP_MODE);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU configure standby mode 0
  * @retval none
  */
#define __HAL_PMU_CONFIG_STANDBY0()     do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            /* Config PMU mode:  Standby Mode*/\
                                            MODIFY_REG(PMU->MR, PMU_MR_PMU_MODE, PMU_STANDBY_MODE);\
                                            /* BKSRAM��IWDT��LPUART��LPTimer 0~1 keep power on */\
                                            CLEAR_BIT(PMU->MR, PMU_MR_BKSRAMOFF);\
                                            /* Standby mode valid */\
                                            SET_BIT(PMU->MR, PMU_MR_STDBY_EN);\
                                            /* Flash Power Down */\
                                            WRITE_REG(EFC->LPCR, 0xa5000001);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU configure standby mode 1
  * @retval none
  */
#define __HAL_PMU_CONFIG_STANDBY1()     do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            /* Config PMU mode:  Standby Mode*/\
                                            MODIFY_REG(PMU->MR, PMU_MR_PMU_MODE, PMU_STANDBY_MODE);\
                                            /* BKSRAM��IWDT��LPUART��LPTimer 0~1 keep power down  */\
                                            SET_BIT(PMU->MR, PMU_MR_BKSRAMOFF);\
                                            /* Standby mode valid */\
                                            SET_BIT(PMU->MR, PMU_MR_STDBY_EN);\
                                            /* Flash Power Down */\
                                            WRITE_REG(EFC->LPCR, 0xa5000001);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU configure deep standby mode 0
  * @retval none
  */
#define __HAL_PMU_CONFIG_DEEPSTANDBY0()     do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            /* Config PMU mode:  DeepStandby Mode*/\
                                            MODIFY_REG(PMU->MR, PMU_MR_PMU_MODE, PMU_DEEPSTANDBY_MODE);\
                                            /* BKSRAM��IWDT��LPUART��LPTimer 0~1 keep power on */\
                                            CLEAR_BIT(PMU->MR, PMU_MR_BKSRAMOFF);\
                                            /* Standby mode valid */\
                                            SET_BIT(PMU->MR, PMU_MR_STDBY_EN);\
                                            /* Flash Power Down */\
                                            WRITE_REG(EFC->LPCR, 0xa5000001);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/** @brief  PMU configure deep standby mode 1
  * @retval none
  */
#define __HAL_PMU_CONFIG_DEEPSTANDBY1()     do { \
                                            /* Unlock PMU protection register */\
                                            __HAL_PMU_UNLOCK_REGISTER();\
                                            /* Config PMU mode:  DeepStandby Mode*/\
                                            MODIFY_REG(PMU->MR, PMU_MR_PMU_MODE, PMU_DEEPSTANDBY_MODE);\
                                            /* BKSRAM��IWDT��LPUART��LPTimer 0~1 keep power down  */\
                                            SET_BIT(PMU->MR, PMU_MR_BKSRAMOFF);\
                                            /* Standby mode valid */\
                                            SET_BIT(PMU->MR, PMU_MR_STDBY_EN);\
                                            /* Flash Power Down */\
                                            WRITE_REG(EFC->LPCR, 0xa5000001);\
                                            /* Lock PMU protection register */\
                                            __HAL_PMU_LOCK_REGISTER();\
                                            } while(0U)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PMU_Exported_Functions
  * @{
  */ 
void HAL_PMU_EnterSLEEPMode(uint8_t SLEEPEntry);
void HAL_PMU_EnterSTOPMode(uint8_t STOPEntry);
void HAL_PMU_EnterSTANDBY0Mode(void);
void HAL_PMU_EnterSTANDBY1Mode(void);
void HAL_PMU_EnterDEEPSTANDBY0Mode(void);
void HAL_PMU_EnterDEEPSTANDBY1Mode(void);

/**
  * @brief Configure wake up IO in standby mode.
  * @param WK_IO Select the four IO types to wake up in standby mode
  *          This parameter can be one of the following values:
  *            @arg PMU_STANDBY_WK_PC13: In standby mode, PC13 wakes up
  *            @arg PMU_STANDBY_WK_PC3: In standby mode, PC3 wakes up
  *            @arg PMU_STANDBY_WK_PC2: In standby mode, PC2 wakes up
  *            @arg PMU_STANDBY_WK_PC0: In standby mode, PC0 wakes up
  *            @arg PMU_STANDBY_WK_PA2: In standby mode, PA2 wakes up
  *            @arg PMU_STANDBY_WK_PA0: In standby mode, PA0 wakes up 
  * @param Trigger Select rising or falling edge trigger
  *         This parameter can be one of the following values:
  *            @arg RISINGEDGE: rising edge trigger
  *            @arg FALLINGEDGE: falling edge trigger
  * @retval None 
  */
void HAL_PMU_Enable_StandbyMode_WakeUpPin(uint32_t WakeUpPinx, uint8_t WakeUpPolarity);

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
void HAL_PMU_Disable_StandbyMode_WakeUpPin(uint32_t WK_IO);

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
void HAL_PMU_Enable_StandbyMode_WakeUpEvent(uint32_t WK_EVENT);

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
void HAL_PMU_Disable_StandbyMode_WakeUpEvent(uint32_t WK_EVENT);

/**
  * @brief Enables CORTEX M4 SEVONPEND bit. 
  * @note Sets SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.
  * @retval None
  */
void HAL_PWR_EnableSEVOnPend(void);

/**
  * @brief Disables CORTEX M4 SEVONPEND bit. 
  * @note Clears SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.         
  * @retval None
  */
void HAL_PWR_DisableSEVOnPend(void);

/**
  * @brief PMU LVD Init.
  * @param hlvd: Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
HAL_StatusTypeDef HAL_PMU_LVD_Init(PMU_HandleTypeDef *hlvd);

/**
  * @brief  PMU LVD MSP Init.
  * @param  hlvd  Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
void HAL_PMU_LVD_MspInit(PMU_HandleTypeDef *hlvd);


/**
  * @brief  PMU LVD MSP DeInit.
  * @param  hlvd  Pointer to a PMU_HandleTypeDef structure that contains
  *                the configuration information for the specified LVD module.
  * @retval None
  */
void HAL_PMU_LVD_MspDeInit(PMU_HandleTypeDef *hlvd);

/**
  * @brief  Reference interrupt function
  * @param  hlvd Pointer to the LVD_HandleTypeDef structure that contains 
  *				 configuration information for the specified LVD module
  * @retval None
  */
void HAL_LVD_IRQHandler(PMU_HandleTypeDef *hlvd);
    
void HAL_LVD_Callback(PMU_HandleTypeDef *hlvd);

void HAL_PMU_EXTIStopMode_EnableWakeUpEXTI(uint32_t WakeUpEXTIx, uint8_t WakeUpPolarity);
/* Private macros ------------------------------------------------------------*/
/** @defgroup PMU_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup PMU_Private_Functions PMU Private Functions
  * @{
  */  
   
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/




