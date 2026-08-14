 /**
  ******************************************************************************
  * @file     um324xx_hal_adc.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-19  
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
#ifndef __UM324XX_HAL_ADC_H__
#define __UM324XX_HAL_ADC_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup ADC
  * @{
  */

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup ADC_Exported_typedefs ADC Exported Typedefs
  * @{
  */ 

/** 
  * @brief  Structure definition of ADC and regular group initialization 
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (affects regular and injected groups): ClockPrescaler, Resolution, ScanConvMode, DataAlign, ScanConvMode, EOCSelection, LowPowerAutoWait, LowPowerAutoPowerOff, ChannelsBank.
  *          - Scope regular group: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge, ExternalTrigConv.
  * @note   The setting of these parameters with function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all parameters except 'Resolution', 'ScanConvMode', 'DiscontinuousConvMode', 'NbrOfDiscConversion' : ADC enabled without conversion on going on regular group.
  *          - For parameters 'ExternalTrigConv' and 'ExternalTrigConvEdge': ADC enabled, even with conversion on going.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fulfills the ADC state condition) on the fly).
  */
typedef struct
{
  uint32_t ClockPrescaler;               /*!< Select ADC clock prescaler. The clock is common for 
                                              all the ADCs.
                                              This parameter can be a value of 1 ~ 65535 */
  FunctionalState Opamp_en;    			 /*!< Specifies whether the opa connect to ADC.
                                              This parameter can be set to ENABLE or DISABLE. */	
  FunctionalState ContinuousConvMode;    /*!< Specifies whether the conversion is performed in single mode (one conversion) or continuous mode for regular group,
                                              after the selected trigger occurred (software start or external trigger).
                                              This parameter can be set to ENABLE or DISABLE. */
  uint32_t DualRglConvMode;              /*!< Specifies whether the conversion is performed in single mode (one conversion) or Dual mode for regular group,
                                              This parameter can be a value of @ref ADC_dual_regular_conversion_mode */
  uint32_t NbrOfConversion;              /*!< Specifies the number of ranks that will be converted within the regular group sequencer.
                                              To use regular group sequencer and convert several ranks, parameter 'ScanConvMode' must be enabled.
                                              This parameter must be a number between Min_Data = 1 and Max_Data = 16. */
  FunctionalState DiscontinuousConvMode; /*!< Specifies whether the conversions sequence of regular group is performed in Complete-sequence/Discontinuous-sequence (main sequence subdivided in successive parts).
                                              Discontinuous mode is used only if sequencer is enabled (parameter 'ScanConvMode'). If sequencer is disabled, this parameter is discarded.
                                              Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded.
                                              This parameter can be set to ENABLE or DISABLE. */
  uint32_t NbrOfDiscConversion;          /*!< Specifies the number of discontinuous conversions in which the  main sequence of regular group (parameter NbrOfConversion) will be subdivided.
                                              If parameter 'DiscontinuousConvMode' is disabled, this parameter is discarded.
                                              This parameter must be a number between Min_Data = 1 and Max_Data = 8. */
  uint32_t ExternalTrigConv;             /*!< Selects the external event used to trigger the conversion start of regular group.
                                              If set to ADC_SOFTWARE_START, external triggers are disabled.
                                              If set to external trigger source, triggering is on event rising edge by default.
                                              This parameter can be a value of @ref ADC_External_trigger_Source_Regular */
  uint32_t ExternalTrigConvEdge;         /*!< Selects the external trigger edge of regular group.
                                              If trigger is set to ADC_SOFTWARE_START, this parameter is discarded.
                                              This parameter can be a value of @ref ADC_External_trigger_edge_Regular */
}ADC_InitTypeDef;



/** 
  * @brief  Structure definition of ADC channel for regular group   
  * @note   The setting of these parameters with function HAL_ADC_ConfigChannel() is conditioned to ADC state.
  *         ADC can be either disabled or enabled without conversion on going on regular group.
  */ 
typedef struct 
{
  uint32_t Channel;                /*!< Specifies the channel to configure into ADC regular group.
                                        This parameter can be a value of @ref ADC_channels */
  uint32_t Rank;                   /*!< Specifies the rank in the regular group sequencer.
                                        This parameter must be a number between Min_Data = 1 and Max_Data = 20 */
  uint32_t AverageTimes;           /*!< Sampling Average times value to be set for the selected channel.*/
}ADC_ChannelConfTypeDef;

/** 
  * @brief ADC Configuration multi-mode structure definition  
  */ 
typedef struct
{
  uint32_t WatchdogITMode;    /*!< Configures the ADC analog watchdog IT mode.
                                   This parameter can be a value of @ref ADC_interrupts_definition */
  uint32_t HighThreshold;     /*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value. */     
  uint32_t LowThreshold;      /*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value. */
  uint32_t Channel;           /*!< Configures ADC channel for the analog watchdog. 
                                   This parameter has an effect only if watchdog mode is configured on single channel 
                                   This parameter can be a value of @ref ADC_channels */      
  FunctionalState ITMode;     /*!< Specifies whether the analog watchdog is configured
                                   is interrupt mode or in polling mode.
                                   This parameter can be set to ENABLE or DISABLE */
}ADC_AnalogWDGConfTypeDef;

/** 
  * @brief  ADC Configuration injected Channel structure definition
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope channel: InjectedChannel, InjectedRank, InjectedSamplingTime, InjectedOffset
  *          - Scope injected group (affects all channels of injected group): InjectedNbrOfConversion, InjectedDiscontinuousConvMode,
  *            AutoInjectedConv, ExternalTrigInjecConvEdge, ExternalTrigInjecConv.
  * @note   The setting of these parameters with function HAL_ADCEx_InjectedConfigChannel() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all except parameters 'InjectedDiscontinuousConvMode' and 'AutoInjectedConv': ADC enabled without conversion on going on injected group.
  *          - For parameters 'ExternalTrigInjecConv' and 'ExternalTrigInjecConvEdge': ADC enabled, even with conversion on going on injected group.
  */
typedef struct 
{
  uint32_t InjectedChannel;                      /*!< Selection of ADC channel to configure
                                                      This parameter can be a value of @ref ADC_channels
                                                      Note: Depending on devices, some channels may not be available on package pins. Refer to device datasheet for channels availability. */
  uint32_t InjectedRank;                         /*!< Rank in the injected group sequencer
                                                      This parameter must be a value of number
                                                      Note: In case of need to disable a channel or change order of conversion sequencer, rank containing a previous channel setting can be overwritten by the new channel setting (or parameter number of conversions can be adjusted) */
  uint32_t AverageTimes;           				 /*!< Sampling Average times value to be set for the selected channel.*/
  uint32_t InjectedNbrOfConversion;              /*!< Specifies the number of ranks that will be converted within the injected group sequencer.
                                                      To use the injected group sequencer and convert several ranks, parameter 'ScanConvMode' must be enabled.
                                                      This parameter must be a number between Min_Data = 1 and Max_Data = 4.
                                                      Caution: this setting impacts the entire injected group. Therefore, call of HAL_ADCEx_InjectedConfigChannel() to 
                                                               configure a channel on injected group can impact the configuration of other channels previously set. */
  uint32_t ExternalTrigInjecConv;                /*!< Selects the external event used to trigger the conversion start of injected group.
                                                      If set to ADC_INJECTED_SOFTWARE_START, external triggers are disabled.
                                                      If set to external trigger source, triggering is on event rising edge.
                                                      This parameter can be a value of @ref ADC_External_trigger_Source_Injection
                                                      Note: This parameter must be modified when ADC is disabled (before ADC start conversion or after ADC stop conversion).
                                                            If ADC is enabled, this parameter setting is bypassed without error reporting (as it can be the expected behaviour in case of another parameter update on the fly)
                                                      Caution: this setting impacts the entire injected group. Therefore, call of HAL_ADCEx_InjectedConfigChannel() to
                                                               configure a channel on injected group can impact the configuration of other channels previously set. */
  uint32_t ExternalTrigInjecConvEdge;            /*!< Selects the external trigger edge of injected group.
                                                      This parameter can be a value of @ref ADC_External_trigger_edge_Injected. 
                                                      If trigger is set to ADC_INJECTED_SOFTWARE_START, this parameter is discarded.
                                                      Caution: this setting impacts the entire injected group. Therefore, call of HAL_ADCEx_InjectedConfigChannel() to 
                                                               configure a channel on injected group can impact the configuration of other channels previously set. */
}ADC_InjectionConfTypeDef;

/** 
  * @brief  ADC Configuration injected Channel structure definition
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope channel: InjectedChannel, InjectedRank, InjectedSamplingTime, InjectedOffset
  *          - Scope injected group (affects all channels of injected group): InjectedNbrOfConversion, InjectedDiscontinuousConvMode,
  *            AutoInjectedConv, ExternalTrigInjecConvEdge, ExternalTrigInjecConv.
  * @note   The setting of these parameters with function HAL_ADCEx_InjectedConfigChannel() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all except parameters 'InjectedDiscontinuousConvMode' and 'AutoInjectedConv': ADC enabled without conversion on going on injected group.
  *          - For parameters 'ExternalTrigInjecConv' and 'ExternalTrigInjecConvEdge': ADC enabled, even with conversion on going on injected group.
  */
typedef struct 
{
  uint32_t DifferentialChannel;                  /*!< Selection of ADC Differential channel 
                                                      This parameter can be a value of @ref ADC_Differential_channel */
  FunctionalState DifferentialConvMode;          /*!< Specifies whether the Differential Mode is enable.
                                                      This parameter can be set to ENABLE or DISABLE */
}ADC_DifferentialConfTypeDef;

/** 
  * @brief ADC Configuration multi-mode structure definition  
  */ 
typedef struct
{
  uint32_t Mode;              /*!< Configures the ADC to operate in independent or multi mode. */
	
  uint32_t DMAAccessMode;     /*!< Configures the Direct memory access mode for multi ADC mode.*/
                                   
  uint32_t TwoSamplingDelay;  /*!< Configures the Delay between 2 sampling phases.*/
                                   
}ADC_MultiModeTypeDef;

/** 
  * @brief  HAL ADC state machine: ADC states definition (bitfields)
  */ 
/* States of ADC global scope */
#define HAL_ADC_STATE_RESET             0x00000000U    /*!< ADC not yet initialized or disabled */
#define HAL_ADC_STATE_READY             0x00000001U    /*!< ADC peripheral ready for use */
#define HAL_ADC_STATE_BUSY_INTERNAL     0x00000002U    /*!< ADC is busy to internal process (initialization, calibration) */
#define HAL_ADC_STATE_TIMEOUT           0x00000004U    /*!< TimeOut occurrence */

/* States of ADC errors */
#define HAL_ADC_STATE_ERROR_INTERNAL    0x00000010U    /*!< Internal error occurrence */
#define HAL_ADC_STATE_ERROR_CONFIG      0x00000020U    /*!< Configuration error occurrence */
#define HAL_ADC_STATE_ERROR_DMA         0x00000040U    /*!< DMA error occurrence */

/* States of ADC group regular */
#define HAL_ADC_STATE_REG_BUSY          0x00000100U    /*!< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                            external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
#define HAL_ADC_STATE_REG_EOC           0x00000200U    /*!< Conversion data available on group regular */
#define HAL_ADC_STATE_REG_OVR           0x00000400U    /*!< Overrun occurrence */

/* States of ADC group injected */
#define HAL_ADC_STATE_INJ_BUSY          0x00001000U    /*!< A conversion on group injected is ongoing or can occur (either by auto-injection mode,
                                                            external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
#define HAL_ADC_STATE_INJ_EOC           0x00002000U    /*!< Conversion data available on group injected */

/* States of ADC analog watchdogs */
#define HAL_ADC_STATE_AWD_RGL           0x00010000U    /*!< Out-of-window occurrence of regular analog watchdog */
#define HAL_ADC_STATE_AWD_INJ           0x00020000U    /*!< Out-of-window occurrence of injected analog watchdog */
#define HAL_ADC_STATE_AWD_CH            0x00040000U    /*!< Out-of-window occurrence of channel analog watchdog */


/** 
  * @brief  ADC handle Structure definition
  */ 
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
typedef struct __ADC_HandleTypeDef
#else
typedef struct
#endif
{
  ADC_TypeDef                   *Instance;                   /*!< Register base address */

  ADC_InitTypeDef               Init;                        /*!< ADC required parameters */

  __IO uint32_t                 NbrOfCurrentConversionRank;  /*!< ADC number of current conversion rank */

  DMA_HandleTypeDef             *DMA_Handle;                 /*!< Pointer DMA Handler */

  HAL_LockTypeDef               Lock;                        /*!< ADC locking object */

  __IO uint32_t                 State;                       /*!< ADC communication state */

  __IO uint32_t                 ErrorCode;                   /*!< ADC Error code */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallback)(struct __ADC_HandleTypeDef *hadc);              /*!< ADC conversion complete callback */
  void (* ConvHalfCpltCallback)(struct __ADC_HandleTypeDef *hadc);          /*!< ADC conversion DMA half-transfer callback */
  void (* LevelOutOfWindowCallback)(struct __ADC_HandleTypeDef *hadc);      /*!< ADC analog watchdog callback */
  void (* ErrorCallback)(struct __ADC_HandleTypeDef *hadc);                 /*!< ADC error callback */
  void (* InjectedConvCpltCallback)(struct __ADC_HandleTypeDef *hadc);      /*!< ADC group injected conversion complete callback */
  void (* MspInitCallback)(struct __ADC_HandleTypeDef *hadc);               /*!< ADC Msp Init callback */
  void (* MspDeInitCallback)(struct __ADC_HandleTypeDef *hadc);             /*!< ADC Msp DeInit callback */
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}ADC_HandleTypeDef;

#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL ADC Callback ID enumeration definition
  */
typedef enum
{
  HAL_ADC_CONVERSION_COMPLETE_CB_ID     = 0x00U,  /*!< ADC conversion complete callback ID */
  HAL_ADC_LEVEL_OUT_OF_WINDOW_CB_ID     = 0x02U,  /*!< ADC analog watchdog callback ID */
  HAL_ADC_ERROR_CB_ID                   = 0x03U,  /*!< ADC error callback ID */
  HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID = 0x04U,  /*!< ADC group injected conversion complete callback ID */
  HAL_ADC_MSPINIT_CB_ID                 = 0x05U,  /*!< ADC Msp Init callback ID          */
  HAL_ADC_MSPDEINIT_CB_ID               = 0x06U   /*!< ADC Msp DeInit callback ID        */
} HAL_ADC_CallbackIDTypeDef;

/**
  * @brief  HAL ADC Callback pointer definition
  */
typedef  void (*pADC_CallbackTypeDef)(ADC_HandleTypeDef *hadc); /*!< pointer to a ADC callback function */

#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

/** @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @{
  */ 
#define ADC_EXTERNALTRIGCONVEDGE_NONE           0x00000000U
#define ADC_EXTERNALTRIGCONVEDGE_RISING         ((uint32_t)ADC_DGCTRL_RGL_TRIG_EDG_0)
#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ((uint32_t)ADC_DGCTRL_RGL_TRIG_EDG_1)
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ((uint32_t)(ADC_DGCTRL_RGL_TRIG_EDG_0 | ADC_DGCTRL_RGL_TRIG_EDG_1))
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_edge_Injected ADC External Trigger Edge Injected
  * @{
  */ 
#define ADC_EXTERNALTRIGINJCONVEDGE_NONE           0x00000000U
#define ADC_EXTERNALTRIGINJCONVEDGE_RISING         ((uint32_t)ADC_DGCTRL_INJ_TRIG_EDG_0)
#define ADC_EXTERNALTRIGINJCONVEDGE_FALLING        ((uint32_t)ADC_DGCTRL_INJ_TRIG_EDG_1)
#define ADC_EXTERNALTRIGINJCONVEDGE_RISINGFALLING  ((uint32_t)(ADC_DGCTRL_INJ_TRIG_EDG_0 | ADC_DGCTRL_INJ_TRIG_EDG_1))
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @{
  */
/* Note: Parameter ADC_SOFTWARE_START is a software parameter */
#define ADC_RGLEXTERNALTRIGCONV_T0_CC1    0x00000000U
#define ADC_RGLEXTERNALTRIGCONV_T0_CC2    ((uint32_t)ADC_DGCTRL_RGL_TRIG_SEL_0)
#define ADC_RGLEXTERNALTRIGCONV_T0_CC3    ((uint32_t)ADC_DGCTRL_RGL_TRIG_SEL_1)
#define ADC_RGLEXTERNALTRIGCONV_T1_CC2    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_0 | ADC_DGCTRL_RGL_TRIG_SEL_1))
#define ADC_RGLEXTERNALTRIGCONV_T1_CC3    ((uint32_t)ADC_DGCTRL_RGL_TRIG_SEL_2)
#define ADC_RGLEXTERNALTRIGCONV_T1_CC4    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_T1_TRGO   ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_1))
#define ADC_RGLEXTERNALTRIGCONV_T2_CC1    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_1 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_T2_TRGO   ((uint32_t)ADC_DGCTRL_RGL_TRIG_SEL_3)
#define ADC_RGLEXTERNALTRIGCONV_T3_CC4    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_T4_CC1    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_1))
#define ADC_RGLEXTERNALTRIGCONV_T4_CC2    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_1 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_T4_CC3    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_2))
#define ADC_RGLEXTERNALTRIGCONV_T7_CC1    ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_T7_TRGO   ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_1))
#define ADC_RGLEXTERNALTRIGCONV_Ext_IT11  ((uint32_t)(ADC_DGCTRL_RGL_TRIG_SEL_3 | ADC_DGCTRL_RGL_TRIG_SEL_2 | ADC_DGCTRL_RGL_TRIG_SEL_1 | ADC_DGCTRL_RGL_TRIG_SEL_0))
#define ADC_SOFTWARE_START                ((uint32_t)ADC_DGCTRL_RGL_TRIG_SEL + 1U)

#define ADC_RGLEXTERNALTRIGCONV_Ext_IT11_syscfg       (0x00000000U)
#define ADC_RGLEXTERNALTRIGCONV_Ext_IT10_syscfg        ((uint32_t)SYSCFG_ADCETSR_RGL_TRIG_SEL_0)
#define ADC_RGLEXTERNALTRIGCONV_Ext_IT6_syscfg        ((uint32_t)SYSCFG_ADCETSR_RGL_TRIG_SEL_1)
#define ADC_RGLEXTERNALTRIGCONV_Ext_lptim0out_syscfg  ((uint32_t)(SYSCFG_ADCETSR_RGL_TRIG_SEL_1 | SYSCFG_ADCETSR_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_Ext_lptim1out_syscfg  ((uint32_t)(SYSCFG_ADCETSR_RGL_TRIG_SEL_2))
#define ADC_RGLEXTERNALTRIGCONV_Ext_acmp0out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_RGL_TRIG_SEL_2 | SYSCFG_ADCETSR_RGL_TRIG_SEL_0))
#define ADC_RGLEXTERNALTRIGCONV_Ext_acmp1out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_RGL_TRIG_SEL_2 | SYSCFG_ADCETSR_RGL_TRIG_SEL_1))
#define ADC_RGLEXTERNALTRIGCONV_Ext_acmp2out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_RGL_TRIG_SEL_2 | SYSCFG_ADCETSR_RGL_TRIG_SEL_1 | SYSCFG_ADCETSR_RGL_TRIG_SEL_0))
/**
  * @}
  */

/** @defgroup ADC_External_trigger_Source_Injection ADC External Trigger Source Injection
  * @{
  */
#define ADC_INJEXTERNALTRIGCONV_T0_CC4    0x00000000U
#define ADC_INJEXTERNALTRIGCONV_T0_TRGO   ((uint32_t)ADC_DGCTRL_INJ_TRIG_SEL_0)
#define ADC_INJEXTERNALTRIGCONV_T1_CC1    ((uint32_t)ADC_DGCTRL_INJ_TRIG_SEL_1)
#define ADC_INJEXTERNALTRIGCONV_T1_TRGO   ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_0 | ADC_DGCTRL_INJ_TRIG_SEL_1))
#define ADC_INJEXTERNALTRIGCONV_T2_CC2    ((uint32_t)ADC_DGCTRL_INJ_TRIG_SEL_2)
#define ADC_INJEXTERNALTRIGCONV_T2_CC4    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_T3_CC1    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_1))
#define ADC_INJEXTERNALTRIGCONV_T3_CC2    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_1 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_T3_CC3    ((uint32_t)ADC_DGCTRL_INJ_TRIG_SEL_3)
#define ADC_INJEXTERNALTRIGCONV_T3_TRGO   ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_T4_CC4    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_1))
#define ADC_INJEXTERNALTRIGCONV_T4_TRGO   ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_1 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_T7_CC2    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_2))
#define ADC_INJEXTERNALTRIGCONV_T7_CC3    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_T7_CC4    ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_1))
#define ADC_INJEXTERNALTRIGCONV_Ext_IT15  ((uint32_t)(ADC_DGCTRL_INJ_TRIG_SEL_3 | ADC_DGCTRL_INJ_TRIG_SEL_2 | ADC_DGCTRL_INJ_TRIG_SEL_1 | ADC_DGCTRL_INJ_TRIG_SEL_0))
#define ADC_INJECTED_SOFTWARE_START       ((uint32_t)ADC_DGCTRL_INJ_TRIG_SEL + 1U)

#define ADC_INJEXTERNALTRIGCONV_Ext_IT15_syscfg       (0x00000000U)
#define ADC_INJEXTERNALTRIGCONV_Ext_IT9_syscfg        ((uint32_t)SYSCFG_ADCETSR_INJ_TRIG_SEL_0)
#define ADC_INJEXTERNALTRIGCONV_Ext_IT5_syscfg        ((uint32_t)SYSCFG_ADCETSR_INJ_TRIG_SEL_1)
#define ADC_INJEXTERNALTRIGCONV_Ext_lptim0out_syscfg  ((uint32_t)(SYSCFG_ADCETSR_INJ_TRIG_SEL_1 | SYSCFG_ADCETSR_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_Ext_lptim1out_syscfg  ((uint32_t)(SYSCFG_ADCETSR_INJ_TRIG_SEL_2))
#define ADC_INJEXTERNALTRIGCONV_Ext_acmp0out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_INJ_TRIG_SEL_2 | SYSCFG_ADCETSR_INJ_TRIG_SEL_0))
#define ADC_INJEXTERNALTRIGCONV_Ext_acmp1out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_INJ_TRIG_SEL_2 | SYSCFG_ADCETSR_INJ_TRIG_SEL_1))
#define ADC_INJEXTERNALTRIGCONV_Ext_acmp2out_syscfg   ((uint32_t)(SYSCFG_ADCETSR_INJ_TRIG_SEL_2 | SYSCFG_ADCETSR_INJ_TRIG_SEL_1 | SYSCFG_ADCETSR_INJ_TRIG_SEL_0))


/**
  * @}
  */

/** @defgroup ADC_channels  ADC Common Channels
  * @{
  */ 
#define ADC_CHANNEL_0           0x00000000U
#define ADC_CHANNEL_1           ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_0)
#define ADC_CHANNEL_2           ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_1)
#define ADC_CHANNEL_3           ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_1 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_4           ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_2)
#define ADC_CHANNEL_5           ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_6           ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_1))
#define ADC_CHANNEL_7           ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_1 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_8           ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_3)
#define ADC_CHANNEL_9           ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_10          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_1))
#define ADC_CHANNEL_11          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_1 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_12          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_2))
#define ADC_CHANNEL_13          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_14          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_1))
#define ADC_CHANNEL_15          ((uint32_t)(ADC_RGLCHCFG0_RGL_CH1_SEL_3 | ADC_RGLCHCFG0_RGL_CH1_SEL_2 | ADC_RGLCHCFG0_RGL_CH1_SEL_1 | ADC_RGLCHCFG0_RGL_CH1_SEL_0))
#define ADC_CHANNEL_16          ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_4)
#define ADC_CHANNEL_17          ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_4 | ADC_RGLCHCFG0_RGL_CH1_SEL_0)
#define ADC_CHANNEL_18          ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_4 | ADC_RGLCHCFG0_RGL_CH1_SEL_1)
#define ADC_CHANNEL_19          ((uint32_t)ADC_RGLCHCFG0_RGL_CH1_SEL_4 | ADC_RGLCHCFG0_RGL_CH1_SEL_1 | ADC_RGLCHCFG0_RGL_CH1_SEL_0)
/**
  * @}
  */

/** @defgroup ADC_channel_average_times  ADC Channel conversion average times
  * @{
  */ 
#define ADC_CHANNEL_AVERAGE_TIMES_1           0x00000000U
#define ADC_CHANNEL_AVERAGE_TIMES_2           ((uint32_t)ADC_CHAVGCFG0_CH0_AVG_SEL_0)
#define ADC_CHANNEL_AVERAGE_TIMES_4           ((uint32_t)ADC_CHAVGCFG0_CH0_AVG_SEL_1)
#define ADC_CHANNEL_AVERAGE_TIMES_8           ((uint32_t)(ADC_CHAVGCFG0_CH0_AVG_SEL_1 | ADC_CHAVGCFG0_CH0_AVG_SEL_0))
#define ADC_CHANNEL_AVERAGE_TIMES_16          ((uint32_t)ADC_CHAVGCFG0_CH0_AVG_SEL_2)
#define ADC_CHANNEL_AVERAGE_TIMES_32          ((uint32_t)(ADC_CHAVGCFG0_CH0_AVG_SEL_2 | ADC_CHAVGCFG0_CH0_AVG_SEL_0))
#define ADC_CHANNEL_AVERAGE_TIMES_64          ((uint32_t)(ADC_CHAVGCFG0_CH0_AVG_SEL_2 | ADC_CHAVGCFG0_CH0_AVG_SEL_1))
#define ADC_CHANNEL_AVERAGE_TIMES_128         ((uint32_t)(ADC_CHAVGCFG0_CH0_AVG_SEL_2 | ADC_CHAVGCFG0_CH0_AVG_SEL_1 | ADC_CHAVGCFG0_CH0_AVG_SEL_0))
/**
  * @}
  */

/** @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @{
  */ 
#define ADC_IT_WDG_INJ      ((uint32_t)ADC_INTEN_WDG_INJ_EN)
#define ADC_IT_WDG_RGL      ((uint32_t)ADC_INTEN_WDG_RGL_EN)
#define ADC_IT_WDG_CH       ((uint32_t)ADC_INTEN_WDG_CH_EN)
#define ADC_IT_FIFO_OVF     ((uint32_t)ADC_INTEN_FIFO_OVF_EN)
#define ADC_IT_FIFO_AVL     ((uint32_t)ADC_INTEN_FIFO_AVL_EN)
#define ADC_IT_EOIC         ((uint32_t)ADC_INTEN_EOIC_EN)
#define ADC_IT_EORC         ((uint32_t)ADC_INTEN_EORC_EN)
/**
  * @}
  */

/** @defgroup ADC_flags_definition ADC IT Flags Definition
  * @{
  */ 
#define ADC_IT_FLAG_WDG_INJ      ((uint32_t)ADC_INTSTAT_WDG_INJ)
#define ADC_IT_FLAG_WDG_RGL      ((uint32_t)ADC_INTSTAT_WDG_RGL)
#define ADC_IT_FLAG_WDG_CH       ((uint32_t)ADC_INTSTAT_WDG_CH)
#define ADC_IT_FLAG_FIFO_OVF     ((uint32_t)ADC_INTSTAT_FIFO_OVF)
#define ADC_IT_FLAG_FIFO_AVL     ((uint32_t)ADC_INTSTAT_FIFO_AVL)
#define ADC_IT_FLAG_EOIC         ((uint32_t)ADC_INTSTAT_EOIC)
#define ADC_IT_FLAG_EORC         ((uint32_t)ADC_INTSTAT_EORC)
/**
  * @}
  */ 

/** @defgroup ADC_flags_definition ADC IT Flags Definition
  * @{
  */ 
#define ADC_WDG_INJ_EVENT      ((uint32_t)ADC_IT_FLAG_WDG_INJ)
#define ADC_WDG_RGL_EVENT      ((uint32_t)ADC_IT_FLAG_WDG_RGL)
#define ADC_WDG_CH_EVENT       ((uint32_t)ADC_IT_FLAG_WDG_CH)
#define ADC_FIFO_OVF_EVENT     ((uint32_t)ADC_IT_FLAG_FIFO_OVF)
#define ADC_FIFO_AVL_EVENT     ((uint32_t)ADC_IT_FLAG_FIFO_AVL)
#define ADC_EOIC_EVENT         ((uint32_t)ADC_IT_FLAG_EOIC)
#define ADC_EORC_EVENT         ((uint32_t)ADC_IT_FLAG_EORC)
/**
  * @}
  */ 

/** @defgroup ADC_regular_sequence_conversion Length ADC regular sequence conversion Length
  * @{
  * @note   When using DMA mode, the length of regular sequence conversion 
  *         can only be one of the following parameters.
  */ 
#define ADC_REGULAR_LENGTH_1     ((uint32_t)1U)
#define ADC_REGULAR_LENGTH_4     ((uint32_t)4U)
#define ADC_REGULAR_LENGTH_8     ((uint32_t)8U)
#define ADC_REGULAR_LENGTH_16    ((uint32_t)16U)

/**
  * @}
  */ 

/** @defgroup ADC_dual_regular_conversion_mode ADC dual regular conversion mode
  * @{
  */ 
#define ADC_DUALRGL_CONVERSION_INDEPENDENT     	0x00000000U
#define ADC_DUALRGL_CONVERSION_PARALLEL     	((uint32_t)ADC_DGCTRL_DUAL_RGL_MOD_0)
#define ADC_DUALRGL_CONVERSION_DELAY   			((uint32_t)ADC_DGCTRL_DUAL_RGL_MOD_1)
/**
  * @}
  */ 

/** @defgroup ADC_Differential_channel ADC Differential channel 
  * @{
  */ 
   #if defined(UM324xF)
#define ADC_DIFFERRNTIAL_CHANNEL_0		     	((uint32_t)ADC_ANCTRL_DIFF_IN0_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_2		     	((uint32_t)ADC_ANCTRL_DIFF_IN2_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_4		     	((uint32_t)ADC_ANCTRL_DIFF_IN4_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_6		     	((uint32_t)ADC_ANCTRL_DIFF_IN6_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_8		     	((uint32_t)ADC_ANCTRL_DIFF_IN8_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_10		     	((uint32_t)ADC_ANCTRL_DIFF_IN10_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_12		     	((uint32_t)ADC_ANCTRL_DIFF_IN12_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_14		     	((uint32_t)ADC_ANCTRL_DIFF_IN14_EN)
#endif
#if defined(UM32x42x) || defined(UM32x41x)
#define ADC_DIFFERRNTIAL_CHANNEL_0		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_2		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_4		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_6		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_8		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_10		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_12		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#define ADC_DIFFERRNTIAL_CHANNEL_14		     	((uint32_t)ADC_ANCTRL_DIFF_EN)
#endif
/**
  * @}
  */ 

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC_Exported_constants ADC Exported Constants
  * @{
  */ 

/** @defgroup ADC_Error_Code ADC Error Code
  * @{
  */
#define HAL_ADC_ERROR_NONE        0x00U   /*!< No error                                              */
#define HAL_ADC_ERROR_INTERNAL    0x01U   /*!< ADC IP internal error: if problem of clocking, 
                                               enable/disable, erroneous state                       */
#define HAL_ADC_ERROR_OVR         0x02U   /*!< Overrun error                                         */
#define HAL_ADC_ERROR_DMA         0x04U   /*!< DMA transfer error                                    */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
#define HAL_ADC_ERROR_INVALID_CALLBACK  (0x10U)   /*!< Invalid Callback error */
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
/**
  * @}
  */
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup ADC_Private_Constants ADC Private Constants
  * @{
  */
/* Delay for ADC stabilization time.                                        */
/* Maximum delay is 1us (refer to device datasheet, parameter tSTAB).       */
/* Unit: us                                                                 */
#define ADC_STAB_DELAY_US               3U
/* Delay for temperature sensor stabilization time.                         */
/* Maximum delay is 10us (refer to device datasheet, parameter tSTART).     */
/* Unit: us                                                                 */
#define ADC_TEMPSENSOR_DELAY_US         10U
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ADC_Exported_macro ADC Exported Macro
  * @{
  */ 

/**
  * @brief Simultaneously clears and sets specific bits of the handle State
  * @note: ADC_STATE_CLR_SET() macro is merely aliased to generic macro MODIFY_REG(),
  *        the first parameter is the ADC handle State, the second parameter is the
  *        bit field to clear, the third and last parameter is the bit field to set.
  * @retval None
  */
#define ADC_STATE_CLR_SET MODIFY_REG

/**
  * @brief Clear ADC error code (set it to error code: "no error")
  * @param __HANDLE__ ADC handle
  * @retval None
  */
#define ADC_CLEAR_ERRORCODE(__HANDLE__)                                        \
  ((__HANDLE__)->ErrorCode = HAL_ADC_ERROR_NONE)

/**
  * @brief  Enable the ADC peripheral.
  * @param  __HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->DGCTRL |=  ADC_DGCTRL_ADCC_EN)

/**
  * @brief  Disable the ADC peripheral.
  * @param  __HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->DGCTRL &=  ~ADC_DGCTRL_ADCC_EN)

/**
  * @brief  Clear the ADC's IT flags.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __FLAG__ ADC IT flag.
  * @retval None
  */
#define __HAL_ADC_CLEAR_IT_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->INTSTAT) |= (__FLAG__))

/**
  * @brief  Get the selected ADC's IT flag status.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __FLAG__ ADC IT flag.
  * @retval None
  */
#define __HAL_ADC_GET_IT_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->INTSTAT) & (__FLAG__)) == (__FLAG__))

/**
  * @brief  Enable the ADC end of conversion interrupt.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __INTERRUPT__ ADC Interrupt.
  * @retval None
  */
#define __HAL_ADC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->INTEN) |= (__INTERRUPT__))

/**
  * @brief  Disable the ADC end of conversion interrupt.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __INTERRUPT__ ADC interrupt.
  * @retval None
  */
#define __HAL_ADC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->INTEN) &= ~(__INTERRUPT__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_Exported_Functions
  * @{
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup ADC_Private_Macros
  * @{
  */

/**
  * @brief  Set ADC Injected channel sequence length.
  * @param  _NbrOfConversion_ Injected channel sequence length.
  * @retval None
  */
#define ADC_INJ_LENG(_InjectedNbrOfConversion_) (((_InjectedNbrOfConversion_) - (uint8_t)1U) << 20U)

/**
  * @brief  Set the selected injected Channel rank.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number. 
  * @retval None
  */
#define ADC_INJCHCFG_RK(_CHANNELNB_, _RANKNB_)  (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * (uint8_t)((_RANKNB_) - 1U)))

/**
  * @brief  Set the ADC's average times for channel numbers between 10 and 15.
  * @param  _AVERAGETIMES_ average times parameter.
  * @param  _CHANNELNB_ Channel number.  
  * @retval None
  */
#define ADC_CHAVGCFG1(_AVERAGETIMES_, _CHANNELNB_) ((_AVERAGETIMES_) << (3U * (((uint32_t)((uint16_t)(_CHANNELNB_))) - 10U)))

/**
  * @brief  Set the ADC's average times for channel numbers between 0 and 9.
  * @param  _AVERAGETIMES_ average times parameter.
  * @param  _CHANNELNB_ Channel number.  
  * @retval None
  */
#define ADC_CHAVGCFG0(_AVERAGETIMES_, _CHANNELNB_) ((_AVERAGETIMES_) << (3U * ((uint32_t)((uint16_t)(_CHANNELNB_)))))

/**
  * @brief  Set the selected regular channel rank for rank between 1 and 6.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_RGLCHCFG0_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 1U)))

/**
  * @brief  Set the selected regular channel rank for rank between 7 and 12.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_RGLCHCFG1_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 7U)))

/**
  * @brief  Set the selected regular channel rank for rank between 13 and 18.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_RGLCHCFG2_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 13U)))

/**
  * @brief  Set the selected regular channel rank for rank between 19 and 20.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_RGLCHCFG3_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 13U)))
/**
  * @brief  Set ADC Regular channel sequence length.
  * @param  _NbrOfConversion_ Regular channel sequence length. 
  * @retval None
  */
#define ADC_RGLCHCFG3_LENG(_NbrOfConversion_) (((_NbrOfConversion_) - (uint8_t)1U) << 10U)

/**
  * @brief  Configures the number of discontinuous conversions for the regular group channels.
  * @param  _NBR_DISCONTINUOUSCONV_ Number of discontinuous conversions.
  * @retval None
  */
#define ADC_RGLCHCFG3_SHORTLENG(_NBR_DISCONTINUOUSCONV_) (((_NBR_DISCONTINUOUSCONV_) - 1U) << 15U)

/**
  * @brief  Configures the watchdog highthreshold for the channels.
  * @param  _HIGHTHRESHOLD_ highthreshold of watchdog.
  * @retval None
  */
#define ADC_WDGCOND_HIGHTHRESHOLD(_HIGHTHRESHOLD_) ((_HIGHTHRESHOLD_) << 16U)

/**
  * @brief  Configures the watchdog channels.
  * @param  _CHANNEL_ ADC channel.
  * @retval None
  */
#define ADC_WDGCOND_CHANNEL(_CHANNEL_) (1U << (_CHANNEL_))

/**
  * @}
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */  
  
/** @addtogroup ADC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions ***********************************/
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);

#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
HAL_StatusTypeDef HAL_ADC_RegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID, pADC_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_ADC_UnRegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData,  uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc, uint32_t channel);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions *************************************************/
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions ***************************************************/
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);
/**
  * @}
  */
  
/** @addtogroup ADCEx_Exported_Functions_Group1
  * @{
  */

/* I/O operation functions ******************************************************/
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_DualModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_DualModeStop_DMA(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_DualModeGetValue(ADC_HandleTypeDef* hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef ADC_EXT_RGL_TRIG_SYSCFG(ADC_HandleTypeDef* hadc,uint32_t trig);

/* Peripheral Control functions *************************************************/
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_DifferentialConfigChannel(ADC_HandleTypeDef* hadc, ADC_DifferentialConfTypeDef* sConfigDifferential);

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

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
