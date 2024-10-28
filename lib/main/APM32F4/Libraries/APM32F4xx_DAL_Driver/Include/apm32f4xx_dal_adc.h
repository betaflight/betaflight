/**
  *
  * @file    apm32f4xx_dal_adc.h
  * @brief   Header file containing functions prototypes of ADC DAL library.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_ADC_H
#define APM32F4xx_DAL_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/* Include low level driver */
#include "apm32f4xx_ddl_adc.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup ADC
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/** @defgroup ADC_Exported_Types ADC Exported Types
  * @{
  */

/** 
  * @brief  Structure definition of ADC and regular group initialization 
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (affects regular and injected groups): ClockPrescaler, Resolution, ScanConvMode, DataAlign, ScanConvMode, EOCSelection, LowPowerAutoWait, LowPowerAutoPowerOff, ChannelsBank.
  *          - Scope regular group: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge, ExternalTrigConv.
  * @note   The setting of these parameters with function DAL_ADC_Init() is conditioned to ADC state.
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
                                              This parameter can be a value of @ref ADC_ClockPrescaler */
  uint32_t Resolution;                   /*!< Configures the ADC resolution.
                                              This parameter can be a value of @ref ADC_Resolution */
  uint32_t DataAlign;                    /*!< Specifies ADC data alignment to right (MSB on register bit 11 and LSB on register bit 0) (default setting)
                                              or to left (if regular group: MSB on register bit 15 and LSB on register bit 4, if injected group (MSB kept as signed value due to potential negative value after offset application): MSB on register bit 14 and LSB on register bit 3).
                                              This parameter can be a value of @ref ADC_Data_align */
  uint32_t ScanConvMode;                 /*!< Configures the sequencer of regular and injected groups.
                                              This parameter can be associated to parameter 'DiscontinuousConvMode' to have main sequence subdivided in successive parts.
                                              If disabled: Conversion is performed in single mode (one channel converted, the one defined in rank 1).
                                                           Parameters 'NbrOfConversion' and 'InjectedNbrOfConversion' are discarded (equivalent to set to 1).
                                              If enabled:  Conversions are performed in sequence mode (multiple ranks defined by 'NbrOfConversion'/'InjectedNbrOfConversion' and each channel rank).
                                                           Scan direction is upward: from rank1 to rank 'n'.
                                              This parameter can be set to ENABLE or DISABLE */
  uint32_t EOCSelection;                 /*!< Specifies what EOC (End Of Conversion) flag is used for conversion by polling and interruption: end of conversion of each rank or complete sequence.
                                              This parameter can be a value of @ref ADC_EOCSelection.
                                              Note: For injected group, end of conversion (flag&IT) is raised only at the end of the sequence.
                                                    Therefore, if end of conversion is set to end of each conversion, injected group should not be used with interruption (DAL_ADCEx_InjectedStart_IT)
                                                    or polling (DAL_ADCEx_InjectedStart and DAL_ADCEx_InjectedPollForConversion). By the way, polling is still possible since driver will use an estimated timing for end of injected conversion.
                                              Note: If overrun feature is intended to be used, use ADC in mode 'interruption' (function DAL_ADC_Start_IT() ) with parameter EOCSelection set to end of each conversion or in mode 'transfer by DMA' (function DAL_ADC_Start_DMA()).
                                                    If overrun feature is intended to be bypassed, use ADC in mode 'polling' or 'interruption' with parameter EOCSelection must be set to end of sequence */
  FunctionalState ContinuousConvMode;    /*!< Specifies whether the conversion is performed in single mode (one conversion) or continuous mode for regular group,
                                              after the selected trigger occurred (software start or external trigger).
                                              This parameter can be set to ENABLE or DISABLE. */
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
  FunctionalState DMAContinuousRequests; /*!< Specifies whether the DMA requests are performed in one shot mode (DMA transfer stop when number of conversions is reached)
                                              or in Continuous mode (DMA transfer unlimited, whatever number of conversions).
                                              Note: In continuous mode, DMA must be configured in circular mode. Otherwise an overrun will be triggered when DMA buffer maximum pointer is reached.
                                              Note: This parameter must be modified when no conversion is on going on both regular and injected groups (ADC disabled, or ADC enabled without continuous mode or external trigger that could launch a conversion).
                                              This parameter can be set to ENABLE or DISABLE. */
}ADC_InitTypeDef;



/** 
  * @brief  Structure definition of ADC channel for regular group   
  * @note   The setting of these parameters with function DAL_ADC_ConfigChannel() is conditioned to ADC state.
  *         ADC can be either disabled or enabled without conversion on going on regular group.
  */ 
typedef struct 
{
  uint32_t Channel;                /*!< Specifies the channel to configure into ADC regular group.
                                        This parameter can be a value of @ref ADC_channels */
  uint32_t Rank;                   /*!< Specifies the rank in the regular group sequencer.
                                        This parameter must be a number between Min_Data = 1 and Max_Data = 16 */
  uint32_t SamplingTime;           /*!< Sampling time value to be set for the selected channel.
                                        Unit: ADC clock cycles
                                        Conversion time is the addition of sampling time and processing time (12 ADC clock cycles at ADC resolution 12 bits, 11 cycles at 10 bits, 9 cycles at 8 bits, 7 cycles at 6 bits).
                                        This parameter can be a value of @ref ADC_sampling_times
                                        Caution: This parameter updates the parameter property of the channel, that can be used into regular and/or injected groups.
                                                 If this same channel has been previously configured in the other group (regular/injected), it will be updated to last setting.
                                        Note: In case of usage of internal measurement channels (VrefInt/Vbat/TempSensor),
                                              sampling time constraints must be respected (sampling time can be adjusted in function of ADC clock frequency and sampling time setting)
                                              Refer to device datasheet for timings values, parameters TS_vrefint, TS_temp (values rough order: 4us min). */
  uint32_t Offset;                 /*!< Reserved for future use, can be set to 0 */
}ADC_ChannelConfTypeDef;

/** 
  * @brief ADC Configuration multi-mode structure definition  
  */ 
typedef struct
{
  uint32_t WatchdogMode;      /*!< Configures the ADC analog watchdog mode.
                                   This parameter can be a value of @ref ADC_analog_watchdog_selection */
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
  uint32_t WatchdogNumber;    /*!< Reserved for future use, can be set to 0 */
}ADC_AnalogWDGConfTypeDef;

/** 
  * @brief  DAL ADC state machine: ADC states definition (bitfields)
  */ 
/* States of ADC global scope */
#define DAL_ADC_STATE_RESET             0x00000000U    /*!< ADC not yet initialized or disabled */
#define DAL_ADC_STATE_READY             0x00000001U    /*!< ADC peripheral ready for use */
#define DAL_ADC_STATE_BUSY_INTERNAL     0x00000002U    /*!< ADC is busy to internal process (initialization, calibration) */
#define DAL_ADC_STATE_TIMEOUT           0x00000004U    /*!< TimeOut occurrence */

/* States of ADC errors */
#define DAL_ADC_STATE_ERROR_INTERNAL    0x00000010U    /*!< Internal error occurrence */
#define DAL_ADC_STATE_ERROR_CONFIG      0x00000020U    /*!< Configuration error occurrence */
#define DAL_ADC_STATE_ERROR_DMA         0x00000040U    /*!< DMA error occurrence */

/* States of ADC group regular */
#define DAL_ADC_STATE_REG_BUSY          0x00000100U    /*!< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                            external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
#define DAL_ADC_STATE_REG_EOC           0x00000200U    /*!< Conversion data available on group regular */
#define DAL_ADC_STATE_REG_OVR           0x00000400U    /*!< Overrun occurrence */

/* States of ADC group injected */
#define DAL_ADC_STATE_INJ_BUSY          0x00001000U    /*!< A conversion on group injected is ongoing or can occur (either by auto-injection mode,
                                                            external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
#define DAL_ADC_STATE_INJ_EOC           0x00002000U    /*!< Conversion data available on group injected */

/* States of ADC analog watchdogs */
#define DAL_ADC_STATE_AWD1              0x00010000U    /*!< Out-of-window occurrence of analog watchdog 1 */
#define DAL_ADC_STATE_AWD2              0x00020000U    /*!< Not available on APM32F4 device: Out-of-window occurrence of analog watchdog 2 */
#define DAL_ADC_STATE_AWD3              0x00040000U    /*!< Not available on APM32F4 device: Out-of-window occurrence of analog watchdog 3 */

/* States of ADC multi-mode */
#define DAL_ADC_STATE_MULTIMODE_SLAVE   0x00100000U    /*!< Not available on APM32F4 device: ADC in multimode slave state, controlled by another ADC master ( */


/** 
  * @brief  ADC handle Structure definition
  */ 
#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
typedef struct __ADC_HandleTypeDef
#else
typedef struct
#endif
{
  ADC_TypeDef                   *Instance;                   /*!< Register base address */

  ADC_InitTypeDef               Init;                        /*!< ADC required parameters */

  __IO uint32_t                 NbrOfCurrentConversionRank;  /*!< ADC number of current conversion rank */

  DMA_HandleTypeDef             *DMA_Handle;                 /*!< Pointer DMA Handler */

  DAL_LockTypeDef               Lock;                        /*!< ADC locking object */

  __IO uint32_t                 State;                       /*!< ADC communication state */

  __IO uint32_t                 ErrorCode;                   /*!< ADC Error code */
#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallback)(struct __ADC_HandleTypeDef *hadc);              /*!< ADC conversion complete callback */
  void (* ConvHalfCpltCallback)(struct __ADC_HandleTypeDef *hadc);          /*!< ADC conversion DMA half-transfer callback */
  void (* LevelOutOfWindowCallback)(struct __ADC_HandleTypeDef *hadc);      /*!< ADC analog watchdog 1 callback */
  void (* ErrorCallback)(struct __ADC_HandleTypeDef *hadc);                 /*!< ADC error callback */
  void (* InjectedConvCpltCallback)(struct __ADC_HandleTypeDef *hadc);      /*!< ADC group injected conversion complete callback */
  void (* MspInitCallback)(struct __ADC_HandleTypeDef *hadc);               /*!< ADC Msp Init callback */
  void (* MspDeInitCallback)(struct __ADC_HandleTypeDef *hadc);             /*!< ADC Msp DeInit callback */
#endif /* USE_DAL_ADC_REGISTER_CALLBACKS */
}ADC_HandleTypeDef;

#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL ADC Callback ID enumeration definition
  */
typedef enum
{
  DAL_ADC_CONVERSION_COMPLETE_CB_ID     = 0x00U,  /*!< ADC conversion complete callback ID */
  DAL_ADC_CONVERSION_HALF_CB_ID         = 0x01U,  /*!< ADC conversion DMA half-transfer callback ID */
  DAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID   = 0x02U,  /*!< ADC analog watchdog 1 callback ID */
  DAL_ADC_ERROR_CB_ID                   = 0x03U,  /*!< ADC error callback ID */
  DAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID = 0x04U,  /*!< ADC group injected conversion complete callback ID */
  DAL_ADC_MSPINIT_CB_ID                 = 0x05U,  /*!< ADC Msp Init callback ID          */
  DAL_ADC_MSPDEINIT_CB_ID               = 0x06U   /*!< ADC Msp DeInit callback ID        */
} DAL_ADC_CallbackIDTypeDef;

/**
  * @brief  DAL ADC Callback pointer definition
  */
typedef  void (*pADC_CallbackTypeDef)(ADC_HandleTypeDef *hadc); /*!< pointer to a ADC callback function */

#endif /* USE_DAL_ADC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup ADC_Error_Code ADC Error Code
  * @{
  */
#define DAL_ADC_ERROR_NONE        0x00U   /*!< No error                                              */
#define DAL_ADC_ERROR_INTERNAL    0x01U   /*!< ADC IP internal error: if problem of clocking, 
                                               enable/disable, erroneous state                       */
#define DAL_ADC_ERROR_OVR         0x02U   /*!< Overrun error                                         */
#define DAL_ADC_ERROR_DMA         0x04U   /*!< DMA transfer error                                    */
#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
#define DAL_ADC_ERROR_INVALID_CALLBACK  (0x10U)   /*!< Invalid Callback error */
#endif /* USE_DAL_ADC_REGISTER_CALLBACKS */
/**
  * @}
  */


/** @defgroup ADC_ClockPrescaler  ADC Clock Prescaler
  * @{
  */ 
#define ADC_CLOCK_SYNC_PCLK_DIV2    0x00000000U
#define ADC_CLOCK_SYNC_PCLK_DIV4    ((uint32_t)ADC_CCTRL_ADCPRE_0)
#define ADC_CLOCK_SYNC_PCLK_DIV6    ((uint32_t)ADC_CCTRL_ADCPRE_1)
#define ADC_CLOCK_SYNC_PCLK_DIV8    ((uint32_t)ADC_CCTRL_ADCPRE)
/**
  * @}
  */ 

/** @defgroup ADC_delay_between_2_sampling_phases ADC Delay Between 2 Sampling Phases
  * @{
  */ 
#define ADC_TWOSAMPLINGDELAY_5CYCLES    0x00000000U
#define ADC_TWOSAMPLINGDELAY_6CYCLES    ((uint32_t)ADC_CCTRL_SMPDEL2_0)
#define ADC_TWOSAMPLINGDELAY_7CYCLES    ((uint32_t)ADC_CCTRL_SMPDEL2_1)
#define ADC_TWOSAMPLINGDELAY_8CYCLES    ((uint32_t)(ADC_CCTRL_SMPDEL2_1 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_9CYCLES    ((uint32_t)ADC_CCTRL_SMPDEL2_2)
#define ADC_TWOSAMPLINGDELAY_10CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_2 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_11CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_2 | ADC_CCTRL_SMPDEL2_1))
#define ADC_TWOSAMPLINGDELAY_12CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_2 | ADC_CCTRL_SMPDEL2_1 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_13CYCLES   ((uint32_t)ADC_CCTRL_SMPDEL2_3)
#define ADC_TWOSAMPLINGDELAY_14CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_15CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_1))
#define ADC_TWOSAMPLINGDELAY_16CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_1 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_17CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_2))
#define ADC_TWOSAMPLINGDELAY_18CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_2 | ADC_CCTRL_SMPDEL2_0))
#define ADC_TWOSAMPLINGDELAY_19CYCLES   ((uint32_t)(ADC_CCTRL_SMPDEL2_3 | ADC_CCTRL_SMPDEL2_2 | ADC_CCTRL_SMPDEL2_1))
#define ADC_TWOSAMPLINGDELAY_20CYCLES   ((uint32_t)ADC_CCTRL_SMPDEL2)
/**
  * @}
  */ 

/** @defgroup ADC_Resolution ADC Resolution
  * @{
  */ 
#define ADC_RESOLUTION_12B  0x00000000U
#define ADC_RESOLUTION_10B  ((uint32_t)ADC_CTRL1_RESSEL_0)
#define ADC_RESOLUTION_8B   ((uint32_t)ADC_CTRL1_RESSEL_1)
#define ADC_RESOLUTION_6B   ((uint32_t)ADC_CTRL1_RESSEL)
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @{
  */ 
#define ADC_EXTERNALTRIGCONVEDGE_NONE           0x00000000U
#define ADC_EXTERNALTRIGCONVEDGE_RISING         ((uint32_t)ADC_CTRL2_REGEXTTRGEN_0)
#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ((uint32_t)ADC_CTRL2_REGEXTTRGEN_1)
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ((uint32_t)ADC_CTRL2_REGEXTTRGEN)
/**
  * @}
  */ 

/** @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @{
  */
/* Note: Parameter ADC_SOFTWARE_START is a software parameter used for        */
/*       compatibility with other APM32 devices.                              */
#define ADC_EXTERNALTRIGCONV_T1_CC1    0x00000000U
#define ADC_EXTERNALTRIGCONV_T1_CC2    ((uint32_t)ADC_CTRL2_REGEXTTRGSEL_0)
#define ADC_EXTERNALTRIGCONV_T1_CC3    ((uint32_t)ADC_CTRL2_REGEXTTRGSEL_1)
#define ADC_EXTERNALTRIGCONV_T2_CC2    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_1 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T2_CC3    ((uint32_t)ADC_CTRL2_REGEXTTRGSEL_2)
#define ADC_EXTERNALTRIGCONV_T2_CC4    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_2 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T2_TRGO   ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_2 | ADC_CTRL2_REGEXTTRGSEL_1))
#define ADC_EXTERNALTRIGCONV_T3_CC1    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_2 | ADC_CTRL2_REGEXTTRGSEL_1 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T3_TRGO   ((uint32_t)ADC_CTRL2_REGEXTTRGSEL_3)
#define ADC_EXTERNALTRIGCONV_T4_CC4    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T5_CC1    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_1))
#define ADC_EXTERNALTRIGCONV_T5_CC2    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_1 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T5_CC3    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_2))
#define ADC_EXTERNALTRIGCONV_T8_CC1    ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_2 | ADC_CTRL2_REGEXTTRGSEL_0))
#define ADC_EXTERNALTRIGCONV_T8_TRGO   ((uint32_t)(ADC_CTRL2_REGEXTTRGSEL_3 | ADC_CTRL2_REGEXTTRGSEL_2 | ADC_CTRL2_REGEXTTRGSEL_1))
#define ADC_EXTERNALTRIGCONV_EXT_IT11  ((uint32_t)ADC_CTRL2_REGEXTTRGSEL)
#define ADC_SOFTWARE_START             ((uint32_t)ADC_CTRL2_REGEXTTRGSEL + 1U)
/**
  * @}
  */ 

/** @defgroup ADC_Data_align ADC Data Align
  * @{
  */ 
#define ADC_DATAALIGN_RIGHT      0x00000000U
#define ADC_DATAALIGN_LEFT       ((uint32_t)ADC_CTRL2_DALIGNCFG)
/**
  * @}
  */ 

/** @defgroup ADC_channels  ADC Common Channels
  * @{
  */ 
#define ADC_CHANNEL_0           0x00000000U
#define ADC_CHANNEL_1           ((uint32_t)ADC_CTRL1_AWDCHSEL_0)
#define ADC_CHANNEL_2           ((uint32_t)ADC_CTRL1_AWDCHSEL_1)
#define ADC_CHANNEL_3           ((uint32_t)(ADC_CTRL1_AWDCHSEL_1 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_4           ((uint32_t)ADC_CTRL1_AWDCHSEL_2)
#define ADC_CHANNEL_5           ((uint32_t)(ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_6           ((uint32_t)(ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_1))
#define ADC_CHANNEL_7           ((uint32_t)(ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_1 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_8           ((uint32_t)ADC_CTRL1_AWDCHSEL_3)
#define ADC_CHANNEL_9           ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_10          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_1))
#define ADC_CHANNEL_11          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_1 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_12          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_2))
#define ADC_CHANNEL_13          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_14          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_1))
#define ADC_CHANNEL_15          ((uint32_t)(ADC_CTRL1_AWDCHSEL_3 | ADC_CTRL1_AWDCHSEL_2 | ADC_CTRL1_AWDCHSEL_1 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_16          ((uint32_t)ADC_CTRL1_AWDCHSEL_4)
#define ADC_CHANNEL_17          ((uint32_t)(ADC_CTRL1_AWDCHSEL_4 | ADC_CTRL1_AWDCHSEL_0))
#define ADC_CHANNEL_18          ((uint32_t)(ADC_CTRL1_AWDCHSEL_4 | ADC_CTRL1_AWDCHSEL_1))

#define ADC_CHANNEL_VREFINT     ((uint32_t)ADC_CHANNEL_17)
#define ADC_CHANNEL_VBAT        ((uint32_t)ADC_CHANNEL_18)
/**
  * @}
  */ 

/** @defgroup ADC_sampling_times  ADC Sampling Times
  * @{
  */ 
#define ADC_SAMPLETIME_3CYCLES    0x00000000U
#define ADC_SAMPLETIME_15CYCLES   ((uint32_t)ADC_SMPTIM1_SMPCYCCFG10_0)
#define ADC_SAMPLETIME_28CYCLES   ((uint32_t)ADC_SMPTIM1_SMPCYCCFG10_1)
#define ADC_SAMPLETIME_56CYCLES   ((uint32_t)(ADC_SMPTIM1_SMPCYCCFG10_1 | ADC_SMPTIM1_SMPCYCCFG10_0))
#define ADC_SAMPLETIME_84CYCLES   ((uint32_t)ADC_SMPTIM1_SMPCYCCFG10_2)
#define ADC_SAMPLETIME_112CYCLES  ((uint32_t)(ADC_SMPTIM1_SMPCYCCFG10_2 | ADC_SMPTIM1_SMPCYCCFG10_0))
#define ADC_SAMPLETIME_144CYCLES  ((uint32_t)(ADC_SMPTIM1_SMPCYCCFG10_2 | ADC_SMPTIM1_SMPCYCCFG10_1))
#define ADC_SAMPLETIME_480CYCLES  ((uint32_t)ADC_SMPTIM1_SMPCYCCFG10)
/**
  * @}
  */ 

  /** @defgroup ADC_EOCSelection ADC EOC Selection
  * @{
  */ 
#define ADC_EOC_SEQ_CONV              0x00000000U
#define ADC_EOC_SINGLE_CONV           0x00000001U
#define ADC_EOC_SINGLE_SEQ_CONV       0x00000002U  /*!< reserved for future use */
/**
  * @}
  */ 

/** @defgroup ADC_Event_type ADC Event Type
  * @{
  */ 
#define ADC_AWD_EVENT             ((uint32_t)ADC_FLAG_AWD)
#define ADC_OVR_EVENT             ((uint32_t)ADC_FLAG_OVR)
/**
  * @}
  */

/** @defgroup ADC_analog_watchdog_selection ADC Analog Watchdog Selection
  * @{
  */ 
#define ADC_ANALOGWATCHDOG_SINGLE_REG         ((uint32_t)(ADC_CTRL1_AWDSGLEN | ADC_CTRL1_REGAWDEN))
#define ADC_ANALOGWATCHDOG_SINGLE_INJEC       ((uint32_t)(ADC_CTRL1_AWDSGLEN | ADC_CTRL1_INJAWDEN))
#define ADC_ANALOGWATCHDOG_SINGLE_REGINJEC    ((uint32_t)(ADC_CTRL1_AWDSGLEN | ADC_CTRL1_REGAWDEN | ADC_CTRL1_INJAWDEN))
#define ADC_ANALOGWATCHDOG_ALL_REG            ((uint32_t)ADC_CTRL1_REGAWDEN)
#define ADC_ANALOGWATCHDOG_ALL_INJEC          ((uint32_t)ADC_CTRL1_INJAWDEN)
#define ADC_ANALOGWATCHDOG_ALL_REGINJEC       ((uint32_t)(ADC_CTRL1_REGAWDEN | ADC_CTRL1_INJAWDEN))
#define ADC_ANALOGWATCHDOG_NONE               0x00000000U
/**
  * @}
  */ 
    
/** @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @{
  */ 
#define ADC_IT_EOC      ((uint32_t)ADC_CTRL1_EOCIEN)
#define ADC_IT_AWD      ((uint32_t)ADC_CTRL1_AWDIEN)
#define ADC_IT_JEOC     ((uint32_t)ADC_CTRL1_INJEOCIEN)
#define ADC_IT_OVR      ((uint32_t)ADC_CTRL1_OVRIEN)
/**
  * @}
  */ 
    
/** @defgroup ADC_flags_definition ADC Flags Definition
  * @{
  */ 
#define ADC_FLAG_AWD    ((uint32_t)ADC_STS_AWDFLG)
#define ADC_FLAG_EOC    ((uint32_t)ADC_STS_EOCFLG)
#define ADC_FLAG_JEOC   ((uint32_t)ADC_STS_INJEOCFLG)
#define ADC_FLAG_JSTRT  ((uint32_t)ADC_STS_INJCSFLG)
#define ADC_FLAG_STRT   ((uint32_t)ADC_STS_REGCSFLG)
#define ADC_FLAG_OVR    ((uint32_t)ADC_STS_OVRFLG)
/**
  * @}
  */ 

/** @defgroup ADC_channels_type ADC Channels Type
  * @{
  */ 
#define ADC_ALL_CHANNELS      0x00000001U
#define ADC_REGULAR_CHANNELS  0x00000002U /*!< reserved for future use */
#define ADC_INJECTED_CHANNELS 0x00000003U /*!< reserved for future use */
/**
  * @}
  */

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ADC_Exported_Macros ADC Exported Macros
  * @{
  */

/** @brief Reset ADC handle state
  * @param  __HANDLE__ ADC handle
  * @retval None
  */
#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
#define __DAL_ADC_RESET_HANDLE_STATE(__HANDLE__)                               \
  do{                                                                          \
     (__HANDLE__)->State = DAL_ADC_STATE_RESET;                               \
     (__HANDLE__)->MspInitCallback = NULL;                                     \
     (__HANDLE__)->MspDeInitCallback = NULL;                                   \
    } while(0)
#else
#define __DAL_ADC_RESET_HANDLE_STATE(__HANDLE__)                               \
  ((__HANDLE__)->State = DAL_ADC_STATE_RESET)
#endif

/**
  * @brief  Enable the ADC peripheral.
  * @param  __HANDLE__ ADC handle
  * @retval None
  */
#define __DAL_ADC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL2 |=  ADC_CTRL2_ADCEN)

/**
  * @brief  Disable the ADC peripheral.
  * @param  __HANDLE__ ADC handle
  * @retval None
  */
#define __DAL_ADC_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL2 &=  ~ADC_CTRL2_ADCEN)

/**
  * @brief  Enable the ADC end of conversion interrupt.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __INTERRUPT__ ADC Interrupt.
  * @retval None
  */
#define __DAL_ADC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL1) |= (__INTERRUPT__))

/**
  * @brief  Disable the ADC end of conversion interrupt.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __INTERRUPT__ ADC interrupt.
  * @retval None
  */
#define __DAL_ADC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL1) &= ~(__INTERRUPT__))

/** @brief  Check if the specified ADC interrupt source is enabled or disabled.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __INTERRUPT__ specifies the ADC interrupt source to check.
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_ADC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CTRL1 & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Clear the ADC's pending flags.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __FLAG__ ADC flag.
  * @retval None
  */
#define __DAL_ADC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS) = ~(__FLAG__))

/**
  * @brief  Get the selected ADC's flag status.
  * @param  __HANDLE__ specifies the ADC Handle.
  * @param  __FLAG__ ADC flag.
  * @retval None
  */
#define __DAL_ADC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->STS) & (__FLAG__)) == (__FLAG__))

/**
  * @}
  */

/* Include ADC DAL Extension module */
#include "apm32f4xx_dal_adc_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_Exported_Functions
  * @{
  */

/** @addtogroup ADC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions ***********************************/
DAL_StatusTypeDef DAL_ADC_Init(ADC_HandleTypeDef* hadc);
DAL_StatusTypeDef DAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void DAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void DAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);

#if (USE_DAL_ADC_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
DAL_StatusTypeDef DAL_ADC_RegisterCallback(ADC_HandleTypeDef *hadc, DAL_ADC_CallbackIDTypeDef CallbackID, pADC_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_ADC_UnRegisterCallback(ADC_HandleTypeDef *hadc, DAL_ADC_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_ADC_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
DAL_StatusTypeDef DAL_ADC_Start(ADC_HandleTypeDef* hadc);
DAL_StatusTypeDef DAL_ADC_Stop(ADC_HandleTypeDef* hadc);
DAL_StatusTypeDef DAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

DAL_StatusTypeDef DAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

DAL_StatusTypeDef DAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
DAL_StatusTypeDef DAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void DAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

DAL_StatusTypeDef DAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
DAL_StatusTypeDef DAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t DAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

void DAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void DAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void DAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void DAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions *************************************************/
DAL_StatusTypeDef DAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
DAL_StatusTypeDef DAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);
/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions ***************************************************/
uint32_t DAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t DAL_ADC_GetError(ADC_HandleTypeDef *hadc);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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

/* Private macro ------------------------------------------------------------*/

/** @defgroup ADC_Private_Macros ADC Private Macros
  * @{
  */
/* Macro reserved for internal DAL driver usage, not intended to be used in
   code of final user */

/**
  * @brief Verification of ADC state: enabled or disabled
  * @param __HANDLE__ ADC handle
  * @retval SET (ADC enabled) or RESET (ADC disabled)
  */
#define ADC_IS_ENABLE(__HANDLE__)                                              \
  ((( ((__HANDLE__)->Instance->CTRL2 & ADC_CTRL2_ADCEN) == ADC_CTRL2_ADCEN )            \
  ) ? SET : RESET)

/**
  * @brief Test if conversion trigger of regular group is software start
  *        or external trigger.
  * @param __HANDLE__ ADC handle
  * @retval SET (software start) or RESET (external trigger)
  */
#define ADC_IS_SOFTWARE_START_REGULAR(__HANDLE__)                              \
  (((__HANDLE__)->Instance->CTRL2 & ADC_CTRL2_REGEXTTRGEN) == RESET)

/**
  * @brief Test if conversion trigger of injected group is software start
  *        or external trigger.
  * @param __HANDLE__ ADC handle
  * @retval SET (software start) or RESET (external trigger)
  */
#define ADC_IS_SOFTWARE_START_INJECTED(__HANDLE__)                             \
  (((__HANDLE__)->Instance->CTRL2 & ADC_CTRL2_INJEXTTRGEN) == RESET)

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
  ((__HANDLE__)->ErrorCode = DAL_ADC_ERROR_NONE)

    
#define IS_ADC_CLOCKPRESCALER(ADC_CLOCK)     (((ADC_CLOCK) == ADC_CLOCK_SYNC_PCLK_DIV2) || \
                                              ((ADC_CLOCK) == ADC_CLOCK_SYNC_PCLK_DIV4) || \
                                              ((ADC_CLOCK) == ADC_CLOCK_SYNC_PCLK_DIV6) || \
                                              ((ADC_CLOCK) == ADC_CLOCK_SYNC_PCLK_DIV8))
#define IS_ADC_SAMPLING_DELAY(DELAY) (((DELAY) == ADC_TWOSAMPLINGDELAY_5CYCLES)  || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_6CYCLES)  || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_7CYCLES)  || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_8CYCLES)  || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_9CYCLES)  || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_10CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_11CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_12CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_13CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_14CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_15CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_16CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_17CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_18CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_19CYCLES) || \
                                      ((DELAY) == ADC_TWOSAMPLINGDELAY_20CYCLES))
#define IS_ADC_RESOLUTION(RESOLUTION) (((RESOLUTION) == ADC_RESOLUTION_12B) || \
                                       ((RESOLUTION) == ADC_RESOLUTION_10B) || \
                                       ((RESOLUTION) == ADC_RESOLUTION_8B)  || \
                                       ((RESOLUTION) == ADC_RESOLUTION_6B))
#define IS_ADC_EXT_TRIG_EDGE(EDGE) (((EDGE) == ADC_EXTERNALTRIGCONVEDGE_NONE)    || \
                                    ((EDGE) == ADC_EXTERNALTRIGCONVEDGE_RISING)  || \
                                    ((EDGE) == ADC_EXTERNALTRIGCONVEDGE_FALLING) || \
                                    ((EDGE) == ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING))
#define IS_ADC_EXT_TRIG(REGTRIG) (((REGTRIG) == ADC_EXTERNALTRIGCONV_T1_CC1)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T1_CC2)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T1_CC3)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T2_CC2)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T2_CC3)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T2_CC4)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T2_TRGO) || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T3_CC1)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T3_TRGO) || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T4_CC4)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T5_CC1)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T5_CC2)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T5_CC3)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T8_CC1)  || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_T8_TRGO) || \
                                  ((REGTRIG) == ADC_EXTERNALTRIGCONV_EXT_IT11)|| \
                                  ((REGTRIG) == ADC_SOFTWARE_START))
#define IS_ADC_DATA_ALIGN(ALIGN) (((ALIGN) == ADC_DATAALIGN_RIGHT) || \
                                  ((ALIGN) == ADC_DATAALIGN_LEFT))
#define IS_ADC_SAMPLE_TIME(TIME) (((TIME) == ADC_SAMPLETIME_3CYCLES)   || \
                                  ((TIME) == ADC_SAMPLETIME_15CYCLES)  || \
                                  ((TIME) == ADC_SAMPLETIME_28CYCLES)  || \
                                  ((TIME) == ADC_SAMPLETIME_56CYCLES)  || \
                                  ((TIME) == ADC_SAMPLETIME_84CYCLES)  || \
                                  ((TIME) == ADC_SAMPLETIME_112CYCLES) || \
                                  ((TIME) == ADC_SAMPLETIME_144CYCLES) || \
                                  ((TIME) == ADC_SAMPLETIME_480CYCLES))
#define IS_ADC_EOCSelection(EOCSelection) (((EOCSelection) == ADC_EOC_SINGLE_CONV)   || \
                                           ((EOCSelection) == ADC_EOC_SEQ_CONV)  || \
                                           ((EOCSelection) == ADC_EOC_SINGLE_SEQ_CONV))
#define IS_ADC_EVENT_TYPE(EVENT) (((EVENT) == ADC_AWD_EVENT) || \
                                  ((EVENT) == ADC_OVR_EVENT))
#define IS_ADC_ANALOG_WATCHDOG(WATCHDOG) (((WATCHDOG) == ADC_ANALOGWATCHDOG_SINGLE_REG)        || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_SINGLE_INJEC)      || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_SINGLE_REGINJEC)   || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_ALL_REG)           || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_ALL_INJEC)         || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_ALL_REGINJEC)      || \
                                          ((WATCHDOG) == ADC_ANALOGWATCHDOG_NONE))
#define IS_ADC_CHANNELS_TYPE(CHANNEL_TYPE) (((CHANNEL_TYPE) == ADC_ALL_CHANNELS) || \
                                            ((CHANNEL_TYPE) == ADC_REGULAR_CHANNELS) || \
                                            ((CHANNEL_TYPE) == ADC_INJECTED_CHANNELS))
#define IS_ADC_THRESHOLD(THRESHOLD) ((THRESHOLD) <= 0xFFFU)

#define IS_ADC_REGULAR_LENGTH(LENGTH) (((LENGTH) >= 1U) && ((LENGTH) <= 16U))
#define IS_ADC_REGULAR_RANK(RANK) (((RANK) >= 1U) && ((RANK) <= (16U)))
#define IS_ADC_REGULAR_DISC_NUMBER(NUMBER) (((NUMBER) >= 1U) && ((NUMBER) <= 8U))
#define IS_ADC_RANGE(RESOLUTION, ADC_VALUE)                                     \
   ((((RESOLUTION) == ADC_RESOLUTION_12B) && ((ADC_VALUE) <= 0x0FFFU)) || \
    (((RESOLUTION) == ADC_RESOLUTION_10B) && ((ADC_VALUE) <= 0x03FFU)) || \
    (((RESOLUTION) == ADC_RESOLUTION_8B)  && ((ADC_VALUE) <= 0x00FFU)) || \
    (((RESOLUTION) == ADC_RESOLUTION_6B)  && ((ADC_VALUE) <= 0x003FU)))

/**
  * @brief  Set ADC Regular channel sequence length.
  * @param  _NbrOfConversion_ Regular channel sequence length. 
  * @retval None
  */
#define ADC_REGSEQ1(_NbrOfConversion_) (((_NbrOfConversion_) - (uint8_t)1U) << 20U)

/**
  * @brief  Set the ADC's sample time for channel numbers between 10 and 18.
  * @param  _SAMPLETIME_ Sample time parameter.
  * @param  _CHANNELNB_ Channel number.  
  * @retval None
  */
#define ADC_SMPTIM1(_SAMPLETIME_, _CHANNELNB_) ((_SAMPLETIME_) << (3U * (((uint32_t)((uint16_t)(_CHANNELNB_))) - 10U)))

/**
  * @brief  Set the ADC's sample time for channel numbers between 0 and 9.
  * @param  _SAMPLETIME_ Sample time parameter.
  * @param  _CHANNELNB_ Channel number.  
  * @retval None
  */
#define ADC_SMPTIM2(_SAMPLETIME_, _CHANNELNB_) ((_SAMPLETIME_) << (3U * ((uint32_t)((uint16_t)(_CHANNELNB_)))))

/**
  * @brief  Set the selected regular channel rank for rank between 1 and 6.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_REGSEQ3_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 1U)))

/**
  * @brief  Set the selected regular channel rank for rank between 7 and 12.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_REGSEQ2_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 7U)))

/**
  * @brief  Set the selected regular channel rank for rank between 13 and 16.
  * @param  _CHANNELNB_ Channel number.
  * @param  _RANKNB_ Rank number.    
  * @retval None
  */
#define ADC_REGSEQ1_RK(_CHANNELNB_, _RANKNB_) (((uint32_t)((uint16_t)(_CHANNELNB_))) << (5U * ((_RANKNB_) - 13U)))

/**
  * @brief  Enable ADC continuous conversion mode.
  * @param  _CONTINUOUS_MODE_ Continuous mode.
  * @retval None
  */
#define ADC_CTRL2_CONTCENINUOUS(_CONTINUOUS_MODE_) ((_CONTINUOUS_MODE_) << 1U)

/**
  * @brief  Configures the number of discontinuous conversions for the regular group channels.
  * @param  _NBR_DISCONTINUOUSCONV_ Number of discontinuous conversions.
  * @retval None
  */
#define ADC_CTRL1_DISCONTINUOUS(_NBR_DISCONTINUOUSCONV_) (((_NBR_DISCONTINUOUSCONV_) - 1U) << ADC_CTRL1_DISCNUMCFG_Pos)

/**
  * @brief  Enable ADC scan mode.
  * @param  _SCANCONV_MODE_ Scan conversion mode.
  * @retval None
  */
#define ADC_CTRL1_SCANENCONV(_SCANCONV_MODE_) ((_SCANCONV_MODE_) << 8U)

/**
  * @brief  Enable the ADC end of conversion selection.
  * @param  _EOCSelection_MODE_ End of conversion selection mode.
  * @retval None
  */
#define ADC_CTRL2_EOCSELelection(_EOCSelection_MODE_) ((_EOCSelection_MODE_) << 10U)

/**
  * @brief  Enable the ADC DMA continuous request.
  * @param  _DMAContReq_MODE_ DMA continuous request mode.
  * @retval None
  */
#define ADC_CTRL2_DMAENContReq(_DMAContReq_MODE_) ((_DMAContReq_MODE_) << 9U)

/**
  * @brief Return resolution bits in CTRL1 register.
  * @param __HANDLE__ ADC handle
  * @retval None
  */
#define ADC_GET_RESOLUTION(__HANDLE__) (((__HANDLE__)->Instance->CTRL1) & ADC_CTRL1_RESSEL)

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup ADC_Private_Functions ADC Private Functions
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

#endif /* APM32F4xx_DAL_ADC_H */


