/**
  ******************************************************************************
  * @file    stm32f4xx_dfsdm.h
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file contains all the functions prototypes for the DFSDM
  *          firmware library
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_DFSDM_H
#define __STM32F4XX_DFSDM_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(STM32F412xG)
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup DFSDM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  DFSDM Transceiver init structure definition
  */
typedef struct
{
  uint32_t DFSDM_Interface;                 /*!< Selects the serial interface type and input clock phase.
                                            This parameter can be a value of @ref DFSDM_Interface_Selection */

  uint32_t DFSDM_Clock;                     /*!< Specifies the clock source for the serial interface transceiver.
                                            This parameter can be a value of @ref DFSDM_Clock_Selection */

  uint32_t DFSDM_Input;                     /*!< Specifies the Input mode for the serial interface transceiver.
                                            This parameter can be a value of @ref DFSDM_Input_Selection */

  uint32_t DFSDM_Redirection;               /*!< Specifies if the channel input is redirected from channel channel (y+1).
                                            This parameter can be a value of @ref DFSDM_Redirection_Selection */

  uint32_t DFSDM_PackingMode;               /*!< Specifies the packing mode for the serial interface transceiver.
                                            This parameter can be a value of @ref DFSDM_Pack_Selection */

  uint32_t DFSDM_DataRightShift;            /*!< Defines the final data right bit shift.
                                            This parameter can be a value between 0 and 31 */

  uint32_t DFSDM_Offset;                    /*!< Sets the calibration offset.
                                            This parameter can be a value between 0 and 0xFFFFFF */

  uint32_t DFSDM_CLKAbsenceDetector;        /*!< Enables or disables the Clock Absence Detector.
                                            This parameter can be a value of @ref DFSDM_Clock_Absence_Detector_state */

  uint32_t DFSDM_ShortCircuitDetector;      /*!< Enables or disables the Short Circuit Detector.
                                            This parameter can be a value of @ref DFSDM_Short_Circuit_Detector_state */
}DFSDM_TransceiverInitTypeDef;

/** 
  * @brief  DFSDM filter analog parameters structure definition  
  */
typedef struct
{
  uint32_t DFSDM_SincOrder;                  /*!< Sets the Sinc Filter Order .
                                             This parameter can be a value of @ref DFSDM_Sinc_Order */

  uint32_t DFSDM_FilterOversamplingRatio;    /*!< Sets the Sinc Filter Oversampling Ratio.
                                             This parameter can be a value between 1 and 1024 */

  uint32_t DFSDM_IntegratorOversamplingRatio;/*!< Sets the Integrator Oversampling Ratio.
                                             This parameter can be a value between 1 and 256 */
}DFSDM_FilterInitTypeDef;

/* Exported constants --------------------------------------------------------*/
/** @defgroup DFSDM_Interface_Selection
  * @{
  */
#define DFSDM_Interface_SPI_RisingEdge   ((uint32_t)0x00000000)  /*!<  DFSDM SPI interface with rising edge to strobe data */
#define DFSDM_Interface_SPI_FallingEdge  ((uint32_t)0x00000001)  /*!<  DFSDM SPI interface with falling edge to strobe data */
#define DFSDM_Interface_Manchester1      ((uint32_t)0x00000002)  /*!<  DFSDM Manchester coded input, rising edge = logic 0, falling edge = logic 1 */
#define DFSDM_Interface_Manchester2      ((uint32_t)0x00000003)  /*!<  DFSDM Manchester coded input, rising edge = logic 1, falling edge = logic 0 */

#define IS_DFSDM_INTERFACE(INTERFACE)    (((INTERFACE) == DFSDM_Interface_SPI_RisingEdge) || \
                                         ((INTERFACE) == DFSDM_Interface_SPI_FallingEdge) || \
                                         ((INTERFACE) == DFSDM_Interface_Manchester1)     || \
                                         ((INTERFACE) == DFSDM_Interface_Manchester2))
/**
  * @}
  */

/** @defgroup DFSDM_Clock_Selection
  * @{
  */
#define DFSDM_Clock_External             ((uint32_t)0x00000000)  /*!<  DFSDM clock coming from external DFSDM_CKINy input */
#define DFSDM_Clock_Internal             ((uint32_t)0x00000004)  /*!<  DFSDM clock coming from internal DFSDM_CKOUT output */
#define DFSDM_Clock_InternalDiv2_Mode1   ((uint32_t)0x00000008)  /*!<  DFSDM clock coming from internal DFSDM_CKOUT output divided by 2
                                                                       and clock change is on every rising edge of DFSDM_CKOUT output signal */
#define DFSDM_Clock_InternalDiv2_Mode2   ((uint32_t)0x0000000C)  /*!<  DFSDM clock coming from internal DFSDM_CKOUT output divided by 2
                                                                       and clock change is on every falling edge of DFSDM_CKOUT output signal */

#define IS_DFSDM_CLOCK(CLOCK)            (((CLOCK) == DFSDM_Clock_External)          || \
                                         ((CLOCK) == DFSDM_Clock_Internal)           || \
                                         ((CLOCK) == DFSDM_Clock_InternalDiv2_Mode1) || \
                                         ((CLOCK) == DFSDM_Clock_InternalDiv2_Mode2))
/**
  * @}
  */

/** @defgroup DFSDM_Input_Selection
  * @{
  */
#define DFSDM_Input_External     ((uint32_t)0x00000000)  /*!<  DFSDM clock coming from external DFSDM_CKINy input */
#define DFSDM_Input_ADC          ((uint32_t)0x00001000)  /*!<  DFSDM clock coming from internal DFSDM_CKOUT output */
#define DFSDM_Input_Internal     ((uint32_t)0x00002000)  /*!<  DFSDM clock coming from internal DFSDM_CKOUT output divided by 2
                                                               and clock change is on every rising edge of DFSDM_CKOUT output signal */

#define IS_DFSDM_Input_MODE(INPUT)      (((INPUT) == DFSDM_Input_External) || \
                                         ((INPUT) == DFSDM_Input_ADC)      || \
                                         ((INPUT) == DFSDM_Input_Internal))
/**
  * @}
  */

/** @defgroup DFSDM_Redirection_Selection
  * @{
  */
#define DFSDM_Redirection_Disabled       ((uint32_t)0x00000000)  /*!< DFSDM Channel serial inputs are taken from pins of the same channel y */
#define DFSDM_Redirection_Enabled         DFSDM_CHCFGR1_CHINSEL  /*!< DFSDM Channel serial inputs are taken from pins of the channel (y+1) modulo 8 */

#define IS_DFSDM_Redirection_STATE(STATE)      (((STATE) == DFSDM_Redirection_Disabled) || \
                                                ((STATE) == DFSDM_Redirection_Enabled))
/**
  * @}
  */

/** @defgroup DFSDM_Pack_Selection
  * @{
  */
#define DFSDM_PackingMode_Standard     ((uint32_t)0x00000000)  /*!<  DFSDM Input data in DFSDM_CHDATINyR register are stored only in INDAT0[15:0] */
#define DFSDM_PackingMode_Interleaved  ((uint32_t)0x00004000)  /*!<  DFSDM Input data in DFSDM_CHDATINyR register are stored as two samples:
                                                                        - first sample in INDAT0[15:0] - assigned to channel y
                                                                        - second sample INDAT1[15:0]   - assigned to channel y */
#define DFSDM_PackingMode_Dual         ((uint32_t)0x00008000)  /*!<  DFSDM Input data in DFSDM_CHDATINyR register are stored as two samples:
                                                                        - first sample INDAT0[15:0]    - assigned to channel y
                                                                        - second sample INDAT1[15:0]   - assigned to channel (y+1) */

#define IS_DFSDM_PACK_MODE(MODE)        (((MODE) == DFSDM_PackingMode_Standard)    || \
                                         ((MODE) == DFSDM_PackingMode_Interleaved) || \
                                         ((MODE) == DFSDM_PackingMode_Dual))
/**
  * @}
  */

/** @defgroup DFSDM_Clock_Absence_Detector_state
  * @{
  */
#define DFSDM_CLKAbsenceDetector_Enable     DFSDM_CHCFGR1_CKABEN    /*!<  DFSDM Clock Absence Detector is Enabled */
#define DFSDM_CLKAbsenceDetector_Disable    ((uint32_t)0x00000000)  /*!<  DFSDM Clock Absence Detector is Disabled */

#define IS_DFSDM_CLK_DETECTOR_STATE(STATE)  (((STATE) == DFSDM_CLKAbsenceDetector_Enable) || \
                                             ((STATE) == DFSDM_CLKAbsenceDetector_Disable))
/**
  * @}
  */

/** @defgroup DFSDM_Short_Circuit_Detector_state
  * @{
  */
#define DFSDM_ShortCircuitDetector_Enable   DFSDM_CHCFGR1_SCDEN     /*!<  DFSDM Short Circuit Detector is Enabled */
#define DFSDM_ShortCircuitDetector_Disable  ((uint32_t)0x00000000)  /*!<  DFSDM Short Circuit Detector is Disabled */

#define IS_DFSDM_SC_DETECTOR_STATE(STATE)  (((STATE) == DFSDM_ShortCircuitDetector_Enable) || \
                                            ((STATE) == DFSDM_ShortCircuitDetector_Disable))
/**
  * @}
  */

/** @defgroup DFSDM_Sinc_Order
  * @{
  */
#define DFSDM_SincOrder_FastSinc        ((uint32_t)0x00000000)  /*!<  DFSDM Sinc filter order = Fast sinc */
#define DFSDM_SincOrder_Sinc1           ((uint32_t)0x20000000)  /*!<  DFSDM Sinc filter order = 1 */
#define DFSDM_SincOrder_Sinc2           ((uint32_t)0x40000000)  /*!<  DFSDM Sinc filter order = 2 */
#define DFSDM_SincOrder_Sinc3           ((uint32_t)0x60000000)  /*!<  DFSDM Sinc filter order = 3 */
#define DFSDM_SincOrder_Sinc4           ((uint32_t)0x80000000)  /*!<  DFSDM Sinc filter order = 4 */
#define DFSDM_SincOrder_Sinc5           ((uint32_t)0xA0000000)  /*!<  DFSDM Sinc filter order = 5 */

#define IS_DFSDM_SINC_ORDER(ORDER)        (((ORDER) == DFSDM_SincOrder_FastSinc) || \
                                          ((ORDER) == DFSDM_SincOrder_Sinc1)     || \
                                          ((ORDER) == DFSDM_SincOrder_Sinc2)     || \
                                          ((ORDER) == DFSDM_SincOrder_Sinc3)     || \
                                          ((ORDER) == DFSDM_SincOrder_Sinc4)     || \
                                          ((ORDER) == DFSDM_SincOrder_Sinc5))
/**
  * @}
  */

/** @defgroup DFSDM_Break_Signal_Assignment
  * @{
  */
#define DFSDM_SCDBreak_0                 ((uint32_t)0x00001000)  /*!<  DFSDM Break 0 signal assigned to short circuit detector */
#define DFSDM_SCDBreak_1                 ((uint32_t)0x00002000)  /*!<  DFSDM Break 1 signal assigned to short circuit detector */
#define DFSDM_SCDBreak_2                 ((uint32_t)0x00004000)  /*!<  DFSDM Break 2 signal assigned to short circuit detector */
#define DFSDM_SCDBreak_3                 ((uint32_t)0x00008000)  /*!<  DFSDM Break 3 signal assigned to short circuit detector */

#define IS_DFSDM_SCD_BREAK_SIGNAL(RANK)    (((RANK) == DFSDM_SCDBreak_0) || \
                                           ((RANK) == DFSDM_SCDBreak_1)  || \
                                           ((RANK) == DFSDM_SCDBreak_2)  || \
                                           ((RANK) == DFSDM_SCDBreak_3))
/**
  * @}
  */

/** @defgroup DFSDM_AWD_Sinc_Order
  * @{
  */
#define DFSDM_AWDSincOrder_Fast            ((uint32_t)0x00000000)  /*!<  DFSDM Fast sinc filter */
#define DFSDM_AWDSincOrder_Sinc1           ((uint32_t)0x00400000)  /*!<  DFSDM sinc1 filter */
#define DFSDM_AWDSincOrder_Sinc2           ((uint32_t)0x00800000)  /*!<  DFSDM sinc2 filter */
#define DFSDM_AWDSincOrder_Sinc3           ((uint32_t)0x00C00000)  /*!<  DFSDM sinc3 filter */

#define IS_DFSDM_AWD_SINC_ORDER(ORDER)    (((ORDER) == DFSDM_AWDSincOrder_Fast)  || \
                                           ((ORDER) == DFSDM_AWDSincOrder_Sinc1) || \
                                           ((ORDER) == DFSDM_AWDSincOrder_Sinc2) || \
                                           ((ORDER) == DFSDM_AWDSincOrder_Sinc3))
/**
  * @}
  */

/** @defgroup DFSDM_AWD_CHANNEL
  * @{
  */
#define DFSDM_AWDChannel0               ((uint32_t)0x00010000)  /*!<  DFSDM AWDx guard channel 0 */
#define DFSDM_AWDChannel1               ((uint32_t)0x00020000)  /*!<  DFSDM AWDx guard channel 1 */
#define DFSDM_AWDChannel2               ((uint32_t)0x00040000)  /*!<  DFSDM AWDx guard channel 2 */
#define DFSDM_AWDChannel3               ((uint32_t)0x00080000)  /*!<  DFSDM AWDx guard channel 3 */
#define DFSDM_AWDChannel4               ((uint32_t)0x00100000)  /*!<  DFSDM AWDx guard channel 4 */
#define DFSDM_AWDChannel5               ((uint32_t)0x00200000)  /*!<  DFSDM AWDx guard channel 5 */
#define DFSDM_AWDChannel6               ((uint32_t)0x00400000)  /*!<  DFSDM AWDx guard channel 6 */
#define DFSDM_AWDChannel7               ((uint32_t)0x00800000)  /*!<  DFSDM AWDx guard channel 7 */

#define IS_DFSDM_AWD_CHANNEL(CHANNEL)    (((CHANNEL) == DFSDM_AWDChannel0) || \
                                          ((CHANNEL) == DFSDM_AWDChannel1) || \
                                          ((CHANNEL) == DFSDM_AWDChannel2) || \
                                          ((CHANNEL) == DFSDM_AWDChannel3) || \
                                          ((CHANNEL) == DFSDM_AWDChannel4) || \
                                          ((CHANNEL) == DFSDM_AWDChannel5) || \
                                          ((CHANNEL) == DFSDM_AWDChannel6) || \
                                          ((CHANNEL) == DFSDM_AWDChannel7))
/**
  * @}
  */

/** @defgroup DFSDM_Threshold_Selection
  * @{
  */
#define DFSDM_Threshold_Low               ((uint8_t)0x00)  /*!<  DFSDM Low threshold */
#define DFSDM_Threshold_High              ((uint8_t)0x08)  /*!<  DFSDM High threshold */

#define IS_DFSDM_Threshold(THR)          (((THR) == DFSDM_Threshold_Low) || \
                                          ((THR) == DFSDM_Threshold_High))
/**
  * @}
  */

/** @defgroup DFSDM_AWD_Fast_Mode_Selection
  * @{
  */
#define DFSDM_AWDFastMode_Disable         ((uint32_t)0x00000000)  /*!<  DFSDM Fast mode for AWD is disabled */
#define DFSDM_AWDFastMode_Enable          ((uint32_t)0x40000000)  /*!<  DFSDM Fast mode for AWD is enabled */

#define IS_DFSDM_AWD_MODE(MODE)          (((MODE) == DFSDM_AWDFastMode_Disable) || \
                                          ((MODE) == DFSDM_AWDFastMode_Enable))
/**
  * @}
  */

/** @defgroup DFSDM_Clock_Output_Source_Selection
  * @{
  */
#define DFSDM_ClkOutSource_SysClock        ((uint32_t)0x00000000)  /*!<  DFSDM Source for output clock is comming from system clock */
#define DFSDM_ClkOutSource_AudioClock      DFSDM_CHCFGR1_CKOUTSRC  /*!<  DFSDM Source for output clock is comming from audio clock */

#define IS_DFSDM_CLOCK_OUT_SOURCE(SRC)    (((SRC) == DFSDM_ClkOutSource_SysClock) || \
                                           ((SRC) == DFSDM_ClkOutSource_AudioClock))
/**
  * @}
  */

/** @defgroup DFSDM_Conversion_Mode
  * @{
  */
#define DFSDM_DMAConversionMode_Regular     ((uint32_t)0x00000010)  /*!<  DFSDM Regular mode */
#define DFSDM_DMAConversionMode_Injected    ((uint32_t)0x00000000)  /*!<  DFSDM Injected mode */

#define IS_DFSDM_CONVERSION_MODE(MODE)    ((MODE) == DFSDM_DMAConversionMode_Regular || \
                                           ((MODE) == DFSDM_DMAConversionMode_Injected))
/**
  * @}
  */

/** @defgroup DFSDM_Extremes_Channel_Selection
  * @{
  */
#define DFSDM_ExtremChannel0              ((uint32_t)0x00000100)  /*!<  DFSDM Extreme detector guard channel 0 */
#define DFSDM_ExtremChannel1              ((uint32_t)0x00000200)  /*!<  DFSDM Extreme detector guard channel 1 */
#define DFSDM_ExtremChannel2              ((uint32_t)0x00000400)  /*!<  DFSDM Extreme detector guard channel 2 */
#define DFSDM_ExtremChannel3              ((uint32_t)0x00000800)  /*!<  DFSDM Extreme detector guard channel 3 */
#define DFSDM_ExtremChannel4              ((uint32_t)0x00001000)  /*!<  DFSDM Extreme detector guard channel 4 */
#define DFSDM_ExtremChannel5              ((uint32_t)0x00002000)  /*!<  DFSDM Extreme detector guard channel 5 */
#define DFSDM_ExtremChannel6              ((uint32_t)0x00004000)  /*!<  DFSDM Extreme detector guard channel 6 */
#define DFSDM_ExtremChannel7              ((uint32_t)0x00008000)  /*!<  DFSDM Extreme detector guard channel 7 */

#define IS_DFSDM_EXTREM_CHANNEL(CHANNEL) (((CHANNEL) == DFSDM_ExtremChannel0) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel1) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel2) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel3) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel4) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel5) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel6) || \
                                          ((CHANNEL) == DFSDM_ExtremChannel7))
/**
  * @}
  */

/** @defgroup DFSDM_Injected_Channel_Selection
  * @{
  */
#define DFSDM_InjectedChannel0            ((uint32_t)0x00000001)  /*!<  DFSDM channel 0 is selected as injected channel */
#define DFSDM_InjectedChannel1            ((uint32_t)0x00000002)  /*!<  DFSDM channel 1 is selected as injected channel */
#define DFSDM_InjectedChannel2            ((uint32_t)0x00000004)  /*!<  DFSDM channel 2 is selected as injected channel */
#define DFSDM_InjectedChannel3            ((uint32_t)0x00000008)  /*!<  DFSDM channel 3 is selected as injected channel */
#define DFSDM_InjectedChannel4            ((uint32_t)0x00000010)  /*!<  DFSDM channel 4 is selected as injected channel */
#define DFSDM_InjectedChannel5            ((uint32_t)0x00000020)  /*!<  DFSDM channel 5 is selected as injected channel */
#define DFSDM_InjectedChannel6            ((uint32_t)0x00000040)  /*!<  DFSDM channel 6 is selected as injected channel */
#define DFSDM_InjectedChannel7            ((uint32_t)0x00000080)  /*!<  DFSDM channel 7 is selected as injected channel */

#define IS_DFSDM_INJECT_CHANNEL(CHANNEL) (((CHANNEL) == DFSDM_InjectedChannel0) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel1) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel2) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel3) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel4) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel5) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel6) || \
                                          ((CHANNEL) == DFSDM_InjectedChannel7))
/**
  * @}
  */

/** @defgroup DFSDM_Regular_Channel_Selection
  * @{
  */
#define DFSDM_RegularChannel0             ((uint32_t)0x00000000)  /*!<  DFSDM channel 0 is selected as regular channel */
#define DFSDM_RegularChannel1             ((uint32_t)0x01000000)  /*!<  DFSDM channel 1 is selected as regular channel */
#define DFSDM_RegularChannel2             ((uint32_t)0x02000000)  /*!<  DFSDM channel 2 is selected as regular channel */
#define DFSDM_RegularChannel3             ((uint32_t)0x03000000)  /*!<  DFSDM channel 3 is selected as regular channel */
#define DFSDM_RegularChannel4             ((uint32_t)0x04000000)  /*!<  DFSDM channel 4 is selected as regular channel */
#define DFSDM_RegularChannel5             ((uint32_t)0x05000000)  /*!<  DFSDM channel 5 is selected as regular channel */
#define DFSDM_RegularChannel6             ((uint32_t)0x06000000)  /*!<  DFSDM channel 6 is selected as regular channel */
#define DFSDM_RegularChannel7             ((uint32_t)0x07000000)  /*!<  DFSDM channel 7 is selected as regular channel */

#define IS_DFSDM_REGULAR_CHANNEL(CHANNEL) (((CHANNEL) == DFSDM_RegularChannel0) || \
                                           ((CHANNEL) == DFSDM_RegularChannel1) || \
                                           ((CHANNEL) == DFSDM_RegularChannel2) || \
                                           ((CHANNEL) == DFSDM_RegularChannel3) || \
                                           ((CHANNEL) == DFSDM_RegularChannel4) || \
                                           ((CHANNEL) == DFSDM_RegularChannel5) || \
                                           ((CHANNEL) == DFSDM_RegularChannel6) || \
                                           ((CHANNEL) == DFSDM_RegularChannel7))
/**
  * @}
  */

/** @defgroup DFSDM_Injected_Trigger_signal
  * @{
  */
#define DFSDM_Trigger_TIM1_TRGO          ((uint32_t)0x00000000)  /*!<  DFSDM Internal trigger 0 */
#define DFSDM_Trigger_TIM1_TRGO2         ((uint32_t)0x00000100)  /*!<  DFSDM Internal trigger 1 */
#define DFSDM_Trigger_TIM8_TRGO          ((uint32_t)0x00000200)  /*!<  DFSDM Internal trigger 2 */
#define DFSDM_Trigger_TIM8_TRGO2         ((uint32_t)0x00000300)  /*!<  DFSDM Internal trigger 3 */
#define DFSDM_Trigger_TIM3_TRGO          ((uint32_t)0x00000300)  /*!<  DFSDM Internal trigger 4 */
#define DFSDM_Trigger_TIM4_TRGO          ((uint32_t)0x00000400)  /*!<  DFSDM Internal trigger 5 */
#define DFSDM_Trigger_TIM16_OC1          ((uint32_t)0x00000400)  /*!<  DFSDM Internal trigger 6 */
#define DFSDM_Trigger_TIM6_TRGO          ((uint32_t)0x00000500)  /*!<  DFSDM Internal trigger 7 */
#define DFSDM_Trigger_TIM7_TRGO          ((uint32_t)0x00000500)  /*!<  DFSDM Internal trigger 8 */
#define DFSDM_Trigger_EXTI11             ((uint32_t)0x00000600)  /*!<  DFSDM External trigger 0 */
#define DFSDM_Trigger_EXTI15             ((uint32_t)0x00000700)  /*!<  DFSDM External trigger 1 */

#define IS_DFSDM0_INJ_TRIGGER(TRIG)      (((TRIG) == DFSDM_Trigger_TIM1_TRGO)  || \
                                          ((TRIG) == DFSDM_Trigger_TIM1_TRGO2) || \
                                          ((TRIG) == DFSDM_Trigger_TIM8_TRGO)  || \
                                          ((TRIG) == DFSDM_Trigger_TIM8_TRGO2) || \
                                          ((TRIG) == DFSDM_Trigger_TIM4_TRGO)  || \
                                          ((TRIG) == DFSDM_Trigger_TIM6_TRGO)  || \
                                          ((TRIG) == DFSDM_Trigger_TIM1_TRGO)  || \
                                          ((TRIG) == DFSDM_Trigger_EXTI15))

#define IS_DFSDM1_INJ_TRIGGER(TRIG)      IS_DFSDM0_INJ_TRIGGER(TRIG)
/**
  * @}
  */

/** @defgroup DFSDM_Trigger_Edge_selection
  * @{
  */
#define DFSDM_TriggerEdge_Disabled        ((uint32_t)0x00000000)  /*!<  DFSDM Trigger detection disabled */
#define DFSDM_TriggerEdge_Rising          ((uint32_t)0x00002000)  /*!<  DFSDM Each rising edge makes a request to launch an injected conversion */
#define DFSDM_TriggerEdge_Falling         ((uint32_t)0x00004000)  /*!<  DFSDM Each falling edge makes a request to launch an injected conversion */
#define DFSDM_TriggerEdge_BothEdges       ((uint32_t)0x00006000)  /*!<  DFSDM Both edges make a request to launch an injected conversion */

#define IS_DFSDM_TRIGGER_EDGE(EDGE)      (((EDGE) == DFSDM_TriggerEdge_Disabled) || \
                                          ((EDGE) == DFSDM_TriggerEdge_Rising) || \
                                          ((EDGE) == DFSDM_TriggerEdge_Falling) || \
                                          ((EDGE) == DFSDM_TriggerEdge_BothEdges))
/**
  * @}
  */

/** @defgroup DFSDM_Injected_Conversion_Mode_Selection
  * @{
  */
#define DFSDM_InjectConvMode_Single        ((uint32_t)0x00000000)  /*!<  DFSDM Trigger detection disabled */
#define DFSDM_InjectConvMode_Scan          ((uint32_t)0x00000010)  /*!<  DFSDM Each rising edge makes a request to launch an injected conversion */

#define IS_DFSDM_INJ_CONV_MODE(MODE)      (((MODE) == DFSDM_InjectConvMode_Single) || \
                                           ((MODE) == DFSDM_InjectConvMode_Scan))
/**
  * @}
  */

/** @defgroup DFSDM_Interrupts_Definition
  * @{
  */
#define DFSDM_IT_JEOC                            DFSDM_FLTCR2_JEOCIE
#define DFSDM_IT_REOC                            DFSDM_FLTCR2_REOCIE
#define DFSDM_IT_JOVR                            DFSDM_FLTCR2_JOVRIE
#define DFSDM_IT_ROVR                            DFSDM_FLTCR2_ROVRIE
#define DFSDM_IT_AWD                             DFSDM_FLTCR2_AWDIE
#define DFSDM_IT_SCD                             DFSDM_FLTCR2_SCDIE
#define DFSDM_IT_CKAB                            DFSDM_FLTCR2_CKABIE

#define IS_DFSDM_IT(IT)                         (((IT) == DFSDM_IT_JEOC) || \
                                                 ((IT) == DFSDM_IT_REOC) || \
                                                 ((IT) == DFSDM_IT_JOVR) || \
                                                 ((IT) == DFSDM_IT_ROVR) || \
                                                 ((IT) == DFSDM_IT_AWD)  || \
                                                 ((IT) == DFSDM_IT_SCD)  || \
                                                 ((IT) == DFSDM_IT_CKAB))
/**
  * @}
  */

/** @defgroup DFSDM_Flag_Definition
  * @{
  */
#define DFSDM_FLAG_JEOC                          DFSDM_FLTISR_JEOCF
#define DFSDM_FLAG_REOC                          DFSDM_FLTISR_REOCF
#define DFSDM_FLAG_JOVR                          DFSDM_FLTISR_JOVRF
#define DFSDM_FLAG_ROVR                          DFSDM_FLTISR_ROVRF
#define DFSDM_FLAG_AWD                           DFSDM_FLTISR_AWDF
#define DFSDM_FLAG_JCIP                          DFSDM_FLTISR_JCIP
#define DFSDM_FLAG_RCIP                          DFSDM_FLTISR_RCIP

#define IS_DFSDM_FLAG(FLAG)                     (((FLAG) == DFSDM_FLAG_JEOC) || \
                                                 ((FLAG) == DFSDM_FLAG_REOC) || \
                                                 ((FLAG) == DFSDM_FLAG_JOVR) || \
                                                 ((FLAG) == DFSDM_FLAG_ROVR) || \
                                                 ((FLAG) == DFSDM_FLAG_AWD)  || \
                                                 ((FLAG) == DFSDM_FLAG_JCIP) || \
                                                 ((FLAG) == DFSDM_FLAG_RCIP))
/**
  * @}
  */

/** @defgroup DFSDM_Clock_Absence_Flag_Definition
  * @{
  */
#define DFSDM_FLAG_CLKAbsence_Channel0           ((uint32_t)0x00010000)
#define DFSDM_FLAG_CLKAbsence_Channel1           ((uint32_t)0x00020000)
#define DFSDM_FLAG_CLKAbsence_Channel2           ((uint32_t)0x00040000)
#define DFSDM_FLAG_CLKAbsence_Channel3           ((uint32_t)0x00080000)
#define DFSDM_FLAG_CLKAbsence_Channel4           ((uint32_t)0x00100000)
#define DFSDM_FLAG_CLKAbsence_Channel5           ((uint32_t)0x00200000)
#define DFSDM_FLAG_CLKAbsence_Channel6           ((uint32_t)0x00400000)
#define DFSDM_FLAG_CLKAbsence_Channel7           ((uint32_t)0x00800000)

#define IS_DFSDM_CLK_ABS_FLAG(FLAG)             (((FLAG) == DFSDM_FLAG_CLKAbsence_Channel0) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel1) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel2) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel3) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel4) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel5) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel6) || \
                                                 ((FLAG) == DFSDM_FLAG_CLKAbsence_Channel7))
/**
  * @}
  */

/** @defgroup DFSDM_SCD_Flag_Definition
  * @{
  */
#define DFSDM_FLAG_SCD_Channel0                  ((uint32_t)0x01000000)
#define DFSDM_FLAG_SCD_Channel1                  ((uint32_t)0x02000000)
#define DFSDM_FLAG_SCD_Channel2                  ((uint32_t)0x04000000)
#define DFSDM_FLAG_SCD_Channel3                  ((uint32_t)0x08000000)
#define DFSDM_FLAG_SCD_Channel4                  ((uint32_t)0x10000000)
#define DFSDM_FLAG_SCD_Channel5                  ((uint32_t)0x20000000)
#define DFSDM_FLAG_SCD_Channel6                  ((uint32_t)0x40000000)
#define DFSDM_FLAG_SCD_Channel7                  ((uint32_t)0x80000000)

#define IS_DFSDM_SCD_FLAG(FLAG)                 (((FLAG) == DFSDM_FLAG_SCD_Channel0) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel1) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel2) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel3) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel4) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel5) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel6) || \
                                                 ((FLAG) == DFSDM_FLAG_SCD_Channel7))
/**
  * @}
  */

/** @defgroup DFSDM_Clear_Flag_Definition
  * @{
  */
#define DFSDM_CLEARF_JOVR                          DFSDM_FLTICR_CLRJOVRF
#define DFSDM_CLEARF_ROVR                          DFSDM_FLTICR_CLRROVRF

#define IS_DFSDM_CLEAR_FLAG(FLAG)                (((FLAG) == DFSDM_CLEARF_JOVR) || \
                                                  ((FLAG) == DFSDM_CLEARF_ROVR))
/**
  * @}
  */

/** @defgroup DFSDM_Clear_ClockAbs_Flag_Definition
  * @{
  */
#define DFSDM_CLEARF_CLKAbsence_Channel0           ((uint32_t)0x00010000)
#define DFSDM_CLEARF_CLKAbsence_Channel1           ((uint32_t)0x00020000)
#define DFSDM_CLEARF_CLKAbsence_Channel2           ((uint32_t)0x00040000)
#define DFSDM_CLEARF_CLKAbsence_Channel3           ((uint32_t)0x00080000)
#define DFSDM_CLEARF_CLKAbsence_Channel4           ((uint32_t)0x00100000)
#define DFSDM_CLEARF_CLKAbsence_Channel5           ((uint32_t)0x00200000)
#define DFSDM_CLEARF_CLKAbsence_Channel6           ((uint32_t)0x00400000)
#define DFSDM_CLEARF_CLKAbsence_Channel7           ((uint32_t)0x00800000)

#define IS_DFSDM_CLK_ABS_CLEARF(FLAG)           (((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel0) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel1) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel2) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel3) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel4) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel5) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel6) || \
                                                 ((FLAG) == DFSDM_CLEARF_CLKAbsence_Channel7))
/**
  * @}
  */

/** @defgroup DFSDM_Clear_Short_Circuit_Flag_Definition
  * @{
  */
#define DFSDM_CLEARF_SCD_Channel0           ((uint32_t)0x01000000)
#define DFSDM_CLEARF_SCD_Channel1           ((uint32_t)0x02000000)
#define DFSDM_CLEARF_SCD_Channel2           ((uint32_t)0x04000000)
#define DFSDM_CLEARF_SCD_Channel3           ((uint32_t)0x08000000)
#define DFSDM_CLEARF_SCD_Channel4           ((uint32_t)0x10000000)
#define DFSDM_CLEARF_SCD_Channel5           ((uint32_t)0x20000000)
#define DFSDM_CLEARF_SCD_Channel6           ((uint32_t)0x40000000)
#define DFSDM_CLEARF_SCD_Channel7           ((uint32_t)0x80000000)

#define IS_DFSDM_SCD_CHANNEL_FLAG(FLAG)         (((FLAG) == DFSDM_CLEARF_SCD_Channel0) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel1) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel2) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel3) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel4) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel5) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel6) || \
                                                 ((FLAG) == DFSDM_CLEARF_SCD_Channel7))
/**
  * @}
  */

/** @defgroup DFSDM_Clock_Absence_Interrupt_Definition
  * @{
  */
#define DFSDM_IT_CLKAbsence_Channel0           ((uint32_t)0x00010000)
#define DFSDM_IT_CLKAbsence_Channel1           ((uint32_t)0x00020000)
#define DFSDM_IT_CLKAbsence_Channel2           ((uint32_t)0x00040000)
#define DFSDM_IT_CLKAbsence_Channel3           ((uint32_t)0x00080000)
#define DFSDM_IT_CLKAbsence_Channel4           ((uint32_t)0x00100000)
#define DFSDM_IT_CLKAbsence_Channel5           ((uint32_t)0x00200000)
#define DFSDM_IT_CLKAbsence_Channel6           ((uint32_t)0x00400000)
#define DFSDM_IT_CLKAbsence_Channel7           ((uint32_t)0x00800000)

#define IS_DFSDM_CLK_ABS_IT(IT)               (((IT) == DFSDM_IT_CLKAbsence_Channel0) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel1) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel2) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel3) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel4) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel5) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel6) || \
                                               ((IT) == DFSDM_IT_CLKAbsence_Channel7))
/**
  * @}
  */

/** @defgroup DFSDM_SCD_Interrupt_Definition
  * @{
  */
#define DFSDM_IT_SCD_Channel0                  ((uint32_t)0x01000000)
#define DFSDM_IT_SCD_Channel1                  ((uint32_t)0x02000000)
#define DFSDM_IT_SCD_Channel2                  ((uint32_t)0x04000000)
#define DFSDM_IT_SCD_Channel3                  ((uint32_t)0x08000000)
#define DFSDM_IT_SCD_Channel4                  ((uint32_t)0x10000000)
#define DFSDM_IT_SCD_Channel5                  ((uint32_t)0x20000000)
#define DFSDM_IT_SCD_Channel6                  ((uint32_t)0x40000000)
#define DFSDM_IT_SCD_Channel7                  ((uint32_t)0x80000000)

#define IS_DFSDM_SCD_IT(IT)                   (((IT) == DFSDM_IT_SCD_Channel0) || \
                                               ((IT) == DFSDM_IT_SCD_Channel1) || \
                                               ((IT) == DFSDM_IT_SCD_Channel2) || \
                                               ((IT) == DFSDM_IT_SCD_Channel3) || \
                                               ((IT) == DFSDM_IT_SCD_Channel4) || \
                                               ((IT) == DFSDM_IT_SCD_Channel5) || \
                                               ((IT) == DFSDM_IT_SCD_Channel6) || \
                                               ((IT) == DFSDM_IT_SCD_Channel7))
/**
  * @}
  */

#define IS_DFSDM_DATA_RIGHT_BIT_SHIFT(SHIFT)  (SHIFT < 0x20 )

#define IS_DFSDM_OFFSET(OFFSET)               (OFFSET < 0x01000000 )

#define IS_DFSDM_ALL_CHANNEL(CHANNEL)   (((CHANNEL) == DFSDM1_Channel0) || \
                                         ((CHANNEL) == DFSDM1_Channel1) || \
                                         ((CHANNEL) == DFSDM1_Channel2) || \
                                         ((CHANNEL) == DFSDM1_Channel3))
                                         
#define IS_DFSDM_ALL_FILTER(FILTER)     (((FILTER) == DFSDM0) || \
                                         ((FILTER) == DFSDM1))

#define IS_DFSDM_SYNC_FILTER(FILTER)    (((FILTER) == DFSDM1))

#define IS_DFSDM_SINC_OVRSMPL_RATIO(RATIO)  ( RATIO < 0x401 ) & ( RATIO >= 0x001 )

#define IS_DFSDM_INTG_OVRSMPL_RATIO(RATIO)  ( RATIO < 0x101 ) & ( RATIO >= 0x001 )

#define IS_DFSDM_CLOCK_OUT_DIVIDER(DIVIDER) ( DIVIDER < 0x101 )

#define IS_DFSDM_CSD_THRESHOLD_VALUE(VALUE) ( VALUE < 256 )

#define IS_DFSDM_AWD_OVRSMPL_RATIO(RATIO)   ( RATIO < 33 ) & ( RATIO >= 0x001 )

#define IS_DFSDM_HIGH_THRESHOLD(VALUE)      (VALUE < 0x1000000 )
#define IS_DFSDM_LOW_THRESHOLD(VALUE)       (VALUE < 0x1000000 )
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initialization functions ***************************************************/
void DFSDM_DeInit(void);
void DFSDM_TransceiverInit(DFSDM_Channel_TypeDef* DFSDM_Channelx, DFSDM_TransceiverInitTypeDef* DFSDM_TransceiverInitStruct);
void DFSDM_TransceiverStructInit(DFSDM_TransceiverInitTypeDef* DFSDM_TransceiverInitStruct);
void DFSDM_FilterInit(DFSDM_TypeDef* DFSDMx, DFSDM_FilterInitTypeDef* DFSDM_FilterInitStruct);
void DFSDM_FilterStructInit(DFSDM_FilterInitTypeDef* DFSDM_FilterInitStruct);

/* Configuration functions ****************************************************/
void DFSDM_Cmd(FunctionalState NewState);
void DFSDM_ChannelCmd(DFSDM_Channel_TypeDef* DFSDM_Channelx, FunctionalState NewState);
void DFSDM_FilterCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState);
void DFSDM_ConfigClkOutputDivider(uint32_t DFSDM_ClkOutDivision);
void DFSDM_ConfigClkOutputSource(uint32_t DFSDM_ClkOutSource);
void DFSDM_SelectInjectedConversionMode(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_InjectConvMode);
void DFSDM_SelectInjectedChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_InjectedChannelx);
void DFSDM_SelectRegularChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_RegularChannelx);
void DFSDM_StartSoftwareInjectedConversion(DFSDM_TypeDef* DFSDMx);
void DFSDM_StartSoftwareRegularConversion(DFSDM_TypeDef* DFSDMx);
void DFSDM_SynchronousFilter0InjectedStart(DFSDM_TypeDef* DFSDMx);
void DFSDM_SynchronousFilter0RegularStart(DFSDM_TypeDef* DFSDMx);
void DFSDM_RegularContinuousModeCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState);
void DFSDM_InjectedContinuousModeCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState);
void DFSDM_FastModeCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState);
void DFSDM_ConfigInjectedTrigger(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_Trigger, uint32_t DFSDM_TriggerEdge);
void DFSDM_ConfigBRKShortCircuitDetector(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDBreak_i, FunctionalState NewState);
void DFSDM_ConfigBRKAnalogWatchDog(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDBreak_i, FunctionalState NewState);
void DFSDM_ConfigShortCircuitThreshold(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDThreshold);
void DFSDM_ConfigAnalogWatchdog(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint32_t DFSDM_AWDFastMode);
void DFSDM_ConfigAWDFilter(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t AWD_SincOrder, uint32_t AWD_SincOverSampleRatio);
uint32_t DFSDM_GetAWDConversionValue(DFSDM_Channel_TypeDef* DFSDM_Channelx);
void DFSDM_SetAWDThreshold(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_HighThreshold, uint32_t DFSDM_LowThreshold);
void DFSDM_SelectExtremesDetectorChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_ExtremChannelx);
int32_t DFSDM_GetRegularConversionData(DFSDM_TypeDef* DFSDMx);
int32_t DFSDM_GetInjectedConversionData(DFSDM_TypeDef* DFSDMx);
int32_t DFSDM_GetMaxValue(DFSDM_TypeDef* DFSDMx);
int32_t DFSDM_GetMinValue(DFSDM_TypeDef* DFSDMx);
int32_t DFSDM_GetMaxValueChannel(DFSDM_TypeDef* DFSDMx);
int32_t DFSDM_GetMinValueChannel(DFSDM_TypeDef* DFSDMx);
uint32_t DFSDM_GetConversionTime(DFSDM_TypeDef* DFSDMx);
void DFSDM_DMATransferConfig(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_DMAConversionMode, FunctionalState NewState);
/* Interrupts and flags management functions **********************************/
void DFSDM_ITConfig(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_IT, FunctionalState NewState);
void DFSDM_ITClockAbsenceCmd(FunctionalState NewState);
void DFSDM_ITShortCircuitDetectorCmd(FunctionalState NewState);

FlagStatus DFSDM_GetFlagStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_FLAG);
FlagStatus DFSDM_GetClockAbsenceFlagStatus(uint32_t DFSDM_FLAG_CLKAbsence);
FlagStatus DFSDM_GetShortCircuitFlagStatus(uint32_t DFSDM_FLAG_SCD);
FlagStatus DFSDM_GetWatchdogFlagStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint8_t DFSDM_Threshold);

void DFSDM_ClearFlag(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_CLEARF);
void DFSDM_ClearClockAbsenceFlag(uint32_t DFSDM_CLEARF_CLKAbsence);
void DFSDM_ClearShortCircuitFlag(uint32_t DFSDM_CLEARF_SCD);
void DFSDM_ClearAnalogWatchdogFlag(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint8_t DFSDM_Threshold);

ITStatus DFSDM_GetITStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_IT);
ITStatus DFSDM_GetClockAbsenceITStatus(uint32_t DFSDM_IT_CLKAbsence);
ITStatus DFSDM_GetGetShortCircuitITStatus(uint32_t DFSDM_IT_SCR);

#endif /* STM32F412xG */

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4XX_DFSDM_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
