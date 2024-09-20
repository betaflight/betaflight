/**
  *
  * @file    apm32f4xx_ddl_tmr.h
  * @brief   Header file of TMR DDL module.
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
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DDL_TMR_H
#define APM32F4xx_DDL_TMR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (TMR1) || defined (TMR2) || defined (TMR3) || defined (TMR4) || defined (TMR5) || defined (TMR6) || defined (TMR7) || defined (TMR8) || defined (TMR9) || defined (TMR10) || defined (TMR11) || defined (TMR12) || defined (TMR13) || defined (TMR14)

/** @defgroup TMR_DDL TMR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup TMR_DDL_Private_Variables TMR Private Variables
  * @{
  */
static const uint8_t OFFSET_TAB_CCMRx[] =
{
  0x00U,   /* 0: TMRx_CH1  */
  0x00U,   /* 1: TMRx_CH1N */
  0x00U,   /* 2: TMRx_CH2  */
  0x00U,   /* 3: TMRx_CH2N */
  0x04U,   /* 4: TMRx_CH3  */
  0x04U,   /* 5: TMRx_CH3N */
  0x04U    /* 6: TMRx_CH4  */
};

static const uint8_t SHIFT_TAB_OCxx[] =
{
  0U,            /* 0: OC1M, OC1FE, OC1PE */
  0U,            /* 1: - NA */
  8U,            /* 2: OC2M, OC2FE, OC2PE */
  0U,            /* 3: - NA */
  0U,            /* 4: OC3M, OC3FE, OC3PE */
  0U,            /* 5: - NA */
  8U             /* 6: OC4M, OC4FE, OC4PE */
};

static const uint8_t SHIFT_TAB_ICxx[] =
{
  0U,            /* 0: CC1S, IC1PSC, IC1F */
  0U,            /* 1: - NA */
  8U,            /* 2: CC2S, IC2PSC, IC2F */
  0U,            /* 3: - NA */
  0U,            /* 4: CC3S, IC3PSC, IC3F */
  0U,            /* 5: - NA */
  8U             /* 6: CC4S, IC4PSC, IC4F */
};

static const uint8_t SHIFT_TAB_CCxP[] =
{
  0U,            /* 0: CC1P */
  2U,            /* 1: CC1NP */
  4U,            /* 2: CC2P */
  6U,            /* 3: CC2NP */
  8U,            /* 4: CC3P */
  10U,           /* 5: CC3NP */
  12U            /* 6: CC4P */
};

static const uint8_t SHIFT_TAB_OISx[] =
{
  0U,            /* 0: OIS1 */
  1U,            /* 1: OIS1N */
  2U,            /* 2: OIS2 */
  3U,            /* 3: OIS2N */
  4U,            /* 4: OIS3 */
  5U,            /* 5: OIS3N */
  6U             /* 6: OIS4 */
};
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup TMR_DDL_Private_Constants TMR Private Constants
  * @{
  */


/* Remap mask definitions */
#define TMRx_OR_RMP_SHIFT  16U
#define TMRx_OR_RMP_MASK   0x0000FFFFU
#define TMR2_OR_RMP_MASK   (TMR_OR_RMPSEL << TMRx_OR_RMP_SHIFT)
#define TMR5_OR_RMP_MASK   (TMR_OR_TI4_RMPSEL << TMRx_OR_RMP_SHIFT)
#define TMR11_OR_RMP_MASK  (TMR_OR_TI1_RMPSEL << TMRx_OR_RMP_SHIFT)

/* Mask used to set the TDG[x:0] of the DTG bits of the TMRx_BDTR register */
#define DT_DELAY_1 ((uint8_t)0x7F)
#define DT_DELAY_2 ((uint8_t)0x3F)
#define DT_DELAY_3 ((uint8_t)0x1F)
#define DT_DELAY_4 ((uint8_t)0x1F)

/* Mask used to set the DTG[7:5] bits of the DTG bits of the TMRx_BDTR register */
#define DT_RANGE_1 ((uint8_t)0x00)
#define DT_RANGE_2 ((uint8_t)0x80)
#define DT_RANGE_3 ((uint8_t)0xC0)
#define DT_RANGE_4 ((uint8_t)0xE0)


/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup TMR_DDL_Private_Macros TMR Private Macros
  * @{
  */
/** @brief  Convert channel id into channel index.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval none
  */
#define TMR_GET_CHANNEL_INDEX( __CHANNEL__) \
  (((__CHANNEL__) == DDL_TMR_CHANNEL_CH1) ? 0U :\
   ((__CHANNEL__) == DDL_TMR_CHANNEL_CH1N) ? 1U :\
   ((__CHANNEL__) == DDL_TMR_CHANNEL_CH2) ? 2U :\
   ((__CHANNEL__) == DDL_TMR_CHANNEL_CH2N) ? 3U :\
   ((__CHANNEL__) == DDL_TMR_CHANNEL_CH3) ? 4U :\
   ((__CHANNEL__) == DDL_TMR_CHANNEL_CH3N) ? 5U : 6U)

/** @brief  Calculate the deadtime sampling period(in ps).
  * @param  __TMRCLK__ timer input clock frequency (in Hz).
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV1
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV2
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV4
  * @retval none
  */
#define TMR_CALC_DTS(__TMRCLK__, __CKD__)                                                        \
  (((__CKD__) == DDL_TMR_CLOCKDIVISION_DIV1) ? ((uint64_t)1000000000000U/(__TMRCLK__))         : \
   ((__CKD__) == DDL_TMR_CLOCKDIVISION_DIV2) ? ((uint64_t)1000000000000U/((__TMRCLK__) >> 1U)) : \
   ((uint64_t)1000000000000U/((__TMRCLK__) >> 2U)))
/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup TMR_DDL_ES_INIT TMR Exported Init structure
  * @{
  */

/**
  * @brief  TMR Time Base configuration structure definition.
  */
typedef struct
{
  uint16_t Prescaler;         /*!< Specifies the prescaler value used to divide the TMR clock.
                                   This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_TMR_SetPrescaler().*/

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TMR_DDL_EC_COUNTERMODE.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_TMR_SetCounterMode().*/

  uint32_t Autoreload;        /*!< Specifies the auto reload value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter must be a number between Min_Data=0x0000 and Max_Data=0xFFFF.
                                   Some timer instances may support 32 bits counters. In that case this parameter must
                                   be a number between 0x0000 and 0xFFFFFFFF.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_TMR_SetAutoReload().*/

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TMR_DDL_EC_CLOCKDIVISION.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_TMR_SetClockDivision().*/

  uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the REPCNT downcounter
                                   reaches zero, an update event is generated and counting restarts
                                   from the REPCNT value (N).
                                   This means in PWM mode that (N+1) corresponds to:
                                      - the number of PWM periods in edge-aligned mode
                                      - the number of half PWM period in center-aligned mode
                                   GP timers: this parameter must be a number between Min_Data = 0x00 and
                                   Max_Data = 0xFF.
                                   Advanced timers: this parameter must be a number between Min_Data = 0x0000 and
                                   Max_Data = 0xFFFF.

                                   This feature can be modified afterwards using unitary function
                                   @ref DDL_TMR_SetRepetitionCounter().*/
} DDL_TMR_InitTypeDef;

/**
  * @brief  TMR Output Compare configuration structure definition.
  */
typedef struct
{
  uint32_t OCMode;        /*!< Specifies the output mode.
                               This parameter can be a value of @ref TMR_DDL_EC_OCMODE.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_OC_SetMode().*/

  uint32_t OCState;       /*!< Specifies the TMR Output Compare state.
                               This parameter can be a value of @ref TMR_DDL_EC_OCSTATE.

                               This feature can be modified afterwards using unitary functions
                               @ref DDL_TMR_CC_EnableChannel() or @ref DDL_TMR_CC_DisableChannel().*/

  uint32_t OCNState;      /*!< Specifies the TMR complementary Output Compare state.
                               This parameter can be a value of @ref TMR_DDL_EC_OCSTATE.

                               This feature can be modified afterwards using unitary functions
                               @ref DDL_TMR_CC_EnableChannel() or @ref DDL_TMR_CC_DisableChannel().*/

  uint32_t CompareValue;  /*!< Specifies the Compare value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                               This feature can be modified afterwards using unitary function
                               DDL_TMR_OC_SetCompareCHx (x=1..6).*/

  uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref TMR_DDL_EC_OCPOLARITY.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_OC_SetPolarity().*/

  uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TMR_DDL_EC_OCPOLARITY.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_OC_SetPolarity().*/


  uint32_t OCIdleState;   /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_DDL_EC_OCIDLESTATE.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_OC_SetIdleState().*/

  uint32_t OCNIdleState;  /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_DDL_EC_OCIDLESTATE.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_OC_SetIdleState().*/
} DDL_TMR_OC_InitTypeDef;

/**
  * @brief  TMR Input Capture configuration structure definition.
  */

typedef struct
{

  uint32_t ICPolarity;    /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TMR_DDL_EC_IC_POLARITY.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_IC_SetPolarity().*/

  uint32_t ICActiveInput; /*!< Specifies the input.
                               This parameter can be a value of @ref TMR_DDL_EC_ACTIVEINPUT.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_IC_SetActiveInput().*/

  uint32_t ICPrescaler;   /*!< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TMR_DDL_EC_ICPSC.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_IC_SetPrescaler().*/

  uint32_t ICFilter;      /*!< Specifies the input capture filter.
                               This parameter can be a value of @ref TMR_DDL_EC_IC_FILTER.

                               This feature can be modified afterwards using unitary function
                               @ref DDL_TMR_IC_SetFilter().*/
} DDL_TMR_IC_InitTypeDef;


/**
  * @brief  TMR Encoder interface configuration structure definition.
  */
typedef struct
{
  uint32_t EncoderMode;     /*!< Specifies the encoder resolution (x2 or x4).
                                 This parameter can be a value of @ref TMR_DDL_EC_ENCODERMODE.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_SetEncoderMode().*/

  uint32_t IC1Polarity;     /*!< Specifies the active edge of TI1 input.
                                 This parameter can be a value of @ref TMR_DDL_EC_IC_POLARITY.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetPolarity().*/

  uint32_t IC1ActiveInput;  /*!< Specifies the TI1 input source
                                 This parameter can be a value of @ref TMR_DDL_EC_ACTIVEINPUT.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetActiveInput().*/

  uint32_t IC1Prescaler;    /*!< Specifies the TI1 input prescaler value.
                                 This parameter can be a value of @ref TMR_DDL_EC_ICPSC.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetPrescaler().*/

  uint32_t IC1Filter;       /*!< Specifies the TI1 input filter.
                                 This parameter can be a value of @ref TMR_DDL_EC_IC_FILTER.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetFilter().*/

  uint32_t IC2Polarity;      /*!< Specifies the active edge of TI2 input.
                                 This parameter can be a value of @ref TMR_DDL_EC_IC_POLARITY.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetPolarity().*/

  uint32_t IC2ActiveInput;  /*!< Specifies the TI2 input source
                                 This parameter can be a value of @ref TMR_DDL_EC_ACTIVEINPUT.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetActiveInput().*/

  uint32_t IC2Prescaler;    /*!< Specifies the TI2 input prescaler value.
                                 This parameter can be a value of @ref TMR_DDL_EC_ICPSC.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetPrescaler().*/

  uint32_t IC2Filter;       /*!< Specifies the TI2 input filter.
                                 This parameter can be a value of @ref TMR_DDL_EC_IC_FILTER.

                                 This feature can be modified afterwards using unitary function
                                 @ref DDL_TMR_IC_SetFilter().*/

} DDL_TMR_ENCODER_InitTypeDef;

/**
  * @brief  TMR Hall sensor interface configuration structure definition.
  */
typedef struct
{

  uint32_t IC1Polarity;        /*!< Specifies the active edge of TI1 input.
                                    This parameter can be a value of @ref TMR_DDL_EC_IC_POLARITY.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_TMR_IC_SetPolarity().*/

  uint32_t IC1Prescaler;       /*!< Specifies the TI1 input prescaler value.
                                    Prescaler must be set to get a maximum counter period longer than the
                                    time interval between 2 consecutive changes on the Hall inputs.
                                    This parameter can be a value of @ref TMR_DDL_EC_ICPSC.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_TMR_IC_SetPrescaler().*/

  uint32_t IC1Filter;          /*!< Specifies the TI1 input filter.
                                    This parameter can be a value of
                                    @ref TMR_DDL_EC_IC_FILTER.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_TMR_IC_SetFilter().*/

  uint32_t CommutationDelay;   /*!< Specifies the compare value to be loaded into the Capture Compare Register.
                                    A positive pulse (TRGO event) is generated with a programmable delay every time
                                    a change occurs on the Hall inputs.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

                                    This feature can be modified afterwards using unitary function
                                    @ref DDL_TMR_OC_SetCompareCH2().*/
} DDL_TMR_HALLSENSOR_InitTypeDef;

/**
  * @brief  BDTR (Break and Dead Time) structure definition
  */
typedef struct
{
  uint32_t OSSRState;            /*!< Specifies the Off-State selection used in Run mode.
                                      This parameter can be a value of @ref TMR_DDL_EC_OSSR

                                      This feature can be modified afterwards using unitary function
                                      @ref DDL_TMR_SetOffStates()

                                      @note This bit-field cannot be modified as long as LOCK level 2 has been
                                       programmed. */

  uint32_t OSSIState;            /*!< Specifies the Off-State used in Idle state.
                                      This parameter can be a value of @ref TMR_DDL_EC_OSSI

                                      This feature can be modified afterwards using unitary function
                                      @ref DDL_TMR_SetOffStates()

                                      @note This bit-field cannot be modified as long as LOCK level 2 has been
                                      programmed. */

  uint32_t LockLevel;            /*!< Specifies the LOCK level parameters.
                                      This parameter can be a value of @ref TMR_DDL_EC_LOCKLEVEL

                                      @note The LOCK bits can be written only once after the reset. Once the TMRx_BDTR
                                      register has been written, their content is frozen until the next reset.*/

  uint8_t DeadTime;              /*!< Specifies the delay time between the switching-off and the
                                      switching-on of the outputs.
                                      This parameter can be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This feature can be modified afterwards using unitary function
                                      @ref DDL_TMR_OC_SetDeadTime()

                                      @note This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been
                                       programmed. */

  uint16_t BreakState;           /*!< Specifies whether the TMR Break input is enabled or not.
                                      This parameter can be a value of @ref TMR_DDL_EC_BREAK_ENABLE

                                      This feature can be modified afterwards using unitary functions
                                      @ref DDL_TMR_EnableBRK() or @ref DDL_TMR_DisableBRK()

                                      @note This bit-field can not be modified as long as LOCK level 1 has been
                                      programmed. */

  uint32_t BreakPolarity;        /*!< Specifies the TMR Break Input pin polarity.
                                      This parameter can be a value of @ref TMR_DDL_EC_BREAK_POLARITY

                                      This feature can be modified afterwards using unitary function
                                      @ref DDL_TMR_ConfigBRK()

                                      @note This bit-field can not be modified as long as LOCK level 1 has been
                                      programmed. */

  uint32_t AutomaticOutput;      /*!< Specifies whether the TMR Automatic Output feature is enabled or not.
                                      This parameter can be a value of @ref TMR_DDL_EC_AUTOMATICOUTPUT_ENABLE

                                      This feature can be modified afterwards using unitary functions
                                      @ref DDL_TMR_EnableAutomaticOutput() or @ref DDL_TMR_DisableAutomaticOutput()

                                      @note This bit-field can not be modified as long as LOCK level 1 has been
                                      programmed. */
} DDL_TMR_BDT_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup TMR_DDL_Exported_Constants TMR Exported Constants
  * @{
  */

/** @defgroup TMR_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_TMR_ReadReg function.
  * @{
  */
#define DDL_TMR_STS_UIFLG                          TMR_STS_UIFLG           /*!< Update interrupt flag */
#define DDL_TMR_STS_CC1IFLG                        TMR_STS_CC1IFLG         /*!< Capture/compare 1 interrupt flag */
#define DDL_TMR_STS_CC2IFLG                        TMR_STS_CC2IFLG         /*!< Capture/compare 2 interrupt flag */
#define DDL_TMR_STS_CC3IFLG                        TMR_STS_CC3IFLG         /*!< Capture/compare 3 interrupt flag */
#define DDL_TMR_STS_CC4IFLG                        TMR_STS_CC4IFLG         /*!< Capture/compare 4 interrupt flag */
#define DDL_TMR_STS_COMIFLG                        TMR_STS_COMIFLG         /*!< COM interrupt flag */
#define DDL_TMR_STS_TRGIFLG                        TMR_STS_TRGIFLG         /*!< Trigger interrupt flag */
#define DDL_TMR_STS_BRKIFLG                        TMR_STS_BRKIFLG         /*!< Break interrupt flag */
#define DDL_TMR_STS_CC1RCFLG                       TMR_STS_CC1RCFLG        /*!< Capture/Compare 1 overcapture flag */
#define DDL_TMR_STS_CC2RCFLG                       TMR_STS_CC2RCFLG        /*!< Capture/Compare 2 overcapture flag */
#define DDL_TMR_STS_CC3RCFLG                       TMR_STS_CC3RCFLG        /*!< Capture/Compare 3 overcapture flag */
#define DDL_TMR_STS_CC4RCFLG                       TMR_STS_CC4RCFLG        /*!< Capture/Compare 4 overcapture flag */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup TMR_DDL_EC_BREAK_ENABLE Break Enable
  * @{
  */
#define DDL_TMR_BREAK_DISABLE            0x00000000U             /*!< Break function disabled */
#define DDL_TMR_BREAK_ENABLE             TMR_BDT_BRKEN            /*!< Break function enabled */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_AUTOMATICOUTPUT_ENABLE Automatic output enable
  * @{
  */
#define DDL_TMR_AUTOMATICOUTPUT_DISABLE         0x00000000U             /*!< MOE can be set only by software */
#define DDL_TMR_AUTOMATICOUTPUT_ENABLE          TMR_BDT_AOEN            /*!< MOE can be set by software or automatically at the next update event */
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/** @defgroup TMR_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_TMR_ReadReg and  DDL_TMR_WriteReg functions.
  * @{
  */
#define DDL_TMR_DIEN_UIEN                        TMR_DIEN_UIEN         /*!< Update interrupt enable */
#define DDL_TMR_DIEN_CC1IEN                      TMR_DIEN_CC1IEN       /*!< Capture/compare 1 interrupt enable */
#define DDL_TMR_DIEN_CC2IEN                      TMR_DIEN_CC2IEN       /*!< Capture/compare 2 interrupt enable */
#define DDL_TMR_DIEN_CC3IEN                      TMR_DIEN_CC3IEN       /*!< Capture/compare 3 interrupt enable */
#define DDL_TMR_DIEN_CC4IEN                      TMR_DIEN_CC4IEN       /*!< Capture/compare 4 interrupt enable */
#define DDL_TMR_DIEN_COMIEN                      TMR_DIEN_COMIEN       /*!< COM interrupt enable */
#define DDL_TMR_DIEN_TRGIEN                        TMR_DIEN_TRGIEN         /*!< Trigger interrupt enable */
#define DDL_TMR_DIEN_BRKIEN                        TMR_DIEN_BRKIEN         /*!< Break interrupt enable */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_UPDATESOURCE Update Source
  * @{
  */
#define DDL_TMR_UPDATESOURCE_REGULAR            0x00000000U          /*!< Counter overflow/underflow, Setting the UG bit or Update generation through the slave mode controller generates an update request */
#define DDL_TMR_UPDATESOURCE_COUNTER            TMR_CTRL1_URSSEL          /*!< Only counter overflow/underflow generates an update request */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ONEPULSEMODE One Pulse Mode
  * @{
  */
#define DDL_TMR_ONEPULSEMODE_SINGLE             TMR_CTRL1_SPMEN          /*!< Counter stops counting at the next update event */
#define DDL_TMR_ONEPULSEMODE_REPETITIVE         0x00000000U          /*!< Counter is not stopped at update event */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_COUNTERMODE Counter Mode
  * @{
  */
#define DDL_TMR_COUNTERMODE_UP                  0x00000000U          /*!<Counter used as upcounter */
#define DDL_TMR_COUNTERMODE_DOWN                TMR_CTRL1_CNTDIR          /*!< Counter used as downcounter */
#define DDL_TMR_COUNTERMODE_CENTER_DOWN         TMR_CTRL1_CAMSEL_0        /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting down. */
#define DDL_TMR_COUNTERMODE_CENTER_UP           TMR_CTRL1_CAMSEL_1        /*!<The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up */
#define DDL_TMR_COUNTERMODE_CENTER_UP_DOWN      TMR_CTRL1_CAMSEL          /*!< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up or down. */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_CLOCKDIVISION Clock Division
  * @{
  */
#define DDL_TMR_CLOCKDIVISION_DIV1              0x00000000U          /*!< tDTS=tCK_INT */
#define DDL_TMR_CLOCKDIVISION_DIV2              TMR_CTRL1_CLKDIV_0        /*!< tDTS=2*tCK_INT */
#define DDL_TMR_CLOCKDIVISION_DIV4              TMR_CTRL1_CLKDIV_1        /*!< tDTS=4*tCK_INT */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_COUNTERDIRECTION Counter Direction
  * @{
  */
#define DDL_TMR_COUNTERDIRECTION_UP             0x00000000U          /*!< Timer counter counts up */
#define DDL_TMR_COUNTERDIRECTION_DOWN           TMR_CTRL1_CNTDIR          /*!< Timer counter counts down */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_CCUPDATESOURCE Capture Compare  Update Source
  * @{
  */
#define DDL_TMR_CCUPDATESOURCE_COMG_ONLY        0x00000000U          /*!< Capture/compare control bits are updated by setting the COMG bit only */
#define DDL_TMR_CCUPDATESOURCE_COMG_AND_TRGI    TMR_CTRL2_CCUSEL         /*!< Capture/compare control bits are updated by setting the COMG bit or when a rising edge occurs on trigger input (TRGI) */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_CCDMAREQUEST Capture Compare DMA Request
  * @{
  */
#define DDL_TMR_CCDMAREQUEST_CC                 0x00000000U          /*!< CCx DMA request sent when CCx event occurs */
#define DDL_TMR_CCDMAREQUEST_UPDATE             TMR_CTRL2_CCDSEL         /*!< CCx DMA requests sent when update event occurs */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_LOCKLEVEL Lock Level
  * @{
  */
#define DDL_TMR_LOCKLEVEL_OFF                   0x00000000U          /*!< LOCK OFF - No bit is write protected */
#define DDL_TMR_LOCKLEVEL_1                     TMR_BDT_LOCKCFG_0      /*!< LOCK Level 1 */
#define DDL_TMR_LOCKLEVEL_2                     TMR_BDT_LOCKCFG_1      /*!< LOCK Level 2 */
#define DDL_TMR_LOCKLEVEL_3                     TMR_BDT_LOCKCFG        /*!< LOCK Level 3 */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_CHANNEL Channel
  * @{
  */
#define DDL_TMR_CHANNEL_CH1                     TMR_CCEN_CC1EN     /*!< Timer input/output channel 1 */
#define DDL_TMR_CHANNEL_CH1N                    TMR_CCEN_CC1NEN    /*!< Timer complementary output channel 1 */
#define DDL_TMR_CHANNEL_CH2                     TMR_CCEN_CC2EN     /*!< Timer input/output channel 2 */
#define DDL_TMR_CHANNEL_CH2N                    TMR_CCEN_CC2NEN    /*!< Timer complementary output channel 2 */
#define DDL_TMR_CHANNEL_CH3                     TMR_CCEN_CC3EN     /*!< Timer input/output channel 3 */
#define DDL_TMR_CHANNEL_CH3N                    TMR_CCEN_CC3NEN    /*!< Timer complementary output channel 3 */
#define DDL_TMR_CHANNEL_CH4                     TMR_CCEN_CC4EN     /*!< Timer input/output channel 4 */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup TMR_DDL_EC_OCSTATE Output Configuration State
  * @{
  */
#define DDL_TMR_OCSTATE_DISABLE                 0x00000000U             /*!< OCx is not active */
#define DDL_TMR_OCSTATE_ENABLE                  TMR_CCEN_CC1EN           /*!< OCx signal is output on the corresponding output pin */
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/** @defgroup TMR_DDL_EC_OCMODE Output Configuration Mode
  * @{
  */
#define DDL_TMR_OCMODE_FROZEN                   0x00000000U                                              /*!<The comparison between the output compare register TMRx_CCRy and the counter TMRx_CNT has no effect on the output channel level */
#define DDL_TMR_OCMODE_ACTIVE                   TMR_CCM1_OC1MOD_0                                         /*!<OCyREF is forced high on compare match*/
#define DDL_TMR_OCMODE_INACTIVE                 TMR_CCM1_OC1MOD_1                                         /*!<OCyREF is forced low on compare match*/
#define DDL_TMR_OCMODE_TOGGLE                   (TMR_CCM1_OC1MOD_1 | TMR_CCM1_OC1MOD_0)                    /*!<OCyREF toggles on compare match*/
#define DDL_TMR_OCMODE_FORCED_INACTIVE          TMR_CCM1_OC1MOD_2                                         /*!<OCyREF is forced low*/
#define DDL_TMR_OCMODE_FORCED_ACTIVE            (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_0)                    /*!<OCyREF is forced high*/
#define DDL_TMR_OCMODE_PWM1                     (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_1)                    /*!<In upcounting, channel y is active as long as TMRx_CNT<TMRx_CCRy else inactive.  In downcounting, channel y is inactive as long as TMRx_CNT>TMRx_CCRy else active.*/
#define DDL_TMR_OCMODE_PWM2                     (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_1 | TMR_CCM1_OC1MOD_0) /*!<In upcounting, channel y is inactive as long as TMRx_CNT<TMRx_CCRy else active.  In downcounting, channel y is active as long as TMRx_CNT>TMRx_CCRy else inactive*/
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_OCPOLARITY Output Configuration Polarity
  * @{
  */
#define DDL_TMR_OCPOLARITY_HIGH                 0x00000000U                 /*!< OCxactive high*/
#define DDL_TMR_OCPOLARITY_LOW                  TMR_CCEN_CC1POL               /*!< OCxactive low*/
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_OCIDLESTATE Output Configuration Idle State
  * @{
  */
#define DDL_TMR_OCIDLESTATE_LOW                 0x00000000U             /*!<OCx=0 (after a dead-time if OC is implemented) when MOE=0*/
#define DDL_TMR_OCIDLESTATE_HIGH                TMR_CTRL2_OC1OIS            /*!<OCx=1 (after a dead-time if OC is implemented) when MOE=0*/
/**
  * @}
  */


/** @defgroup TMR_DDL_EC_ACTIVEINPUT Active Input Selection
  * @{
  */
#define DDL_TMR_ACTIVEINPUT_DIRECTTI            (TMR_CCM1_CC1SEL_0 << 16U) /*!< ICx is mapped on TIx */
#define DDL_TMR_ACTIVEINPUT_INDIRECTTI          (TMR_CCM1_CC1SEL_1 << 16U) /*!< ICx is mapped on TIy */
#define DDL_TMR_ACTIVEINPUT_TRC                 (TMR_CCM1_CC1SEL << 16U)   /*!< ICx is mapped on TRC */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ICPSC Input Configuration Prescaler
  * @{
  */
#define DDL_TMR_ICPSC_DIV1                      0x00000000U                    /*!< No prescaler, capture is done each time an edge is detected on the capture input */
#define DDL_TMR_ICPSC_DIV2                      (TMR_CCM1_IC1PSC_0 << 16U)    /*!< Capture is done once every 2 events */
#define DDL_TMR_ICPSC_DIV4                      (TMR_CCM1_IC1PSC_1 << 16U)    /*!< Capture is done once every 4 events */
#define DDL_TMR_ICPSC_DIV8                      (TMR_CCM1_IC1PSC << 16U)      /*!< Capture is done once every 8 events */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_IC_FILTER Input Configuration Filter
  * @{
  */
#define DDL_TMR_IC_FILTER_FDIV1                 0x00000000U                                                        /*!< No filter, sampling is done at fDTS */
#define DDL_TMR_IC_FILTER_FDIV1_N2              (TMR_CCM1_IC1F_0 << 16U)                                          /*!< fSAMPLING=fCK_INT, N=2 */
#define DDL_TMR_IC_FILTER_FDIV1_N4              (TMR_CCM1_IC1F_1 << 16U)                                          /*!< fSAMPLING=fCK_INT, N=4 */
#define DDL_TMR_IC_FILTER_FDIV1_N8              ((TMR_CCM1_IC1F_1 | TMR_CCM1_IC1F_0) << 16U)                     /*!< fSAMPLING=fCK_INT, N=8 */
#define DDL_TMR_IC_FILTER_FDIV2_N6              (TMR_CCM1_IC1F_2 << 16U)                                          /*!< fSAMPLING=fDTS/2, N=6 */
#define DDL_TMR_IC_FILTER_FDIV2_N8              ((TMR_CCM1_IC1F_2 | TMR_CCM1_IC1F_0) << 16U)                     /*!< fSAMPLING=fDTS/2, N=8 */
#define DDL_TMR_IC_FILTER_FDIV4_N6              ((TMR_CCM1_IC1F_2 | TMR_CCM1_IC1F_1) << 16U)                     /*!< fSAMPLING=fDTS/4, N=6 */
#define DDL_TMR_IC_FILTER_FDIV4_N8              ((TMR_CCM1_IC1F_2 | TMR_CCM1_IC1F_1 | TMR_CCM1_IC1F_0) << 16U)  /*!< fSAMPLING=fDTS/4, N=8 */
#define DDL_TMR_IC_FILTER_FDIV8_N6              (TMR_CCM1_IC1F_3 << 16U)                                          /*!< fSAMPLING=fDTS/8, N=6 */
#define DDL_TMR_IC_FILTER_FDIV8_N8              ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_0) << 16U)                     /*!< fSAMPLING=fDTS/8, N=8 */
#define DDL_TMR_IC_FILTER_FDIV16_N5             ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_1) << 16U)                     /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_TMR_IC_FILTER_FDIV16_N6             ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_1 | TMR_CCM1_IC1F_0) << 16U)  /*!< fSAMPLING=fDTS/16, N=6 */
#define DDL_TMR_IC_FILTER_FDIV16_N8             ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_2) << 16U)                     /*!< fSAMPLING=fDTS/16, N=8 */
#define DDL_TMR_IC_FILTER_FDIV32_N5             ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_2 | TMR_CCM1_IC1F_0) << 16U)  /*!< fSAMPLING=fDTS/32, N=5 */
#define DDL_TMR_IC_FILTER_FDIV32_N6             ((TMR_CCM1_IC1F_3 | TMR_CCM1_IC1F_2 | TMR_CCM1_IC1F_1) << 16U)  /*!< fSAMPLING=fDTS/32, N=6 */
#define DDL_TMR_IC_FILTER_FDIV32_N8             (TMR_CCM1_IC1F << 16U)                                            /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_IC_POLARITY Input Configuration Polarity
  * @{
  */
#define DDL_TMR_IC_POLARITY_RISING              0x00000000U                      /*!< The circuit is sensitive to TIxFP1 rising edge, TIxFP1 is not inverted */
#define DDL_TMR_IC_POLARITY_FALLING             TMR_CCEN_CC1POL                    /*!< The circuit is sensitive to TIxFP1 falling edge, TIxFP1 is inverted */
#define DDL_TMR_IC_POLARITY_BOTHEDGE            (TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL) /*!< The circuit is sensitive to both TIxFP1 rising and falling edges, TIxFP1 is not inverted */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_CLOCKSOURCE Clock Source
  * @{
  */
#define DDL_TMR_CLOCKSOURCE_INTERNAL            0x00000000U                                          /*!< The timer is clocked by the internal clock provided from the RCC */
#define DDL_TMR_CLOCKSOURCE_EXT_MODE1           (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_1 | TMR_SMCTRL_SMFSEL_0)   /*!< Counter counts at each rising or falling edge on a selected input*/
#define DDL_TMR_CLOCKSOURCE_EXT_MODE2           TMR_SMCTRL_ECEN                                         /*!< Counter counts at each rising or falling edge on the external trigger input ETR */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ENCODERMODE Encoder Mode
  * @{
  */
#define DDL_TMR_ENCODERMODE_X2_TI1                     TMR_SMCTRL_SMFSEL_0                                                     /*!< Quadrature encoder mode 1, x2 mode - Counter counts up/down on TI1FP1 edge depending on TI2FP2 level */
#define DDL_TMR_ENCODERMODE_X2_TI2                     TMR_SMCTRL_SMFSEL_1                                                     /*!< Quadrature encoder mode 2, x2 mode - Counter counts up/down on TI2FP2 edge depending on TI1FP1 level */
#define DDL_TMR_ENCODERMODE_X4_TI12                   (TMR_SMCTRL_SMFSEL_1 | TMR_SMCTRL_SMFSEL_0)                                   /*!< Quadrature encoder mode 3, x4 mode - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_TRGO Trigger Output
  * @{
  */
#define DDL_TMR_TRGO_RESET                      0x00000000U                                     /*!< UG bit from the TMRx_EGR register is used as trigger output */
#define DDL_TMR_TRGO_ENABLE                     TMR_CTRL2_MMSEL_0                                   /*!< Counter Enable signal (CNT_EN) is used as trigger output */
#define DDL_TMR_TRGO_UPDATE                     TMR_CTRL2_MMSEL_1                                   /*!< Update event is used as trigger output */
#define DDL_TMR_TRGO_CC1IF                      (TMR_CTRL2_MMSEL_1 | TMR_CTRL2_MMSEL_0)                 /*!< CC1 capture or a compare match is used as trigger output */
#define DDL_TMR_TRGO_OC1REF                     TMR_CTRL2_MMSEL_2                                   /*!< OC1REF signal is used as trigger output */
#define DDL_TMR_TRGO_OC2REF                     (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_0)                 /*!< OC2REF signal is used as trigger output */
#define DDL_TMR_TRGO_OC3REF                     (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_1)                 /*!< OC3REF signal is used as trigger output */
#define DDL_TMR_TRGO_OC4REF                     (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_1 | TMR_CTRL2_MMSEL_0) /*!< OC4REF signal is used as trigger output */
/**
  * @}
  */


/** @defgroup TMR_DDL_EC_SLAVEMODE Slave Mode
  * @{
  */
#define DDL_TMR_SLAVEMODE_DISABLED              0x00000000U                         /*!< Slave mode disabled */
#define DDL_TMR_SLAVEMODE_RESET                 TMR_SMCTRL_SMFSEL_2                      /*!< Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter */
#define DDL_TMR_SLAVEMODE_GATED                 (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_0)   /*!< Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high */
#define DDL_TMR_SLAVEMODE_TRIGGER               (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_1)   /*!< Trigger Mode - The counter starts at a rising edge of the trigger TRGI */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_TS Trigger Selection
  * @{
  */
#define DDL_TMR_TS_ITR0                         0x00000000U                                                     /*!< Internal Trigger 0 (ITR0) is used as trigger input */
#define DDL_TMR_TS_ITR1                         TMR_SMCTRL_TRGSEL_0                                                   /*!< Internal Trigger 1 (ITR1) is used as trigger input */
#define DDL_TMR_TS_ITR2                         TMR_SMCTRL_TRGSEL_1                                                   /*!< Internal Trigger 2 (ITR2) is used as trigger input */
#define DDL_TMR_TS_ITR3                         (TMR_SMCTRL_TRGSEL_0 | TMR_SMCTRL_TRGSEL_1)                                 /*!< Internal Trigger 3 (ITR3) is used as trigger input */
#define DDL_TMR_TS_TI1F_ED                      TMR_SMCTRL_TRGSEL_2                                                   /*!< TI1 Edge Detector (TI1F_ED) is used as trigger input */
#define DDL_TMR_TS_TI1FP1                       (TMR_SMCTRL_TRGSEL_2 | TMR_SMCTRL_TRGSEL_0)                                 /*!< Filtered Timer Input 1 (TI1FP1) is used as trigger input */
#define DDL_TMR_TS_TI2FP2                       (TMR_SMCTRL_TRGSEL_2 | TMR_SMCTRL_TRGSEL_1)                                 /*!< Filtered Timer Input 2 (TI12P2) is used as trigger input */
#define DDL_TMR_TS_ETRF                         (TMR_SMCTRL_TRGSEL_2 | TMR_SMCTRL_TRGSEL_1 | TMR_SMCTRL_TRGSEL_0)                 /*!< Filtered external Trigger (ETRF) is used as trigger input */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ETR_POLARITY External Trigger Polarity
  * @{
  */
#define DDL_TMR_ETR_POLARITY_NONINVERTED        0x00000000U             /*!< ETR is non-inverted, active at high level or rising edge */
#define DDL_TMR_ETR_POLARITY_INVERTED           TMR_SMCTRL_ETPOL            /*!< ETR is inverted, active at low level or falling edge */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ETR_PRESCALER External Trigger Prescaler
  * @{
  */
#define DDL_TMR_ETR_PRESCALER_DIV1              0x00000000U             /*!< ETR prescaler OFF */
#define DDL_TMR_ETR_PRESCALER_DIV2              TMR_SMCTRL_ETPCFG_0         /*!< ETR frequency is divided by 2 */
#define DDL_TMR_ETR_PRESCALER_DIV4              TMR_SMCTRL_ETPCFG_1         /*!< ETR frequency is divided by 4 */
#define DDL_TMR_ETR_PRESCALER_DIV8              TMR_SMCTRL_ETPCFG           /*!< ETR frequency is divided by 8 */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_ETR_FILTER External Trigger Filter
  * @{
  */
#define DDL_TMR_ETR_FILTER_FDIV1                0x00000000U                                          /*!< No filter, sampling is done at fDTS */
#define DDL_TMR_ETR_FILTER_FDIV1_N2             TMR_SMCTRL_ETFCFG_0                                       /*!< fSAMPLING=fCK_INT, N=2 */
#define DDL_TMR_ETR_FILTER_FDIV1_N4             TMR_SMCTRL_ETFCFG_1                                       /*!< fSAMPLING=fCK_INT, N=4 */
#define DDL_TMR_ETR_FILTER_FDIV1_N8             (TMR_SMCTRL_ETFCFG_1 | TMR_SMCTRL_ETFCFG_0)                    /*!< fSAMPLING=fCK_INT, N=8 */
#define DDL_TMR_ETR_FILTER_FDIV2_N6             TMR_SMCTRL_ETFCFG_2                                       /*!< fSAMPLING=fDTS/2, N=6 */
#define DDL_TMR_ETR_FILTER_FDIV2_N8             (TMR_SMCTRL_ETFCFG_2 | TMR_SMCTRL_ETFCFG_0)                    /*!< fSAMPLING=fDTS/2, N=8 */
#define DDL_TMR_ETR_FILTER_FDIV4_N6             (TMR_SMCTRL_ETFCFG_2 | TMR_SMCTRL_ETFCFG_1)                    /*!< fSAMPLING=fDTS/4, N=6 */
#define DDL_TMR_ETR_FILTER_FDIV4_N8             (TMR_SMCTRL_ETFCFG_2 | TMR_SMCTRL_ETFCFG_1 | TMR_SMCTRL_ETFCFG_0)   /*!< fSAMPLING=fDTS/4, N=8 */
#define DDL_TMR_ETR_FILTER_FDIV8_N6             TMR_SMCTRL_ETFCFG_3                                       /*!< fSAMPLING=fDTS/8, N=8 */
#define DDL_TMR_ETR_FILTER_FDIV8_N8             (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_0)                    /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_TMR_ETR_FILTER_FDIV16_N5            (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_1)                    /*!< fSAMPLING=fDTS/16, N=6 */
#define DDL_TMR_ETR_FILTER_FDIV16_N6            (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_1 | TMR_SMCTRL_ETFCFG_0)   /*!< fSAMPLING=fDTS/16, N=8 */
#define DDL_TMR_ETR_FILTER_FDIV16_N8            (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_2)                    /*!< fSAMPLING=fDTS/16, N=5 */
#define DDL_TMR_ETR_FILTER_FDIV32_N5            (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_2 | TMR_SMCTRL_ETFCFG_0)   /*!< fSAMPLING=fDTS/32, N=5 */
#define DDL_TMR_ETR_FILTER_FDIV32_N6            (TMR_SMCTRL_ETFCFG_3 | TMR_SMCTRL_ETFCFG_2 | TMR_SMCTRL_ETFCFG_1)   /*!< fSAMPLING=fDTS/32, N=6 */
#define DDL_TMR_ETR_FILTER_FDIV32_N8            TMR_SMCTRL_ETFCFG                                         /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */


/** @defgroup TMR_DDL_EC_BREAK_POLARITY break polarity
  * @{
  */
#define DDL_TMR_BREAK_POLARITY_LOW              0x00000000U               /*!< Break input BRK is active low */
#define DDL_TMR_BREAK_POLARITY_HIGH             TMR_BDT_BRKPOL              /*!< Break input BRK is active high */
/**
  * @}
  */




/** @defgroup TMR_DDL_EC_OSSI OSSI
  * @{
  */
#define DDL_TMR_OSSI_DISABLE                    0x00000000U             /*!< When inactive, OCx/OCxN outputs are disabled */
#define DDL_TMR_OSSI_ENABLE                     TMR_BDT_IMOS           /*!< When inactive, OxC/OCxN outputs are first forced with their inactive level then forced to their idle level after the deadtime */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_OSSR OSSR
  * @{
  */
#define DDL_TMR_OSSR_DISABLE                    0x00000000U            /*!< When inactive, OCx/OCxN outputs are disabled */
#define DDL_TMR_OSSR_ENABLE                     TMR_BDT_RMOS           /*!< When inactive, OC/OCN outputs are enabled with their inactive level as soon as CCxE=1 or CCxNE=1 */
/**
  * @}
  */


/** @defgroup TMR_DDL_EC_DMABURST_BASEADDR DMA Burst Base Address
  * @{
  */                    
#define DDL_TMR_DMABURST_BASEADDR_CTRL1         0x00000000U                                                                             /*!< TMRx_CTRL1 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CTRL2         TMR_DCTRL_DBADDR_0                                                                      /*!< TMRx_CTRL2 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_SMCTRL        TMR_DCTRL_DBADDR_1                                                                      /*!< TMRx_SMCTRL register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_DIEN          (TMR_DCTRL_DBADDR_1 |  TMR_DCTRL_DBADDR_0)                                              /*!< TMRx_DIEN register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_STS           TMR_DCTRL_DBADDR_2                                                                      /*!< TMRx_STS register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CEG           (TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_0)                                               /*!< TMRx_CEG register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CCM1          (TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_1)                                               /*!< TMRx_CCM1 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CCM2          (TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_1 | TMR_DCTRL_DBADDR_0)                          /*!< TMRx_CCM2 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CCEN          TMR_DCTRL_DBADDR_3                                                                      /*!< TMRx_CCEN register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CNT           (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_0)                                               /*!< TMRx_CNT register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_PSC           (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_1)                                               /*!< TMRx_PSC register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_AUTORLD       (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_1 | TMR_DCTRL_DBADDR_0)                          /*!< TMRx_AUTORLD register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_REPCNT        (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_2)                                               /*!< TMRx_REPCNT register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CC1           (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_0)                          /*!< TMRx_CC1 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CC2           (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_1)                          /*!< TMRx_CC2 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CC3           (TMR_DCTRL_DBADDR_3 | TMR_DCTRL_DBADDR_2 | TMR_DCTRL_DBADDR_1 | TMR_DCTRL_DBADDR_0)     /*!< TMRx_CC3 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_CC4           TMR_DCTRL_DBADDR_4                                                                      /*!< TMRx_CC4 register is the DMA base address for DMA burst */
#define DDL_TMR_DMABURST_BASEADDR_BDT           (TMR_DCTRL_DBADDR_4 | TMR_DCTRL_DBADDR_0)                                               /*!< TMRx_BDT register is the DMA base address for DMA burst */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_DMABURST_LENGTH DMA Burst Length
  * @{
  */
#define DDL_TMR_DMABURST_LENGTH_1TRANSFER       0x00000000U                                                     /*!< Transfer is done to 1 register starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_2TRANSFERS      TMR_DCTRL_DBLEN_0                                                   /*!< Transfer is done to 2 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_3TRANSFERS      TMR_DCTRL_DBLEN_1                                                   /*!< Transfer is done to 3 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_4TRANSFERS      (TMR_DCTRL_DBLEN_1 |  TMR_DCTRL_DBLEN_0)                                /*!< Transfer is done to 4 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_5TRANSFERS      TMR_DCTRL_DBLEN_2                                                   /*!< Transfer is done to 5 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_6TRANSFERS      (TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_0)                                 /*!< Transfer is done to 6 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_7TRANSFERS      (TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_1)                                 /*!< Transfer is done to 7 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_8TRANSFERS      (TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_1 | TMR_DCTRL_DBLEN_0)                 /*!< Transfer is done to 1 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_9TRANSFERS      TMR_DCTRL_DBLEN_3                                                   /*!< Transfer is done to 9 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_10TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_0)                                 /*!< Transfer is done to 10 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_11TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_1)                                 /*!< Transfer is done to 11 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_12TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_1 | TMR_DCTRL_DBLEN_0)                 /*!< Transfer is done to 12 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_13TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_2)                                 /*!< Transfer is done to 13 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_14TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_0)                 /*!< Transfer is done to 14 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_15TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_1)                 /*!< Transfer is done to 15 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_16TRANSFERS     (TMR_DCTRL_DBLEN_3 | TMR_DCTRL_DBLEN_2 | TMR_DCTRL_DBLEN_1 | TMR_DCTRL_DBLEN_0) /*!< Transfer is done to 16 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_17TRANSFERS     TMR_DCTRL_DBLEN_4                                                   /*!< Transfer is done to 17 registers starting from the DMA burst base address */
#define DDL_TMR_DMABURST_LENGTH_18TRANSFERS     (TMR_DCTRL_DBLEN_4 |  TMR_DCTRL_DBLEN_0)                                /*!< Transfer is done to 18 registers starting from the DMA burst base address */
/**
  * @}
  */


/** @defgroup TMR_DDL_EC_TMR2_ITR1_RMP_TMR8  TMR2 Internal Trigger1 Remap TMR8
  * @{
  */
#define DDL_TMR_TMR2_ITR1_RMP_TMR8_TRGO    TMR2_OR_RMP_MASK                        /*!< TMR2_ITR1 is connected to TMR8_TRGO */
#define DDL_TMR_TMR2_ITR1_RMP_ETH_PTP      (TMR_OR_RMPSEL_0 | TMR2_OR_RMP_MASK)  /*!< TMR2_ITR1 is connected to ETH_PTP */
#define DDL_TMR_TMR2_ITR1_RMP_OTG_FS_SOF   (TMR_OR_RMPSEL_1 | TMR2_OR_RMP_MASK)  /*!< TMR2_ITR1 is connected to OTG_FS SOF */
#define DDL_TMR_TMR2_ITR1_RMP_OTG_HS_SOF   (TMR_OR_RMPSEL | TMR2_OR_RMP_MASK)    /*!< TMR2_ITR1 is connected to OTG_HS SOF */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_TMR5_TI4_RMP  TMR5 External Input Ch4 Remap
  * @{
  */
#define DDL_TMR_TMR5_TI4_RMP_GPIO        TMR5_OR_RMP_MASK                         /*!< TMR5 channel 4 is connected to GPIO */
#define DDL_TMR_TMR5_TI4_RMP_LSI         (TMR_OR_TI4_RMPSEL_0 | TMR5_OR_RMP_MASK)    /*!< TMR5 channel 4 is connected to LSI internal clock */
#define DDL_TMR_TMR5_TI4_RMP_LSE         (TMR_OR_TI4_RMPSEL_1 | TMR5_OR_RMP_MASK)    /*!< TMR5 channel 4 is connected to LSE */
#define DDL_TMR_TMR5_TI4_RMP_RTC         (TMR_OR_TI4_RMPSEL | TMR5_OR_RMP_MASK)      /*!< TMR5 channel 4 is connected to RTC wakeup interrupt */
/**
  * @}
  */

/** @defgroup TMR_DDL_EC_TMR11_TI1_RMP  TMR11 External Input Capture 1 Remap
  * @{
  */
#define DDL_TMR_TMR11_TI1_RMP_GPIO        TMR11_OR_RMP_MASK                          /*!< TMR11 channel 1 is connected to GPIO */
#if defined(SPDIFRX)
#define DDL_TMR_TMR11_TI1_RMP_SPDIFRX     (TMR_OR_TI1_RMPSEL_0 | TMR11_OR_RMP_MASK)     /*!< TMR11 channel 1 is connected to SPDIFRX */

/* Legacy define */
#define  DDL_TMR_TMR11_TI1_RMP_GPIO1      DDL_TMR_TMR11_TI1_RMP_SPDIFRX               /*!< Legacy define for DDL_TMR_TMR11_TI1_RMP_SPDIFRX */

#else
#define DDL_TMR_TMR11_TI1_RMP_GPIO1       (TMR_OR_TI1_RMPSEL_0 | TMR11_OR_RMP_MASK)     /*!< TMR11 channel 1 is connected to GPIO */
#endif /* SPDIFRX */
#define DDL_TMR_TMR11_TI1_RMP_GPIO2       (TMR_OR_TI1_RMPSEL   | TMR11_OR_RMP_MASK)     /*!< TMR11 channel 1 is connected to GPIO */
#define DDL_TMR_TMR11_TI1_RMP_HSE_RTC     (TMR_OR_TI1_RMPSEL_1 | TMR11_OR_RMP_MASK)     /*!< TMR11 channel 1 is connected to HSE_RTC */
/**
  * @}
  */
#if defined(LPTMR_OR_TMR1_ITR2_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP) && defined(LPTMR_OR_TMR9_ITR1_RMP)

#define DDL_TMR_LPTMR_REMAP_MASK           0x10000000U

#define DDL_TMR_TMR9_ITR1_RMP_TMR3_TRGO    DDL_TMR_LPTMR_REMAP_MASK                              /*!< TMR9_ITR1 is connected to TMR3 TRGO */
#define DDL_TMR_TMR9_ITR1_RMP_LPTMR       (DDL_TMR_LPTMR_REMAP_MASK | LPTMR_OR_TMR9_ITR1_RMP)    /*!< TMR9_ITR1 is connected to LPTMR1 output */

#define DDL_TMR_TMR5_ITR1_RMP_TMR3_TRGO    DDL_TMR_LPTMR_REMAP_MASK                              /*!< TMR5_ITR1 is connected to TMR3 TRGO */
#define DDL_TMR_TMR5_ITR1_RMP_LPTMR       (DDL_TMR_LPTMR_REMAP_MASK | LPTMR_OR_TMR5_ITR1_RMP)    /*!< TMR5_ITR1 is connected to LPTMR1 output */

#define DDL_TMR_TMR1_ITR2_RMP_TMR3_TRGO    DDL_TMR_LPTMR_REMAP_MASK                              /*!< TMR1_ITR2 is connected to TMR3 TRGO */
#define DDL_TMR_TMR1_ITR2_RMP_LPTMR       (DDL_TMR_LPTMR_REMAP_MASK | LPTMR_OR_TMR1_ITR2_RMP)    /*!< TMR1_ITR2 is connected to LPTMR1 output */

#endif /* LPTMR_OR_TMR1_ITR2_RMP &&  LPTMR_OR_TMR5_ITR1_RMP && LPTMR_OR_TMR9_ITR1_RMP */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TMR_DDL_Exported_Macros TMR Exported Macros
  * @{
  */

/** @defgroup TMR_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/**
  * @brief  Write a value in TMR register.
  * @param  __INSTANCE__ TMR Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_TMR_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in TMR register.
  * @param  __INSTANCE__ TMR Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_TMR_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/** @defgroup TMR_DDL_EM_Exported_Macros Exported_Macros
  * @{
  */

/**
  * @brief  HELPER macro calculating DTG[0:7] in the TMRx_BDTR register to achieve the requested dead time duration.
  * @note ex: @ref __DDL_TMR_CALC_DEADTMRE (80000000, @ref DDL_TMR_GetClockDivision (), 120);
  * @param  __TMRCLK__ timer input clock frequency (in Hz)
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV1
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV2
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV4
  * @param  __DT__ deadtime duration (in ns)
  * @retval DTG[0:7]
  */
#define __DDL_TMR_CALC_DEADTMRE(__TMRCLK__, __CKD__, __DT__)  \
  ( (((uint64_t)((__DT__)*1000U)) < ((DT_DELAY_1+1U) * TMR_CALC_DTS((__TMRCLK__), (__CKD__))))    ?  \
    (uint8_t)(((uint64_t)((__DT__)*1000U) / TMR_CALC_DTS((__TMRCLK__), (__CKD__)))  & DT_DELAY_1) :      \
    (((uint64_t)((__DT__)*1000U)) < ((64U + (DT_DELAY_2+1U)) * 2U * TMR_CALC_DTS((__TMRCLK__), (__CKD__))))  ?  \
    (uint8_t)(DT_RANGE_2 | ((uint8_t)((uint8_t)((((uint64_t)((__DT__)*1000U))/ TMR_CALC_DTS((__TMRCLK__),   \
                                                 (__CKD__))) >> 1U) - (uint8_t) 64) & DT_DELAY_2)) :\
    (((uint64_t)((__DT__)*1000U)) < ((32U + (DT_DELAY_3+1U)) * 8U * TMR_CALC_DTS((__TMRCLK__), (__CKD__))))  ?  \
    (uint8_t)(DT_RANGE_3 | ((uint8_t)((uint8_t)(((((uint64_t)(__DT__)*1000U))/ TMR_CALC_DTS((__TMRCLK__),  \
                                                 (__CKD__))) >> 3U) - (uint8_t) 32) & DT_DELAY_3)) :\
    (((uint64_t)((__DT__)*1000U)) < ((32U + (DT_DELAY_4+1U)) * 16U * TMR_CALC_DTS((__TMRCLK__), (__CKD__)))) ?  \
    (uint8_t)(DT_RANGE_4 | ((uint8_t)((uint8_t)(((((uint64_t)(__DT__)*1000U))/ TMR_CALC_DTS((__TMRCLK__),  \
                                                 (__CKD__))) >> 4U) - (uint8_t) 32) & DT_DELAY_4)) :\
    0U)

/**
  * @brief  HELPER macro calculating the prescaler value to achieve the required counter clock frequency.
  * @note ex: @ref __DDL_TMR_CALC_PSC (80000000, 1000000);
  * @param  __TMRCLK__ timer input clock frequency (in Hz)
  * @param  __CNTCLK__ counter clock frequency (in Hz)
  * @retval Prescaler value  (between Min_Data=0 and Max_Data=65535)
  */
#define __DDL_TMR_CALC_PSC(__TMRCLK__, __CNTCLK__)   \
  (((__TMRCLK__) >= (__CNTCLK__)) ? (uint32_t)((((__TMRCLK__) + (__CNTCLK__)/2U)/(__CNTCLK__)) - 1U) : 0U)

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the required output signal frequency.
  * @note ex: @ref __DDL_TMR_CALC_ARR (1000000, @ref DDL_TMR_GetPrescaler (), 10000);
  * @param  __TMRCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __FREQ__ output signal frequency (in Hz)
  * @retval  Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
#define __DDL_TMR_CALC_ARR(__TMRCLK__, __PSC__, __FREQ__) \
  ((((__TMRCLK__)/((__PSC__) + 1U)) >= (__FREQ__)) ? (((__TMRCLK__)/((__FREQ__) * ((__PSC__) + 1U))) - 1U) : 0U)

/**
  * @brief  HELPER macro calculating the compare value required to achieve the required timer output compare
  *         active/inactive delay.
  * @note ex: @ref __DDL_TMR_CALC_DELAY (1000000, @ref DDL_TMR_GetPrescaler (), 10);
  * @param  __TMRCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @retval Compare value  (between Min_Data=0 and Max_Data=65535)
  */
#define __DDL_TMR_CALC_DELAY(__TMRCLK__, __PSC__, __DELAY__)  \
  ((uint32_t)(((uint64_t)(__TMRCLK__) * (uint64_t)(__DELAY__)) \
              / ((uint64_t)1000000U * (uint64_t)((__PSC__) + 1U))))

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the required pulse duration
  *         (when the timer operates in one pulse mode).
  * @note ex: @ref __DDL_TMR_CALC_PULSE (1000000, @ref DDL_TMR_GetPrescaler (), 10, 20);
  * @param  __TMRCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @param  __PULSE__ pulse duration (in us)
  * @retval Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
#define __DDL_TMR_CALC_PULSE(__TMRCLK__, __PSC__, __DELAY__, __PULSE__)  \
  ((uint32_t)(__DDL_TMR_CALC_DELAY((__TMRCLK__), (__PSC__), (__PULSE__)) \
              + __DDL_TMR_CALC_DELAY((__TMRCLK__), (__PSC__), (__DELAY__))))

/**
  * @brief  HELPER macro retrieving the ratio of the input capture prescaler
  * @note ex: @ref __DDL_TMR_GET_ICPSC_RATIO (@ref DDL_TMR_IC_GetPrescaler ());
  * @param  __ICPSC__ This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ICPSC_DIV1
  *         @arg @ref DDL_TMR_ICPSC_DIV2
  *         @arg @ref DDL_TMR_ICPSC_DIV4
  *         @arg @ref DDL_TMR_ICPSC_DIV8
  * @retval Input capture prescaler ratio (1, 2, 4 or 8)
  */
#define __DDL_TMR_GET_ICPSC_RATIO(__ICPSC__)  \
  ((uint32_t)(0x01U << (((__ICPSC__) >> 16U) >> TMR_CCM1_IC1PSC_Pos)))


/**
  * @}
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup TMR_DDL_Exported_Functions TMR Exported Functions
  * @{
  */

/** @defgroup TMR_DDL_EF_Time_Base Time Base configuration
  * @{
  */
/**
  * @brief  Enable timer counter.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableCounter(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CTRL1, TMR_CTRL1_CNTEN);
}

/**
  * @brief  Disable timer counter.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableCounter(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->CTRL1, TMR_CTRL1_CNTEN);
}

/**
  * @brief  Indicates whether the timer counter is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledCounter(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->CTRL1, TMR_CTRL1_CNTEN) == (TMR_CTRL1_CNTEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable update event generation.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableUpdateEvent(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->CTRL1, TMR_CTRL1_UD);
}

/**
  * @brief  Disable update event generation.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableUpdateEvent(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CTRL1, TMR_CTRL1_UD);
}

/**
  * @brief  Indicates whether update event generation is enabled.
  * @param  TMRx Timer instance
  * @retval Inverted state of bit (0 or 1).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledUpdateEvent(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->CTRL1, TMR_CTRL1_UD) == (uint32_t)RESET) ? 1UL : 0UL);
}

/**
  * @brief  Set update event source
  * @note Update event source set to DDL_TMR_UPDATESOURCE_REGULAR: any of the following events
  *       generate an update interrupt or DMA request if enabled:
  *        - Counter overflow/underflow
  *        - Setting the UG bit
  *        - Update generation through the slave mode controller
  * @note Update event source set to DDL_TMR_UPDATESOURCE_COUNTER: only counter
  *       overflow/underflow generates an update interrupt or DMA request if enabled.
  * @param  TMRx Timer instance
  * @param  UpdateSource This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_UPDATESOURCE_REGULAR
  *         @arg @ref DDL_TMR_UPDATESOURCE_COUNTER
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetUpdateSource(TMR_TypeDef *TMRx, uint32_t UpdateSource)
{
  MODIFY_REG(TMRx->CTRL1, TMR_CTRL1_URSSEL, UpdateSource);
}

/**
  * @brief  Get actual event update source
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_UPDATESOURCE_REGULAR
  *         @arg @ref DDL_TMR_UPDATESOURCE_COUNTER
  */
__STATIC_INLINE uint32_t DDL_TMR_GetUpdateSource(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_URSSEL));
}

/**
  * @brief  Set one pulse mode (one shot v.s. repetitive).
  * @param  TMRx Timer instance
  * @param  OnePulseMode This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ONEPULSEMODE_SINGLE
  *         @arg @ref DDL_TMR_ONEPULSEMODE_REPETITIVE
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetOnePulseMode(TMR_TypeDef *TMRx, uint32_t OnePulseMode)
{
  MODIFY_REG(TMRx->CTRL1, TMR_CTRL1_SPMEN, OnePulseMode);
}

/**
  * @brief  Get actual one pulse mode.
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_ONEPULSEMODE_SINGLE
  *         @arg @ref DDL_TMR_ONEPULSEMODE_REPETITIVE
  */
__STATIC_INLINE uint32_t DDL_TMR_GetOnePulseMode(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_SPMEN));
}

/**
  * @brief  Set the timer counter counting mode.
  * @note Macro IS_TMR_COUNTER_MODE_SELECT_INSTANCE(TMRx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @note Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *       requires a timer reset to avoid unexpected direction
  *       due to DIR bit readonly in center aligned mode.
  * @param  TMRx Timer instance
  * @param  CounterMode This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_COUNTERMODE_UP
  *         @arg @ref DDL_TMR_COUNTERMODE_DOWN
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_UP
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_DOWN
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_UP_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetCounterMode(TMR_TypeDef *TMRx, uint32_t CounterMode)
{
  MODIFY_REG(TMRx->CTRL1, (TMR_CTRL1_CNTDIR | TMR_CTRL1_CAMSEL), CounterMode);
}

/**
  * @brief  Get actual counter mode.
  * @note Macro IS_TMR_COUNTER_MODE_SELECT_INSTANCE(TMRx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_COUNTERMODE_UP
  *         @arg @ref DDL_TMR_COUNTERMODE_DOWN
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_UP
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_DOWN
  *         @arg @ref DDL_TMR_COUNTERMODE_CENTER_UP_DOWN
  */
__STATIC_INLINE uint32_t DDL_TMR_GetCounterMode(TMR_TypeDef *TMRx)
{
  uint32_t counter_mode;

  counter_mode = (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_CAMSEL));

  if (counter_mode == 0U)
  {
    counter_mode = (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_CNTDIR));
  }

  return counter_mode;
}

/**
  * @brief  Enable auto-reload (ARR) preload.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableARRPreload(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CTRL1, TMR_CTRL1_ARPEN);
}

/**
  * @brief  Disable auto-reload (ARR) preload.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableARRPreload(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->CTRL1, TMR_CTRL1_ARPEN);
}

/**
  * @brief  Indicates whether auto-reload (ARR) preload is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledARRPreload(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->CTRL1, TMR_CTRL1_ARPEN) == (TMR_CTRL1_ARPEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the division ratio between the timer clock  and the sampling clock used by the dead-time generators
  *         (when supported) and the digital filters.
  * @note Macro IS_TMR_CLOCK_DIVISION_INSTANCE(TMRx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @param  TMRx Timer instance
  * @param  ClockDivision This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV1
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV2
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetClockDivision(TMR_TypeDef *TMRx, uint32_t ClockDivision)
{
  MODIFY_REG(TMRx->CTRL1, TMR_CTRL1_CLKDIV, ClockDivision);
}

/**
  * @brief  Get the actual division ratio between the timer clock  and the sampling clock used by the dead-time
  *         generators (when supported) and the digital filters.
  * @note Macro IS_TMR_CLOCK_DIVISION_INSTANCE(TMRx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV1
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV2
  *         @arg @ref DDL_TMR_CLOCKDIVISION_DIV4
  */
__STATIC_INLINE uint32_t DDL_TMR_GetClockDivision(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_CLKDIV));
}

/**
  * @brief  Set the counter value.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @param  TMRx Timer instance
  * @param  Counter Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetCounter(TMR_TypeDef *TMRx, uint32_t Counter)
{
  WRITE_REG(TMRx->CNT, Counter);
}

/**
  * @brief  Get the counter value.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @param  TMRx Timer instance
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  */
__STATIC_INLINE uint32_t DDL_TMR_GetCounter(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CNT));
}

/**
  * @brief  Get the current direction of the counter
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_COUNTERDIRECTION_UP
  *         @arg @ref DDL_TMR_COUNTERDIRECTION_DOWN
  */
__STATIC_INLINE uint32_t DDL_TMR_GetDirection(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_BIT(TMRx->CTRL1, TMR_CTRL1_CNTDIR));
}

/**
  * @brief  Set the prescaler value.
  * @note The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
  * @note The prescaler can be changed on the fly as this control register is buffered. The new
  *       prescaler ratio is taken into account at the next update event.
  * @note Helper macro @ref __DDL_TMR_CALC_PSC can be used to calculate the Prescaler parameter
  * @param  TMRx Timer instance
  * @param  Prescaler between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetPrescaler(TMR_TypeDef *TMRx, uint32_t Prescaler)
{
  WRITE_REG(TMRx->PSC, Prescaler);
}

/**
  * @brief  Get the prescaler value.
  * @param  TMRx Timer instance
  * @retval  Prescaler value between Min_Data=0 and Max_Data=65535
  */
__STATIC_INLINE uint32_t DDL_TMR_GetPrescaler(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->PSC));
}

/**
  * @brief  Set the auto-reload value.
  * @note The counter is blocked while the auto-reload value is null.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Helper macro @ref __DDL_TMR_CALC_ARR can be used to calculate the AutoReload parameter
  * @param  TMRx Timer instance
  * @param  AutoReload between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetAutoReload(TMR_TypeDef *TMRx, uint32_t AutoReload)
{
  WRITE_REG(TMRx->AUTORLD, AutoReload);
}

/**
  * @brief  Get the auto-reload value.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @param  TMRx Timer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t DDL_TMR_GetAutoReload(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->AUTORLD));
}

/**
  * @brief  Set the repetition counter value.
  * @note Macro IS_TMR_REPETITION_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  * @param  TMRx Timer instance
  * @param  RepetitionCounter between Min_Data=0 and Max_Data=255 or 65535 for advanced timer.
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetRepetitionCounter(TMR_TypeDef *TMRx, uint32_t RepetitionCounter)
{
  WRITE_REG(TMRx->REPCNT, RepetitionCounter);
}

/**
  * @brief  Get the repetition counter value.
  * @note Macro IS_TMR_REPETITION_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  * @param  TMRx Timer instance
  * @retval Repetition counter value
  */
__STATIC_INLINE uint32_t DDL_TMR_GetRepetitionCounter(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->REPCNT));
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Capture_Compare Capture Compare configuration
  * @{
  */
/**
  * @brief  Enable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note CCxE, CCxNE and OCxM bits are preloaded, after having been written,
  *       they are updated only when a commutation event (COM) occurs.
  * @note Only on channels that have a complementary output.
  * @note Macro IS_TMR_COMMUTATION_EVENT_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_EnablePreload(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CTRL2, TMR_CTRL2_CCPEN);
}

/**
  * @brief  Disable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note Macro IS_TMR_COMMUTATION_EVENT_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_DisablePreload(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->CTRL2, TMR_CTRL2_CCPEN);
}

/**
  * @brief  Set the updated source of the capture/compare control bits (CCxE, CCxNE and OCxM).
  * @note Macro IS_TMR_COMMUTATION_EVENT_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @param  TMRx Timer instance
  * @param  CCUpdateSource This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CCUPDATESOURCE_COMG_ONLY
  *         @arg @ref DDL_TMR_CCUPDATESOURCE_COMG_AND_TRGI
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_SetUpdate(TMR_TypeDef *TMRx, uint32_t CCUpdateSource)
{
  MODIFY_REG(TMRx->CTRL2, TMR_CTRL2_CCUSEL, CCUpdateSource);
}

/**
  * @brief  Set the trigger of the capture/compare DMA request.
  * @param  TMRx Timer instance
  * @param  DMAReqTrigger This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CCDMAREQUEST_CC
  *         @arg @ref DDL_TMR_CCDMAREQUEST_UPDATE
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_SetDMAReqTrigger(TMR_TypeDef *TMRx, uint32_t DMAReqTrigger)
{
  MODIFY_REG(TMRx->CTRL2, TMR_CTRL2_CCDSEL, DMAReqTrigger);
}

/**
  * @brief  Get actual trigger of the capture/compare DMA request.
  * @param  TMRx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_CCDMAREQUEST_CC
  *         @arg @ref DDL_TMR_CCDMAREQUEST_UPDATE
  */
__STATIC_INLINE uint32_t DDL_TMR_CC_GetDMAReqTrigger(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_BIT(TMRx->CTRL2, TMR_CTRL2_CCDSEL));
}

/**
  * @brief  Set the lock level to freeze the
  *         configuration of several capture/compare parameters.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       the lock mechanism is supported by a timer instance.
  * @param  TMRx Timer instance
  * @param  LockLevel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_LOCKLEVEL_OFF
  *         @arg @ref DDL_TMR_LOCKLEVEL_1
  *         @arg @ref DDL_TMR_LOCKLEVEL_2
  *         @arg @ref DDL_TMR_LOCKLEVEL_3
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_SetLockLevel(TMR_TypeDef *TMRx, uint32_t LockLevel)
{
  MODIFY_REG(TMRx->BDT, TMR_BDT_LOCKCFG, LockLevel);
}

/**
  * @brief  Enable capture/compare channels.
  * @param  TMRx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_EnableChannel(TMR_TypeDef *TMRx, uint32_t Channels)
{
  SET_BIT(TMRx->CCEN, Channels);
}

/**
  * @brief  Disable capture/compare channels.
  * @param  TMRx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_CC_DisableChannel(TMR_TypeDef *TMRx, uint32_t Channels)
{
  CLEAR_BIT(TMRx->CCEN, Channels);
}

/**
  * @brief  Indicate whether channel(s) is(are) enabled.
  * @param  TMRx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_CC_IsEnabledChannel(TMR_TypeDef *TMRx, uint32_t Channels)
{
  return ((READ_BIT(TMRx->CCEN, Channels) == (Channels)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Output_Channel Output channel configuration
  * @{
  */
/**
  * @brief  Configure an output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref DDL_TMR_OCPOLARITY_HIGH or @ref DDL_TMR_OCPOLARITY_LOW
  *         @arg @ref DDL_TMR_OCIDLESTATE_LOW or @ref DDL_TMR_OCIDLESTATE_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_ConfigOutput(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t Configuration)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TMR_CCM1_CC1SEL << SHIFT_TAB_OCxx[iChannel]));
  MODIFY_REG(TMRx->CCEN, (TMR_CCEN_CC1POL << SHIFT_TAB_CCxP[iChannel]),
             (Configuration & TMR_CCEN_CC1POL) << SHIFT_TAB_CCxP[iChannel]);
  MODIFY_REG(TMRx->CTRL2, (TMR_CTRL2_OC1OIS << SHIFT_TAB_OISx[iChannel]),
             (Configuration & TMR_CTRL2_OC1OIS) << SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Define the behavior of the output reference signal OCxREF from which
  *         OCx and OCxN (when relevant) are derived.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_OCMODE_FROZEN
  *         @arg @ref DDL_TMR_OCMODE_ACTIVE
  *         @arg @ref DDL_TMR_OCMODE_INACTIVE
  *         @arg @ref DDL_TMR_OCMODE_TOGGLE
  *         @arg @ref DDL_TMR_OCMODE_FORCED_INACTIVE
  *         @arg @ref DDL_TMR_OCMODE_FORCED_ACTIVE
  *         @arg @ref DDL_TMR_OCMODE_PWM1
  *         @arg @ref DDL_TMR_OCMODE_PWM2
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetMode(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t Mode)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TMR_CCM1_OC1MOD  | TMR_CCM1_CC1SEL) << SHIFT_TAB_OCxx[iChannel]), Mode << SHIFT_TAB_OCxx[iChannel]);
}

/**
  * @brief  Get the output compare mode of an output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_OCMODE_FROZEN
  *         @arg @ref DDL_TMR_OCMODE_ACTIVE
  *         @arg @ref DDL_TMR_OCMODE_INACTIVE
  *         @arg @ref DDL_TMR_OCMODE_TOGGLE
  *         @arg @ref DDL_TMR_OCMODE_FORCED_INACTIVE
  *         @arg @ref DDL_TMR_OCMODE_FORCED_ACTIVE
  *         @arg @ref DDL_TMR_OCMODE_PWM1
  *         @arg @ref DDL_TMR_OCMODE_PWM2
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetMode(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  return (READ_BIT(*pReg, ((TMR_CCM1_OC1MOD | TMR_CCM1_CC1SEL) << SHIFT_TAB_OCxx[iChannel])) >> SHIFT_TAB_OCxx[iChannel]);
}

/**
  * @brief  Set the polarity of an output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_OCPOLARITY_HIGH
  *         @arg @ref DDL_TMR_OCPOLARITY_LOW
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetPolarity(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t Polarity)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TMRx->CCEN, (TMR_CCEN_CC1POL << SHIFT_TAB_CCxP[iChannel]),  Polarity << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Get the polarity of an output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_OCPOLARITY_HIGH
  *         @arg @ref DDL_TMR_OCPOLARITY_LOW
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetPolarity(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TMRx->CCEN, (TMR_CCEN_CC1POL << SHIFT_TAB_CCxP[iChannel])) >> SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Set the IDLE state of an output channel
  * @note This function is significant only for the timer instances
  *       supporting the break feature. Macro IS_TMR_BREAK_INSTANCE(TMRx)
  *       can be used to check whether or not a timer instance provides
  *       a break input.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  IdleState This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_OCIDLESTATE_LOW
  *         @arg @ref DDL_TMR_OCIDLESTATE_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetIdleState(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t IdleState)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TMRx->CTRL2, (TMR_CTRL2_OC1OIS << SHIFT_TAB_OISx[iChannel]),  IdleState << SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Get the IDLE state of an output channel
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH1N
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH2N
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH3N
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_OCIDLESTATE_LOW
  *         @arg @ref DDL_TMR_OCIDLESTATE_HIGH
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetIdleState(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TMRx->CTRL2, (TMR_CTRL2_OC1OIS << SHIFT_TAB_OISx[iChannel])) >> SHIFT_TAB_OISx[iChannel]);
}

/**
  * @brief  Enable fast mode for the output channel.
  * @note Acts only if the channel is configured in PWM1 or PWM2 mode.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_EnableFast(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TMR_CCM1_OC1FEN << SHIFT_TAB_OCxx[iChannel]));

}

/**
  * @brief  Disable fast mode for the output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_DisableFast(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TMR_CCM1_OC1FEN << SHIFT_TAB_OCxx[iChannel]));

}

/**
  * @brief  Indicates whether fast mode is enabled for the output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_IsEnabledFast(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  uint32_t bitfield = TMR_CCM1_OC1FEN << SHIFT_TAB_OCxx[iChannel];
  return ((READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Enable compare register (TMRx_CCx) preload for the output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_EnablePreload(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TMR_CCM1_OC1PEN << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Disable compare register (TMRx_CCx) preload for the output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_DisablePreload(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TMR_CCM1_OC1PEN << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Indicates whether compare register (TMRx_CCx) preload is enabled for the output channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_IsEnabledPreload(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  uint32_t bitfield = TMR_CCM1_OC1PEN << SHIFT_TAB_OCxx[iChannel];
  return ((READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Enable clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro IS_TMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_EnableClear(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TMR_CCM1_OC1CEN << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Disable clearing the output channel on an external event.
  * @note Macro IS_TMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_DisableClear(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TMR_CCM1_OC1CEN << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  Indicates clearing the output channel on an external event is enabled for the output channel.
  * @note This function enables clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro IS_TMR_OCXREF_CLEAR_INSTANCE(TMRx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_IsEnabledClear(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  uint32_t bitfield = TMR_CCM1_OC1CEN << SHIFT_TAB_OCxx[iChannel];
  return ((READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Set the dead-time delay (delay inserted between the rising edge of the OCxREF signal and the rising edge of
  *         the Ocx and OCxN signals).
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       dead-time insertion feature is supported by a timer instance.
  * @note Helper macro @ref __DDL_TMR_CALC_DEADTMRE can be used to calculate the DeadTime parameter
  * @param  TMRx Timer instance
  * @param  DeadTime between Min_Data=0 and Max_Data=255
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetDeadTime(TMR_TypeDef *TMRx, uint32_t DeadTime)
{
  MODIFY_REG(TMRx->BDT, TMR_BDT_DTS, DeadTime);
}

/**
  * @brief  Set compare value for output channel 1 (TMRx_CC1).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC1_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetCompareCH1(TMR_TypeDef *TMRx, uint32_t CompareValue)
{
  WRITE_REG(TMRx->CC1, CompareValue);
}

/**
  * @brief  Set compare value for output channel 2 (TMRx_CC2).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC2_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetCompareCH2(TMR_TypeDef *TMRx, uint32_t CompareValue)
{
  WRITE_REG(TMRx->CC2, CompareValue);
}

/**
  * @brief  Set compare value for output channel 3 (TMRx_CC3).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC3_INSTANCE(TMRx) can be used to check whether or not
  *       output channel is supported by a timer instance.
  * @param  TMRx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetCompareCH3(TMR_TypeDef *TMRx, uint32_t CompareValue)
{
  WRITE_REG(TMRx->CC3, CompareValue);
}

/**
  * @brief  Set compare value for output channel 4 (TMRx_CC4).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC4_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_OC_SetCompareCH4(TMR_TypeDef *TMRx, uint32_t CompareValue)
{
  WRITE_REG(TMRx->CC4, CompareValue);
}

/**
  * @brief  Get compare value (TMRx_CC1) set for  output channel 1.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC1_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetCompareCH1(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC1));
}

/**
  * @brief  Get compare value (TMRx_CC2) set for  output channel 2.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC2_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetCompareCH2(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC2));
}

/**
  * @brief  Get compare value (TMRx_CC3) set for  output channel 3.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC3_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 3 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetCompareCH3(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC3));
}

/**
  * @brief  Get compare value (TMRx_CC4) set for  output channel 4.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC4_INSTANCE(TMRx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_OC_GetCompareCH4(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC4));
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Input_Channel Input channel configuration
  * @{
  */
/**
  * @brief  Configure input channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref DDL_TMR_ACTIVEINPUT_DIRECTTI or @ref DDL_TMR_ACTIVEINPUT_INDIRECTTI or @ref DDL_TMR_ACTIVEINPUT_TRC
  *         @arg @ref DDL_TMR_ICPSC_DIV1 or ... or @ref DDL_TMR_ICPSC_DIV8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1 or ... or @ref DDL_TMR_IC_FILTER_FDIV32_N8
  *         @arg @ref DDL_TMR_IC_POLARITY_RISING or @ref DDL_TMR_IC_POLARITY_FALLING or @ref DDL_TMR_IC_POLARITY_BOTHEDGE
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_Config(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t Configuration)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TMR_CCM1_IC1F | TMR_CCM1_IC1PSC | TMR_CCM1_CC1SEL) << SHIFT_TAB_ICxx[iChannel]),
             ((Configuration >> 16U) & (TMR_CCM1_IC1F | TMR_CCM1_IC1PSC | TMR_CCM1_CC1SEL))                \
             << SHIFT_TAB_ICxx[iChannel]);
  MODIFY_REG(TMRx->CCEN, ((TMR_CCEN_CC1NPOL | TMR_CCEN_CC1POL) << SHIFT_TAB_CCxP[iChannel]),
             (Configuration & (TMR_CCEN_CC1NPOL | TMR_CCEN_CC1POL)) << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Set the active input.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  ICActiveInput This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ACTIVEINPUT_DIRECTTI
  *         @arg @ref DDL_TMR_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref DDL_TMR_ACTIVEINPUT_TRC
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_SetActiveInput(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICActiveInput)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TMR_CCM1_CC1SEL) << SHIFT_TAB_ICxx[iChannel]), (ICActiveInput >> 16U) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the current active input.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_ACTIVEINPUT_DIRECTTI
  *         @arg @ref DDL_TMR_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref DDL_TMR_ACTIVEINPUT_TRC
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetActiveInput(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  return ((READ_BIT(*pReg, ((TMR_CCM1_CC1SEL) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
  * @brief  Set the prescaler of input channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  ICPrescaler This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ICPSC_DIV1
  *         @arg @ref DDL_TMR_ICPSC_DIV2
  *         @arg @ref DDL_TMR_ICPSC_DIV4
  *         @arg @ref DDL_TMR_ICPSC_DIV8
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_SetPrescaler(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICPrescaler)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TMR_CCM1_IC1PSC) << SHIFT_TAB_ICxx[iChannel]), (ICPrescaler >> 16U) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the current prescaler value acting on an  input channel.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_ICPSC_DIV1
  *         @arg @ref DDL_TMR_ICPSC_DIV2
  *         @arg @ref DDL_TMR_ICPSC_DIV4
  *         @arg @ref DDL_TMR_ICPSC_DIV8
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetPrescaler(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  return ((READ_BIT(*pReg, ((TMR_CCM1_IC1PSC) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
  * @brief  Set the input filter duration.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  ICFilter This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N2
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N4
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV2_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV2_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV4_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV4_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV8_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV8_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N5
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N5
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_SetFilter(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICFilter)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  MODIFY_REG(*pReg, ((TMR_CCM1_IC1F) << SHIFT_TAB_ICxx[iChannel]), (ICFilter >> 16U) << SHIFT_TAB_ICxx[iChannel]);
}

/**
  * @brief  Get the input filter duration.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N2
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N4
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV1_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV2_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV2_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV4_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV4_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV8_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV8_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N5
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV16_N8
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N5
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N6
  *         @arg @ref DDL_TMR_IC_FILTER_FDIV32_N8
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetFilter(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TMRx->CCM1) + OFFSET_TAB_CCMRx[iChannel]));
  return ((READ_BIT(*pReg, ((TMR_CCM1_IC1F) << SHIFT_TAB_ICxx[iChannel])) >> SHIFT_TAB_ICxx[iChannel]) << 16U);
}

/**
  * @brief  Set the input channel polarity.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  ICPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_IC_POLARITY_RISING
  *         @arg @ref DDL_TMR_IC_POLARITY_FALLING
  *         @arg @ref DDL_TMR_IC_POLARITY_BOTHEDGE
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_SetPolarity(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ICPolarity)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  MODIFY_REG(TMRx->CCEN, ((TMR_CCEN_CC1NPOL | TMR_CCEN_CC1POL) << SHIFT_TAB_CCxP[iChannel]),
             ICPolarity << SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Get the current input channel polarity.
  * @param  TMRx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_TMR_IC_POLARITY_RISING
  *         @arg @ref DDL_TMR_IC_POLARITY_FALLING
  *         @arg @ref DDL_TMR_IC_POLARITY_BOTHEDGE
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetPolarity(TMR_TypeDef *TMRx, uint32_t Channel)
{
  uint8_t iChannel = TMR_GET_CHANNEL_INDEX(Channel);
  return (READ_BIT(TMRx->CCEN, ((TMR_CCEN_CC1NPOL | TMR_CCEN_CC1POL) << SHIFT_TAB_CCxP[iChannel])) >>
          SHIFT_TAB_CCxP[iChannel]);
}

/**
  * @brief  Connect the TMRx_CH1, CH2 and CH3 pins  to the TI1 input (XOR combination).
  * @note Macro IS_TMR_XOR_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_EnableXORCombination(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CTRL2, TMR_CTRL2_TI1SEL);
}

/**
  * @brief  Disconnect the TMRx_CH1, CH2 and CH3 pins  from the TI1 input.
  * @note Macro IS_TMR_XOR_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_IC_DisableXORCombination(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->CTRL2, TMR_CTRL2_TI1SEL);
}

/**
  * @brief  Indicates whether the TMRx_CH1, CH2 and CH3 pins are connectected to the TI1 input.
  * @note Macro IS_TMR_XOR_INSTANCE(TMRx) can be used to check whether or not
  * a timer instance provides an XOR input.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_IsEnabledXORCombination(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->CTRL2, TMR_CTRL2_TI1SEL) == (TMR_CTRL2_TI1SEL)) ? 1UL : 0UL);
}

/**
  * @brief  Get captured value for input channel 1.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC1_INSTANCE(TMRx) can be used to check whether or not
  *       input channel 1 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetCaptureCH1(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC1));
}

/**
  * @brief  Get captured value for input channel 2.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC2_INSTANCE(TMRx) can be used to check whether or not
  *       input channel 2 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetCaptureCH2(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC2));
}

/**
  * @brief  Get captured value for input channel 3.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC3_INSTANCE(TMRx) can be used to check whether or not
  *       input channel 3 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetCaptureCH3(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC3));
}

/**
  * @brief  Get captured value for input channel 4.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TMR_32B_COUNTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro IS_TMR_CC4_INSTANCE(TMRx) can be used to check whether or not
  *       input channel 4 is supported by a timer instance.
  * @param  TMRx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t DDL_TMR_IC_GetCaptureCH4(TMR_TypeDef *TMRx)
{
  return (uint32_t)(READ_REG(TMRx->CC4));
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Clock_Selection Counter clock selection
  * @{
  */
/**
  * @brief  Enable external clock mode 2.
  * @note When external clock mode 2 is enabled the counter is clocked by any active edge on the ETRF signal.
  * @note Macro IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableExternalClock(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->SMCTRL, TMR_SMCTRL_ECEN);
}

/**
  * @brief  Disable external clock mode 2.
  * @note Macro IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableExternalClock(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->SMCTRL, TMR_SMCTRL_ECEN);
}

/**
  * @brief  Indicate whether external clock mode 2 is enabled.
  * @note Macro IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledExternalClock(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->SMCTRL, TMR_SMCTRL_ECEN) == (TMR_SMCTRL_ECEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the clock source of the counter clock.
  * @note when selected clock source is external clock mode 1, the timer input
  *       the external clock is applied is selected by calling the @ref DDL_TMR_SetTriggerInput()
  *       function. This timer input must be configured by calling
  *       the @ref DDL_TMR_IC_Config() function.
  * @note Macro IS_TMR_CLOCKSOURCE_ETRMODE1_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports external clock mode1.
  * @note Macro IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @param  TMRx Timer instance
  * @param  ClockSource This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CLOCKSOURCE_INTERNAL
  *         @arg @ref DDL_TMR_CLOCKSOURCE_EXT_MODE1
  *         @arg @ref DDL_TMR_CLOCKSOURCE_EXT_MODE2
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetClockSource(TMR_TypeDef *TMRx, uint32_t ClockSource)
{
  MODIFY_REG(TMRx->SMCTRL, TMR_SMCTRL_SMFSEL | TMR_SMCTRL_ECEN, ClockSource);
}

/**
  * @brief  Set the encoder interface mode.
  * @note Macro IS_TMR_ENCODER_INTERFACE_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance supports the encoder mode.
  * @param  TMRx Timer instance
  * @param  EncoderMode This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ENCODERMODE_X2_TI1
  *         @arg @ref DDL_TMR_ENCODERMODE_X2_TI2
  *         @arg @ref DDL_TMR_ENCODERMODE_X4_TI12
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetEncoderMode(TMR_TypeDef *TMRx, uint32_t EncoderMode)
{
  MODIFY_REG(TMRx->SMCTRL, TMR_SMCTRL_SMFSEL, EncoderMode);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Timer_Synchronization Timer synchronisation configuration
  * @{
  */
/**
  * @brief  Set the trigger output (TRGO) used for timer synchronization .
  * @note Macro IS_TMR_MASTER_INSTANCE(TMRx) can be used to check
  *       whether or not a timer instance can operate as a master timer.
  * @param  TMRx Timer instance
  * @param  TimerSynchronization This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_TRGO_RESET
  *         @arg @ref DDL_TMR_TRGO_ENABLE
  *         @arg @ref DDL_TMR_TRGO_UPDATE
  *         @arg @ref DDL_TMR_TRGO_CC1IF
  *         @arg @ref DDL_TMR_TRGO_OC1REF
  *         @arg @ref DDL_TMR_TRGO_OC2REF
  *         @arg @ref DDL_TMR_TRGO_OC3REF
  *         @arg @ref DDL_TMR_TRGO_OC4REF
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetTriggerOutput(TMR_TypeDef *TMRx, uint32_t TimerSynchronization)
{
  MODIFY_REG(TMRx->CTRL2, TMR_CTRL2_MMSEL, TimerSynchronization);
}

/**
  * @brief  Set the synchronization mode of a slave timer.
  * @note Macro IS_TMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @param  TMRx Timer instance
  * @param  SlaveMode This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_SLAVEMODE_DISABLED
  *         @arg @ref DDL_TMR_SLAVEMODE_RESET
  *         @arg @ref DDL_TMR_SLAVEMODE_GATED
  *         @arg @ref DDL_TMR_SLAVEMODE_TRIGGER
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetSlaveMode(TMR_TypeDef *TMRx, uint32_t SlaveMode)
{
  MODIFY_REG(TMRx->SMCTRL, TMR_SMCTRL_SMFSEL, SlaveMode);
}

/**
  * @brief  Set the selects the trigger input to be used to synchronize the counter.
  * @note Macro IS_TMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @param  TMRx Timer instance
  * @param  TriggerInput This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_TS_ITR0
  *         @arg @ref DDL_TMR_TS_ITR1
  *         @arg @ref DDL_TMR_TS_ITR2
  *         @arg @ref DDL_TMR_TS_ITR3
  *         @arg @ref DDL_TMR_TS_TI1F_ED
  *         @arg @ref DDL_TMR_TS_TI1FP1
  *         @arg @ref DDL_TMR_TS_TI2FP2
  *         @arg @ref DDL_TMR_TS_ETRF
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetTriggerInput(TMR_TypeDef *TMRx, uint32_t TriggerInput)
{
  MODIFY_REG(TMRx->SMCTRL, TMR_SMCTRL_TRGSEL, TriggerInput);
}

/**
  * @brief  Enable the Master/Slave mode.
  * @note Macro IS_TMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableMasterSlaveMode(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->SMCTRL, TMR_SMCTRL_MSMEN);
}

/**
  * @brief  Disable the Master/Slave mode.
  * @note Macro IS_TMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableMasterSlaveMode(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->SMCTRL, TMR_SMCTRL_MSMEN);
}

/**
  * @brief Indicates whether the Master/Slave mode is enabled.
  * @note Macro IS_TMR_SLAVE_INSTANCE(TMRx) can be used to check whether or not
  * a timer instance can operate as a slave timer.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledMasterSlaveMode(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->SMCTRL, TMR_SMCTRL_MSMEN) == (TMR_SMCTRL_MSMEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the external trigger (ETR) input.
  * @note Macro IS_TMR_ETR_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides an external trigger input.
  * @param  TMRx Timer instance
  * @param  ETRPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ETR_POLARITY_NONINVERTED
  *         @arg @ref DDL_TMR_ETR_POLARITY_INVERTED
  * @param  ETRPrescaler This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ETR_PRESCALER_DIV1
  *         @arg @ref DDL_TMR_ETR_PRESCALER_DIV2
  *         @arg @ref DDL_TMR_ETR_PRESCALER_DIV4
  *         @arg @ref DDL_TMR_ETR_PRESCALER_DIV8
  * @param  ETRFilter This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV1
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV1_N2
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV1_N4
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV1_N8
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV2_N6
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV2_N8
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV4_N6
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV4_N8
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV8_N6
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV8_N8
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV16_N5
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV16_N6
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV16_N8
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV32_N5
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV32_N6
  *         @arg @ref DDL_TMR_ETR_FILTER_FDIV32_N8
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ConfigETR(TMR_TypeDef *TMRx, uint32_t ETRPolarity, uint32_t ETRPrescaler,
                                      uint32_t ETRFilter)
{
  MODIFY_REG(TMRx->SMCTRL, TMR_SMCTRL_ETPOL | TMR_SMCTRL_ETPCFG | TMR_SMCTRL_ETFCFG, ETRPolarity | ETRPrescaler | ETRFilter);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Break_Function Break function configuration
  * @{
  */
/**
  * @brief  Enable the break function.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableBRK(TMR_TypeDef *TMRx)
{
  __IO uint32_t tmpreg;
  SET_BIT(TMRx->BDT, TMR_BDT_BRKEN);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = READ_REG(TMRx->BDT);
  (void)(tmpreg);
}

/**
  * @brief  Disable the break function.
  * @param  TMRx Timer instance
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableBRK(TMR_TypeDef *TMRx)
{
  __IO uint32_t tmpreg;
  CLEAR_BIT(TMRx->BDT, TMR_BDT_BRKEN);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = READ_REG(TMRx->BDT);
  (void)(tmpreg);
}

/**
  * @brief  Configure the break input.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @param  BreakPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_BREAK_POLARITY_LOW
  *         @arg @ref DDL_TMR_BREAK_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ConfigBRK(TMR_TypeDef *TMRx, uint32_t BreakPolarity)
{
  __IO uint32_t tmpreg;
  MODIFY_REG(TMRx->BDT, TMR_BDT_BRKPOL, BreakPolarity);
  /* Note: Any write operation to BKP bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = READ_REG(TMRx->BDT);
  (void)(tmpreg);
}

/**
  * @brief  Select the outputs off state (enabled v.s. disabled) in Idle and Run modes.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @param  OffStateIdle This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_OSSI_DISABLE
  *         @arg @ref DDL_TMR_OSSI_ENABLE
  * @param  OffStateRun This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_OSSR_DISABLE
  *         @arg @ref DDL_TMR_OSSR_ENABLE
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetOffStates(TMR_TypeDef *TMRx, uint32_t OffStateIdle, uint32_t OffStateRun)
{
  MODIFY_REG(TMRx->BDT, TMR_BDT_IMOS | TMR_BDT_RMOS, OffStateIdle | OffStateRun);
}

/**
  * @brief  Enable automatic output (MOE can be set by software or automatically when a break input is active).
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableAutomaticOutput(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->BDT, TMR_BDT_AOEN);
}

/**
  * @brief  Disable automatic output (MOE can be set only by software).
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableAutomaticOutput(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->BDT, TMR_BDT_AOEN);
}

/**
  * @brief  Indicate whether automatic output is enabled.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledAutomaticOutput(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->BDT, TMR_BDT_AOEN) == (TMR_BDT_AOEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the outputs (set the MOE bit in TMRx_BDTR register).
  * @note The MOE bit in TMRx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableAllOutputs(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->BDT, TMR_BDT_MOEN);
}

/**
  * @brief  Disable the outputs (reset the MOE bit in TMRx_BDTR register).
  * @note The MOE bit in TMRx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableAllOutputs(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->BDT, TMR_BDT_MOEN);
}

/**
  * @brief  Indicates whether outputs are enabled.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledAllOutputs(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->BDT, TMR_BDT_MOEN) == (TMR_BDT_MOEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_DMA_Burst_Mode DMA burst mode configuration
  * @{
  */
/**
  * @brief  Configures the timer DMA burst feature.
  * @note Macro IS_TMR_DMABURST_INSTANCE(TMRx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  * @param  TMRx Timer instance
  * @param  DMABurstBaseAddress This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CTRL1
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CTRL2
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_SMCTRL
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_DIEN
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_SR
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_EGR
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CCM1
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CCM2
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CCEN
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CNT
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_PSC
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_AUTORLD
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_REPCNT
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CC1
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CC2
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CC3
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_CC4
  *         @arg @ref DDL_TMR_DMABURST_BASEADDR_BDT
  * @param  DMABurstLength This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref DDL_TMR_DMABURST_LENGTH_18TRANSFERS
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ConfigDMABurst(TMR_TypeDef *TMRx, uint32_t DMABurstBaseAddress, uint32_t DMABurstLength)
{
  MODIFY_REG(TMRx->DCTRL, (TMR_DCTRL_DBLEN | TMR_DCTRL_DBADDR), (DMABurstBaseAddress | DMABurstLength));
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_Timer_Inputs_Remapping Timer input remapping
  * @{
  */
/**
  * @brief  Remap TMR inputs (input channel, internal/external triggers).
  * @note Macro IS_TMR_REMAP_INSTANCE(TMRx) can be used to check whether or not
  *       a some timer inputs can be remapped.
  * @param  TMRx Timer instance
  * @param  Remap Remap param depends on the TMRx. Description available only
  *         in CHM version of the User Manual (not in .pdf).
  *         Otherwise see Reference Manual description of OR registers.
  *
  *         Below description summarizes "Timer Instance" and "Remap" param combinations:
  *
  *         TMR1: one of the following values
  *
  *            ITR2_RMP can be one of the following values
  *            @arg @ref DDL_TMR_TMR1_ITR2_RMP_TMR3_TRGO (*)
  *            @arg @ref DDL_TMR_TMR1_ITR2_RMP_LPTMR (*)
  *
  *         TMR2: one of the following values
  *
  *            ITR1_RMP can be one of the following values
  *            @arg @ref DDL_TMR_TMR2_ITR1_RMP_TMR8_TRGO
  *            @arg @ref DDL_TMR_TMR2_ITR1_RMP_OTG_FS_SOF
  *            @arg @ref DDL_TMR_TMR2_ITR1_RMP_OTG_HS_SOF
  *
  *         TMR5: one of the following values
  *
  *            @arg @ref DDL_TMR_TMR5_TI4_RMP_GPIO
  *            @arg @ref DDL_TMR_TMR5_TI4_RMP_LSI
  *            @arg @ref DDL_TMR_TMR5_TI4_RMP_LSE
  *            @arg @ref DDL_TMR_TMR5_TI4_RMP_RTC
  *            @arg @ref DDL_TMR_TMR5_ITR1_RMP_TMR3_TRGO (*)
  *            @arg @ref DDL_TMR_TMR5_ITR1_RMP_LPTMR (*)
  *
  *         TMR9: one of the following values
  *
  *            ITR1_RMP can be one of the following values
  *            @arg @ref DDL_TMR_TMR9_ITR1_RMP_TMR3_TRGO (*)
  *            @arg @ref DDL_TMR_TMR9_ITR1_RMP_LPTMR (*)
  *
  *         TMR11: one of the following values
  *
  *            @arg @ref DDL_TMR_TMR11_TI1_RMP_GPIO
  *            @arg @ref DDL_TMR_TMR11_TI1_RMP_GPIO1 (*)
  *            @arg @ref DDL_TMR_TMR11_TI1_RMP_HSE_RTC
  *            @arg @ref DDL_TMR_TMR11_TI1_RMP_GPIO2
  *            @arg @ref DDL_TMR_TMR11_TI1_RMP_SPDIFRX (*)
  *
  *         (*)  Value not defined in all devices. \n
  *
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_SetRemap(TMR_TypeDef *TMRx, uint32_t Remap)
{
#if defined(LPTMR_OR_TMR1_ITR2_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP) && defined(LPTMR_OR_TMR9_ITR1_RMP)
  if ((Remap & DDL_TMR_LPTMR_REMAP_MASK) == DDL_TMR_LPTMR_REMAP_MASK)
  {
    /* Connect TMRx internal trigger to LPTMR1 output */
    SET_BIT(RCC->APB1ENR, RCM_APB1ENR_LPTMR1EN);
    MODIFY_REG(LPTMR1->OR,
               (LPTMR_OR_TMR1_ITR2_RMP | LPTMR_OR_TMR5_ITR1_RMP | LPTMR_OR_TMR9_ITR1_RMP),
               Remap & ~(DDL_TMR_LPTMR_REMAP_MASK));
  }
  else
  {
    MODIFY_REG(TMRx->OR, (Remap >> TMRx_OR_RMP_SHIFT), (Remap & TMRx_OR_RMP_MASK));
  }
#else
  MODIFY_REG(TMRx->OR, (Remap >> TMRx_OR_RMP_SHIFT), (Remap & TMRx_OR_RMP_MASK));
#endif /* LPTMR_OR_TMR1_ITR2_RMP &&  LPTMR_OR_TMR5_ITR1_RMP && LPTMR_OR_TMR9_ITR1_RMP */
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_FLAG_Management FLAG-Management
  * @{
  */
/**
  * @brief  Clear the update interrupt flag (UIF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_UPDATE(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_UIFLG));
}

/**
  * @brief  Indicate whether update interrupt flag (UIF) is set (update interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_UPDATE(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_UIFLG) == (TMR_STS_UIFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 1 interrupt flag (CC1F).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC1(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC1IFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 1 interrupt flag (CC1F) is set (Capture/Compare 1 interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC1(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC1IFLG) == (TMR_STS_CC1IFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 2 interrupt flag (CC2F).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC2(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC2IFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 2 interrupt flag (CC2F) is set (Capture/Compare 2 interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC2(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC2IFLG) == (TMR_STS_CC2IFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 3 interrupt flag (CC3F).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC3(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC3IFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 3 interrupt flag (CC3F) is set (Capture/Compare 3 interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC3(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC3IFLG) == (TMR_STS_CC3IFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 4 interrupt flag (CC4F).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC4(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC4IFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 4 interrupt flag (CC4F) is set (Capture/Compare 4 interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC4(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC4IFLG) == (TMR_STS_CC4IFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the commutation interrupt flag (COMIF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_COM(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_COMIFLG));
}

/**
  * @brief  Indicate whether commutation interrupt flag (COMIF) is set (commutation interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_COM(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_COMIFLG) == (TMR_STS_COMIFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the trigger interrupt flag (TIF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_TRIG(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_TRGIFLG));
}

/**
  * @brief  Indicate whether trigger interrupt flag (TIF) is set (trigger interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_TRIG(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_TRGIFLG) == (TMR_STS_TRGIFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the break interrupt flag (BIF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_BRK(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_BRKIFLG));
}

/**
  * @brief  Indicate whether break interrupt flag (BIF) is set (break interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_BRK(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_BRKIFLG) == (TMR_STS_BRKIFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 1 over-capture interrupt flag (CC1OF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC1OVR(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC1RCFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 1 over-capture interrupt flag (CC1OF) is set
  *         (Capture/Compare 1 interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC1OVR(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC1RCFLG) == (TMR_STS_CC1RCFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 2 over-capture interrupt flag (CC2OF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC2OVR(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC2RCFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 2 over-capture interrupt flag (CC2OF) is set
  *         (Capture/Compare 2 over-capture interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC2OVR(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC2RCFLG) == (TMR_STS_CC2RCFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 3 over-capture interrupt flag (CC3OF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC3OVR(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC3RCFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 3 over-capture interrupt flag (CC3OF) is set
  *         (Capture/Compare 3 over-capture interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC3OVR(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC3RCFLG) == (TMR_STS_CC3RCFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 4 over-capture interrupt flag (CC4OF).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_ClearFlag_CC4OVR(TMR_TypeDef *TMRx)
{
  WRITE_REG(TMRx->STS, ~(TMR_STS_CC4RCFLG));
}

/**
  * @brief  Indicate whether Capture/Compare 4 over-capture interrupt flag (CC4OF) is set
  *         (Capture/Compare 4 over-capture interrupt is pending).
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsActiveFlag_CC4OVR(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->STS, TMR_STS_CC4RCFLG) == (TMR_STS_CC4RCFLG)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_IT_Management IT-Management
  * @{
  */
/**
  * @brief  Enable update interrupt (UIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_UPDATE(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_UIEN);
}

/**
  * @brief  Disable update interrupt (UIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_UPDATE(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_UIEN);
}

/**
  * @brief  Indicates whether the update interrupt (UIE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_UPDATE(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_UIEN) == (TMR_DIEN_UIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 1 interrupt (CC1IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_CC1(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC1IEN);
}

/**
  * @brief  Disable capture/compare 1  interrupt (CC1IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_CC1(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC1IEN);
}

/**
  * @brief  Indicates whether the capture/compare 1 interrupt (CC1IE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_CC1(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC1IEN) == (TMR_DIEN_CC1IEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 2 interrupt (CC2IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_CC2(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC2IEN);
}

/**
  * @brief  Disable capture/compare 2  interrupt (CC2IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_CC2(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC2IEN);
}

/**
  * @brief  Indicates whether the capture/compare 2 interrupt (CC2IE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_CC2(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC2IEN) == (TMR_DIEN_CC2IEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 3 interrupt (CC3IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_CC3(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC3IEN);
}

/**
  * @brief  Disable capture/compare 3  interrupt (CC3IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_CC3(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC3IEN);
}

/**
  * @brief  Indicates whether the capture/compare 3 interrupt (CC3IE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_CC3(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC3IEN) == (TMR_DIEN_CC3IEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 4 interrupt (CC4IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_CC4(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC4IEN);
}

/**
  * @brief  Disable capture/compare 4  interrupt (CC4IE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_CC4(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC4IEN);
}

/**
  * @brief  Indicates whether the capture/compare 4 interrupt (CC4IE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_CC4(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC4IEN) == (TMR_DIEN_CC4IEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable commutation interrupt (COMIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_COM(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_COMIEN);
}

/**
  * @brief  Disable commutation interrupt (COMIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_COM(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_COMIEN);
}

/**
  * @brief  Indicates whether the commutation interrupt (COMIE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_COM(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_COMIEN) == (TMR_DIEN_COMIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable trigger interrupt (TIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_TRIG(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_TRGIEN);
}

/**
  * @brief  Disable trigger interrupt (TIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_TRIG(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_TRGIEN);
}

/**
  * @brief  Indicates whether the trigger interrupt (TIE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_TRIG(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_TRGIEN) == (TMR_DIEN_TRGIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable break interrupt (BIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableIT_BRK(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_BRKIEN);
}

/**
  * @brief  Disable break interrupt (BIE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableIT_BRK(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_BRKIEN);
}

/**
  * @brief  Indicates whether the break interrupt (BIE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledIT_BRK(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_BRKIEN) == (TMR_DIEN_BRKIEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_DMA_Management DMA Management
  * @{
  */
/**
  * @brief  Enable update DMA request (UDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_UPDATE(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_UDIEN);
}

/**
  * @brief  Disable update DMA request (UDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_UPDATE(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_UDIEN);
}

/**
  * @brief  Indicates whether the update DMA request  (UDE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_UPDATE(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_UDIEN) == (TMR_DIEN_UDIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 1 DMA request (CC1DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_CC1(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC1DEN);
}

/**
  * @brief  Disable capture/compare 1  DMA request (CC1DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_CC1(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC1DEN);
}

/**
  * @brief  Indicates whether the capture/compare 1 DMA request (CC1DE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_CC1(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC1DEN) == (TMR_DIEN_CC1DEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 2 DMA request (CC2DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_CC2(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC2DEN);
}

/**
  * @brief  Disable capture/compare 2  DMA request (CC2DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_CC2(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC2DEN);
}

/**
  * @brief  Indicates whether the capture/compare 2 DMA request (CC2DE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_CC2(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC2DEN) == (TMR_DIEN_CC2DEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 3 DMA request (CC3DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_CC3(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC3DEN);
}

/**
  * @brief  Disable capture/compare 3  DMA request (CC3DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_CC3(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC3DEN);
}

/**
  * @brief  Indicates whether the capture/compare 3 DMA request (CC3DE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_CC3(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC3DEN) == (TMR_DIEN_CC3DEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 4 DMA request (CC4DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_CC4(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_CC4DEN);
}

/**
  * @brief  Disable capture/compare 4  DMA request (CC4DE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_CC4(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_CC4DEN);
}

/**
  * @brief  Indicates whether the capture/compare 4 DMA request (CC4DE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_CC4(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_CC4DEN) == (TMR_DIEN_CC4DEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable commutation DMA request (COMDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_COM(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_COMDEN);
}

/**
  * @brief  Disable commutation DMA request (COMDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_COM(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_COMDEN);
}

/**
  * @brief  Indicates whether the commutation DMA request (COMDE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_COM(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_COMDEN) == (TMR_DIEN_COMDEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable trigger interrupt (TDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_EnableDMAReq_TRIG(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->DIEN, TMR_DIEN_TRGDEN);
}

/**
  * @brief  Disable trigger interrupt (TDE).
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_DisableDMAReq_TRIG(TMR_TypeDef *TMRx)
{
  CLEAR_BIT(TMRx->DIEN, TMR_DIEN_TRGDEN);
}

/**
  * @brief  Indicates whether the trigger interrupt (TDE) is enabled.
  * @param  TMRx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_TMR_IsEnabledDMAReq_TRIG(TMR_TypeDef *TMRx)
{
  return ((READ_BIT(TMRx->DIEN, TMR_DIEN_TRGDEN) == (TMR_DIEN_TRGDEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TMR_DDL_EF_EVENT_Management EVENT-Management
  * @{
  */
/**
  * @brief  Generate an update event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_UPDATE(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_UEG);
}

/**
  * @brief  Generate Capture/Compare 1 event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_CC1(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_CC1EG);
}

/**
  * @brief  Generate Capture/Compare 2 event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_CC2(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_CC2EG);
}

/**
  * @brief  Generate Capture/Compare 3 event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_CC3(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_CC3EG);
}

/**
  * @brief  Generate Capture/Compare 4 event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_CC4(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_CC4EG);
}

/**
  * @brief  Generate commutation event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_COM(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_COMG);
}

/**
  * @brief  Generate trigger event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_TRIG(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_TEG);
}

/**
  * @brief  Generate break event.
  * @param  TMRx Timer instance
  * @retval None
  */
__STATIC_INLINE void DDL_TMR_GenerateEvent_BRK(TMR_TypeDef *TMRx)
{
  SET_BIT(TMRx->CEG, TMR_CEG_BEG);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup TMR_DDL_EF_Init Initialisation and deinitialisation functions
  * @{
  */

ErrorStatus DDL_TMR_DeInit(TMR_TypeDef *TMRx);
void DDL_TMR_StructInit(DDL_TMR_InitTypeDef *TMR_InitStruct);
ErrorStatus DDL_TMR_Init(TMR_TypeDef *TMRx, DDL_TMR_InitTypeDef *TMR_InitStruct);
void DDL_TMR_OC_StructInit(DDL_TMR_OC_InitTypeDef *TMR_OC_InitStruct);
ErrorStatus DDL_TMR_OC_Init(TMR_TypeDef *TMRx, uint32_t Channel, DDL_TMR_OC_InitTypeDef *TMR_OC_InitStruct);
void DDL_TMR_IC_StructInit(DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct);
ErrorStatus DDL_TMR_IC_Init(TMR_TypeDef *TMRx, uint32_t Channel, DDL_TMR_IC_InitTypeDef *TMR_IC_InitStruct);
void DDL_TMR_ENCODER_StructInit(DDL_TMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct);
ErrorStatus DDL_TMR_ENCODER_Init(TMR_TypeDef *TMRx, DDL_TMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct);
void DDL_TMR_HALLSENSOR_StructInit(DDL_TMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct);
ErrorStatus DDL_TMR_HALLSENSOR_Init(TMR_TypeDef *TMRx, DDL_TMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct);
void DDL_TMR_BDT_StructInit(DDL_TMR_BDT_InitTypeDef *TMR_BDTInitStruct);
ErrorStatus DDL_TMR_BDT_Init(TMR_TypeDef *TMRx, DDL_TMR_BDT_InitTypeDef *TMR_BDTInitStruct);
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* TMR1 || TMR2 || TMR3 || TMR4 || TMR5 || TMR6 || TMR7 || TMR8 || TMR9 || TMR10 || TMR11 || TMR12 || TMR13 || TMR14 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_TMR_H */
