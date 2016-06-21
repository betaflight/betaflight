/**
  ******************************************************************************
  * @file    stm32f4xx_lptim.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the Low Power Timer (LPT) peripheral:
  *           + Initialization functions.
  *           + Configuration functions.
  *           + Interrupts and flags management functions.
  *
  *  @verbatim
  *
================================================================================
                   ##### How to use this driver #####
================================================================================

           Basic configuration:
           --------------------
           - Configure the clock source, the prescaler, the waveform shape and
             the output polarity by filling the "LPTIM_InitTypeDef" structure and
             calling LPTIM_Init.
           - If the ULPTIM source is selected as clock source, configure the digital
             Glitch filter by setting the number of consecutive samples
             to be detected by using LPTIM_ConfigClockGlitchFilter.
           - To select a software start use LPTIM_SelectSoftwareStart.
           - To select an external trigger for the start of the counter, configure
             the source and its active edge polarity by calling
             LPTIM_ConfigExternalTrigger. Configure the Digital Glitch filter for
             the external triggers by setting the number of consecutive samples
             to be detected by using LPTIM_ConfigTriggerGlitchFilter.
           - Select the operating mode of the peripheral by using
             LPTIM_SelectOperatingMode, 2 modes can be selected:
               + Continuous mode: the timer is free running, the timer is started
                 from a trigger event and never stops until the timer is disabled
               + One shot mode: the timer is started from a trigger event and
                 stops when reaching the auto-reload value.
           - Use LPTIM_SetAutoreloadValue to set the auto-reload value and
             LPTIM_SetCompareValue to set the compare value.
           - Configure the preload mode by using LPTIM_ConfigUpdate function. 2 modes
             are available:
               + The Autoreload and compare registers are updated immediately after
                 APB write.
               + The Autoreload and compare registers are updated at the end of
                 counter period.
            - Enable the peripheral by calling LPTIM_Cmd.

           Encoder mode:
           -------------
           - To select the encoder feature, use the function: LPTIM_SelectEncoderMode.
           - To select on which edge (Rising edge, falling edge or both edges)
             the counter is incremented, use LPTIM_SelectClockPolarity.

           Counter mode:
           -------------
           - Use LPTIM_SelectCounterMode to select the counting mode. In this mode
             the counter is incremented on each valid event on ULPTIM.

           Timeout function:
           -----------------
           In this case, the trigger will reset the timer. The first trigger event
           will start the timer, any successive trigger event will reset the counter
           and the timer restarts.
           - To active this feature use LPTIM_TimoutCmd.

           Interrupt configuration:
           ------------------------
           - Use LPTIM_ITConfig to configure an interruption.
           - Call LPTIM_GetFlagStatus to get a flag status.
           - Call LPTIM_GetITStatus to get an interrupt status.
           - Use LPTIM_ClearFlag to clear a flag.
  @endverbatim
  *
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_lptim.h"


/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup LPTIM
  * @brief LPTIM driver modules
  * @{
  */
#if defined(STM32F410xx)
/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define CFGR_INIT_CLEAR_MASK                 ((uint32_t) 0xFFCFF1FE)
#define CFGR_TRIG_AND_POL_CLEAR_MASK         ((uint32_t) 0xFFF91FFF)
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup LPTIM_Private_Functions
  * @{
  */

/** @defgroup LPTIM_Group1 Initialization functions
  *  @brief   Initialization functions
  *
@verbatim
 ===============================================================================
                             Initialization functions
 ===============================================================================
  This section provides functions allowing to:
   - Deinitialize  the LPTimer
   - Initialize the Clock source, the Prescaler, the Ouput Waveform shape and Polarity
   - Initialize the member of LPTIM_InitStruct structer with default value

@endverbatim
  * @{
  */

/**
  * @brief  Deinitializes the LPTIMx peripheral registers to their default reset values.
  * @param  LPTIMx: where x can be 1.
  * @retval None
  *
  */
void LPTIM_DeInit(LPTIM_TypeDef* LPTIMx)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Deinitializes the LPTIM1 peripheral */
  if(LPTIMx == LPTIM1)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_LPTIM1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_LPTIM1, DISABLE);
  }
}

/**
  * @brief  Initializes the LPTIMx peripheral according to the specified parameters
  *         in the LPTIM_InitStruct.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_InitStruct: pointer to an LPTIM_InitTypeDef structure that contains
  *         the configuration information for the specified LPTIM peripheral.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_Init(LPTIM_TypeDef* LPTIMx, LPTIM_InitTypeDef* LPTIM_InitStruct)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLOCK_SOURCE(LPTIM_InitStruct->LPTIM_ClockSource));
  assert_param(IS_LPTIM_CLOCK_PRESCALER(LPTIM_InitStruct->LPTIM_Prescaler));
  assert_param(IS_LPTIM_WAVEFORM(LPTIM_InitStruct->LPTIM_Waveform));
  assert_param(IS_LPTIM_OUTPUT_POLARITY(LPTIM_InitStruct->LPTIM_OutputPolarity));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear CKSEL, PRESC, WAVE and WAVEPOL bits */
  tmpreg1 &= CFGR_INIT_CLEAR_MASK;
  
  /* Set or Reset CKSEL bit according to LPTIM_ClockSource value */
  /* Set or Reset PRESC bits according to LPTIM_Prescaler value */
  /* Set or Reset WAVE bit according to LPTIM_Waveform value */
  /* Set or Reset WAVEPOL bit according to LPTIM_OutputPolarity value */
  tmpreg1 |= (LPTIM_InitStruct->LPTIM_ClockSource | LPTIM_InitStruct->LPTIM_Prescaler
              |LPTIM_InitStruct->LPTIM_Waveform | LPTIM_InitStruct->LPTIM_OutputPolarity);
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Fills each LPTIM_InitStruct member with its default value.
  * @param  LPTIM_InitStruct : pointer to a LPTIM_InitTypeDef structure which will be initialized.
  * @retval None
  */
void LPTIM_StructInit(LPTIM_InitTypeDef* LPTIM_InitStruct)
{
  /* APB Clock/Low Power oscillators is selected as default Clock source*/
  LPTIM_InitStruct->LPTIM_ClockSource = LPTIM_ClockSource_APBClock_LPosc;
  
  /* High Polarity is selected as default polarity */
  LPTIM_InitStruct->LPTIM_OutputPolarity = LPTIM_OutputPolarity_High;
  
  /* DIV=1 is selected as default prescaler */
  LPTIM_InitStruct->LPTIM_Prescaler = LPTIM_Prescaler_DIV1;
  
  /* PWM/One pulse mode is selected as default Waveform shape */
  LPTIM_InitStruct->LPTIM_Waveform = LPTIM_Waveform_PWM_OnePulse;
}

/**
  * @}
  */

/** @defgroup LPTIM_Group2 Configuration functions
  *  @brief   Configuration functions
  *
@verbatim
 ===============================================================================
                       Configuration functions
 ===============================================================================
    This section provides functions allowing to configure the Low Power Timer:
    - Select the Clock source.
    - Configure the Glitch filter for the external clock and the external clock.
    - Configure the prescaler of the counter.
    - Select the Trigger source of the counter.
    - Configure the operating mode (Single or Continuous mode).
    - Select the Waveform shape (PWM/One Pulse or Set once) and polarity.
    - Enable or disable the Encoder mode and the Timeout function.
    - Write on the Autoreload and the Compare registers and configure the
      preload mode.
    - Get the Counter value.
    - Enable or disable the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified LPTIM peripheral.
  * @param  LPTIMx: where x can be 1.
  * @param  NewState: new state of the LPTIMx peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void LPTIM_Cmd(LPTIM_TypeDef* LPTIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Set the ENABLE bit */
    LPTIMx->CR |= LPTIM_CR_ENABLE;
  }
  else
  {
    /* Reset the ENABLE bit */
    LPTIMx->CR &= ~(LPTIM_CR_ENABLE);
  }
}

/**
  * @brief  Selects the Clock source of the LPTIM counter.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_ClockSource: the selected clock source.
  *         This parameter can be:
  *         @arg LPTIM_ClockSource_APBClock_LPosc : APB clock/LP oscillators selected
  *         @arg LPTIM_ClockSource_ULPTIM: ULPTIM (external input) selected
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_SelectClockSource(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_ClockSource)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLOCK_SOURCE(LPTIM_ClockSource));
  
  /* Clear the CKSEL bit */
  LPTIMx->CFGR &= ~(LPTIM_CFGR_CKSEL);
  
  /* Set or Reset the CKSEL bit */
  LPTIMx->CFGR |= LPTIM_ClockSource;
}

/**
  * @brief  Configures the polarity of the edge to be used to count
  *         if the ULPTIM input is selected.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_ClockPolarity: the selected clock polarity.
  * This parameter can be:
  *     @arg LPTIM_ClockPolarity_RisingEdge : Counter Clock = LPTIM Clock / 1
  *     @arg LPTIM_ClockPolarity_FallingEdge : Counter Clock = LPTIM Clock / 2
  *     @arg LPTIM_ClockPolarity_BothEdges : Counter Clock = LPTIM Clock / 4
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_SelectULPTIMClockPolarity(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_ClockPolarity)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLOCK_POLARITY(LPTIM_ClockPolarity));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear the CKPOL bits */
  tmpreg1 &= ~(LPTIM_CFGR_CKPOL);
  
  /* Set or Reset the PRESC bits */
  tmpreg1 |= LPTIM_ClockPolarity;
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Configures the Clock Prescaler.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Prescaler: the selected clock prescaler.
  * This parameter can be:
  *     @arg LPTIM_Prescaler_DIV1 : Counter Clock = LPTIM Clock / 1
  *     @arg LPTIM_Prescaler_DIV2 : Counter Clock = LPTIM Clock / 2
  *     @arg LPTIM_Prescaler_DIV4 : Counter Clock = LPTIM Clock / 4
  *     @arg LPTIM_Prescaler_DIV8 : Counter Clock = LPTIM Clock / 8
  *     @arg LPTIM_Prescaler_DIV16 : Counter Clock = LPTIM Clock / 16
  *     @arg LPTIM_Prescaler_DIV32 : Counter Clock = LPTIM Clock / 32
  *     @arg LPTIM_Prescaler_DIV64 : Counter Clock = LPTIM Clock / 64
  *     @arg LPTIM_Prescaler_DIV128 : Counter Clock = LPTIM Clock / 128
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_ConfigPrescaler(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Prescaler)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLOCK_PRESCALER(LPTIM_Prescaler));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear the PRESC bits */
  tmpreg1 &= ~(LPTIM_CFGR_PRESC);
  
  /* Set or Reset the PRESC bits */
  tmpreg1 |= LPTIM_Prescaler;
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Selects the trigger source for the counter and its polarity.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_ExtTRGSource: the selected external trigger.
  * This parameter can be:
  *     @arg LPTIM_ExtTRGSource_Trig0 : ext_trig0
  *     @arg LPTIM_ExtTRGSource_Trig1 : ext_trig1
  *     @arg LPTIM_ExtTRGSource_Trig2 : ext_trig2
  *     @arg LPTIM_ExtTRGSource_Trig3 : ext_trig3
  *     @arg LPTIM_ExtTRGSource_Trig4 : ext_trig4
  *     @arg LPTIM_ExtTRGSource_Trig5 : ext_trig5
  *     @arg LPTIM_ExtTRGSource_Trig6 : ext_trig6
  *     @arg LPTIM_ExtTRGSource_Trig7 : ext_trig7
  * @param  LPTIM_ExtTRGPolarity: the selected external trigger.
  * This parameter can be:
  *     @arg LPTIM_ExtTRGPolarity_RisingEdge : Rising edge polarity selected
  *     @arg LPTIM_ExtTRGPolarity_FallingEdge : Falling edge polarity selected
  *     @arg LPTIM_ExtTRGPolarity_BothEdges : Both edges polarity selected
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_ConfigExternalTrigger(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_ExtTRGSource, uint32_t LPTIM_ExtTRGPolarity)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_EXT_TRG_SOURCE(LPTIM_ExtTRGSource));
  assert_param(IS_LPTIM_EXT_TRG_POLARITY(LPTIM_ExtTRGPolarity));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear the TRIGEN and TRIGSEL bits */
  tmpreg1 &= CFGR_TRIG_AND_POL_CLEAR_MASK;
  
  /* Set or Reset the TRIGEN and TRIGSEL bits */
  tmpreg1 |= (LPTIM_ExtTRGSource | LPTIM_ExtTRGPolarity);
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Selects a software start of the counter.
  * @param  LPTIMx: where x can be 1.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_SelectSoftwareStart(LPTIM_TypeDef* LPTIMx)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Reset the TRIGEN bits to allow a software start */
  LPTIMx->CFGR &= ~(LPTIM_CFGR_TRIGEN);
}

/**
  * @brief  Configures the digital filter for trigger by determining the number of consecutive
  *         samples at the specified level to detect a correct transition.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_TrigSampleTime: the number of samples to detect a valid transition.
  * This parameter can be:
  *     @arg LPTIM_TrigSampleTime_DirectTransistion : Event is detected on input transitions
  *     @arg LPTIM_TrigSampleTime_2Transistions : Event is detected after 2 consecutive samples at the active level
  *     @arg LPTIM_TrigSampleTime_4Transistions : Event is detected after 4 consecutive samples at the active level
  *     @arg LPTIM_TrigSampleTime_8Transistions : Event is detected after 8 consecutive samples at the active level
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  * @note   An auxiliary clock must be present to use this feature.
  */
void LPTIM_ConfigTriggerGlitchFilter(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_TrigSampleTime)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_TRIG_SAMPLE_TIME(LPTIM_TrigSampleTime));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear the TRGFLT bits */
  tmpreg1 &= ~(LPTIM_CFGR_TRGFLT);
  
  /* Set or Reset the TRGFLT bits according to LPTIM_TrigSampleTime */
  tmpreg1 |= (LPTIM_TrigSampleTime);
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Configures the digital filter for  the external clock by determining the number
            of consecutive samples at the specified level to detect a correct transition.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_ClockSampleTime: the number of samples to detect a valid transition.
  * This parameter can be:
  *     @arg LPTIM_ClockSampleTime_DirectTransistion : Event is detected on input transitions
  *     @arg LPTIM_ClockSampleTime_2Transistions : Event is detected after 2 consecutive samples at the active level
  *     @arg LPTIM_ClockSampleTime_4Transistions : Event is detected after 4 consecutive samples at the active level
  *     @arg LPTIM_ClockSampleTime_8Transistions : Event is detected after 8 consecutive samples at the active level
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  * @note   An auxiliary clock must be present to use this feature.
  */
void LPTIM_ConfigClockGlitchFilter(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_ClockSampleTime)
{
  uint32_t tmpreg1 = 0;
  
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLOCK_SAMPLE_TIME(LPTIM_ClockSampleTime));
  
  /* Get the LPTIMx CFGR value */
  tmpreg1 = LPTIMx->CFGR;
  
  /* Clear the CKFLT bits */
  tmpreg1 &= ~(LPTIM_CFGR_CKFLT);
  
  /* Set or Reset the CKFLT bits according to LPTIM_ClockSampleTime */
  tmpreg1 |= LPTIM_ClockSampleTime;
  
  /* Write to LPTIMx CFGR */
  LPTIMx->CFGR = tmpreg1;
}

/**
  * @brief  Selects an operating mode.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Mode: the selected mode.
  * This parameter can be:
  *     @arg LPTIM_Mode_Continuous : Timer starts in Continuous mode
  *     @arg LPTIM_Mode_Single : Timer will starts in Single mode
  * @retval None
  */
void LPTIM_SelectOperatingMode(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Mode)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_MODE(LPTIM_Mode));
  
  
  if(LPTIM_Mode == LPTIM_Mode_Continuous)
  {
    /* Set the CNTSTRT to select the continuous start*/
    LPTIMx->CR |= LPTIM_Mode_Continuous;
  }
  else if(LPTIM_Mode == LPTIM_Mode_Single)
  {
    /* Set the SNGSTRT to select the continuous start*/
    LPTIMx->CR |= LPTIM_Mode_Single;
  }
}

/**
  * @brief  Enables or disables the Timeout function.
  * @param  LPTIMx: where x can be 1.
  * @param  NewState: new state of the Timeout function.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_TimoutCmd(LPTIM_TypeDef* LPTIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Set the TIMOUT bit */
    LPTIMx->CFGR |= LPTIM_CFGR_TIMOUT;
  }
  else
  {
    /* Reset the TIMOUT bit */
    LPTIMx->CFGR &= ~(LPTIM_CFGR_TIMOUT);
  }
}

/**
  * @brief  Configures the Waveform shape.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Waveform: the selected waveform shape.
  * This parameter can be:
  *     @arg LPTIM_Waveform_PWM_OnePulse : PWM/One Pulse is selected
  *     @arg LPTIM_Waveform_SetOnce : Set once is selected
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_ConfigWaveform(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Waveform)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_WAVEFORM(LPTIM_Waveform));
  
  /* Clear the WAVE bit */
  LPTIMx->CFGR &= ~(LPTIM_CFGR_CKFLT);
  
  /* Set or Reset the WAVE bit according to LPTIM_Waveform */
  LPTIMx->CFGR |= (LPTIM_Waveform);
}

/**
  * @brief  Configures the Autoreload and Compare registers update mode.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Update: The selected update mode.
  * This parameter can be:
  *     @arg LPTIM_Update_Immediate : Registers updated after APB write
  *     @arg LPTIM_Update_EndOfPeriod : Registers updated at the end of current timer preload
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_ConfigUpdate(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Update)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_UPDATE(LPTIM_Update));
  
  /* Clear the PRELOAD bit */
  LPTIMx->CFGR &= ~(LPTIM_CFGR_PRELOAD);
  
  /* Set or Reset the PRELOAD bit according to LPTIM_Update */
  LPTIMx->CFGR |= (LPTIM_Update);
}

/**
  * @brief  Writes the passed parameter in the Autoreload register.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Autoreload: The Autoreload value.
  *         This parameter must be a value between 0x0000 and 0xFFFF
  * @retval None
  */
void LPTIM_SetAutoreloadValue(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Autoreload)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_AUTORELOAD(LPTIM_Autoreload));
  
  /* Write LPTIM_Autoreload in Autoreload register */
  LPTIMx->ARR = LPTIM_Autoreload;
}

/**
  * @brief  Writes the passed parameter in the Compare register.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_Compare: The Compare value.
  *         This parameter must be a value between 0x0000 and 0xFFFF
  * @retval None
  */
void LPTIM_SetCompareValue(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_Compare)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_COMPARE(LPTIM_Compare));
  
  /* Write LPTIM_Compare in Compare register */
  LPTIMx->CMP = LPTIM_Compare;
}

/**
  * @brief  Enables or disables the Counter mode. When the Counter mode is enabled,
  *         the counter is incremented each valid event on ULPTIM
  * @param  LPTIMx: where x can be 1.
  * @param  NewState: new state of the Counter mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_SelectCounterMode(LPTIM_TypeDef* LPTIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Set the COUNTMODE bit */
    LPTIMx->CFGR |= LPTIM_CFGR_COUNTMODE;
  }
  else
  {
    /* Reset the COUNTMODE bit */
    LPTIMx->CFGR &= ~(LPTIM_CFGR_COUNTMODE);
  }
}

/**
  * @brief  Enables or disables the Encoder mode.
  * @param  LPTIMx: where x can be 1.
  * @param  NewState: New state of the encoder mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_SelectEncoderMode(LPTIM_TypeDef* LPTIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Set the ENC bit */
    LPTIMx->CFGR |= LPTIM_CFGR_ENC;
  }
  else
  {
    /* Reset the ENC bit */
    LPTIMx->CFGR &= ~(LPTIM_CFGR_ENC);
  }
}

/**
  * @brief  Gets the LPTIMx counter value.
  * @param  LPTIMx: where x can be 1.
  * @retval Counter Register value
  */
uint32_t LPTIM_GetCounterValue(LPTIM_TypeDef* LPTIMx)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Get the Counter Register value */
  return LPTIMx->CNT;
}

/**
  * @brief  Gets the LPTIMx Autoreload value.
  * @param  LPTIMx: where x can be 1.
  * @retval Counter Register value
  */
uint32_t LPTIM_GetAutoreloadValue(LPTIM_TypeDef* LPTIMx)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Get the Counter Register value */
  return LPTIMx->ARR;
}

/**
  * @brief  Gets the LPTIMx Compare value.
  * @param  LPTIMx: where x can be 1.
  * @retval Counter Register value
  */
uint32_t LPTIM_GetCompareValue(LPTIM_TypeDef* LPTIMx)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Get the Counter Register value */
  return LPTIMx->CMP;
}

/**
  * @brief  LPTIM Input 1 Remap.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_OPTR :
  * This Parameter can be :
  *    @arg LPTIM_OP_PAD_AF  : Port B5 on AF1 or Port C0 on AF1 for input timer
  *    @arg LPTIM_OP_PAD_PA4 : Input remapped to Port A4
  *    @arg RCC_LPTIM1CLKSOURCE_LSI : Input remapped to Port B9
  *    @arg LPTIM_OP_TIM_DAC : Input coming from timer 6 output (for encoder mode)
  * @retval Counter Register value
  */
void LPTIM_RemapConfig(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_OPTR)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  
  /* Get the Counter Register value */
  LPTIMx->OR = LPTIM_OPTR;
}

/**
  * @}
  */

/** @defgroup LPTIM_Group3 Interrupts and flags management functions
  *  @brief    Interrupts and flags management functions
  *
@verbatim
 ===============================================================================
                  Interrupts and flags management functions
 ===============================================================================
  This section provides functions allowing to configure the LPTIM Interrupts, get
  the status and clear flags bits.

  The LPTIM provides 7 Flags and Interrupts sources (2 flags and Interrupt sources
  are available only on LPTIM peripherals equipped with encoder mode interface)

  Flags and Interrupts sources:
  =============================
  1. Compare match.
  2. Auto-reload match.
  3. External trigger event.
  4. Autoreloaded register write completed.
  5. Compare register write completed.
  6. Direction change: from up to down [Available only for LPTIM peripheral with
     encoder mode module]
  7. Direction change: from down to up [Available only for LPTIM peripheral with
     encoder mode module]

  - To enable a specific interrupt source, use "LPTIM_ITConfig" function.
  - To check if an interrupt was occurred, call "LPTIM_GetITStatus" function and read
    the returned value.
  - To get a flag status, call the "LPTIM_GetFlagStatus" function and read the returned
    value.
  - To clear a flag or an interrupt, use LPTIM_ClearFlag function with the
    corresponding flag (interrupt).

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified LPTIM interrupts.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_IT: specifies the TIM interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *            @arg LPTIM_IT_DOWN: Counter direction change up to down Interrupt source
  *            @arg LPTIM_IT_UP: Counter direction change down to up Interrupt source
  *            @arg LPTIM_IT_ARROK: Autoreload register update OK Interrupt source
  *            @arg LPTIM_IT_CMPOK: Compare register update OK Interrupt source
  *            @arg LPTIM_IT_EXTTRIG: External trigger edge event Interrupt source
  *            @arg LPTIM_IT_ARRM: Autoreload match Interrupt source
  *            @arg LPTIM_IT_CMPM: Compare match Interrupt source
  * @note   LPTIM_IT_DOWN is available only for LPTIM1.
  * @note   LPTIM_IT_UP is available only for LPTIM1.
  * @param  NewState: new state of the TIM interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  *
  * @note   It is mandatory to disable the peripheral to use this function.
  */
void LPTIM_ITConfig(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_IT, FunctionalState NewState)
 {
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_IT(LPTIM_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    LPTIMx->IER |= LPTIM_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    LPTIMx->IER &= ~(LPTIM_IT);
  }
}

/**
  * @brief  Checks whether the specified LPTIM flag is set or not.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_FLAG: specifies the flag to check.
  *         This parameter can be any combination of the following values:
  *            @arg LPTIM_FLAG_DOWN: Counter direction change up Flag
  *            @arg LPTIM_FLAG_UP: Counter direction change down to up Flag
  *            @arg LPTIM_FLAG_ARROK: Autoreload register update OK Flag
  *            @arg LPTIM_FLAG_CMPOK: Compare register update OK Flag
  *            @arg LPTIM_FLAG_EXTTRIG: External trigger edge event Flag
  *            @arg LPTIM_FLAG_ARRM: Autoreload match Flag
  *            @arg LPTIM_FLAG_CMPM: Compare match Flag
  * @note   LPTIM_Flag_DOWN is generated only for LPTIM1.
  * @note   LPTIM_Flag_UP is generated only for LPTIM1.
  * @param  NewState: new state of the TIM interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
FlagStatus LPTIM_GetFlagStatus(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_FLAG)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_GET_FLAG(LPTIM_FLAG));

  if((LPTIMx->ISR & LPTIM_FLAG) != (RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the LPTIMx's pending flag.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_CLEARF: specifies the pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg LPTIM_CLEARF_DOWN: Counter direction change up Clear Flag
  *            @arg LPTIM_CLEARF_UP: Counter direction change down to up Clear Flag
  *            @arg LPTIM_CLEARF_ARROK: Autoreload register update OK Clear Flag
  *            @arg LPTIM_CLEARF_CMPOK: Compare register update OK Clear Flag
  *            @arg LPTIM_CLEARF_EXTTRIG: External trigger edge event Clear Flag
  *            @arg LPTIM_CLEARF_ARRM: Autoreload match Clear Flag
  *            @arg LPTIM_CLEARF_CMPM: Compare match Clear Flag
  * @note   LPTIM_Flag_DOWN is generated only for LPTIM1.
  * @note   LPTIM_Flag_UP is generated only for LPTIM1.
  * @retval None
  */
void LPTIM_ClearFlag(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_CLEARF)
{
  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_CLEAR_FLAG(LPTIM_CLEARF));

  /* Clear the IT pending Bit */
  LPTIMx->ICR |= LPTIM_CLEARF;
}

/**
  * @brief  Check whether the specified LPTIM interrupt has occurred or not.
  * @param  LPTIMx: where x can be 1.
  * @param  LPTIM_IT: specifies the LPTIM interrupt source to check.
  *            @arg LPTIM_IT_DOWN: Counter direction change up to down Interrupt source
  *            @arg LPTIM_IT_UP: Counter direction change down to up Interrupt source
  *            @arg LPTIM_IT_ARROK: Autoreload register update OK Interrupt source
  *            @arg LPTIM_IT_CMPOK: Compare register update OK Interrupt source
  *            @arg LPTIM_IT_EXTTRIG: External trigger edge event Interrupt source
  *            @arg LPTIM_IT_ARRM: Autoreload match Interrupt source
  *            @arg LPTIM_IT_CMPM: Compare match Interrupt source
  * @retval The new state of LPTIM_IT (SET or RESET).
  */
ITStatus LPTIM_GetITStatus(LPTIM_TypeDef* LPTIMx, uint32_t LPTIM_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t itstatus = 0x0, itenable = 0x0;

  /* Check the parameters */
  assert_param(IS_LPTIM_ALL_PERIPH(LPTIMx));
  assert_param(IS_LPTIM_IT(LPTIM_IT));

  /* Get the Interrupt Status bit value */
  itstatus = LPTIMx->ISR & LPTIM_IT;

  /* Check if the Interrupt is enabled */
  itenable = LPTIMx->IER & LPTIM_IT;

  if((itstatus != RESET) && (itenable != RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F410xx */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

