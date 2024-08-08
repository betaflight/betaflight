/**
  *
  * @file    apm32f4xx_dal_tmr.h
  * @brief   Header file of TMR DAL module.
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
#ifndef APM32F4xx_DAL_TMR_H
#define APM32F4xx_DAL_TMR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup TMR
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TMR_Exported_Types TMR Exported Types
  * @{
  */

/**
  * @brief  TMR Time base Configuration Structure definition
  */
typedef struct
{
  uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TMR clock.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TMR_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TMR_ClockDivision */

  uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the REPCNT downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the REPCNT value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     GP timers: this parameter must be a number between Min_Data = 0x00 and
                                     Max_Data = 0xFF.
                                     Advanced timers: this parameter must be a number between Min_Data = 0x0000 and
                                     Max_Data = 0xFFFF. */

  uint32_t AutoReloadPreload;  /*!< Specifies the auto-reload preload.
                                   This parameter can be a value of @ref TMR_AutoReloadPreload */
} TMR_Base_InitTypeDef;

/**
  * @brief  TMR Output Compare Configuration Structure definition
  */
typedef struct
{
  uint32_t OCMode;        /*!< Specifies the TMR mode.
                               This parameter can be a value of @ref TMR_Output_Compare_and_PWM_modes */

  uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref TMR_Output_Compare_Polarity */

  uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TMR_Output_Compare_N_Polarity
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OCFastMode;    /*!< Specifies the Fast mode state.
                               This parameter can be a value of @ref TMR_Output_Fast_State
                               @note This parameter is valid only in PWM1 and PWM2 mode. */


  uint32_t OCIdleState;   /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_Output_Compare_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OCNIdleState;  /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_Output_Compare_N_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */
} TMR_OC_InitTypeDef;

/**
  * @brief  TMR One Pulse Mode Configuration Structure definition
  */
typedef struct
{
  uint32_t OCMode;        /*!< Specifies the TMR mode.
                               This parameter can be a value of @ref TMR_Output_Compare_and_PWM_modes */

  uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref TMR_Output_Compare_Polarity */

  uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TMR_Output_Compare_N_Polarity
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OCIdleState;   /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_Output_Compare_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OCNIdleState;  /*!< Specifies the TMR Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TMR_Output_Compare_N_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t ICPolarity;    /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TMR_Input_Capture_Polarity */

  uint32_t ICSelection;   /*!< Specifies the input.
                              This parameter can be a value of @ref TMR_Input_Capture_Selection */

  uint32_t ICFilter;      /*!< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TMR_OnePulse_InitTypeDef;

/**
  * @brief  TMR Input Capture Configuration Structure definition
  */
typedef struct
{
  uint32_t ICPolarity;   /*!< Specifies the active edge of the input signal.
                              This parameter can be a value of @ref TMR_Input_Capture_Polarity */

  uint32_t ICSelection;  /*!< Specifies the input.
                              This parameter can be a value of @ref TMR_Input_Capture_Selection */

  uint32_t ICPrescaler;  /*!< Specifies the Input Capture Prescaler.
                              This parameter can be a value of @ref TMR_Input_Capture_Prescaler */

  uint32_t ICFilter;     /*!< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TMR_IC_InitTypeDef;

/**
  * @brief  TMR Encoder Configuration Structure definition
  */
typedef struct
{
  uint32_t EncoderMode;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TMR_Encoder_Mode */

  uint32_t IC1Polarity;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TMR_Encoder_Input_Polarity */

  uint32_t IC1Selection;  /*!< Specifies the input.
                               This parameter can be a value of @ref TMR_Input_Capture_Selection */

  uint32_t IC1Prescaler;  /*!< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TMR_Input_Capture_Prescaler */

  uint32_t IC1Filter;     /*!< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

  uint32_t IC2Polarity;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TMR_Encoder_Input_Polarity */

  uint32_t IC2Selection;  /*!< Specifies the input.
                              This parameter can be a value of @ref TMR_Input_Capture_Selection */

  uint32_t IC2Prescaler;  /*!< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TMR_Input_Capture_Prescaler */

  uint32_t IC2Filter;     /*!< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TMR_Encoder_InitTypeDef;

/**
  * @brief  Clock Configuration Handle Structure definition
  */
typedef struct
{
  uint32_t ClockSource;     /*!< TMR clock sources
                                 This parameter can be a value of @ref TMR_Clock_Source */
  uint32_t ClockPolarity;   /*!< TMR clock polarity
                                 This parameter can be a value of @ref TMR_Clock_Polarity */
  uint32_t ClockPrescaler;  /*!< TMR clock prescaler
                                 This parameter can be a value of @ref TMR_Clock_Prescaler */
  uint32_t ClockFilter;     /*!< TMR clock filter
                                 This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TMR_ClockConfigTypeDef;

/**
  * @brief  TMR Clear Input Configuration Handle Structure definition
  */
typedef struct
{
  uint32_t ClearInputState;      /*!< TMR clear Input state
                                      This parameter can be ENABLE or DISABLE */
  uint32_t ClearInputSource;     /*!< TMR clear Input sources
                                      This parameter can be a value of @ref TMR_ClearInput_Source */
  uint32_t ClearInputPolarity;   /*!< TMR Clear Input polarity
                                      This parameter can be a value of @ref TMR_ClearInput_Polarity */
  uint32_t ClearInputPrescaler;  /*!< TMR Clear Input prescaler
                                      This parameter must be 0: When OCRef clear feature is used with ETR source,
                                      ETR prescaler must be off */
  uint32_t ClearInputFilter;     /*!< TMR Clear Input filter
                                      This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TMR_ClearInputConfigTypeDef;

/**
  * @brief  TMR Master configuration Structure definition
  */
typedef struct
{
  uint32_t  MasterOutputTrigger;   /*!< Trigger output (TRGO) selection
                                        This parameter can be a value of @ref TMR_Master_Mode_Selection */
  uint32_t  MasterSlaveMode;       /*!< Master/slave mode selection
                                        This parameter can be a value of @ref TMR_Master_Slave_Mode
                                        @note When the Master/slave mode is enabled, the effect of
                                        an event on the trigger input (TRGI) is delayed to allow a
                                        perfect synchronization between the current timer and its
                                        slaves (through TRGO). It is not mandatory in case of timer
                                        synchronization mode. */
} TMR_MasterConfigTypeDef;

/**
  * @brief  TMR Slave configuration Structure definition
  */
typedef struct
{
  uint32_t  SlaveMode;         /*!< Slave mode selection
                                    This parameter can be a value of @ref TMR_Slave_Mode */
  uint32_t  InputTrigger;      /*!< Input Trigger source
                                    This parameter can be a value of @ref TMR_Trigger_Selection */
  uint32_t  TriggerPolarity;   /*!< Input Trigger polarity
                                    This parameter can be a value of @ref TMR_Trigger_Polarity */
  uint32_t  TriggerPrescaler;  /*!< Input trigger prescaler
                                    This parameter can be a value of @ref TMR_Trigger_Prescaler */
  uint32_t  TriggerFilter;     /*!< Input trigger filter
                                    This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */

} TMR_SlaveConfigTypeDef;

/**
  * @brief  TMR Break input(s) and Dead time configuration Structure definition
  * @note   2 break inputs can be configured (BKIN and BKIN2) with configurable
  *        filter and polarity.
  */
typedef struct
{
  uint32_t OffStateRunMode;      /*!< TMR off state in run mode, This parameter can be a value of @ref TMR_OSSR_Off_State_Selection_for_Run_mode_state */

  uint32_t OffStateIDLEMode;     /*!< TMR off state in IDLE mode, This parameter can be a value of @ref TMR_OSSI_Off_State_Selection_for_Idle_mode_state */

  uint32_t LockLevel;            /*!< TMR Lock level, This parameter can be a value of @ref TMR_Lock_level */

  uint32_t DeadTime;             /*!< TMR dead Time, This parameter can be a number between Min_Data = 0x00 and Max_Data = 0xFF */

  uint32_t BreakState;           /*!< TMR Break State, This parameter can be a value of @ref TMR_Break_Input_enable_disable */

  uint32_t BreakPolarity;        /*!< TMR Break input polarity, This parameter can be a value of @ref TMR_Break_Polarity */

  uint32_t BreakFilter;          /*!< Specifies the break input filter.This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

  uint32_t AutomaticOutput;      /*!< TMR Automatic Output Enable state, This parameter can be a value of @ref TMR_AOE_Bit_Set_Reset */

} TMR_BreakDeadTimeConfigTypeDef;

/**
  * @brief  DAL State structures definition
  */
typedef enum
{
  DAL_TMR_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  DAL_TMR_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  DAL_TMR_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
  DAL_TMR_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
  DAL_TMR_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} DAL_TMR_StateTypeDef;

/**
  * @brief  TMR Channel States definition
  */
typedef enum
{
  DAL_TMR_CHANNEL_STATE_RESET             = 0x00U,    /*!< TMR Channel initial state                         */
  DAL_TMR_CHANNEL_STATE_READY             = 0x01U,    /*!< TMR Channel ready for use                         */
  DAL_TMR_CHANNEL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing on the TMR channel */
} DAL_TMR_ChannelStateTypeDef;

/**
  * @brief  DMA Burst States definition
  */
typedef enum
{
  DAL_DMA_BURST_STATE_RESET             = 0x00U,    /*!< DMA Burst initial state */
  DAL_DMA_BURST_STATE_READY             = 0x01U,    /*!< DMA Burst ready for use */
  DAL_DMA_BURST_STATE_BUSY              = 0x02U,    /*!< Ongoing DMA Burst       */
} DAL_TMR_DMABurstStateTypeDef;

/**
  * @brief  DAL Active channel structures definition
  */
typedef enum
{
  DAL_TMR_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
  DAL_TMR_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
  DAL_TMR_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
  DAL_TMR_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
  DAL_TMR_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} DAL_TMR_ActiveChannel;

/**
  * @brief  TMR Time Base Handle Structure definition
  */
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
typedef struct __TMR_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
{
  TMR_TypeDef                        *Instance;         /*!< Register base address                             */
  TMR_Base_InitTypeDef               Init;              /*!< TMR Time Base required parameters                 */
  DAL_TMR_ActiveChannel              Channel;           /*!< Active channel                                    */
  DMA_HandleTypeDef                  *hdma[7];          /*!< DMA Handlers array
                                                             This array is accessed by a @ref DMA_Handle_index */
  DAL_LockTypeDef                    Lock;              /*!< Locking object                                    */
  __IO DAL_TMR_StateTypeDef          State;             /*!< TMR operation state                               */
  __IO DAL_TMR_ChannelStateTypeDef   ChannelState[4];   /*!< TMR channel operation state                       */
  __IO DAL_TMR_ChannelStateTypeDef   ChannelNState[4];  /*!< TMR complementary channel operation state         */
  __IO DAL_TMR_DMABurstStateTypeDef  DMABurstState;     /*!< DMA burst operation state                         */

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  void (* Base_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);              /*!< TMR Base Msp Init Callback                              */
  void (* Base_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);            /*!< TMR Base Msp DeInit Callback                            */
  void (* IC_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);                /*!< TMR IC Msp Init Callback                                */
  void (* IC_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);              /*!< TMR IC Msp DeInit Callback                              */
  void (* OC_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);                /*!< TMR OC Msp Init Callback                                */
  void (* OC_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);              /*!< TMR OC Msp DeInit Callback                              */
  void (* PWM_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);               /*!< TMR PWM Msp Init Callback                               */
  void (* PWM_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);             /*!< TMR PWM Msp DeInit Callback                             */
  void (* OnePulse_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);          /*!< TMR One Pulse Msp Init Callback                         */
  void (* OnePulse_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);        /*!< TMR One Pulse Msp DeInit Callback                       */
  void (* Encoder_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);           /*!< TMR Encoder Msp Init Callback                           */
  void (* Encoder_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);         /*!< TMR Encoder Msp DeInit Callback                         */
  void (* HallSensor_MspInitCallback)(struct __TMR_HandleTypeDef *htmr);        /*!< TMR Hall Sensor Msp Init Callback                       */
  void (* HallSensor_MspDeInitCallback)(struct __TMR_HandleTypeDef *htmr);      /*!< TMR Hall Sensor Msp DeInit Callback                     */
  void (* PeriodElapsedCallback)(struct __TMR_HandleTypeDef *htmr);             /*!< TMR Period Elapsed Callback                             */
  void (* PeriodElapsedHalfCpltCallback)(struct __TMR_HandleTypeDef *htmr);     /*!< TMR Period Elapsed half complete Callback               */
  void (* TriggerCallback)(struct __TMR_HandleTypeDef *htmr);                   /*!< TMR Trigger Callback                                    */
  void (* TriggerHalfCpltCallback)(struct __TMR_HandleTypeDef *htmr);           /*!< TMR Trigger half complete Callback                      */
  void (* IC_CaptureCallback)(struct __TMR_HandleTypeDef *htmr);                /*!< TMR Input Capture Callback                              */
  void (* IC_CaptureHalfCpltCallback)(struct __TMR_HandleTypeDef *htmr);        /*!< TMR Input Capture half complete Callback                */
  void (* OC_DelayElapsedCallback)(struct __TMR_HandleTypeDef *htmr);           /*!< TMR Output Compare Delay Elapsed Callback               */
  void (* PWM_PulseFinishedCallback)(struct __TMR_HandleTypeDef *htmr);         /*!< TMR PWM Pulse Finished Callback                         */
  void (* PWM_PulseFinishedHalfCpltCallback)(struct __TMR_HandleTypeDef *htmr); /*!< TMR PWM Pulse Finished half complete Callback           */
  void (* ErrorCallback)(struct __TMR_HandleTypeDef *htmr);                     /*!< TMR Error Callback                                      */
  void (* CommutationCallback)(struct __TMR_HandleTypeDef *htmr);               /*!< TMR Commutation Callback                                */
  void (* CommutationHalfCpltCallback)(struct __TMR_HandleTypeDef *htmr);       /*!< TMR Commutation half complete Callback                  */
  void (* BreakCallback)(struct __TMR_HandleTypeDef *htmr);                     /*!< TMR Break Callback                                      */
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
} TMR_HandleTypeDef;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL TMR Callback ID enumeration definition
  */
typedef enum
{
  DAL_TMR_BASE_MSPINIT_CB_ID              = 0x00U   /*!< TMR Base MspInit Callback ID                              */
  , DAL_TMR_BASE_MSPDEINIT_CB_ID          = 0x01U   /*!< TMR Base MspDeInit Callback ID                            */
  , DAL_TMR_IC_MSPINIT_CB_ID              = 0x02U   /*!< TMR IC MspInit Callback ID                                */
  , DAL_TMR_IC_MSPDEINIT_CB_ID            = 0x03U   /*!< TMR IC MspDeInit Callback ID                              */
  , DAL_TMR_OC_MSPINIT_CB_ID              = 0x04U   /*!< TMR OC MspInit Callback ID                                */
  , DAL_TMR_OC_MSPDEINIT_CB_ID            = 0x05U   /*!< TMR OC MspDeInit Callback ID                              */
  , DAL_TMR_PWM_MSPINIT_CB_ID             = 0x06U   /*!< TMR PWM MspInit Callback ID                               */
  , DAL_TMR_PWM_MSPDEINIT_CB_ID           = 0x07U   /*!< TMR PWM MspDeInit Callback ID                             */
  , DAL_TMR_ONE_PULSE_MSPINIT_CB_ID       = 0x08U   /*!< TMR One Pulse MspInit Callback ID                         */
  , DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID     = 0x09U   /*!< TMR One Pulse MspDeInit Callback ID                       */
  , DAL_TMR_ENCODER_MSPINIT_CB_ID         = 0x0AU   /*!< TMR Encoder MspInit Callback ID                           */
  , DAL_TMR_ENCODER_MSPDEINIT_CB_ID       = 0x0BU   /*!< TMR Encoder MspDeInit Callback ID                         */
  , DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID     = 0x0CU   /*!< TMR Hall Sensor MspDeInit Callback ID                     */
  , DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID   = 0x0DU   /*!< TMR Hall Sensor MspDeInit Callback ID                     */
  , DAL_TMR_PERIOD_ELAPSED_CB_ID          = 0x0EU   /*!< TMR Period Elapsed Callback ID                             */
  , DAL_TMR_PERIOD_ELAPSED_HALF_CB_ID     = 0x0FU   /*!< TMR Period Elapsed half complete Callback ID               */
  , DAL_TMR_TRIGGER_CB_ID                 = 0x10U   /*!< TMR Trigger Callback ID                                    */
  , DAL_TMR_TRIGGER_HALF_CB_ID            = 0x11U   /*!< TMR Trigger half complete Callback ID                      */

  , DAL_TMR_IC_CAPTURE_CB_ID              = 0x12U   /*!< TMR Input Capture Callback ID                              */
  , DAL_TMR_IC_CAPTURE_HALF_CB_ID         = 0x13U   /*!< TMR Input Capture half complete Callback ID                */
  , DAL_TMR_OC_DELAY_ELAPSED_CB_ID        = 0x14U   /*!< TMR Output Compare Delay Elapsed Callback ID               */
  , DAL_TMR_PWM_PULSE_FINISHED_CB_ID      = 0x15U   /*!< TMR PWM Pulse Finished Callback ID           */
  , DAL_TMR_PWM_PULSE_FINISHED_HALF_CB_ID = 0x16U   /*!< TMR PWM Pulse Finished half complete Callback ID           */
  , DAL_TMR_ERROR_CB_ID                   = 0x17U   /*!< TMR Error Callback ID                                      */
  , DAL_TMR_COMMUTATION_CB_ID             = 0x18U   /*!< TMR Commutation Callback ID                                */
  , DAL_TMR_COMMUTATION_HALF_CB_ID        = 0x19U   /*!< TMR Commutation half complete Callback ID                  */
  , DAL_TMR_BREAK_CB_ID                   = 0x1AU   /*!< TMR Break Callback ID                                      */
} DAL_TMR_CallbackIDTypeDef;

/**
  * @brief  DAL TMR Callback pointer definition
  */
typedef  void (*pTMR_CallbackTypeDef)(TMR_HandleTypeDef *htmr);  /*!< pointer to the TMR callback function */

#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @}
  */
/* End of exported types -----------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup TMR_Exported_Constants TMR Exported Constants
  * @{
  */

/** @defgroup TMR_ClearInput_Source TMR Clear Input Source
  * @{
  */
#define TMR_CLEARINPUTSOURCE_NONE           0x00000000U   /*!< OCREF_CLR is disabled */
#define TMR_CLEARINPUTSOURCE_ETR            0x00000001U   /*!< OCREF_CLR is connected to ETRF input */
/**
  * @}
  */

/** @defgroup TMR_DMA_Base_address TMR DMA Base Address
  * @{
  */
#define TMR_DMABASE_CTRL1                   0x00000000U
#define TMR_DMABASE_CTRL2                   0x00000001U
#define TMR_DMABASE_SMCTRL                  0x00000002U
#define TMR_DMABASE_DIEN                    0x00000003U
#define TMR_DMABASE_STS                     0x00000004U
#define TMR_DMABASE_CEG                     0x00000005U
#define TMR_DMABASE_CCM1                    0x00000006U
#define TMR_DMABASE_CCM2                    0x00000007U
#define TMR_DMABASE_CCEN                    0x00000008U
#define TMR_DMABASE_CNT                     0x00000009U
#define TMR_DMABASE_PSC                     0x0000000AU
#define TMR_DMABASE_AUTORLD                 0x0000000BU
#define TMR_DMABASE_REPCNT                  0x0000000CU
#define TMR_DMABASE_CC1                     0x0000000DU
#define TMR_DMABASE_CC2                     0x0000000EU
#define TMR_DMABASE_CC3                     0x0000000FU
#define TMR_DMABASE_CC4                     0x00000010U
#define TMR_DMABASE_BDT                     0x00000011U
#define TMR_DMABASE_DCR                     0x00000012U
#define TMR_DMABASE_DMAR                    0x00000013U
/**
  * @}
  */

/** @defgroup TMR_Event_Source TMR Event Source
  * @{
  */
#define TMR_EVENTSOURCE_UPDATE              TMR_CEG_UEG     /*!< Reinitialize the counter and generates an update of the registers */
#define TMR_EVENTSOURCE_CC1                 TMR_CEG_CC1EG   /*!< A capture/compare event is generated on channel 1 */
#define TMR_EVENTSOURCE_CC2                 TMR_CEG_CC2EG   /*!< A capture/compare event is generated on channel 2 */
#define TMR_EVENTSOURCE_CC3                 TMR_CEG_CC3EG   /*!< A capture/compare event is generated on channel 3 */
#define TMR_EVENTSOURCE_CC4                 TMR_CEG_CC4EG   /*!< A capture/compare event is generated on channel 4 */
#define TMR_EVENTSOURCE_COM                 TMR_CEG_COMG   /*!< A commutation event is generated */
#define TMR_EVENTSOURCE_TRIGGER             TMR_CEG_TEG     /*!< A trigger event is generated */
#define TMR_EVENTSOURCE_BREAK               TMR_CEG_BEG     /*!< A break event is generated */
/**
  * @}
  */

/** @defgroup TMR_Input_Channel_Polarity TMR Input Channel polarity
  * @{
  */
#define  TMR_INPUTCHANNELPOLARITY_RISING      0x00000000U                       /*!< Polarity for TIx source */
#define  TMR_INPUTCHANNELPOLARITY_FALLING     TMR_CCEN_CC1POL                     /*!< Polarity for TIx source */
#define  TMR_INPUTCHANNELPOLARITY_BOTHEDGE    (TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL)  /*!< Polarity for TIx source */
/**
  * @}
  */

/** @defgroup TMR_ETR_Polarity TMR ETR Polarity
  * @{
  */
#define TMR_ETRPOLARITY_INVERTED              TMR_SMCTRL_ETPOL                      /*!< Polarity for ETR source */
#define TMR_ETRPOLARITY_NONINVERTED           0x00000000U                       /*!< Polarity for ETR source */
/**
  * @}
  */

/** @defgroup TMR_ETR_Prescaler TMR ETR Prescaler
  * @{
  */
#define TMR_ETRPRESCALER_DIV1                 0x00000000U                       /*!< No prescaler is used */
#define TMR_ETRPRESCALER_DIV2                 TMR_SMCTRL_ETPCFG_0                   /*!< ETR input source is divided by 2 */
#define TMR_ETRPRESCALER_DIV4                 TMR_SMCTRL_ETPCFG_1                   /*!< ETR input source is divided by 4 */
#define TMR_ETRPRESCALER_DIV8                 TMR_SMCTRL_ETPCFG                     /*!< ETR input source is divided by 8 */
/**
  * @}
  */

/** @defgroup TMR_Counter_Mode TMR Counter Mode
  * @{
  */
#define TMR_COUNTERMODE_UP                 0x00000000U                          /*!< Counter used as up-counter   */
#define TMR_COUNTERMODE_DOWN               TMR_CTRL1_CNTDIR                          /*!< Counter used as down-counter */
#define TMR_COUNTERMODE_CENTERALIGNED1     TMR_CTRL1_CAMSEL_0                        /*!< Center-aligned mode 1        */
#define TMR_COUNTERMODE_CENTERALIGNED2     TMR_CTRL1_CAMSEL_1                        /*!< Center-aligned mode 2        */
#define TMR_COUNTERMODE_CENTERALIGNED3     TMR_CTRL1_CAMSEL                          /*!< Center-aligned mode 3        */
/**
  * @}
  */

/** @defgroup TMR_ClockDivision TMR Clock Division
  * @{
  */
#define TMR_CLOCKDIVISION_DIV1             0x00000000U                          /*!< Clock division: tDTS=tCK_INT   */
#define TMR_CLOCKDIVISION_DIV2             TMR_CTRL1_CLKDIV_0                        /*!< Clock division: tDTS=2*tCK_INT */
#define TMR_CLOCKDIVISION_DIV4             TMR_CTRL1_CLKDIV_1                        /*!< Clock division: tDTS=4*tCK_INT */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_State TMR Output Compare State
  * @{
  */
#define TMR_OUTPUTSTATE_DISABLE            0x00000000U                          /*!< Capture/Compare 1 output disabled */
#define TMR_OUTPUTSTATE_ENABLE             TMR_CCEN_CC1EN                        /*!< Capture/Compare 1 output enabled */
/**
  * @}
  */

/** @defgroup TMR_AutoReloadPreload TMR Auto-Reload Preload
  * @{
  */
#define TMR_AUTORELOAD_PRELOAD_DISABLE                0x00000000U               /*!< TMRx_ARR register is not buffered */
#define TMR_AUTORELOAD_PRELOAD_ENABLE                 TMR_CTRL1_ARPEN              /*!< TMRx_ARR register is buffered */

/**
  * @}
  */

/** @defgroup TMR_Output_Fast_State TMR Output Fast State
  * @{
  */
#define TMR_OCFAST_DISABLE                 0x00000000U                          /*!< Output Compare fast disable */
#define TMR_OCFAST_ENABLE                  TMR_CCM1_OC1FEN                      /*!< Output Compare fast enable  */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_N_State TMR Complementary Output Compare State
  * @{
  */
#define TMR_OUTPUTNSTATE_DISABLE           0x00000000U                          /*!< OCxN is disabled  */
#define TMR_OUTPUTNSTATE_ENABLE            TMR_CCEN_CC1NEN                       /*!< OCxN is enabled   */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_Polarity TMR Output Compare Polarity
  * @{
  */
#define TMR_OCPOLARITY_HIGH                0x00000000U                          /*!< Capture/Compare output polarity  */
#define TMR_OCPOLARITY_LOW                 TMR_CCEN_CC1POL                        /*!< Capture/Compare output polarity  */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_N_Polarity TMR Complementary Output Compare Polarity
  * @{
  */
#define TMR_OCNPOLARITY_HIGH               0x00000000U                          /*!< Capture/Compare complementary output polarity */
#define TMR_OCNPOLARITY_LOW                TMR_CCEN_CC1NPOL                       /*!< Capture/Compare complementary output polarity */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_Idle_State TMR Output Compare Idle State
  * @{
  */
#define TMR_OCIDLESTATE_SET                TMR_CTRL2_OC1OIS                         /*!< Output Idle state: OCx=1 when MOE=0 */
#define TMR_OCIDLESTATE_RESET              0x00000000U                          /*!< Output Idle state: OCx=0 when MOE=0 */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_N_Idle_State TMR Complementary Output Compare Idle State
  * @{
  */
#define TMR_OCNIDLESTATE_SET               TMR_CTRL2_OC1NOIS                        /*!< Complementary output Idle state: OCxN=1 when MOE=0 */
#define TMR_OCNIDLESTATE_RESET             0x00000000U                          /*!< Complementary output Idle state: OCxN=0 when MOE=0 */
/**
  * @}
  */

/** @defgroup TMR_Input_Capture_Polarity TMR Input Capture Polarity
  * @{
  */
#define  TMR_ICPOLARITY_RISING             TMR_INPUTCHANNELPOLARITY_RISING      /*!< Capture triggered by rising edge on timer input                  */
#define  TMR_ICPOLARITY_FALLING            TMR_INPUTCHANNELPOLARITY_FALLING     /*!< Capture triggered by falling edge on timer input                 */
#define  TMR_ICPOLARITY_BOTHEDGE           TMR_INPUTCHANNELPOLARITY_BOTHEDGE    /*!< Capture triggered by both rising and falling edges on timer input*/
/**
  * @}
  */

/** @defgroup TMR_Encoder_Input_Polarity TMR Encoder Input Polarity
  * @{
  */
#define  TMR_ENCODERINPUTPOLARITY_RISING   TMR_INPUTCHANNELPOLARITY_RISING      /*!< Encoder input with rising edge polarity  */
#define  TMR_ENCODERINPUTPOLARITY_FALLING  TMR_INPUTCHANNELPOLARITY_FALLING     /*!< Encoder input with falling edge polarity */
/**
  * @}
  */

/** @defgroup TMR_Input_Capture_Selection TMR Input Capture Selection
  * @{
  */
#define TMR_ICSELECTION_DIRECTTI           TMR_CCM1_CC1SEL_0                     /*!< TMR Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively */
#define TMR_ICSELECTION_INDIRECTTI         TMR_CCM1_CC1SEL_1                     /*!< TMR Input 1, 2, 3 or 4 is selected to be connected to IC2, IC1, IC4 or IC3, respectively */
#define TMR_ICSELECTION_TRC                TMR_CCM1_CC1SEL                       /*!< TMR Input 1, 2, 3 or 4 is selected to be connected to TRC */
/**
  * @}
  */

/** @defgroup TMR_Input_Capture_Prescaler TMR Input Capture Prescaler
  * @{
  */
#define TMR_ICPSC_DIV1                     0x00000000U                          /*!< Capture performed each time an edge is detected on the capture input */
#define TMR_ICPSC_DIV2                     TMR_CCM1_IC1PSC_0                   /*!< Capture performed once every 2 events                                */
#define TMR_ICPSC_DIV4                     TMR_CCM1_IC1PSC_1                   /*!< Capture performed once every 4 events                                */
#define TMR_ICPSC_DIV8                     TMR_CCM1_IC1PSC                     /*!< Capture performed once every 8 events                                */
/**
  * @}
  */

/** @defgroup TMR_One_Pulse_Mode TMR One Pulse Mode
  * @{
  */
#define TMR_OPMODE_SINGLE                  TMR_CTRL1_SPMEN                          /*!< Counter stops counting at the next update event */
#define TMR_OPMODE_REPETITIVE              0x00000000U                          /*!< Counter is not stopped at update event          */
/**
  * @}
  */

/** @defgroup TMR_Encoder_Mode TMR Encoder Mode
  * @{
  */
#define TMR_ENCODERMODE_TI1                      TMR_SMCTRL_SMFSEL_0                                                      /*!< Quadrature encoder mode 1, x2 mode, counts up/down on TI1FP1 edge depending on TI2FP2 level  */
#define TMR_ENCODERMODE_TI2                      TMR_SMCTRL_SMFSEL_1                                                      /*!< Quadrature encoder mode 2, x2 mode, counts up/down on TI2FP2 edge depending on TI1FP1 level. */
#define TMR_ENCODERMODE_TI12                     (TMR_SMCTRL_SMFSEL_1 | TMR_SMCTRL_SMFSEL_0)                                   /*!< Quadrature encoder mode 3, x4 mode, counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input. */
/**
  * @}
  */

/** @defgroup TMR_Interrupt_definition TMR interrupt Definition
  * @{
  */
#define TMR_IT_UPDATE                      TMR_DIEN_UIEN                         /*!< Update interrupt            */
#define TMR_IT_CC1                         TMR_DIEN_CC1IEN                       /*!< Capture/Compare 1 interrupt */
#define TMR_IT_CC2                         TMR_DIEN_CC2IEN                       /*!< Capture/Compare 2 interrupt */
#define TMR_IT_CC3                         TMR_DIEN_CC3IEN                       /*!< Capture/Compare 3 interrupt */
#define TMR_IT_CC4                         TMR_DIEN_CC4IEN                       /*!< Capture/Compare 4 interrupt */
#define TMR_IT_COM                         TMR_DIEN_COMIEN                       /*!< Commutation interrupt       */
#define TMR_IT_TRIGGER                     TMR_DIEN_TRGIEN                         /*!< Trigger interrupt           */
#define TMR_IT_BREAK                       TMR_DIEN_BRKIEN                         /*!< Break interrupt             */
/**
  * @}
  */

/** @defgroup TMR_Commutation_Source  TMR Commutation Source
  * @{
  */
#define TMR_COMMUTATION_TRGI              TMR_CTRL2_CCUSEL                          /*!< When Capture/compare control bits are preloaded, they are updated by setting the COMG bit or when an rising edge occurs on trigger input */
#define TMR_COMMUTATION_SOFTWARE          0x00000000U                           /*!< When Capture/compare control bits are preloaded, they are updated by setting the COMG bit */
/**
  * @}
  */

/** @defgroup TMR_DMA_sources TMR DMA Sources
  * @{
  */
#define TMR_DMA_UPDATE                     TMR_DIEN_UDIEN                         /*!< DMA request is triggered by the update event */
#define TMR_DMA_CC1                        TMR_DIEN_CC1DEN                       /*!< DMA request is triggered by the capture/compare macth 1 event */
#define TMR_DMA_CC2                        TMR_DIEN_CC2DEN                       /*!< DMA request is triggered by the capture/compare macth 2 event event */
#define TMR_DMA_CC3                        TMR_DIEN_CC3DEN                       /*!< DMA request is triggered by the capture/compare macth 3 event event */
#define TMR_DMA_CC4                        TMR_DIEN_CC4DEN                       /*!< DMA request is triggered by the capture/compare macth 4 event event */
#define TMR_DMA_COM                        TMR_DIEN_COMDEN                       /*!< DMA request is triggered by the commutation event */
#define TMR_DMA_TRIGGER                    TMR_DIEN_TRGDEN                         /*!< DMA request is triggered by the trigger event */
/**
  * @}
  */

/** @defgroup TMR_CC_DMA_Request CCx DMA request selection
  * @{
  */
#define TMR_CCDMAREQUEST_CC                 0x00000000U                         /*!< CCx DMA request sent when capture or compare match event occurs */
#define TMR_CCDMAREQUEST_UPDATE             TMR_CTRL2_CCDSEL                        /*!< CCx DMA requests sent when update event occurs */
/**
  * @}
  */

/** @defgroup TMR_Flag_definition TMR Flag Definition
  * @{
  */
#define TMR_FLAG_UPDATE                    TMR_STS_UIFLG                           /*!< Update interrupt flag         */
#define TMR_FLAG_CC1                       TMR_STS_CC1IFLG                         /*!< Capture/Compare 1 interrupt flag */
#define TMR_FLAG_CC2                       TMR_STS_CC2IFLG                         /*!< Capture/Compare 2 interrupt flag */
#define TMR_FLAG_CC3                       TMR_STS_CC3IFLG                         /*!< Capture/Compare 3 interrupt flag */
#define TMR_FLAG_CC4                       TMR_STS_CC4IFLG                         /*!< Capture/Compare 4 interrupt flag */
#define TMR_FLAG_COM                       TMR_STS_COMIFLG                         /*!< Commutation interrupt flag    */
#define TMR_FLAG_TRIGGER                   TMR_STS_TRGIFLG                           /*!< Trigger interrupt flag        */
#define TMR_FLAG_BREAK                     TMR_STS_BRKIFLG                           /*!< Break interrupt flag          */
#define TMR_FLAG_CC1OF                     TMR_STS_CC1RCFLG                         /*!< Capture 1 overcapture flag    */
#define TMR_FLAG_CC2OF                     TMR_STS_CC2RCFLG                         /*!< Capture 2 overcapture flag    */
#define TMR_FLAG_CC3OF                     TMR_STS_CC3RCFLG                         /*!< Capture 3 overcapture flag    */
#define TMR_FLAG_CC4OF                     TMR_STS_CC4RCFLG                         /*!< Capture 4 overcapture flag    */
/**
  * @}
  */

/** @defgroup TMR_Channel TMR Channel
  * @{
  */
#define TMR_CHANNEL_1                      0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define TMR_CHANNEL_2                      0x00000004U                          /*!< Capture/compare channel 2 identifier      */
#define TMR_CHANNEL_3                      0x00000008U                          /*!< Capture/compare channel 3 identifier      */
#define TMR_CHANNEL_4                      0x0000000CU                          /*!< Capture/compare channel 4 identifier      */
#define TMR_CHANNEL_ALL                    0x0000003CU                          /*!< Global Capture/compare channel identifier  */
/**
  * @}
  */

/** @defgroup TMR_Clock_Source TMR Clock Source
  * @{
  */
#define TMR_CLOCKSOURCE_INTERNAL    TMR_SMCTRL_ETPCFG_0      /*!< Internal clock source                                 */
#define TMR_CLOCKSOURCE_ETRMODE1    TMR_TS_ETRF          /*!< External clock source mode 1 (ETRF)                   */
#define TMR_CLOCKSOURCE_ETRMODE2    TMR_SMCTRL_ETPCFG_1      /*!< External clock source mode 2                          */
#define TMR_CLOCKSOURCE_TI1ED       TMR_TS_TI1F_ED       /*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
#define TMR_CLOCKSOURCE_TI1         TMR_TS_TI1FP1        /*!< External clock source mode 1 (TTI1FP1)                */
#define TMR_CLOCKSOURCE_TI2         TMR_TS_TI2FP2        /*!< External clock source mode 1 (TTI2FP2)                */
#define TMR_CLOCKSOURCE_ITR0        TMR_TS_ITR0          /*!< External clock source mode 1 (ITR0)                   */
#define TMR_CLOCKSOURCE_ITR1        TMR_TS_ITR1          /*!< External clock source mode 1 (ITR1)                   */
#define TMR_CLOCKSOURCE_ITR2        TMR_TS_ITR2          /*!< External clock source mode 1 (ITR2)                   */
#define TMR_CLOCKSOURCE_ITR3        TMR_TS_ITR3          /*!< External clock source mode 1 (ITR3)                   */
/**
  * @}
  */

/** @defgroup TMR_Clock_Polarity TMR Clock Polarity
  * @{
  */
#define TMR_CLOCKPOLARITY_INVERTED           TMR_ETRPOLARITY_INVERTED           /*!< Polarity for ETRx clock sources */
#define TMR_CLOCKPOLARITY_NONINVERTED        TMR_ETRPOLARITY_NONINVERTED        /*!< Polarity for ETRx clock sources */
#define TMR_CLOCKPOLARITY_RISING             TMR_INPUTCHANNELPOLARITY_RISING    /*!< Polarity for TIx clock sources */
#define TMR_CLOCKPOLARITY_FALLING            TMR_INPUTCHANNELPOLARITY_FALLING   /*!< Polarity for TIx clock sources */
#define TMR_CLOCKPOLARITY_BOTHEDGE           TMR_INPUTCHANNELPOLARITY_BOTHEDGE  /*!< Polarity for TIx clock sources */
/**
  * @}
  */

/** @defgroup TMR_Clock_Prescaler TMR Clock Prescaler
  * @{
  */
#define TMR_CLOCKPRESCALER_DIV1                 TMR_ETRPRESCALER_DIV1           /*!< No prescaler is used                                                     */
#define TMR_CLOCKPRESCALER_DIV2                 TMR_ETRPRESCALER_DIV2           /*!< Prescaler for External ETR Clock: Capture performed once every 2 events. */
#define TMR_CLOCKPRESCALER_DIV4                 TMR_ETRPRESCALER_DIV4           /*!< Prescaler for External ETR Clock: Capture performed once every 4 events. */
#define TMR_CLOCKPRESCALER_DIV8                 TMR_ETRPRESCALER_DIV8           /*!< Prescaler for External ETR Clock: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup TMR_ClearInput_Polarity TMR Clear Input Polarity
  * @{
  */
#define TMR_CLEARINPUTPOLARITY_INVERTED           TMR_ETRPOLARITY_INVERTED      /*!< Polarity for ETRx pin */
#define TMR_CLEARINPUTPOLARITY_NONINVERTED        TMR_ETRPOLARITY_NONINVERTED   /*!< Polarity for ETRx pin */
/**
  * @}
  */

/** @defgroup TMR_ClearInput_Prescaler TMR Clear Input Prescaler
  * @{
  */
#define TMR_CLEARINPUTPRESCALER_DIV1              TMR_ETRPRESCALER_DIV1         /*!< No prescaler is used                                                   */
#define TMR_CLEARINPUTPRESCALER_DIV2              TMR_ETRPRESCALER_DIV2         /*!< Prescaler for External ETR pin: Capture performed once every 2 events. */
#define TMR_CLEARINPUTPRESCALER_DIV4              TMR_ETRPRESCALER_DIV4         /*!< Prescaler for External ETR pin: Capture performed once every 4 events. */
#define TMR_CLEARINPUTPRESCALER_DIV8              TMR_ETRPRESCALER_DIV8         /*!< Prescaler for External ETR pin: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup TMR_OSSR_Off_State_Selection_for_Run_mode_state TMR OSSR OffState Selection for Run mode state
  * @{
  */
#define TMR_OSSR_ENABLE                          TMR_BDT_RMOS                  /*!< When inactive, OC/OCN outputs are enabled (still controlled by the timer)           */
#define TMR_OSSR_DISABLE                         0x00000000U                    /*!< When inactive, OC/OCN outputs are disabled (not controlled any longer by the timer) */
/**
  * @}
  */

/** @defgroup TMR_OSSI_Off_State_Selection_for_Idle_mode_state TMR OSSI OffState Selection for Idle mode state
  * @{
  */
#define TMR_OSSI_ENABLE                          TMR_BDT_IMOS                  /*!< When inactive, OC/OCN outputs are enabled (still controlled by the timer)           */
#define TMR_OSSI_DISABLE                         0x00000000U                    /*!< When inactive, OC/OCN outputs are disabled (not controlled any longer by the timer) */
/**
  * @}
  */
/** @defgroup TMR_Lock_level  TMR Lock level
  * @{
  */
#define TMR_LOCKLEVEL_OFF                  0x00000000U                          /*!< LOCK OFF     */
#define TMR_LOCKLEVEL_1                    TMR_BDT_LOCKCFG_0                      /*!< LOCK Level 1 */
#define TMR_LOCKLEVEL_2                    TMR_BDT_LOCKCFG_1                      /*!< LOCK Level 2 */
#define TMR_LOCKLEVEL_3                    TMR_BDT_LOCKCFG                        /*!< LOCK Level 3 */
/**
  * @}
  */

/** @defgroup TMR_Break_Input_enable_disable TMR Break Input Enable
  * @{
  */
#define TMR_BREAK_ENABLE                   TMR_BDT_BRKEN                         /*!< Break input BRK is enabled  */
#define TMR_BREAK_DISABLE                  0x00000000U                          /*!< Break input BRK is disabled */
/**
  * @}
  */

/** @defgroup TMR_Break_Polarity TMR Break Input Polarity
  * @{
  */
#define TMR_BREAKPOLARITY_LOW              0x00000000U                          /*!< Break input BRK is active low  */
#define TMR_BREAKPOLARITY_HIGH             TMR_BDT_BRKPOL                         /*!< Break input BRK is active high */
/**
  * @}
  */

/** @defgroup TMR_AOE_Bit_Set_Reset TMR Automatic Output Enable
  * @{
  */
#define TMR_AUTOMATICOUTPUT_DISABLE        0x00000000U                          /*!< MOE can be set only by software */
#define TMR_AUTOMATICOUTPUT_ENABLE         TMR_BDT_AOEN                         /*!< MOE can be set by software or automatically at the next update event (if none of the break inputs BRK and BRK2 is active) */
/**
  * @}
  */

/** @defgroup TMR_Master_Mode_Selection TMR Master Mode Selection
  * @{
  */
#define TMR_TRGO_RESET            0x00000000U                                      /*!< TMRx_EGR.UG bit is used as trigger output (TRGO)              */
#define TMR_TRGO_ENABLE           TMR_CTRL2_MMSEL_0                                    /*!< TMRx_CR1.CEN bit is used as trigger output (TRGO)             */
#define TMR_TRGO_UPDATE           TMR_CTRL2_MMSEL_1                                    /*!< Update event is used as trigger output (TRGO)                 */
#define TMR_TRGO_OC1              (TMR_CTRL2_MMSEL_1 | TMR_CTRL2_MMSEL_0)                  /*!< Capture or a compare match 1 is used as trigger output (TRGO) */
#define TMR_TRGO_OC1REF           TMR_CTRL2_MMSEL_2                                    /*!< OC1REF signal is used as trigger output (TRGO)                */
#define TMR_TRGO_OC2REF           (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_0)                  /*!< OC2REF signal is used as trigger output(TRGO)                 */
#define TMR_TRGO_OC3REF           (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_1)                  /*!< OC3REF signal is used as trigger output(TRGO)                 */
#define TMR_TRGO_OC4REF           (TMR_CTRL2_MMSEL_2 | TMR_CTRL2_MMSEL_1 | TMR_CTRL2_MMSEL_0)  /*!< OC4REF signal is used as trigger output(TRGO)                 */
/**
  * @}
  */

/** @defgroup TMR_Master_Slave_Mode TMR Master/Slave Mode
  * @{
  */
#define TMR_MASTERSLAVEMODE_ENABLE         TMR_SMCTRL_MSMEN                         /*!< No action */
#define TMR_MASTERSLAVEMODE_DISABLE        0x00000000U                          /*!< Master/slave mode is selected */
/**
  * @}
  */

/** @defgroup TMR_Slave_Mode TMR Slave mode
  * @{
  */
#define TMR_SLAVEMODE_DISABLE                0x00000000U                                        /*!< Slave mode disabled           */
#define TMR_SLAVEMODE_RESET                  TMR_SMCTRL_SMFSEL_2                                     /*!< Reset Mode                    */
#define TMR_SLAVEMODE_GATED                  (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_0)                  /*!< Gated Mode                    */
#define TMR_SLAVEMODE_TRIGGER                (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_1)                  /*!< Trigger Mode                  */
#define TMR_SLAVEMODE_EXTERNAL1              (TMR_SMCTRL_SMFSEL_2 | TMR_SMCTRL_SMFSEL_1 | TMR_SMCTRL_SMFSEL_0) /*!< External Clock Mode 1         */
/**
  * @}
  */

/** @defgroup TMR_Output_Compare_and_PWM_modes TMR Output Compare and PWM Modes
  * @{
  */
#define TMR_OCMODE_TIMING                   0x00000000U                                              /*!< Frozen                                 */
#define TMR_OCMODE_ACTIVE                   TMR_CCM1_OC1MOD_0                                         /*!< Set channel to active level on match   */
#define TMR_OCMODE_INACTIVE                 TMR_CCM1_OC1MOD_1                                         /*!< Set channel to inactive level on match */
#define TMR_OCMODE_TOGGLE                   (TMR_CCM1_OC1MOD_1 | TMR_CCM1_OC1MOD_0)                    /*!< Toggle                                 */
#define TMR_OCMODE_PWM1                     (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_1)                    /*!< PWM mode 1                             */
#define TMR_OCMODE_PWM2                     (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_1 | TMR_CCM1_OC1MOD_0) /*!< PWM mode 2                             */
#define TMR_OCMODE_FORCED_ACTIVE            (TMR_CCM1_OC1MOD_2 | TMR_CCM1_OC1MOD_0)                    /*!< Force active level                     */
#define TMR_OCMODE_FORCED_INACTIVE          TMR_CCM1_OC1MOD_2                                         /*!< Force inactive level                   */
/**
  * @}
  */

/** @defgroup TMR_Trigger_Selection TMR Trigger Selection
  * @{
  */
#define TMR_TS_ITR0          0x00000000U                                                       /*!< Internal Trigger 0 (ITR0)              */
#define TMR_TS_ITR1          TMR_SMCTRL_TRGSEL_0                                                     /*!< Internal Trigger 1 (ITR1)              */
#define TMR_TS_ITR2          TMR_SMCTRL_TRGSEL_1                                                     /*!< Internal Trigger 2 (ITR2)              */
#define TMR_TS_ITR3          (TMR_SMCTRL_TRGSEL_0 | TMR_SMCTRL_TRGSEL_1)                                   /*!< Internal Trigger 3 (ITR3)              */
#define TMR_TS_TI1F_ED       TMR_SMCTRL_TRGSEL_2                                                     /*!< TI1 Edge Detector (TI1F_ED)            */
#define TMR_TS_TI1FP1        (TMR_SMCTRL_TRGSEL_0 | TMR_SMCTRL_TRGSEL_2)                                   /*!< Filtered Timer Input 1 (TI1FP1)        */
#define TMR_TS_TI2FP2        (TMR_SMCTRL_TRGSEL_1 | TMR_SMCTRL_TRGSEL_2)                                   /*!< Filtered Timer Input 2 (TI2FP2)        */
#define TMR_TS_ETRF          (TMR_SMCTRL_TRGSEL_0 | TMR_SMCTRL_TRGSEL_1 | TMR_SMCTRL_TRGSEL_2)                   /*!< Filtered External Trigger input (ETRF) */
#define TMR_TS_NONE          0x0000FFFFU                                                       /*!< No trigger selected                    */
/**
  * @}
  */

/** @defgroup TMR_Trigger_Polarity TMR Trigger Polarity
  * @{
  */
#define TMR_TRIGGERPOLARITY_INVERTED           TMR_ETRPOLARITY_INVERTED               /*!< Polarity for ETRx trigger sources             */
#define TMR_TRIGGERPOLARITY_NONINVERTED        TMR_ETRPOLARITY_NONINVERTED            /*!< Polarity for ETRx trigger sources             */
#define TMR_TRIGGERPOLARITY_RISING             TMR_INPUTCHANNELPOLARITY_RISING        /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define TMR_TRIGGERPOLARITY_FALLING            TMR_INPUTCHANNELPOLARITY_FALLING       /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define TMR_TRIGGERPOLARITY_BOTHEDGE           TMR_INPUTCHANNELPOLARITY_BOTHEDGE      /*!< Polarity for TIxFPx or TI1_ED trigger sources */
/**
  * @}
  */

/** @defgroup TMR_Trigger_Prescaler TMR Trigger Prescaler
  * @{
  */
#define TMR_TRIGGERPRESCALER_DIV1             TMR_ETRPRESCALER_DIV1             /*!< No prescaler is used                                                       */
#define TMR_TRIGGERPRESCALER_DIV2             TMR_ETRPRESCALER_DIV2             /*!< Prescaler for External ETR Trigger: Capture performed once every 2 events. */
#define TMR_TRIGGERPRESCALER_DIV4             TMR_ETRPRESCALER_DIV4             /*!< Prescaler for External ETR Trigger: Capture performed once every 4 events. */
#define TMR_TRIGGERPRESCALER_DIV8             TMR_ETRPRESCALER_DIV8             /*!< Prescaler for External ETR Trigger: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup TMR_TI1_Selection TMR TI1 Input Selection
  * @{
  */
#define TMR_TI1SELECTION_CH1               0x00000000U                          /*!< The TMRx_CH1 pin is connected to TI1 input */
#define TMR_TI1SELECTION_XORCOMBINATION    TMR_CTRL2_TI1SEL                         /*!< The TMRx_CH1, CH2 and CH3 pins are connected to the TI1 input (XOR combination) */
/**
  * @}
  */

/** @defgroup TMR_DMA_Burst_Length TMR DMA Burst Length
  * @{
  */
#define TMR_DMABURSTLENGTH_1TRANSFER       0x00000000U                          /*!< The transfer is done to 1 register starting from TMRx_CR1 + TMRx_DCR.DBA   */
#define TMR_DMABURSTLENGTH_2TRANSFERS      0x00000100U                          /*!< The transfer is done to 2 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_3TRANSFERS      0x00000200U                          /*!< The transfer is done to 3 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_4TRANSFERS      0x00000300U                          /*!< The transfer is done to 4 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_5TRANSFERS      0x00000400U                          /*!< The transfer is done to 5 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_6TRANSFERS      0x00000500U                          /*!< The transfer is done to 6 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_7TRANSFERS      0x00000600U                          /*!< The transfer is done to 7 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_8TRANSFERS      0x00000700U                          /*!< The transfer is done to 8 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_9TRANSFERS      0x00000800U                          /*!< The transfer is done to 9 registers starting from TMRx_CR1 + TMRx_DCR.DBA  */
#define TMR_DMABURSTLENGTH_10TRANSFERS     0x00000900U                          /*!< The transfer is done to 10 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_11TRANSFERS     0x00000A00U                          /*!< The transfer is done to 11 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_12TRANSFERS     0x00000B00U                          /*!< The transfer is done to 12 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_13TRANSFERS     0x00000C00U                          /*!< The transfer is done to 13 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_14TRANSFERS     0x00000D00U                          /*!< The transfer is done to 14 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_15TRANSFERS     0x00000E00U                          /*!< The transfer is done to 15 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_16TRANSFERS     0x00000F00U                          /*!< The transfer is done to 16 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_17TRANSFERS     0x00001000U                          /*!< The transfer is done to 17 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
#define TMR_DMABURSTLENGTH_18TRANSFERS     0x00001100U                          /*!< The transfer is done to 18 registers starting from TMRx_CR1 + TMRx_DCR.DBA */
/**
  * @}
  */

/** @defgroup DMA_Handle_index TMR DMA Handle Index
  * @{
  */
#define TMR_DMA_ID_UPDATE                ((uint16_t) 0x0000)       /*!< Index of the DMA handle used for Update DMA requests */
#define TMR_DMA_ID_CC1                   ((uint16_t) 0x0001)       /*!< Index of the DMA handle used for Capture/Compare 1 DMA requests */
#define TMR_DMA_ID_CC2                   ((uint16_t) 0x0002)       /*!< Index of the DMA handle used for Capture/Compare 2 DMA requests */
#define TMR_DMA_ID_CC3                   ((uint16_t) 0x0003)       /*!< Index of the DMA handle used for Capture/Compare 3 DMA requests */
#define TMR_DMA_ID_CC4                   ((uint16_t) 0x0004)       /*!< Index of the DMA handle used for Capture/Compare 4 DMA requests */
#define TMR_DMA_ID_COMMUTATION           ((uint16_t) 0x0005)       /*!< Index of the DMA handle used for Commutation DMA requests */
#define TMR_DMA_ID_TRIGGER               ((uint16_t) 0x0006)       /*!< Index of the DMA handle used for Trigger DMA requests */
/**
  * @}
  */

/** @defgroup Channel_CC_State TMR Capture/Compare Channel State
  * @{
  */
#define TMR_CCx_ENABLE                   0x00000001U                            /*!< Input or output channel is enabled */
#define TMR_CCx_DISABLE                  0x00000000U                            /*!< Input or output channel is disabled */
#define TMR_CCxN_ENABLE                  0x00000004U                            /*!< Complementary output channel is enabled */
#define TMR_CCxN_DISABLE                 0x00000000U                            /*!< Complementary output channel is enabled */
/**
  * @}
  */

/**
  * @}
  */
/* End of exported constants -------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/** @defgroup TMR_Exported_Macros TMR Exported Macros
  * @{
  */

/** @brief  Reset TMR handle state.
  * @param  __HANDLE__ TMR handle.
  * @retval None
  */
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
#define __DAL_TMR_RESET_HANDLE_STATE(__HANDLE__) do {                                                               \
                                                      (__HANDLE__)->State            = DAL_TMR_STATE_RESET;         \
                                                      (__HANDLE__)->ChannelState[0]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[1]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[2]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[3]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[0] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[1] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[2] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[3] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->DMABurstState    = DAL_DMA_BURST_STATE_RESET;   \
                                                      (__HANDLE__)->Base_MspInitCallback         = NULL;            \
                                                      (__HANDLE__)->Base_MspDeInitCallback       = NULL;            \
                                                      (__HANDLE__)->IC_MspInitCallback           = NULL;            \
                                                      (__HANDLE__)->IC_MspDeInitCallback         = NULL;            \
                                                      (__HANDLE__)->OC_MspInitCallback           = NULL;            \
                                                      (__HANDLE__)->OC_MspDeInitCallback         = NULL;            \
                                                      (__HANDLE__)->PWM_MspInitCallback          = NULL;            \
                                                      (__HANDLE__)->PWM_MspDeInitCallback        = NULL;            \
                                                      (__HANDLE__)->OnePulse_MspInitCallback     = NULL;            \
                                                      (__HANDLE__)->OnePulse_MspDeInitCallback   = NULL;            \
                                                      (__HANDLE__)->Encoder_MspInitCallback      = NULL;            \
                                                      (__HANDLE__)->Encoder_MspDeInitCallback    = NULL;            \
                                                      (__HANDLE__)->HallSensor_MspInitCallback   = NULL;            \
                                                      (__HANDLE__)->HallSensor_MspDeInitCallback = NULL;            \
                                                     } while(0)
#else
#define __DAL_TMR_RESET_HANDLE_STATE(__HANDLE__) do {                                                               \
                                                      (__HANDLE__)->State            = DAL_TMR_STATE_RESET;         \
                                                      (__HANDLE__)->ChannelState[0]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[1]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[2]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelState[3]  = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[0] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[1] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[2] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->ChannelNState[3] = DAL_TMR_CHANNEL_STATE_RESET; \
                                                      (__HANDLE__)->DMABurstState    = DAL_DMA_BURST_STATE_RESET;   \
                                                     } while(0)
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @brief  Enable the TMR peripheral.
  * @param  __HANDLE__ TMR handle
  * @retval None
  */
#define __DAL_TMR_ENABLE(__HANDLE__)                 ((__HANDLE__)->Instance->CTRL1|=(TMR_CTRL1_CNTEN))

/**
  * @brief  Enable the TMR main Output.
  * @param  __HANDLE__ TMR handle
  * @retval None
  */
#define __DAL_TMR_MOE_ENABLE(__HANDLE__)             ((__HANDLE__)->Instance->BDT|=(TMR_BDT_MOEN))

/**
  * @brief  Disable the TMR peripheral.
  * @param  __HANDLE__ TMR handle
  * @retval None
  */
#define __DAL_TMR_DISABLE(__HANDLE__) \
  do { \
    if (((__HANDLE__)->Instance->CCEN & TMR_CCEN_CCxE_MASK) == 0UL) \
    { \
      if(((__HANDLE__)->Instance->CCEN & TMR_CCEN_CCxNE_MASK) == 0UL) \
      { \
        (__HANDLE__)->Instance->CTRL1 &= ~(TMR_CTRL1_CNTEN); \
      } \
    } \
  } while(0)

/**
  * @brief  Disable the TMR main Output.
  * @param  __HANDLE__ TMR handle
  * @retval None
  * @note The Main Output Enable of a timer instance is disabled only if all the CCx and CCxN channels have been
  *       disabled
  */
#define __DAL_TMR_MOE_DISABLE(__HANDLE__) \
  do { \
    if (((__HANDLE__)->Instance->CCEN & TMR_CCEN_CCxE_MASK) == 0UL) \
    { \
      if(((__HANDLE__)->Instance->CCEN & TMR_CCEN_CCxNE_MASK) == 0UL) \
      { \
        (__HANDLE__)->Instance->BDT &= ~(TMR_BDT_MOEN); \
      } \
    } \
  } while(0)

/**
  * @brief  Disable the TMR main Output.
  * @param  __HANDLE__ TMR handle
  * @retval None
  * @note The Main Output Enable of a timer instance is disabled unconditionally
  */
#define __DAL_TMR_MOE_DISABLE_UNCONDITIONALLY(__HANDLE__)  (__HANDLE__)->Instance->BDT &= ~(TMR_BDT_MOEN)

/** @brief  Enable the specified TMR interrupt.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __INTERRUPT__ specifies the TMR interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg TMR_IT_UPDATE: Update interrupt
  *            @arg TMR_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg TMR_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg TMR_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg TMR_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg TMR_IT_COM:   Commutation interrupt
  *            @arg TMR_IT_TRIGGER: Trigger interrupt
  *            @arg TMR_IT_BREAK: Break interrupt
  * @retval None
  */
#define __DAL_TMR_ENABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->DIEN |= (__INTERRUPT__))

/** @brief  Disable the specified TMR interrupt.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __INTERRUPT__ specifies the TMR interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg TMR_IT_UPDATE: Update interrupt
  *            @arg TMR_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg TMR_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg TMR_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg TMR_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg TMR_IT_COM:   Commutation interrupt
  *            @arg TMR_IT_TRIGGER: Trigger interrupt
  *            @arg TMR_IT_BREAK: Break interrupt
  * @retval None
  */
#define __DAL_TMR_DISABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->DIEN &= ~(__INTERRUPT__))

/** @brief  Enable the specified DMA request.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __DMA__ specifies the TMR DMA request to enable.
  *          This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: Update DMA request
  *            @arg TMR_DMA_CC1:   Capture/Compare 1 DMA request
  *            @arg TMR_DMA_CC2:  Capture/Compare 2 DMA request
  *            @arg TMR_DMA_CC3:  Capture/Compare 3 DMA request
  *            @arg TMR_DMA_CC4:  Capture/Compare 4 DMA request
  *            @arg TMR_DMA_COM:   Commutation DMA request
  *            @arg TMR_DMA_TRIGGER: Trigger DMA request
  * @retval None
  */
#define __DAL_TMR_ENABLE_DMA(__HANDLE__, __DMA__)         ((__HANDLE__)->Instance->DIEN |= (__DMA__))

/** @brief  Disable the specified DMA request.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __DMA__ specifies the TMR DMA request to disable.
  *          This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: Update DMA request
  *            @arg TMR_DMA_CC1:   Capture/Compare 1 DMA request
  *            @arg TMR_DMA_CC2:  Capture/Compare 2 DMA request
  *            @arg TMR_DMA_CC3:  Capture/Compare 3 DMA request
  *            @arg TMR_DMA_CC4:  Capture/Compare 4 DMA request
  *            @arg TMR_DMA_COM:   Commutation DMA request
  *            @arg TMR_DMA_TRIGGER: Trigger DMA request
  * @retval None
  */
#define __DAL_TMR_DISABLE_DMA(__HANDLE__, __DMA__)        ((__HANDLE__)->Instance->DIEN &= ~(__DMA__))

/** @brief  Check whether the specified TMR interrupt flag is set or not.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __FLAG__ specifies the TMR interrupt flag to check.
  *        This parameter can be one of the following values:
  *            @arg TMR_FLAG_UPDATE: Update interrupt flag
  *            @arg TMR_FLAG_CC1: Capture/Compare 1 interrupt flag
  *            @arg TMR_FLAG_CC2: Capture/Compare 2 interrupt flag
  *            @arg TMR_FLAG_CC3: Capture/Compare 3 interrupt flag
  *            @arg TMR_FLAG_CC4: Capture/Compare 4 interrupt flag
  *            @arg TMR_FLAG_COM:  Commutation interrupt flag
  *            @arg TMR_FLAG_TRIGGER: Trigger interrupt flag
  *            @arg TMR_FLAG_BREAK: Break interrupt flag
  *            @arg TMR_FLAG_CC1OF: Capture/Compare 1 overcapture flag
  *            @arg TMR_FLAG_CC2OF: Capture/Compare 2 overcapture flag
  *            @arg TMR_FLAG_CC3OF: Capture/Compare 3 overcapture flag
  *            @arg TMR_FLAG_CC4OF: Capture/Compare 4 overcapture flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_TMR_GET_FLAG(__HANDLE__, __FLAG__)          (((__HANDLE__)->Instance->STS &(__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified TMR interrupt flag.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __FLAG__ specifies the TMR interrupt flag to clear.
  *        This parameter can be one of the following values:
  *            @arg TMR_FLAG_UPDATE: Update interrupt flag
  *            @arg TMR_FLAG_CC1: Capture/Compare 1 interrupt flag
  *            @arg TMR_FLAG_CC2: Capture/Compare 2 interrupt flag
  *            @arg TMR_FLAG_CC3: Capture/Compare 3 interrupt flag
  *            @arg TMR_FLAG_CC4: Capture/Compare 4 interrupt flag
  *            @arg TMR_FLAG_COM:  Commutation interrupt flag
  *            @arg TMR_FLAG_TRIGGER: Trigger interrupt flag
  *            @arg TMR_FLAG_BREAK: Break interrupt flag
  *            @arg TMR_FLAG_CC1OF: Capture/Compare 1 overcapture flag
  *            @arg TMR_FLAG_CC2OF: Capture/Compare 2 overcapture flag
  *            @arg TMR_FLAG_CC3OF: Capture/Compare 3 overcapture flag
  *            @arg TMR_FLAG_CC4OF: Capture/Compare 4 overcapture flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_TMR_CLEAR_FLAG(__HANDLE__, __FLAG__)        ((__HANDLE__)->Instance->STS = ~(__FLAG__))

/**
  * @brief  Check whether the specified TMR interrupt source is enabled or not.
  * @param  __HANDLE__ TMR handle
  * @param  __INTERRUPT__ specifies the TMR interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg TMR_IT_UPDATE: Update interrupt
  *            @arg TMR_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg TMR_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg TMR_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg TMR_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg TMR_IT_COM:   Commutation interrupt
  *            @arg TMR_IT_TRIGGER: Trigger interrupt
  *            @arg TMR_IT_BREAK: Break interrupt
  * @retval The state of TMR_IT (SET or RESET).
  */
#define __DAL_TMR_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->DIEN & (__INTERRUPT__)) \
                                                             == (__INTERRUPT__)) ? SET : RESET)

/** @brief Clear the TMR interrupt pending bits.
  * @param  __HANDLE__ TMR handle
  * @param  __INTERRUPT__ specifies the interrupt pending bit to clear.
  *          This parameter can be one of the following values:
  *            @arg TMR_IT_UPDATE: Update interrupt
  *            @arg TMR_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg TMR_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg TMR_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg TMR_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg TMR_IT_COM:   Commutation interrupt
  *            @arg TMR_IT_TRIGGER: Trigger interrupt
  *            @arg TMR_IT_BREAK: Break interrupt
  * @retval None
  */
#define __DAL_TMR_CLEAR_IT(__HANDLE__, __INTERRUPT__)      ((__HANDLE__)->Instance->STS = ~(__INTERRUPT__))

/**
  * @brief  Indicates whether or not the TMR Counter is used as downcounter.
  * @param  __HANDLE__ TMR handle.
  * @retval False (Counter used as upcounter) or True (Counter used as downcounter)
  * @note This macro is particularly useful to get the counting mode when the timer operates in Center-aligned mode
  *       or Encoder mode.
  */
#define __DAL_TMR_IS_TMR_COUNTING_DOWN(__HANDLE__)    (((__HANDLE__)->Instance->CTRL1 &(TMR_CTRL1_CNTDIR)) == (TMR_CTRL1_CNTDIR))

/**
  * @brief  Set the TMR Prescaler on runtime.
  * @param  __HANDLE__ TMR handle.
  * @param  __PRESC__ specifies the Prescaler new value.
  * @retval None
  */
#define __DAL_TMR_SET_PRESCALER(__HANDLE__, __PRESC__)       ((__HANDLE__)->Instance->PSC = (__PRESC__))

/**
  * @brief  Set the TMR Counter Register value on runtime.
  * @param  __HANDLE__ TMR handle.
  * @param  __COUNTER__ specifies the Counter register new value.
  * @retval None
  */
#define __DAL_TMR_SET_COUNTER(__HANDLE__, __COUNTER__)  ((__HANDLE__)->Instance->CNT = (__COUNTER__))

/**
  * @brief  Get the TMR Counter Register value on runtime.
  * @param  __HANDLE__ TMR handle.
  * @retval 16-bit or 32-bit value of the timer counter register (TMRx_CNT)
  */
#define __DAL_TMR_GET_COUNTER(__HANDLE__)  ((__HANDLE__)->Instance->CNT)

/**
  * @brief  Set the TMR Autoreload Register value on runtime without calling another time any Init function.
  * @param  __HANDLE__ TMR handle.
  * @param  __AUTORELOAD__ specifies the Counter register new value.
  * @retval None
  */
#define __DAL_TMR_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) \
  do{                                                    \
    (__HANDLE__)->Instance->AUTORLD = (__AUTORELOAD__);  \
    (__HANDLE__)->Init.Period = (__AUTORELOAD__);    \
  } while(0)

/**
  * @brief  Get the TMR Autoreload Register value on runtime.
  * @param  __HANDLE__ TMR handle.
  * @retval 16-bit or 32-bit value of the timer auto-reload register(TMRx_AUTORLD)
  */
#define __DAL_TMR_GET_AUTORELOAD(__HANDLE__)  ((__HANDLE__)->Instance->AUTORLD)

/**
  * @brief  Set the TMR Clock Division value on runtime without calling another time any Init function.
  * @param  __HANDLE__ TMR handle.
  * @param  __CKD__ specifies the clock division value.
  *          This parameter can be one of the following value:
  *            @arg TMR_CLOCKDIVISION_DIV1: tDTS=tCK_INT
  *            @arg TMR_CLOCKDIVISION_DIV2: tDTS=2*tCK_INT
  *            @arg TMR_CLOCKDIVISION_DIV4: tDTS=4*tCK_INT
  * @retval None
  */
#define __DAL_TMR_SET_CLOCKDIVISION(__HANDLE__, __CKD__) \
  do{                                                   \
    (__HANDLE__)->Instance->CTRL1 &= (~TMR_CTRL1_CLKDIV);  \
    (__HANDLE__)->Instance->CTRL1 |= (__CKD__);       \
    (__HANDLE__)->Init.ClockDivision = (__CKD__);   \
  } while(0)

/**
  * @brief  Get the TMR Clock Division value on runtime.
  * @param  __HANDLE__ TMR handle.
  * @retval The clock division can be one of the following values:
  *            @arg TMR_CLOCKDIVISION_DIV1: tDTS=tCK_INT
  *            @arg TMR_CLOCKDIVISION_DIV2: tDTS=2*tCK_INT
  *            @arg TMR_CLOCKDIVISION_DIV4: tDTS=4*tCK_INT
  */
#define __DAL_TMR_GET_CLOCKDIVISION(__HANDLE__)  ((__HANDLE__)->Instance->CTRL1 & TMR_CTRL1_CLKDIV)

/**
  * @brief  Set the TMR Input Capture prescaler on runtime without calling another time DAL_TMR_IC_ConfigChannel()
  *         function.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  __ICPSC__ specifies the Input Capture4 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPSC_DIV1: no prescaler
  *            @arg TMR_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TMR_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TMR_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#define __DAL_TMR_SET_ICPRESCALER(__HANDLE__, __CHANNEL__, __ICPSC__) \
  do{                                                    \
    TMR_RESET_ICPRESCALERVALUE((__HANDLE__), (__CHANNEL__));  \
    TMR_SET_ICPRESCALERVALUE((__HANDLE__), (__CHANNEL__), (__ICPSC__)); \
  } while(0)

/**
  * @brief  Get the TMR Input Capture prescaler on runtime.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: get input capture 1 prescaler value
  *            @arg TMR_CHANNEL_2: get input capture 2 prescaler value
  *            @arg TMR_CHANNEL_3: get input capture 3 prescaler value
  *            @arg TMR_CHANNEL_4: get input capture 4 prescaler value
  * @retval The input capture prescaler can be one of the following values:
  *            @arg TMR_ICPSC_DIV1: no prescaler
  *            @arg TMR_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TMR_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TMR_ICPSC_DIV8: capture is done once every 8 events
  */
#define __DAL_TMR_GET_ICPRESCALER(__HANDLE__, __CHANNEL__)  \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 & TMR_CCM1_IC1PSC) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? (((__HANDLE__)->Instance->CCM1 & TMR_CCM1_IC2PSC) >> 8U) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 & TMR_CCM2_IC3PSC) :\
   (((__HANDLE__)->Instance->CCM2 & TMR_CCM2_IC4PSC)) >> 8U)

/**
  * @brief  Set the TMR Capture Compare Register value on runtime without calling another time ConfigChannel function.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  __COMPARE__ specifies the Capture Compare register new value.
  * @retval None
  */
#define __DAL_TMR_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CC1 = (__COMPARE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CC2 = (__COMPARE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CC3 = (__COMPARE__)) :\
   ((__HANDLE__)->Instance->CC4 = (__COMPARE__)))

/**
  * @brief  Get the TMR Capture Compare Register value on runtime.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channel associated with the capture compare register
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: get capture/compare 1 register value
  *            @arg TMR_CHANNEL_2: get capture/compare 2 register value
  *            @arg TMR_CHANNEL_3: get capture/compare 3 register value
  *            @arg TMR_CHANNEL_4: get capture/compare 4 register value
  * @retval 16-bit or 32-bit value of the capture/compare register (TMRx_CCRy)
  */
#define __DAL_TMR_GET_COMPARE(__HANDLE__, __CHANNEL__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CC1) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CC2) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CC3) :\
   ((__HANDLE__)->Instance->CC4))

/**
  * @brief  Set the TMR Output compare preload.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval None
  */
#define __DAL_TMR_ENABLE_OCxPRELOAD(__HANDLE__, __CHANNEL__)    \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 |= TMR_CCM1_OC1PEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 |= TMR_CCM1_OC2PEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 |= TMR_CCM2_OC3PEN) :\
   ((__HANDLE__)->Instance->CCM2 |= TMR_CCM2_OC4PEN))

/**
  * @brief  Reset the TMR Output compare preload.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval None
  */
#define __DAL_TMR_DISABLE_OCxPRELOAD(__HANDLE__, __CHANNEL__)    \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_OC1PEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_OC2PEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_OC3PEN) :\
   ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_OC4PEN))

/**
  * @brief  Enable fast mode for a given channel.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @note  When fast mode is enabled an active edge on the trigger input acts
  *        like a compare match on CCx output. Delay to sample the trigger
  *        input and to activate CCx output is reduced to 3 clock cycles.
  * @note  Fast mode acts only if the channel is configured in PWM1 or PWM2 mode.
  * @retval None
  */
#define __DAL_TMR_ENABLE_OCxFAST(__HANDLE__, __CHANNEL__)    \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 |= TMR_CCM1_OC1FEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 |= TMR_CCM1_OC2FEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 |= TMR_CCM2_OC3FEN) :\
   ((__HANDLE__)->Instance->CCM2 |= TMR_CCM2_OC4FEN))

/**
  * @brief  Disable fast mode for a given channel.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @note  When fast mode is disabled CCx output behaves normally depending
  *        on counter and CCx values even when the trigger is ON. The minimum
  *        delay to activate CCx output when an active edge occurs on the
  *        trigger input is 5 clock cycles.
  * @retval None
  */
#define __DAL_TMR_DISABLE_OCxFAST(__HANDLE__, __CHANNEL__)    \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_OC1FEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_OC2FEN) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_OC3FEN) :\
   ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_OC4FEN))

/**
  * @brief  Set the Update Request Source (URS) bit of the TMRx_CTRL1 register.
  * @param  __HANDLE__ TMR handle.
  * @note  When the URS bit of the TMRx_CTRL1 register is set, only counter
  *        overflow/underflow generates an update interrupt or DMA request (if
  *        enabled)
  * @retval None
  */
#define __DAL_TMR_URS_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL1|= TMR_CTRL1_URSSEL)

/**
  * @brief  Reset the Update Request Source (URS) bit of the TMRx_CTRL1 register.
  * @param  __HANDLE__ TMR handle.
  * @note  When the URS bit of the TMRx_CTRL1 register is reset, any of the
  *        following events generate an update interrupt or DMA request (if
  *        enabled):
  *           _ Counter overflow underflow
  *           _ Setting the UG bit
  *           _ Update generation through the slave mode controller
  * @retval None
  */
#define __DAL_TMR_URS_DISABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL1&=~TMR_CTRL1_URSSEL)

/**
  * @brief  Set the TMR Capture x input polarity on runtime.
  * @param  __HANDLE__ TMR handle.
  * @param  __CHANNEL__ TMR Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  __POLARITY__ Polarity for TIx source
  *            @arg TMR_INPUTCHANNELPOLARITY_RISING: Rising Edge
  *            @arg TMR_INPUTCHANNELPOLARITY_FALLING: Falling Edge
  *            @arg TMR_INPUTCHANNELPOLARITY_BOTHEDGE: Rising and Falling Edge
  * @retval None
  */
#define __DAL_TMR_SET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__, __POLARITY__)    \
  do{                                                                     \
    TMR_RESET_CAPTUREPOLARITY((__HANDLE__), (__CHANNEL__));               \
    TMR_SET_CAPTUREPOLARITY((__HANDLE__), (__CHANNEL__), (__POLARITY__)); \
  }while(0)

/** @brief  Select the Capture/compare DMA request source.
  * @param  __HANDLE__ specifies the TMR Handle.
  * @param  __CCDMA__ specifies Capture/compare DMA request source
  *          This parameter can be one of the following values:
  *            @arg TMR_CCDMAREQUEST_CC: CCx DMA request generated on Capture/Compare event
  *            @arg TMR_CCDMAREQUEST_UPDATE: CCx DMA request generated on Update event
  * @retval None
  */
#define __DAL_TMR_SELECT_CCDMAREQUEST(__HANDLE__, __CCDMA__)    \
  MODIFY_REG((__HANDLE__)->Instance->CTRL2, TMR_CTRL2_CCDSEL, (__CCDMA__))

/**
  * @}
  */
/* End of exported macros ----------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup TMR_Private_Constants TMR Private Constants
  * @{
  */
/* The counter of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
#define TMR_CCEN_CCxE_MASK  ((uint32_t)(TMR_CCEN_CC1EN | TMR_CCEN_CC2EN | TMR_CCEN_CC3EN | TMR_CCEN_CC4EN))
#define TMR_CCEN_CCxNE_MASK ((uint32_t)(TMR_CCEN_CC1NEN | TMR_CCEN_CC2NEN | TMR_CCEN_CC3NEN))
/**
  * @}
  */
/* End of private constants --------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/** @defgroup TMR_Private_Macros TMR Private Macros
  * @{
  */
#define IS_TMR_CLEARINPUT_SOURCE(__MODE__)  (((__MODE__) == TMR_CLEARINPUTSOURCE_NONE)      || \
                                             ((__MODE__) == TMR_CLEARINPUTSOURCE_ETR))

#define IS_TMR_DMA_BASE(__BASE__) (((__BASE__) == TMR_DMABASE_CTRL1)    || \
                                   ((__BASE__) == TMR_DMABASE_CTRL2)    || \
                                   ((__BASE__) == TMR_DMABASE_SMCTRL)   || \
                                   ((__BASE__) == TMR_DMABASE_DIEN)     || \
                                   ((__BASE__) == TMR_DMABASE_STS)      || \
                                   ((__BASE__) == TMR_DMABASE_CEG)      || \
                                   ((__BASE__) == TMR_DMABASE_CCM1)     || \
                                   ((__BASE__) == TMR_DMABASE_CCM2)     || \
                                   ((__BASE__) == TMR_DMABASE_CCEN)     || \
                                   ((__BASE__) == TMR_DMABASE_CNT)      || \
                                   ((__BASE__) == TMR_DMABASE_PSC)      || \
                                   ((__BASE__) == TMR_DMABASE_AUTORLD)  || \
                                   ((__BASE__) == TMR_DMABASE_REPCNT)   || \
                                   ((__BASE__) == TMR_DMABASE_CC1)      || \
                                   ((__BASE__) == TMR_DMABASE_CC2)      || \
                                   ((__BASE__) == TMR_DMABASE_CC3)      || \
                                   ((__BASE__) == TMR_DMABASE_CC4)      || \
                                   ((__BASE__) == TMR_DMABASE_BDT))

#define IS_TMR_EVENT_SOURCE(__SOURCE__) ((((__SOURCE__) & 0xFFFFFF00U) == 0x00000000U) && ((__SOURCE__) != 0x00000000U))

#define IS_TMR_COUNTER_MODE(__MODE__)      (((__MODE__) == TMR_COUNTERMODE_UP)              || \
                                            ((__MODE__) == TMR_COUNTERMODE_DOWN)            || \
                                            ((__MODE__) == TMR_COUNTERMODE_CENTERALIGNED1)  || \
                                            ((__MODE__) == TMR_COUNTERMODE_CENTERALIGNED2)  || \
                                            ((__MODE__) == TMR_COUNTERMODE_CENTERALIGNED3))

#define IS_TMR_CLOCKDIVISION_DIV(__DIV__)  (((__DIV__) == TMR_CLOCKDIVISION_DIV1) || \
                                            ((__DIV__) == TMR_CLOCKDIVISION_DIV2) || \
                                            ((__DIV__) == TMR_CLOCKDIVISION_DIV4))

#define IS_TMR_AUTORELOAD_PRELOAD(PRELOAD) (((PRELOAD) == TMR_AUTORELOAD_PRELOAD_DISABLE) || \
                                            ((PRELOAD) == TMR_AUTORELOAD_PRELOAD_ENABLE))

#define IS_TMR_FAST_STATE(__STATE__)       (((__STATE__) == TMR_OCFAST_DISABLE) || \
                                            ((__STATE__) == TMR_OCFAST_ENABLE))

#define IS_TMR_OC_POLARITY(__POLARITY__)   (((__POLARITY__) == TMR_OCPOLARITY_HIGH) || \
                                            ((__POLARITY__) == TMR_OCPOLARITY_LOW))

#define IS_TMR_OCN_POLARITY(__POLARITY__)  (((__POLARITY__) == TMR_OCNPOLARITY_HIGH) || \
                                            ((__POLARITY__) == TMR_OCNPOLARITY_LOW))

#define IS_TMR_OCIDLE_STATE(__STATE__)     (((__STATE__) == TMR_OCIDLESTATE_SET) || \
                                            ((__STATE__) == TMR_OCIDLESTATE_RESET))

#define IS_TMR_OCNIDLE_STATE(__STATE__)    (((__STATE__) == TMR_OCNIDLESTATE_SET) || \
                                            ((__STATE__) == TMR_OCNIDLESTATE_RESET))

#define IS_TMR_ENCODERINPUT_POLARITY(__POLARITY__)   (((__POLARITY__) == TMR_ENCODERINPUTPOLARITY_RISING)   || \
                                                      ((__POLARITY__) == TMR_ENCODERINPUTPOLARITY_FALLING))

#define IS_TMR_IC_POLARITY(__POLARITY__)   (((__POLARITY__) == TMR_ICPOLARITY_RISING)   || \
                                            ((__POLARITY__) == TMR_ICPOLARITY_FALLING)  || \
                                            ((__POLARITY__) == TMR_ICPOLARITY_BOTHEDGE))

#define IS_TMR_IC_SELECTION(__SELECTION__) (((__SELECTION__) == TMR_ICSELECTION_DIRECTTI) || \
                                            ((__SELECTION__) == TMR_ICSELECTION_INDIRECTTI) || \
                                            ((__SELECTION__) == TMR_ICSELECTION_TRC))

#define IS_TMR_IC_PRESCALER(__PRESCALER__) (((__PRESCALER__) == TMR_ICPSC_DIV1) || \
                                            ((__PRESCALER__) == TMR_ICPSC_DIV2) || \
                                            ((__PRESCALER__) == TMR_ICPSC_DIV4) || \
                                            ((__PRESCALER__) == TMR_ICPSC_DIV8))

#define IS_TMR_OPM_MODE(__MODE__)          (((__MODE__) == TMR_OPMODE_SINGLE) || \
                                            ((__MODE__) == TMR_OPMODE_REPETITIVE))

#define IS_TMR_ENCODER_MODE(__MODE__)      (((__MODE__) == TMR_ENCODERMODE_TI1) || \
                                            ((__MODE__) == TMR_ENCODERMODE_TI2) || \
                                            ((__MODE__) == TMR_ENCODERMODE_TI12))

#define IS_TMR_DMA_SOURCE(__SOURCE__) ((((__SOURCE__) & 0xFFFF80FFU) == 0x00000000U) && ((__SOURCE__) != 0x00000000U))

#define IS_TMR_CHANNELS(__CHANNEL__)       (((__CHANNEL__) == TMR_CHANNEL_1) || \
                                            ((__CHANNEL__) == TMR_CHANNEL_2) || \
                                            ((__CHANNEL__) == TMR_CHANNEL_3) || \
                                            ((__CHANNEL__) == TMR_CHANNEL_4) || \
                                            ((__CHANNEL__) == TMR_CHANNEL_ALL))

#define IS_TMR_OPM_CHANNELS(__CHANNEL__)   (((__CHANNEL__) == TMR_CHANNEL_1) || \
                                            ((__CHANNEL__) == TMR_CHANNEL_2))

#define IS_TMR_COMPLEMENTARY_CHANNELS(__CHANNEL__) (((__CHANNEL__) == TMR_CHANNEL_1) || \
                                                    ((__CHANNEL__) == TMR_CHANNEL_2) || \
                                                    ((__CHANNEL__) == TMR_CHANNEL_3))

#define IS_TMR_CLOCKSOURCE(__CLOCK__) (((__CLOCK__) == TMR_CLOCKSOURCE_INTERNAL) || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ETRMODE1) || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ETRMODE2) || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_TI1ED)    || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_TI1)      || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_TI2)      || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ITR0)     || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ITR1)     || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ITR2)     || \
                                       ((__CLOCK__) == TMR_CLOCKSOURCE_ITR3))

#define IS_TMR_CLOCKPOLARITY(__POLARITY__) (((__POLARITY__) == TMR_CLOCKPOLARITY_INVERTED)    || \
                                            ((__POLARITY__) == TMR_CLOCKPOLARITY_NONINVERTED) || \
                                            ((__POLARITY__) == TMR_CLOCKPOLARITY_RISING)      || \
                                            ((__POLARITY__) == TMR_CLOCKPOLARITY_FALLING)     || \
                                            ((__POLARITY__) == TMR_CLOCKPOLARITY_BOTHEDGE))

#define IS_TMR_CLOCKPRESCALER(__PRESCALER__) (((__PRESCALER__) == TMR_CLOCKPRESCALER_DIV1) || \
                                              ((__PRESCALER__) == TMR_CLOCKPRESCALER_DIV2) || \
                                              ((__PRESCALER__) == TMR_CLOCKPRESCALER_DIV4) || \
                                              ((__PRESCALER__) == TMR_CLOCKPRESCALER_DIV8))

#define IS_TMR_CLOCKFILTER(__ICFILTER__)      ((__ICFILTER__) <= 0xFU)

#define IS_TMR_CLEARINPUT_POLARITY(__POLARITY__) (((__POLARITY__) == TMR_CLEARINPUTPOLARITY_INVERTED) || \
                                                  ((__POLARITY__) == TMR_CLEARINPUTPOLARITY_NONINVERTED))

#define IS_TMR_CLEARINPUT_PRESCALER(__PRESCALER__) (((__PRESCALER__) == TMR_CLEARINPUTPRESCALER_DIV1) || \
                                                    ((__PRESCALER__) == TMR_CLEARINPUTPRESCALER_DIV2) || \
                                                    ((__PRESCALER__) == TMR_CLEARINPUTPRESCALER_DIV4) || \
                                                    ((__PRESCALER__) == TMR_CLEARINPUTPRESCALER_DIV8))

#define IS_TMR_CLEARINPUT_FILTER(__ICFILTER__) ((__ICFILTER__) <= 0xFU)

#define IS_TMR_OSSR_STATE(__STATE__)       (((__STATE__) == TMR_OSSR_ENABLE) || \
                                            ((__STATE__) == TMR_OSSR_DISABLE))

#define IS_TMR_OSSI_STATE(__STATE__)       (((__STATE__) == TMR_OSSI_ENABLE) || \
                                            ((__STATE__) == TMR_OSSI_DISABLE))

#define IS_TMR_LOCK_LEVEL(__LEVEL__)       (((__LEVEL__) == TMR_LOCKLEVEL_OFF) || \
                                            ((__LEVEL__) == TMR_LOCKLEVEL_1)   || \
                                            ((__LEVEL__) == TMR_LOCKLEVEL_2)   || \
                                            ((__LEVEL__) == TMR_LOCKLEVEL_3))

#define IS_TMR_BREAK_FILTER(__BRKFILTER__) ((__BRKFILTER__) <= 0xFUL)


#define IS_TMR_BREAK_STATE(__STATE__)      (((__STATE__) == TMR_BREAK_ENABLE) || \
                                            ((__STATE__) == TMR_BREAK_DISABLE))

#define IS_TMR_BREAK_POLARITY(__POLARITY__) (((__POLARITY__) == TMR_BREAKPOLARITY_LOW) || \
                                             ((__POLARITY__) == TMR_BREAKPOLARITY_HIGH))

#define IS_TMR_AUTOMATIC_OUTPUT_STATE(__STATE__) (((__STATE__) == TMR_AUTOMATICOUTPUT_ENABLE) || \
                                                  ((__STATE__) == TMR_AUTOMATICOUTPUT_DISABLE))

#define IS_TMR_TRGO_SOURCE(__SOURCE__) (((__SOURCE__) == TMR_TRGO_RESET)  || \
                                        ((__SOURCE__) == TMR_TRGO_ENABLE) || \
                                        ((__SOURCE__) == TMR_TRGO_UPDATE) || \
                                        ((__SOURCE__) == TMR_TRGO_OC1)    || \
                                        ((__SOURCE__) == TMR_TRGO_OC1REF) || \
                                        ((__SOURCE__) == TMR_TRGO_OC2REF) || \
                                        ((__SOURCE__) == TMR_TRGO_OC3REF) || \
                                        ((__SOURCE__) == TMR_TRGO_OC4REF))

#define IS_TMR_MSM_STATE(__STATE__)      (((__STATE__) == TMR_MASTERSLAVEMODE_ENABLE) || \
                                          ((__STATE__) == TMR_MASTERSLAVEMODE_DISABLE))

#define IS_TMR_SLAVE_MODE(__MODE__) (((__MODE__) == TMR_SLAVEMODE_DISABLE)   || \
                                     ((__MODE__) == TMR_SLAVEMODE_RESET)     || \
                                     ((__MODE__) == TMR_SLAVEMODE_GATED)     || \
                                     ((__MODE__) == TMR_SLAVEMODE_TRIGGER)   || \
                                     ((__MODE__) == TMR_SLAVEMODE_EXTERNAL1))

#define IS_TMR_PWM_MODE(__MODE__) (((__MODE__) == TMR_OCMODE_PWM1)               || \
                                   ((__MODE__) == TMR_OCMODE_PWM2))

#define IS_TMR_OC_MODE(__MODE__)  (((__MODE__) == TMR_OCMODE_TIMING)             || \
                                   ((__MODE__) == TMR_OCMODE_ACTIVE)             || \
                                   ((__MODE__) == TMR_OCMODE_INACTIVE)           || \
                                   ((__MODE__) == TMR_OCMODE_TOGGLE)             || \
                                   ((__MODE__) == TMR_OCMODE_FORCED_ACTIVE)      || \
                                   ((__MODE__) == TMR_OCMODE_FORCED_INACTIVE))

#define IS_TMR_TRIGGER_SELECTION(__SELECTION__) (((__SELECTION__) == TMR_TS_ITR0)    || \
                                                 ((__SELECTION__) == TMR_TS_ITR1)    || \
                                                 ((__SELECTION__) == TMR_TS_ITR2)    || \
                                                 ((__SELECTION__) == TMR_TS_ITR3)    || \
                                                 ((__SELECTION__) == TMR_TS_TI1F_ED) || \
                                                 ((__SELECTION__) == TMR_TS_TI1FP1)  || \
                                                 ((__SELECTION__) == TMR_TS_TI2FP2)  || \
                                                 ((__SELECTION__) == TMR_TS_ETRF))

#define IS_TMR_INTERNAL_TRIGGEREVENT_SELECTION(__SELECTION__) (((__SELECTION__) == TMR_TS_ITR0) || \
                                                               ((__SELECTION__) == TMR_TS_ITR1) || \
                                                               ((__SELECTION__) == TMR_TS_ITR2) || \
                                                               ((__SELECTION__) == TMR_TS_ITR3) || \
                                                               ((__SELECTION__) == TMR_TS_NONE))

#define IS_TMR_TRIGGERPOLARITY(__POLARITY__)   (((__POLARITY__) == TMR_TRIGGERPOLARITY_INVERTED   ) || \
                                                ((__POLARITY__) == TMR_TRIGGERPOLARITY_NONINVERTED) || \
                                                ((__POLARITY__) == TMR_TRIGGERPOLARITY_RISING     ) || \
                                                ((__POLARITY__) == TMR_TRIGGERPOLARITY_FALLING    ) || \
                                                ((__POLARITY__) == TMR_TRIGGERPOLARITY_BOTHEDGE   ))

#define IS_TMR_TRIGGERPRESCALER(__PRESCALER__) (((__PRESCALER__) == TMR_TRIGGERPRESCALER_DIV1) || \
                                                ((__PRESCALER__) == TMR_TRIGGERPRESCALER_DIV2) || \
                                                ((__PRESCALER__) == TMR_TRIGGERPRESCALER_DIV4) || \
                                                ((__PRESCALER__) == TMR_TRIGGERPRESCALER_DIV8))

#define IS_TMR_TRIGGERFILTER(__ICFILTER__) ((__ICFILTER__) <= 0xFU)

#define IS_TMR_TI1SELECTION(__TI1SELECTION__)  (((__TI1SELECTION__) == TMR_TI1SELECTION_CH1) || \
                                                ((__TI1SELECTION__) == TMR_TI1SELECTION_XORCOMBINATION))

#define IS_TMR_DMA_LENGTH(__LENGTH__)      (((__LENGTH__) == TMR_DMABURSTLENGTH_1TRANSFER)   || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_2TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_3TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_4TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_5TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_6TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_7TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_8TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_9TRANSFERS)  || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_10TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_11TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_12TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_13TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_14TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_15TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_16TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_17TRANSFERS) || \
                                            ((__LENGTH__) == TMR_DMABURSTLENGTH_18TRANSFERS))

#define IS_TMR_DMA_DATA_LENGTH(LENGTH) (((LENGTH) >= 0x1U) && ((LENGTH) < 0x10000U))

#define IS_TMR_IC_FILTER(__ICFILTER__)   ((__ICFILTER__) <= 0xFU)

#define IS_TMR_DEADTIME(__DEADTIME__)    ((__DEADTIME__) <= 0xFFU)

#define IS_TMR_SLAVEMODE_TRIGGER_ENABLED(__TRIGGER__) ((__TRIGGER__) == TMR_SLAVEMODE_TRIGGER)

#define TMR_SET_ICPRESCALERVALUE(__HANDLE__, __CHANNEL__, __ICPSC__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 |= (__ICPSC__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 |= ((__ICPSC__) << 8U)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 |= (__ICPSC__)) :\
   ((__HANDLE__)->Instance->CCM2 |= ((__ICPSC__) << 8U)))

#define TMR_RESET_ICPRESCALERVALUE(__HANDLE__, __CHANNEL__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_IC1PSC) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCM1 &= ~TMR_CCM1_IC2PSC) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_IC3PSC) :\
   ((__HANDLE__)->Instance->CCM2 &= ~TMR_CCM2_IC4PSC))

#define TMR_SET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__, __POLARITY__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCEN |= (__POLARITY__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCEN |= ((__POLARITY__) << 4U)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCEN |= ((__POLARITY__) << 8U)) :\
   ((__HANDLE__)->Instance->CCEN |= (((__POLARITY__) << 12U))))

#define TMR_RESET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->Instance->CCEN &= ~(TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->Instance->CCEN &= ~(TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->Instance->CCEN &= ~(TMR_CCEN_CC3POL | TMR_CCEN_CC3NPOL)) :\
   ((__HANDLE__)->Instance->CCEN &= ~(TMR_CCEN_CC4POL | TMR_CCEN_CC4NPOL)))

#define TMR_CHANNEL_STATE_GET(__HANDLE__, __CHANNEL__)\
  (((__CHANNEL__) == TMR_CHANNEL_1) ? (__HANDLE__)->ChannelState[0] :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? (__HANDLE__)->ChannelState[1] :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? (__HANDLE__)->ChannelState[2] :\
   (__HANDLE__)->ChannelState[3])

#define TMR_CHANNEL_STATE_SET(__HANDLE__, __CHANNEL__, __CHANNEL_STATE__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->ChannelState[0] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->ChannelState[1] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->ChannelState[2] = (__CHANNEL_STATE__)) :\
   ((__HANDLE__)->ChannelState[3] = (__CHANNEL_STATE__)))

#define TMR_CHANNEL_STATE_SET_ALL(__HANDLE__,  __CHANNEL_STATE__) do { \
                                                                       (__HANDLE__)->ChannelState[0]  = (__CHANNEL_STATE__);  \
                                                                       (__HANDLE__)->ChannelState[1]  = (__CHANNEL_STATE__);  \
                                                                       (__HANDLE__)->ChannelState[2]  = (__CHANNEL_STATE__);  \
                                                                       (__HANDLE__)->ChannelState[3]  = (__CHANNEL_STATE__);  \
                                                                     } while(0)

#define TMR_CHANNEL_N_STATE_GET(__HANDLE__, __CHANNEL__)\
  (((__CHANNEL__) == TMR_CHANNEL_1) ? (__HANDLE__)->ChannelNState[0] :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? (__HANDLE__)->ChannelNState[1] :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? (__HANDLE__)->ChannelNState[2] :\
   (__HANDLE__)->ChannelNState[3])

#define TMR_CHANNEL_N_STATE_SET(__HANDLE__, __CHANNEL__, __CHANNEL_STATE__) \
  (((__CHANNEL__) == TMR_CHANNEL_1) ? ((__HANDLE__)->ChannelNState[0] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_2) ? ((__HANDLE__)->ChannelNState[1] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TMR_CHANNEL_3) ? ((__HANDLE__)->ChannelNState[2] = (__CHANNEL_STATE__)) :\
   ((__HANDLE__)->ChannelNState[3] = (__CHANNEL_STATE__)))

#define TMR_CHANNEL_N_STATE_SET_ALL(__HANDLE__,  __CHANNEL_STATE__) do { \
                                                                         (__HANDLE__)->ChannelNState[0] = \
                                                                         (__CHANNEL_STATE__);  \
                                                                         (__HANDLE__)->ChannelNState[1] = \
                                                                         (__CHANNEL_STATE__);  \
                                                                         (__HANDLE__)->ChannelNState[2] = \
                                                                         (__CHANNEL_STATE__);  \
                                                                         (__HANDLE__)->ChannelNState[3] = \
                                                                         (__CHANNEL_STATE__);  \
                                                                       } while(0)

/**
  * @}
  */
/* End of private macros -----------------------------------------------------*/

/* Include TMR DAL Extended module */
#include "apm32f4xx_dal_tmr_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup TMR_Exported_Functions TMR Exported Functions
  * @{
  */

/** @addtogroup TMR_Exported_Functions_Group1 TMR Time Base functions
  *  @brief   Time Base functions
  * @{
  */
/* Time Base functions ********************************************************/
DAL_StatusTypeDef DAL_TMR_Base_Init(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_Base_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_Base_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_Base_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_Base_Start(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_Base_Stop(TMR_HandleTypeDef *htmr);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_Base_Start_IT(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_Base_Stop_IT(TMR_HandleTypeDef *htmr);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMR_Base_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMR_Base_Stop_DMA(TMR_HandleTypeDef *htmr);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group2 TMR Output Compare functions
  *  @brief   TMR Output Compare functions
  * @{
  */
/* Timer Output Compare functions *********************************************/
DAL_StatusTypeDef DAL_TMR_OC_Init(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_OC_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_OC_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_OC_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_OC_Start(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_OC_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_OC_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_OC_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMR_OC_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMR_OC_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group3 TMR PWM functions
  *  @brief   TMR PWM functions
  * @{
  */
/* Timer PWM functions ********************************************************/
DAL_StatusTypeDef DAL_TMR_PWM_Init(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_PWM_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_PWM_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_PWM_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_PWM_Start(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_PWM_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_PWM_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_PWM_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMR_PWM_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMR_PWM_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group4 TMR Input Capture functions
  *  @brief   TMR Input Capture functions
  * @{
  */
/* Timer Input Capture functions **********************************************/
DAL_StatusTypeDef DAL_TMR_IC_Init(TMR_HandleTypeDef *htmr);
DAL_StatusTypeDef DAL_TMR_IC_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_IC_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_IC_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_IC_Start(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_IC_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_IC_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_IC_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMR_IC_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMR_IC_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group5 TMR One Pulse functions
  *  @brief   TMR One Pulse functions
  * @{
  */
/* Timer One Pulse functions **************************************************/
DAL_StatusTypeDef DAL_TMR_OnePulse_Init(TMR_HandleTypeDef *htmr, uint32_t OnePulseMode);
DAL_StatusTypeDef DAL_TMR_OnePulse_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_OnePulse_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_OnePulse_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_OnePulse_Start(TMR_HandleTypeDef *htmr, uint32_t OutputChannel);
DAL_StatusTypeDef DAL_TMR_OnePulse_Stop(TMR_HandleTypeDef *htmr, uint32_t OutputChannel);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_OnePulse_Start_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel);
DAL_StatusTypeDef DAL_TMR_OnePulse_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group6 TMR Encoder functions
  *  @brief   TMR Encoder functions
  * @{
  */
/* Timer Encoder functions ****************************************************/
DAL_StatusTypeDef DAL_TMR_Encoder_Init(TMR_HandleTypeDef *htmr,  TMR_Encoder_InitTypeDef *sConfig);
DAL_StatusTypeDef DAL_TMR_Encoder_DeInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_Encoder_MspInit(TMR_HandleTypeDef *htmr);
void DAL_TMR_Encoder_MspDeInit(TMR_HandleTypeDef *htmr);
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMR_Encoder_Start(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_Encoder_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMR_Encoder_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_Encoder_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMR_Encoder_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
DAL_StatusTypeDef DAL_TMR_Encoder_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMR_Exported_Functions_Group7 TMR IRQ handler management
  *  @brief   IRQ handler management
  * @{
  */
/* Interrupt Handler functions  ***********************************************/
void DAL_TMR_IRQHandler(TMR_HandleTypeDef *htmr);
/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group8 TMR Peripheral Control functions
  *  @brief   Peripheral Control functions
  * @{
  */
/* Control functions  *********************************************************/
DAL_StatusTypeDef DAL_TMR_OC_ConfigChannel(TMR_HandleTypeDef *htmr, TMR_OC_InitTypeDef *sConfig, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_PWM_ConfigChannel(TMR_HandleTypeDef *htmr, TMR_OC_InitTypeDef *sConfig, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_IC_ConfigChannel(TMR_HandleTypeDef *htmr, TMR_IC_InitTypeDef *sConfig, uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_OnePulse_ConfigChannel(TMR_HandleTypeDef *htmr, TMR_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel,  uint32_t InputChannel);
DAL_StatusTypeDef DAL_TMR_ConfigOCrefClear(TMR_HandleTypeDef *htmr, TMR_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
DAL_StatusTypeDef DAL_TMR_ConfigClockSource(TMR_HandleTypeDef *htmr, TMR_ClockConfigTypeDef *sClockSourceConfig);
DAL_StatusTypeDef DAL_TMR_ConfigTI1Input(TMR_HandleTypeDef *htmr, uint32_t TI1_Selection);
DAL_StatusTypeDef DAL_TMR_SlaveConfigSynchro(TMR_HandleTypeDef *htmr, TMR_SlaveConfigTypeDef *sSlaveConfig);
DAL_StatusTypeDef DAL_TMR_SlaveConfigSynchro_IT(TMR_HandleTypeDef *htmr, TMR_SlaveConfigTypeDef *sSlaveConfig);
DAL_StatusTypeDef DAL_TMR_DMABurst_WriteStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
DAL_StatusTypeDef DAL_TMR_DMABurst_MultiWriteStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, uint32_t *BurstBuffer,
                                                   uint32_t BurstLength,  uint32_t DataLength);
DAL_StatusTypeDef DAL_TMR_DMABurst_WriteStop(TMR_HandleTypeDef *htmr, uint32_t BurstRequestSrc);
DAL_StatusTypeDef DAL_TMR_DMABurst_ReadStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
DAL_StatusTypeDef DAL_TMR_DMABurst_MultiReadStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t  *BurstBuffer,
                                                  uint32_t  BurstLength, uint32_t  DataLength);
DAL_StatusTypeDef DAL_TMR_DMABurst_ReadStop(TMR_HandleTypeDef *htmr, uint32_t BurstRequestSrc);
DAL_StatusTypeDef DAL_TMR_GenerateEvent(TMR_HandleTypeDef *htmr, uint32_t EventSource);
uint32_t DAL_TMR_ReadCapturedValue(TMR_HandleTypeDef *htmr, uint32_t Channel);
/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group9 TMR Callbacks functions
  *  @brief   TMR Callbacks functions
  * @{
  */
/* Callback in non blocking modes (Interrupt and DMA) *************************/
void DAL_TMR_PeriodElapsedCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_PeriodElapsedHalfCpltCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_OC_DelayElapsedCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_IC_CaptureCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_IC_CaptureHalfCpltCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_PWM_PulseFinishedCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_PWM_PulseFinishedHalfCpltCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_TriggerCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_TriggerHalfCpltCallback(TMR_HandleTypeDef *htmr);
void DAL_TMR_ErrorCallback(TMR_HandleTypeDef *htmr);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_TMR_RegisterCallback(TMR_HandleTypeDef *htmr, DAL_TMR_CallbackIDTypeDef CallbackID,
                                           pTMR_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_TMR_UnRegisterCallback(TMR_HandleTypeDef *htmr, DAL_TMR_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group10 TMR Peripheral State functions
  *  @brief  Peripheral State functions
  * @{
  */
/* Peripheral State functions  ************************************************/
DAL_TMR_StateTypeDef DAL_TMR_Base_GetState(TMR_HandleTypeDef *htmr);
DAL_TMR_StateTypeDef DAL_TMR_OC_GetState(TMR_HandleTypeDef *htmr);
DAL_TMR_StateTypeDef DAL_TMR_PWM_GetState(TMR_HandleTypeDef *htmr);
DAL_TMR_StateTypeDef DAL_TMR_IC_GetState(TMR_HandleTypeDef *htmr);
DAL_TMR_StateTypeDef DAL_TMR_OnePulse_GetState(TMR_HandleTypeDef *htmr);
DAL_TMR_StateTypeDef DAL_TMR_Encoder_GetState(TMR_HandleTypeDef *htmr);

/* Peripheral Channel state functions  ************************************************/
DAL_TMR_ActiveChannel DAL_TMR_GetActiveChannel(TMR_HandleTypeDef *htmr);
DAL_TMR_ChannelStateTypeDef DAL_TMR_GetChannelState(TMR_HandleTypeDef *htmr,  uint32_t Channel);
DAL_TMR_DMABurstStateTypeDef DAL_TMR_DMABurstState(TMR_HandleTypeDef *htmr);
/**
  * @}
  */

/**
  * @}
  */
/* End of exported functions -------------------------------------------------*/

/* Private functions----------------------------------------------------------*/
/** @defgroup TMR_Private_Functions TMR Private Functions
  * @{
  */
void TMR_Base_SetConfig(TMR_TypeDef *TMRx, TMR_Base_InitTypeDef *Structure);
void TMR_TI1_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection, uint32_t TMR_ICFilter);
void TMR_OC2_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config);
void TMR_ETR_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ExtTRGPrescaler,
                       uint32_t TMR_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TMR_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TMR_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TMR_DMAError(DMA_HandleTypeDef *hdma);
void TMR_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TMR_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TMR_CCxChannelCmd(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ChannelState);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
void TMR_ResetCallback(TMR_HandleTypeDef *htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @}
  */
/* End of private functions --------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_TMR_H */
