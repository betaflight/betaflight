/**
  *
  * @file    apm32f4xx_dal_rtc.h
  * @brief   Header file of RTC DAL module.
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
#ifndef APM32F4xx_DAL_RTC_H
#define APM32F4xx_DAL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup RTC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RTC_Exported_Types RTC Exported Types
  * @{
  */

/**
  * @brief  DAL State structures definition
  */
typedef enum
{
  DAL_RTC_STATE_RESET             = 0x00U,  /*!< RTC not yet initialized or disabled */
  DAL_RTC_STATE_READY             = 0x01U,  /*!< RTC initialized and ready for use   */
  DAL_RTC_STATE_BUSY              = 0x02U,  /*!< RTC process is ongoing              */
  DAL_RTC_STATE_TIMEOUT           = 0x03U,  /*!< RTC timeout state                   */
  DAL_RTC_STATE_ERROR             = 0x04U   /*!< RTC error state                     */
} DAL_RTCStateTypeDef;

/**
  * @brief  RTC Configuration Structure definition
  */
typedef struct
{
  uint32_t HourFormat;      /*!< Specifies the RTC Hour Format.
                                 This parameter can be a value of @ref RTC_Hour_Formats */

  uint32_t AsynchPrediv;    /*!< Specifies the RTC Asynchronous Predivider value.
                                 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x7F */

  uint32_t SynchPrediv;     /*!< Specifies the RTC Synchronous Predivider value.
                                 This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x7FFF */

  uint32_t OutPut;          /*!< Specifies which signal will be routed to the RTC output.
                                 This parameter can be a value of @ref RTC_Output_selection_Definitions */

  uint32_t OutPutPolarity;  /*!< Specifies the polarity of the output signal.
                                 This parameter can be a value of @ref RTC_Output_Polarity_Definitions */

  uint32_t OutPutType;      /*!< Specifies the RTC Output Pin mode.
                                 This parameter can be a value of @ref RTC_Output_Type_ALARM_OUT */
} RTC_InitTypeDef;

/**
  * @brief  RTC Time structure definition
  */
typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t TimeFormat;       /*!< Specifies the RTC AM/PM Time.
                                 This parameter can be a value of @ref RTC_AM_PM_Definitions */

  uint32_t SubSeconds;      /*!< Specifies the RTC_SUBSEC RTC Sub Second register content.
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity */

  uint32_t SecondFraction;  /*!< Specifies the range or granularity of Sub Second register content
                                 corresponding to Synchronous prescaler factor value (PREDIV_S)
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity.
                                 This field will be used only by DAL_RTC_GetTime function */

  uint32_t DayLightSaving;  /*!< This interface is deprecated. To manage Daylight
                                 Saving Time, please use DAL_RTC_DST_xxx functions */

  uint32_t StoreOperation;  /*!< This interface is deprecated. To manage Daylight
                                 Saving Time, please use DAL_RTC_DST_xxx functions */
} RTC_TimeTypeDef;

/**
  * @brief  RTC Date structure definition
  */
typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay.
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} RTC_DateTypeDef;

/**
  * @brief  RTC Alarm structure definition
  */
typedef struct
{
  RTC_TimeTypeDef AlarmTime;     /*!< Specifies the RTC Alarm Time members */

  uint32_t AlarmMask;            /*!< Specifies the RTC Alarm Masks.
                                      This parameter can be a value of @ref RTC_AlarmMask_Definitions */

  uint32_t AlarmSubSecondMask;   /*!< Specifies the RTC Alarm SubSeconds Masks.
                                      This parameter can be a value of @ref RTC_Alarm_Sub_Seconds_Masks_Definitions */

  uint32_t AlarmDateWeekDaySel;  /*!< Specifies the RTC Alarm is on Date or WeekDay.
                                      This parameter can be a value of @ref RTC_AlarmDateWeekDay_Definitions */

  uint8_t AlarmDateWeekDay;      /*!< Specifies the RTC Alarm Date/WeekDay.
                                      If the Alarm Date is selected, this parameter must be set to a value in the 1-31 range.
                                      If the Alarm WeekDay is selected, this parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint32_t Alarm;                /*!< Specifies the alarm .
                                      This parameter can be a value of @ref RTC_Alarms_Definitions */
} RTC_AlarmTypeDef;

/**
  * @brief  RTC Handle Structure definition
  */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
typedef struct __RTC_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */
{
  RTC_TypeDef                 *Instance;  /*!< Register base address    */

  RTC_InitTypeDef             Init;       /*!< RTC required parameters  */

  DAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  __IO DAL_RTCStateTypeDef    State;      /*!< Time communication state */

#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
  void (* AlarmAEventCallback)      (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Alarm A Event callback         */

  void (* AlarmBEventCallback)      (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Alarm B Event callback         */

  void (* TimeStampEventCallback)   (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Timestamp Event callback       */

  void (* WakeUpTimerEventCallback) (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC WakeUpTimer Event callback     */

  void (* Tamper1EventCallback)     (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Tamper 1 Event callback        */

#if defined(RTC_TAMPER2_SUPPORT)
  void (* Tamper2EventCallback)     (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Tamper 2 Event callback        */
#endif /* RTC_TAMPER2_SUPPORT */

  void (* MspInitCallback)          (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Msp Init callback              */

  void (* MspDeInitCallback)        (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Msp DeInit callback            */

#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

} RTC_HandleTypeDef;

#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL RTC Callback ID enumeration definition
  */
typedef enum
{
  DAL_RTC_ALARM_A_EVENT_CB_ID           = 0x00U,    /*!< RTC Alarm A Event Callback ID       */
  DAL_RTC_ALARM_B_EVENT_CB_ID           = 0x01U,    /*!< RTC Alarm B Event Callback ID       */
  DAL_RTC_TIMESTAMP_EVENT_CB_ID         = 0x02U,    /*!< RTC Timestamp Event Callback ID     */
  DAL_RTC_WAKEUPTIMER_EVENT_CB_ID       = 0x03U,    /*!< RTC Wakeup Timer Event Callback ID  */
  DAL_RTC_TAMPER1_EVENT_CB_ID           = 0x04U,    /*!< RTC Tamper 1 Callback ID            */
#if defined(RTC_TAMPER2_SUPPORT)
  DAL_RTC_TAMPER2_EVENT_CB_ID           = 0x05U,    /*!< RTC Tamper 2 Callback ID            */
#endif /* RTC_TAMPER2_SUPPORT */
  DAL_RTC_MSPINIT_CB_ID                 = 0x0EU,    /*!< RTC Msp Init callback ID            */
  DAL_RTC_MSPDEINIT_CB_ID               = 0x0FU     /*!< RTC Msp DeInit callback ID          */
} DAL_RTC_CallbackIDTypeDef;

/**
  * @brief  DAL RTC Callback pointer definition
  */
typedef  void (*pRTC_CallbackTypeDef)(RTC_HandleTypeDef *hrtc);  /*!< pointer to an RTC callback function */
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup RTC_Exported_Constants RTC Exported Constants
  * @{
  */

/** @defgroup RTC_Hour_Formats RTC Hour Formats
  * @{
  */
#define RTC_HOURFORMAT_24              0x00000000U
#define RTC_HOURFORMAT_12              RTC_CTRL_TIMEFCFG
/**
  * @}
  */

/** @defgroup RTC_Output_selection_Definitions RTC Output Selection Definitions
  * @{
  */
#define RTC_OUTPUT_DISABLE             0x00000000U
#define RTC_OUTPUT_ALARMA              RTC_CTRL_OUTSEL_0
#define RTC_OUTPUT_ALARMB              RTC_CTRL_OUTSEL_1
#define RTC_OUTPUT_WAKEUP              RTC_CTRL_OUTSEL
/**
  * @}
  */

/** @defgroup RTC_Output_Polarity_Definitions RTC Output Polarity Definitions
  * @{
  */
#define RTC_OUTPUT_POLARITY_HIGH       0x00000000U
#define RTC_OUTPUT_POLARITY_LOW        RTC_CTRL_POLCFG
/**
  * @}
  */

/** @defgroup RTC_Output_Type_ALARM_OUT RTC Output Type ALARM OUT
  * @{
  */
#define RTC_OUTPUT_TYPE_OPENDRAIN      0x00000000U
#define RTC_OUTPUT_TYPE_PUSHPULL       RTC_TACFG_ALRMOT
/**
  * @}
  */

/** @defgroup RTC_AM_PM_Definitions RTC AM PM Definitions
  * @{
  */
#define RTC_HOURFORMAT12_AM            ((uint8_t)0x00)
#define RTC_HOURFORMAT12_PM            ((uint8_t)0x01)
/**
  * @}
  */

/** @defgroup RTC_DayLightSaving_Definitions RTC DayLight Saving Definitions
  * @{
  */
#define RTC_DAYLIGHTSAVING_SUB1H       RTC_CTRL_WTCCFG
#define RTC_DAYLIGHTSAVING_ADD1H       RTC_CTRL_STCCFG
#define RTC_DAYLIGHTSAVING_NONE        0x00000000U
/**
  * @}
  */

/** @defgroup RTC_StoreOperation_Definitions RTC Store Operation Definitions
  * @{
  */
#define RTC_STOREOPERATION_RESET        0x00000000U
#define RTC_STOREOPERATION_SET          RTC_CTRL_BAKP
/**
  * @}
  */

/** @defgroup RTC_Input_parameter_format_definitions RTC Input Parameter Format Definitions
  * @{
  */
#define RTC_FORMAT_BIN                  0x00000000U
#define RTC_FORMAT_BCD                  0x00000001U
/**
  * @}
  */

/** @defgroup RTC_Month_Date_Definitions RTC Month Date Definitions (in BCD format)
  * @{
  */
#define RTC_MONTH_JANUARY              ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY             ((uint8_t)0x02)
#define RTC_MONTH_MARCH                ((uint8_t)0x03)
#define RTC_MONTH_APRIL                ((uint8_t)0x04)
#define RTC_MONTH_MAY                  ((uint8_t)0x05)
#define RTC_MONTH_JUNE                 ((uint8_t)0x06)
#define RTC_MONTH_JULY                 ((uint8_t)0x07)
#define RTC_MONTH_AUGUST               ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER            ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER              ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER             ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER             ((uint8_t)0x12)
/**
  * @}
  */

/** @defgroup RTC_WeekDay_Definitions RTC WeekDay Definitions
  * @{
  */
#define RTC_WEEKDAY_MONDAY             ((uint8_t)0x01)
#define RTC_WEEKDAY_TUESDAY            ((uint8_t)0x02)
#define RTC_WEEKDAY_WEDNESDAY          ((uint8_t)0x03)
#define RTC_WEEKDAY_THURSDAY           ((uint8_t)0x04)
#define RTC_WEEKDAY_FRIDAY             ((uint8_t)0x05)
#define RTC_WEEKDAY_SATURDAY           ((uint8_t)0x06)
#define RTC_WEEKDAY_SUNDAY             ((uint8_t)0x07)
/**
  * @}
  */

/** @defgroup RTC_AlarmDateWeekDay_Definitions RTC Alarm Date WeekDay Definitions
  * @{
  */
#define RTC_ALARMDATEWEEKDAYSEL_DATE      0x00000000U
#define RTC_ALARMDATEWEEKDAYSEL_WEEKDAY   RTC_ALRMA_WEEKSEL
/**
  * @}
  */

/** @defgroup RTC_AlarmMask_Definitions RTC Alarm Mask Definitions
  * @{
  */
#define RTC_ALARMMASK_NONE                0x00000000U
#define RTC_ALARMMASK_DATEWEEKDAY         RTC_ALRMA_DATEMEN
#define RTC_ALARMMASK_HOURS               RTC_ALRMA_HRMEN
#define RTC_ALARMMASK_MINUTES             RTC_ALRMA_MINMEN
#define RTC_ALARMMASK_SECONDS             RTC_ALRMA_SECMEN
#define RTC_ALARMMASK_ALL                 (RTC_ALARMMASK_DATEWEEKDAY | \
                                           RTC_ALARMMASK_HOURS       | \
                                           RTC_ALARMMASK_MINUTES     | \
                                           RTC_ALARMMASK_SECONDS)
/**
  * @}
  */

/** @defgroup RTC_Alarms_Definitions RTC Alarms Definitions
  * @{
  */
#define RTC_ALARM_A                       RTC_CTRL_ALRAEN
#define RTC_ALARM_B                       RTC_CTRL_ALRBEN
/**
  * @}
  */

/** @defgroup RTC_Alarm_Sub_Seconds_Masks_Definitions RTC Alarm Sub Seconds Masks Definitions
  * @{
  */
/*!< All Alarm SS fields are masked. There is no comparison on sub seconds for Alarm */
#define RTC_ALARMSUBSECONDMASK_ALL         0x00000000U
/*!< SS[14:1] are don't care in Alarm comparison. Only SS[0] is compared.     */
#define RTC_ALARMSUBSECONDMASK_SS14_1      RTC_ALRMASS_MASKSEL_0
/*!< SS[14:2] are don't care in Alarm comparison. Only SS[1:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_2      RTC_ALRMASS_MASKSEL_1
/*!< SS[14:3] are don't care in Alarm comparison. Only SS[2:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_3      (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_1)
/*!< SS[14:4] are don't care in Alarm comparison. Only SS[3:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_4      RTC_ALRMASS_MASKSEL_2
/*!< SS[14:5] are don't care in Alarm comparison. Only SS[4:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_5      (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_2)
/*!< SS[14:6] are don't care in Alarm comparison. Only SS[5:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_6      (RTC_ALRMASS_MASKSEL_1 | RTC_ALRMASS_MASKSEL_2)
/*!< SS[14:7] are don't care in Alarm comparison. Only SS[6:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_7      (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_1 | RTC_ALRMASS_MASKSEL_2)
/*!< SS[14:8] are don't care in Alarm comparison. Only SS[7:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_8      RTC_ALRMASS_MASKSEL_3
/*!< SS[14:9] are don't care in Alarm comparison. Only SS[8:0] are compared.  */
#define RTC_ALARMSUBSECONDMASK_SS14_9      (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14:10] are don't care in Alarm comparison. Only SS[9:0] are compared. */
#define RTC_ALARMSUBSECONDMASK_SS14_10     (RTC_ALRMASS_MASKSEL_1 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14:11] are don't care in Alarm comparison. Only SS[10:0] are compared. */
#define RTC_ALARMSUBSECONDMASK_SS14_11     (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_1 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14:12] are don't care in Alarm comparison. Only SS[11:0] are compared. */
#define RTC_ALARMSUBSECONDMASK_SS14_12     (RTC_ALRMASS_MASKSEL_2 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14:13] are don't care in Alarm comparison. Only SS[12:0] are compared. */
#define RTC_ALARMSUBSECONDMASK_SS14_13     (RTC_ALRMASS_MASKSEL_0 | RTC_ALRMASS_MASKSEL_2 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14] is don't care in Alarm comparison. Only SS[13:0] are compared. */
#define RTC_ALARMSUBSECONDMASK_SS14        (RTC_ALRMASS_MASKSEL_1 | RTC_ALRMASS_MASKSEL_2 | RTC_ALRMASS_MASKSEL_3)
/*!< SS[14:0] are compared and must match to activate alarm. */
#define RTC_ALARMSUBSECONDMASK_NONE        RTC_ALRMASS_MASKSEL
/**
  * @}
  */

/** @defgroup RTC_Interrupts_Definitions RTC Interrupts Definitions
  * @{
  */
#define RTC_IT_TS                         RTC_CTRL_TSIEN         /*!< Enable Timestamp Interrupt               */
#define RTC_IT_WUT                        RTC_CTRL_WUTIEN        /*!< Enable Wakeup timer Interrupt            */
#define RTC_IT_ALRB                       RTC_CTRL_ALRBIEN       /*!< Enable Alarm B Interrupt                 */
#define RTC_IT_ALRA                       RTC_CTRL_ALRAIEN       /*!< Enable Alarm A Interrupt                 */
/**
  * @}
  */

/** @defgroup RTC_Flags_Definitions RTC Flags Definitions
  * @{
  */
#define RTC_FLAG_RECALPF                  RTC_STS_RCALPFLG     /*!< Recalibration pending flag               */
#if defined(RTC_TAMPER2_SUPPORT)
#define RTC_FLAG_TAMP2F                   RTC_STS_TP2FLG      /*!< Tamper 2 event flag                      */
#endif /* RTC_TAMPER2_SUPPORT */
#define RTC_FLAG_TAMP1F                   RTC_STS_TP1FLG      /*!< Tamper 1 event flag                      */
#define RTC_FLAG_TSOVF                    RTC_STS_TSOVRFLG       /*!< Timestamp overflow flag                  */
#define RTC_FLAG_TSF                      RTC_STS_TSFLG         /*!< Timestamp event flag                     */
#define RTC_FLAG_WUTF                     RTC_STS_WUTFLG        /*!< Wakeup timer event flag                  */
#define RTC_FLAG_ALRBF                    RTC_STS_ALRBFLG       /*!< Alarm B event flag                       */
#define RTC_FLAG_ALRAF                    RTC_STS_ALRAFLG       /*!< Alarm A event flag                       */
#define RTC_FLAG_INITF                    RTC_STS_RINITFLG       /*!< RTC in initialization mode flag          */
#define RTC_FLAG_RSF                      RTC_STS_RSFLG         /*!< Register synchronization flag            */
#define RTC_FLAG_INITS                    RTC_STS_INITSFLG       /*!< RTC initialization status flag           */
#define RTC_FLAG_SHPF                     RTC_STS_SOPFLG        /*!< Shift operation pending flag             */
#define RTC_FLAG_WUTWF                    RTC_STS_WUTWFLG       /*!< WUTR register write allowance flag       */
#define RTC_FLAG_ALRBWF                   RTC_STS_ALRBWFLG      /*!< ALRMBR register write allowance flag     */
#define RTC_FLAG_ALRAWF                   RTC_STS_ALRAWFLG      /*!< ALRMAR register write allowance flag     */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/

/** @defgroup RTC_Exported_Macros RTC Exported Macros
  * @{
  */

/** @brief Reset RTC handle state
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
#define __DAL_RTC_RESET_HANDLE_STATE(__HANDLE__) do {                                            \
                                                      (__HANDLE__)->State = DAL_RTC_STATE_RESET; \
                                                      (__HANDLE__)->MspInitCallback = NULL;      \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;    \
                                                    } while(0U)
#else
#define __DAL_RTC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_RTC_STATE_RESET)
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

/**
  * @brief  Disable the write protection for RTC registers.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_WRITEPROTECTION_DISABLE(__HANDLE__) do {                                       \
                                                           (__HANDLE__)->Instance->WRPROT = 0xCAU;  \
                                                           (__HANDLE__)->Instance->WRPROT = 0x53U;  \
                                                         } while(0U)

/**
  * @brief  Enable the write protection for RTC registers.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_WRITEPROTECTION_ENABLE(__HANDLE__) do {                                       \
                                                          (__HANDLE__)->Instance->WRPROT = 0xFFU;  \
                                                        } while(0U)


/**
  * @brief  Enable the RTC ALARMA peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_ALARMA_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->CTRL |= (RTC_CTRL_ALRAEN))

/**
  * @brief  Disable the RTC ALARMA peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_ALARMA_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->CTRL &= ~(RTC_CTRL_ALRAEN))

/**
  * @brief  Enable the RTC ALARMB peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_ALARMB_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->CTRL |= (RTC_CTRL_ALRBEN))

/**
  * @brief  Disable the RTC ALARMB peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __DAL_RTC_ALARMB_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->CTRL &= ~(RTC_CTRL_ALRBEN))

/**
  * @brief  Enable the RTC Alarm interrupt.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg RTC_IT_ALRA: Alarm A interrupt
  *             @arg RTC_IT_ALRB: Alarm B interrupt
  * @retval None
  */
#define __DAL_RTC_ALARM_ENABLE_IT(__HANDLE__, __INTERRUPT__)          ((__HANDLE__)->Instance->CTRL |= (__INTERRUPT__))

/**
  * @brief  Disable the RTC Alarm interrupt.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg RTC_IT_ALRA: Alarm A interrupt
  *             @arg RTC_IT_ALRB: Alarm B interrupt
  * @retval None
  */
#define __DAL_RTC_ALARM_DISABLE_IT(__HANDLE__, __INTERRUPT__)         ((__HANDLE__)->Instance->CTRL &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has occurred or not.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt to check.
  *         This parameter can be:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  *            @arg RTC_IT_ALRB: Alarm B interrupt
  * @retval None
  */
#define __DAL_RTC_ALARM_GET_IT(__HANDLE__, __INTERRUPT__)           (((((__HANDLE__)->Instance->STS) & ((__INTERRUPT__) >> 4U)) != 0U) ? 1U : 0U)

/**
  * @brief  Get the selected RTC Alarm's flag status.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __FLAG__ specifies the RTC Alarm Flag to check.
  *         This parameter can be:
  *            @arg RTC_FLAG_ALRAF: Alarm A interrupt flag
  *            @arg RTC_FLAG_ALRAWF: Alarm A 'write allowed' flag
  *            @arg RTC_FLAG_ALRBF: Alarm B interrupt flag
  *            @arg RTC_FLAG_ALRBWF: Alarm B 'write allowed' flag
  * @retval None
  */
#define __DAL_RTC_ALARM_GET_FLAG(__HANDLE__, __FLAG__)                (((((__HANDLE__)->Instance->STS) & (__FLAG__)) != 0U) ? 1U : 0U)

/**
  * @brief  Clear the RTC Alarm's pending flags.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __FLAG__ specifies the RTC Alarm flag to be cleared.
  *          This parameter can be:
  *             @arg RTC_FLAG_ALRAF
  *             @arg RTC_FLAG_ALRBF
  * @retval None
  */
#define __DAL_RTC_ALARM_CLEAR_FLAG(__HANDLE__, __FLAG__)                  ((__HANDLE__)->Instance->STS) = (~((__FLAG__) | RTC_STS_INITEN)|((__HANDLE__)->Instance->STS & RTC_STS_INITEN))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has been enabled or not.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to check.
  *         This parameter can be:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  *            @arg RTC_IT_ALRB: Alarm B interrupt
  * @retval None
  */
#define __DAL_RTC_ALARM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (((((__HANDLE__)->Instance->CTRL) & (__INTERRUPT__)) != 0U) ? 1U : 0U)

/**
  * @brief  Enable interrupt on the RTC Alarm associated EINT line.
  * @retval None
  */
#define __DAL_RTC_ALARM_EINT_ENABLE_IT()            (EINT->IMASK |= RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Disable interrupt on the RTC Alarm associated EINT line.
  * @retval None
  */
#define __DAL_RTC_ALARM_EINT_DISABLE_IT()           (EINT->IMASK &= ~RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Enable event on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_ENABLE_EVENT()          (EINT->EMASK |= RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Disable event on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_DISABLE_EVENT()         (EINT->EMASK &= ~RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Enable falling edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_ENABLE_FALLING_EDGE()   (EINT->FTEN |= RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Disable falling edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_DISABLE_FALLING_EDGE()  (EINT->FTEN &= ~RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Enable rising edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_ENABLE_RISING_EDGE()    (EINT->RTEN |= RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Disable rising edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_DISABLE_RISING_EDGE()   (EINT->RTEN &= ~RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief  Enable rising & falling edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_ENABLE_RISING_FALLING_EDGE() do {                                             \
                                                               __DAL_RTC_ALARM_EINT_ENABLE_RISING_EDGE();  \
                                                               __DAL_RTC_ALARM_EINT_ENABLE_FALLING_EDGE(); \
                                                             } while(0U)

/**
  * @brief  Disable rising & falling edge trigger on the RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_DISABLE_RISING_FALLING_EDGE() do {                                              \
                                                                __DAL_RTC_ALARM_EINT_DISABLE_RISING_EDGE();  \
                                                                __DAL_RTC_ALARM_EINT_DISABLE_FALLING_EDGE(); \
                                                              } while(0U)

/**
  * @brief Check whether the RTC Alarm associated EINT line interrupt flag is set or not.
  * @retval Line Status.
  */
#define __DAL_RTC_ALARM_EINT_GET_FLAG()              (EINT->IPEND & RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief Clear the RTC Alarm associated EINT line flag.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_CLEAR_FLAG()            (EINT->IPEND = RTC_EINT_LINE_ALARM_EVENT)

/**
  * @brief Generate a Software interrupt on RTC Alarm associated EINT line.
  * @retval None.
  */
#define __DAL_RTC_ALARM_EINT_GENERATE_SWIT()         (EINT->SWINTE |= RTC_EINT_LINE_ALARM_EVENT)
/**
  * @}
  */

/* Include RTC DAL Extended module */
#include "apm32f4xx_dal_rtc_ex.h"

/* Exported functions --------------------------------------------------------*/

/** @addtogroup RTC_Exported_Functions
  * @{
  */

/** @addtogroup RTC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ****************************/
DAL_StatusTypeDef DAL_RTC_Init(RTC_HandleTypeDef *hrtc);
DAL_StatusTypeDef DAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void              DAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void              DAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_RTC_RegisterCallback(RTC_HandleTypeDef *hrtc, DAL_RTC_CallbackIDTypeDef CallbackID, pRTC_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_RTC_UnRegisterCallback(RTC_HandleTypeDef *hrtc, DAL_RTC_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group2
  * @{
  */
/* RTC Time and Date functions ************************************************/
DAL_StatusTypeDef DAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
DAL_StatusTypeDef DAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
DAL_StatusTypeDef DAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
DAL_StatusTypeDef DAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group3
  * @{
  */
/* RTC Alarm functions ********************************************************/
DAL_StatusTypeDef DAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
DAL_StatusTypeDef DAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
DAL_StatusTypeDef DAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
DAL_StatusTypeDef DAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void              DAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
DAL_StatusTypeDef DAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void              DAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group4
  * @{
  */
/* Peripheral Control functions ***********************************************/
DAL_StatusTypeDef   DAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);

/* RTC Daylight Saving Time functions *****************************************/
void              DAL_RTC_DST_Add1Hour(RTC_HandleTypeDef *hrtc);
void              DAL_RTC_DST_Sub1Hour(RTC_HandleTypeDef *hrtc);
void              DAL_RTC_DST_SetStoreOperation(RTC_HandleTypeDef *hrtc);
void              DAL_RTC_DST_ClearStoreOperation(RTC_HandleTypeDef *hrtc);
uint32_t          DAL_RTC_DST_ReadStoreOperation(RTC_HandleTypeDef *hrtc);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group5
  * @{
  */
/* Peripheral State functions *************************************************/
DAL_RTCStateTypeDef DAL_RTC_GetState(RTC_HandleTypeDef *hrtc);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/** @defgroup RTC_Private_Constants RTC Private Constants
  * @{
  */
/* Masks Definition */
#define RTC_TIME_RESERVED_MASK    ((uint32_t)(RTC_TIME_HRT  | RTC_TIME_HRU  | \
                                            RTC_TIME_MINT | RTC_TIME_MINU | \
                                            RTC_TIME_SECT  | RTC_TIME_SECU  | \
                                            RTC_TIME_TIMEFCFG))
#define RTC_DATE_RESERVED_MASK    ((uint32_t)(RTC_DATE_YRT | RTC_DATE_YRU | \
                                            RTC_DATE_MONT | RTC_DATE_MONU | \
                                            RTC_DATE_DAYT | RTC_DATE_DAYU | \
                                            RTC_DATE_WEEKSEL))
#define RTC_INIT_MASK           0xFFFFFFFFU
#define RTC_RSF_MASK            ((uint32_t)~(RTC_STS_INITEN | RTC_STS_RSFLG))
#define RTC_FLAGS_MASK          ((uint32_t)(RTC_FLAG_INITF   | RTC_FLAG_INITS  | \
                                            RTC_FLAG_ALRAF   | RTC_FLAG_ALRAWF | \
                                            RTC_FLAG_ALRBF   | RTC_FLAG_ALRBWF | \
                                            RTC_FLAG_WUTF    | RTC_FLAG_WUTWF  | \
                                            RTC_FLAG_RECALPF | RTC_FLAG_SHPF   | \
                                            RTC_FLAG_TSF     | RTC_FLAG_TSOVF  | \
                                            RTC_FLAG_RSF     | RTC_TAMPER_FLAGS_MASK))

#define RTC_TIMEOUT_VALUE       1000U

#define RTC_EINT_LINE_ALARM_EVENT  EINT_IMASK_IMASK17  /*!< External interrupt line 17 Connected to the RTC Alarm event */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/** @defgroup RTC_Private_Macros RTC Private Macros
  * @{
  */

/** @defgroup RTC_IS_RTC_Definitions RTC Private macros to check input parameters
  * @{
  */
#define IS_RTC_HOUR_FORMAT(FORMAT)     (((FORMAT) == RTC_HOURFORMAT_12) || \
                                        ((FORMAT) == RTC_HOURFORMAT_24))

#define IS_RTC_OUTPUT(OUTPUT) (((OUTPUT) == RTC_OUTPUT_DISABLE) || \
                               ((OUTPUT) == RTC_OUTPUT_ALARMA)  || \
                               ((OUTPUT) == RTC_OUTPUT_ALARMB)  || \
                               ((OUTPUT) == RTC_OUTPUT_WAKEUP))

#define IS_RTC_OUTPUT_POL(POL) (((POL) == RTC_OUTPUT_POLARITY_HIGH) || \
                                ((POL) == RTC_OUTPUT_POLARITY_LOW))

#define IS_RTC_OUTPUT_TYPE(TYPE) (((TYPE) == RTC_OUTPUT_TYPE_OPENDRAIN) || \
                                  ((TYPE) == RTC_OUTPUT_TYPE_PUSHPULL))

#define IS_RTC_ASYNCH_PREDIV(PREDIV)   ((PREDIV) <= 0x7FU)
#define IS_RTC_SYNCH_PREDIV(PREDIV)    ((PREDIV) <= 0x7FFFU)

#define IS_RTC_HOUR12(HOUR)            (((HOUR) > 0U) && ((HOUR) <= 12U))
#define IS_RTC_HOUR24(HOUR)            ((HOUR) <= 23U)
#define IS_RTC_MINUTES(MINUTES)        ((MINUTES) <= 59U)
#define IS_RTC_SECONDS(SECONDS)        ((SECONDS) <= 59U)

#define IS_RTC_HOURFORMAT12(PM)  (((PM) == RTC_HOURFORMAT12_AM) || \
                                  ((PM) == RTC_HOURFORMAT12_PM))

#define IS_RTC_DAYLIGHT_SAVING(SAVE) (((SAVE) == RTC_DAYLIGHTSAVING_SUB1H) || \
                                      ((SAVE) == RTC_DAYLIGHTSAVING_ADD1H) || \
                                      ((SAVE) == RTC_DAYLIGHTSAVING_NONE))

#define IS_RTC_STORE_OPERATION(OPERATION) (((OPERATION) == RTC_STOREOPERATION_RESET) || \
                                           ((OPERATION) == RTC_STOREOPERATION_SET))

#define IS_RTC_FORMAT(FORMAT) (((FORMAT) == RTC_FORMAT_BIN) || ((FORMAT) == RTC_FORMAT_BCD))

#define IS_RTC_YEAR(YEAR)              ((YEAR) <= 99U)
#define IS_RTC_MONTH(MONTH)            (((MONTH) >= 1U) && ((MONTH) <= 12U))
#define IS_RTC_DATE(DATE)              (((DATE) >= 1U) && ((DATE) <= 31U))

#define IS_RTC_WEEKDAY(WEEKDAY) (((WEEKDAY) == RTC_WEEKDAY_MONDAY)    || \
                                 ((WEEKDAY) == RTC_WEEKDAY_TUESDAY)   || \
                                 ((WEEKDAY) == RTC_WEEKDAY_WEDNESDAY) || \
                                 ((WEEKDAY) == RTC_WEEKDAY_THURSDAY)  || \
                                 ((WEEKDAY) == RTC_WEEKDAY_FRIDAY)    || \
                                 ((WEEKDAY) == RTC_WEEKDAY_SATURDAY)  || \
                                 ((WEEKDAY) == RTC_WEEKDAY_SUNDAY))

#define IS_RTC_ALARM_DATE_WEEKDAY_DATE(DATE) (((DATE) > 0U) && ((DATE) <= 31U))

#define IS_RTC_ALARM_DATE_WEEKDAY_WEEKDAY(WEEKDAY) (((WEEKDAY) == RTC_WEEKDAY_MONDAY)    || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_TUESDAY)   || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_WEDNESDAY) || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_THURSDAY)  || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_FRIDAY)    || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_SATURDAY)  || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_SUNDAY))

#define IS_RTC_ALARM_DATE_WEEKDAY_SEL(SEL) (((SEL) == RTC_ALARMDATEWEEKDAYSEL_DATE) || \
                                            ((SEL) == RTC_ALARMDATEWEEKDAYSEL_WEEKDAY))

#define IS_RTC_ALARM_MASK(MASK)  (((MASK) & ((uint32_t)~RTC_ALARMMASK_ALL)) == 0U)

#define IS_RTC_ALARM(ALARM)      (((ALARM) == RTC_ALARM_A) || ((ALARM) == RTC_ALARM_B))

#define IS_RTC_ALARM_SUB_SECOND_VALUE(VALUE) ((VALUE) <= RTC_ALRMASS_SUBSEC)

#define IS_RTC_ALARM_SUB_SECOND_MASK(MASK)   (((MASK) == RTC_ALARMSUBSECONDMASK_ALL)     || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_1)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_2)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_3)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_4)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_5)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_6)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_7)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_8)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_9)  || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_10) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_11) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_12) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_13) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14)    || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_NONE))
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup RTC_Private_Functions RTC Private Functions
  * @{
  */
DAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);
DAL_StatusTypeDef  RTC_ExitInitMode(RTC_HandleTypeDef *hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t number);
uint8_t            RTC_Bcd2ToByte(uint8_t number);
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

#endif /* APM32F4xx_DAL_RTC_H */
