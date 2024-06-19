/**
  *
  * @file    apm32f4xx_ddl_rtc.h
  * @brief   Header file of RTC DDL module.
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
#ifndef APM32F4xx_DDL_RTC_H
#define APM32F4xx_DDL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(RTC)

/** @defgroup RTC_DDL RTC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup RTC_DDL_Private_Constants RTC Private Constants
  * @{
  */
/* Masks Definition */
#define RTC_INIT_MASK                 0xFFFFFFFFU
#define RTC_RSF_MASK                  ((uint32_t)~(RTC_STS_INITEN | RTC_STS_RSFLG))

/* Write protection defines */
#define RTC_WRITE_PROTECTION_DISABLE  ((uint8_t)0xFFU)
#define RTC_WRITE_PROTECTION_ENABLE_1 ((uint8_t)0xCAU)
#define RTC_WRITE_PROTECTION_ENABLE_2 ((uint8_t)0x53U)

/* Defines used to combine date & time */
#define RTC_OFFSET_WEEKDAY            24U
#define RTC_OFFSET_DAY                16U
#define RTC_OFFSET_MONTH              8U
#define RTC_OFFSET_HOUR               16U
#define RTC_OFFSET_MINUTE             8U

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RTC_DDL_Private_Macros RTC Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RTC_DDL_ES_INIT RTC Exported Init structure
  * @{
  */

/**
  * @brief  RTC Init structures definition
  */
typedef struct
{
  uint32_t HourFormat;   /*!< Specifies the RTC Hours Format.
                              This parameter can be a value of @ref RTC_DDL_EC_HOURFORMAT

                              This feature can be modified afterwards using unitary function
                              @ref DDL_RTC_SetHourFormat(). */

  uint32_t AsynchPrescaler; /*!< Specifies the RTC Asynchronous Predivider value.
                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x7F

                              This feature can be modified afterwards using unitary function
                              @ref DDL_RTC_SetAsynchPrescaler(). */

  uint32_t SynchPrescaler;  /*!< Specifies the RTC Synchronous Predivider value.
                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x7FFF

                              This feature can be modified afterwards using unitary function
                              @ref DDL_RTC_SetSynchPrescaler(). */
} DDL_RTC_InitTypeDef;

/**
  * @brief  RTC Time structure definition
  */
typedef struct
{
  uint32_t TimeFormat; /*!< Specifies the RTC AM/PM Time.
                            This parameter can be a value of @ref RTC_DDL_EC_TIME_FORMAT

                            This feature can be modified afterwards using unitary function @ref DDL_RTC_TIME_SetFormat(). */

  uint8_t Hours;       /*!< Specifies the RTC Time Hours.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the @ref DDL_RTC_TIME_FORMAT_PM is selected.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the @ref DDL_RTC_TIME_FORMAT_AM_OR_24 is selected.

                            This feature can be modified afterwards using unitary function @ref DDL_RTC_TIME_SetHour(). */

  uint8_t Minutes;     /*!< Specifies the RTC Time Minutes.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 59

                            This feature can be modified afterwards using unitary function @ref DDL_RTC_TIME_SetMinute(). */

  uint8_t Seconds;     /*!< Specifies the RTC Time Seconds.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 59

                            This feature can be modified afterwards using unitary function @ref DDL_RTC_TIME_SetSecond(). */
} DDL_RTC_TimeTypeDef;

/**
  * @brief  RTC Date structure definition
  */
typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay.
                         This parameter can be a value of @ref RTC_DDL_EC_WEEKDAY

                         This feature can be modified afterwards using unitary function @ref DDL_RTC_DATE_SetWeekDay(). */

  uint8_t Month;    /*!< Specifies the RTC Date Month.
                         This parameter can be a value of @ref RTC_DDL_EC_MONTH

                         This feature can be modified afterwards using unitary function @ref DDL_RTC_DATE_SetMonth(). */

  uint8_t Day;      /*!< Specifies the RTC Date Day.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31

                         This feature can be modified afterwards using unitary function @ref DDL_RTC_DATE_SetDay(). */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99

                         This feature can be modified afterwards using unitary function @ref DDL_RTC_DATE_SetYear(). */
} DDL_RTC_DateTypeDef;

/**
  * @brief  RTC Alarm structure definition
  */
typedef struct
{
  DDL_RTC_TimeTypeDef AlarmTime;  /*!< Specifies the RTC Alarm Time members. */

  uint32_t AlarmMask;            /*!< Specifies the RTC Alarm Masks.
                                      This parameter can be a value of @ref RTC_DDL_EC_ALMA_MASK for ALARM A or @ref RTC_DDL_EC_ALMB_MASK for ALARM B.

                                      This feature can be modified afterwards using unitary function @ref DDL_RTC_ALMA_SetMask() for ALARM A
                                      or @ref DDL_RTC_ALMB_SetMask() for ALARM B.
                                 */

  uint32_t AlarmDateWeekDaySel;  /*!< Specifies the RTC Alarm is on day or WeekDay.
                                      This parameter can be a value of @ref RTC_DDL_EC_ALMA_WEEKDAY_SELECTION for ALARM A or @ref RTC_DDL_EC_ALMB_WEEKDAY_SELECTION for ALARM B

                                      This feature can be modified afterwards using unitary function @ref DDL_RTC_ALMA_EnableWeekday() or @ref DDL_RTC_ALMA_DisableWeekday()
                                      for ALARM A or @ref DDL_RTC_ALMB_EnableWeekday() or @ref DDL_RTC_ALMB_DisableWeekday() for ALARM B
                                 */

  uint8_t AlarmDateWeekDay;      /*!< Specifies the RTC Alarm Day/WeekDay.
                                      If AlarmDateWeekDaySel set to day, this parameter  must be a number between Min_Data = 1 and Max_Data = 31.

                                      This feature can be modified afterwards using unitary function @ref DDL_RTC_ALMA_SetDay()
                                      for ALARM A or @ref DDL_RTC_ALMB_SetDay() for ALARM B.

                                      If AlarmDateWeekDaySel set to Weekday, this parameter can be a value of @ref RTC_DDL_EC_WEEKDAY.

                                      This feature can be modified afterwards using unitary function @ref DDL_RTC_ALMA_SetWeekDay()
                                      for ALARM A or @ref DDL_RTC_ALMB_SetWeekDay() for ALARM B.
                                 */
} DDL_RTC_AlarmTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RTC_DDL_Exported_Constants RTC Exported Constants
  * @{
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RTC_DDL_EC_FORMAT FORMAT
  * @{
  */
#define DDL_RTC_FORMAT_BIN                  0x00000000U /*!< Binary data format */
#define DDL_RTC_FORMAT_BCD                  0x00000001U /*!< BCD data format */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMA_WEEKDAY_SELECTION RTC Alarm A Date WeekDay
  * @{
  */
#define DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE    0x00000000U             /*!< Alarm A Date is selected */
#define DDL_RTC_ALMA_DATEWEEKDAYSEL_WEEKDAY RTC_ALRMA_WEEKSEL        /*!< Alarm A WeekDay is selected */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMB_WEEKDAY_SELECTION RTC Alarm B Date WeekDay
  * @{
  */
#define DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE    0x00000000U             /*!< Alarm B Date is selected */
#define DDL_RTC_ALMB_DATEWEEKDAYSEL_WEEKDAY RTC_ALRMB_WEEKSEL        /*!< Alarm B WeekDay is selected */
/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

/** @defgroup RTC_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_RTC_ReadReg function
  * @{
  */
#define DDL_RTC_STS_RCALPFLG                 RTC_STS_RCALPFLG
#if defined(RTC_TAMPER2_SUPPORT)
#define DDL_RTC_STS_TP2FLG                  RTC_STS_TP2FLG
#endif /* RTC_TAMPER2_SUPPORT */
#define DDL_RTC_STS_TP1FLG                  RTC_STS_TP1FLG
#define DDL_RTC_STS_TSOVRFLG                   RTC_STS_TSOVRFLG
#define DDL_RTC_STS_TSFLG                     RTC_STS_TSFLG
#define DDL_RTC_STS_WUTFLG                    RTC_STS_WUTFLG
#define DDL_RTC_STS_ALRBFLG                   RTC_STS_ALRBFLG
#define DDL_RTC_STS_ALRAFLG                   RTC_STS_ALRAFLG
#define DDL_RTC_STS_RINITFLG                   RTC_STS_RINITFLG
#define DDL_RTC_STS_RSFLG                     RTC_STS_RSFLG
#define DDL_RTC_STS_INITSFLG                   RTC_STS_INITSFLG
#define DDL_RTC_STS_SOPFLG                    RTC_STS_SOPFLG
#define DDL_RTC_STS_WUTWFLG                   RTC_STS_WUTWFLG
#define DDL_RTC_STS_ALRBWFLG                  RTC_STS_ALRBWFLG
#define DDL_RTC_STS_ALRAWFLG                  RTC_STS_ALRAWFLG
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_RTC_ReadReg and  DDL_RTC_WriteReg functions
  * @{
  */
#define DDL_RTC_CTRL_TSIEN                     RTC_CTRL_TSIEN
#define DDL_RTC_CTRL_WUTIEN                    RTC_CTRL_WUTIEN
#define DDL_RTC_CTRL_ALRBIEN                   RTC_CTRL_ALRBIEN
#define DDL_RTC_CTRL_ALRAIEN                   RTC_CTRL_ALRAIEN
#define DDL_RTC_TACFG_TPIEN               RTC_TACFG_TPIEN
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_WEEKDAY  WEEK DAY
  * @{
  */
#define DDL_RTC_WEEKDAY_MONDAY              ((uint8_t)0x01U) /*!< Monday    */
#define DDL_RTC_WEEKDAY_TUESDAY             ((uint8_t)0x02U) /*!< Tuesday   */
#define DDL_RTC_WEEKDAY_WEDNESDAY           ((uint8_t)0x03U) /*!< Wednesday */
#define DDL_RTC_WEEKDAY_THURSDAY            ((uint8_t)0x04U) /*!< Thrusday  */
#define DDL_RTC_WEEKDAY_FRIDAY              ((uint8_t)0x05U) /*!< Friday    */
#define DDL_RTC_WEEKDAY_SATURDAY            ((uint8_t)0x06U) /*!< Saturday  */
#define DDL_RTC_WEEKDAY_SUNDAY              ((uint8_t)0x07U) /*!< Sunday    */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_MONTH  MONTH
  * @{
  */
#define DDL_RTC_MONTH_JANUARY               ((uint8_t)0x01U)  /*!< January   */
#define DDL_RTC_MONTH_FEBRUARY              ((uint8_t)0x02U)  /*!< February  */
#define DDL_RTC_MONTH_MARCH                 ((uint8_t)0x03U)  /*!< March     */
#define DDL_RTC_MONTH_APRIL                 ((uint8_t)0x04U)  /*!< April     */
#define DDL_RTC_MONTH_MAY                   ((uint8_t)0x05U)  /*!< May       */
#define DDL_RTC_MONTH_JUNE                  ((uint8_t)0x06U)  /*!< June      */
#define DDL_RTC_MONTH_JULY                  ((uint8_t)0x07U)  /*!< July      */
#define DDL_RTC_MONTH_AUGUST                ((uint8_t)0x08U)  /*!< August    */
#define DDL_RTC_MONTH_SEPTEMBER             ((uint8_t)0x09U)  /*!< September */
#define DDL_RTC_MONTH_OCTOBER               ((uint8_t)0x10U)  /*!< October   */
#define DDL_RTC_MONTH_NOVEMBER              ((uint8_t)0x11U)  /*!< November  */
#define DDL_RTC_MONTH_DECEMBER              ((uint8_t)0x12U)  /*!< December  */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_HOURFORMAT  HOUR FORMAT
  * @{
  */
#define DDL_RTC_HOURFORMAT_24HOUR           0x00000000U           /*!< 24 hour/day format */
#define DDL_RTC_HOURFORMAT_AMPM             RTC_CTRL_TIMEFCFG            /*!< AM/PM hour format */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALARMOUT  ALARM OUTPUT
  * @{
  */
#define DDL_RTC_ALARMOUT_DISABLE            0x00000000U             /*!< Output disabled */
#define DDL_RTC_ALARMOUT_ALMA               RTC_CTRL_OUTSEL_0           /*!< Alarm A output enabled */
#define DDL_RTC_ALARMOUT_ALMB               RTC_CTRL_OUTSEL_1           /*!< Alarm B output enabled */
#define DDL_RTC_ALARMOUT_WAKEUP             RTC_CTRL_OUTSEL             /*!< Wakeup output enabled */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALARM_OUTPUTTYPE  ALARM OUTPUT TYPE
  * @{
  */
#define DDL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN  0x00000000U                          /*!< RTC_ALARM, when mapped on PC13, is open-drain output */
#define DDL_RTC_ALARM_OUTPUTTYPE_PUSHPULL   RTC_TACFG_ALRMOT /*!< RTC_ALARM, when mapped on PC13, is push-pull output */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_OUTPUTPOLARITY_PIN  OUTPUT POLARITY PIN
  * @{
  */
#define DDL_RTC_OUTPUTPOLARITY_PIN_HIGH     0x00000000U           /*!< Pin is high when ALRAF/ALRBF/WUTF is asserted (depending on OSEL)*/
#define DDL_RTC_OUTPUTPOLARITY_PIN_LOW      RTC_CTRL_POLCFG            /*!< Pin is low when ALRAF/ALRBF/WUTF is asserted (depending on OSEL) */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TIME_FORMAT TIME FORMAT
  * @{
  */
#define DDL_RTC_TIME_FORMAT_AM_OR_24        0x00000000U           /*!< AM or 24-hour format */
#define DDL_RTC_TIME_FORMAT_PM              RTC_TIME_TIMEFCFG             /*!< PM */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_SHIFT_SECOND  SHIFT SECOND
  * @{
  */
#define DDL_RTC_SHIFT_SECOND_DELAY          0x00000000U           /* Delay (seconds) = SUBFS / (PREDIV_S + 1) */
#define DDL_RTC_SHIFT_SECOND_ADVANCE        RTC_SHIFT_ADD1SECEN      /* Advance (seconds) = (1 - (SUBFS / (PREDIV_S + 1))) */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMA_MASK  ALARMA MASK
  * @{
  */
#define DDL_RTC_ALMA_MASK_NONE              0x00000000U             /*!< No masks applied on Alarm A*/
#define DDL_RTC_ALMA_MASK_DATEWEEKDAY       RTC_ALRMA_DATEMEN         /*!< Date/day do not care in Alarm A comparison */
#define DDL_RTC_ALMA_MASK_HOURS             RTC_ALRMA_HRMEN         /*!< Hours do not care in Alarm A comparison */
#define DDL_RTC_ALMA_MASK_MINUTES           RTC_ALRMA_MINMEN         /*!< Minutes do not care in Alarm A comparison */
#define DDL_RTC_ALMA_MASK_SECONDS           RTC_ALRMA_SECMEN         /*!< Seconds do not care in Alarm A comparison */
#define DDL_RTC_ALMA_MASK_ALL               (RTC_ALRMA_DATEMEN | RTC_ALRMA_HRMEN | RTC_ALRMA_MINMEN | RTC_ALRMA_SECMEN) /*!< Masks all */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMA_TIME_FORMAT  ALARMA TIME FORMAT
  * @{
  */
#define DDL_RTC_ALMA_TIME_FORMAT_AM         0x00000000U           /*!< AM or 24-hour format */
#define DDL_RTC_ALMA_TIME_FORMAT_PM         RTC_ALRMA_TIMEFCFG         /*!< PM */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMB_MASK  ALARMB MASK
  * @{
  */
#define DDL_RTC_ALMB_MASK_NONE              0x00000000U             /*!< No masks applied on Alarm B                */
#define DDL_RTC_ALMB_MASK_DATEWEEKDAY       RTC_ALRMB_DATEMEN         /*!< Date/day do not care in Alarm B comparison */
#define DDL_RTC_ALMB_MASK_HOURS             RTC_ALRMB_HRMEN         /*!< Hours do not care in Alarm B comparison    */
#define DDL_RTC_ALMB_MASK_MINUTES           RTC_ALRMB_MINMEN         /*!< Minutes do not care in Alarm B comparison  */
#define DDL_RTC_ALMB_MASK_SECONDS           RTC_ALRMB_SECMEN         /*!< Seconds do not care in Alarm B comparison  */
#define DDL_RTC_ALMB_MASK_ALL               (RTC_ALRMB_DATEMEN | RTC_ALRMB_HRMEN | RTC_ALRMB_MINMEN | RTC_ALRMB_SECMEN) /*!< Masks all */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_ALMB_TIME_FORMAT  ALARMB TIME FORMAT
  * @{
  */
#define DDL_RTC_ALMB_TIME_FORMAT_AM         0x00000000U           /*!< AM or 24-hour format */
#define DDL_RTC_ALMB_TIME_FORMAT_PM         RTC_ALRMB_TIMEFCFG         /*!< PM */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TIMESTAMP_EDGE  TIMESTAMP EDGE
  * @{
  */
#define DDL_RTC_TIMESTAMP_EDGE_RISING       0x00000000U           /*!< RTC_TS input rising edge generates a time-stamp event */
#define DDL_RTC_TIMESTAMP_EDGE_FALLING      RTC_CTRL_TSETECFG         /*!< RTC_TS input falling edge generates a time-stamp even */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TS_TIME_FORMAT  TIMESTAMP TIME FORMAT
  * @{
  */
#define DDL_RTC_TS_TIME_FORMAT_AM           0x00000000U           /*!< AM or 24-hour format */
#define DDL_RTC_TS_TIME_FORMAT_PM           RTC_TSTIME_TIMEFCFG           /*!< PM */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMPER  TAMPER
  * @{
  */
#define DDL_RTC_TAMPER_1                    RTC_TACFG_TP1EN /*!< RTC_TAMP1 input detection */
#if defined(RTC_TAMPER2_SUPPORT)
#define DDL_RTC_TAMPER_2                    RTC_TACFG_TP2EN /*!< RTC_TAMP2 input detection */
#endif /* RTC_TAMPER2_SUPPORT */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMPER_DURATION  TAMPER DURATION
  * @{
  */
#define DDL_RTC_TAMPER_DURATION_1RTCCLK     0x00000000U                             /*!< Tamper pins are pre-charged before sampling during 1 RTCCLK cycle  */
#define DDL_RTC_TAMPER_DURATION_2RTCCLK     RTC_TACFG_TPPRDUSEL_0  /*!< Tamper pins are pre-charged before sampling during 2 RTCCLK cycles */
#define DDL_RTC_TAMPER_DURATION_4RTCCLK     RTC_TACFG_TPPRDUSEL_1  /*!< Tamper pins are pre-charged before sampling during 4 RTCCLK cycles */
#define DDL_RTC_TAMPER_DURATION_8RTCCLK     RTC_TACFG_TPPRDUSEL    /*!< Tamper pins are pre-charged before sampling during 8 RTCCLK cycles */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMPER_FILTER  TAMPER FILTER
  * @{
  */
#define DDL_RTC_TAMPER_FILTER_DISABLE       0x00000000U                              /*!< Tamper filter is disabled */
#define DDL_RTC_TAMPER_FILTER_2SAMPLE       RTC_TACFG_TPFCSEL_0    /*!< Tamper is activated after 2 consecutive samples at the active level */
#define DDL_RTC_TAMPER_FILTER_4SAMPLE       RTC_TACFG_TPFCSEL_1    /*!< Tamper is activated after 4 consecutive samples at the active level */
#define DDL_RTC_TAMPER_FILTER_8SAMPLE       RTC_TACFG_TPFCSEL      /*!< Tamper is activated after 8 consecutive samples at the active level. */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMPER_SAMPLFREQDIV  TAMPER SAMPLING FREQUENCY DIVIDER
  * @{
  */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_32768   0x00000000U                                                      /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 32768 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_16384   RTC_TACFG_TPSFSEL_0                           /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 16384 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_8192    RTC_TACFG_TPSFSEL_1                           /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 8192 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_4096    (RTC_TACFG_TPSFSEL_1 | RTC_TACFG_TPSFSEL_0) /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 4096 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_2048    RTC_TACFG_TPSFSEL_2                           /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 2048 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_1024    (RTC_TACFG_TPSFSEL_2 | RTC_TACFG_TPSFSEL_0) /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 1024 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_512     (RTC_TACFG_TPSFSEL_2 | RTC_TACFG_TPSFSEL_1) /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 512 */
#define DDL_RTC_TAMPER_SAMPLFREQDIV_256     RTC_TACFG_TPSFSEL                             /*!< Each of the tamper inputs are sampled with a frequency =  RTCCLK / 256 */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMPER_ACTIVELEVEL  TAMPER ACTIVE LEVEL
  * @{
  */
#define DDL_RTC_TAMPER_ACTIVELEVEL_TAMP1    RTC_TACFG_TP1ALCFG /*!< RTC_TAMP1 input falling edge (if TAMPFLT = 00) or staying high (if TAMPFLT != 00) triggers a tamper detection event */
#if defined(RTC_TAMPER2_SUPPORT)
#define DDL_RTC_TAMPER_ACTIVELEVEL_TAMP2    RTC_TACFG_TP2ALCFG /*!< RTC_TAMP2 input falling edge (if TAMPFLT = 00) or staying high (if TAMPFLT != 00) triggers a tamper detection event */
#endif /* RTC_TAMPER2_SUPPORT */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_WAKEUPCLOCK_DIV  WAKEUP CLOCK DIV
  * @{
  */
#define DDL_RTC_WAKEUPCLOCK_DIV_16          0x00000000U                           /*!< RTC/16 clock is selected */
#define DDL_RTC_WAKEUPCLOCK_DIV_8           (RTC_CTRL_WUCLKSEL_0)                    /*!< RTC/8 clock is selected */
#define DDL_RTC_WAKEUPCLOCK_DIV_4           (RTC_CTRL_WUCLKSEL_1)                    /*!< RTC/4 clock is selected */
#define DDL_RTC_WAKEUPCLOCK_DIV_2           (RTC_CTRL_WUCLKSEL_1 | RTC_CTRL_WUCLKSEL_0) /*!< RTC/2 clock is selected */
#define DDL_RTC_WAKEUPCLOCK_CKSPRE          (RTC_CTRL_WUCLKSEL_2)                    /*!< ck_spre (usually 1 Hz) clock is selected */
#define DDL_RTC_WAKEUPCLOCK_CKSPRE_WUT      (RTC_CTRL_WUCLKSEL_2 | RTC_CTRL_WUCLKSEL_1) /*!< ck_spre (usually 1 Hz) clock is selected and 2exp16 is added to the WUT counter value*/
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_BKP  BACKUP
  * @{
  */
#define DDL_RTC_BKP_DR0                     0x00000000U
#define DDL_RTC_BKP_DR1                     0x00000001U
#define DDL_RTC_BKP_DR2                     0x00000002U
#define DDL_RTC_BKP_DR3                     0x00000003U
#define DDL_RTC_BKP_DR4                     0x00000004U
#define DDL_RTC_BKP_DR5                     0x00000005U
#define DDL_RTC_BKP_DR6                     0x00000006U
#define DDL_RTC_BKP_DR7                     0x00000007U
#define DDL_RTC_BKP_DR8                     0x00000008U
#define DDL_RTC_BKP_DR9                     0x00000009U
#define DDL_RTC_BKP_DR10                    0x0000000AU
#define DDL_RTC_BKP_DR11                    0x0000000BU
#define DDL_RTC_BKP_DR12                    0x0000000CU
#define DDL_RTC_BKP_DR13                    0x0000000DU
#define DDL_RTC_BKP_DR14                    0x0000000EU
#define DDL_RTC_BKP_DR15                    0x0000000FU
#define DDL_RTC_BKP_DR16                    0x00000010U
#define DDL_RTC_BKP_DR17                    0x00000011U
#define DDL_RTC_BKP_DR18                    0x00000012U
#define DDL_RTC_BKP_DR19                    0x00000013U
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_CALIB_OUTPUT  Calibration output
  * @{
  */
#define DDL_RTC_CALIB_OUTPUT_NONE           0x00000000U                 /*!< Calibration output disabled */
#define DDL_RTC_CALIB_OUTPUT_1HZ            (RTC_CTRL_CALOEN | RTC_CTRL_CALOSEL) /*!< Calibration output is 1 Hz */
#define DDL_RTC_CALIB_OUTPUT_512HZ          (RTC_CTRL_CALOEN)                /*!< Calibration output is 512 Hz */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_CALIB_SIGN Coarse digital calibration sign
  * @{
  */
#define DDL_RTC_CALIB_SIGN_POSITIVE         0x00000000U           /*!< Positive calibration: calendar update frequency is increased */
#define DDL_RTC_CALIB_SIGN_NEGATIVE         RTC_DCAL_DCALCFG        /*!< Negative calibration: calendar update frequency is decreased */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_CALIB_INSERTPULSE  Calibration pulse insertion
  * @{
  */
#define DDL_RTC_CALIB_INSERTPULSE_NONE      0x00000000U           /*!< No RTCCLK pulses are added */
#define DDL_RTC_CALIB_INSERTPULSE_SET       RTC_CAL_ICALFEN         /*!< One RTCCLK pulse is effectively inserted every 2exp11 pulses (frequency increased by 488.5 ppm) */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_CALIB_PERIOD  Calibration period
  * @{
  */
#define DDL_RTC_CALIB_PERIOD_32SEC          0x00000000U           /*!< Use a 32-second calibration cycle period */
#define DDL_RTC_CALIB_PERIOD_16SEC          RTC_CAL_CAL16CFG       /*!< Use a 16-second calibration cycle period */
#define DDL_RTC_CALIB_PERIOD_8SEC           RTC_CAL_CAL8CFG        /*!< Use a 8-second calibration cycle period */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TSINSEL  TIMESTAMP mapping
  * @{
  */
#define DDL_RTC_TimeStampPin_Default        0x00000000U           /*!< Use RTC_AF1 as TIMESTAMP */
#if defined(RTC_AF2_SUPPORT)
#define DDL_RTC_TimeStampPin_Pos1           RTC_TACFG_TSMSEL     /*!< Use RTC_AF2 as TIMESTAMP */
#endif /* RTC_AF2_SUPPORT */
/**
  * @}
  */

/** @defgroup RTC_DDL_EC_TAMP1INSEL  TAMPER1 mapping
  * @{
  */
#define DDL_RTC_TamperPin_Default           0x00000000U           /*!< Use RTC_AF1 as TAMPER1 */
#if defined(RTC_AF2_SUPPORT)
#define DDL_RTC_TamperPin_Pos1              RTC_TACFG_TP1MSEL  /*!< Use RTC_AF2 as TAMPER1 */
#endif /* RTC_AF2_SUPPORT */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RTC_DDL_Exported_Macros RTC Exported Macros
  * @{
  */

/** @defgroup RTC_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RTC register
  * @param  __INSTANCE__ RTC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_RTC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RTC register
  * @param  __INSTANCE__ RTC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_RTC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup RTC_DDL_EM_Convert Convert helper Macros
  * @{
  */

/**
  * @brief  Helper macro to convert a value from 2 digit decimal format to BCD format
  * @param  __VALUE__ Byte to be converted
  * @retval Converted byte
  */
#define __DDL_RTC_CONVERT_BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))

/**
  * @brief  Helper macro to convert a value from BCD format to 2 digit decimal format
  * @param  __VALUE__ BCD value to be converted
  * @retval Converted byte
  */
#define __DDL_RTC_CONVERT_BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

/**
  * @}
  */

/** @defgroup RTC_DDL_EM_Date Date helper Macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve weekday.
  * @param  __RTC_DATE__ Date returned by @ref  DDL_RTC_DATE_Get function.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  */
#define __DDL_RTC_GET_WEEKDAY(__RTC_DATE__) (((__RTC_DATE__) >> RTC_OFFSET_WEEKDAY) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Year in BCD format
  * @param  __RTC_DATE__ Value returned by @ref  DDL_RTC_DATE_Get
  * @retval Year in BCD format (0x00 . . . 0x99)
  */
#define __DDL_RTC_GET_YEAR(__RTC_DATE__) ((__RTC_DATE__) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Month in BCD format
  * @param  __RTC_DATE__ Value returned by @ref  DDL_RTC_DATE_Get
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_MONTH_JANUARY
  *         @arg @ref DDL_RTC_MONTH_FEBRUARY
  *         @arg @ref DDL_RTC_MONTH_MARCH
  *         @arg @ref DDL_RTC_MONTH_APRIL
  *         @arg @ref DDL_RTC_MONTH_MAY
  *         @arg @ref DDL_RTC_MONTH_JUNE
  *         @arg @ref DDL_RTC_MONTH_JULY
  *         @arg @ref DDL_RTC_MONTH_AUGUST
  *         @arg @ref DDL_RTC_MONTH_SEPTEMBER
  *         @arg @ref DDL_RTC_MONTH_OCTOBER
  *         @arg @ref DDL_RTC_MONTH_NOVEMBER
  *         @arg @ref DDL_RTC_MONTH_DECEMBER
  */
#define __DDL_RTC_GET_MONTH(__RTC_DATE__) (((__RTC_DATE__) >>RTC_OFFSET_MONTH) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Day in BCD format
  * @param  __RTC_DATE__ Value returned by @ref  DDL_RTC_DATE_Get
  * @retval Day in BCD format (0x01 . . . 0x31)
  */
#define __DDL_RTC_GET_DAY(__RTC_DATE__) (((__RTC_DATE__) >>RTC_OFFSET_DAY) & 0x000000FFU)

/**
  * @}
  */

/** @defgroup RTC_DDL_EM_Time Time helper Macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve hour in BCD format
  * @param  __RTC_TIME__ RTC time returned by @ref DDL_RTC_TIME_Get function
  * @retval Hours in BCD format (0x01. . .0x12 or between Min_Data=0x00 and Max_Data=0x23)
  */
#define __DDL_RTC_GET_HOUR(__RTC_TIME__) (((__RTC_TIME__) >> RTC_OFFSET_HOUR) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve minute in BCD format
  * @param  __RTC_TIME__ RTC time returned by @ref DDL_RTC_TIME_Get function
  * @retval Minutes in BCD format (0x00. . .0x59)
  */
#define __DDL_RTC_GET_MINUTE(__RTC_TIME__) (((__RTC_TIME__) >> RTC_OFFSET_MINUTE) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve second in BCD format
  * @param  __RTC_TIME__ RTC time returned by @ref DDL_RTC_TIME_Get function
  * @retval Seconds in  format (0x00. . .0x59)
  */
#define __DDL_RTC_GET_SECOND(__RTC_TIME__) ((__RTC_TIME__) & 0x000000FFU)

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RTC_DDL_Exported_Functions RTC Exported Functions
  * @{
  */

/** @defgroup RTC_DDL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Set Hours format (24 hour/day or AM/PM hour format)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @param  HourFormat This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_HOURFORMAT_24HOUR
  *         @arg @ref DDL_RTC_HOURFORMAT_AMPM
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetHourFormat(RTC_TypeDef *RTCx, uint32_t HourFormat)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_TIMEFCFG, HourFormat);
}

/**
  * @brief  Get Hours format (24 hour/day or AM/PM hour format)
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_HOURFORMAT_24HOUR
  *         @arg @ref DDL_RTC_HOURFORMAT_AMPM
  */
__STATIC_INLINE uint32_t DDL_RTC_GetHourFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_TIMEFCFG));
}

/**
  * @brief  Select the flag to be routed to RTC_ALARM output
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @param  AlarmOutput This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALARMOUT_DISABLE
  *         @arg @ref DDL_RTC_ALARMOUT_ALMA
  *         @arg @ref DDL_RTC_ALARMOUT_ALMB
  *         @arg @ref DDL_RTC_ALARMOUT_WAKEUP
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetAlarmOutEvent(RTC_TypeDef *RTCx, uint32_t AlarmOutput)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_OUTSEL, AlarmOutput);
}

/**
  * @brief  Get the flag to be routed to RTC_ALARM output
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_ALARMOUT_DISABLE
  *         @arg @ref DDL_RTC_ALARMOUT_ALMA
  *         @arg @ref DDL_RTC_ALARMOUT_ALMB
  *         @arg @ref DDL_RTC_ALARMOUT_WAKEUP
  */
__STATIC_INLINE uint32_t DDL_RTC_GetAlarmOutEvent(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_OUTSEL));
}

/**
  * @brief  Set RTC_ALARM output type (ALARM in push-pull or open-drain output)
  * @note   Used only when RTC_ALARM is mapped on PC13
  * @param  RTCx RTC Instance
  * @param  Output This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  *         @arg @ref DDL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetAlarmOutputType(RTC_TypeDef *RTCx, uint32_t Output)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_ALRMOT, Output);
}

/**
  * @brief  Get RTC_ALARM output type (ALARM in push-pull or open-drain output)
  * @note   used only when RTC_ALARM is mapped on PC13
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  *         @arg @ref DDL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  */
__STATIC_INLINE uint32_t DDL_RTC_GetAlarmOutputType(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_ALRMOT));
}

/**
  * @brief  Enable initialization mode
  * @note   Initialization mode is used to program time and date register (RTC_TIME and RTC_DATE)
  *         and prescaler register (RTC_PSC).
  *         Counters are stopped and start counting from the new value when INIT is reset.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableInitMode(RTC_TypeDef *RTCx)
{
  /* Set the Initialization mode */
  WRITE_REG(RTCx->STS, RTC_INIT_MASK);
}

/**
  * @brief  Disable initialization mode (Free running mode)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableInitMode(RTC_TypeDef *RTCx)
{
  /* Exit Initialization mode */
  WRITE_REG(RTCx->STS, (uint32_t)~RTC_STS_INITEN);
}

/**
  * @brief  Set Output polarity (pin is low when ALRAF/ALRBF/WUTF is asserted)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref DDL_RTC_OUTPUTPOLARITY_PIN_LOW
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetOutputPolarity(RTC_TypeDef *RTCx, uint32_t Polarity)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_POLCFG, Polarity);
}

/**
  * @brief  Get Output polarity
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref DDL_RTC_OUTPUTPOLARITY_PIN_LOW
  */
__STATIC_INLINE uint32_t DDL_RTC_GetOutputPolarity(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_POLCFG));
}

/**
  * @brief  Enable Bypass the shadow registers
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableShadowRegBypass(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_RCMCFG);
}

/**
  * @brief  Disable Bypass the shadow registers
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableShadowRegBypass(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_RCMCFG);
}

/**
  * @brief  Check if Shadow registers bypass is enabled or not.
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsShadowRegBypassEnabled(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_RCMCFG) == (RTC_CTRL_RCMCFG)) ? 1UL : 0UL);
}

/**
  * @brief  Enable RTC_REFIN reference clock detection (50 or 60 Hz)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableRefClock(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_RCLKDEN);
}

/**
  * @brief  Disable RTC_REFIN reference clock detection (50 or 60 Hz)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableRefClock(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_RCLKDEN);
}

/**
  * @brief  Set Asynchronous prescaler factor
  * @param  RTCx RTC Instance
  * @param  AsynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7F
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetAsynchPrescaler(RTC_TypeDef *RTCx, uint32_t AsynchPrescaler)
{
  MODIFY_REG(RTCx->PSC, RTC_PSC_APSC, AsynchPrescaler << RTC_PSC_APSC_Pos);
}

/**
  * @brief  Set Synchronous prescaler factor
  * @param  RTCx RTC Instance
  * @param  SynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7FFF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_SetSynchPrescaler(RTC_TypeDef *RTCx, uint32_t SynchPrescaler)
{
  MODIFY_REG(RTCx->PSC, RTC_PSC_SPSC, SynchPrescaler);
}

/**
  * @brief  Get Asynchronous prescaler factor
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data = 0 and Max_Data = 0x7F
  */
__STATIC_INLINE uint32_t DDL_RTC_GetAsynchPrescaler(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->PSC, RTC_PSC_APSC) >> RTC_PSC_APSC_Pos);
}

/**
  * @brief  Get Synchronous prescaler factor
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data = 0 and Max_Data = 0x7FFF
  */
__STATIC_INLINE uint32_t DDL_RTC_GetSynchPrescaler(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->PSC, RTC_PSC_SPSC));
}

/**
  * @brief  Enable the write protection for RTC registers.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableWriteProtection(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->WRPROT, RTC_WRITE_PROTECTION_DISABLE);
}

/**
  * @brief  Disable the write protection for RTC registers.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableWriteProtection(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->WRPROT, RTC_WRITE_PROTECTION_ENABLE_1);
  WRITE_REG(RTCx->WRPROT, RTC_WRITE_PROTECTION_ENABLE_2);
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Time Time
  * @{
  */

/**
  * @brief  Set time format (AM/24-hour or PM notation)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TIME_FORMAT_AM_OR_24
  *         @arg @ref DDL_RTC_TIME_FORMAT_PM
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_SetFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  MODIFY_REG(RTCx->TIME, RTC_TIME_TIMEFCFG, TimeFormat);
}

/**
  * @brief  Get time format (AM or PM notation)
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note Read either RTC_SUBSEC or RTC_TIME locks the values in the higher-order calendar
  *       shadow registers until RTC_DATE is read (DDL_RTC_ReadReg(RTC, DR)).
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TIME_FORMAT_AM_OR_24
  *         @arg @ref DDL_RTC_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_GetFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TIME, RTC_TIME_TIMEFCFG));
}

/**
  * @brief  Set Hours in BCD format
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert hour from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  MODIFY_REG(RTCx->TIME, (RTC_TIME_HRT | RTC_TIME_HRU),
             (((Hours & 0xF0U) << (RTC_TIME_HRT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_TIME_HRU_Pos)));
}

/**
  * @brief  Get Hours in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note Read either RTC_SUBSEC or RTC_TIME locks the values in the higher-order calendar
  *       shadow registers until RTC_DATE is read (DDL_RTC_ReadReg(RTC, DR)).
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert hour from BCD to
  *       Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->TIME, (RTC_TIME_HRT | RTC_TIME_HRU))) >> RTC_TIME_HRU_Pos);
}

/**
  * @brief  Set Minutes in BCD format
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Minutes from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  MODIFY_REG(RTCx->TIME, (RTC_TIME_MINT | RTC_TIME_MINU),
             (((Minutes & 0xF0U) << (RTC_TIME_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_TIME_MINU_Pos)));
}

/**
  * @brief  Get Minutes in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note Read either RTC_SUBSEC or RTC_TIME locks the values in the higher-order calendar
  *       shadow registers until RTC_DATE is read (DDL_RTC_ReadReg(RTC, DR)).
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert minute from BCD
  *       to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TIME, (RTC_TIME_MINT | RTC_TIME_MINU)) >> RTC_TIME_MINU_Pos);
}

/**
  * @brief  Set Seconds in BCD format
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Seconds from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  MODIFY_REG(RTCx->TIME, (RTC_TIME_SECT | RTC_TIME_SECU),
             (((Seconds & 0xF0U) << (RTC_TIME_SECT_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_TIME_SECU_Pos)));
}

/**
  * @brief  Get Seconds in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note Read either RTC_SUBSEC or RTC_TIME locks the values in the higher-order calendar
  *       shadow registers until RTC_DATE is read (DDL_RTC_ReadReg(RTC, DR)).
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD
  *       to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TIME, (RTC_TIME_SECT | RTC_TIME_SECU)) >> RTC_TIME_SECU_Pos);
}

/**
  * @brief  Set time (hour, minute and second) in BCD format
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @note TimeFormat and Hours should follow the same format
  * @param  RTCx RTC Instance
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TIME_FORMAT_AM_OR_24
  *         @arg @ref DDL_RTC_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_Config(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24                                                                            | \
         (((Hours   & 0xF0U) << (RTC_TIME_HRT_Pos  - 4U)) | ((Hours & 0x0FU) << RTC_TIME_HRU_Pos))    | \
         (((Minutes & 0xF0U) << (RTC_TIME_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_TIME_MINU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_TIME_SECT_Pos  - 4U)) | ((Seconds & 0x0FU) << RTC_TIME_SECU_Pos));
  MODIFY_REG(RTCx->TIME, (RTC_TIME_TIMEFCFG | RTC_TIME_HRT | RTC_TIME_HRU | RTC_TIME_MINT | RTC_TIME_MINU | RTC_TIME_SECT | RTC_TIME_SECU), temp);
}

/**
  * @brief  Get time (hour, minute and second) in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note Read either RTC_SUBSEC or RTC_TIME locks the values in the higher-order calendar
  *       shadow registers until RTC_DATE is read (DDL_RTC_ReadReg(RTC, DR)).
  * @note helper macros __DDL_RTC_GET_HOUR, __DDL_RTC_GET_MINUTE and __DDL_RTC_GET_SECOND
  *       are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of hours, minutes and seconds (Format: 0x00HHMMSS).
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_Get(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TIME, (RTC_TIME_HRT | RTC_TIME_HRU | RTC_TIME_MINT | RTC_TIME_MINU | RTC_TIME_SECT | RTC_TIME_SECU)));
}

/**
  * @brief  Memorize whether the daylight saving time change has been performed
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_EnableDayLightStore(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_BAKP);
}

/**
  * @brief  Disable memorization whether the daylight saving time change has been performed.
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_DisableDayLightStore(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_BAKP);
}

/**
  * @brief  Check if RTC Day Light Saving stored operation has been enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_IsDayLightStoreEnabled(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_BAKP) == (RTC_CTRL_BAKP)) ? 1UL : 0UL);
}

/**
  * @brief  Subtract 1 hour (winter time change)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_DecHour(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_WTCCFG);
}

/**
  * @brief  Add 1 hour (summer time change)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_IncHour(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_STCCFG);
}

/**
  * @brief  Get subseconds value in the synchronous prescaler counter.
  * @note  You can use both SubSeconds value and SecondFraction (PREDIV_S through
  *        DDL_RTC_GetSynchPrescaler function) terms returned to convert Calendar
  *        SubSeconds value in second fraction ratio with time unit following
  *        generic formula:
  *          ==> Seconds fraction ratio * time_unit =
  *                 [(SecondFraction-SubSeconds)/(SecondFraction+1)] * time_unit
  *        This conversion can be performed only if no shift operation is pending
  *        (ie. SHFP=0) when PREDIV_S >= SS.
  * @param  RTCx RTC Instance
  * @retval Subseconds value (number between 0 and 65535)
  */
__STATIC_INLINE uint32_t DDL_RTC_TIME_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->SUBSEC, RTC_SUBSEC_SUBSEC));
}

/**
  * @brief  Synchronize to a remote clock with a high degree of precision.
  * @note   This operation effectively subtracts from (delays) or advance the clock of a fraction of a second.
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   When REFCKON is set, firmware must not write to Shift control register.
  * @param  RTCx RTC Instance
  * @param  ShiftSecond This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_SHIFT_SECOND_DELAY
  *         @arg @ref DDL_RTC_SHIFT_SECOND_ADVANCE
  * @param  Fraction Number of Seconds Fractions (any value from 0 to 0x7FFF)
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TIME_Synchronize(RTC_TypeDef *RTCx, uint32_t ShiftSecond, uint32_t Fraction)
{
  WRITE_REG(RTCx->SHIFT, ShiftSecond | Fraction);
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Date Date
  * @{
  */

/**
  * @brief  Set Year in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Year from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Year Value between Min_Data=0x00 and Max_Data=0x99
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DATE_SetYear(RTC_TypeDef *RTCx, uint32_t Year)
{
  MODIFY_REG(RTCx->DATE, (RTC_DATE_YRT | RTC_DATE_YRU),
             (((Year & 0xF0U) << (RTC_DATE_YRT_Pos - 4U)) | ((Year & 0x0FU) << RTC_DATE_YRU_Pos)));
}

/**
  * @brief  Get Year in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Year from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x99
  */
__STATIC_INLINE uint32_t DDL_RTC_DATE_GetYear(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->DATE, (RTC_DATE_YRT | RTC_DATE_YRU))) >> RTC_DATE_YRU_Pos);
}

/**
  * @brief  Set Week day
  * @param  RTCx RTC Instance
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DATE_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  MODIFY_REG(RTCx->DATE, RTC_DATE_WEEKSEL, WeekDay << RTC_DATE_WEEKSEL_Pos);
}

/**
  * @brief  Get Week day
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t DDL_RTC_DATE_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->DATE, RTC_DATE_WEEKSEL) >> RTC_DATE_WEEKSEL_Pos);
}

/**
  * @brief  Set Month in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Month from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Month This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_MONTH_JANUARY
  *         @arg @ref DDL_RTC_MONTH_FEBRUARY
  *         @arg @ref DDL_RTC_MONTH_MARCH
  *         @arg @ref DDL_RTC_MONTH_APRIL
  *         @arg @ref DDL_RTC_MONTH_MAY
  *         @arg @ref DDL_RTC_MONTH_JUNE
  *         @arg @ref DDL_RTC_MONTH_JULY
  *         @arg @ref DDL_RTC_MONTH_AUGUST
  *         @arg @ref DDL_RTC_MONTH_SEPTEMBER
  *         @arg @ref DDL_RTC_MONTH_OCTOBER
  *         @arg @ref DDL_RTC_MONTH_NOVEMBER
  *         @arg @ref DDL_RTC_MONTH_DECEMBER
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DATE_SetMonth(RTC_TypeDef *RTCx, uint32_t Month)
{
  MODIFY_REG(RTCx->DATE, (RTC_DATE_MONT | RTC_DATE_MONU),
             (((Month & 0xF0U) << (RTC_DATE_MONT_Pos - 4U)) | ((Month & 0x0FU) << RTC_DATE_MONU_Pos)));
}

/**
  * @brief  Get Month in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Month from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_MONTH_JANUARY
  *         @arg @ref DDL_RTC_MONTH_FEBRUARY
  *         @arg @ref DDL_RTC_MONTH_MARCH
  *         @arg @ref DDL_RTC_MONTH_APRIL
  *         @arg @ref DDL_RTC_MONTH_MAY
  *         @arg @ref DDL_RTC_MONTH_JUNE
  *         @arg @ref DDL_RTC_MONTH_JULY
  *         @arg @ref DDL_RTC_MONTH_AUGUST
  *         @arg @ref DDL_RTC_MONTH_SEPTEMBER
  *         @arg @ref DDL_RTC_MONTH_OCTOBER
  *         @arg @ref DDL_RTC_MONTH_NOVEMBER
  *         @arg @ref DDL_RTC_MONTH_DECEMBER
  */
__STATIC_INLINE uint32_t DDL_RTC_DATE_GetMonth(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->DATE, (RTC_DATE_MONT | RTC_DATE_MONU))) >> RTC_DATE_MONU_Pos);
}

/**
  * @brief  Set Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DATE_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  MODIFY_REG(RTCx->DATE, (RTC_DATE_DAYT | RTC_DATE_DAYU),
             (((Day & 0xF0U) << (RTC_DATE_DAYT_Pos - 4U)) | ((Day & 0x0FU) << RTC_DATE_DAYU_Pos)));
}

/**
  * @brief  Get Day in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  */
__STATIC_INLINE uint32_t DDL_RTC_DATE_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->DATE, (RTC_DATE_DAYT | RTC_DATE_DAYU))) >> RTC_DATE_DAYU_Pos);
}

/**
  * @brief  Set date (WeekDay, Day, Month and Year) in BCD format
  * @param  RTCx RTC Instance
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @param  Month This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_MONTH_JANUARY
  *         @arg @ref DDL_RTC_MONTH_FEBRUARY
  *         @arg @ref DDL_RTC_MONTH_MARCH
  *         @arg @ref DDL_RTC_MONTH_APRIL
  *         @arg @ref DDL_RTC_MONTH_MAY
  *         @arg @ref DDL_RTC_MONTH_JUNE
  *         @arg @ref DDL_RTC_MONTH_JULY
  *         @arg @ref DDL_RTC_MONTH_AUGUST
  *         @arg @ref DDL_RTC_MONTH_SEPTEMBER
  *         @arg @ref DDL_RTC_MONTH_OCTOBER
  *         @arg @ref DDL_RTC_MONTH_NOVEMBER
  *         @arg @ref DDL_RTC_MONTH_DECEMBER
  * @param  Year Value between Min_Data=0x00 and Max_Data=0x99
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DATE_Config(RTC_TypeDef *RTCx, uint32_t WeekDay, uint32_t Day, uint32_t Month, uint32_t Year)
{
  uint32_t temp;

  temp = (  WeekDay                                                    << RTC_DATE_WEEKSEL_Pos) | \
         (((Year  & 0xF0U) << (RTC_DATE_YRT_Pos - 4U)) | ((Year  & 0x0FU) << RTC_DATE_YRU_Pos)) | \
         (((Month & 0xF0U) << (RTC_DATE_MONT_Pos - 4U)) | ((Month & 0x0FU) << RTC_DATE_MONU_Pos)) | \
         (((Day   & 0xF0U) << (RTC_DATE_DAYT_Pos - 4U)) | ((Day   & 0x0FU) << RTC_DATE_DAYU_Pos));

  MODIFY_REG(RTCx->DATE, (RTC_DATE_WEEKSEL | RTC_DATE_MONT | RTC_DATE_MONU | RTC_DATE_DAYT | RTC_DATE_DAYU | RTC_DATE_YRT | RTC_DATE_YRU), temp);
}

/**
  * @brief  Get date (WeekDay, Day, Month and Year) in BCD format
  * @note if shadow mode is disabled (BYPSHAD=0), need to check if RSF flag is set
  *       before reading this bit
  * @note helper macros __DDL_RTC_GET_WEEKDAY, __DDL_RTC_GET_YEAR, __DDL_RTC_GET_MONTH,
  * and __DDL_RTC_GET_DAY are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of WeekDay, Day, Month and Year (Format: 0xWWDDMMYY).
  */
__STATIC_INLINE uint32_t DDL_RTC_DATE_Get(RTC_TypeDef *RTCx)
{
  uint32_t temp;

  temp = READ_BIT(RTCx->DATE, (RTC_DATE_WEEKSEL | RTC_DATE_MONT | RTC_DATE_MONU | RTC_DATE_DAYT | RTC_DATE_DAYU | RTC_DATE_YRT | RTC_DATE_YRU));

  return (uint32_t)((((temp &              RTC_DATE_WEEKSEL) >> RTC_DATE_WEEKSEL_Pos) << RTC_OFFSET_WEEKDAY) | \
                    (((temp & (RTC_DATE_DAYT | RTC_DATE_DAYU)) >> RTC_DATE_DAYU_Pos)  << RTC_OFFSET_DAY)     | \
                    (((temp & (RTC_DATE_MONT | RTC_DATE_MONU)) >> RTC_DATE_MONU_Pos)  << RTC_OFFSET_MONTH)   | \
                     ((temp & (RTC_DATE_YRT | RTC_DATE_YRU)) >> RTC_DATE_YRU_Pos));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_ALARMA ALARMA
  * @{
  */

/**
  * @brief  Enable Alarm A
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_Enable(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_ALRAEN);
}

/**
  * @brief  Disable Alarm A
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_Disable(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_ALRAEN);
}

/**
  * @brief  Specify the Alarm A masks.
  * @param  RTCx RTC Instance
  * @param  Mask This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_ALMA_MASK_NONE
  *         @arg @ref DDL_RTC_ALMA_MASK_DATEWEEKDAY
  *         @arg @ref DDL_RTC_ALMA_MASK_HOURS
  *         @arg @ref DDL_RTC_ALMA_MASK_MINUTES
  *         @arg @ref DDL_RTC_ALMA_MASK_SECONDS
  *         @arg @ref DDL_RTC_ALMA_MASK_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  MODIFY_REG(RTCx->ALRMA, RTC_ALRMA_DATEMEN | RTC_ALRMA_HRMEN | RTC_ALRMA_MINMEN | RTC_ALRMA_SECMEN, Mask);
}

/**
  * @brief  Get the Alarm A masks.
  * @param  RTCx RTC Instance
  * @retval Returned value can be can be a combination of the following values:
  *         @arg @ref DDL_RTC_ALMA_MASK_NONE
  *         @arg @ref DDL_RTC_ALMA_MASK_DATEWEEKDAY
  *         @arg @ref DDL_RTC_ALMA_MASK_HOURS
  *         @arg @ref DDL_RTC_ALMA_MASK_MINUTES
  *         @arg @ref DDL_RTC_ALMA_MASK_SECONDS
  *         @arg @ref DDL_RTC_ALMA_MASK_ALL
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMA, RTC_ALRMA_DATEMEN | RTC_ALRMA_HRMEN | RTC_ALRMA_MINMEN | RTC_ALRMA_SECMEN));
}

/**
  * @brief  Enable AlarmA Week day selection (DU[3:0] represents the week day. DT[1:0] is do not care)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_EnableWeekday(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->ALRMA, RTC_ALRMA_WEEKSEL);
}

/**
  * @brief  Disable AlarmA Week day selection (DU[3:0] represents the date )
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_DisableWeekday(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->ALRMA, RTC_ALRMA_WEEKSEL);
}

/**
  * @brief  Set ALARM A Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  MODIFY_REG(RTCx->ALRMA, (RTC_ALRMA_DAYT | RTC_ALRMA_DAYU),
             (((Day & 0xF0U) << (RTC_ALRMA_DAYT_Pos - 4U)) | ((Day & 0x0FU) << RTC_ALRMA_DAYU_Pos)));
}

/**
  * @brief  Get ALARM A Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMA, (RTC_ALRMA_DAYT | RTC_ALRMA_DAYU))) >> RTC_ALRMA_DAYU_Pos);
}

/**
  * @brief  Set ALARM A Weekday
  * @param  RTCx RTC Instance
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  MODIFY_REG(RTCx->ALRMA, RTC_ALRMA_DAYU, WeekDay << RTC_ALRMA_DAYU_Pos);
}

/**
  * @brief  Get ALARM A Weekday
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMA, RTC_ALRMA_DAYU) >> RTC_ALRMA_DAYU_Pos);
}

/**
  * @brief  Set Alarm A time format (AM/24-hour or PM notation)
  * @param  RTCx RTC Instance
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_PM
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetTimeFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  MODIFY_REG(RTCx->ALRMA, RTC_ALRMA_TIMEFCFG, TimeFormat);
}

/**
  * @brief  Get Alarm A time format (AM or PM notation)
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMA, RTC_ALRMA_TIMEFCFG));
}

/**
  * @brief  Set ALARM A Hours in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Hours from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  MODIFY_REG(RTCx->ALRMA, (RTC_ALRMA_HRT | RTC_ALRMA_HRU),
             (((Hours & 0xF0U) << (RTC_ALRMA_HRT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMA_HRU_Pos)));
}

/**
  * @brief  Get ALARM A Hours in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Hours from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMA, (RTC_ALRMA_HRT | RTC_ALRMA_HRU))) >> RTC_ALRMA_HRU_Pos);
}

/**
  * @brief  Set ALARM A Minutes in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Minutes from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  MODIFY_REG(RTCx->ALRMA, (RTC_ALRMA_MINT | RTC_ALRMA_MINU),
             (((Minutes & 0xF0U) << (RTC_ALRMA_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMA_MINU_Pos)));
}

/**
  * @brief  Get ALARM A Minutes in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Minutes from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMA, (RTC_ALRMA_MINT | RTC_ALRMA_MINU))) >> RTC_ALRMA_MINU_Pos);
}

/**
  * @brief  Set ALARM A Seconds in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Seconds from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  MODIFY_REG(RTCx->ALRMA, (RTC_ALRMA_SECT | RTC_ALRMA_SECU),
             (((Seconds & 0xF0U) << (RTC_ALRMA_SECT_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMA_SECU_Pos)));
}

/**
  * @brief  Get ALARM A Seconds in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMA, (RTC_ALRMA_SECT | RTC_ALRMA_SECU))) >> RTC_ALRMA_SECU_Pos);
}

/**
  * @brief  Set Alarm A Time (hour, minute and second) in BCD format
  * @param  RTCx RTC Instance
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMA_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_ConfigTime(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24                                                                                    | \
         (((Hours   & 0xF0U) << (RTC_ALRMA_HRT_Pos  - 4U)) | ((Hours   & 0x0FU) << RTC_ALRMA_HRU_Pos))  | \
         (((Minutes & 0xF0U) << (RTC_ALRMA_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMA_MINU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_ALRMA_SECT_Pos  - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMA_SECU_Pos));

  MODIFY_REG(RTCx->ALRMA, RTC_ALRMA_TIMEFCFG | RTC_ALRMA_HRT | RTC_ALRMA_HRU | RTC_ALRMA_MINT | RTC_ALRMA_MINU | RTC_ALRMA_SECT | RTC_ALRMA_SECU, temp);
}

/**
  * @brief  Get Alarm B Time (hour, minute and second) in BCD format
  * @note helper macros __DDL_RTC_GET_HOUR, __DDL_RTC_GET_MINUTE and __DDL_RTC_GET_SECOND
  * are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of hours, minutes and seconds.
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)((DDL_RTC_ALMA_GetHour(RTCx) << RTC_OFFSET_HOUR) | (DDL_RTC_ALMA_GetMinute(RTCx) << RTC_OFFSET_MINUTE) | DDL_RTC_ALMA_GetSecond(RTCx));
}

/**
  * @brief  Mask the most-significant bits of the subseconds field starting from
  *         the bit specified in parameter Mask
  * @note This register can be written only when ALRAE is reset in RTC_CTRL register,
  *       or in initialization mode.
  * @param  RTCx RTC Instance
  * @param  Mask Value between Min_Data=0x00 and Max_Data=0xF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetSubSecondMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  MODIFY_REG(RTCx->ALRMASS, RTC_ALRMASS_MASKSEL, Mask << RTC_ALRMASS_MASKSEL_Pos);
}

/**
  * @brief  Get Alarm A subseconds mask
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetSubSecondMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMASS, RTC_ALRMASS_MASKSEL) >> RTC_ALRMASS_MASKSEL_Pos);
}

/**
  * @brief  Set Alarm A subseconds value
  * @param  RTCx RTC Instance
  * @param  Subsecond Value between Min_Data=0x00 and Max_Data=0x7FFF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMA_SetSubSecond(RTC_TypeDef *RTCx, uint32_t Subsecond)
{
  MODIFY_REG(RTCx->ALRMASS, RTC_ALRMASS_SUBSEC, Subsecond);
}

/**
  * @brief  Get Alarm A subseconds value
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x7FFF
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMA_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMASS, RTC_ALRMASS_SUBSEC));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_ALARMB ALARMB
  * @{
  */

/**
  * @brief  Enable Alarm B
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_Enable(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_ALRBEN);
}

/**
  * @brief  Disable Alarm B
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_Disable(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_ALRBEN);
}

/**
  * @brief  Specify the Alarm B masks.
  * @param  RTCx RTC Instance
  * @param  Mask This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_ALMB_MASK_NONE
  *         @arg @ref DDL_RTC_ALMB_MASK_DATEWEEKDAY
  *         @arg @ref DDL_RTC_ALMB_MASK_HOURS
  *         @arg @ref DDL_RTC_ALMB_MASK_MINUTES
  *         @arg @ref DDL_RTC_ALMB_MASK_SECONDS
  *         @arg @ref DDL_RTC_ALMB_MASK_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  MODIFY_REG(RTCx->ALRMB, RTC_ALRMB_DATEMEN | RTC_ALRMB_HRMEN | RTC_ALRMB_MINMEN | RTC_ALRMB_SECMEN, Mask);
}

/**
  * @brief  Get the Alarm B masks.
  * @param  RTCx RTC Instance
  * @retval Returned value can be can be a combination of the following values:
  *         @arg @ref DDL_RTC_ALMB_MASK_NONE
  *         @arg @ref DDL_RTC_ALMB_MASK_DATEWEEKDAY
  *         @arg @ref DDL_RTC_ALMB_MASK_HOURS
  *         @arg @ref DDL_RTC_ALMB_MASK_MINUTES
  *         @arg @ref DDL_RTC_ALMB_MASK_SECONDS
  *         @arg @ref DDL_RTC_ALMB_MASK_ALL
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMB, RTC_ALRMB_DATEMEN | RTC_ALRMB_HRMEN | RTC_ALRMB_MINMEN | RTC_ALRMB_SECMEN));
}

/**
  * @brief  Enable AlarmB Week day selection (DU[3:0] represents the week day. DT[1:0] is do not care)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_EnableWeekday(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->ALRMB, RTC_ALRMB_WEEKSEL);
}

/**
  * @brief  Disable AlarmB Week day selection (DU[3:0] represents the date )
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_DisableWeekday(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->ALRMB, RTC_ALRMB_WEEKSEL);
}

/**
  * @brief  Set ALARM B Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetDay(RTC_TypeDef *RTCx, uint32_t Day)
{
  MODIFY_REG(RTCx->ALRMB, (RTC_ALRMB_DAYT | RTC_ALRMB_DAYU),
             (((Day & 0xF0U) << (RTC_ALRMB_DAYT_Pos - 4U)) | ((Day & 0x0FU) << RTC_ALRMB_DAYU_Pos)));
}

/**
  * @brief  Get ALARM B Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMB, (RTC_ALRMB_DAYT | RTC_ALRMB_DAYU))) >> RTC_ALRMB_DAYU_Pos);
}

/**
  * @brief  Set ALARM B Weekday
  * @param  RTCx RTC Instance
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetWeekDay(RTC_TypeDef *RTCx, uint32_t WeekDay)
{
  MODIFY_REG(RTCx->ALRMB, RTC_ALRMB_DAYU, WeekDay << RTC_ALRMB_DAYU_Pos);
}

/**
  * @brief  Get ALARM B Weekday
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMB, RTC_ALRMB_DAYU) >> RTC_ALRMB_DAYU_Pos);
}

/**
  * @brief  Set ALARM B time format (AM/24-hour or PM notation)
  * @param  RTCx RTC Instance
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_PM
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetTimeFormat(RTC_TypeDef *RTCx, uint32_t TimeFormat)
{
  MODIFY_REG(RTCx->ALRMB, RTC_ALRMB_TIMEFCFG, TimeFormat);
}

/**
  * @brief  Get ALARM B time format (AM or PM notation)
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMB, RTC_ALRMB_TIMEFCFG));
}

/**
  * @brief  Set ALARM B Hours in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Hours from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetHour(RTC_TypeDef *RTCx, uint32_t Hours)
{
  MODIFY_REG(RTCx->ALRMB, (RTC_ALRMB_HRT | RTC_ALRMB_HRU),
             (((Hours & 0xF0U) << (RTC_ALRMB_HRT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMB_HRU_Pos)));
}

/**
  * @brief  Get ALARM B Hours in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Hours from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMB, (RTC_ALRMB_HRT | RTC_ALRMB_HRU))) >> RTC_ALRMB_HRU_Pos);
}

/**
  * @brief  Set ALARM B Minutes in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Minutes from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Minutes between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetMinute(RTC_TypeDef *RTCx, uint32_t Minutes)
{
  MODIFY_REG(RTCx->ALRMB, (RTC_ALRMB_MINT | RTC_ALRMB_MINU),
             (((Minutes & 0xF0U) << (RTC_ALRMB_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMB_MINU_Pos)));
}

/**
  * @brief  Get ALARM B Minutes in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Minutes from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMB, (RTC_ALRMB_MINT | RTC_ALRMB_MINU))) >> RTC_ALRMB_MINU_Pos);
}

/**
  * @brief  Set ALARM B Seconds in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BIN2BCD is available to convert Seconds from binary to BCD format
  * @param  RTCx RTC Instance
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetSecond(RTC_TypeDef *RTCx, uint32_t Seconds)
{
  MODIFY_REG(RTCx->ALRMB, (RTC_ALRMB_SECT | RTC_ALRMB_SECU),
             (((Seconds & 0xF0U) << (RTC_ALRMB_SECT_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMB_SECU_Pos)));
}

/**
  * @brief  Get ALARM B Seconds in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)((READ_BIT(RTCx->ALRMB, (RTC_ALRMB_SECT | RTC_ALRMB_SECU))) >> RTC_ALRMB_SECU_Pos);
}

/**
  * @brief  Set Alarm B Time (hour, minute and second) in BCD format
  * @param  RTCx RTC Instance
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_ALMB_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_ConfigTime(RTC_TypeDef *RTCx, uint32_t Format12_24, uint32_t Hours, uint32_t Minutes, uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24                                                                                    | \
         (((Hours   & 0xF0U) << (RTC_ALRMB_HRT_Pos  - 4U)) | ((Hours   & 0x0FU) << RTC_ALRMB_HRU_Pos))  | \
         (((Minutes & 0xF0U) << (RTC_ALRMB_MINT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMB_MINU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_ALRMB_SECT_Pos  - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMB_SECU_Pos));

  MODIFY_REG(RTCx->ALRMB, RTC_ALRMB_TIMEFCFG | RTC_ALRMB_HRT | RTC_ALRMB_HRU | RTC_ALRMB_MINT | RTC_ALRMB_MINU | RTC_ALRMB_SECT | RTC_ALRMB_SECU, temp);
}

/**
  * @brief  Get Alarm B Time (hour, minute and second) in BCD format
  * @note helper macros __DDL_RTC_GET_HOUR, __DDL_RTC_GET_MINUTE and __DDL_RTC_GET_SECOND
  * are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of hours, minutes and seconds.
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)((DDL_RTC_ALMB_GetHour(RTCx) << RTC_OFFSET_HOUR) | (DDL_RTC_ALMB_GetMinute(RTCx) << RTC_OFFSET_MINUTE) | DDL_RTC_ALMB_GetSecond(RTCx));
}

/**
  * @brief  Mask the most-significant bits of the subseconds field starting from
  *         the bit specified in parameter Mask
  * @note This register can be written only when ALRBE is reset in RTC_CTRL register,
  *       or in initialization mode.
  * @param  RTCx RTC Instance
  * @param  Mask Value between Min_Data=0x00 and Max_Data=0xF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetSubSecondMask(RTC_TypeDef *RTCx, uint32_t Mask)
{
  MODIFY_REG(RTCx->ALRMBSS, RTC_ALRMBSS_MASKSEL, Mask << RTC_ALRMBSS_MASKSEL_Pos);
}

/**
  * @brief  Get Alarm B subseconds mask
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetSubSecondMask(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMBSS, RTC_ALRMBSS_MASKSEL)  >> RTC_ALRMBSS_MASKSEL_Pos);
}

/**
  * @brief  Set Alarm B subseconds value
  * @param  RTCx RTC Instance
  * @param  Subsecond Value between Min_Data=0x00 and Max_Data=0x7FFF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ALMB_SetSubSecond(RTC_TypeDef *RTCx, uint32_t Subsecond)
{
  MODIFY_REG(RTCx->ALRMBSS, RTC_ALRMBSS_SUBSEC, Subsecond);
}

/**
  * @brief  Get Alarm B subseconds value
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x7FFF
  */
__STATIC_INLINE uint32_t DDL_RTC_ALMB_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->ALRMBSS, RTC_ALRMBSS_SUBSEC));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Timestamp Timestamp
  * @{
  */

/**
  * @brief  Enable Timestamp
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_Enable(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_TSEN);
}

/**
  * @brief  Disable Timestamp
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_Disable(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_TSEN);
}

/**
  * @brief  Set Time-stamp event active edge
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note TSE must be reset when TSEDGE is changed to avoid unwanted TSF setting
  * @param  RTCx RTC Instance
  * @param  Edge This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TIMESTAMP_EDGE_RISING
  *         @arg @ref DDL_RTC_TIMESTAMP_EDGE_FALLING
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_SetActiveEdge(RTC_TypeDef *RTCx, uint32_t Edge)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_TSETECFG, Edge);
}

/**
  * @brief  Get Time-stamp event active edge
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TIMESTAMP_EDGE_RISING
  *         @arg @ref DDL_RTC_TIMESTAMP_EDGE_FALLING
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetActiveEdge(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_TSETECFG));
}

/**
  * @brief  Get Timestamp AM/PM notation (AM or 24-hour format)
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TS_TIME_FORMAT_AM
  *         @arg @ref DDL_RTC_TS_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetTimeFormat(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSTIME, RTC_TSTIME_TIMEFCFG));
}

/**
  * @brief  Get Timestamp Hours in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Hours from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetHour(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSTIME, RTC_TSTIME_HRT | RTC_TSTIME_HRU) >> RTC_TSTIME_HRU_Pos);
}

/**
  * @brief  Get Timestamp Minutes in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Minutes from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetMinute(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSTIME, RTC_TSTIME_MINT | RTC_TSTIME_MINU) >> RTC_TSTIME_MINU_Pos);
}

/**
  * @brief  Get Timestamp Seconds in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSTIME, RTC_TSTIME_SECT | RTC_TSTIME_SECU));
}

/**
  * @brief  Get Timestamp time (hour, minute and second) in BCD format
  * @note helper macros __DDL_RTC_GET_HOUR, __DDL_RTC_GET_MINUTE and __DDL_RTC_GET_SECOND
  * are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of hours, minutes and seconds.
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetTime(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSTIME,
                             RTC_TSTIME_HRT | RTC_TSTIME_HRU | RTC_TSTIME_MINT | RTC_TSTIME_MINU | RTC_TSTIME_SECT | RTC_TSTIME_SECU));
}

/**
  * @brief  Get Timestamp Week day
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WEEKDAY_MONDAY
  *         @arg @ref DDL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref DDL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref DDL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref DDL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetWeekDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSDATE, RTC_TSDATE_WEEKSEL) >> RTC_TSDATE_WEEKSEL_Pos);
}

/**
  * @brief  Get Timestamp Month in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Month from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_MONTH_JANUARY
  *         @arg @ref DDL_RTC_MONTH_FEBRUARY
  *         @arg @ref DDL_RTC_MONTH_MARCH
  *         @arg @ref DDL_RTC_MONTH_APRIL
  *         @arg @ref DDL_RTC_MONTH_MAY
  *         @arg @ref DDL_RTC_MONTH_JUNE
  *         @arg @ref DDL_RTC_MONTH_JULY
  *         @arg @ref DDL_RTC_MONTH_AUGUST
  *         @arg @ref DDL_RTC_MONTH_SEPTEMBER
  *         @arg @ref DDL_RTC_MONTH_OCTOBER
  *         @arg @ref DDL_RTC_MONTH_NOVEMBER
  *         @arg @ref DDL_RTC_MONTH_DECEMBER
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetMonth(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSDATE, RTC_TSDATE_MONT | RTC_TSDATE_MONU) >> RTC_TSDATE_MONU_Pos);
}

/**
  * @brief  Get Timestamp Day in BCD format
  * @note helper macro __DDL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetDay(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSDATE, RTC_TSDATE_DAYT | RTC_TSDATE_DAYU));
}

/**
  * @brief  Get Timestamp date (WeekDay, Day and Month) in BCD format
  * @note helper macros __DDL_RTC_GET_WEEKDAY, __DDL_RTC_GET_MONTH,
  * and __DDL_RTC_GET_DAY are available to get independently each parameter.
  * @param  RTCx RTC Instance
  * @retval Combination of Weekday, Day and Month
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetDate(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSDATE, RTC_TSDATE_WEEKSEL | RTC_TSDATE_MONT | RTC_TSDATE_MONU | RTC_TSDATE_DAYT | RTC_TSDATE_DAYU));
}

/**
  * @brief  Get time-stamp subseconds value
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetSubSecond(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TSSUBSEC, RTC_TSSUBSEC_SUBSE));
}

#if defined(RTC_TACFG_TPTSEN)
/**
  * @brief  Activate timestamp on tamper detection event
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_EnableOnTamper(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->TACFG, RTC_TACFG_TPTSEN);
}

/**
  * @brief  Disable timestamp on tamper detection event
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_DisableOnTamper(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->TACFG, RTC_TACFG_TPTSEN);
}
#endif /* RTC_TACFG_TPTSEN */

/**
  * @brief  Set timestamp Pin
  * @param  RTCx RTC Instance
  * @param  TSPin specifies the RTC Timestamp Pin.
  *          This parameter can be one of the following values:
  *            @arg DDL_RTC_TimeStampPin_Default: RTC_AF1 is used as RTC Timestamp Pin.
  *            @arg DDL_RTC_TimeStampPin_Pos1: RTC_AF2 is used as RTC Timestamp Pin. (*)
  *
  *            (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TS_SetPin(RTC_TypeDef *RTCx, uint32_t TSPin)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_TSMSEL, TSPin);
}

/**
  * @brief  Get timestamp Pin
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *            @arg DDL_RTC_TimeStampPin_Default: RTC_AF1 is used as RTC Timestamp Pin.
  *            @arg DDL_RTC_TimeStampPin_Pos1: RTC_AF2 is used as RTC Timestamp Pin. (*)
  *
  *            (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_RTC_TS_GetPin(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_TSMSEL));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Tamper Tamper
  * @{
  */

/**
  * @brief  Enable RTC_TAMPx input detection
  * @param  RTCx RTC Instance
  * @param  Tamper This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_TAMPER_1
  *         @arg @ref DDL_RTC_TAMPER_2 (*)
  *
  *         (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_Enable(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  SET_BIT(RTCx->TACFG, Tamper);
}

/**
  * @brief  Clear RTC_TAMPx input detection
  * @param  RTCx RTC Instance
  * @param  Tamper This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_TAMPER_1
  *         @arg @ref DDL_RTC_TAMPER_2 (*)
  *
  *         (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_Disable(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  CLEAR_BIT(RTCx->TACFG, Tamper);
}

/**
  * @brief  Disable RTC_TAMPx pull-up disable (Disable precharge of RTC_TAMPx pins)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_DisablePullUp(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->TACFG, RTC_TACFG_TPPUDIS);
}

/**
  * @brief  Enable RTC_TAMPx pull-up disable ( Precharge RTC_TAMPx pins before sampling)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_EnablePullUp(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->TACFG, RTC_TACFG_TPPUDIS);
}

/**
  * @brief  Set RTC_TAMPx precharge duration
  * @param  RTCx RTC Instance
  * @param  Duration This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_DURATION_1RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_2RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_4RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_8RTCCLK
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_SetPrecharge(RTC_TypeDef *RTCx, uint32_t Duration)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_TPPRDUSEL, Duration);
}

/**
  * @brief  Get RTC_TAMPx precharge duration
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_DURATION_1RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_2RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_4RTCCLK
  *         @arg @ref DDL_RTC_TAMPER_DURATION_8RTCCLK
  */
__STATIC_INLINE uint32_t DDL_RTC_TAMPER_GetPrecharge(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_TPPRDUSEL));
}

/**
  * @brief  Set RTC_TAMPx filter count
  * @param  RTCx RTC Instance
  * @param  FilterCount This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_FILTER_DISABLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_2SAMPLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_4SAMPLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_8SAMPLE
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_SetFilterCount(RTC_TypeDef *RTCx, uint32_t FilterCount)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_TPFCSEL, FilterCount);
}

/**
  * @brief  Get RTC_TAMPx filter count
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_FILTER_DISABLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_2SAMPLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_4SAMPLE
  *         @arg @ref DDL_RTC_TAMPER_FILTER_8SAMPLE
  */
__STATIC_INLINE uint32_t DDL_RTC_TAMPER_GetFilterCount(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_TPFCSEL));
}

/**
  * @brief  Set Tamper sampling frequency
  * @param  RTCx RTC Instance
  * @param  SamplingFreq This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_32768
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_16384
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_8192
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_4096
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_2048
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_1024
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_512
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_256
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_SetSamplingFreq(RTC_TypeDef *RTCx, uint32_t SamplingFreq)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_TPSFSEL, SamplingFreq);
}

/**
  * @brief  Get Tamper sampling frequency
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_32768
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_16384
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_8192
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_4096
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_2048
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_1024
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_512
  *         @arg @ref DDL_RTC_TAMPER_SAMPLFREQDIV_256
  */
__STATIC_INLINE uint32_t DDL_RTC_TAMPER_GetSamplingFreq(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_TPSFSEL));
}

/**
  * @brief  Enable Active level for Tamper input
  * @param  RTCx RTC Instance
  * @param  Tamper This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_TAMPER_ACTIVELEVEL_TAMP1
  *         @arg @ref DDL_RTC_TAMPER_ACTIVELEVEL_TAMP2 (*)
  *
  *         (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_EnableActiveLevel(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  SET_BIT(RTCx->TACFG, Tamper);
}

/**
  * @brief  Disable Active level for Tamper input
  * @param  RTCx RTC Instance
  * @param  Tamper This parameter can be a combination of the following values:
  *         @arg @ref DDL_RTC_TAMPER_ACTIVELEVEL_TAMP1
  *         @arg @ref DDL_RTC_TAMPER_ACTIVELEVEL_TAMP2 (*)
  *
  *         (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_DisableActiveLevel(RTC_TypeDef *RTCx, uint32_t Tamper)
{
  CLEAR_BIT(RTCx->TACFG, Tamper);
}

/**
  * @brief  Set Tamper Pin
  * @param  RTCx RTC Instance
  * @param  TamperPin specifies the RTC Tamper Pin.
  *          This parameter can be one of the following values:
  *            @arg DDL_RTC_TamperPin_Default: RTC_AF1 is used as RTC Tamper Pin.
  *            @arg DDL_RTC_TamperPin_Pos1: RTC_AF2 is used as RTC Tamper Pin. (*)
  *
  *            (*) value not applicable to all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_TAMPER_SetPin(RTC_TypeDef *RTCx, uint32_t TamperPin)
{
  MODIFY_REG(RTCx->TACFG, RTC_TACFG_TP1MSEL, TamperPin);
}

/**
  * @brief  Get Tamper Pin
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *            @arg DDL_RTC_TamperPin_Default: RTC_AF1 is used as RTC Tamper Pin.
  *            @arg DDL_RTC_TamperPin_Pos1: RTC_AF2 is selected as RTC Tamper Pin. (*)
  *
  *            (*) value not applicable to all devices.
  * @retval None
  */

__STATIC_INLINE uint32_t DDL_RTC_TAMPER_GetPin(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->TACFG, RTC_TACFG_TP1MSEL));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Wakeup Wakeup
  * @{
  */

/**
  * @brief  Enable Wakeup timer
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_WAKEUP_Enable(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_WUTEN);
}

/**
  * @brief  Disable Wakeup timer
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_WAKEUP_Disable(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_WUTEN);
}

/**
  * @brief  Check if Wakeup timer is enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_WAKEUP_IsEnabled(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_WUTEN) == (RTC_CTRL_WUTEN)) ? 1UL : 0UL);
}

/**
  * @brief  Select Wakeup clock
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   Bit can be written only when RTC_CTRL WUTE bit = 0 and RTC_STS WUTWF bit = 1
  * @param  RTCx RTC Instance
  * @param  WakeupClock This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_16
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_8
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_4
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_2
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_CKSPRE
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_CKSPRE_WUT
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_WAKEUP_SetClock(RTC_TypeDef *RTCx, uint32_t WakeupClock)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_WUCLKSEL, WakeupClock);
}

/**
  * @brief  Get Wakeup clock
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_16
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_8
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_4
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_DIV_2
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_CKSPRE
  *         @arg @ref DDL_RTC_WAKEUPCLOCK_CKSPRE_WUT
  */
__STATIC_INLINE uint32_t DDL_RTC_WAKEUP_GetClock(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_WUCLKSEL));
}

/**
  * @brief  Set Wakeup auto-reload value
  * @note   Bit can be written only when WUTWF is set to 1 in RTC_STS
  * @param  RTCx RTC Instance
  * @param  Value Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_WAKEUP_SetAutoReload(RTC_TypeDef *RTCx, uint32_t Value)
{
  MODIFY_REG(RTCx->AUTORLD, RTC_AUTORLD_WUAUTORE, Value);
}

/**
  * @brief  Get Wakeup auto-reload value
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t DDL_RTC_WAKEUP_GetAutoReload(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->AUTORLD, RTC_AUTORLD_WUAUTORE));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Backup_Registers Backup_Registers
  * @{
  */

/**
  * @brief  Writes a data in a specified RTC Backup data register.
  * @param  RTCx RTC Instance
  * @param  BackupRegister This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_BKP_DR0
  *         @arg @ref DDL_RTC_BKP_DR1
  *         @arg @ref DDL_RTC_BKP_DR2
  *         @arg @ref DDL_RTC_BKP_DR3
  *         @arg @ref DDL_RTC_BKP_DR4
  *         @arg @ref DDL_RTC_BKP_DR5
  *         @arg @ref DDL_RTC_BKP_DR6
  *         @arg @ref DDL_RTC_BKP_DR7
  *         @arg @ref DDL_RTC_BKP_DR8
  *         @arg @ref DDL_RTC_BKP_DR9
  *         @arg @ref DDL_RTC_BKP_DR10
  *         @arg @ref DDL_RTC_BKP_DR11
  *         @arg @ref DDL_RTC_BKP_DR12
  *         @arg @ref DDL_RTC_BKP_DR13
  *         @arg @ref DDL_RTC_BKP_DR14
  *         @arg @ref DDL_RTC_BKP_DR15
  *         @arg @ref DDL_RTC_BKP_DR16
  *         @arg @ref DDL_RTC_BKP_DR17
  *         @arg @ref DDL_RTC_BKP_DR18
  *         @arg @ref DDL_RTC_BKP_DR19
  * @param  Data Value between Min_Data=0x00 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_BAK_SetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister, uint32_t Data)
{
  uint32_t temp;

  temp = (uint32_t)(&(RTCx->BAKP0));
  temp += (BackupRegister * 4U);

  /* Write the specified register */
  *(__IO uint32_t *)temp = (uint32_t)Data;
}

/**
  * @brief  Reads data from the specified RTC Backup data Register.
  * @param  RTCx RTC Instance
  * @param  BackupRegister This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_BKP_DR0
  *         @arg @ref DDL_RTC_BKP_DR1
  *         @arg @ref DDL_RTC_BKP_DR2
  *         @arg @ref DDL_RTC_BKP_DR3
  *         @arg @ref DDL_RTC_BKP_DR4
  *         @arg @ref DDL_RTC_BKP_DR5
  *         @arg @ref DDL_RTC_BKP_DR6
  *         @arg @ref DDL_RTC_BKP_DR7
  *         @arg @ref DDL_RTC_BKP_DR8
  *         @arg @ref DDL_RTC_BKP_DR9
  *         @arg @ref DDL_RTC_BKP_DR10
  *         @arg @ref DDL_RTC_BKP_DR11
  *         @arg @ref DDL_RTC_BKP_DR12
  *         @arg @ref DDL_RTC_BKP_DR13
  *         @arg @ref DDL_RTC_BKP_DR14
  *         @arg @ref DDL_RTC_BKP_DR15
  *         @arg @ref DDL_RTC_BKP_DR16
  *         @arg @ref DDL_RTC_BKP_DR17
  *         @arg @ref DDL_RTC_BKP_DR18
  *         @arg @ref DDL_RTC_BKP_DR19
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_RTC_BAK_GetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister)
{
  uint32_t temp;

  temp = (uint32_t)(&(RTCx->BAKP0));
  temp += (BackupRegister * 4U);

  /* Read the specified register */
  return (*(__IO uint32_t *)temp);
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_Calibration Calibration
  * @{
  */

/**
  * @brief  Set Calibration output frequency (1 Hz or 512 Hz)
  * @note Bits are write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @param  Frequency This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_512HZ
  *
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_SetOutputFreq(RTC_TypeDef *RTCx, uint32_t Frequency)
{
  MODIFY_REG(RTCx->CTRL, RTC_CTRL_CALOEN | RTC_CTRL_CALOSEL, Frequency);
}

/**
  * @brief  Get Calibration output frequency (1 Hz or 512 Hz)
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref DDL_RTC_CALIB_OUTPUT_512HZ
  *
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_GetOutputFreq(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CTRL, RTC_CTRL_CALOEN | RTC_CTRL_CALOSEL));
}

/**
  * @brief  Enable Coarse digital calibration
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_EnableCoarseDigital(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_DCALEN);
}

/**
  * @brief  Disable Coarse digital calibration
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_DisableCoarseDigital(RTC_TypeDef  *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_DCALEN);
}

/**
  * @brief  Set the coarse digital calibration
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref DDL_RTC_EnableInitMode function)
  * @param  RTCx RTC Instance
  * @param  Sign This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_SIGN_POSITIVE
  *         @arg @ref DDL_RTC_CALIB_SIGN_NEGATIVE
  * @param  Value value of coarse calibration expressed in ppm (coded on 5 bits)
  * @note   This Calibration value should be between 0 and 63 when using negative sign with a 2-ppm step.
  * @note   This Calibration value should be between 0 and 126 when using positive sign with a 4-ppm step.
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_ConfigCoarseDigital(RTC_TypeDef *RTCx, uint32_t Sign, uint32_t Value)
{
  MODIFY_REG(RTCx->DCAL, RTC_DCAL_DCALCFG | RTC_DCAL_DCAL, Sign | Value);
}

/**
  * @brief  Get the coarse digital calibration value
  * @param  RTCx RTC Instance
  * @retval value of coarse calibration expressed in ppm (coded on 5 bits)
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_GetCoarseDigitalValue(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->DCAL, RTC_DCAL_DCAL));
}

/**
  * @brief  Get the coarse digital calibration sign
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_SIGN_POSITIVE
  *         @arg @ref DDL_RTC_CALIB_SIGN_NEGATIVE
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_GetCoarseDigitalSign(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->DCAL, RTC_DCAL_DCALCFG));
}

/**
  * @brief  Insert or not One RTCCLK pulse every 2exp11 pulses (frequency increased by 488.5 ppm)
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_STS
  * @param  RTCx RTC Instance
  * @param  Pulse This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_INSERTPULSE_NONE
  *         @arg @ref DDL_RTC_CALIB_INSERTPULSE_SET
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_SetPulse(RTC_TypeDef *RTCx, uint32_t Pulse)
{
  MODIFY_REG(RTCx->CAL, RTC_CAL_ICALFEN, Pulse);
}

/**
  * @brief  Check if one RTCCLK has been inserted or not every 2exp11 pulses (frequency increased by 488.5 ppm)
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_IsPulseInserted(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CAL, RTC_CAL_ICALFEN) == (RTC_CAL_ICALFEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set smooth calibration cycle period
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_STS
  * @param  RTCx RTC Instance
  * @param  Period This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_PERIOD_32SEC
  *         @arg @ref DDL_RTC_CALIB_PERIOD_16SEC
  *         @arg @ref DDL_RTC_CALIB_PERIOD_8SEC
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_SetPeriod(RTC_TypeDef *RTCx, uint32_t Period)
{
  MODIFY_REG(RTCx->CAL, RTC_CAL_CAL8CFG | RTC_CAL_CAL16CFG, Period);
}

/**
  * @brief  Get smooth calibration cycle period
  * @param  RTCx RTC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RTC_CALIB_PERIOD_32SEC
  *         @arg @ref DDL_RTC_CALIB_PERIOD_16SEC
  *         @arg @ref DDL_RTC_CALIB_PERIOD_8SEC
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_GetPeriod(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CAL, RTC_CAL_CAL8CFG | RTC_CAL_CAL16CFG));
}

/**
  * @brief  Set smooth Calibration minus
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_STS
  * @param  RTCx RTC Instance
  * @param  CalibMinus Value between Min_Data=0x00 and Max_Data=0x1FF
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_CAL_SetMinus(RTC_TypeDef *RTCx, uint32_t CalibMinus)
{
  MODIFY_REG(RTCx->CAL, RTC_CAL_RECALF, CalibMinus);
}

/**
  * @brief  Get smooth Calibration minus
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data=0x00 and Max_Data= 0x1FF
  */
__STATIC_INLINE uint32_t DDL_RTC_CAL_GetMinus(RTC_TypeDef *RTCx)
{
  return (uint32_t)(READ_BIT(RTCx->CAL, RTC_CAL_RECALF));
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get Recalibration pending Flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_RECALP(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_RCALPFLG) == (RTC_STS_RCALPFLG)) ? 1UL : 0UL);
}

#if defined(RTC_TAMPER2_SUPPORT)
/**
  * @brief  Get RTC_TAMP2 detection flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_TAMP2(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_TP2FLG) == (RTC_STS_TP2FLG)) ? 1UL : 0UL);
}
#endif /* RTC_TAMPER2_SUPPORT */

/**
  * @brief  Get RTC_TAMP1 detection flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_TAMP1(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_TP1FLG) == (RTC_STS_TP1FLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Time-stamp overflow flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_TSOV(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_TSOVRFLG) == (RTC_STS_TSOVRFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Time-stamp flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_TS(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_TSFLG) == (RTC_STS_TSFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Wakeup timer flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_WUT(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_WUTFLG) == (RTC_STS_WUTFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Alarm B flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_ALRB(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_ALRBFLG) == (RTC_STS_ALRBFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Alarm A flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_ALRA(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_ALRAFLG) == (RTC_STS_ALRAFLG)) ? 1UL : 0UL);
}

#if defined(RTC_TAMPER2_SUPPORT)
/**
  * @brief  Clear RTC_TAMP2 detection flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_TAMP2(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_TP2FLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}
#endif /* RTC_TAMPER2_SUPPORT */

/**
  * @brief  Clear RTC_TAMP1 detection flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_TAMP1(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_TP1FLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Clear Time-stamp overflow flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_TSOV(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_TSOVRFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Clear Time-stamp flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_TS(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_TSFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Clear Wakeup timer flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_WUT(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_WUTFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Clear Alarm B flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_ALRB(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_ALRBFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Clear Alarm A flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_ALRA(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_ALRAFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Get Initialization flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_INIT(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_RINITFLG) == (RTC_STS_RINITFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Registers synchronization flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_RS(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_RSFLG) == (RTC_STS_RSFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Registers synchronization flag
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_ClearFlag_RS(RTC_TypeDef *RTCx)
{
  WRITE_REG(RTCx->STS, (~((RTC_STS_RSFLG | RTC_STS_INITEN) & 0x0000FFFFU) | (RTCx->STS & RTC_STS_INITEN)));
}

/**
  * @brief  Get Initialization status flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_INITS(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_INITSFLG) == (RTC_STS_INITSFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Shift operation pending flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_SHP(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_SOPFLG) == (RTC_STS_SOPFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Wakeup timer write flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_WUTW(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_WUTWFLG) == (RTC_STS_WUTWFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Alarm B write flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_ALRBW(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_ALRBWFLG) == (RTC_STS_ALRBWFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Get Alarm A write flag
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsActiveFlag_ALRAW(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->STS, RTC_STS_ALRAWFLG) == (RTC_STS_ALRAWFLG)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RTC_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable Time-stamp interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableIT_TS(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_TSIEN);
}

/**
  * @brief  Disable Time-stamp interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableIT_TS(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_TSIEN);
}

/**
  * @brief  Enable Wakeup timer interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableIT_WUT(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_WUTIEN);
}

/**
  * @brief  Disable Wakeup timer interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableIT_WUT(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_WUTIEN);
}

/**
  * @brief  Enable Alarm B interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableIT_ALRB(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_ALRBIEN);
}

/**
  * @brief  Disable Alarm B interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableIT_ALRB(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_ALRBIEN);
}

/**
  * @brief  Enable Alarm A interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableIT_ALRA(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CTRL, RTC_CTRL_ALRAIEN);
}

/**
  * @brief  Disable Alarm A interrupt
  * @note   Bit is write-protected. @ref DDL_RTC_DisableWriteProtection function should be called before.
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableIT_ALRA(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CTRL, RTC_CTRL_ALRAIEN);
}

/**
  * @brief  Enable all Tamper Interrupt
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_EnableIT_TAMP(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->TACFG, RTC_TACFG_TPIEN);
}

/**
  * @brief  Disable all Tamper Interrupt
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RTC_DisableIT_TAMP(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->TACFG, RTC_TACFG_TPIEN);
}

/**
  * @brief  Check if  Time-stamp interrupt is enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsEnabledIT_TS(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_TSIEN) == (RTC_CTRL_TSIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if  Wakeup timer interrupt is enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsEnabledIT_WUT(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_WUTIEN) == (RTC_CTRL_WUTIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if  Alarm B interrupt is enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsEnabledIT_ALRB(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_ALRBIEN) == (RTC_CTRL_ALRBIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if  Alarm A interrupt is enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsEnabledIT_ALRA(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->CTRL, RTC_CTRL_ALRAIEN) == (RTC_CTRL_ALRAIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Check if all the TAMPER interrupts are enabled or not
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RTC_IsEnabledIT_TAMP(RTC_TypeDef *RTCx)
{
  return ((READ_BIT(RTCx->TACFG,
                    RTC_TACFG_TPIEN) == (RTC_TACFG_TPIEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RTC_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_RTC_DeInit(RTC_TypeDef *RTCx);
ErrorStatus DDL_RTC_Init(RTC_TypeDef *RTCx, DDL_RTC_InitTypeDef *RTC_InitStruct);
void        DDL_RTC_StructInit(DDL_RTC_InitTypeDef *RTC_InitStruct);
ErrorStatus DDL_RTC_TIME_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_TimeTypeDef *RTC_TimeStruct);
void        DDL_RTC_TIME_StructInit(DDL_RTC_TimeTypeDef *RTC_TimeStruct);
ErrorStatus DDL_RTC_DATE_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_DateTypeDef *RTC_DateStruct);
void        DDL_RTC_DATE_StructInit(DDL_RTC_DateTypeDef *RTC_DateStruct);
ErrorStatus DDL_RTC_ALMA_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_AlarmTypeDef *RTC_AlarmStruct);
ErrorStatus DDL_RTC_ALMB_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_AlarmTypeDef *RTC_AlarmStruct);
void        DDL_RTC_ALMA_StructInit(DDL_RTC_AlarmTypeDef *RTC_AlarmStruct);
void        DDL_RTC_ALMB_StructInit(DDL_RTC_AlarmTypeDef *RTC_AlarmStruct);
ErrorStatus DDL_RTC_EnterInitMode(RTC_TypeDef *RTCx);
ErrorStatus DDL_RTC_ExitInitMode(RTC_TypeDef *RTCx);
ErrorStatus DDL_RTC_WaitForSynchro(RTC_TypeDef *RTCx);

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

#endif /* defined(RTC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_RTC_H */
