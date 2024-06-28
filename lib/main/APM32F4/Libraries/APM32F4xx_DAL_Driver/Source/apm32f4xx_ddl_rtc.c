/**
  *
  * @file    apm32f4xx_ddl_rtc.c
  * @brief   RTC DDL module driver.
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
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_rtc.h"
#include "apm32f4xx_ddl_cortex.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(RTC)

/** @addtogroup RTC_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup RTC_DDL_Private_Constants
  * @{
  */
/* Default values used for prescaler */
#define RTC_ASYNCH_PRESC_DEFAULT     0x0000007FU
#define RTC_SYNCH_PRESC_DEFAULT      0x000000FFU

/* Values used for timeout */
#define RTC_INITMODE_TIMEOUT         1000U /* 1s when tick set to 1ms */
#define RTC_SYNCHRO_TIMEOUT          1000U /* 1s when tick set to 1ms */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup RTC_DDL_Private_Macros
  * @{
  */

#define IS_DDL_RTC_HOURFORMAT(__VALUE__) (((__VALUE__) == DDL_RTC_HOURFORMAT_24HOUR) \
                                      || ((__VALUE__) == DDL_RTC_HOURFORMAT_AMPM))

#define IS_DDL_RTC_ASYNCH_PREDIV(__VALUE__)   ((__VALUE__) <= 0x7FU)

#define IS_DDL_RTC_SYNCH_PREDIV(__VALUE__)    ((__VALUE__) <= 0x7FFFU)

#define IS_DDL_RTC_FORMAT(__VALUE__) (((__VALUE__) == DDL_RTC_FORMAT_BIN) \
                                  || ((__VALUE__) == DDL_RTC_FORMAT_BCD))

#define IS_DDL_RTC_TIME_FORMAT(__VALUE__) (((__VALUE__) == DDL_RTC_TIME_FORMAT_AM_OR_24) \
                                       || ((__VALUE__) == DDL_RTC_TIME_FORMAT_PM))

#define IS_DDL_RTC_HOUR12(__HOUR__)            (((__HOUR__) > 0U) && ((__HOUR__) <= 12U))
#define IS_DDL_RTC_HOUR24(__HOUR__)            ((__HOUR__) <= 23U)
#define IS_DDL_RTC_MINUTES(__MINUTES__)        ((__MINUTES__) <= 59U)
#define IS_DDL_RTC_SECONDS(__SECONDS__)        ((__SECONDS__) <= 59U)

#define IS_DDL_RTC_WEEKDAY(__VALUE__) (((__VALUE__) == DDL_RTC_WEEKDAY_MONDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_TUESDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_WEDNESDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_THURSDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_FRIDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_SATURDAY) \
                                   || ((__VALUE__) == DDL_RTC_WEEKDAY_SUNDAY))

#define IS_DDL_RTC_DAY(__DAY__)     (((__DAY__) >= 1U) && ((__DAY__) <= 31U))

#define IS_DDL_RTC_MONTH(__MONTH__) (((__MONTH__) >= 1U) && ((__MONTH__) <= 12U))

#define IS_DDL_RTC_YEAR(__YEAR__)   ((__YEAR__) <= 99U)

#define IS_DDL_RTC_ALMA_MASK(__VALUE__) (((__VALUE__) == DDL_RTC_ALMA_MASK_NONE) \
                                     || ((__VALUE__) == DDL_RTC_ALMA_MASK_DATEWEEKDAY) \
                                     || ((__VALUE__) == DDL_RTC_ALMA_MASK_HOURS) \
                                     || ((__VALUE__) == DDL_RTC_ALMA_MASK_MINUTES) \
                                     || ((__VALUE__) == DDL_RTC_ALMA_MASK_SECONDS) \
                                     || ((__VALUE__) == DDL_RTC_ALMA_MASK_ALL))

#define IS_DDL_RTC_ALMB_MASK(__VALUE__) (((__VALUE__) == DDL_RTC_ALMB_MASK_NONE) \
                                     || ((__VALUE__) == DDL_RTC_ALMB_MASK_DATEWEEKDAY) \
                                     || ((__VALUE__) == DDL_RTC_ALMB_MASK_HOURS) \
                                     || ((__VALUE__) == DDL_RTC_ALMB_MASK_MINUTES) \
                                     || ((__VALUE__) == DDL_RTC_ALMB_MASK_SECONDS) \
                                     || ((__VALUE__) == DDL_RTC_ALMB_MASK_ALL))

#define IS_DDL_RTC_ALMA_DATE_WEEKDAY_SEL(__SEL__) (((__SEL__) == DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE) || \
                                                  ((__SEL__) == DDL_RTC_ALMA_DATEWEEKDAYSEL_WEEKDAY))

#define IS_DDL_RTC_ALMB_DATE_WEEKDAY_SEL(__SEL__) (((__SEL__) == DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE) || \
                                                  ((__SEL__) == DDL_RTC_ALMB_DATEWEEKDAYSEL_WEEKDAY))

/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup RTC_DDL_Exported_Functions
  * @{
  */

/** @addtogroup RTC_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-Initializes the RTC registers to their default reset values.
  * @note   This function does not reset the RTC Clock source and RTC Backup Data
  *         registers.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are de-initialized
  *          - ERROR: RTC registers are not de-initialized
  */
ErrorStatus DDL_RTC_DeInit(RTC_TypeDef *RTCx)
{
  ErrorStatus status = ERROR;

  /* Check the parameter */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Set Initialization mode */
  if (DDL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Reset TIME, DATE and CTRL registers */
    DDL_RTC_WriteReg(RTCx, TIME,       0x00000000U);
    DDL_RTC_WriteReg(RTCx, AUTORLD,     RTC_AUTORLD_WUAUTORE);
    DDL_RTC_WriteReg(RTCx, DATE,      (RTC_DATE_WEEKSEL_0 | RTC_DATE_MONU_0 | RTC_DATE_DAYU_0));

    /* Reset All CTRL bits except CTRL[2:0] */
    DDL_RTC_WriteReg(RTCx, CTRL, (DDL_RTC_ReadReg(RTCx, CTRL) & RTC_CTRL_WUCLKSEL));

    DDL_RTC_WriteReg(RTCx, PSC,    (RTC_PSC_APSC | RTC_SYNCH_PRESC_DEFAULT));
    DDL_RTC_WriteReg(RTCx, ALRMA,   0x00000000U);
    DDL_RTC_WriteReg(RTCx, ALRMB,   0x00000000U);
    DDL_RTC_WriteReg(RTCx, CAL,     0x00000000U);
    DDL_RTC_WriteReg(RTCx, SHIFT,   0x00000000U);
    DDL_RTC_WriteReg(RTCx, ALRMASS, 0x00000000U);
    DDL_RTC_WriteReg(RTCx, ALRMBSS, 0x00000000U);

    /* Reset STS register and exit initialization mode */
    DDL_RTC_WriteReg(RTCx, STS,      0x00000000U);

    /* Reset Tamper and alternate functions configuration register */
    DDL_RTC_WriteReg(RTCx, TACFG, 0x00000000U);

    /* Wait till the RTC RSF flag is set */
    status = DDL_RTC_WaitForSynchro(RTCx);
  }

  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return status;
}

/**
  * @brief  Initializes the RTC registers according to the specified parameters
  *         in RTC_InitStruct.
  * @param  RTCx RTC Instance
  * @param  RTC_InitStruct pointer to a @ref DDL_RTC_InitTypeDef structure that contains
  *         the configuration information for the RTC peripheral.
  * @note   The RTC Prescaler register is write protected and can be written in
  *         initialization mode only.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are initialized
  *          - ERROR: RTC registers are not initialized
  */
ErrorStatus DDL_RTC_Init(RTC_TypeDef *RTCx, DDL_RTC_InitTypeDef *RTC_InitStruct)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));
  ASSERT_PARAM(IS_DDL_RTC_HOURFORMAT(RTC_InitStruct->HourFormat));
  ASSERT_PARAM(IS_DDL_RTC_ASYNCH_PREDIV(RTC_InitStruct->AsynchPrescaler));
  ASSERT_PARAM(IS_DDL_RTC_SYNCH_PREDIV(RTC_InitStruct->SynchPrescaler));

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Set Initialization mode */
  if (DDL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Set Hour Format */
    DDL_RTC_SetHourFormat(RTCx, RTC_InitStruct->HourFormat);

    /* Configure Synchronous and Asynchronous prescaler factor */
    DDL_RTC_SetSynchPrescaler(RTCx, RTC_InitStruct->SynchPrescaler);
    DDL_RTC_SetAsynchPrescaler(RTCx, RTC_InitStruct->AsynchPrescaler);

    /* Exit Initialization mode */
    DDL_RTC_DisableInitMode(RTCx);

    status = SUCCESS;
  }
  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return status;
}

/**
  * @brief  Set each @ref DDL_RTC_InitTypeDef field to default value.
  * @param  RTC_InitStruct pointer to a @ref DDL_RTC_InitTypeDef structure which will be initialized.
  * @retval None
  */
void DDL_RTC_StructInit(DDL_RTC_InitTypeDef *RTC_InitStruct)
{
  /* Set RTC_InitStruct fields to default values */
  RTC_InitStruct->HourFormat      = DDL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct->AsynchPrescaler = RTC_ASYNCH_PRESC_DEFAULT;
  RTC_InitStruct->SynchPrescaler  = RTC_SYNCH_PRESC_DEFAULT;
}

/**
  * @brief  Set the RTC current time.
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_FORMAT_BIN
  *         @arg @ref DDL_RTC_FORMAT_BCD
  * @param  RTC_TimeStruct pointer to a RTC_TimeTypeDef structure that contains
  *                        the time configuration information for the RTC.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Time register is configured
  *          - ERROR: RTC Time register is not configured
  */
ErrorStatus DDL_RTC_TIME_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_TimeTypeDef *RTC_TimeStruct)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));
  ASSERT_PARAM(IS_DDL_RTC_FORMAT(RTC_Format));

  if (RTC_Format == DDL_RTC_FORMAT_BIN)
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(RTC_TimeStruct->Hours));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_TimeStruct->TimeFormat));
    }
    else
    {
      RTC_TimeStruct->TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(RTC_TimeStruct->Hours));
    }
    ASSERT_PARAM(IS_DDL_RTC_MINUTES(RTC_TimeStruct->Minutes));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(RTC_TimeStruct->Seconds));
  }
  else
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(__DDL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Hours)));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_TimeStruct->TimeFormat));
    }
    else
    {
      RTC_TimeStruct->TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(__DDL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Hours)));
    }
    ASSERT_PARAM(IS_DDL_RTC_MINUTES(__DDL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Minutes)));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(__DDL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Seconds)));
  }

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Set Initialization mode */
  if (DDL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Check the input parameters format */
    if (RTC_Format != DDL_RTC_FORMAT_BIN)
    {
      DDL_RTC_TIME_Config(RTCx, RTC_TimeStruct->TimeFormat, RTC_TimeStruct->Hours,
                         RTC_TimeStruct->Minutes, RTC_TimeStruct->Seconds);
    }
    else
    {
      DDL_RTC_TIME_Config(RTCx, RTC_TimeStruct->TimeFormat, __DDL_RTC_CONVERT_BIN2BCD(RTC_TimeStruct->Hours),
                         __DDL_RTC_CONVERT_BIN2BCD(RTC_TimeStruct->Minutes),
                         __DDL_RTC_CONVERT_BIN2BCD(RTC_TimeStruct->Seconds));
    }

    /* Exit Initialization mode */
    DDL_RTC_DisableInitMode(RTCx);

    /* If  RTC_CTRL_RCMCFG bit = 0, wait for synchro else this check is not needed */
    if (DDL_RTC_IsShadowRegBypassEnabled(RTCx) == 0U)
    {
      status = DDL_RTC_WaitForSynchro(RTCx);
    }
    else
    {
      status = SUCCESS;
    }
  }
  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return status;
}

/**
  * @brief  Set each @ref DDL_RTC_TimeTypeDef field to default value (Time = 00h:00min:00sec).
  * @param  RTC_TimeStruct pointer to a @ref DDL_RTC_TimeTypeDef structure which will be initialized.
  * @retval None
  */
void DDL_RTC_TIME_StructInit(DDL_RTC_TimeTypeDef *RTC_TimeStruct)
{
  /* Time = 00h:00min:00sec */
  RTC_TimeStruct->TimeFormat = DDL_RTC_TIME_FORMAT_AM_OR_24;
  RTC_TimeStruct->Hours      = 0U;
  RTC_TimeStruct->Minutes    = 0U;
  RTC_TimeStruct->Seconds    = 0U;
}

/**
  * @brief  Set the RTC current date.
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_FORMAT_BIN
  *         @arg @ref DDL_RTC_FORMAT_BCD
  * @param  RTC_DateStruct pointer to a RTC_DateTypeDef structure that contains
  *                         the date configuration information for the RTC.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Day register is configured
  *          - ERROR: RTC Day register is not configured
  */
ErrorStatus DDL_RTC_DATE_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_DateTypeDef *RTC_DateStruct)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));
  ASSERT_PARAM(IS_DDL_RTC_FORMAT(RTC_Format));

  if ((RTC_Format == DDL_RTC_FORMAT_BIN) && ((RTC_DateStruct->Month & 0x10U) == 0x10U))
  {
    RTC_DateStruct->Month = (uint8_t)(RTC_DateStruct->Month & (uint8_t)~(0x10U)) + 0x0AU;
  }
  if (RTC_Format == DDL_RTC_FORMAT_BIN)
  {
    ASSERT_PARAM(IS_DDL_RTC_YEAR(RTC_DateStruct->Year));
    ASSERT_PARAM(IS_DDL_RTC_MONTH(RTC_DateStruct->Month));
    ASSERT_PARAM(IS_DDL_RTC_DAY(RTC_DateStruct->Day));
  }
  else
  {
    ASSERT_PARAM(IS_DDL_RTC_YEAR(__DDL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Year)));
    ASSERT_PARAM(IS_DDL_RTC_MONTH(__DDL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Month)));
    ASSERT_PARAM(IS_DDL_RTC_DAY(__DDL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Day)));
  }
  ASSERT_PARAM(IS_DDL_RTC_WEEKDAY(RTC_DateStruct->WeekDay));

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Set Initialization mode */
  if (DDL_RTC_EnterInitMode(RTCx) != ERROR)
  {
    /* Check the input parameters format */
    if (RTC_Format != DDL_RTC_FORMAT_BIN)
    {
      DDL_RTC_DATE_Config(RTCx, RTC_DateStruct->WeekDay, RTC_DateStruct->Day, RTC_DateStruct->Month, RTC_DateStruct->Year);
    }
    else
    {
      DDL_RTC_DATE_Config(RTCx, RTC_DateStruct->WeekDay, __DDL_RTC_CONVERT_BIN2BCD(RTC_DateStruct->Day),
                         __DDL_RTC_CONVERT_BIN2BCD(RTC_DateStruct->Month), __DDL_RTC_CONVERT_BIN2BCD(RTC_DateStruct->Year));
    }

    /* Exit Initialization mode */
    DDL_RTC_DisableInitMode(RTCx);

    /* If  RTC_CTRL_RCMCFG bit = 0, wait for synchro else this check is not needed */
    if (DDL_RTC_IsShadowRegBypassEnabled(RTCx) == 0U)
    {
      status = DDL_RTC_WaitForSynchro(RTCx);
    }
    else
    {
      status = SUCCESS;
    }
  }
  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return status;
}

/**
  * @brief  Set each @ref DDL_RTC_DateTypeDef field to default value (date = Monday, January 01 xx00)
  * @param  RTC_DateStruct pointer to a @ref DDL_RTC_DateTypeDef structure which will be initialized.
  * @retval None
  */
void DDL_RTC_DATE_StructInit(DDL_RTC_DateTypeDef *RTC_DateStruct)
{
  /* Monday, January 01 xx00 */
  RTC_DateStruct->WeekDay = DDL_RTC_WEEKDAY_MONDAY;
  RTC_DateStruct->Day     = 1U;
  RTC_DateStruct->Month   = DDL_RTC_MONTH_JANUARY;
  RTC_DateStruct->Year    = 0U;
}

/**
  * @brief  Set the RTC Alarm A.
  * @note   The Alarm register can only be written when the corresponding Alarm
  *         is disabled (Use @ref DDL_RTC_ALMA_Disable function).
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_FORMAT_BIN
  *         @arg @ref DDL_RTC_FORMAT_BCD
  * @param  RTC_AlarmStruct pointer to a @ref DDL_RTC_AlarmTypeDef structure that
  *                         contains the alarm configuration parameters.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ALARMA registers are configured
  *          - ERROR: ALARMA registers are not configured
  */
ErrorStatus DDL_RTC_ALMA_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));
  ASSERT_PARAM(IS_DDL_RTC_FORMAT(RTC_Format));
  ASSERT_PARAM(IS_DDL_RTC_ALMA_MASK(RTC_AlarmStruct->AlarmMask));
  ASSERT_PARAM(IS_DDL_RTC_ALMA_DATE_WEEKDAY_SEL(RTC_AlarmStruct->AlarmDateWeekDaySel));

  if (RTC_Format == DDL_RTC_FORMAT_BIN)
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(RTC_AlarmStruct->AlarmTime.Hours));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_AlarmStruct->AlarmTime.TimeFormat));
    }
    else
    {
      RTC_AlarmStruct->AlarmTime.TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(RTC_AlarmStruct->AlarmTime.Hours));
    }
    ASSERT_PARAM(IS_DDL_RTC_MINUTES(RTC_AlarmStruct->AlarmTime.Minutes));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(RTC_AlarmStruct->AlarmTime.Seconds));

    if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE)
    {
      ASSERT_PARAM(IS_DDL_RTC_DAY(RTC_AlarmStruct->AlarmDateWeekDay));
    }
    else
    {
      ASSERT_PARAM(IS_DDL_RTC_WEEKDAY(RTC_AlarmStruct->AlarmDateWeekDay));
    }
  }
  else
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_AlarmStruct->AlarmTime.TimeFormat));
    }
    else
    {
      RTC_AlarmStruct->AlarmTime.TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)));
    }

    ASSERT_PARAM(IS_DDL_RTC_MINUTES(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Minutes)));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Seconds)));

    if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE)
    {
      ASSERT_PARAM(IS_DDL_RTC_DAY(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmDateWeekDay)));
    }
    else
    {
      ASSERT_PARAM(IS_DDL_RTC_WEEKDAY(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmDateWeekDay)));
    }
  }

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Select weekday selection */
  if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE)
  {
    /* Set the date for ALARM */
    DDL_RTC_ALMA_DisableWeekday(RTCx);
    if (RTC_Format != DDL_RTC_FORMAT_BIN)
    {
      DDL_RTC_ALMA_SetDay(RTCx, RTC_AlarmStruct->AlarmDateWeekDay);
    }
    else
    {
      DDL_RTC_ALMA_SetDay(RTCx, __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmDateWeekDay));
    }
  }
  else
  {
    /* Set the week day for ALARM */
    DDL_RTC_ALMA_EnableWeekday(RTCx);
    DDL_RTC_ALMA_SetWeekDay(RTCx, RTC_AlarmStruct->AlarmDateWeekDay);
  }

  /* Configure the Alarm register */
  if (RTC_Format != DDL_RTC_FORMAT_BIN)
  {
    DDL_RTC_ALMA_ConfigTime(RTCx, RTC_AlarmStruct->AlarmTime.TimeFormat, RTC_AlarmStruct->AlarmTime.Hours,
                           RTC_AlarmStruct->AlarmTime.Minutes, RTC_AlarmStruct->AlarmTime.Seconds);
  }
  else
  {
    DDL_RTC_ALMA_ConfigTime(RTCx, RTC_AlarmStruct->AlarmTime.TimeFormat,
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Hours),
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Minutes),
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Seconds));
  }
  /* Set ALARM mask */
  DDL_RTC_ALMA_SetMask(RTCx, RTC_AlarmStruct->AlarmMask);

  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return SUCCESS;
}

/**
  * @brief  Set the RTC Alarm B.
  * @note   The Alarm register can only be written when the corresponding Alarm
  *         is disabled (@ref DDL_RTC_ALMB_Disable function).
  * @param  RTCx RTC Instance
  * @param  RTC_Format This parameter can be one of the following values:
  *         @arg @ref DDL_RTC_FORMAT_BIN
  *         @arg @ref DDL_RTC_FORMAT_BCD
  * @param  RTC_AlarmStruct pointer to a @ref DDL_RTC_AlarmTypeDef structure that
  *                         contains the alarm configuration parameters.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ALARMB registers are configured
  *          - ERROR: ALARMB registers are not configured
  */
ErrorStatus DDL_RTC_ALMB_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, DDL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));
  ASSERT_PARAM(IS_DDL_RTC_FORMAT(RTC_Format));
  ASSERT_PARAM(IS_DDL_RTC_ALMB_MASK(RTC_AlarmStruct->AlarmMask));
  ASSERT_PARAM(IS_DDL_RTC_ALMB_DATE_WEEKDAY_SEL(RTC_AlarmStruct->AlarmDateWeekDaySel));

  if (RTC_Format == DDL_RTC_FORMAT_BIN)
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(RTC_AlarmStruct->AlarmTime.Hours));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_AlarmStruct->AlarmTime.TimeFormat));
    }
    else
    {
      RTC_AlarmStruct->AlarmTime.TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(RTC_AlarmStruct->AlarmTime.Hours));
    }
    ASSERT_PARAM(IS_DDL_RTC_MINUTES(RTC_AlarmStruct->AlarmTime.Minutes));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(RTC_AlarmStruct->AlarmTime.Seconds));

    if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE)
    {
      ASSERT_PARAM(IS_DDL_RTC_DAY(RTC_AlarmStruct->AlarmDateWeekDay));
    }
    else
    {
      ASSERT_PARAM(IS_DDL_RTC_WEEKDAY(RTC_AlarmStruct->AlarmDateWeekDay));
    }
  }
  else
  {
    if (DDL_RTC_GetHourFormat(RTCx) != DDL_RTC_HOURFORMAT_24HOUR)
    {
      ASSERT_PARAM(IS_DDL_RTC_HOUR12(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)));
      ASSERT_PARAM(IS_DDL_RTC_TIME_FORMAT(RTC_AlarmStruct->AlarmTime.TimeFormat));
    }
    else
    {
      RTC_AlarmStruct->AlarmTime.TimeFormat = 0x00U;
      ASSERT_PARAM(IS_DDL_RTC_HOUR24(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Hours)));
    }

    ASSERT_PARAM(IS_DDL_RTC_MINUTES(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Minutes)));
    ASSERT_PARAM(IS_DDL_RTC_SECONDS(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmTime.Seconds)));

    if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE)
    {
      ASSERT_PARAM(IS_DDL_RTC_DAY(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmDateWeekDay)));
    }
    else
    {
      ASSERT_PARAM(IS_DDL_RTC_WEEKDAY(__DDL_RTC_CONVERT_BCD2BIN(RTC_AlarmStruct->AlarmDateWeekDay)));
    }
  }

  /* Disable the write protection for RTC registers */
  DDL_RTC_DisableWriteProtection(RTCx);

  /* Select weekday selection */
  if (RTC_AlarmStruct->AlarmDateWeekDaySel == DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE)
  {
    /* Set the date for ALARM */
    DDL_RTC_ALMB_DisableWeekday(RTCx);
    if (RTC_Format != DDL_RTC_FORMAT_BIN)
    {
      DDL_RTC_ALMB_SetDay(RTCx, RTC_AlarmStruct->AlarmDateWeekDay);
    }
    else
    {
      DDL_RTC_ALMB_SetDay(RTCx, __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmDateWeekDay));
    }
  }
  else
  {
    /* Set the week day for ALARM */
    DDL_RTC_ALMB_EnableWeekday(RTCx);
    DDL_RTC_ALMB_SetWeekDay(RTCx, RTC_AlarmStruct->AlarmDateWeekDay);
  }

  /* Configure the Alarm register */
  if (RTC_Format != DDL_RTC_FORMAT_BIN)
  {
    DDL_RTC_ALMB_ConfigTime(RTCx, RTC_AlarmStruct->AlarmTime.TimeFormat, RTC_AlarmStruct->AlarmTime.Hours,
                           RTC_AlarmStruct->AlarmTime.Minutes, RTC_AlarmStruct->AlarmTime.Seconds);
  }
  else
  {
    DDL_RTC_ALMB_ConfigTime(RTCx, RTC_AlarmStruct->AlarmTime.TimeFormat,
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Hours),
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Minutes),
                           __DDL_RTC_CONVERT_BIN2BCD(RTC_AlarmStruct->AlarmTime.Seconds));
  }
  /* Set ALARM mask */
  DDL_RTC_ALMB_SetMask(RTCx, RTC_AlarmStruct->AlarmMask);

  /* Enable the write protection for RTC registers */
  DDL_RTC_EnableWriteProtection(RTCx);

  return SUCCESS;
}

/**
  * @brief  Set each @ref DDL_RTC_AlarmTypeDef of ALARMA field to default value (Time = 00h:00mn:00sec /
  *         Day = 1st day of the month/Mask = all fields are masked).
  * @param  RTC_AlarmStruct pointer to a @ref DDL_RTC_AlarmTypeDef structure which will be initialized.
  * @retval None
  */
void DDL_RTC_ALMA_StructInit(DDL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  /* Alarm Time Settings : Time = 00h:00mn:00sec */
  RTC_AlarmStruct->AlarmTime.TimeFormat = DDL_RTC_ALMA_TIME_FORMAT_AM;
  RTC_AlarmStruct->AlarmTime.Hours      = 0U;
  RTC_AlarmStruct->AlarmTime.Minutes    = 0U;
  RTC_AlarmStruct->AlarmTime.Seconds    = 0U;

  /* Alarm Day Settings : Day = 1st day of the month */
  RTC_AlarmStruct->AlarmDateWeekDaySel = DDL_RTC_ALMA_DATEWEEKDAYSEL_DATE;
  RTC_AlarmStruct->AlarmDateWeekDay    = 1U;

  /* Alarm Masks Settings : Mask =  all fields are not masked */
  RTC_AlarmStruct->AlarmMask           = DDL_RTC_ALMA_MASK_NONE;
}

/**
  * @brief  Set each @ref DDL_RTC_AlarmTypeDef of ALARMA field to default value (Time = 00h:00mn:00sec /
  *         Day = 1st day of the month/Mask = all fields are masked).
  * @param  RTC_AlarmStruct pointer to a @ref DDL_RTC_AlarmTypeDef structure which will be initialized.
  * @retval None
  */
void DDL_RTC_ALMB_StructInit(DDL_RTC_AlarmTypeDef *RTC_AlarmStruct)
{
  /* Alarm Time Settings : Time = 00h:00mn:00sec */
  RTC_AlarmStruct->AlarmTime.TimeFormat = DDL_RTC_ALMB_TIME_FORMAT_AM;
  RTC_AlarmStruct->AlarmTime.Hours      = 0U;
  RTC_AlarmStruct->AlarmTime.Minutes    = 0U;
  RTC_AlarmStruct->AlarmTime.Seconds    = 0U;

  /* Alarm Day Settings : Day = 1st day of the month */
  RTC_AlarmStruct->AlarmDateWeekDaySel = DDL_RTC_ALMB_DATEWEEKDAYSEL_DATE;
  RTC_AlarmStruct->AlarmDateWeekDay    = 1U;

  /* Alarm Masks Settings : Mask =  all fields are not masked */
  RTC_AlarmStruct->AlarmMask           = DDL_RTC_ALMB_MASK_NONE;
}

/**
  * @brief  Enters the RTC Initialization mode.
  * @note   The RTC Initialization mode is write protected, use the
  *         @ref DDL_RTC_DisableWriteProtection before calling this function.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC is in Init mode
  *          - ERROR: RTC is not in Init mode
  */
ErrorStatus DDL_RTC_EnterInitMode(RTC_TypeDef *RTCx)
{
  __IO uint32_t timeout = RTC_INITMODE_TIMEOUT;
  ErrorStatus status = SUCCESS;
  uint32_t tmp = 0U;

  /* Check the parameter */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));

  /* Check if the Initialization mode is set */
  if (DDL_RTC_IsActiveFlag_INIT(RTCx) == 0U)
  {
    /* Set the Initialization mode */
    DDL_RTC_EnableInitMode(RTCx);

    /* Wait till RTC is in INIT state and if Time out is reached exit */
    tmp = DDL_RTC_IsActiveFlag_INIT(RTCx);
    while ((timeout != 0U) && (tmp != 1U))
    {
      if (DDL_SYSTICK_IsActiveCounterFlag() == 1U)
      {
        timeout --;
      }
      tmp = DDL_RTC_IsActiveFlag_INIT(RTCx);
      if (timeout == 0U)
      {
        status = ERROR;
      }
    }
  }
  return status;
}

/**
  * @brief  Exit the RTC Initialization mode.
  * @note   When the initialization sequence is complete, the calendar restarts
  *         counting after 4 RTCCLK cycles.
  * @note   The RTC Initialization mode is write protected, use the
  *         @ref DDL_RTC_DisableWriteProtection before calling this function.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC exited from in Init mode
  *          - ERROR: Not applicable
  */
ErrorStatus DDL_RTC_ExitInitMode(RTC_TypeDef *RTCx)
{
  /* Check the parameter */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));

  /* Disable initialization mode */
  DDL_RTC_DisableInitMode(RTCx);

  return SUCCESS;
}

/**
  * @brief  Waits until the RTC Time and Day registers (RTC_TIME and RTC_DATE) are
  *         synchronized with RTC APB clock.
  * @note   The RTC Resynchronization mode is write protected, use the
  *         @ref DDL_RTC_DisableWriteProtection before calling this function.
  * @note   To read the calendar through the shadow registers after calendar
  *         initialization, calendar update or after wakeup from low power modes
  *         the software must first clear the RSF flag.
  *         The software must then wait until it is set again before reading
  *         the calendar, which means that the calendar registers have been
  *         correctly copied into the RTC_TIME and RTC_DATE shadow registers.
  * @param  RTCx RTC Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are synchronised
  *          - ERROR: RTC registers are not synchronised
  */
ErrorStatus DDL_RTC_WaitForSynchro(RTC_TypeDef *RTCx)
{
  __IO uint32_t timeout = RTC_SYNCHRO_TIMEOUT;
  ErrorStatus status = SUCCESS;
  uint32_t tmp = 0U;

  /* Check the parameter */
  ASSERT_PARAM(IS_RTC_ALL_INSTANCE(RTCx));

  /* Clear RSF flag */
  DDL_RTC_ClearFlag_RS(RTCx);

  /* Wait the registers to be synchronised */
  tmp = DDL_RTC_IsActiveFlag_RS(RTCx);
  while ((timeout != 0U) && (tmp != 1U))
  {
    if (DDL_SYSTICK_IsActiveCounterFlag() == 1U)
    {
      timeout--;
    }
    tmp = DDL_RTC_IsActiveFlag_RS(RTCx);
    if (timeout == 0U)
    {
      status = ERROR;
    }
  }

  return (status);
}

/**
  * @}
  */

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

#endif /* USE_FULL_DDL_DRIVER */
