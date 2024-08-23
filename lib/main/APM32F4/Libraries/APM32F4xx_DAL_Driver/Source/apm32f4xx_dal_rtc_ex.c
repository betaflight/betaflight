/**
  *
  * @file    apm32f4xx_dal_rtc_ex.c
  * @brief   Extended RTC DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Real-Time Clock (RTC) Extended peripheral:
  *           + RTC Timestamp functions
  *           + RTC Tamper functions
  *           + RTC Wakeup functions
  *           + Extended Control functions
  *           + Extended RTC features functions
  *
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
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
    (+) Enable the RTC domain access.
    (+) Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour
        format using the DAL_RTC_Init() function.

  *** RTC Wakeup configuration ***
  ================================
  [..]
    (+) To configure the RTC Wakeup Clock source and Counter use the
        DAL_RTCEx_SetWakeUpTimer() function.
        You can also configure the RTC Wakeup timer in interrupt mode using the
        DAL_RTCEx_SetWakeUpTimer_IT() function.
    (+) To read the RTC Wakeup Counter register, use the DAL_RTCEx_GetWakeUpTimer()
        function.

  *** Timestamp configuration ***
  ===============================
  [..]
    (+) To configure the RTC Timestamp use the DAL_RTCEx_SetTimeStamp() function.
        You can also configure the RTC Timestamp with interrupt mode using the
        DAL_RTCEx_SetTimeStamp_IT() function.
    (+) To read the RTC Timestamp Time and Date register, use the
        DAL_RTCEx_GetTimeStamp() function.
    (+) The Timestamp alternate function can be mapped either to RTC_AF1 (PC13)
        or RTC_AF2 (PI8) depending on the value of TSINSEL bit in RTC_TACFG
        register.

  *** Tamper configuration ***
  ============================
  [..]
    (+) To Enable the RTC Tamper and configure the Tamper filter count, trigger
        Edge or Level according to the Tamper filter value (if equal to 0 Edge
        else Level), sampling frequency, precharge or discharge and Pull-UP use
        the DAL_RTCEx_SetTamper() function.
        You can configure RTC Tamper in interrupt mode using DAL_RTCEx_SetTamper_IT()
        function.
    (+) The TAMPER1 alternate function can be mapped either to RTC_AF1 (PC13)
        or RTC_AF2 (PI8) depending on the value of TAMP1INSEL bit in RTC_TACFG
        register.
        The corresponding pin is also selected by DAL_RTCEx_SetTamper()
        or DAL_RTCEx_SetTamper_IT() functions.
    (+) The TAMPER2 alternate function is mapped to RTC_AF2 (PI8).

  *** Backup Data Registers configuration ***
  ===========================================
  [..]
    (+) To write to the RTC Backup Data registers, use the DAL_RTCEx_BKUPWrite()
        function.
    (+) To read the RTC Backup Data registers, use the DAL_RTCEx_BKUPRead()
        function.

  *** Coarse Digital Calibration configuration ***
  ================================================
  [..]
    (+) The Coarse Digital Calibration can be used to compensate crystal inaccuracy
        by setting the DCS bit in RTC_DCAL register.
    (+) When positive calibration is enabled (DCS = ��0��), 2 asynchronous prescaler
        clock cycles are added every minute during 2xDC minutes.
        This causes the calendar to be updated sooner, thereby adjusting the
        effective RTC frequency to be a bit higher.
    (+) When negative calibration is enabled (DCS = ��1��), 1 asynchronous prescaler
        clock cycle is removed every minute during 2xDC minutes.
        This causes the calendar to be updated later, thereby adjusting the
        effective RTC frequency to be a bit lower.
    (+) DC is configured through bits DC[4:0] of RTC_DCAL register. This number
        ranges from 0 to 31 corresponding to a time interval (2xDC) ranging from
        0 to 62.
    (+) In order to measure the clock deviation, a 512 Hz clock is output for
        calibration.
    (+) The RTC Coarse Digital Calibration value and sign can be calibrated using
        the DAL_RTCEx_SetCoarseCalib() function.

  *** Smooth Digital Calibration configuration ***
  ================================================
  [..]
    (+) RTC frequency can be digitally calibrated with a resolution of about
        0.954 ppm with a range from -487.1 ppm to +488.5 ppm.
        The correction of the frequency is performed using a series of small
        adjustments (adding and/or subtracting individual RTCCLK pulses).
    (+) The smooth digital calibration is performed during a cycle of about 2^20
        RTCCLK pulses (or 32 seconds) when the input frequency is 32,768 Hz.
        This cycle is maintained by a 20-bit counter clocked by RTCCLK.
    (+) The smooth calibration register (RTC_CAL) specifies the number of RTCCLK
        clock cycles to be masked during the 32-second cycle.
    (+) The RTC Smooth Digital Calibration value and the corresponding calibration
        cycle period (32s, 16s, or 8s) can be calibrated using the
        DAL_RTCEx_SetSmoothCalib() function.

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup RTCEx RTCEx
  * @brief    RTC Extended DAL module driver
  * @{
  */

#ifdef DAL_RTC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RTCEx_Exported_Functions RTCEx Exported Functions
  * @{
  */

/** @defgroup RTCEx_Exported_Functions_Group1 RTC Timestamp and Tamper functions
  * @brief    RTC Timestamp and Tamper functions
  *
@verbatim
 ===============================================================================
                 ##### RTC Timestamp and Tamper functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Timestamp feature

@endverbatim
  * @{
  */

/**
  * @brief  Sets Timestamp.
  * @note   This API must be called before enabling the Timestamp feature.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  RTC_TimeStampEdge Specifies the pin edge on which the Timestamp is
  *         activated.
  *          This parameter can be one of the following values:
  *             @arg RTC_TIMESTAMPEDGE_RISING: the Timestamp event occurs on
  *                                        the rising edge of the related pin.
  *             @arg RTC_TIMESTAMPEDGE_FALLING: the Timestamp event occurs on
  *                                        the falling edge of the related pin.
  * @param  RTC_TimeStampPin Specifies the RTC Timestamp Pin.
  *          This parameter can be one of the following values:
  *             @arg RTC_TIMESTAMPPIN_DEFAULT: PC13 is selected as RTC Timestamp Pin.
  *             @arg RTC_TIMESTAMPPIN_POS1: PI8 is selected as RTC Timestamp Pin.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t RTC_TimeStampEdge, uint32_t RTC_TimeStampPin)
{
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_TIMESTAMP_EDGE(RTC_TimeStampEdge));
  ASSERT_PARAM(IS_RTC_TIMESTAMP_PIN(RTC_TimeStampPin));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  /* Change RTC state to BUSY */
  hrtc->State = DAL_RTC_STATE_BUSY;

  hrtc->Instance->TACFG &= (uint32_t)~RTC_TACFG_TSMSEL;
  hrtc->Instance->TACFG |= (uint32_t)(RTC_TimeStampPin);

  /* Get the RTC_CTRL register and clear the bits to be configured */
  tmpreg = (uint32_t)(hrtc->Instance->CTRL & (uint32_t)~(RTC_CTRL_TSETECFG | RTC_CTRL_TSEN));

  /* Configure the Timestamp TSEDGE bit */
  tmpreg |= RTC_TimeStampEdge;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Copy the desired configuration into the CTRL register */
  hrtc->Instance->CTRL = (uint32_t)tmpreg;

  /* Clear RTC Timestamp flag */
  __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSF);

  /* Clear RTC Timestamp overrun Flag */
  __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSOVF);

  /* Enable the Timestamp saving */
  __DAL_RTC_TIMESTAMP_ENABLE(hrtc);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state back to READY */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Sets Timestamp with Interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   This API must be called before enabling the Timestamp feature.
  * @param  RTC_TimeStampEdge Specifies the pin edge on which the Timestamp is
  *         activated.
  *          This parameter can be one of the following values:
  *             @arg RTC_TIMESTAMPEDGE_RISING: the Timestamp event occurs on
  *                                        the rising edge of the related pin.
  *             @arg RTC_TIMESTAMPEDGE_FALLING: the Timestamp event occurs on
  *                                        the falling edge of the related pin.
  * @param  RTC_TimeStampPin Specifies the RTC Timestamp Pin.
  *          This parameter can be one of the following values:
  *             @arg RTC_TIMESTAMPPIN_DEFAULT: PC13 is selected as RTC Timestamp Pin.
  *             @arg RTC_TIMESTAMPPIN_POS1: PI8 is selected as RTC Timestamp Pin.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t RTC_TimeStampEdge, uint32_t RTC_TimeStampPin)
{
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_TIMESTAMP_EDGE(RTC_TimeStampEdge));
  ASSERT_PARAM(IS_RTC_TIMESTAMP_PIN(RTC_TimeStampPin));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  /* Change RTC state to BUSY */
  hrtc->State = DAL_RTC_STATE_BUSY;

  hrtc->Instance->TACFG &= (uint32_t)~RTC_TACFG_TSMSEL;
  hrtc->Instance->TACFG |= (uint32_t)(RTC_TimeStampPin);

  /* Get the RTC_CTRL register and clear the bits to be configured */
  tmpreg = (uint32_t)(hrtc->Instance->CTRL & (uint32_t)~(RTC_CTRL_TSETECFG | RTC_CTRL_TSEN));

  /* Configure the Timestamp TSETECFG bit */
  tmpreg |= RTC_TimeStampEdge;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Copy the desired configuration into the CTRL register */
  hrtc->Instance->CTRL = (uint32_t)tmpreg;

  /* Clear RTC Timestamp flag */
  __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSF);

  /* Clear RTC Timestamp overrun Flag */
  __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSOVF);

  /* Enable the Timestamp saving */
  __DAL_RTC_TIMESTAMP_ENABLE(hrtc);

  /* Enable IT Timestamp */
  __DAL_RTC_TIMESTAMP_ENABLE_IT(hrtc, RTC_IT_TS);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* RTC Timestamp Interrupt Configuration: EINT configuration */
  __DAL_RTC_TAMPER_TIMESTAMP_EINT_ENABLE_IT();
  __DAL_RTC_TAMPER_TIMESTAMP_EINT_ENABLE_RISING_EDGE();

  /* Change RTC state back to READY */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Deactivates Timestamp.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc)
{
  uint32_t tmpreg = 0U;

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* In case of interrupt mode is used, the interrupt source must disabled */
  __DAL_RTC_TIMESTAMP_DISABLE_IT(hrtc, RTC_IT_TS);

  /* Get the RTC_CTRL register and clear the bits to be configured */
  tmpreg = (uint32_t)(hrtc->Instance->CTRL & (uint32_t)~(RTC_CTRL_TSETECFG | RTC_CTRL_TSEN));

  /* Configure the Timestamp TSETECFG and Enable bits */
  hrtc->Instance->CTRL = (uint32_t)tmpreg;

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Gets the RTC Timestamp value.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTimeStamp Pointer to Time structure
  * @param  sTimeStampDate Pointer to Date structure
  * @param  Format specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format)
{
  uint32_t tmptime = 0U;
  uint32_t tmpdate = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_FORMAT(Format));

  /* Get the Timestamp time and date registers values */
  tmptime = (uint32_t)(hrtc->Instance->TSTIME & RTC_TIME_RESERVED_MASK);
  tmpdate = (uint32_t)(hrtc->Instance->TSDATE & RTC_DATE_RESERVED_MASK);

  /* Fill the Time structure fields with the read parameters */
  sTimeStamp->Hours      = (uint8_t)((tmptime & (RTC_TSTIME_HRT  | RTC_TSTIME_HRU))  >> RTC_TSTIME_HRU_Pos);
  sTimeStamp->Minutes    = (uint8_t)((tmptime & (RTC_TSTIME_MINT | RTC_TSTIME_MINU)) >> RTC_TSTIME_MINU_Pos);
  sTimeStamp->Seconds    = (uint8_t)((tmptime & (RTC_TSTIME_SECT  | RTC_TSTIME_SECU))  >> RTC_TSTIME_SECU_Pos);
  sTimeStamp->TimeFormat = (uint8_t)((tmptime & (RTC_TSTIME_TIMEFCFG))                 >> RTC_TSTIME_TIMEFCFG_Pos);
  sTimeStamp->SubSeconds = (uint32_t) hrtc->Instance->TSSUBSEC;

  /* Fill the Date structure fields with the read parameters */
  sTimeStampDate->Year    = 0U;
  sTimeStampDate->Month   = (uint8_t)((tmpdate & (RTC_TSDATE_MONT | RTC_TSDATE_MONU)) >> RTC_TSDATE_MONU_Pos);
  sTimeStampDate->Date    = (uint8_t)((tmpdate & (RTC_TSDATE_DAYT | RTC_TSDATE_DAYU)) >> RTC_TSDATE_DAYU_Pos);
  sTimeStampDate->WeekDay = (uint8_t)((tmpdate & (RTC_TSDATE_WEEKSEL))              >> RTC_TSDATE_WEEKSEL_Pos);

  /* Check the input parameters format */
  if (Format == RTC_FORMAT_BIN)
  {
    /* Convert the Timestamp structure parameters to Binary format */
    sTimeStamp->Hours   = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Hours);
    sTimeStamp->Minutes = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Minutes);
    sTimeStamp->Seconds = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Seconds);

    /* Convert the DateTimeStamp structure parameters to Binary format */
    sTimeStampDate->Month   = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->Month);
    sTimeStampDate->Date    = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->Date);
    sTimeStampDate->WeekDay = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->WeekDay);
  }

  /* Clear the Timestamp Flag */
  __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSF);

  return DAL_OK;
}

/**
  * @brief  Sets Tamper.
  * @note   By calling this API the tamper global interrupt will be disabled.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTamper Pointer to Tamper Structure.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper)
{
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_TAMPER(sTamper->Tamper));
  ASSERT_PARAM(IS_RTC_TAMPER_PIN(sTamper->PinSelection));
  ASSERT_PARAM(IS_RTC_TAMPER_TRIGGER(sTamper->Trigger));
  ASSERT_PARAM(IS_RTC_TAMPER_FILTER(sTamper->Filter));
  ASSERT_PARAM(IS_RTC_TAMPER_FILTER_CONFIG_CORRECT(sTamper->Filter, sTamper->Trigger));
  ASSERT_PARAM(IS_RTC_TAMPER_SAMPLING_FREQ(sTamper->SamplingFrequency));
  ASSERT_PARAM(IS_RTC_TAMPER_PRECHARGE_DURATION(sTamper->PrechargeDuration));
  ASSERT_PARAM(IS_RTC_TAMPER_PULLUP_STATE(sTamper->TamperPullUp));
  ASSERT_PARAM(IS_RTC_TAMPER_TIMESTAMPONTAMPER_DETECTION(sTamper->TimeStampOnTamperDetection));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Copy control register into temporary variable */
  tmpreg = hrtc->Instance->TACFG;

  /* Enable selected tamper */
  tmpreg |= (sTamper->Tamper);

  /* Configure the tamper trigger bit (this bit is just on the right of the
       tamper enable bit, hence the one-time right shift before updating it) */
  if (sTamper->Trigger == RTC_TAMPERTRIGGER_FALLINGEDGE)
  {
    /* Set the tamper trigger bit (case of falling edge or high level) */
    tmpreg |= (uint32_t)(sTamper->Tamper << 1U);
  }
  else
  {
    /* Clear the tamper trigger bit (case of rising edge or low level) */
    tmpreg &= (uint32_t)~(sTamper->Tamper << 1U);
  }

  /* Clear remaining fields before setting them */
  tmpreg &= ~(RTC_TAMPERFILTER_MASK              | \
              RTC_TAMPERSAMPLINGFREQ_RTCCLK_MASK | \
              RTC_TAMPERPRECHARGEDURATION_MASK   | \
              RTC_TAMPER_PULLUP_MASK             | \
              RTC_TACFG_TP1MSEL               | \
              RTC_TIMESTAMPONTAMPERDETECTION_MASK);

  /* Set remaining parameters of desired configuration into temporary variable */
  tmpreg |= ((uint32_t)sTamper->Filter            | \
             (uint32_t)sTamper->SamplingFrequency | \
             (uint32_t)sTamper->PrechargeDuration | \
             (uint32_t)sTamper->TamperPullUp      | \
             (uint32_t)sTamper->PinSelection      | \
             (uint32_t)sTamper->TimeStampOnTamperDetection);

  /* Disable tamper global interrupt in case it is enabled */
  tmpreg &= (uint32_t)~RTC_TACFG_TPIEN;

  /* Copy desired configuration into configuration register */
  hrtc->Instance->TACFG = tmpreg;

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Sets Tamper with interrupt.
  * @note   By calling this API the tamper global interrupt will be enabled.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTamper Pointer to RTC Tamper.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper)
{
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_TAMPER(sTamper->Tamper));
  ASSERT_PARAM(IS_RTC_TAMPER_PIN(sTamper->PinSelection));
  ASSERT_PARAM(IS_RTC_TAMPER_TRIGGER(sTamper->Trigger));
  ASSERT_PARAM(IS_RTC_TAMPER_FILTER(sTamper->Filter));
  ASSERT_PARAM(IS_RTC_TAMPER_FILTER_CONFIG_CORRECT(sTamper->Filter, sTamper->Trigger));
  ASSERT_PARAM(IS_RTC_TAMPER_SAMPLING_FREQ(sTamper->SamplingFrequency));
  ASSERT_PARAM(IS_RTC_TAMPER_PRECHARGE_DURATION(sTamper->PrechargeDuration));
  ASSERT_PARAM(IS_RTC_TAMPER_PULLUP_STATE(sTamper->TamperPullUp));
  ASSERT_PARAM(IS_RTC_TAMPER_TIMESTAMPONTAMPER_DETECTION(sTamper->TimeStampOnTamperDetection));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Copy control register into temporary variable */
  tmpreg = hrtc->Instance->TACFG;

  /* Enable selected tamper */
  tmpreg |= (sTamper->Tamper);

  /* Configure the tamper trigger bit (this bit is just on the right of the
       tamper enable bit, hence the one-time right shift before updating it) */
  if (sTamper->Trigger == RTC_TAMPERTRIGGER_FALLINGEDGE)
  {
    /* Set the tamper trigger bit (case of falling edge or high level) */
    tmpreg |= (uint32_t)(sTamper->Tamper << 1U);
  }
  else
  {
    /* Clear the tamper trigger bit (case of rising edge or low level) */
    tmpreg &= (uint32_t)~(sTamper->Tamper << 1U);
  }

  /* Clear remaining fields before setting them */
  tmpreg &= ~(RTC_TAMPERFILTER_MASK              | \
              RTC_TAMPERSAMPLINGFREQ_RTCCLK_MASK | \
              RTC_TAMPERPRECHARGEDURATION_MASK   | \
              RTC_TAMPER_PULLUP_MASK             | \
              RTC_TACFG_TP1MSEL               | \
              RTC_TIMESTAMPONTAMPERDETECTION_MASK);

  /* Set remaining parameters of desired configuration into temporary variable */
  tmpreg |= ((uint32_t)sTamper->Filter            | \
             (uint32_t)sTamper->SamplingFrequency | \
             (uint32_t)sTamper->PrechargeDuration | \
             (uint32_t)sTamper->TamperPullUp      | \
             (uint32_t)sTamper->PinSelection      | \
             (uint32_t)sTamper->TimeStampOnTamperDetection);

  /* Enable global tamper interrupt */
  tmpreg |= (uint32_t)RTC_TACFG_TPIEN;

  /* Copy desired configuration into configuration register */
  hrtc->Instance->TACFG = tmpreg;

  /* RTC Tamper Interrupt Configuration: EINT configuration */
  __DAL_RTC_TAMPER_TIMESTAMP_EINT_ENABLE_IT();
  __DAL_RTC_TAMPER_TIMESTAMP_EINT_ENABLE_RISING_EDGE();

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Deactivates Tamper.
  * @note   The tamper global interrupt bit will remain unchanged.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Tamper Selected tamper pin.
  *          This parameter can be any combination of the following values:
  *            @arg RTC_TAMPER_1:  Tamper 1
  *            @arg RTC_TAMPER_2:  Tamper 2
  * @note   RTC_TAMPER_2 is not applicable to all devices.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper)
{
  ASSERT_PARAM(IS_RTC_TAMPER(Tamper));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the selected Tamper pin */
  hrtc->Instance->TACFG &= (uint32_t)~Tamper;

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Handles Timestamp and Tamper interrupt request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void DAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc)
{
  /* Get the Timestamp interrupt source enable status */
  if (__DAL_RTC_TIMESTAMP_GET_IT_SOURCE(hrtc, RTC_IT_TS) != 0U)
  {
    /* Get the pending status of the Timestamp Interrupt */
    if (__DAL_RTC_TIMESTAMP_GET_FLAG(hrtc, RTC_FLAG_TSF) != 0U)
    {
      /* Timestamp callback */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
      hrtc->TimeStampEventCallback(hrtc);
#else
      DAL_RTCEx_TimeStampEventCallback(hrtc);
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

      /* Clear the Timestamp interrupt pending bit */
      __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSF);
    }
  }

  /* Get the Tamper 1 interrupt source enable status */
  if (__DAL_RTC_TAMPER_GET_IT_SOURCE(hrtc, RTC_IT_TAMP) != 0U)
  {
    /* Get the pending status of the Tamper 1 Interrupt */
    if (__DAL_RTC_TAMPER_GET_FLAG(hrtc, RTC_FLAG_TAMP1F) != 0U)
    {
      /* Tamper callback */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
      hrtc->Tamper1EventCallback(hrtc);
#else
      DAL_RTCEx_Tamper1EventCallback(hrtc);
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

      /* Clear the Tamper interrupt pending bit */
      __DAL_RTC_TAMPER_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP1F);
    }
  }

#if defined(RTC_TAMPER2_SUPPORT)
  /* Get the Tamper 2 interrupt source enable status */
  if (__DAL_RTC_TAMPER_GET_IT_SOURCE(hrtc, RTC_IT_TAMP) != 0U)
  {
    /* Get the pending status of the Tamper 2 Interrupt */
    if (__DAL_RTC_TAMPER_GET_FLAG(hrtc, RTC_FLAG_TAMP2F) != 0U)
    {
      /* Tamper callback */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
      hrtc->Tamper2EventCallback(hrtc);
#else
      DAL_RTCEx_Tamper2EventCallback(hrtc);
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

      /* Clear the Tamper interrupt pending bit */
      __DAL_RTC_TAMPER_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP2F);
    }
  }
#endif /* RTC_TAMPER2_SUPPORT */

  /* Clear the EINT's Flag for RTC Timestamp and Tamper */
  __DAL_RTC_TAMPER_TIMESTAMP_EINT_CLEAR_FLAG();

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;
}

/**
  * @brief  Timestamp callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void DAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_RTCEx_TimeStampEventCallback could be implemented in the user file
  */
}

/**
  * @brief  Tamper 1 callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void DAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_RTCEx_Tamper1EventCallback could be implemented in the user file
   */
}

#if defined(RTC_TAMPER2_SUPPORT)
/**
  * @brief  Tamper 2 callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void DAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_RTCEx_Tamper2EventCallback could be implemented in the user file
   */
}
#endif /* RTC_TAMPER2_SUPPORT */

/**
  * @brief  Handles Timestamp polling request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = DAL_GetTick();

  while (__DAL_RTC_TIMESTAMP_GET_FLAG(hrtc, RTC_FLAG_TSF) == 0U)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
      {
        hrtc->State = DAL_RTC_STATE_TIMEOUT;
        return DAL_TIMEOUT;
      }
    }

    if (__DAL_RTC_TIMESTAMP_GET_FLAG(hrtc, RTC_FLAG_TSOVF) != 0U)
    {
      /* Clear the Timestamp Overrun Flag */
      __DAL_RTC_TIMESTAMP_CLEAR_FLAG(hrtc, RTC_FLAG_TSOVF);

      /* Change Timestamp state */
      hrtc->State = DAL_RTC_STATE_ERROR;

      return DAL_ERROR;
    }
  }

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Handles Tamper 1 Polling.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Get the status of the Interrupt */
  while (__DAL_RTC_TAMPER_GET_FLAG(hrtc, RTC_FLAG_TAMP1F) == 0U)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
      {
        hrtc->State = DAL_RTC_STATE_TIMEOUT;
        return DAL_TIMEOUT;
      }
    }
  }

  /* Clear the Tamper Flag */
  __DAL_RTC_TAMPER_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP1F);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  return DAL_OK;
}

#if defined(RTC_TAMPER2_SUPPORT)
/**
  * @brief  Handles Tamper 2 Polling.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Get the status of the Interrupt */
  while (__DAL_RTC_TAMPER_GET_FLAG(hrtc, RTC_FLAG_TAMP2F) == 0U)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
      {
        hrtc->State = DAL_RTC_STATE_TIMEOUT;
        return DAL_TIMEOUT;
      }
    }
  }

  /* Clear the Tamper Flag */
  __DAL_RTC_TAMPER_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP2F);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  return DAL_OK;
}
#endif /* RTC_TAMPER2_SUPPORT */

/**
  * @}
  */

/** @defgroup RTCEx_Exported_Functions_Group2 RTC Wakeup functions
  * @brief    RTC Wakeup functions
  *
@verbatim
 ===============================================================================
                        ##### RTC Wakeup functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Wakeup feature

@endverbatim
  * @{
  */

/**
  * @brief  Sets wakeup timer.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  WakeUpCounter Wakeup counter
  * @param  WakeUpClock Wakeup clock
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock)
{
  uint32_t tickstart = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_WAKEUP_CLOCK(WakeUpClock));
  ASSERT_PARAM(IS_RTC_WAKEUP_COUNTER(WakeUpCounter));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Check RTC WUTWF flag is reset only when wakeup timer enabled*/
  if ((hrtc->Instance->CTRL & RTC_CTRL_WUTEN) != 0U)
  {
    tickstart = DAL_GetTick();

    /* Wait till RTC WUTWF flag is reset and if timeout is reached exit */
    while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) != 0U)
    {
      if ((DAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
      {
        /* Enable the write protection for RTC registers */
        __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        hrtc->State = DAL_RTC_STATE_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hrtc);

        return DAL_TIMEOUT;
      }
    }
  }

  /* Disable the Wakeup timer */
  __DAL_RTC_WAKEUPTIMER_DISABLE(hrtc);

  /* Clear the Wakeup flag */
  __DAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait till RTC WUTWF flag is set and if timeout is reached exit */
  while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == 0U)
  {
    if ((DAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
    {
      /* Enable the write protection for RTC registers */
      __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

      hrtc->State = DAL_RTC_STATE_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hrtc);

      return DAL_TIMEOUT;
    }
  }

  /* Clear the Wakeup Timer clock source bits in CTRL register */
  hrtc->Instance->CTRL &= (uint32_t)~RTC_CTRL_WUCLKSEL;

  /* Configure the clock source */
  hrtc->Instance->CTRL |= (uint32_t)WakeUpClock;

  /* Configure the Wakeup Timer counter */
  hrtc->Instance->AUTORLD = (uint32_t)WakeUpCounter;

  /* Enable the Wakeup Timer */
  __DAL_RTC_WAKEUPTIMER_ENABLE(hrtc);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Sets wakeup timer with interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  WakeUpCounter Wakeup counter
  * @param  WakeUpClock Wakeup clock
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock)
{
  __IO uint32_t count  = RTC_TIMEOUT_VALUE * (SystemCoreClock / 32U / 1000U);

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_WAKEUP_CLOCK(WakeUpClock));
  ASSERT_PARAM(IS_RTC_WAKEUP_COUNTER(WakeUpCounter));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Check RTC WUTWF flag is reset only when wakeup timer enabled */
  if ((hrtc->Instance->CTRL & RTC_CTRL_WUTEN) != 0U)
  {
    /* Wait till RTC WUTWF flag is reset and if timeout is reached exit */
    do
    {
      if (count-- == 0U)
      {
        /* Enable the write protection for RTC registers */
        __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        hrtc->State = DAL_RTC_STATE_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hrtc);

        return DAL_TIMEOUT;
      }
    } while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) != 0U);
  }

  /* Disable the Wakeup timer */
  __DAL_RTC_WAKEUPTIMER_DISABLE(hrtc);

  /* Clear the Wakeup flag */
  __DAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);

  /* Reload the counter */
  count = RTC_TIMEOUT_VALUE * (SystemCoreClock / 32U / 1000U);

  /* Wait till RTC WUTWF flag is set and if timeout is reached exit */
  do
  {
    if (count-- == 0U)
    {
      /* Enable the write protection for RTC registers */
      __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

      hrtc->State = DAL_RTC_STATE_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hrtc);

      return DAL_TIMEOUT;
    }
  } while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == 0U);

  /* Clear the Wakeup Timer clock source bits in CTRL register */
  hrtc->Instance->CTRL &= (uint32_t)~RTC_CTRL_WUCLKSEL;

  /* Configure the clock source */
  hrtc->Instance->CTRL |= (uint32_t)WakeUpClock;

  /* Configure the Wakeup Timer counter */
  hrtc->Instance->AUTORLD = (uint32_t)WakeUpCounter;

  /* RTC wakeup timer Interrupt Configuration: EINT configuration */
  __DAL_RTC_WAKEUPTIMER_EINT_ENABLE_IT();
  __DAL_RTC_WAKEUPTIMER_EINT_ENABLE_RISING_EDGE();

  /* Configure the interrupt in the RTC_CTRL register */
  __DAL_RTC_WAKEUPTIMER_ENABLE_IT(hrtc, RTC_IT_WUT);

  /* Enable the Wakeup Timer */
  __DAL_RTC_WAKEUPTIMER_ENABLE(hrtc);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Deactivates wakeup timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc)
{
  uint32_t tickstart = 0U;

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Disable the Wakeup Timer */
  __DAL_RTC_WAKEUPTIMER_DISABLE(hrtc);

  /* In case of interrupt mode is used, the interrupt source must disabled */
  __DAL_RTC_WAKEUPTIMER_DISABLE_IT(hrtc, RTC_IT_WUT);

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait till RTC WUTWF flag is set and if timeout is reached exit */
  while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == 0U)
  {
    if ((DAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
    {
      /* Enable the write protection for RTC registers */
      __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

      hrtc->State = DAL_RTC_STATE_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hrtc);

      return DAL_TIMEOUT;
    }
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Gets wakeup timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Counter value
  */
uint32_t DAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc)
{
  /* Get the counter value */
  return ((uint32_t)(hrtc->Instance->AUTORLD & RTC_AUTORLD_WUAUTORE));
}

/**
  * @brief  Handles Wakeup Timer interrupt request.
  * @note   Unlike alarm interrupt line (shared by Alarms A and B) or tamper
  *         interrupt line (shared by timestamp and tampers) wakeup timer
  *         interrupt line is exclusive to the wakeup timer.
  *         There is no need in this case to check on the interrupt enable
  *         status via __DAL_RTC_WAKEUPTIMER_GET_IT_SOURCE().
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void DAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc)
{
  /* Get the pending status of the Wakeup timer Interrupt */
  if (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF) != 0U)
  {
    /* Wakeup timer callback */
#if (USE_DAL_RTC_REGISTER_CALLBACKS == 1)
    hrtc->WakeUpTimerEventCallback(hrtc);
#else
    DAL_RTCEx_WakeUpTimerEventCallback(hrtc);
#endif /* USE_DAL_RTC_REGISTER_CALLBACKS */

    /* Clear the Wakeup timer interrupt pending bit */
    __DAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);
  }

  /* Clear the EINT's line Flag for RTC WakeUpTimer */
  __DAL_RTC_WAKEUPTIMER_EINT_CLEAR_FLAG();

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;
}

/**
  * @brief  Wakeup Timer callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void DAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_RTCEx_WakeUpTimerEventCallback could be implemented in the user file
   */
}

/**
  * @brief  Handles Wakeup Timer Polling.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = DAL_GetTick();

  while (__DAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF) == 0U)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
      {
        hrtc->State = DAL_RTC_STATE_TIMEOUT;
        return DAL_TIMEOUT;
      }
    }
  }

  /* Clear the Wakeup timer Flag */
  __DAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup RTCEx_Exported_Functions_Group3 Extended Peripheral Control functions
  * @brief    Extended Peripheral Control functions
  *
@verbatim
 ===============================================================================
              ##### Extended Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Write a data in a specified RTC Backup data register
      (+) Read a data in a specified RTC Backup data register
      (+) Set the Coarse calibration parameters.
      (+) Deactivate the Coarse calibration parameters
      (+) Set the Smooth calibration parameters.
      (+) Configure the Synchronization Shift Control Settings.
      (+) Configure the Calibration Pinout (RTC_CALIB) Selection (1Hz or 512Hz).
      (+) Deactivate the Calibration Pinout (RTC_CALIB) Selection (1Hz or 512Hz).
      (+) Enable the RTC reference clock detection.
      (+) Disable the RTC reference clock detection.
      (+) Enable the Bypass Shadow feature.
      (+) Disable the Bypass Shadow feature.

@endverbatim
  * @{
  */

/**
  * @brief  Writes a data in a specified RTC Backup data register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  BackupRegister RTC Backup data Register number.
  *          This parameter can be: RTC_BKP_DRx (where x can be from 0 to 19)
  *                                 to specify the register.
  * @param  Data Data to be written in the specified RTC Backup data register.
  * @retval None
  */
void DAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data)
{
  uint32_t tmp = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_BKP(BackupRegister));

  tmp = (uint32_t) & (hrtc->Instance->BAKP0);
  tmp += (BackupRegister * 4U);

  /* Write the specified register */
  *(__IO uint32_t *)tmp = (uint32_t)Data;
}

/**
  * @brief  Reads data from the specified RTC Backup data Register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  BackupRegister RTC Backup data Register number.
  *          This parameter can be: RTC_BKP_DRx (where x can be from 0 to 19)
  *                                 to specify the register.
  * @retval Read value
  */
uint32_t DAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister)
{
  uint32_t tmp = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_BKP(BackupRegister));

  tmp = (uint32_t) & (hrtc->Instance->BAKP0);
  tmp += (BackupRegister * 4U);

  /* Read the specified register */
  return (*(__IO uint32_t *)tmp);
}

/**
  * @brief  Sets the Coarse calibration parameters.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  CalibSign Specifies the sign of the coarse calibration value.
  *          This parameter can be  one of the following values:
  *             @arg RTC_CALIBSIGN_POSITIVE: The value sign is positive
  *             @arg RTC_CALIBSIGN_NEGATIVE: The value sign is negative
  * @param  Value value of coarse calibration expressed in ppm (coded on 5 bits).
  *
  * @note   This Calibration value should be between 0 and 63 when using negative
  *         sign with a 2-ppm step.
  *
  * @note   This Calibration value should be between 0 and 126 when using positive
  *         sign with a 4-ppm step.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetCoarseCalib(RTC_HandleTypeDef *hrtc, uint32_t CalibSign, uint32_t Value)
{
  DAL_StatusTypeDef status;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_CALIB_SIGN(CalibSign));
  ASSERT_PARAM(IS_RTC_CALIB_VALUE(Value));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Enter Initialization mode */
  status = RTC_EnterInitMode(hrtc);

  if (status == DAL_OK)
  {
    /* Enable the Coarse Calibration */
    __DAL_RTC_COARSE_CALIB_ENABLE(hrtc);

    /* Set the coarse calibration value */
    hrtc->Instance->DCAL = (uint32_t)(CalibSign | Value);

    /* Exit Initialization mode */
    status = RTC_ExitInitMode(hrtc);
  }

  if (status == DAL_OK)
  {
    hrtc->State = DAL_RTC_STATE_READY;
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return status;
}

/**
  * @brief  Deactivates the Coarse calibration parameters.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateCoarseCalib(RTC_HandleTypeDef *hrtc)
{
  DAL_StatusTypeDef status;

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Enter Initialization mode */
  status = RTC_EnterInitMode(hrtc);

  if (status == DAL_OK)
  {
    /* Disable the Coarse Calibration */
    __DAL_RTC_COARSE_CALIB_DISABLE(hrtc);

    /* Exit Initialization mode */
    status = RTC_ExitInitMode(hrtc);
  }

  if (status == DAL_OK)
  {
    hrtc->State = DAL_RTC_STATE_READY;
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return status;
}

/**
  * @brief  Sets the Smooth calibration parameters.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  SmoothCalibPeriod Select the Smooth Calibration Period.
  *          This parameter can be can be one of the following values:
  *             @arg RTC_SMOOTHCALIB_PERIOD_32SEC: The smooth calibration period is 32s.
  *             @arg RTC_SMOOTHCALIB_PERIOD_16SEC: The smooth calibration period is 16s.
  *             @arg RTC_SMOOTHCALIB_PERIOD_8SEC: The smooth calibration period is 8s.
  * @param  SmoothCalibPlusPulses Select to Set or reset the CALP bit.
  *          This parameter can be one of the following values:
  *             @arg RTC_SMOOTHCALIB_PLUSPULSES_SET: Add one RTCCLK pulse every 2*11 pulses.
  *             @arg RTC_SMOOTHCALIB_PLUSPULSES_RESET: No RTCCLK pulses are added.
  * @param  SmoothCalibMinusPulsesValue Select the value of CALM[8:0] bits.
  *          This parameter can be one any value from 0 to 0x000001FF.
  * @note   To deactivate the smooth calibration, the field SmoothCalibPlusPulses
  *         must be equal to SMOOTHCALIB_PLUSPULSES_RESET and the field
  *         SmoothCalibMinusPulsesValue must be equal to 0.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmoothCalibMinusPulsesValue)
{
  uint32_t tickstart = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_SMOOTH_CALIB_PERIOD(SmoothCalibPeriod));
  ASSERT_PARAM(IS_RTC_SMOOTH_CALIB_PLUS(SmoothCalibPlusPulses));
  ASSERT_PARAM(IS_RTC_SMOOTH_CALIB_MINUS(SmoothCalibMinusPulsesValue));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* check if a calibration is pending*/
  if ((hrtc->Instance->STS & RTC_STS_RCALPFLG) != 0U)
  {
    /* Get tick */
    tickstart = DAL_GetTick();

    /* check if a calibration is pending*/
    while ((hrtc->Instance->STS & RTC_STS_RCALPFLG) != 0U)
    {
      if ((DAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
      {
        /* Enable the write protection for RTC registers */
        __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        /* Change RTC state */
        hrtc->State = DAL_RTC_STATE_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hrtc);

        return DAL_TIMEOUT;
      }
    }
  }

  /* Configure the Smooth calibration settings */
  hrtc->Instance->CAL = (uint32_t)((uint32_t)SmoothCalibPeriod     | \
                                    (uint32_t)SmoothCalibPlusPulses | \
                                    (uint32_t)SmoothCalibMinusPulsesValue);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Configures the Synchronization Shift Control Settings.
  * @note   When REFCKON is set, firmware must not write to Shift control register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  ShiftAdd1S Select to add or not 1 second to the time calendar.
  *          This parameter can be one of the following values:
  *             @arg RTC_SHIFTADD1S_SET: Add one second to the clock calendar.
  *             @arg RTC_SHIFTADD1S_RESET: No effect.
  * @param  ShiftSubFS Select the number of Second Fractions to substitute.
  *          This parameter can be one any value from 0 to 0x7FFF.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS)
{
  uint32_t tickstart = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_SHIFT_ADD1SECEN(ShiftAdd1S));
  ASSERT_PARAM(IS_RTC_SHIFT_SFSEC(ShiftSubFS));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait until the shift is completed */
  while ((hrtc->Instance->STS & RTC_STS_SOPFLG) != 0U)
  {
    if ((DAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
    {
      /* Enable the write protection for RTC registers */
      __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

      hrtc->State = DAL_RTC_STATE_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hrtc);

      return DAL_TIMEOUT;
    }
  }

  /* Check if the reference clock detection is disabled */
  if ((hrtc->Instance->CTRL & RTC_CTRL_RCLKDEN) == 0U)
  {
    /* Configure the Shift settings */
    hrtc->Instance->SHIFT = (uint32_t)(uint32_t)(ShiftSubFS) | (uint32_t)(ShiftAdd1S);

    /* If  RTC_CTRL_RCMCFG bit = 0, wait for synchro else this check is not needed */
    if ((hrtc->Instance->CTRL & RTC_CTRL_RCMCFG) == 0U)
    {
      if (DAL_RTC_WaitForSynchro(hrtc) != DAL_OK)
      {
        /* Enable the write protection for RTC registers */
        __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        hrtc->State = DAL_RTC_STATE_ERROR;

        /* Process Unlocked */
        __DAL_UNLOCK(hrtc);

        return DAL_ERROR;
      }
    }
  }
  else
  {
    /* Enable the write protection for RTC registers */
    __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    /* Change RTC state */
    hrtc->State = DAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    __DAL_UNLOCK(hrtc);

    return DAL_ERROR;
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Configures the Calibration Pinout (RTC_CALIB) Selection (1Hz or 512Hz).
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  CalibOutput Select the Calibration output Selection.
  *          This parameter can be one of the following values:
  *             @arg RTC_CALIBOUTPUT_512HZ: A signal has a regular waveform at 512Hz.
  *             @arg RTC_CALIBOUTPUT_1HZ: A signal has a regular waveform at 1Hz.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_RTC_CALIB_OUTPUT(CalibOutput));

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Clear flags before config */
  hrtc->Instance->CTRL &= (uint32_t)~RTC_CTRL_CALOSEL;

  /* Configure the RTC_CTRL register */
  hrtc->Instance->CTRL |= (uint32_t)CalibOutput;

  __DAL_RTC_DCALATION_OUTPUT_ENABLE(hrtc);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Deactivates the Calibration Pinout (RTC_CALIB) Selection (1Hz or 512Hz).
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc)
{
  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  __DAL_RTC_DCALATION_OUTPUT_DISABLE(hrtc);

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Enables the RTC reference clock detection.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc)
{
  DAL_StatusTypeDef status;

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Enter Initialization mode */
  status = RTC_EnterInitMode(hrtc);

  if (status == DAL_OK)
  {
    /* Enable the reference clock detection */
    __DAL_RTC_CLOCKREF_DETECTION_ENABLE(hrtc);

    /* Exit Initialization mode */
    status = RTC_ExitInitMode(hrtc);
  }

  if (status == DAL_OK)
  {
    hrtc->State = DAL_RTC_STATE_READY;
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return status;
}

/**
  * @brief  Disable the RTC reference clock detection.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc)
{
  DAL_StatusTypeDef status;

  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Enter Initialization mode */
  status = RTC_EnterInitMode(hrtc);

  if (status == DAL_OK)
  {
    /* Disable the reference clock detection */
    __DAL_RTC_CLOCKREF_DETECTION_DISABLE(hrtc);

    /* Exit Initialization mode */
    status = RTC_ExitInitMode(hrtc);
  }

  if (status == DAL_OK)
  {
    hrtc->State = DAL_RTC_STATE_READY;
  }

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return status;
}

/**
  * @brief  Enables the Bypass Shadow feature.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   When the Bypass Shadow is enabled the calendar value are taken
  *         directly from the Calendar counter.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc)
{
  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set the BYPSHAD bit */
  hrtc->Instance->CTRL |= (uint8_t)RTC_CTRL_RCMCFG;

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @brief  Disables the Bypass Shadow feature.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   When the Bypass Shadow is enabled the calendar value are taken
  *         directly from the Calendar counter.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc)
{
  /* Process Locked */
  __DAL_LOCK(hrtc);

  hrtc->State = DAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Reset the RCMCFG bit */
  hrtc->Instance->CTRL &= (uint8_t)~RTC_CTRL_RCMCFG;

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hrtc);

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup RTCEx_Exported_Functions_Group4 Extended features functions
  * @brief    Extended features functions
  *
@verbatim
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) RTC Alarm B callback
      (+) RTC Poll for Alarm B request

@endverbatim
  * @{
  */

/**
  * @brief  Alarm B callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void DAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_RTCEx_AlarmBEventCallback could be implemented in the user file
   */
}

/**
  * @brief  Handles Alarm B Polling request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait till RTC ALRBF flag is set and if timeout is reached exit */
  while (__DAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF) == 0U)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
      {
        hrtc->State = DAL_RTC_STATE_TIMEOUT;
        return DAL_TIMEOUT;
      }
    }
  }

  /* Clear the Alarm flag */
  __DAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRBF);

  /* Change RTC state */
  hrtc->State = DAL_RTC_STATE_READY;

  return DAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_RTC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
