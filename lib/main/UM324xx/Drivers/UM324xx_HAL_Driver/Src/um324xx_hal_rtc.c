/**
  ******************************************************************************
  * @file     um324xx_hal_rtc.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-06-14
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324XX_HAL_Driver
  * @{
  */

/** @defgroup RTC RTC
  * @brief    RTC HAL module driver
  * @{
  */

#ifdef HAL_RTC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RTC_Exported_Functions RTC Exported Functions
  * @{
  */

/** @defgroup RTC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
   [..] This section provides functions allowing to initialize and configure the
         RTC Prescaler (Synchronous and Asynchronous), RTC Hour format, disable
         RTC registers Write protection, enter and exit the RTC initialization mode,
         RTC registers synchronization check and reference clock detection enable.
         (#) The RTC Prescaler is programmed to generate the RTC 1Hz time base.
             It is split into 2 programmable prescalers to minimize power consumption.
             (++) A 7-bit asynchronous prescaler and a 15-bit synchronous prescaler.
             (++) When both prescalers are used, it is recommended to configure the
                 asynchronous prescaler to a high value to minimize power consumption.
         (#) All RTC registers are Write protected. Writing to the RTC registers
             is enabled by writing a key into the Write Protection register, RTC_WPR.
         (#) To configure the RTC Calendar, user application should enter
             initialization mode. In this mode, the calendar counter is stopped
             and its value can be updated. When the initialization sequence is
             complete, the calendar restarts counting after 4 RTCCLK cycles.
         (#) To read the calendar through the shadow registers after Calendar
             initialization, calendar update or after wakeup from low power modes
             the software must first clear the RSF flag. The software must then
             wait until it is set again before reading the calendar, which means
             that the calendar registers have been correctly copied into the
             RTC_TR and RTC_DR shadow registers. The HAL_RTC_WaitForSynchro() function
             implements the above software sequence (RSF clear and RSF check).

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Check RTC handler validity */
    if (hrtc == NULL)
    {
        return HAL_ERROR;
    }

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
    if (hrtc->State == HAL_RTC_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hrtc->Lock = HAL_UNLOCKED;

        hrtc->Alarm1EventCallback          =  HAL_RTC_Alarm1EventCallback;        /* Legacy weak Alarm1EventCallback      */
        hrtc->Alarm2EventCallback          =  HAL_RTC_Alarm2EventCallback;        /* Legacy weak Alarm2EventCallback      */
        hrtc->TamperEventCallback          =  HAL_RTCEx_TamperEventCallback;      /* Legacy weak TamperEventCallback      */

        if (hrtc->MspInitCallback == NULL)
        {
            hrtc->MspInitCallback = HAL_RTC_MspInit;
        }
        /* Init the low level hardware */
        hrtc->MspInitCallback(hrtc);

        if (hrtc->MspDeInitCallback == NULL)
        {
            hrtc->MspDeInitCallback = HAL_RTC_MspDeInit;
        }
    }
#else /* USE_HAL_RTC_REGISTER_CALLBACKS */
    if (hrtc->State == HAL_RTC_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hrtc->Lock = HAL_UNLOCKED;

        /* Initialize RTC MSP */
        HAL_RTC_MspInit(hrtc);
    }
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_BUSY;

    /* Clear RTC_CR FMT, OSEL and POL Bits */
    hrtc->Instance->TIME &= (uint32_t)~(RTC_TIME_HOUR12_24);
    /* Set RTC_CR register */
    hrtc->Instance->TIME |= (uint32_t)(hrtc->Init.HourFormat);

    hrtc->State = HAL_RTC_STATE_READY;

    status = HAL_OK;

    return status;
}


/**
  * @brief  DeInitializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   This function does not reset the RTC Backup Data registers.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_BUSY;

    if (status == HAL_OK)
    {

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
        if (hrtc->MspDeInitCallback == NULL)
        {
            hrtc->MspDeInitCallback = HAL_RTC_MspDeInit;
        }

        /* DeInit the low level hardware: CLOCK, NVIC.*/
        hrtc->MspDeInitCallback(hrtc);
#else /* USE_HAL_RTC_REGISTER_CALLBACKS */
        /* De-Initialize RTC MSP */
        HAL_RTC_MspDeInit(hrtc);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

        hrtc->State = HAL_RTC_STATE_RESET;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrtc);

    return status;
}

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Registers a User RTC Callback
  *         To be used instead of the weak predefined callback
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RTC_ALARM_1_EVENT_CB_ID          Alarm 1 Event Callback ID
  *          @arg @ref HAL_RTC_ALARM_2_EVENT_CB_ID          Alarm 2 Event Callback ID
  *          @arg @ref HAL_RTC_TAMPER_EVENT_CB_ID           Tamper Callback ID
  *          @arg @ref HAL_RTC_MSPINIT_CB_ID                Msp Init callback ID
  *          @arg @ref HAL_RTC_MSPDEINIT_CB_ID              Msp DeInit callback ID
  * @note   HAL_RTC_TAMPER2_EVENT_CB_ID is not applicable to all devices.
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_RegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID, pRTC_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hrtc);

    if (HAL_RTC_STATE_READY == hrtc->State)
    {
        switch (CallbackID)
        {
        case HAL_RTC_ALARM_1_EVENT_CB_ID :
            hrtc->Alarm1EventCallback = pCallback;
            break;

        case HAL_RTC_ALARM_2_EVENT_CB_ID :
            hrtc->Alarm2EventCallback = pCallback;
            break;

        case HAL_RTC_TAMPER_EVENT_CB_ID :
            hrtc->TamperEventCallback = pCallback;
            break;

        case HAL_RTC_MSPINIT_CB_ID :
            hrtc->MspInitCallback = pCallback;
            break;

        case HAL_RTC_MSPDEINIT_CB_ID :
            hrtc->MspDeInitCallback = pCallback;
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_RTC_STATE_RESET == hrtc->State)
    {
        switch (CallbackID)
        {
        case HAL_RTC_MSPINIT_CB_ID :
            hrtc->MspInitCallback = pCallback;
            break;

        case HAL_RTC_MSPDEINIT_CB_ID :
            hrtc->MspDeInitCallback = pCallback;
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrtc);

    return status;
}

/**
  * @brief  Unregisters an RTC Callback
  *         RTC callabck is redirected to the weak predefined callback
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RTC_ALARM_1_EVENT_CB_ID          Alarm 1 Event Callback ID
  *          @arg @ref HAL_RTC_ALARM_2_EVENT_CB_ID          Alarm 2 Event Callback ID
  *          @arg @ref HAL_RTC_TAMPER_EVENT_CB_ID           Tamper Callback ID
  *          @arg @ref HAL_RTC_MSPINIT_CB_ID                Msp Init callback ID
  *          @arg @ref HAL_RTC_MSPDEINIT_CB_ID 				Msp DeInit callback ID
  * @note   HAL_RTC_TAMPER2_EVENT_CB_ID is not applicable to all devices.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_UnRegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hrtc);

    if (HAL_RTC_STATE_READY == hrtc->State)
    {
        switch (CallbackID)
        {
        case HAL_RTC_ALARM_1_EVENT_CB_ID :
            hrtc->Alarm1EventCallback = HAL_RTC_Alarm1EventCallback;            /* Legacy weak Alarm1EventCallback    */
            break;

        case HAL_RTC_ALARM_2_EVENT_CB_ID :
            hrtc->Alarm2EventCallback = HAL_RTC_Alarm2EventCallback;            /* Legacy weak Alarm2EventCallback    */
            break;

        case HAL_RTC_TAMPER_EVENT_CB_ID :
            hrtc->TamperEventCallback = HAL_RTCEx_TamperEventCallback;            /* Legacy weak TamperEventCallback    */
            break;

        case HAL_RTC_MSPINIT_CB_ID :
            hrtc->MspInitCallback = HAL_RTC_MspInit;
            break;

        case HAL_RTC_MSPDEINIT_CB_ID :
            hrtc->MspDeInitCallback = HAL_RTC_MspDeInit;
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_RTC_STATE_RESET == hrtc->State)
    {
        switch (CallbackID)
        {
        case HAL_RTC_MSPINIT_CB_ID :
            hrtc->MspInitCallback = HAL_RTC_MspInit;
            break;

        case HAL_RTC_MSPDEINIT_CB_ID :
            hrtc->MspDeInitCallback = HAL_RTC_MspDeInit;
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrtc);

    return status;
}
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrtc);

    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_RTC_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrtc);

    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_RTC_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Group2 RTC Time and Date functions
  * @brief    RTC Time and Date functions
  *
@verbatim
 ===============================================================================
                 ##### RTC Time and Date functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Time and Date features

@endverbatim
  * @{
  */

/**
  * @brief  Sets RTC current time.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime Pointer to Time structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
    uint32_t tmpreg = 0U;
	
    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

	/* Enable the write protection for RTC TIME registers */
	__HAL_RTC_WRITECOUNTER_START(hrtc);
	
    if (Format == RTC_FORMAT_BIN)
    {
        tmpreg = (uint32_t)(( (uint32_t)RTC_ByteToBcd2(sTime->Hours)   << RTC_TIME_HOUR19_Pos) | \
                            ( (uint32_t)RTC_ByteToBcd2(sTime->Minutes) << RTC_TIME_MINUTE_Pos) | \
                            ( (uint32_t)RTC_ByteToBcd2(sTime->Seconds) << RTC_TIME_SECOND_Pos) | \
		                    ( (uint32_t)RTC_ByteToBcd2(sTime->CentiSeconds)                  ));
    }
    else
    {
        tmpreg = (((uint32_t)(sTime->Hours)      << RTC_TIME_HOUR19_Pos)  | \
                  ((uint32_t)(sTime->Minutes)    << RTC_TIME_MINUTE_Pos)  | \
                  ((uint32_t) sTime->Seconds)    << RTC_TIME_SECOND_Pos   | \
                  ((uint32_t)(sTime->CentiSeconds) 							));
    }

	if(hrtc->Init.HourFormat == RTC_HOURFORMAT_12)
	{
		tmpreg |= (sTime->TimeFormat << RTC_TIME_H20_PA_Pos);
	}
	
	/* Set the RTC_TIME register */
	hrtc->Instance->TIME = (uint32_t)((tmpreg) & RTC_TIME_RESERVED_MASK) | hrtc->Init.HourFormat;
	
	hrtc->State = HAL_RTC_STATE_READY;
	
    /* Disable the write protection for RTC TIME registers */
    __HAL_RTC_WRITECOUNTER_STOP(hrtc);

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;
}

/**
  * @brief  Gets RTC current time.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime Pointer to Time structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
    uint32_t tmpreg = 0U;

	__HAL_RTC_READCOUNTER_STOP(hrtc);
	__HAL_RTC_READCOUNTER_START(hrtc);
	
    /* Get the TIME register */
    tmpreg = (uint32_t)(hrtc->Instance->TIME & RTC_TIME_RESERVED_MASK);

    /* Fill the structure fields with the read parameters */
	if((hrtc->Instance->TIME & RTC_TIME_HOUR12_24) ==  RTC_TIME_HOUR12_24)
	{
		sTime->Hours = (uint8_t)((tmpreg & (RTC_TIME_H20_PA  | RTC_TIME_HOUR19))  >> RTC_TIME_HOUR19_Pos);
	}
    else
	{
		sTime->Hours = (uint8_t)((tmpreg & RTC_TIME_HOUR19)  >> RTC_TIME_HOUR19_Pos);
		sTime->TimeFormat = (uint8_t)((tmpreg & (RTC_TIME_H20_PA )) >> RTC_TIME_H20_PA_Pos);
	}
    sTime->Minutes    = (uint8_t)((tmpreg & (RTC_TIME_MINUTE )) >> RTC_TIME_MINUTE_Pos);
    sTime->Seconds    = (uint8_t)((tmpreg & (RTC_TIME_SECOND )) >> RTC_TIME_SECOND_Pos);
	sTime->CentiSeconds = (uint8_t)((tmpreg & (RTC_TIME_CENTISEC )) >> RTC_TIME_CENTISEC_Pos);
    

    /* Check the input parameters format */
    if (Format == RTC_FORMAT_BIN)
    {
        /* Convert the time structure parameters to Binary format */
        sTime->Hours = (uint8_t)RTC_Bcd2ToByte(sTime->Hours);
        sTime->Minutes = (uint8_t)RTC_Bcd2ToByte(sTime->Minutes);
        sTime->Seconds = (uint8_t)RTC_Bcd2ToByte(sTime->Seconds);
		sTime->CentiSeconds = (uint8_t)RTC_Bcd2ToByte(sTime->CentiSeconds);
    }

    return HAL_OK;
}

/**
  * @brief  Sets RTC current date.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate Pointer to date structure
  * @param  Format specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
    uint32_t datetmpreg = 0U;
    HAL_StatusTypeDef status;

    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

	__HAL_RTC_WRITECOUNTER_START(hrtc);
	
    if (Format == RTC_FORMAT_BIN)
    {
        datetmpreg = (((uint32_t)RTC_ByteToBcd2(sDate->Year)  << RTC_DATE_YEAR_Pos)  | \
                      ((uint32_t)RTC_ByteToBcd2(sDate->Month) << RTC_DATE_MONTH_Pos) | \
                      ((uint32_t)RTC_ByteToBcd2(sDate->Date)) << RTC_DATE_DAY_Pos  | \
                      ((uint32_t)sDate->WeekDay               << RTC_DATE_WEEK_Pos));
    }
    else
    {
        datetmpreg = ((((uint32_t)sDate->Year)    << RTC_DATE_YEAR_Pos)  | \
                      (((uint32_t)sDate->Month)   << RTC_DATE_MONTH_Pos) | \
                      ((uint32_t) sDate->Date)    << RTC_DATE_DAY_Pos    | \
                      (((uint32_t)sDate->WeekDay) << RTC_DATE_WEEK_Pos));
    }


    /* Set the RTC_DATE register */
    hrtc->Instance->DATE = (uint32_t)((datetmpreg | RTC_DATE_CENTURY) & RTC_DATE_RESERVED_MASK);
		
	__HAL_RTC_WRITECOUNTER_STOP(hrtc);

    hrtc->State = HAL_RTC_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);
	
	status = HAL_OK;
	
    return status;
}

/**
  * @brief  Gets RTC current date.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate Pointer to Date structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN:  Binary data format
  *            @arg RTC_FORMAT_BCD:  BCD data format
  * @note  You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the
  *        values in the higher-order calendar shadow registers to ensure
  *        consistency between the time and date values.
  *        Reading RTC current time locks the values in calendar shadow registers
  *        until current date is read to ensure consistency between the time and
  *        date values.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
    uint32_t datetmpreg = 0U;

	__HAL_RTC_READCOUNTER_STOP(hrtc);
	__HAL_RTC_READCOUNTER_START(hrtc);
	
    /* Get the DATE register */
    datetmpreg = (uint32_t)(hrtc->Instance->DATE & RTC_DATE_RESERVED_MASK);

    /* Fill the structure fields with the read parameters */
    sDate->Year    = (uint8_t)((datetmpreg & (RTC_DATE_YEAR )) >> RTC_DATE_YEAR_Pos );
    sDate->Month   = (uint8_t)((datetmpreg & (RTC_DATE_MONTH)) >> RTC_DATE_MONTH_Pos);
    sDate->Date    = (uint8_t)((datetmpreg & (RTC_DATE_DAY  )) >> RTC_DATE_DAY_Pos  );
    sDate->WeekDay = (uint8_t) (datetmpreg & (RTC_DATE_WEEK ));

    /* Check the input parameters format */
    if (Format == RTC_FORMAT_BIN)
    {
        /* Convert the date structure parameters to Binary format */
        sDate->Year  = (uint8_t)RTC_Bcd2ToByte(sDate->Year);
        sDate->Month = (uint8_t)RTC_Bcd2ToByte(sDate->Month);
        sDate->Date  = (uint8_t)RTC_Bcd2ToByte(sDate->Date);
    }
	
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Group3 RTC Alarm functions
  * @brief    RTC Alarm functions
  *
@verbatim
 ===============================================================================
                 ##### RTC Alarm functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Alarm feature

@endverbatim
  * @{
  */
/**
  * @brief  Sets the specified RTC Alarm.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sAlarm Pointer to Alarm structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{
	uint32_t datetmpreg = 0U;
	uint32_t timetmpreg = 0U;

	/* Process Locked */
	__HAL_LOCK(hrtc);

	/* Change RTC state to BUSY */
	hrtc->State = HAL_RTC_STATE_BUSY;

	if(sAlarm->Alarm == RTC_ALARM_1)
	{
		/* Check the data format (binary or BCD) and store the Alarm time and date
		 configuration accordingly */
		if (Format == RTC_FORMAT_BIN)
		{
			datetmpreg = (((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Year)  << RTC_ALM1DATE_ALM1_YEAR_Pos)  | \
						  ((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Month) << RTC_ALM1DATE_ALM1_MONTH_Pos) | \
						  ((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Date)) << RTC_ALM1DATE_ALM1_DAY_Pos    | \
						  ((uint32_t)sAlarm->AlarmDate.WeekDay               << RTC_ALM1DATE_ALM1_WEEK_Pos));
		}
		else
		{
			datetmpreg = ((((uint32_t)sAlarm->AlarmDate.Year)    << RTC_ALM1DATE_ALM1_YEAR_Pos)      | \
						  (((uint32_t)sAlarm->AlarmDate.Month)   << RTC_ALM1DATE_ALM1_MONTH_Pos) | \
						  ((uint32_t) sAlarm->AlarmDate.Date)    << RTC_ALM1DATE_ALM1_DAY_Pos    | \
						  (((uint32_t)sAlarm->AlarmDate.WeekDay) << RTC_ALM1DATE_ALM1_WEEK_Pos));
		}
		
		hrtc->Instance->ALM1DATE = (uint32_t)datetmpreg;
		
		if (Format == RTC_FORMAT_BIN)
		{
			timetmpreg = (uint32_t)(( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Hours)   << RTC_ALM1TIME_ALM1_HOUR19_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Minutes) << RTC_ALM1TIME_ALM1_MINUTE_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Seconds) << RTC_ALM1TIME_ALM1_SECOND_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.CentiSeconds)                  ));
		}
		else
		{
			timetmpreg = (((uint32_t)(sAlarm->AlarmTime.Hours)      << RTC_ALM1TIME_ALM1_HOUR19_Pos)  | \
					  ((uint32_t)(sAlarm->AlarmTime.Minutes)    << RTC_ALM1TIME_ALM1_MINUTE_Pos)  | \
					  ((uint32_t) sAlarm->AlarmTime.Seconds)    << RTC_ALM1TIME_ALM1_SECOND_Pos   | \
					  ((uint32_t)(sAlarm->AlarmTime.CentiSeconds) 						));
		}

		if(hrtc->Init.HourFormat == RTC_HOURFORMAT_12)
		{
			timetmpreg |= (sAlarm->AlarmTime.TimeFormat << RTC_ALM1TIME_ALM1_H20_PA_Pos);
		}
		
		/* Set the RTC_ALM1 register */
		hrtc->Instance->ALM1TIME = (uint32_t)(timetmpreg | hrtc->Init.HourFormat);

		/* Set the RTC_ALM1EN register */
		hrtc->Instance->ALM1EN = sAlarm->AlarmMask;
		
		/* Enable RTC_ALM1 */
		__HAL_RTC_ALARM1_ENABLE(hrtc);
	}
	else
	{
		/* Enable RTC_ALM2 interrupt */
		hrtc->Instance->ALM2SETTING = sAlarm->AlarmMask;
		
		__HAL_RTC_ALARM2_ENABLE(hrtc);
	}

	/* Change RTC state back to READY */
	hrtc->State = HAL_RTC_STATE_READY;
	
	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);

	return HAL_OK;
}

/**
  * @brief  Sets the specified RTC Alarm with Interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sAlarm Pointer to Alarm structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{
	uint32_t datetmpreg = 0U;
	uint32_t timetmpreg = 0U;

	/* Process Locked */
	__HAL_LOCK(hrtc);

	/* Change RTC state to BUSY */
	hrtc->State = HAL_RTC_STATE_BUSY;

	if(sAlarm->Alarm == RTC_ALARM_1)
	{
		/* Check the data format (binary or BCD) and store the Alarm time and date
		 configuration accordingly */
		if (Format == RTC_FORMAT_BIN)
		{
			datetmpreg = (((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Year)  << RTC_ALM1DATE_ALM1_YEAR_Pos)  | \
						  ((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Month) << RTC_ALM1DATE_ALM1_MONTH_Pos) | \
						  ((uint32_t)RTC_ByteToBcd2(sAlarm->AlarmDate.Date)) << RTC_ALM1DATE_ALM1_DAY_Pos    | \
						  ((uint32_t)sAlarm->AlarmDate.WeekDay               << RTC_ALM1DATE_ALM1_WEEK_Pos));
		}
		else
		{
			datetmpreg = ((((uint32_t)sAlarm->AlarmDate.Year)    << RTC_ALM1DATE_ALM1_YEAR_Pos)      | \
						  (((uint32_t)sAlarm->AlarmDate.Month)   << RTC_ALM1DATE_ALM1_MONTH_Pos) | \
						  ((uint32_t) sAlarm->AlarmDate.Date)    << RTC_ALM1DATE_ALM1_DAY_Pos    | \
						  (((uint32_t)sAlarm->AlarmDate.WeekDay) << RTC_ALM1DATE_ALM1_WEEK_Pos));
		}
		
		hrtc->Instance->ALM1DATE = (uint32_t)datetmpreg;
		
		if (Format == RTC_FORMAT_BIN)
		{
			timetmpreg = (uint32_t)(( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Hours)   << RTC_ALM1TIME_ALM1_HOUR19_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Minutes) << RTC_ALM1TIME_ALM1_MINUTE_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.Seconds) << RTC_ALM1TIME_ALM1_SECOND_Pos) | \
								( (uint32_t)RTC_ByteToBcd2(sAlarm->AlarmTime.CentiSeconds)                  ));
		}
		else
		{
			timetmpreg = (((uint32_t)(sAlarm->AlarmTime.Hours)      << RTC_ALM1TIME_ALM1_HOUR19_Pos)  | \
					  ((uint32_t)(sAlarm->AlarmTime.Minutes)    << RTC_ALM1TIME_ALM1_MINUTE_Pos)  | \
					  ((uint32_t) sAlarm->AlarmTime.Seconds)    << RTC_ALM1TIME_ALM1_SECOND_Pos   | \
					  ((uint32_t)(sAlarm->AlarmTime.CentiSeconds) 						));
		}

		if(hrtc->Init.HourFormat == RTC_HOURFORMAT_12)
		{
			timetmpreg |= (sAlarm->AlarmTime.TimeFormat << RTC_ALM1TIME_ALM1_H20_PA_Pos);
		}
		
		/* Set the RTC_ALM1 register */
		hrtc->Instance->ALM1TIME = (uint32_t)(timetmpreg | hrtc->Init.HourFormat);
		
		/* Enable RTC_ALM1 interrupt */
		__HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALM1);

		/* Set the RTC_ALM1EN register */
		hrtc->Instance->ALM1EN = sAlarm->AlarmMask;
		
		/* Enable RTC_ALM1 */
		__HAL_RTC_ALARM1_ENABLE(hrtc);
	}
	else
	{
		/* Enable RTC_ALM2 interrupt */
		__HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALM2);
		
		/* Enable RTC_ALM2 interrupt */
		hrtc->Instance->ALM2SETTING = sAlarm->AlarmMask;
		
		__HAL_RTC_ALARM2_ENABLE(hrtc);
	}

	/* Change RTC state back to READY */
	hrtc->State = HAL_RTC_STATE_READY;
	
	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);

	return HAL_OK;
}

/**
  * @brief  Gets the RTC Alarm value and masks.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sAlarm Pointer to Date structure
  * @param  Alarm Specifies the Alarm.
  *          This parameter can be one of the following values:
  *            @arg RTC_ALARM_1: Alarm 1
  *            @arg RTC_ALARM_2: Alarm 2
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format)
{
	uint32_t tmptime = 0U;
	uint32_t tmpdate = 0U;
	
	if(Alarm == RTC_ALARM_1)
	{
		sAlarm->Alarm = RTC_ALARM_1;
		
		/* Get the Timestamp time and date registers values */
		tmptime = (uint32_t)(hrtc->Instance->ALM1TIME & RTC_TIME_RESERVED_MASK);
		tmpdate = (uint32_t)(hrtc->Instance->ALM1DATE & RTC_DATE_RESERVED_MASK);
		
		if((hrtc->Instance->TIME & RTC_TIME_HOUR12_24) ==  RTC_HOURFORMAT_24)
		{
			sAlarm->AlarmTime.Hours = (uint8_t)((tmptime & (RTC_TIME_H20_PA  | RTC_TIME_HOUR19))  >> RTC_TIME_HOUR19_Pos);
		}
		else
		{
			sAlarm->AlarmTime.Hours = (uint8_t)((tmptime & RTC_TIME_HOUR19)  >> RTC_TIME_HOUR19_Pos);
			sAlarm->AlarmTime.TimeFormat = (uint8_t)((tmptime & (RTC_TIME_H20_PA )) >> RTC_TIME_H20_PA_Pos);
		}
		sAlarm->AlarmTime.Minutes    = (uint8_t)((tmptime & (RTC_TIME_MINUTE )) >> RTC_TIME_MINUTE_Pos);
		sAlarm->AlarmTime.Seconds    = (uint8_t)((tmptime & (RTC_TIME_SECOND )) >> RTC_TIME_SECOND_Pos);
		sAlarm->AlarmTime.CentiSeconds = (uint8_t)((tmptime & (RTC_TIME_CENTISEC )) >> RTC_TIME_CENTISEC_Pos);

		/* Fill the Date structure fields with the read parameters */
		sAlarm->AlarmDate.Year    = (uint8_t)((tmpdate & (RTC_DATE_YEAR )) >> RTC_DATE_YEAR_Pos );
		sAlarm->AlarmDate.Month   = (uint8_t)((tmpdate & (RTC_DATE_MONTH)) >> RTC_DATE_MONTH_Pos);
		sAlarm->AlarmDate.Date    = (uint8_t)((tmpdate & (RTC_DATE_DAY  )) >> RTC_DATE_DAY_Pos  );
		sAlarm->AlarmDate.WeekDay = (uint8_t) (tmpdate & (RTC_DATE_WEEK ));
		
		/* Check the input parameters format */
		if (Format == RTC_FORMAT_BIN)
		{
			/* Convert the Timestamp structure parameters to Binary format */
			sAlarm->AlarmTime.Hours   = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmTime.Hours);
			sAlarm->AlarmTime.Minutes = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes);
			sAlarm->AlarmTime.Seconds = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds);

			/* Convert the DateTimeStamp structure parameters to Binary format */
			sAlarm->AlarmDate.Year    = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmDate.Year);
			sAlarm->AlarmDate.Month   = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmDate.Month);
			sAlarm->AlarmDate.Date    = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmDate.Date);
			sAlarm->AlarmDate.WeekDay = (uint8_t)RTC_Bcd2ToByte(sAlarm->AlarmDate.WeekDay);
		}
	}
	else
	{
		sAlarm->Alarm = RTC_ALARM_2;
		
		sAlarm->AlarmMask = hrtc->Instance->ALM2SETTING & 0x0f;
	}
	
	return HAL_OK;
}

/**
  * @brief  Handles Alarm interrupt request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc)
{
	/* Get the Alarm 1 interrupt source enable status */
	if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALM1) != 0U)
	{
		/* Get the pending status of the Alarm 1 Interrupt */
		if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALM1) != 0U)
		{
			/* Alarm 1 callback */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
			hrtc->Alarm1EventCallback(hrtc);
#else
			HAL_RTC_Alarm1EventCallback(hrtc);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

		  /* Clear the Alarm 1 interrupt pending bit */
		  __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALM1);
		}
	}

	/* Get the Alarm 2 interrupt source enable status */
	if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALM2) != 0U)
	{
		/* Get the pending status of the Alarm 2 Interrupt */
		if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALM2) != 0U)
		{
			/* Alarm 2 callback */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
			hrtc->Alarm2EventCallback(hrtc);
#else
			HAL_RTC_Alarm2EventCallback(hrtc);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

		  /* Clear the Alarm 2 interrupt pending bit */
		  __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALM2);
		}
	}

	/* Get the Tamper event interrupt source enable status */
	if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_TAMP) != 0U)
	{
		/* Get the pending status of the Tamper event Interrupt */
		if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_TAMP) != 0U)
		{
			/* Tamper callback */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
			hrtc->TamperEventCallback(hrtc);
#else
			HAL_RTCEx_TamperEventCallback(hrtc);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

		  /* Clear the Tamper event interrupt pending bit */
		  __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP);
		}
	}

	/* Change RTC state */
	hrtc->State = HAL_RTC_STATE_READY;
}

/**
  * @brief  Handles Alarm 1 Polling request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_PollForAlarm1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
	uint32_t tickstart = 0U;

	/* Get tick */
	tickstart = HAL_GetTick();

	/* Wait till RTC ALM1 flag is set and if timeout is reached exit */
	while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALM1) == 0U)
	{
		if (Timeout != HAL_MAX_DELAY)
		{
		  if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
		  {
			hrtc->State = HAL_RTC_STATE_TIMEOUT;
			return HAL_TIMEOUT;
		  }
		}
	}

	/* Clear the Alarm flag */
	__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALM1);

	/* Change RTC state */
	hrtc->State = HAL_RTC_STATE_READY;

	return HAL_OK;
}

/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Group5 Peripheral State functions
  * @brief    Peripheral State functions
  *
@verbatim
 ===============================================================================
                     ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Get RTC state

@endverbatim
  * @{
  */
/**
  * @brief  Returns the RTC state.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL state
  */
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc)
{
	return hrtc->State;
}

/**
  * @}
  */

/**
  * @brief  Converts a 2-digit number from decimal to BCD format.
  * @param  number decimal-formatted number (from 0 to 99) to be converted
  * @retval Converted byte
  */
uint8_t RTC_ByteToBcd2(uint8_t number)
{
    uint8_t bcdhigh = 0U;

    while (number >= 10U)
    {
        bcdhigh++;
        number -= 10U;
    }

    return ((uint8_t)(bcdhigh << 4U) | number);
}

/**
  * @brief  Converts a 2-digit number from BCD to decimal format.
  * @param  number BCD-formatted number (from 00 to 99) to be converted
  * @retval Converted word
  */
uint8_t RTC_Bcd2ToByte(uint8_t number)
{
    uint8_t tmp = 0U;
    tmp = ((uint8_t)(number & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (tmp + (number & (uint8_t)0x0F));
}

/**
  * @brief  Writes a data in a specified RTC Backup data register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  BackupRegister RTC Backup data Register number.
  *          This parameter can be: RTC_BKREGx (where x can be from 0 to 19)
  *                                 to specify the register.
  * @param  Data Data to be written in the specified RTC Backup data register.
  * @retval None
  */
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data)
{
	uint32_t tmp = 0U;

	tmp = (uint32_t) & (hrtc->Instance->BKREG);
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
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister)
{
	uint32_t tmp = 0U;

	tmp = (uint32_t) & (hrtc->Instance->BKREG);
	tmp += (BackupRegister * 4U);

	/* Read the specified register */
	return (*(__IO uint32_t *)tmp);
}

/** @defgroup RTCEx_Exported_Functions RTCEx Exported Functions
  * @{
  */

/** @defgroup RTCEx_Exported_Functions_Group4 RTC Timestamp and Tamper functions
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
  * @brief  Gets the RTC Timestamp value.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTimeStamp Pointer to Time structure
  * @param  sTimeStampDate Pointer to Date structure
  * @param  Format specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, uint8_t TimeStampx, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format)
{
	uint32_t tmptime = 0U;
	uint32_t tmpdate = 0U;

	if(TimeStampx == RTC_TIMESTAMP_1)
	{
		/* Get the Timestamp time and date registers values */
		tmptime = (uint32_t)(hrtc->Instance->TAMP1TIME & RTC_TIME_RESERVED_MASK);
		tmpdate = (uint32_t)(hrtc->Instance->TAMP1DATE & RTC_DATE_RESERVED_MASK);
	}
	else if(TimeStampx == RTC_TIMESTAMP_2)
	{
		/* Get the Timestamp time and date registers values */
		tmptime = (uint32_t)(hrtc->Instance->TAMP2TIME & RTC_TIME_RESERVED_MASK);
		tmpdate = (uint32_t)(hrtc->Instance->TAMP2DATE & RTC_DATE_RESERVED_MASK);
	}
	else if(TimeStampx == RTC_TIMESTAMP_3)
	{
		/* Get the Timestamp time and date registers values */
		tmptime = (uint32_t)(hrtc->Instance->TAMP3TIME & RTC_TIME_RESERVED_MASK);
		tmpdate = (uint32_t)(hrtc->Instance->TAMP3DATE & RTC_DATE_RESERVED_MASK);
	}
	
	if((hrtc->Instance->TIME & RTC_TIME_HOUR12_24) ==  RTC_HOURFORMAT_24)
	{
		sTimeStamp->Hours = (uint8_t)((tmptime & (RTC_TIME_H20_PA  | RTC_TIME_HOUR19))  >> RTC_TIME_HOUR19_Pos);
	}
    else
	{
		sTimeStamp->Hours = (uint8_t)((tmptime & RTC_TIME_HOUR19)  >> RTC_TIME_HOUR19_Pos);
		sTimeStamp->TimeFormat = (uint8_t)((tmptime & (RTC_TIME_H20_PA )) >> RTC_TIME_H20_PA_Pos);
	}
    sTimeStamp->Minutes    = (uint8_t)((tmptime & (RTC_TIME_MINUTE )) >> RTC_TIME_MINUTE_Pos);
    sTimeStamp->Seconds    = (uint8_t)((tmptime & (RTC_TIME_SECOND )) >> RTC_TIME_SECOND_Pos);
	sTimeStamp->CentiSeconds = (uint8_t)((tmptime & (RTC_TIME_CENTISEC )) >> RTC_TIME_CENTISEC_Pos);

	/* Fill the Date structure fields with the read parameters */
	sTimeStampDate->Year    = (uint8_t)((tmpdate & (RTC_DATE_YEAR )) >> RTC_DATE_YEAR_Pos );
    sTimeStampDate->Month   = (uint8_t)((tmpdate & (RTC_DATE_MONTH)) >> RTC_DATE_MONTH_Pos);
    sTimeStampDate->Date    = (uint8_t)((tmpdate & (RTC_DATE_DAY  )) >> RTC_DATE_DAY_Pos  );
    sTimeStampDate->WeekDay = (uint8_t) (tmpdate & (RTC_DATE_WEEK ));
	
	/* Check the input parameters format */
	if (Format == RTC_FORMAT_BIN)
	{
		/* Convert the Timestamp structure parameters to Binary format */
		sTimeStamp->Hours   = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Hours);
		sTimeStamp->Minutes = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Minutes);
		sTimeStamp->Seconds = (uint8_t)RTC_Bcd2ToByte(sTimeStamp->Seconds);

		/* Convert the DateTimeStamp structure parameters to Binary format */
		sTimeStampDate->Year    = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->Year);
		sTimeStampDate->Month   = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->Month);
		sTimeStampDate->Date    = (uint8_t)RTC_Bcd2ToByte(sTimeStampDate->Date);
	}

	return HAL_OK;
}

/**
  * @brief  Sets Tamper with interrupt.
  * @note   By calling this API the tamper global interrupt will be enabled.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTamper Pointer to RTC Tamper.
  * @retval HAL status
  */
uint8_t	HAL_RTCEx_GetTamperCnt(RTC_HandleTypeDef *hrtc)
{
	return ((uint8_t)hrtc->Instance->TAMPCNT & 0x3f);
}

/**
  * @brief  Sets Tamper.
  * @note   By calling this API the tamper global interrupt will be enabled.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTamper Pointer to RTC Tamper.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper)
{
	/* Process Locked */
	__HAL_LOCK(hrtc);

	hrtc->State = HAL_RTC_STATE_BUSY;

	/* Set the TAPM_DBNC bit according to the  sTamper->FilterTime */
	MODIFY_REG(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_DBNC_Msk, sTamper->FilterTime);
	
	/* Set the TAPM_EDGE bit according to the  sTamper->TriggerEdge */
	MODIFY_REG(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_DBNC_Msk, sTamper->TriggerEdge);
	
	/* Clear RTC Tamper cnt */
	SET_BIT(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_CNT_CLR);
	
	/* Enable RTC Tamper */
	__HAL_RTC_TAMPER_ENABLE(hrtc);
	
	hrtc->State = HAL_RTC_STATE_READY;

	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);

	return HAL_OK;
}

/**
  * @brief  Sets Tamper with interrupt.
  * @note   By calling this API the tamper global interrupt will be enabled.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTamper Pointer to RTC Tamper.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper)
{
	/* Process Locked */
	__HAL_LOCK(hrtc);

	hrtc->State = HAL_RTC_STATE_BUSY;
	
	/* Enable RTC Tamper Wakeup */
	__HAL_PMU_UNLOCK_REGISTER();
	SET_BIT(PMU->PDWKCR, PMU_PDWKCR_RTC_TAMP_WKE);
	__HAL_PMU_LOCK_REGISTER();
	
	/* Set the TAPM_DBNC bit according to the  sTamper->FilterTime */
	MODIFY_REG(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_DBNC_Msk, sTamper->FilterTime);
	
	/* Set the TAPM_EDGE bit according to the  sTamper->TriggerEdge */
	MODIFY_REG(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_DBNC_Msk, sTamper->TriggerEdge);
	
	/* Clear RTC Tamper cnt */
	SET_BIT(hrtc->Instance->TAMPCTRL, RTC_TAMPCTRL_TAMP_CNT_CLR);
	
	/* Enable RTC Tamper */
	__HAL_RTC_TAMPER_ENABLE(hrtc);
	
	/* Enable RTC Tamper interrupt */
	__HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_TAMP);
	
	hrtc->State = HAL_RTC_STATE_READY;

	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);

	return HAL_OK;
}

/**
  * @brief  Handles Timestamp and Tamper interrupt request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc)
{
	/* Get the Tamper event interrupt source enable status */
	if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_TAMP) != 0U)
	{
		/* Get the pending status of the Tamper event Interrupt */
		if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_TAMP) != 0U)
		{
          /* Clear the Tamper event interrupt pending bit */
		  __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP);  
            
			/* Tamper callback */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
			hrtc->TamperEventCallback(hrtc);
#else
			HAL_RTCEx_TamperEventCallback(hrtc);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

		  
		}
	}

	/* Change RTC state */
	hrtc->State = HAL_RTC_STATE_READY;
}



#if defined(UM32x42x)
/**
  * @brief  This function handles Alarm interrupt request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTC_AlarmWakeupIRQHandler(RTC_HandleTypeDef* hrtc)
{
    /* Get the AlarmA interrupt source enable status */
    if(__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALM1) != (uint32_t)RESET)
    {
        /* Get the pending status of the AlarmA Interrupt */
        if(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALM1) != (uint32_t)RESET)
        {
            /* Clear the AlarmA interrupt pending bit */
            __HAL_RTC_ALARM_CLEAR_FLAG(hrtc,RTC_FLAG_ALM1); 
            
            /* AlarmA callback */
          #if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
            hrtc->Alarm1EventCallback(hrtc);
          #else
            HAL_RTC_Alarm1EventCallback(hrtc);
          #endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

           
        }
    }

    /* Get the AlarmB interrupt source enable status */
    if(__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALM2) != (uint32_t)RESET)
    {
        /* Get the pending status of the AlarmB Interrupt */
        if(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALM2) != (uint32_t)RESET)
        {
           /* Clear the AlarmB interrupt pending bit */
            __HAL_RTC_ALARM_CLEAR_FLAG(hrtc,RTC_FLAG_ALM2); 
            
            /* AlarmB callback */
          #if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
            hrtc->Alarm2EventCallback(hrtc);
          #else
            HAL_RTC_Alarm2EventCallback(hrtc);
          #endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

            
        }
    }



    /* Change RTC state */
    hrtc->State = HAL_RTC_STATE_READY;
}
#endif



/**
  * @brief  Handles Tamper Polling.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTCEx_PollForTamperEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
	uint32_t tickstart = 0U;

	/* Get tick */
	tickstart = HAL_GetTick();

	/* Get the status of the Interrupt */
	while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_TAMP) == 0U)
	{
	if (Timeout != HAL_MAX_DELAY)
	{
	  if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
	  {
		hrtc->State = HAL_RTC_STATE_TIMEOUT;
		return HAL_TIMEOUT;
	  }
	}
	}

	/* Clear the Tamper Flag */
	__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_TAMP);

	/* Change RTC state */
	hrtc->State = HAL_RTC_STATE_READY;

	return HAL_OK;
}

/**
  * @}
  */

/**
  * @brief  Alarm 1 callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_Alarm1EventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_RTC_AlarmAEventCallback could be implemented in the user file
   */
}

/**
  * @brief  Alarm 2 callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_Alarm2EventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_RTC_AlarmAEventCallback could be implemented in the user file
   */
}

/**
  * @brief  Tamper callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTCEx_TamperEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_RTC_AlarmAEventCallback could be implemented in the user file
   */
}








/**
  * @}
  */

#endif /* HAL_RTC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
