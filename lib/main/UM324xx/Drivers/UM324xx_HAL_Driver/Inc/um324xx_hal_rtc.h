/**
  ******************************************************************************
  * @file     um324xx_hal_rtc.h 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_RTC_H__
#define __UM324XX_HAL_RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "um324xx_hal_def.h"



/** @addtogroup UM324xx_HAL_Driver
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
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,  /*!< RTC not yet initialized or disabled */
  HAL_RTC_STATE_READY             = 0x01U,  /*!< RTC initialized and ready for use   */
  HAL_RTC_STATE_BUSY              = 0x02U,  /*!< RTC process is ongoing              */
  HAL_RTC_STATE_TIMEOUT           = 0x03U,  /*!< RTC timeout state                   */
  HAL_RTC_STATE_ERROR             = 0x04U   /*!< RTC error state                     */
} HAL_RTCStateTypeDef;

/**
  * @brief  RTC Configuration Structure definition
  */
typedef struct
{
  uint32_t HourFormat;      /*!< Specifies the RTC Hour Format.
                                 This parameter can be a value of @ref RTC_Hour_Formats */

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
	
  uint8_t CentiSeconds;     /*!< Specifies the RTC Time CentiSeconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

  uint8_t TimeFormat;       /*!< Specifies the RTC AM/PM Time.
                                 This parameter can be a value of @ref RTC_AM_PM_Definitions */

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
	
  RTC_DateTypeDef AlarmDate;     /*!< Specifies the RTC Alarm Date members */	

  uint32_t AlarmMask;           /*!< Specifies the RTC Alarm1 Masks.
                                      This parameter can be a value of @ref RTC_Alarm1Mask_Definitions  or @ref RTC_Alarm2Mask_Definitions*/

  uint32_t Alarm;                /*!< Specifies the alarm.
                                      This parameter can be a value of @ref RTC_Alarms_Definitions */
} RTC_AlarmTypeDef;

/**
  * @brief  RTC Tamper structure definition
  */
typedef struct
{

  uint32_t TriggerEdge;                 /*!< Specifies the Tamper Trigger Edge.
                                             This parameter can be a value of @ref RTCEx_Tamper_Trigger_Edge_Definitions */

  uint32_t FilterTime;                  /*!< Specifies the RTC Tamper Filter Time.
                                             This parameter can be a value of @ref RTCEx_Tamper_Filter_Time_Definitions */

} RTC_TamperTypeDef;

/**
  * @brief  RTC Handle Structure definition
  */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
typedef struct __RTC_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */
{
  RTC_TypeDef                 *Instance;  /*!< Register base address    */

  RTC_InitTypeDef             Init;       /*!< RTC required parameters  */

  HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  __IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
  void (* Alarm1EventCallback)      (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Alarm 1 Event callback         */

  void (* Alarm2EventCallback)      (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Alarm 2 Event callback         */

  void (* TamperEventCallback)      (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Tamper Event callback          */

  void (* MspInitCallback)          (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Msp Init callback              */

  void (* MspDeInitCallback)        (struct __RTC_HandleTypeDef *hrtc);  /*!< RTC Msp DeInit callback            */

#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

} RTC_HandleTypeDef;

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL RTC Callback ID enumeration definition
  */
typedef enum
{
  HAL_RTC_ALARM_1_EVENT_CB_ID           = 0x00U,    /*!< RTC Alarm 1 Event Callback ID       */
  HAL_RTC_ALARM_2_EVENT_CB_ID           = 0x01U,    /*!< RTC Alarm 2 Event Callback ID       */
  HAL_RTC_TAMPER_EVENT_CB_ID            = 0x02U,    /*!< RTC Tamper Callback ID            */
  HAL_RTC_MSPINIT_CB_ID                 = 0x03U,    /*!< RTC Msp Init callback ID            */
  HAL_RTC_MSPDEINIT_CB_ID               = 0x04U     /*!< RTC Msp DeInit callback ID          */
} HAL_RTC_CallbackIDTypeDef;

/**
  * @brief  HAL RTC Callback pointer definition
  */
typedef  void (*pRTC_CallbackTypeDef)(RTC_HandleTypeDef *hrtc);  /*!< pointer to an RTC callback function */
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup RTC_Exported_Constants RTC Exported Constants
  * @{
  */

/** @defgroup RTC_AM_PM_Definitions RTC AM PM Definitions
  * @{
  */
#define RTC_HOURFORMAT12_AM            0x00U
#define RTC_HOURFORMAT12_PM            ((uint8_t)0x01)
/**
  * @}
  */

/** @defgroup RTC_Hour_Formats RTC Hour Formats
  * @{
  */
#define RTC_HOURFORMAT_12              0x00000000U
#define RTC_HOURFORMAT_24              RTC_TIME_HOUR12_24
/**
  * @}
  */

/** @defgroup RTCEx_Backup_Registers_Definitions RTCEx Backup Registers Definitions
  * @{
  */
#define RTC_BKREG0                       0x00000000U
#define RTC_BKREG1                       0x00000001U
#define RTC_BKREG2                       0x00000002U
#define RTC_BKREG3                       0x00000003U
#define RTC_BKREG4                       0x00000004U
#define RTC_BKREG5                       0x00000005U
#define RTC_BKREG6                       0x00000006U
#define RTC_BKREG7                       0x00000007U
#define RTC_BKREG8                       0x00000008U
#define RTC_BKREG9                       0x00000009U
#define RTC_BKREG10                      0x0000000AU
#define RTC_BKREG11                      0x0000000BU
#define RTC_BKREG12                      0x0000000CU
#define RTC_BKREG13                      0x0000000DU
#define RTC_BKREG14                      0x0000000EU
#define RTC_BKREG15                      0x0000000FU
#define RTC_BKREG16                      0x00000010U
#define RTC_BKREG17                      0x00000011U
#define RTC_BKREG18                      0x00000012U
#define RTC_BKREG19                      0x00000013U
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

/** @defgroup RTC_Alarm1Mask_Definitions RTC Alarm1 Mask Definitions
  * @{
  */
#define RTC_ALARM1MASK_NONE                	0x00000000U
#define RTC_ALARM1MASK_YEAR		          	RTC_ALM1EN_ALM1_EN_YEAR
#define RTC_ALARM1MASK_MON		          	RTC_ALM1EN_ALM1_EN_MON
#define RTC_ALARM1MASK_DAY		          	RTC_ALM1EN_ALM1_EN_DAY
#define RTC_ALARM1MASK_WEEK		          	RTC_ALM1EN_ALM1_EN_WEEK
#define RTC_ALARM1MASK_HOURS               	RTC_ALM1EN_ALM1_EN_HOUR
#define RTC_ALARM1MASK_MINUTES             	RTC_ALM1EN_ALM1_EN_MIN
#define RTC_ALARM1MASK_SECONDS             	RTC_ALM1EN_ALM1_EN_SEC
#define RTC_ALARM1MASK_CENTISEC            	RTC_ALM1EN_ALM1_EN_CS
#define RTC_ALARM1MASK_DATE                	(RTC_ALARM1MASK_YEAR | \
											RTC_ALARM1MASK_MON  | \
											RTC_ALARM1MASK_DAY)
#define RTC_ALARM1MASK_DATEWEEK            	(RTC_ALARM1MASK_DATE | \
											RTC_ALARM1MASK_WEEK)
#define RTC_ALARM1MASK_TIME                	(RTC_ALARM1MASK_HOURS   | \
											RTC_ALARM1MASK_MINUTES | \
											RTC_ALARM1MASK_SECONDS | \
											RTC_ALARM1MASK_CENTISEC)
#define RTC_ALARM1MASK_ALL		          	(RTC_ALARM1MASK_DATEWEEK | \
											RTC_ALARM1MASK_TIME)									   
/**
  * @}
  */

/** @defgroup RTC_Alarm2Mask_Definitions RTC Alarm2 Mask Definitions
  * @{
  */
#define RTC_ALARM2MASK_NONE                	0x00000000U
#define RTC_ALARM2MASK_SEC_1DIV	          	((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_0))
#define RTC_ALARM2MASK_SEC_2DIV	          	((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_1))
#define RTC_ALARM2MASK_SEC_4DIV	          	((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_0 | RTC_ALM2SETTING_ALM2_SETTING_1))
#define RTC_ALARM2MASK_SEC_8DIV	          	((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_2))
#define RTC_ALARM2MASK_SEC_16DIV	        ((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_2 | RTC_ALM2SETTING_ALM2_SETTING_0))
#define RTC_ALARM2MASK_SEC_32DIV	        ((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_2 | RTC_ALM2SETTING_ALM2_SETTING_1))
#define RTC_ALARM2MASK_SEC_64DIV	        ((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_0 | RTC_ALM2SETTING_ALM2_SETTING_1 | RTC_ALM2SETTING_ALM2_SETTING_2))
#define RTC_ALARM2MASK_SEC_128DIV	        ((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_3))
#define RTC_ALARM2MASK_MIN_1DIV	          	((uint32_t)(RTC_ALM2SETTING_ALM2_SETTING_3 | RTC_ALM2SETTING_ALM2_SETTING_0))
/**
  * @}
  */

/** @defgroup RTC_Alarms_Definitions RTC Alarms Definitions
  * @{
  */
#define RTC_TIMESTAMP_1                       0x00000001U
#define RTC_TIMESTAMP_2                       0x00000002U
#define RTC_TIMESTAMP_3                       0x00000003U
/**
  * @}
  */

/** @defgroup RTC_TimeStampx_Definitions RTC Alarms Definitions
  * @{
  */
#define RTC_ALARM_1                       0x00000001U
#define RTC_ALARM_2                       0x00000002U
/**
  * @}
  */

/** @defgroup RTC_Interrupts_Definitions RTC Interrupts Definitions
  * @{
  */
#define RTC_IT_TAMP                       RTC_INTEN_TAMP_INT_EN       /*!< Enable Tamper Interrupt                  */
#define RTC_IT_ALM2                       RTC_INTEN_ALM2_INT_EN       /*!< Enable Alarm 2 Interrupt                 */
#define RTC_IT_ALM1                       RTC_INTEN_ALM1_INT_EN       /*!< Enable Alarm 1 Interrupt                 */
/**
  * @}
  */

/** @defgroup RTC_Flags_Definitions RTC Flags Definitions
  * @{
  */
#define RTC_FLAG_TAMP                     RTC_INTSTA_TAMP_INT_STA     /*!< Tamper event interrupt flag              */
#define RTC_FLAG_ALM2                     RTC_INTSTA_ALM2_INT_STA     /*!< Alarm2 interrupt flag                    */
#define RTC_FLAG_ALM1                     RTC_INTSTA_ALM1_INT_STA     /*!< Alarm1 interrupt flag                    */
/**
  * @}
  */

/** @defgroup RTCEx_Tamper_Filter_Time_Definitions RTCEx Tamper Filter Definitions
  * @{
  */
#define RTC_TAMP_FILTER_NONE              0x00000000			 
#define RTC_TAMP_FILTER_2MS               ((uint32_t)RTC_TAMPCTRL_TAMP_DBNC_0)
#define RTC_TAMP_FILTER_4MS               ((uint32_t)RTC_TAMPCTRL_TAMP_DBNC_1)
#define RTC_TAMP_FILTER_6MS               ((uint32_t)RTC_TAMPCTRL_TAMP_DBNC_0 | RTC_TAMPCTRL_TAMP_DBNC_1)
/**
  * @}
  */

/** @defgroup RTCEx_Tamper_Trigger_Edge_Definitions RTCEx Trigger Edge Definitions
  * @{
  */
#define RTC_TAMP_TRIGGER_EDGE_RISING      			0x00000000
#define RTC_TAMP_TRIGGER_EDGE_FALLING     			((uint32_t)RTC_TAMPCTRL_TAMP_EDGE_0)
#define RTC_TAMP_TRIGGER_EDGE_RISING_FALLING        ((uint32_t)RTC_TAMPCTRL_TAMP_EDGE_1)
#define RTC_TAMP_TRIGGER_OVER_DBNC_HIGH         	((uint32_t)RTC_TAMPCTRL_TAMP_EDGE_0 | RTC_TAMPCTRL_TAMP_EDGE_1)
#define RTC_TAMP_TRIGGER_OVER_DBNC_LOW	         	((uint32_t)RTC_TAMPCTRL_TAMP_EDGE_2)
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
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__) do {                                            \
                                                      (__HANDLE__)->State = HAL_RTC_STATE_RESET; \
                                                      (__HANDLE__)->MspInitCallback = NULL;      \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;    \
                                                    } while(0U)
#else
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_RTC_STATE_RESET)
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

/**
  * @brief  Start writing to register RTC_TIME and RTC_DATE, CENTISEC counter stop count.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WRITECOUNTER_START(__HANDLE__) do {                                          \
                                                      (__HANDLE__)->Instance->ACCESS = RTC_ACCESS_WRSTA;  \
                                                    } while(0U)
/**
  * @brief  Stop writing to register RTC_TIME and RTC_DATE, CENTISEC counter start count.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WRITECOUNTER_STOP(__HANDLE__)  do {                                          \
                                                      (__HANDLE__)->Instance->ACCESS = RTC_ACCESS_WRSTP;  \
                                                    } while(0U)

/**
  * @brief  Start reading register RTC_TIME and RTC_DATE.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_READCOUNTER_START(__HANDLE__) do {                                          \
                                                      (__HANDLE__)->Instance->ACCESS = RTC_ACCESS_RDSTA;  \
                                                    } while(0U)
/**
  * @brief  Stop reading to register RTC_TIME and RTC_DATE.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_READCOUNTER_STOP(__HANDLE__)  do {                                          \
                                                      (__HANDLE__)->Instance->ACCESS = RTC_ACCESS_RDSTP;  \
                                                    } while(0U)													
													
/**
  * @brief  Enable the RTC ALARM1 peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM1_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->ALM1EN |= (RTC_ALM1EN_ALM1_EN))

/**
  * @brief  Disable the RTC ALARM1 peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM1_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->ALM1EN &= ~(RTC_ALM1EN_ALM1_EN))

/**
  * @brief  Enable the RTC ALARM2 peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM2_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->ALM2SETTING |= (RTC_ALM2SETTING_ALM2_EN))

/**
  * @brief  Disable the RTC ALARM2 peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM2_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->ALM2SETTING &= ~(RTC_ALM2SETTING_ALM2_EN))

/**
  * @brief  Enable the RTC Tamper peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_TAMPER_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->TAMPCTRL |= (RTC_TAMPCTRL_TAMP_EN))

/**
  * @brief  Disable the RTC Tamper peripheral.
  * @param  __HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_TAMPER_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->TAMPCTRL &= ~(RTC_TAMPCTRL_TAMP_EN))

/**
  * @brief  Enable the RTC Alarm interrupt.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg RTC_IT_ALM1: Alarm 1 interrupt
  *             @arg RTC_IT_ALM2: Alarm 2 interrupt
  *             @arg RTC_IT_TAMP: Alarm Tamper interrupt 
  * @retval None
  */
#define __HAL_RTC_ALARM_ENABLE_IT(__HANDLE__, __INTERRUPT__)          ((__HANDLE__)->Instance->INTEN |= (__INTERRUPT__))

/**
  * @brief  Disable the RTC Alarm interrupt.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg RTC_IT_ALM1: Alarm 1 interrupt
  *             @arg RTC_IT_ALM2: Alarm 2 interrupt
  *             @arg RTC_IT_TAMP: Alarm Tamper interrupt 
  * @retval None
  */
#define __HAL_RTC_ALARM_DISABLE_IT(__HANDLE__, __INTERRUPT__)         ((__HANDLE__)->Instance->INTEN &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has occurred or not.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt to check.
  *         This parameter can be:
  *             @arg RTC_IT_ALM1: Alarm 1 interrupt
  *             @arg RTC_IT_ALM2: Alarm 2 interrupt
  *             @arg RTC_IT_TAMP: Alarm Tamper interrupt 
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT(__HANDLE__, __INTERRUPT__)           (((((__HANDLE__)->Instance->INTEN) & ((__INTERRUPT__))) != 0U) ? 1U : 0U)

/**
  * @brief  Get the selected RTC Alarm's flag status.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __FLAG__ specifies the RTC Alarm Flag to check.
  *         This parameter can be:
  *            @arg RTC_FLAG_TAMP: Tamper interrupt flag
  *            @arg RTC_FLAG_ALM2: Alarm 2 interrupt flag
  *            @arg RTC_FLAG_ALM1: Alarm 1 interrupt flag
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_FLAG(__HANDLE__, __FLAG__)                (((((__HANDLE__)->Instance->INTSTA) & (__FLAG__)) != 0U) ? 1U : 0U)

/**
  * @brief  Clear the RTC Alarm's pending flags.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __FLAG__ specifies the RTC Alarm flag to be cleared.
  *          This parameter can be:
  *            @arg RTC_FLAG_TAMP
  *            @arg RTC_FLAG_ALM2
  *            @arg RTC_FLAG_ALM1
  * @retval None
  */
#define __HAL_RTC_ALARM_CLEAR_FLAG(__HANDLE__, __FLAG__)                  ((__HANDLE__)->Instance->INTCLR = (__FLAG__))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has been enabled or not.
  * @param  __HANDLE__ specifies the RTC handle.
  * @param  __INTERRUPT__ specifies the RTC Alarm interrupt sources to check.
  *         This parameter can be:
  *            @arg RTC_IT_ALM1: Alarm 1 interrupt
  *            @arg RTC_IT_ALM2: Alarm 2 interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (((((__HANDLE__)->Instance->INTEN) & (__INTERRUPT__)) != 0U) ? 1U : 0U)

/* Exported functions --------------------------------------------------------*/

/** @addtogroup RTC_Exported_Functions
  * @{
  */

/** @addtogroup RTC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ****************************/
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_RTC_RegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID, pRTC_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_RTC_UnRegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group2
  * @{
  */
/* RTC Time and Date functions ************************************************/
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group3
  * @{
  */
/* RTC Alarm functions ********************************************************/
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void              HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarm1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void              HAL_RTC_Alarm1EventCallback(RTC_HandleTypeDef *hrtc);
void 			  HAL_RTC_Alarm2EventCallback(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t          HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);
/**
  * @}
  */

/** @addtogroup RTCEx_Exported_Functions_Group4
  * @{
  */
/* RTC Timestamp and Tamper functions *****************************************/
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, uint8_t TimeStampx, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);
uint8_t			  HAL_RTCEx_GetTamperCnt(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
void              HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);
void 			  HAL_RTCEx_TamperEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTamperEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group5
  * @{
  */
/* Peripheral State functions *************************************************/
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);
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
#define RTC_TIME_RESERVED_MASK    ((uint32_t)( RTC_TIME_H20_PA  | \
                                            RTC_TIME_HOUR19 | RTC_TIME_MINUTE | \
                                            RTC_TIME_SECOND  | RTC_TIME_CENTISEC ))
#define RTC_DATE_RESERVED_MASK    ((uint32_t)(RTC_DATE_CENTURY | RTC_DATE_YEAR | \
                                            RTC_DATE_MONTH | RTC_DATE_DAY    | \
                                            RTC_DATE_WEEK))

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup RTC_Private_Functions RTC Private Functions
  * @{
  */
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

#endif /* HAL_RTC_H */
