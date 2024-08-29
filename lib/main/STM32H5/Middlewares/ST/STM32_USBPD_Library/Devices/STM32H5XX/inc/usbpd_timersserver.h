/**
  ******************************************************************************
  * @file    usbpd_timersserver.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_timerserver.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __USBPD_TIMERSSERVER_H_
#define __USBPD_TIMERSSERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_TIMESERVER
  * @{
  */

/* Exported constants --------------------------------------------------------*/
typedef enum
{
  TIM_PORT0_CRC,
  TIM_PORT0_RETRY,
  TIM_PORT1_CRC,
  TIM_PORT1_RETRY,
  TIM_MAX
}
TIM_identifier;

#define TIM_MAX_TIME 10000u /*time in us, means 10 ms */
/* Exported types ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void     USBPD_TIM_Init(void);
void     USBPD_TIM_DeInit(void);
void     USBPD_TIM_Start(TIM_identifier Id, uint32_t TimeUs);
uint32_t USBPD_TIM_IsExpired(TIM_identifier Id);

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

#endif /* __USBPD_TIMERSSERVER_H_ */

