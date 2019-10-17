/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.c
  * @author  MCD Application Team
  * @brief   USB Device Custom HID interface file.
  *		     This template should be copied to the user folder, renamed and customized
  *          following user needs.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid_if_template.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static int8_t TEMPLATE_CUSTOM_HID_Init     (void);
static int8_t TEMPLATE_CUSTOM_HID_DeInit   (void);
static int8_t TEMPLATE_CUSTOM_HID_OutEvent (uint8_t event_idx, uint8_t state);
/* Private variables ---------------------------------------------------------*/
USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_template_fops =
{
  TEMPLATE_CUSTOM_HID_ReportDesc,
  TEMPLATE_CUSTOM_HID_Init,
  TEMPLATE_CUSTOM_HID_DeInit,
  TEMPLATE_CUSTOM_HID_OutEvent,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TEMPLATE_CUSTOM_HID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_Init(void)
{

  return (0);
}

/**
  * @brief  TEMPLATE_CUSTOM_HID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  TEMPLATE_CUSTOM_HID_Control
  *         Manage the CUSTOM HID class events
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_OutEvent  (uint8_t event_idx, uint8_t state)
{

  return (0);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
