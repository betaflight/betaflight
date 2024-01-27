/**
  ******************************************************************************
  * @file    usbd_customhid_if_template.c
  * @author  MCD Application Team
  * @brief   USB Device Custom HID interface file.
  *        This template should be copied to the user folder, renamed and customized
  *          following user needs.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

static int8_t TEMPLATE_CUSTOM_HID_Init(void);
static int8_t TEMPLATE_CUSTOM_HID_DeInit(void);
static int8_t TEMPLATE_CUSTOM_HID_OutEvent(uint8_t event_idx, uint8_t state);

#ifdef USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED
static int8_t TEMPLATE_CUSTOM_HID_CtrlReqComplete(uint8_t request, uint16_t wLength);
#endif /* USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED */

#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED
static uint8_t *TEMPLATE_CUSTOM_HID_GetReport(uint16_t *ReportLength);
#endif /* USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED */
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef USBD_Device;

__ALIGN_BEGIN static uint8_t TEMPLATE_CUSTOM_HID_ReportDesc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END = {0};

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_template_fops =
{
  TEMPLATE_CUSTOM_HID_ReportDesc,
  TEMPLATE_CUSTOM_HID_Init,
  TEMPLATE_CUSTOM_HID_DeInit,
  TEMPLATE_CUSTOM_HID_OutEvent,
#ifdef USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED
  TEMPLATE_CUSTOM_HID_CtrlReqComplete,
#endif /* USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED */
#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED
  TEMPLATE_CUSTOM_HID_GetReport,
#endif /* USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED */
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
static int8_t TEMPLATE_CUSTOM_HID_OutEvent(uint8_t event_idx, uint8_t state)
{
  UNUSED(event_idx);
  UNUSED(state);

  /* Start next USB packet transfer once data processing is completed */
  if (USBD_CUSTOM_HID_ReceivePacket(&USBD_Device) != (uint8_t)USBD_OK)
  {
    return -1;
  }

  return (0);
}

#ifdef USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED
/**
  * @brief  TEMPLATE_CUSTOM_HID_CtrlReqComplete
  *         Manage the CUSTOM HID control request complete
  * @param  request: control request
  * @param  wLength: request wLength
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TEMPLATE_CUSTOM_HID_CtrlReqComplete(uint8_t request, uint16_t wLength)
{
  UNUSED(wLength);

  switch (request)
  {
    case CUSTOM_HID_REQ_SET_REPORT:

      break;

    case CUSTOM_HID_REQ_GET_REPORT:

      break;

    default:
      break;
  }

  return (0);
}
#endif /* USBD_CUSTOMHID_CTRL_REQ_COMPLETE_CALLBACK_ENABLED */


#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED
/**
  * @brief  TEMPLATE_CUSTOM_HID_GetReport
  *         Manage the CUSTOM HID control Get Report request
  * @param  event_idx: event index
  * @param  state: event state
  * @retval return pointer to HID report
  */
static uint8_t *TEMPLATE_CUSTOM_HID_GetReport(uint16_t *ReportLength)
{
  UNUSED(ReportLength);
  uint8_t *pbuff;

  return (pbuff);
}
#endif /* USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED */
