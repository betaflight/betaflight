/**
  ******************************************************************************
  * @file    Src/usbd_cdc_ecm_if_template.c
  * @author  MCD Application Team
  * @brief   Source file for USBD CDC_ECM interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "usbd_cdc_ecm_if_template.h"
/*

  Include here  LwIP files if used

*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Received Data over USB are stored in this buffer */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* ( __ICCARM__ ) */
__ALIGN_BEGIN static uint8_t UserRxBuffer[CDC_ECM_ETH_MAX_SEGSZE + 100]__ALIGN_END;

/* Transmitted Data over CDC_ECM (CDC_ECM interface) are stored in this buffer */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* ( __ICCARM__ ) */
__ALIGN_BEGIN  static uint8_t UserTxBuffer[CDC_ECM_ETH_MAX_SEGSZE + 100]__ALIGN_END;

static uint8_t CDC_ECMInitialized = 0U;

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_ECM_Itf_Init(void);
static int8_t CDC_ECM_Itf_DeInit(void);
static int8_t CDC_ECM_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_ECM_Itf_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_ECM_Itf_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_ECM_Itf_Process(USBD_HandleTypeDef *pdev);

USBD_CDC_ECM_ItfTypeDef USBD_CDC_ECM_fops =
{
  CDC_ECM_Itf_Init,
  CDC_ECM_Itf_DeInit,
  CDC_ECM_Itf_Control,
  CDC_ECM_Itf_Receive,
  CDC_ECM_Itf_TransmitCplt,
  CDC_ECM_Itf_Process,
  (uint8_t *)CDC_ECM_MAC_STR_DESC,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_ECM_Itf_Init
  *         Initializes the CDC_ECM media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_Init(void)
{
  if (CDC_ECMInitialized == 0U)
  {
    /*
      Initialize the TCP/IP stack here
    */

    CDC_ECMInitialized = 1U;
  }

  /* Set Application Buffers */
#ifdef USE_USBD_COMPOSITE
  (void)USBD_CDC_ECM_SetTxBuffer(&USBD_Device, UserTxBuffer, 0U, 0U);
#else
  (void)USBD_CDC_ECM_SetTxBuffer(&USBD_Device, UserTxBuffer, 0U);
#endif /* USE_USBD_COMPOSITE */
  (void)USBD_CDC_ECM_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (0);
}

/**
  * @brief  CDC_ECM_Itf_DeInit
  *         DeInitializes the CDC_ECM media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_DeInit(void)
{
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *) \
                                             (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  /* Notify application layer that link is down */
  hcdc_cdc_ecm->LinkStatus = 0U;

  return (0);
}

/**
  * @brief  CDC_ECM_Itf_Control
  *         Manage the CDC_ECM class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *) \
                                             (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  switch (cmd)
  {
    case CDC_ECM_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_ECM_GET_ENCAPSULATED_RESPONSE:
      /* Add your code here */
      break;

    case CDC_ECM_SET_ETH_MULTICAST_FILTERS:
      /* Add your code here */
      break;

    case CDC_ECM_SET_ETH_PWRM_PATTERN_FILTER:
      /* Add your code here */
      break;

    case CDC_ECM_GET_ETH_PWRM_PATTERN_FILTER:
      /* Add your code here */
      break;

    case CDC_ECM_SET_ETH_PACKET_FILTER:
      /* Check if this is the first time we enter */
      if (hcdc_cdc_ecm->LinkStatus == 0U)
      {
        /*
          Setup the Link up at TCP/IP level
        */
        hcdc_cdc_ecm->LinkStatus = 1U;

        /* Modification for MacOS which doesn't send SetInterface before receiving INs */
        if (hcdc_cdc_ecm->NotificationStatus == 0U)
        {
          /* Send notification: NETWORK_CONNECTION Event */
          (void)USBD_CDC_ECM_SendNotification(&USBD_Device, NETWORK_CONNECTION,
                                              CDC_ECM_NET_CONNECTED, NULL);

          /* Prepare for sending Connection Speed Change notification */
          hcdc_cdc_ecm->NotificationStatus = 1U;
        }
      }
      /* Add your code here */
      break;

    case CDC_ECM_GET_ETH_STATISTIC:
      /* Add your code here */
      break;

    default:
      break;
  }
  UNUSED(length);
  UNUSED(pbuf);

  return (0);
}

/**
  * @brief  CDC_ECM_Itf_Receive
  *         Data received over USB OUT endpoint are sent over CDC_ECM interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  /* Get the CDC_ECM handler pointer */
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *) \
                                             (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  /* Call Eth buffer processing */
  hcdc_cdc_ecm->RxState = 1U;

  UNUSED(Len);
  UNUSED(Buf);

  return (0);
}

/**
  * @brief  CDC_ECM_Itf_TransmitCplt
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return (0);
}

/**
  * @brief  CDC_ECM_Itf_Process
  *         Data received over USB OUT endpoint are sent over CDC_ECM interface
  *         through this function.
  * @param  pdef: pointer to the USB Device Handle
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_ECM_Itf_Process(USBD_HandleTypeDef *pdev)
{
  /* Get the CDC_ECM handler pointer */
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *)(pdev->pClassDataCmsit[pdev->classId]);
#else
  USBD_CDC_ECM_HandleTypeDef *hcdc_cdc_ecm = (USBD_CDC_ECM_HandleTypeDef *)(pdev->pClassData);
#endif /* USE_USBD_COMPOSITE */

  if (hcdc_cdc_ecm == NULL)
  {
    return (-1);
  }

  if (hcdc_cdc_ecm->LinkStatus != 0U)
  {
    /*
      Read a received packet from the Ethernet buffers and send it
      to the lwIP for handling
      Call here the TCP/IP background tasks.
    */
  }

  return (0);
}

