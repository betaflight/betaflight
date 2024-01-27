/**
  ******************************************************************************
  * @file    usbd_cdc_rndis_if_template.c
  * @author  MCD Application Team
  * @brief   Source file for USBD CDC_RNDIS interface template
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

/* Include TCP/IP stack header files */
/*
#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "http_cgi_ssi.h"
#include "ethernetif.h"
*/

#include "usbd_cdc_rndis_if_template.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Received Data over USB are stored in this buffer */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* __ICCARM__ */
__ALIGN_BEGIN uint8_t UserRxBuffer[CDC_RNDIS_ETH_MAX_SEGSZE + 100] __ALIGN_END;

/* Transmitted Data over CDC_RNDIS (CDC_RNDIS interface) are stored in this buffer */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* __ICCARM__ */
__ALIGN_BEGIN static uint8_t UserTxBuffer[CDC_RNDIS_ETH_MAX_SEGSZE + 100] __ALIGN_END;

static uint8_t CDC_RNDISInitialized = 0U;

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;


/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_RNDIS_Itf_Init(void);
static int8_t CDC_RNDIS_Itf_DeInit(void);
static int8_t CDC_RNDIS_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_RNDIS_Itf_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_RNDIS_Itf_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_RNDIS_Itf_Process(USBD_HandleTypeDef *pdev);

USBD_CDC_RNDIS_ItfTypeDef USBD_CDC_RNDIS_fops =
{
  CDC_RNDIS_Itf_Init,
  CDC_RNDIS_Itf_DeInit,
  CDC_RNDIS_Itf_Control,
  CDC_RNDIS_Itf_Receive,
  CDC_RNDIS_Itf_TransmitCplt,
  CDC_RNDIS_Itf_Process,
  (uint8_t *)CDC_RNDIS_MAC_STR_DESC,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_RNDIS_Itf_Init
  *         Initializes the CDC_RNDIS media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_Init(void)
{
  if (CDC_RNDISInitialized == 0U)
  {
    /*
      Initialize the LwIP stack
      Add your code here

    */

    CDC_RNDISInitialized = 1U;
  }

  /* Set Application Buffers */
#ifdef USE_USBD_COMPOSITE
  (void)USBD_CDC_RNDIS_SetTxBuffer(&USBD_Device, UserTxBuffer, 0U, 0U);
#else
  (void)USBD_CDC_RNDIS_SetTxBuffer(&USBD_Device, UserTxBuffer, 0U);
#endif /* USE_USBD_COMPOSITE */
  (void)USBD_CDC_RNDIS_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (0);
}

/**
  * @brief  CDC_RNDIS_Itf_DeInit
  *         DeInitializes the CDC_RNDIS media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_DeInit(void)
{
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *) \
                                                 (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  /*
     Add your code here
  */

  /* Notify application layer that link is down */
  hcdc_cdc_rndis->LinkStatus = 0U;

  return (0);
}

/**
  * @brief  CDC_RNDIS_Itf_Control
  *         Manage the CDC_RNDIS class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *) \
                                                 (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  switch (cmd)
  {
    case CDC_RNDIS_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_RNDIS_GET_ENCAPSULATED_RESPONSE:
      /* Check if this is the first time we enter */
      if (hcdc_cdc_rndis->LinkStatus == 0U)
      {
        /* Setup the Link up at TCP/IP stack level */
        hcdc_cdc_rndis->LinkStatus = 1U;
        /*
          Add your code here
        */
      }
      /* Add your code here */
      break;

    default:
      /* Add your code here */
      break;
  }

  UNUSED(length);
  UNUSED(pbuf);

  return (0);
}

/**
  * @brief  CDC_RNDIS_Itf_Receive
  *         Data received over USB OUT endpoint are sent over CDC_RNDIS interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  /* Get the CDC_RNDIS handler pointer */
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *) \
                                                 (USBD_Device.pClassDataCmsit[USBD_Device.classId]);
#else
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *)(USBD_Device.pClassData);
#endif /* USE_USBD_COMPOSITE */

  /* Call Eth buffer processing */
  hcdc_cdc_rndis->RxState = 1U;

  UNUSED(Buf);
  UNUSED(Len);

  return (0);
}

/**
  * @brief  CDC_RNDIS_Itf_TransmitCplt
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @param  epnum: EP number
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return (0);
}

/**
  * @brief  CDC_RNDIS_Itf_Process
  *         Data received over USB OUT endpoint are sent over CDC_RNDIS interface
  *         through this function.
  * @param  pdef: pointer to the USB Device Handle
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_RNDIS_Itf_Process(USBD_HandleTypeDef *pdev)
{
  /* Get the CDC_RNDIS handler pointer */
#ifdef USE_USBD_COMPOSITE
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *)(pdev->pClassDataCmsit[pdev->classId]);
#else
  USBD_CDC_RNDIS_HandleTypeDef *hcdc_cdc_rndis = (USBD_CDC_RNDIS_HandleTypeDef *)(pdev->pClassData);
#endif /* USE_USBD_COMPOSITE */

  if (hcdc_cdc_rndis == NULL)
  {
    return (-1);
  }

  if (hcdc_cdc_rndis->LinkStatus != 0U)
  {
    /*
       Add your code here
       Read a received packet from the Ethernet buffers and send it
       to the lwIP for handling
    */
  }

  return (0);
}

