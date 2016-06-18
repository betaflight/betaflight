/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if_template.h"
#include  "stm32_eval.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

/* Private function prototypes -----------------------------------------------*/
static uint16_t TEMPLATE_Init     (void);
static uint16_t TEMPLATE_DeInit   (void);
static uint16_t TEMPLATE_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t TEMPLATE_DataTx   (uint8_t* Buf, uint32_t Len);
static uint16_t TEMPLATE_DataRx (uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef TEMPLATE_fops = 
{
  TEMPLATE_Init,
  TEMPLATE_DeInit,
  TEMPLATE_Ctrl,
  TEMPLATE_DataTx,
  TEMPLATE_DataRx
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TEMPLATE_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t TEMPLATE_Init(void)
{
  /*
     Add your initialization code here 
  */  
  return USBD_OK;
}

/**
  * @brief  TEMPLATE_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t TEMPLATE_DeInit(void)
{
  /*
     Add your deinitialization code here 
  */  
  return USBD_OK;
}


/**
  * @brief  TEMPLATE_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t TEMPLATE_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case SET_LINE_CODING:
    /* Add your code here */
    break;

  case GET_LINE_CODING:
    /* Add your code here */
    break;

  case SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  TEMPLATE_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t TEMPLATE_DataTx (uint8_t* Buf, uint32_t Len)
{

  /* Get the data to be sent */
  for (i = 0; i < Len; i++)
  {
    /* APP_Rx_Buffer[APP_Rx_ptr_in] = XXX_ReceiveData(XXX); */
  }

  /* Increment the in pointer */
  APP_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  }  
  
  return USBD_OK;
}

/**
  * @brief  TEMPLATE_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t TEMPLATE_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;
  
  /* Send the received buffer */
  for (i = 0; i < Len; i++)
  {
    /* XXXX_SendData(XXXX, *(Buf + i) ); */
  } 
 
  return USBD_OK;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
