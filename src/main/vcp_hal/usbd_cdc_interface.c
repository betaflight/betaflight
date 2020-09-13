/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "platform.h"

#include "build/atomic.h"

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "stdbool.h"

#include "drivers/nvic.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define APP_TX_BLOCK_SIZE 512

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};

volatile uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;
volatile uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
volatile uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */

uint32_t rxAvailable = 0;
uint8_t* rxBuffPtr = NULL;

/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;

static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);

// The CDC_Itf_TransmitCplt field was introduced in MiddleWare that comes with
// H7 V1.8.0.
// Other MCU can be add here as the MiddleWare version advances.
#ifdef STM32H7
static int8_t CDC_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
#endif

static void TIM_Config(void);
static void Error_Handler(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive,
#ifdef STM32H7
  CDC_Itf_TransmitCplt
#endif
};


void TIMx_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TimHandle);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /*##-3- Configure the TIM Base generation  #################################*/
  TIM_Config();

  /*##-4- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /*##-5- Set Application Buffers ############################################*/
  USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, (uint8_t *)UserRxBuffer);

  ctrlLineStateCb = NULL;
  baudRateCb = NULL;

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  LINE_CODING* plc = (LINE_CODING*)pbuf;

  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    if (pbuf && (length == sizeof (*plc))) {
        LineCoding.bitrate    = plc->bitrate;
        LineCoding.format     = plc->format;
        LineCoding.paritytype = plc->paritytype;
        LineCoding.datatype   = plc->datatype;

        // If a callback is provided, tell the upper driver of changes in baud rate
        if (baudRateCb) {
            baudRateCb(baudRateCbContext, LineCoding.bitrate);
        }
    }

    break;

  case CDC_GET_LINE_CODING:
    if (pbuf && (length == sizeof (*plc))) {
        plc->bitrate = LineCoding.bitrate;
        plc->format = LineCoding.format;
        plc->paritytype = LineCoding.paritytype;
        plc->datatype = LineCoding.datatype;
    }
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    // If a callback is provided, tell the upper driver of changes in DTR/RTS state
    if (pbuf && (length == sizeof (uint16_t))) {
         if (ctrlLineStateCb) {
             ctrlLineStateCb(ctrlLineStateCbContext, *((uint16_t *)pbuf));
         }
    }
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIMusb) {
        return;
    }

    uint32_t buffsize;
    static uint32_t lastBuffsize = 0;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)USBD_Device.pCDC_ClassData;

    if (hcdc->TxState == 0) {
        // endpoint has finished transmitting previous block
        if (lastBuffsize) {
            bool needZeroLengthPacket = lastBuffsize % 64 == 0;

            // move the ring buffer tail based on the previous succesful transmission
            UserTxBufPtrOut += lastBuffsize;
            if (UserTxBufPtrOut == APP_TX_DATA_SIZE) {
                UserTxBufPtrOut = 0;
            }
            lastBuffsize = 0;

            if (needZeroLengthPacket) {
                USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], 0);
                return;
            }
        }
        if (UserTxBufPtrOut != UserTxBufPtrIn) {
            if (UserTxBufPtrOut > UserTxBufPtrIn) { /* Roll-back */
                buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
            } else {
                buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
            }
            if (buffsize > APP_TX_BLOCK_SIZE) {
                buffsize = APP_TX_BLOCK_SIZE;
            }

            USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], buffsize);

            if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) {
                lastBuffsize = buffsize;
            }
        }
    }
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
    rxAvailable = *Len;
    rxBuffPtr = Buf;
    if (!rxAvailable) {
        // Received an empty packet, trigger receiving the next packet.
        // This will happen after a packet that's exactly 64 bytes is received.
        // The USB protocol requires that an empty (0 byte) packet immediately follow.
        USBD_CDC_ReceivePacket(&USBD_Device);
    }
    return (USBD_OK);
}

#ifdef STM32H7
static int8_t CDC_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);

    return (USBD_OK);
}
#endif

/**
  * @brief  TIM_Config: Configure TIMusb timer
  * @param  None.
  * @retval None
  */
static void TIM_Config(void)
{
  /* Set TIMusb instance */
  TimHandle.Instance = TIMusb;

  /* Initialize TIMx peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*1000) - 1;
  TimHandle.Init.Prescaler = (SystemCoreClock / 2 / (1000000)) - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-6- Enable TIM peripherals Clock #######################################*/
  TIMx_CLK_ENABLE();

  /*##-7- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, 6, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Add your own code here */
}

uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len)
{
    uint32_t count = 0;
    if ( (rxBuffPtr != NULL))
    {
        while ((rxAvailable > 0) && count < len)
        {
            recvBuf[count] = rxBuffPtr[0];
            rxBuffPtr++;
            rxAvailable--;
            count++;
            if (rxAvailable < 1)
                USBD_CDC_ReceivePacket(&USBD_Device);
        }
    }
    return count;
}

uint32_t CDC_Receive_BytesAvailable(void)
{
    return rxAvailable;
}

uint32_t CDC_Send_FreeBytes(void)
{
    /*
        return the bytes free in the circular buffer

        functionally equivalent to:
        (APP_Rx_ptr_out > APP_Rx_ptr_in ? APP_Rx_ptr_out - APP_Rx_ptr_in : APP_RX_DATA_SIZE - APP_Rx_ptr_in + APP_Rx_ptr_in)
        but without the impact of the condition check.
    */
    uint32_t freeBytes;

    ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
        freeBytes = ((UserTxBufPtrOut - UserTxBufPtrIn) + (-((int)(UserTxBufPtrOut <= UserTxBufPtrIn)) & APP_TX_DATA_SIZE)) - 1;
    }

    return freeBytes;
}

/**
 * @brief  CDC_Send_DATA
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  ptrBuffer: Buffer of data to be sent
 * @param  sendLength: Number of data to be sent (in bytes)
 * @retval Bytes sent
 */
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0) {
            // block until there is free space in the ring buffer
            delay(1);
        }
        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) { // Paranoia
            UserTxBuffer[UserTxBufPtrIn] = ptrBuffer[i];
            UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
        }
    }
    return sendLength;
}


/*******************************************************************************
 * Function Name  : usbIsConfigured.
 * Description    : Determines if USB VCP is configured or not
 * Input          : None.
 * Output         : None.
 * Return         : True if configured.
 *******************************************************************************/
uint8_t usbIsConfigured(void)
{
    return (USBD_Device.dev_state == USBD_STATE_CONFIGURED);
}

/*******************************************************************************
 * Function Name  : usbIsConnected.
 * Description    : Determines if USB VCP is connected ot not
 * Input          : None.
 * Output         : None.
 * Return         : True if connected.
 *******************************************************************************/
uint8_t usbIsConnected(void)
{
    return (USBD_Device.dev_state != USBD_STATE_DEFAULT);
}

/*******************************************************************************
 * Function Name  : CDC_BaudRate.
 * Description    : Get the current baud rate
 * Input          : None.
 * Output         : None.
 * Return         : Baud rate in bps
 *******************************************************************************/
uint32_t CDC_BaudRate(void)
{
    return LineCoding.bitrate;
}

/*******************************************************************************
 * Function Name  : CDC_SetBaudRateCb
 * Description    : Set a callback to call when baud rate changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

/*******************************************************************************
 * Function Name  : CDC_SetCtrlLineStateCb
 * Description    : Set a callback to call when control line state changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
