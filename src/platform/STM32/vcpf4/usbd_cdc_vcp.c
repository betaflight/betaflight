/**
 ******************************************************************************
 * @file    usbd_cdc_vcp.c
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

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

#include "platform.h"

#include "build/atomic.h"

#include "usbd_cdc_vcp.h"
#include "stm32f4xx_conf.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma     data_alignment = 4
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

LINE_CODING g_lc;

extern __IO uint8_t USB_Tx_State;
__IO uint32_t bDeviceState = UNCONNECTED; /* USB device status */

/* These are external variables imported from CDC core to be used for IN transfer management. */

/* This is the buffer for data received from the MCU to APP (i.e. MCU TX, APP RX) */
extern uint8_t APP_Rx_Buffer[];
extern volatile uint32_t APP_Rx_ptr_out;
/* Increment this buffer position or roll it back to
 start address when writing received data
 in the buffer APP_Rx_Buffer. */
extern volatile uint32_t APP_Rx_ptr_in;

/*
    APP TX is the circular buffer for data that is transmitted from the APP (host)
    to the USB device (flight controller).
*/
static uint8_t APP_Tx_Buffer[APP_TX_DATA_SIZE];
static uint32_t APP_Tx_ptr_out = 0;
static uint32_t APP_Tx_ptr_in = 0;

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(const uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len);
static void (*ctrlLineStateCb)(void* context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;


CDC_IF_Prop_TypeDef VCP_fops = {VCP_Init, VCP_DeInit, VCP_Ctrl, VCP_DataTx, VCP_DataRx };

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  VCP_Init
 *         Initializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_Init(void)
{
    bDeviceState = CONFIGURED;
    ctrlLineStateCb = NULL;
    baudRateCb = NULL;
    return USBD_OK;
}

/**
 * @brief  VCP_DeInit
 *         DeInitializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_DeInit(void)
{
    bDeviceState = UNCONNECTED;
    return USBD_OK;
}

void ust_cpy(LINE_CODING* plc2, const LINE_CODING* plc1)
{
   plc2->bitrate    = plc1->bitrate;
   plc2->format     = plc1->format;
   plc2->paritytype = plc1->paritytype;
   plc2->datatype   = plc1->datatype;
}

/**
 * @brief  VCP_Ctrl
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
    LINE_CODING* plc = (LINE_CODING*)Buf;

    assert_param(Len>=sizeof(LINE_CODING));

    switch (Cmd) {
       /* Not  needed for this driver, AT modem commands */
      case SEND_ENCAPSULATED_COMMAND:
      case GET_ENCAPSULATED_RESPONSE:
         break;

      // Not needed for this driver
      case SET_COMM_FEATURE:
      case GET_COMM_FEATURE:
      case CLEAR_COMM_FEATURE:
         break;


      //Note - hw flow control on UART 1-3 and 6 only
      case SET_LINE_CODING:
         // If a callback is provided, tell the upper driver of changes in baud rate
         if (plc && (Len == sizeof(*plc))) {
             if (baudRateCb) {
                 baudRateCb(baudRateCbContext, plc->bitrate);
             }
             ust_cpy(&g_lc, plc);           //Copy into structure to save for later
         }
         break;


      case GET_LINE_CODING:
         if (plc && (Len == sizeof(*plc))) {
             ust_cpy(plc, &g_lc);
         }
         break;


      case SET_CONTROL_LINE_STATE:
         // If a callback is provided, tell the upper driver of changes in DTR/RTS state
         if (plc && (Len == sizeof(uint16_t))) {
             if (ctrlLineStateCb) {
                 ctrlLineStateCb(ctrlLineStateCbContext, *((uint16_t *)Buf));
             }
         }
         break;

      case SEND_BREAK:
         /* Not  needed for this driver */
         break;

      default:
         break;
    }

    return USBD_OK;
}

/*******************************************************************************
 * Function Name  : Send DATA .
 * Description    : send the data received from the STM32 to the PC through USB
 * Input          : buffer to send, and the length of the buffer.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    VCP_DataTx(ptrBuffer, sendLength);
    return sendLength;
}

uint32_t CDC_Send_FreeBytes(void)
{
    return APP_RX_DATA_SIZE - CDC_Receive_BytesAvailable();
}

/**
 * @brief  VCP_DataTx
 *         CDC data to be sent to the Host (app) over USB
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
 */
static uint16_t VCP_DataTx(const uint8_t* Buf, uint32_t Len)
{
    /*
        make sure that any paragraph end frame is not in play
        could just check for: USB_CDC_ZLP, but better to be safe
        and wait for any existing transmission to complete.
    */
    while (USB_Tx_State != 0);

    for (uint32_t i = 0; i < Len; i++) {
        // Stall if the ring buffer is full
        while (((APP_Rx_ptr_in + 1) % APP_RX_DATA_SIZE) == APP_Rx_ptr_out) {
            delay(1);
        }

        APP_Rx_Buffer[APP_Rx_ptr_in] = Buf[i];
        APP_Rx_ptr_in = (APP_Rx_ptr_in + 1) % APP_RX_DATA_SIZE;
    }

    return USBD_OK;
}

/*******************************************************************************
 * Function Name  : Receive DATA .
 * Description    : receive the data from the PC to STM32 and send it through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len)
{
    uint32_t count = 0;

    while (APP_Tx_ptr_out != APP_Tx_ptr_in && (count < len)) {
        recvBuf[count] = APP_Tx_Buffer[APP_Tx_ptr_out];
        APP_Tx_ptr_out = (APP_Tx_ptr_out + 1) % APP_TX_DATA_SIZE;
        count++;
    }
    return count;
}

uint32_t CDC_Receive_BytesAvailable(void)
{
    /* return the bytes available in the receive circular buffer */
    return (APP_Tx_ptr_in + APP_TX_DATA_SIZE - APP_Tx_ptr_out) % APP_TX_DATA_SIZE;
}

/**
 * @brief  VCP_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         until exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len)
{
    if (CDC_Receive_BytesAvailable() + Len > APP_TX_DATA_SIZE) {
        return USBD_FAIL;
    }

    for (uint32_t i = 0; i < Len; i++) {
        APP_Tx_Buffer[APP_Tx_ptr_in] = Buf[i];
        APP_Tx_ptr_in = (APP_Tx_ptr_in + 1) % APP_TX_DATA_SIZE;
    }

    return USBD_OK;
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
    return (bDeviceState == CONFIGURED);
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
    return (bDeviceState != UNCONNECTED);
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
    return g_lc.bitrate;
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
