/**
 * @file        usbd_cdc_vcp.c
 *
 * @brief       usb device CDC class Virtual Com Port handler
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes ***************************************************************/
#include "usbd_cdc_vcp.h"

/* Private includes *******************************************************/
#include <stdbool.h>

#include "platform.h"

#include "build/atomic.h"

#include "drivers/nvic.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"

/* Private macro **********************************************************/
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define APP_TX_BLOCK_SIZE 512

/* Private variables ******************************************************/
volatile uint8_t cdcTxBuffer[APP_RX_DATA_SIZE];
volatile uint8_t cdcRxBuffer[APP_TX_DATA_SIZE];
uint32_t BuffLength;
volatile uint32_t cdcTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
volatile uint32_t cdcTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */

uint32_t rxAvailable = 0;
uint8_t* rxBuffPtr = NULL;

/* Private typedef ********************************************************/

static USBD_STA_T USBD_CDC_ItfInit(void);
static USBD_STA_T USBD_CDC_ItfDeInit(void);
static USBD_STA_T USBD_CDC_ItfCtrl(uint8_t command, uint8_t *buffer, uint16_t length);
static USBD_STA_T USBD_CDC_ItfSend(uint8_t *buffer, uint16_t length);
static USBD_STA_T USBD_CDC_ItfSendEnd(uint8_t epNum, uint8_t *buffer, uint32_t *length);
static USBD_STA_T USBD_CDC_ItfReceive(uint8_t *buffer, uint32_t *length);
static USBD_STA_T USBD_CDC_ItfSOF(void);


/* USB CDC interface handler */
USBD_CDC_INTERFACE_T USBD_CDC_INTERFACE =
{
    "CDC Interface",
    USBD_CDC_ItfInit,
    USBD_CDC_ItfDeInit,
    USBD_CDC_ItfCtrl,
    USBD_CDC_ItfSend,
    USBD_CDC_ItfSendEnd,
    USBD_CDC_ItfReceive,
    USBD_CDC_ItfSOF,
};

/* CDC Line Code Information */
USBD_CDC_LINE_CODING_T LineCoding =
{
    115200,     /*!< baud rate */
    0x00,       /*!< stop bits 1 */
    0x00,       /*!< parity none */
    0x08,       /*!< word length 8 bits */
};

/* Private function prototypes ********************************************/
static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;

/* External variables *****************************************************/
extern USBD_INFO_T gUsbDevice;

/* External functions *****************************************************/

/**
 * @brief       USB device initializes CDC media handler
 *
 * @param       None
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfInit(void)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_CDC_ConfigRxBuffer(&gUsbDevice, (uint8_t *)cdcRxBuffer);
    USBD_CDC_ConfigTxBuffer(&gUsbDevice, (uint8_t *)cdcTxBuffer, 0);

    ctrlLineStateCb = NULL;
    baudRateCb = NULL;

    return usbStatus;
}

/**
 * @brief       USB device deinitializes CDC media handler
 *
 * @param       None
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfDeInit(void)
{
    USBD_STA_T usbStatus = USBD_OK;

    return usbStatus;
}

/**
 * @brief       USB device CDC interface control request handler
 *
 * @param       command: Command code
 *
 * @param       buffer: Command data buffer
 *
 * @param       length: Command data length
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfCtrl(uint8_t command, uint8_t *buffer, uint16_t length)
{
    USBD_STA_T usbStatus = USBD_OK;
    LINE_CODING* plc = (LINE_CODING*)buffer;

    switch(command)
    {
        case USBD_CDC_SEND_ENCAPSULATED_COMMAND:
            break;

        case USBD_CDC_GET_ENCAPSULATED_RESPONSE:
            break;

        case USBD_CDC_SET_COMM_FEATURE:
            break;

        case USBD_CDC_GET_COMM_FEATURE:
            break;

        case USBD_CDC_CLEAR_COMM_FEATURE:
            break;

        /* Line Coding Data Structure 
        *  | Offset(Byte) | Field       | Length | Desc                 |
        *  | 0            | dwDTERate   | 4      | Data Terminal rate   |
        *  | 4            | bCharFormat | 1      | Stop bits            |
        *                                          (0 : 1 Stop bit)
        *                                          (1 : 1.5 Stop bits)
        *                                          (2 : 2 Stop bits)
        *  | 5            | bParityType | 1      | Parity               |
        *                                          (0 : None)
        *                                          (1 : Odd)
        *                                          (2 : Even)
        *                                          (3 : Mark)
        *                                          (4 : Space)
        *  | 6            | bDataBits   | 1      | Data bits            |
        *                                          (5 : 5 bits)
        *                                          (6 : 6 bits)
        *                                          (7 : 7 bits)
        *                                          (8 : 8 bits)
        *                                          (16 : 16 bits)
        */
        case USBD_CDC_SET_LINE_CODING:
            if (buffer && (length == sizeof(*plc))) {
                LineCoding.baudRate     = plc->bitrate;
                LineCoding.format       = plc->format;
                LineCoding.parityType   = plc->paritytype;
                LineCoding.WordLen      = plc->datatype;

                // If a callback is provided, tell the upper driver of changes in baud rate
                if (baudRateCb) {
                    baudRateCb(baudRateCbContext, LineCoding.baudRate);
                }
            }
        break;

        case USBD_CDC_GET_LINE_CODING:
            if (buffer && (length == sizeof(*plc))) {
                plc->bitrate = LineCoding.baudRate;
                plc->format = LineCoding.format;
                plc->paritytype = LineCoding.parityType;
                plc->datatype = LineCoding.WordLen;
            }
            break;

        case USBD_CDC_SET_CONTROL_LINE_STATE:
            // If a callback is provided, tell the upper driver of changes in DTR/RTS state
            if (buffer && (length == sizeof(uint16_t))) {
                if (ctrlLineStateCb) {
                    ctrlLineStateCb(ctrlLineStateCbContext, *((uint16_t *)buffer));
                }
            }
            break;

        case USBD_CDC_SEND_BREAK:
            break;

        default:
            break;
    }

    return usbStatus;
}

/**
 * @brief       USB device CDC interface send handler
 *
 * @param       buffer: Command data buffer
 *
 * @param       length: Command data length
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfSend(uint8_t *buffer, uint16_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_CDC_INFO_T *usbDevCDC = (USBD_CDC_INFO_T*)gUsbDevice.devClass[gUsbDevice.classID]->classData;
    
    if(usbDevCDC->cdcTx.state != USBD_CDC_XFER_IDLE)
    {
        return USBD_BUSY;
    }
    
    USBD_CDC_ConfigTxBuffer(&gUsbDevice, buffer, length);
    
    usbStatus = USBD_CDC_TxPacket(&gUsbDevice);
    
    return usbStatus;
}

/**
 * @brief       USB device CDC interface send end event handler
 *
 * @param       epNum: endpoint number
 *
 * @param       buffer: Command data buffer
 *
 * @param       length: Command data length
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfSendEnd(uint8_t epNum, uint8_t *buffer, uint32_t *length)
{
    USBD_STA_T usbStatus = USBD_OK;

    UNUSED(epNum);
    UNUSED(buffer);
    UNUSED(length);

    return usbStatus;
}

/**
 * @brief       USB device CDC interface receive handler
 *
 * @param       buffer: Command data buffer
 *
 * @param       length: Command data length
 *
 * @retval      USB device operation status
 */
static USBD_STA_T USBD_CDC_ItfReceive(uint8_t *buffer, uint32_t *length)
{
    USBD_STA_T usbStatus = USBD_OK;
    
    // USBD_CDC_ConfigRxBuffer(&gUsbDevice, &buffer[0]);
    rxAvailable = *length;
    rxBuffPtr = buffer;
    if (!rxAvailable) {
        // Received an empty packet, trigger receiving the next packet.
        // This will happen after a packet that's exactly 64 bytes is received.
        // The USB protocol requires that an empty (0 byte) packet immediately follow.
        USBD_CDC_RxPacket(&gUsbDevice);
    }
    
    return usbStatus;
}

static USBD_STA_T USBD_CDC_ItfSOF(void)
{
    static uint32_t FrameCount = 0;

    uint32_t buffsize;
    static uint32_t lastBuffsize = 0;

    USBD_CDC_INFO_T *usbDevCDC = (USBD_CDC_INFO_T*)gUsbDevice.devClass[gUsbDevice.classID]->classData;

    if (FrameCount++ == USBD_CDC_FS_INTERVAL)
    {
        FrameCount = 0;

        if (usbDevCDC->cdcTx.state == USBD_CDC_XFER_IDLE) {
            // endpoint has finished transmitting previous block
            if (lastBuffsize) {
                bool needZeroLengthPacket = lastBuffsize % 64 == 0;

                // move the ring buffer tail based on the previous succesful transmission
                cdcTxBufPtrOut += lastBuffsize;
                if (cdcTxBufPtrOut == APP_TX_DATA_SIZE) {
                    cdcTxBufPtrOut = 0;
                }
                lastBuffsize = 0;

                if (needZeroLengthPacket) {
                    USBD_CDC_ConfigTxBuffer(&gUsbDevice, (uint8_t*)&cdcTxBuffer[cdcTxBufPtrOut], 0);
                    return USBD_OK;
                }
            }
            if (cdcTxBufPtrOut != cdcTxBufPtrIn) {
                if (cdcTxBufPtrOut > cdcTxBufPtrIn) { /* Roll-back */
                    buffsize = APP_TX_DATA_SIZE - cdcTxBufPtrOut;
                } else {
                    buffsize = cdcTxBufPtrIn - cdcTxBufPtrOut;
                }
                if (buffsize > APP_TX_BLOCK_SIZE) {
                    buffsize = APP_TX_BLOCK_SIZE;
                }

                USBD_CDC_ConfigTxBuffer(&gUsbDevice, (uint8_t*)&cdcTxBuffer[cdcTxBufPtrOut], buffsize);

                if (USBD_CDC_TxPacket(&gUsbDevice) == USBD_OK) {
                    lastBuffsize = buffsize;
                }
            }
        }
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
    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0) {
            // block until there is free space in the ring buffer
            delay(1);
        }
        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) { // Paranoia
            cdcTxBuffer[cdcTxBufPtrIn] = ptrBuffer[i];
            cdcTxBufPtrIn = (cdcTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
        }
    }
    return sendLength;
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
    if ( (rxBuffPtr != NULL))
    {
        while ((rxAvailable > 0) && count < len)
        {
            recvBuf[count] = rxBuffPtr[0];
            rxBuffPtr++;
            rxAvailable--;
            count++;
            if (rxAvailable < 1)
                USBD_CDC_RxPacket(&gUsbDevice);
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
        freeBytes = ((cdcTxBufPtrOut - cdcTxBufPtrIn) + (-((int)(cdcTxBufPtrOut <= cdcTxBufPtrIn)) & APP_TX_DATA_SIZE)) - 1;
    }

    return freeBytes;
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
    return (gUsbDevice.devState == USBD_DEV_CONFIGURE);
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
    return (gUsbDevice.devState != USBD_DEV_DEFAULT);
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
    return LineCoding.baudRate;
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
