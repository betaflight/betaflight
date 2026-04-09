/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/usb_io.h"

#include "pg/usb.h"

#include "ch32_debug.h"
#include "cdc_vcp_ch32h41x.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usb_ch32h41x_usbhs_reg.h"

#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/nvic.h"

#define USB_TIMEOUT  50

static vcpPort_t vcpPort = {0};

// #define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define APP_TX_BLOCK_SIZE 512

// volatile uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;

/* Increment this pointer or roll it back to start address when data are received over USART */
volatile uint32_t UserTxBufPtrIn = 0;
/* Increment this pointer or roll it back to start address when data are sent over USB */
volatile uint32_t UserTxBufPtrOut = 0;

// volatile uint32_t APP_Rx_ptr_out = 0;
// volatile uint32_t APP_Rx_ptr_in = 0;
// static uint8_t APP_Rx_Buffer[APP_RX_DATA_SIZE];


// #define  CDC_POLLING_INTERVAL 5



static void (*ctrlLineStateCb)(void* context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;

void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}



uint32_t CDC_Send_FreeBytes(void)
{
    uint32_t freeBytes;

    ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
        freeBytes = ((UserTxBufPtrOut - UserTxBufPtrIn) + (-((int)(UserTxBufPtrOut <= UserTxBufPtrIn)) & APP_TX_DATA_SIZE)) - 1;
    }

    return freeBytes;
}

uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0) {
            delay(1);
        }
        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
            UserTxBuffer[UserTxBufPtrIn] = ptrBuffer[i];
            UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_TX_DATA_SIZE;
        }
    }
    return sendLength;
}


#if 0
static void TxTimerConfig(void)
{
    TIM9_12_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_TIM12, ENABLE );
    
    TIM_TimeBaseInitStructure.TIM_Period = CDC_POLLING_INTERVAL - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = HCLKClock / 4 / 1000  - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision =  TIM_CKD_DIV4; //TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM9_12_TimeBaseInit( TIM12, &TIM_TimeBaseInitStructure);
    
    TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM12, TIM_FLAG_Update );

    NVIC_SetPriority(TIM12_IRQn, NVIC_PRIO_USB);
	NVIC_EnableIRQ(TIM12_IRQn);
    TIM_Cmd( TIM12, ENABLE );
}
static volatile uint32_t lastBuffsize = 0;
__attribute__((interrupt("WCH-Interrupt-fast")))
void TIM12_IRQHandler(void)
{
    uint32_t buffsize;
    
    if (ep_tx_busy_flag == 0) //pre finish
    {
        if (lastBuffsize) 
        {
            bool needZeroLengthPacket = lastBuffsize % 64 == 0;
            UserTxBufPtrOut += lastBuffsize;
            if (UserTxBufPtrOut == APP_TX_DATA_SIZE) 
            {
                UserTxBufPtrOut = 0;
            }
            lastBuffsize = 0;
            if (needZeroLengthPacket) 
            {
                usb_vcp_send_data(0, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], 0);
                return;
            }
        }
        if (UserTxBufPtrOut != UserTxBufPtrIn) 
        {
            if (UserTxBufPtrOut > UserTxBufPtrIn) {
                buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
            } else {
                buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
            }
            if (buffsize > APP_TX_BLOCK_SIZE) {
                buffsize = APP_TX_BLOCK_SIZE;
            }
            uint32_t txed = usb_vcp_send_data(0,(uint8_t*)&UserTxBuffer[UserTxBufPtrOut], buffsize);
            if (txed == 0) 
            {
                lastBuffsize = buffsize;
            }
        }
    }
   TIM12->INTFR = (uint16_t)~TIM_IT_Update;
}
#else
    static volatile uint32_t lastBuffsize = 0;
    void usb_int_rxsof_handler(void) //1ms
    {
        static uint8_t FrameCount = 0;

        if(FrameCount++ == 16) //
        {
            FrameCount = 0;

            uint32_t buffsize;
            if (ep_tx_busy_flag == 0) //pre finish
            {
                if (lastBuffsize) 
                {
                    bool needZeroLengthPacket = lastBuffsize % 64 == 0;
                    UserTxBufPtrOut += lastBuffsize;
                    if (UserTxBufPtrOut == APP_TX_DATA_SIZE) 
                    {
                        UserTxBufPtrOut = 0;
                    }
                    lastBuffsize = 0;
                    if (needZeroLengthPacket) 
                    {
                        usb_vcp_send_data(0, (uint8_t*)&UserTxBuffer[UserTxBufPtrOut], 0);
                        return;
                    }
                }
                if (UserTxBufPtrOut != UserTxBufPtrIn) 
                {
                    if (UserTxBufPtrOut > UserTxBufPtrIn) {
                        buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
                    } else {
                        buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
                    }
                    if (buffsize > APP_TX_BLOCK_SIZE) {
                        buffsize = APP_TX_BLOCK_SIZE;
                    }
                    uint32_t txed = usb_vcp_send_data(0,(uint8_t*)&UserTxBuffer[UserTxBufPtrOut], buffsize);
                    if (txed == 0) 
                    {
                        lastBuffsize = buffsize;
                    }
                }
            }
        }
    }
#endif

uint8_t usbIsConnected(void)
{
    return ((usbd_cdc_info != USBD_EVENT_DISCONNECTED) && (usbd_cdc_info != USBD_EVENT_UNKNOWN));
}

uint8_t usbIsConfigured(void)
{
    return (usbd_cdc_info == USBD_EVENT_CONFIGURED);
}

uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}



static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    CDC_SetCtrlLineStateCb((void (*)(void *context, uint16_t ctrlLineState))cb, context);
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    CDC_SetBaudRateCb((void (*)(void *context, uint32_t baud))cb, (void *)context);
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

//     uint32_t available = APP_Rx_ptr_in-APP_Rx_ptr_out;

//     // Return the sum of the bytes in the APP_Rx_Buffer buffer and those received by the VCP driver
//   if(ep_rx_finish == 1)
//     {
//         available += ep_rx_length;
//     }

//     return available;

    return usb_vcp_rx_available( ); 
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

//    if ((APP_Rx_ptr_in == 0) || (APP_Rx_ptr_out == APP_Rx_ptr_in))
//    {
//         APP_Rx_ptr_out = 0;
//         APP_Rx_ptr_in = usb_vcp_get_rx_data(0, APP_Rx_Buffer);
//         if(APP_Rx_ptr_in == 0) {
//             // We've drained the input buffer
//             return 0;
//         }
//     }
//     return APP_Rx_Buffer[APP_Rx_ptr_out++];

    uint8_t buf[1];
    while(true)
    {
            if (usb_vcp_get_rx_data(0, buf, 1))
            return buf[0];
    }

}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    uint32_t start = millis();
    const uint8_t *p = data;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

static bool usbVcpFlush(vcpPort_t *port)
{
    uint32_t count = port->txAt;
    port->txAt = 0;

    if (count == 0) {
        return true;
    }

    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }

    uint32_t start = millis();
    uint8_t *p = port->txBuf;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
    return count == 0;
}
static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .setCtrlLineStateCb = usbVcpSetCtrlLineStateCb,
        .setBaudRateCb = usbVcpSetBaudRateCb,
        .writeBuf =  usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

void usbVcpInit(void)
{

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    
    IOInit(IOGetByTag(IO_TAG(PB8)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PB9)), OWNER_USB, 0);

    usbGenerateDisconnectPulse();

	if((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
	{
		/* Initialize USBHS 480M PLL */
		RCC_USBHS_PLLCmd(DISABLE);
		RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
		RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
		RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
		RCC_USBHS_PLLCmd(ENABLE);
	}
	/* Enable UTMI Clock */
	RCC_UTMIcmd(ENABLE);
	/* Enable USBHS Clock */
	RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);

    usb_rxsof_handler = usb_int_rxsof_handler;
    cdc_acm_init( 0, 0);   

    NVIC_SetPriority(USBHS_IRQn, NVIC_PRIO_USB);
    NVIC_EnableIRQ(USBHS_IRQn);

    // TxTimerConfig();

    // vcpPort_t *s = &vcpPort;
    // s->port.vTable = usbVTable;
    // return &s->port;
}


serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s = &vcpPort;
    s->port.vTable = usbVTable;
    return &s->port;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    return cdc_vcp_line_coding.dwDTERate;
}

#endif
