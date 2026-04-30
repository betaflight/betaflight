/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * X32M7 USB CDC VCP (Virtual COM Port) interface layer.
 *
 * It sits between the upper serial_usb_vcp layer and the lower X32 CDC
 * class driver (usbd_cdc_core.c), providing:
 *
 *   1. VCP_fops callback table — registered as APP_FOPS in usbd_conf.h,
 *      called by the CDC class driver on USB events.
 *   2. Two ring buffers — TX path uses APP_Rx_Buffer (CDC library owned),
 *      RX path uses local usbRxBuffer.
 *   3. CDC public API — CDC_Send_DATA, CDC_Receive_DATA, etc.,
 *      called by serial_usb_vcp vtable functions.
 *   4. USB device instance — USB_dev, referenced by x32h7xx_it.c IRQ handlers.
 *
 * Data flow:
 *
 *   TX (MCU -> PC):
 *     usbVcpWrite() -> CDC_Send_DATA() -> APP_Rx_Buffer -> [SOF IRQ] -> USB IN EP -> PC
 *
 *   RX (PC -> MCU):
 *     PC -> USB OUT EP -> [OUT IRQ] -> VCP_DataRx() -> usbRxBuffer -> CDC_Receive_DATA() -> usbVcpRead()
 */

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "usbd_cdc_vcp.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#define USB_CDC_RX_BUFFER_SIZE  2048U

/*
 * USB device core instance.
 *
 * This is the central USB device handle for the X32M7 USB HS controller.
 * It holds all device state: configuration, endpoints, class/user callbacks.
 * Referenced externally by:
 *   - X32M7xx_it.c: IRQ handlers call USBD_ISTR_Handler(&USB_dev)
 *   - serial_usb_vcp.c: usbVcpInit() calls USBD_Init(&USB_dev, ...)
 *
 * __ALIGN_BEGIN / __ALIGN_END ensure 4-byte alignment required by USB DMA.
 */
#ifdef USB_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* IAR Compiler */
#pragma data_alignment=4
#endif
#endif
__ALIGN_BEGIN USB_CORE_MODULE USB_dev __ALIGN_END;

/*
 * TX path (MCU -> PC): APP_Rx_Buffer ring buffer
 *
 * Defined in usbd_cdc_core.c (CDC class driver).
 * Application writes data into APP_Rx_Buffer via APP_Rx_ptr_in,
 * CDC library reads and sends to host via USB IN endpoint using APP_Rx_ptr_out.
 * Named "Rx" from the host/APP perspective (host receives this data).
 */
extern uint8_t  APP_Rx_Buffer[APP_RX_DATA_SIZE];
extern volatile uint32_t APP_Rx_ptr_in;   /* Write pointer: application increments after writing */
extern volatile uint32_t APP_Rx_ptr_out;  /* Read pointer: CDC library increments after sending */

/*
 * CDC transmit state (defined in usbd_cdc_core.c).
 *
 * 0 = USB_CDC_IDLE: no IN transfer in flight, library ready to pick up new data
 * 1 = USB_CDC_BUSY: library has a packet submitted to USBDEV_EP_Tx
 * 2 = USB_CDC_ZLP : library needs to send a Zero-Length Packet next
 *
 * The underlying symbols are file-local #defines in usbd_cdc_core.c, so we
 * use the literal numeric value 0 here to mirror the STM32F4 VCP pattern.
 */
extern volatile uint8_t USB_Tx_State;

/*
 * RX path (PC -> MCU): usbRxBuffer ring buffer
 *
 * Defined locally. USB OUT interrupt delivers data via VCP_DataRx callback,
 * which writes into usbRxBuffer via usbRxHead.
 * Application reads from usbRxBuffer via usbRxTail through CDC_Receive_DATA.
 * Protected by ATOMIC_BLOCK against concurrent interrupt/main-loop access.
 */
static volatile uint8_t usbRxBuffer[USB_CDC_RX_BUFFER_SIZE];
static volatile uint32_t usbRxHead = 0;  /* Write pointer: USB OUT interrupt increments */
static volatile uint32_t usbRxTail = 0;  /* Read pointer: application increments after reading */

__IO uint32_t bDeviceConnectState = 0U; /* USB device status */

/*
 * USB CDC line coding structure.
 *
 * Mirrors the USB CDC LINE_CODING descriptor (7 bytes, packed).
 * The host sets this via SET_LINE_CODING request to indicate desired
 * serial parameters (baud rate, stop bits, parity, data bits).
 * Betaflight doesn't use a real UART behind VCP, but stores these values
 * so the host can query them back via GET_LINE_CODING, and so the upper
 * layer can detect baud rate changes (e.g., for MSP auto-detection).
 */
typedef struct __packed {
    uint32_t bitrate;
    uint8_t  format;
    uint8_t  parityType;
    uint8_t  dataBits;
} usbLineCoding_t;

static usbLineCoding_t lineCoding = {
    .bitrate    = 115200,
    .format     = 0,
    .parityType = 0,
    .dataBits   = 8,
};

/*
 * USB CDC control line state and callbacks.
 *
 * ctrlLineState:    DTR/RTS bit field set by host via SET_CONTROL_LINE_STATE.
 *                   Bit 0 = DTR, Bit 1 = RTS.
 * ctrlLineStateCb:  Upper layer callback, notified when DTR/RTS changes.
 *                   Used by Betaflight to detect host terminal open/close.
 * baudRateCb:       Upper layer callback, notified when host changes baud rate.
 *                   Used by Betaflight for MSP baud rate auto-detection.
 */
static uint16_t ctrlLineState;
static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineStateValue);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;


/* ========================================================================== */
/*                    Receive Ring Buffer Internal Operations                  */
/* ========================================================================== */

/**
 * @brief  Get the number of bytes available in the receive ring buffer.
 *
 * Uses a branchless bit trick to compute (head - tail) mod SIZE:
 *   - When head >= tail: (head - tail) + 0 = head - tail
 *   - When head <  tail: (head - tail) + SIZE = SIZE - tail + head
 *
 * The expression -(int)(head < tail) evaluates to:
 *   - 0x00000000 when head >= tail (condition false, -(int)0 = 0)
 *   - 0xFFFFFFFF when head <  tail (condition true,  -(int)1 = -1)
 * ANDing with SIZE selects SIZE or 0 respectively.
 *
 * Protected by ATOMIC_BLOCK because head is written by USB OUT interrupt
 * and tail is written by main loop — both must be read consistently.
 *
 * @return Number of bytes available to read.
 */
static uint32_t usbRxBytesAvailable(void)
{
    uint32_t available;

    ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
        available = ((usbRxHead - usbRxTail) + (-(int)(usbRxHead < usbRxTail) & USB_CDC_RX_BUFFER_SIZE));
    }

    return available;
}

/**
 * @brief  Read data from the receive ring buffer into user buffer.
 *
 * Reads up to 'len' bytes. Each byte is read inside an ATOMIC_BLOCK
 * to ensure the check (head != tail) and the subsequent read+advance
 * happen as one indivisible operation. Without this, a USB OUT interrupt
 * arriving between the check and the read could corrupt tail state.
 *
 * Returns immediately when the buffer is empty (non-blocking).
 *
 * @param  recvBuf  Destination buffer.
 * @param  len      Maximum number of bytes to read.
 * @return Actual number of bytes read (0 to len).
 */
static uint32_t usbRxRead(uint8_t *recvBuf, uint32_t len)
{
    uint32_t count = 0;

    while (count < len) {
        bool hasData;

        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
            hasData = (usbRxHead != usbRxTail);
            if (hasData) {
                recvBuf[count] = usbRxBuffer[usbRxTail];
                usbRxTail = (usbRxTail + 1U) % USB_CDC_RX_BUFFER_SIZE;
            }
        }

        if (!hasData) {
            break;
        }

        count++;
    }

    return count;
}

/**
 * @brief  Write data into the receive ring buffer (called from USB interrupt).
 *
 * Called by VCP_DataRx when the CDC library receives a USB OUT packet.
 * Runs in interrupt context, so no ATOMIC_BLOCK is needed here — the main
 * loop cannot preempt an interrupt. Only usbRxHead is modified here;
 * usbRxTail is only modified by the main loop (usbRxRead).
 *
 * If the buffer is full (nextHead == tail), remaining data is silently
 * discarded. This is a deliberate design choice — the alternative (blocking)
 * would stall the USB interrupt and cause USB protocol timeouts.
 *
 * @param  buf  Source data from USB OUT endpoint.
 * @param  len  Number of bytes received.
 */
static void usbRxWrite(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        const uint32_t nextHead = (usbRxHead + 1U) % USB_CDC_RX_BUFFER_SIZE;
        if (nextHead == usbRxTail) {
            break;
        }

        usbRxBuffer[usbRxHead] = buf[i];
        usbRxHead = nextHead;
    }
}

/* ========================================================================== */
/*              CDC Interface Callbacks (VCP_fops / APP_FOPS)                  */
/* ========================================================================== */

/**
 * @brief  VCP_Init — CDC class initialization callback.
 *
 * Called by the CDC class driver when the host completes SET_CONFIGURATION.
 * Resets the receive ring buffer and clears all control state.
 *
 * Callbacks (ctrlLineStateCb, baudRateCb) are set to NULL because a new
 * USB enumeration means the previous session's callbacks may reference
 * stale context. The upper layer (serial_usb_vcp) will re-register them
 * via CDC_SetBaudRateCb / CDC_SetCtrlLineStateCb if needed.
 *
 * @return USBD_OK always (initialization cannot fail for VCP).
 */
static uint16_t VCP_Init(void)
{
    usbRxHead = 0;
    usbRxTail = 0;
    ctrlLineState = 0;
    ctrlLineStateCb = NULL;
    baudRateCb = NULL;
    bDeviceConnectState = 1;

    return USBD_OK;
}

/**
 * @brief  VCP_DeInit — CDC class de-initialization callback.
 *
 * Called by the CDC class driver when the device is deconfigured
 * (e.g., USB reset, bus reset). Clears the receive buffer and
 * control line state, but does NOT clear callbacks — they belong
 * to the upper layer and will be cleaned up on next VCP_Init.
 *
 * @return USBD_OK always.
 */
static uint16_t VCP_DeInit(void)
{
    usbRxHead = 0;
    usbRxTail = 0;
    ctrlLineState = 0;
    bDeviceConnectState = 0;

    return USBD_OK;
}

/**
 * @brief  VCP_Ctrl — Handle CDC class-specific control requests.
 *
 * Called by the CDC class driver (usbd_cdc_core.c) when the host sends
 * a class-specific control request on the CDC command endpoint.
 *
 * Supported commands (USB CDC PSTN Subclass specification):
 *
 *   SET_LINE_CODING (0x20):
 *     Host tells device the desired serial parameters.
 *     Buf points to a 7-byte LINE_CODING structure.
 *     We copy each field individually (no memcpy) for clarity.
 *     If a baud rate callback is registered, notify the upper layer.
 *
 *   GET_LINE_CODING (0x21):
 *     Host queries current serial parameters.
 *     We copy our stored lineCoding back into the buffer.
 *
 *   SET_CONTROL_LINE_STATE (0x22):
 *     Host sets DTR/RTS signals (2 bytes: bit0=DTR, bit1=RTS).
 *     Used by terminals to signal open/close. Betaflight uses DTR
 *     changes to detect when a configurator connects/disconnects.
 *
 *   SEND_ENCAPSULATED_COMMAND / GET_ENCAPSULATED_RESPONSE:
 *     AT modem commands — not used by VCP, safely ignored.
 *
 *   SET/GET/CLEAR_COMM_FEATURE:
 *     Communication feature negotiation — not used, safely ignored.
 *
 *   SEND_BREAK (0x23):
 *     Serial break signal — not applicable to VCP, ignored.
 *
 * @param  cmd  CDC request code (see usbd_cdc_core.h defines).
 * @param  buf  Data buffer associated with the request.
 * @param  len  Length of data in buffer (bytes).
 * @return USBD_OK always (all requests are handled or safely ignored).
 */
static uint16_t VCP_Ctrl(uint32_t cmd, uint8_t *buf, uint32_t len)
{
    usbLineCoding_t *plc = (usbLineCoding_t *)buf;

    switch (cmd) {
    /* Not  needed for this driver, AT modem commands */
    case SEND_ENCAPSULATED_COMMAND:
    case GET_ENCAPSULATED_RESPONSE:
        break;

    /* Not needed for this driver */
    case SET_COMM_FEATURE:
    case GET_COMM_FEATURE:
    case CLEAR_COMM_FEATURE:
        break;

    case SET_LINE_CODING:
        if (plc && (len == sizeof(*plc))) {
            lineCoding.bitrate   = plc->bitrate;
            lineCoding.format    = plc->format;
            lineCoding.parityType = plc->parityType;
            lineCoding.dataBits  = plc->dataBits;
            if (baudRateCb) {
                baudRateCb(baudRateCbContext, lineCoding.bitrate);
            }
        }
        break;

    case GET_LINE_CODING:
        if (plc && (len == sizeof(*plc))) {
            plc->bitrate   = lineCoding.bitrate;
            plc->format    = lineCoding.format;
            plc->parityType = lineCoding.parityType;
            plc->dataBits  = lineCoding.dataBits;
        }
        break;

    case SET_CONTROL_LINE_STATE:
        if (buf && (len == sizeof(uint16_t))) {
            ctrlLineState = *((uint16_t *)buf);
            if (ctrlLineStateCb) {
                ctrlLineStateCb(ctrlLineStateCbContext, ctrlLineState);
            }
        }
        break;

    case SEND_BREAK:
    default:
        break;
    }

    return USBD_OK;
}

/**
 * @brief  VCP_DataTx — CDC SOF transmit notification callback.
 *
 * Called by the CDC class driver from the SOF (Start-of-Frame) interrupt
 * handler (Handle_USBAsynchXfer in usbd_cdc_core.c) every 15 frames.
 *
 * In the X32 CDC architecture, this callback has NO parameters. The CDC library
 * reads directly from APP_Rx_Buffer autonomously — the application writes
 * to APP_Rx_Buffer via CDC_Send_DATA(), and the library picks it up
 * during SOF processing without needing this callback to transfer data.
 *
 * Therefore this function is intentionally empty.
 *
 * @return USBD_OK always.
 */
static uint16_t VCP_DataTx(void)
{
    return USBD_OK;
}

/**
 * @brief  VCP_DataRx — CDC OUT endpoint receive callback.
 *
 * Called by the CDC class driver (USBD_CDC_DataOut in usbd_cdc_core.c)
 * when a complete USB OUT packet is received from the host.
 *
 * This function runs in USB interrupt context. It copies the received
 * data into the local usbRxBuffer ring buffer via usbRxWrite().
 * The upper layer later retrieves this data by calling CDC_Receive_DATA()
 * from the main loop.
 *
 * If the ring buffer is full, excess data is silently dropped rather
 * than blocking — blocking in interrupt context would freeze USB.
 *
 * @param  buf  Pointer to received data from USB OUT endpoint.
 * @param  len  Number of bytes received.
 * @return USBD_OK always.
 */
static uint16_t VCP_DataRx(uint8_t *buf, uint32_t len)
{
    if (buf && len) {
        usbRxWrite(buf, len);
    }

    return USBD_OK;
}

/*
 * VCP_fops — CDC interface callback table.
 *
 * Registered as APP_FOPS via the macro in usbd_conf.h:
 *   #define APP_FOPS  VCP_fops
 *
 * The CDC class driver (usbd_cdc_core.c) declares:
 *   extern CDC_IF_Prop_TypeDef APP_FOPS;
 * which the preprocessor expands to:
 *   extern CDC_IF_Prop_TypeDef VCP_fops;
 *
 * This is the C "vtable" pattern — the CDC driver calls these function
 * pointers without knowing the concrete VCP implementation.
 */
CDC_IF_Prop_TypeDef VCP_fops = {
    .pIf_Init   = VCP_Init,
    .pIf_DeInit = VCP_DeInit,
    .pIf_Ctrl   = VCP_Ctrl,
    .pIf_DataTx = VCP_DataTx,
    .pIf_DataRx = VCP_DataRx,
};

/* ========================================================================== */
/*                           CDC Public API                                    */
/* ========================================================================== */

/**
 * @brief  Get the number of free bytes in the TX (MCU->PC) ring buffer.
 *
 * Calculates free space in APP_Rx_Buffer by comparing the write pointer
 * (APP_Rx_ptr_in, advanced by application) with the read pointer
 * (APP_Rx_ptr_out, advanced by CDC library after USB IN transfer).
 *
 * Uses the same branchless bit trick as usbRxBytesAvailable():
 *   When out > in:  free = out - in - 1
 *   When out <= in: free = SIZE - in + out - 1
 *
 * The -1 reserves one slot to distinguish full from empty (standard
 * ring buffer convention: full when nextWrite == read).
 *
 * Protected by ATOMIC_BLOCK because APP_Rx_ptr_out is modified by the
 * CDC library in USB IN interrupt context, while APP_Rx_ptr_in is
 * modified by the main loop.
 *
 * Called by: serial_usb_vcp's usbTxBytesFree() and CDC_Send_DATA().
 *
 * @return Number of bytes that can be written before the buffer is full.
 */
uint32_t CDC_Send_FreeBytes(void)
{
    uint32_t freeBytes;

    ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
        freeBytes = ((APP_Rx_ptr_out - APP_Rx_ptr_in) + (-(int)(APP_Rx_ptr_out <= APP_Rx_ptr_in) & APP_RX_DATA_SIZE)) - 1U;
    }

    return freeBytes;
}

/**
 * @brief  Send data from MCU to PC via USB CDC IN endpoint.
 *
 * Writes data byte-by-byte into the CDC library's APP_Rx_Buffer.
 * The CDC library's SOF interrupt handler (every 15 frames, ~15ms)
 * will automatically read from APP_Rx_Buffer and transmit via USB IN.
 *
 * Each byte write is protected by ATOMIC_BLOCK to prevent the CDC
 * library's SOF handler from reading a partially-updated pointer.
 *
 * If the buffer is full, busy-waits with delay(1) until space becomes
 * available (the CDC library will free space as it sends data).
 * The upper layer (usbVcpWriteBuf) has a 50ms timeout to prevent
 * infinite blocking if USB is disconnected.
 *
 * Called by: serial_usb_vcp's usbVcpWrite() and usbVcpWriteBuf().
 *
 * @param  ptrBuffer   Data to send.
 * @param  sendLength  Number of bytes to send.
 * @return sendLength (always succeeds, may block until buffer has space).
 */
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    /*
     * Wait for any paragraph-end frame (ZLP) or in-flight IN transfer
     * to complete before queuing the next batch.
     *
     * Break conditions (prevent deadlock when host is gone):
     *   - Device disconnected   (bDeviceConnectState == 0)
     *   - Device not configured (device_status != USB_CONFIGURED)
     *   The upper layer (serial_usb_vcp) additionally enforces a 50 ms
     *   timeout around CDC_Send_DATA, so a bounded wait here is safe.
     */
    while (USB_Tx_State != 0U) {
        if (!usbIsConnected() || !usbIsConfigured()) {
            return 0;
        }
        delay(1);
    }

    for (uint32_t i = 0; i < sendLength; i++) {
        while (CDC_Send_FreeBytes() == 0U) {
            if (!usbIsConnected() || !usbIsConfigured()) {
                return i;
            }
            delay(1);
        }

        ATOMIC_BLOCK(NVIC_BUILD_PRIORITY(6, 0)) {
            APP_Rx_Buffer[APP_Rx_ptr_in] = ptrBuffer[i];
            APP_Rx_ptr_in = (APP_Rx_ptr_in + 1U) % APP_RX_DATA_SIZE;
        }
    }

    return sendLength;
}

/**
 * @brief  Read received data from the RX ring buffer (PC -> MCU direction).
 *
 * Delegates to usbRxRead() which reads from the local usbRxBuffer.
 * Non-blocking: returns immediately with whatever data is available.
 *
 * Called by: serial_usb_vcp's usbVcpRead() (reads 1 byte in a loop)
 *           and usbVcpAvailable() indirectly via CDC_Receive_BytesAvailable().
 *
 * @param  recvBuf  Destination buffer.
 * @param  len      Maximum number of bytes to read.
 * @return Actual number of bytes read (0 if buffer is empty).
 */
uint32_t CDC_Receive_DATA(uint8_t *recvBuf, uint32_t len)
{
    return usbRxRead(recvBuf, len);
}

/**
 * @brief  Get the number of bytes available in the RX ring buffer.
 *
 * Delegates to usbRxBytesAvailable().
 *
 * Called by: serial_usb_vcp's usbVcpAvailable() to report how many
 *           bytes the application can read without blocking.
 *
 * @return Number of unread bytes in the receive buffer.
 */
uint32_t CDC_Receive_BytesAvailable(void)
{
    return usbRxBytesAvailable();
}

/**
 * @brief  Check if the USB device is in CONFIGURED state.
 *
 * Queries the X32 USB library's internal device_status field directly.
 * USB_CONFIGURED (value 3) is set by the library when the host completes
 * SET_CONFIGURATION successfully.
 *
 * X32 reads the library's authoritative state.
 *
 * Called by: serial_usb_vcp's usbVcpWriteBuf() and usbVcpFlush()
 *           to skip sending if USB is not ready.
 *
 * @return 1 if configured, 0 otherwise.
 */
uint8_t usbIsConfigured(void)
{
    return (USB_dev.dev.device_status == USB_CONFIGURED);
}

/**
 * @brief  Check if the USB device is physically connected to a host.
 *
 * Called by: serial_usb_vcp's usbVcpWriteBuf() and usbVcpFlush()
 *           — both check usbIsConnected() && usbIsConfigured()
 *           before attempting to send data.
 *
 * @return 1 if connected, 0 if disconnected.
 */
uint8_t usbIsConnected(void)
{
    return (bDeviceConnectState != 0U);
}

/**
 * @brief  Get the current baud rate set by the host.
 *
 * Returns the baud rate from the most recent SET_LINE_CODING request.
 * Default is 115200 if the host hasn't sent SET_LINE_CODING yet.
 *
 * Called by: serial_usb_vcp's usbVcpGetBaudRate().
 *
 * @return Baud rate in bps.
 */
uint32_t CDC_BaudRate(void)
{
    return lineCoding.bitrate;
}

/**
 * @brief  Register a callback for baud rate changes.
 *
 * The callback is invoked from VCP_Ctrl when the host sends
 * SET_LINE_CODING with a new baud rate. Runs in USB interrupt context.
 *
 * Called by: serial_usb_vcp's usbVcpSetBaudRateCb() during port setup.
 *
 * @param  cb       Callback function, or NULL to unregister.
 * @param  context  Opaque pointer passed to callback (typically serialPort_t*).
 */
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

/**
 * @brief  Register a callback for control line state (DTR/RTS) changes.
 *
 * The callback is invoked from VCP_Ctrl when the host sends
 * SET_CONTROL_LINE_STATE. Bit 0 = DTR, Bit 1 = RTS.
 * Runs in USB interrupt context.
 *
 * Betaflight uses DTR transitions to detect when a configurator
 * application opens or closes the virtual COM port.
 *
 * Called by: serial_usb_vcp's usbVcpSetCtrlLineStateCb() during port setup.
 *
 * @param  cb       Callback function, or NULL to unregister.
 * @param  context  Opaque pointer passed to callback.
 */
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineStateValue), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}
