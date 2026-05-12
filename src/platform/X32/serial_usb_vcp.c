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
 * X32M7 USB VCP serial port abstraction layer.
 *
 * It implements the serialPortVTable interface for USB Virtual COM Port,
 * bridging the Betaflight serial framework to the CDC transport layer
 * (usbd_cdc_vcp.c).
 *
 * Architecture:
 *
 *   Betaflight serial framework (serialPort_t / serialPortVTable)
 *          |
 *          v
 *   serial_usb_vcp.c  (this file — vtable functions, init, open)
 *          |
 *          v
 *   usbd_cdc_vcp.c    (CDC_Send_DATA, CDC_Receive_DATA, usbIsConfigured...)
 *          |
 *          v
 *   usbd_cdc_core.c   (X32 CDC class driver — APP_Rx_Buffer, SOF, DataOut)
 *          |
 *          v
 *   X32 USB HS hardware (USB_dev, IRQ handlers in X32M7xx_it.c)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"

#include "pg/usb.h"

/* X32 USB library headers */
#include "usbd_cdc_vcp.h"
#include "usbd_cdc_core.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_user.h"

/*
 * USB core ID selection.
 *
 * X32M7 has two USB HS controllers (USB1_HS, USB2_HS).
 * Default to USB1 unless overridden by the target definition.
 */
#ifdef USE_USBHS1
#ifndef USB_CORE_ID
#define USB_CORE_ID USBHS1_CORE_ID
#endif
#endif /* USE_USBHS1 */

#ifdef USE_USBHS2
#ifndef USB_CORE_ID
#define USB_CORE_ID USBHS2_CORE_ID
#endif
#endif /* USE_USBHS2 */

/*
 * USB send timeout in milliseconds.
 *
 * usbVcpWriteBuf and usbVcpFlush will abort if sending takes longer
 * than this. Prevents infinite blocking when USB is disconnected
 * mid-transfer (CDC_Send_DATA busy-waits internally).
 */
#define USB_TIMEOUT  50

/* USB pin definitions, can be overridden by target */
#ifdef USE_USBHS1
#ifndef USB_DM_PIN
#define USB_DM_PIN PA11
#endif

#ifndef USB_DP_PIN
#define USB_DP_PIN PA12
#endif
#endif /* USE_USBHS1 */

#ifdef USE_USBHS2
#ifndef USB_DM_PIN
#define USB_DM_PIN PB14
#endif

#ifndef USB_DP_PIN
#define USB_DP_PIN PB15
#endif
#endif /* USE_USBHS2 */

static vcpPort_t vcpPort = {0};

/* ========================================================================== */
/*                    serialPortVTable Implementation                          */
/* ========================================================================== */

/**
 * @brief  Set the VCP baud rate (no-op for virtual port).
 *
 * USB VCP doesn't have a physical UART, so there's no hardware baud rate
 * to configure. This is a no-op placeholder to satisfy the vtable interface.
 * The actual baud rate negotiation happens via CDC SET_LINE_CODING in
 * usbd_cdc_vcp.c, which is host-driven.
 */
static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

/**
 * @brief  Set the VCP port mode (no-op for virtual port).
 *
 * USB VCP is always full-duplex, so MODE_RX / MODE_TX have no effect.
 */
static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

/**
 * @brief  Register a callback for DTR/RTS control line state changes.
 *
 * Delegates to CDC_SetCtrlLineStateCb in usbd_cdc_vcp.c, which stores
 * the callback and invokes it from VCP_Ctrl when the host sends
 * SET_CONTROL_LINE_STATE.
 *
 * @param  instance  Serial port instance (unused, only one VCP).
 * @param  cb        Callback function.
 * @param  context   Opaque context passed to callback.
 */
static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);

    CDC_SetCtrlLineStateCb((void (*)(void *context, uint16_t ctrlLineState))cb, context);
}

/**
 * @brief  Register a callback for baud rate changes.
 *
 * Delegates to CDC_SetBaudRateCb in usbd_cdc_vcp.c. The callback is
 * invoked when the host sends SET_LINE_CODING with a new baud rate.
 *
 * Note: The serialPort_t* context is cast to void* because the CDC layer
 * uses a generic void* context, while the serial framework uses serialPort_t*.
 *
 * @param  instance  Serial port instance (unused).
 * @param  cb        Callback function.
 * @param  context   Serial port passed as context to callback.
 */
static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);

    CDC_SetBaudRateCb((void (*)(void *context, uint32_t baud))cb, (void *)context);
}

/**
 * @brief  Check if the USB VCP transmit buffer is empty.
 *
 * Always returns true because USB VCP uses a write-through model:
 * data is pushed into APP_Rx_Buffer immediately by CDC_Send_DATA,
 * and the CDC library's SOF handler drains it asynchronously.
 * There is no intermediate "transmit holding register" to check.
 */
static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

/**
 * @brief  Get the number of bytes available to read from USB.
 *
 * Delegates to CDC_Receive_BytesAvailable() which checks the
 * usbRxBuffer ring buffer in usbd_cdc_vcp.c.
 *
 * Called by the serial framework via serialRxBytesWaiting().
 *
 * @return Number of unread bytes.
 */
static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    return CDC_Receive_BytesAvailable();
}

/**
 * @brief  Read a single byte from USB VCP (blocking).
 *
 * Spins until at least one byte is available, then returns it.
 * This is called by the serial framework's serialRead(), which
 * expects a blocking read of exactly one byte.
 *
 * The caller (MSP, CLI, etc.) typically checks usbVcpAvailable()
 * first, so the busy-wait rarely actually spins.
 *
 * @return The byte read.
 */
static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t buf[1];

    while (true) {
        if (CDC_Receive_DATA(buf, 1)) {
            return buf[0];
        }
    }
}

/**
 * @brief  Write a buffer of data to USB VCP.
 *
 * Sends data in a loop via CDC_Send_DATA(), which writes into
 * APP_Rx_Buffer for the CDC library's SOF handler to transmit.
 *
 * Checks usbIsConnected() && usbIsConfigured() first — if USB is
 * not ready, silently discards the data (no point blocking).
 *
 * Has a USB_TIMEOUT (50ms) safety net to prevent infinite blocking
 * if USB disconnects mid-transfer.
 *
 * @param  instance  Serial port (unused).
 * @param  data      Data buffer to send.
 * @param  count     Number of bytes.
 */
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

/**
 * @brief  Flush the small write-coalescing buffer to USB.
 *
 * vcpPort_t has a 20-byte txBuf used by usbVcpWrite() to coalesce
 * single-byte writes (e.g., printf character-by-character output).
 * This function sends the accumulated bytes via CDC_Send_DATA().
 *
 * Called when:
 *   - txBuf is full (20 bytes)
 *   - Buffering mode ends (usbVcpEndWrite)
 *   - A single write occurs outside buffering mode
 *
 * @param  port  VCP port with txBuf and txAt.
 * @return true if all bytes were sent, false on timeout or disconnect.
 */
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

/**
 * @brief  Write a single byte to USB VCP.
 *
 * Accumulates bytes in the 20-byte txBuf. If buffering mode is off
 * (normal mode), flushes immediately after each byte. If buffering
 * mode is on (between beginWrite/endWrite), flushes only when txBuf
 * is full.
 *
 * This coalescing avoids sending many tiny USB packets (each with
 * protocol overhead), improving throughput for character-at-a-time
 * callers like CLI printf.
 *
 * Uses container_of() to get vcpPort_t from serialPort_t — the C
 * equivalent of C++ static_cast for embedded inheritance.
 */
static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

/**
 * @brief  Enter bulk write mode — defer flushing until endWrite.
 *
 * Called by the serial framework before a batch of writes (e.g.,
 * sending an MSP response). Sets buffering=true so usbVcpWrite()
 * accumulates in txBuf instead of flushing after each byte.
 */
static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

/**
 * @brief  Get the number of free bytes in the USB TX buffer.
 *
 * Delegates to CDC_Send_FreeBytes() which checks free space in
 * APP_Rx_Buffer (the CDC library's TX ring buffer).
 *
 * Used by the serial framework to implement flow control — callers
 * can check if there's room before writing.
 */
static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

/**
 * @brief  Exit bulk write mode — flush accumulated data.
 *
 * Called by the serial framework after a batch of writes.
 * Clears buffering flag and flushes any remaining data in txBuf.
 */
static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

/**
 * USB VCP virtual function table.
 *
 * This is the "vtable" that connects the generic serialPort_t interface
 * to our USB-specific implementations. When the serial framework calls
 * serialWrite(port, c), it dispatches to port->vTable->serialWrite(),
 * which is usbVcpWrite.
 *
 * All 12 function pointers must be filled — this is the complete
 * serialPortVTable contract.
 */
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
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite,
    }
};

/* ========================================================================== */
/*                        Public API (called by main.c)                        */
/* ========================================================================== */

/**
 * @brief  Initialize the USB VCP subsystem.
 *
 * Called from initPhase1() during boot. Performs:
 *
 *   1. Register USB DM/DP pins with the IO subsystem (OWNER_USB).
 *   2. Generate a USB disconnect pulse — forces the host to re-enumerate
 *      after MCU reset (otherwise the host may think the old session is
 *      still active and not re-initialize the CDC driver).
 *   3. Initialize the X32 USB device library:
 *      - USB_dev:            USB device handle (defined in usbd_cdc_vcp.c)
 *      - USB_CORE_ID:        Which USB HS controller to use
 *      - USBD_VCP_desc:      Device descriptors (VID/PID/strings, from usbd_desc.c)
 *      - USBD_CDC_cb:        CDC class callbacks (from usbd_cdc_core.c)
 *      - USER_cb:            Device user callbacks (from usbd_user.c)
 */
void usbVcpInit(void)
{
    /* Select USB clock source */
    RCC_ConfigUSBRefClk(RCC_USBREFCLK_HSE_DIV1);
    /* Enable the PWR clock */
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_PWR, ENABLE);
#ifdef USE_USBHS1
    /* Enale USBHS1 clock */
    RCC_EnableAHB2PeriphClk1(RCC_AHB2_PERIPHEN_M7_USB1, ENABLE);
    PWR_MoudlePowerEnable(HSC1_USB1_PWRCTRL, ENABLE);
#endif

#ifdef USE_USBHS2
    /* Enale USBHS2 clock */
    RCC_EnableAHB1PeriphClk1(RCC_AHB1_PERIPHEN_M7_USB2, ENABLE);
    PWR_MoudlePowerEnable(HSC2_USB2_PWRCTRL, ENABLE);
#endif

    IOInit(IOGetByTag(IO_TAG(USB_DM_PIN)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(USB_DP_PIN)), OWNER_USB, 0);

    usbGenerateDisconnectPulse();

    memset(&USB_dev, 0, sizeof(USB_dev));
    USBD_Init(&USB_dev, USB_CORE_ID, &USBD_VCP_desc, &USBD_CDC_cb, &USER_cb);
}

/**
 * @brief  Open the USB VCP serial port.
 *
 * Called from initPhase3() after usbVcpInit(). Sets up the vcpPort_t
 * structure and assigns the vtable. Returns the serialPort_t* that the
 * serial framework uses for all subsequent I/O.
 *
 * There is only one VCP port (static vcpPort), so this always returns
 * the same pointer.
 *
 * @return Pointer to the VCP serial port.
 */
serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s = &vcpPort;
    s->port.vTable = usbVTable;
    return &s->port;
}

/**
 * @brief  Get the current baud rate negotiated with the host.
 *
 * Delegates to CDC_BaudRate() which returns the baud rate from the
 * most recent SET_LINE_CODING request.
 *
 * @return Baud rate in bps (default 115200 if host hasn't set it).
 */
uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);

    return CDC_BaudRate();
}

/**
 * @brief  Check if the USB VCP is physically connected to a host.
 *
 * Delegates to usbIsConnected() in usbd_cdc_vcp.c which checks
 * USB_dev.dev.connection_status.
 *
 * @return 1 if connected, 0 if disconnected.
 */
uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}

#endif /* USE_VCP */
