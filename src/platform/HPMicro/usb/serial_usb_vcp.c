/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// USB VCP serial port: implements the Betaflight serialPort interface
// over the CDC ACM device, including buffered TX with flush.

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"
#include "usb_config.h"
#ifdef USE_VCP

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "nvic_hpmicro.h"

#include "pg/usb.h"

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include "drivers/time.h"

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/usb_io.h"
#include "hpm_usb_drv.h"

extern uint32_t bf_usbd_ep_write_buffer(const void *data, int count);
extern void bf_usbd_ep_start_write(void);
extern bool bfIsTxBusy(void);
extern bool bfIsTxEmpty(void);
extern uint32_t bf_usbd_ep_tx_free(void);
extern uint32_t bf_usbd_ep_rx_available(void);
extern uint8_t bf_usbd_ep_read(void);
extern uint8_t usbIsConnected(void);
extern uint8_t usbIsConfigured(void);
extern uint32_t bf_usbd_cdc_baud_rate(void);
extern void bf_usbd_cdc_set_baud_rate_cb(void (*cb)(void *context, uint32_t baud), void *context);
extern void bf_usbd_cdc_set_ctrl_line_state_cb(void (*cb)(void *context, uint16_t state), void *context);
#define USB_TIMEOUT  50

static vcpPort_t vcpPort;
static void (*usbVcpBaudRateCallback)(serialPort_t *context, uint32_t baud);
static serialPort_t *usbVcpBaudRateContext;


bool usbCableIsInserted(void)
{
    bool result = usb_get_port_ccs((USB_Type *)CONFIG_HPM_USBD_BASE);

    return result;
}

static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState),
                                     void *context)
{
    UNUSED(instance);

    // Register upper driver control line state callback routine with USB driver
    bf_usbd_cdc_set_ctrl_line_state_cb(cb, context);
}

static void usbVcpBaudRateCb(void *context, uint32_t baud)
{
    UNUSED(context);
    if (usbVcpBaudRateCallback) {
        usbVcpBaudRateCallback(usbVcpBaudRateContext, baud);
    }
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud),
                                serialPort_t *context)
{
    UNUSED(instance);

    usbVcpBaudRateCallback = cb;
    usbVcpBaudRateContext = context;
    bf_usbd_cdc_set_baud_rate_cb(usbVcpBaudRateCb, NULL);
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return bfIsTxEmpty();
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    return bf_usbd_ep_rx_available();
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    return bf_usbd_ep_read();
}

static bool usbVcpFlushWithTimeout(vcpPort_t *port, uint32_t start)
{
    (void) port;

    if (!(usbIsConnected() && usbIsConfigured()) || millis() - start > USB_TIMEOUT) {
        return false;
    }

    bf_usbd_ep_start_write();
    while (bfIsTxBusy()) {
        if (millis() - start > USB_TIMEOUT) {
            return false;
        }
    }
    return true;
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    uint32_t start = millis();
    const uint8_t *p = data;

    while (count > 0) {
        const uint32_t written = bf_usbd_ep_write_buffer(p, count);
        count -= written;
        p += written;

        if (!port->buffering) {
            bf_usbd_ep_start_write();
        }

        if (count > 0 && !usbVcpFlushWithTimeout(port, start)) {
            break;
        }
    }
}

static bool usbVcpFlush(vcpPort_t *port)
{
    return usbVcpFlushWithTimeout(port, millis());
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    usbVcpWriteBuf(instance, &c, 1);
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return bf_usbd_ep_tx_free();
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
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

extern void cdc_acm_init(uint8_t busid, uint32_t reg_base);

void usbVcpInit(void)
{
    extern void hpm_usb_init(USB_Type *ptr);
    hpm_usb_init((USB_Type *) CONFIG_HPM_USBD_BASE);
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, hpmPlicPriorityFromNvic(NVIC_PRIO_USB));
    cdc_acm_init(0, CONFIG_HPM_USBD_BASE);
}

serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s = &vcpPort;

    s->port.vTable = usbVTable;

    return (serialPort_t *) s;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);

    return bf_usbd_cdc_baud_rate();
}

uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}
#endif
