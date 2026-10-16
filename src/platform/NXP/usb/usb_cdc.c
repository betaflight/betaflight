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

// TinyUSB glue for the i.MXRT USB1 OTG core. TinyUSB buffers are unlocked, so
// tud_task() and the read and write calls must all run from the main loop.

// TinyUSB comes first: platform.h poisons sprintf and snprintf.
#include <stdbool.h>
#include <stdint.h>

#include "tusb.h"
#include "device/dcd.h"

#include "platform.h"

#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "platform/nxp_clock.h"

#include "usb_cdc.h"

// How long a write waits for the host to drain the endpoint: one host frame.
#define CDC_WRITE_TIMEOUT_US    1000

// Reference manual trim values for the USB PHY transmitter: the D+ and D- 45
// ohm terminations and the DC voltage level.
#define USBPHY_TX_D_CAL_TRIM        0x0C
#define USBPHY_TX_TXCAL45_TRIM      0x06

// How long D+ is held low at startup so the host sees a disconnect.
#define USB_DETACH_HOLD_MS          50

// The 11xx headers name the device controller USB_OTG1 and the 10xx ones USB1.
#if !defined(USB1) && defined(USB_OTG1)
#define USB1 USB_OTG1
#endif

static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;
static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;

// Host line coding, read back by the configurator and serial passthrough.
static uint32_t cdcBaudRate = 115200;

static bool inTask;

// Set when the host has stopped reading for longer than CDC_WRITE_TIMEOUT_US;
// the port then drops written bytes until the FIFO takes bytes again.
static bool txStalled;

FAST_IRQ_HANDLER void USB_OTG1_IRQHandler(void)
{
    tusb_int_handler(0, true);
}

void cdcUsbInit(void)
{
    nxpClockGateEnable(NXP_CLK_USBOH3);

    // A controller left running by an external reset keeps D+ pulled up; drop
    // it so the host enumerates this boot's identity.
    if (USB1->USBCMD & USB_USBCMD_RS_MASK) {
        USB1->USBCMD &= ~USB_USBCMD_RS_MASK;
        delay(USB_DETACH_HOLD_MS);
    }

    // Gives the PHY a locked 480 MHz and an analog supply.
    imxrtUsbPhyPllInit();

    // Release the PHY from reset before ungating its clock.
    USBPHY1->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;
    USBPHY1->CTRL_CLR = USBPHY_CTRL_CLKGATE_MASK;

    // Reset leaves every PHY analog block powered down.
    USBPHY1->PWD = 0;

    // ENUTMILEVEL2/3 are needed for high speed; the auto clear bits power the PHY up on resume.
    USBPHY1->CTRL_SET = USBPHY_CTRL_ENAUTOCLR_PHY_PWD_MASK
                      | USBPHY_CTRL_ENAUTOCLR_CLKGATE_MASK
                      | USBPHY_CTRL_ENUTMILEVEL2_MASK
                      | USBPHY_CTRL_ENUTMILEVEL3_MASK;

    USBPHY1->TX = (USBPHY1->TX & ~(USBPHY_TX_D_CAL_MASK
                                 | USBPHY_TX_TXCAL45DP_MASK
                                 | USBPHY_TX_TXCAL45DM_MASK))
                | USBPHY_TX_D_CAL(USBPHY_TX_D_CAL_TRIM)
                | USBPHY_TX_TXCAL45DP(USBPHY_TX_TXCAL45_TRIM)
                | USBPHY_TX_TXCAL45DM(USBPHY_TX_TXCAL45_TRIM);

    // Set the priority before the stack enables the interrupt for us.
    NVIC_SetPriority(USB_OTG1_IRQn, NVIC_PRIORITY_LEVEL(NVIC_PRIO_USB));

    tusb_init();
}

void cdcUsbDetach(void)
{
    if (!tud_inited()) {
        return;
    }

    tud_disconnect();

    USBPHY1->PWD = 0xffffffffu;
}

FAST_CODE void cdcUsbTask(void)
{
    // tud_task() is not reentrant and its callbacks can write back into here.
    if (inTask) {
        return;
    }

    inTask = true;
    tud_task();
    inTask = false;
}

FAST_CODE bool cdcUsbMounted(void)
{
    return tud_mounted();
}

FAST_CODE bool cdcUsbConnected(void)
{
    return tud_cdc_connected();
}

#ifdef USE_USB_DETECT
FAST_CODE void cdcUsbUnplugged(void)
{
    // The controller does not see VBUS leave, so tell the stack.
    if (tud_connected()) {
        dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
    }
}
#endif

FAST_CODE uint32_t cdcUsbBytesAvailable(void)
{
    cdcUsbTask();
    return tud_cdc_available();
}

FAST_CODE uint32_t cdcUsbTxBytesFree(void)
{
    // The transmit FIFO only empties from tud_task(), and callers spin on this.
    cdcUsbTask();

    if (txStalled || !tud_cdc_connected()) {
        return CFG_TUD_CDC_TX_BUFSIZE;
    }

    return tud_cdc_write_available();
}

FAST_CODE bool cdcUsbTxIdle(void)
{
    // Run the stack, then offer what the endpoint has not taken yet.
    cdcUsbTask();
    tud_cdc_write_flush();

    // A port nobody is reading counts as idle.
    if (txStalled || !tud_cdc_connected()) {
        return true;
    }

    return tud_cdc_write_available() == CFG_TUD_CDC_TX_BUFSIZE;
}

FAST_CODE uint32_t cdcUsbRead(uint8_t *buf, uint32_t length)
{
    cdcUsbTask();

    if (!tud_cdc_available()) {
        return 0;
    }

    return tud_cdc_read(buf, length);
}

FAST_CODE uint32_t cdcUsbWrite(const uint8_t *buf, uint32_t length)
{
    if (!tud_cdc_connected()) {
        txStalled = false;
        return 0;
    }

    timeUs_t startUs = micros();
    uint32_t written = 0;

    while (written < length) {
        const uint32_t accepted = tud_cdc_write(buf + written, length - written);

        cdcUsbTask();
        tud_cdc_write_flush();

        if (accepted > 0) {
            written += accepted;
            txStalled = false;
            startUs = micros();
            continue;
        }

        // FIFO full: wait one host frame, then latch the stall and return short.
        if (txStalled || !tud_cdc_connected() || cmpTimeUs(micros(), startUs) > CDC_WRITE_TIMEOUT_US) {
            txStalled = true;
            break;
        }
    }

    return written;
}

uint32_t cdcUsbBaudRate(void)
{
    return cdcBaudRate;
}

void cdcUsbSetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

void cdcUsbSetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}

// SET_LINE_CODING from the host, forwarded for serial passthrough.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *lineCoding)
{
    UNUSED(itf);

    cdcBaudRate = lineCoding->bit_rate;

    if (baudRateCb) {
        baudRateCb(baudRateCbContext, cdcBaudRate);
    }
}

// SET_CONTROL_LINE_STATE from the host: bit 0 is DTR, bit 1 is RTS.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    UNUSED(itf);

    if (ctrlLineStateCb) {
        ctrlLineStateCb(ctrlLineStateCbContext, (dtr ? 1 : 0) | (rts ? 2 : 0));
    }
}
