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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if ENABLE_VTX_UART_PIN_SWITCH

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_uart_pin_switch.h"

#include "io/serial.h"
#include "io/vtx.h"

#include "platform/vtx_uart_pin_switch.h"

typedef struct vtxUartPinSwitchState_s {
    serialPort_t *port;
    IO_t txIO;
    IO_t rxIO;
    timeMs_t startedMs;
    uint16_t rxDelayMs;
    uint8_t af;
} vtxUartPinSwitchState_t;

static vtxUartPinSwitchState_t pinSwitch;

void vtxUartPinSwitchInit(serialPort_t *port, portOptions_e options, IO_t txIO, IO_t rxIO, uint8_t af)
{
    if (!port || port->identifier != SERIAL_PORT_UART1) {
        return;
    }

    // Reopening UART1 for another function must cancel its previous binding.
    pinSwitch = (vtxUartPinSwitchState_t) { 0 };
    const vtxSettingsConfig_t *settings = vtxSettingsConfig();
    if (!(options & SERIAL_BIDIR) || !txIO || settings->vtx_uart != SERIAL_PORT_UART1) {
        return;
    }

    bool supportedProtocol = false;
#ifdef USE_VTX_SMARTAUDIO
    supportedProtocol |= settings->vtx_type == VTXDEV_SMARTAUDIO;
#endif
#ifdef USE_VTX_TRAMP
    supportedProtocol |= settings->vtx_type == VTXDEV_TRAMP;
#endif
    if (!supportedProtocol) {
        return;
    }

    IO_t switchRxIO = IOGetByTag(IO_TAG(SA_UART_RX_PIN));
    if (!switchRxIO || switchRxIO == txIO) {
        return;
    }

    pinSwitch.port = port;
    pinSwitch.txIO = txIO;
    pinSwitch.rxIO = switchRxIO;
    pinSwitch.af = af;

    IOConfigGPIO(txIO, IOCFG_IN_FLOATING);
    if (rxIO) {
        IOConfigGPIO(rxIO, IOCFG_IN_FLOATING);
    }
    IOConfigGPIO(switchRxIO, IOCFG_IN_FLOATING);
}

void vtxUartPinSwitchBegin(serialPort_t *port, timeMs_t nowMs, uint16_t rxDelayMs)
{
    if (!port || port != pinSwitch.port) {
        return;
    }

    IOConfigGPIOAF(pinSwitch.txIO, IOCFG_AF_PP, pinSwitch.af);
    IOConfigGPIO(pinSwitch.rxIO, IOCFG_IN_FLOATING);
    pinSwitch.startedMs = nowMs;
    pinSwitch.rxDelayMs = rxDelayMs;
}

void vtxUartPinSwitchUpdate(void)
{
    // Preserve the existing zero-timestamp convention and one-shot >= timeout.
    if (pinSwitch.startedMs && (millis() - pinSwitch.startedMs >= pinSwitch.rxDelayMs)) {
        IOConfigGPIO(pinSwitch.txIO, IOCFG_IN_FLOATING);
        IOConfigGPIOAF(pinSwitch.rxIO, IOCFG_AF_PP, pinSwitch.af);
        pinSwitch.startedMs = 0;
    }
}

#endif // ENABLE_VTX_UART_PIN_SWITCH
