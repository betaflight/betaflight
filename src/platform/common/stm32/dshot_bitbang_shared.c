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

#include "drivers/dma.h"

#include "platform/dma.h"

#include "dshot_bitbang_impl.h"

FAST_DATA_ZERO_INIT bbPacer_t bbPacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
FAST_DATA_ZERO_INIT int usedMotorPacers = 0;

FAST_DATA_ZERO_INIT bbPort_t bbPorts[MAX_SUPPORTED_MOTOR_PORTS];
FAST_DATA_ZERO_INIT int usedMotorPorts;

FAST_DATA_ZERO_INIT bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];

dshotBitbangStatus_e bbStatus;

// Deal with a DMA transfer error, if that is what this IRQ was. Returns true when
// it was, in which case the caller must return without doing any direction work.
//
// The stream aborted partway, so its registers no longer describe a usable
// transfer. The caller has already stopped the stream and the pacer request, so
// clear the error and mark the port as an input: the next bbUpdateComplete() then
// runs bbSwitchToOutput(), which reloads the cached register set and reconfigures
// the pin. Spinning here instead, as this used to, took the flight controller down
// with it - there is no watchdog to recover an ISR that never returns, so one
// transfer error meant losing the craft.
//
// telemetryPending is left alone if a capture was running: the port then waits out
// its telemetry timeout before being handed back, by which point the ESC has
// certainly stopped replying and bbSwitchToOutput() cannot drive the line against
// it (#15533).
bool bbDMAHandleTransferError(bbPort_t *bbPort, dmaChannelDescriptor_t *descriptor)
{
    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        return false;
    }

#ifdef DEBUG_COUNT_INTERRUPT
    bbPort->errorIrq++;
#endif
    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    // Cleared one at a time: DMA_CLEAR_FLAG() does not parenthesise its flag
    // argument before shifting it, so an OR-ed mask shifts only the last term and
    // writes ones into other streams' bits.
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

    return true;
}

void bbDshotRequestTelemetry(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(bbMotors)) {
        return;
    }
    bbMotor_t *const bbmotor = &bbMotors[motorIndex];

    if (!bbmotor->configured) {
        return;
    }
    bbmotor->protocolControl.requestTelemetry = true;
}

bool bbDshotIsMotorIdle(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(bbMotors)) {
        return false;
    }

    bbMotor_t *const bbmotor = &bbMotors[motorIndex];
    return bbmotor->protocolControl.value == 0;
}

IO_t bbGetMotorIO(unsigned index)
{
    if (index >= dshotMotorCount) {
        return IO_NONE;
    }
    return bbMotors[index].io;
}

#ifdef USE_DSHOT_BITBANG
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
#if defined(STM32F4) || defined(APM32F4)
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO && motorDevConfig->useDshotTelemetry && motorDevConfig->motorProtocol != MOTOR_PROTOCOL_PROSHOT1000);
#else
    return motorDevConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorDevConfig->useDshotBitbang == DSHOT_BITBANG_AUTO && motorDevConfig->motorProtocol != MOTOR_PROTOCOL_PROSHOT1000);
#endif
}
#endif
