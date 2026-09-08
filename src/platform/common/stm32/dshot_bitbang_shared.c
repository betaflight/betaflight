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

#include "build/debug.h"

#include "drivers/time.h"

#include "dshot_bitbang_impl.h"

FAST_DATA_ZERO_INIT bbPacer_t bbPacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
FAST_DATA_ZERO_INIT int usedMotorPacers = 0;

FAST_DATA_ZERO_INIT bbPort_t bbPorts[MAX_SUPPORTED_MOTOR_PORTS];
FAST_DATA_ZERO_INIT int usedMotorPorts;

FAST_DATA_ZERO_INIT bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];

dshotBitbangStatus_e bbStatus;

// Derive how long a port may sit in BB_CAPTURE_IN_FLIGHT: the capture window plus
// a 25% margin. DShot600: ~62 us -> ~78 us, DShot300: ~124 us -> ~155 us.
void bbSetCaptureTimeout(bbPort_t *bbPort, uint32_t inputFreq)
{
    bbPort->captureTimeoutUs = (timeDelta_t)(DSHOT_BB_PORT_IP_BUF_LENGTH * 1000000 / inputFreq) * 5 / 4;
}

// Record, without blocking, where each port's ESC reply capture stands.
//
// The capture must not be cut short: the window is only a few microseconds longer
// than the reply (~62 us against ~58 us at DShot600), so stopping it early leaves
// bbSwitchToOutput() driving the line push-pull against a still transmitting ESC.
// That contention couples into the 3.3 V rail and has been seen to corrupt I2C
// barometers on AIO boards (#15533).
//
// Waiting for it here is not an option either. This runs in the PID task, and at
// loop rates where the round trip does not fit inside one cycle - DShot300 at
// 8 kHz needs ~182 us against a 125 us period - the spin cost tens of microseconds
// every cycle and starved TASK_RX (#15332).
//
// So do neither: a port still capturing keeps its capture, and skips this cycle's
// motor frame and telemetry decode instead. Both resume on the next cycle, and the
// ESC holds its last commanded value meanwhile - which is what it does between
// DShot frames anyway.
//
// A capture overdue past captureTimeoutUs is wedged rather than busy, so the port
// is handed back to bbUpdateComplete() to have its stream reinitialised; without
// that the group would never send another motor frame. Its buffer stays off limits
// to the decode either way. The deadline cannot fire on a live capture, since it is
// armed at the first update the capture overlaps and by then at most one window
// remains.
bool bbTelemetryWait(void)
{
    const timeUs_t currentTimeUs = micros();
    bool captureIncomplete = false;

    for (int i = 0; i < usedMotorPorts; i++) {
        bbPort_t *bbPort = &bbPorts[i];

        if (!bbPort->telemetryPending) {
            bbPort->captureState = BB_CAPTURE_COMPLETE;
            continue;
        }

        // Not signalled complete, so the buffer is not filled, either way below.
        captureIncomplete = true;

        if (bbPort->captureState != BB_CAPTURE_IN_FLIGHT) {
            // First update this capture has overlapped.
            bbPort->captureState = BB_CAPTURE_IN_FLIGHT;
            bbPort->captureDeadlineUs = currentTimeUs + bbPort->captureTimeoutUs;
        } else if (cmpTimeUs(currentTimeUs, bbPort->captureDeadlineUs) > 0) {
            // Overdue. Release the frame but keep telemetryPending: the IRQ owns
            // that flag, and leaving it set is what stops the next cycle reading
            // the torn buffer as if a fresh capture had landed in it. Once the
            // released frame goes out the IRQ drives the flag again as usual; if
            // it does not, this re-arms and retries.
            bbPort->captureState = BB_CAPTURE_STALLED;
        }
    }

    if (captureIncomplete) {
        // A nonzero count means the ESC round trip does not fit in the loop period,
        // so motor frames are being dropped. Lower the loop rate or raise the DShot
        // speed to clear it.
        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 2, debug[2] + 1);  //!< Reception Not Complete Count
    }

    return captureIncomplete;
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
