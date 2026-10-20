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

// DShot shared implementation: DMA buffer loading for DShot/ProShot
// packets and telemetry decoding from GPTMR edge-capture DMA streams.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "dma_hpmicro.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"

#include "pwm_output_dshot_shared.h"
#include "hpm_dma_drv.h"
#include "timer_hw_ext.h"
#include "hpm_dma_reqmap.h"
#include "trgm_dshot_resource.h"
FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount = 0;

FAST_DATA_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
FAST_DATA_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT uint32_t dshotTelemetryBitWidth[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT bool dshotFrameOutputEnabled;
static FAST_DATA_ZERO_INIT bool dshotFrameCommandActive;

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT uint32_t inputStampUs;

FAST_DATA_ZERO_INIT dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

motorDmaOutput_t *getMotorDmaOutput(unsigned index)
{
    if (index >= ARRAYLEN(dmaMotors)) {
        return NULL;
    }
    return &dmaMotors[index];
}

bool pwmDshotIsMotorIdle(unsigned index)
{
    const motorDmaOutput_t *motor = getMotorDmaOutput(index);
    return motor && motor->protocolControl.value == 0;
}

void pwmDshotRequestTelemetry(unsigned index)
{
    motorDmaOutput_t *const motor = getMotorDmaOutput(index);
    if (motor) {
        motor->protocolControl.requestTelemetry = true;
    }
}

uint8_t getTimerIndex(void *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    if (dmaMotorTimerCount >= MAX_DMA_TIMERS) {
        return 0xFF;            // overflow guard: caller must check
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}

/**
 * Prepare to send dshot data for one motor
 *
 * Formats the value into the appropriate dma buffer and enables the dma channel.
 * The packet won't start transmitting until later since the dma requests from the timer
 * are disabled when this function is called.
 *
 * @param index index of the motor that the data is to be sent to
 * @param value the dshot value to be sent
*/
FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    uint8_t bufferSize;
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    if (index == 0) {
        dshotFrameCommandActive = dshotCommandIsProcessing();
        dshotFrameOutputEnabled = dshotCommandQueueEmpty()
                                  || dshotCommandOutputIsEnabled(dshotMotorCount);
    }

    /*If there is a command ready to go overwrite the value and send that instead */
    if (dshotFrameCommandActive) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);

    bufferSize = loadDmaBuffer(motor, packet);

    motor->timer->timerDmaSources |= motor->timerDmaSource;

    // HPM starts DMA while staging each motor instead of at
    // pwmCompleteDshotMotorUpdate().  Suppress every motor in frames used
    // for command start/repeat/post delays.
    if (!dshotFrameOutputEnabled) {
        return;
    }

    pwmDshotStartTransfer(motor, bufferSize * 4);

}

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(unsigned motorCount)
{
    // pwmDshotSetDirectionOutput() is already called per-motor inside
    // pwmTelemetryDecode() — output channel re-enable is handled there.
    (void) motorCount;
}

static FAST_CODE uint32_t decodeTelemetryPacket(const uint32_t buffer[], uint32_t count, uint32_t bitWidth)
{
    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    // Level after the first (always falling) edge is 0; toggled on each edge.
    int level = 0;

    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;          // all bits consumed
            }
            len = (diff + bitWidth / 2) / bitWidth;
            if (len <= 0 || len > 21 - bits) {
                break;          // guard UB: shifts by <= 0 or >= 32 are undefined;
                                // a valid packet holds exactly 21 bits, so any
                                // run longer than the remainder is invalid
            }
            level ^= 1;
        } else {
            // Only pad trailing 1s when the final run is high (level==1).
            // level==0 means the pullup return-to-idle may have been captured
            // as a spurious extra edge; skip padding so the bits != 21 check
            // below rejects the packet (returns 0xffff / DSHOT_TELEMETRY_INVALID).
            len = 21 - bits;
            if (len <= 0 || level == 0) {
                break;          // done or spurious pullup edge
            }
        }
        value <<= len;
        value |= 1 << (len - 1);
        if (i < count) {
            oldValue = buffer[i];
        }
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0
    };

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8);  // xor bytes
    csum = csum ^ (csum >> 4);  // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return DSHOT_TELEMETRY_INVALID;
    }

    return decodedValue >> 4;
}

#endif


#ifdef USE_DSHOT_TELEMETRY
static FAST_CODE int combineEdgeData(uint32_t *pos, uint32_t *neg, uint32_t *buf, uint32_t bufSize,
                                     uint32_t posCnt, uint32_t negCnt)
{
    uint32_t posidx = 0;
    uint32_t negidx = 0;
    uint32_t idx = 0;
    // Skip all leading zeros in both buffers
    while (posidx < posCnt && pos[posidx] == 0) {
        posidx++;
    }
    while (negidx < negCnt && neg[negidx] == 0) {
        negidx++;
    }

    while (posidx < posCnt || negidx < negCnt) {
        if (idx >= bufSize) {
            break;              // output buffer full
        }
        // An exhausted side reads as 0 (no more edges) without touching memory past the end
        const uint32_t posVal = (posidx < posCnt) ? pos[posidx] : 0;
        const uint32_t negVal = (negidx < negCnt) ? neg[negidx] : 0;

        if (posVal == 0 && negVal == 0) {
            break;
        }
        if (posVal == 0) {
            buf[idx] = negVal;
            negidx++;
        } else if (negVal == 0) {
            buf[idx] = posVal;
            posidx++;
        } else if (posVal > negVal) {
            buf[idx] = negVal;
            negidx++;
        } else {
            buf[idx] = posVal;
            posidx++;
        }
        idx++;
    }
    return idx;
}
#endif

// Process dshot telemetry packets before switching the channels back to outputs
FAST_CODE_NOINLINE bool pwmTelemetryDecode(void)
{
#ifndef USE_DSHOT_TELEMETRY
    return true;
#else
    if (!useDshotTelemetry) {
        return true;
    }
#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif
    const timeUs_t currentUs = micros();

    for (int i = 0; i < dshotMotorCount; i++) {
        timeDelta_t usSinceInput = cmpTimeUs(currentUs, inputStampUs);
        if (usSinceInput >= 0 && usSinceInput < dmaMotors[i].dshotTelemetryDeadtimeUs) {
            return false;
        }

        if (dmaMotors[i].isInput) {
            const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(dmaMotors[i].timerHardware);
            int gptmrIndex = trgmDshotGptmrIndexByGptmr((hwExt) ? hwExt->gptmr : NULL);

            if (gptmrIndex < 0) {
                pwmDshotSetDirectionOutput(&dmaMotors[i]);
                continue;
            }
            const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
            const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);

            if (capNegSpec == NULL || capPosSpec == NULL) {
                pwmDshotSetDirectionOutput(&dmaMotors[i]);
                continue;
            }
            DMA_Type *dmaBaseNeg = capNegSpec->ref->base;
            DMA_Type *dmaBasePos = capPosSpec->ref->base;
            uint8_t chNeg = capNegSpec->ref->channel;
            uint8_t chPos = capPosSpec->ref->channel;

            // Freeze both edge streams before sampling their counts or buffers.
            // Otherwise a capture can update the count and data independently
            // while the CPU is combining the two streams.
            dma_disable_channel(dmaBaseNeg, chNeg);
            dma_disable_channel(dmaBasePos, chPos);
            __asm__ volatile ("fence iorw, iorw":::"memory");

            uint32_t edgesNeg = GCR_TELEMETRY_INPUT_LEN - dma_get_remaining_transfer_size(dmaBaseNeg, chNeg);
            uint32_t edgesPos = GCR_TELEMETRY_INPUT_LEN - dma_get_remaining_transfer_size(dmaBasePos, chPos);

            uint16_t rawValue;

            if (edgesNeg > MIN_GCR_EDGES) {
                dshotTelemetryState.readCount++;
                memset(dmaMotors[i].dmaBuffer, 0, DSHOT_DMA_BUFFER_ALLOC_SIZE * sizeof(uint32_t));
                uint32_t len = combineEdgeData(dmaMotors[i].dmaBuffer_pos_edge, dmaMotors[i].dmaBuffer_neg_edge,
                                               dmaMotors[i].dmaBuffer, DSHOT_DMA_BUFFER_ALLOC_SIZE, edgesPos,
                                               edgesNeg);
                rawValue = decodeTelemetryPacket(dmaMotors[i].dmaBuffer, len, dshotTelemetryBitWidth[i]);

                if (rawValue != DSHOT_TELEMETRY_INVALID) {
                    // Check EDT enable or store raw value
                    if (rawValue == 0x0E00 && dshotCommandGetCurrent(i) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE) {
                        dshotTelemetryState.motorState[i].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
                    } else {
                        dshotTelemetryState.motorState[i].rawValue = rawValue;
                    }
                } else {
                    dshotTelemetryState.invalidPacketCount++;
                    if (i == 0) {
                        memcpy(dshotTelemetryState.inputBuffer, dmaMotors[i].dmaBuffer,
                               sizeof(dshotTelemetryState.inputBuffer));
                    }
                }

#ifdef USE_DSHOT_TELEMETRY_STATS
                updateDshotTelemetryQuality(&dshotTelemetryQuality[i], rawValue != DSHOT_TELEMETRY_INVALID,
                                            currentTimeMs);
#endif
            }
        }
        pwmDshotSetDirectionOutput(&dmaMotors[i]);
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    inputStampUs = 0;
    dshotEnableChannels(dshotMotorCount);
    return true;
#endif                          // USE_DSHOT_TELEMETRY
}

#endif                          // USE_DSHOT
