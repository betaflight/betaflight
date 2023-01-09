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


#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#if defined(STM32F4)
#include "stm32f4xx.h"
#endif

#include "pwm_output.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"

#include "pwm_output_dshot_shared.h"

FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
#ifdef STM32F7
FAST_DATA_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
FAST_DATA_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#else
motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#endif

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT uint32_t inputStampUs;

FAST_DATA_ZERO_INIT dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}


FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    uint8_t bufferSize;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        motor->timer->dmaBurstLength = bufferSize * 4;
    } else
#endif
    {
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
#ifdef USE_FULL_LL_DRIVER
        xLL_EX_DMA_SetDataLength(motor->dmaRef, bufferSize);
        xLL_EX_DMA_EnableResource(motor->dmaRef);
#else
        xDMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
        xDMA_Cmd(motor->dmaRef, ENABLE);
#endif
    }
}


#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount);


static uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count)
{
    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;
            }
            len = (diff + 8) / 16;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return DSHOT_TELEMETRY_INVALID;
    }

    return decodedValue >> 4;
}

#endif

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE_NOINLINE bool pwmStartDshotMotorUpdate(void)
{
    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif
    const timeUs_t currentUs = micros();

    for (int i = 0; i < dshotPwmDevice.count; i++) {
        timeDelta_t usSinceInput = cmpTimeUs(currentUs, inputStampUs);
        if (usSinceInput >= 0 && usSinceInput < dmaMotors[i].dshotTelemetryDeadtimeUs) {
            return false;
        }
        if (dmaMotors[i].isInput) {
#ifdef USE_FULL_LL_DRIVER
            uint32_t edges = GCR_TELEMETRY_INPUT_LEN - xLL_EX_DMA_GetDataLength(dmaMotors[i].dmaRef);
#else
            uint32_t edges = GCR_TELEMETRY_INPUT_LEN - xDMA_GetCurrDataCounter(dmaMotors[i].dmaRef);
#endif

#ifdef USE_FULL_LL_DRIVER
            LL_EX_TIM_DisableIT(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource);
#else
            TIM_DMACmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource, DISABLE);
#endif

            uint16_t rawValue;

            if (edges > MIN_GCR_EDGES) {
                dshotTelemetryState.readCount++;

                rawValue = decodeTelemetryPacket(dmaMotors[i].dmaBuffer, edges);

                if (rawValue != DSHOT_TELEMETRY_INVALID) {
                    // Check EDT enable or store raw value
                    if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(i) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                        dshotTelemetryState.motorState[i].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
                    } else {
                        dshotTelemetryState.motorState[i].rawValue = rawValue;
                    }
                } else {
                    dshotTelemetryState.invalidPacketCount++;
                    if (i == 0) {
                        memcpy(dshotTelemetryState.inputBuffer, dmaMotors[i].dmaBuffer, sizeof(dshotTelemetryState.inputBuffer));
                    }
                }
#ifdef USE_DSHOT_TELEMETRY_STATS
                updateDshotTelemetryQuality(&dshotTelemetryQuality[i], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
            }
        }
        pwmDshotSetDirectionOutput(&dmaMotors[i]);
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    inputStampUs = 0;
    dshotEnableChannels(dshotPwmDevice.count);
    return true;
}

#endif // USE_DSHOT_TELEMETRY
#endif // USE_DSHOT
