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


#include <stdbool.h>
#include <stdint.h>
#include <math.h>

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
#elif defined(STM32F3)
#include "stm32f30x.h"
#endif

#include "pwm_output.h"

#include "pwm_output_dshot_shared.h"

FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
#ifdef STM32F7
FAST_RAM_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
FAST_RAM_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#else
motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#endif

#ifdef USE_DSHOT_TELEMETRY
uint32_t readDoneCount;

// TODO remove once debugging no longer needed
FAST_RAM_ZERO_INIT uint32_t dshotInvalidPacketCount;
FAST_RAM_ZERO_INIT uint32_t inputBuffer[DSHOT_TELEMETRY_INPUT_LEN];
FAST_RAM_ZERO_INIT uint32_t setDirectionMicros;
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
    if (pwmDshotCommandIsProcessing()) {
        value = pwmGetDshotCommand(index);
#ifdef USE_DSHOT_TELEMETRY
        // reset telemetry debug statistics every time telemetry is enabled
        if (value == DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY) {
            dshotInvalidPacketCount = 0;
            readDoneCount = 0;
        }
#endif
        if (value) {
            motor->requestTelemetry = true;
        }
    }

    motor->value = value;

    uint16_t packet = prepareDshotPacket(motor);
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
#ifdef STM32F7
        LL_EX_DMA_SetDataLength(motor->dmaRef, bufferSize);
        LL_EX_DMA_EnableStream(motor->dmaRef);
#else
        DMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
        DMA_Cmd(motor->dmaRef, ENABLE);
#endif
    }
}


#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount);

static uint16_t decodeDshotPacket(uint32_t buffer[])
{
    uint32_t value = 0;
    for (int i = 1; i < DSHOT_TELEMETRY_INPUT_LEN; i += 2) {
        int diff = buffer[i] - buffer[i-1];
        value <<= 1;
        if (diff > 0) {
            if (diff >= 11) value |= 1;
        } else {
            if (diff >= -9) value |= 1;
        }
    }

    uint32_t csum = value;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if (csum & 0xf) {
        return 0xffff;
    }
    return value >> 4;
}

static uint16_t decodeProshotPacket(uint32_t buffer[])
{
    uint32_t value = 0;
    for (int i = 1; i < PROSHOT_TELEMETRY_INPUT_LEN; i += 2) {
        const int proshotModulo = MOTOR_NIBBLE_LENGTH_PROSHOT;
        int diff = ((buffer[i] + proshotModulo - buffer[i-1]) % proshotModulo) - PROSHOT_BASE_SYMBOL;
        int nibble;
        if (diff < 0) {
            nibble = 0;
        } else {
            nibble = (diff + PROSHOT_BIT_WIDTH / 2) / PROSHOT_BIT_WIDTH;
        }
        value <<= 4;
        value |= (nibble & 0xf);
    }

    uint32_t csum = value;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if (csum & 0xf) {
        return 0xffff;
    }
    return value >> 4;
}


uint16_t getDshotTelemetry(uint8_t index)
{
    return dmaMotors[index].dshotTelemetryValue;
}

#endif

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor, bool output
#ifndef USE_DSHOT_TELEMETRY
#ifdef STM32F7
    , LL_TIM_OC_InitTypeDef* pOcInit, LL_DMA_InitTypeDef* pDmaInit)
#else
    , TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef* pDmaInit
#endif
#endif
);

#ifdef USE_DSHOT_TELEMETRY
#ifdef USE_DSHOT_TELEMETRY_STATS
void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, timeMs_t currentTimeMs)
{
    uint8_t statsBucketIndex = (currentTimeMs / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;
    if (statsBucketIndex != qualityStats->lastBucketIndex) {
        qualityStats->packetCountSum -= qualityStats->packetCountArray[statsBucketIndex];
        qualityStats->invalidCountSum -= qualityStats->invalidCountArray[statsBucketIndex];
        qualityStats->packetCountArray[statsBucketIndex] = 0;
        qualityStats->invalidCountArray[statsBucketIndex] = 0;
        qualityStats->lastBucketIndex = statsBucketIndex;
    }
    qualityStats->packetCountSum++;
    qualityStats->packetCountArray[statsBucketIndex]++;
    if (!packetValid) {
        qualityStats->invalidCountSum++;
        qualityStats->invalidCountArray[statsBucketIndex]++;
    }
}
#endif // USE_DSHOT_TELEMETRY_STATS

bool pwmStartDshotMotorUpdate(uint8_t motorCount)
{
    if (!useDshotTelemetry) {
        return true;
    }
#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif
    for (int i = 0; i < motorCount; i++) {
        if (dmaMotors[i].hasTelemetry) {
#ifdef STM32F7
            uint32_t edges = LL_EX_DMA_GetDataLength(dmaMotors[i].dmaRef);
#else
            uint32_t edges = DMA_GetCurrDataCounter(dmaMotors[i].dmaRef);
#endif
            uint16_t value = 0xffff;
            if (edges == 0) {
                if (dmaMotors[i].useProshot) {
                    value = decodeProshotPacket(dmaMotors[i].dmaBuffer);
                } else {
                    value = decodeDshotPacket(dmaMotors[i].dmaBuffer);
                }
            }
#ifdef USE_DSHOT_TELEMETRY_STATS
            bool validTelemetryPacket = false;
#endif
            if (value != 0xffff) {
                dmaMotors[i].dshotTelemetryValue = value;
                dmaMotors[i].dshotTelemetryActive = true;
                if (i < 4) {
                    DEBUG_SET(DEBUG_DSHOT_RPM_TELEMETRY, i, value);
                }
#ifdef USE_DSHOT_TELEMETRY_STATS
                validTelemetryPacket = true;
#endif
            } else {
                dshotInvalidPacketCount++;
                if (i == 0) {
                    memcpy(inputBuffer,dmaMotors[i].dmaBuffer,sizeof(inputBuffer));
                }
            }
            dmaMotors[i].hasTelemetry = false;
#ifdef USE_DSHOT_TELEMETRY_STATS
            updateDshotTelemetryQuality(&dmaMotors[i].dshotTelemetryQuality, validTelemetryPacket, currentTimeMs);
#endif
        } else {
            timeDelta_t usSinceInput = cmpTimeUs(micros(), dmaMotors[i].timer->inputDirectionStampUs);
            if (usSinceInput >= 0 && usSinceInput < dmaMotors[i].dshotTelemetryDeadtimeUs) {
                return false;
            }
#ifdef STM32F7
            LL_EX_TIM_DisableIT(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource);
#else
            TIM_DMACmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource, DISABLE);
#endif
        }
        pwmDshotSetDirectionOutput(&dmaMotors[i], true);
    }
    dshotEnableChannels(motorCount);
    return true;
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    return dmaMotors[motorIndex].dshotTelemetryActive;
}

#ifdef USE_DSHOT_TELEMETRY_STATS
int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    int16_t invalidPercent = 0;

    if (dmaMotors[motorIndex].dshotTelemetryActive) {
        const uint32_t totalCount = dmaMotors[motorIndex].dshotTelemetryQuality.packetCountSum;
        const uint32_t invalidCount = dmaMotors[motorIndex].dshotTelemetryQuality.invalidCountSum;
        if (totalCount > 0) {
            invalidPercent = lrintf(invalidCount * 10000.0f / totalCount);
        }
    } else {
        invalidPercent = 10000;  // 100.00%
    }
    return invalidPercent;
}
#endif // USE_DSHOT_TELEMETRY_STATS
#endif // USE_DSHOT_TELEMETRY
#endif // USE_DSHOT
