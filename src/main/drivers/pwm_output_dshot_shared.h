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

#ifdef USE_DSHOT

// TODO remove once debugging no longer needed
#ifdef USE_DSHOT_TELEMETRY
#include <string.h>
#endif

#if defined(STM32F4) || defined(STM32F7)
typedef DMA_Stream_TypeDef dmaStream_t;
#else
typedef DMA_Channel_TypeDef dmaStream_t;
#endif

extern FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount;
#ifdef STM32F7
extern FAST_RAM_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern FAST_RAM_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#else
extern motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#endif

#ifdef USE_DSHOT_TELEMETRY
extern uint32_t readDoneCount;

// TODO remove once debugging no longer needed
FAST_RAM_ZERO_INIT extern uint32_t dshotInvalidPacketCount;
FAST_RAM_ZERO_INIT extern uint32_t inputBuffer[DSHOT_TELEMETRY_INPUT_LEN];
FAST_RAM_ZERO_INIT extern uint32_t setDirectionMicros;
#endif

uint8_t getTimerIndex(TIM_TypeDef *timer);
motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
void dshotEnableChannels(uint8_t motorCount);

#ifdef USE_DSHOT_TELEMETRY

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

bool pwmStartDshotMotorUpdate(uint8_t motorCount);

#endif
#endif
