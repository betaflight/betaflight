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

#pragma once

#include "platform.h"

#include "drivers/timer.h"

typedef struct timerDef_s {
    void *TIMx;
#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rcc;
#endif
    uint8_t inputIrq;
} timerDef_t;

extern const timerDef_t timerDefinitions[];

typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;

volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);

// Platform-internal timer functions (not called from src/main)

// Get timer clock from raw timer instance pointer (for platform-internal use)
uint32_t timerClockFromInstance(const void *tim);

void *timerFindTimerHandle(void *tim);
void timerForceOverflow(void *tim);
volatile timCCR_t* timerCCR(void *tim, uint8_t channel);

uint16_t timerGetPrescalerByDesiredHertz(void *tim, uint32_t hz);
uint16_t timerGetPrescalerByDesiredMhz(void *tim, uint16_t mhz);
uint16_t timerGetPeriodByPrescaler(void *tim, uint16_t prescaler, uint32_t hz);

uint16_t timerDmaSource(uint8_t channel);
uint8_t timerLookupChannelIndex(const uint16_t channel);

void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallbackLo, timerEdgeHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChannelConfigInterruptDualLo(const timerHardware_t *timHw, FunctionalState newState);

#if PLATFORM_TRAIT_RCC
rccPeriphTag_t timerRCC(const void *tim);
#endif

#if defined(USE_HAL_DRIVER)
HAL_StatusTypeDef TIM_DMACmd(TIM_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState);
HAL_StatusTypeDef DMA_SetCurrDataCounter(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
uint16_t timerDmaIndex(uint8_t channel);
#else
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);
#endif
