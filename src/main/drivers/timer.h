/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef CHEBUZZF3
#define USABLE_TIMER_CHANNEL_COUNT 18
#endif

#ifdef CC3D
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#if !defined(USABLE_TIMER_CHANNEL_COUNT)
#define USABLE_TIMER_CHANNEL_COUNT 14
#endif

#ifdef STM32F303xC
typedef uint32_t captureCompare_t;
#endif
#ifdef STM32F10X_MD
typedef uint16_t captureCompare_t;
#endif

typedef void timerCCCallbackPtr(uint8_t port, captureCompare_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
} timerHardware_t;

extern const timerHardware_t timerHardware[];

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerNVICConfigure(uint8_t irq);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *edgeCallback);
void configureTimerChannelCallbacks(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback);
