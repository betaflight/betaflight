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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "nvic.h"

#include "gpio.h"
#include "system.h"

#include "timer.h"

/* FreeFlight/Naze32 timer layout
    TIM2_CH1    RC1             PWM1
    TIM2_CH2    RC2             PWM2
    TIM2_CH3    RC3/UA2_TX      PWM3
    TIM2_CH4    RC4/UA2_RX      PWM4
    TIM3_CH1    RC5             PWM5
    TIM3_CH2    RC6             PWM6
    TIM3_CH3    RC7             PWM7
    TIM3_CH4    RC8             PWM8
    TIM1_CH1    PWM1            PWM9
    TIM1_CH4    PWM2            PWM10
    TIM4_CH1    PWM3            PWM11
    TIM4_CH2    PWM4            PWM12
    TIM4_CH3    PWM5            PWM13
    TIM4_CH4    PWM6            PWM14

    RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    RX2  TIM2_CH2 PA1
    RX3  TIM2_CH3 PA2 [also UART2_TX]
    RX4  TIM2_CH4 PA3 [also UART2_RX]
    RX5  TIM3_CH1 PA6 [also ADC_IN6]
    RX6  TIM3_CH2 PA7 [also ADC_IN7]
    RX7  TIM3_CH3 PB0 [also ADC_IN8]
    RX8  TIM3_CH4 PB1 [also ADC_IN9]

    Outputs
    PWM1 TIM1_CH1 PA8
    PWM2 TIM1_CH4 PA11
    PWM3 TIM4_CH1 PB6 [also I2C1_SCL]
    PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    PWM5 TIM4_CH3 PB8
    PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels
*/

#if defined(CJMCU) || defined(EUSTM32F103RC) || defined(NAZE) || defined(OLIMEXINO) || defined(PORT103R)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD},          // PWM1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD},          // PWM2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_IPD},          // PWM3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_IPD},          // PWM4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, Mode_IPD},          // PWM5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD},          // PWM6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD},          // PWM7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD},          // PWM8
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD},       // PWM9
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD},      // PWM10
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD},          // PWM11
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, Mode_IPD},          // PWM12
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, Mode_IPD},          // PWM13
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, Mode_IPD}           // PWM14
};

#define MAX_TIMERS 4 // TIM1..TIM4

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#endif

#ifdef CC3D
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD}, // S1_IN
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD}, // S2_IN - GPIO_PartialRemap_TIM3
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD}, // S3_IN - SoftSerial RX
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD}, // S4_IN - SoftSerial TX
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD}, // S5_IN
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD}, // S6_IN

    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S1_OUT
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S2_OUT
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S3_OUT
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, GPIO_Mode_AF_PP}, // S4_OUT
    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF_PP},    // S5_OUT - GPIO_PartialRemap_TIM3 - LED Strip
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF_PP},    // S6_OUT
};

#define MAX_TIMERS 4 // TIM1..TIM4

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#endif

#if defined(STM32F3DISCOVERY) && !(defined(CHEBUZZF3))
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_6},             // PWM1 - PA8
    { TIM16, GPIOB, Pin_8, TIM_Channel_1, TIM1_UP_TIM16_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_1},      // PWM2 - PB8
    { TIM17, GPIOB, Pin_9, TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_1}, // PWM3 - PB9
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource6, GPIO_AF_4},             // PWM4 - PC6
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource7, GPIO_AF_4},             // PWM5 - PC7
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_4},             // PWM6 - PC8
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource1, GPIO_AF_2},                // PWM7 - PB1
    { TIM3, GPIOA, Pin_4, TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource4, GPIO_AF_2},                // PWM8 - PA2
    { TIM4, GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_2},                  // PWM9 - PD12
    { TIM4, GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_2},                  // PWM10 - PD13
    { TIM4, GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_2},                  // PWM11 - PD14
    { TIM4, GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2},                  // PWM12 - PD15
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1},                   // PWM13 - PA1
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1}                    // PWM14 - PA2
};

#define MAX_TIMERS 7

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4, TIM8, TIM16, TIM17
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD)

#endif

#ifdef CHEBUZZF3
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // INPUTS CH1-8
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_6}, // PWM1 - PA8
    { TIM16, GPIOB, Pin_8,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      0, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_1}, // PWM2 - PB8
    { TIM17, GPIOB, Pin_9,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_1}, // PWM3 - PB9
    { TIM8,  GPIOC, Pin_6,  TIM_Channel_1, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource6, GPIO_AF_4}, // PWM4 - PC6
    { TIM8,  GPIOC, Pin_7,  TIM_Channel_2, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource7, GPIO_AF_4}, // PWM5 - PC7
    { TIM8,  GPIOC, Pin_8,  TIM_Channel_3, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_4}, // PWM6 - PC8
    { TIM15, GPIOF, Pin_9,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_3}, // PWM7 - PF9
    { TIM15, GPIOF, Pin_10, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     0, Mode_AF_PP_PD, GPIO_PinSource10, GPIO_AF_3}, // PWM8 - PF10

    // OUTPUTS CH1-10
    { TIM4,  GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_2},    // PWM9 - PD12
    { TIM4,  GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_2},    // PWM10 - PD13
    { TIM4,  GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_2},    // PWM11 - PD14
    { TIM4,  GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2},    // PWM12 - PD15
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1},    // PWM13 - PA1
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1},    // PWM14 - PA2
    { TIM2,  GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1},    // PWM15 - PA3
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2},    // PWM16 - PB0
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2},    // PWM17 - PB1
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_2}     // PWM18 - PA4
};

#define MAX_TIMERS 8

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16, TIM17
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF)

#endif

#ifdef NAZE32PRO
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_6}, // PA8 - AF6
    { TIM1,  GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_6}, // PA9 - AF6
    { TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource10, GPIO_AF_6}, // PA10 - AF6
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource4,  GPIO_AF_2}, // PB4 - AF2
    { TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource6,  GPIO_AF_2}, // PB6 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_2}, // PB7 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_2}, // PB8 - AF2
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_2}, // PB9 - AF2

    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2}, // PA0 - untested
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2}, // PA1 - untested
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_9}, // PA2 - untested
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_9}, // PA3 - untested
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_1}, // PA6 - untested
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_1}, // PA7 - untested
};

#define MAX_TIMERS 7

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4, TIM15, TIM16, TIM17
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif


#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4
static const uint16_t const channels[CC_CHANNELS_PER_TIMER] = {
    TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

typedef struct timerConfig_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    timerCCCallbackPtr *edgeCallback;
    timerCCCallbackPtr *overflowCallback;
    uint8_t reference;
} timerConfig_t;

static timerConfig_t timerConfig[MAX_TIMERS * CC_CHANNELS_PER_TIMER];

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
    uint8_t timerIndex = 0;
    while (timers[timerIndex] != tim) {
        timerIndex++;
    }
    return timerIndex;
}

static uint8_t lookupChannelIndex(const uint16_t channel)
{
    uint8_t channelIndex = 0;
    while (channels[channelIndex] != channel) {
        channelIndex++;
    }
    return channelIndex;
}

static uint8_t lookupTimerConfigIndex(TIM_TypeDef *tim, const uint16_t channel)
{
    return lookupTimerIndex(tim) + (MAX_TIMERS * lookupChannelIndex(channel));
}

void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *edgeCallback)
{
    configureTimerChannelCallbacks(tim, channel, reference, edgeCallback, NULL);
}

void configureTimerChannelCallbacks(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback)
{
    assert_param(IS_TIM_CHANNEL(channel));

    uint8_t timerConfigIndex = lookupTimerConfigIndex(tim, channel);

    if (timerConfigIndex >= MAX_TIMERS * CC_CHANNELS_PER_TIMER) {
        return;
    }

    timerConfig[timerConfigIndex].edgeCallback = edgeCallback;
    timerConfig[timerConfigIndex].overflowCallback = overflowCallback;
    timerConfig[timerConfigIndex].channel = channel;
    timerConfig[timerConfigIndex].reference = reference;
}

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel)
{
    switch (channel) {
        case TIM_Channel_1:
            TIM_ITConfig(tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback)
{
    configureTimerChannelCallbacks(timerHardwarePtr->tim, timerHardwarePtr->channel, reference, edgeCallback, overflowCallback);
    configureTimerInputCaptureCompareChannel(timerHardwarePtr->tim, timerHardwarePtr->channel);
    if (overflowCallback) {
        TIM_ITConfig(timerHardwarePtr->tim, TIM_IT_Update, ENABLE);
    }
}

void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;


    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    configTimeBase(timerHardwarePtr->tim, period, mhz);
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);
    timerNVICConfigure(timerHardwarePtr->irq);
}

timerConfig_t *findTimerConfig(TIM_TypeDef *tim, uint16_t channel)
{
    uint8_t timerConfigIndex = lookupTimerConfigIndex(tim, channel);
    return &(timerConfig[timerConfigIndex]);
}

static void timCCxHandler(TIM_TypeDef *tim)
{
    captureCompare_t capture;
    timerConfig_t *timerConfig;
    uint8_t channel;
    uint8_t channelIndex;

    if (TIM_GetITStatus(tim, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(tim, TIM_IT_Update);
        capture = tim->ARR;

        for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++) {
            channel = channels[channelIndex];
            timerConfig = findTimerConfig(tim, channel);

            if (!timerConfig->overflowCallback) {
                continue;
            }
            timerConfig->overflowCallback(timerConfig->reference, capture);
        }
    }

    for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++) {
        channel = channels[channelIndex];

        if (channel == TIM_Channel_1 && TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC1);

            timerConfig = findTimerConfig(tim, TIM_Channel_1);
            capture = TIM_GetCapture1(tim);
        } else if (channel == TIM_Channel_2 && TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC2);

            timerConfig = findTimerConfig(tim, TIM_Channel_2);
            capture = TIM_GetCapture2(tim);
        } else if (channel == TIM_Channel_3 && TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC3);

            timerConfig = findTimerConfig(tim, TIM_Channel_3);
            capture = TIM_GetCapture3(tim);
        } else if (channel == TIM_Channel_4 && TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC4);

            timerConfig = findTimerConfig(tim, TIM_Channel_4);
            capture = TIM_GetCapture4(tim);
        } else {
            continue; // avoid uninitialised variable dereference
        }

        if (!timerConfig->edgeCallback) {
            continue;
        }
        timerConfig->edgeCallback(timerConfig->reference, capture);
    }
}

void TIM1_CC_IRQHandler(void)
{
    timCCxHandler(TIM1);
}

void TIM2_IRQHandler(void)
{
    timCCxHandler(TIM2);
}

void TIM3_IRQHandler(void)
{
    timCCxHandler(TIM3);
}

void TIM4_IRQHandler(void)
{
    timCCxHandler(TIM4);
}

#if defined(STM32F303) || defined(STM32F3DISCOVERY)
void TIM8_CC_IRQHandler(void)
{
    timCCxHandler(TIM8);
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    timCCxHandler(TIM15);
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    timCCxHandler(TIM16);
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    timCCxHandler(TIM17);
}
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

#ifdef CC3D
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif

#ifdef TIMER_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_AHB_PERIPHERALS
    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
#endif

#ifdef STM32F303xC
    for (uint8_t timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        GPIO_PinAFConfig(timerHardwarePtr->gpio, (uint16_t)timerHardwarePtr->gpioPinSource, timerHardwarePtr->alternateFunction);
    }
#endif

#if 0
#if defined(STMF3DISCOVERY) || defined(NAZE32PRO)
    // FIXME move these where they are needed.
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
#endif
#endif

}
