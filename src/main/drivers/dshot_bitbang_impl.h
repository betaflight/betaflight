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

#pragma once

#include "common/time.h"
#include "drivers/motor.h"

#define USE_DMA_REGISTER_CACHE

#define DEBUG_COUNT_INTERRUPT
#define DEBUG_MONITOR_PACER

#define MAX_SUPPORTED_MOTOR_PORTS 4 // Max direct dshot port groups, limited by number of usable timer (TIM1 and TIM8) x number of channels per timer (4), 3 is enough to cover motor pins on GPIOA, GPIOB and GPIOC.

#define DSHOT_BITBANG_TELEMETRY_OVER_SAMPLE 3

#define DSHOT_BITBANG_DIRECTION_OUTPUT 0
#define DSHOT_BITBANG_DIRECTION_INPUT  1

#define DSHOT_BITBANG_INVERTED         true
#define DSHOT_BITBANG_NONINVERTED      false

// XXX MOTOR_xSHOTyyyy_HZ is not usable as generic frequency for timers.
// XXX Trying to fiddle with constants here.

// Symbol rate [symbol/sec]
#define MOTOR_DSHOT600_SYMBOL_RATE     (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE     (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE     (150 * 1000)

#define MOTOR_DSHOT_SYMBOL_TIME_NS(rate)  (1000000000 / (rate))

#define MOTOR_DSHOT_BIT_PER_SYMBOL         1

#define MOTOR_DSHOT_STATE_PER_SYMBOL       3  // Initial high, 0/1, low

#define MOTOR_DSHOT_FRAME_BITS             16

#define MOTOR_DSHOT_FRAME_TIME_NS(rate)    ((MOTOR_DSHOT_FRAME_BITS / MOTOR_DSHOT_BIT_PER_SYMBOL) * MOTOR_DSHOT_SYMBOL_TIME_NS(rate))

#define MOTOR_DSHOT_TELEMETRY_WINDOW_US    (30000 + MOTOR_DSHOT_FRAME_TIME_NS(rate) * (1.1)) / 1000

#define MOTOR_DSHOT_CHANGE_INTERVAL_NS(rate) (MOTOR_DSHOT_SYMBOL_TIME_NS(rate) / MOTOR_DSHOT_STATE_PER_SYMBOL)

#define MOTOR_DSHOT_GCR_CHANGE_INTERVAL_NS(rate) (MOTOR_DSHOT_CHANGE_INTERVAL_NS(rate) * 5 / 4)

#define MOTOR_DSHOT_BUFFER_SIZE            ((MOTOR_DSHOT_FRAME_BITS / MOTOR_DSHOT_BIT_PER_SYMBOL) * MOTOR_DSHOT_STATE_PER_SYMBOL)

#ifdef USE_HAL_DRIVER
#define BB_GPIO_PULLDOWN GPIO_PULLDOWN
#define BB_GPIO_PULLUP   GPIO_PULLUP
#else
#define BB_GPIO_PULLDOWN GPIO_PuPd_DOWN
#define BB_GPIO_PULLUP   GPIO_PuPd_UP
#endif

#ifdef USE_DMA_REGISTER_CACHE
typedef struct dmaRegCache_s {
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    uint32_t CR;
    uint32_t FCR;
    uint32_t NDTR;
    uint32_t PAR;
    uint32_t M0AR;
#elif defined(STM32G4)
    uint32_t CCR;
    uint32_t CNDTR;
    uint32_t CPAR;
    uint32_t CMAR;
#else
#error No MCU dependent code here
#endif
} dmaRegCache_t;
#endif

// Per pacer timer

typedef struct bbPacer_s {
    TIM_TypeDef *tim;
    uint16_t dmaSources;
} bbPacer_t;

// Per GPIO port and timer channel

typedef struct bbPort_s {
    int portIndex;
    GPIO_TypeDef *gpio;
    const timerHardware_t *timhw;
#ifdef USE_HAL_DRIVER
    uint32_t llChannel;
#endif

    uint16_t dmaSource;

    dmaResource_t *dmaResource; // DMA resource for this port & timer channel
    uint32_t dmaChannel;        // DMA channel or peripheral request

    uint8_t direction;

#ifdef USE_DMA_REGISTER_CACHE
    // DMA resource register cache
    dmaRegCache_t dmaRegOutput;
    dmaRegCache_t dmaRegInput;
#endif

    // For direct manipulation of GPIO_MODER register
    uint32_t gpioModeMask;
    uint32_t gpioModeInput;
    uint32_t gpioModeOutput;

    // Idle value
    uint32_t gpioIdleBSRR;

#ifdef USE_HAL_DRIVER
    LL_TIM_InitTypeDef timeBaseInit;
#else
    TIM_TimeBaseInitTypeDef timeBaseInit;
#endif

    // Output
    uint16_t outputARR;
#ifdef USE_HAL_DRIVER
    LL_DMA_InitTypeDef outputDmaInit;
#else
    DMA_InitTypeDef outputDmaInit;
#endif
    uint32_t *portOutputBuffer;
    uint32_t portOutputCount;

    // Input
    uint16_t inputARR;
#ifdef USE_HAL_DRIVER
    LL_DMA_InitTypeDef inputDmaInit;
#else
    DMA_InitTypeDef inputDmaInit;
#endif
    uint16_t *portInputBuffer;
    uint32_t portInputCount;
    bool inputActive;

    // Misc
#ifdef DEBUG_COUNT_INTERRUPT
    uint32_t outputIrq;
    uint32_t inputIrq;
#endif
    resourceOwner_t owner;
} bbPort_t;

// Per motor output

typedef struct bbMotor_s {
    dshotProtocolControl_t protocolControl;
    int pinIndex;            // pinIndex of this motor output within a group that bbPort points to
    int portIndex;
    IO_t io;                 // IO_t for this output
    uint8_t output;
    uint32_t iocfg;
    bbPort_t *bbPort;
    bool configured;
    bool enabled;
} bbMotor_t;

#define MAX_MOTOR_PACERS  4
extern FAST_DATA_ZERO_INIT bbPacer_t bbPacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
extern FAST_DATA_ZERO_INIT int usedMotorPacers;

extern FAST_DATA_ZERO_INIT bbPort_t bbPorts[MAX_SUPPORTED_MOTOR_PORTS];
extern FAST_DATA_ZERO_INIT int usedMotorPorts;

extern FAST_DATA_ZERO_INIT bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];

extern uint8_t bbPuPdMode;

// DMA buffers
// Note that we are not sharing input and output buffers,
// as output buffer is only modified for middle bits

// DMA output buffer:
// DShot requires 3 [word/bit] * 16 [bit] = 48 [word]
extern uint32_t bbOutputBuffer[MOTOR_DSHOT_BUFFER_SIZE * MAX_SUPPORTED_MOTOR_PORTS];

// DMA input buffer
// (30us + <frame time> + <slack>) / <input sampling clock perid>
// <frame time> = <DShot symbol time> * 16
// Temporary size for DS600
// <frame time> = 26us
// <sampling period> = 0.44us
// <slack> = 10%
// (30 + 26 + 3) / 0.44 = 134
// In some cases this was not enough, so we add 6 extra samples
#define DSHOT_BITBANG_PORT_INPUT_BUFFER_LENGTH 140
extern uint16_t bbInputBuffer[DSHOT_BITBANG_PORT_INPUT_BUFFER_LENGTH * MAX_SUPPORTED_MOTOR_PORTS];

void bbGpioSetup(bbMotor_t *bbMotor);
void bbTimerChannelInit(bbPort_t *bbPort);
void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction);
void bbDMAIrqHandler(dmaChannelDescriptor_t *descriptor);
void bbSwitchToOutput(bbPort_t * bbPort);
void bbSwitchToInput(bbPort_t * bbPort);

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period);
void bbTIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void bbDMA_ITConfig(bbPort_t *bbPort);
void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState);
int  bbDMA_Count(bbPort_t *bbPort);
