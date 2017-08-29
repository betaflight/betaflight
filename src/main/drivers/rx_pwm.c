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

#include <platform.h>

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "timer.h"

#include "pwm_output.h"
#include "pwm_mapping.h"

#include "rx_pwm.h"

#define DEBUG_PPM_ISR

#define PPM_CAPTURE_COUNT 16
#define PWM_INPUT_PORT_COUNT 8

#if PPM_CAPTURE_COUNT > MAX_PWM_INPUT_PORTS
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

// TODO - change to timer clocks ticks
#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 0x03

static inputFilteringMode_e inputFilteringMode;

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

typedef enum {
    INPUT_MODE_PPM,
    INPUT_MODE_PWM,
} pwmInputMode_t;

typedef struct {
    pwmInputMode_t mode;
    uint8_t channel; // only used for pwm, ignored by ppm

    uint8_t state;
    captureCompare_t rise;
    captureCompare_t fall;
    captureCompare_t capture;

    uint8_t missedEvents;

    const timerHardware_t *timerHardware;
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0x10000
#define PWM_TIMER_PERIOD 0x10000

static uint8_t ppmFrameCount = 0;
static uint8_t lastPPMFrameCount = 0;
static uint8_t ppmCountDivisor = 1;

typedef struct ppmDevice_s {
    uint8_t  pulseIndex;
    //uint32_t previousTime;
    uint32_t currentCapture;
    uint32_t currentTime;
    uint32_t deltaTime;
    uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
    uint32_t largeCounter;
    int8_t   numChannels;
    int8_t   numChannelsPrevFrame;
    uint8_t  stableFramesSeenCount;

    bool     tracking;
    bool     overflowed;
} ppmDevice_t;

ppmDevice_t ppmDev;


#define PPM_IN_MIN_SYNC_PULSE_US    2700    // microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US 750     // microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US 2250    // microseconds
#define PPM_STABLE_FRAMES_REQUIRED_COUNT    25
#define PPM_IN_MIN_NUM_CHANNELS     4
#define PPM_IN_MAX_NUM_CHANNELS     PWM_PORTS_OR_PPM_CAPTURE_COUNT


bool isPPMDataBeingReceived(void)
{
    return (ppmFrameCount != lastPPMFrameCount);
}

void resetPPMDataReceivedState(void)
{
    lastPPMFrameCount = ppmFrameCount;
}

#define MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID 4

void pwmRxInit(inputFilteringMode_e inputFilteringModeToUse)
{
    inputFilteringMode = inputFilteringModeToUse;
}

#ifdef DEBUG_PPM_ISR
typedef enum {
    SOURCE_OVERFLOW = 0,
    SOURCE_EDGE = 1
} eventSource_e;

typedef struct ppmISREvent_s {
    eventSource_e source;
    uint32_t capture;
} ppmISREvent_t;

static ppmISREvent_t ppmEvents[20];
static uint8_t ppmEventIndex = 0;

void ppmISREvent(eventSource_e source, uint32_t capture)
{
    ppmEventIndex = (ppmEventIndex + 1) % (sizeof(ppmEvents) / sizeof(ppmEvents[0]));

    ppmEvents[ppmEventIndex].source = source;
    ppmEvents[ppmEventIndex].capture = capture;
}
#else
void ppmISREvent(eventSource_e source, uint32_t capture) {}
#endif

static void ppmInit(void)
{
    ppmDev.pulseIndex   = 0;
    ppmDev.currentCapture = 0;
    ppmDev.currentTime  = 0;
    ppmDev.deltaTime    = 0;
    ppmDev.largeCounter = 0;
    ppmDev.numChannels  = -1;
    ppmDev.numChannelsPrevFrame = -1;
    ppmDev.stableFramesSeenCount = 0;
    ppmDev.tracking     = false;
    ppmDev.overflowed   = false;
}

static void ppmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    ppmISREvent(SOURCE_OVERFLOW, capture);

    ppmDev.largeCounter += capture + 1;
    if (capture == PPM_TIMER_PERIOD - 1) {
        ppmDev.overflowed = true;
    }
}

static void ppmEdgeCallback(timerCCHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    ppmISREvent(SOURCE_EDGE, capture);

    int32_t i;

    uint32_t previousTime = ppmDev.currentTime;
    uint32_t previousCapture = ppmDev.currentCapture;

    /* Grab the new count */
    uint32_t currentTime = capture;

    /* Convert to 32-bit timer result */
    currentTime += ppmDev.largeCounter;

    if (capture < previousCapture) {
        if (ppmDev.overflowed) {
            currentTime += PPM_TIMER_PERIOD;
        }
    }

    // Divide by 8 if Oneshot125 is active and this is a CC3D board
    currentTime = currentTime / ppmCountDivisor;

    /* Capture computation */
    if (currentTime > previousTime) {
        ppmDev.deltaTime    = currentTime - (previousTime + (ppmDev.overflowed ? (PPM_TIMER_PERIOD / ppmCountDivisor) : 0));
    } else {
        ppmDev.deltaTime    = (PPM_TIMER_PERIOD / ppmCountDivisor) + currentTime - previousTime;
    }

    ppmDev.overflowed = false;


    /* Store the current measurement */
    ppmDev.currentTime = currentTime;
    ppmDev.currentCapture = capture;

#if 1
    static uint32_t deltaTimes[20];
    static uint8_t deltaIndex = 0;

    deltaIndex = (deltaIndex + 1) % 20;
    deltaTimes[deltaIndex] = ppmDev.deltaTime;
    UNUSED(deltaTimes);
#endif


#if 1
    static uint32_t captureTimes[20];
    static uint8_t captureIndex = 0;

    captureIndex = (captureIndex + 1) % 20;
    captureTimes[captureIndex] = capture;
    UNUSED(captureTimes);
#endif

    /* Sync pulse detection */
    if (ppmDev.deltaTime > PPM_IN_MIN_SYNC_PULSE_US) {
        if (ppmDev.pulseIndex == ppmDev.numChannelsPrevFrame
            && ppmDev.pulseIndex >= PPM_IN_MIN_NUM_CHANNELS
            && ppmDev.pulseIndex <= PPM_IN_MAX_NUM_CHANNELS) {
            /* If we see n simultaneous frames of the same
               number of channels we save it as our frame size */
            if (ppmDev.stableFramesSeenCount < PPM_STABLE_FRAMES_REQUIRED_COUNT) {
                ppmDev.stableFramesSeenCount++;
            } else {
                ppmDev.numChannels = ppmDev.pulseIndex;
            }
        } else {
            ppmDev.stableFramesSeenCount = 0;
        }

        /* Check if the last frame was well formed */
        if (ppmDev.pulseIndex == ppmDev.numChannels && ppmDev.tracking) {
            /* The last frame was well formed */
            for (i = 0; i < ppmDev.numChannels; i++) {
                captures[i] = ppmDev.captures[i];
            }
            for (i = ppmDev.numChannels; i < PPM_IN_MAX_NUM_CHANNELS; i++) {
                captures[i] = PPM_RCVR_TIMEOUT;
            }
            ppmFrameCount++;
        }

        ppmDev.tracking   = true;
        ppmDev.numChannelsPrevFrame = ppmDev.pulseIndex;
        ppmDev.pulseIndex = 0;

        /* We rely on the supervisor to set captureValue to invalid
           if no valid frame is found otherwise we ride over it */
    } else if (ppmDev.tracking) {
        /* Valid pulse duration 0.75 to 2.5 ms*/
        if (ppmDev.deltaTime > PPM_IN_MIN_CHANNEL_PULSE_US
            && ppmDev.deltaTime < PPM_IN_MAX_CHANNEL_PULSE_US
            && ppmDev.pulseIndex < PPM_IN_MAX_NUM_CHANNELS) {
            ppmDev.captures[ppmDev.pulseIndex] = ppmDev.deltaTime;
            ppmDev.pulseIndex++;
        } else {
            /* Not a valid pulse duration */
            ppmDev.tracking = false;
            for (i = 0; i < PWM_PORTS_OR_PPM_CAPTURE_COUNT; i++) {
                ppmDev.captures[i] = PPM_RCVR_TIMEOUT;
            }
        }
    }
}

#define MAX_MISSED_PWM_EVENTS 10

bool isPWMDataBeingReceived(void)
{
    int channel;
    for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) {
        if (captures[channel] != PPM_RCVR_TIMEOUT) {
            return true;
        }
    }
    return false;
}

static void pwmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, overflowCb);

    if (++pwmInputPort->missedEvents > MAX_MISSED_PWM_EVENTS) {
        captures[pwmInputPort->channel] = PPM_RCVR_TIMEOUT;
        pwmInputPort->missedEvents = 0;
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, edgeCb);
    const timerHardware_t *timerHardwarePtr = pwmInputPort->timerHardware;

    if (pwmInputPort->state == 0) {
        pwmInputPort->rise = capture;
        pwmInputPort->state = 1;
#if defined(USE_HAL_DRIVER)
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_FALLING);
#else
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
#endif
    } else {
        pwmInputPort->fall = capture;

        // compute and store capture and handle overflow correctly - timer may be configured for PWM output in such case overflow value is not 0xFFFF
        if (pwmInputPort->fall >= pwmInputPort->rise) {
            pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
        }
        else {
            pwmInputPort->capture = (pwmInputPort->fall + timerGetPeriod(timerHardwarePtr)) - pwmInputPort->rise;
        }
        captures[pwmInputPort->channel] = pwmInputPort->capture;

        // switch state
        pwmInputPort->state = 0;
#if defined(USE_HAL_DRIVER)
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_RISING);
#else
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
#endif
        pwmInputPort->missedEvents = 0;
    }
}

#ifdef USE_HAL_DRIVER

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarity;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;

    if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
        TIM_ICInitStructure.ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
    } else {
        TIM_ICInitStructure.ICFilter = 0x00;
    }

    HAL_TIM_IC_ConfigChannel(Handle, &TIM_ICInitStructure, channel);
    HAL_TIM_IC_Start_IT(Handle,channel);
}
#else
void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;

    if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
        TIM_ICInitStructure.TIM_ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
    } else {
        TIM_ICInitStructure.TIM_ICFilter = 0x00;
    }

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

#endif


void pwmInConfig(const timerHardware_t *timerHardwarePtr, uint8_t channel)
{
    pwmInputPort_t *self = &pwmInputPorts[channel];

    self->state = 0;
    self->missedEvents = 0;
    self->channel = channel;
    self->mode = INPUT_MODE_PWM;
    self->timerHardware = timerHardwarePtr;

    IO_t io = IOGetByTag(timerHardwarePtr->tag);
    IOInit(io, OWNER_PWMINPUT, RESOURCE_INPUT, RESOURCE_INDEX(channel));
#if defined(STM32F7)
    IOConfigGPIOAF(io, timerHardwarePtr->ioMode, timerHardwarePtr->alternateFunction);
#else
    IOConfigGPIO(io, timerHardwarePtr->ioMode);
#endif

    timerConfigure(timerHardwarePtr, (uint16_t)PWM_TIMER_PERIOD, PWM_TIMER_MHZ);
    timerChCCHandlerInit(&self->edgeCb, pwmEdgeCallback);
    timerChOvrHandlerInit(&self->overflowCb, pwmOverflowCallback);
    timerChConfigCallbacks(timerHardwarePtr, &self->edgeCb, &self->overflowCb);

#if defined(USE_HAL_DRIVER)
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_RISING);
#else
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
#endif
}

#define UNUSED_PPM_TIMER_REFERENCE 0
#define FIRST_PWM_PORT 0

void ppmAvoidPWMTimerClash(const timerHardware_t *timerHardwarePtr, uint8_t motorPwmProtocol)
{
    for (int timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        // If PPM input timer is also mapped to motor - set PPM divisor accordingly
        if (((timerHardware[timerIndex].usageFlags & (TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR)) == 0) || timerHardware[timerIndex].tim != timerHardwarePtr->tim)
            continue;

        switch (motorPwmProtocol) {
        case PWM_TYPE_ONESHOT125:
            ppmCountDivisor = ONESHOT125_TIMER_MHZ;
            break;
        case PWM_TYPE_ONESHOT42:
            ppmCountDivisor = ONESHOT42_TIMER_MHZ;
            break;
        case PWM_TYPE_MULTISHOT:
            ppmCountDivisor = MULTISHOT_TIMER_MHZ;
            break;
        case PWM_TYPE_BRUSHED:
            ppmCountDivisor = PWM_BRUSHED_TIMER_MHZ;
            break;
        default:
            break;
        }

        return;
    }
}

void ppmInConfig(const timerHardware_t *timerHardwarePtr, uint8_t motorPwmProtocol)
{
    ppmInit();

    ppmAvoidPWMTimerClash(timerHardwarePtr, motorPwmProtocol);

    pwmInputPort_t *self = &pwmInputPorts[FIRST_PWM_PORT];

    self->mode = INPUT_MODE_PPM;
    self->timerHardware = timerHardwarePtr;

    IO_t io = IOGetByTag(timerHardwarePtr->tag);
    IOInit(io, OWNER_PPMINPUT, RESOURCE_INPUT, 0);
#if defined(STM32F7)
    IOConfigGPIOAF(io, timerHardwarePtr->ioMode, timerHardwarePtr->alternateFunction);
#else
    IOConfigGPIO(io, timerHardwarePtr->ioMode);
#endif

    timerConfigure(timerHardwarePtr, (uint16_t)PPM_TIMER_PERIOD, PWM_TIMER_MHZ);
    timerChCCHandlerInit(&self->edgeCb, ppmEdgeCallback);
    timerChOvrHandlerInit(&self->overflowCb, ppmOverflowCallback);
    timerChConfigCallbacks(timerHardwarePtr, &self->edgeCb, &self->overflowCb);

#if defined(USE_HAL_DRIVER)
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_RISING);
#else
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
#endif
}

uint16_t ppmRead(uint8_t channel)
{
    return captures[channel];
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
#endif
