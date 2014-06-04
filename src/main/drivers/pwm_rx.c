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

#include "platform.h"

#include "gpio.h"
#include "timer.h"

#include "pwm_mapping.h"

#include "pwm_rx.h"

#define PPM_CAPTURE_COUNT 12
#define PWM_INPUT_PORT_COUNT 8

#if PPM_CAPTURE_COUNT > MAX_PWM_INPUT_PORTS
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity); // from pwm_output.c

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

    const timerHardware_t *timerHardware;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0xFFFF
#define PWM_TIMER_PERIOD 0xFFFF

static uint8_t ppmFrameCount = 0;
static uint8_t lastPPMFrameCount = 0;

bool isPPMDataBeingReceived(void)
{
    return (ppmFrameCount != lastPPMFrameCount);
}

void resetPPMDataReceivedState(void)
{
    lastPPMFrameCount = ppmFrameCount;
}

#define MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID 4

static void ppmCallback(uint8_t port, captureCompare_t capture)
{
    uint16_t diff; // See PPM_TIMER_PERIOD
    static captureCompare_t now;
    static captureCompare_t last = 0;

    static uint8_t chan = 0;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        if (chan >= MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID) {
            ppmFrameCount++;
        }
        chan = 0;
    } else {
        if (chan < PPM_CAPTURE_COUNT) {
            captures[chan] = diff;
        }
        chan++;
    }

}

static void pwmCallback(uint8_t port, captureCompare_t capture)
{
    pwmInputPort_t *pwmInputPort = &pwmInputPorts[port];
    const timerHardware_t *timerHardware = pwmInputPort->timerHardware;

    if (pwmInputPort->state == 0) {
        pwmInputPort->rise = capture;
        pwmInputPort->state = 1;
        pwmICConfig(timerHardware->tim, timerHardware->channel, TIM_ICPolarity_Falling);
    } else {
        pwmInputPort->fall = capture;

        // compute and store capture
        pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
        captures[pwmInputPort->channel] = pwmInputPort->capture;

        // switch state
        pwmInputPort->state = 0;
        pwmICConfig(timerHardware->tim, timerHardware->channel, TIM_ICPolarity_Rising);
    }
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 0x03

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

void pwmInConfig(uint8_t timerIndex, uint8_t channel)
{
    pwmInputPort_t *p = &pwmInputPorts[channel];

    const timerHardware_t *timerHardwarePtr = &(timerHardware[timerIndex]);

    p->channel = channel;
    p->mode = INPUT_MODE_PWM;
    p->timerHardware = timerHardwarePtr;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, PWM_TIMER_PERIOD, PWM_TIMER_MHZ);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, channel, pwmCallback);
}

#define UNUSED_PPM_TIMER_REFERENCE 0
#define FIRST_PWM_PORT 0

void ppmInConfig(uint8_t timerIndex)
{
    pwmInputPort_t *p = &pwmInputPorts[FIRST_PWM_PORT];

    const timerHardware_t *timerHardwarePtr = &(timerHardware[timerIndex]);

    p->mode = INPUT_MODE_PPM;
    p->timerHardware = timerHardwarePtr;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, PPM_TIMER_PERIOD, PWM_TIMER_MHZ);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, UNUSED_PPM_TIMER_REFERENCE, ppmCallback);
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}

