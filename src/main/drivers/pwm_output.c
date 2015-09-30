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

#include "flight/failsafe.h" // FIXME dependency into the main code from a driver

#include "pwm_mapping.h"

#include "pwm_output.h"

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef *tim;
    uint16_t period;
    pwmWriteFuncPtr pwmWritePtr;
} pwmOutputPort_t;

static pwmOutputPort_t pwmOutputPorts[MAX_PWM_OUTPUT_PORTS];

static pwmOutputPort_t *motors[MAX_PWM_MOTORS];

#ifdef USE_SERVOS
static pwmOutputPort_t *servos[MAX_PWM_SERVOS];
#endif

static uint8_t allocatedOutputPortCount = 0;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
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

static pwmOutputPort_t *pwmOutConfig(const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    pwmOutputPort_t *p = &pwmOutputPorts[allocatedOutputPortCount++];

    configTimeBase(timerHardware->tim, period, mhz);
    pwmGPIOConfig(timerHardware->gpio, timerHardware->pin, Mode_AF_PP);


    pwmOCConfig(timerHardware->tim, timerHardware->channel, value);
    if (timerHardware->outputEnable)
        TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);

    switch (timerHardware->channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware->tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware->tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware->tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware->tim->CCR4;
            break;
    }
    p->period = period;
    p->tim = timerHardware->tim;

    return p;
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value - 1000) * motors[index]->period / 1000;
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = value;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (motors[index] && index < MAX_MOTORS)
        motors[index]->pwmWritePtr(index, value);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    uint8_t index;

    for(index = 0; index < motorCount; index++){
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        *motors[index]->ccr = 0;
    }
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    uint8_t index;
    TIM_TypeDef *lastTimerPtr = NULL;

    for(index = 0; index < motorCount; index++){

        // Force the timer to overflow if it's the first motor to output, or if we change timers
        if(motors[index]->tim != lastTimerPtr){
            lastTimerPtr = motors[index]->tim;

            timerForceOverflow(motors[index]->tim);
        }

        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index]->ccr = 0;
    }
}

bool isMotorBrushed(uint16_t motorPwmRate)
{
    return (motorPwmRate > 500);
}

void pwmBrushedMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse)
{
    uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;
    motors[motorIndex] = pwmOutConfig(timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / motorPwmRate, idlePulse);
    motors[motorIndex]->pwmWritePtr = pwmWriteBrushed;
}

void pwmBrushlessMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse)
{
    uint32_t hz = PWM_TIMER_MHZ * 1000000;
    motors[motorIndex] = pwmOutConfig(timerHardware, PWM_TIMER_MHZ, hz / motorPwmRate, idlePulse);
    motors[motorIndex]->pwmWritePtr = pwmWriteStandard;
}

void pwmOneshotMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex)
{
    motors[motorIndex] = pwmOutConfig(timerHardware, ONESHOT125_TIMER_MHZ, 0xFFFF, 0);
    motors[motorIndex]->pwmWritePtr = pwmWriteStandard;
}

#ifdef USE_SERVOS
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse)
{
    servos[servoIndex] = pwmOutConfig(timerHardware, PWM_TIMER_MHZ, 1000000 / servoPwmRate, servoCenterPulse);
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (servos[index] && index < MAX_SERVOS)
        *servos[index]->ccr = value;
}
#endif
