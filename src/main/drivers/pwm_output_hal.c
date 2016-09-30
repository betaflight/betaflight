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
#include <math.h>

#include "platform.h"

#include "io.h"
#include "timer.h"
#include "pwm_mapping.h"
#include "pwm_output.h"

#define MULTISHOT_5US_PW    (MULTISHOT_TIMER_MHZ * 5)
#define MULTISHOT_20US_MULT (MULTISHOT_TIMER_MHZ * 20 / 1000.0f)


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

static bool pwmMotorsEnabled = true;
static void pwmOCConfig(TIM_TypeDef *tim, uint32_t channel, uint16_t value)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if(Handle == NULL) return;
    
    TIM_OC_InitTypeDef  TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OCNIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
    //HAL_TIM_PWM_Start(Handle, channel);
}

static pwmOutputPort_t *pwmOutConfig(const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    pwmOutputPort_t *p = &pwmOutputPorts[allocatedOutputPortCount++];
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if(Handle == NULL) return p;

    configTimeBase(timerHardware->tim, period, mhz);

    const IO_t io = IOGetByTag(timerHardware->tag);
    IOInit(io, OWNER_MOTOR, RESOURCE_OUTPUT, allocatedOutputPortCount);
    IOConfigGPIOAF(io, IOCFG_AF_PP, timerHardware->alternateFunction);

    pwmOCConfig(timerHardware->tim, timerHardware->channel, value);
    if (timerHardware->output & TIMER_OUTPUT_ENABLED) {
        HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    } else {
        HAL_TIM_PWM_Stop(Handle, timerHardware->channel);
    }
    HAL_TIM_Base_Start(Handle);

    switch (timerHardware->channel) {
        case TIM_CHANNEL_1:
            p->ccr = &timerHardware->tim->CCR1;
            break;
        case TIM_CHANNEL_2:
            p->ccr = &timerHardware->tim->CCR2;
            break;
        case TIM_CHANNEL_3:
            p->ccr = &timerHardware->tim->CCR3;
            break;
        case TIM_CHANNEL_4:
            p->ccr = &timerHardware->tim->CCR4;
            break;
    }
    p->period = period;
    p->tim = timerHardware->tim;

    *p->ccr = 0;
   
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

static void pwmWriteOneShot125(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = lrintf((float)(value * ONESHOT125_TIMER_MHZ/8.0f));
}

static void pwmWriteOneShot42(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = lrintf((float)(value * ONESHOT42_TIMER_MHZ/24.0f));
}

static void pwmWriteMultiShot(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = lrintf(((float)(value-1000) * MULTISHOT_20US_MULT) + MULTISHOT_5US_PW);
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (motors[index] && index < MAX_MOTORS && pwmMotorsEnabled) {
        motors[index]->pwmWritePtr(index, value);
    }
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        *motors[index]->ccr = 0;
    }
}

void pwmDisableMotors(void)
{
    pwmMotorsEnabled = false;
}

void pwmEnableMotors(void)
{
    pwmMotorsEnabled = true;
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        bool overflowed = false;
        // If we have not already overflowed this timer
        for (int j = 0; j < index; j++) {
            if (motors[j]->tim == motors[index]->tim) {
                overflowed = true;
                break;
            }
        }
        if (!overflowed) {
            timerForceOverflow(motors[index]->tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index]->ccr = 0;
    }
}

void pwmBrushedMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate)
{
    const uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;
    motors[motorIndex] = pwmOutConfig(timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / motorPwmRate, 0);
    motors[motorIndex]->pwmWritePtr = pwmWriteBrushed;
}

void pwmBrushlessMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse)
{
    const uint32_t hz = PWM_TIMER_MHZ * 1000000;
    motors[motorIndex] = pwmOutConfig(timerHardware, PWM_TIMER_MHZ, hz / motorPwmRate, idlePulse);
    motors[motorIndex]->pwmWritePtr = pwmWriteStandard;
}

void pwmFastPwmMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse, uint8_t fastPwmProtocolType)
{
    uint32_t timerMhzCounter;
    pwmWriteFuncPtr pwmWritePtr;

    switch (fastPwmProtocolType) {
    default:
    case (PWM_TYPE_ONESHOT125):
        timerMhzCounter = ONESHOT125_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot125;
        break;
    case (PWM_TYPE_ONESHOT42):
        timerMhzCounter = ONESHOT42_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot42;
        break;
    case (PWM_TYPE_MULTISHOT):
        timerMhzCounter = MULTISHOT_TIMER_MHZ;
        pwmWritePtr = pwmWriteMultiShot;
        break;
    }

    if (motorPwmRate > 0) {
        const uint32_t hz = timerMhzCounter * 1000000;
        motors[motorIndex] = pwmOutConfig(timerHardware, timerMhzCounter, hz / motorPwmRate, idlePulse);
    } else {
        motors[motorIndex] = pwmOutConfig(timerHardware, timerMhzCounter, 0xFFFF, 0);
    }
    motors[motorIndex]->pwmWritePtr = pwmWritePtr;
}

#ifdef USE_SERVOS
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse)
{
    servos[servoIndex] = pwmOutConfig(timerHardware, PWM_TIMER_MHZ, 1000000 / servoPwmRate, servoCenterPulse);
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (servos[index] && index < MAX_SERVOS) {
        *servos[index]->ccr = value;
    }
}
#endif
