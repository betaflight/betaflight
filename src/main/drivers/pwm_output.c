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
#include "pwm_output.h"

#define MULTISHOT_5US_PW    (MULTISHOT_TIMER_MHZ * 5)
#define MULTISHOT_20US_MULT (MULTISHOT_TIMER_MHZ * 20 / 1000.0f)

static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];
#endif

static bool pwmMotorsEnabled = true;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    }
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
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

static void pwmOutConfig(pwmOutputPort_t *port, const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    configTimeBase(timerHardware->tim, period, mhz);
    pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);

    if (timerHardware->output & TIMER_OUTPUT_ENABLED) {
        TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    }
    TIM_Cmd(timerHardware->tim, ENABLE);

    switch (timerHardware->channel) {
    case TIM_Channel_1:
        port->ccr = &timerHardware->tim->CCR1;
        break;
    case TIM_Channel_2:
        port->ccr = &timerHardware->tim->CCR2;
        break;
    case TIM_Channel_3:
        port->ccr = &timerHardware->tim->CCR3;
        break;
    case TIM_Channel_4:
        port->ccr = &timerHardware->tim->CCR4;
        break;
    }
    port->period = period;
    port->tim = timerHardware->tim;

    *port->ccr = 0;
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index].ccr = (value - 1000) * motors[index].period / 1000;
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index].ccr = value;
}

static void pwmWriteOneShot125(uint8_t index, uint16_t value)
{
    *motors[index].ccr = lrintf((float)(value * ONESHOT125_TIMER_MHZ/8.0f));
}

static void pwmWriteOneShot42(uint8_t index, uint16_t value)
{
    *motors[index].ccr = lrintf((float)(value * ONESHOT42_TIMER_MHZ/24.0f));
}

static void pwmWriteMultiShot(uint8_t index, uint16_t value)
{
    *motors[index].ccr = lrintf(((float)(value-1000) * MULTISHOT_20US_MULT) + MULTISHOT_5US_PW);
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < MAX_SUPPORTED_MOTORS && pwmMotorsEnabled && motors[index].pwmWritePtr) {
        motors[index].pwmWritePtr(index, value);
    }
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        *motors[index].ccr = 0;
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
            if (motors[j].tim == motors[index].tim) {
                overflowed = true;
                break;
            }
        }
        if (!overflowed) {
            timerForceOverflow(motors[index].tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].ccr = 0;
    }
}

void motorInit(escAndServoConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    uint32_t timerMhzCounter;
    pwmWriteFuncPtr pwmWritePtr;
        
    switch (motorConfig->motor_pwm_protocol) {
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
        case (PWM_TYPE_BRUSHED):
            timerMhzCounter = PWM_BRUSHED_TIMER_MHZ;
            pwmWritePtr = pwmWriteBrushed;
            motorConfig->use_unsyncedPwm = true;
            idlePulse = 0;
            break;
        case (PWM_TYPE_CONVENTIONAL):
            timerMhzCounter = PWM_TIMER_MHZ;
            pwmWritePtr = pwmWriteStandard;
            motorConfig->use_unsyncedPwm = true;
            idlePulse = 0;
            break;
    }
    
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        ioTag_t tag = motorConfig->motorTags[motorIndex];
        
        if (DEFIO_TAG_ISEMPTY(tag)) {
            break;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_OUTPUT, RESOURCE_INDEX(motorIndex));
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
        
        const timerHardware_t *timer = timerGetByTag(tag, TIMER_OUTPUT_ENABLED);
        
        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }
        
        motors[motorIndex].pwmWritePtr = pwmWritePtr;
        if (motorConfig->use_unsyncedPwm) {
            const uint32_t hz = timerMhzCounter * 1000000;
            pwmOutConfig(&motors[motorIndex], timer, timerMhzCounter, hz / motorConfig->motor_pwm_rate, idlePulse);
        } else {
            pwmOutConfig(&motors[motorIndex], timer, timerMhzCounter, 0xFFFF, 0);
        }
        motors[motorIndex].enabled = true;
    }
}

pwmOutputPort_t *pwmGetMotors()
{
    return motors;
}

#ifdef USE_SERVOS
void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].ccr) {
        *servos[index].ccr = value;
    }
}

void servoInit(escAndServoConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        ioTag_t tag = servoConfig->servoTags[servoIndex];
        
        if (DEFIO_TAG_ISEMPTY(tag)) {
            break;
        }
        
        servos[servoIndex].io = IOGetByTag(tag);
        
        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_OUTPUT, RESOURCE_INDEX(servoIndex));
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
        
        const timerHardware_t *timer = timerGetByTag(tag, TIMER_OUTPUT_ENABLED);
        
        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }
        
        pwmOutConfig(&servos[servoIndex], timer, PWM_TIMER_MHZ, 1000000 / servoConfig->servo_pwm_rate, servoConfig->servoCenterPulse);
        servos[servoIndex].enabled = true;
    }
}

#endif
