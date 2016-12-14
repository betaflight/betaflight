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

static pwmWriteFuncPtr pwmWritePtr;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmCompleteWriteFuncPtr pwmCompleteWritePtr = NULL;

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];
#endif

bool pwmMotorsEnabled = false;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if(Handle == NULL) return;

    TIM_OC_InitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
    //HAL_TIM_PWM_Start(Handle, channel);
}

static void pwmOutConfig(pwmOutputPort_t *port, const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if(Handle == NULL) return;

    configTimeBase(timerHardware->tim, period, mhz);
    pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);

    HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    HAL_TIM_Base_Start(Handle);

    switch (timerHardware->channel) {
    case TIM_CHANNEL_1:
        port->ccr = &timerHardware->tim->CCR1;
        break;
    case TIM_CHANNEL_2:
        port->ccr = &timerHardware->tim->CCR2;
        break;
    case TIM_CHANNEL_3:
        port->ccr = &timerHardware->tim->CCR3;
        break;
    case TIM_CHANNEL_4:
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
    pwmWritePtr(index, value);
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

bool pwmAreMotorsEnabled(void)
{
    return pwmMotorsEnabled;
}

static void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
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

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
    if (pwmCompleteWritePtr) {
        pwmCompleteWritePtr(motorCount);
    }
}

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    uint32_t timerMhzCounter;
    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;
    bool isDigital = false;

    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        timerMhzCounter = ONESHOT125_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot125;
        break;
    case PWM_TYPE_ONESHOT42:
        timerMhzCounter = ONESHOT42_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot42;
        break;
    case PWM_TYPE_MULTISHOT:
        timerMhzCounter = MULTISHOT_TIMER_MHZ;
        pwmWritePtr = pwmWriteMultiShot;
        break;
    case PWM_TYPE_BRUSHED:
        timerMhzCounter = PWM_BRUSHED_TIMER_MHZ;
        pwmWritePtr = pwmWriteBrushed;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    case PWM_TYPE_STANDARD:
        timerMhzCounter = PWM_TIMER_MHZ;
        pwmWritePtr = pwmWriteStandard;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        pwmWritePtr = pwmWriteDigital;
        pwmCompleteWritePtr = pwmCompleteDigitalMotorUpdate;
        isDigital = true;
        break;
#endif
    }

    if (!useUnsyncedPwm && !isDigital) {
        pwmCompleteWritePtr = pwmCompleteOneshotMotorUpdate;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];

        if (!tag) {
            break;
        }

        const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);

        if (timerHardware == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }

#ifdef USE_DSHOT
        if (isDigital) {
            pwmDigitalMotorHardwareConfig(timerHardware, motorIndex, motorConfig->motorPwmProtocol);
            motors[motorIndex].enabled = true;
            continue;
        }
#endif
        motors[motorIndex].io = IOGetByTag(tag);

        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        //IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        if (useUnsyncedPwm) {
            const uint32_t hz = timerMhzCounter * 1000000;
            pwmOutConfig(&motors[motorIndex], timerHardware, timerMhzCounter, hz / motorConfig->motorPwmProtocol, idlePulse);
        } else {
            pwmOutConfig(&motors[motorIndex], timerHardware, timerMhzCounter, 0xFFFF, 0);
        }
        motors[motorIndex].enabled = true;
    }
    pwmMotorsEnabled = true;
}

pwmOutputPort_t *pwmGetMotors(void)
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

void servoInit(const servoConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag) {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));
        //IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);

        const timerHardware_t *timer = timerGetByTag(tag, TIM_USE_ANY);
        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);

        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }

        pwmOutConfig(&servos[servoIndex], timer, PWM_TIMER_MHZ, 1000000 / servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
        servos[servoIndex].enabled = true;
    }
}

#endif
