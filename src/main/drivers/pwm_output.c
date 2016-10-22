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

pwmMotorOutput_t motors[MAX_SUPPORTED_MOTORS];
static pwmCompleteWriteFuncPtr pwmCompleteWritePtr = NULL;
static pwmWriteFuncPtr pwmWritePtr = NULL;

#ifdef USE_SERVOS
static pwmServoOutput_t servos[MAX_SUPPORTED_SERVOS];
#endif

static bool pwmMotorsEnabled = true;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

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

static void pwmAnalogueMotorHardwareConfig(pwmMotorOutput_t *motor, const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    configTimeBase(timerHardware->tim, period, mhz);
    pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);

    if (timerHardware->output & TIMER_OUTPUT_ENABLED) {
        TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    }
    TIM_Cmd(timerHardware->tim, ENABLE);

    switch (timerHardware->channel) {
    case TIM_Channel_1:
        motor->ccr = &timerHardware->tim->CCR1;
        break;
    case TIM_Channel_2:
        motor->ccr = &timerHardware->tim->CCR2;
        break;
    case TIM_Channel_3:
        motor->ccr = &timerHardware->tim->CCR3;
        break;
    case TIM_Channel_4:
        motor->ccr = &timerHardware->tim->CCR4;
        break;
    }
    motor->period = period;
    motor->tim = timerHardware->tim;

    *motor->ccr = 0;
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

#ifdef USE_DSHOT
static void pwmWriteDigital(uint8_t index, uint16_t value)
{
    pwmMotorOutput_t * const motor = &motors[index];

    motor->value = value;
    motor->updateFlags |= MOTOR_UPDATE_VALUE;
}

void pwmCompleteDigitalMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);

}

void pwmWriteValueToDmaBuffer(uint16_t value, uint32_t *buffer, uint8_t flags)
{
    buffer[0]  = (value & 0x400) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[1]  = (value & 0x200) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[2]  = (value & 0x100) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[3]  = (value & 0x80)  ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[4]  = (value & 0x40)  ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[5]  = (value & 0x20)  ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[6]  = (value & 0x10)  ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[7]  = (value & 0x8)   ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[8]  = (value & 0x4)   ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[9]  = (value & 0x2)   ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[10] = (value & 0x1)   ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[11] = (flags & MOTOR_UPDATE_TELEMETRY) ? MOTOR_BIT_1 : MOTOR_BIT_0; 
        
        /* check sum */
    buffer[12] = (value & 0x400) ^ (value & 0x40) ^ (value & 0x4) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[13] = (value & 0x200) ^ (value & 0x20) ^ (value & 0x2) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[14] = (value & 0x100) ^ (value & 0x10) ^ (value & 0x1) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    buffer[15] = (value & 0x80)  ^ (value & 0x8)  ^ ((flags & MOTOR_UPDATE_TELEMETRY) ? 0x1 : 0x0) ? MOTOR_BIT_1 : MOTOR_BIT_0;
}   
#endif

void pwmWriteMotors(const int16_t *value, uint8_t motorCount)
{
    if (motorCount > MAX_SUPPORTED_MOTORS || !pwmMotorsEnabled || !pwmWritePtr) {
        return;
    }
    
    for (uint8_t i = 0; i < motorCount; i++) {
        pwmWritePtr(i, value[i]);    
    }
    
    if (pwmCompleteWritePtr) {
        pwmCompleteWritePtr(motorCount);
    }
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
#ifdef USE_DSHOT
    if (pwmWritePtr == pwmWriteDigital) {
        pwmStopDigitalOutput();    
        return;
    }
#endif

    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].ccr) {
            *motors[index].ccr = 0;
        }
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

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    uint32_t timerMhzCounter;
    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;
    
    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        timerMhzCounter = ONESHOT125_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot125;
        if (!useUnsyncedPwm) {
            pwmCompleteWritePtr = pwmCompleteOneshotMotorUpdate;
        }
        break;
    case PWM_TYPE_ONESHOT42:
        timerMhzCounter = ONESHOT42_TIMER_MHZ;
        pwmWritePtr = pwmWriteOneShot42;
        if (!useUnsyncedPwm) {
            pwmCompleteWritePtr = pwmCompleteOneshotMotorUpdate;
        }
        break;
    case PWM_TYPE_MULTISHOT:
        timerMhzCounter = MULTISHOT_TIMER_MHZ;
        pwmWritePtr = pwmWriteMultiShot;
        if (!useUnsyncedPwm) {
            pwmCompleteWritePtr = pwmCompleteOneshotMotorUpdate;
        }
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
    case PWM_TYPE_DSHOT150:
        pwmCompleteWritePtr = pwmCompleteDigitalMotorUpdate;
        pwmWritePtr = pwmWriteDigital;
        break;
#endif
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        
        if (!tag) {
            break;
        }

        const timerHardware_t *timerHardware = timerGetByTag(tag, TIMER_OUTPUT_ENABLED);
        
        if (timerHardware == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }

        motors[motorIndex].enabled = true;

#ifdef USE_DSHOT
        if (pwmWritePtr == pwmWriteDigital) {
            pwmDigitalMotorHardwareConfig(timerHardware, motorIndex, motorConfig->motorPwmProtocol);
            continue;
        }
#endif
        motors[motorIndex].io = IOGetByTag(tag);
        
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_OUTPUT, RESOURCE_INDEX(motorIndex));
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
        
        if (useUnsyncedPwm) {
            const uint32_t hz = timerMhzCounter * 1000000;
            pwmAnalogueMotorHardwareConfig(&motors[motorIndex], timerHardware, timerMhzCounter, hz / motorConfig->motorPwmProtocol, idlePulse);
        } else {
            pwmAnalogueMotorHardwareConfig(&motors[motorIndex], timerHardware, timerMhzCounter, 0xFFFF, 0);
        }
    }
    
#ifdef USE_DSHOT
    if (pwmWritePtr == pwmWriteDigital) {
        pwmStartDigitalOutput();
    }
#endif
}

bool pwmIsSynced(void) 
{
    return pwmCompleteWritePtr != NULL;
}

pwmMotorOutput_t *pwmGetMotors(void)
{
    return motors;
}

#ifdef USE_SERVOS

static void pwmAnalogueServoHardwareConfig(pwmServoOutput_t *servo, const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
    configTimeBase(timerHardware->tim, period, mhz);
    pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);

    if (timerHardware->output & TIMER_OUTPUT_ENABLED) {
        TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    }
    TIM_Cmd(timerHardware->tim, ENABLE);

    switch (timerHardware->channel) {
        case TIM_Channel_1:
            servo->ccr = &timerHardware->tim->CCR1;
            break;
        case TIM_Channel_2:
            servo->ccr = &timerHardware->tim->CCR2;
            break;
        case TIM_Channel_3:
            servo->ccr = &timerHardware->tim->CCR3;
            break;
        case TIM_Channel_4:
            servo->ccr = &timerHardware->tim->CCR4;
            break;
    }
    servo->period = period;
    servo->tim = timerHardware->tim;

    *servo->ccr = 0;
}

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
        
        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_OUTPUT, RESOURCE_INDEX(servoIndex));
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
        
        const timerHardware_t *timer = timerGetByTag(tag, TIMER_OUTPUT_ENABLED);
        
        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }
        
        pwmAnalogueServoHardwareConfig(&servos[servoIndex], timer, PWM_TIMER_MHZ, 1000000 / servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
        servos[servoIndex].enabled = true;
    }
}

#endif
