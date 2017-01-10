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

#include "io/motors.h"
#include "io/servos.h"
#include "drivers/timer.h"
#include "drivers/dma.h"

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_BRUSHED,
#ifdef USE_DSHOT
    PWM_TYPE_DSHOT150,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_DSHOT1200,
#endif
    PWM_TYPE_MAX
} motorPwmProtocolTypes_e;

#define PWM_TIMER_MHZ         1

#ifdef USE_DSHOT
#define MAX_DMA_TIMERS        8

#define MOTOR_DSHOT1200_MHZ   24
#define MOTOR_DSHOT600_MHZ    12
#define MOTOR_DSHOT300_MHZ    6
#define MOTOR_DSHOT150_MHZ    3

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       19
#endif

#if defined(STM32F40_41xxx) // must be multiples of timer clock
#define ONESHOT125_TIMER_MHZ  12
#define ONESHOT42_TIMER_MHZ   21
#define MULTISHOT_TIMER_MHZ   84
#define PWM_BRUSHED_TIMER_MHZ 21
#elif defined(STM32F7) // must be multiples of timer clock
#define ONESHOT125_TIMER_MHZ  9
#define ONESHOT42_TIMER_MHZ   27
#define MULTISHOT_TIMER_MHZ   54
#define PWM_BRUSHED_TIMER_MHZ 27
#else
#define ONESHOT125_TIMER_MHZ  8
#define ONESHOT42_TIMER_MHZ   24
#define MULTISHOT_TIMER_MHZ   72
#define PWM_BRUSHED_TIMER_MHZ 24
#endif

#define MOTOR_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */

typedef struct {
    TIM_TypeDef *timer;
    uint16_t timerDmaSources;
} motorDmaTimer_t;

typedef struct {
    ioTag_t ioTag;
    const timerHardware_t *timerHardware;
    uint16_t value;
    uint16_t timerDmaSource;
    volatile bool requestTelemetry;
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    uint32_t dmaBuffer[MOTOR_DMA_BUFFER_SIZE];
#else
    uint8_t dmaBuffer[MOTOR_DMA_BUFFER_SIZE];
#endif
    dmaChannelDescriptor_t* dmaDescriptor;
#if defined(STM32F7)
    TIM_HandleTypeDef TimHandle;
    DMA_HandleTypeDef hdma_tim;
#endif
} motorDmaOutput_t;

motorDmaOutput_t *getMotorDmaOutput(uint8_t index);

extern bool pwmMotorsEnabled;

struct timerHardware_s;
typedef void(*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors
typedef void(*pwmCompleteWriteFuncPtr)(uint8_t motorCount);   // function pointer used after motors are written

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef *tim;
    uint16_t period;
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount);
void servoInit(const servoConfig_t *servoConfig);

void pwmServoConfig(const struct timerHardware_s *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);

#ifdef USE_DSHOT
uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType);
void pwmWriteDigital(uint8_t index, uint16_t value);
void pwmDigitalMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType);
void pwmCompleteDigitalMotorUpdate(uint8_t motorCount);
#endif

void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmShutdownPulsesForAllMotors(uint8_t motorCount);
void pwmCompleteMotorUpdate(uint8_t motorCount);

void pwmWriteServo(uint8_t index, uint16_t value);

pwmOutputPort_t *pwmGetMotors(void);
bool pwmIsSynced(void);
void pwmDisableMotors(void);
void pwmEnableMotors(void);
bool pwmAreMotorsEnabled(void);
