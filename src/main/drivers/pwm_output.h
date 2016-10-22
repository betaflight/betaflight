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

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_BRUSHED,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_DSHOT150,
    PWM_TYPE_MAX
} motorPwmProtocolTypes_e;

#define PWM_TIMER_MHZ         1

#if defined(STM32F40_41xxx) // must be multiples of timer clock
#define ONESHOT125_TIMER_MHZ  12
#define ONESHOT42_TIMER_MHZ   21
#define MULTISHOT_TIMER_MHZ   84
#define PWM_BRUSHED_TIMER_MHZ 21

#define MOTOR_BIT_0     7
#define MOTOR_BIT_1     14
#define MOTOR_BITLENGTH 19

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

#define MOTOR_BIT_0     14
#define MOTOR_BIT_1     29
#define MOTOR_BITLENGTH 39

#endif

#define MOTOR_DMA_BUFFER_SIZE 20 /* resolution + frame reset (4 periods) */

#ifdef USE_DSHOT

typedef enum {
    MOTOR_UPDATE_NONE = 0x0,
    MOTOR_UPDATE_VALUE = 0x1,
    MOTOR_UPDATE_TELEMETRY = 0x2,
} motorUpdateFlags_e;

typedef struct {
    TIM_TypeDef *timer;
    uint16_t timerDmaSources;
} motorDmaTimer_t;
#endif

struct timerHardware_s;
typedef void(*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors
typedef void(*pwmCompleteWriteFuncPtr)(uint8_t motorCount);     // function pointer used after motors are written

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef *tim;
    uint16_t period;
    bool enabled;
    IO_t io;
#ifdef USE_DSHOT
    const timerHardware_t *timerHardware;
    uint16_t value;
    uint16_t timerDmaSource;
    uint32_t dmaBuffer[MOTOR_DMA_BUFFER_SIZE];
#if defined(STM32F7)
    TIM_HandleTypeDef TimHandle;
    uint32_t Channel;
#endif
    volatile uint8_t updateFlags;
#endif
} pwmMotorOutput_t;

#ifdef USE_SERVOS
typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef *tim;
    uint16_t period;
    bool enabled;
    IO_t io;
} pwmServoOutput_t;
#endif 

extern pwmMotorOutput_t motors[];

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount);

#ifdef USE_DSHOT
void pwmWriteValueToDmaBuffer(uint16_t value, uint32_t *buffer, uint8_t flags);
void pwmStartDigitalOutput(void);
void pwmStopDigitalOutput(void);
void pwmDigitalMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType);
#endif

void pwmWriteMotors(const int16_t *value, uint8_t motorCount);
void pwmShutdownPulsesForAllMotors(uint8_t motorCount);

#ifdef USE_SERVOS
void servoInit(const servoConfig_t *servoConfig);
void pwmWriteServo(uint8_t index, uint16_t value);
#endif 

bool pwmIsSynced(void);
void pwmDisableMotors(void);
void pwmEnableMotors(void);

