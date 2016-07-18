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
#include <string.h>
#include <stdlib.h>

#include <platform.h>

#include "config/parameter_group.h"

#include "gpio.h"
#include "timer.h"
#include "drivers/bus_i2c.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

#ifdef STM32F10X
#include "serial_uart_stm32f10x.h"
#endif
#ifdef STM32F303xC
#include "serial_uart_stm32f30x.h"
#endif

void pwmBrushedMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmBrushlessMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmOneshotMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex);
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);

/*
    Configuration maps

    Note: this documentation is only valid for STM32F10x, for STM32F30x please read the code itself.

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

enum {
    MAP_TO_PPM_INPUT = 1,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
};

#if defined(NAZE) || defined(OLIMEXINO) || defined(NAZE32PRO) || defined(STM32F3DISCOVERY) || defined(EUSTM32F103RC) || defined(PORT103R) || defined(PORT103V)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     // Swap to servo if needed
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #5
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #8
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};
#endif

#ifdef CC3D
static const uint16_t multiPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #2
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #4
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #3
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};
#endif

#ifdef CJMCU
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),        // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),        // Swap to servo if needed
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),         // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),        // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),        // Swap to servo if needed
    0xFFFF
};
#endif

#if defined(SPARKY) || defined(ALIENFLIGHTF3)
static const uint16_t multiPPM[] = {
    PWM11 | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};
#endif

#ifdef SPRACINGF3
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #10
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), // server #6
    0xFFFF
};
#endif

#ifdef SPRACINGF3EVO
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};
#endif

#if defined(MOTOLAB)
static const uint16_t multiPPM[] = {
    PWM9  | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};
#endif

#ifdef SPRACINGF3MINI
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM1  | (MAP_TO_PWM_INPUT    << 8), // Use PWM input for AUX1, PWM6-9 get remapped by conditional code.
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #8
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM1  | (MAP_TO_PWM_INPUT    << 8), // Use PWM input for AUX1, PWM6-9 get remapped by conditional code.
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #8
    0xFFFF
};
#endif

#ifdef RCEXPLORERF3
static const uint16_t multiPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),    // PPM input
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

static const uint16_t multiPWM[] = {
        PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
        PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
        PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
        PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
        PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),    // PPM input
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};
#endif

static const uint16_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static pwmIOConfiguration_t pwmIOConfiguration;

pwmIOConfiguration_t *pwmGetOutputConfiguration(void){
    return &pwmIOConfiguration;
}

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint16_t *setup;

    int channelIndex = 0;


    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]  PWM mappings are used for RX_MSP.
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM || init->useSerialRx)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < USABLE_TIMER_CHANNEL_COUNT && setup[i] != 0xFFFF; i++) {
        uint8_t timerIndex = setup[i] & 0x00FF;
        uint8_t type = (setup[i] & 0xFF00) >> 8;

        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2)
            continue;
#endif

        // skip UART ports
#ifdef USE_UART2
        if (init->useUART2 && timerHardwarePtr->gpio == UART2_GPIO && (timerHardwarePtr->pin == UART2_TX_PIN || timerHardwarePtr->pin == UART2_RX_PIN))
            continue;
#endif

#ifdef USE_UART3
        if (init->useUART3 && timerHardwarePtr->gpio == UART3_GPIO && (timerHardwarePtr->pin == UART3_TX_PIN || timerHardwarePtr->pin == UART3_RX_PIN))
            continue;
#endif

#ifdef USE_UART4
        if (init->useUART4 && timerHardwarePtr->gpio == UART4_GPIO && (timerHardwarePtr->pin == UART4_TX_PIN || timerHardwarePtr->pin == UART4_RX_PIN))
            continue;
#endif

#ifdef USE_UART5
        if (init->useUART5 &&
            (
                (timerHardwarePtr->gpio == UART5_GPIO_TX && timerHardwarePtr->pin == UART5_TX_PIN)
                || (timerHardwarePtr->gpio == UART5_GPIO_RX && timerHardwarePtr->pin == UART5_RX_PIN)
            )
        )
            continue;
#endif

#ifdef STM32F10X
        // skip I2C ports if device 1 is selected
        if (I2C_DEVICE == I2CDEV_1 && timerHardwarePtr->gpio == GPIOB && (timerHardwarePtr->pin == Pin_6 || timerHardwarePtr->pin == Pin_7))
            continue;
#endif

#ifdef SOFTSERIAL_1_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_1_TIMER)
            continue;
#endif
#ifdef SOFTSERIAL_2_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_2_TIMER)
            continue;
#endif

#ifdef LED_STRIP_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == LED_STRIP_TIMER)
                continue;
#if defined(STM32F303xC) && defined(WS2811_GPIO) && defined(WS2811_PIN_SOURCE)
            if (timerHardwarePtr->gpio == WS2811_GPIO && timerHardwarePtr->gpioPinSource == WS2811_PIN_SOURCE)
                continue;
#endif
        }

#endif

#ifdef VBAT_ADC_GPIO
        if (init->useVbat && timerHardwarePtr->gpio == VBAT_ADC_GPIO && timerHardwarePtr->pin == VBAT_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef RSSI_ADC_GPIO
        if (init->useRSSIADC && timerHardwarePtr->gpio == RSSI_ADC_GPIO && timerHardwarePtr->pin == RSSI_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_GPIO
        if (init->useCurrentMeterADC && timerHardwarePtr->gpio == CURRENT_METER_ADC_GPIO && timerHardwarePtr->pin == CURRENT_METER_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef SONAR
        if (init->sonarGPIOConfig && timerHardwarePtr->gpio == init->sonarGPIOConfig->gpio &&
            (
                timerHardwarePtr->pin == init->sonarGPIOConfig->triggerPin ||
                timerHardwarePtr->pin == init->sonarGPIOConfig->echoPin
            )
        ) {
            continue;
        }
#endif

        // hacks to allow current functionality

#if defined(SPRACINGF3MINI)
            // remap PWM1, 6-9 as PWM input when parallel PWM is used (for AUX1 and RC1-4, respectively)
            if (init->useParallelPWM && (timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9 || timerIndex == PWM1)) {
                type = MAP_TO_PWM_INPUT;
            }
#endif

        if (type == MAP_TO_PWM_INPUT && !init->useParallelPWM)
            continue;

        if (type == MAP_TO_PPM_INPUT && !init->usePPM)
            continue;

#ifdef USE_SERVOS
        if (init->useServos && !init->airplane) {
#if defined(NAZE)
            // remap PWM9+10 as servos
            if ((timerIndex == PWM9 || timerIndex == PWM10) && timerHardwarePtr->tim == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
            // remap PWM1+2 as servos
            if ((timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9) && timerHardwarePtr->tim == TIM2)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(CC3D)
            // remap 10 as servo
            if (timerIndex == PWM10 && timerHardwarePtr->tim == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPARKY)
            // remap PWM1+2 as servos
            if ((timerIndex == PWM1 || timerIndex == PWM2) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPRACINGF3)
            // remap PWM15+16 as servos
            if ((timerIndex == PWM15 || timerIndex == PWM16) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPRACINGF3MINI)
            // remap PWM6+7 as servos
            if ((timerIndex == PWM6 || timerIndex == PWM7) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(RCEXPLORERF3)
            if (timerIndex == PWM2)
            {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(NAZE32PRO) || (defined(STM32F3DISCOVERY) && !defined(CHEBUZZF3))
            // remap PWM 5+6 or 9+10 as servos - softserial pin pairs require timer ports that use the same timer
            if (init->useSoftSerial) {
                if (timerIndex == PWM5 || timerIndex == PWM6)
                    type = MAP_TO_SERVO_OUTPUT;
            } else {
                if (timerIndex == PWM9 || timerIndex == PWM10)
                    type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(MOTOLAB)
            // remap PWM 7+8 as servos
            if (timerIndex == PWM7 || timerIndex == PWM8)
                type = MAP_TO_SERVO_OUTPUT;
#endif
        }

        if (init->useChannelForwarding && !init->airplane) {
#if defined(NAZE) && defined(LED_STRIP_TIMER)
            // if LED strip is active, PWM5-8 are unavailable, so map AUX1+AUX2 to PWM13+PWM14
            if (init->useLEDStrip) { 
                if (timerIndex >= PWM13 && timerIndex <= PWM14) {
                  type = MAP_TO_SERVO_OUTPUT;
                }
            } else
#endif

#if defined(SPRACINGF3) || defined(NAZE)
                // remap PWM5..8 as servos when used in extended servo mode
                if (timerIndex >= PWM5 && timerIndex <= PWM8)
                    type = MAP_TO_SERVO_OUTPUT;
#endif
        }

#endif // USE_SERVOS

#ifdef CC3D
        if (init->useParallelPWM) {
            // Skip PWM inputs that conflict with timers used outputs.
            if ((type == MAP_TO_SERVO_OUTPUT || type == MAP_TO_MOTOR_OUTPUT) && (timerHardwarePtr->tim == TIM2 || timerHardwarePtr->tim == TIM3)) {
                continue;
            }
            if (type == MAP_TO_PWM_INPUT && timerHardwarePtr->tim == TIM4) {
                continue;
            }

        }
#endif

        if (type == MAP_TO_PPM_INPUT) {
#if defined(SPARKY) || defined(ALIENFLIGHTF3)
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM2);
            }
#endif
            ppmInConfig(timerHardwarePtr);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;
        } else if (type == MAP_TO_PWM_INPUT) {
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
#ifdef CC3D
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)){
            	// Skip it if it would cause PPM capture timer to be reconfigured or manually overflowed
            	if (timerHardwarePtr->tim == TIM2)
            		continue;
            }
#endif
            if (init->useOneshot) {

                pwmOneshotMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_ONESHOT|PWM_PF_OUTPUT_PROTOCOL_PWM;

            } else if (isMotorBrushed(init->motorPwmRate)) {

                pwmBrushedMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;

            } else {

                pwmBrushlessMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            }

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.motorCount++;

        } else if (type == MAP_TO_SERVO_OUTPUT) {
#ifdef USE_SERVOS
            pwmServoConfig(timerHardwarePtr, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse);

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.servoCount++;
#endif
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}
