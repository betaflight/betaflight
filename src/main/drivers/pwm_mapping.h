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

#include "drivers/io_types.h"

#if defined(USE_QUAD_MIXER_ONLY)
#define MAX_PWM_MOTORS  4
#define MAX_PWM_SERVOS  1
#define MAX_MOTORS  4
#define MAX_SERVOS  1

#elif defined(TARGET_MOTOR_COUNT)
#define MAX_PWM_MOTORS TARGET_MOTOR_COUNT
#define MAX_PWM_SERVOS 8
#define MAX_MOTORS  TARGET_MOTOR_COUNT
#define MAX_SERVOS  8

#else
#define MAX_PWM_MOTORS  12
#define MAX_PWM_SERVOS  8
#define MAX_MOTORS  12
#define MAX_SERVOS  8
#endif

#define PWM_TIMER_MHZ 1

#define PULSE_1MS   (1000)      // 1ms pulse width

#define MAX_INPUTS  8

typedef struct rangefinderIOConfig_s {
    ioTag_t triggerTag;
    ioTag_t echoTag;
} rangefinderIOConfig_t;

typedef enum {
    PLATFORM_MULTIROTOR     = 0,
    PLATFORM_AIRPLANE       = 1,
    PLATFORM_HELICOPTER     = 2
} flyingPlatformType_e;

typedef struct drv_pwm_config_s {
    int flyingPlatformType;

    bool enablePWMOutput;
    bool useParallelPWM;
    bool usePPM;
    bool useSerialRx;
    bool useRSSIADC;
    bool useCurrentMeterADC;
    bool useUART2;
    bool useUART3;
    bool useUART6;
    bool useVbat;
    bool useFastPwm;
    bool useSoftSerial;
    bool useLEDStrip;
#ifdef USE_RANGEFINDER
    bool useTriggerRangefinder;
#endif
#ifdef USE_SERVOS
    bool useServoOutputs;
    bool useChannelForwarding;      // configure additional channels as servos
    uint16_t servoPwmRate;
    uint16_t servoCenterPulse;
#endif
    uint8_t pwmProtocolType;
    uint16_t motorPwmRate;
    uint16_t idlePulse;  // PWM value to use when initializing the driver. set this to either PULSE_1MS (regular pwm),
                         // some higher value (used by 3d mode), or 0, for brushed pwm drivers.
    rangefinderIOConfig_t rangefinderIOConfig;
} drv_pwm_config_t;

typedef enum {
    PWM_PF_NONE = 0,
    PWM_PF_MOTOR = (1 << 0),
    PWM_PF_SERVO = (1 << 1),
    PWM_PF_MOTOR_MODE_BRUSHED = (1 << 2),
    PWM_PF_OUTPUT_PROTOCOL_PWM = (1 << 3),
    PWM_PF_OUTPUT_PROTOCOL_FASTPWM = (1 << 4),
    PWM_PF_PPM = (1 << 5),
    PWM_PF_PWM = (1 << 6)
} pwmPortFlags_e;

struct timerHardware_s;
typedef struct pwmPortConfiguration_s {
    uint8_t index;
    pwmPortFlags_e flags;
    const struct timerHardware_s *timerHardware;
} pwmPortConfiguration_t;

typedef struct pwmIOConfiguration_s {
    uint8_t servoCount;
    uint8_t motorCount;
    uint8_t ioCount;
    uint8_t pwmInputCount;
    uint8_t ppmInputCount;
    pwmPortConfiguration_t ioConfigurations[USABLE_TIMER_CHANNEL_COUNT];
} pwmIOConfiguration_t;

// This indexes into the read-only hardware definition structure, timerHardware_t
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    PWM15,
    PWM16,
    PWM17,
    PWM18,
    PWM19,
    PWM20
};

extern const uint16_t multiPPM[];
extern const uint16_t multiPWM[];
extern const uint16_t airPPM[];
extern const uint16_t airPWM[];

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init);
pwmIOConfiguration_t *pwmGetOutputConfiguration(void);
