
#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>

#include "platform.h"

#include "gpio_common.h"
#include "timer_common.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"
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
    TYPE_IP = 0x10,
    TYPE_IW = 0x20,
    TYPE_M = 0x40,
    TYPE_S = 0x80
};

static const uint8_t multiPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // Swap to servo if needed
    PWM10 | TYPE_M,     // Swap to servo if needed
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM5 | TYPE_M,      // Swap to servo if needed
    PWM6 | TYPE_M,      // Swap to servo if needed
    PWM7 | TYPE_M,      // Swap to servo if needed
    PWM8 | TYPE_M,      // Swap to servo if needed
    0xFF
};

static const uint8_t multiPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | TYPE_M,     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | TYPE_M,     // motor #1 or #3
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,     // motor #4 or #6
    0xFF
};

static const uint8_t airPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_S,     // servo #1
    PWM12 | TYPE_S,
    PWM13 | TYPE_S,
    PWM14 | TYPE_S,     // servo #4
    PWM5 | TYPE_S,      // servo #5
    PWM6 | TYPE_S,
    PWM7 | TYPE_S,
    PWM8 | TYPE_S,      // servo #8
    0xFF
};

static const uint8_t airPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_S,     // servo #1
    PWM12 | TYPE_S,
    PWM13 | TYPE_S,
    PWM14 | TYPE_S,     // servo #4
    0xFF
};

static const uint8_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

void pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint8_t *setup;

    int channelIndex = 0;
    int servoIndex = 0;
    int motorIndex = 0;

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        uint8_t timerIndex = setup[i] & 0x0F;
        uint8_t mask = setup[i] & 0xF0;

        if (setup[i] == 0xFF) // terminator
            break;

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2)
            continue;
#endif

#ifdef STM32F10X_MD
        // skip UART ports for GPS
        if (init->useUART && (timerIndex == PWM3 || timerIndex == PWM4))
            continue;
#endif

#ifdef STM32F10X_MD
        // skip softSerial ports
        if (init->useSoftSerial && (timerIndex == PWM5 || timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8))
            continue;
#endif

#ifdef CHEBUZZF3
        // skip softSerial ports
        // PWM4 can no-longer be used since it uses the same timer as PWM5 and PWM6
        if (init->useSoftSerial && (timerIndex == PWM4 || timerIndex == PWM5 || timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8))
            continue;
#endif

#ifdef STM32F3DISCOVERY
        // skip softSerial ports
        if (init->useSoftSerial && (timerIndex == PWM9 || timerIndex == PWM10 || timerIndex == PWM11 || timerIndex == PWM12))
            continue;
#endif

#ifdef STM32F10X_MD
        // skip ADC for powerMeter if configured
        // See FIXME where init->adcChannel is initialised
        if (init->adcChannel && (init->adcChannel == timerIndex))
            continue;
#endif

        // hacks to allow current functionality
        if (mask & (TYPE_IP | TYPE_IW) && !init->enableInput)
            mask = 0;

        if (init->useServos && !init->airplane) {
#if defined(STM32F10X_MD) || defined(CHEBUZZF3)
            // remap PWM9+10 as servos
            if (timerIndex == PWM9 || timerIndex == PWM10)
                mask = TYPE_S;
#endif

#if (defined(STM32F303xC) || defined(STM32F3DISCOVERY)) && !defined(CHEBUZZF3)
            // remap PWM 5+6 or 9+10 as servos - softserial pin pairs require timer ports that use the same timer
            if (init->useSoftSerial) {
                if (timerIndex == PWM5 || timerIndex == PWM6)
                    mask = TYPE_S;
            } else {
                if (timerIndex == PWM9 || timerIndex == PWM10)
                    mask = TYPE_S;
            }
#endif
        }

        if (init->extraServos && !init->airplane) {
            // remap PWM5..8 as servos when used in extended servo mode
            if (timerIndex >= PWM5 && timerIndex <= PWM8)
                mask = TYPE_S;
        }

        if (mask & TYPE_IP) {
            ppmInConfig(timerIndex);
        } else if (mask & TYPE_IW) {
            pwmInConfig(timerIndex, channelIndex);
            channelIndex++;
        } else if (mask & TYPE_M) {


            if (init->motorPwmRate > 500) {
            	pwmBrushedMotorConfig(&timerHardware[timerIndex], motorIndex, init->motorPwmRate, init->idlePulse);
            } else {
            	pwmBrushlessMotorConfig(&timerHardware[timerIndex], motorIndex, init->motorPwmRate, init->idlePulse);
            }
            motorIndex++;
        } else if (mask & TYPE_S) {
            pwmServoConfig(&timerHardware[timerIndex], servoIndex, init->servoPwmRate, init->servoCenterPulse);
            servoIndex++;
        }
    }
}
