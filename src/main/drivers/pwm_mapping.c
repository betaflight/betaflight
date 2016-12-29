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

#include "platform.h"

#include "gpio.h"
#include "io.h"
#include "io_impl.h"
#include "timer.h"

#include "logging.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

void pwmMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse, motorPwmProtocolTypes_e proto, bool enableOutput);
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse, bool enableOutput);

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

const uint16_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static pwmIOConfiguration_t pwmIOConfiguration;

pwmIOConfiguration_t *pwmGetOutputConfiguration(void)
{
    return &pwmIOConfiguration;
}

bool CheckGPIOPin(ioTag_t tag, GPIO_TypeDef *gpio, uint16_t pin)
{
    return IO_GPIOBYTAG(tag) == gpio && IO_PINBYTAG(tag) == pin;
}

bool CheckGPIOPinSource(ioTag_t tag, GPIO_TypeDef *gpio, uint16_t pin)
{
    return IO_GPIOBYTAG(tag) == gpio && IO_GPIO_PinSource(IOGetByTag(tag)) == pin;
}

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
#ifndef SKIP_RX_PWM_PPM
    int channelIndex = 0;
#endif

    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    int i = 0;
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM || init->useSerialRx)
        i++; // next index is for PPM

    const uint16_t *setup = hardwareMaps[i];

    for (i = 0; i < USABLE_TIMER_CHANNEL_COUNT && setup[i] != 0xFFFF; i++) {
        uint8_t timerIndex = setup[i] & 0x00FF;
        uint8_t type = (setup[i] & 0xFF00) >> 8;

        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef STM32F10X
        // skip UART2 ports
        if (init->useUART2 && (timerHardwarePtr->tag == IO_TAG(PA2) || timerHardwarePtr->tag == IO_TAG(PA3))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(STM32F303xC) && defined(USE_UART3)
        // skip UART3 ports (PB10/PB11)
        if (init->useUART3 && (timerHardwarePtr->tag == IO_TAG(UART3_TX_PIN) || timerHardwarePtr->tag == IO_TAG(UART3_RX_PIN))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef SOFTSERIAL_1_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_1_TIMER) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif
#ifdef SOFTSERIAL_2_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_2_TIMER) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef WS2811_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == WS2811_TIMER) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#if defined(STM32F303xC) && defined(WS2811_PIN)
            if (timerHardwarePtr->tag == IO_TAG(WS2811_PIN)) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#endif
        }

#endif

#ifdef VBAT_ADC_PIN
        if (init->useVbat && timerHardwarePtr->tag == IO_TAG(VBAT_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef RSSI_ADC_PIN
        if (init->useRSSIADC && timerHardwarePtr->tag == IO_TAG(RSSI_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_PIN
        if (init->useCurrentMeterADC && timerHardwarePtr->tag == IO_TAG(CURRENT_METER_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef SONAR
        if (init->useSonar &&
            (
                timerHardwarePtr->tag == init->sonarIOConfig.triggerTag ||
                timerHardwarePtr->tag == init->sonarIOConfig.echoTag
            )) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

        // hacks to allow current functionality
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

#if defined(DOGE)
            // remap outputs 1+2 (PWM2+3) as servos
            if ((timerIndex == PWM2 || timerIndex == PWM3) && timerHardwarePtr->tim == TIM4)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
            // remap PWM1+2 as servos
            if ((timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9) && timerHardwarePtr->tim == TIM2)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(CC3D)
            // remap PWM9+10 as servos
            if ((timerIndex == PWM9 || timerIndex == PWM10) && timerHardwarePtr->tim == TIM1)
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

#if defined(OMNIBUS)
            // remap PWM2 (OUT1) as servo
            if (timerIndex == PWM2 && timerHardwarePtr->tim == TIM8)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(RCEXPLORERF3)
            if (timerIndex == PWM2)
            {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(SPRACINGF3EVO)
            if ((timerIndex == PWM8 || timerIndex == PWM9) && timerHardwarePtr->tim == TIM3) {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if (defined(STM32F3DISCOVERY) && !defined(CHEBUZZF3))
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

#if defined(SINGULARITY)
            // remap PWM6+7 as servos
            if (timerIndex == PWM6 || timerIndex == PWM7)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(AIRBOTF4)
            // remap PWM11+PWM12 as servos on multirotor mixers that use servos (i.e. Tri)
            if (timerIndex == PWM11 || timerIndex == PWM12)
                type = MAP_TO_SERVO_OUTPUT;
#endif
        }

        if (init->useChannelForwarding && !init->airplane) {
#if defined(NAZE) && defined(WS2811_TIMER)
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

#if defined(SPRACINGF3EVO)
            if ((timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9) && timerHardwarePtr->tim == TIM3) {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(SPRACINGF3MINI)
            if (((timerIndex == PWM6 || timerIndex == PWM7) && timerHardwarePtr->tim == TIM15)
                || ((timerIndex == PWM8 || timerIndex == PWM9 || timerIndex == PWM10 || timerIndex == PWM11) && timerHardwarePtr->tim == TIM2)) {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif
        }

#endif // USE_SERVOS

#ifdef CC3D
        // This part of code is unnecessary and can be removed - timer clash is resolved by forcing configuration with the same
        // timer tick rate - PWM_TIMER_MHZ
        /*
        if (init->useParallelPWM) {
            // Skip PWM inputs that conflict with timers used outputs.
            if ((type == MAP_TO_SERVO_OUTPUT || type == MAP_TO_MOTOR_OUTPUT) && (timerHardwarePtr->tim == TIM2 || timerHardwarePtr->tim == TIM3)) {
                continue;
            }
            if (type == MAP_TO_PWM_INPUT && timerHardwarePtr->tim == TIM4) {
                continue;
            }

        }
        */
#endif

        if (type == MAP_TO_PPM_INPUT) {
#ifndef SKIP_RX_PWM_PPM
#ifdef CC3D_PPM1
            if (init->useFastPwm || init->pwmProtocolType == PWM_TYPE_BRUSHED) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM4);
            }
#endif
#if defined(SPARKY) || defined(ALIENFLIGHTF3)
            if (init->useFastPwm || init->pwmProtocolType == PWM_TYPE_BRUSHED) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM2);
            }
#endif
            ppmInConfig(timerHardwarePtr);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 0);
#endif
        } else if (type == MAP_TO_PWM_INPUT) {
#ifndef SKIP_RX_PWM_PPM
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 1);
#endif
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
            /* Check if we already configured maximum supported number of motors and skip the rest */
            if (pwmIOConfiguration.motorCount >= MAX_MOTORS) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 1);
                continue;
            }

#if defined(CC3D) && !defined(CC3D_PPM1)
            if (init->useFastPwm || init->pwmProtocolType == PWM_TYPE_BRUSHED) {
                // Skip it if it would cause PPM capture timer to be reconfigured or manually overflowed
                if (timerHardwarePtr->tim == TIM2) {
                    addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                    continue;
                }
            }
#endif

            pwmMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse, init->pwmProtocolType, init->enablePWMOutput);
            if (init->useFastPwm) {
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_FASTPWM | PWM_PF_OUTPUT_PROTOCOL_PWM;
            } else if (init->pwmProtocolType == PWM_TYPE_BRUSHED) {
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;
            } else {
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            }

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.motorCount++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
        } else if (type == MAP_TO_SERVO_OUTPUT) {
            if (pwmIOConfiguration.servoCount >=  MAX_SERVOS) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
                continue;
            }

#ifdef USE_SERVOS
            pwmServoConfig(timerHardwarePtr, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse, init->enablePWMOutput);

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.servoCount++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, i, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
#endif
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}
