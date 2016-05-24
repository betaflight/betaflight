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

#include "platform.h"

#include "gpio.h"
#include "timer.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

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
#ifndef SKIP_RX_PWM
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
        if (timerIndex == PWM2)
            continue;
#endif

#ifdef STM32F10X
        // skip UART2 ports
        if (init->useUART2 && (timerIndex == PWM3 || timerIndex == PWM4))
            continue;
#endif

#if defined(STM32F303xC) && defined(USE_USART3)
        // skip UART3 ports (PB10/PB11)
        if (init->useUART3 && timerHardwarePtr->gpio == UART3_GPIO && (timerHardwarePtr->pin == UART3_TX_PIN || timerHardwarePtr->pin == UART3_RX_PIN))
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

        /*if (init->useSoftSPI) {
            if (type == MAP_TO_PWM_INPUT)
                continue;
            if (type == MAP_TO_PPM_INPUT)
                continue;
            if (timerHardwarePtr->gpio == NRF24_CE_GPIO && timerHardwarePtr->pin == NRF24_CE_PIN) {
                continue;
            }
            if (timerHardwarePtr->gpio == NRF24_CSN_GPIO && timerHardwarePtr->pin == NRF24_CSN_PIN) {
                continue;
            }
            if (timerHardwarePtr->gpio == NRF24_SCK_GPIO && timerHardwarePtr->pin == NRF24_SCK_PIN) {
                continue;
            }
            if (timerHardwarePtr->gpio == NRF24_MOSI_GPIO && timerHardwarePtr->pin == NRF24_MOSI_PIN) {
                continue;
            }
            if (timerHardwarePtr->gpio == NRF24_MISO_GPIO && timerHardwarePtr->pin == NRF24_MISO_PIN) {
                continue;
            }
        }*/

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
                // remap PWM5..8 as servos when used in extended servo mode
                if (timerIndex >= PWM5 && timerIndex <= PWM8)
                    type = MAP_TO_SERVO_OUTPUT;
        }
#endif

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
#ifndef SKIP_RX_PWM
#ifdef CC3D_PPM1
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM4);
            }
#endif
#ifdef SPARKY
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM2);
            }
#endif
            ppmInConfig(timerHardwarePtr);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;
#endif
        } else if (type == MAP_TO_PWM_INPUT) {
#ifndef SKIP_RX_PWM
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;
#endif
        } else if (type == MAP_TO_MOTOR_OUTPUT) {

#if defined(CC3D) && !defined(CC3D_PPM1)
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
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
