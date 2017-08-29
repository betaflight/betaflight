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

#include "drivers/gpio.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "timer.h"

#include "drivers/logging.h"

#include "pwm_output.h"
#include "pwm_mapping.h"
#include "rx_pwm.h"

enum {
    MAP_TO_NONE,
    MAP_TO_PPM_INPUT,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
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
    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    int channelIndex = 0;
#endif

    for (int timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        int type = MAP_TO_NONE;

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef STM32F10X
        // skip UART2 ports
        if (init->useUART2 && (timerHardwarePtr->tag == IO_TAG(PA2) || timerHardwarePtr->tag == IO_TAG(PA3))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(STM32F303xC) && defined(USE_UART3)
        // skip UART3 ports (PB10/PB11)
        if (init->useUART3 && (timerHardwarePtr->tag == IO_TAG(UART3_TX_PIN) || timerHardwarePtr->tag == IO_TAG(UART3_RX_PIN))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(UART6_TX_PIN) || defined(UART6_RX_PIN)
        if (init->useUART6 && (timerHardwarePtr->tag == IO_TAG(UART6_TX_PIN) || timerHardwarePtr->tag == IO_TAG(UART6_RX_PIN))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(USE_SOFTSERIAL1)
        const timerHardware_t *ss1TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_1_RX_PIN), TIM_USE_ANY);
        if (init->useSoftSerial && ss1TimerHardware != NULL && ss1TimerHardware->tim == timerHardwarePtr->tim) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(USE_SOFTSERIAL2)
        const timerHardware_t *ss2TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_2_RX_PIN), TIM_USE_ANY);
        if (init->useSoftSerial && ss2TimerHardware != NULL && ss2TimerHardware->tim == timerHardwarePtr->tim) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef WS2811_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == WS2811_TIMER) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#if defined(STM32F303xC) && defined(WS2811_PIN)
            if (timerHardwarePtr->tag == IO_TAG(WS2811_PIN)) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#endif
        }

#endif

#ifdef VBAT_ADC_PIN
        if (init->useVbat && timerHardwarePtr->tag == IO_TAG(VBAT_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef RSSI_ADC_PIN
        if (init->useRSSIADC && timerHardwarePtr->tag == IO_TAG(RSSI_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_PIN
        if (init->useCurrentMeterADC && timerHardwarePtr->tag == IO_TAG(CURRENT_METER_ADC_PIN)) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef USE_RANGEFINDER_HCSR04
        if (init->useTriggerRangefinder &&
            (
                timerHardwarePtr->tag == init->rangefinderIOConfig.triggerTag ||
                timerHardwarePtr->tag == init->rangefinderIOConfig.echoTag
            )) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

        // Handle timer mapping to PWM/PPM inputs
        if (init->useSerialRx) {
            type = MAP_TO_NONE;
        }
        else if (init->useParallelPWM && (timerHardwarePtr->usageFlags & TIM_USE_PWM)) {
            type = MAP_TO_PWM_INPUT;
        }
        else if (init->usePPM && (timerHardwarePtr->usageFlags & TIM_USE_PPM)) {
            type = MAP_TO_PPM_INPUT;
        }

        // Handle outputs - may override the PWM/PPM inputs
        if (init->flyingPlatformType == PLATFORM_MULTIROTOR) {
            // Multicopter
#ifdef USE_SERVOS
            if (init->useServoOutputs && (timerHardwarePtr->usageFlags & TIM_USE_MC_SERVO)) {
                type = MAP_TO_SERVO_OUTPUT;
            }
            else if (init->useChannelForwarding && (timerHardwarePtr->usageFlags & TIM_USE_MC_CHNFW)) {
                type = MAP_TO_SERVO_OUTPUT;
            }
            else
#endif
            if (timerHardwarePtr->usageFlags & TIM_USE_MC_MOTOR) {
                type = MAP_TO_MOTOR_OUTPUT;
            }
        }
#ifdef USE_SERVOS
        else if (init->flyingPlatformType == PLATFORM_AIRPLANE || init->flyingPlatformType == PLATFORM_HELICOPTER) {
            // Fixed wing or HELI (one/two motors and a lot of servos
            if (timerHardwarePtr->usageFlags & TIM_USE_FW_SERVO) {
                type = MAP_TO_SERVO_OUTPUT;
            }
            else if (timerHardwarePtr->usageFlags & TIM_USE_FW_MOTOR) {
                type = MAP_TO_MOTOR_OUTPUT;
            }
        }
#endif

        // If timer not mapped - skip
        if (type == MAP_TO_NONE)
            continue;
/*
#ifdef USE_SERVOS
        if (init->useServos && !init->airplane) {
#if defined(SPRACINGF3MINI)
            // remap PWM6+7 as servos
            if ((timerIndex == PWM6 || timerIndex == PWM7) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SINGULARITY)
            // remap PWM6+7 as servos
            if (timerIndex == PWM6 || timerIndex == PWM7)
                type = MAP_TO_SERVO_OUTPUT;
#endif
        }

#if defined(SPRACINGF3MINI)
            if (((timerIndex == PWM6 || timerIndex == PWM7) && timerHardwarePtr->tim == TIM15)
                || ((timerIndex == PWM8 || timerIndex == PWM9 || timerIndex == PWM10 || timerIndex == PWM11) && timerHardwarePtr->tim == TIM2)) {
                type = MAP_TO_SERVO_OUTPUT;
            }
#endif
        }

#endif // USE_SERVOS
*/

        if (type == MAP_TO_PPM_INPUT) {
#if defined(USE_RX_PPM)
            ppmInConfig(timerHardwarePtr, init->pwmProtocolType);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 0);
#endif
        } else if (type == MAP_TO_PWM_INPUT) {
#if defined(USE_RX_PWM)
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 1);
#endif
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
            /* Check if we already configured maximum supported number of motors and skip the rest */
            if (pwmIOConfiguration.motorCount >= MAX_MOTORS) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 1);
                continue;
            }

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

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
        } else if (type == MAP_TO_SERVO_OUTPUT) {
            if (pwmIOConfiguration.servoCount >=  MAX_SERVOS) {
                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
                continue;
            }

#ifdef USE_SERVOS
            pwmServoConfig(timerHardwarePtr, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse, init->enablePWMOutput);

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.servoCount++;

            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
#endif
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}
