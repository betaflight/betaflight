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

#include <platform.h>
#include "build_config.h"

#if defined(SONAR)

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/nvic.h"

#include "drivers/rangefinder.h"
#include "drivers/sonar_hcsr04.h"

#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well


/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

STATIC_UNIT_TESTED volatile int32_t hcsr04SonarPulseTravelTime = 0;
sonarHcsr04Hardware_t sonarHcsr04Hardware;

#if !defined(UNIT_TEST)
static void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;

    if (digitalIn(sonarHcsr04Hardware.echo_gpio, sonarHcsr04Hardware.echo_pin) != 0) {
        timing_start = micros();
    } else {
        const uint32_t timing_stop = micros();
        if (timing_stop > timing_start) {
            hcsr04SonarPulseTravelTime = timing_stop - timing_start;
        }
    }

    EXTI_ClearITPendingBit(sonarHcsr04Hardware.exti_line);
}

void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

void hcsr04_set_sonar_hardware(void)
{
#if !defined(UNIT_TEST)

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    gpio_config_t gpio;
    // trigger pin
    gpio.pin = sonarHcsr04Hardware.trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonarHcsr04Hardware.trigger_gpio, &gpio);
    // echo pin
    gpio.pin = sonarHcsr04Hardware.echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonarHcsr04Hardware.echo_gpio, &gpio);

#ifdef STM32F10X
    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHcsr04Hardware.exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(EXTI_PortSourceGPIOB, sonarHcsr04Hardware.exti_pin_source);
#endif

    EXTI_ClearITPendingBit(sonarHcsr04Hardware.exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = sonarHcsr04Hardware.exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = sonarHcsr04Hardware.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void hcsr04_init(rangefinder_t *rangefinder)
{
    rangefinder->maxRangeCm = HCSR04_MAX_RANGE_CM;
    rangefinder->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;
}

/*
 * Start a range reading
 * Called periodically by the scheduler
 * Measurement reading is done asynchronously, using interrupt
 */
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
     static uint32_t timeOfLastMeasurementMs = 0;
    // the firing interval of the trigger signal should be greater than 60ms
    // to avoid interference between consecutive measurements.
    #define HCSR04_MinimumFiringIntervalMs 60
    const uint32_t timeNowMs = millis();
    if (timeNowMs > timeOfLastMeasurementMs + HCSR04_MinimumFiringIntervalMs) {
        timeOfLastMeasurementMs = timeNowMs;
        digitalHi(sonarHcsr04Hardware.trigger_gpio, sonarHcsr04Hardware.trigger_pin);
        //  The width of trigger signal must be greater than 10us, according to device spec
        delayMicroseconds(11);
        digitalLo(sonarHcsr04Hardware.trigger_gpio, sonarHcsr04Hardware.trigger_pin);
    }
#endif
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(void)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
    int32_t distance = hcsr04SonarPulseTravelTime / 59;
    if (distance > HCSR04_MAX_RANGE_CM) {
        distance = RANGEFINDER_OUT_OF_RANGE;
    }
    return distance;
}
#endif
